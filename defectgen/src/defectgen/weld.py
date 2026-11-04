"""Lay the weld beads: beam trace -> deposited energy -> bead geometry.

A seam is not drawn as a shape. It is the result of a beam moving over the
busbar. This module does that in four steps, all in the busbar's own frame, so
a bead rides a tilted busbar exactly as a real one would:

1. **Trace.** The spot's centre over time: the commanded straight stitch, plus
   whatever the motion added -- slow lateral drift of the end effector, the
   path bowed into an arc, deliberate circular beam wobble. Sampled every
   `weld_trace_step` at constant `weld_speed`, so each sample is the same dwell.

2. **Energy.** Each sample deposits a Gaussian of heat around the spot. Summed
   over the trace on a `weld_grid` heightfield, normalised so the middle of a
   steady straight stitch reads 1.0. Where it exceeds `weld_melt_threshold` the
   metal melts. The spot size is chosen so a straight stitch melts exactly
   `weld_stitch_w` wide; wobble widens it, slowing or doubling back raises it.

3. **Bead.** The crown follows the energy, scaled to `weld_bead_h`, with
   solidification ripples along the path and a crater where the beam stops and
   the pool is no longer fed.

4. **Gap.** The real busbar-to-terminal gap is measured under every trace sample
   *after settling*, so a busbar tilted on a chip gives a different gap along
   one stitch. Melt runs down to bridge it: the crown drops by
   `weld_fill_factor * gap`, the toes undercut, and past `weld_gap_hole` the
   melt drops through and leaves a hole. Each consequence is labelled with the
   upstream defect that made the gap -- the weld is not the root cause.

Thresholds are assumptions until sourced for Cu-on-Al (status_update.md).
"""

from __future__ import annotations

import numpy as np
import trimesh

from .plan import Plan, SeamPlan
from .raycast import AxisCaster


def trace(seam: SeamPlan, step: float) -> np.ndarray:
    """(N, 2) beam positions in the stitch frame (x along, y across, origin at
    the commanded centre), one per `step` of travel along the seam."""
    n = max(2, int(round(seam.length / step)) + 1)
    s = np.linspace(-seam.length / 2, seam.length / 2, n)
    y = np.zeros(n)
    if seam.arc_sagitta:
        # A circular bow through both ends with this much rise at the middle,
        # small-angle: y = sagitta * (1 - (2s/L)^2).
        y += seam.arc_sagitta * (1.0 - (2.0 * s / seam.length) ** 2)
    if seam.drift_amp:
        y += seam.drift_amp * np.sin(2 * np.pi * (s + seam.length / 2) / seam.drift_wavelength)
    x = s.copy()
    if seam.wobble_amp:
        phase = 2 * np.pi * (s + seam.length / 2) / seam.wobble_pitch
        x += seam.wobble_amp * np.cos(phase)
        y += seam.wobble_amp * np.sin(phase)
    return np.column_stack([x, y])


def spot_sigma(cfg, width: float) -> float:
    """Gaussian spot radius that melts a straight stitch exactly `width` wide:
    across a steady line the energy falls as exp(-y^2 / 2 sigma^2)."""
    return (width / 2) / np.sqrt(2 * np.log(1.0 / cfg.weld_melt_threshold))


def energy(points: np.ndarray, beam: np.ndarray, sigma: float, step: float) -> np.ndarray:
    """Deposited energy at each point, 1.0 at the middle of a steady straight
    stitch. Beam samples further than 4 sigma contribute nothing and are
    skipped, in chunks, so this stays linear in the seam length."""
    out = np.zeros(len(points))
    reach = 4 * sigma
    order = np.argsort(points[:, 0])
    px = points[order, 0]
    for chunk in np.array_split(np.arange(len(beam)), max(1, len(beam) // 400)):
        b = beam[chunk]
        lo, hi = np.searchsorted(px, [b[:, 0].min() - reach, b[:, 0].max() + reach])
        idx = order[lo:hi]
        d2 = ((points[idx, None, :] - b[None, :, :]) ** 2).sum(axis=2)
        out[idx] += np.exp(-d2 / (2 * sigma**2)).sum(axis=1)
    # A line of samples `step` apart integrates to sqrt(2 pi) sigma / step.
    return out * step / (np.sqrt(2 * np.pi) * sigma)


def _nearest_along(points: np.ndarray, beam: np.ndarray, step: float):
    """For each point, the distance along the trace of its nearest beam sample,
    and that sample's index. Used for ripples and the per-sample gap."""
    from scipy.spatial import cKDTree

    _, idx = cKDTree(beam).query(points)
    return idx * step, idx


def _no_corner_touches(cells: np.ndarray) -> np.ndarray:
    """Fill one cell of every 2x2 block where two selected cells meet only at a
    corner. Such a pair shares a single vertex, which is not a manifold, and
    the boolean engine refuses it. The fill moves the surface by one grid cell
    (50 um) at those points only."""
    cells = cells.copy()
    while True:
        a, b = cells[:-1, :-1], cells[1:, 1:]
        c, d = cells[1:, :-1], cells[:-1, 1:]
        bad = (a & b & ~c & ~d) | (c & d & ~a & ~b)
        if not bad.any():
            return cells
        i, j = np.nonzero(bad)
        # Fill the whole block: either diagonal pattern then closes.
        cells[i, j] = cells[i + 1, j] = cells[i, j + 1] = cells[i + 1, j + 1] = True


def _extrude(xs, ys, top, bottom, cells) -> trimesh.Trimesh | None:
    """A closed solid over the selected grid cells, between `bottom` and `top`
    (node arrays shaped (len(xs), len(ys))). Top and bottom faces per cell,
    walls along every edge a selected cell shares with an unselected one."""
    if not cells.any():
        return None
    cells = _no_corner_touches(cells)
    nx, ny = len(xs), len(ys)
    top = np.maximum(top, bottom + 1e-4)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    vt = np.column_stack([gx.ravel(), gy.ravel(), top.ravel()])
    vb = np.column_stack([gx.ravel(), gy.ravel(), bottom.ravel()])
    vertices = np.vstack([vt, vb])
    off = nx * ny
    node = lambda i, j: i * ny + j  # noqa: E731

    ci, cj = np.nonzero(cells)
    a, b = node(ci, cj), node(ci + 1, cj)
    c, d = node(ci + 1, cj + 1), node(ci, cj + 1)
    faces = [
        np.column_stack([a, b, c]), np.column_stack([a, c, d]),
        np.column_stack([a + off, c + off, b + off]), np.column_stack([a + off, d + off, c + off]),
    ]
    padded = np.pad(cells, 1)
    # (neighbour offset, edge from node p to node q going CCW seen from above)
    for (di, dj), (p, q) in (
        ((0, -1), (a, b)),
        ((1, 0), (b, c)),
        ((0, 1), (c, d)),
        ((-1, 0), (d, a)),
    ):
        open_ = ~padded[ci + 1 + di, cj + 1 + dj]
        p, q = p[open_], q[open_]
        faces += [np.column_stack([p + off, q + off, q]), np.column_stack([p + off, q, p])]
    mesh = trimesh.Trimesh(vertices, np.vstack(faces), process=True)
    mesh.remove_unreferenced_vertices()
    return mesh


def _cause(plan: Plan, seam: SeamPlan) -> str:
    """Which injected defects the gap under this seam traces back to: whatever
    shaped its busbar's pose -- directly, or through what the busbar settled
    on (`plan.influence`, built by settle.py) -- and whatever moved the cell
    and terminals it is welded to."""
    hits = set(plan.influence.get(("busbar", seam.busbar_index), set()))
    for ref, classes in plan.influence.items():
        if ref[0] in ("cell", "terminal") and ref[1] == seam.cell_index:
            hits |= classes
    return "+".join(sorted(hits)) if hits else "unattributed"


# Beads already built, keyed by everything that shapes one: the seam's
# commanded geometry and motion, and the gap profile under its trace. A clean
# module's 48 seams are identical, so they are built once.
_CACHE: dict = {}


def _shape(cfg, seam: SeamPlan, gap: np.ndarray):
    """Bead and cut solids for one seam, centred on its commanded centre with
    the busbar top at z = 0, plus the deepest undercut and whether it holed."""
    key = (
        id(cfg), seam.length, seam.width, seam.height, seam.drift_amp, seam.drift_wavelength,
        seam.arc_sagitta, seam.wobble_amp, seam.wobble_pitch,
        np.round(np.where(np.isfinite(gap), gap, -1.0), 6).tobytes(),
    )
    if key in _CACHE:
        return _CACHE[key]

    step = cfg.weld_trace_step
    beam = trace(seam, step)
    sigma = spot_sigma(cfg, seam.width)

    g = cfg.weld_grid
    pad = 3 * sigma + seam.envelope
    xs = np.arange(beam[:, 0].min() - pad, beam[:, 0].max() + pad + g, g)
    ys = np.arange(beam[:, 1].min() - pad, beam[:, 1].max() + pad + g, g)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    pts = np.column_stack([gx.ravel(), gy.ravel()])
    e = energy(pts, beam, sigma, step)
    m = cfg.weld_melt_threshold
    melt = np.clip((e - m) / (1.0 - m), 0.0, None)

    along, nearest = _nearest_along(pts, beam, step)
    local_gap = gap[nearest]
    finite = np.isfinite(local_gap)

    # Crown: elliptical across the melt, capped where the energy overshoots
    # (that excess is the heat-input signal step 3 of the queue uses).
    crown = seam.height * np.sqrt(np.clip(melt, 0.0, 1.5))
    # Ripples and the crater scale the crown rather than cut into it, so a
    # good weld never dips below the busbar surface: ripple height goes with
    # the pool, and an unfed pool at the stop sinks toward the surface.
    ripple = 1.0 + (cfg.weld_ripple_amp / seam.height) * np.cos(
        2 * np.pi * along / cfg.weld_ripple_pitch
    )
    crater = 1.0 - (cfg.weld_crater_depth / seam.height) * np.exp(
        -((pts - beam[-1]) ** 2).sum(axis=1) / (2 * sigma**2)
    )
    h = crown * ripple * np.clip(crater, 0.0, 1.0)

    # The gap takes melt with it: the crown drops by what it costs to bridge,
    # the toes undercut, and past weld_gap_hole -- or with nothing under the
    # beam at all -- the melt drops through.
    over = np.where(finite, np.maximum(local_gap - cfg.weld_gap_ok, 0.0), 0.0)
    h -= np.where(melt > 0, cfg.weld_fill_factor * np.where(finite, local_gap, 0.0), 0.0)
    toe = np.clip(1.0 - np.abs(e - m) / 0.25, 0.0, 1.0) * (e < m)
    undercut = np.minimum(cfg.weld_undercut_factor * over, 0.3 * cfg.busbar_t) * toe
    h -= undercut
    hole = (melt > 0.6) & (~finite | (local_gap > cfg.weld_gap_hole))
    h = np.where(hole, -cfg.busbar_t - 0.05, h).reshape(gx.shape)

    cells = lambda mask: (  # noqa: E731
        mask[:-1, :-1] & mask[1:, :-1] & mask[1:, 1:] & mask[:-1, 1:]
    )
    raised = (melt.reshape(gx.shape) > 0) & (h > 1e-4)
    sunk = h < -1e-4
    flat = np.zeros(h.shape)
    bead = _extrude(xs, ys, np.maximum(h, 0.0), flat, cells(raised))
    cut = _extrude(xs, ys, flat + 0.01, np.minimum(h, 0.0), cells(sunk))
    result = (bead, cut, float(undercut.max()), bool(hole.any()))
    _CACHE[key] = result
    return result


def lay(plan: Plan, parts) -> None:
    """Build every present seam's bead from its trace, after settling."""
    from .settle import world_mesh  # settle imports module, module imports this

    cfg = plan.cfg
    if not plan.seams:
        return
    terminals = AxisCaster(
        trimesh.util.concatenate(
            [world_mesh(plan, parts, p) for p in plan.of_kind("terminal") if p.present]
        ),
        2,
    )
    cuts: dict = {}
    upstream = len(plan.defects)

    for index, seam in enumerate(plan.seams):
        if not seam.present:
            continue
        busbar = plan.find(("busbar", seam.busbar_index))
        to_world = busbar.transform
        centre = (np.linalg.inv(to_world) @ np.append(seam.centre, 1.0))[:3]
        place = to_world @ trimesh.transformations.translation_matrix(centre)

        # Gap under each beam sample, measured in the world after settling.
        beam = trace(seam, cfg.weld_trace_step)
        under = trimesh.transform_points(
            np.column_stack([beam, np.full(len(beam), -cfg.busbar_t)]), place
        )
        gap = under[:, 2] - terminals.extreme(under, highest=True)
        gap = np.where(np.isnan(gap), np.inf, np.maximum(gap, 0.0))
        seam.gap = float(np.max(gap[np.isfinite(gap)])) if np.isfinite(gap).any() else float("inf")

        bead, cut, undercut, holed = _shape(cfg, seam, gap)
        if bead is not None:
            plan.beads[index] = bead.copy().apply_transform(place)
        if cut is not None:
            cuts.setdefault(busbar.ref, []).append(cut.copy().apply_transform(place))
        _record(plan, seam, gap, undercut=undercut, holed=holed, upstream=upstream)

    for ref, pieces in cuts.items():
        busbar = plan.find(ref)
        mesh = world_mesh(plan, parts, busbar)
        plan.overrides[ref] = trimesh.boolean.difference([mesh, *pieces], engine="manifold")


def _record(plan, seam, gap, undercut, holed, upstream) -> None:
    """Label what the gap did to this weld, with its cause. Only consequences
    past the thresholds are recorded -- a weld over a 0.1 mm gap is a good weld.
    Underfill is the crown lost to the gap, visible whether or not the bead
    ends up below the busbar surface."""
    cfg = plan.cfg
    finite_max = seam.gap
    if finite_max <= cfg.weld_gap_ok and not holed:
        return
    cause = _cause(plan, seam)
    at = seam.centre
    underfill = cfg.weld_fill_factor * finite_max if np.isfinite(finite_max) else seam.height
    if underfill > 0:
        plan.record("weld_underfill", underfill, at, seams=(seam,), cause=cause)
    if undercut > 0:
        plan.record("weld_undercut", undercut, at, seams=(seam,), cause=cause)
    if finite_max > cfg.weld_gap_fusion or not np.isfinite(gap).all():
        # Internal: no surface sensor sees it. Labelled so the gap check can be
        # credited with catching it before welding (spec §8 steps 8-9).
        plan.record("lack_of_fusion", finite_max, at, seams=(seam,), cause=cause)
    if holed:
        plan.record("burn_through", 1.0, at, seams=(seam,), cause=cause)

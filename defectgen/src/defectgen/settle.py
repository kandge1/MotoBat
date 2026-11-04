"""Bring an injected plan to rest, so no two parts occupy the same space.

Defect injectors move parts by transform, and a moved part can end up inside
its neighbours: a rotated busbar through the carrier wall, a shifted cell
through the side plate. Real parts do not do that. They rest on whatever is
under them, or they push something that gives. This module makes the plan do
the same, in assembly order, so every consequence of a defect propagates to
what is built on top of it -- a raised terminal holds the busbar off its
neighbours, which is how a real pre-weld gap forms.

Three kinds of response:

**Rigid parts settle under gravity.** A part dropped onto its supports comes to
rest where its centre of mass is lowest without passing through anything. For
small rotations that is a linear program over (dz, rx, ry):

    minimise  dz
    subject to  z_i + dz + rx (y_i - cy) - ry (x_i - cx)  >=  s_i   for every i

where z_i are points on the part's underside and s_i the height of the support
directly below each. The optimum is a static equilibrium: the active
constraints are the contact points, and the LP's optimality conditions say the
centre of mass lies over their convex hull. The linearisation is redone a few
times so larger tilts converge on the exact pose.

**Compliant parts deform.** Foam pads compress where a cell presses into them.
A 1.5 mm side plate welded on after compression bulges outward where a cell
stands proud of it, the push spreading along the sheet.

**Rigid against rigid is an error.** A cell that would have to pass through its
neighbour or an end plate is not a placement error any line could produce; it
raises, naming the cause, rather than producing an impossible module.
"""

from __future__ import annotations

import numpy as np
import trimesh
from scipy.optimize import linprog

from .plan import Placement, Plan
from .raycast import AxisCaster

# A part needing more than this to come to rest has nothing under one side and
# would fall off, which is a broken setup rather than a defect.
MAX_TILT = np.radians(15.0)
# Below this a pose change is numerical noise, not movement.
EPS = 1e-7
ITERATIONS = 8
BULGE_MARGIN = 5e-3
# Must be at least the side plate's remesh edge length (2.0).
PLATEAU = 2.0
# Spacing of points sampled along edges, where contact on a tilted face peaks.
EDGE_STEP = 0.1
# The rest LP starts from this many of the nearest contacts and adds back, in
# batches of the same size, the ones its answer would push through. The far
# ones never bind, and leaving them out is most of the LP's cost.
ACTIVE_START = 500

# What each part rests on, in assembly order. Particles flagged as supports
# (debris, FOD) are added to every list.
ORDER: list[tuple[str, tuple[str, ...]]] = [
    ("carrier", ("cell", "terminal", "pad", "end_plate", "side_plate")),
    ("busbar", ("cell", "terminal", "carrier")),
    # The CMC is not settled: it is located by 6 mm pegs in 6 mm holes, which
    # fixes its position exactly, and with zero clearance the two tessellated
    # circles graze at the rim, which settling would read as contact. It
    # rides with the carrier instead (`_carry_cmc`).
    ("cover", ("end_plate", "side_plate", "carrier", "busbar", "cmc", "cell", "terminal")),
]

SUPPORT_PARTICLES = ("debris", "fod")


def world_mesh(plan: Plan, parts: dict[str, trimesh.Trimesh], placement: Placement):
    if placement.ref in plan.overrides:
        return plan.overrides[placement.ref].copy()
    return parts[placement.part].copy().apply_transform(placement.transform)


def particle_mesh(particle) -> trimesh.Trimesh:
    if particle.kind == "debris":
        # A chip, flat on top so the gap it makes is exactly its height.
        blob = trimesh.creation.box(extents=[2 * particle.radius] * 3)
    else:
        blob = trimesh.creation.icosphere(subdivisions=1, radius=particle.radius)
    return blob.apply_translation(particle.centre)


def _supports(plan: Plan, parts, kinds: tuple[str, ...]) -> trimesh.Trimesh | None:
    meshes = []
    for placement in plan.placements:
        if not placement.present:
            continue
        kind = placement.ref[0]
        if kind in kinds:
            meshes.append(world_mesh(plan, parts, placement))
    meshes += [particle_mesh(p) for p in plan.particles if p.kind in SUPPORT_PARTICLES]
    return trimesh.util.concatenate(meshes) if meshes else None


def _edge_points(mesh: trimesh.Trimesh, faces: np.ndarray, step: float) -> np.ndarray:
    """Points every `step` along the edges of the given faces. The highest
    support under a tilted part sits where an edge of one crosses an edge of
    the other -- a vertex of neither -- so edges are sampled densely."""
    edges = mesh.faces[faces][:, [[0, 1], [1, 2], [2, 0]]].reshape(-1, 2)
    edges = np.unique(np.sort(edges, axis=1), axis=0)
    a, b = mesh.vertices[edges[:, 0]], mesh.vertices[edges[:, 1]]
    n = np.maximum(1, np.ceil(np.linalg.norm(b - a, axis=1) / step).astype(int))
    reps = np.repeat(np.arange(len(edges)), n + 1)
    t = np.concatenate([np.linspace(0.0, 1.0, k + 1) for k in n])
    return a[reps] + t[:, None] * (b[reps] - a[reps])


def _underside(mesh: trimesh.Trimesh, spacing: float) -> np.ndarray:
    """Points on the part's underside: dense samples along the edges of every
    downward face, which catch edge and corner contact, plus a grid where the
    lowest surface of the part is, which catches contact mid-face."""
    down = mesh.face_normals[:, 2] < -0.5
    corners = _edge_points(mesh, down, EDGE_STEP)

    lo, hi = mesh.bounds
    xs = np.arange(lo[0] + spacing / 2, hi[0], spacing)
    ys = np.arange(lo[1] + spacing / 2, hi[1], spacing)
    gx, gy = np.meshgrid(xs, ys)
    grid = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)])
    grid[:, 2] = AxisCaster(mesh, 2).extreme(grid, highest=False)
    return np.vstack([corners, grid[~np.isnan(grid[:, 2])]])


def _heights_below(support: AxisCaster, points: np.ndarray) -> np.ndarray:
    """Top of the support directly below each point's (x, y); NaN where none."""
    return support.extreme(points, highest=True)


def _peaks_under(support: trimesh.Trimesh, part: trimesh.Trimesh):
    """The other side of the contact: points along every upward-facing edge of
    the support that lies under the part, paired with the part's underside
    height directly above. A grid on the part alone misses any support narrower
    than its spacing -- a 0.4 mm chip under a 12 mm busbar."""
    lo, hi = part.bounds
    up = support.face_normals[:, 2] > 0.5
    tri = support.triangles
    near = np.all((tri.max(axis=1)[:, :2] >= lo[:2]) & (tri.min(axis=1)[:, :2] <= hi[:2]), axis=1)
    if not (up & near).any():
        return np.empty((0, 3)), np.empty(0)
    tops = _edge_points(support, up & near, EDGE_STEP)
    tops = tops[np.all((tops[:, :2] >= lo[:2]) & (tops[:, :2] <= hi[:2]), axis=1)]
    under = AxisCaster(part, 2).extreme(tops, highest=False)
    keep = ~np.isnan(under)
    return np.column_stack([tops[keep, :2], under[keep]]), tops[keep, 2]


def _hull_reduce(points: np.ndarray, heights: np.ndarray):
    """Drop contact constraints that can never bind. All points at the same gap
    give constraints that differ only in (x, y), and a linear function of
    (x, y) peaks on the convex hull -- so per gap level only the hull's
    vertices matter. A busbar flat on three pads goes from thousands of rows to
    a few dozen, with the same optimum."""
    from scipy.spatial import ConvexHull, QhullError

    gap = np.round(points[:, 2] - heights, 9)
    order = np.argsort(gap, kind="stable")
    levels, starts = np.unique(gap[order], return_index=True)
    bounds = np.append(starts, len(order))
    keep = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        idx = order[a:b]
        if len(idx) > 3:
            try:
                idx = idx[ConvexHull(points[idx, :2]).vertices]
            except QhullError:
                # Collinear: the two ends of the line are all that can bind.
                xy = points[idx, :2]
                along = (xy - xy[0]) @ (xy[-1] - xy[0])
                idx = idx[[np.argmin(along), np.argmax(along)]]
        keep.append(idx)
    keep = np.concatenate(keep)
    return points[keep], heights[keep]


def _rest_delta(points: np.ndarray, heights: np.ndarray, centre: np.ndarray, ref) -> np.ndarray:
    keep = ~np.isnan(heights)
    if not keep.any():
        raise ValueError(f"{ref} has nothing under it to rest on")
    p, s = _hull_reduce(points[keep], heights[keep])
    dx, dy = p[:, 0] - centre[0], p[:, 1] - centre[1]
    a_ub = -np.column_stack([np.ones(len(p)), dy, -dx])
    b_ub = p[:, 2] - s
    # Active set: solve on the near contacts, then add back any far one the
    # answer would push through, until none is violated. Same optimum as the
    # full LP, a fraction of the rows.
    active = np.zeros(len(b_ub), bool)
    active[np.argsort(b_ub)[:ACTIVE_START]] = True
    while True:
        result = linprog(
            c=[1.0, 0.0, 0.0],
            A_ub=a_ub[active],
            b_ub=b_ub[active],
            bounds=[(None, None), (-MAX_TILT, MAX_TILT), (-MAX_TILT, MAX_TILT)],
            method="highs",
        )
        if not result.success:
            raise ValueError(f"{ref} cannot come to rest: {result.message}")
        excess = np.where(active, -np.inf, a_ub @ result.x - b_ub)
        violated = np.where(excess > 1e-12)[0]
        if not len(violated):
            break
        worst = violated[np.argsort(excess[violated])[::-1][:ACTIVE_START]]
        active[worst] = True
    dz, rx, ry = result.x
    if max(abs(rx), abs(ry)) > MAX_TILT - 1e-6:
        raise ValueError(
            f"{ref} would tip more than {np.degrees(MAX_TILT):.0f} deg to rest -- "
            "it is not supported on one side"
        )
    if max(abs(dz), abs(rx), abs(ry)) < EPS:
        return np.eye(4)
    rot = trimesh.transformations.rotation_matrix(ry, [0, 1, 0], centre)
    rot = rot @ trimesh.transformations.rotation_matrix(rx, [1, 0, 0], centre)
    shift = np.eye(4)
    shift[2, 3] = dz
    return shift @ rot


def _settle_part(plan, parts, placement, support, spacing) -> np.ndarray:
    """Iterate the linearised rest until the pose stops changing. Returns the
    total delta applied, so things riding on the part can follow it."""
    total = np.eye(4)
    support_caster = AxisCaster(support, 2)
    for _ in range(ITERATIONS):
        mesh = world_mesh(plan, parts, placement)
        points = _underside(mesh, spacing)
        heights = _heights_below(support_caster, points)
        peak_points, peak_heights = _peaks_under(support, mesh)
        delta = _rest_delta(
            np.vstack([points, peak_points]),
            np.concatenate([heights, peak_heights]),
            mesh.center_mass,
            placement.ref,
        )
        if np.allclose(delta, np.eye(4), atol=EPS):
            return total
        placement.compose(delta)
        total = delta @ total
    return total


def _seat(plan, parts, placement: Placement) -> None:
    """Lift a disturbed terminal straight up until it clears its own cap."""
    _, k, _ = placement.ref
    can = world_mesh(plan, parts, plan.find(("cell", k)))
    mesh = world_mesh(plan, parts, placement)
    points = _underside(mesh, 0.5)
    heights = _heights_below(AxisCaster(can, 2), points)
    keep = ~np.isnan(heights)
    lift = float(np.max(heights[keep] - points[keep, 2])) if keep.any() else 0.0
    if lift > EPS:
        placement.compose(_z(lift))


def _z(dz: float) -> np.ndarray:
    t = np.eye(4)
    t[2, 3] = dz
    return t


# -- compliant parts -------------------------------------------------------

def _face_x(mesh: trimesh.Trimesh, yz: np.ndarray, highest: bool) -> np.ndarray:
    """The mesh's furthest +X (or -X) surface at each (y, z); NaN where none."""
    points = np.column_stack([np.zeros(len(yz)), yz])
    return AxisCaster(mesh, 0).extreme(points, highest)


def _check_and_compress_stack(plan: Plan, parts) -> None:
    """Between each pair of neighbours in the X stack: measure the gap on a
    grid over their shared face, compress the pad between them to fit, and
    refuse a gap no pad could absorb."""
    cfg = plan.cfg
    ys = np.arange(-cfg.cell_w / 2 + 1, cfg.cell_w / 2, 2.0)
    zs = np.arange(cfg.z_cell_bottom + 1, cfg.z_cell_top, 2.0)
    gy, gz = np.meshgrid(ys, zs)
    yz = np.column_stack([gy.ravel(), gz.ravel()])

    cells = {p.ref[1]: p for p in plan.of_kind("cell") if p.present}
    meshes = {k: world_mesh(plan, parts, p) for k, p in cells.items()}

    end_x = cfg.module_l / 2 - cfg.end_plate_t
    for k in range(cfg.n_cells):
        if k not in meshes:
            continue
        right = _face_x(meshes[k], yz, highest=True)
        left = _face_x(meshes[k], yz, highest=False)
        if k == 0 and np.nanmin(left) < -end_x - 1e-6:
            raise ValueError(f"cell 0 passes {-end_x - np.nanmin(left):.2f} mm into end plate a")
        if k == cfg.n_cells - 1 and np.nanmax(right) > end_x + 1e-6:
            raise ValueError(f"cell {k} passes {np.nanmax(right) - end_x:.2f} mm into end plate b")
        if k + 1 not in meshes:
            continue
        nxt = _face_x(meshes[k + 1], yz, highest=False)
        gap = nxt - right
        pad = next((p for p in plan.of_kind("pad") if p.ref[1] == k and p.present), None)
        floor = cfg.pad_min_t if pad else 0.0
        if np.nanmin(gap) < floor - 1e-6:
            what = f"pad {k} to {np.nanmin(gap):.2f} mm" if pad else "the gap where pad {k} is missing"
            raise ValueError(
                f"cells {k} and {k + 1} crush {what} (pad_min_t={cfg.pad_min_t}); "
                "they would collide -- reduce the placement error"
            )
        if pad is not None and np.nanmin(gap) < cfg.pad_t - 1e-6:
            plan.overrides[pad.ref] = _compress_pad(
                world_mesh(plan, parts, pad), meshes[k], meshes[k + 1]
            )


def _compress_pad(pad: trimesh.Trimesh, before: trimesh.Trimesh, after: trimesh.Trimesh):
    """Foam takes the shape of what presses on it, and loses the volume -- it
    is compressible, which is the point of it. So the pad is exactly the pad
    minus the two cells. A vertex-by-vertex push cannot do this: the cans have
    rounded edges, and a straight edge between two pushed vertices cuts back
    inside the curve."""
    return trimesh.boolean.difference([pad, before, after], engine="manifold")


def _bulge_side_plates(plan: Plan, parts) -> None:
    """Where a cell stands proud of the side plate's inner face, push the sheet
    out by that much and let the push fade along the sheet over the spread
    length -- both faces together, because a 1.5 mm sheet bends as one."""
    cfg = plan.cfg
    cells = [world_mesh(plan, parts, p) for p in plan.of_kind("cell") if p.present]
    if not cells:
        return
    stack = AxisCaster(trimesh.util.concatenate(cells), 1)
    xs = np.arange(-cfg.module_l / 2, cfg.module_l / 2, 1.0)
    zs = np.arange(cfg.z_cell_bottom + 0.5, cfg.z_cell_top, 1.0)
    gx, gz = np.meshgrid(xs, zs)
    xz = np.column_stack([gx.ravel(), gz.ravel()])
    inner = cfg.cell_w / 2

    for placement in plan.of_kind("side_plate"):
        if not placement.present:
            continue
        sign = 1.0 if placement.ref[1] == "right" else -1.0
        points = np.column_stack([xz[:, 0], np.zeros(len(xz)), xz[:, 1]])
        reach = sign * stack.extreme(points, highest=sign > 0)
        push = np.nan_to_num(reach - inner, nan=0.0)
        contact = push > 1e-6
        # Contacts are sampled on a 1 mm grid; between samples a yawed can
        # can stand a few um prouder. Overshooting by 5 um closes that.
        push = push + BULGE_MARGIN
        if not contact.any():
            continue

        mesh = world_mesh(plan, parts, placement)
        vertices, faces = trimesh.remesh.subdivide_to_size(mesh.vertices, mesh.faces, max_edge=2.0)
        mesh = trimesh.Trimesh(vertices, faces, process=False)
        pts, depth = xz[contact], push[contact]
        spread = cfg.side_plate_bulge_spread
        bulge = np.zeros(len(mesh.vertices))
        for chunk in np.array_split(np.arange(len(mesh.vertices)), max(1, len(mesh.vertices) // 4000)):
            d = np.linalg.norm(mesh.vertices[chunk][:, None, [0, 2]] - pts[None], axis=2)
            # Full depth within one triangle's size of a contact, so no
            # triangle straddling it can interpolate back into the can.
            r = np.maximum(d - PLATEAU, 0.0)
            weight = np.where(r < spread, 0.5 * (1 + np.cos(np.pi * r / spread)), 0.0)
            bulge[chunk] = np.max(weight * depth[None], axis=1)
        mesh.vertices[:, 1] += sign * bulge
        plan.overrides[placement.ref] = mesh


# -- driver ----------------------------------------------------------------

def _moved(plan: Plan, nominal: Plan, placement: Placement) -> bool:
    want = nominal.find(placement.ref)
    return (
        placement.present != want.present
        or placement.ref in plan.overrides
        or not np.allclose(placement.transform, want.transform, atol=1e-9)
    )


def settle(plan: Plan, parts: dict[str, trimesh.Trimesh], force: bool = False) -> None:
    """Resolve every interpenetration the injectors left, in assembly order.

    A part is only re-settled if it, or something it rests on, differs from
    the nominal plan -- the nominal module is at rest by construction (and a
    test holds it to that with `force=True`), so this keeps clean generation
    as cheap as it was.
    """
    from .module import build_plan  # module imports this file

    nominal = build_plan(plan.cfg, plan.stage)
    dirty = {p.ref[0] for p in plan.placements if _moved(plan, nominal, p)}
    if any(p.kind in SUPPORT_PARTICLES for p in plan.particles):
        dirty.add("particle")
    if force:
        dirty |= {p.ref[0] for p in plan.placements}

    if force or "cell" in dirty:
        _check_and_compress_stack(plan, parts)
        _bulge_side_plates(plan, parts)
        dirty |= {"pad", "side_plate"}

    for placement in plan.placements:
        if placement.seat and placement.present:
            _seat(plan, parts, placement)
            dirty.add("terminal")

    for kind, kinds in ORDER:
        if not (dirty & ({kind, "particle"} | set(kinds))):
            continue
        support = _supports(plan, parts, kinds)
        if support is None:
            continue
        spacing = 1.0 if kind in ("busbar", "cmc") else 2.0
        for placement in plan.of_kind(kind):
            if not placement.present or placement.pinned:
                continue
            delta = _settle_part(plan, parts, placement, support, spacing)
            if not np.allclose(delta, np.eye(4), atol=EPS):
                dirty.add(kind)
                if kind == "busbar":
                    _carry_seams(plan, placement.ref[1], delta)
                if kind == "carrier":
                    _carry_cmc(plan, delta)
                    dirty.add("cmc")

    _land_spatter(plan, parts)


def _carry_cmc(plan: Plan, delta: np.ndarray) -> None:
    """The CMC is pegged to the carrier, so wherever the carrier goes it goes."""
    for placement in plan.of_kind("cmc"):
        if placement.present:
            placement.compose(delta)


def _carry_seams(plan: Plan, busbar_index: int, delta: np.ndarray) -> None:
    for seam in plan.seams:
        if seam.busbar_index == busbar_index:
            seam.centre = (delta @ np.append(seam.centre, 1.0))[:3]


def _land_spatter(plan: Plan, parts) -> None:
    """Spatter is placed before settling, beside seams that may since have
    moved; drop each droplet onto whatever surface is now below it, half-sunk
    as a fused droplet is."""
    spatter = [p for p in plan.particles if p.kind == "spatter"]
    if not spatter:
        return
    # Welding happens before the CMC and cover go on.
    everything = AxisCaster(
        trimesh.util.concatenate(
            [
                world_mesh(plan, parts, p)
                for p in plan.placements
                if p.present and p.ref[0] not in ("cmc", "cover")
            ]
        ),
        2,
    )
    centres = np.array([p.centre for p in spatter])
    tops = _heights_below(everything, centres)
    for particle, top in zip(spatter, tops):
        if not np.isnan(top):
            particle.centre = np.array([particle.centre[0], particle.centre[1], top])

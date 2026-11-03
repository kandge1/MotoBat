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

# A part needing more than this to come to rest has nothing under one side and
# would fall off, which is a broken setup rather than a defect.
MAX_TILT = np.radians(15.0)
# Below this a pose change is numerical noise, not movement.
EPS = 1e-7
ITERATIONS = 8
FAR = 1.0e3
BULGE_MARGIN = 5e-3
# Must be at least the side plate's remesh edge length (2.0).
PLATEAU = 2.0
# Spacing of points sampled along edges, where contact on a tilted face peaks.
EDGE_STEP = 0.1

# What each part rests on, in assembly order. Particles flagged as supports
# (debris, FOD) are added to every list.
ORDER: list[tuple[str, tuple[str, ...]]] = [
    ("carrier", ("cell", "terminal_pad", "pad", "end_plate", "side_plate")),
    ("busbar", ("cell", "terminal", "carrier")),
    # The CMC is not settled yet (fixlist E3): the standalone busbar_carrier.stl
    # has wide bosses where module_assembled has slim pegs through the CMC's
    # holes, so settled on this carrier the CMC rides 2.6 high. It stays at the
    # assembly's deck height until the carrier is re-exported. Then add:
    #   ("cmc", ("carrier", "busbar")),
    ("cover", ("end_plate", "side_plate", "carrier", "busbar", "cmc", "cell", "terminal")),
]

SUPPORT_PARTICLES = ("debris", "fod")


def world_mesh(plan: Plan, parts: dict[str, trimesh.Trimesh], placement: Placement):
    if placement.ref in plan.overrides:
        return plan.overrides[placement.ref].copy()
    return parts[placement.part].copy().apply_transform(placement.transform)


def _pad_only(parts, placement: Placement) -> trimesh.Trimesh:
    """terminal_neg without its merged fill plug, as a support for the carrier.

    Workaround for fixlist E5: the plug stands 5 mm proud with no pocket in the
    carrier above it, so a carrier settled onto the full part would ride 5 mm
    high in a clean module. Remove this once the carrier is re-exported with
    its pockets.
    """
    mesh = parts[placement.part]
    if placement.part != "terminal_neg":
        return mesh.copy().apply_transform(placement.transform)
    lo, hi = mesh.bounds
    box = trimesh.creation.box(bounds=[[lo[0] - 1, -11.0, lo[2] - 1], [hi[0] + 1, 11.0, hi[2] + 1]])
    pad = trimesh.boolean.intersection([mesh, box], engine="manifold")
    return pad.apply_transform(placement.transform)


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
        elif kind == "terminal" and "terminal_pad" in kinds:
            meshes.append(_pad_only(parts, placement))
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
    downward face, which catch edge and corner contact, plus a grid of upward
    rays, which catches contact in the middle of a face."""
    down = mesh.face_normals[:, 2] < -0.5
    corners = _edge_points(mesh, down, EDGE_STEP)

    lo, hi = mesh.bounds
    xs = np.arange(lo[0] + spacing / 2, hi[0], spacing)
    ys = np.arange(lo[1] + spacing / 2, hi[1], spacing)
    gx, gy = np.meshgrid(xs, ys)
    origins = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, lo[2] - 1.0)])
    hits, rays, _ = mesh.ray.intersects_location(
        origins, np.tile([0.0, 0.0, 1.0], (len(origins), 1)), multiple_hits=False
    )
    grid = hits if len(rays) else np.empty((0, 3))
    return np.vstack([corners, grid])


def _heights_below(support: trimesh.Trimesh, points: np.ndarray) -> np.ndarray:
    """Top of the support directly below each point's (x, y); NaN where none."""
    origins = np.column_stack([points[:, :2], np.full(len(points), FAR)])
    hits, rays, _ = support.ray.intersects_location(
        origins, np.tile([0.0, 0.0, -1.0], (len(points), 1)), multiple_hits=False
    )
    out = np.full(len(points), np.nan)
    out[rays] = hits[:, 2]
    return out


def _peaks_under(support: trimesh.Trimesh, part: trimesh.Trimesh):
    """The other side of the contact: every upward-facing vertex of the
    support that lies under the part, paired with the part's underside height
    directly above it. A grid on the part alone misses any support narrower
    than its spacing -- a 0.4 mm chip under a 12 mm busbar."""
    lo, hi = part.bounds
    up = support.face_normals[:, 2] > 0.5
    tri = support.triangles
    near = np.all((tri.max(axis=1)[:, :2] >= lo[:2]) & (tri.min(axis=1)[:, :2] <= hi[:2]), axis=1)
    tops = _edge_points(support, up & near, EDGE_STEP)
    inside = np.all((tops[:, :2] >= lo[:2]) & (tops[:, :2] <= hi[:2]), axis=1)
    tops = tops[inside]
    if not len(tops):
        return np.empty((0, 3)), np.empty(0)
    origins = np.column_stack([tops[:, :2], np.full(len(tops), lo[2] - 1.0)])
    hits, rays, _ = part.ray.intersects_location(
        origins, np.tile([0.0, 0.0, 1.0], (len(tops), 1)), multiple_hits=False
    )
    return hits, tops[rays, 2]


def _rest_delta(points: np.ndarray, heights: np.ndarray, centre: np.ndarray, ref) -> np.ndarray:
    keep = ~np.isnan(heights)
    if not keep.any():
        raise ValueError(f"{ref} has nothing under it to rest on")
    p, s = points[keep], heights[keep]
    dx, dy = p[:, 0] - centre[0], p[:, 1] - centre[1]
    a_ub = -np.column_stack([np.ones(len(p)), dy, -dx])
    b_ub = p[:, 2] - s
    result = linprog(
        c=[1.0, 0.0, 0.0],
        A_ub=a_ub,
        b_ub=b_ub,
        bounds=[(None, None), (-MAX_TILT, MAX_TILT), (-MAX_TILT, MAX_TILT)],
        method="highs",
    )
    if not result.success:
        raise ValueError(f"{ref} cannot come to rest: {result.message}")
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
    for _ in range(ITERATIONS):
        mesh = world_mesh(plan, parts, placement)
        points = _underside(mesh, spacing)
        heights = _heights_below(support, points)
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
    heights = _heights_below(can, points)
    keep = ~np.isnan(heights)
    lift = float(np.max(heights[keep] - points[keep, 2])) if keep.any() else 0.0
    if lift > EPS:
        placement.compose(_z(lift))


def _z(dz: float) -> np.ndarray:
    t = np.eye(4)
    t[2, 3] = dz
    return t


# -- compliant parts -------------------------------------------------------

def _face_x(mesh: trimesh.Trimesh, yz: np.ndarray, from_x: float, direction: float) -> np.ndarray:
    """First X a ray at each (y, z) meets on `mesh`, travelling from `from_x`."""
    origins = np.column_stack([np.full(len(yz), from_x), yz])
    hits, rays, _ = mesh.ray.intersects_location(
        origins, np.tile([direction, 0.0, 0.0], (len(yz), 1)), multiple_hits=False
    )
    out = np.full(len(yz), np.nan)
    out[rays] = hits[:, 0]
    return out


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
        right = _face_x(meshes[k], yz, FAR, -1.0)
        left = _face_x(meshes[k], yz, -FAR, 1.0)
        if k == 0 and np.nanmin(left) < -end_x - 1e-6:
            raise ValueError(f"cell 0 passes {-end_x - np.nanmin(left):.2f} mm into end plate a")
        if k == cfg.n_cells - 1 and np.nanmax(right) > end_x + 1e-6:
            raise ValueError(f"cell {k} passes {np.nanmax(right) - end_x:.2f} mm into end plate b")
        if k + 1 not in meshes:
            continue
        nxt = _face_x(meshes[k + 1], yz, -FAR, 1.0)
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
    stack = trimesh.util.concatenate(cells)
    xs = np.arange(-cfg.module_l / 2, cfg.module_l / 2, 1.0)
    zs = np.arange(cfg.z_cell_bottom + 0.5, cfg.z_cell_top, 1.0)
    gx, gz = np.meshgrid(xs, zs)
    xz = np.column_stack([gx.ravel(), gz.ravel()])
    inner = cfg.cell_w / 2

    for placement in plan.of_kind("side_plate"):
        if not placement.present:
            continue
        sign = 1.0 if placement.ref[1] == "right" else -1.0
        origins = np.column_stack([xz[:, 0], np.zeros(len(xz)), xz[:, 1]])
        hits, rays, _ = stack.ray.intersects_location(
            origins, np.tile([0.0, sign, 0.0], (len(xz), 1)), multiple_hits=True
        )
        if not len(rays):
            continue
        reach = np.zeros(len(xz))
        np.maximum.at(reach, rays, sign * hits[:, 1])
        push = reach - inner
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
        if not (dirty & ({kind, "particle"} | set(kinds) | ({"terminal"} if "terminal_pad" in kinds else set()))):
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

    _land_spatter(plan, parts)


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
    everything = trimesh.util.concatenate(
        [
            world_mesh(plan, parts, p)
            for p in plan.placements
            if p.present and p.ref[0] not in ("cmc", "cover")
        ]
    )
    centres = np.array([p.centre for p in spatter])
    tops = _heights_below(everything, centres)
    for particle, top in zip(spatter, tops):
        if not np.isnan(top):
            particle.centre = np.array([particle.centre[0], particle.centre[1], top])

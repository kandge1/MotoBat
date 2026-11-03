"""Assemble a VDA 355 module from the normalised CAD parts.

Three steps, kept apart on purpose: `build_plan` places every part, `defects`
perturbs the plan, `realise` turns it into triangles. `generate` runs all three
and returns the `(mesh, labels)` seam everything downstream consumes.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import replace
from pathlib import Path

import numpy as np
import trimesh

from .config import Config, load
from .groundtruth import (
    STAGES,
    BusbarGT,
    CellGT,
    Labels,
    Polarity,
    SeamGT,
    Stage,
    TerminalGT,
)
from .parts import PARTS, load_all, mesh_dir
from .settle import particle_mesh, settle
from .plan import Dent, Placement, Plan, SeamPlan, rotation_z, translation


def is_flipped(cfg: Config, cell_index: int) -> bool:
    """Odd parallel groups face the other way, which is what makes the busbar
    pattern non-trivial and a reversed cell a meaningful defect (spec §2)."""
    return cfg.group_of(cell_index) % 2 == 1


# Where each terminal sits on an unflipped cell, in the cell's own frame. Read
# off the user's Fusion cell_assembled: positive at -Y, negative at +Y with the
# fill plug on its inboard side. test_module checks every cell against it.
TERMINAL_LOCAL_Y = {"pos": -1.0, "neg": 1.0}


def cell_transform(cfg: Config, cell_index: int) -> np.ndarray:
    """A flipped cell is the whole cell turned 180 degrees about Z -- can,
    terminals and fill plug together -- not its terminals swapped."""
    t = translation(cfg.cell_centre_x(cell_index), 0, cfg.z_cell_bottom)
    if is_flipped(cfg, cell_index):
        t = t @ rotation_z(180.0)
    return t


def terminal_local(cfg: Config, polarity: Polarity) -> np.ndarray:
    return translation(0, TERMINAL_LOCAL_Y[polarity] * cfg.terminal_y, cfg.cell_h)


def terminal_y(cfg: Config, cell_index: int, polarity: Polarity) -> float:
    return float((cell_transform(cfg, cell_index) @ terminal_local(cfg, polarity))[1, 3])


def busbar_plan(cfg: Config) -> list[tuple[str, tuple[tuple[int, Polarity], ...]]]:
    """3P4S: a series busbar bridges each adjacent pair of parallel groups, and
    an end busbar caps the first group's negative and the last group's positive.
    For 3P4S that is three series plus two end busbars."""
    size = cfg.n_parallel
    groups = [tuple(range(g * size, (g + 1) * size)) for g in range(cfg.n_series)]

    plan: list[tuple[str, tuple[tuple[int, Polarity], ...]]] = []
    for g in range(cfg.n_series - 1):
        terminals = tuple(
            [(c, "pos") for c in groups[g]] + [(c, "neg") for c in groups[g + 1]]
        )
        plan.append(("series", terminals))
    plan.append(("end", tuple((c, "neg") for c in groups[0])))
    plan.append(("end", tuple((c, "pos") for c in groups[-1])))
    return plan


def busbar_length(cfg: Config, kind: str) -> float:
    return cfg.busbar_end_l if kind == "end" else cfg.busbar_series_l


def seam_overlap_x(cfg: Config, busbar_centre_x: float, length: float, terminal_x: float):
    """X interval where the busbar actually covers the terminal pad.

    At an end terminal the busbar stops 4.25 past the terminal centre while the
    pad runs 9.25, so the overlap is 13.5 of the pad's 18.5 rather than all of
    it. A seam has to sit inside that or it would be welding air (spec §5).
    """
    lo = max(busbar_centre_x - length / 2, terminal_x - cfg.terminal_d / 2)
    hi = min(busbar_centre_x + length / 2, terminal_x + cfg.terminal_d / 2)
    return lo, hi


def stitch_offsets(cfg: Config) -> list[float]:
    if cfg.weld_stitches == 1:
        return [0.0]
    span = 2 * cfg.weld_stitch_offset_y
    step = span / (cfg.weld_stitches - 1)
    offsets = [-cfg.weld_stitch_offset_y + i * step for i in range(cfg.weld_stitches)]
    limit = cfg.busbar_w / 2 - cfg.weld_edge_margin
    for dy in offsets:
        if abs(dy) + cfg.weld_stitch_w / 2 > limit:
            raise ValueError(
                f"weld stitch at y offset {dy} comes within {cfg.weld_edge_margin} mm "
                f"of a busbar edge (busbar_w={cfg.busbar_w})"
            )
    return offsets


def build_plan(cfg: Config, stage: Stage) -> Plan:
    if stage not in STAGES:
        raise ValueError(f"unknown stage {stage!r}, expected one of {STAGES}")
    plan = Plan(cfg=cfg, stage=stage)

    plan.placements.append(
        Placement("cooling_plate", translation(0, 0, 0), ("cooling_plate",))
    )

    for k in range(cfg.n_cells):
        cell = cell_transform(cfg, k)
        plan.placements.append(Placement("cell_can", cell, ("cell", k)))
        for polarity in ("pos", "neg"):
            plan.placements.append(
                Placement(
                    f"terminal_{polarity}",
                    cell @ terminal_local(cfg, polarity),
                    ("terminal", k, polarity),
                )
            )

    for k in range(cfg.n_cells - 1):
        px = cfg.cell_centre_x(k) + cfg.cell_t / 2 + cfg.pad_t / 2
        plan.placements.append(
            Placement("interface_pad", translation(px, 0, cfg.z_cell_bottom), ("pad", k))
        )

    end_x = cfg.module_l / 2 - cfg.end_plate_t / 2
    for side, sign in (("a", -1.0), ("b", 1.0)):
        plan.placements.append(
            Placement(
                "end_plate",
                translation(sign * end_x, 0, cfg.z_cell_bottom),
                ("end_plate", side),
            )
        )
    side_y = cfg.cell_w / 2 + cfg.side_plate_t / 2
    for side, sign in (("left", -1.0), ("right", 1.0)):
        plan.placements.append(
            Placement(
                "side_plate",
                translation(0, sign * side_y, cfg.z_cell_bottom),
                ("side_plate", side),
            )
        )

    if stage in ("pre_weld", "post_weld"):
        plan.placements.append(
            Placement(
                "busbar_carrier", translation(0, 0, cfg.z_cell_top), ("carrier",)
            )
        )
        for index, (kind, terms) in enumerate(busbar_plan(cfg)):
            ys = {terminal_y(cfg, c, p) for c, p in terms}
            if len(ys) != 1:
                raise ValueError(
                    f"busbar {index} ({kind}) spans both terminal rows: {sorted(ys)}"
                )
            by = ys.pop()
            xs = [cfg.cell_centre_x(c) for c, _ in terms]
            bx = (min(xs) + max(xs)) / 2
            plan.placements.append(
                Placement(
                    f"busbar_{kind}",
                    translation(bx, by, cfg.z_terminal_top),
                    ("busbar", index),
                    )
            )

    if stage == "post_weld":
        offsets = stitch_offsets(cfg)
        for index, (kind, terms) in enumerate(busbar_plan(cfg)):
            placement = plan.find(("busbar", index))
            bx, by, bz = placement.origin
            length = busbar_length(cfg, kind)
            for cell_index, _polarity in terms:
                lo, hi = seam_overlap_x(cfg, bx, length, cfg.cell_centre_x(cell_index))
                if hi - lo < cfg.weld_stitch_l:
                    raise ValueError(
                        f"weld_stitch_l={cfg.weld_stitch_l} does not fit the "
                        f"{hi - lo:.2f} mm busbar/pad overlap at cell {cell_index}"
                    )
                sx = (lo + hi) / 2
                for dy in offsets:
                    plan.seams.append(
                        SeamPlan(
                            busbar_index=index,
                            cell_index=cell_index,
                            centre=np.array([sx, by + dy, bz + cfg.busbar_t]),
                            length=cfg.weld_stitch_l,
                            width=cfg.weld_stitch_w,
                            height=cfg.weld_bead_h,
                        )
                    )

        plan.placements.append(
            Placement("cmc", translation(0, 0, cfg.z_carrier_deck), ("cmc",))
        )
        plan.placements.append(
            Placement("cover", translation(0, 0, cfg.z_wall_top), ("cover",))
        )

    return plan


def _apply_dent(mesh: trimesh.Trimesh, dent: Dent) -> trimesh.Trimesh:
    """Push vertices in along `direction`, falling off smoothly to the rim.

    The part has to be refined first. A flat CAD face is two triangles with four
    corner vertices and nothing in between, so displacing "vertices near the
    dent" would move either nothing or the whole face. Subdividing to roughly a
    third of the dent radius gives the deformation something to act on, and
    doing it to the whole part rather than a patch keeps the mesh free of the
    T-junctions a local subdivision would leave.
    """
    target = max(dent.radius / 3.0, 0.5)
    vertices, faces = trimesh.remesh.subdivide_to_size(
        mesh.vertices, mesh.faces, max_edge=target
    )
    mesh = trimesh.Trimesh(vertices, faces, process=False)

    distance = np.linalg.norm(mesh.vertices - dent.centre, axis=1)
    inside = distance < dent.radius
    if not inside.any():
        raise ValueError(
            f"dent at {dent.centre.tolist()} radius {dent.radius} touches no part "
            f"of {dent.ref}, whose bounds are {mesh.bounds.tolist()}"
        )
    falloff = 0.5 * (1.0 + np.cos(np.pi * distance[inside] / dent.radius))
    direction = dent.direction / np.linalg.norm(dent.direction)
    mesh.vertices[inside] += np.outer(falloff * dent.depth, direction)
    return mesh


# Body groups, keyed by the first element of a placement ref. A viewer shows or
# hides these as a unit -- lift the cover off, look at the busbars.
GROUPS = {
    "cooling_plate": "cooling_plate",
    "cell": "cells",
    "pad": "pads",
    "end_plate": "end_plates",
    "side_plate": "side_plates",
    "carrier": "carrier",
    "busbar": "busbars",
    "cmc": "cmc",
    "cover": "cover",
}


def _solids(plan: Plan, parts: dict[str, trimesh.Trimesh]):
    """Yield (group, mesh) for every present body, in one fixed order."""
    for placement in plan.placements:
        if not placement.present:
            continue
        if placement.ref in plan.overrides:
            mesh = plan.overrides[placement.ref].copy()
        else:
            mesh = parts[placement.part].copy()
            mesh.apply_transform(placement.transform)
        for dent in plan.dents:
            if dent.ref == placement.ref:
                mesh = _apply_dent(mesh, dent)
        kind = placement.ref[0]
        # Terminals split by polarity, so a viewer can tell them apart.
        group = f"terminals_{placement.ref[2]}" if kind == "terminal" else GROUPS[kind]
        yield group, mesh

    for seam in plan.seams:
        if not seam.present:
            continue
        box = trimesh.creation.box(extents=(seam.length, seam.width, abs(seam.height)))
        box.apply_translation(seam.centre + np.array([0.0, 0.0, seam.height / 2]))
        yield "welds", box

    for particle in plan.particles:
        yield "fod", particle_mesh(particle)


def realise(plan: Plan, parts: dict[str, trimesh.Trimesh]) -> trimesh.Trimesh:
    return trimesh.util.concatenate([mesh for _, mesh in _solids(plan, parts)])


def realise_groups(
    plan: Plan, parts: dict[str, trimesh.Trimesh]
) -> dict[str, trimesh.Trimesh]:
    """The same solids as `realise`, one mesh per body group. For viewing only:
    the merged mesh from `realise` stays the scan target."""
    buckets: dict[str, list[trimesh.Trimesh]] = {}
    for group, mesh in _solids(plan, parts):
        buckets.setdefault(group, []).append(mesh)
    return {g: trimesh.util.concatenate(ms) for g, ms in buckets.items()}


def _box(lo, hi) -> np.ndarray:
    return np.array([lo, hi], dtype=float)


def _part_box(part: trimesh.Trimesh, transform: np.ndarray) -> np.ndarray:
    """Box of the part's local bounding box after `transform`. Conservative for
    a rotated part, which is the right way to err for a box drawn around it."""
    corners = trimesh.bounds.corners(part.bounds)
    moved = trimesh.transform_points(corners, transform)
    return _box(moved.min(axis=0), moved.max(axis=0))


def _seam_box(seam: SeamPlan) -> np.ndarray:
    half = np.array([seam.length / 2, seam.width / 2, 0.0])
    lo = seam.centre - half
    hi = seam.centre + half
    lo[2], hi[2] = sorted((seam.centre[2], seam.centre[2] + seam.height))
    return _box(lo, hi)


def defect_bounds(plan: Plan, parts: dict[str, trimesh.Trimesh]) -> list[np.ndarray]:
    """One (min, max) box per recorded defect, spanning where everything it
    touched should be (a fresh nominal plan) and where it ended up. Seams are
    matched by index: injectors edit seams in place and never reorder them."""
    nominal = build_plan(plan.cfg, plan.stage)
    boxes = []
    for target in plan.targets:
        pieces = []
        for ref in target.refs:
            want, got = nominal.find(ref), plan.find(ref)
            pieces.append(_part_box(parts[want.part], want.transform))
            if got.present:
                pieces.append(_part_box(parts[got.part], got.transform))
        for i in target.seams:
            pieces.append(_seam_box(nominal.seams[i]))
            if plan.seams[i].present:
                pieces.append(_seam_box(plan.seams[i]))
        if target.region is not None:
            pieces.append(_box(*target.region))
        if target.particle is not None:
            pieces.append(_box(*particle_mesh(plan.particles[target.particle]).bounds))
        stack = np.array(pieces)
        boxes.append(_box(stack[:, 0].min(axis=0), stack[:, 1].max(axis=0)))
    return boxes


def collect_labels(
    plan: Plan, seed: int, parts: dict[str, trimesh.Trimesh] | None = None
) -> Labels:
    """Ground truth read off the final plan, so it describes the module that was
    actually built rather than the one that was intended."""
    cfg = plan.cfg
    cells, terminals, busbars, seams = [], [], [], []

    for placement in plan.of_kind("cell"):
        if not placement.present:
            continue
        k = placement.ref[1]
        yaw = np.degrees(np.arctan2(placement.transform[1, 0], placement.transform[0, 0]))
        cells.append(
            CellGT(
                index=k,
                group=cfg.group_of(k),
                flipped=is_flipped(cfg, k),
                centre=tuple(placement.origin),
                yaw_deg=float(yaw),
            )
        )

    for placement in plan.of_kind("terminal"):
        if not placement.present:
            continue
        _, k, polarity = placement.ref
        top = placement.transform @ np.array([0.0, 0.0, cfg.terminal_h, 1.0])
        normal = placement.transform[:3, :3] @ np.array([0.0, 0.0, 1.0])
        terminals.append(
            TerminalGT(
                cell_index=k,
                polarity=polarity,
                centre=tuple(placement.origin),
                top_z=float(top[2]),
                normal=tuple(normal),
            )
        )

    layout = busbar_plan(cfg)
    for placement in plan.of_kind("busbar"):
        if not placement.present:
            continue
        index = placement.ref[1]
        kind, terms = layout[index]
        busbars.append(
            BusbarGT(
                index=index,
                kind=kind,
                centre=tuple(placement.origin),
                underside_z=float(placement.origin[2]),
                terminals=terms,
            )
        )

    for seam in plan.seams:
        if not seam.present:
            continue
        half = np.array([seam.length / 2, 0.0, 0.0])
        seams.append(
            SeamGT(
                busbar_index=seam.busbar_index,
                cell_index=seam.cell_index,
                start=tuple(seam.centre - half),
                end=tuple(seam.centre + half),
                width=seam.width,
            )
        )

    return Labels(
        stage=plan.stage,
        config_name=cfg.name,
        seed=seed,
        cells=tuple(cells),
        terminals=tuple(terminals),
        busbars=tuple(busbars),
        seams=tuple(seams),
        defects=_with_bounds(plan, parts),
    )


def _with_bounds(plan: Plan, parts) -> tuple:
    if parts is None or not plan.defects:
        return tuple(plan.defects)
    out = []
    for d, t, b in zip(plan.defects, plan.targets, defect_bounds(plan, parts)):
        # Settling may have moved a seam or a particle after it was recorded;
        # the label describes where it ended up.
        location = d.location
        if t.particle is not None:
            location = tuple(map(float, plan.particles[t.particle].centre))
        elif len(t.seams) == 1:
            location = tuple(map(float, plan.seams[t.seams[0]].centre))
        out.append(
            replace(
                d,
                location=location,
                bounds=(tuple(map(float, b[0])), tuple(map(float, b[1]))),
            )
        )
    return tuple(out)


DefectFn = Callable[[Plan, np.random.Generator], None]


def injected_plan(
    stage: Stage = "post_weld",
    cfg: Config | None = None,
    seed: int = 0,
    defects: Sequence[DefectFn] = (),
    parts: dict[str, trimesh.Trimesh] | None = None,
) -> Plan:
    """Build the plan for a stage, run every injector that applies to it, then
    settle it so no two parts overlap (settle.py)."""
    cfg = cfg or load()
    plan = build_plan(cfg, stage)

    rng = np.random.default_rng(seed)
    reached = STAGES.index(stage)
    for inject in defects:
        # A defect that targets a part this stage has not placed yet is skipped,
        # not an error -- the same defect list describes all three stages.
        if STAGES.index(getattr(inject, "min_stage", STAGES[0])) > reached:
            continue
        inject(plan, rng)
    settle(plan, parts if parts is not None else load_all(cfg))
    return plan


def generate(
    stage: Stage = "post_weld",
    cfg: Config | None = None,
    seed: int = 0,
    defects: Sequence[DefectFn] = (),
    mesh_dir: Path | None = None,
) -> tuple[trimesh.Trimesh, Labels]:
    cfg = cfg or load()
    parts = load_all(cfg, mesh_dir)
    plan = injected_plan(stage, cfg, seed, defects, parts)
    return realise(plan, parts), collect_labels(plan, seed, parts)


def export(
    outdir: Path | str,
    cfg: Config | None = None,
    seed: int = 0,
    defects: Sequence[DefectFn] = (),
) -> list[Path]:
    """Write every stage as STL plus its labels as JSON, and each stage's body
    groups as <stage>/<group>.stl so a viewer can hide them one at a time."""
    cfg = cfg or load()
    parts = load_all(cfg)
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    written = []
    for stage in STAGES:
        plan = injected_plan(stage, cfg, seed, defects, parts)
        stl = outdir / f"{stage}.stl"
        realise(plan, parts).export(stl)
        js = outdir / f"{stage}.labels.json"
        js.write_text(collect_labels(plan, seed, parts).to_json())
        written += [stl, js]

        # Clear first: a clean run into a dir that once held a defective one
        # would otherwise leave its fod.stl behind, and the viewer would show it.
        groups_dir = outdir / stage
        groups_dir.mkdir(exist_ok=True)
        for old in groups_dir.glob("*.stl"):
            old.unlink()
        for group, mesh in realise_groups(plan, parts).items():
            path = groups_dir / f"{group}.stl"
            mesh.export(path)
            written.append(path)
    return written


def export_cells(outdir: Path | str, cells: Sequence[int], cfg: Config | None = None) -> list[Path]:
    """Just the named cells and their terminals, clean, as the generator places
    them -- plus the user's Fusion cell_assembled under the same cell transforms
    in `reference/`, to overlay as a ghost. Any disagreement between the two is
    a placement bug, not a defect."""
    cfg = cfg or load()
    parts = load_all(cfg)
    outdir = Path(outdir)
    stage: Stage = "pre_busbar"
    plan = build_plan(cfg, stage)
    plan.placements = [
        p for p in plan.placements if p.ref[0] in ("cell", "terminal") and p.ref[1] in cells
    ]
    groups_dir = outdir / stage
    groups_dir.mkdir(parents=True, exist_ok=True)
    for old in groups_dir.glob("*.stl"):
        old.unlink()
    written = [outdir / f"{stage}.stl", outdir / f"{stage}.labels.json"]
    realise(plan, parts).export(written[0])
    written[1].write_text(collect_labels(plan, 0, parts).to_json())
    for group, mesh in realise_groups(plan, parts).items():
        written.append(groups_dir / f"{group}.stl")
        mesh.export(written[-1])

    ref = (outdir / "reference")
    ref.mkdir(exist_ok=True)
    can = trimesh.load_mesh(mesh_dir() / "cell_can.stl")
    merged = trimesh.load_mesh(mesh_dir() / "cell_assembled.stl")
    rot = trimesh.transformations.rotation_matrix(
        np.radians(PARTS["cell_can"].rot_z_deg), [0, 0, 1]
    )
    can.apply_transform(rot)
    lo, hi = can.bounds
    merged.apply_transform(translation(-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]) @ rot)
    copies = [merged.copy().apply_transform(cell_transform(cfg, k)) for k in cells]
    written.append(ref / f"{stage}.stl")
    trimesh.util.concatenate(copies).export(written[-1])
    return written

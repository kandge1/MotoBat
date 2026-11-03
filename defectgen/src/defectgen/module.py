"""Assemble a VDA 355 module from the normalised CAD parts.

Three steps, kept apart on purpose: `build_plan` places every part, `defects`
perturbs the plan, `realise` turns it into triangles. `generate` runs all three
and returns the `(mesh, labels)` seam everything downstream consumes.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
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
from .parts import load_all
from .plan import Dent, Placement, Plan, SeamPlan, rotation_z, translation


def is_flipped(cfg: Config, cell_index: int) -> bool:
    """Odd parallel groups face the other way, which is what makes the busbar
    pattern non-trivial and a reversed cell a meaningful defect (spec §2)."""
    return cfg.group_of(cell_index) % 2 == 1


def terminal_y(cfg: Config, cell_index: int, polarity: Polarity) -> float:
    pos_y = -cfg.terminal_y if is_flipped(cfg, cell_index) else cfg.terminal_y
    return pos_y if polarity == "pos" else -pos_y


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
        cx = cfg.cell_centre_x(k)
        plan.placements.append(
            Placement("cell_can", translation(cx, 0, cfg.z_cell_bottom), ("cell", k))
        )
        for polarity in ("pos", "neg"):
            plan.placements.append(
                Placement(
                    f"terminal_{polarity}",
                    translation(cx, terminal_y(cfg, k, polarity), cfg.z_cell_top),
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
            Placement("cmc", translation(0, 0, cfg.z_carrier_top), ("cmc",))
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


def realise(plan: Plan, parts: dict[str, trimesh.Trimesh]) -> trimesh.Trimesh:
    solids: list[trimesh.Trimesh] = []

    for placement in plan.placements:
        if not placement.present:
            continue
        mesh = parts[placement.part].copy()
        mesh.apply_transform(placement.transform)
        for dent in plan.dents:
            if dent.ref == placement.ref:
                mesh = _apply_dent(mesh, dent)
        solids.append(mesh)

    for seam in plan.seams:
        if not seam.present:
            continue
        box = trimesh.creation.box(extents=(seam.length, seam.width, abs(seam.height)))
        box.apply_translation(seam.centre + np.array([0.0, 0.0, seam.height / 2]))
        solids.append(box)

    for particle in plan.particles:
        blob = trimesh.creation.icosphere(subdivisions=1, radius=particle.radius)
        blob.apply_translation(particle.centre)
        solids.append(blob)

    return trimesh.util.concatenate(solids)


def collect_labels(plan: Plan, seed: int) -> Labels:
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
        defects=tuple(plan.defects),
    )


DefectFn = Callable[[Plan, np.random.Generator], None]


def generate(
    stage: Stage = "post_weld",
    cfg: Config | None = None,
    seed: int = 0,
    defects: Sequence[DefectFn] = (),
    mesh_dir: Path | None = None,
) -> tuple[trimesh.Trimesh, Labels]:
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

    mesh = realise(plan, load_all(cfg, mesh_dir))
    return mesh, collect_labels(plan, seed)


def export(
    outdir: Path | str,
    cfg: Config | None = None,
    seed: int = 0,
    defects: Sequence[DefectFn] = (),
) -> list[Path]:
    """Write every stage as STL plus its labels as JSON."""
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    written = []
    for stage in STAGES:
        mesh, labels = generate(stage, cfg=cfg, seed=seed, defects=defects)
        stl = outdir / f"{stage}.stl"
        mesh.export(stl)
        js = outdir / f"{stage}.labels.json"
        js.write_text(labels.to_json())
        written += [stl, js]
    return written

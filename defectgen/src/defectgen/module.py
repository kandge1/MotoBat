"""Assemble a VDA 355 module from the normalised CAD parts.

`generate(stage)` returns `(mesh, labels)` -- the seam everything downstream
consumes. Defects are off; task 1.2 perturbs the same placements.
"""

from __future__ import annotations

from pathlib import Path

import trimesh

from . import welds
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


def _placed(mesh: trimesh.Trimesh, x: float, y: float, z: float) -> trimesh.Trimesh:
    out = mesh.copy()
    out.apply_translation((x, y, z))
    return out


def generate(
    stage: Stage = "post_weld",
    cfg: Config | None = None,
    seed: int = 0,
    mesh_dir: Path | None = None,
) -> tuple[trimesh.Trimesh, Labels]:
    if stage not in STAGES:
        raise ValueError(f"unknown stage {stage!r}, expected one of {STAGES}")
    cfg = cfg or load()
    part = load_all(cfg, mesh_dir)

    solids: list[trimesh.Trimesh] = []
    cells: list[CellGT] = []
    terminals: list[TerminalGT] = []
    busbars: list[BusbarGT] = []
    seams: list[SeamGT] = []

    solids.append(_placed(part["cooling_plate"], 0.0, 0.0, 0.0))

    for k in range(cfg.n_cells):
        cx = cfg.cell_centre_x(k)
        flipped = is_flipped(cfg, k)
        solids.append(_placed(part["cell_can"], cx, 0.0, cfg.z_cell_bottom))
        cells.append(
            CellGT(
                index=k,
                group=cfg.group_of(k),
                flipped=flipped,
                centre=(cx, 0.0, cfg.z_cell_bottom),
                yaw_deg=180.0 if flipped else 0.0,
            )
        )
        for polarity in ("pos", "neg"):
            ty = terminal_y(cfg, k, polarity)
            solids.append(
                _placed(part[f"terminal_{polarity}"], cx, ty, cfg.z_cell_top)
            )
            terminals.append(
                TerminalGT(
                    cell_index=k,
                    polarity=polarity,
                    centre=(cx, ty, cfg.z_cell_top),
                    top_z=cfg.z_terminal_top,
                    normal=(0.0, 0.0, 1.0),
                )
            )

    for k in range(cfg.n_cells - 1):
        px = cfg.cell_centre_x(k) + cfg.cell_t / 2 + cfg.pad_t / 2
        solids.append(_placed(part["interface_pad"], px, 0.0, cfg.z_cell_bottom))

    end_x = cfg.module_l / 2 - cfg.end_plate_t / 2
    for sign in (-1.0, 1.0):
        solids.append(
            _placed(part["end_plate"], sign * end_x, 0.0, cfg.z_cell_bottom)
        )
    side_y = cfg.cell_w / 2 + cfg.side_plate_t / 2
    for sign in (-1.0, 1.0):
        solids.append(
            _placed(part["side_plate"], 0.0, sign * side_y, cfg.z_cell_bottom)
        )

    if stage in ("pre_weld", "post_weld"):
        solids.append(_placed(part["busbar_carrier"], 0.0, 0.0, cfg.z_cell_top))

        for index, (kind, terms) in enumerate(busbar_plan(cfg)):
            ys = {terminal_y(cfg, c, p) for c, p in terms}
            if len(ys) != 1:
                raise ValueError(
                    f"busbar {index} ({kind}) spans both terminal rows: {sorted(ys)}"
                )
            by = ys.pop()
            xs = [cfg.cell_centre_x(c) for c, _ in terms]
            bx = (min(xs) + max(xs)) / 2

            solids.append(
                _placed(part[f"busbar_{kind}"], bx, by, cfg.z_terminal_top)
            )
            busbars.append(
                BusbarGT(
                    index=index,
                    kind=kind,
                    centre=(bx, by, cfg.z_terminal_top),
                    underside_z=cfg.z_terminal_top,
                    terminals=terms,
                )
            )

    if stage == "post_weld":
        for busbar in busbars:
            length = cfg.busbar_end_l if busbar.kind == "end" else cfg.busbar_series_l
            seam_meshes, seam_labels = welds.seams_for_busbar(cfg, busbar, length)
            solids.extend(seam_meshes)
            seams.extend(seam_labels)

        solids.append(_placed(part["cmc"], 0.0, 0.0, cfg.z_carrier_top))
        solids.append(_placed(part["cover"], 0.0, 0.0, cfg.z_wall_top))

    mesh = trimesh.util.concatenate(solids)
    labels = Labels(
        stage=stage,
        config_name=cfg.name,
        seed=seed,
        cells=tuple(cells),
        terminals=tuple(terminals),
        busbars=tuple(busbars),
        seams=tuple(seams),
    )
    return mesh, labels


def export(outdir: Path | str, cfg: Config | None = None, seed: int = 0) -> list[Path]:
    """Write every stage as STL plus its labels as JSON."""
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    written = []
    for stage in STAGES:
        mesh, labels = generate(stage, cfg=cfg, seed=seed)
        stl = outdir / f"{stage}.stl"
        mesh.export(stl)
        js = outdir / f"{stage}.labels.json"
        js.write_text(labels.to_json())
        written += [stl, js]
    return written

"""Nominal laser weld seams. Generated geometry, not CAD (spec §7).

A busbar-to-terminal joint is a lap weld: the beam fires down through the 1.5 mm
busbar and fuses into the pad beneath, leaving a bead proud of the busbar top.
What a post-weld scan measures is that bead, so that is what is modelled.

Seam *defects* -- missing, short, offset, wrong width -- belong to task 1.2.
"""

from __future__ import annotations

import trimesh

from .config import Config
from .groundtruth import BusbarGT, SeamGT


def _overlap_x(cfg: Config, busbar_centre_x: float, busbar_len: float, terminal_x: float):
    """X interval where the busbar actually covers the terminal pad.

    At an end terminal the busbar stops 4.25 past the terminal centre while the
    pad runs 9.25, so the overlap is 13.5 of the pad's 18.5 rather than all of
    it. A seam has to sit inside that or it would be welding air (spec §5).
    """
    lo = max(busbar_centre_x - busbar_len / 2, terminal_x - cfg.terminal_d / 2)
    hi = min(busbar_centre_x + busbar_len / 2, terminal_x + cfg.terminal_d / 2)
    return lo, hi


def seams_for_busbar(
    cfg: Config, busbar: BusbarGT, busbar_len: float
) -> tuple[list[trimesh.Trimesh], list[SeamGT]]:
    meshes: list[trimesh.Trimesh] = []
    labels: list[SeamGT] = []

    half_w = cfg.busbar_w / 2
    offsets = (
        [0.0]
        if cfg.weld_stitches == 1
        else [
            -cfg.weld_stitch_offset_y + i * (2 * cfg.weld_stitch_offset_y / (cfg.weld_stitches - 1))
            for i in range(cfg.weld_stitches)
        ]
    )
    for dy in offsets:
        if abs(dy) + cfg.weld_stitch_w / 2 > half_w - cfg.weld_edge_margin:
            raise ValueError(
                f"weld stitch at y offset {dy} comes within {cfg.weld_edge_margin} mm "
                f"of a busbar edge (busbar_w={cfg.busbar_w})"
            )

    z0 = busbar.underside_z + cfg.busbar_t

    for cell_index, _polarity in busbar.terminals:
        tx = cfg.cell_centre_x(cell_index)
        lo, hi = _overlap_x(cfg, busbar.centre[0], busbar_len, tx)
        if hi - lo < cfg.weld_stitch_l:
            raise ValueError(
                f"weld_stitch_l={cfg.weld_stitch_l} does not fit the "
                f"{hi - lo:.2f} mm busbar/pad overlap at cell {cell_index}"
            )
        sx = (lo + hi) / 2

        for dy in offsets:
            sy = busbar.centre[1] + dy
            box = trimesh.creation.box(
                extents=(cfg.weld_stitch_l, cfg.weld_stitch_w, cfg.weld_bead_h)
            )
            box.apply_translation((sx, sy, z0 + cfg.weld_bead_h / 2))
            meshes.append(box)
            labels.append(
                SeamGT(
                    busbar_index=busbar.index,
                    cell_index=cell_index,
                    start=(sx - cfg.weld_stitch_l / 2, sy, z0),
                    end=(sx + cfg.weld_stitch_l / 2, sy, z0),
                    width=cfg.weld_stitch_w,
                )
            )

    return meshes, labels

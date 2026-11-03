"""Write the three stages to disk so they can be opened in a mesh viewer.

    python -m defectgen --out data/preview
    python -m defectgen --out data/preview_bad --demo-defects
    python -m defectgen --out data/preview_real --demo-defects realistic
    python -m defectgen --out data/cell_check --cells 0 3

The default demo set is deliberately exaggerated: it exists to check that each
injector moves the thing it claims to, not to look like a real module.
`realistic` puts the same classes at the scale a line actually produces --
tenths of a millimetre, invisible by eye, which is what the boxes in the
viewer are for. Those numbers are assumed (A), not sourced; see
status_update.md "Source before claiming results".
"""

from __future__ import annotations

import argparse
from pathlib import Path

from . import defects
from .config import load
from .module import export, export_cells


def demo_defects(cfg):
    """One of each primary class, sized to be obvious in a viewer -- but still
    physically possible. The cell error is near the most the 1.5 mm pads can
    absorb: 0.4 deg of yaw moves a corner 74 sin(0.4) = 0.52 mm, plus 0.5 of
    shift, against 1.2 mm of crushable foam. Any more and settling refuses it,
    because the cells would collide."""
    return [
        defects.cell_placement(4, dx=0.5, dy=1.5, dtheta_deg=0.4),
        defects.terminal_height(6, "pos", dz=1.5),
        defects.terminal_tilt(7, "neg", deg=6.0),
        defects.missing_cell(9),
        defects.reversed_polarity(1),
        defects.busbar_pose(0, dx=3.0, dtheta_deg=1.5),
        defects.busbar_liftoff(2, gap=2.0),
        defects.missing_weld(1, 5),
        defects.short_weld(1, 4, fraction=0.4),
        defects.spatter(count=25, radius=0.6),
        defects.handling_damage(
            ("cell", 0),
            depth=2.0,
            radius=12.0,
            centre=(cfg.cell_centre_x(0), cfg.cell_w / 2, 50.0),
            direction=(0.0, -1.0, 0.0),
        ),
        defects.missing_pad(3),
        defects.cover_misalignment(dx=4.0, dy=2.0, dtheta_deg=1.0),
    ]


def realistic_defects(cfg):
    """Same classes and sites as `demo_defects`, at plausible line severities.

    Each sits a little past the tolerance it would be judged against, where one
    exists, so it is a defect rather than normal scatter.
    """
    return [
        # pick-and-place repeatability is a few tenths; this is just past it
        defects.cell_placement(4, dx=0.3, dy=0.2, dtheta_deg=0.15),
        # spec §4: 0.1 mm between adjacent terminals
        defects.terminal_height(6, "pos", dz=0.15),
        defects.terminal_tilt(7, "neg", deg=0.5),
        defects.missing_cell(9),
        defects.reversed_polarity(1),
        defects.busbar_pose(0, dx=0.2, dtheta_deg=0.1),
        # spec §5: a gap beyond a small fraction of the 1.5 mm sheet
        defects.busbar_liftoff(2, gap=0.15),
        defects.missing_weld(1, 5),
        defects.short_weld(1, 4, fraction=0.8),
        defects.spatter(count=10, radius=0.15),
        defects.handling_damage(
            ("cell", 0),
            depth=0.3,
            radius=6.0,
            centre=(cfg.cell_centre_x(0), cfg.cell_w / 2, 50.0),
            direction=(0.0, -1.0, 0.0),
        ),
        defects.missing_pad(3),
        defects.cover_misalignment(dx=0.5, dy=0.3, dtheta_deg=0.2),
    ]


DEMOS = {"exaggerated": demo_defects, "realistic": realistic_defects}


def main() -> None:
    parser = argparse.ArgumentParser(prog="defectgen")
    parser.add_argument("--out", type=Path, default=Path("data/preview"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--demo-defects",
        nargs="?",
        const="exaggerated",
        choices=sorted(DEMOS),
        help="inject the demo set; bare flag means exaggerated",
    )
    parser.add_argument(
        "--cells",
        nargs="+",
        type=int,
        metavar="K",
        help="write only these cells and their terminals, clean, with the Fusion "
        "cell_assembled alongside in reference/ (pre_busbar stage only)",
    )
    args = parser.parse_args()

    cfg = load()
    if args.cells:
        for path in export_cells(args.out, args.cells, cfg=cfg):
            print(path)
        return
    injectors = DEMOS[args.demo_defects](cfg) if args.demo_defects else []
    for path in export(args.out, cfg=cfg, seed=args.seed, defects=injectors):
        print(path)


if __name__ == "__main__":
    main()

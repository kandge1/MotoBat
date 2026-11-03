"""Write the three stages to disk so they can be opened in a mesh viewer.

    python -m defectgen --out data/preview
    python -m defectgen --out data/preview_bad --demo-defects

The demo set is deliberately exaggerated -- real severities are tenths of a
millimetre and invisible by eye. It exists to check that each injector moves
the thing it claims to, not to look like a real module.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from . import defects
from .config import load
from .module import export


def demo_defects(cfg):
    """One of each primary class, sized to be obvious in a viewer."""
    return [
        defects.cell_placement(4, dx=2.0, dy=1.5, dtheta_deg=2.0),
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


def main() -> None:
    parser = argparse.ArgumentParser(prog="defectgen")
    parser.add_argument("--out", type=Path, default=Path("data/preview"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--demo-defects", action="store_true")
    args = parser.parse_args()

    cfg = load()
    injectors = demo_defects(cfg) if args.demo_defects else []
    for path in export(args.out, cfg=cfg, seed=args.seed, defects=injectors):
        print(path)


if __name__ == "__main__":
    main()

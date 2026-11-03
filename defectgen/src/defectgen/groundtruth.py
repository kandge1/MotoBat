"""Ground-truth records emitted alongside every mesh.

These hold the *nominal* quantities that detection (§9.8) fits against: cell
poses, terminal top planes, busbar poses and seam paths. They are computed
forward from the placement maths and never inferred back from the mesh, which
is what keeps the labels independent of the thing being measured.

`defects` is empty for a clean module. Task 1.2 populates it.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Literal

Stage = Literal["pre_busbar", "pre_weld", "post_weld"]
STAGES: tuple[Stage, ...] = ("pre_busbar", "pre_weld", "post_weld")

Polarity = Literal["pos", "neg"]
Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class CellGT:
    index: int
    group: int
    flipped: bool
    centre: Vec3
    yaw_deg: float


@dataclass(frozen=True)
class TerminalGT:
    cell_index: int
    polarity: Polarity
    centre: Vec3
    top_z: float
    normal: Vec3


@dataclass(frozen=True)
class BusbarGT:
    index: int
    kind: Literal["end", "series"]
    centre: Vec3
    underside_z: float
    terminals: tuple[tuple[int, Polarity], ...]


@dataclass(frozen=True)
class SeamGT:
    busbar_index: int
    cell_index: int
    start: Vec3
    end: Vec3
    width: float


@dataclass(frozen=True)
class Defect:
    cls: str
    magnitude: float
    location: Vec3
    stage: Stage


@dataclass(frozen=True)
class Labels:
    stage: Stage
    config_name: str
    seed: int
    cells: tuple[CellGT, ...] = ()
    terminals: tuple[TerminalGT, ...] = ()
    busbars: tuple[BusbarGT, ...] = ()
    seams: tuple[SeamGT, ...] = ()
    defects: tuple[Defect, ...] = field(default=())

    def adjacent_terminal_deltas(self) -> list[tuple[int, int, float]]:
        """Top-height difference between terminals adjacent along the module.

        The pre-weld check in spec §4 is a +/-0.1 mm tolerance between adjacent
        terminals, so this is the quantity detection compares against, not the
        absolute height.
        """
        out = []
        for polarity in ("pos", "neg"):
            row = sorted(
                (t for t in self.terminals if t.polarity == polarity),
                key=lambda t: t.centre[0],
            )
            for a, b in zip(row, row[1:]):
                out.append((a.cell_index, b.cell_index, b.top_z - a.top_z))
        return out

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(asdict(self), indent=indent, sort_keys=True)

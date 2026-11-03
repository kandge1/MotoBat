"""The mutable description of a module, sitting between placement and mesh.

Defects are mostly transform perturbations (spec §7), so they need something to
perturb. `build_plan` produces this; `defects` mutates it; `module.realise`
turns it into triangles. Keeping the three apart is what stops defect logic
leaking into the assembly code, and it means a defect is recorded as a change
to a named placement rather than as surgery on an anonymous mesh.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh

from .config import Config
from .groundtruth import Defect, Stage

Ref = tuple


def translation(x: float, y: float, z: float) -> np.ndarray:
    t = np.eye(4)
    t[:3, 3] = (x, y, z)
    return t


def rotation_z(deg: float, about: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    return trimesh.transformations.rotation_matrix(np.radians(deg), (0, 0, 1), about)


@dataclass
class Placement:
    """One instance of a CAD part. `transform` maps part-local mm into module mm."""

    part: str
    transform: np.ndarray
    ref: Ref
    present: bool = True

    def compose(self, delta: np.ndarray) -> None:
        """Apply `delta` in module coordinates, i.e. after the existing placement."""
        self.transform = delta @ self.transform

    @property
    def origin(self) -> np.ndarray:
        return self.transform[:3, 3].copy()


@dataclass
class SeamPlan:
    """A weld stitch. Held as parameters rather than a mesh so that seam defects
    (short, offset, wrong width, sagging) are edits to numbers."""

    busbar_index: int
    cell_index: int
    centre: np.ndarray
    length: float
    width: float
    height: float
    present: bool = True


@dataclass
class Particle:
    """Spatter or foreign debris. A blob on a surface."""

    centre: np.ndarray
    radius: float
    kind: str


@dataclass
class Dent:
    """Handling damage, as plastic deformation rather than material removal.

    Vertices of the targeted part within `radius` of `centre` are pushed in
    along `direction`, falling off smoothly to nothing at the rim. This is what
    a dent physically is, and it avoids depending on a boolean backend.
    """

    ref: Ref
    centre: np.ndarray
    radius: float
    depth: float
    direction: np.ndarray


@dataclass
class Plan:
    cfg: Config
    stage: Stage
    placements: list[Placement] = field(default_factory=list)
    seams: list[SeamPlan] = field(default_factory=list)
    particles: list[Particle] = field(default_factory=list)
    dents: list[Dent] = field(default_factory=list)
    defects: list[Defect] = field(default_factory=list)

    def find(self, ref: Ref) -> Placement:
        for p in self.placements:
            if p.ref == ref:
                return p
        raise KeyError(f"no placement {ref!r} at stage {self.stage}")

    def of_kind(self, kind: str) -> list[Placement]:
        return [p for p in self.placements if p.ref[0] == kind]

    def record(self, cls: str, magnitude: float, location) -> None:
        self.defects.append(
            Defect(
                cls=cls,
                magnitude=float(magnitude),
                location=tuple(float(v) for v in location),
                stage=self.stage,
            )
        )

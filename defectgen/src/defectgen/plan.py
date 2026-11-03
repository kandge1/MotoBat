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
    # Held where it is by something the model does not contain (a clip, a
    # fixture), so settling leaves it alone. It still supports what rests on it.
    pinned: bool = False
    # Rigidly fixed to its cell but disturbed: lift it clear of the cap rather
    # than let it rest under gravity (a tilted terminal stands on its low edge).
    seat: bool = False

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
    """Spatter, foreign debris or a chip. A blob on a surface.

    Debris and FOD are present before later parts arrive, so settling treats
    them as supports -- a chip on a terminal holds the busbar off it. Spatter
    lands after welding and supports nothing.
    """

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
class Target:
    """What a defect touched. Placement refs and seam indices are resolved to
    boxes against both the nominal and the final plan; `region` is a box given
    outright, for things with no nominal pose (spatter, a dent)."""

    refs: tuple[Ref, ...] = ()
    seams: tuple[int, ...] = ()
    region: tuple[np.ndarray, np.ndarray] | None = None
    particle: int | None = None


@dataclass
class Plan:
    cfg: Config
    stage: Stage
    placements: list[Placement] = field(default_factory=list)
    seams: list[SeamPlan] = field(default_factory=list)
    particles: list[Particle] = field(default_factory=list)
    dents: list[Dent] = field(default_factory=list)
    defects: list[Defect] = field(default_factory=list)
    # Parallel to `defects`: what each one touched, for its bounding box.
    targets: list[Target] = field(default_factory=list)
    # Deformed parts, in module coordinates, keyed by ref. Settling writes these
    # for the compliant parts (pads compress, side plates bulge); `realise`
    # uses them in place of transforming the CAD part.
    overrides: dict = field(default_factory=dict)

    def find(self, ref: Ref) -> Placement:
        for p in self.placements:
            if p.ref == ref:
                return p
        raise KeyError(f"no placement {ref!r} at stage {self.stage}")

    def of_kind(self, kind: str) -> list[Placement]:
        return [p for p in self.placements if p.ref[0] == kind]

    def seam_index(self, seam: SeamPlan) -> int:
        return next(i for i, s in enumerate(self.seams) if s is seam)

    def record(
        self,
        cls: str,
        magnitude: float,
        location,
        *,
        refs: tuple[Ref, ...] = (),
        seams: tuple[SeamPlan, ...] = (),
        radius: float | None = None,
        particle: Particle | None = None,
    ) -> None:
        """`radius` boxes a sphere of that size around `location`. `particle`
        boxes that particle wherever settling leaves it."""
        region = None
        if radius is not None:
            centre = np.asarray(location, dtype=float)
            region = (centre - radius, centre + radius)
        index = None
        if particle is not None:
            index = next(i for i, q in enumerate(self.particles) if q is particle)
        self.targets.append(
            Target(tuple(refs), tuple(self.seam_index(s) for s in seams), region, index)
        )
        self.defects.append(
            Defect(
                cls=cls,
                magnitude=float(magnitude),
                location=tuple(float(v) for v in location),
                stage=self.stage,
            )
        )

"""Defect injection -- the taxonomy in project_plan.md §4B.2.

Each function here is a factory: call it with a severity and it returns
something `generate(defects=[...])` applies to the plan. Severity is always an
explicit argument rather than a random draw, so a dataset sweeps a parameter
instead of sampling one (§4B.2).

Most of these are transform perturbations, which is why `plan.py` exists. The
exceptions are the weld-seam classes, spatter and FOD -- generated geometry --
and handling damage, which deforms an existing part.

Numbered to match the §4B.2 table. Classes 10 (gap-filler bead) and 12
(insulation film) are absent on purpose: the bead has no pre-cell stage yet,
and the film was deliberately never modelled as geometry (module_spec §3).
Classes 16 and 17 are internal to the weld and out of scope by definition.
"""

from __future__ import annotations

import functools

import numpy as np
import trimesh

from .groundtruth import Stage
from .plan import Dent, Particle, Plan, rotation_z, translation


def _rotation(deg: float, axis, about) -> np.ndarray:
    return trimesh.transformations.rotation_matrix(np.radians(deg), axis, about)


def _from(stage: Stage):
    """Tag an injector with the earliest stage it means anything at.

    A busbar defect is meaningless before the busbars are placed, so `generate`
    skips an injector whose stage has not been reached. The stages come
    straight from the §4B.2 table. Tagging the closure rather than the factory
    is the point -- `generate` only ever sees the closure.
    """

    def tag(factory):
        @functools.wraps(factory)
        def wrapper(*args, **kwargs):
            apply = factory(*args, **kwargs)
            apply.min_stage = stage
            return apply

        return wrapper

    return tag


def _cell_refs(k: int):
    return (("cell", k), ("terminal", k, "pos"), ("terminal", k, "neg"))


# 1 -- cell placement error. The loop target: §4B.7 corrects the next cell from this.

@_from("pre_busbar")
def cell_placement(k: int, dx: float = 0.0, dy: float = 0.0, dtheta_deg: float = 0.0):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        cx = plan.cfg.cell_centre_x(k)
        delta = translation(dx, dy, 0.0) @ rotation_z(dtheta_deg, about=(cx, 0.0, 0.0))
        for ref in _cell_refs(k):
            plan.find(ref).compose(delta)
        plan.record(
            "cell_placement",
            float(np.hypot(dx, dy)),
            (cx, 0.0, plan.cfg.z_cell_bottom),
            refs=_cell_refs(k),
        )

    return apply


# 2 -- terminal height and tilt. The 0.1 mm adjacent-terminal tolerance (spec §4).

@_from("pre_busbar")
def terminal_height(k: int, polarity: str, dz: float):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        placement = plan.find(("terminal", k, polarity))
        placement.compose(translation(0.0, 0.0, dz))
        plan.record("terminal_height", dz, placement.origin, refs=(placement.ref,))

    return apply


@_from("pre_busbar")
def terminal_tilt(k: int, polarity: str, deg: float, axis: str = "y"):
    """Rotates about the seating-face centre, which is where the part's origin
    sits -- so a tilt is a pure rotation and does not also drag the contact
    point sideways (spec §4)."""

    def apply(plan: Plan, rng: np.random.Generator) -> None:
        placement = plan.find(("terminal", k, polarity))
        pivot = placement.origin
        vector = (0.0, 1.0, 0.0) if axis == "y" else (1.0, 0.0, 0.0)
        placement.compose(_rotation(deg, vector, pivot))
        plan.record("terminal_tilt", deg, pivot, refs=(placement.ref,))

    return apply


# 3 -- missing cell.

@_from("pre_busbar")
def missing_cell(k: int):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        location = plan.find(("cell", k)).origin
        for ref in _cell_refs(k):
            plan.find(ref).present = False
        plan.record("missing_cell", 1.0, location, refs=_cell_refs(k))

    return apply


# 4 -- reversed polarity. Physically a cell placed 180 degrees out, so that is
# how it is modelled rather than by swapping two terminals.

@_from("pre_busbar")
def reversed_polarity(k: int):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        cx = plan.cfg.cell_centre_x(k)
        delta = rotation_z(180.0, about=(cx, 0.0, 0.0))
        for ref in _cell_refs(k):
            plan.find(ref).compose(delta)
        plan.record(
            "reversed_polarity", 180.0, (cx, 0.0, plan.cfg.z_cell_top), refs=_cell_refs(k)
        )

    return apply


# 5, 6 -- busbar pose and lift-off. Seams ride with the busbar, since welding
# happens after placement.

def _move_busbar(plan: Plan, index: int, delta: np.ndarray) -> None:
    plan.find(("busbar", index)).compose(delta)
    for seam in plan.seams:
        if seam.busbar_index == index:
            seam.centre = (delta @ np.append(seam.centre, 1.0))[:3]


@_from("pre_weld")
def busbar_pose(index: int, dx: float = 0.0, dy: float = 0.0, dtheta_deg: float = 0.0):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        pivot = plan.find(("busbar", index)).origin
        delta = translation(dx, dy, 0.0) @ rotation_z(dtheta_deg, about=tuple(pivot))
        _move_busbar(plan, index, delta)
        plan.record("busbar_pose", float(np.hypot(dx, dy)), pivot, refs=(("busbar", index),))

    return apply


@_from("pre_weld")
def busbar_liftoff(index: int, gap: float):
    """The pre-weld gap is the critical process variable: too large and the lap
    weld undercuts, goes porous or fails to fuse (spec §5)."""

    def apply(plan: Plan, rng: np.random.Generator) -> None:
        pivot = plan.find(("busbar", index)).origin
        _move_busbar(plan, index, translation(0.0, 0.0, gap))
        plan.record("busbar_liftoff", gap, pivot, refs=(("busbar", index),))

    return apply


# 7 -- weld seam geometry. Surface classes only; ISO 13919-2 sets the limits.

def _seams(plan: Plan, busbar_index: int, cell_index: int | None):
    return [
        s
        for s in plan.seams
        if s.busbar_index == busbar_index
        and (cell_index is None or s.cell_index == cell_index)
    ]


@_from("post_weld")
def missing_weld(busbar_index: int, cell_index: int):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        for seam in _seams(plan, busbar_index, cell_index):
            seam.present = False
            plan.record("missing_weld", 1.0, seam.centre, seams=(seam,))

    return apply


@_from("post_weld")
def short_weld(busbar_index: int, cell_index: int, fraction: float):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        for seam in _seams(plan, busbar_index, cell_index):
            seam.length *= fraction
            plan.record("short_weld", 1.0 - fraction, seam.centre, seams=(seam,))

    return apply


@_from("post_weld")
def seam_deviation(busbar_index: int, cell_index: int, dy: float):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        for seam in _seams(plan, busbar_index, cell_index):
            seam.centre = seam.centre + np.array([0.0, dy, 0.0])
            plan.record("seam_deviation", dy, seam.centre, seams=(seam,))

    return apply


@_from("post_weld")
def width_deviation(busbar_index: int, cell_index: int, factor: float):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        for seam in _seams(plan, busbar_index, cell_index):
            seam.width *= factor
            plan.record("width_deviation", seam.width, seam.centre, seams=(seam,))

    return apply


@_from("post_weld")
def reinforcement(busbar_index: int, cell_index: int, dh: float):
    """Positive is excess reinforcement, negative is sagging or undercut."""

    def apply(plan: Plan, rng: np.random.Generator) -> None:
        for seam in _seams(plan, busbar_index, cell_index):
            seam.height += dh
            plan.record("reinforcement", dh, seam.centre, seams=(seam,))

    return apply


# 8 -- landed spatter, scattered around the seams it came from.

@_from("post_weld")
def spatter(count: int, radius: float = 0.2, spread: float = 6.0):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        live = [s for s in plan.seams if s.present]
        if not live:
            return
        for seam in rng.choice(len(live), size=count):
            origin = live[int(seam)].centre
            offset = np.append(rng.normal(0.0, spread, 2), 0.0)
            centre = origin + offset
            plan.particles.append(Particle(centre=centre, radius=radius, kind="spatter"))
            plan.record("spatter", radius, centre, radius=radius)

    return apply


# 9 -- handling damage. The NHTSA 26V188 mechanism (§4B.9): a mis-placed part
# touches tooling and is deformed by the overtravel.

@_from("pre_busbar")
def handling_damage(
    ref: tuple, depth: float, radius: float = 4.0, centre=None, direction=(0.0, -1.0, 0.0)
):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        placement = plan.find(ref)
        at = np.array(centre if centre is not None else placement.origin, dtype=float)
        plan.dents.append(
            Dent(
                ref=ref,
                centre=at,
                radius=radius,
                depth=depth,
                direction=np.array(direction, dtype=float),
            )
        )
        # Boxed to the dent, not the struck part -- the whole cell is not damaged.
        plan.record("handling_damage", depth, at, radius=radius)

    return apply


# 11 -- compression pad missing or misplaced.

@_from("pre_busbar")
def missing_pad(k: int):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        placement = plan.find(("pad", k))
        placement.present = False
        plan.record("missing_pad", 1.0, placement.origin, refs=(placement.ref,))

    return apply


# 13 -- foreign object debris.

@_from("pre_busbar")
def fod(centre, radius: float = 0.5):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        at = np.array(centre, dtype=float)
        plan.particles.append(Particle(centre=at, radius=radius, kind="fod"))
        plan.record("fod", radius, at, radius=radius)

    return apply


# 14 -- carrier not fully seated.

@_from("pre_weld")
def carrier_seating(dz: float, tilt_deg: float = 0.0, axis: str = "y"):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        placement = plan.find(("carrier",))
        pivot = placement.origin
        delta = translation(0.0, 0.0, dz)
        if tilt_deg:
            vector = (0.0, 1.0, 0.0) if axis == "y" else (1.0, 0.0, 0.0)
            delta = delta @ _rotation(tilt_deg, vector, pivot)
        placement.compose(delta)
        plan.record("carrier_seating", dz, pivot, refs=(placement.ref,))

    return apply


# 15 -- cover misalignment.

@_from("post_weld")
def cover_misalignment(dx: float = 0.0, dy: float = 0.0, dtheta_deg: float = 0.0):
    def apply(plan: Plan, rng: np.random.Generator) -> None:
        placement = plan.find(("cover",))
        pivot = placement.origin
        placement.compose(
            translation(dx, dy, 0.0) @ rotation_z(dtheta_deg, about=tuple(pivot))
        )
        plan.record(
            "cover_misalignment", float(np.hypot(dx, dy)), pivot, refs=(placement.ref,)
        )

    return apply

"""Load the exported CAD meshes and normalise them into the spec §7 frame.

The Fusion exports are not in that frame. Seven of the twelve bodies are rotated
90 degrees about Z relative to "X along the module length", and every origin sits
at a body centroid rather than at the part's named datum. Rather than re-export,
each part carries the rotation needed and is translated so that the centre of its
bottom face lands on the origin.

Bottom-face-centre is the right datum for every part here: it is what spec §7
names for the cell, it is the "centre of underside" §7 names for a busbar, and
for a terminal it is the seating face -- so a height defect is a pure +Z
translation and a tilt rotates about the real pivot rather than dragging the
contact point sideways (spec §4).

`extent` names the config fields that give each part's size after normalisation.
Every load is checked against them, so a re-export that quietly changes
orientation or origin fails here instead of producing a subtly wrong module.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh

from .config import Config

MESH_DIR = Path(__file__).parents[3] / "src" / "motobat_description" / "meshes" / "module"

EXTENT_TOL = 0.05


@dataclass(frozen=True)
class PartSpec:
    rot_z_deg: int
    extent: tuple[str, str, str]
    # Datum Y at the centre of the top face rather than of the bounding box.
    # terminal_neg has the fill plug merged in beside the pad, so its bounding
    # box centre is 8.25 off the pad centre -- the point L7's 115.0 is measured
    # between. Placed by the box, the plug hangs past the side plate.
    pad_datum: bool = False


PARTS: dict[str, PartSpec] = {
    "cell_can": PartSpec(90, ("cell_t", "cell_w", "cell_h")),
    "terminal_pos": PartSpec(90, ("terminal_d", "terminal_w", "terminal_h"), pad_datum=True),
    "terminal_neg": PartSpec(90, ("terminal_d", "terminal_neg_w", "terminal_h"), pad_datum=True),
    "interface_pad": PartSpec(0, ("pad_t", "pad_w", "pad_h")),
    "end_plate": PartSpec(0, ("end_plate_t", "end_plate_w", "end_plate_h")),
    "side_plate": PartSpec(0, ("side_plate_l", "side_plate_t", "side_plate_h")),
    "cooling_plate": PartSpec(90, ("cooling_plate_l", "cooling_plate_w", "cooling_plate_t")),
    "cover": PartSpec(0, ("cover_l", "cover_w", "cover_t")),
    "busbar_carrier": PartSpec(90, ("carrier_l", "carrier_w", "carrier_h")),
    "busbar_end": PartSpec(90, ("busbar_end_l", "busbar_w", "busbar_t")),
    "busbar_series": PartSpec(90, ("busbar_series_l", "busbar_w", "busbar_t")),
    "cmc": PartSpec(90, ("cmc_l", "cmc_w", "cmc_t")),
}

# cell_assembled and module_assembled are deliberately absent. They are merged
# meshes kept for renders; building from them would make the terminal-height and
# terminal-tilt defect classes ungeneratable (spec §7).


def mesh_dir() -> Path:
    return Path(os.environ.get("MOTOBAT_MESHES", MESH_DIR))


def expected_extent(name: str, cfg: Config) -> np.ndarray:
    return np.array([getattr(cfg, f) for f in PARTS[name].extent], dtype=float)


def load_part(name: str, cfg: Config, directory: Path | None = None) -> trimesh.Trimesh:
    """Return the named part, normalised to the spec §7 frame."""
    spec = PARTS[name]
    path = (directory or mesh_dir()) / f"{name}.stl"
    if not path.is_file():
        raise FileNotFoundError(f"mesh not found: {path}")

    # process=True merges the duplicate vertices an STL stores per triangle.
    # Without it every edge reads as a boundary and nothing is watertight.
    mesh = trimesh.load_mesh(path)

    if spec.rot_z_deg:
        mesh.apply_transform(
            trimesh.transformations.rotation_matrix(
                np.radians(spec.rot_z_deg), [0.0, 0.0, 1.0]
            )
        )

    low, high = mesh.bounds
    mesh.apply_translation([-(low[0] + high[0]) / 2, -(low[1] + high[1]) / 2, -low[2]])

    actual = mesh.extents
    expected = expected_extent(name, cfg)
    if not np.allclose(actual, expected, atol=EXTENT_TOL):
        raise ValueError(
            f"{name}: normalised extent {np.round(actual, 3).tolist()} does not match "
            f"config {expected.tolist()} (fields {spec.extent}). The mesh was "
            f"re-exported with a different orientation or size, or the config is stale."
        )

    if spec.pad_datum:
        top = mesh.vertices[np.isclose(mesh.vertices[:, 2], mesh.bounds[1][2], atol=1e-3)]
        y_lo, y_hi = top[:, 1].min(), top[:, 1].max()
        if not np.isclose(y_hi - y_lo, cfg.terminal_w, atol=EXTENT_TOL):
            raise ValueError(
                f"{name}: top face spans {y_hi - y_lo:.3f} in Y, expected the "
                f"{cfg.terminal_w} pad (terminal_w). Cannot locate the pad centre."
            )
        mesh.apply_translation([0.0, -(y_lo + y_hi) / 2, 0.0])
    return mesh


def load_all(cfg: Config, directory: Path | None = None) -> dict[str, trimesh.Trimesh]:
    return {name: load_part(name, cfg, directory) for name in PARTS}


def is_closed(mesh: trimesh.Trimesh) -> bool:
    """True if no edge is used by exactly one face -- the mesh has no holes.

    Weaker than `trimesh.is_watertight`, which additionally forbids an edge
    shared by more than two faces. That stricter condition is wrong for this
    project: CAD parts whose sub-bodies sit exactly face-to-face become
    non-manifold the moment duplicate vertices are merged, yet they are still
    sealed. What matters downstream is that a ray cannot leak through a hole
    (§4B.3), and that is what this checks.
    """
    return len(trimesh.grouping.group_rows(mesh.edges_sorted, require_count=1)) == 0

"""The normalisation table is only safe if it is checked. These tests are that check."""

from __future__ import annotations

import numpy as np
import pytest

from defectgen.config import load
from defectgen.parts import PARTS, expected_extent, is_closed, load_part, mesh_dir

cfg = load()

pytestmark = pytest.mark.skipif(
    not mesh_dir().is_dir(), reason=f"CAD meshes not found at {mesh_dir()}"
)


@pytest.mark.parametrize("name", sorted(PARTS))
def test_extent_matches_config(name):
    mesh = load_part(name, cfg)
    np.testing.assert_allclose(mesh.extents, expected_extent(name, cfg), atol=0.05)


@pytest.mark.parametrize("name", sorted(PARTS))
def test_datum_is_bottom_face_centre(name):
    low, high = load_part(name, cfg).bounds
    assert low[2] == pytest.approx(0.0, abs=1e-6)
    assert (low[0] + high[0]) / 2 == pytest.approx(0.0, abs=1e-6)
    assert (low[1] + high[1]) / 2 == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("name", sorted(PARTS))
def test_part_has_no_holes(name):
    assert is_closed(load_part(name, cfg))


@pytest.mark.xfail(
    strict=True,
    reason="terminal_neg exports 38.5 wide against terminal_pos's 22.0 -- the "
    "fill plug is merged into it. Placed on the terminal centre it reaches "
    "y=76.75, past the side plate at 74.0 and past the 75.5 envelope. Fix by "
    "re-exporting the terminal alone.",
)
def test_terminals_have_matching_footprints():
    """Spec §4 sources both pads from the vendor drawing at 22.0 x 18.5."""
    pos = load_part("terminal_pos", cfg).extents
    neg = load_part("terminal_neg", cfg).extents
    assert neg[1] == pytest.approx(pos[1], abs=0.05), (
        "terminal_neg is wider than terminal_pos, which means the fill plug is "
        "merged into it. Re-export the terminal alone; the plug belongs as a "
        "feature on the can (spec §4)."
    )


def test_merged_cell_is_not_a_part():
    """Building from cell_assembled would make terminal height and tilt defects
    ungeneratable, because there would be no terminal to transform (spec §7)."""
    assert "cell_assembled" not in PARTS
    assert "module_assembled" not in PARTS


def test_every_spec_body_is_present():
    assert len(PARTS) == 12

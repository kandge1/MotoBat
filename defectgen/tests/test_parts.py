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
    """Terminals are the exception in Y: their datum is the pad centre, which
    for terminal_neg is not the bounding-box centre (see PartSpec.pad_datum)."""
    mesh = load_part(name, cfg)
    low, high = mesh.bounds
    assert low[2] == pytest.approx(0.0, abs=1e-6)
    assert (low[0] + high[0]) / 2 == pytest.approx(0.0, abs=1e-6)
    if PARTS[name].pad_datum:
        top = mesh.vertices[np.isclose(mesh.vertices[:, 2], high[2], atol=1e-3)]
        low, high = top.min(axis=0), top.max(axis=0)
        assert high[1] - low[1] == pytest.approx(cfg.terminal_w, abs=0.05)
    assert (low[1] + high[1]) / 2 == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("name", sorted(PARTS))
def test_part_has_no_holes(name):
    assert is_closed(load_part(name, cfg))


def test_terminals_are_asymmetric_so_the_carrier_can_key_them():
    """The negative terminal carries the fill plug on its inboard side, so it
    is 38.5 wide against the positive's 22.0 pad. That asymmetry is a design
    feature, not an export error: the carrier is pocketed to fit it, so a cell
    put in backwards cannot take the carrier (fixlist E1, closed 4 Nov 2026).
    Both pads are still the vendor drawing's 22.0 x 18.5 -- the datum test
    checks the top face."""
    pos = load_part("terminal_pos", cfg).extents
    neg = load_part("terminal_neg", cfg).extents
    assert pos[1] == pytest.approx(cfg.terminal_w, abs=0.05)
    assert neg[1] == pytest.approx(cfg.terminal_neg_w, abs=0.05)
    assert neg[1] > pos[1] + 10.0


def test_merged_cell_is_not_a_part():
    """Building from cell_assembled would make terminal height and tilt defects
    ungeneratable, because there would be no terminal to transform (spec §7)."""
    assert "cell_assembled" not in PARTS
    assert "module_assembled" not in PARTS


def test_every_spec_body_is_present():
    assert len(PARTS) == 12

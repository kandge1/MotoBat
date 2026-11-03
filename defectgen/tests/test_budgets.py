"""module_spec.md's arithmetic, as assertions.

The spec's three budgets close exactly at nominal. Writing them down here means a
future parameter edit that breaks one fails in CI rather than in a fit check.
"""

from __future__ import annotations

import pytest

from defectgen.config import load

cfg = load()


def test_x_budget_closes():
    total = 2 * cfg.end_plate_t + cfg.n_cells * cfg.cell_t + (cfg.n_cells - 1) * cfg.pad_t
    assert total == pytest.approx(cfg.module_l)


def test_y_budget_closes():
    assert 2 * cfg.side_plate_t + cfg.cell_w == pytest.approx(cfg.module_w)


def test_z_budget_fits_the_envelope():
    assert cfg.z_cover_top == pytest.approx(107.0)
    assert cfg.z_cover_top <= cfg.module_h


def test_cell_pitch_is_cell_plus_pad():
    assert cfg.cell_pitch == pytest.approx(cfg.cell_t + cfg.pad_t)


def test_walls_clear_the_busbars():
    """Spec §5 raised the walls so a flat cover closes over everything inside."""
    assert cfg.z_wall_top > cfg.z_busbar_top


def test_terminals_fit_the_cap():
    assert cfg.terminal_pitch_y + cfg.terminal_w <= cfg.cell_w
    assert cfg.terminal_d <= cfg.cell_t


def test_busbar_rows_sit_inside_the_side_plates():
    assert cfg.terminal_y + cfg.busbar_w / 2 <= cfg.cell_w / 2


def test_carrier_fits_between_the_end_plates():
    stack = cfg.n_cells * cfg.cell_t + (cfg.n_cells - 1) * cfg.pad_t
    assert cfg.carrier_l <= stack
    assert cfg.carrier_w <= cfg.cell_w


@pytest.mark.parametrize(
    "length,spanned", [("busbar_end_l", 3), ("busbar_series_l", 6)]
)
def test_busbar_overhang_is_consistent(length, spanned):
    """Both busbars overhang their end terminals by the same amount, so one rule
    covers the seam placement constraint in spec §5."""
    overhang = getattr(cfg, length) - (spanned - 1) * cfg.cell_pitch
    assert overhang == pytest.approx(8.5)


def test_seams_fit_the_end_terminal_overlap():
    """At an end terminal the busbar covers only part of the pad, and the stitch
    has to sit inside that overlap rather than weld air."""
    overhang = (cfg.busbar_end_l - (cfg.n_parallel - 1) * cfg.cell_pitch) / 2
    overlap = cfg.terminal_d / 2 + overhang
    assert cfg.weld_stitch_l <= overlap


def test_cmc_clears_the_cover():
    """Fixlist E2 was arithmetic on the wrong datum: the CMC sits on the carrier
    deck (module_assembled, 98.0 -> 101.2), not on top of the carrier at 102.0."""
    assert cfg.z_carrier_deck + cfg.cmc_t <= cfg.z_wall_top


def test_cmc_sits_on_the_deck_as_in_the_fusion_assembly():
    assert cfg.z_carrier_deck == pytest.approx(98.0)
    assert cfg.z_carrier_deck + cfg.cmc_t == pytest.approx(101.2)

from __future__ import annotations

import numpy as np
import pytest

from defectgen.config import load
from defectgen.groundtruth import STAGES
from defectgen.module import busbar_plan, generate, is_flipped, terminal_y
from defectgen.parts import is_closed, mesh_dir

cfg = load()

needs_meshes = pytest.mark.skipif(
    not mesh_dir().is_dir(), reason=f"CAD meshes not found at {mesh_dir()}"
)


def test_busbar_plan_is_3p4s():
    plan = busbar_plan(cfg)
    kinds = [kind for kind, _ in plan]
    assert kinds.count("series") == 3
    assert kinds.count("end") == 2


def test_every_terminal_is_joined_exactly_once():
    seen = [t for _, terms in busbar_plan(cfg) for t in terms]
    assert len(seen) == len(set(seen)) == cfg.n_cells * 2


def test_each_busbar_stays_on_one_terminal_row():
    for _kind, terms in busbar_plan(cfg):
        assert len({terminal_y(cfg, c, p) for c, p in terms}) == 1


def test_alternate_groups_are_flipped():
    assert [is_flipped(cfg, k) for k in (0, 3, 6, 9)] == [False, True, False, True]


def test_cells_span_the_stack():
    first, last = cfg.cell_centre_x(0), cfg.cell_centre_x(cfg.n_cells - 1)
    assert first - cfg.cell_t / 2 == pytest.approx(-cfg.module_l / 2 + cfg.end_plate_t)
    assert last + cfg.cell_t / 2 == pytest.approx(cfg.module_l / 2 - cfg.end_plate_t)


def test_unknown_stage_is_rejected():
    with pytest.raises(ValueError, match="unknown stage"):
        generate("post_bake")  # type: ignore[arg-type]


@needs_meshes
@pytest.mark.parametrize("stage", STAGES)
def test_stage_fits_the_envelope_in_x_and_z(stage):
    mesh, _ = generate(stage)
    low, high = mesh.bounds
    assert low[2] == pytest.approx(0.0, abs=1e-6)
    assert high[2] <= cfg.module_h
    assert high[0] - low[0] == pytest.approx(cfg.module_l, abs=0.05)


@needs_meshes
@pytest.mark.xfail(
    strict=True,
    reason="blocked by the merged fill plug in terminal_neg -- see "
    "test_parts.py::test_terminals_have_matching_footprints. The module is "
    "153.5 wide instead of 151.0 until that part is re-exported.",
)
def test_module_fits_the_envelope_in_y():
    mesh, _ = generate("pre_busbar")
    low, high = mesh.bounds
    assert high[1] - low[1] == pytest.approx(cfg.module_w, abs=0.05)


@needs_meshes
@pytest.mark.parametrize("stage", STAGES)
def test_stage_mesh_has_no_holes(stage):
    mesh, _ = generate(stage)
    assert is_closed(mesh)


@needs_meshes
def test_labels_count_per_stage():
    _, pre_busbar = generate("pre_busbar")
    assert len(pre_busbar.cells) == cfg.n_cells
    assert len(pre_busbar.terminals) == cfg.n_cells * 2
    assert pre_busbar.busbars == ()
    assert pre_busbar.seams == ()

    _, pre_weld = generate("pre_weld")
    assert len(pre_weld.busbars) == 5
    assert pre_weld.seams == ()

    _, post_weld = generate("post_weld")
    joints = sum(len(b.terminals) for b in post_weld.busbars)
    assert joints == cfg.n_cells * 2
    assert len(post_weld.seams) == joints * cfg.weld_stitches


@needs_meshes
def test_clean_module_has_no_defects_and_flat_terminals():
    _, labels = generate("pre_busbar")
    assert labels.defects == ()
    deltas = [d for *_ , d in labels.adjacent_terminal_deltas()]
    assert deltas and max(abs(d) for d in deltas) == pytest.approx(0.0, abs=1e-9)


@needs_meshes
def test_generation_is_deterministic():
    a, _ = generate("post_weld", seed=7)
    b, _ = generate("post_weld", seed=7)
    np.testing.assert_array_equal(a.vertices, b.vertices)


@needs_meshes
def test_labels_serialise():
    _, labels = generate("post_weld")
    assert '"stage": "post_weld"' in labels.to_json()

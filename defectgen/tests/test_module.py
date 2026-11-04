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
    """Even groups flipped, as in module_assembled (module + on group 0)."""
    assert [is_flipped(cfg, k) for k in (0, 3, 6, 9)] == [True, False, True, False]


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


@needs_meshes
@pytest.mark.parametrize("stage", STAGES)
def test_body_groups_are_exactly_the_scan_mesh(stage):
    """The per-group STLs are for hiding bodies in a viewer. If they ever
    differed from the merged mesh, the viewer would show a module the scanner
    never saw."""
    from defectgen import defects
    from defectgen.module import injected_plan, realise, realise_groups
    from defectgen.parts import load_all

    parts = load_all(cfg)
    injectors = [defects.spatter(count=5, radius=0.5), defects.missing_cell(9)]
    plan = injected_plan(stage, cfg, seed=3, defects=injectors)
    whole = realise(plan, parts)
    groups = realise_groups(plan, parts)

    assert sum(len(m.faces) for m in groups.values()) == len(whole.faces)
    assert np.isclose(sum(m.volume for m in groups.values()), whole.volume)
    assert ("busbars" in groups) == (stage != "pre_busbar")
    assert ("cover" in groups) == (stage == "post_weld")
    assert ("fod" in groups) == (stage == "post_weld")


def _reference_cell():
    """The user's Fusion cell_assembled, put through cell_can's normalisation so
    it sits in the same frame as the generator's can."""
    import trimesh

    from defectgen.parts import PARTS

    rot = trimesh.transformations.rotation_matrix(
        np.radians(PARTS["cell_can"].rot_z_deg), [0, 0, 1]
    )
    can = trimesh.load_mesh(mesh_dir() / "cell_can.stl")
    can.apply_transform(rot)
    lo, hi = can.bounds
    shift = [-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]]
    ref = trimesh.load_mesh(mesh_dir() / "cell_assembled.stl")
    ref.apply_transform(rot)
    ref.apply_translation(shift)
    return ref


@needs_meshes
@pytest.mark.parametrize("k", [0, 3], ids=["unflipped", "flipped"])
def test_terminals_sit_where_the_fusion_cell_has_them(k):
    """Place cell_assembled with the generator's own cell transform, and every
    terminal must land on the matching terminal of the user's CAD. This is the
    check that would have caught pos and neg on swapped sides, and the negative
    terminal's fill plug hanging past the side plate."""
    from defectgen.module import build_plan, cell_transform
    from defectgen.parts import load_all

    parts = load_all(cfg)
    ref = _reference_cell()
    ref.apply_transform(cell_transform(cfg, k))
    above_cap = sorted(
        (c for c in ref.split(only_watertight=False) if c.bounds[0][2] >= cfg.z_cell_top - 1e-6),
        key=lambda c: abs(c.volume),
    )
    assert len(above_cap) == 2, "expected exactly two terminals on the Fusion cell"
    expected = {"pos": above_cap[0].bounds, "neg": above_cap[1].bounds}

    plan = build_plan(cfg, "pre_busbar")
    for polarity in ("pos", "neg"):
        placement = plan.find(("terminal", k, polarity))
        built = parts[placement.part].copy().apply_transform(placement.transform)
        np.testing.assert_allclose(built.bounds, expected[polarity], atol=0.05)


@needs_meshes
def test_no_terminal_reaches_past_the_cell():
    from defectgen.module import build_plan
    from defectgen.parts import load_all

    parts = load_all(cfg)
    for placement in build_plan(cfg, "pre_busbar").of_kind("terminal"):
        built = parts[placement.part].copy().apply_transform(placement.transform)
        assert np.abs(built.bounds[:, 1]).max() <= cfg.cell_w / 2 + 1e-6, placement.ref


@needs_meshes
@pytest.mark.parametrize("stage", STAGES)
def test_a_clean_module_is_already_at_rest(stage):
    """Settling with every step forced must move nothing in a clean build. If
    it did, either the solver is wrong or the CAD does not sit where the plan
    puts it -- both worth knowing before any defect result."""
    from defectgen.parts import load_all
    from defectgen.module import build_plan
    from defectgen.settle import settle

    plan, nominal = build_plan(cfg, stage), build_plan(cfg, stage)
    settle(plan, load_all(cfg), force=True)
    for p, q in zip(plan.placements, nominal.placements):
        np.testing.assert_allclose(p.transform, q.transform, atol=1e-6, err_msg=str(p.ref))
    assert not plan.overrides


@needs_meshes
def test_module_matches_the_fusion_assembly():
    """Every placed part must coincide with a body of the user's Fusion
    module_assembled, to 0.05 mm on its bounding box. The assembly's frame is
    the module's turned 90 deg about Z (its Y runs along the stack); that is
    the only mapping under which all five busbars land.

    The CMC is the one exception: the assembly splits it into a board and
    separate components, which together span what the single CMC part does.
    """
    import trimesh

    from defectgen.module import build_plan
    from defectgen.parts import load_all

    parts = load_all(cfg)
    ref = trimesh.load_mesh(mesh_dir() / "module_assembled.stl")
    turn = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1])
    boxes = []
    for body in ref.split(only_watertight=False):
        corners = trimesh.transform_points(trimesh.bounds.corners(body.bounds), turn)
        boxes.append([corners.min(axis=0), corners.max(axis=0)])
    boxes = np.array(boxes)

    missing = []
    for placement in build_plan(cfg, "post_weld").placements:
        if placement.ref == ("cmc",):
            continue
        built = parts[placement.part].copy().apply_transform(placement.transform)
        if np.abs(boxes - built.bounds[None]).max(axis=(1, 2)).min() >= 0.05:
            missing.append(placement.ref)
    assert missing == []


@needs_meshes
def test_cmc_holes_sit_on_the_carrier_pegs():
    """Every 6 mm carrier peg must pass through a CMC hole: the centre of the
    open area of the board around each peg lands on the peg centre. This is
    what catches the CMC loaded 180 deg out, which its extents cannot."""
    from defectgen.module import build_plan
    from defectgen.parts import load_all
    from defectgen.raycast import AxisCaster
    from defectgen.settle import world_mesh

    parts = load_all(cfg)
    plan = build_plan(cfg, "post_weld")
    carrier = AxisCaster(world_mesh(plan, parts, plan.find(("carrier",))), 2)
    board = AxisCaster(world_mesh(plan, parts, plan.find(("cmc",))), 2)

    xs, ys = np.meshgrid(np.arange(-130, 130, 0.25), np.arange(-20, 20, 0.25))
    grid = np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)])
    peg_top = cfg.z_cell_top + cfg.carrier_deck_h + 2.6
    on_peg = np.isclose(carrier.extreme(grid, True), peg_top, atol=1e-3)
    assert on_peg.any(), "no peg tops found"

    from scipy import ndimage

    labels, count = ndimage.label(on_peg.reshape(xs.shape))
    assert count == 12
    for k in range(1, count + 1):
        c = grid[(labels.ravel() == k), :2].mean(axis=0)
        g = np.arange(-4, 4.001, 0.05)
        gx, gy = np.meshgrid(c[0] + g, c[1] + g)
        q = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)])
        open_ = np.isnan(board.extreme(q, False))
        np.testing.assert_allclose(q[open_, :2].mean(axis=0), c, atol=0.05)

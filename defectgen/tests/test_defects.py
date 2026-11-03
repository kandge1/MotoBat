from __future__ import annotations

import numpy as np
import pytest

from defectgen import defects, generate
from defectgen.config import load
from defectgen.parts import mesh_dir

cfg = load()

needs_meshes = pytest.mark.skipif(
    not mesh_dir().is_dir(), reason=f"CAD meshes not found at {mesh_dir()}"
)
pytestmark = needs_meshes


def classes(labels):
    return [d.cls for d in labels.defects]


def test_clean_module_records_nothing():
    _, labels = generate("post_weld")
    assert labels.defects == ()


def test_cell_placement_moves_the_terminals_with_the_cell():
    """A shifted cell carries its terminals. If it did not, the defect would be
    a cell moving out from under its own pads."""
    _, clean = generate("pre_busbar")
    _, bad = generate("pre_busbar", defects=[defects.cell_placement(4, dx=0.8, dy=0.5)])

    before = {(t.cell_index, t.polarity): np.array(t.centre) for t in clean.terminals}
    after = {(t.cell_index, t.polarity): np.array(t.centre) for t in bad.terminals}
    for polarity in ("pos", "neg"):
        shift = after[(4, polarity)] - before[(4, polarity)]
        np.testing.assert_allclose(shift, [0.8, 0.5, 0.0], atol=1e-9)
        np.testing.assert_allclose(after[(5, polarity)], before[(5, polarity)], atol=1e-9)

    assert classes(bad) == ["cell_placement"]
    assert bad.defects[0].magnitude == pytest.approx(np.hypot(0.8, 0.5))


def test_terminal_height_shows_up_as_an_adjacent_delta():
    """The pre-weld check measures neighbour-to-neighbour, not absolute height."""
    _, labels = generate(
        "pre_busbar", defects=[defects.terminal_height(6, "pos", dz=0.25)]
    )
    worst = max(abs(d) for *_, d in labels.adjacent_terminal_deltas())
    assert worst == pytest.approx(0.25, abs=1e-9)


def test_terminal_tilt_stands_on_its_low_edge():
    """A tilted terminal cannot sink into the cap: it stands on the edge that
    went down, so its seating centre rises by half its depth times sin(tilt).
    (Rotating about the centre and lifting differs from a true edge pivot by
    (d/2)(1 - cos) sideways -- 3 um at 1.5 deg.)"""
    _, clean = generate("pre_busbar")
    _, bad = generate("pre_busbar", defects=[defects.terminal_tilt(2, "neg", deg=1.5)])

    seat = {(t.cell_index, t.polarity): np.array(t.centre) for t in clean.terminals}
    tilted = {(t.cell_index, t.polarity): t for t in bad.terminals}
    target = tilted[(2, "neg")]

    rise = cfg.terminal_d / 2 * np.sin(np.radians(1.5))
    np.testing.assert_allclose(np.array(target.centre), seat[(2, "neg")] + [0, 0, rise], atol=1e-6)
    assert target.normal[2] == pytest.approx(np.cos(np.radians(1.5)), abs=1e-9)


def test_missing_cell_removes_its_terminals_too():
    _, labels = generate("pre_busbar", defects=[defects.missing_cell(7)])
    assert len(labels.cells) == cfg.n_cells - 1
    assert len(labels.terminals) == (cfg.n_cells - 1) * 2
    assert all(c.index != 7 for c in labels.cells)


def test_reversed_polarity_swaps_the_terminal_rows():
    _, clean = generate("pre_busbar")
    _, bad = generate("pre_busbar", defects=[defects.reversed_polarity(1)])

    before = {(t.cell_index, t.polarity): t.centre[1] for t in clean.terminals}
    after = {(t.cell_index, t.polarity): t.centre[1] for t in bad.terminals}
    assert after[(1, "pos")] == pytest.approx(-before[(1, "pos")], abs=1e-9)
    assert after[(1, "neg")] == pytest.approx(-before[(1, "neg")], abs=1e-9)
    assert after[(2, "pos")] == pytest.approx(before[(2, "pos")], abs=1e-9)


def test_busbar_liftoff_rests_the_busbar_on_the_chip():
    """Lift-off is a chip on the busbar's last terminal. The busbar tilts onto
    it: the gap is the chip height over that terminal and shrinks toward the
    far end, which still rests on its pad. The seams ride with it."""
    from defectgen.module import busbar_plan

    gap = 0.4
    _, clean = generate("post_weld")
    _, bad = generate("post_weld", defects=[defects.busbar_liftoff(1, gap=gap)])

    lifted = [s for s in bad.seams if s.busbar_index == 1]
    original = [s for s in clean.seams if s.busbar_index == 1]
    rise = {s.cell_index: b.start[2] - s.start[2] for s, b in zip(original, lifted)}
    terms = [k for k, _ in busbar_plan(cfg)[1][1]]
    near, far = terms[-1], terms[0]
    assert 0.5 * gap < rise[near] <= gap + 1e-6
    assert rise[far] < rise[near]
    assert min(rise.values()) > -1e-6, "nothing may sink below its nominal seat"
    assert bad.busbars[0].underside_z == pytest.approx(clean.busbars[0].underside_z)


def test_busbar_pose_leaves_other_busbars_alone():
    _, bad = generate("pre_weld", defects=[defects.busbar_pose(3, dx=1.2, dtheta_deg=0.8)])
    assert classes(bad) == ["busbar_pose"]
    assert bad.defects[0].magnitude == pytest.approx(1.2)


def test_missing_weld_drops_only_that_joint():
    _, clean = generate("post_weld")
    _, bad = generate("post_weld", defects=[defects.missing_weld(0, 2)])
    assert len(clean.seams) - len(bad.seams) == cfg.weld_stitches
    assert not [s for s in bad.seams if s.busbar_index == 0 and s.cell_index == 2]


def test_short_weld_shortens_the_seam():
    _, bad = generate("post_weld", defects=[defects.short_weld(0, 1, fraction=0.5)])
    seam = next(s for s in bad.seams if s.busbar_index == 0 and s.cell_index == 1)
    length = seam.end[0] - seam.start[0]
    assert length == pytest.approx(cfg.weld_stitch_l * 0.5)


def test_seam_defects_record_magnitudes():
    _, labels = generate(
        "post_weld",
        defects=[
            defects.seam_deviation(1, 4, dy=0.6),
            defects.width_deviation(1, 5, factor=1.8),
            defects.reinforcement(1, 6, dh=-0.15),
        ],
    )
    assert set(classes(labels)) == {"seam_deviation", "width_deviation", "reinforcement"}


def test_spatter_is_reproducible_for_a_seed():
    a, la = generate("post_weld", seed=3, defects=[defects.spatter(count=5)])
    b, lb = generate("post_weld", seed=3, defects=[defects.spatter(count=5)])
    c, lc = generate("post_weld", seed=4, defects=[defects.spatter(count=5)])

    assert len(la.defects) == 5
    np.testing.assert_array_equal(a.vertices, b.vertices)
    assert a.vertices.shape == c.vertices.shape
    assert not np.array_equal(a.vertices, c.vertices)


def test_handling_damage_dents_the_struck_face():
    """26V188 (§4B.9): a mis-placed cell touches tooling and deforms. The dent
    goes on the long face, pushed inward."""
    struck = (cfg.cell_centre_x(0), cfg.cell_w / 2, 50.0)
    clean, _ = generate("pre_busbar")
    dented, labels = generate(
        "pre_busbar",
        defects=[
            defects.handling_damage(
                ("cell", 0), depth=0.6, radius=8.0, centre=struck,
                direction=(0.0, -1.0, 0.0),
            )
        ],
    )
    assert classes(labels) == ["handling_damage"]
    assert labels.defects[0].magnitude == pytest.approx(0.6)

    # The struck face is pulled in, so the cell no longer reaches cell_w/2 there.
    near = dented.vertices[
        np.linalg.norm(dented.vertices - np.array(struck), axis=1) < 1.0
    ]
    assert near.size, "subdivision produced no vertices at the dent centre"
    assert near[:, 1].max() < cfg.cell_w / 2 - 0.5
    assert clean.bounds[1][1] == pytest.approx(dented.bounds[1][1])


def test_dent_that_misses_the_part_is_an_error():
    with pytest.raises(ValueError, match="touches no part"):
        generate(
            "pre_busbar",
            defects=[
                defects.handling_damage(("cell", 0), depth=0.5, radius=1.0,
                                        centre=(0.0, 0.0, 500.0))
            ],
        )


def test_missing_pad_and_cover_misalignment():
    _, labels = generate(
        "post_weld",
        defects=[defects.missing_pad(3), defects.cover_misalignment(dx=1.5, dy=-0.7)],
    )
    assert set(classes(labels)) == {"missing_pad", "cover_misalignment"}


def test_carrier_seating_lifts_the_carrier():
    _, labels = generate("pre_weld", defects=[defects.carrier_seating(dz=0.5, tilt_deg=0.3)])
    assert classes(labels) == ["carrier_seating"]
    assert labels.defects[0].magnitude == pytest.approx(0.5)


def test_fod_adds_a_particle():
    clean, _ = generate("pre_busbar")
    withfod, labels = generate(
        "pre_busbar", defects=[defects.fod(centre=(0.0, 0.0, 95.0), radius=0.8)]
    )
    assert len(withfod.vertices) > len(clean.vertices)
    assert classes(labels) == ["fod"]


def test_defects_compose_and_all_are_recorded():
    _, labels = generate(
        "post_weld",
        defects=[
            defects.cell_placement(0, dx=0.4),
            defects.terminal_height(0, "pos", dz=0.2),
            defects.missing_weld(3, 1),
            defects.spatter(count=2),
        ],
    )
    assert classes(labels)[:2] == ["cell_placement", "terminal_height"]
    assert len(labels.defects) == 2 + cfg.weld_stitches + 2


def test_defects_are_skipped_before_their_stage_exists():
    """The same defect list describes all three stages, so an injector that
    targets a part not yet placed is skipped rather than raising. Stages are
    the ones in the §4B.2 table."""
    recipe = [
        defects.cell_placement(2, dx=0.3),
        defects.busbar_liftoff(0, gap=0.3),
        defects.missing_weld(0, 1),
    ]
    _, early = generate("pre_busbar", defects=recipe)
    _, mid = generate("pre_weld", defects=recipe)
    _, late = generate("post_weld", defects=recipe)

    assert classes(early) == ["cell_placement"]
    assert classes(mid) == ["cell_placement", "busbar_liftoff"]
    assert classes(late)[:2] == ["cell_placement", "busbar_liftoff"]
    assert "missing_weld" in classes(late)


def test_every_defect_carries_class_magnitude_location_and_stage():
    """§4B.6: that label set is what makes precision/recall measurable."""
    _, labels = generate("post_weld", defects=[defects.terminal_height(1, "neg", dz=0.3)])
    d = labels.defects[0]
    assert d.cls and d.stage == "post_weld"
    assert isinstance(d.magnitude, float)
    assert len(d.location) == 3


# Bounding boxes -- what a viewer draws, and what detection matches against.

def _size(defect):
    lo, hi = np.array(defect.bounds)
    return hi - lo


def test_every_defect_gets_a_box():
    from defectgen.__main__ import demo_defects

    _, labels = generate("post_weld", defects=demo_defects(cfg))
    assert labels.defects and all(d.bounds is not None for d in labels.defects)
    for d in labels.defects:
        lo, hi = np.array(d.bounds)
        assert np.all(lo <= hi), d.cls


def test_short_weld_box_keeps_the_nominal_length():
    """The point of the box: the short seam visibly fails to fill it."""
    _, labels = generate("post_weld", defects=[defects.short_weld(0, 1, fraction=0.5)])
    for d in labels.defects:
        assert _size(d)[0] == pytest.approx(cfg.weld_stitch_l)
        assert _size(d)[1] == pytest.approx(cfg.weld_stitch_w)


def test_missing_cell_box_is_where_the_cell_belonged():
    _, labels = generate("pre_busbar", defects=[defects.missing_cell(5)])
    (d,) = labels.defects
    lo, hi = np.array(d.bounds)
    assert (lo[0] + hi[0]) / 2 == pytest.approx(cfg.cell_centre_x(5))
    assert _size(d)[0] == pytest.approx(cfg.cell_t)
    assert hi[2] == pytest.approx(cfg.z_terminal_top)


def test_moved_busbar_box_spans_both_poses():
    """The busbar tilts onto the chip, so the box runs from the nominal
    underside up to at least the chip-raised top -- a little more, since the
    tilt adds height across the busbar's width."""
    _, labels = generate("pre_weld", defects=[defects.busbar_liftoff(2, gap=0.2)])
    (d,) = labels.defects
    lo, hi = np.array(d.bounds)
    assert lo[2] == pytest.approx(cfg.z_terminal_top, abs=1e-6)
    assert hi[2] >= cfg.z_terminal_top + cfg.busbar_t + 0.2 - 1e-6
    assert hi[2] <= cfg.z_terminal_top + cfg.busbar_t + 0.2 + 0.05


def test_spatter_box_is_the_particle():
    _, labels = generate("post_weld", defects=[defects.spatter(count=3, radius=0.2)])
    for d in labels.defects:
        np.testing.assert_allclose(_size(d), 0.4)


# Settling -- nothing passes through anything.

def _overlaps(stage, injectors):
    """Exact overlap volume between every pair of rigid placed parts."""
    import itertools

    import trimesh

    from defectgen.module import injected_plan
    from defectgen.parts import load_all
    from defectgen.settle import world_mesh

    parts = load_all(cfg)
    plan = injected_plan(stage, cfg, 0, injectors, parts)
    meshes = [
        (p.ref, world_mesh(plan, parts, p))
        for p in plan.placements
        # cmc is non-manifold (spec §7) and terminal_neg's plug sits in the
        # carrier by design until fixlist E5 is resolved.
        if p.present and p.ref[0] != "cmc"
    ]
    found = []
    for (ra, a), (rb, b) in itertools.combinations(meshes, 2):
        if "carrier" in (ra[0], rb[0]) and "terminal" in (ra[0], rb[0]):
            continue
        lo, hi = np.maximum(a.bounds[0], b.bounds[0]), np.minimum(a.bounds[1], b.bounds[1])
        if np.any(hi - lo <= 1e-6):
            continue
        v = trimesh.boolean.intersection([a, b], engine="manifold").volume
        if v > 1e-3:
            found.append((ra, rb, round(v, 3)))
    return found


def test_realistic_defects_leave_nothing_interpenetrating():
    from defectgen.__main__ import realistic_defects

    assert _overlaps("post_weld", realistic_defects(cfg)) == []


def test_a_cell_pushed_sideways_bulges_the_side_plate():
    from defectgen.module import injected_plan
    from defectgen.parts import load_all
    from defectgen.settle import world_mesh

    parts = load_all(cfg)
    plan = injected_plan("pre_busbar", cfg, 0, [defects.cell_placement(6, dy=0.8)], parts)
    plate = world_mesh(plan, parts, plan.find(("side_plate", "right")))
    assert plate.bounds[1][1] == pytest.approx(cfg.cell_w / 2 + cfg.side_plate_t + 0.8, abs=0.05)


def test_a_cell_error_no_pad_can_absorb_is_refused():
    with pytest.raises(ValueError, match="would collide"):
        generate("pre_busbar", defects=[defects.cell_placement(4, dx=2.0)])

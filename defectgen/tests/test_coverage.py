"""Coverage (task 1.5): the geometry helpers on known answers, then coverage
on toy scenes small enough to reason about by hand."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("open3d")
import trimesh  # noqa: E402

from defectgen.coverage import (  # noqa: E402
    Requirement, View, _cone, build_elements, capture, counts, coverage, greedy, load_coverage_params,
    look_at, observe, requirements, sweep_pose,
)
from defectgen.scan import Scene, looking_down  # noqa: E402


def test_cone_stays_within_its_half_angle():
    d = _cone(32, 60.0)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)
    assert np.degrees(np.arccos(d[:, 2])).max() <= 60.0 + 1e-9


def test_look_at_and_sweep_pose_point_where_asked():
    pose = look_at([100.0, -50.0, 600.0], [0.0, 0.0, 100.0])
    r = pose[:3, :3]
    assert np.allclose(r.T @ r, np.eye(3)) and np.isclose(np.linalg.det(r), 1.0)
    z = np.array([0.0, 0.0, 100.0]) - [100.0, -50.0, 600.0]
    assert np.allclose(r[:, 2], z / np.linalg.norm(z))
    # A sweep along +X, rolled to look 30 deg off vertical, keeps its target on its axis.
    look = np.array([0.0, np.sin(np.radians(30)), -np.cos(np.radians(30))])
    sp = sweep_pose([10.0, 20.0, 101.0], [1, 0, 0], look, 245.0)
    assert np.allclose(sp[:3, 1], [1, 0, 0]) and np.allclose(sp[:3, 2], look)
    assert np.allclose(sp[:3, 3] + 245.0 * sp[:3, 2], [10.0, 20.0, 101.0])


# -- toy scenes -----------------------------------------------------------------------

def _q(D: float) -> dict:
    """The real coverage config, with one toy stage: the top of an end-plate box."""
    q = load_coverage_params()
    q["stages"] = {"toy": {"top": {"D": D, "surfaces": ["housing_tops"], "members": ["x"]}}}
    return q


def _plate(stacked: bool = False) -> dict:
    """A 40 x 40 x 5 aluminium block (the end-plate material), top at z = 5;
    optionally a second block standing on its left half."""
    a = trimesh.creation.box((40.0, 40.0, 5.0))
    a.apply_translation((0.0, 0.0, 2.5))
    if stacked:
        b = trimesh.creation.box((20.0, 40.0, 5.0))
        b.apply_translation((-10.0, 0.0, 7.5))
        a = trimesh.util.concatenate([a, b])
    return {"end_plates": a}


def _setup(groups, q):
    el = build_elements(groups, q)
    observe(el, Scene.from_groups(groups), q)
    el.make_patches(q["patch_size"])
    return el, requirements(el, "toy", q)


def test_faces_in_contact_are_not_required():
    """The lower block's top under the upper block cannot be seen from
    anywhere: it is not part of what coverage asks for."""
    q = _q(1.0)
    el, (req,) = _setup(_plate(stacked=True), q)
    tops = el.p_centroid[req.elements]
    # Required tops: the lower block's exposed right half (z 5) and the upper block (z 10).
    assert not ((tops[:, 2] < 6) & (tops[:, 0] < -1)).any()
    assert ((tops[:, 2] < 6) & (tops[:, 0] > 1)).any() and (tops[:, 2] > 9).any()


def test_a_sweep_covers_a_plate_at_the_spacing_it_delivers():
    """The Keyence over a flat plate gives ~0.05 x 0.0225 mm sampling (about
    890 points per mm^2): enough for d = 0.1 (100 per mm^2), not for
    d = 0.02 (2,500 per mm^2)."""
    groups = _plate()
    scene = Scene.from_groups(groups)
    start = looking_down(-25.0, 0.0, 5.0 + 245.0)
    view = View("toy_sweep", "profilometer", "sweep", start, 50.0)
    for D, want in ((0.2, 1.0), (0.04, 0.0)):
        q = _q(D)
        el, reqs = _setup(groups, q)
        r, scale = capture(view, scene, q, cast_pitch=None)
        cov = coverage(counts(r, el, q, scale), reqs)["top"]
        assert cov == pytest.approx(want, abs=0.02)


def test_points_too_noisy_for_a_class_do_not_count():
    """§3.7.3's other half: a point serves a measurement of size D only if
    its noise is <= D/3. A Keyence sweep samples finely enough for D = 0.2
    (spacing passes); the same points read with 0.1 mm noise (> 0.2/3) cover
    nothing, and the model's own noise (~1 um) covers it all."""
    groups = _plate()
    scene = Scene.from_groups(groups)
    q = _q(0.2)
    el, reqs = _setup(groups, q)
    view = View("toy_sweep", "profilometer", "sweep", looking_down(-25.0, 0.0, 250.0), 50.0)
    r, scale = capture(view, scene, q, cast_pitch=None)
    assert np.median(r.sigma) < 0.005
    assert coverage(counts(r, el, q, scale), reqs)["top"] == pytest.approx(1.0, abs=0.02)
    r.sigma = np.full_like(r.sigma, 0.1)
    assert coverage(counts(r, el, q, scale), reqs)["top"] == pytest.approx(0.0, abs=0.02)


def test_a_coarse_cast_scaled_counts_like_the_full_density():
    """Counts scale linearly with profile density, so casting candidates at a
    coarser pitch and scaling recovers the flown sweep's counts."""
    groups = _plate()
    scene = Scene.from_groups(groups)
    q = _q(0.2)
    el, _ = _setup(groups, q)
    view = View("toy_sweep", "profilometer", "sweep", looking_down(-25.0, 0.0, 250.0), 50.0)
    r, scale = capture(view, scene, q, None)
    full = counts(r, el, q, scale)
    r, scale = capture(view, scene, q, 0.2)
    coarse = counts(r, el, q, scale)
    assert scale == pytest.approx(4.0)
    assert coarse.sum() == pytest.approx(full.sum(), rel=0.02)


def test_coverage_is_per_capture_never_summed():
    """Two captures that each deliver half the points a patch needs do not
    cover it together: every measurement happens within one capture."""
    req = Requirement("x", 0.2, ["x"], np.array([0, 1]), np.array([10.0, 10.0]), np.array([1.0, 1.0]), 2.0)
    a = np.array([[6.0, 12.0]])
    b = np.array([[6.0, 0.0]])
    assert coverage([a, b], [req])["x"] == pytest.approx(0.5)
    assert coverage(a + b, [req])["x"] == pytest.approx(1.0)   # what summing would wrongly claim


def test_greedy_prefers_coverage_per_second_and_stops():
    """Equal-weight classes; the fast view that covers class `a` beats a slow
    one that covers the same, and a view adding nothing is never taken."""
    q = {"plan": {"t_move": 1.0, "t_snapshot": 0.1, "sweep_speed": 50.0, "min_gain": 1e-3}}
    ra = Requirement("a", 1, [], np.array([0]), np.array([1.0]), np.array([1.0]), 1.0)
    rb = Requirement("b", 1, [], np.array([1]), np.array([1.0]), np.array([1.0]), 1.0)
    fast = View("fast", "snapshot", "snapshot", np.eye(4))
    slow = View("slow", "profilometer", "sweep", np.eye(4), 500.0)
    other = View("other", "snapshot", "snapshot", np.eye(4))
    useless = View("useless", "snapshot", "snapshot", np.eye(4))
    n = {"fast": np.array([[1.0, 0.0]]), "slow": np.array([[1.0, 0.0]]),
         "other": np.array([[0.0, 1.0]]), "useless": np.array([[0.0, 0.0]])}
    steps = greedy([slow, fast, other, useless], n, [ra, rb], q)
    assert [s.view.id for s in steps] == ["fast", "other"]
    assert steps[-1].coverage == {"a": 1.0, "b": 1.0}


def test_a_tilted_snapshot_sees_what_a_straight_one_does():
    """Regression: the snapshot model's projector field-of-view test used
    camera-to-world where it needed world-to-camera. Straight-down views hid
    it (that rotation is its own transpose); a tilted view lost every point
    as 'outside_projector'."""
    from defectgen.scan import snapshot

    plate = trimesh.creation.box((400.0, 400.0, 5.0))
    scene = Scene.from_groups({"end_plates": plate})
    straight = snapshot(scene, look_at([0.0, 0.0, 600.0], [0.0, 0.0, 0.0]))
    for az in (0.0, 90.0, 200.0):
        a, e = np.radians(az), np.radians(65.0)
        eye = 600.0 * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        tilted = snapshot(scene, look_at(eye, [0.0, 0.0, 0.0]))
        assert len(tilted.points) > 0.5 * len(straight.points)

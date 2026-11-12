"""The labelled dataset (task 1.7): recipes are deterministic, the out-of-spec
rule judges from the true parameters, and the GRR design is crossed.
No scans here; the pilot run exercises the whole pipeline."""

from __future__ import annotations

import json

import numpy as np
import pytest

pytest.importorskip("open3d")

from defectgen.dataset import (  # noqa: E402
    Recipe, SetupCoverage, _seam_pairs, _seed, clean_cases, fixture_error, grr_cases, judge, levels,
    load_dataset_params, pod_cases, score, tolerance,
)
from defectgen.config import load  # noqa: E402
from defectgen.detect import load_detect_params  # noqa: E402

Q, DP = load_dataset_params(), load_detect_params()


def test_seeds_are_stable_and_distinct():
    assert _seed("pod", "fod", 2, 3) == _seed("pod", "fod", 2, 3)
    cases = pod_cases(Q, DP)
    assert len({c.seed for c in cases}) == len(cases)
    assert [c.seed for c in pod_cases(Q, DP)] == [c.seed for c in cases]


def test_tolerance_levels_straddle_the_tolerance():
    fam = Q["families"]["terminal_height"]
    tol = tolerance(fam, DP)
    assert tol == DP["terminal"]["height_tol"]
    lv = levels(fam, Q, DP)
    assert sum(v <= tol for v in lv) == 2 and sum(v > tol for v in lv) == len(lv) - 2


def _r(family, magnitude, **params):
    fam = Q["families"][family]
    p = dict(params)
    if "D" in fam:
        p["D"] = fam["D"]
    return Recipe("pod", family, fam["cls"], fam["stage"], fam["kind"], magnitude, 0, "x", p,
                  tol=tolerance(fam, DP))


def test_judge_uses_the_true_parameters():
    tol = DP["cell"]["pos_tol"]
    # A diagonal shift is judged on its length, not on either component.
    assert judge(_r("cell_shift", 0.15, dx=0.15, dy=0.15)) == "must_find"
    assert judge(_r("cell_shift", 0.8 * tol, dx=0.8 * tol, dy=0.0)) == "in_spec"
    assert judge(_r("terminal_height", 0.08, dz=-0.08)) == "in_spec"
    assert judge(_r("terminal_height", 0.12, dz=-0.12)) == "must_find"
    assert judge(_r("spatter", 0.1)) == "dont_care"
    assert judge(_r("spatter", 0.15)) == "must_find"
    assert judge(_r("missing_cell", 1.0)) == "must_find"


def test_fixture_error_is_a_small_rigid_motion():
    rng = np.random.default_rng(0)
    ts = [fixture_error(rng, Q) for _ in range(500)]
    for t in ts[:20]:
        assert np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3)) and np.isclose(np.linalg.det(t[:3, :3]), 1.0)
    xy = np.array([t[:2, 3] for t in ts])
    assert np.std(xy) == pytest.approx(Q["fixture"]["sigma_xy"], rel=0.15)


def test_grr_is_crossed_parts_by_loads_by_trials():
    g = Q["tiers"]["grr"]
    cases = [c for c in grr_cases(Q, DP) if c.name.startswith("terminal_height")]
    assert len(cases) == g["parts"] * g["loads"] * g["trials"]
    # A part keeps its seed (so its defect location and true value) across loads and trials.
    by_part = {}
    for c in cases:
        by_part.setdefault(c.params["part"], set()).add((c.seed, c.params["dz"]))
    assert all(len(v) == 1 for v in by_part.values())
    dz = sorted(next(iter(v))[1] for v in by_part.values())
    tol = DP["terminal"]["height_tol"]
    assert dz[0] == pytest.approx(-g["spread"] * tol) and dz[-1] == pytest.approx(g["spread"] * tol)


def test_every_family_is_injectable_and_every_stage_has_clean_modules():
    from defectgen.dataset import injector

    cfg = load()
    assert len(clean_cases(Q)) == 3 * Q["tiers"]["clean"]["modules"]
    assert all(0 <= b < 5 and 0 <= c < cfg.n_cells for b, c in _seam_pairs(cfg))
    for fam in Q["families"]:
        if fam in ("fod", "handling_damage"):
            continue          # these need the scene to find a surface point; the pilot covers them
        r = _r(fam, levels(Q["families"][fam], Q, DP)[-1])
        assert callable(injector(r, cfg, None, np.random.default_rng(1)))
        assert r.params


def _dent(x, y, z, r=4.0):
    return {"cls": "handling_damage", "magnitude": 0.4, "location": [x, y, z],
            "bounds": [[x - r, y - 0.4, z - r], [x + r, y + 0.01, z + r]], "stage": "pre_busbar"}


def _report(*finds):
    return {"stage": "pre_busbar", "result": "NOK" if finds else "OK", "uncovered": {},
            "findings": [{"cls": "handling_damage", "magnitude": 0.4, "location": d["location"], "bounds": d["bounds"],
                          "feature": "", "capture": "fixed"} for d in finds]}


def test_a_miss_where_the_setup_does_not_look_is_not_covered():
    """POD is conditional on the location being inspected: a dent on a face
    the setup never looks at is a coverage gap, scored apart from misses."""
    r = Recipe("pod", "handling_damage", "handling_damage", "pre_busbar", "presence", 0.4, 1, "x", truth="must_find")
    a, b = _dent(0, -75.5, 50), _dent(100, -75.5, 50)
    labels = {"defects": [a, b]}
    blind = lambda d: d["location"][0] != 0  # noqa: E731   (a is out of view, b in view)
    assert score(_report(), {"defects": [a]}, r, seen=blind)["outcome"] == "not_covered"
    assert score(_report(), {"defects": [a]}, r)["outcome"] == "miss"            # no coverage map: a miss
    assert score(_report(), {"defects": [b]}, r, seen=blind)["outcome"] == "miss"
    assert score(_report(b), labels, r, seen=blind)["outcome"] == "hit"         # found all it looks at
    assert score(_report(a), labels, r, seen=blind)["outcome"] == "partial"     # missed one in view
    sc = score(_report(), labels, r, seen=blind)
    assert sc["outcome"] == "miss" and sc["not_looked_at"] == 1


def test_setup_coverage_reads_the_patches_task_1_5_saved(tmp_path):
    """Two handling-damage patches: a top face both setups cover and a side
    face only the arm covers. A dent's box is looked up against them."""
    np.savez(tmp_path / "pre_busbar.patches.npz",
             centroid=np.array([[0.0, 70.0, 105.0], [0.0, 75.5, 50.0]]),
             classes=np.array(["cell_placement", "handling_damage"]),
             required=np.array([[False, False], [True, True]]),
             fixed=np.array([[False, False], [True, False]]),
             arm=np.array([[False, False], [True, True]]))
    cov = SetupCoverage(tmp_path)
    side = [[-4, 75.1, 46], [4, 75.51, 54]]
    assert cov.seen("pre_busbar", "fixed", "handling_damage", side) is False
    assert cov.seen("pre_busbar", "arm", "handling_damage", side) is True
    assert cov.seen("pre_busbar", "fixed", "handling_damage", [[-1, 69, 104], [1, 71, 105]]) is True
    assert cov.seen("pre_busbar", "fixed", "missing_pad", side) is None      # family not mapped here
    assert cov.seen("pre_busbar", "fixed", "handling_damage", [[50, 0, 0], [51, 1, 1]]) is None  # nothing required
    assert cov.seen("post_weld", "fixed", "handling_damage", side) is None   # no map for that stage


def test_a_second_setup_run_later_does_not_hide_the_first(tmp_path):
    from defectgen.dataset import read_index

    f = tmp_path / "index.jsonl"
    rows = [{"family": "a", "name": "x", "setups": {"fixed": {"outcome": "miss"}}},
            {"family": "a", "name": "y", "setups": {"fixed": {"outcome": "hit"}}},
            {"family": "a", "name": "x", "setups": {"fixed": {"outcome": "hit"}}},     # resumed: latest wins
            {"family": "a", "name": "x", "setups": {"arm": {"outcome": "hit"}}}]       # arm column, later
    f.write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    idx = read_index(f)
    assert list(idx) == [("a", "y"), ("a", "x")]
    assert idx[("a", "x")]["setups"] == {"fixed": {"outcome": "hit"}, "arm": {"outcome": "hit"}}

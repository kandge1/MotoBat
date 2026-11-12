"""validate.py: the statistics behind 1.8 and 1.9, and how they read the
stored dataset. Light: no scanning, no detection."""

import json

import numpy as np
import pytest

from defectgen import validate as V


def test_wilson_matches_the_published_interval():
    # Newcombe (1998), Stat. Med. 17:857, method 3: 5/10 -> 0.2366-0.7634.
    lo, hi = V.wilson(5, 10)
    assert lo == pytest.approx(0.2366, abs=1e-4) and hi == pytest.approx(0.7634, abs=1e-4)
    # The edges stay informative and inside [0, 1]: 0 of 10 is not "0 % exactly".
    assert V.wilson(0, 10) == (0.0, pytest.approx(0.2775, abs=1e-4))
    assert V.wilson(10, 10) == (pytest.approx(0.7225, abs=1e-4), 1.0)


def test_poisson_bound_is_the_rule_of_three_at_zero():
    assert V.poisson_upper(0, 100) == pytest.approx(-np.log(0.05) / 100)
    assert V.poisson_upper(2, 50) == pytest.approx(6.296 / 50, rel=1e-3)     # tabulated 95 % bound for k = 2


def test_a_perfect_gauge_has_no_grr():
    true = np.linspace(-1, 1, 10)
    x = np.repeat(np.repeat(true[:, None, None], 3, axis=1), 2, axis=2)
    r = V.anova_grr(x, tol_width=2.0)
    assert r["GRR"] == pytest.approx(0.0, abs=1e-12)
    assert r["pct_grr_tol"] == pytest.approx(0.0, abs=1e-9)
    assert r["ndc"] is None


def test_anova_recovers_known_variance_components():
    rng = np.random.default_rng(0)
    p, o, r = 400, 6, 4
    sp, so, spo, se = 1.0, 0.2, 0.15, 0.1
    x = (rng.normal(0, sp, p)[:, None, None] + rng.normal(0, so, o)[None, :, None]
         + rng.normal(0, spo, (p, o))[:, :, None] + rng.normal(0, se, (p, o, r)))
    res = V.anova_grr(x, tol_width=6.0)
    assert not res["interaction_pooled"]
    assert res["EV"] == pytest.approx(se, rel=0.05)
    assert np.sqrt(res["var"]["interaction"]) == pytest.approx(spo, rel=0.1)
    assert res["PV"] == pytest.approx(sp, rel=0.1)
    # Six appraisers estimate their own spread only roughly: a loose check.
    assert np.sqrt(res["var"]["appraiser"]) == pytest.approx(so, rel=0.6)
    assert res["pct_grr_tol"] == pytest.approx(100 * res["GRR"], rel=1e-12)     # 6 sigma over a width of 6


def test_no_interaction_is_pooled_into_repeatability():
    rng = np.random.default_rng(1)
    x = np.linspace(0, 1, 10)[:, None, None] + rng.normal(0, 0.01, (10, 3, 2))
    res = V.anova_grr(x)
    if res["interaction_pooled"]:
        ss, df = res["ss"], res["df"]
        assert res["var"]["repeatability"] == pytest.approx(
            (ss["interaction"] + ss["repeatability"]) / (df["interaction"] + df["repeatability"]))
        assert res["var"]["interaction"] == 0.0


def test_bias_and_linearity():
    true = np.array([-0.2, -0.1, 0.0, 0.1, 0.2])
    x = np.repeat((true * 1.1 + 0.01)[:, None], 6, axis=1)     # reads 10 % long, 10 um high
    b = V.bias_linearity(true, x)
    assert b["slope"] == pytest.approx(0.1) and b["intercept"] == pytest.approx(0.01)


def _case(root, tier, family, name, truth, outcome, cls="terminal_height", result="NOK", fa=0, params=None,
          measured=None, tol=0.1, stage="pre_busbar"):
    d = root / tier / family / name
    d.mkdir(parents=True)
    (d / "case.json").write_text(json.dumps({"params": params or {}, "tol": tol}))
    (d / "labels.json").write_text(json.dumps({"defects": []}))
    (d / "fixed.report.json").write_text(json.dumps({"stage": stage, "findings": []}))
    return {"tier": tier, "family": family, "cls": cls, "name": name, "truth": truth, "magnitude": 0.1,
            "setups": {"fixed": {"outcome": outcome, "result": result, "false_alarms": fa, "measured": measured}}}


def test_pr_counts_outcomes_decisions_and_false_alarms(tmp_path):
    rows = {"pod": [_case(tmp_path, "pod", "terminal_height", "a", "must_find", "hit"),
                    _case(tmp_path, "pod", "terminal_height", "b", "must_find", "partial"),
                    _case(tmp_path, "pod", "terminal_height", "c", "must_find", "miss", result="OK"),
                    _case(tmp_path, "pod", "terminal_height", "d", "must_find", "not_covered", result="OK"),
                    _case(tmp_path, "pod", "terminal_height", "e", "in_spec", "false_reject"),
                    _case(tmp_path, "pod", "terminal_height", "f", "in_spec", "pass", result="OK", fa=1)],
            "clean": [_case(tmp_path, "clean", "pre_busbar", "m0", "clean", "clean_ok", cls="", result="OK")]}
    for tier, rs in rows.items():
        (tmp_path / tier / "index.jsonl").write_text("\n".join(json.dumps(r) for r in rs) + "\n")
    res = V.pr(tmp_path, "fixed")
    f = res["families"]["terminal_height"]
    assert f["in_view"] == 3 and f["not_covered"] == 1          # not_covered is outside recall
    assert f["recall"] == pytest.approx(1 / 3)                  # a partial is not a hit ...
    assert f["rejected"] == pytest.approx(2 / 3)                # ... but its module was rejected
    assert f["false_reject"] == pytest.approx(1 / 2)
    assert f["in_spec_rejected"] == pytest.approx(1 / 2)        # e NOK, f OK
    assert res["false_alarms"]["total"] == 1 and res["false_alarms"]["total_modules"] == 7
    assert res["clean"]["pre_busbar"]["with_findings"] == 0
    assert "terminal_height" in V.format_pr(res)


def test_grr_reads_a_crossed_study_and_drops_incomplete_parts(tmp_path):
    rows = []
    for pi, v in enumerate(np.linspace(-0.15, 0.15, 4)):
        for li in range(3):
            for ti in range(2):
                m = None if (pi == 3 and li == 2) else v + 0.001 * li + 0.0005 * ti
                rows.append(_case(tmp_path, "grr", "terminal_height", f"terminal_height_p{pi}_l{li}_t{ti}",
                                  "must_find", "hit", params={"dz": v, "part": pi, "load": li, "trial": ti},
                                  measured=m))
    (tmp_path / "grr" / "index.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    res = V.grr(tmp_path, "fixed")["characteristics"]["terminal_height"]
    assert res["shape"] == [3, 3, 2] and res["parts_dropped"] == [3]
    assert res["tol_width"] == pytest.approx(0.2)               # +/- 0.1: two-sided
    assert res["AV"] > res["EV"] > 0


def test_reading_error_shows_a_proportional_shortfall():
    from defectgen.validate import reading_error

    true = np.repeat([0.1, 0.2, 0.3, 0.4], 5)
    r = reading_error(list(zip(true, 0.9 * true + 0.002)))
    assert r["n"] == 20
    assert r["slope"] == pytest.approx(-0.1, abs=1e-9)
    assert r["mean"] == pytest.approx(-0.1 * true.mean() + 0.002, abs=1e-9)
    assert reading_error([(0.2, 0.21)] * 5)["slope"] is None    # one size: no trend to fit

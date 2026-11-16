"""The POD fit's claims, each checked against a case where the answer is known."""
import math

import numpy as np
import pytest

from analysis.pod import (LOGIT90, a90_95, analyse, firth_logistic, nonparametric, separated,
                          size_at)


def simulate(b0, b1, sizes, n, seed=0):
    rng = np.random.default_rng(seed)
    a = np.repeat(sizes, n)
    p = 1 / (1 + np.exp(-(b0 + b1 * np.log(a))))
    return np.log(a), (rng.random(a.size) < p).astype(float)


def test_firth_recovers_a_known_curve_on_a_large_sample():
    # a50 = 0.2, slope 8 in log size
    b1 = 8.0
    b0 = -b1 * math.log(0.2)
    x, y = simulate(b0, b1, np.geomspace(0.08, 0.5, 9), 400)
    b, V = firth_logistic(x, y)
    assert size_at(b, 0.0) == pytest.approx(0.2, rel=0.03)
    assert b[1] == pytest.approx(b1, rel=0.1)
    assert np.all(np.linalg.eigvalsh(V) > 0)


def test_firth_is_close_to_the_mle_without_separation():
    x, y = simulate(4.0, 3.0, np.geomspace(0.05, 1.0, 7), 30, seed=3)
    assert not separated(x, y)
    X = np.column_stack([np.ones_like(x), x])
    mle = np.zeros(2)
    for _ in range(50):                      # plain Newton on the unpenalised likelihood
        p = 1 / (1 + np.exp(-X @ mle))
        mle += np.linalg.solve(X.T @ (X * (p * (1 - p))[:, None]), X.T @ (y - p))
    b, _ = firth_logistic(x, y)
    assert np.allclose(b, mle, rtol=0.15)


def test_separated_data_gives_a_finite_step_between_the_levels():
    x = np.log(np.repeat([0.1, 0.16, 0.22, 0.26], 5))
    y = np.repeat([0, 0, 1, 1], 5).astype(float)
    assert separated(x, y)
    b, V = firth_logistic(x, y)
    assert np.all(np.isfinite(b)) and np.all(np.isfinite(V))
    assert 0.16 < size_at(b, 0.0) < 0.22
    # with 95 % confidence it is a larger size than the point estimate
    a9095 = a90_95(b, V, 0.1, 0.26)
    assert a9095 is None or a9095 > size_at(b, LOGIT90)


def test_a90_95_is_not_extrapolated_past_the_largest_size():
    x = np.log(np.repeat([0.1, 0.2, 0.3], 5))
    y = np.array([0] * 5 + [1, 0, 1, 0, 1] + [1, 1, 1, 0, 1], float)
    b, V = firth_logistic(x, y)
    assert a90_95(b, V, 0.1, 0.3) is None


def test_the_n_of_n_bound_is_the_binomial_one():
    lv = [{"size": 0.1, "n": 5, "detected": 2}, {"size": 0.2, "n": 5, "detected": 5},
          {"size": 0.3, "n": 24, "detected": 24}]
    r = nonparametric(lv)
    assert r["from_size"] == 0.2 and r["n"] == 29
    assert r["pod_lower95"] == pytest.approx(0.902, abs=1e-3)     # the 29 of 29 rule
    assert nonparametric([{"size": 0.3, "n": 5, "detected": 4}]) is None


def _cases(rows):
    return [{"size": a, "y": y, "truth": t, "outcome": "", "name": "", "cls": ""} for a, y, t in rows]


def test_a_tolerance_family_gets_t_from_its_levels_and_flags_a_dip():
    rows = ([(0.10, 0, "in_spec")] * 5 + [(0.16, 0, "in_spec")] * 5 + [(0.22, 1, "must_find")] * 5
            + [(0.26, 1, "must_find")] * 5 + [(0.32, 0, "must_find")] * 3 + [(0.32, 1, "must_find")] * 2)
    r = analyse(_cases(rows))
    assert r["kind"] == "tolerance"
    assert r["threshold"] == pytest.approx(0.2)
    assert r["non_monotone"] == [(0.26, 0.32)]
    assert r["nonparametric"] is None

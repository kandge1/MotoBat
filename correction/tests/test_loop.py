"""The closed form and the simulation must agree; each property the write-up claims is a test.
Run from the repo root:  python -m pytest -p no:launch_testing -p no:launch_ros -q correction/tests
"""
import math

import numpy as np
import pytest

from correction import loop as L


def plant(offset=(0.15, -0.1, 0.06), ramp=(0, 0, 0), step=(0, 0, 0), sp=(0.05, 0.05, 0.03),
          sm=(0.03, 0.03, 0.02), bias=(0, 0, 0), g=1.0, step_at=10**9):
    a = lambda t: np.array(t, float)
    return L.Plant(g, a(sp), a(offset), a(ramp), step_at, a(step), a(sm), a(bias))


TOL = np.array([0.2, 0.2, 0.1])


@pytest.mark.parametrize("a, expect", [(0.5, True), (1.0, True), (1.99, True), (2.01, False), (-0.1, False)])
def test_i_form_is_stable_exactly_for_0_lt_gk_lt_2(a, expect):
    assert L.stable(0.0, a) is expect


@pytest.mark.parametrize("a, expect", [(0.5, True), (0.99, True), (1.01, False)])
def test_p_form_is_stable_only_for_gk_lt_1(a, expect):
    assert L.stable(a, 0.0) is expect


def test_jury_conditions_match_the_poles():
    rng = np.random.default_rng(1)
    for a_p, a_i in rng.uniform([-0.5, 0.0], [1.2, 2.5], (500, 2)):
        jury = abs(a_p) < 1 and a_i > 0 and a_i + 2 * a_p < 2
        assert L.stable(a_p, a_i) == jury


def test_i_form_error_shrinks_by_1_minus_gk_each_cell():
    p = plant(sp=(0, 0, 0), sm=(0, 0, 0))
    for k in (0.3, 1.0, 1.6):
        x = L.simulate(p, L.I(k), 6, 1, noise=False)["x"][0, :, 0]
        assert np.allclose(x[1:], x[:-1] * (1 - k))


def test_deadbeat_removes_an_offset_in_one_cell_and_divergence_grows():
    p = plant(sp=(0, 0, 0), sm=(0, 0, 0))
    x = L.simulate(p, L.I(1.0), 5, 1, noise=False)["x"][0]
    assert np.allclose(x[1:], 0)
    x = L.simulate(p, L.I(2.1), 40, 1, noise=False)["x"][0, :, 0]
    assert abs(x[-1]) > 10 * abs(x[0])


def test_p_form_leaves_an_offset_and_i_form_does_not():
    p = plant(sp=(0, 0, 0), sm=(0, 0, 0))
    xp = L.simulate(p, L.P(0.5), 200, 1, noise=False)["x"][0, -1]
    xi = L.simulate(p, L.I(0.5), 200, 1, noise=False)["x"][0, -1]
    assert np.allclose(xp, p.offset / 1.5)
    assert np.allclose(xi, 0, atol=1e-12)


def test_a_gauge_bias_lands_in_placement_and_a_ramp_lags_by_r_over_gk():
    p = plant(sp=(0, 0, 0), sm=(0, 0, 0), ramp=(0.001, 0, 0), bias=(-0.025, 0.01, 0))
    x = L.simulate(p, L.I(0.4), 400, 1, noise=False)["x"][0, -1]
    assert x[0] == pytest.approx(0.001 / 0.4 + 0.025, abs=1e-9)
    assert x[1] == pytest.approx(-0.01, abs=1e-9)
    assert L.steady_mean(0, 0.4, 0, 0.001, -0.025) == pytest.approx(x[0])


@pytest.mark.parametrize("gains", [L.I(0.3), L.I(1.0), L.I(1.7), L.P(0.6), L.Gains("PI", 0.3, 0.4)])
def test_steady_variance_matches_monte_carlo(gains):
    p = plant(offset=(0, 0, 0))
    x = L.simulate(p, gains, 3000, 60, seed=3)["x"][:, 500:, 0]
    var = L.steady_var(gains.kp, gains.ki, 0.05, 0.03)
    assert x.var() == pytest.approx(var, rel=0.05)


def test_deadbeat_doubles_placement_variance():
    """Correcting every cell fully on one noisy reading: σ² = 2σ_w² + σ_v² (Deming's funnel)."""
    assert L.steady_var(0, 1.0, 0.05, 0.03) == pytest.approx(2 * 0.05**2 + 0.03**2)
    assert L.steady_var(0, 1.0, 0.05, 0.03) > L.steady_var(0, 0.0001, 0.05, 0.03)


def test_the_best_gain_is_below_deadbeat_when_noise_dominates_drift():
    k = L.best_i_gain(ramp=1e-4, bias=0.0, s_place=0.05, s_meas=0.03, tol=0.2)
    assert 0 < k < 1


def test_wrong_plant_gain_shifts_the_stability_band():
    p = plant(sp=(0, 0, 0), sm=(0, 0, 0))
    x = L.simulate(p, L.I(1.8), 80, 1, g=1.2, noise=False)["x"][0, :, 0]   # gK = 2.16
    assert abs(x[-1]) > abs(x[0])
    x = L.simulate(p, L.I(1.8), 80, 1, g=1.0, noise=False)["x"][0, :, 0]
    assert abs(x[-1]) < 1e-3


def test_closed_loop_beats_open_loop_on_the_default_scenario():
    p = plant(ramp=(1e-4, 0, 0), step=(-0.1, 0.1, -0.05), step_at=600)
    op = L.defect_rate(L.simulate(p, L.OPEN, 1200, 30)["x"], TOL)
    cl = L.defect_rate(L.simulate(p, L.I(0.3), 1200, 30)["x"], TOL)
    assert cl["hi"] < op["lo"]


def test_out_of_tolerance_is_radial_and_counts_divergence():
    x = np.array([[0.15, 0.15, 0.0], [0.19, 0.0, 0.0], [0.0, 0.0, 0.11], [np.inf, 0, 0]])
    assert L.out_of_tol(x, TOL).tolist() == [True, False, True, True]


def test_guard_band_trades_escapes_for_false_rework():
    r = [L.guard_band_rates(0.04, 0.01, 0.1, g) for g in (0.0, 0.01, 0.02, 0.03)]
    assert all(a["escape"] > b["escape"] for a, b in zip(r, r[1:]))
    assert all(a["false_rework"] < b["false_rework"] for a, b in zip(r, r[1:]))
    perfect = L.guard_band_rates(0.04, 1e-9, 0.1, 0.0)
    assert perfect["escape"] < 1e-6 and perfect["false_rework"] < 1e-6
    assert perfect["rework"] == pytest.approx(2 * (1 - 0.99379), rel=1e-3)   # P(|z| > 2.5)


def test_every_pre_weld_class_has_a_decision_and_post_weld_is_reject():
    for cls in ("cell_placement", "terminal_height", "terminal_tilt", "missing_cell", "reversed_polarity"):
        assert "before carrier" in L.decide(cls, "pre_busbar")
    for cls in ("busbar_pose", "busbar_liftoff", "carrier_seating"):
        assert "before weld" in L.decide(cls, "pre_weld")
    assert L.decide("spatter", "post_weld").startswith("reject")


def test_load_reads_tolerances_and_the_19_gauge():
    s = L.load("fixed")
    assert s.tol.tolist() == [0.2, 0.2, 0.1] and s.height_tol == 0.1
    assert 0 < s.plant.sigma_meas[0] < 0.1
    assert s.plant.sigma_meas[2] == pytest.approx(math.degrees(math.sqrt(2) * s.plant.sigma_meas[0] / 115))

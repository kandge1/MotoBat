"""ILoop against the closed forms in its docstring.  python -m pytest src/motobat_coordination/test"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from loop_controller import ILoop  # noqa: E402


def run(K, d, n, g=1.0, s_w=0.0, s_v=0.0, seed=0):
    rng = np.random.default_rng(seed)
    loop = ILoop([K], [10.0])
    xs = []
    for _ in range(n):
        x = g * loop.u[0] + d + rng.normal(0, s_w) if s_w else g * loop.u[0] + d
        xs.append(x)
        loop.update([x + (rng.normal(0, s_v) if s_v else 0.0)])
    return np.array(xs)


@pytest.mark.parametrize('K', [0.1, 0.5, 1.0, 1.5, 1.9])
def test_constant_offset_decays_as_the_pole_says(K):
    x = run(K, 0.15, 30)
    assert np.allclose(x, 0.15 * (1 - K) ** np.arange(30), atol=1e-12)


def test_diverges_past_two_over_g():
    x = run(2.1, 0.15, 40)
    assert abs(x[-1]) > abs(x[0])
    x = run(1.1, 0.15, 40, g=2.0)                      # gK = 2.2: unstable although K < 2
    assert abs(x[-1]) > abs(x[0])


def test_steady_variance_matches_the_closed_form():
    K, s_w, s_v = 0.5, 0.02, 0.005
    x = run(K, 0.0, 200000, s_w=s_w, s_v=s_v, seed=1)[1000:]
    predicted = s_w ** 2 + K * (s_w ** 2 + s_v ** 2) / (2 - K)
    assert abs(x.var() / predicted - 1) < 0.02


def test_anti_windup_clamp():
    loop = ILoop([1.0], [0.3])
    for _ in range(5):
        loop.update([1.0])                             # a 1 mm error the clamp cannot reach
    assert loop.u[0] == pytest.approx(-0.3)

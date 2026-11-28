"""SSM formulas and the walk-in safety check. (task 3.5)  python -m pytest src/motobat_safety/test"""
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(HERE))
import ssm  # noqa: E402

REACH = Path(__file__).resolve().parents[3] / 'data/reach/base_A/reach.json'


def test_iso13855_terms():
    assert ssm.intrusion_C(300) == pytest.approx(1080)          # 1200 - 0.4 H
    assert ssm.intrusion_C(1000) == pytest.approx(850)          # never below 850
    assert ssm.min_scan_height(70) == pytest.approx(300)        # H >= 15 (d - 50)
    assert ssm.min_scan_height(40) == 0


def test_field_sizes():
    d = ssm.Design()
    # 1600 (0.07 + 0.05 + 0.5) + 1080 + 65 + 100
    assert d.distance(1.0) == pytest.approx(1600 * 0.62 + 1245)
    assert d.distance(0.065) < d.distance(1.0)
    assert d.slowdown_distance(0.065) > d.distance(1.0)        # safe slowdown needs MORE room, not less
    assert ssm.Design(K=2000).distance(1.0) > d.distance(1.0)


@pytest.mark.skipif(not REACH.exists(), reason='needs 2.5 plan data (data/ is not in git)')
def test_walk_in_is_safe_and_a_short_field_is_not():
    import ssm_design as SD
    import ssm_sim as SIM
    des = ssm.Design()
    rs = SD.hull(SD.swept_footprint(SIM.S3A_X, SIM.BASE, REACH, every=8))
    ang = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    rs = SD.hull((rs[:, None, :] + des.z_r / 1e3 * np.stack([np.cos(ang), np.sin(ang)], 1)[None]).reshape(-1, 2))
    lo, hi = rs.min(0), rs.max(0)
    corridor = (lo[0], hi[0], hi[1], hi[1] + des.distance(1.0) / 1e3)
    track, pts = SIM.plan_track(REACH), SIM.robot_points_fn()
    for var in ('core', 'network'):
        assert SIM.run('walk_in', var, rs, corridor, track, pts, warning=False)['safe']
    assert not SIM.run('walk_in', 'core', rs, corridor, track, pts, shrink=0.8, warning=False)['safe']

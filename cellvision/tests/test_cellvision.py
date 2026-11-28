"""cellvision tests (task 3.2). Fast: small regions, fixed seeds.

    python -m pytest -p no:launch_testing -p no:launch_ros -q cellvision/tests
"""
import numpy as np
import pytest

from cellvision import calib as C
from cellvision import locate as Lo
from cellvision import scenes as SC
from cellvision.camera import render


def test_dlt_recovers_an_exact_homography():
    rng = np.random.default_rng(0)
    H = np.array([[7.1, 0.3, 1200.0], [-0.2, 7.3, 900.0], [1e-4, -2e-4, 1.0]])
    xy = rng.uniform(-100, 100, (40, 2))
    uv = C.apply(H, xy)
    assert np.allclose(C.dlt(xy, uv), H, rtol=1e-8, atol=1e-8)


@pytest.fixture(scope='module')
def infeed():
    cam = SC.infeed_camera()
    H, rep = C.calibrate(cam, SC.CELL_TOP, SC.INFEED, rng=np.random.default_rng(1))
    return cam, H, rep


def test_calibration_is_micron_level(infeed):
    _, _, rep = infeed
    assert rep['reproj_rms_px'] < 0.05
    assert rep['check_rms_mm'] < 0.01


def _locate_infeed(infeed, x, y, yaw, seed=2):
    cam, H, _ = infeed
    Z = SC.CAMERA_Z - SC.CELL_TOP
    roi = SC.roi_for(cam, *SC.INFEED, half=(35, 100))
    img, off = render(cam, SC.infeed_scene(x, y, yaw), roi=roi, rng=np.random.default_rng(seed))
    return Lo.locate(img, off, H, SC.CELL[:2], near_xy=np.array(SC.INFEED), gsd=cam.gsd(Z))


def test_infeed_cell_found_from_scratch(infeed):
    x, y, yaw = SC.INFEED[0] + 3.2, SC.INFEED[1] - 2.1, np.radians(1.7)
    r = _locate_infeed(infeed, x, y, yaw)
    assert abs(r['x'] - x) < 0.01 and abs(r['y'] - y) < 0.01
    assert abs(np.degrees(r['yaw'] - yaw)) < 0.01
    assert abs(r['sx'] - SC.CELL[0]) < 0.02 and abs(r['sy'] - SC.CELL[1]) < 0.02


@pytest.mark.parametrize('dx,dy,yaw_deg', [(-5, 5, -3), (5, -5, 3)])
def test_infeed_extremes(infeed, dx, dy, yaw_deg):
    x, y, yaw = SC.INFEED[0] + dx, SC.INFEED[1] + dy, np.radians(yaw_deg)
    r = _locate_infeed(infeed, x, y, yaw, seed=3)
    assert np.hypot(r['x'] - x, r['y'] - y) < 0.02
    assert abs(np.degrees(r['yaw'] - yaw)) < 0.02


def test_stack_cell_measured_among_neighbours():
    rng = np.random.default_rng(4)
    cam = SC.stack_camera()
    Z = SC.CAMERA_Z - SC.CELL_TOP
    H, _ = C.calibrate(cam, SC.CELL_TOP, (0.0, 0.0), rng=rng, n=(37, 17))
    poses = [(SC.nominal_x(i), 0.0, 0.0, 0.0) for i in range(5)] + [(SC.nominal_x(5) + 0.05, 0.2, np.radians(0.2), 0.0)]
    roi = SC.roi_for(cam, SC.nominal_x(5), 0.0, half=(25, 85))
    img, off = render(cam, SC.stack_scene(poses), roi=roi, rng=rng)
    r = Lo.locate(img, off, H, SC.CELL[:2], near_pose=(SC.nominal_x(5), 0.0, 0.0), gsd=cam.gsd(Z))
    x, y, yaw, _ = poses[-1]
    assert abs(r['x'] - x) < 0.01 and abs(r['y'] - y) < 0.01
    assert abs(np.degrees(r['yaw'] - yaw)) < 0.01


def test_parallax_matches_the_pinhole_formula():
    rng = np.random.default_rng(5)
    cam = SC.stack_camera()
    Z = SC.CAMERA_Z - SC.CELL_TOP
    H, _ = C.calibrate(cam, SC.CELL_TOP, (0.0, 0.0), rng=rng, n=(37, 17))
    k, dz = 11, 0.5
    x0 = SC.nominal_x(k)
    poses = [(SC.nominal_x(i), 0.0, 0.0, 0.0) for i in range(k)] + [(x0, 0.0, 0.0, dz)]
    roi = SC.roi_for(cam, x0, 0.0, half=(25, 85))
    img, off = render(cam, SC.stack_scene(poses), roi=roi, rng=rng)
    r = Lo.locate(img, off, H, SC.CELL[:2], near_pose=(x0, 0.0, 0.0), gsd=cam.gsd(Z))
    predicted = (x0 - cam.t[0]) * dz / (Z - dz)
    assert abs((r['x'] - x0) - predicted) < 0.01

"""Accuracy of the two S1 cameras, measured against the truth they were rendered from. (task 3.2)

    python -m cellvision.study [--n 200] [--out data/cellvision]      # system or venv Python; ~6 min

A  Infeed (locating for the pick). One cell on the infeed pad at a random pose round the pick
   point: x, y uniform +-5 mm, yaw uniform +-3 deg (A: how loosely an infeed conveyor and its
   side guides present a cell), can-top height uniform +-0.3 mm (A), sizes +-0.1 mm (A).
   Located from scratch (blob, then edges). Error = located - true.
B  Stack (measuring the placement). A random number of cells 1..12 already placed after end
   plate A, each at y ~ N(0, 0.15 mm), yaw ~ N(0, 0.15 deg), x = nominal + N(0, 0.03 mm), top
   height +-0.3 mm (A). The last cell is measured from its nominal pose (its neighbours and the
   pads are in the image). Error = measured - true.
C  Parallax. One cell, every stack position k = 1..12, top raised dz = 0.5 mm above the
   calibrated plane; the apparent shift against r * dz / (Z - dz) (calib.py docstring), r the
   cell's distance from the optical axis along x.

Calibration for each camera: once, from a rendered dot target on the measurement plane
(calib.calibrate), so its error is in every number. What is NOT modelled (all would add error;
the numbers are the ideal-world floor, as dataset 1 is): lens-distortion residual, uneven
lighting, can-edge rounding and lid print, dust, vibration, thermal drift of the camera mount.
The renderer blurs the pixelated image rather than the continuous one (camera.py), a model
mismatch of up to ~0.03 px (~5 um) for the edge-model fit; both locators run on every image.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from . import calib as C
from . import locate as Lo
from . import scenes as SC
from .camera import render


def stats(err):
    e = np.asarray(err)
    return dict(mean=float(e.mean()), sd=float(e.std(ddof=1)) if len(e) > 1 else 0.0,
                max_abs=float(np.abs(e).max()), n=int(len(e)))


def infeed_study(n, rng):
    cam = SC.infeed_camera()
    Z = SC.CAMERA_Z - SC.CELL_TOP
    H, cal = C.calibrate(cam, SC.CELL_TOP, SC.INFEED, rng=rng)
    roi = SC.roi_for(cam, *SC.INFEED, half=(35, 100))
    ex, ey, eyaw, fails, t_loc = [], [], [], 0, []
    px, py, pyaw = [], [], []
    for _ in range(n):
        x = SC.INFEED[0] + rng.uniform(-5, 5)
        y = SC.INFEED[1] + rng.uniform(-5, 5)
        yaw = np.radians(rng.uniform(-3, 3))
        dz = rng.uniform(-0.3, 0.3)
        size = (SC.CELL[0] + rng.uniform(-0.1, 0.1), SC.CELL[1] + rng.uniform(-0.1, 0.1), SC.CELL[2])
        img, off = render(cam, SC.infeed_scene(x, y, yaw, dz=dz, size=size), roi=roi, rng=rng)
        t = time.perf_counter()
        try:
            r = Lo.locate(img, off, H, SC.CELL[:2], near_xy=np.array(SC.INFEED), gsd=cam.gsd(Z))
            rp = Lo.locate(img, off, H, SC.CELL[:2], near_xy=np.array(SC.INFEED), gsd=cam.gsd(Z), method='peak')
        except Exception:                     # noqa: BLE001 - counted, not hidden
            fails += 1
            continue
        t_loc.append(time.perf_counter() - t)
        ex.append(r['x'] - x)
        ey.append(r['y'] - y)
        eyaw.append(np.degrees(r['yaw'] - yaw))
        px.append(rp['x'] - x)
        py.append(rp['y'] - y)
        pyaw.append(np.degrees(rp['yaw'] - yaw))
    return dict(camera=dict(f_mm=cam.f, height_above_tops_mm=Z, gsd_mm=cam.gsd(Z)), calibration=cal,
                x_mm=stats(ex), y_mm=stats(ey), yaw_deg=stats(eyaw), failures=fails,
                peak=dict(x_mm=stats(px), y_mm=stats(py), yaw_deg=stats(pyaw)),
                locate_ms_median=float(np.median(t_loc) * 1e3))


def stack_cam():
    cam = SC.stack_camera()
    Z = SC.CAMERA_Z - SC.CELL_TOP
    return cam, Z


def stack_study(n, rng):
    cam, Z = stack_cam()
    H, cal = C.calibrate(cam, SC.CELL_TOP, (0.0, 0.0), rng=rng, n=(37, 17))
    ex, ey, eyaw, fails = [], [], [], 0
    px, py, pyaw = [], [], []
    for _ in range(n):
        k = int(rng.integers(1, 13))
        poses = [(SC.nominal_x(i) + rng.normal(0, 0.03), rng.normal(0, 0.15), np.radians(rng.normal(0, 0.15)),
                  rng.uniform(-0.3, 0.3)) for i in range(k)]
        x, y, yaw, _ = poses[-1]
        roi = SC.roi_for(cam, SC.nominal_x(k - 1), 0.0, half=(25, 85))
        img, off = render(cam, SC.stack_scene(poses), roi=roi, rng=rng)
        try:
            r = Lo.locate(img, off, H, SC.CELL[:2], near_pose=(SC.nominal_x(k - 1), 0.0, 0.0), gsd=cam.gsd(Z))
            rp = Lo.locate(img, off, H, SC.CELL[:2], near_pose=(SC.nominal_x(k - 1), 0.0, 0.0), gsd=cam.gsd(Z),
                           method='peak')
        except Exception:                     # noqa: BLE001
            fails += 1
            continue
        ex.append(r['x'] - x)
        ey.append(r['y'] - y)
        eyaw.append(np.degrees(r['yaw'] - yaw))
        px.append(rp['x'] - x)
        py.append(rp['y'] - y)
        pyaw.append(np.degrees(rp['yaw'] - yaw))
    return dict(camera=dict(f_mm=cam.f, height_above_tops_mm=Z, gsd_mm=cam.gsd(Z)), calibration=cal,
                x_mm=stats(ex), y_mm=stats(ey), yaw_deg=stats(eyaw), failures=fails,
                peak=dict(x_mm=stats(px), y_mm=stats(py), yaw_deg=stats(pyaw)))


def parallax_study(rng, dz=0.5):
    cam, Z = stack_cam()
    H, _ = C.calibrate(cam, SC.CELL_TOP, (0.0, 0.0), rng=rng, n=(37, 17))
    rows = []
    for k in range(12):
        x0 = SC.nominal_x(k)
        poses = [(SC.nominal_x(i), 0.0, 0.0, 0.0) for i in range(k)] + [(x0, 0.0, 0.0, dz)]
        roi = SC.roi_for(cam, x0, 0.0, half=(25, 85))
        img, off = render(cam, SC.stack_scene(poses), roi=roi, rng=rng)
        r = Lo.locate(img, off, H, SC.CELL[:2], near_pose=(x0, 0.0, 0.0), gsd=cam.gsd(Z))
        rx = x0 - cam.t[0]
        rows.append(dict(k=k + 1, r_mm=rx, measured_shift_mm=r['x'] - x0, predicted_mm=rx * dz / (Z - dz)))
    return dict(dz_mm=dz, rows=rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--n', type=int, default=200)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', type=Path, default=Path('data/cellvision'))
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    t0 = time.time()
    res = dict(infeed=infeed_study(a.n, rng), stack=stack_study(a.n // 2, rng), parallax=parallax_study(rng))
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / 'study.json').write_text(json.dumps(res, indent=1))
    for name in ('infeed', 'stack'):
        r = res[name]
        print(f'{name}: f {r["camera"]["f_mm"]:.0f} mm, {r["camera"]["gsd_mm"]:.4f} mm/px; calibration check '
              f'{r["calibration"]["check_rms_mm"] * 1e3:.1f} um RMS; {r["x_mm"]["n"]} located, {r["failures"]} failed')
        for label, src in (('edge-model fit', r), ('gradient peak (comparison)', r['peak'])):
            print(f'  {label}:')
            for ax, unit, scale in (('x_mm', 'um', 1e3), ('y_mm', 'um', 1e3), ('yaw_deg', 'mdeg', 1e3)):
                s = src[ax]
                print(f'   {ax[:-3] if ax != "yaw_deg" else "yaw":4s} bias {s["mean"] * scale:+7.1f} {unit}, '
                      f'sd {s["sd"] * scale:6.1f} {unit}, max |e| {s["max_abs"] * scale:6.1f} {unit}')
    print(f'parallax, dz = {res["parallax"]["dz_mm"]} mm (stack camera):')
    for row in res['parallax']['rows']:
        print(f'   cell {row["k"]:2d}  r {row["r_mm"]:+7.1f} mm  shift {row["measured_shift_mm"] * 1e3:+6.1f} um  '
              f'predicted {row["predicted_mm"] * 1e3:+6.1f} um')
    print(f'({time.time() - t0:.0f} s; {a.out / "study.json"})')


if __name__ == '__main__':
    main()

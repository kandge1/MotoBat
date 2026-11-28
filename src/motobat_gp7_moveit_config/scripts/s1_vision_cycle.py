#!/usr/bin/python3
"""Station 1, closed: cells located by the infeed camera, picked and placed by the GP7 through
MoveIt, each placed cell measured by the stack camera. (task 3.2)

Per part, in build order (end plate A, cells 1-12, end plate B):
  1. a cell arrives on the infeed at a random TRUE pose (x, y +-5 mm, yaw +-3 deg; A);
  2. the infeed camera renders it (cellvision) and the locator finds it (edge-model fit);
  3. the GP7 picks at the LOCATED pose: Pilz PTP above it, LIN down, attach, LIN up;
  4. grasp (A): a self-centring parallel gripper closes on the cell's end faces. It re-centres
     the cell along its length and squares its yaw to the jaws (residuals N(0, 0.02 mm) and
     N(0, 0.05 deg)); across its thickness the cell keeps whatever error the pick had;
  5. the GP7 places at the nominal cell position: PTP above, LIN down, detach, LIN up.
     Where the cell really lands = the commanded TCP pose, composed with a constant TCP
     calibration error (A: +0.05 mm along the line, +0.15 mm across, +0.10 deg: the kind of
     offset a cell-placement loop exists to remove) and robot repeatability (S: GP7 +-0.03 mm,
     Yaskawa DS-699-H; ISO 9283 RP = mean + 3 sigma, so N(0, 0.01 mm) per axis);
  6. the stack camera renders the module as it really is and measures the cell: e_k =
     measured - nominal, the error 3.4's loop will correct. Gauge error = measured - true.
End plates come from a precision nest (A): no camera, picked at their nominal pose.

Logged per cell (data/s1_cycle/cycle.json): infeed true vs located, placed true, measured, the
three error terms, planned motion time. The camera pictures are taken while the arm is away
(s1_view_check.py proves the GP7 is out of each view at those poses), so they cost no cycle time.

Not modelled: contact in the stack (a real cell is pushed against its neighbour; here it lands
where the robot puts it and may overlap a pad by the placement error), gripper open/close time,
the camera exposure and processing time (~50 ms locate here).

    ros2 launch motobat_gp7_moveit_config demo.launch.py use_rviz:=false
    ros2 run motobat_gp7_moveit_config s1_vision_cycle.py [--seed 0] [--out data/s1_cycle]
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[2]))                   # repo root: cellvision
import placer_check as P  # noqa: E402
from cellvision import calib as CC  # noqa: E402
from cellvision import locate as Lo  # noqa: E402
from cellvision import scenes as SC  # noqa: E402
from cellvision.camera import render  # noqa: E402

TCP_ERROR = (0.05, 0.15, math.radians(0.10))   # mm, mm, rad (A): constant, in the world frame
REPEAT_SD = 0.01                               # mm per axis (S-derived: GP7 +-0.03 mm)
JAW_SD, JAW_YAW_SD = 0.02, math.radians(0.05)  # A
INFEED_SPREAD, INFEED_YAW = 5.0, math.radians(3.0)


def compose(a, b):
    """2-D pose composition (x, y, yaw) a then b (b expressed in a's frame)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', type=Path, default=Path('data/s1_cycle'))
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    rclpy.init()
    n = P.Check()
    x0, d = P.station()
    if abs(x0) > 1e-9:
        sys.exit('cellvision scenes assume the S1 stop at world x = 0')
    top = P.LC.PALLET_TOP

    cam_in, cam_st = SC.infeed_camera(), SC.stack_camera()
    Z = SC.CAMERA_Z - SC.CELL_TOP
    H_in, cal_in = CC.calibrate(cam_in, SC.CELL_TOP, SC.INFEED, rng=rng)
    H_st, cal_st = CC.calibrate(cam_st, SC.CELL_TOP, (0.0, 0.0), rng=rng, n=(37, 17))
    print(f'calibrated: infeed {cal_in["check_rms_mm"] * 1e3:.1f} um, stack {cal_st["check_rms_mm"] * 1e3:.1f} um RMS')

    n.apply(world=[P.box('pallet', (0.405, 0.201, 0.020), (x0, 0.0, top - 0.010)),
                   P.box('cooling_plate', (0.355, 0.151, 0.004), (x0, 0.0, top + 0.002))])
    up = P.LC.ASSUMED['approach']
    stack_true = []                       # placed cells, world mm: (x, y, yaw, dz)
    log, total, ok = [], 0.0, True
    for name, (tx, ty, tz), _, part, _ in P.LC.tasks('gp7'):
        size = P.LC.PARTS[part]
        mx, my, mz = P.LC.ASSUMED['magazine'][part]
        pick_nom = (x0 + mx, -d + my, top + mz)
        rec = dict(part=name)
        if part == 'cell':
            # 1-2. arrival and locating
            true_in = (SC.INFEED[0] + rng.uniform(-INFEED_SPREAD, INFEED_SPREAD),
                       SC.INFEED[1] + rng.uniform(-INFEED_SPREAD, INFEED_SPREAD),
                       rng.uniform(-INFEED_YAW, INFEED_YAW))
            roi = SC.roi_for(cam_in, *SC.INFEED, half=(35, 100))
            img, off = render(cam_in, SC.infeed_scene(*true_in), roi=roi, rng=rng)
            loc = Lo.locate(img, off, H_in, SC.CELL[:2], near_xy=np.array(SC.INFEED), gsd=cam_in.gsd(Z))
            located = (loc['x'], loc['y'], loc['yaw'])
            rec.update(infeed_true=true_in, infeed_located=located,
                       vision_err_um=[(located[0] - true_in[0]) * 1e3, (located[1] - true_in[1]) * 1e3],
                       vision_err_mdeg=math.degrees(located[2] - true_in[2]) * 1e3)
            pick = (located[0] / 1e3, located[1] / 1e3, top + mz)
            pick_yaw = math.pi + located[2]
            # the cell on the infeed, for collision checking (1 mm short at the top)
            n.apply(world=[P.placed_box('infeed_cell', size, pick)])
        else:
            pick, pick_yaw = pick_nom, math.pi
        place = (x0 + tx, ty, top + tz)
        above = lambda p: (p[0], p[1], p[2] + up)  # noqa: E731
        # The gripper is symmetric: yaw and yaw + 180 deg hold the part the same. Take the one with a
        # collision-free solution at both the approach and the target, preferring no wrist turn.
        def yaw_for(p, preferred):
            seed = n.joints()
            for yaw in (preferred, preferred - math.pi):
                if n.solve(above(p), [yaw], seed)[0] is not None and n.solve(p, [yaw], seed)[0] is not None:
                    return yaw
            return preferred
        pick_yaw = yaw_for(pick, pick_yaw)
        place_yaw = yaw_for(place, math.pi)
        steps = [('PTP', above(pick), pick_yaw, None), ('LIN', pick, pick_yaw, 'attach'),
                 ('LIN', above(pick), pick_yaw, None), ('PTP', above(place), place_yaw, None),
                 ('LIN', place, place_yaw, 'detach'), ('LIN', above(place), place_yaw, None)]
        t_part = 0.0
        for planner, p, yaw, action in steps:
            good, dur, err, bad = n.go(p, yaw, planner)
            if not good and planner == 'PTP':
                # The straight joint-space move (MOVJ) hits something on the way, e.g. the forearm
                # rising into the camera beam over the pallet: plan round it instead (OMPL), as
                # view_reach.py's planner does for the HC10DT. Counted, so the cost is visible.
                good, dur, err, bad = n.go(p, yaw, 'RRTConnectkConfigDefault', pipeline='ompl')
                rec.setdefault('ompl_moves', 0)
                rec['ompl_moves'] += 1
                print(f'  {name}: PTP to {np.round(p, 3)} blocked, OMPL instead: {"ok" if good else "FAILED"} ({dur:.2f} s)')
            t_part += dur
            if not good:
                ok = False
                print(f'  {name}: {planner} to {np.round(p, 4)} FAILED (end error {err:.3f} mm, {bad} invalid states)')
            if action == 'attach':
                if part == 'cell':
                    n.apply(remove=['infeed_cell'])
                n.apply(attach=n.attached(name, size))
            if action == 'detach':
                n.apply(detach=name)
                n.apply(world=[P.placed_box(name, size, place)])
        total += t_part
        rec['motion_s'] = t_part
        if part == 'cell':
            # 4. grasp: the cell's pose in the TCP frame (TCP frame: x along the line at yaw pi...)
            #    expressed as the cell's error relative to the gripper's intended pose
            g_across = math.cos(true_in[2]) * (true_in[0] - located[0]) + math.sin(true_in[2]) * (true_in[1] - located[1])
            grasp = (g_across, rng.normal(0, JAW_SD), rng.normal(0, JAW_YAW_SD))   # cell frame: x across, y along
            # 5. where it lands: nominal place, TCP error, grasp offset, repeatability
            nominal = (place[0] * 1e3, place[1] * 1e3, 0.0)
            landed = compose(compose((nominal[0] + TCP_ERROR[0], nominal[1] + TCP_ERROR[1], TCP_ERROR[2]), grasp),
                             (rng.normal(0, REPEAT_SD), rng.normal(0, REPEAT_SD), 0.0))
            dz = 0.0
            stack_true.append((landed[0], landed[1], landed[2], dz))
            # 6. the stack camera measures it
            k = len(stack_true) - 1
            roi = SC.roi_for(cam_st, SC.nominal_x(k), 0.0, half=(25, 85))
            img, off = render(cam_st, SC.stack_scene(stack_true), roi=roi, rng=rng)
            m = Lo.locate(img, off, H_st, SC.CELL[:2], near_pose=(SC.nominal_x(k), 0.0, 0.0), gsd=cam_st.gsd(Z))
            rec.update(placed_true=landed, measured=(m['x'], m['y'], m['yaw']),
                       placement_err_um=[(landed[0] - nominal[0]) * 1e3, (landed[1] - nominal[1]) * 1e3],
                       placement_err_mdeg=math.degrees(landed[2]) * 1e3,
                       measured_e_um=[(m['x'] - nominal[0]) * 1e3, (m['y'] - nominal[1]) * 1e3],
                       measured_e_mdeg=math.degrees(m['yaw']) * 1e3,
                       gauge_err_um=[(m['x'] - landed[0]) * 1e3, (m['y'] - landed[1]) * 1e3])
            print(f'  {name:7s} vision {rec["vision_err_um"][0]:+5.1f} {rec["vision_err_um"][1]:+5.1f} um | placed '
                  f'{rec["placement_err_um"][0]:+6.1f} {rec["placement_err_um"][1]:+6.1f} um {rec["placement_err_mdeg"]:+6.1f} mdeg | '
                  f'measured e {rec["measured_e_um"][0]:+6.1f} {rec["measured_e_um"][1]:+6.1f} um | gauge '
                  f'{rec["gauge_err_um"][0]:+5.1f} {rec["gauge_err_um"][1]:+5.1f} um | {t_part:.2f} s')
        else:
            print(f'  {name:7s} (precision nest, no camera) | {t_part:.2f} s')
        log.append(rec)
    # home
    g = P.MoveGroup.Goal()
    r = g.request
    r.group_name, r.pipeline_id, r.planner_id = P.GROUP, 'pilz_industrial_motion_planner', 'PTP'
    r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = 1.0
    r.goal_constraints = [P.Constraints(joint_constraints=[
        P.JointConstraint(joint_name=j, position=v, tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0)
        for j, v in zip(P.JOINTS, P.HOME)])]
    fut = n.move.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut)
    rf = fut.result().get_result_async()
    rclpy.spin_until_future_complete(n, rf)
    cells = [rec for rec in log if 'measured' in rec]
    arr = lambda k: np.array([rec[k] for rec in cells])  # noqa: E731
    summary = dict(parts=len(log), motion_s=total, all_moves_ok=ok,
                   ompl_moves=int(sum(rec.get('ompl_moves', 0) for rec in log)),
                   vision_um_sd=arr('vision_err_um').std(0, ddof=1).tolist(),
                   placement_um_mean=arr('placement_err_um').mean(0).tolist(),
                   placement_um_sd=arr('placement_err_um').std(0, ddof=1).tolist(),
                   gauge_um_mean=arr('gauge_err_um').mean(0).tolist(),
                   gauge_um_sd=arr('gauge_err_um').std(0, ddof=1).tolist(),
                   calibration=dict(infeed=cal_in, stack=cal_st),
                   assumptions=dict(tcp_error_mm_mm_rad=TCP_ERROR, repeat_sd_mm=REPEAT_SD,
                                    jaw_sd_mm=JAW_SD, jaw_yaw_sd_rad=JAW_YAW_SD))
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / 'cycle.json').write_text(json.dumps(dict(summary=summary, parts=log), indent=1))
    print(f'\n{len(log)} parts, {total:.1f} s of motion ({total / len(log):.2f} s per part), moves '
          f'{"all ok" if ok else "SOME FAILED"}, {summary["ompl_moves"]} planned round an obstacle (OMPL)')
    print(f'cells: vision sd x/y {summary["vision_um_sd"][0]:.1f} / {summary["vision_um_sd"][1]:.1f} um; '
          f'placement mean {summary["placement_um_mean"][0]:+.0f} / {summary["placement_um_mean"][1]:+.0f} um '
          f'(sd {summary["placement_um_sd"][0]:.0f} / {summary["placement_um_sd"][1]:.0f}); '
          f'gauge mean {summary["gauge_um_mean"][0]:+.1f} / {summary["gauge_um_mean"][1]:+.1f} um '
          f'(sd {summary["gauge_um_sd"][0]:.1f} / {summary["gauge_um_sd"][1]:.1f})')
    print(f'-> {a.out / "cycle.json"}')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

#!/usr/bin/python3
"""The S1 cell-placement correction loop as a ROS node: CorrectionCmd client. (task 3.4)

For modules m = 1..M and cells k = 1..12 it sends the S1 station "place cell k with offset u_k",
waits for the measured error e_k, and computes the next offset with the I-form law
(loop_controller.ILoop: u_{k+1} = u_k - K e_k, per axis). k runs across modules: a gripper's
error does not reset with the pallet. A cell the camera could not measure (NaN) leaves u as it is.

Event-driven (task 3.6): the measured error comes from the station's /s1/defect_report (stage
s1_placement); the node updates u the moment a report arrives and publishes it on /s1/correction
(geometry_msgs/Vector3Stamped: x, y in mm, z = yaw in deg; header.stamp = the report's CAPTURE stamp,
so a bag shows capture -> report -> correction -> next goal). Before sending cell k+1 it waits for
cell k's report to have been applied. The action result is logged, not used for control.

Parameters: modules (3), gains [Kx, Ky, Kyaw] ([0.5, 0.5, 0.5]; [0, 0, 0] = open loop),
u_max [mm, mm, deg] ([0.5, 0.5, 0.5]), label (run name), out (data/loop).
Writes <out>/<label>.json: per cell the command, the measured error, the station's truth
(simulation only) and timing; prints a summary against the +-0.20 mm / +-0.10 deg tolerance (A).

    ros2 launch motobat_coordination s1_loop.launch.py gains:="[0.5,0.5,0.5]" label:=closed_k05
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.action import ActionClient
from rclpy.node import Node

from motobat_interfaces.action import CorrectionCmd
from motobat_interfaces.msg import DefectReport

sys.path.insert(0, str(Path(__file__).resolve().parent))
from loop_controller import ILoop  # noqa: E402

POS_TOL, YAW_TOL = 0.20, 0.10          # mm, deg (A: detect.yaml cell.pos_tol, cell.yaw_tol)


class CorrectionNode(Node):
    def __init__(self):
        super().__init__('s1_correction')
        self.declare_parameter('modules', 3)
        self.declare_parameter('gains', [0.5, 0.5, 0.5])
        self.declare_parameter('u_max', [0.5, 0.5, 0.5])
        self.declare_parameter('label', 'run')
        self.declare_parameter('out', 'data/loop')
        self.client = ActionClient(self, CorrectionCmd, 'correction_cmd')
        self.loop = None
        self.reported = {}                                    # (module, cell) -> e (mm, mm, deg)
        self.create_subscription(DefectReport, '/s1/defect_report', self.on_report, 10)
        self.pub = self.create_publisher(Vector3Stamped, '/s1/correction', 10)

    def on_report(self, r):
        if r.stage != DefectReport.S1_PLACEMENT or not r.measurements:
            return
        m = r.measurements[0]
        key = (r.module_id, int(m.id))
        if m.seen:
            v = {kv.key: kv.value for kv in m.values}
            e = np.array([v['dx'], v['dy'], v['dyaw']])
            self.loop.update(e)
        else:
            e = np.full(3, np.nan)                            # not measured: u stays as it is
        out = Vector3Stamped()
        out.header.stamp = r.header.stamp                     # the capture this correction answers
        out.header.frame_id = f'{r.module_id}/cell{m.id}'
        out.vector.x, out.vector.y, out.vector.z = (float(x) for x in self.loop.u)
        self.pub.publish(out)
        self.reported[key] = e

    def wait_report(self, key, timeout=30.0):
        end = time.time() + timeout
        while key not in self.reported and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.reported.get(key)

    def place(self, module_id, k, u):
        goal = CorrectionCmd.Goal(module_id=module_id, cell_index=k, based_on_cell=max(k - 1, 0),
                                  dx=float(u[0]) / 1e3, dy=float(u[1]) / 1e3, dyaw=math.radians(float(u[2])))
        t0 = time.time()
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        res = rf.result().result
        e = np.array([res.measured_error[0] * 1e3, res.measured_error[1] * 1e3, math.degrees(res.measured_error[2])])
        return res, e, time.time() - t0


def summary(rows, skip):
    """Placement error (truth) of the cells after `skip`, against the tolerance."""
    t = np.array([r['truth'] for r in rows[skip:]])
    if not len(t):
        return {}
    out = (np.hypot(t[:, 0], t[:, 1]) > POS_TOL) | (np.abs(t[:, 2]) > YAW_TOL)
    return dict(cells=int(len(t)), mean=t.mean(0).round(4).tolist(), sd=t.std(0, ddof=1).round(4).tolist(),
                out_of_tol=int(out.sum()))


def main():
    rclpy.init()
    n = CorrectionNode()
    if not n.client.wait_for_server(timeout_sec=120):
        sys.exit('no correction_cmd action server (is s1_station running?)')
    gp = lambda name: n.get_parameter(name).value  # noqa: E731
    gains, u_max = list(gp('gains')), list(gp('u_max'))
    loop = ILoop(gains, u_max)                       # units: mm, mm, deg
    n.loop = loop
    rows = []
    for m in range(1, int(gp('modules')) + 1):
        module_id = f'{gp("label")}-m{m}'
        for k in range(1, 13):
            u = loop.u.copy()
            res, e, dt = n.place(module_id, k, u)
            info = json.loads(res.message)
            truth = [info['truth_mm_mm_rad'][0], info['truth_mm_mm_rad'][1], math.degrees(info['truth_mm_mm_rad'][2])]
            if n.wait_report((module_id, k)) is None:          # control runs off the report, not the result
                n.get_logger().warn(f'{module_id} cell {k}: no DefectReport within 30 s; u unchanged')
            rows.append(dict(module=module_id, cell=k, u=u.round(4).tolist(), e=e.round(4).tolist(), truth=truth,
                             placed=res.placed, motion_s=info['motion_s'], round_trip_s=dt))
            n.get_logger().info(f'{module_id} cell {k:2d}: u {np.round(u, 3)}  e {np.round(e, 3)}  '
                                f'truth {np.round(truth, 3)}  ({dt:.1f} s)')
    s_all, s_after = summary(rows, 0), summary(rows, 6)
    out = Path(gp('out'))
    out.mkdir(parents=True, exist_ok=True)
    (out / f'{gp("label")}.json').write_text(json.dumps(dict(gains=gains, u_max=u_max, rows=rows,
                                                             summary_all=s_all, summary_after_6=s_after), indent=1))
    n.get_logger().info(f'{gp("label")}: gains {gains}; all cells {s_all}; after the first 6 {s_after}')
    print(json.dumps(dict(label=gp('label'), gains=gains, all=s_all, after_6=s_after)))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

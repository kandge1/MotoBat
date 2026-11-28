#!/usr/bin/python3
"""Station 1 as a CorrectionCmd action server: place cell k with an offset, return its measured
error. (task 3.4)

    goal    module_id, cell_index (1..12), dx, dy, dyaw (m, rad: the correction loop's u_k)
    result  placed, measured_error = e_k (m, rad), message = JSON with the simulation's truth
    feedback picking -> placing -> measuring
    also publishes  /s1/defect_report  (motobat_interfaces/DefectReport, stage s1_placement)

For each goal, the station (all of it simulated, all of it in this node so the loop only ever sees
what a real station would report):
  1. the cell arrives on the infeed at a random pose, with a can height dz off nominal (+-0.3 mm, A);
  2. the infeed point laser reads that height (sigma 5 um, A); the infeed camera locates the cell;
  3. the GP7 (MoveIt, Pilz; OMPL if a straight joint move is blocked) picks it at the located pose
     and places it at nominal + (dx, dy, dyaw);
  4. where it really lands: the command composed with the gripper's TCP calibration error (A,
     constant: what the loop exists to remove), the grasp (self-centring jaws, A) and the GP7's
     repeatability (S: +-0.03 mm); it seats with dz + N(0, 20 um) (A) above nominal;
  5. the stack camera measures it; the parallax of its raised top is removed with the laser's dz
     (cellvision.locate.parallax_correct). e_k = corrected measurement - nominal.
A new module_id starts a new module: the placed cells are cleared and end plate A is in place
(end plates are not executed in loop runs). The gripper's errors carry across modules, as on a
real line (k runs across modules).

Parameters (ros2 param): seed, tcp_error [mm, mm, deg], dz_spread, laser_sd, seat_sd (mm);
image_mode 'roi' | 'full' (the stack camera's image on /s1/stack_camera/image: the measured window, or
a whole 2448 x 2048 mono16 frame, 10 MB, the window pasted in), image_reliable (QoS).

Latency (task 3.6): the stack image and its DefectReport carry the CAPTURE time in header.stamp:
the moment the image exists (rendering is simulation cost, not a camera's), so stamp -> receipt is
processing + transport, measured in rosbag2.
"""
import json
import math
import sys
import threading
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from motobat_interfaces.action import CorrectionCmd
from motobat_interfaces.msg import DefectReport, KeyValue, Measurement

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'motobat_gp7_moveit_config' / 'scripts'))
sys.path.insert(0, str(HERE.parents[2]))                     # repo root: cellvision
import placer_check as P  # noqa: E402
from cellvision import calib as CC  # noqa: E402
from cellvision import locate as Lo  # noqa: E402
from cellvision import scenes as SC  # noqa: E402
from cellvision.camera import render  # noqa: E402

JAW_SD, JAW_YAW_SD = 0.02, math.radians(0.05)   # mm, rad (A)
REPEAT_SD = 0.01                                # mm per axis (S-derived: GP7 +-0.03 mm, ISO 9283)
INFEED_SPREAD, INFEED_YAW = 5.0, math.radians(3.0)
POS_TOL, YAW_TOL = 0.20, 0.10                   # mm, deg (A, detect.yaml cell.pos_tol / yaw_tol)


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


class S1Station(Node):
    def __init__(self):
        super().__init__('s1_station')
        self.declare_parameter('seed', 0)
        self.declare_parameter('tcp_error', [0.05, 0.15, 0.10])     # mm, mm, deg (A)
        self.declare_parameter('dz_spread', 0.3)                    # mm (A)
        self.declare_parameter('laser_sd', 0.005)                   # mm (A)
        self.declare_parameter('seat_sd', 0.02)                     # mm (A)
        self.declare_parameter('image_mode', 'roi')
        self.declare_parameter('image_reliable', True)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.rng = np.random.default_rng(g('seed'))
        te = g('tcp_error')
        self.tcp_error = (te[0], te[1], math.radians(te[2]))
        self.dz_spread, self.laser_sd, self.seat_sd = g('dz_spread'), g('laser_sd'), g('seat_sd')

        self.mv = P.Check(name='s1_station_moveit')                 # spun by hand, never by the executor
        self.x0, self.d = P.station()
        self.top = P.LC.PALLET_TOP
        self.cam_in, self.cam_st = SC.infeed_camera(), SC.stack_camera()
        self.Z = SC.CAMERA_Z - SC.CELL_TOP
        self.H_in, _ = CC.calibrate(self.cam_in, SC.CELL_TOP, SC.INFEED, rng=self.rng)
        self.H_st, _ = CC.calibrate(self.cam_st, SC.CELL_TOP, (0.0, 0.0), rng=self.rng, n=(37, 17))
        self.mv.apply(world=[P.box('pallet', (0.405, 0.201, 0.020), (self.x0, 0.0, self.top - 0.010)),
                             P.box('cooling_plate', (0.355, 0.151, 0.004), (self.x0, 0.0, self.top + 0.002))])
        self.module, self.stack, self.placed = None, [], []
        self.lock = threading.Lock()
        self.pub = self.create_publisher(DefectReport, '/s1/defect_report', 10)
        self.image_mode = g('image_mode')
        rel = ReliabilityPolicy.RELIABLE if g('image_reliable') else ReliabilityPolicy.BEST_EFFORT
        self.img_pub = self.create_publisher(Image, '/s1/stack_camera/image', QoSProfile(depth=5, reliability=rel))
        self.server = ActionServer(self, CorrectionCmd, 'correction_cmd', self.execute,
                                   callback_group=ReentrantCallbackGroup())
        self.get_logger().info('S1 station ready: cameras calibrated, scene loaded')

    # --- one module ---------------------------------------------------------------------------
    def new_module(self, module_id):
        if self.placed:
            self.mv.apply(remove=self.placed)
        self.placed, self.stack, self.module = [], [], module_id
        ep = P.LC.PARTS['end_plate']
        self.mv.apply(world=[P.placed_box('end_plate_A', ep, (self.x0 - P.LC.END_PLATE_X, 0.0, self.top + 0.105))])
        self.placed.append('end_plate_A')

    def move(self, p, yaw, planner):
        good, dur, err, bad = self.mv.go(p, yaw, planner)
        if not good and planner == 'PTP':
            good, dur, err, bad = self.mv.go(p, yaw, 'RRTConnectkConfigDefault', pipeline='ompl')
        return good, dur

    def yaw_for(self, p, preferred, up):
        seed = self.mv.joints()
        for yaw in (preferred, preferred - math.pi):
            if self.mv.solve((p[0], p[1], p[2] + up), [yaw], seed)[0] is not None and \
                    self.mv.solve(p, [yaw], seed)[0] is not None:
                return yaw
        return preferred

    def execute(self, goal_handle):
        with self.lock:
            return self._execute(goal_handle)

    def _execute(self, goal_handle):
        g = goal_handle.request
        fb = CorrectionCmd.Feedback()
        if g.module_id != self.module:
            self.new_module(g.module_id)
        k = int(g.cell_index) - 1
        size = P.LC.PARTS['cell']
        up = P.LC.ASSUMED['approach']
        above = lambda p: (p[0], p[1], p[2] + up)  # noqa: E731
        # 1-2. arrival, height, locating
        fb.phase = fb.PICKING
        goal_handle.publish_feedback(fb)
        dz = self.rng.uniform(-self.dz_spread, self.dz_spread)
        dz_meas = dz + self.rng.normal(0, self.laser_sd)
        true_in = (SC.INFEED[0] + self.rng.uniform(-INFEED_SPREAD, INFEED_SPREAD),
                   SC.INFEED[1] + self.rng.uniform(-INFEED_SPREAD, INFEED_SPREAD),
                   self.rng.uniform(-INFEED_YAW, INFEED_YAW))
        roi = SC.roi_for(self.cam_in, *SC.INFEED, half=(35, 100))
        img, off = render(self.cam_in, SC.infeed_scene(*true_in, dz=dz), roi=roi, rng=self.rng)
        loc = Lo.parallax_correct(Lo.locate(img, off, self.H_in, SC.CELL[:2], near_xy=np.array(SC.INFEED),
                                            gsd=self.cam_in.gsd(self.Z)), self.cam_in, SC.CELL_TOP, dz_meas)
        located = (loc['x'], loc['y'], loc['yaw'])
        mz = P.LC.ASSUMED['magazine']['cell'][2]
        pick = (located[0] / 1e3, located[1] / 1e3, self.top + mz)
        self.mv.apply(world=[P.placed_box('infeed_cell', size, pick)])
        # 3. pick, then place at nominal + u
        nominal = (self.x0 * 1e3 + SC.nominal_x(k), 0.0, 0.0)
        cmd = (nominal[0] + g.dx * 1e3, nominal[1] + g.dy * 1e3, g.dyaw)
        place = (cmd[0] / 1e3, cmd[1] / 1e3, self.top + 0.095)
        pick_yaw = self.yaw_for(pick, math.pi + located[2], up)
        place_yaw = self.yaw_for(place, math.pi + cmd[2], up)
        moves, ok = 0.0, True
        for planner, p, yaw, action in (('PTP', above(pick), pick_yaw, None), ('LIN', pick, pick_yaw, 'attach'),
                                        ('LIN', above(pick), pick_yaw, None), ('PTP', above(place), place_yaw, 'placing'),
                                        ('LIN', place, place_yaw, 'detach'), ('LIN', above(place), place_yaw, None)):
            if action == 'placing':
                fb.phase = fb.PLACING
                goal_handle.publish_feedback(fb)
            good, dur = self.move(p, yaw, planner)
            ok &= good
            moves += dur
            name = f'cell_{k + 1}'
            if action == 'attach':
                self.mv.apply(remove=['infeed_cell'])
                self.mv.apply(attach=self.mv.attached(name, size))
            if action == 'detach':
                self.mv.apply(detach=name)
                self.mv.apply(world=[P.placed_box(name, size, place)])
                self.placed.append(name)
        # 4. where it really lands
        g_across = math.cos(true_in[2]) * (true_in[0] - located[0]) + math.sin(true_in[2]) * (true_in[1] - located[1])
        grasp = (g_across, self.rng.normal(0, JAW_SD), self.rng.normal(0, JAW_YAW_SD))
        te = self.tcp_error
        landed = compose(compose((cmd[0] + te[0], cmd[1] + te[1], cmd[2] + te[2]), grasp),
                         (self.rng.normal(0, REPEAT_SD), self.rng.normal(0, REPEAT_SD), 0.0))
        dz_seat = dz + self.rng.normal(0, self.seat_sd)
        self.stack.append((landed[0] - self.x0 * 1e3, landed[1], landed[2], dz_seat))
        # 5. measure
        fb.phase = fb.MEASURING
        goal_handle.publish_feedback(fb)
        roi = SC.roi_for(self.cam_st, SC.nominal_x(k), 0.0, half=(25, 85))
        img, off = render(self.cam_st, SC.stack_scene(self.stack), roi=roi, rng=self.rng)
        stamp = self.get_clock().now().to_msg()                     # capture: the image exists now
        self.publish_image(img, off, stamp)
        try:
            m = Lo.locate(img, off, self.H_st, SC.CELL[:2], near_pose=(SC.nominal_x(k), 0.0, 0.0),
                          gsd=self.cam_st.gsd(self.Z))
            m = Lo.parallax_correct(m, self.cam_st, SC.CELL_TOP, dz_meas)
            e = (m['x'] - SC.nominal_x(k), m['y'], m['yaw'])          # mm, mm, rad
            seen = True
        except ValueError as ex:
            self.get_logger().warn(f'cell {k + 1}: stack camera could not measure it ({ex})')
            e, seen = (float('nan'),) * 3, False
        truth = (landed[0] - nominal[0], landed[1] - nominal[1], landed[2])
        self.publish_report(k, e, seen, stamp)
        res = CorrectionCmd.Result()
        res.placed = bool(ok)
        res.measured_error = [e[0] / 1e3, e[1] / 1e3, e[2]]
        res.message = json.dumps(dict(module=self.module, cell=k + 1, motion_s=moves, dz_mm=dz, dz_laser_mm=dz_meas,
                                      truth_mm_mm_rad=truth, located_infeed=located, infeed_true=true_in))
        goal_handle.succeed()
        return res

    def publish_image(self, img, off, stamp):
        if self.image_mode == 'full':
            full = np.zeros((self.cam_st.height, self.cam_st.width), np.uint16)
            full[off[1]:off[1] + img.shape[0], off[0]:off[0] + img.shape[1]] = img
            img = full
        m = Image(height=int(img.shape[0]), width=int(img.shape[1]), encoding='mono16', is_bigendian=0,
                  step=int(img.shape[1] * 2), data=img.astype('<u2').tobytes())
        m.header.stamp, m.header.frame_id = stamp, 's1_cam_stack_optical'
        self.img_pub.publish(m)

    def publish_report(self, k, e, seen, stamp):
        r = DefectReport()
        r.header.stamp = stamp                                      # capture time (task 3.6)
        r.header.frame_id = 'pallet_datum'
        r.module_id, r.stage = self.module, DefectReport.S1_PLACEMENT
        out = seen and (math.hypot(e[0], e[1]) > POS_TOL or abs(math.degrees(e[2])) > YAW_TOL)
        r.result = DefectReport.INCOMPLETE if not seen else (DefectReport.NOK if out else DefectReport.OK)
        mm = Measurement(kind='cells', id=str(k + 1), capture='s1_stack', seen=bool(seen))
        if seen:
            mm.values = [KeyValue(key='dx', value=float(e[0]), unit='mm'), KeyValue(key='dy', value=float(e[1]), unit='mm'),
                         KeyValue(key='dyaw', value=float(math.degrees(e[2])), unit='deg')]
        r.measurements = [mm]
        self.pub.publish(r)


def main():
    rclpy.init()
    node = S1Station()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

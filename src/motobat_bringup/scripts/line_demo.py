#!/usr/bin/python3
"""
One module through the inspection end of the line, in Gazebo (task 2.6).

    ros2 launch motobat_bringup inspector_gazebo.launch.py world:=line          # terminal 1
    /usr/bin/python3 src/motobat_bringup/scripts/line_demo.py --stage post_weld  # terminal 2

  1. Fixed pass: the pallet runs under the line-scan gate at 100 mm/s (defectgen's fixed pass:
     0.1 mm x 1 kHz) and every gate profile is placed in the pallet's frame by where the pallet
     was when it was taken -> /fixed/cloud.
  2. The pallet indexes to the Inspector (pallet_x = 0) and stops.
  3. The HC10DT flies the stage's plan from 2.5 (data/reach/base_A/reach.json: the order, the
     timed moves, each view's joint path), sent as one FollowJointTrajectory. Every LJ-X profile
     taken during a sweep is placed by TF at its timestamp, and the Zivid cloud taken at each
     snapshot pose is added -> /arm/cloud.
  4. Prints the measured times (sim clock) against the plan.

Both clouds are in `pallet_datum`, a TF frame this node broadcasts on the moving pallet (the
module frame of defectgen, in metres), so they line up with each other and with the module.
/module (MarkerArray) draws the module meshes there for RViz.

Gazebo's sensors are geometric stand-ins (inspector_sensors.gazebo.xacro): this shows motion,
timing and data flow, not measurement quality (plan §3.7.8).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration as RosDuration
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64, Header
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

JOINTS = ['s3_joint_1_s', 's3_joint_2_l', 's3_joint_3_u', 's3_joint_4_r', 's3_joint_5_b', 's3_joint_6_t']
S3_X, LINE_Y, PALLET_TOP = 1.8, 0.0, 0.9      # line_layout.xacro
GATE_X = 0.6                                   # inspection_line.sdf
LINE_SPEED = 0.10                              # m/s, defectgen's fixed pass
MODULE_HALF = 0.18                             # m, half the module length plus a little
SNAP_DWELL = 0.6                               # s at a snapshot pose in Gazebo (A: the 5 Hz depth
                                               #   camera needs a frame; the plan budgets 0.1 s)
ROOT = Path(os.environ.get('MOTOBAT_ROOT', Path(__file__).resolve().parents[3]))


def scan_points(msg: LaserScan) -> np.ndarray:
    r = np.asarray(msg.ranges, float)
    a = msg.angle_min + msg.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())])


def tf_matrix(t) -> np.ndarray:
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


class Demo(Node):
    def __init__(self, voxel):
        super().__init__('line_demo', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.voxel = voxel
        self.tf = Buffer(cache_time=RosDuration(seconds=900))   # scans are placed after the flight
        self.raw = []                          # (stamp, frame, points) taken while flying
        self.tfl = TransformListener(self.tf, self)
        self.tfb = TransformBroadcaster(self)
        self.pallet_x = None
        self.pallet_hist = []                  # (stamp s, x)
        self.collect = None                    # 'fixed' | 'arm' | None
        self.windows = []                      # (t0, t1, kind) in sim seconds, while flying
        self.clouds = {'fixed': [], 'arm': []}
        self.latest_zivid = None
        self.cmd = self.create_publisher(Float64, '/pallet/x', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = {k: self.create_publisher(PointCloud2, f'/{k}/cloud', latched) for k in self.clouds}
        self.markers = self.create_publisher(MarkerArray, '/module', latched)
        self.create_subscription(JointState, '/pallet/joint_states', self.on_pallet, 10)
        self.create_subscription(LaserScan, '/ranger3/scan', self.on_ranger, 50)
        self.create_subscription(LaserScan, '/ljx/scan', self.on_ljx, 200)
        self.create_subscription(PointCloud2, '/zivid/points', self.on_zivid, 2)
        self.arm = ActionClient(self, FollowJointTrajectory, '/inspector_arm_controller/follow_joint_trajectory')
        self.create_timer(0.02, self.broadcast)
        self.joint_now = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

    # -- pallet ---------------------------------------------------------------------------
    def on_pallet(self, msg):
        if 'pallet_x' in msg.name:
            x = msg.position[msg.name.index('pallet_x')]
            self.pallet_x = x
            self.pallet_hist.append((self.now_s(), x))
            self.pallet_hist = self.pallet_hist[-2000:]

    def x_at(self, t):
        if not self.pallet_hist:
            return self.pallet_x or 0.0
        ts, xs = zip(*self.pallet_hist)
        return float(np.interp(t, ts, xs))

    def broadcast(self):
        if self.pallet_x is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'world', 'pallet_datum'
        t.transform.translation.x = S3_X + self.pallet_x
        t.transform.translation.y, t.transform.translation.z = LINE_Y, PALLET_TOP
        t.transform.rotation.w = 1.0
        self.tfb.sendTransform(t)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, seconds):
        end = self.now_s() + seconds
        while rclpy.ok() and self.now_s() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def move_pallet(self, x, speed=None):
        """Command the pallet to x; at `speed` m/s as a ramp of set points, else in one step."""
        x0 = self.pallet_x
        if speed:
            n = max(1, int(abs(x - x0) / speed / 0.02))
            for k in range(1, n + 1):
                self.cmd.publish(Float64(data=x0 + (x - x0) * k / n))
                self.spin_for(0.02)
        self.cmd.publish(Float64(data=x))
        while rclpy.ok() and abs(self.pallet_x - x) > 0.0005:
            self.cmd.publish(Float64(data=x))
            self.spin_for(0.05)

    # -- sensors -------------------------------------------------------------------------
    def keep(self, kind, pts):
        if len(pts):
            self.clouds[kind].append(pts.astype(np.float32))

    def on_ranger(self, msg):
        if self.collect != 'fixed':
            return
        p = scan_points(msg)
        # gate sensor: +x down, fan along world y (inspection_line.sdf rpy 0 pi/2 0)
        w = np.column_stack([GATE_X + p[:, 2], LINE_Y + p[:, 1], 1.501 - p[:, 0]])
        x = self.x_at(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)
        self.keep('fixed', w - [S3_X + x, LINE_Y, PALLET_TOP])

    def in_window(self, stamp, kind):
        return any(a <= stamp <= b and k == kind for a, b, k in self.windows)

    def to_pallet(self, frame, stamp, pts):
        try:
            t = self.tf.lookup_transform('pallet_datum', frame, stamp)
        except Exception:
            return None
        T = tf_matrix(t)
        return pts @ T[:3, :3].T + T[:3, 3]

    def on_ljx(self, msg):
        if self.collect != 'arm':
            return
        s = Time.from_msg(msg.header.stamp)
        if not self.in_window(s.nanoseconds * 1e-9, 'sweep'):
            return
        self.raw.append((s, msg.header.frame_id, scan_points(msg)))

    def on_zivid(self, msg):
        self.latest_zivid = msg

    def on_joints(self, msg):
        if all(j in msg.name for j in JOINTS):
            self.joint_now = [msg.position[msg.name.index(j)] for j in JOINTS]

    def grab_zivid(self, after_s):
        """The first depth cloud stamped after `after_s` (the arm has settled), in pallet_datum."""
        end = self.now_s() + 2.0
        while rclpy.ok() and self.now_s() < end:
            m = self.latest_zivid
            if m is not None and Time.from_msg(m.header.stamp).nanoseconds * 1e-9 >= after_s:
                xyz = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
                xyz = xyz[np.isfinite(xyz).all(axis=1)][::4]
                self.raw.append((Time.from_msg(m.header.stamp), m.header.frame_id, xyz.astype(float)))
                return
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish_cloud(self, kind):
        if not self.clouds[kind]:
            return 0
        p = np.concatenate(self.clouds[kind])
        if self.voxel:
            _, i = np.unique(np.floor(p / self.voxel).astype(np.int64), axis=0, return_index=True)
            p = p[i]
        msg = PointCloud2(header=Header(frame_id='pallet_datum', stamp=self.get_clock().now().to_msg()),
                          height=1, width=len(p), is_bigendian=False, point_step=12, row_step=12 * len(p),
                          is_dense=True, fields=[PointField(name=c, offset=4 * i, datatype=PointField.FLOAT32,
                                                            count=1) for i, c in enumerate('xyz')])
        msg.data = np.ascontiguousarray(p, dtype=np.float32).tobytes()
        self.pub[kind].publish(msg)
        return len(p)

    def place_raw(self):
        """Put every profile and depth cloud taken while flying into pallet_datum, by TF at its
        own timestamp. Returns how many could not be placed."""
        lost = 0
        for stamp, frame, pts in self.raw:
            p = self.to_pallet(frame, stamp, pts)
            if p is None:
                lost += 1
            else:
                self.keep('arm', p[p[:, 2] > -0.05])
        self.raw = []
        return lost

    def publish_module(self, folder):
        arr = MarkerArray()
        for i, f in enumerate(sorted(Path(folder).glob('*.stl'))):
            m = Marker(header=Header(frame_id='pallet_datum'), ns='module', id=i, type=Marker.MESH_RESOURCE,
                       action=Marker.ADD, mesh_resource=f'file://{f.resolve()}')
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = 0.7, 0.7, 0.75, 0.6
            arr.markers.append(m)
        self.markers.publish(arr)

    # -- the arm -------------------------------------------------------------------------
    def fly(self, points, windows):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=JOINTS)
        for q, t in points:
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=[float(v) for v in q],
                time_from_start=Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))))
        self.arm.wait_for_server()
        fut = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            raise RuntimeError('trajectory rejected')
        t0 = self.now_s()
        self.windows = [(t0 + a, t0 + b, k) for a, b, k in windows]
        res = handle.get_result_async()
        snaps = sorted((t0 + a, t0 + b) for a, b, k in windows if k == 'snapshot')
        for a, b in snaps:
            while rclpy.ok() and self.now_s() < a + 0.25:
                rclpy.spin_once(self, timeout_sec=0.01)
            self.grab_zivid(a + 0.25)
        rclpy.spin_until_future_complete(self, res)
        return self.now_s() - t0, res.result().result.error_code


def build(stage_plan, views, home, start):
    """(joint points with times, capture windows) for one stage: start -> home, then the plan."""
    pts, wins, t = [], [], 0.0
    vmax = np.radians([130, 130, 180, 180, 250, 250]) * 0.3      # 30 % for the unplanned first move (A)
    d = np.abs(np.subtract(home, start))
    t_home = max(2.0, float((d / vmax).max()))
    pts += [(start, 0.0), (home, t_home)]
    t = t_home

    def add_leg(leg):
        nonlocal t
        for q, tt in zip(leg['q'][1:], np.diff(leg['t'])):
            t += max(float(tt), 1e-3)
            pts.append((q, t))

    for i, vid in enumerate(stage_plan['order']):
        add_leg(stage_plan['legs'][i])
        path = stage_plan['paths'][vid]
        v = views[vid]
        if len(path) == 1:                  # snapshot: hold still for the camera
            wins.append((t, t + SNAP_DWELL, 'snapshot'))
            t += SNAP_DWELL
            pts.append((path[0], t))
        else:                               # sweep: constant sensor speed along the path
            dt = v['capture_s'] / (len(path) - 1)
            wins.append((t, t + v['capture_s'], 'sweep'))
            for q in path[1:]:
                t += dt
                pts.append((q, t))
    add_leg(stage_plan['legs'][-1])
    gap = max(float(np.abs(np.subtract(a[0], b[0])).max()) for a, b in zip(pts[1:], pts[:-1]))
    return pts, wins, gap


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--reach', type=Path, default=ROOT / 'data/reach/base_A/reach.json')
    ap.add_argument('--stage', default='post_weld', choices=['pre_busbar', 'pre_weld', 'post_weld'])
    ap.add_argument('--module-meshes', type=Path, default=ROOT / 'data/gazebo/models/motobat_pallet/meshes')
    ap.add_argument('--voxel', type=float, default=0.0005, help='m, thin the published clouds (0 = all)')
    ap.add_argument('--skip-fixed', action='store_true')
    ap.add_argument('--exit', action='store_true', help='quit when done instead of keeping the clouds latched')
    ap.add_argument('--save', type=Path, default=None, help='write the clouds as <dir>/{fixed,arm}.npy')
    a = ap.parse_args()

    res = json.load(open(a.reach))['results'][0]
    plan = res['sequences'][a.stage]
    views = {v['id']: v for v in res['views']}

    rclpy.init()
    n = Demo(a.voxel)
    while rclpy.ok() and (n.pallet_x is None or n.joint_now is None or n.now_s() == 0.0):
        rclpy.spin_once(n, timeout_sec=0.1)
    n.publish_module(a.module_meshes)
    report = {'stage': a.stage}

    if not a.skip_fixed:
        start, end = GATE_X - S3_X - MODULE_HALF - 0.03, GATE_X - S3_X + MODULE_HALF + 0.03
        n.get_logger().info(f'fixed pass: pallet to {start:+.3f} m, then under the gate at {LINE_SPEED * 1000:.0f} mm/s')
        n.move_pallet(start)
        n.collect = 'fixed'
        t0 = n.now_s()
        n.move_pallet(end, LINE_SPEED)
        report['fixed_pass_s'] = n.now_s() - t0
        n.collect = None
        report['fixed_points'] = n.publish_cloud('fixed')
    t0 = n.now_s()
    n.move_pallet(0.0)
    report['index_to_s3_s'] = n.now_s() - t0
    n.get_logger().info(f"pallet at the Inspector ({report['index_to_s3_s']:.1f} s)")

    pts, wins, gap = build(plan, views, res['home'], n.joint_now)
    report['largest_joint_step_rad'] = gap
    report['planned_s'] = pts[-1][1]
    report['plan_cycle_s'] = plan['cycle_s']
    n.get_logger().info(f"flying {a.stage}: {len(plan['order'])} views, {len(pts)} points, "
                        f"{pts[-1][1]:.1f} s (plan cycle {plan['cycle_s']:.1f} s + first move + "
                        f"{SNAP_DWELL - 0.1:.1f} s more per snapshot)")
    n.collect = 'arm'
    took, err = n.fly(pts, wins)
    n.collect = None
    n.spin_for(0.5)
    report['captures_taken'] = len(n.raw)
    report['captures_unplaced'] = n.place_raw()
    report.update(flown_s=took, error_code=int(err), arm_points=n.publish_cloud('arm'))
    if a.save:
        a.save.mkdir(parents=True, exist_ok=True)
        for k, c in n.clouds.items():
            if c:
                np.save(a.save / f'{k}.npy', np.concatenate(c))
        (a.save / 'report.json').write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))
    if not a.exit:
        n.get_logger().info('clouds latched on /fixed/cloud and /arm/cloud; Ctrl-C to quit')
        try:
            rclpy.spin(n)
        except KeyboardInterrupt:
            pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

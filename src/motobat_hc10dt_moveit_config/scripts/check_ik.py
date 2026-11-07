#!/usr/bin/python3
"""
How well does an IK solver handle the HC10DT? (task 2.1)

The HC10DT has no closed-form IK (offset wrist, project_plan.md §4A.2), so
MoveIt solves it numerically. This script measures whichever solver
move_group has loaded, through move_group's own services, exactly as
planning will use it:

  targets   poses for s3_tool0, in s3_base_link
  solve     /compute_ik from a fixed seed (the SRDF "ready" pose)
  verify    /compute_fk on the answer: accepted only within 0.1 mm and 0.06 deg

Three target sets:
  random  tool0 pose at a uniformly sampled collision-free joint state.
          Reachable by construction.
  down    tool0 pointing straight down over the area in front of the arm,
          random yaw about the tool axis. The orientation that defeated KDL
          on the myCobot in the swarm project.
  cone    as `down`, tilted 0-45 deg off vertical in a random direction. The
          oblique views multi-angle inspection needs (module walls block the
          outer terminal edges below ~39 deg elevation, module_spec.md §5).
  `down` and `cone` are not reachable by construction; compare_ik.py counts a
  target as reachable if any solver solves it.

Run against a live move_group (no RViz needed):

    ros2 launch motobat_hc10dt_moveit_config demo.launch.py use_rviz:=false
    ros2 run motobat_hc10dt_moveit_config check_ik.py --targets down

compare_ik.py drives this for several solvers on identical targets.
Uses system Python 3.12 (rclpy), never the 3.11 project venv.
"""
import argparse
import json
import math
import random
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

GROUP = 'inspector_arm'
TIP = 's3_tool0'
FRAME = 's3_base_link'
JOINTS = ['s3_joint_1_s', 's3_joint_2_l', 's3_joint_3_u',
          's3_joint_4_r', 's3_joint_5_b', 's3_joint_6_t']
# URDF limits (motoman_hc10_support/urdf/hc10dt_macro.xacro), radians.
LIMITS = [(-math.pi, math.pi), (-math.pi, math.pi), (math.radians(-5), math.radians(355)),
          (-math.pi, math.pi), (-math.pi, math.pi), (-math.pi, math.pi)]
# URDF velocity limits, rad/s: S 130, L 130, U 180, R 180, B 250, T 250 deg/s.
VMAX = [math.radians(v) for v in (130, 130, 180, 180, 250, 250)]
# SRDF "ready": forearm forward, tool down. The seed a planner would start from.
READY = [0.0, 0.0, math.pi / 2, 0.0, -math.pi / 2, 0.0]

# Acceptance: well inside what any registration downstream could notice.
POS_TOL = 1e-4   # m
ROT_TOL = 1e-3   # rad, 0.057 deg

# Where `down` and `cone` targets fall, in s3_base_link (m). The module's
# position relative to the Inspector is not fixed until task 2.5, so this is
# a generous box in front of the arm; tool0 is above the work surface by the
# sensor standoffs (LJ-X8200 ~245 mm, Zivid ~600 mm) plus the flange.
BOX = {'x': (0.30, 0.95), 'y': (-0.50, 0.50), 'z': (-0.10, 0.70)}
CONE_MAX_TILT = math.radians(45)


# ---- quaternions (x, y, z, w), no numpy needed ---------------------------------

def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_axis(axis, angle):
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def make_pose(p, q):
    m = Pose()
    m.position.x, m.position.y, m.position.z = p
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
    return m


def pose_to_list(m):
    return [m.position.x, m.position.y, m.position.z,
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w]


def list_to_pose(v):
    return make_pose(v[:3], v[3:])


def pose_error(a, b):
    """Translation error (m) and rotation error (rad) between two Poses."""
    dp = math.dist((a.position.x, a.position.y, a.position.z),
                   (b.position.x, b.position.y, b.position.z))
    qa, qb = a.orientation, b.orientation
    dot = abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w)
    return dp, 2.0 * math.acos(min(1.0, dot))


def move_time_bound(q0, q1):
    """Lower bound on joint-move time (s): the slowest joint at full speed.
    Ignores acceleration, so it ranks answers rather than predicting cycle time."""
    return max(abs(b - a) / v for a, b, v in zip(q0, q1, VMAX))


# ---- ROS -------------------------------------------------------------------

def state(q):
    rs = RobotState()
    rs.joint_state = JointState(name=JOINTS, position=list(q))
    return rs


class IkClient(Node):
    def __init__(self):
        super().__init__('check_ik')
        self.valid = self.create_client(GetStateValidity, '/check_state_validity')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        for c in (self.valid, self.fk, self.ik):
            if not c.wait_for_service(timeout_sec=30.0):
                sys.exit(f'{c.srv_name} not available: is move_group running?')

    def call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def is_valid(self, q):
        req = GetStateValidity.Request(robot_state=state(q), group_name=GROUP)
        return self.call(self.valid, req).valid

    def tip_pose(self, q):
        req = GetPositionFK.Request(fk_link_names=[TIP], robot_state=state(q))
        req.header.frame_id = FRAME
        res = self.call(self.fk, req)
        return res.pose_stamped[0].pose if res.error_code.val == 1 else None

    def solve(self, pose, seed, timeout, avoid_collisions=True):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name, r.ik_link_name, r.avoid_collisions = GROUP, TIP, avoid_collisions
        r.robot_state = state(seed)
        r.pose_stamped = PoseStamped(pose=pose)
        r.pose_stamped.header.frame_id = FRAME
        r.timeout.sec, r.timeout.nanosec = int(timeout), int((timeout % 1) * 1e9)
        t0 = time.perf_counter()
        res = self.call(self.ik, req)
        dt = time.perf_counter() - t0
        if res.error_code.val != 1:
            return None, dt
        names = list(res.solution.joint_state.name)
        return [res.solution.joint_state.position[names.index(j)] for j in JOINTS], dt


# ---- targets and evaluation ------------------------------------------------

def make_targets(node, kind, n, rng):
    """Return n target poses (as 7-lists) of the given kind."""
    out = []
    if kind == 'random':
        while len(out) < n:
            q = [rng.uniform(lo, hi) for lo, hi in LIMITS]
            if node.is_valid(q):
                p = node.tip_pose(q)
                if p is None:   # FK on a valid state cannot fail unless TF is broken
                    sys.exit('/compute_fk failed: is robot_state_publisher running '
                             '(move_group needs the world -> s3_base_link transform)?')
                out.append(pose_to_list(p))
        return out
    down = q_axis((1, 0, 0), math.pi)               # tool0 z -> base -z
    for _ in range(n):
        p = [rng.uniform(*BOX[k]) for k in 'xyz']
        q = q_mul(q_axis((0, 0, 1), rng.uniform(-math.pi, math.pi)), down)  # yaw about vertical
        if kind == 'cone':
            az = rng.uniform(-math.pi, math.pi)
            tilt = math.acos(rng.uniform(math.cos(CONE_MAX_TILT), 1.0))  # uniform on the cap
            q = q_mul(q_axis((math.cos(az), math.sin(az), 0), tilt), q)
        out.append(p + list(q))
    return out


def evaluate(node, targets, timeout, seed=READY, avoid_collisions=True):
    """Solve every target; return one record per target."""
    recs = []
    for t in targets:
        target = list_to_pose(t)
        sol, dt = node.solve(target, seed, timeout, avoid_collisions)
        rec = {'solved': False, 'time': dt, 'q': None, 'pos_err': None, 'rot_err': None,
               'move_time': None}
        if sol is not None:
            dp, dr = pose_error(target, node.tip_pose(sol))
            rec.update(q=sol, pos_err=dp, rot_err=dr, move_time=move_time_bound(seed, sol),
                       solved=dp < POS_TOL and dr < ROT_TOL)
        recs.append(rec)
    return recs


def pct(xs, p):
    xs = sorted(xs)
    return xs[int(p * (len(xs) - 1))] if xs else float('nan')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--targets', choices=['random', 'down', 'cone'], default='random')
    ap.add_argument('--samples', type=int, default=300)
    ap.add_argument('--timeout', type=float, default=0.05,
                    help='IK timeout per call, s (kinematics.yaml uses 0.05)')
    ap.add_argument('--rng', type=int, default=0, help='RNG seed, for repeatable runs')
    ap.add_argument('--load', help='read targets from this JSON instead of generating')
    ap.add_argument('--save', help='write the targets to this JSON')
    args = ap.parse_args()

    rclpy.init()
    node = IkClient()
    if args.load:
        targets = json.load(open(args.load))['targets']
    else:
        targets = make_targets(node, args.targets, args.samples, random.Random(args.rng))
    if args.save:
        json.dump({'kind': args.targets, 'rng': args.rng, 'frame': FRAME, 'tip': TIP,
                   'targets': targets}, open(args.save, 'w'))

    recs = evaluate(node, targets, args.timeout)
    ok = [r for r in recs if r['solved']]
    print(f'\n{len(ok)}/{len(recs)} solved ({args.targets}, timeout {1e3 * args.timeout:.0f} ms, '
          f'seed ready). Solve time median {1e3 * statistics.median([r["time"] for r in ok] or [0]):.1f} ms, '
          f'p95 {1e3 * pct([r["time"] for r in ok], 0.95):.1f} ms.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

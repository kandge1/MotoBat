#!/usr/bin/python3
"""
Where must the HC10DT stand to fly every view of the 1.5 plan? (task 2.5, step 1)

The view plan (`data/coverage/<stage>.plan.json`, from defectgen task 1.5) gives
each capture as a full 6-DOF sensor pose in the module frame. A 6-axis arm has
no slack left for a fully specified pose: only its IK branch is free. So the
design freedom is where the module sits relative to the robot base. This script
searches a grid of base placements and, for each, asks move_group whether every
distinct view can be flown:

  snapshot  collision-free IK for the Zivid camera frame at the planned pose
  sweep     collision-free IK at the run-in point, then a straight, collision-free
            Cartesian path through the whole measured length plus run-out
            (/compute_cartesian_path, 5 mm steps, no branch flips)

Sensor frames (defectgen -> URDF), checked against both models 14 Nov 2026:
  snapshot      defectgen puts the projector on the camera's -x; the URDF's
                zivid_optical has x toward the projector. Same optical centre and
                +z, so T(zivid_optical) = T(plan) . Rz(180 deg).
  profilometer  defectgen's sensor origin is reference_distance (245 mm) before the
                measuring centre; the URDF's ljx_laser is the emitter face, 275 mm
                before it. Both put the receiver on -y and sweep along +y, so the
                plan is matched at the measuring centre:
                T(ljx_reference) = T(plan) . Tz(245 mm).

Scene (all A, until a conveyor is designed): a keep-out box, the module's
355 x 151 x 108 mm envelope grown by `--pad` (default 50 mm) on every side but
the bottom, that no part of the robot or flange may enter; a pallet 25 mm larger
per side and 20 mm thick; directly under the pallet a support column of the
pallet's footprint down to the floor, which is `--floor` below the pallet datum;
the robot on a pedestal (`--mount floor`) or hung from a plate (`--mount
ceiling`). Nothing else: the point is to measure how much room the arm needs
around the pallet, so the conveyor can be sized to stay out of it.

With `--plan` (one base), it also orders each stage's views and plans every move
between them with OMPL in the same scene, so the moves are collision-checked too,
and times them (time-optimal parameterisation at the joint limits in
joint_limits.yaml; the accelerations there are A). Captures take what the 1.5 plan
says, less the 1.5 s per move it assumed: 0.1 s per snapshot, sweeps at their
planned speed including the run-up.

    ros2 run motobat_hc10dt_moveit_config view_reach.py --out data/reach/grid1
    ros2 run motobat_hc10dt_moveit_config view_reach.py --base 0.15 0.65 -0.3 --plan \
        --out data/reach/base_X                                             # one base

Starts its own move_group (launch/ik_compare.launch.py, no RViz, no controllers).
Run it under a memory cap. Uses system Python 3.12 (rclpy), never the project venv.
"""
import argparse
import itertools
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MoveItErrorCodes,
                             PlanningScene, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPositionFK,
                             GetPositionIK, GetStateValidity)
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

PKG = 'motobat_hc10dt_moveit_config'
GROUP = 'inspector_arm'
FRAME = 's3_base_link'
TOOL = 's3_tool0'
JOINTS = ['s3_joint_1_s', 's3_joint_2_l', 's3_joint_3_u',
          's3_joint_4_r', 's3_joint_5_b', 's3_joint_6_t']
LINKS = ['s3_link_1_s', 's3_link_2_l', 's3_link_3_u', 's3_link_4_r',
         's3_link_5_b', 's3_link_6_t', 's3_flange']
SENSOR_LINK = {'snapshot': 's3_zivid_optical', 'profilometer': 's3_ljx_reference'}
STAGES = ['pre_busbar', 'pre_weld', 'post_weld']
SHORT = {'pre_busbar': 'busbar', 'pre_weld': 'preweld', 'post_weld': 'postweld'}

PROFILE_REF = 0.245        # m, defectgen profilometer reference_distance (sensors.yaml, S)
RUN_UP = 0.025             # m before and after each sweep, to reach constant speed (A)
MODULE = (0.355, 0.151, 0.108)   # VDA 355 envelope (vda355_3p4s.yaml, S)
PALLET_MARGIN = 0.025      # A
PALLET_T = 0.020           # A
PEDESTAL = 0.30            # m square under the robot base (A)
PAD = 0.05                 # m keep-out around the module, every side but the bottom (user, 14 Nov)
ASSUMED_MOVE = 1.5         # s per capture that 1.5's plan 'seconds' includes for the move (A there)
READY = [0.0, 0.0, math.pi / 2, 0.0, -math.pi / 2, 0.0]
HOMES = [READY, [0.0, -0.6, 1.2, 0.0, -1.2, 0.0], [0.0, 0.6, 1.2, 0.0, -1.2, 0.0],
         [0.0, -1.0, 0.6, 0.0, -1.0, 0.0], [0.0, 1.0, 0.6, 0.0, -1.0, 0.0]]

IK_TIMEOUT = 0.05
CART_STEP = 0.005
JUMP = 0.3                 # rad per 5 mm step: anything bigger is a branch flip


# ---- transforms ------------------------------------------------------------------

def tf(R=None, p=(0, 0, 0)):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = p
    return T


def rz(a):
    return Rotation.from_euler('z', a).as_matrix()


def to_pose(T):
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


def from_pose(p):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([p.orientation.x, p.orientation.y,
                                    p.orientation.z, p.orientation.w]).as_matrix()
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def base_in_module(bx, d, bz, turn=0.0, mount='floor'):
    """Robot base `d` from the module's centreline on its -Y side, `bx` along it and `bz`
    above the pallet datum, x axis toward the module. `ceiling` hangs it upside down.
    `turn` rad turns the pallet about the module's vertical axis (a turntable)."""
    R = rz(math.pi / 2)
    if mount == 'ceiling':
        R = R @ Rotation.from_euler('x', math.pi).as_matrix()
    return tf(rz(-turn)) @ tf(R, (bx, -d, bz))


# ---- the plan --------------------------------------------------------------------

def load_views(coverage_dir):
    """Distinct views over the three stages, in metres, with the stages that use each."""
    views = {}
    for stage in STAGES:
        plan = json.load(open(Path(coverage_dir) / f'{stage}.plan.json'))
        for v in plan['views']:
            T = np.array(v['pose_mm'], dtype=float)
            T[:3, 3] /= 1000.0
            key = (v['sensor'], tuple(np.round(T, 4).ravel()), round(v['length_mm'], 1))
            if key not in views:
                cap = v['seconds'] - ASSUMED_MOVE
                speed = v['length_mm'] / cap if v['sensor'] == 'profilometer' else None
                views[key] = {'id': v['id'], 'sensor': v['sensor'], 'kind': v['kind'],
                              'T_plan': T, 'length': v['length_mm'] / 1000.0,
                              'speed_mm_s': speed, 'stages': [],
                              'capture_s': cap if speed is None else
                              (v['length_mm'] + 2000 * RUN_UP) / speed}
            views[key]['stages'].append(stage)
    return list(views.values())


def sensor_targets(view):
    """Sensor-link poses in the module frame: one for a snapshot, run-in and run-out for a sweep."""
    T = view['T_plan']
    if view['sensor'] == 'snapshot':
        return [T @ tf(rz(math.pi))]
    ref = T @ tf(p=(0, 0, PROFILE_REF))
    return [ref @ tf(p=(0, -RUN_UP, 0)), ref @ tf(p=(0, view['length'] + RUN_UP, 0))]


# ---- scene -----------------------------------------------------------------------

def box(name, size, centre_module, T_bm):
    co = CollisionObject()
    co.header.frame_id = FRAME
    co.id = name
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    co.primitives = [prim]
    co.primitive_poses = [to_pose(T_bm @ tf(p=centre_module))]
    co.operation = CollisionObject.ADD
    return co


def scene_boxes(T_mb, floor, pad=PAD, mount='floor'):
    """(name, size, centre) in the module frame, m: what the robot must not touch. The
    module's own envelope is drawn by the viewer only; the keep-out box contains it."""
    L, W, H = MODULE
    pl, pw = L + 2 * PALLET_MARGIN, W + 2 * PALLET_MARGIN
    col_h = floor - PALLET_T
    bx, by, bz = T_mb[:3, 3]
    out = [
        ('keepout', (L + 2 * pad, W + 2 * pad, H + pad), (0, 0, (H + pad) / 2)),
        ('pallet', (pl, pw, PALLET_T), (0, 0, -PALLET_T / 2)),
        ('support', (pl, pw, col_h), (0, 0, -PALLET_T - col_h / 2)),
        ('floor', (6.0, 6.0, 0.02), (0, 0, -floor - 0.01)),
    ]
    if mount == 'ceiling':
        out.append(('mount_plate', (PEDESTAL, PEDESTAL, 0.2), (bx, by, bz + 0.105)))
    elif bz + floor - 0.005 > 0.01:
        ped_h = bz + floor - 0.005
        out.append(('pedestal', (PEDESTAL, PEDESTAL, ped_h), (bx, by, -floor + ped_h / 2)))
    return out


def scene_objects(T_mb, floor, pad=PAD, mount='floor'):
    T_bm = np.linalg.inv(T_mb)
    return [box(n, size, c, T_bm) for n, size, c in scene_boxes(T_mb, floor, pad, mount)]


# ---- move_group client -------------------------------------------------------------

class Client(Node):
    def __init__(self):
        super().__init__('view_reach')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.cart = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.valid = self.create_client(GetStateValidity, '/check_state_validity')
        for c in (self.ik, self.fk, self.cart, self.scene, self.planner, self.valid):
            if not c.wait_for_service(timeout_sec=60.0):
                raise RuntimeError(f'{c.srv_name} not available')

    def call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        return fut.result()

    def state(self, q):
        rs = RobotState()
        rs.joint_state = JointState(name=JOINTS, position=[float(v) for v in q])
        return rs

    def set_scene(self, objs):
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = objs
        res = self.call(self.scene, ApplyPlanningScene.Request(scene=ps))
        if not (res and res.success):
            raise RuntimeError('apply_planning_scene failed')

    def fk_links(self, q, links):
        req = GetPositionFK.Request()
        req.header.frame_id = FRAME
        req.fk_link_names = links
        req.robot_state = self.state(q)
        res = self.call(self.fk, req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError('compute_fk failed')
        return {n: from_pose(p.pose) for n, p in zip(res.fk_link_names, res.pose_stamped)}

    def solve(self, T_tool, seed, avoid=True):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.robot_state = self.state(seed)
        r.avoid_collisions = avoid
        r.ik_link_name = TOOL
        r.pose_stamped = PoseStamped(pose=to_pose(T_tool))
        r.pose_stamped.header.frame_id = FRAME
        r.timeout.nanosec = int(IK_TIMEOUT * 1e9)
        res = self.call(self.ik, req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        names = list(res.solution.joint_state.name)
        return [res.solution.joint_state.position[names.index(j)] for j in JOINTS]

    def line(self, q0, T_end):
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.start_state = self.state(q0)
        req.group_name = GROUP
        req.link_name = TOOL
        req.waypoints = [to_pose(T_end)]
        req.max_step = CART_STEP
        req.jump_threshold = 0.0
        req.revolute_jump_threshold = JUMP
        req.avoid_collisions = True
        res = self.call(self.cart, req)
        pts = [list(p.positions) for p in res.solution.joint_trajectory.points]
        return res.fraction, pts

    def is_valid(self, q):
        req = GetStateValidity.Request(robot_state=self.state(q), group_name=GROUP)
        return self.call(self.valid, req).valid

    def motion(self, q0, q1, seconds=2.0, attempts=3):
        """OMPL joint-space plan in the current scene, time-parameterised by move_group
        at full joint speed. {'q': [...], 't': [...]} or None."""
        req = GetMotionPlan.Request()
        m = req.motion_plan_request
        m.group_name = GROUP
        m.start_state = self.state(q0)
        m.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(v), tolerance_above=1e-4,
                            tolerance_below=1e-4, weight=1.0) for j, v in zip(JOINTS, q1)])]
        m.pipeline_id = 'ompl'
        m.allowed_planning_time = seconds
        m.num_planning_attempts = attempts
        m.max_velocity_scaling_factor = 1.0
        m.max_acceleration_scaling_factor = 1.0
        res = self.call(self.planner, req)
        if res is None or res.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        jt = res.motion_plan_response.trajectory.joint_trajectory
        idx = [list(jt.joint_names).index(j) for j in JOINTS]
        return {'q': [[p.positions[i] for i in idx] for p in jt.points],
                't': [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in jt.points]}


def seeds(T_tool):
    """IK seeds spread over the HC10DT's branches, S axis aimed at the target."""
    aim = math.atan2(T_tool[1, 3], T_tool[0, 3])
    arms = [(0.0, 1.57), (0.6, 1.0), (-0.5, 2.2), (0.9, 2.5), (1.2, 0.6)]
    wrists = [(0.0, -1.57), (math.pi, 1.57), (0.0, 1.0)]
    return [[aim, l, u, r, b, 0.0] for (l, u), (r, b) in itertools.product(arms, wrists)]


def distinct(sols, tol=0.3):
    out = []
    for s in sols:
        if all(max(abs(a - b) for a, b in zip(s, o)) > tol for o in out):
            out.append(s)
    return out


# ---- one base ----------------------------------------------------------------------

def envelope_points(cli, q, T_mb, flange_boxes):
    """Forearm and wrist link origins and flange collision-box corners, in the module
    frame (m): the parts that come near the pallet. The shoulder stays by the base."""
    fr = cli.fk_links(q, LINKS)
    pts = [(T_mb @ fr[n])[:3, 3] for n in LINKS[3:]]
    Tf = T_mb @ fr['s3_flange']
    for b in flange_boxes.values():
        c, s = np.array(b['xyz']), np.array(b['size']) / 2
        for sign in itertools.product((-1, 1), repeat=3):
            pts.append((Tf @ np.append(c + s * np.array(sign), 1.0))[:3])
    return np.array(pts)


def check_base(cli, views, T_mb, T_tool_sensor, flange_boxes, floor, keep_paths=False,
               pad=PAD, mount='floor'):
    cli.set_scene(scene_objects(T_mb, floor, pad, mount))
    T_bm = np.linalg.inv(T_mb)
    out = []
    for v in views:
        targets = [T_bm @ T @ np.linalg.inv(T_tool_sensor[SENSOR_LINK[v['sensor']]])
                   for T in sensor_targets(v)]
        # Every seed, every distinct IK branch: a sweep that fails from one branch often
        # works from another, and KDL's random restarts make a short search hit or miss.
        r = {'id': v['id'], 'sensor': v['sensor'], 'stages': v['stages'], 'ok': False}
        sols, tried, best = [], [], 0.0
        for s in seeds(targets[0]):
            q = cli.solve(targets[0], s)
            if q is None:
                continue
            sols.append(q)
            if v['sensor'] == 'snapshot':
                r.update(ok=True, q=q, path=[q])
                break
            if not all(max(abs(a - b) for a, b in zip(q, o)) > 0.3 for o in tried):
                continue
            tried.append(q)
            frac, pts = cli.line(q, targets[1])
            best = max(best, frac)
            if frac > 0.999:
                r.update(ok=True, q=q, path=pts)
                break
        if sols:
            # Proof the frame chain is right: FK the sensor frame back into the module frame.
            link = SENSOR_LINK[v['sensor']]
            T_got = T_mb @ cli.fk_links(sols[0], [link])[link]
            T_want = sensor_targets(v)[0]
            r['check_mm'] = float(np.linalg.norm(T_got[:3, 3] - T_want[:3, 3]) * 1000)
            r['check_deg'] = float(np.degrees(Rotation.from_matrix(
                T_got[:3, :3].T @ T_want[:3, :3]).magnitude()))
        if not sols:
            free = any(cli.solve(targets[0], s, avoid=False) is not None for s in seeds(targets[0]))
            r['why'] = 'reachable but every IK answer collides' if free else 'out of reach'
        elif not r['ok']:
            r['why'] = (f'sweep path {best:.0%} of the way from the best of {len(tried)} IK branches '
                        '(collision, reach or branch flip)')
        if r['ok']:
            path = r.pop('path')
            if keep_paths:
                r['path'] = path
            sample = path[::max(1, len(path) // 6)] + [path[-1]]
            pts = np.vstack([envelope_points(cli, q, T_mb, flange_boxes) for q in sample])
            below = pts[pts[:, 2] < 0.0]
            r['min_z'] = float(pts[:, 2].min())
            r['below_datum_min_abs_y'] = float(np.abs(below[:, 1]).min()) if len(below) else None
            r['below_datum_min_abs_x'] = float(np.abs(below[:, 0]).min()) if len(below) else None
        out.append(r)
    return out


# ---- order and transits (--plan) ---------------------------------------------------

def joint_limits():
    import yaml
    from ament_index_python.packages import get_package_share_directory
    jl = yaml.safe_load(open(Path(get_package_share_directory(PKG)) / 'config' / 'joint_limits.yaml'))
    lim = jl['joint_limits']
    return ([lim[j]['max_velocity'] for j in JOINTS], [lim[j]['max_acceleration'] for j in JOINTS])


def move_estimate(q0, q1, vmax, amax):
    """Seconds for a synchronised joint move: the slowest joint's trapezoid (or triangle)."""
    t = 0.0
    for a, b, v, ac in zip(q0, q1, vmax, amax):
        dq = abs(b - a)
        t = max(t, dq / v + v / ac if dq > v * v / ac else 2.0 * math.sqrt(dq / ac))
    return t


def order_views(views, home, vmax, amax):
    """Order to fly a stage's views, home to home, shortest estimated move time.
    Nearest neighbour, then relocate / segment-reverse local search until nothing helps.
    Sweeps keep the plan's direction, so a view is entered at path[0] and left at path[-1]."""
    n = len(views)
    if n < 2:
        return list(range(n))
    ent = [v['path'][0] for v in views]
    ext = [v['path'][-1] for v in views]

    def cost(order):
        t, q = 0.0, home
        for i in order:
            t += move_estimate(q, ent[i], vmax, amax)
            q = ext[i]
        return t + move_estimate(q, home, vmax, amax)

    order, q, left = [], home, set(range(n))
    while left:
        i = min(left, key=lambda k: move_estimate(q, ent[k], vmax, amax))
        order.append(i)
        left.remove(i)
        q = ext[i]
    best = cost(order)
    improved = True
    while improved:
        improved = False
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                cand = order[:i] + order[i + 1:]
                cand.insert(j, order[i])
                for c in (cand, order[:min(i, j)] + order[min(i, j):max(i, j) + 1][::-1]
                          + order[max(i, j) + 1:]):
                    cc = cost(c)
                    if cc < best - 1e-9:
                        order, best, improved = c, cc, True
    return order


def plan_leg(cli, q0, q1):
    for seconds, attempts in ((2.0, 3), (5.0, 3), (10.0, 6)):
        leg = cli.motion(q0, q1, seconds, attempts)
        if leg is not None:
            return leg
    return None


def plan_sequences(cli, res_views, home):
    """Per stage: the flying order and an OMPL-planned, timed move into every view and
    back home. Also home -> view -> home for every view, so any single view or any
    subset can be shown with checked moves (going through home)."""
    vmax, amax = joint_limits()
    flown = [v for v in res_views if v['ok']]
    solo = {}
    for v in flown:
        solo[v['id']] = {'in': plan_leg(cli, home, v['path'][0]),
                         'out': plan_leg(cli, v['path'][-1], home)}
    seqs = {}
    for stage in STAGES:
        vs = [v for v in flown if stage in v['stages']]
        order = [vs[i] for i in order_views(vs, home, vmax, amax)]
        legs, q = [], home
        for v in order:
            legs.append(plan_leg(cli, q, v['path'][0]))
            q = v['path'][-1]
        legs.append(plan_leg(cli, q, home))
        move = sum(leg['t'][-1] for leg in legs if leg)
        capture = sum(v['capture_s'] for v in order)
        seqs[stage] = {'order': [v['id'] for v in order], 'legs': legs,
                       'unplanned_legs': sum(1 for leg in legs if leg is None),
                       'move_s': move, 'capture_s': capture, 'cycle_s': move + capture,
                       'missing': [v['id'] for v in res_views
                                   if stage in v['stages'] and not v['ok']]}
        print(f"  {stage}: {len(order)} views, cycle {move + capture:.1f} s "
              f"(moves {move:.1f} s, captures {capture:.1f} s)"
              + (f", {seqs[stage]['unplanned_legs']} moves FAILED to plan"
                 if seqs[stage]['unplanned_legs'] else '')
              + (f", not reachable {seqs[stage]['missing']}" if seqs[stage]['missing'] else ''),
              flush=True)
    return seqs, solo


def pick_home(cli):
    for q in HOMES:
        if cli.is_valid(q):
            return q
    raise RuntimeError('no collision-free home pose in HOMES')


# ---- main --------------------------------------------------------------------------

def start_move_group(log_path):
    return subprocess.Popen(['ros2', 'launch', PKG, 'ik_compare.launch.py'],
                            stdout=open(log_path, 'w'), stderr=subprocess.STDOUT,
                            start_new_session=True)


def stop(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--coverage', default='data/coverage', help='folder with <stage>.plan.json')
    ap.add_argument('--out', default='data/reach/grid1')
    ap.add_argument('--floor', type=float, default=0.9, help='pallet datum above the floor, m (A)')
    ap.add_argument('--base', type=float, nargs=3, metavar=('BX', 'D', 'BZ'),
                    help='one base: along the module, distance from its centreline, height '
                         'above the pallet datum (m)')
    ap.add_argument('--bx', type=float, nargs='+', default=[0.0, 0.15, 0.30])
    ap.add_argument('--d', type=float, nargs='+', default=[0.45, 0.55, 0.65, 0.75, 0.85, 0.95])
    ap.add_argument('--bz', type=float, nargs='+', default=[-0.60, -0.45, -0.30, -0.15, 0.0, 0.15])
    ap.add_argument('--turns', type=float, nargs='+', default=[0.0],
                    help='turntable positions, deg; a view counts if any position reaches it')
    ap.add_argument('--mount', choices=['floor', 'ceiling'], default='floor',
                    help='floor: on a pedestal; ceiling: hung upside down (BZ above the datum)')
    ap.add_argument('--pad', type=float, default=PAD,
                    help=f'keep-out margin around the module, m (default {PAD})')
    ap.add_argument('--keep-paths', action='store_true',
                    help='store every view\'s joint path, for show_view_plan.py')
    ap.add_argument('--plan', action='store_true',
                    help='one --base only: also order each stage and plan every move (implies '
                         '--keep-paths, no turntable)')
    ap.add_argument('--no-launch', action='store_true', help='use an already running move_group')
    a = ap.parse_args()

    if a.plan:
        if not a.base:
            sys.exit('--plan needs --base')
        a.keep_paths, a.turns = True, [0.0]
    views = load_views(a.coverage)
    bases = [tuple(a.base)] if a.base else list(itertools.product(a.bx, a.d, a.bz))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    print(f'{len(views)} distinct views over {len(STAGES)} stages; {len(bases)} bases', flush=True)

    import yaml
    from ament_index_python.packages import get_package_share_directory
    fl = yaml.safe_load(open(Path(get_package_share_directory('motobat_description'))
                             / 'config' / 'inspector_flange.yaml'))['inspector_flange']

    proc = None if a.no_launch else start_move_group(out / 'move_group.log')
    rclpy.init()
    try:
        cli = Client()
        for attempt in range(20):    # move_group answers before robot_state_publisher's TF lands
            try:
                fr = cli.fk_links(READY, [TOOL] + list(SENSOR_LINK.values()))
                break
            except RuntimeError:
                if attempt == 19:
                    raise
                time.sleep(1.0)
        T_tool_sensor = {n: np.linalg.inv(fr[TOOL]) @ fr[n] for n in SENSOR_LINK.values()}
        results = []
        t0 = time.time()
        for i, (bx, d, bz) in enumerate(bases):
            per_turn = {}
            for turn in a.turns:
                T_mb = base_in_module(bx, d, bz, math.radians(turn), a.mount)
                per_turn[turn] = check_base(cli, views, T_mb, T_tool_sensor,
                                            fl['collision_boxes'], a.floor, a.keep_paths,
                                            a.pad, a.mount)
            res = []
            for k in range(len(views)):
                for t in a.turns:
                    per_turn[t][k]['speed_mm_s'] = views[k]['speed_mm_s']
                    per_turn[t][k]['capture_s'] = views[k]['capture_s']
                    per_turn[t][k]['length'] = views[k]['length']
                    per_turn[t][k]['T_plan'] = views[k]['T_plan'].tolist()
                hit = [t for t in a.turns if per_turn[t][k]['ok']]
                r = dict(per_turn[hit[0] if hit else a.turns[0]][k])
                r['turns_ok'] = hit
                res.append(r)
            n_ok = sum(r['ok'] for r in res)
            slots = {s: (sum(r['ok'] for r in res if s in r['stages']),
                         sum(1 for r in res if s in r['stages'])) for s in STAGES}
            entry = {'base': {'bx': bx, 'd': d, 'bz': bz, 'mount': a.mount}, 'ok': n_ok,
                     'of': len(res), 'stages': slots, 'views': res}
            if len(a.turns) > 1:
                entry['per_turn'] = {str(t): v for t, v in per_turn.items()}
            results.append(entry)
            miss = [r['id'] for r in res if not r['ok']]
            print(f'[{i + 1}/{len(bases)} {time.time() - t0:5.0f}s] bx {bx:+.2f} d {d:.2f} '
                  f'bz {bz:+.2f}: {n_ok}/{len(res)}  '
                  + ' '.join(f'{SHORT[s]}:{o}/{n}'
                             for s, (o, n) in slots.items())
                  + (f'  missing {miss}' if miss else ''), flush=True)
            if a.plan:
                home = pick_home(cli)
                entry['home'] = home
                entry['sequences'], entry['solo'] = plan_sequences(cli, res, home)
            json.dump({'floor': a.floor, 'run_up': RUN_UP, 'module': MODULE, 'pad': a.pad,
                       'mount': a.mount, 'pallet_margin': PALLET_MARGIN, 'results': results},
                      open(out / 'reach.json', 'w'), indent=1)
    finally:
        rclpy.try_shutdown()
        if proc:
            stop(proc)
    best = sorted(results, key=lambda r: (-r['ok'], -min(o / n for o, n in r['stages'].values())))[:10]
    print('\nbest bases:')
    for r in best:
        print(f"  {r['ok']}/{r['of']}  {r['base']}  " +
              ' '.join(f'{s}:{o}/{n}' for s, (o, n) in r['stages'].items()))


if __name__ == '__main__':
    main()

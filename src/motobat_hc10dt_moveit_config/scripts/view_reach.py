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
import functools
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
from moveit_msgs.msg import (AllowedCollisionEntry, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, PlanningScene, PlanningSceneComponents, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene,
                             GetPositionFK, GetPositionIK, GetStateValidity)
from scipy.spatial.transform import Slerp
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
# Placeholder conveyor (18 Nov, sized to base A's envelope: the robot never goes more than
# 40 mm below the pallet top, but past the module ends it does so on the centreline, where
# the belt runs). The conveyor runs along the module's length; at the station a
# lift-and-locate raises the pallet CONVEYOR_LIFT off the belt (A). None = no conveyor.
CONVEYOR_LIFT = None       # m, set by --conveyor-lift
CONVEYOR_LEN = 3.0         # m of line drawn and checked, centred on the station (A)
CONVEYOR_RAIL = 0.04       # m of frame each side of the pallet (A)
CONVEYOR_H = 0.10          # m, belt + frame profile depth (A)
ASSUMED_MOVE = 1.5         # s per capture that 1.5's plan 'seconds' includes for the move (A there)
READY = [0.0, 0.0, math.pi / 2, 0.0, -math.pi / 2, 0.0]
HOMES = [READY, [0.0, -0.6, 1.2, 0.0, -1.2, 0.0], [0.0, 0.6, 1.2, 0.0, -1.2, 0.0],
         [0.0, -1.0, 0.6, 0.0, -1.0, 0.0], [0.0, 1.0, 0.6, 0.0, -1.0, 0.0]]

# Moves between views keep the wrist, flange and sensors above the module top (user, 18 Nov:
# "not below ground + height of the battery module"). A view lower than that is entered by a
# straight vertical approach from above it and left the same way, like an INFORM approach point.
TRANSIT_FLOOR = MODULE[2]  # m above the pallet top
EFFECTOR_LINKS = ['s3_link_4_r', 's3_link_5_b', 's3_link_6_t', 's3_flange']
APPROACH_SPEED = 0.25      # m/s, straight approach and depart (A)
LIFT_MAX = 0.5             # m, highest approach point tried
SLAB = 'transit_floor'

MAX_BRANCHES = 4           # distinct IK branches kept per view for the sequence to choose from
MOVJ_STEP = 0.01           # rad: largest joint step between collision checks on a joint move
LEG_KIND = {'joint': 0, 'via': 0, 'ompl': 0, 'failed': 0}   # how the moves were planned (for the record)
VIA_HEIGHTS = (0.15, 0.30, 0.45)  # m: raised waypoints tried when a straight joint move collides
VIA_BRANCH = 1.5           # rad: a raised waypoint further than this from its source is another branch

IK_TIMEOUT = 0.05
CART_STEP = 0.005
JUMP = 0.3                 # rad per 5 mm step: anything bigger is a branch flip
DENSE_JUMP = 0.1           # rad per <=5 mm step when rebuilding a checked line point by point


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
    if CONVEYOR_LIFT is not None:
        top = -PALLET_T - CONVEYOR_LIFT
        out.append(('conveyor', (CONVEYOR_LEN, pw + 2 * CONVEYOR_RAIL, CONVEYOR_H),
                    (0, 0, top - CONVEYOR_H / 2)))
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
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        for c in (self.ik, self.fk, self.cart, self.scene, self.planner, self.valid, self.get_scene):
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

    def line(self, q0, T_end, dense=False):
        """Straight tool line from q0 to T_end: (fraction reached, joint path).
        move_group checks the line every CART_STEP, but returns it re-timed and resampled at
        ~0.1 s, i.e. a point every 25-40 mm. With `dense`, the reached part is rebuilt at
        CART_STEP spacing by IK seeded from that path (see dense_line)."""
        T0 = self.fk_links(q0, [TOOL])[TOOL]
        n = max(1, int(math.ceil(np.linalg.norm(T_end[:3, 3] - T0[:3, 3]) / CART_STEP)))
        rot = Slerp([0, 1], Rotation.from_matrix([T0[:3, :3], T_end[:3, :3]]))
        wps = []
        for k in range(1, n + 1):
            f = k / n
            wps.append(to_pose(tf(rot(f).as_matrix(), T0[:3, 3] + f * (T_end[:3, 3] - T0[:3, 3]))))
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.start_state = self.state(q0)
        req.group_name = GROUP
        req.link_name = TOOL
        req.waypoints = wps
        req.max_step = CART_STEP
        req.jump_threshold = 0.0
        req.revolute_jump_threshold = JUMP
        req.avoid_collisions = True
        res = self.call(self.cart, req)
        pts = [list(p.positions) for p in res.solution.joint_trajectory.points]
        if dense and len(pts) > 1:
            d = self.dense_line(T0, T_end, res.fraction, pts)
            if d is None:
                self.get_logger().warn('could not densify a line; keeping move_group\'s points')
            else:
                pts = d
        return res.fraction, pts

    def dense_line(self, T0, T_end, frac, sparse):
        """The line from T0 to `frac` of the way to T_end, one collision-free IK solution
        every <= CART_STEP, each seeded from move_group's path at the same fraction; None if
        any point fails or any joint steps more than 0.05 rad (a branch change)."""
        L = np.linalg.norm(T_end[:3, 3] - T0[:3, 3]) * frac
        n = max(1, int(math.ceil(L / CART_STEP)))
        rot = Slerp([0, 1], Rotation.from_matrix([T0[:3, :3], T_end[:3, :3]]))
        S = np.array(sparse)
        u = np.linspace(0.0, 1.0, len(S))
        out = [list(sparse[0])]
        for k in range(1, n + 1):
            f = k / n * frac
            T = tf(rot(f).as_matrix(), T0[:3, 3] + f * (T_end[:3, 3] - T0[:3, 3]))
            q = None
            for seed in (out[-1], [float(np.interp(k / n, u, S[:, j])) for j in range(6)]):
                q = self.solve(T, seed)
                if q is not None and max(abs(a - b) for a, b in zip(q, out[-1])) <= DENSE_JUMP:
                    break
                q = None
            if q is None:
                self.get_logger().warn(f'dense line stops at {k}/{n}')
                return None
            out.append(q)
        return out

    def allow_slab_except_effector(self):
        """The transit floor may touch every robot link except the wrist, flange and sensors.
        A diff's ACM replaces the whole matrix, so read it, extend it, send it back."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self.call(self.get_scene, req).scene.allowed_collision_matrix
        if SLAB in acm.entry_names:
            return
        names = list(acm.entry_names)
        for name, row in zip(names, acm.entry_values):
            row.enabled = list(row.enabled) + [name not in EFFECTOR_LINKS]
        acm.entry_names = names + [SLAB]
        acm.entry_values = list(acm.entry_values) + [AllowedCollisionEntry(
            enabled=[n not in EFFECTOR_LINKS for n in names] + [True])]
        ps = PlanningScene(is_diff=True)
        ps.allowed_collision_matrix = acm
        if not self.call(self.scene, ApplyPlanningScene.Request(scene=ps)).success:
            raise RuntimeError('apply_planning_scene (ACM) failed')

    def slab(self, T_mb, floor, on):
        """Add or remove the transit floor: everything from the real floor up to TRANSIT_FLOOR."""
        if on:
            h = TRANSIT_FLOOR + floor
            co = box(SLAB, (6.0, 6.0, h), (0, 0, TRANSIT_FLOOR - h / 2), np.linalg.inv(T_mb))
        else:
            co = CollisionObject(id=SLAB, operation=CollisionObject.REMOVE)
            co.header.frame_id = FRAME
        self.set_scene([co])

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


EXTRA_SEEDS = 40


def extra_seeds(T_tool, found):
    """Up to EXTRA_SEEDS more seeds, uniform over the joint ranges (S near the target),
    from a fixed random stream so a run is repeatable; stops as soon as found() is non-empty."""
    rng = np.random.default_rng(7)
    aim = math.atan2(T_tool[1, 3], T_tool[0, 3])
    for _ in range(EXTRA_SEEDS):
        if found():
            return
        yield [aim + rng.uniform(-0.8, 0.8), rng.uniform(-2.5, 2.5), rng.uniform(0.1, 6.0),
               rng.uniform(-math.pi, math.pi), rng.uniform(-2.5, 2.5), rng.uniform(-math.pi, math.pi)]


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
        # With keep_paths (--plan), keep up to MAX_BRANCHES distinct working branches: which
        # one to fly is chosen with the view's neighbours (order_views), not here. Taking the
        # first that works made consecutive views elbow-up then elbow-down (18 Nov).
        r = {'id': v['id'], 'sensor': v['sensor'], 'stages': v['stages'], 'ok': False}
        sols, tried, best, branches = [], [], 0.0, []
        # the fixed seeds first; only if none works, EXTRA_SEEDS more spread over the joint
        # ranges (seen 18 Nov: a one-branch view found on one run and missed on the next)
        for s in itertools.chain(seeds(targets[0]), extra_seeds(targets[0], lambda: branches)):
            q = cli.solve(targets[0], s)
            if q is None:
                continue
            sols.append(q)
            if not all(max(abs(a - b) for a, b in zip(q, o)) > 0.3 for o in tried):
                continue
            tried.append(q)
            if v['sensor'] == 'snapshot':
                branches.append([q])
            else:
                frac, pts = cli.line(q, targets[1], dense=keep_paths)
                best = max(best, frac)
                if frac > 0.999:
                    branches.append(pts)
            if branches and (not keep_paths or len(branches) >= MAX_BRANCHES):
                break
        if branches:
            r.update(ok=True, q=branches[0][0], path=branches[0])
            if keep_paths:
                r['branches'] = branches
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
            # the envelope over every kept branch: any of them may be the one flown
            sample = [q for p in branches for q in p[::max(1, len(p) // 6)] + [p[-1]]]
            pts = np.vstack([envelope_points(cli, q, T_mb, flange_boxes) for q in sample])
            below = pts[pts[:, 2] < 0.0]
            r['min_z'] = float(pts[:, 2].min())
            r['below_datum_min_abs_y'] = float(np.abs(below[:, 1]).min()) if len(below) else None
            r['below_datum_min_abs_x'] = float(np.abs(below[:, 0]).min()) if len(below) else None
        out.append(r)
    return out


# ---- order and transits (--plan) ---------------------------------------------------

@functools.lru_cache(maxsize=1)
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


def order_views(options, home, vmax, amax):
    """Order to fly a stage's views, and the IK branch to fly each in, home to home, for the
    shortest estimated move time. options[i] = [(entry q, exit q), ...], one per branch.
    For a given order the best branches are a shortest path through a layered graph (one
    layer per view, one node per branch), solved exactly by dynamic programming. The order:
    nearest neighbour, then relocate / segment-reverse local search until nothing helps.
    Sweeps keep the plan's direction. Returns (order, branch index per position)."""
    n = len(options)
    nodes = [(i, b) for i in range(n) for b in range(len(options[i]))]
    at = {nb: k for k, nb in enumerate(nodes)}
    ent = [options[i][b][0] for i, b in nodes]
    ext = [options[i][b][1] for i, b in nodes]
    M = np.array([[move_estimate(x, e, vmax, amax) for e in ent] for x in ext])
    H0 = np.array([move_estimate(home, e, vmax, amax) for e in ent])
    H1 = np.array([move_estimate(x, home, vmax, amax) for x in ext])
    ks = [[at[(i, b)] for b in range(len(options[i]))] for i in range(n)]

    def solve(order):
        cost = H0[ks[order[0]]].copy()
        back = []
        for a, b in zip(order, order[1:]):
            step = cost[:, None] + M[np.ix_(ks[a], ks[b])]
            back.append(step.argmin(0))
            cost = step.min(0)
        cost = cost + H1[ks[order[-1]]]
        pick = [int(cost.argmin())]
        for bk in reversed(back):
            pick.append(int(bk[pick[-1]]))
        return float(cost.min()), pick[::-1]

    if n == 0:
        return [], []
    order, left = [], set(range(n))
    prev = None
    while left:
        def near(k):
            row = H0 if prev is None else M[ks[prev]].min(0)
            return row[ks[k]].min()
        i = min(left, key=near)
        order.append(i)
        left.remove(i)
        prev = i
    best, pick = solve(order)
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
                    cc, pc = solve(c)
                    if cc < best - 1e-9:
                        order, best, pick, improved = c, cc, pc, True
    return order, pick


def joint_move(cli, q0, q1, vmax, amax):
    """A synchronised joint-interpolated move (Yaskawa MOVJ): straight in joint space, every
    joint on the same normalised trapezoid so they start and stop together, at the slowest
    joint's limits. Checked for collisions every MOVJ_STEP rad of the joint that moves most.
    {'q', 't'}, or None if any checked state collides."""
    q0, q1 = np.array(q0, float), np.array(q1, float)
    d = q1 - q0
    D = np.abs(d).max()
    if D < 1e-9:
        return {'q': [list(q0)], 't': [0.0]}
    n = max(2, int(math.ceil(D / MOVJ_STEP)))
    for k in range(1, n):
        if not cli.is_valid(q0 + d * k / n):
            return None
    big = np.abs(d) > 1e-9
    v = min(np.array(vmax)[big] / np.abs(d[big]))      # path parameter s: 0 -> 1
    a = min(np.array(amax)[big] / np.abs(d[big]))
    if v * v / a >= 1.0:                                 # never reaches cruise: triangle
        v = math.sqrt(a)
    ta, sa = v / a, v * v / (2 * a)
    T = 2 * ta + (1 - 2 * sa) / v

    def time_at(s):
        if s <= sa:
            return math.sqrt(2 * s / a)
        if s >= 1 - sa:
            return T - math.sqrt(max(0.0, 2 * (1 - s) / a))
        return ta + (s - sa) / v

    return {'q': [list(q0 + d * k / n) for k in range(n + 1)], 't': [time_at(k / n) for k in range(n + 1)]}


def raised(cli, q, h):
    """q with the tool moved h straight up (module +z), same branch; None if no such state."""
    T = cli.fk_links(q, [TOOL])[TOOL]
    T[:3, 3] += h * cli.up
    q2 = cli.solve(T, q)
    if q2 is None or max(abs(a - b) for a, b in zip(q2, q)) > VIA_BRANCH or not cli.is_valid(q2):
        return None
    return q2


def via_move(cli, q0, q1, vmax, amax):
    """When the straight joint move collides: joint moves through one or two raised waypoints
    (above the start, above the end, both, or above the joint-space midpoint; or the home
    pose), the arm
    stopping at each, as a teach-pendant program would (MOVJ ... MOVJ). The quickest by
    estimate that is collision-free, or None."""
    mid = [(a + b) / 2 for a, b in zip(q0, q1)]
    cands = []
    for h in VIA_HEIGHTS:
        a, b, c = raised(cli, q0, h), raised(cli, q1, h), raised(cli, mid, h)
        for vias in ([a], [b], [c], [a, b]):
            if any(v is None for v in vias):
                continue
            pts = [q0] + vias + [q1]
            cands.append((sum(move_estimate(x, y, vmax, amax) for x, y in zip(pts, pts[1:])), pts))
    home = getattr(cli, 'home', None)
    if home is not None:                     # last: through the home pose, known clear
        a = raised(cli, q0, VIA_HEIGHTS[-1])
        b = raised(cli, q1, VIA_HEIGHTS[-1])
        for vias in ([home], [a, home, b]):
            if any(v is None for v in vias):
                continue
            pts = [q0] + vias + [q1]
            cands.append((sum(move_estimate(x, y, vmax, amax) for x, y in zip(pts, pts[1:])), pts))
    for _, pts in sorted(cands, key=lambda c: c[0]):
        legs = []
        for x, y in zip(pts, pts[1:]):
            legs.append(joint_move(cli, x, y, vmax, amax))
            if legs[-1] is None:
                break
        else:
            return chain(*legs)
    return None


def plan_leg(cli, q0, q1):
    """A joint move if it is collision-free (smooth, predictable, what a teach-pendant program
    does), else joint moves through raised waypoints, else OMPL (which detours and wiggles:
    sampled, then only shortcut)."""
    vmax, amax = joint_limits()
    leg = joint_move(cli, q0, q1, vmax, amax)
    if leg is not None:
        LEG_KIND['joint'] += 1
        leg['how'] = ['joint']
        return leg
    leg = via_move(cli, q0, q1, vmax, amax) if getattr(cli, 'up', None) is not None else None
    if leg is not None:
        LEG_KIND['via'] += 1
        leg['how'] = ['via']
        return leg
    for seconds, attempts in ((2.0, 3), (5.0, 3), (10.0, 6)):
        leg = cli.motion(q0, q1, seconds, attempts)
        if leg is not None:
            LEG_KIND['ompl'] += 1
            leg['how'] = ['ompl']
            return leg
    LEG_KIND['failed'] += 1
    return None


def lift_path(cli, q, T_mb):
    """Straight up (module +z) from q, as far as LIFT_MAX: the candidate depart path.
    Run with the transit floor removed; trimmed against it by trim_lift."""
    T = cli.fk_links(q, [TOOL])[TOOL]
    up = np.linalg.inv(T_mb)[:3, :3] @ np.array([0.0, 0.0, 1.0])
    _, pts = cli.line(q, tf(T[:3, :3], T[:3, 3] + LIFT_MAX * up), dense=True)
    return pts or [q]


def trim_lift(cli, pts):
    """With the transit floor in place: the lift up to the first state clear of it, plus
    2 cm. [q] if q is already clear; None if the lift never clears it."""
    if cli.is_valid(pts[0]):
        return pts[:1]
    for i in range(4, len(pts), 4):
        if cli.is_valid(pts[i]):
            j = min(len(pts) - 1, i + 4)
            return pts[:j + 1] if cli.is_valid(pts[j]) else pts[:i + 1]
    return None


def straight(pts):
    """A lift path as a timed leg: CART_STEP per point at APPROACH_SPEED."""
    return {'q': [list(q) for q in pts], 't': [k * CART_STEP / APPROACH_SPEED for k in range(len(pts))]}


def chain(*legs):
    """Join timed legs end to end; None if any is missing."""
    if any(leg is None for leg in legs):
        return None
    q, t, how = [], [], []
    for leg in legs:
        how += leg.get('how', [])
        if len(leg['q']) < 2:
            continue
        skip = 1 if q else 0
        off = t[-1] if t else 0.0
        q += leg['q'][skip:]
        t += [off + x - leg['t'][0] for x in leg['t'][skip:]]
    out = {'q': q, 't': t} if q else {'q': legs[0]['q'][:1], 't': [0.0]}
    if how:
        out['how'] = how          # how each planned (non-straight) part was made
    return out


def plan_sequences(cli, res_views, home, T_mb, floor):
    """Per stage: the flying order, the IK branch each view is flown in, and the timed moves
    between views, home to home. Each move = straight depart up out of the last view (if it
    is below the transit floor), a joint move (OMPL if that collides) with the wrist and
    sensors kept above the transit floor, and a straight approach down into the next view.
    Also home -> view -> home for every view, in its cheapest branch from home, so any single
    view or subset can be shown with checked moves (going through home)."""
    vmax, amax = joint_limits()
    flown = [v for v in res_views if v['ok']]
    cli.allow_slab_except_effector()
    raw = {v['id']: [(lift_path(cli, p[0], T_mb), lift_path(cli, p[-1], T_mb))
                     for p in v.get('branches', [v['path']])] for v in flown}
    cli.slab(T_mb, floor, True)
    cli.up = np.linalg.inv(T_mb)[:3, :3] @ np.array([0.0, 0.0, 1.0])   # module +z, in the base frame
    if not cli.is_valid(home):
        home = pick_home(cli)
    cli.home = home
    ups = {}                         # id -> [(branch index, approach lift, depart lift)]
    for v in flown:
        opts = []
        for b, (ra, rd) in enumerate(raw[v['id']]):
            a, d = trim_lift(cli, ra), trim_lift(cli, rd)
            if a is not None and d is not None:
                opts.append((b, a, d))
        if not opts:
            print(f"  {v['id']}: no straight approach clears the transit floor: not reachable",
                  flush=True)
            v['ok'] = False
            v['why'] = (f'no straight vertical approach/depart reaches {TRANSIT_FLOOR * 1000:.0f} mm '
                        'above the pallet top (transit floor)')
            continue
        ups[v['id']] = opts
        v['branches_ok'] = [b for b, _, _ in opts]
        v['approach_mm'] = 1000 * CART_STEP * (len(opts[0][1]) - 1)
        if 'branches' in v:
            v['path'] = v['branches'][opts[0][0]]
    flown = [v for v in flown if v['ok']]

    def path_of(v, o):
        return v.get('branches', [v['path']])[ups[v['id']][o][0]]

    def into(q_from, i_d, o):        # joint move to the approach point, then straight down
        up = ups[i_d][o][1]
        return chain(plan_leg(cli, q_from, up[-1]), straight(up[::-1]))

    def outof(i_d, o):               # straight up out of a view
        return straight(ups[i_d][o][2])

    solo = {}
    for v in flown:                  # each branch in turn, cheapest from home first
        for o in sorted(range(len(ups[v['id']])),
                        key=lambda k: move_estimate(home, ups[v['id']][k][1][-1], vmax, amax)
                        + move_estimate(ups[v['id']][k][2][-1], home, vmax, amax)):
            solo[v['id']] = {'in': into(home, v['id'], o),
                             'out': chain(outof(v['id'], o), plan_leg(cli, ups[v['id']][o][2][-1], home)),
                             'path': path_of(v, o)}
            if solo[v['id']]['in'] and solo[v['id']]['out']:
                break
    seqs = {}
    for stage in STAGES:
        vs = [v for v in flown if stage in v['stages']]
        options = [[(a[-1], d[-1]) for _, a, d in ups[v['id']]] for v in vs]
        idx, pick = order_views(options, home, vmax, amax)
        order = [vs[i] for i in idx]
        legs, q, prev = [], home, None
        for v, o in zip(order, pick):
            legs.append(chain(*([outof(*prev)] if prev else []), into(q, v['id'], o)))
            q, prev = ups[v['id']][o][2][-1], (v['id'], o)
        legs.append(chain(outof(*prev), plan_leg(cli, q, home)) if prev else None)
        move = sum(leg['t'][-1] for leg in legs if leg)
        capture = sum(v['capture_s'] for v in order)
        seqs[stage] = {'order': [v['id'] for v in order], 'legs': legs,
                       'branch': {v['id']: ups[v['id']][o][0] for v, o in zip(order, pick)},
                       'paths': {v['id']: path_of(v, o) for v, o in zip(order, pick)},
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
    print(f"  moves: {LEG_KIND['joint']} joint moves, {LEG_KIND['via']} via raised waypoints, {LEG_KIND['ompl']} OMPL, "
          f"{LEG_KIND['failed']} failed", flush=True)
    return seqs, solo, home


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
    ap.add_argument('--conveyor-lift', type=float, default=None, metavar='M',
                    help='add the placeholder conveyor along the module, the pallet lifted M off '
                         'its belt at the station (e.g. 0.05); default: no conveyor')
    ap.add_argument('--keep-paths', action='store_true',
                    help='store every view\'s joint path, for show_view_plan.py')
    ap.add_argument('--plan', action='store_true',
                    help='one --base only: also order each stage and plan every move (implies '
                         '--keep-paths, no turntable)')
    ap.add_argument('--no-launch', action='store_true', help='use an already running move_group')
    a = ap.parse_args()

    global CONVEYOR_LIFT
    CONVEYOR_LIFT = a.conveyor_lift
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

    rclpy.init()
    if not a.no_launch:
        # A second move_group on this domain answers some of the requests with its own scene
        # (seen 18 Nov: leftovers from killed runs). Refuse rather than mix them.
        probe = rclpy.create_node('view_reach_probe')
        end = time.time() + 3.0
        while time.time() < end:
            rclpy.spin_once(probe, timeout_sec=0.2)
        others = [n for n in probe.get_node_names() if n == 'move_group']
        probe.destroy_node()
        if others:
            rclpy.try_shutdown()
            sys.exit(f'a move_group is already running on ROS_DOMAIN_ID='
                     f'{os.environ.get("ROS_DOMAIN_ID", "0")}: stop it first (or use --no-launch)')
    proc = None if a.no_launch else start_move_group(out / 'move_group.log')
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
                entry['sequences'], entry['solo'], entry['home'] = plan_sequences(
                    cli, res, home, T_mb, a.floor)
                entry['ok'] = sum(r['ok'] for r in res)      # the approach check may drop views
                entry['stages'] = {s: (sum(r['ok'] for r in res if s in r['stages']),
                                       sum(1 for r in res if s in r['stages'])) for s in STAGES}
                entry['transit_floor'] = TRANSIT_FLOOR
                entry['approach_speed'] = APPROACH_SPEED
                entry['planner'] = 3      # 18 Nov: branches chosen per sequence; joint moves, then via raised waypoints, then OMPL
                entry['legs_by'] = dict(LEG_KIND)
            json.dump({'floor': a.floor, 'run_up': RUN_UP, 'module': MODULE, 'pad': a.pad,
                       'conveyor_lift': a.conveyor_lift,
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

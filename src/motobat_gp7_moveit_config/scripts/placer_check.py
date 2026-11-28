#!/usr/bin/python3
"""
Acceptance check for Station 1 (GP7, cell placing) through move_group. (task 3.1)

A. Every task pose, in build order. The module is built up as the GP7 builds it: end plate
   A, cells 1-12, end plate B, each part added to the planning scene once placed. For each
   part, its four poses (above the magazine pick, pick, above the place, place) must have a
   collision-free IK solution from /compute_ik (the OPW plugin, seeded at home) with the
   part attached to the TCP, against the robot itself, the station's fixtures (conveyor,
   gate, pedestal, magazines), the pallet, the cooling plate and every part already placed.
   Gripper yaw 0 or 180 deg. This is where the 24 mm gripper has to fit between neighbours.

B. One full cycle, executed: home (100 mm above the cell magazine's pick) -> LIN down -> attach
   cell 1 -> LIN up -> PTP above its place -> LIN down -> detach -> LIN up -> PTP home.
   Pilz PTP / LIN (Yaskawa MOVJ / MOVL) through MoveGroup -> FollowJointTrajectory ->
   placer_arm_controller -> mock hardware. Every planned trajectory point is re-checked with
   /check_state_validity; every move must end within 1 mm / 0.1 deg of its target.
   Reports the cycle at full speed (velocity and acceleration scaling 1.0) on the assumed
   accelerations (joint_limits.yaml, A).

Modelling choices (stated, not hidden): parts are boxes from the CAD bounds, 0.5 mm undersize
per side and 1 mm short (held: at the bottom; placed: at the top), so resting contact (a cell
against end plate A, a part on its magazine, the gripper on the part it just let go) does not
count as a collision. Interface pads (1.5 mm) are left out.

    ros2 launch motobat_gp7_moveit_config demo.launch.py use_rviz:=false
    ros2 run motobat_gp7_moveit_config placer_check.py          # --debug: trajectory end points
"""
import math
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume, CollisionObject, Constraints,
                             JointConstraint, OrientationConstraint, PlanningScene, PositionConstraint, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

DESC = Path(get_package_share_directory('motobat_description'))
# layout_check (task poses, magazines) lives beside this package in the source tree; the
# symlink install resolves back to it.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'motobat_description' / 'scripts'))
import layout_check as LC  # noqa: E402

GROUP, TIP = 'placer_arm', 's1_tcp'
JOINTS = [f's1_joint_{j}' for j in ('1_s', '2_l', '3_u', '4_r', '5_b', '6_t')]
HOME = [1.8925, 0.009, 0.1467, 0.0, -1.7085, -0.3218]
TOUCH = ['s1_gripper', 's1_tcp']
SHRINK, SHORT = 0.0005, 0.001                     # m: attached part undersize per side, short at the bottom


def quat_down(yaw):
    """Quaternion of LC.down(yaw): tool z straight down, tool x at yaw in the floor plane.
    down(yaw) = Rz(yaw) * Rx(pi): q = (cos(yaw/2) * x + sin(yaw/2) * y) * 1 with w = 0."""
    return (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)   # x, y, z, w


def box(name, size, xyz, frame='world', q=(0.0, 0.0, 0.0, 1.0)):
    o = CollisionObject(id=name, operation=CollisionObject.ADD)
    o.header.frame_id = frame
    o.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))]
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    o.primitive_poses = [p]
    return o


def placed_box(name, size, place):
    """A part in the module: top 1 mm below the TCP that placed it, 0.5 mm undersize per side."""
    sx, sy, sz = size
    h = sz - SHORT
    return box(name, (sx - 2 * SHRINK, sy - 2 * SHRINK, h), (place[0], place[1], place[2] - SHORT - h / 2))


class Check(Node):
    def __init__(self):
        super().__init__('placer_check')
        self.js = None
        self.create_subscription(JointState, '/joint_states', lambda m: setattr(self, 'js', m), 10)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.valid = self.create_client(GetStateValidity, '/check_state_validity')
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        for c in (self.ik, self.fk, self.valid, self.scene):
            if not c.wait_for_service(timeout_sec=60):
                sys.exit(f'{c.srv_name} not available: is move_group running?')
        if not self.move.wait_for_server(timeout_sec=60):
            sys.exit('/move_action not available')

    def call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def joints(self):
        self.js = None
        end = time.time() + 10
        while self.js is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in JOINTS]

    def apply(self, world=(), remove=(), attach=None, detach=None):
        """One planning-scene diff. A detach drops the body into the world where it is (MoveIt's
        rule), so it is removed from the world right after; callers add the placed part back."""
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.world.collision_objects = list(world) + [
            CollisionObject(id=n, operation=CollisionObject.REMOVE) for n in remove]
        if attach is not None:
            ps.robot_state.attached_collision_objects = [attach]
        if detach is not None:
            a = AttachedCollisionObject(link_name=TIP)
            a.object = CollisionObject(id=detach, operation=CollisionObject.REMOVE)
            ps.robot_state.attached_collision_objects = [a]
        if not self.call(self.scene, ApplyPlanningScene.Request(scene=ps)).success:
            sys.exit('apply_planning_scene failed')
        if detach is not None:
            self.apply(remove=[detach])

    def attached(self, name, part):
        """The part hanging under the TCP (tool z points down, into the part)."""
        sx, sy, sz = part
        a = AttachedCollisionObject(link_name=TIP, touch_links=TOUCH)
        a.object = box(name, (sx - 2 * SHRINK, sy - 2 * SHRINK, sz - SHORT), (0, 0, (sz - SHORT) / 2), frame=TIP)
        return a

    def solve(self, xyz, yaws, seed):
        for yaw in yaws:
            req = GetPositionIK.Request()
            r = req.ik_request
            r.group_name, r.ik_link_name, r.avoid_collisions = GROUP, TIP, True
            r.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=list(seed)), is_diff=True)
            r.pose_stamped.header.frame_id = 'world'
            p = r.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat_down(yaw)
            res = self.call(self.ik, req)
            if res.error_code.val == 1:
                d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [d[j] for j in JOINTS], yaw
        return None, None

    def tcp(self, q, with_quat=False):
        req = GetPositionFK.Request(fk_link_names=[TIP], robot_state=RobotState(
            joint_state=JointState(name=JOINTS, position=list(q))))
        req.header.frame_id = 'world'
        p = self.call(self.fk, req).pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z])
        if with_quat:
            return xyz, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return xyz

    def state_valid(self, q):
        req = GetStateValidity.Request(group_name=GROUP)
        req.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=list(q)), is_diff=True)
        return self.call(self.valid, req).valid

    def go(self, xyz, yaw, planner, pipeline='pilz_industrial_motion_planner'):
        """Pilz PTP or LIN (or an OMPL planner) to a TCP pose, executed.
        Returns (ok, seconds, end error mm, invalid points)."""
        target = Pose()
        target.position.x, target.position.y, target.position.z = (float(v) for v in xyz)
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = quat_down(yaw)
        pc = PositionConstraint(link_name=TIP, weight=1.0, constraint_region=BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-4])], primitive_poses=[target]))
        pc.header.frame_id = 'world'
        oc = OrientationConstraint(link_name=TIP, orientation=target.orientation, weight=1.0,
                                   absolute_x_axis_tolerance=1e-3, absolute_y_axis_tolerance=1e-3,
                                   absolute_z_axis_tolerance=1e-3)
        oc.header.frame_id = 'world'
        g = MoveGroup.Goal()
        r = g.request
        r.group_name, r.pipeline_id, r.planner_id = GROUP, pipeline, planner
        r.num_planning_attempts, r.allowed_planning_time = 1, 5.0
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = 1.0
        r.goal_constraints = [Constraints(position_constraints=[pc], orientation_constraints=[oc])]
        g.planning_options.plan_only = False
        fut = self.move.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        res = rf.result().result
        pts = res.planned_trajectory.joint_trajectory.points
        if res.error_code.val != 1 or not pts:
            return False, 0.0, float('nan'), 0
        names = res.planned_trajectory.joint_trajectory.joint_names
        idx = [names.index(j) for j in JOINTS]
        bad = sum(not self.state_valid([p.positions[i] for i in idx]) for p in pts)
        self.travel = np.degrees(np.abs(np.array(pts[-1].positions)[idx] - np.array(pts[0].positions)[idx]))
        if '--debug' in sys.argv:
            print('     start', np.round(np.array(pts[0].positions)[idx], 4).tolist(), ' end',
                  np.round(np.array(pts[-1].positions)[idx], 4).tolist(), ' max |q6| step',
                  round(max(abs(b.positions[idx[5]] - a.positions[idx[5]]) for a, b in zip(pts, pts[1:])), 4),
                  ' points', len(pts))
        dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9
        end = time.time() + 1.0
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        got, quat = self.tcp(self.joints(), with_quat=True)
        err = np.linalg.norm(got - np.array(xyz)) * 1e3
        want = np.array(quat_down(yaw))
        self.rot_err = math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(quat, want))))))
        return bad == 0 and err < 1.0 and self.rot_err < 0.1, dur, err, bad


def station():
    """S1 pallet stop x and the GP7 base offset d, from the expanded station xacro."""
    with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
        subprocess.run(['xacro', str(DESC / 'urdf/placer_station.urdf.xacro'), '-o', f.name], check=True)
        _, mounts = LC.urdf_fixtures_and_mounts(f.name)
    xyz, _ = mounts['s1_']
    return xyz[0], -xyz[1]


def main():
    rclpy.init()
    n = Check()
    x0, d = station()
    top = LC.PALLET_TOP
    ok = True

    # pallet (20 mm plate, 25 mm margin round the module) and the cooling plate (CAD 355 x 151 x 4)
    n.apply(world=[box('pallet', (0.405, 0.201, 0.020), (x0, 0.0, top - 0.010)),
                   box('cooling_plate', (0.355, 0.151, 0.004), (x0, 0.0, top + 0.002))])
    if not n.state_valid(HOME):
        sys.exit('home is in collision with the scene')

    # ---- A: every pose in build order ----
    print('A. IK (OPW, collision-checked by move_group) for every pick and place, in build order')
    placed, misses, t_ik = [], 0, []
    for name, (x, y, z), yaws, part, _ in LC.tasks('gp7'):
        size = LC.PARTS[part]
        mx, my, mz = LC.ASSUMED['magazine'][part]
        pick = (x0 + mx, -d + my, top + mz)
        place = (x0 + x, y, top + z)
        up = LC.ASSUMED['approach']
        n.apply(attach=n.attached('held', size))
        row = []
        for label, p in (('pick_above', (pick[0], pick[1], pick[2] + up)), ('pick', pick),
                         ('place_above', (place[0], place[1], place[2] + up)), ('place', place)):
            t = time.perf_counter()
            q, yaw = n.solve(p, yaws, HOME)
            t_ik.append(time.perf_counter() - t)
            row.append(f'{label} {"ok" if q else "NONE"}' + (f' (yaw {math.degrees(yaw):.0f})' if q else ''))
            misses += q is None
        n.apply(detach='held')
        # the part is now in the module
        n.apply(world=[placed_box(name, size, place)])
        placed.append(name)
        print(f'  {name:12s} ' + ', '.join(row))
    ok &= misses == 0
    t_ik = np.array(t_ik) * 1e3
    print(f'  {4 * len(placed) - misses} of {4 * len(placed)} poses solved; /compute_ik round trip median '
          f'{np.median(t_ik):.1f} ms (both yaws tried on a miss)  {"ok" if misses == 0 else "FAIL"}')

    # ---- B: one executed cycle for cell 1 (end plate A is already in the scene from A; reset the rest) ----
    n.apply(remove=[nm for nm in placed if nm != 'end_plate_A'])
    print('B. One executed cycle, cell 1, Pilz PTP / LIN at full speed (accelerations A)')
    name, (x, y, z), yaws, part, _ = [t for t in LC.tasks('gp7') if t[0] == 'cell_1'][0]
    size = LC.PARTS[part]
    mx, my, mz = LC.ASSUMED['magazine'][part]
    pick, place = (x0 + mx, -d + my, top + mz), (x0 + x, y, top + z)
    up = LC.ASSUMED['approach']
    above = lambda p: (p[0], p[1], p[2] + up)  # noqa: E731
    yaw = math.pi                                  # home's yaw (initial_positions.yaml): no wrist turn
    # home is above the cell magazine's pick (initial_positions.yaml), so the cycle runs home to home
    steps = [('LIN to pick', pick, 'LIN', None),
             ('LIN up with cell', above(pick), 'LIN', 'attach'), ('PTP to place_above', above(place), 'PTP', None),
             ('LIN to place', place, 'LIN', None), ('LIN up, cell left', above(place), 'LIN', 'detach')]
    total = 0.0
    for label, p, planner, action in steps:
        if action == 'attach':
            n.apply(attach=n.attached('cell_1', size))
        if action == 'detach':
            n.apply(detach='cell_1')
            n.apply(world=[placed_box('cell_1', size, place)])
        good, dur, err, bad = n.go(p, yaw, planner)
        total += dur
        ok &= good
        print(f'  {label:20s} {planner}  {dur:5.2f} s   end error {err:6.3f} mm   '
              f'{bad} invalid planned states, end rotation error {n.rot_err:.3f} deg  {"ok" if good else "FAIL"}   joint travel (deg) '
              f'{np.round(getattr(n, "travel", np.zeros(6)), 1).tolist()}')
    # back home by joints
    g = MoveGroup.Goal()
    r = g.request
    r.group_name, r.pipeline_id, r.planner_id = GROUP, 'pilz_industrial_motion_planner', 'PTP'
    r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = 1.0
    r.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=j, position=v, tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0)
        for j, v in zip(JOINTS, HOME)])]
    fut = n.move.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut)
    rf = fut.result().get_result_async()
    rclpy.spin_until_future_complete(n, rf)
    res = rf.result().result
    pts = res.planned_trajectory.joint_trajectory.points
    dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9 if pts else 0.0
    total += dur
    jerr = max(abs(a - b) for a, b in zip(n.joints(), HOME))
    good = res.error_code.val == 1 and jerr < math.radians(0.1)
    ok &= good
    print(f'  {"PTP home":20s} PTP  {dur:5.2f} s   end error {math.degrees(jerr):6.3f} deg  {"ok" if good else "FAIL"}')
    print(f'  one cell, home to home: {total:.2f} s of motion (no gripper open/close time; accelerations A)')

    n.apply(remove=['pallet', 'cooling_plate', 'cell_1', 'end_plate_A'])
    print('PASS' if ok else 'FAIL')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

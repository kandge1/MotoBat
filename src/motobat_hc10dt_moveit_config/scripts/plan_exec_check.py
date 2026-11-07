#!/usr/bin/python3
"""
Acceptance check for the Inspector's control path. (tasks 2.1, 2.2)

Plans and executes two moves through move_group's MoveGroup action, so the
whole chain runs: MoveIt -> FollowJointTrajectory -> inspector_arm_controller
-> hardware (mock or Gazebo) -> /joint_states.

  1. OMPL joint-space move from wherever the arm is to a fixed joint goal
     (the Yaskawa analogue is MOVJ)
  2. Pilz LIN: tool0 straight down 10 cm with orientation held (MOVL)

Passes if both succeed, the arm ends within 0.1 deg of the joint goal, and
the LIN move ends within 1 mm of its target. Also reports the controller's
worst tracking error during each move, which is what the Gazebo
position gain trades against.

    ros2 launch motobat_hc10dt_moveit_config demo.launch.py use_rviz:=false     # mock
    ros2 launch motobat_bringup inspector_gazebo.launch.py gui:=false rviz:=false  # Gazebo
    ros2 run motobat_hc10dt_moveit_config plan_exec_check.py
"""
import math
import sys
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, OrientationConstraint,
                             PositionConstraint, RobotState)
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

GROUP = 'inspector_arm'
TIP = 's3_tool0'
JOINTS = ['s3_joint_1_s', 's3_joint_2_l', 's3_joint_3_u',
          's3_joint_4_r', 's3_joint_5_b', 's3_joint_6_t']
JOINT_GOAL = [0.6, 0.3, 1.9, 0.2, -1.2, 0.5]   # rad; collision-free, well inside limits
LIN_DROP = 0.10                                # m


class Check(Node):
    def __init__(self):
        super().__init__('plan_exec_check')
        self.js = None
        self.track_err = 0.0
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/inspector_arm_controller/controller_state', self._ctrl, 10)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        if not self.move.wait_for_server(timeout_sec=60) or not self.fk.wait_for_service(timeout_sec=60):
            sys.exit('move_group not available')

    def _js(self, m):
        self.js = m

    def _ctrl(self, m):
        if m.error.positions:
            self.track_err = max(self.track_err, max(abs(e) for e in m.error.positions))

    def joints(self):
        self.js = None
        end = time.time() + 10
        while self.js is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.js is None:
            sys.exit('no /joint_states')
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in JOINTS]

    def settle(self, seconds=1.0):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def tool(self, q):
        req = GetPositionFK.Request(fk_link_names=[TIP],
                                    robot_state=RobotState(joint_state=JointState(name=JOINTS, position=q)))
        req.header.frame_id = 'world'
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().pose_stamped[0].pose

    def execute(self, goal, pipeline, planner):
        g = MoveGroup.Goal()
        r = g.request
        r.group_name, r.pipeline_id, r.planner_id = GROUP, pipeline, planner
        r.num_planning_attempts, r.allowed_planning_time = 1, 5.0
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = 0.2
        r.goal_constraints = [goal]
        g.planning_options.plan_only = False
        self.track_err = 0.0
        fut = self.move.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        res_fut = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result().result
        traj = res.planned_trajectory.joint_trajectory.points
        duration = traj[-1].time_from_start.sec + traj[-1].time_from_start.nanosec * 1e-9 if traj else 0.0
        return res.error_code.val, duration


def main():
    rclpy.init()
    n = Check()
    ok = True
    q0 = n.joints()
    print(f'start (deg): {[round(math.degrees(v), 1) for v in q0]}')

    goal = Constraints(joint_constraints=[
        JointConstraint(joint_name=j, position=v, tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0)
        for j, v in zip(JOINTS, JOINT_GOAL)])
    code, dur = n.execute(goal, 'ompl', 'RRTConnectkConfigDefault')
    n.settle()
    err = max(abs(a - b) for a, b in zip(n.joints(), JOINT_GOAL))
    good = code == 1 and err < math.radians(0.1)
    ok &= good
    print(f'OMPL joint move   code {code}, {dur:.2f} s, final joint error {math.degrees(err):.3f} deg, '
          f'worst tracking error {math.degrees(n.track_err):.3f} deg  {"ok" if good else "FAIL"}')

    p0 = n.tool(n.joints())
    target = Pose(orientation=p0.orientation)
    target.position.x, target.position.y, target.position.z = p0.position.x, p0.position.y, p0.position.z - LIN_DROP
    pc = PositionConstraint(link_name=TIP, weight=1.0, constraint_region=BoundingVolume(
        primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-4])], primitive_poses=[target]))
    pc.header.frame_id = 'world'
    oc = OrientationConstraint(link_name=TIP, orientation=target.orientation, weight=1.0,
                               absolute_x_axis_tolerance=1e-3, absolute_y_axis_tolerance=1e-3,
                               absolute_z_axis_tolerance=1e-3)
    oc.header.frame_id = 'world'
    code, dur = n.execute(Constraints(position_constraints=[pc], orientation_constraints=[oc]),
                          'pilz_industrial_motion_planner', 'LIN')
    n.settle()
    p1 = n.tool(n.joints())
    d = [p1.position.x - target.position.x, p1.position.y - target.position.y, p1.position.z - target.position.z]
    miss = math.sqrt(sum(v * v for v in d))
    good = code == 1 and miss < 1e-3
    ok &= good
    print(f'Pilz LIN -{LIN_DROP * 100:.0f} cm   code {code}, {dur:.2f} s, final tool0 error {miss * 1e3:.3f} mm, '
          f'worst tracking error {math.degrees(n.track_err):.3f} deg  {"ok" if good else "FAIL"}')

    print('PASS' if ok else 'FAIL')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

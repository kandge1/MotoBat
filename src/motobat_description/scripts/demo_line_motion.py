#!/usr/bin/env python3
"""
MotoBat — scripted motion demo for the four-station line.

Publishes /joint_states directly to sweep all four arms through a
station-appropriate cycle. Run this INSTEAD of joint_state_publisher_gui:

    ros2 launch motobat_description view_line.launch.py gui:=false
    ros2 run motobat_description demo_line_motion.py

WHAT THIS IS: a kinematic visualisation. Joint angles are interpolated
directly, with no dynamics, no collision checking, and no controllers.

WHAT THIS IS NOT: how the real system will work. The proposal's architecture
drives arms through MoveIt2 -> FollowJointTrajectory. This node bypasses all
of that to give you something moving on day one. Treat it as a smoke test for
the descriptions and TF tree, then delete it once real controllers exist.
"""
import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS: List[str] = ['joint_1_s', 'joint_2_l', 'joint_3_u',
                     'joint_4_r', 'joint_5_b', 'joint_6_t']

STATIONS = ['s1', 's2', 's3', 's4']

# Per-station motion signature. Amplitudes in radians, kept well inside the
# real joint limits so nothing clips visually. Phase offsets stagger the
# stations so the line reads as a pipeline rather than four arms twitching
# in unison.
PROFILES: Dict[str, dict] = {
    # Stacker: short, repetitive pick-place arc. Tight amplitude, quick cycle.
    's1': {'amp': [0.55, 0.30, 0.35, 0.0, 0.40, 0.9], 'freq': 0.30, 'phase': 0.0},
    # Welder: smaller travel, slower -- a weld pass dwells.
    's2': {'amp': [0.35, 0.22, 0.28, 0.0, 0.30, 0.5], 'freq': 0.22, 'phase': 1.2},
    # Inspector: wide, sweeping multi-angle scan. This is the HC10 doing what
    # Section 2.1 argues a collaborative arm is for -- exploratory sensing,
    # not repeatable path-following. Largest amplitude, slowest sweep.
    's3': {'amp': [1.20, 0.45, 0.55, 0.8, 0.70, 1.4], 'freq': 0.15, 'phase': 2.4},
    # Sealer: deliberate press-and-hold motion.
    's4': {'amp': [0.45, 0.35, 0.40, 0.0, 0.35, 0.6], 'freq': 0.25, 'phase': 3.6},
}


class LineMotionDemo(Node):
    def __init__(self) -> None:
        super().__init__('motobat_line_motion_demo')

        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('speed', 1.0)

        self.rate_hz = self.get_parameter('rate_hz').value
        self.speed = self.get_parameter('speed').value

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.names = [f'{s}_{j}' for s in STATIONS for j in JOINTS]
        self.t = 0.0
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f'Publishing {len(self.names)} joints at {self.rate_hz} Hz '
            f'(speed={self.speed}x). Set gui:=false on the launch file.')

    def _tick(self) -> None:
        self.t += self.speed / self.rate_hz

        positions: List[float] = []
        for station in STATIONS:
            p = PROFILES[station]
            theta = 2.0 * math.pi * p['freq'] * self.t + p['phase']
            for i, amp in enumerate(p['amp']):
                # Slight per-joint phase spread so the arm articulates through
                # the motion instead of every joint peaking together.
                positions.append(amp * math.sin(theta + i * 0.4))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = positions
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = LineMotionDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

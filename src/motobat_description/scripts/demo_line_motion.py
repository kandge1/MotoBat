#!/usr/bin/python3
"""
MotoBat line in RViz: every robot replays its real motion, in a loop. (task 3.0)

  s1_            GP7: end plate A, 12 cells, end plate B, from the magazines
  s4a_           GP4: busbar carrier
  s4b_           GP25: 24 weld holds
  s5_            GP4: CMC, then cover
      the task sequences layout_check.py placed each robot with (closed-form IK, nearest
      branch, joint moves; 1 s per move here, a visual pace, not a cycle time)
  s3a_/s3b_/s3c_ HC10DT: 2.5's base-A plan for its stage (pre-busbar / pre-weld / post-weld),
      moves at their planned timing, sweeps at 50 points per second

Kinematics only (no collision checking here: layout_check.py --verify did that). Run INSTEAD
of joint_state_publisher_gui:

    ros2 launch motobat_description view_line.launch.py gui:=false
    /usr/bin/python3 src/motobat_description/scripts/demo_line_motion.py

The floor robots' poses are re-solved from their mounts in the expanded motobat_line xacro
(layout_check.Station), so the demo follows a re-run layout.
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
import layout_check as LC  # noqa: E402

JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
REPO = Path(__file__).resolve().parents[3]
FLOOR = [('s1_', 'gp7'), ('s4a_', 'gp4a'), ('s4b_', 'gp25'), ('s5_', 'gp4b')]   # prefix, layout_check station
INSPECTORS = [('s3a_', 'pre_busbar'), ('s3b_', 'pre_weld'), ('s3c_', 'post_weld')]
MOVE_S = 1.0          # s per floor-robot move (visual)
SWEEP_HZ = 50.0       # sweep points per second


def floor_track(station, d, h):
    st = LC.Station(station)
    ev, missing = st.evaluate(d, h)
    if ev is None:
        raise SystemExit(f'{station}: pose {missing} unreachable from d {d}, h {h}')
    chain = list(ev['chain']) + [ev['chain'][0]]          # loop back to the start
    t = np.arange(len(chain)) * MOVE_S
    return t, np.array(chain)


def inspector_track(reach, stage):
    s = reach['sequences'][stage]
    ts, qs, now = [], [], 0.0
    for i, leg in enumerate(s['legs']):
        lt = np.array(leg['t'], float)
        ts += list(now + lt - lt[0])
        qs += leg['q']
        now = ts[-1]
        if i < len(s['order']):
            path = s['paths'][s['order'][i]]
            ts += list(now + (np.arange(len(path)) + 1) / SWEEP_HZ)
            qs += path
            now = ts[-1]
    return np.array(ts), np.array(qs)


class LineMotionDemo(Node):
    def __init__(self):
        super().__init__('motobat_line_motion_demo')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('speed', 1.0)
        self.rate_hz = self.get_parameter('rate_hz').value
        self.speed = self.get_parameter('speed').value
        reach = json.loads((REPO / 'data/reach/base_A/reach.json').read_text())['results'][0]
        from ament_index_python.packages import get_package_share_directory
        xacro_in = Path(get_package_share_directory('motobat_description')) / 'urdf/motobat_line.urdf.xacro'
        with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
            subprocess.run(['xacro', str(xacro_in), '-o', f.name], check=True)
            _, mounts = LC.urdf_fixtures_and_mounts(f.name)
        self.tracks = {}
        for prefix, station in FLOOR:
            xyz, _ = mounts[prefix]
            self.tracks[prefix] = floor_track(station, -xyz[1], xyz[2])
        for prefix, stage in INSPECTORS:
            self.tracks[prefix] = inspector_track(reach, stage)
        self.names = [p + j for p in self.tracks for j in JOINTS]
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / self.rate_hz, self.tick)
        for p, (t, _) in self.tracks.items():
            self.get_logger().info(f'{p:5s} {t[-1]:6.1f} s loop')

    def tick(self):
        el = (self.get_clock().now() - self.t0).nanoseconds * 1e-9 * self.speed
        pos = []
        for t, q in self.tracks.values():
            x = el % t[-1]
            pos += [float(np.interp(x, t, q[:, j])) for j in range(6)]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = self.names, pos
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LineMotionDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

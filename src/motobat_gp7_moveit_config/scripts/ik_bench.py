#!/usr/bin/python3
"""
OPW against KDL on the GP7, through move_group, on identical targets. (task 3.1)

For each solver config in config/ik_candidates/ this starts move_group with that solver
alone (launch/ik_compare.launch.py), confirms which plugin loaded, and sends the same
/compute_ik requests:

  targets  N random joint states inside the URDF limits that /check_state_validity calls
           collision-free (robot, gripper, conveyor, gate, pedestal, magazines), converted
           to s1_tcp poses by /compute_fk under the first solver. Saved, so every solver
           sees the same poses.
  request  seeded at home, avoid_collisions on, the config's own timeout.

Reports, per solver: solved / reachable (a target is reachable because a valid state put
the TCP there), /compute_ik round-trip time (median, p95; service overhead included), and
how far the answer is from the seed in joint space (max over joints). A short answer
matters: the next move goes from the seed to it.

    ros2 run motobat_gp7_moveit_config ik_bench.py --samples 1000 --out data/ik_bench_gp7

One move_group at a time; the script refuses to start if one is already running on this
ROS_DOMAIN_ID's machine. System Python 3.12.
"""
import argparse
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
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'motobat_description' / 'scripts'))
import layout_check as LC  # noqa: E402

PKG, GROUP, TIP = 'motobat_gp7_moveit_config', 'placer_arm', 's1_tcp'
JOINTS = [f's1_joint_{j}' for j in ('1_s', '2_l', '3_u', '4_r', '5_b', '6_t')]
HOME = [1.8925, 0.009, 0.1467, 0.0, -1.7085, -0.3218]
SOLVERS = ['opw', 'kdl']


class Client(Node):
    def __init__(self):
        super().__init__('ik_bench')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.valid = self.create_client(GetStateValidity, '/check_state_validity')
        self.params = self.create_client(GetParameters, '/move_group/get_parameters')

    def ready(self, timeout=60):
        return all(c.wait_for_service(timeout_sec=timeout) for c in (self.ik, self.fk, self.valid, self.params))

    def call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        return fut.result()

    def solver(self):
        n = f'robot_description_kinematics.{GROUP}.kinematics_solver'
        r = self.call(self.params, GetParameters.Request(names=[n]))
        return r.values[0].string_value if r and r.values else None

    def is_valid(self, q):
        req = GetStateValidity.Request(group_name=GROUP)
        req.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=list(q)))
        return self.call(self.valid, req).valid

    def pose(self, q):
        req = GetPositionFK.Request(fk_link_names=[TIP], robot_state=RobotState(
            joint_state=JointState(name=JOINTS, position=list(q))))
        req.header.frame_id = 'world'
        p = self.call(self.fk, req).pose_stamped[0].pose
        return [p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

    def solve(self, pose):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name, r.ik_link_name, r.avoid_collisions = GROUP, TIP, True
        r.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=HOME))
        r.pose_stamped.header.frame_id = 'world'
        p = r.pose_stamped.pose
        (p.position.x, p.position.y, p.position.z,
         p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w) = pose
        t = time.perf_counter()
        res = self.call(self.ik, req)
        dt = (time.perf_counter() - t) * 1e3
        if res is None or res.error_code.val != 1:
            return None, dt
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [d[j] for j in JOINTS], dt


def start(kin, log):
    return subprocess.Popen(['ros2', 'launch', PKG, 'ik_compare.launch.py', f'kinematics:={kin}'],
                            stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)


def stop(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--samples', type=int, default=1000)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', type=Path, default=Path('data/ik_bench_gp7'))
    a = ap.parse_args()
    running = subprocess.run(['pgrep', '-f', 'moveit_ros_move_group/move_group'], capture_output=True, text=True).stdout
    if running.strip():
        sys.exit(f'a move_group is already running (pids {running.split()}): stop it first')
    a.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Client()
    st = LC.Station('gp7')
    rng = np.random.default_rng(a.seed)
    targets, results = None, {}
    for name in SOLVERS:
        proc = start(f'config/ik_candidates/{name}.yaml', a.out / f'move_group_{name}.log')
        try:
            if not node.ready():
                sys.exit(f'{name}: move_group did not come up (see {a.out}/move_group_{name}.log)')
            loaded = node.solver()
            print(f'== {name}: move_group loaded {loaded}')
            if targets is None:
                qs, tried = [], 0
                while len(qs) < a.samples:
                    q = rng.uniform(st.lo, st.hi)
                    tried += 1
                    if node.is_valid(q):
                        qs.append(q.tolist())
                targets = [dict(q=q, pose=node.pose(q)) for q in qs]
                (a.out / 'targets.json').write_text(json.dumps(targets))
                print(f'   {len(targets)} collision-free targets ({tried} random states drawn)')
            times, dist, solved = [], [], 0
            for t in targets:
                q, dt = node.solve(t['pose'])
                times.append(dt)
                if q is not None:
                    solved += 1
                    dist.append(max(abs(x - y) for x, y in zip(q, HOME)))
            times = np.array(times)
            results[name] = dict(solver=loaded, solved=solved, n=len(targets),
                                 median_ms=float(np.median(times)), p95_ms=float(np.quantile(times, 0.95)),
                                 seed_dist_median_deg=float(np.degrees(np.median(dist))) if dist else None)
            r = results[name]
            print(f'   solved {solved}/{len(targets)}, /compute_ik {r["median_ms"]:.2f} ms median, '
                  f'{r["p95_ms"]:.2f} ms p95, answer {r["seed_dist_median_deg"]:.1f} deg from the seed (median, max joint)')
        finally:
            stop(proc)
            time.sleep(2)
    (a.out / 'results.json').write_text(json.dumps(results, indent=1))
    print(f'\n| solver | solved / reachable | /compute_ik median / p95 (ms) | answer from seed, median max-joint (deg) |')
    print('|---|---|---|---|')
    for name, r in results.items():
        print(f'| {name} | {r["solved"]} / {r["n"]} | {r["median_ms"]:.2f} / {r["p95_ms"]:.2f} | '
              f'{r["seed_dist_median_deg"]:.1f} |')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

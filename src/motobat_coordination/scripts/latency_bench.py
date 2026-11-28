#!/usr/bin/python3
"""Camera-image transport between two processes: RMW x QoS x message size. (task 3.6, plan §4A.5, §5)

    latency_bench.py                       the whole matrix (~3 min), one table
    latency_bench.py --role pub|sub ...    (used by the matrix: one process each)

Messages: sensor_msgs/Image mono16, the S1 stack camera's measured window (~1000 x 300 px, 0.6 MB)
or a whole midiCam2 frame (2448 x 2048, 10.0 MB), 100 of them at 10 Hz, header.stamp = send time.
Latency = receipt - stamp, same machine, same clock. RMWs: rmw_fastrtps_cpp (Jazzy's default; what
this lane's helper runs used) and rmw_cyclonedds_cpp with the project's configuration (motobat_env.sh:
loopback only, no multicast). QoS: reliable / best effort, depth 10.
Reported per configuration: delivered, median, p95, max.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

SIZES = {'roi': (300, 1000), 'full': (2048, 2448)}
CYCLONE_URI = ('<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces>'
               '<AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex>'
               '<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers>'
               '</Discovery></Domain></CycloneDDS>')


def qos(reliable):
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)


def role_pub(a):
    import rclpy
    from sensor_msgs.msg import Image
    rclpy.init()
    n = rclpy.create_node('bench_pub')
    p = n.create_publisher(Image, '/bench/image', qos(a.reliable))
    h, w = SIZES[a.size]
    data = np.random.default_rng(0).integers(0, 4095, (h, w), dtype=np.uint16).astype('<u2').tobytes()
    end = time.time() + 20
    while p.get_subscription_count() == 0 and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(1.0)                                         # let discovery settle both ways
    for i in range(a.count):
        m = Image(height=h, width=w, encoding='mono16', step=w * 2, data=data)
        m.header.frame_id = str(i)
        m.header.stamp = n.get_clock().now().to_msg()
        p.publish(m)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(1.0 / a.rate)
    time.sleep(2.0)
    n.destroy_node()
    rclpy.shutdown()


def role_sub(a):
    import rclpy
    from sensor_msgs.msg import Image
    rclpy.init()
    n = rclpy.create_node('bench_sub')
    lat = []

    def cb(m):
        now = n.get_clock().now().nanoseconds
        lat.append((now - (m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec)) / 1e6)
    n.create_subscription(Image, '/bench/image', cb, qos(a.reliable))
    end = time.time() + a.count / a.rate + 25
    while time.time() < end and len(lat) < a.count:
        rclpy.spin_once(n, timeout_sec=0.05)
    print(json.dumps(lat))
    n.destroy_node()
    rclpy.shutdown()


def matrix(a):
    rows = []
    for rmw in ('rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp'):
        for size in ('roi', 'full'):
            for reliable in (True, False):
                env = dict(os.environ, RMW_IMPLEMENTATION=rmw, ROS_DOMAIN_ID=str(a.domain))
                if rmw == 'rmw_cyclonedds_cpp':
                    env['CYCLONEDDS_URI'] = CYCLONE_URI
                args = ['--size', size, '--count', str(a.count), '--rate', str(a.rate)] + ([] if reliable else ['--best-effort'])
                sub = subprocess.Popen([sys.executable, __file__, '--role', 'sub'] + args, env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                time.sleep(1.0)
                subprocess.run([sys.executable, __file__, '--role', 'pub'] + args, env=env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                out, _ = sub.communicate(timeout=60)
                lat = np.array(json.loads(out.strip().splitlines()[-1]) if out.strip() else [])
                row = dict(rmw=rmw.replace('rmw_', '').replace('_cpp', ''), size=size, mb=SIZES[size][0] * SIZES[size][1] * 2 / 1e6,
                           qos='reliable' if reliable else 'best effort', delivered=int(len(lat)), sent=a.count,
                           median=float(np.median(lat)) if len(lat) else float('nan'),
                           p95=float(np.percentile(lat, 95)) if len(lat) else float('nan'),
                           max=float(lat.max()) if len(lat) else float('nan'))
                rows.append(row)
                print(f'| {row["rmw"]} | {row["size"]} ({row["mb"]:.1f} MB) | {row["qos"]} | {row["delivered"]}/{row["sent"]} | '
                      f'{row["median"]:.2f} | {row["p95"]:.2f} | {row["max"]:.2f} |', flush=True)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--role', choices=['pub', 'sub'])
    ap.add_argument('--size', choices=list(SIZES), default='roi')
    ap.add_argument('--count', type=int, default=100)
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--best-effort', dest='reliable', action='store_false')
    ap.add_argument('--domain', type=int, default=79)
    ap.add_argument('--out', default='data/latency/bench.json')
    a = ap.parse_args()
    if a.role == 'pub':
        return role_pub(a)
    if a.role == 'sub':
        return role_sub(a)
    print('| RMW | image | QoS | delivered | median ms | p95 ms | max ms |')
    print('|---|---|---|---|---|---|---|')
    rows = matrix(a)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(rows, f, indent=1)


if __name__ == '__main__':
    main()

#!/usr/bin/python3
"""Where the S1 loop's time goes, from a rosbag2 recording. (task 3.6, plan §4A.5)

    latency_report.py data/latency/<bag>

The bag (s1_loop.launch.py bag:=...) holds, per cell k:
  /s1/stack_camera/image   header.stamp = capture (the image exists); bag time = when it arrived
  /s1/defect_report        same capture stamp; bag time = measurement done and delivered
  /s1/correction           same capture stamp; bag time = u_{k+1} computed and delivered
  /correction_cmd/_action/status   the goal for cell k+1 turning ACCEPTED: the correction is used
Bag time is when ros2 bag record received the message: an observer on the same machine and clock,
which adds its own (small) transport delay to every hop. The hops:
  image transport   capture -> image received
  measurement       capture -> report received   (locate + parallax correction + transport)
  control           report received -> correction received   (the I-law + transport)
  slack             correction received -> next goal accepted (how early the correction was ready)
"""
import argparse
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = defaultdict(list)
    sizes = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        out[topic].append((t, deserialize_message(data, types[topic])))
        sizes[topic] = len(data)
    return out, sizes


def stats(ms):
    a = np.array(ms)
    return f'median {np.median(a):7.2f}  p95 {np.percentile(a, 95):7.2f}  max {a.max():7.2f} ms  (n {len(a)})'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag', type=Path)
    a = ap.parse_args()
    msgs, sizes = read(a.bag)
    img = {stamp_ns(m.header): t for t, m in msgs['/s1/stack_camera/image']}
    rep = {stamp_ns(m.header): t for t, m in msgs['/s1/defect_report']}
    cor = {stamp_ns(m.header): t for t, m in msgs['/s1/correction']}
    # goal acceptances, in time order (status lists carry every live goal: keep first sight of each id)
    accepted, seen = [], set()
    for t, m in msgs['/correction_cmd/_action/status']:
        for st in m.status_list:
            gid = bytes(st.goal_info.goal_id.uuid)
            if st.status == 2 and gid not in seen:          # STATUS_EXECUTING: accepted and started
                seen.add(gid)
                accepted.append(t)
    accepted.sort()
    hops = defaultdict(list)
    for cap in sorted(rep):
        if cap in img:
            hops['image transport'].append((img[cap] - cap) / 1e6)
        hops['measurement'].append((rep[cap] - cap) / 1e6)
        if cap in cor:
            hops['control'].append((cor[cap] - rep[cap]) / 1e6)
            nxt = [t for t in accepted if t > cor[cap]]
            if nxt:
                hops['slack'].append((nxt[0] - cor[cap]) / 1e6)
    print(f'{a.bag}: {len(rep)} cells; image {sizes.get("/s1/stack_camera/image", 0) / 1e6:.2f} MB per message')
    for name in ('image transport', 'measurement', 'control', 'slack'):
        if hops[name]:
            print(f'  {name:16s} {stats(hops[name])}')
    if not hops['measurement']:
        sys.exit('no reports in the bag')


if __name__ == '__main__':
    main()

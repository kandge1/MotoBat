#!/usr/bin/env python3
"""
MotoBat — show coverage (task 1.5) in RViz: which required surfaces the fixed
line-scan and the arm's view plan can each measure, and where the arm looks.

Reads what `python -m defectgen.coverage` wrote to data/coverage/:
<stage>.patches.npz (one entry per ~1.5 mm surface patch) and
<stage>.plan.json (the arm's views, in order).

    # Terminal 4 -- system Python, NOT mb-venv
    source ~/projects/MotoBat/motobat_env.sh
    /usr/bin/python3 src/motobat_description/scripts/show_coverage.py --stage post_weld
    /usr/bin/python3 src/motobat_description/scripts/show_coverage.py --stage pre_busbar --class handling_damage

WHAT YOU SEE ("Coverage" and "View plan" in RViz's left panel):

  Coverage, one dot per required patch, for the class you pick (default: all
  classes -- a patch then counts as seen only if it is seen for every class
  that needs it):
    green   both the fixed line-scan and the arm plan measure it
    blue    only the arm plan does   (what the arm adds)
    orange  only the fixed line-scan does
    red     neither                  (left uncovered)
  View plan: each snapshot as an arrow from the camera to where it looks,
  each sweep as a line along its path with an arrow down its line of sight,
  numbered in the order the plan flies them.

Run it next to show_module.py (Terminal 2, same --stage) to see the module
under the dots; untick "Defects" there for a clearer view.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).parent))
from show_module import FRAME, MM, STAGES, _marker  # noqa: E402

CLOUD_TOPIC = '/defectgen/coverage'
PLAN_TOPIC = '/defectgen/view_plan'
COLOURS = {'both': (0.15, 0.85, 0.25), 'arm': (0.20, 0.45, 1.00),
           'fixed': (1.00, 0.60, 0.10), 'none': (1.00, 0.10, 0.10)}


def coverage_cloud(npz, cls):
    names = [str(c) for c in npz['classes']]
    if cls:
        if cls not in names:
            raise SystemExit(f'no class {cls!r} at this stage; choose from {names}')
        rows = [names.index(cls)]
    else:
        rows = list(range(len(names)))
    req = npz['required'][rows]
    need = req.any(axis=0)
    # Seen for every class that requires the patch.
    fixed = np.all(~req | npz['fixed'][rows], axis=0) & need
    arm = np.all(~req | npz['arm'][rows], axis=0) & need
    state = np.where(fixed & arm, 'both', np.where(arm, 'arm', np.where(fixed, 'fixed', 'none')))[need]
    pts = npz['centroid'][need] + 0.2 * npz['normal'][need]   # just off the surface, so it is not hidden
    rgb = np.array([COLOURS[s] for s in state])
    c = (rgb * 255).astype(np.uint32)
    packed = ((c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]).view(np.float32)
    buf = np.column_stack([(pts * MM).astype(np.float32), packed]).astype(np.float32)
    msg = PointCloud2()
    msg.header.frame_id = FRAME
    msg.height, msg.width = 1, len(buf)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'rgb'))]
    msg.is_bigendian, msg.point_step = False, 16
    msg.row_step, msg.is_dense = 16 * len(buf), True
    msg.data = buf.tobytes()
    area = npz['area'][need]
    shares = {k: float(area[state == k].sum() / area.sum()) for k in COLOURS}
    return msg, shares


def plan_markers(plan):
    out = []
    for v in plan['views']:
        pose = np.array(v['pose_mm'])
        o, x, y, z = pose[:3, 3], pose[:3, 0], pose[:3, 1], pose[:3, 2]
        rgb = (0.95, 0.85, 0.20) if v['kind'] == 'snapshot' else (0.85, 0.30, 0.95)
        i = v['step']
        if v['kind'] == 'snapshot':
            a = _marker('views/snapshots', i, Marker.ARROW)
            a.points = [Point(x=o[0] * MM, y=o[1] * MM, z=o[2] * MM),
                        Point(x=(o + 150 * z)[0] * MM, y=(o + 150 * z)[1] * MM, z=(o + 150 * z)[2] * MM)]
            a.scale.x, a.scale.y, a.scale.z = 4 * MM, 10 * MM, 16 * MM
            label_at = o
        else:
            end = o + v['length_mm'] * y
            line = _marker('views/sweeps', i, Marker.LINE_STRIP)
            line.points = [Point(x=p[0] * MM, y=p[1] * MM, z=p[2] * MM) for p in (o, end)]
            line.scale.x = 3 * MM
            (line.color.r, line.color.g, line.color.b), line.color.a = rgb, 1.0
            out.append(line)
            mid = (o + end) / 2
            a = _marker('views/sweeps', 1000 + i, Marker.ARROW)
            a.points = [Point(x=mid[0] * MM, y=mid[1] * MM, z=mid[2] * MM),
                        Point(x=(mid + 120 * z)[0] * MM, y=(mid + 120 * z)[1] * MM, z=(mid + 120 * z)[2] * MM)]
            a.scale.x, a.scale.y, a.scale.z = 3 * MM, 8 * MM, 12 * MM
            label_at = mid
        (a.color.r, a.color.g, a.color.b), a.color.a = rgb, 1.0
        out.append(a)
        t = _marker('views/labels', i, Marker.TEXT_VIEW_FACING)
        t.pose.position.x, t.pose.position.y, t.pose.position.z = (label_at + [0, 0, 15]) * MM
        t.scale.z = 12 * MM
        t.text = f"{i}:{v['id']}"
        (t.color.r, t.color.g, t.color.b), t.color.a = rgb, 1.0
        out.append(t)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('--stage', choices=STAGES, default='post_weld')
    p.add_argument('--class', dest='cls', default=None, help='one class family (default: all)')
    p.add_argument('--dir', type=Path, default=Path('data/coverage'))
    args, ros_args = p.parse_known_args()
    npz_f, plan_f = args.dir / f'{args.stage}.patches.npz', args.dir / f'{args.stage}.plan.json'
    if not npz_f.is_file():
        p.error(f'{npz_f} not found -- run, in the venv: python -m defectgen.coverage --stage {args.stage}')
    cloud, shares = coverage_cloud(np.load(npz_f), args.cls)
    plan = json.loads(plan_f.read_text())

    clear = Marker()
    clear.header.frame_id = FRAME
    clear.action = Marker.DELETEALL
    markers = plan_markers(plan)

    rclpy.init(args=ros_args)
    node = Node('show_coverage')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cpub = node.create_publisher(PointCloud2, CLOUD_TOPIC, qos)
    mpub = node.create_publisher(MarkerArray, PLAN_TOPIC, qos)
    cpub.publish(cloud)
    mpub.publish(MarkerArray(markers=[clear] + markers))
    node.create_timer(1.0, lambda: mpub.publish(MarkerArray(markers=markers)))
    log = node.get_logger()
    log.info(f"{args.stage}, class {args.cls or 'all'}: of the required area, "
             + ', '.join(f'{k} {100 * v:.1f}%' for k, v in shares.items()))
    log.info(f"view plan: {len(plan['views'])} captures -> {PLAN_TOPIC}; coverage -> {CLOUD_TOPIC}")
    log.info('Ctrl-C to stop.')
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

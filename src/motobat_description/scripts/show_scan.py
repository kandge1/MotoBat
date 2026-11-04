#!/usr/bin/env python3
"""
MotoBat — show simulated sensor scans in RViz, alongside the module.

Each scan written by `python -m defectgen.scan` is published as a PointCloud2
on its own topic, /defectgen/scan/<sensor>, so RViz has one display per
sensor (Line scan, Profilometer, Snapshot) that can be ticked on and off.
A snapshot's 2D picture, if one was saved next to it, goes to
/defectgen/snapshot_2d for RViz's Image display.

    /usr/bin/python3 src/motobat_description/scripts/show_scan.py \
        data/preview_welds/scans/post_weld_*.npz --colour signal

--side-by-side lays the clouds out next to each other across the module
(line scan, profilometer, snapshot from left to right) instead of on top of
one another and of the module.

Run it alongside show_module.py, not instead of it. System Python, not
mb-venv (rclpy needs 3.12; the venv is 3.11).
"""
import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image, PointCloud2

sys.path.insert(0, str(Path(__file__).parent))
from show_module import FRAME, scan_cloud, scan_topic  # noqa: E402

ORDER = {'line_scan': 0, 'profilometer': 1, 'snapshot': 2}
SPACING_MM = 200.0  # module is 151 wide; this leaves a gap between copies
IMAGE_TOPIC = '/defectgen/snapshot_2d'


def image_msg(png):
    from PIL import Image as PILImage
    import numpy as np

    rgb = np.asarray(PILImage.open(png).convert('RGB'))
    msg = Image()
    msg.header.frame_id = FRAME
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding, msg.is_bigendian, msg.step = 'rgb8', 0, rgb.shape[1] * 3
    msg.data = rgb.tobytes()
    return msg


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('scans', type=Path, nargs='+', help='one or more .npz from python -m defectgen.scan')
    p.add_argument('--colour', choices=('height', 'material', 'signal'), default='height')
    p.add_argument('--max', type=int, default=2_000_000, help='decimate each cloud to this many points')
    p.add_argument('--side-by-side', action='store_true',
                   help='offset each sensor across the module so they do not overlap')
    args, ros_args = p.parse_known_args()
    for f in args.scans:
        if not f.exists():
            p.error(f'{f} not found -- run: python -m defectgen.scan <run dir> --sensor ...')

    rclpy.init(args=ros_args)
    node = Node('show_scan')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    log = node.get_logger()
    keep = []
    for f in args.scans:
        cloud, note, sensor = scan_cloud(f, args.colour, args.max)
        if args.side_by_side:
            slot = ORDER.get(sensor, len(ORDER)) - 1
            cloud, note, sensor = scan_cloud(f, args.colour, args.max, offset=(0.0, slot * SPACING_MM, 0.0))
        topic = scan_topic(sensor)
        pub = node.create_publisher(PointCloud2, topic, qos)
        pub.publish(cloud)
        keep.append(pub)
        log.info(f'{f.name}: {note} -> {topic}')
        png = f.with_name(f.stem + '_2d.png')
        if png.exists():
            ipub = node.create_publisher(Image, IMAGE_TOPIC, qos)
            ipub.publish(image_msg(png))
            keep.append(ipub)
            log.info(f'{png.name} -> {IMAGE_TOPIC}')
    log.info('Ctrl-C to stop; clouds stay in RViz until another run replaces them.')
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

#!/usr/bin/env python3
"""
MotoBat — show a defectgen module and its ground-truth defects in RViz.

Reads what `python -m defectgen` wrote to disk and publishes it as a
MarkerArray. No colcon build needed: run it as a plain script.

    # Terminal 1
    source ~/projects/MotoBat/motobat_env.sh
    rviz2 -d src/motobat_description/rviz/defects.rviz

    # Terminal 2 -- system Python, NOT mb-venv
    source ~/projects/MotoBat/motobat_env.sh
    /usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective
    /usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective \
        --stage pre_weld --ghost data/preview_clean --hide cover

    # one cell and its terminals against the Fusion cell_assembled
    /usr/bin/python3 src/motobat_description/scripts/show_module.py data/cell_check \
        --stage pre_busbar --ghost data/cell_check/reference

WHAT YOU SEE: the module, opaque, one body group per marker namespace
(part/cells, part/cover, part/busbars, ...). Each defect class is its own
namespace too (defect/terminal_tilt, ...): a wireframe box around what the
defect touched -- where it should be and where it is -- and a tag giving class
and magnitude. A short weld sits inside a box of full nominal length; a
missing part leaves an empty box. In RViz, expand Defects -> Namespaces and
untick a box to hide that group -- untick part/cover to see the busbars.

Markers are republished every second. RViz deletes a namespace's markers when
it is unticked and does not keep a copy, so ticking it back shows nothing
until the next message arrives.

--ghost overlays the clean module, translucent, so a 2 mm offset or a 6 deg
tilt shows as a step between the two surfaces. --hide drops groups before
publishing, for when you always want the same ones gone.

WHY NOT THE VENV: the venv is Python 3.11 and ROS Jazzy's rclpy is built for
3.12, so `import rclpy` fails inside it. This script needs only json and
rclpy, so system Python is all it wants.

WHY IT DOES NOT IMPORT defectgen: defectgen must never touch ROS
(project_structure.md §2), and this script must not need to know defectgen's
internals. The STLs and labels JSON on disk are the contract between them.

Units: defectgen works in millimetres, RViz in metres. Every position and
the mesh scale go through MM.
"""
import argparse
import colorsys
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

MM = 0.001
FRAME = 'module'
TOPIC = '/defectgen/markers'
STAGES = ('pre_busbar', 'pre_weld', 'post_weld')

# Body-group colours: roughly the material, kept dull so the saturated
# defect colours stand out against them.
PART_RGB = {
    'cooling_plate': (0.45, 0.48, 0.52),
    'cells':         (0.22, 0.26, 0.34),
    # Colour-coded for viewing only. Both terminals are bare aluminium
    # (module_spec §4) -- never feed these colours to a detector.
    'terminals_pos': (0.85, 0.35, 0.30),
    'terminals_neg': (0.30, 0.40, 0.85),
    'pads':          (0.85, 0.80, 0.55),
    'end_plates':    (0.58, 0.60, 0.63),
    'side_plates':   (0.58, 0.60, 0.63),
    'carrier':       (0.15, 0.15, 0.16),
    'busbars':       (0.72, 0.45, 0.22),
    'welds':         (0.95, 0.90, 0.75),
    'cmc':           (0.10, 0.42, 0.20),
    'cover':         (0.50, 0.52, 0.56),
    'fod':           (1.00, 1.00, 1.00),
}

# Boxes sit this far outside what they enclose, so the lines are not buried
# in the part's faces.
BOX_PAD_MM = 0.5
LINE_MM = 0.4

# What `magnitude` means per class, from the plan.record() calls in
# defectgen/defects.py. None means it is a flag, so the tag shows no number.
UNITS = {
    'cell_placement': 'mm', 'terminal_height': 'mm', 'terminal_tilt': 'deg',
    'missing_cell': None, 'reversed_polarity': None, 'busbar_pose': 'mm',
    'busbar_liftoff': 'mm', 'missing_weld': None, 'short_weld': 'fraction',
    'seam_deviation': 'mm', 'width_deviation': 'mm', 'reinforcement': 'mm',
    'spatter': 'mm', 'handling_damage': 'mm', 'missing_pad': None,
    'fod': 'mm', 'carrier_seating': 'mm', 'cover_misalignment': 'mm',
}
# Fixed class order so a class keeps its colour from run to run.
CLASSES = tuple(UNITS)


def class_colour(cls):
    i = CLASSES.index(cls) if cls in CLASSES else len(CLASSES)
    return colorsys.hsv_to_rgb(i / (len(CLASSES) + 1), 0.85, 1.0)


def _marker(ns, mid, kind):
    m = Marker()
    m.header.frame_id = FRAME
    m.ns, m.id = ns, mid
    m.type, m.action = kind, Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def mesh_marker(stl, ns, rgb, alpha):
    m = _marker(ns, 0, Marker.MESH_RESOURCE)
    m.mesh_resource = stl.resolve().as_uri()
    m.mesh_use_embedded_materials = False
    m.scale.x = m.scale.y = m.scale.z = MM
    (m.color.r, m.color.g, m.color.b), m.color.a = rgb, alpha
    return m


def amount(cls, magnitude):
    unit = UNITS.get(cls, '')
    if unit is None:
        return ''
    if unit == 'fraction':
        return f'={magnitude * 100:.0f}%short'
    if unit == 'mm' and abs(magnitude) < 1.0:
        return f'={magnitude * 1000:.0f}um'
    return f'={magnitude:.2f}{unit}'


def tag(cls, magnitude, count):
    # No spaces: RViz2 lays out a space in a TEXT_VIEW_FACING marker far wider
    # than the font, which split the first version's tags into a name hanging
    # off one side of the module and a number off the other.
    return f'{cls}{amount(cls, magnitude)}' + (f'(x{count})' if count > 1 else '')


def box_marker(ns, mid, lo, hi, rgb):
    """Wireframe box: the 12 edges as a LINE_LIST, in metres."""
    m = _marker(ns, mid, Marker.LINE_LIST)
    m.scale.x = LINE_MM * MM
    (m.color.r, m.color.g, m.color.b), m.color.a = rgb, 1.0
    lo = [(v - BOX_PAD_MM) * MM for v in lo]
    hi = [(v + BOX_PAD_MM) * MM for v in hi]
    corner = lambda i: Point(x=(lo, hi)[i & 1][0], y=(lo, hi)[i >> 1 & 1][1],
                             z=(lo, hi)[i >> 2 & 1][2])
    for a in range(8):
        for bit in (1, 2, 4):
            if not a & bit:
                m.points += [corner(a), corner(a | bit)]
    return m


def defect_markers(defects):
    out = []
    by_class = {}
    for d in defects:
        by_class.setdefault(d['cls'], []).append(d)

    for cls, ds in by_class.items():
        ns = f'defect/{cls}'
        rgb = class_colour(cls)
        # One tag per class when every instance has the same magnitude
        # (25 spatter tags bury the module); otherwise one per instance.
        same = len({e['magnitude'] for e in ds}) == 1
        for i, d in enumerate(ds):
            if d.get('bounds'):
                lo, hi = d['bounds']
            else:
                # Labels from before defectgen wrote boxes: a 4 mm cube.
                lo = [c - 2.0 for c in d['location']]
                hi = [c + 2.0 for c in d['location']]
            out.append(box_marker(ns, 2 * i, lo, hi, rgb))
            if same and i > 0:
                continue
            t = _marker(ns, 2 * i + 1, Marker.TEXT_VIEW_FACING)
            t.pose.position.x = (lo[0] + hi[0]) / 2 * MM
            t.pose.position.y = (lo[1] + hi[1]) / 2 * MM
            t.pose.position.z = (hi[2] + 8) * MM
            t.scale.z = 6 * MM
            t.text = tag(cls, d['magnitude'], len(ds) if same else 1)
            (t.color.r, t.color.g, t.color.b), t.color.a = rgb, 1.0
            out.append(t)
    return out


def part_markers(run_dir, stage, hide, alpha):
    groups_dir = run_dir / stage
    stls = sorted(groups_dir.glob('*.stl')) if groups_dir.is_dir() else []
    if not stls:
        # A run from before defectgen wrote body groups: one mesh, no toggles.
        return [mesh_marker(run_dir / f'{stage}.stl', 'part/module',
                            (0.6, 0.6, 0.62), alpha)], ['module']
    shown = [p for p in stls if p.stem not in hide]
    return ([mesh_marker(p, f'part/{p.stem}', PART_RGB.get(p.stem, (0.6, 0.6, 0.6)), alpha)
             for p in shown], [p.stem for p in shown])


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('dir', type=Path, help='a defectgen --out directory')
    p.add_argument('--stage', choices=STAGES, default='post_weld')
    p.add_argument('--ghost', type=Path, default=None,
                   help='a clean defectgen --out directory to overlay translucent')
    p.add_argument('--hide', nargs='+', default=[], metavar='GROUP',
                   help=f'body groups not to publish: {" ".join(PART_RGB)}')
    p.add_argument('--alpha', type=float, default=1.0,
                   help='opacity of the module bodies (default 1.0, opaque)')
    args, ros_args = p.parse_known_args()

    # Fail before ROS starts, with a message that names the fix.
    for d in [args.dir] + ([args.ghost] if args.ghost else []):
        if not (d / f'{args.stage}.stl').exists():
            p.error(f'{d / (args.stage + ".stl")} not found -- '
                    f'run: python -m defectgen --out {d}' +
                    ('' if d == args.ghost else ' --demo-defects'))
    unknown = set(args.hide) - set(PART_RGB)
    if unknown:
        p.error(f'unknown --hide group(s) {sorted(unknown)}; choose from {list(PART_RGB)}')

    labels = json.loads((args.dir / f'{args.stage}.labels.json').read_text())
    parts, groups = part_markers(args.dir, args.stage, set(args.hide), args.alpha)
    # RViz keeps markers from earlier runs until told otherwise, so a group
    # dropped with --hide, or a class this stage lacks, would linger.
    clear = Marker()
    clear.header.frame_id = FRAME
    clear.action = Marker.DELETEALL
    markers = parts
    if args.ghost:
        markers.append(mesh_marker(args.ghost / f'{args.stage}.stl', 'ghost',
                                   (0.3, 0.7, 1.0), 0.25))
    markers += defect_markers(labels['defects'])

    rclpy.init(args=ros_args)
    node = Node('show_module')
    # Transient local: RViz started after this script still gets the markers.
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, TOPIC, qos)
    pub.publish(MarkerArray(markers=[clear] + markers))
    # DELETEALL goes out once only: repeating it every second would blank the
    # display between messages. The timer brings back re-ticked namespaces.
    node.create_timer(1.0, lambda: pub.publish(MarkerArray(markers=markers)))

    log = node.get_logger()
    log.info(f"{args.dir}/{args.stage} -> {TOPIC} (frame '{FRAME}')")
    log.info(f"bodies: {' '.join(groups)}" + (' + ghost' if args.ghost else ''))
    counts = {}
    for d in labels['defects']:
        counts[d['cls']] = counts.get(d['cls'], 0) + 1
    log.info(f"{len(labels['defects'])} defects in {len(counts)} classes")
    for cls, n in counts.items():
        log.info(f'  {cls:20s} x{n}')
    log.info('Ctrl-C to stop. Markers stay latched while this runs.')

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

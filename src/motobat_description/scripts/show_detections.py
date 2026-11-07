#!/usr/bin/env python3
"""
MotoBat — show what detection found in RViz, scored against the truth.

Reads what `python -m defectgen.detect` wrote to disk,
<run>/detections/<stage>.report.json and, if the run has labels,
<stage>.evaluation.json, and publishes them as a MarkerArray on
/defectgen/detections ("Detections" in RViz's left panel).

    # Terminal 4 -- system Python, NOT mb-venv (same reason as show_module.py)
    source ~/projects/MotoBat/motobat_env.sh
    /usr/bin/python3 src/motobat_description/scripts/show_detections.py data/preview_welds --stage post_weld

WHAT YOU SEE: one wireframe box per finding, coloured by how it scored, and a
tag with its class and size:

    green    found     -- right class, right place (TP)
    red      false alarm -- nothing there (FP)
    amber    knock-on  -- a real effect of a defect of another class, e.g. the
                          busbars a lifted carrier raised (counted as "other")
    magenta  missed    -- a defect detection should have found and did not
                          (FN); the box is the truth label's, the tag says what,
                          if anything, was found there instead
    cyan     unscored  -- the run has no labels to score against

Several findings on one part (a stitch that is both offset and bowed) share
one box and one tag. Each colour is its own namespace (found/..., missed/...),
so you can untick, say, everything that was found and see only the misses.
A summary tag floats above the module: the station result (OK, NOK, or
INCOMPLETE when some feature was never measured), the score, and what no
capture could measure.

Run it next to show_module.py, which draws the module and its ground-truth
boxes ("Defects"). Untick "Defects" to see only what detection reported.

WHY IT DOES NOT IMPORT defectgen: the JSON files are the contract, as for
show_module.py. Matching findings to labels happens in the venv
(defectgen/evaluate.py); this only draws the result.
"""
import argparse
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).parent))
from show_module import FRAME, MM, STAGES, _marker, box_marker, tag  # noqa: E402

TOPIC = '/defectgen/detections'
VERDICT = {   # namespace, colour, worst first
    'fp':       ('false_alarm', (1.00, 0.15, 0.15)),
    'fn':       ('missed',      (1.00, 0.20, 0.85)),
    'other':    ('knock_on',    (1.00, 0.70, 0.10)),
    'tp':       ('found',       (0.20, 0.95, 0.30)),
    'unscored': ('unscored',    (0.20, 0.85, 1.00)),
}
WORST = tuple(VERDICT)


def _group(items):
    """Items that share a box (several findings on one stitch) as one entry."""
    out = {}
    for it in items:
        lo, hi = it['bounds']
        key = tuple(round(v, 1) for v in list(lo) + list(hi))
        out.setdefault(key, (lo, hi, []))[2].append(it)
    return list(out.values())


def detection_markers(findings, verdicts, labels):
    items = [dict(f, verdict=v) for f, v in zip(findings, verdicts)]
    items += [dict(d, verdict='fn') for d in labels if d['verdict'] == 'fn']
    out = []
    for i, (lo, hi, its) in enumerate(_group(items)):
        worst = min((it['verdict'] for it in its), key=WORST.index)
        ns, rgb = VERDICT[worst]
        out.append(box_marker(f'{ns}/box', i, lo, hi, rgb))
        words = []
        for it in sorted(its, key=lambda it: WORST.index(it['verdict'])):
            if it['verdict'] == 'fn':
                found = '/'.join(it.get('found_there') or []) or 'nothing'
                words.append(f"MISSED:{tag(it['cls'], it.get('magnitude') or 0, 1)}(found:{found})")
            else:
                words.append(tag(it['cls'], it['magnitude'], 1))
        t = _marker(f'{ns}/tag', i, Marker.TEXT_VIEW_FACING)
        t.pose.position.x = (lo[0] + hi[0]) / 2 * MM
        t.pose.position.y = (lo[1] + hi[1]) / 2 * MM
        t.pose.position.z = (hi[2] + 5) * MM
        t.scale.z = 4 * MM
        t.text = '+'.join(dict.fromkeys(words))
        (t.color.r, t.color.g, t.color.b), t.color.a = rgb, 1.0
        out.append(t)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('dir', type=Path, help='a defectgen --out directory that detect has been run on')
    p.add_argument('--stage', choices=STAGES, default='post_weld')
    args, ros_args = p.parse_known_args()

    report_f = args.dir / 'detections' / f'{args.stage}.report.json'
    if not report_f.is_file():
        p.error(f'{report_f} not found -- run, in the venv: '
                f'python -m defectgen.detect {args.dir} --stage {args.stage}')
    report = json.loads(report_f.read_text())
    findings = report['findings']
    eval_f = report_f.with_name(f'{args.stage}.evaluation.json')
    if eval_f.is_file() and eval_f.stat().st_mtime >= report_f.stat().st_mtime:
        ev = json.loads(eval_f.read_text())
        verdicts, labels = ev['finding_verdicts'], ev['labels']
        t = ev['total']
        summary = (f"{args.stage} {report.get('result', '')}: {t['tp']} found, {t['fn']} missed, "
                   f"{t['fp']} false alarms, {t['other']} knock-on")
    else:
        verdicts, labels = ['unscored'] * len(findings), []
        summary = f"{args.stage} {report.get('result', '')}: {len(findings)} findings (no labels, unscored)"
    if report.get('uncovered'):
        summary += ' | not measured: ' + ', '.join(f'{k} x{len(v)}' for k, v in report['uncovered'].items())

    clear = Marker()
    clear.header.frame_id = FRAME
    clear.ns = ''
    clear.action = Marker.DELETEALL
    markers = detection_markers(findings, verdicts, labels)
    s = _marker('summary', 0, Marker.TEXT_VIEW_FACING)
    s.pose.position.z = 160 * MM
    s.scale.z = 8 * MM
    s.text = summary.replace(' ', '_')   # RViz spaces render far too wide (show_module.tag)
    s.color.r = s.color.g = s.color.b = s.color.a = 1.0
    markers.append(s)

    rclpy.init(args=ros_args)
    node = Node('show_detections')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, TOPIC, qos)
    pub.publish(MarkerArray(markers=[clear] + markers))
    node.create_timer(1.0, lambda: pub.publish(MarkerArray(markers=markers)))
    log = node.get_logger()
    log.info(f'{report_f} -> {TOPIC}')
    log.info(summary)
    for f, v in zip(findings, verdicts):
        log.info(f"  {VERDICT[v][0]:11s} {f['cls']:18s} {f['magnitude']:8.3f}  {f['feature']}")
    for d in labels:
        if d['verdict'] == 'fn':
            log.info(f"  {'missed':11s} {d['cls']:18s} found there: {d['found_there'] or 'nothing'}")
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

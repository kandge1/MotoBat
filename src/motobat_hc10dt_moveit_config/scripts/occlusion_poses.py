#!/usr/bin/python3
"""
Robot link poses during every capture of a base's flight, for the occlusion check (task 2.5).
Step 1 of 2; step 2 is occlusion_check.py (venv), which casts the sensors' rays against them.

    /usr/bin/python3 src/motobat_hc10dt_moveit_config/scripts/occlusion_poses.py data/reach/base_A/reach.json

For each stage, each view in flying order, in the IK branch the sequence chose: a snapshot at
its one pose; a sweep at SAMPLES points evenly along its measured length (not the run-up).
Writes <reach dir>/occlusion_poses.json: every link with collision geometry, as 4x4 transforms
in the module frame (m), plus where along the sweep (0..1 of the measured length) each was
taken. Starts its own headless move_group (for /compute_fk); run it on a free ROS domain.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import view_reach as vr  # noqa: E402

SAMPLES = 16
LINKS = ['s3_base_link'] + vr.LINKS


def main():
    reach = Path(sys.argv[1])
    d = json.load(open(reach))
    r = d['results'][0]
    b = r['base']
    T_mb = vr.base_in_module(b['bx'], b['d'], b['bz'], 0.0, b.get('mount', 'floor'))
    views = {v['id']: v for v in r['views']}
    proc = vr.start_move_group(reach.parent / 'move_group_occlusion.log')
    rclpy.init()
    out = {'reach': str(reach), 'run_up': d['run_up'], 'stages': {}}
    try:
        cli = vr.Client()
        for attempt in range(20):
            try:
                cli.fk_links(vr.READY, [vr.TOOL])
                break
            except RuntimeError:
                time.sleep(1.0)
        for stage, seq in r['sequences'].items():
            rows = []
            for vid in seq['order']:
                v = views[vid]
                P = np.array(seq.get('paths', {}).get(vid, v['path']))
                if v['sensor'] == 'snapshot':
                    picks = [(0.0, P[0])]
                else:
                    # the path runs run-up, measured length, run-up at even spacing
                    L, U = v['length'], d['run_up']
                    u = np.linspace(0.0, 1.0, len(P))
                    picks = []
                    for w in (np.arange(SAMPLES) + 0.5) / SAMPLES:
                        f = (U + w * L) / (L + 2 * U)
                        picks.append((float(w), [float(np.interp(f, u, P[:, j])) for j in range(6)]))
                for w, q in picks:
                    fr = cli.fk_links(list(q), LINKS)
                    rows.append({'id': vid, 'sensor': v['sensor'], 'when': w, 'q': list(map(float, q)),
                                 'links': {n: (T_mb @ fr[n]).tolist() for n in LINKS}})
            out['stages'][stage] = rows
            print(f'  {stage}: {len(rows)} robot poses over {len(seq["order"])} views', flush=True)
        dst = reach.parent / 'occlusion_poses.json'
        json.dump(out, open(dst, 'w'))
        print(f'wrote {dst}')
    finally:
        rclpy.try_shutdown()
        vr.stop(proc)


if __name__ == '__main__':
    main()

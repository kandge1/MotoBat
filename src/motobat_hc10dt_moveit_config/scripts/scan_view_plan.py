#!/usr/bin/env python3
"""
Scan a module through the arm's view plan, as a named base reaches it, then detect
and score (task 2.5). Runs in the project venv (defectgen, Open3D), not ROS.

    cd ~/projects/MotoBat-arm && source motobat_env.sh && mb-venv
    PYTHONPATH=$PWD/defectgen/src python src/motobat_hc10dt_moveit_config/scripts/scan_view_plan.py

What it does, per base (default A and B) and stage (default post_weld):
  1. the module: `--module DIR` (a defectgen --out directory), or by default a test module
     it writes once to data/arm_scans/weld_walls/module: the weld-defect demo set
     (defectgen --demo-defects welds) plus a 0.4 mm dent on each long side wall,
     the one thing only the arm's wall sweeps can see;
  2. every view the base reaches (data/reach/base_<NAME>/reach.json, in flying order),
     scanned with the frozen 1.3 sensor models exactly as the 1.7 dataset's arm column
     scans them (dataset.scan_setup): same poses, speeds and rates. The module sits
     exactly on the pallet datum (no fixture error) and the motion is perfect: dataset 1;
  3. detection on those captures (detect.detect) and scoring against the labels
     (evaluate.evaluate), as `python -m defectgen.detect` does;
  4. writes data/arm_scans/<name>/base_<NAME>/<stage>/:
       clouds.npz                 up to --display-points points per view, for RViz
                                  (show_view_plan.py --scans), with when each was measured
       detections/<stage>.report.json, <stage>.evaluation.json   for show_detections.py
       summary.json               the per-label result
     and prints the score table, then a label-by-label comparison of the bases.

PYTHONPATH=$PWD/defectgen/src makes it use this worktree's defectgen (the venv's editable
install points at ../MotoBat, whose working copy may carry uncommitted changes).
Heavy-ish: about a minute per view set and up to ~6 GB; run it capped:
    systemd-run --user --scope -p MemoryMax=7G -p MemorySwapMax=0 python ...scan_view_plan.py
"""
from __future__ import annotations

import argparse
import json
import logging
import shutil
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
STAGES = ['pre_busbar', 'pre_weld', 'post_weld']
log = logging.getLogger('scan_view_plan')


def make_module(out: Path) -> None:
    from defectgen import defects as D
    from defectgen.__main__ import weld_defects
    from defectgen.config import load
    from defectgen.module import export

    cfg = load()
    y = cfg.cell_w / 2 + cfg.side_plate_t          # outer face of each side plate
    inj = weld_defects(cfg) + [
        D.handling_damage(('side_plate', 'left'), depth=0.4, radius=4.0, centre=(-60.0, -y, 50.0),
                          direction=(0.0, 1.0, 0.0)),
        D.handling_damage(('side_plate', 'right'), depth=0.4, radius=4.0, centre=(80.0, y, 50.0),
                          direction=(0.0, -1.0, 0.0)),
    ]
    export(out, cfg=cfg, seed=0, defects=inj)
    (out / 'README.txt').write_text(
        'Test module for scan_view_plan.py: defectgen --demo-defects welds, plus a 0.4 mm\n'
        'deep, 4 mm radius dent at mid-height on each long side wall (x = -60 left, +80 right).\n')


def scan_base(run: Path, stage: str, base: str, out: Path, display_points: int, noise: int,
              seam_speed: float | None = None, reach_file: Path | None = None,
              plans: Path | None = None, with_fixed: bool = False) -> dict:
    from defectgen.config import load
    from defectgen.dataset import _spill
    from defectgen.detect import Capture, Reference, detect
    from defectgen.evaluate import evaluate, format_table, to_json
    from defectgen.scan import Scene, line_scan, snapshot, without_groups

    reach_file = reach_file or REPO / 'data' / 'reach' / f'base_{base}' / 'reach.json'
    reach = json.loads(Path(reach_file).read_text())['results'][0]
    seq = reach['sequences'][stage]
    plan = json.loads((Path(plans or REPO / 'data' / 'coverage') / f'{stage}.plan.json').read_text())['views']
    index = {v['id']: i for i, v in enumerate(plan)}
    by_id = {v['id']: v for v in plan}
    cfg = load()
    scene = Scene.from_run(run, stage, without=without_groups(stage))
    spill = out / '_spill'
    caps, clouds, rng = [], {}, np.random.default_rng(0)
    t0 = time.perf_counter()
    quiet = logging.getLogger('defectgen.scan')
    quiet.setLevel(logging.WARNING)
    if with_fixed:
        # the fixed line-scan pass, exactly as dataset.scan_setup's fixed branch flies it: the
        # arm then only has to fill its gaps (task 2.9's gap-filling plans)
        from defectgen.coverage import fixed_baseline
        fv = fixed_baseline(cfg)
        caps.append(_spill(Capture.from_scan(line_scan(scene, 'line_scan', fv.pose, fv.length, seed=noise),
                                             name='fixed'), spill))
        log.info('  %s: fixed line-scan pass added', base)
    for k, vid in enumerate(seq['order'], 1):
        v, i = by_id[vid], index[vid]
        pose = np.array(v['pose_mm'])
        t1 = time.perf_counter()
        # the same call, seed and settings as dataset.scan_setup's arm branch
        if v['kind'] == 'snapshot':
            r = snapshot(scene, pose, sensor=v['sensor'], seed=noise + i)
        elif seam_speed and not v.get('speed_mm_s'):
            # a plan sweep at the default 50 mm/s, flown at --seam-speed with the same
            # 1 kHz profile rate: a finer pitch along the travel, the same exposure
            r = line_scan(scene, v['sensor'], pose, v['length_mm'], seed=noise + i,
                          speed=seam_speed, rate=1000.0)
        else:
            r = line_scan(scene, v['sensor'], pose, v['length_mm'], seed=noise + i,
                          speed=v.get('speed_mm_s'), rate=v.get('rate_hz'))
        n = len(r.points)
        # the slow seam/busbar sweeps carry the weld detail: give them a bigger display sample
        keep = display_points * (4 if v['kind'] == 'sweep' and not v.get('speed_mm_s') else 1)
        pick = np.sort(rng.choice(n, min(n, keep), replace=False)) if n else np.zeros(0, int)
        if v['kind'] == 'sweep' and r.grid is not None and r.grid[0] > 1:
            when = (r.index[pick] // r.grid[1]) / (r.grid[0] - 1)     # profile number -> 0..1 of the sweep
        else:
            when = np.zeros(len(pick))
        clouds[vid] = (r.points[pick].astype(np.float32) / 1000.0, when.astype(np.float32))
        caps.append(_spill(Capture.from_scan(r, name=vid), spill))
        log.info('  %s %2d/%d %-28s %9d points  %5.1f s', base, k, len(seq['order']), vid, n,
                 time.perf_counter() - t1)
        del r
    log.info('  %s: %d views scanned in %.0f s; detecting...', base, len(caps), time.perf_counter() - t0)
    ref = Reference(stage, cfg, run=run)
    report = detect(caps, stage, ref)
    del caps
    shutil.rmtree(spill, ignore_errors=True)
    labels = json.loads((run / f'{stage}.labels.json').read_text())
    result = evaluate(report, labels)
    det = out / 'detections'
    det.mkdir(parents=True, exist_ok=True)
    (det / f'{stage}.report.json').write_text(report.to_json())
    (det / f'{stage}.evaluation.json').write_text(to_json(result))
    ids = list(clouds)
    np.savez_compressed(out / 'clouds.npz', ids=np.array(ids),
                        **{f'p{k}': clouds[i][0] for k, i in enumerate(ids)},
                        **{f't{k}': clouds[i][1] for k, i in enumerate(ids)})
    summary = {'base': base, 'stage': stage, 'module': str(run), 'views': ids, 'missing': seq['missing'],
               'result': report.result, 'findings': len(report.findings), 'total': result['total'],
               'labels': result['labels'], 'seconds': round(time.perf_counter() - t0, 1)}
    (out / 'summary.json').write_text(json.dumps(summary, indent=1, default=str))
    print(f"\nbase {base}, {stage}: {len(ids)} views (not reachable: {', '.join(seq['missing']) or 'none'})")
    print(format_table(result))
    return summary


def compare(stage: str, sums: dict) -> None:
    if len(sums) < 2:
        return
    names = list(sums)
    print(f'\n{stage}: label by label ({" vs ".join(names)})')
    rows = zip(*[sums[n]['labels'] for n in names])
    diff = 0
    for row in rows:
        lab = row[0]
        verdicts = ['found' if r['verdict'] == 'tp' else
                    ('missed' + (f" (saw {'/'.join(r['found_there'])})" if r['found_there'] else ''))
                    for r in row]
        mark = '' if len(set(r['verdict'] for r in row)) == 1 else '   <-- differs'
        diff += bool(mark)
        loc = ', '.join(f'{c:.0f}' for c in lab['location'])
        print(f"  {lab['cls']:18s} at ({loc:>14s})  " + '  '.join(f'{n}: {v:22s}' for n, v in zip(names, verdicts)) + mark)
    fps = '  '.join(f"{n}: {sums[n]['total']['fp']} false alarms" for n in names)
    print(f'  {diff} label(s) differ; {fps}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--module', type=Path, default=None,
                    help='a defectgen --out directory (default: the weld_walls test module)')
    ap.add_argument('--name', default=None, help='output folder name under data/arm_scans (default: module name)')
    ap.add_argument('--stage', default='post_weld', choices=STAGES + ['all'])
    ap.add_argument('--base', nargs='+', default=['A', 'B'])
    ap.add_argument('--variant', nargs=4, action='append', default=None,
                    metavar=('NAME', 'REACH_JSON', 'PLAN_DIR', 'FIXED'),
                    help='instead of --base: a named plan (view_reach.py --plan output and the '
                         '<stage>.plan.json folder it was planned from); FIXED 1 adds the fixed '
                         'line-scan capture, as a fixed+arm setup would. Repeatable (task 2.9)')
    ap.add_argument('--display-points', type=int, default=60000,
                    help='per view, for RViz (4x this for the slow seam sweeps)')
    ap.add_argument('--noise', type=int, default=0, help='noise seed')
    ap.add_argument('--seam-speed', type=float, default=None, metavar='MM_S',
                    help='fly the plan\'s 50 mm/s sweeps (seams, busbars) at this speed instead, same '
                         '1 kHz rate: 25 = twice as dense along the travel. Output goes to <name>_seam<MM_S>. '
                         'Heavy: at 25 mm/s a seam sweep is 21-25 M points; it did not fit in 7 GB '
                         '(18 Nov), so run it on a bigger machine')
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    for noisy in ('defectgen.detect', 'defectgen.module', 'trimesh'):
        logging.getLogger(noisy).setLevel(logging.WARNING)

    run = a.module
    if run is None:
        run = REPO / 'data' / 'arm_scans' / 'weld_walls' / 'module'
        if not (run / 'post_weld.labels.json').is_file():
            log.info('writing the weld_walls test module to %s', run)
            make_module(run)
    name = a.name or (run.parent.name if run.name == 'module' else run.name)
    if a.seam_speed:
        name += f'_seam{a.seam_speed:g}'
    for stage in (STAGES if a.stage == 'all' else [a.stage]):
        sums = {}
        if a.variant:
            for vname, reach_file, plans, fixed in a.variant:
                out = REPO / 'data' / 'arm_scans' / name / vname / stage
                sums[vname] = scan_base(run.resolve(), stage, vname, out, a.display_points, a.noise,
                                        a.seam_speed, Path(reach_file), Path(plans), fixed == '1')
        else:
            for b in a.base:
                out = REPO / 'data' / 'arm_scans' / name / f'base_{b}' / stage
                sums[b] = scan_base(run.resolve(), stage, b, out, a.display_points, a.noise, a.seam_speed)
        compare(stage, sums)


if __name__ == '__main__':
    main()

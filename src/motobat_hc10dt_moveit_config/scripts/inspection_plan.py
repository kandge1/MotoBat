#!/usr/bin/env python3
"""
Minimum-time inspection plan (task 2.9). Runs in the project venv (defectgen, scipy), not ROS.

    export PYTHONPATH=$PWD/defectgen/src
    # 1. every 1.5 candidate view, as plan files view_reach.py can check from the chosen base
    python src/motobat_hc10dt_moveit_config/scripts/inspection_plan.py candidates --out data/plan29/candidates
    # 2. (ROS) reach + collision for each, from base A
    python3 src/motobat_hc10dt_moveit_config/scripts/view_reach.py --coverage data/plan29/candidates \\
        --base 0 0.30 1.10 --mount ceiling --pad 0.10 --conveyor-lift 0.05 --keep-paths --out data/reach/cand_A
    # 3. the exact minimum-time cover per stage, arm alone and arm + the fixed pass
    python src/motobat_hc10dt_moveit_config/scripts/inspection_plan.py select \\
        --reach data/reach/cand_A/reach.json --cache ../MotoBat/data/coverage/cache --out data/plan29
    # 4. (for the dataset) which patches each written plan covers, for dataset.SetupCoverage
    python src/motobat_hc10dt_moveit_config/scripts/inspection_plan.py patches data/plan29/s1/arm_fixed

What 1.5 did (coverage.py): greedy set cover over 113 candidate views, by coverage gained per
second, each capture charged an assumed 1.5 s move, no reach limit. What this changes:

  reach      a candidate counts only if 2.5's checker flies it from the chosen base (IK, collision
             with the module + keep-out, conveyor, transit floor), and, for a sweep at a speed,
             only if no joint then needs more than JOINT_SPEED_SHARE of its speed limit
  speeds     every sweep geometry is offered at SPEEDS mm/s at the same 1 kHz profile rate. Counts
             scale with the profile pitch, exactly as 1.5's own fast sweeps do (checked against
             the cached _fast counts), so no new ray casting
  time       capture (run-up and run-out included, as 2.5 flies it) + MOVE_S per view, the mean
             of 2.5's planned moves; the real order and moves come from view_reach.py --plan
  cover      an integer program (scipy milp / HiGHS), exact, not greedy:
               minimise    sum_v t_v x_v
               subject to  y_p <= sum_{v covers p} x_v               (patch p covered by a view)
                           sum_p area_p y_p >= target_k * area_k     (per class k)
             x_v, y_p binary. Patches covered by exactly the same views are merged first.
  target     per class, the coverage the 1.5 greedy plan reached in v3 (arm alone; or arm +
             fixed for the gap-filling variant), capped at what the reachable candidates can
             reach at all, minus --slack: with slack 0, never worse than v3 where reach allows.
             Further slacks print the trade-off (views and seconds per point of coverage)
  fixed      `--with-fixed`: the fixed line-scan's covered patches count as done, so the arm
             only fills its gaps. Needs a fixed+arm setup in defectgen.dataset to be scored.

Coverage rules are 1.5's, unchanged (single-capture rule, D/2 spacing, noise <= D/3, theta_max).
"""
from __future__ import annotations

import argparse
import json
import logging
import shutil
import sys
import time
from pathlib import Path

import numpy as np

SPEEDS = (50.0, 100.0, 200.0)     # mm/s (A); 50 is 1.5's default, 200 its fast_speed
JOINT_SPEED_SHARE = 0.9           # A: a sweep may use at most this share of any joint's speed limit
MOVE_S = 2.0                      # D: 2.5's planned moves, 26-29 s over 14-16 views per stage (base A)
RUN_UP = 25.0                     # mm before and after each sweep (view_reach.py RUN_UP, A)
STAGES = ['pre_busbar', 'pre_weld', 'post_weld']
ASSUMED_MOVE = 1.5                # what 1.5's plan 'seconds' includes (view_reach.py ASSUMED_MOVE)

log = logging.getLogger('inspection_plan')


def setup_stage(stage, q, cfg):
    from defectgen import coverage as C
    from defectgen.detect import Reference
    ref = Reference(stage, cfg)
    el = C.build_elements(ref.groups, q)
    C.observe(el, ref.scene, q)
    el.make_patches(q['patch_size'])
    return ref, el, C.requirements(el, stage, q)


def cmd_candidates(a):
    from defectgen import coverage as C
    from defectgen.config import load
    q, cfg = C.load_coverage_params(), load()
    views = [v for v in C.candidates(cfg, q) if not v.speed]
    out = []
    for v in views:
        cap = q['plan']['t_snapshot'] if v.kind == 'snapshot' else v.length / q['plan']['sweep_speed']
        out.append({**v.to_json(), 'seconds': ASSUMED_MOVE + cap})
    a.out.mkdir(parents=True, exist_ok=True)
    for stage in STAGES:
        (a.out / f'{stage}.plan.json').write_text(json.dumps(
            {'stage': stage, 'frame': 'module (pallet datum), mm',
             'note': 'every 1.5 candidate geometry at 1.5 sweep speed, for the reach check (2.9)',
             'views': out}, indent=1))
    print(f'{len(out)} candidate geometries ({sum(v.kind == "sweep" for v in views)} sweeps) -> {a.out}')


def joint_speed_ok(r, speed, vmax):
    """Does a sweep's stored joint path stay within JOINT_SPEED_SHARE of every joint limit
    when the sensor moves at `speed` mm/s? The path's points are evenly spaced along the
    sweep (run-up to run-out)."""
    path = np.array(r.get('path') or [])
    if len(path) < 2:
        return False
    span = (r['length'] * 1000.0 + 2 * RUN_UP) / (len(path) - 1)       # mm between points
    rate = np.abs(np.diff(path, axis=0)).max(axis=0) / span * speed     # rad/s per joint
    return bool(np.all(rate <= JOINT_SPEED_SHARE * vmax))


def offered(views, reach, vmax, q, sensors):
    """(view, seconds, pitch) for every reachable candidate at every speed it can be flown at."""
    from defectgen.coverage import View
    rate = sensors['profilometer']['reference_rate']
    by_id = {r['id']: r for r in reach}
    out, why = [], {}
    for v in views:
        r = by_id.get(v.id)
        if r is None or not r.get('ok'):
            why[v.id] = 'not reachable'
            continue
        if v.kind == 'snapshot':
            out.append((v, q['plan']['t_snapshot'] + MOVE_S, None, v.id))
            continue
        for s in SPEEDS:
            if not joint_speed_ok(r, s, vmax):
                why[f'{v.id}@{s:g}'] = 'joint speed'
                continue
            vid = v.id if s == q['plan']['sweep_speed'] else f'{v.id}_{s:g}'
            w = View(vid, v.sensor, v.kind, v.pose, v.length,
                     None if s == q['plan']['sweep_speed'] else s, s / rate)
            out.append((w, (v.length + 2 * RUN_UP) / s + MOVE_S, s / rate, v.id))
    return out, why


def solve_cover(cov, seconds, reqs, need, done, limit=600):
    """Exact minimum-time cover. cov[v][k]: bool per patch of class k; need[k]: area to reach;
    done[k]: patches already covered (the fixed pass). Returns chosen view indices."""
    from scipy.optimize import Bounds, LinearConstraint, milp
    from scipy.sparse import coo_matrix
    nv = len(seconds)
    rows, cols, vals, lo, hi = [], [], [], [], []
    obj = list(seconds)
    n_var = nv
    area_rows = []
    for k, r in enumerate(reqs):
        M = np.array([cov[v][k] for v in range(nv)]).T & ~done[k][:, None]   # patches x views
        live = M.any(axis=1)
        if not live.any():
            area_rows.append(None)
            continue
        # merge patches covered by exactly the same views
        keys, inv = np.unique(np.packbits(M[live], axis=1), axis=0, return_inverse=True)
        inv = inv.ravel()
        area = np.bincount(inv, weights=r.areas[live], minlength=len(keys))
        sets = [np.flatnonzero(M[live][np.flatnonzero(inv == g)[0]]) for g in range(len(keys))]
        ys = range(n_var, n_var + len(keys))
        obj += [0.0] * len(keys)
        for y, s in zip(ys, sets):          # y - sum x_v <= 0
            row = len(lo)
            rows += [row] * (1 + len(s)); cols += [y] + list(s); vals += [1.0] + [-1.0] * len(s)
            lo.append(-np.inf); hi.append(0.0)
        row = len(lo)                         # sum area_y y >= need_k - done area
        rows += [row] * len(keys); cols += list(ys); vals += list(area)
        lo.append(need[k] - r.areas[done[k]].sum() - 1e-6); hi.append(np.inf)
        n_var += len(keys)
    A = coo_matrix((vals, (rows, cols)), shape=(len(lo), n_var)).tocsr()
    t0 = time.perf_counter()
    res = milp(np.array(obj), constraints=LinearConstraint(A, lo, hi), integrality=np.ones(n_var),
               bounds=Bounds(0, 1), options={'time_limit': limit, 'mip_rel_gap': 1e-4})
    if res.x is None:
        raise RuntimeError(f'no cover: {res.message}')
    log.info('  milp: %d views, %d variables, %d rows, %s, %.1f s, gap %.2g', nv, n_var, len(lo),
             res.message.split('.')[0], time.perf_counter() - t0, getattr(res, 'mip_gap', float('nan')))
    return [v for v in range(nv) if res.x[v] > 0.5], res


def cmd_select(a):
    from defectgen import coverage as C
    from defectgen.config import load
    from defectgen.scan import load_params
    sys.path.insert(0, str(Path(__file__).parent))
    q, cfg = C.load_coverage_params(), load()
    sensors, _ = load_params()
    reach = json.load(open(a.reach))
    reach_views = reach['results'][0]['views']
    if a.vmax:
        vmax = np.array(a.vmax)
    else:
        import yaml
        lim = yaml.safe_load(open(Path(__file__).resolve().parents[1] / 'config' / 'joint_limits.yaml'))['joint_limits']
        vmax = np.array([lim[f's3_joint_{j}']['max_velocity'] for j in ('1_s', '2_l', '3_u', '4_r', '5_b', '6_t')])
    report = {'speeds': SPEEDS, 'joint_speed_share': JOINT_SPEED_SHARE, 'move_s': MOVE_S, 'stages': {}}
    for stage in STAGES:
        t0 = time.perf_counter()
        ref, el, reqs = setup_stage(stage, q, cfg)
        key = C._cache_key(stage, q, el)
        src = a.cache / f'{stage}_{key}'
        dst = a.cache_local / f'{stage}_{key}'
        if not dst.is_dir() and src.is_dir():
            shutil.copytree(src, dst)
        views = [v for v in C.candidates(cfg, q)]
        fine = {v.id: v for v in views if not v.speed}
        fixed = C.fixed_baseline(cfg)
        n_fixed = C.evaluate_views([fixed], el, ref.scene, q, dst, None)[fixed.id]
        n = C.evaluate_views(list(fine.values()), el, ref.scene, q, dst, q['candidates']['sweep']['cast_pitch'])
        # 1.5's own fast sweeps were cast too: check that scaling by the pitch reproduces them.
        fast = [v for v in views if v.speed]
        nf = C.evaluate_views(fast[:3], el, ref.scene, q, dst, q['candidates']['sweep']['cast_pitch'])
        for v in fast[:3]:
            base = n[v.id[:-5]] * (fine[v.id[:-5]].pitch / v.pitch)
            err = float(np.abs(nf[v.id] - base).max() / max(1.0, float(np.abs(nf[v.id]).max())))
            if err > 1e-4:
                raise RuntimeError(f'{v.id}: pitch scaling does not reproduce the cached counts ({err:.2g})')
        cands, why = offered(list(fine.values()), reach_views, vmax, q, sensors)
        counts = [n[g] * (sensors['profilometer']['profile_pitch'] / p) if p else n[g] for _, _, p, g in cands]
        cov = [[r.covered(c, p) for r in reqs] for c, (_, _, p, _) in zip(counts, cands)]
        seconds = [s for _, s, _, _ in cands]
        live = [r for r in reqs if r.area > 0]
        v3 = json.load(open(a.v3 / f'{stage}.json'))
        fixed_ok = [r.covered(n_fixed, fixed.pitch) for r in reqs]
        reach_all = [np.any([c[k] for c in cov], axis=0) for k in range(len(reqs))]
        st = {'candidates': len(cands), 'not_offered': why, 'variants': {}}
        st['pareto'] = {}
        for variant in ('arm', 'arm+fixed'):
            done = fixed_ok if variant == 'arm+fixed' else [np.zeros(len(r.elements), bool) for r in reqs]
            v3_key = 'arm_plan' if variant == 'arm' else 'arm_plus_fixed'
            for slack in a.slack[1:]:
                need = [max(0.0, min(v3[v3_key][r.name], r.fraction(reach_all[k] | done[k])) - slack) * r.area
                        for k, r in enumerate(reqs)]
                pk, rs = solve_cover(cov, seconds, reqs, need, done, limit=60)
                st['pareto'].setdefault(variant, {})[slack] = {
                    'views': len(pk), 'gap': float(getattr(rs, 'mip_gap', 0.0) or 0.0), 'capture_s': sum(seconds[i] - MOVE_S for i in pk),
                    'estimate_s': sum(seconds[i] for i in pk), 'plan': sorted(cands[i][0].id for i in pk)}
        for variant in ('arm', 'arm+fixed'):
            done = fixed_ok if variant == 'arm+fixed' else [np.zeros(len(r.elements), bool) for r in reqs]
            v3_key = 'arm_plan' if variant == 'arm' else 'arm_plus_fixed'
            need, target = [], {}
            for k, r in enumerate(reqs):
                ceiling = r.fraction(reach_all[k] | done[k])
                t = max(0.0, min(v3[v3_key][r.name], ceiling) - a.slack[0])
                target[r.name] = {'v3': v3[v3_key][r.name], 'reachable_ceiling': ceiling, 'target': t}
                need.append(t * r.area)
            pick, res = solve_cover(cov, seconds, reqs, need, done)
            got = {}
            for k, r in enumerate(reqs):
                ok = done[k].copy()
                for i in pick:
                    ok |= cov[i][k]
                got[r.name] = r.fraction(ok)
            plan = []
            for i in sorted(pick, key=lambda i: cands[i][0].id):
                v, s, p, g = cands[i]
                cap = s - MOVE_S
                bare = q['plan']['t_snapshot'] if v.kind == 'snapshot' else v.length / (v.speed or q['plan']['sweep_speed'])
                # 'seconds' as 1.5 wrote it (1.5 s move + capture without run-up): view_reach.py
                # derives the sweep speed from it. 'capture_s' includes the run-up and run-out.
                plan.append({**v.to_json(), 'seconds': ASSUMED_MOVE + bare, 'capture_s': cap, 'geometry': g})
            st['variants'][variant] = {
                'views': len(pick), 'capture_s': sum(x['capture_s'] for x in plan),
                'estimate_s': sum(seconds[i] for i in pick), 'coverage': got, 'target': target,
                'v3_views': len(v3['plan']), 'v3_capture_s': v3['plan_seconds'] - ASSUMED_MOVE * len(v3['plan']),
                'plan': [x['id'] for x in plan]}
            d = a.out / variant.replace('+', '_')
            d.mkdir(parents=True, exist_ok=True)
            (d / f'{stage}.plan.json').write_text(json.dumps(
                {'stage': stage, 'frame': 'module (pallet datum), mm', 'params_hash': v3['params_hash'],
                 'note': f'task 2.9 minimum-time cover ({variant}, coverage slack {a.slack[0]:g}); '
                         'sensor poses: +z is the viewing direction; a sweep travels along its own +y. '
                         'seconds = 1.5 + capture without run-up, as 1.5 wrote it (view_reach.py '
                         'derives the sweep speed from it)',
                 'views': plan}, indent=1))
        report['stages'][stage] = st
        print(f'\n{stage}  ({len(cands)} reachable candidates incl. speeds; {time.perf_counter() - t0:.0f} s)')
        for variant, s in st['variants'].items():
            print(f"  {variant:9s} {s['views']:2d} views, capture {s['capture_s']:5.1f} s, est. {s['estimate_s']:5.1f} s"
                  f"   (v3: {s['v3_views']} views, capture {s['v3_capture_s']:.1f} s)")
            for name, c in s['coverage'].items():
                t = s['target'][name]
                print(f"      {name:16s} {c:6.1%}  target {t['target']:6.1%}  (v3 {t['v3']:6.1%}, "
                      f"reachable {t['reachable_ceiling']:6.1%})")
            print('      ' + ', '.join(s['plan']))
        for variant, rows in st['pareto'].items():
            print(f"  {variant:9s} slack: " + '   '.join(
                f"-{sl:.1%}: {r['views']} views {r['estimate_s']:.0f} s" + (f" (gap {r['gap']:.0%})" if r['gap'] > 1e-3 else '')
                for sl, r in rows.items()))
    (a.out / 'report.json').write_text(json.dumps(report, indent=1, default=float))
    print(f'\nwrote {a.out}/arm, {a.out}/arm_fixed, {a.out}/report.json')


def cmd_patches(a):
    """<stage>.patches.npz beside each written plan, in coverage.run_stage's format, so
    dataset.SetupCoverage can tell `not_covered` from a miss for these plans: per class and
    patch, required / fixed / arm (this plan alone) / fixed+arm (either). Rebuilt from the plan
    files, not re-solved, so the plans cannot change."""
    from defectgen import coverage as C
    from defectgen.config import load
    from defectgen.scan import load_params
    q, cfg = C.load_coverage_params(), load()
    sensors, _ = load_params()
    fine_pitch = sensors['profilometer']['profile_pitch']
    for stage in STAGES:
        ref, el, reqs = setup_stage(stage, q, cfg)
        cache = a.cache_local / f'{stage}_{C._cache_key(stage, q, el)}'
        fine = {v.id: v for v in C.candidates(cfg, q) if not v.speed}
        fixed = C.fixed_baseline(cfg)
        n_fixed = C.evaluate_views([fixed], el, ref.scene, q, cache, None)[fixed.id]
        P = len(el.p_area)

        def flags(cov):
            out = np.zeros((len(reqs), P), bool)
            for i, r in enumerate(reqs):
                out[i, r.elements[cov[i]]] = True
            return out
        req = flags([np.ones(len(r.elements), bool) for r in reqs])
        fx = flags([r.covered(n_fixed, fixed.pitch) for r in reqs])
        for d in a.plans:
            views = json.loads((d / f'{stage}.plan.json').read_text())['views']
            geo = [v.get('geometry', v['id']) for v in views]
            n = C.evaluate_views([fine[g] for g in geo], el, ref.scene, q, cache,
                                 q['candidates']['sweep']['cast_pitch'])
            ok = [np.zeros(len(r.elements), bool) for r in reqs]
            for v, g in zip(views, geo):
                pitch = v.get('pitch_mm') or (fine_pitch if v['kind'] == 'sweep' else None)
                cnt = n[g] * (fine_pitch / pitch) if pitch else n[g]
                for i, r in enumerate(reqs):
                    ok[i] |= r.covered(cnt, pitch)
            arm = flags(ok)
            np.savez_compressed(d / f'{stage}.patches.npz', centroid=el.p_centroid, normal=el.p_normal,
                                area=el.p_area, classes=np.array([r.name for r in reqs]), required=req,
                                fixed=fx, arm=arm, **{'fixed+arm': fx | arm})
            got = {r.name: f'{r.fraction((fx | arm)[i, r.elements]):.1%}' for i, r in enumerate(reqs)}
            print(f'{stage} {d}: fixed+arm {got}')


def main():
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    for name in ('defectgen', 'trimesh'):
        logging.getLogger(name).setLevel(logging.WARNING)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('candidates')
    c.add_argument('--out', type=Path, default=Path('data/plan29/candidates'))
    s = sub.add_parser('select')
    s.add_argument('--reach', type=Path, required=True, help='view_reach.py reach.json for the candidates')
    s.add_argument('--cache', type=Path, default=Path('data/coverage/cache'),
                   help="1.5's coverage cache (copied into --out/cache on first use)")
    s.add_argument('--v3', type=Path, default=Path('../MotoBat/data/coverage'), help="1.5's <stage>.json results")
    s.add_argument('--out', type=Path, default=Path('data/plan29'))
    s.add_argument('--cache-local', type=Path, default=Path('data/plan29/cache'),
                   help='where the copied coverage cache lives (shared by every --out)')
    s.add_argument('--slack', type=float, nargs='+', default=[0.0, 0.002, 0.005, 0.01, 0.02, 0.05],
                   help='coverage given up per class, as a fraction (absolute): the first sets the '
                        'written plan, the rest only print the time/coverage trade-off')
    s.add_argument('--vmax', type=float, nargs=6, default=None,
                   help='joint speed limits, rad/s (default: config/joint_limits.yaml)')
    t = sub.add_parser('patches')
    t.add_argument('plans', type=Path, nargs='+', help='folders of <stage>.plan.json written by select')
    t.add_argument('--cache-local', type=Path, default=Path('data/plan29/cache'))
    a = ap.parse_args()
    {'candidates': cmd_candidates, 'select': cmd_select, 'patches': cmd_patches}[a.cmd](a)


if __name__ == '__main__':
    main()

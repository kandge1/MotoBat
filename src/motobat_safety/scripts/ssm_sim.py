#!/usr/bin/python3
"""Speed and separation monitoring at an HC10DT cell, simulated in time. (task 3.5)

    ssm_sim.py [--plot data/safety/ssm_sim.png]          (needs ssm_design's geometry; ~1 min)

The robot flies its post-weld inspection plan (2.5's base-A plan, looped), with a speed scale s(t):
the plan's time runs at ds/dt... more exactly the plan's clock advances at rate s, so s = 1 is the
plan as timed, s = 0 is standing still. A speed command takes effect T_R = 120 ms after a person
crosses a field boundary (scanner 70 ms, S; robot safety unit 50 ms, A); s then ramps at most at
1 / T_S per second (linear braking, full speed to rest in T_S = 0.5 s, A).

Fields (ssm.Design, guarded opening of ssm_design.py; distances from the restricted space):
  core     one safety-rated STOP field at D(1) = 2.24 m; a non-safety warning field at the network
           slowdown distance commands the reduced speed (availability only, the safety case ignores it)
  network  safety-rated STOP field at D(reduced) = 1.49 m and SLOWDOWN field at 2.43 m
Reduced speed: s = 0.065 (250 mm/s at the sensors' far end, whose plan peak is 3.87 m/s).

Scenarios: a person walks at K = 1.6 m/s from the aisle (y = 4.6 m) toward the cell's centre and
  walk_in  keeps walking to the fence line (the worst case the field is sized for);
  stand    stops 3.0 m out, waits 8 s, walks away;
  pass_by  walks along the aisle past the opening, 3.6 m out.
Checked: SAFE if the robot is at rest (s = 0) before the person is within C + Z_S of the restricted
space (what the stop distance promises; reach-over is inside C). For 'core' the verdict is taken with
the non-safety warning field OFF: a safety case may not rely on a non-safety output (without that, a
20 % short stop field still passed, because the warning field had already slowed the robot).
Control: a stop field 20 % short must come out UNSAFE. Reported: that margin, the closest
horizontal approach of the person to any robot link, and the plan time lost against an empty aisle.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / 'src/motobat_description/scripts'))
import derive_opw as D  # noqa: E402
import line_envelope as L  # noqa: E402
import ssm  # noqa: E402
import ssm_design as SD  # noqa: E402

DT = 0.01
S3A_X, BASE = 1.957, np.array([1.957, -0.30, 2.00])     # line_layout.xacro (s3a)
REDUCED = 0.065


def plan_track(reach_json, stage='post_weld', sweep_dt=0.05):
    """(times, joint states) of one pass of a stage's plan; sweeps at 5 mm / 0.05 s = 100 mm/s (A)."""
    r = json.loads(Path(reach_json).read_text())['results'][0]
    s = r['sequences'][stage]
    ts, qs, now = [], [], 0.0
    for i, leg in enumerate(s['legs']):
        lt = np.array(leg['t'], float)
        ts += list(now + lt - lt[0])
        qs += leg['q']
        now = ts[-1]
        if i < len(s['order']):
            path = s['paths'][s['order'][i]]
            ts += list(now + (np.arange(len(path)) + 1) * sweep_dt)
            qs += path
            now = ts[-1]
    return np.array(ts), np.array(qs)


def robot_points_fn():
    macro = D.VENDOR / D.ARMS['hc10dt'][0]
    u = D.Urdf(macro)
    meshes = {n: v[::25] for n, v in L.link_meshes(macro).items()}
    Rm = D.rpy(math.pi, 0, math.pi / 2)

    def pts(q):
        fr = L.link_frames(u, q)
        P = np.vstack([v @ fr[n][1].T + fr[n][0] for n, v in meshes.items()] + [fr['tool0'][0][None]])
        return (P @ Rm.T + BASE)[:, :2]
    return pts


def person_path(scenario, y_fence, T):
    """Person position (x, y) over time, and walking speed K."""
    K = ssm.K_APPROACH / 1e3
    t = np.arange(0, T, DT)
    y_start = 4.6
    if scenario == 'walk_in':
        y = np.maximum(y_start - K * np.maximum(t - 2.0, 0), y_fence)
        x = np.full_like(t, S3A_X)
    elif scenario == 'stand':
        t_in = (y_start - 3.0) / K
        y = np.where(t < 2.0, y_start, np.where(t < 2.0 + t_in, y_start - K * (t - 2.0),
                     np.where(t < 10.0 + t_in, 3.0, np.minimum(3.0 + K * (t - 10.0 - t_in), y_start))))
        x = np.full_like(t, S3A_X)
    else:                                                    # pass_by
        x = S3A_X - 4.0 + K * t
        y = np.full_like(t, 3.6)
    return t, np.column_stack([x, y])


def run(scenario, variant, rs_poly, corridor, track, pts_fn, T=16.0, shrink=1.0, warning=True):
    """warning: whether the core variant's NON-safety warning field acts. The safety verdict for
    'core' is taken with it off (a non-safety output may fail); it only buys availability."""
    des = ssm.Design()
    D_stop = shrink * (des.distance(1.0) if variant == 'core' else des.distance(REDUCED)) / 1e3
    D_slow = des.slowdown_distance(REDUCED) / 1e3
    y_rs = corridor[2]
    t, P = person_path(scenario, y_rs, T)
    in_x = (P[:, 0] >= corridor[0] - 0.1) & (P[:, 0] <= corridor[1] + 0.1)   # leg circle r 0.1 m (A)
    depth = P[:, 1] - 0.1 - y_rs                                               # leg edge to the restricted space
    slow_acts = variant == 'network' or warning
    zone = np.where(in_x & (depth < D_stop), 2, np.where(in_x & (depth < D_slow) & slow_acts, 1, 0))
    lag = int(round(des.t_r / DT))
    cmd = np.concatenate([np.zeros(lag, int), zone[:-lag]])                     # what the robot is told, T_R late
    target = np.where(cmd == 2, 0.0, np.where(cmd == 1, REDUCED, 1.0))
    s = np.ones_like(t)
    rate = 1.0 / des.t_s_full * DT
    for i in range(1, len(t)):
        s[i] = s[i - 1] + np.clip(target[i] - s[i - 1], -rate, rate)
    tau = np.concatenate([[0.0], np.cumsum(s[1:] * DT)])                       # plan clock
    tt, qq = track
    period = tt[-1]
    q_at = np.array([[np.interp(x % period, tt, qq[:, j]) for j in range(6)] for x in tau[::5]])
    sep = []
    for k, q in enumerate(q_at):
        R = pts_fn(q)
        sep.append(np.min(np.linalg.norm(R - P[k * 5], axis=1)))
    sep = np.array(sep)
    reach_limit = (des.C + des.z_s) / 1e3
    danger = np.nonzero(depth <= reach_limit)[0]
    stopped = np.nonzero(s <= 1e-9)[0]
    if len(danger):
        t_danger = t[danger[0]]
        t_rest = t[stopped[0]] if len(stopped) else np.inf
        safe, margin_s = t_rest <= t_danger, t_danger - t_rest
    else:
        safe, margin_s, t_rest = True, np.inf, (t[stopped[0]] if len(stopped) else np.inf)
    lost = T - tau[-1]
    return dict(scenario=scenario, variant=variant, D_stop=D_stop, D_slow=D_slow, safe=bool(safe),
                margin_s=float(margin_s), t_rest=float(t_rest), lost_s=float(lost),
                min_sep_m=float(sep.min()), min_depth_m=float(depth.min()),
                series=dict(t=t[::5].tolist(), depth=depth[::5].tolist(), s=s[::5].tolist(), sep=sep.tolist()))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--reach', default=str(REPO / 'data/reach/base_A/reach.json'))
    ap.add_argument('--plot', type=Path)
    ap.add_argument('--out', type=Path, default=REPO / 'data/safety/ssm_sim.json')
    a = ap.parse_args()
    des = ssm.Design()
    fp = SD.swept_footprint(S3A_X, BASE, a.reach)
    rs = SD.hull(fp)
    ang = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    rs = SD.hull((rs[:, None, :] + des.z_r / 1e3 * np.stack([np.cos(ang), np.sin(ang)], 1)[None]).reshape(-1, 2))
    lo, hi = rs.min(0), rs.max(0)
    corridor = (lo[0], hi[0], hi[1], hi[1] + des.distance(1.0) / 1e3)
    track = plan_track(a.reach)
    pts_fn = robot_points_fn()
    results = []
    print(f'restricted space edge y {hi[1]:.2f} m; reach-over limit C + Z_S = {(des.C + des.z_s) / 1e3:.3f} m')
    print('| scenario | variant | stop field | slowdown | SAFE | robot at rest before the person could reach | '
          'closest to a robot link | plan time lost (16 s) |')
    print('|---|---|---|---|---|---|---|---|')
    for scen in ('walk_in', 'stand', 'pass_by'):
        for var in ('core', 'network'):
            r = run(scen, var, rs, corridor, track, pts_fn)                 # availability: as operated
            if var == 'core':                                               # safety: non-safety warning off
                rs_ = run(scen, var, rs, corridor, track, pts_fn, warning=False)
                r.update(safe=rs_['safe'], margin_s=rs_['margin_s'], t_rest=rs_['t_rest'])
            results.append(r)
            m = f'{r["margin_s"]:.2f} s early' if np.isfinite(r['margin_s']) else 'never needed'
            print(f'| {scen} | {var} | {r["D_stop"]:.2f} m | {r["D_slow"]:.2f} m | {"yes" if r["safe"] else "NO"} | {m} | '
                  f'{r["min_sep_m"]:.2f} m | {r["lost_s"]:.2f} s |')
    # Control: the check must be able to fail. A stop field 20 % short must be reported unsafe.
    ctrl = run('walk_in', 'core', rs, corridor, track, pts_fn, shrink=0.8, warning=False)
    ok_ctrl = not ctrl['safe']
    print(f'control: core stop field shrunk to {ctrl["D_stop"]:.2f} m, walk_in: '
          f'{"UNSAFE as it must be" if ok_ctrl else "still safe: THE CHECK CANNOT FAIL"} '
          f'(robot at rest {ctrl["margin_s"]:+.2f} s relative to the reach limit)')
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(results))
    if not (all(r['safe'] for r in results) and ok_ctrl):
        print('FAIL')
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 3, figsize=(15, 6.5), sharex=True)
        for j, scen in enumerate(('walk_in', 'stand', 'pass_by')):
            for var, c in (('core', 'tab:red'), ('network', 'tab:blue')):
                r = next(x for x in results if x['scenario'] == scen and x['variant'] == var)
                S = r['series']
                ax[0, j].plot(S['t'], S['depth'], color='0.3', lw=1)
                ax[0, j].axhline(r['D_stop'], color=c, ls='--', lw=0.8, label=f'{var} stop field')
                ax[1, j].plot(S['t'], S['s'], color=c, label=f'{var}: speed scale')
            ax[0, j].axhline(r['D_slow'], color='tab:blue', ls=':', lw=0.8, label='network slowdown field')
            ax[0, j].axhline((des.C + des.z_s) / 1e3, color='k', ls='-.', lw=0.8, label='C + Z_S (reach)')
            ax[0, j].set_title(scen)
            ax[0, j].set_ylim(0, 3.6)
            ax[1, j].set_xlabel('t, s')
            ax[0, j].grid(alpha=0.3)
            ax[1, j].grid(alpha=0.3)
        ax[0, 0].set_ylabel('person to restricted space, m')
        ax[1, 0].set_ylabel('robot speed scale s')
        ax[0, 0].legend(fontsize=7)
        ax[1, 0].legend(fontsize=7)
        fig.suptitle('HC10DT speed and separation monitoring, simulated (S3a, post-weld plan; K 1.6 m/s)')
        fig.tight_layout()
        a.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(a.plot, dpi=100)
        print(f'plot: {a.plot}')


if __name__ == '__main__':
    main()

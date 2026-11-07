#!/usr/bin/python3
"""
Compare IK solvers on the HC10DT, on identical targets. (task 2.1)

For each solver config in config/ik_candidates/, this starts move_group with
that solver alone (launch/ik_compare.launch.py), confirms the solver actually
loaded, runs every target set through check_ik.evaluate, and shuts it down.
Targets are generated once and saved, so every solver sees the same poses.

A target counts as reachable if any solver solves it. A solver's success rate
is over reachable targets, so every miss is a real solution the solver did not
find. Move-time bounds are compared only on targets every solver solved.

    ros2 run motobat_hc10dt_moveit_config compare_ik.py
    ros2 run motobat_hc10dt_moveit_config compare_ik.py --samples 500 --out data/ik_compare

Heavy-ish: one move_group at a time, but don't run it alongside a Gazebo or
RViz session in the other lane. Uses system Python 3.12.
"""
import argparse
import json
import os
import random
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import check_ik  # noqa: E402

PKG = 'motobat_hc10dt_moveit_config'
SETS = ['random', 'down', 'cone']


def loaded_solver(node):
    """Ask the running move_group which IK plugin it loaded."""
    name = f'robot_description_kinematics.{check_ik.GROUP}.kinematics_solver'
    cli = node.create_client(GetParameters, '/move_group/get_parameters')
    if not cli.wait_for_service(timeout_sec=30.0):
        return None
    res = node.call(cli, GetParameters.Request(names=[name]))
    return res.values[0].string_value if res and res.values else None


def start_move_group(kin_path, log_path):
    log = open(log_path, 'w')
    return subprocess.Popen(
        ['ros2', 'launch', PKG, 'ik_compare.launch.py', f'kinematics:={kin_path}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def stop(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass


def other_move_group_running():
    out = subprocess.run(['pgrep', '-f', 'moveit_ros_move_group/move_group'],
                         capture_output=True, text=True).stdout.split()
    return [p for p in out if int(p) != os.getpid()]


def summarise(results, names):
    """Markdown table: one row per (set, solver)."""
    lines = ['| set | solver | solved / reachable | solve time median / p95 (ms) '
             '| time on a miss, median (ms) | move-time bound, median (s) |',
             '|---|---|---|---|---|---|']
    for s in SETS:
        recs = {n: results[n][s] for n in names}
        count = len(next(iter(recs.values())))
        reachable = [i for i in range(count) if any(recs[n][i]['solved'] for n in names)]
        common = [i for i in reachable if all(recs[n][i]['solved'] for n in names)]
        for n in names:
            ok = [recs[n][i] for i in reachable if recs[n][i]['solved']]
            miss = [recs[n][i] for i in reachable if not recs[n][i]['solved']]
            t = [r['time'] * 1e3 for r in ok]
            mt = [recs[n][i]['move_time'] for i in common]
            lines.append(
                f'| {s} | {n} | {len(ok)}/{len(reachable)} = {100 * len(ok) / max(1, len(reachable)):.1f} % '
                f'| {statistics.median(t) if t else float("nan"):.1f} / {check_ik.pct(t, 0.95):.1f} '
                f'| {statistics.median([r["time"] * 1e3 for r in miss]) if miss else float("nan"):.1f} '
                f'| {statistics.median(mt) if mt else float("nan"):.2f} |')
        lines.append(f'| {s} | *{count - len(reachable)} of {count} targets solved by no solver; '
                     f'{len(common)} solved by all* | | | | |')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--samples', type=int, default=300, help='targets per set')
    ap.add_argument('--timeout', type=float, default=0.05, help='IK timeout per call, s')
    ap.add_argument('--rng', type=int, default=0)
    ap.add_argument('--out', default=os.path.join(os.environ.get('MOTOBAT_ROOT', '.'),
                                                   'data', 'ik_compare'))
    ap.add_argument('--regen', action='store_true', help='regenerate targets even if saved')
    args = ap.parse_args()

    if other_move_group_running():
        sys.exit('Another move_group is running. Stop it first: its /compute_ik would answer '
                 'instead of the solver under test.')

    share = Path(get_package_share_directory(PKG))
    candidates = sorted((share / 'config' / 'ik_candidates').glob('*.yaml'))
    names = [c.stem for c in candidates]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()

    results = {}
    for cand in candidates:
        print(f'\n=== {cand.stem}', flush=True)
        proc = start_move_group(cand, out / f'move_group_{cand.stem}.log')
        try:
            node = check_ik.IkClient()
            solver = loaded_solver(node)
            want = check_ik_solver_name(cand)
            if solver != want:
                sys.exit(f'move_group loaded {solver!r}, expected {want!r}. Aborting.')
            print(f'  move_group has {solver}', flush=True)

            for s in SETS:                       # targets: generate once, then reuse
                tfile = out / f'targets_{s}.json'
                if args.regen or not tfile.exists():
                    rng = random.Random(f'{args.rng}-{s}')
                    targets = check_ik.make_targets(node, s, args.samples, rng)
                    json.dump({'kind': s, 'rng': args.rng, 'frame': check_ik.FRAME,
                               'tip': check_ik.TIP, 'targets': targets}, open(tfile, 'w'))
                    print(f'  generated {len(targets)} {s} targets -> {tfile}', flush=True)
            args.regen = False

            results[cand.stem] = {}
            for s in SETS:
                targets = json.load(open(out / f'targets_{s}.json'))['targets']
                t0 = time.perf_counter()
                recs = check_ik.evaluate(node, targets, args.timeout)
                results[cand.stem][s] = recs
                print(f'  {s:6s} {sum(r["solved"] for r in recs):4d}/{len(recs)} solved '
                      f'({time.perf_counter() - t0:.0f} s)', flush=True)
            node.destroy_node()
        finally:
            stop(proc)
            time.sleep(2)                        # let DDS forget the old move_group

    rclpy.shutdown()
    table = summarise(results, names)
    json.dump(results, open(out / 'results.json', 'w'))
    (out / 'summary.md').write_text(
        f'IK comparison, HC10DT inspector_arm, tip {check_ik.TIP}. Timeout '
        f'{args.timeout * 1e3:.0f} ms per call, seed = SRDF ready, acceptance '
        f'{check_ik.POS_TOL * 1e3:.1f} mm / {check_ik.ROT_TOL:.3f} rad.\n\n{table}\n')
    print('\n' + table + f'\n\nWritten to {out}/')


def check_ik_solver_name(yaml_path):
    """The kinematics_solver string a candidate file asks for."""
    for line in open(yaml_path):
        if 'kinematics_solver:' in line:
            return line.split(':', 1)[1].strip()
    return None


if __name__ == '__main__':
    main()

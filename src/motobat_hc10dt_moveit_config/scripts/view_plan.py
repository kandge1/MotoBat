#!/usr/bin/python3
"""
One command for the 2.5 view-plan viewer: RViz, the module, and the HC10DT flying
a stage's views from a named base. Ctrl-C stops all of it.

    ros2 run motobat_hc10dt_moveit_config view_plan.py --base A --scan weld
    ros2 run motobat_hc10dt_moveit_config view_plan.py --list

Bases are named in config/view_bases.yaml. The first time a base is shown (or
after its entry changes, or with --recompute) its plan is computed: move_group
headless checks every view, orders each stage, and plans every move with OMPL.
About a minute; the result goes to data/reach/base_<NAME>/reach.json.

Each process runs under its own memory cap (systemd-run), together under 2.5 GB:
RViz 1.2 GB, module 0.5 GB, player 0.3 GB, plan compute 1 GB (before the others).
Run from the MotoBat-arm folder with its motobat_env.sh sourced.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
PKG = 'motobat_hc10dt_moveit_config'
BASES = HERE.parent / 'config' / 'view_bases.yaml'
STAGES = ['pre_busbar', 'pre_weld', 'post_weld']
SCAN = {'all': STAGES, 'busbar': ['pre_busbar'], 'pre_busbar': ['pre_busbar'],
        'preweld': ['pre_weld'], 'pre_weld': ['pre_weld'], 'weld': ['post_weld'],
        'postweld': ['post_weld'], 'post_weld': ['post_weld']}
# RViz holds every displayed point (~1.9 M with --scans): 1.2 GB was too little and the cap killed
# it (CONSTRAINT_MEMCG, 18 Nov). Watching takes ~4 GB in all: don't run scan_view_plan.py alongside.
CAPS = {'compute': '1G', 'rviz': '2500M', 'module': '400M', 'player': '500M', 'detections': '300M'}
COMPUTE_DOMAIN = '43'   # its own move_group, kept off the viewer's domain


def capped(cmd, cap):
    if shutil.which('systemd-run'):
        return ['systemd-run', '--user', '--scope', '-q', '-p', f'MemoryMax={cap}',
                '-p', 'MemorySwapMax=0'] + cmd
    return cmd


def stop(proc, name):
    """SIGINT the child's own process group (the player shares ours: signal it alone)."""
    if proc is None or proc.poll() is not None:
        return
    own = os.getpgid(proc.pid) != os.getpgrp()
    if not own:              # it got the terminal's Ctrl-C too: give it time to finish
        try:
            proc.wait(timeout=4)
            return
        except subprocess.TimeoutExpired:
            pass
    try:
        os.killpg(proc.pid, signal.SIGINT) if own else proc.send_signal(signal.SIGINT)
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL) if own else proc.kill()
        print(f'  ({name} did not stop on Ctrl-C: killed)')
    except ProcessLookupError:
        pass


def check_env():
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory(PKG)).resolve()
    if not str(share).startswith(str(REPO)):
        sys.exit(f'{PKG} resolves to {share}, not this folder ({REPO}).\n'
                 f'In this terminal: cd {REPO} && source motobat_env.sh')


def load_bases():
    return yaml.safe_load(open(BASES))['bases']


def result_path(name):
    return REPO / 'data' / 'reach' / f'base_{name}' / 'reach.json'


def up_to_date(name, b):
    p = result_path(name)
    if not p.exists():
        return False
    d = json.load(open(p))
    r = d['results'][0]
    have = (r['base']['bx'], r['base']['d'], r['base']['bz'], r['base'].get('mount'), d.get('pad'),
            d.get('conveyor_lift'))
    want = (b['bx'], b['d'], b['bz'], b['mount'], b['pad'], b.get('conveyor_lift'))
    # plans from before 18 Nov have no transit floor and sparse sweeps: recompute those
    same = lambda x, y: x == y or (isinstance(x, (int, float)) and isinstance(y, (int, float))
                                   and abs(x - y) < 1e-9)
    # planner 3 (18 Nov): IK branches chosen per sequence; joint moves, raised waypoints, then OMPL
    return ('sequences' in r and 'transit_floor' in r and r.get('planner', 1) >= 3
            and all(same(x, y) for x, y in zip(have, want)))


def compute(name, b):
    out = result_path(name).parent
    out.mkdir(parents=True, exist_ok=True)
    print(f'computing base {name} (about a minute): {b}', flush=True)
    env = dict(os.environ, ROS_DOMAIN_ID=COMPUTE_DOMAIN)
    cmd = capped(['/usr/bin/python3', str(HERE / 'view_reach.py'), '--coverage',
                  str(REPO / 'data' / 'coverage'), '--out', str(out), '--plan',
                  '--base', str(b['bx']), str(b['d']), str(b['bz']),
                  '--mount', b['mount'], '--pad', str(b['pad'])]
                 + (['--conveyor-lift', str(b['conveyor_lift'])] if b.get('conveyor_lift') is not None else []),
                 CAPS['compute'])
    if subprocess.run(cmd, env=env, cwd=REPO).returncode != 0:
        sys.exit(f'computing base {name} failed; see {out}/move_group.log')


def summary(name, b):
    print(f"base {name}: {b['mount']}, {b['bx']:+.2f} m along the module, {b['d']:.2f} m from its "
          f"centreline, {b['bz']:+.2f} m from the pallet top; keep-out {b['pad'] * 100:.0f} cm")
    if b.get('note'):
        print(f"  {b['note']}")
    if not up_to_date(name, b):
        print('  (not computed yet)')
        return
    r = json.load(open(result_path(name)))['results'][0]
    for s in STAGES:
        q = r['sequences'][s]
        print(f"  {s:10s} {len(q['order']):2d} views  cycle {q['cycle_s']:5.1f} s "
              f"(moves {q['move_s']:4.1f} + captures {q['capture_s']:4.1f})"
              + (f"  NOT REACHABLE: {', '.join(q['missing'])}" if q['missing'] else ''))


DEFAULT_MODULE = REPO.parent / 'MotoBat' / 'data' / 'preview_clean'


def scan_cmd(name, base, stage):
    st = {'weld': 'post_weld', 'busbar': 'pre_busbar', 'preweld': 'pre_weld', 'postweld': 'post_weld'}.get(stage, stage)
    mod = '' if name == 'weld_walls' else f' --module <defectgen run> --name {name}'
    return (f'mb-venv && PYTHONPATH=$PWD/defectgen/src python src/{PKG}/scripts/scan_view_plan.py'
            f' --stage {st} --base {base}{mod}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', default='A', help='a name from config/view_bases.yaml (default A)')
    ap.add_argument('--scan', default='weld', choices=sorted(SCAN),
                    help='which stage(s) to fly; weld = post_weld (default), all = the three in turn')
    ap.add_argument('--view', default=None, help='only this view: its number on screen, or its id')
    ap.add_argument('--sensor', choices=['snapshot', 'profilometer'], default=None,
                    help='only this sensor\'s views')
    ap.add_argument('--rate', type=float, default=1.0, help='playback speed, 0.3 = 0.3x real time')
    ap.add_argument('--step', action='store_true', help='stop at every view until Enter')
    ap.add_argument('--once', action='store_true', help='fly once, then leave RViz open until Ctrl-C')
    ap.add_argument('--no-rviz', action='store_true', help='RViz is already open (from an earlier run)')
    ap.add_argument('--module', type=Path, default=DEFAULT_MODULE,
                    help='defectgen run to show as the module (default ../MotoBat/data/preview_clean)')
    ap.add_argument('--scans', default=None, metavar='NAME',
                    help='show simulated scans and detections from data/arm_scans/NAME '
                         '(made by scan_view_plan.py; e.g. weld_walls)')
    ap.add_argument('--recompute', action='store_true', help='recompute the base\'s plan')
    ap.add_argument('--list', action='store_true', help='list the bases and their cycle times')
    a = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)

    os.chdir(REPO)
    bases = load_bases()
    if a.list:
        for n, b in bases.items():
            summary(n, b)
            print()
        return
    if a.base not in bases:
        sys.exit(f'--base {a.base}: not in {BASES} (have {", ".join(bases)})')
    check_env()
    b = bases[a.base]
    if a.recompute or not up_to_date(a.base, b):
        compute(a.base, b)
    summary(a.base, b)
    scans = REPO / 'data' / 'arm_scans' / a.scans if a.scans else None
    if scans is not None:
        if not scans.is_dir():
            sys.exit(f'--scans {a.scans}: no {scans}. Make it (venv):\n  {scan_cmd(a.scans, a.base, a.scan)}')
        if (scans / 'module').is_dir() and a.module == DEFAULT_MODULE:
            a.module = scans / 'module'

    procs = {}
    try:
        if not a.no_rviz:
            procs['rviz'] = subprocess.Popen(
                capped(['ros2', 'launch', PKG, 'view_plan.launch.py'], CAPS['rviz']),
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
            time.sleep(3.0)
        stages = SCAN[a.scan]
        while True:
            for stage in stages:
                if (a.module / stage).exists():
                    stop(procs.pop('module', None), 'module')
                    procs['module'] = subprocess.Popen(
                        capped(['/usr/bin/python3', str(REPO / 'src' / 'motobat_description' /
                                                        'scripts' / 'show_module.py'),
                                str(a.module), '--stage', stage, '--hide', 'cmc', 'cover'],
                               CAPS['module']),
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                else:
                    print(f'  (no module mesh at {a.module / stage}: showing its envelope only)')
                cmd = ['/usr/bin/python3', str(HERE / 'show_view_plan.py'), str(result_path(a.base)),
                       '--stage', stage, '--rate', str(a.rate), '--name', a.base]
                stop(procs.pop('detections', None), 'detections')
                if scans is not None:
                    here = scans / f'base_{a.base}' / stage
                    if (here / 'clouds.npz').is_file():
                        cmd += ['--scans', str(here)]
                        procs['detections'] = subprocess.Popen(
                            capped(['/usr/bin/python3', str(REPO / 'src' / 'motobat_description' /
                                                            'scripts' / 'show_detections.py'),
                                    str(here), '--stage', stage], CAPS['detections']),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                    else:
                        print(f'  (no scans of {stage} from base {a.base} in {scans}; make them with\n'
                              f'   {scan_cmd(a.scans, a.base, stage)})')
                if a.view:
                    cmd += ['--view', a.view]
                if a.sensor:
                    cmd += ['--sensor', a.sensor]
                if a.step:
                    cmd.append('--step')
                if a.once or len(stages) > 1:
                    cmd.append('--once')
                procs['player'] = subprocess.Popen(capped(cmd, CAPS['player']))
                if procs['player'].wait() != 0:
                    raise KeyboardInterrupt
            if a.once:
                print('done; RViz stays open. Ctrl-C to close it.')
                while True:
                    time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # a second Ctrl-C must not cut the cleanup short
        print('\nstopping...')
        for n in ('player', 'detections', 'module', 'rviz'):
            stop(procs.get(n), n)


if __name__ == '__main__':
    main()

#!/usr/bin/python3
"""Acceptance check for a GP4 station through move_group. (task 3.3)

    ros2 launch motobat_gp4_moveit_config demo.launch.py station:=s4a use_rviz:=false
    ros2 run motobat_gp4_moveit_config gp4_check.py --station s4a          (then the same for s5)

The module arrives as it would at that station (boxes from the CAD bounds, module_spec.md §7):
    S4a (pre-carrier):  pallet, cooling plate, the cell block (cells + pads, tops at 95 mm), end
                        plates and side plates (tops at 105 mm), 24 terminals (95 - 101 mm).
    S5 (post-weld):     the same, plus the busbar carrier (95 - 98 mm under the CMC's recess) and
                        the busbars (101 - 103.6 mm).
Then every part this station handles is executed from home and back, in order:
    S4a  carrier                        S5  CMC, then cover
as home -> PTP above the magazine -> LIN pick -> attach -> LIN up -> PTP above its place ->
LIN place -> detach -> LIN up -> PTP home (Pilz = Yaskawa MOVJ / MOVL), on the gripper yaw (0 or
180 deg: the vacuum gripper is symmetric) that has a collision-free solution, preferring no wrist
turn; a PTP whose straight joint path is blocked falls back to OMPL, and is counted. Every planned
point is re-checked with /check_state_validity; every move must end within 1 mm / 0.1 deg.

Stated simplifications: the carrier rests on the cell tops with the terminals through its
openings; for collision it is its top layer only (101.5 - 102 mm, above the terminals). Parts are
0.5 mm undersize per side and 1 mm short, as at S1 (placer_check.py), so resting contact is not a
collision. Cycle times use the assumed accelerations (joint_limits, A) and leave out vacuum time.
"""
import argparse
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'motobat_gp7_moveit_config' / 'scripts'))
import placer_check as P  # noqa: E402

STATIONS = {
    's4a': dict(layout='gp4a', home=[1.7088, 0.2875, 0.3037, 0.0, -1.5869, -0.138]),
    's5': dict(layout='gp4b', home=[1.7088, 0.2873, 0.2999, 0.0, -1.5834, -0.138]),
}
COLLISION_SIZE = {'carrier': (0.333, 0.146, 0.0015)}  # its top layer, 101.5 - 102 mm (0.5 mm over the terminals)


def module_scene(x0, top, station):
    """The module as it reaches the station, as world boxes (m)."""
    b = P.box
    objs = [b('pallet', (0.405, 0.201, 0.020), (x0, 0.0, top - 0.010)),
            b('cooling_plate', (0.355, 0.151, 0.004), (x0, 0.0, top + 0.002)),
            b('cell_block', (0.3345, 0.148, 0.091), (x0, 0.0, top + 0.004 + 0.0455))]
    for s in (-1, 1):
        objs.append(b(f'end_plate_{s:+d}', (0.01025, 0.148, 0.101), (x0 + s * 0.172375, 0.0, top + 0.004 + 0.0505)))
        objs.append(b(f'side_plate_{s:+d}', (0.355, 0.0015, 0.101), (x0, s * 0.07475, top + 0.004 + 0.0505)))
    for k in range(12):
        x = x0 + (k - 5.5) * 0.028
        flip = (k // 3) % 2 == 1
        for ly, ty in ((0.0575, 0.022), (-0.0492, 0.0385)):
            y = -ly if flip else ly
            objs.append(b(f'terminal_{k}_{ty:.3f}', (0.0185, ty, 0.006), (x, y, top + 0.098)))
    if station == 's5':
        objs.append(b('carrier', (0.333, 0.146, 0.003), (x0, 0.0, top + 0.0965)))
        for s in (-1, 1):
            objs.append(b(f'busbars_{s:+d}', (0.3165, 0.0175, 0.0026), (x0, s * 0.05475, top + 0.1023)))
    return objs


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--station', choices=list(STATIONS), required=True)
    a = ap.parse_args()
    cfg = STATIONS[a.station]
    st = a.station
    joints = [f'{st}_joint_{j}' for j in ('1_s', '2_l', '3_u', '4_r', '5_b', '6_t')]
    rclpy.init()
    n = P.Check(group='gp4_arm', tip=f'{st}_tcp', joints=joints, touch=[f'{st}_gripper', f'{st}_tcp'],
                name='gp4_check')
    with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
        xacro_in = Path(get_package_share_directory('motobat_description')) / 'urdf/gp4_station.urdf.xacro'
        subprocess.run(['xacro', str(xacro_in), f'station:={st}', '-o', f.name], check=True)
        _, mounts = P.LC.urdf_fixtures_and_mounts(f.name)
    xyz, _ = mounts[f'{st}_']
    x0, d = xyz[0], -xyz[1]
    top = P.LC.PALLET_TOP
    scene = module_scene(x0, top, st)
    n.apply(world=scene)
    home = cfg['home']
    ok = n.state_valid(home)
    print(f'{st.upper()}: GP4 base at x {x0:.3f}, {d:.3f} m off the line; home {"valid" if ok else "IN COLLISION"}; '
          f'module as it arrives: {len(scene)} boxes')
    up = P.LC.ASSUMED['approach']
    above = lambda p: (p[0], p[1], p[2] + up)  # noqa: E731
    total, ompl, placed = 0.0, 0, []
    for name, (tx, ty, tz), _, part, _ in P.LC.tasks(cfg['layout']):
        size = COLLISION_SIZE.get(part, P.LC.PARTS[part])
        mx, my, mz = P.LC.ASSUMED['magazine'][part]
        pick = (x0 + mx, -d + my, top + mz)
        place = (x0 + tx, ty, top + tz)

        def yaw_for(p, preferred=math.pi):
            seed = n.joints()
            for yaw in (preferred, preferred - math.pi):
                if n.solve(above(p), [yaw], seed)[0] is not None and n.solve(p, [yaw], seed)[0] is not None:
                    return yaw
            return None
        pick_yaw, place_yaw = yaw_for(pick), yaw_for(place)
        if pick_yaw is None or place_yaw is None:
            print(f'  {name}: no collision-free IK at its {"pick" if pick_yaw is None else "place"}  FAIL')
            ok = False
            continue
        steps = [('PTP to pick_above', above(pick), pick_yaw, 'PTP', None),
                 ('LIN to pick', pick, pick_yaw, 'LIN', None),
                 ('LIN up with part', above(pick), pick_yaw, 'LIN', 'attach'),
                 ('PTP to place_above', above(place), place_yaw, 'PTP', None),
                 ('LIN to place', place, place_yaw, 'LIN', None),
                 ('LIN up, part left', above(place), place_yaw, 'LIN', 'detach')]
        part_s = 0.0
        print(f'  {name} (pick yaw {math.degrees(pick_yaw):.0f}, place yaw {math.degrees(place_yaw):.0f}):')
        for label, p, yaw, planner, action in steps:
            if action == 'attach':
                n.apply(attach=n.attached(name, size))
            if action == 'detach':
                n.apply(detach=name)
                n.apply(world=[P.placed_box(name, size, place)])
                placed.append(name)
            good, dur, err, bad = n.go(p, yaw, planner)
            how = planner
            if not good and planner == 'PTP':
                good, dur, err, bad = n.go(p, yaw, 'RRTConnectkConfigDefault', pipeline='ompl')
                how, ompl = 'OMPL', ompl + 1
            part_s += dur
            ok &= good
            print(f'    {label:20s} {how:4s} {dur:5.2f} s  end {err:6.3f} mm / {n.rot_err:.3f} deg  '
                  f'{bad} invalid planned states  {"ok" if good else "FAIL"}')
        good, dur, jerr = n.go_joints(home)
        part_s += dur
        ok &= good
        print(f'    {"PTP home":20s} PTP  {dur:5.2f} s  end {math.degrees(jerr):.3f} deg  {"ok" if good else "FAIL"}')
        print(f'    {name}: {part_s:.2f} s of motion, home to home')
        total += part_s
    print(f'{st.upper()}: {total:.2f} s of motion for the station\'s parts (accelerations A, no vacuum time); '
          f'{ompl} move(s) planned round an obstacle')
    n.apply(remove=[o.id for o in scene] + placed)
    print('PASS' if ok else 'FAIL')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

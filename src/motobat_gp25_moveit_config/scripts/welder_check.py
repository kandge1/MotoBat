#!/usr/bin/python3
"""Acceptance check for Station 4b (GP25 + MOTOPAC-RL2D) through move_group. (task 3.3b)

    ros2 launch motobat_gp25_moveit_config demo.launch.py use_rviz:=false
    ros2 run motobat_gp25_moveit_config welder_check.py

The module arrives pre-weld (busbar carrier and busbars on, no CMC; the same boxes gp4_check.py
builds for S5), inside the laser enclosure. The GP25 then makes the 24 weld holds: the scanner's
focus on each terminal's seams (x = (k - 5.5) * 28 mm, y = +-57.5 mm, 102.7 mm above the pallet
datum), in a snake (one terminal row out, the other back). The arm holds still while the scanner
draws the seam (project_plan.md §4A.7), so each hold is: Pilz PTP (Yaskawa MOVJ) to the focus pose,
then a dwell. The scanner keeps one orientation throughout (tool down, yaw 90 deg, joint 6 at 0 at
home): its field is round, so only the arm's convenience picks the yaw. A PTP whose straight
joint path is blocked falls back to OMPL, counted. Every planned point is re-checked with
/check_state_validity; every hold must end within 1 mm / 0.1 deg (the scanner's field is 25 mm
across, so the arm's hold accuracy is not the limit: the scanner's own +-30 um is, S).

Cycle: motion (accelerations A) + 24 dwells of DWELL_S (A: two 12 mm seams per terminal at
defectgen's assumed 100 mm/s, weld_process.weld_speed; Kaiser 2020's two-pass spiral took ~0.5 s,
reported as the sensitivity).

Also printed: the static wrist-load check against the GP25 data sheet (S: Yaskawa DS-873, Jan 2021:
allowable moment R/B 52 N.m, T 32 N.m; moment of inertia R/B 2.3, T 1.2 kg.m^2) for the 18.3 kg
package (S: MOTOPAC catalogue, head 15.8 + air knife 2.5 kg). Its centre of mass is not published:
assumed on the tool axis 150 mm from the flange (A); worst case over all wrist poses.
"""
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
sys.path.insert(0, str(HERE.parents[1] / 'motobat_gp4_moveit_config' / 'scripts'))
import placer_check as P  # noqa: E402
import gp4_check as G4  # noqa: E402

JOINTS = [f's4b_joint_{j}' for j in ('1_s', '2_l', '3_u', '4_r', '5_b', '6_t')]
HOME = [0.0, -0.4932, -0.3548, 0.0, -1.7092, 0.0]
YAW = math.pi / 2                    # the scanner yaw that has joint 6 at 0 at home
DWELL_S, DWELL_SENS_S = 0.25, 0.5    # A, and the sensitivity
WELD_Z = 0.1027                      # weld surface above the pallet datum (CAD)

MASS, COM = 18.3, 0.150              # kg (S), m from the flange along the tool axis (A)
C4 = 0.100                           # m, B-axis (wrist centre) to flange: derive_opw.py gp25
LIMITS = dict(R=(52.0, 2.3), B=(52.0, 2.3), T=(32.0, 1.2))   # N.m, kg.m^2 (S: DS-873)
HEAD_BOX = (0.20, 0.25, 0.25)        # m, line_tools.xacro (A)


def wrist_load():
    """Worst case over wrist poses: gravity moment and moment of inertia about R, B and T."""
    L = C4 + COM                                         # B axis to the centre of mass
    a, b, c = HEAD_BOX
    I_own_perp = MASS * (a * a + c * c) / 12             # about an axis across the tool
    I_own_axis = MASS * (a * a + b * b) / 12             # about the tool axis
    rows = []
    for ax, (m_lim, i_lim) in LIMITS.items():
        if ax == 'T':
            moment, inertia = 0.0, I_own_axis            # mass on the T axis: no gravity lever
        else:
            moment, inertia = MASS * 9.81 * L, I_own_perp + MASS * L * L
        rows.append((ax, moment, m_lim, inertia, i_lim))
    return L, rows


def main():
    rclpy.init()
    n = P.Check(group='welder_arm', tip='s4b_focus', joints=JOINTS, touch=['s4b_scanner', 's4b_focus'],
                name='welder_check')
    with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
        xacro_in = Path(get_package_share_directory('motobat_description')) / 'urdf/welder_station.urdf.xacro'
        subprocess.run(['xacro', str(xacro_in), '-o', f.name], check=True)
        _, mounts = P.LC.urdf_fixtures_and_mounts(f.name)
    xyz, _ = mounts['s4b_']
    x0 = xyz[0]
    top = P.LC.PALLET_TOP
    scene = G4.module_scene(x0, top, 's5')            # pre-weld: carrier and busbars, no CMC
    n.apply(world=scene)
    ok = n.state_valid(HOME)
    print(f'S4B: GP25 base at x {x0:.3f}, {-xyz[1]:.3f} m off the line, {xyz[2]:.3f} m up; home '
          f'{"valid" if ok else "IN COLLISION"}; module pre-weld: {len(scene)} boxes')

    L, rows = wrist_load()
    print(f'wrist load, worst case over poses: {MASS} kg (S) with its centre of mass {L * 1e3:.0f} mm from the B axis (A)')
    for ax, m, m_lim, i, i_lim in rows:
        print(f'   {ax}: moment {m:5.1f} of {m_lim:.0f} N.m ({m / m_lim:4.0%}), inertia {i:4.2f} of {i_lim} kg.m^2 ({i / i_lim:4.0%})')

    holds = [(k, +1) for k in range(12)] + [(k, -1) for k in reversed(range(12))]
    total, ompl, worst_mm = 0.0, 0, 0.0
    for k, s in holds:
        p = (x0 + (k - 5.5) * 0.028, s * 0.0575, top + WELD_Z)
        good, dur, err, bad = n.go(p, YAW, 'PTP')
        how = 'PTP'
        if not good:
            good, dur, err, bad = n.go(p, YAW, 'RRTConnectkConfigDefault', pipeline='ompl')
            how, ompl = 'OMPL', ompl + 1
        total += dur
        worst_mm = max(worst_mm, err if np.isfinite(err) else np.inf)
        ok &= good
        if not good or how != 'PTP':
            print(f'  weld {k + 1:2d}{"+" if s > 0 else "-"}  {how}  {dur:.2f} s  {"ok" if good else "FAIL"}')
    good, dur, jerr = n.go_joints(HOME)
    total += dur
    ok &= good
    dwell = len(holds) * DWELL_S
    print(f'24 holds: every focus within {worst_mm:.3f} mm of its weld; {ompl} move(s) planned round an obstacle')
    print(f'cycle: {total:.2f} s of motion (home, 24 holds, home; accelerations A) + {dwell:.1f} s of scanning '
          f'({DWELL_S} s per hold, A) = {total + dwell:.1f} s  (at {DWELL_SENS_S} s per hold: '
          f'{total + len(holds) * DWELL_SENS_S:.1f} s)')
    n.apply(remove=[o.id for o in scene])
    print('PASS' if ok else 'FAIL')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

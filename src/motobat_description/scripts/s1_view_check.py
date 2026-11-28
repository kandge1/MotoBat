#!/usr/bin/python3
"""Is the GP7 out of the S1 cameras' views when they take their pictures? (task 3.2)

The two overhead cameras (cellvision/, line_fixtures.xacro) are timed against the robot:
  infeed camera   images the NEXT cell while the GP7 is at the module placing the current one
                  (its place_above / place / retract poses);
  stack camera    images the cell just placed while the GP7 is back at the infeed picking the
                  next one (its pick_above / pick poses).
So neither picture costs cycle time, if the arm is really out of the way then. This checks it:
a robot point P blocks a camera at C if it lies between C and the measured plane and its
projection from C onto that plane,  Q = C + (P - C) (z_plane - C_z) / (P_z - C_z),  falls inside
the measured area (the target cell's top, widened by `margin`). Robot geometry: the vendor
collision meshes' surfaces (4 mm sampling), the gripper box and the part it holds, at every
task pose of layout_check's collision-free sequence. Pure numpy; layout poses from the expanded
placer station URDF (xacro).

    s1_view_check.py            exit 0 if both cameras are clear at every imaging pose (and the
                                control, the infeed camera during the picks, is blocked)
"""
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import derive_opw as D  # noqa: E402
import layout_check as LC  # noqa: E402
import line_envelope as L  # noqa: E402
from cellvision import scenes as SC  # noqa: E402

MARGIN = 10.0     # mm round the target cell top (A: the locator samples 1.5 mm beyond its edges)


def robot_points(st, q, part, d, h):
    """World points (mm) of the GP7 at q: links, gripper box, held part."""
    if st._surf is None:
        st.self_collision(q, None)                       # builds the surface samples
    fr = L.link_frames(st.urdf, q)
    pts = [st._surf[n] @ fr[n][1].T + fr[n][0] for n in LC.CHAIN[:7]]
    p0, R0 = fr['tool0']
    pts.append(st._surf['tool'] @ R0.T + p0)
    if part is not None:
        pts.append(st._part[part] @ R0.T + p0)
    P = np.vstack(pts)
    Rb, pb = D.rz(math.pi / 2), np.array([0.0, -d, h])  # base_link in world (S1 stop at x = 0)
    return (P @ Rb.T + pb) * 1e3


def blocks(P, cam_xyz, plane_z, cx, cy, yaw, half):
    """Points between camera and plane whose projection lands in the rotated rectangle."""
    C = np.asarray(cam_xyz)
    between = (P[:, 2] > plane_z) & (P[:, 2] < C[2])
    s = (plane_z - C[2]) / (P[between, 2] - C[2])
    Q = C[:2] + (P[between, :2] - C[:2]) * s[:, None]
    c, sn = math.cos(yaw), math.sin(yaw)
    a = c * (Q[:, 0] - cx) + sn * (Q[:, 1] - cy)
    b = -sn * (Q[:, 0] - cx) + c * (Q[:, 1] - cy)
    return int(((np.abs(a) <= half[0]) & (np.abs(b) <= half[1])).sum())


def main():
    from ament_index_python.packages import get_package_share_directory
    xacro_in = Path(get_package_share_directory('motobat_description')) / 'urdf/placer_station.urdf.xacro'
    with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
        subprocess.run(['xacro', str(xacro_in), '-o', f.name], check=True)
        _, mounts = LC.urdf_fixtures_and_mounts(f.name)
    xyz, _ = mounts['s1_']
    d, h = -xyz[1], xyz[2]
    st = LC.Station('gp7')
    ev, missing = st.evaluate(d, h, selfcheck=True)
    if ev is None:
        sys.exit(f'GP7 placement unreachable at {missing}')
    infeed_cam = SC.infeed_camera().t
    stack_cam = SC.stack_camera().t
    half = (SC.CELL[0] / 2 + MARGIN, SC.CELL[1] / 2 + MARGIN)
    tasks = {t[0]: t for t in LC.tasks('gp7')}
    worst = {'infeed': 0, 'stack': 0}
    checked = {'infeed': 0, 'stack': 0}
    control = []                                  # infeed camera at the pick poses: must be blocked
    for label, q in zip(ev['labels'], ev['chain']):
        name, pose = label.split(':')
        part = tasks[name][3] if pose in ('pick', 'place_above', 'place') else None
        P = robot_points(st, q, part, d, h)
        if pose == 'pick' and name.startswith('cell_'):
            control.append(blocks(P, infeed_cam, SC.CELL_TOP, SC.INFEED[0], SC.INFEED[1], 0.0, half))
        if pose in ('place_above', 'place', 'retract'):
            n = blocks(P, infeed_cam, SC.CELL_TOP, SC.INFEED[0], SC.INFEED[1], 0.0, half)
            cam = 'infeed'
        elif pose in ('pick_above', 'pick') and name.startswith('cell_') and name != 'cell_1':
            k = int(name.split('_')[1]) - 2                    # the cell placed just before this pick
            n = blocks(P, stack_cam, SC.CELL_TOP, SC.nominal_x(k), 0.0, 0.0, half)
            cam = 'stack'
        else:
            continue
        checked[cam] += 1
        worst[cam] = max(worst[cam], n)
        if n:
            print(f'  {cam:6s} camera blocked at {label}: {n} robot points in its view')
    for cam in ('infeed', 'stack'):
        print(f'{cam} camera: {checked[cam]} imaging poses checked, '
              f'{"clear at all" if worst[cam] == 0 else "BLOCKED at some"}')
    ok_control = all(n > 0 for n in control)
    print(f'control: infeed camera at the {len(control)} cell picks (arm over the infeed) blocked at '
          f'{sum(n > 0 for n in control)}: {"the test detects blocking" if ok_control else "TEST BROKEN"}')
    sys.exit(0 if not any(worst.values()) and ok_control else 1)


if __name__ == '__main__':
    main()

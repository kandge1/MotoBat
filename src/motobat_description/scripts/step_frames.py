#!/usr/bin/python3
"""
Flange and sensor-mount frames, derived numerically from the flange STEP. (task 2.4)

project_plan.md §4A.4: "Derive the URDF origins from the CAD numerically; do
not eyeball them." This script reads cad/inspector/inspector_flange.step
(via step_geometry.py) and writes config/inspector_flange.yaml, which
urdf/inspector_flange.xacro loads. Nothing in the xacro is typed by hand.

Everything is expressed in the Mount frame (the STEP assembly origin):
  origin  centre of the PCD-63 bolt pattern, on the robot-side face (z = 0)
  +Z      out of the robot, through the plate
These are checked, not assumed: the script fails if the bolt circle is not
centred on the origin, is not 63 mm, or does not lie in z = 0.

Frames written (all "mount datum" frames, exact from the CAD):
  zivid_mount   centroid of the Zivid's three screw holes (the ones the bracket
                counterbores), on the Zivid/bracket contact face; Z = viewing
                direction, X = mount X
  ljx_mount     the same for the LJ-X8200's three screw holes on its top face
The optical frames (Zivid camera origin, LJ-X laser emission / reference point)
are defined by the vendors relative to these mounting holes. They are added on
top as datasheet offsets, not guessed from the housing.

Also written: one collision box per part (vertex bounding box + 2 mm), in metres.

    step_frames.py [cad/inspector/inspector_flange.step] > src/motobat_description/config/inspector_flange.yaml
"""
import hashlib
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from step_geometry import StepAssembly  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
STEP = REPO / 'cad' / 'inspector' / 'inspector_flange.step'
PARTS = {'mount': 'Mount v3', 'zivid': 'Zivid 2 Plus M60 with FOV v1 (1)', 'ljx': 'LJ-X8200 v2'}
PAD = 0.002          # m, collision box margin
TOL = 1e-5           # m, geometric checks


def axis_hits(cyls, axis, plane_coord):
    """Points where cylinders parallel to `axis` cross the plane {x[axis] = plane_coord}."""
    out = []
    for c in cyls:
        if abs(abs(c.axis[axis]) - 1) > 1e-6:
            continue
        t = (plane_coord - c.origin[axis]) / c.axis[axis]
        out.append((c, c.origin + t * c.axis))
    return out


def unique_points(pts, tol=1e-4):
    out = []
    for p in pts:
        if all(np.linalg.norm(p - q) > tol for q in out):
            out.append(p)
    return out


def frame(origin, z, x):
    z = z / np.linalg.norm(z)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    R = np.column_stack([x, np.cross(z, x), z])
    return origin, R


def rpy(R):
    """URDF fixed-axis roll-pitch-yaw from a rotation matrix."""
    pitch = math.asin(-max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else STEP
    asm = StepAssembly(path)
    mount, zivid, ljx = (asm.parts[PARTS[k]] for k in ('mount', 'zivid', 'ljx'))
    notes = []

    # ---- robot interface: PCD 63 bolt circle centred on the origin, in z = 0 ------------
    bolts = unique_points([p for c, p in axis_hits(mount.cylinders, 2, 0.0) if abs(c.radius - 0.0025) < 1e-4])
    bolts = [p for p in bolts if abs(np.hypot(p[0], p[1]) - 0.0315) < 1e-4]
    if len(bolts) != 4:
        sys.exit(f'expected 4 bolt holes on PCD 63, found {len(bolts)}')
    centre = np.mean(bolts, axis=0)
    pcd = 2 * np.mean([np.hypot(*(p - centre)[:2]) for p in bolts])
    assert np.linalg.norm(centre) < TOL, f'bolt circle centre {centre} is not the mount origin'
    assert abs(pcd - 0.063) < TOL, f'PCD {pcd * 1e3:.4f} mm, expected 63'
    assert any(abs(p.normal[2]) > 1 - 1e-9 and abs(p.origin[2]) < TOL for p in mount.planes), \
        'no planar face in z = 0'
    bolt_diam = 2e3 * 0.0025
    dowels = unique_points([p for c, p in axis_hits(mount.cylinders, 2, 0.0) if abs(c.radius - 0.003) < 1e-4])
    if len(dowels) != 1:
        sys.exit(f'expected one Ø6 dowel hole, found {len(dowels)}')
    dowel = dowels[0]
    dowel_r, dowel_ang = np.hypot(dowel[0], dowel[1]), math.degrees(math.atan2(dowel[1], dowel[0]))
    notes.append(f'bolt holes are Ø{bolt_diam:.1f} mm: the M6 tapping drill. An M6 screw needs a '
                 f'Ø6.6 clearance hole through the plate.')

    # ---- Zivid: screw holes the bracket counterbores, on the contact face y = y_min(zivid) --
    y_contact = zivid.points[:, 1].min()
    cbores = unique_points([p for c, p in axis_hits(mount.cylinders, 1, y_contact) if c.radius > 0.005])
    zholes = unique_points([p for c, p in axis_hits(zivid.cylinders, 1, y_contact) if abs(c.radius - 0.0025) < 1e-4])
    zh = [h for h in zholes if any(np.linalg.norm(h - c) < 5e-4 for c in cbores)]
    if len(zh) != 3:
        sys.exit(f'Zivid: expected 3 counterbored screw holes, matched {len(zh)}')
    z_origin = np.mean(zh, axis=0)
    # viewing direction from the two front windows (tilted planes near the far face)
    far = zivid.points[:, 2].max()
    windows = [p for p in zivid.planes if p.normal[2] ** 2 > 0.99 and abs(p.normal[0]) > 1e-3
               and p.origin[2] > far - 0.02]
    view = np.mean([p.normal * np.sign(p.normal[2]) for p in windows], axis=0) if windows else np.array([0, 0, 1.0])
    tilts = sorted({round(math.degrees(math.acos(abs(p.normal[2]))), 2) for p in windows})
    z_frame = frame(z_origin, view, np.array([1.0, 0, 0]))
    notes.append(f'Zivid front windows are toed in by {tilts} deg; the mount frame Z is their mean, '
                 f'{np.round(view / np.linalg.norm(view), 6).tolist()}.')

    # ---- LJ-X8200: screw holes on its top face, matched to the bracket's holes -----------
    # contact face: the bracket's horizontal face nearest the head's top
    ltop = ljx.points[:, 2].min()
    z_contact = min((p.origin[2] for p in mount.planes if abs(p.normal[2]) > 1 - 1e-9),
                    key=lambda z: abs(z - ltop))
    if abs(z_contact - ltop) > 0.001:
        sys.exit(f'LJ-X8200 top ({ltop * 1e3:.3f} mm) is not on a bracket face (nearest {z_contact * 1e3:.3f})')
    if ltop < z_contact - TOL:
        notes.append(f'LJ-X8200 top reaches {ltop * 1e3:.3f} mm, {(z_contact - ltop) * 1e3:.3f} mm past the '
                     f'bracket face at {z_contact * 1e3:.3f} mm: check for interference in Fusion.')
    lholes = unique_points([p for c, p in axis_hits(ljx.cylinders, 2, z_contact) if abs(c.radius - 0.0025) < 1e-4])
    mholes = unique_points([p for c, p in axis_hits(mount.cylinders, 2, z_contact) if abs(c.radius - 0.0025) < 1e-4])
    lh = [h for h in lholes if any(np.linalg.norm(h[:2] - m[:2]) < 5e-4 for m in mholes)]
    if len(lh) != 3:
        sys.exit(f'LJ-X8200: expected 3 screw holes matching the bracket, matched {len(lh)}')
    l_frame = frame(np.mean(lh, axis=0), np.array([0, 0, 1.0]), np.array([1.0, 0, 0]))

    # ---- collision boxes ------------------------------------------------------------------
    boxes = {}
    for k in ('mount', 'zivid', 'ljx'):
        lo, hi = asm.parts[PARTS[k]].points.min(0) - PAD, asm.parts[PARTS[k]].points.max(0) + PAD
        lo[2] = max(lo[2], 0.0)   # never into the robot flange
        boxes[k] = ((lo + hi) / 2, hi - lo)

    # ---- mass properties: each part as a solid box of its datasheet mass ---------------------
    # Zivid 2+ M60 1.0 kg and LJ-X8200 head 1.2 kg (S, project_plan.md §4A.4 payload table);
    # plate + bracket + cabling 0.5 kg (A). Boxes, because the vendor shells' material is
    # meaningless (§4A.4). Good enough for Gazebo and a payload check, not for identification.
    masses = {'mount': 0.5, 'zivid': 1.0, 'ljx': 1.2}
    m_tot = sum(masses.values())
    com = sum(masses[k] * boxes[k][0] for k in masses) / m_tot
    inertia = np.zeros((3, 3))
    for k, m in masses.items():
        c, s = boxes[k]
        Ib = m / 12 * np.diag([s[1] ** 2 + s[2] ** 2, s[0] ** 2 + s[2] ** 2, s[0] ** 2 + s[1] ** 2])
        d = c - com
        inertia += Ib + m * (d @ d * np.eye(3) - np.outer(d, d))

    # ---- write ----------------------------------------------------------------------------
    sha = hashlib.sha256(path.read_bytes()).hexdigest()[:16]
    f = lambda v: '[' + ', '.join(f'{x:.6f}' for x in v) + ']'   # noqa: E731
    print('# GENERATED by motobat_description/scripts/step_frames.py. Do not hand-edit.')
    print(f'# Source: {path.relative_to(REPO) if path.is_relative_to(REPO) else path} (sha256 {sha}...)')
    print('# Mount frame = STEP assembly origin: PCD-63 bolt circle centre on the robot-side face,')
    print('# +Z out of the robot. Lengths in metres, angles in radians.')
    for n in ([asm.orphan_note] if asm.orphan_note else []) + notes:
        print(f'# NOTE: {n}')
    print('inspector_flange:')
    print(f'  step_sha256_16: "{sha}"')
    print(f'  bolt_pcd: {pcd:.6f}')
    print(f'  dowel_xyz: {f(dowel)}   # radius {dowel_r * 1e3:.3f} mm, at {dowel_ang:.2f} deg from mount +X')
    print('  frames:')
    for name, (o, R) in (('zivid_mount', z_frame), ('ljx_mount', l_frame)):
        print(f'    {name}:')
        print(f'      xyz: {f(o)}')
        print(f'      rpy: {f(rpy(R))}')
    print('  collision_boxes:')
    for k, (c, s) in boxes.items():
        print(f'    {k}:')
        print(f'      xyz: {f(c)}')
        print(f'      size: {f(s)}')
    print(f'  inertial:   # {", ".join(f"{k} {m} kg" for k, m in masses.items())}: '
          f'{m_tot:.1f} kg of the HC10DT\'s 10 kg payload')
    print(f'    mass: {m_tot:.3f}')
    print(f'    xyz: {f(com)}')
    for (i, j), n in zip([(0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2)],
                         ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']):
        print(f'    {n}: {inertia[i, j]:.6f}')


if __name__ == '__main__':
    main()

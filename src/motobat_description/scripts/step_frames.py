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


def robot_dowel_deg():
    """Angle of the HC10DT's dowel hole in tool0, measured from the vendor link_6_t visual mesh.

    The flange end face has four M6 tapped holes (Ø5, 11.5 mm deep) at r = 31.5 and one
    Ø6 H7 dowel hole, 6 mm deep. Depth is what tells them apart in a low-poly mesh."""
    import xml.etree.ElementTree as ET
    from scipy.cluster.hierarchy import fcluster, linkage
    rel = 'motoman/motoman_hc10_support/meshes/hc10dt/visual/link_6_t.dae'
    path = REPO / 'src' / '_vendor' / rel
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('motoman_hc10_support')) / 'meshes/hc10dt/visual/link_6_t.dae'
    root = ET.parse(path).getroot()
    c = root.tag.split('}')[0] + '}'
    M = np.array(root.find(f'.//{c}node/{c}matrix').text.split(), float).reshape(4, 4)
    v = np.vstack([np.array(s.text.split(), float).reshape(-1, 3) for s in root.iter(c + 'float_array')
                   if 'position' in s.get('id', '').lower()])
    v = v @ M[:3, :3].T + M[:3, 3]                      # link_6_t frame, metres
    t = np.column_stack([v[:, 0], -v[:, 1], -(v[:, 2] + 0.170)])   # tool0 = link_6_t, z -0.170, rpy (pi,0,0)
    r = np.hypot(t[:, 0], t[:, 1])
    wall = t[(r > 0.025) & (r < 0.036) & (t[:, 2] < -1e-4) & (t[:, 2] > -0.02)]
    lab = fcluster(linkage(wall[:, :2], 'single'), 0.0025, 'distance')
    holes = []
    for k in np.unique(lab):
        p = wall[lab == k]
        if len(p) >= 4:
            ctr = p[:, :2].mean(0)
            holes.append((math.degrees(math.atan2(ctr[1], ctr[0])), -p[:, 2].min() * 1e3))
    bolts = [a for a, d in holes if abs(d - 11.5) < 0.5]
    dowels = [a for a, d in holes if abs(d - 6.0) < 0.5]
    if len(bolts) != 4 or len(dowels) != 1:
        sys.exit(f'HC10DT flange mesh: expected 4 bolt holes (11.5 deep) and 1 dowel (6 deep), got {holes}')
    a = 90.0 * round(dowels[0] / 90.0)                  # midway between bolts: a multiple of 90 in tool0
    if abs(dowels[0] - a) > 3.0:
        sys.exit(f'HC10DT dowel at {dowels[0]:.1f} deg in tool0: not midway between the bolt holes')
    return a, (f'HC10DT dowel (Ø6, 6 deep) at {dowels[0]:.1f} deg in tool0, bolts (Ø5, 11.5 deep) at '
               f'{sorted(round(b) for b in bolts)} deg: vendor link_6_t mesh')


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
    notes.append(f'bolt holes are Ø{bolt_diam:.1f} mm, matching the robot flange\'s tapped holes. '
                 'Fine for the current cosmetic sheet-metal mount; a machined plate would need Ø6.6 clearance.')

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

    # ---- Zivid optical frame: Zivid's published offsets from its Ø5 positioning hole (S) ----
    # support.zivid.com, "Camera Coordinate System", Zivid 2+ M60/L110 drawings (read 9 Nov 2026):
    #   origin = optical centre of the 2D camera (imager) lens. From the Ø5 positioning hole
    #   (the round one; its partner is a slot): 32.02 mm further out, 49.00 mm toward the
    #   front, 29.5 mm above the mounting face. The positioning hole is 25 mm across and 1 mm
    #   back from the centre screw hole. Camera axis toed in 2.5 deg toward the projector.
    #   Axes: x toward the projector, y down (toward the mounting face), z out.
    loose = [h for h in zholes if not any(np.linalg.norm(h - c) < 5e-4 for c in cbores)]
    round_holes = [h for h in loose if not any(0 < np.linalg.norm(h - o) < 0.0015 for o in loose)]
    if len(round_holes) != 1:
        sys.exit(f'Zivid: expected one round Ø5 positioning hole, found {len(round_holes)}')
    pos = round_holes[0]
    centre_screw = min(zh, key=lambda h: np.linalg.norm(h - pos))
    zhat = view / np.linalg.norm(view)
    yup = np.array([0, 1.0, 0])                          # away from the mounting face (y = y_min)
    across = (pos - centre_screw) - ((pos - centre_screw) @ zhat) * zhat
    u = across / np.linalg.norm(across)                  # outward, toward the camera side
    d_across, d_back = np.linalg.norm(across), -((pos - centre_screw) @ zhat)
    if abs(d_across - 0.025) > 1e-5 or abs(d_back - 0.001) > 1e-5:
        sys.exit(f'Zivid positioning hole is {d_across * 1e3:.3f} across / {d_back * 1e3:.3f} back from '
                 'the centre screw; the datasheet says 25 / 1. Wrong hole, or the model changed.')
    o_opt = pos + 0.03202 * u + 0.04900 * zhat + 0.0295 * yup
    toe = math.radians(2.5)
    z_opt = math.cos(toe) * zhat - math.sin(toe) * u     # toed in toward the projector (-u)
    y_opt = -yup
    x_opt = np.cross(y_opt, z_opt)
    assert x_opt @ -u > 0, 'Zivid x must point toward the projector'
    zivid_optical = (o_opt, np.column_stack([x_opt, y_opt, z_opt]))
    notes.append(f'Zivid 2D camera is on mount {"+" if u[0] > 0 else "-"}X; optical centre '
                 f'{np.round(o_opt * 1e3, 2).tolist()} mm (Zivid datasheet offsets from the Ø5 positioning '
                 'hole, which the CAD matches: 25.000 across, 1.000 back).')
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

    # ---- LJ-X8200 laser frames: Keyence's dimension drawing (S), checked against the CAD ----
    # LJ-X8200 data sheet p.2 (keyence.com, downloaded 9 Nov 2026); the same dimensions are
    # on Keyence drawing AS_108280. Local copies, gitignored (vendor copyright, public repo):
    # docs/datasheets/LJ-X8200_datasheet.pdf, LJ-X8200_drawing.pdf. Head 185 x 55 x 108 mm.
    # Top face: 3 x M5, a pair 24 apart and a single 60 along from the pair, 51 from the
    # laser-end face. Laser plane 12 from that end face; laser line centred 27 from the side
    # face that carries the M3 holes at 12.5 and 42.5. Emitter face 30 above the head's far
    # face; reference distance 245 from that far face; Z range +-34 about it. Receiver 21.7 deg.
    lp = sorted(lh, key=lambda h: h[0])
    single, pair = (lp[0], lp[1:]) if abs(lp[1][0] - lp[2][0]) < 1e-5 else (lp[2], lp[:2])
    pair_mid = (pair[0] + pair[1]) / 2
    along = (pair_mid - single)
    along[2] = 0
    L = along / np.linalg.norm(along)                  # head length direction, laser end -> cable end
    W = np.cross(np.array([0, 0, 1.0]), L)             # head width direction
    proj = lambda P, d: P @ d                          # noqa: E731
    lv = ljx.points
    end_face = proj(lv, L).min()                       # laser-end face
    far_face = lv[:, 2].max()                          # the head's far face (lowest in the drawing)
    # The 27 is measured from one side face; the M3 holes are symmetric, so pick the side by
    # the emitter window's own features: the Ø2 holes around it, near the laser end.
    win = [c.origin for c in ljx.cylinders if abs(c.radius - 0.001) < 1e-4 and abs(c.axis[2]) > 0.999
           and proj(c.origin, L) - proj(lv, L).min() < 0.03]
    if len(win) < 2:
        sys.exit('LJ-X8200: cannot find the emitter window holes')
    win_w = np.mean([proj(p, W) for p in win])
    sides = (proj(lv, W).min(), proj(lv, W).max())
    side_face = min(sides, key=lambda s: abs(abs(win_w - s) - 0.027))
    if abs(abs(win_w - side_face) - 0.027) > 2e-4:
        sys.exit(f'LJ-X8200: emitter window centre is {abs(win_w - side_face) * 1e3:.2f} mm from the side; drawing: 27')
    sgn = 1.0 if side_face == sides[0] else -1.0       # measure inward from that face
    checks = {
        'M5 pair spacing 24': (np.linalg.norm(pair[0] - pair[1]), 0.024),
        'M5 single to pair 60': (np.linalg.norm(along), 0.060),
        'end face to single M5 51': (proj(single, L) - end_face, 0.051),
        'head length 185': (np.ptp(proj(lv, L)), 0.185),
        'head width 55': (np.ptp(proj(lv, W)), 0.055),
        'head height 108': (np.ptp(lv[:, 2]), 0.108),
    }
    bad = {k: v for k, (v, want) in checks.items() if abs(v - want) > 2e-5}
    if bad:
        sys.exit(f'LJ-X8200 CAD disagrees with the Keyence drawing: {bad}')
    m3 = sorted({round(abs(proj(c.origin, W) - side_face) * 1e3, 1) for c in ljx.cylinders
                 if abs(c.radius - 0.0015) < 1e-4 and abs(c.axis @ L) > 0.999})
    if m3 != [12.5, 42.5]:
        sys.exit(f'LJ-X8200: end-face M3 holes at {m3} mm from the side face; the drawing says [12.5, 42.5]')
    emitter_z = far_face - 0.030
    if not any(abs(p.normal[2]) > 1 - 1e-9 and abs(p.origin[2] - emitter_z) < 2e-5 for p in ljx.planes):
        sys.exit('LJ-X8200: no emitter face 30 mm above the far face, as the drawing has')
    laser_o = (end_face + 0.012) * L + (side_face + sgn * 0.027) * W + emitter_z * np.array([0, 0, 1.0])
    # Frame: z along the beam (out of the head), x along the laser line (the profile's X;
    # sign assumed, A: it only mirrors the profile), y along the head = the scan direction.
    ljx_laser = frame(laser_o, np.array([0, 0, 1.0]), W)
    ljx_reference = (laser_o + np.array([0, 0, 0.030 + 0.245]), ljx_laser[1])
    notes.append(f'LJ-X8200 laser line at {np.round(laser_o * 1e3, 2).tolist()} mm (emitter face); reference '
                 f'point 275 mm further along the beam. All six drawing dimensions checked against the CAD.')

    # ---- flange rotation on tool0, from the two dowel holes --------------------------------
    robot_dowel, robot_note = robot_dowel_deg()
    notes.append(robot_note)
    flange_yaw = math.radians(robot_dowel) - math.atan2(dowel[1], dowel[0])
    flange_yaw = math.atan2(math.sin(flange_yaw), math.cos(flange_yaw))

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
    print(f'  flange_yaw: {flange_yaw:.6f}   # plate about tool0 Z: robot dowel at tool0 +X, plate dowel at '
          f'{dowel_ang:.1f} deg -> {math.degrees(flange_yaw):.1f} deg')
    print('  frames:   # each relative to its parent')
    world = {'flange': (np.zeros(3), np.eye(3)), 'zivid_mount': z_frame, 'ljx_mount': l_frame,
             'zivid_optical': zivid_optical, 'ljx_laser': ljx_laser, 'ljx_reference': ljx_reference}
    for name, parent in (('zivid_mount', 'flange'), ('ljx_mount', 'flange'),
                         ('zivid_optical', 'zivid_mount'), ('ljx_laser', 'ljx_mount'),
                         ('ljx_reference', 'ljx_laser')):
        (po, pR), (o, R) = world[parent], world[name]
        print(f'    {name}:')
        print(f'      parent: {parent}')
        print(f'      xyz: {f(pR.T @ (o - po))}')
        print(f'      rpy: {f(rpy(pR.T @ R))}')
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

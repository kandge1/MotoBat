#!/usr/bin/python3
"""
OPW kinematic parameters derived from a vendor URDF, then checked. (tasks 3.1, 3.3, 3.3b)

No OPW parameter file ships for the GP7 or the GP25 (motoman_gp7_support and
motoman_gp25_support hold only urdf/ and meshes/). The GP4's does
(motoman_gp4_support/config/opw_parameters_gp4.yaml), so it is the control:
the same derivation must reproduce it exactly.

    derive_opw.py gp7                 report + checks
    derive_opw.py gp7 --yaml          parameter file on stdout
    derive_opw.py all                 gp4, gp7, gp25, and the HC10DT (must fail)

Pure numpy, no ROS: the vendor macro's <joint> tags are read directly.

----------------------------------------------------------------------------
1. What OPW is
----------------------------------------------------------------------------
OPW (Brandstötter, Angerer, Hofbaur, Austrian Robotics Workshop 2014) is a
closed-form inverse kinematics for the common six-axis industrial layout:

  * an "ortho-parallel basis": axis 1 vertical; axes 2 and 3 horizontal,
    parallel to each other and perpendicular to axis 1;
  * a spherical wrist: axes 4, 5, 6 meet in one point, the wrist centre W.

Seven lengths describe every arm of that shape (the paper's figure 1), measured
with the arm in OPW's zero pose, i.e. pointing straight up:

  c1  height of axis 2 above the base frame
  a1  horizontal offset of axis 2 from axis 1
  b   sideways offset of the arm plane (0 on every arm here)
  c2  distance from axis 2 to axis 3 (upper arm)
  c3  forearm length, axis 3 to W, measured along the forearm
  a2  elbow offset, axis 3 to W, measured across the forearm
  c4  W to the flange, along axis 6

Forward kinematics in OPW's own joint angles theta (written out, so the
inverse below can be checked against it):

  R  = Rz(t1) . Ry(t2 + t3) . Rz(t4) . Ry(t5) . Rz(t6)      tool orientation
  W  = Rz(t1) . [ a1 + c2 sin t2 + k sin(t2 + t3 + psi),
                  b,
                  c2 cos t2 + k cos(t2 + t3 + psi) ]  +  [0, 0, c1]
  p  = W + c4 . R . z                                         flange position

with k = sqrt(a2^2 + c3^2) and psi = atan2(a2, c3): the forearm is the vector
[a2, 0, c3] rotated by Ry(t2 + t3).

The inverse decouples because the wrist is spherical. Given (R, p):

  W = p - c4 . R . z              (wrist centre, independent of t4..t6)
  t1, t2, t3 from W               (a planar 2-link problem: 2 x 2 branches,
                                   shoulder front/back x elbow up/down)
  Rz(t4) Ry(t5) Rz(t6) = (Rz(t1) Ry(t2+t3))^T R     (ZYZ Euler angles: 2
                                   branches, wrist flip t5 <-> -t5)

giving up to 8 exact solutions. KDL on the HC10DT gives one, iteratively.

----------------------------------------------------------------------------
2. Reading the parameters off the URDF
----------------------------------------------------------------------------
Yaskawa's zero is not OPW's: at q = 0 the upper arm is vertical but the
forearm points forward, horizontal. The script does not assume that. It runs
URDF forward kinematics at q = 0 and reads every quantity from the joint
axes as lines in the base frame:

  check  axis 1 vertical; axes 2, 3 horizontal, parallel, perpendicular to 1;
         axes 4, 5, 6 meet in one point W (within 1 um)
  c1, a1 axis 2's height and horizontal distance from axis 1
  c2     distance between axes 2 and 3, and axis 3 must be straight above 2
  c3, a2 W - J3 in the arm plane. At Yaskawa zero the forearm is OPW's
         theta3 = +90 deg, where [a2, 0, c3] rotated by Ry(90) is [c3, 0, -a2]:
         so c3 = dx, a2 = -dz
  c4     distance from W to the flange (tool0) origin, which must lie on axis 6

Signs and offsets map the URDF's joint angle q to OPW's theta:

  theta_i = sign_i . q_i - offset_i        (the opw_kinematics library's
                                            convention, and the vendor file's)
  sign_i   = +1 if the URDF axis points the way OPW's axis i does at q = 0,
             else -1 (OPW: axis 1 +z; 2, 3, 5 +y; 4 along the forearm; 6 along
             the tool)
  offset_3 = -theta_3(q = 0) = -atan2 of the forearm direction  (-90 deg here)
  offset_6 = -(the rotation about the tool axis left between tool0 at q = 0
             and OPW's R at theta(q = 0)); the remainder must be a pure
             rotation about z, or the tool frame does not fit OPW
  offsets 1, 2, 4, 5 come out 0 from the same comparison; the script reports
  them rather than assuming them.

----------------------------------------------------------------------------
3. Checks (all must pass, else exit code 1)
----------------------------------------------------------------------------
  a. FK agreement: 20,000 random joint vectors inside the URDF limits;
     URDF chain FK vs OPW FK on the derived parameters. Pass < 1e-9 m, 1e-9 rad.
  b. IK round trip: for 5,000 random poses (FK of random q), every OPW IK
     solution's FK reproduces the pose, and the solution set contains the q
     that made it (angles compared modulo 2 pi).
  c. GP4 only: the derived parameters equal the vendor file.
  d. Reach: a1 + c2 + sqrt(c3^2 + a2^2), the wrist centre's horizontal reach,
     is compared with the Yaskawa data-sheet P-point reach (S): GP4 550 mm,
     GP7 927 mm, GP25 1,730 mm.
The HC10DT is run as a negative control: the spherical-wrist check must fail.
"""
import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

VENDOR = Path(__file__).resolve().parents[2] / '_vendor' / 'motoman'

ARMS = {
    #  name   macro file                                      data-sheet reach (S), m
    'gp4':    ('motoman_gp4_support/urdf/gp4_macro.xacro',     0.550),
    'gp7':    ('motoman_gp7_support/urdf/gp7_macro.xacro',     0.927),
    'gp25':   ('motoman_gp25_support/urdf/gp25_macro.xacro',   1.730),
    'hc10dt': ('motoman_hc10_support/urdf/hc10dt_macro.xacro', 1.200),
}
GP4_VENDOR = dict(a1=0.000, a2=-0.015, b=0.000, c1=0.330, c2=0.260, c3=0.290, c4=0.072,
                  offsets=[0, 0, -90, 0, 0, 180], signs=[1, 1, -1, -1, -1, -1])
REVOLUTE = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
TOL_GEOM = 1e-6       # m / rad: assumption checks on the URDF geometry
TOL_FK = 1e-9


# --- rotations ---------------------------------------------------------------
def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_axis(axis, a):
    """Rodrigues: rotation by a about the unit vector axis."""
    k = np.asarray(axis, float)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def rpy(r, p, y):
    """URDF rpy: fixed axes X, then Y, then Z, i.e. R = Rz(y) Ry(p) Rx(r)."""
    return rz(y) @ ry(p) @ rx(r)


def rot_angle(R):
    """Angle of a rotation matrix. atan2 form: acos((tr - 1)/2) cannot resolve below ~1e-8 rad."""
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return math.atan2(np.linalg.norm(w) / 2, (np.trace(R) - 1) / 2)


# --- URDF --------------------------------------------------------------------
def _num(text):
    """Evaluate a xacro attribute like '${radians(-170)} 0 0.330'."""
    out = []
    for tok in text.split():
        m = re.fullmatch(r'\$\{(.*)\}', tok)
        out.append(float(eval(m.group(1), {'__builtins__': {}},
                              {'radians': math.radians, 'pi': math.pi}) if m else tok))
    return out


class Urdf:
    """The base_link -> tool0 chain of a Yaskawa macro, read from its <joint> tags."""

    def __init__(self, macro, snap=False):
        """snap: replace rpy values within 1e-5 rad of a multiple of pi/2 by the exact
        multiple. The GP7 and GP25 macros write tool0's rpy as '3.1415926 -1.570796',
        pi and pi/2 cut to 7 digits (3.3e-7 rad off); the GP4 uses radians(180)."""
        root = ET.parse(macro).getroot()
        self.snapped = []
        joints = {}
        for j in root.iter('joint'):
            name = j.get('name').replace('${prefix}', '')
            o = j.find('origin')
            joints[name] = dict(
                parent=j.find('parent').get('link').replace('${prefix}', ''),
                child=j.find('child').get('link').replace('${prefix}', ''),
                xyz=np.array(_num(o.get('xyz', '0 0 0'))),
                R=rpy(*self._snap(name, _num(o.get('rpy', '0 0 0')), snap)),
                axis=np.array(_num(j.find('axis').get('xyz'))) if j.find('axis') is not None else None,
                limit=(_num(j.find('limit').get('lower'))[0], _num(j.find('limit').get('upper'))[0])
                if j.find('limit') is not None else None,
            )
        # walk tool0 back to base_link
        by_child = {v['child']: (k, v) for k, v in joints.items()}
        chain, link = [], 'tool0'
        while link != 'base_link':
            name, j = by_child[link]
            chain.append((name, j))
            link = j['parent']
        self.chain = chain[::-1]
        names = [n for n, j in self.chain if j['axis'] is not None]
        assert names == REVOLUTE, names
        self.limits = np.array([joints[n]['limit'] for n in REVOLUTE])

    def _snap(self, name, angles, snap):
        out = []
        for a in angles:
            m = round(a / (math.pi / 2)) * (math.pi / 2)
            if snap and 0 < abs(a - m) < 1e-5:
                self.snapped.append(f'{name}: {a!r} -> {m!r}')
                a = m
            out.append(a)
        return out

    def frames(self, q):
        """Per revolute joint: (origin, axis) in base_link at q; plus tool0 (p, R)."""
        T_p, T_R = np.zeros(3), np.eye(3)
        axes, i = [], 0
        for name, j in self.chain:
            T_p = T_p + T_R @ j['xyz']
            T_R = T_R @ j['R']
            if j['axis'] is not None:
                axes.append((T_p.copy(), T_R @ j['axis']))
                T_R = T_R @ rot_axis(j['axis'], q[i])
                i += 1
        return axes, T_p, T_R

    def fk(self, q):
        _, p, R = self.frames(q)
        return p, R


# --- OPW ---------------------------------------------------------------------
def opw_fk(P, theta):
    """Forward kinematics in OPW's own angles (module docstring, section 1)."""
    t1, t2, t3, t4, t5, t6 = theta
    k, psi = math.hypot(P['a2'], P['c3']), math.atan2(P['a2'], P['c3'])
    R = rz(t1) @ ry(t2 + t3) @ rz(t4) @ ry(t5) @ rz(t6)
    W = rz(t1) @ np.array([P['a1'] + P['c2'] * math.sin(t2) + k * math.sin(t2 + t3 + psi),
                           P['b'],
                           P['c2'] * math.cos(t2) + k * math.cos(t2 + t3 + psi)]) + [0, 0, P['c1']]
    return W + P['c4'] * R[:, 2], R


def to_theta(P, q):
    return np.asarray(q) * P['signs'] - np.radians(P['offsets'])


def to_q(P, theta):
    return (np.asarray(theta) + np.radians(P['offsets'])) * P['signs']


def opw_ik(P, p, R):
    """All closed-form solutions, as URDF joint angles. Up to 8; fewer if a branch is out of reach."""
    a1, a2, b, c1, c2, c3, c4 = (P[k] for k in ('a1', 'a2', 'b', 'c1', 'c2', 'c3', 'c4'))
    W = p - c4 * R[:, 2]                                  # wrist centre
    k2 = a2 * a2 + c3 * c3                                # forearm length^2 (to W)
    psi = math.atan2(a2, c3)
    out = []
    # axis 1: point the arm plane at W (front), or away from it (back, reaching over)
    # solve Rz(t1) [x, b] = W_xy with x = +r_xy (front) or -r_xy (back)
    r_xy = math.sqrt(max(W[0] ** 2 + W[1] ** 2 - b * b, 0.0))
    phi = math.atan2(W[1], W[0])
    for x in (r_xy, -r_xy):
        t1, reach = phi - math.atan2(b, x), x - a1
        # in the arm plane: W at horizontal 'reach' from axis 2, height h above it
        h = W[2] - c1
        d2 = reach * reach + h * h                        # axis 2 -> W, squared
        # law of cosines in the triangle (axis 2, axis 3, W), sides c2, sqrt(k2), sqrt(d2)
        cos_e = (d2 - c2 * c2 - k2) / (2 * c2 * math.sqrt(k2))
        if abs(cos_e) > 1 + 1e-12:
            continue                                      # out of reach on this side
        cos_e = max(-1.0, min(1.0, cos_e))
        for e in (math.acos(cos_e), -math.acos(cos_e)):   # elbow up / down
            # e is the angle between upper arm and forearm(-to-W) directions
            t3 = e - psi
            # upper arm direction: rotate so c2.u + sqrt(k2).Ry(e)u lands on (reach, h)
            sk = math.sqrt(k2)
            ax, az = c2 + sk * math.cos(e), sk * math.sin(e)   # W in the upper-arm frame (along, across)
            t2 = math.atan2(reach, h) - math.atan2(az, ax)
            R0c = rz(t1) @ ry(t2 + t3)
            M = R0c.T @ R                                 # = Rz(t4) Ry(t5) Rz(t6)
            t5 = math.atan2(math.hypot(M[0, 2], M[1, 2]), M[2, 2])
            if t5 < 1e-9:                                 # wrist singular: t4 + t6 only
                t4, t6 = 0.0, math.atan2(M[1, 0], M[0, 0])
            else:
                t4, t6 = math.atan2(M[1, 2], M[0, 2]), math.atan2(M[2, 1], -M[2, 0])
            for th in ((t1, t2, t3, t4, t5, t6), (t1, t2, t3, t4 + math.pi, -t5, t6 + math.pi)):
                q = to_q(P, th)
                out.append((q + math.pi) % (2 * math.pi) - math.pi)
    return out


# --- derivation ----------------------------------------------------------------
def line_distance(p1, d1, p2, d2):
    n = np.cross(d1, d2)
    if np.linalg.norm(n) < 1e-12:                         # parallel
        return np.linalg.norm(np.cross(p2 - p1, d1))
    return abs(np.dot(p2 - p1, n)) / np.linalg.norm(n)


def closest_point(p1, d1, p2, d2):
    """Midpoint of the common perpendicular of two non-parallel lines."""
    A = np.array([[d1 @ d1, -d1 @ d2], [d1 @ d2, -d2 @ d2]])
    s, t = np.linalg.solve(A, [(p2 - p1) @ d1, (p2 - p1) @ d2])
    return (p1 + s * d1 + p2 + t * d2) / 2


def derive(urdf):
    """Returns (params, list of (check, ok, detail))."""
    axes, p_tool, R_tool = urdf.frames(np.zeros(6))
    (o1, z1), (o2, z2), (o3, z3), (o4, z4), (o5, z5), (o6, z6) = axes
    ex, ey, ez = np.eye(3)
    checks = []

    def check(name, ok, detail=''):
        checks.append((name, bool(ok), detail))
        return ok

    check('axis 1 vertical', abs(abs(z1 @ ez) - 1) < TOL_GEOM, f'{z1}')
    check('axes 2, 3 horizontal and parallel', abs(z2 @ ez) < TOL_GEOM and abs(abs(z2 @ z3) - 1) < TOL_GEOM,
          f'{z2}, {z3}')
    check('axes 2 and 1 perpendicular', abs(z1 @ z2) < TOL_GEOM)
    d45, d46, d56 = line_distance(o4, z4, o5, z5), line_distance(o4, z4, o6, z6), line_distance(o5, z5, o6, z6)
    spherical = check('spherical wrist: axes 4, 5, 6 meet in one point',
                      max(d45, d56) < TOL_GEOM and (abs(abs(z4 @ z6) - 1) > 1e-6 or d46 < TOL_GEOM),
                      f'axis distances 4-5 {d45*1e3:.3f} mm, 5-6 {d56*1e3:.3f} mm, 4-6 {d46*1e3:.3f} mm')
    if not spherical:
        return None, checks
    W = closest_point(o4, z4, o5, z5)

    # arm plane: x along the line from axis 1 to W (Yaskawa zero points the arm at +x)
    fwd = np.array([W[0], W[1], 0.0])
    fwd /= np.linalg.norm(fwd)
    side = np.cross(ez, fwd)
    check('arm points along base +x at q = 0', abs(fwd @ ex - 1) < TOL_GEOM, f'{fwd}')
    check('axis 2 along base y (arm plane = xz)', abs(abs(z2 @ ey) - 1) < TOL_GEOM)

    c1 = o2 @ ez
    a1 = o2 @ fwd
    b = W @ side
    J3 = o3 - (o3 - o2) @ z2 * z2                         # axis 3's point in axis 2's plane
    up = J3 - o2
    c2 = np.linalg.norm(up)
    check('axis 3 straight above axis 2 at q = 0 (upper arm vertical)', np.linalg.norm(up - c2 * ez) < TOL_GEOM,
          f'{up}')
    # Forearm: OPW writes J3 -> W as Ry(t3)[a2, 0, c3] (upper arm vertical, t2 = 0).
    # c3 is measured along axis 4, so t3(q=0) is axis 4's angle from vertical;
    # its direction is u = Ry(t3) z = (sin t3, cos t3) and a2's is v = Ry(t3) x
    # = (cos t3, -sin t3), in (fwd, up) coordinates.
    u4 = z4 if (W - J3) @ z4 > 0 else -z4                 # axis 4, pointing out along the forearm
    t3_0 = math.atan2(u4 @ fwd, u4 @ ez)
    u = math.sin(t3_0) * fwd + math.cos(t3_0) * ez
    v = math.cos(t3_0) * fwd - math.sin(t3_0) * ez
    dW = W - J3
    c3, a2 = dW @ u, dW @ v
    check('W - J3 lies in the arm plane', abs(dW @ side) < TOL_GEOM)
    c4 = np.linalg.norm(p_tool - W)
    check('flange origin on axis 6', np.linalg.norm(np.cross(p_tool - W, z6)) < TOL_GEOM)

    # signs: URDF axis direction vs OPW's at theta(q=0) = (0, 0, t3_0, 0, 0, t6)
    opw_axes = [ez, ey, ey, u, ey, u]                      # OPW axes 4 and 6 lie along the forearm
    urdf_axes = [z1, z2, z3, z4, z5, z6]
    signs = []
    for i, (u, o) in enumerate(zip(urdf_axes, opw_axes)):
        c = u @ o
        check(f'axis {i+1} parallel to OPW axis {i+1}', abs(abs(c) - 1) < TOL_GEOM, f'{u}')
        signs.append(1 if c > 0 else -1)

    # theta at q = 0: t1 = 0 (arm along +x, checked), t2 = 0 (upper arm vertical,
    # checked), t3 = t3_0; t4 = t5 = 0 because axes 4 and 6 are aligned (checked
    # above via the sign test); t6 from the tool frame, below.
    P = dict(a1=a1, a2=a2, b=b, c1=c1, c2=c2, c3=c3, c4=c4,
             signs=np.array(signs), offsets=np.array([0.0, 0.0, -math.degrees(t3_0), 0.0, 0.0, 0.0]))
    # t6: rotation left about the tool axis between URDF tool0 and OPW's R at theta(q=0)
    _, R_opw = opw_fk(P, to_theta(P, np.zeros(6)))
    Rres = R_opw.T @ R_tool
    check('tool0 differs from OPW tool frame only about the tool axis',
          abs(Rres[2, 2] - 1) < TOL_GEOM, f'residual z.z = {Rres[2, 2]:.9f}')
    t6_0 = math.degrees(math.atan2(Rres[1, 0], Rres[0, 0]))
    P['offsets'][5] = -t6_0
    P['offsets'] = (P['offsets'] + 180) % 360 - 180       # report in (-180, 180]
    P['offsets'][P['offsets'] == -180] = 180
    return P, checks


def verify(urdf, P, rng, n_fk=20000, n_ik=5000, tol_q=1e-6):
    lo, hi = urdf.limits[:, 0], urdf.limits[:, 1]
    Q = rng.uniform(lo, hi, size=(n_fk, 6))
    dp = dr = 0.0
    for q in Q:
        p_u, R_u = urdf.fk(q)
        p_o, R_o = opw_fk(P, to_theta(P, q))
        dp = max(dp, np.linalg.norm(p_u - p_o))
        dr = max(dr, rot_angle(R_u.T @ R_o))
    fk = (dp < TOL_FK and dr < TOL_FK, f'max |dp| {dp:.2e} m, max |dR| {dr:.2e} rad over {n_fk}')

    worst, missing, counts = 0.0, 0, []
    for q in Q[:n_ik]:
        p, R = urdf.fk(q)
        sols = opw_ik(P, p, R)
        counts.append(len(sols))
        for s in sols:
            ps, Rs = urdf.fk(s)
            worst = max(worst, np.linalg.norm(ps - p), rot_angle(Rs.T @ R))
        wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi  # noqa: E731
        if not any(np.max(np.abs(wrap(s - q))) < tol_q for s in sols):
            missing += 1
    ik = (worst < 1e-8 and missing == 0,
          f'{n_ik} poses: every solution reproduces its pose (worst {worst:.1e}), '
          f'source q found in {n_ik - missing}/{n_ik}; solutions per pose {min(counts)}-{max(counts)}')
    return fk, ik


def as_yaml(name, P):
    f = lambda v: f'{v:.3f}'  # noqa: E731
    offs = ', '.join(f'deg({o:.1f})' if o else '0.0' for o in P['offsets'])
    return (f"# OPW parameters for the Yaskawa {name.upper()}, derived from motoman_{name}_support's URDF\n"
            f"# by motobat_description/scripts/derive_opw.py (FK-checked; see its docstring).\n"
            f"opw_kinematics_geometric_parameters:\n"
            + ''.join(f'    {k}: {" " if P[k] >= 0 else ""}{f(P[k])}\n' for k in ('a1', 'a2', 'b', 'c1', 'c2', 'c3', 'c4'))
            + f"opw_kinematics_joint_offsets: [{offs}]\n"
            f"opw_kinematics_joint_sign_corrections: [{', '.join(str(int(s)) for s in P['signs'])}]\n")


def run(name, rng, quiet=False):
    macro, reach_ds = ARMS[name]
    urdf = Urdf(VENDOR / macro, snap=True)
    P, checks = derive(urdf)
    ok = all(c[1] for c in checks)
    lines = [f'== {name.upper()}  ({macro})']
    lines += [f'  [{"ok" if c[1] else "FAIL"}] {c[0]}' + (f'  ({c[2]})' if c[2] and not c[1] else '') for c in checks]
    if P is not None and ok:
        lines.append('  params: ' + '  '.join(f'{k} {P[k]:+.4f}' for k in ('a1', 'a2', 'b', 'c1', 'c2', 'c3', 'c4')))
        lines.append(f'  offsets (deg) {list(P["offsets"])}   signs {list(P["signs"])}')
        for s in urdf.snapped:
            lines.append(f'  note: vendor rpy truncated, checked with it exact: {s}')
        (fk_ok, fk_d), (ik_ok, ik_d) = verify(urdf, P, rng)
        lines.append(f'  [{"ok" if fk_ok else "FAIL"}] FK agreement: {fk_d}')
        lines.append(f'  [{"ok" if ik_ok else "FAIL"}] IK round trip: {ik_d}')
        ok &= fk_ok and ik_ok
        if urdf.snapped:
            # the same parameters against the URDF exactly as shipped: the residual is the vendor's rounding
            # Informational: the pose error is the 3.3e-7 rad tilt alone; joint angles near
            # the wrist singularity (t5 -> 0) amplify it by 1/sin(t5), so a few sources
            # differ by more than 1e-4 rad in t4/t6 there, with the same pose.
            (_, fk_d), (_, ik_d) = verify(Urdf(VENDOR / macro), P, np.random.default_rng(1),
                                          n_fk=5000, n_ik=2000, tol_q=1e-4)
            lines.append(f'  [info] as shipped (truncated rpy): FK {fk_d}')
            lines.append(f'  [info] as shipped, source q within 1e-4 rad: {ik_d}')
        reach = P['a1'] + P['c2'] + math.hypot(P['c3'], P['a2'])
        r_ok = abs(reach - reach_ds) < 0.0015
        lines.append(f'  [{"ok" if r_ok else "FAIL"}] reach to wrist centre {reach*1e3:.1f} mm, '
                     f'data sheet {reach_ds*1e3:.0f} mm (S)')
        ok &= r_ok
        if name == 'gp4':
            same = all(abs(P[k] - GP4_VENDOR[k]) < 1e-9 for k in ('a1', 'a2', 'b', 'c1', 'c2', 'c3', 'c4')) \
                and list(P['signs']) == GP4_VENDOR['signs'] \
                and np.allclose(P['offsets'], GP4_VENDOR['offsets'])
            lines.append(f'  [{"ok" if same else "FAIL"}] equals vendor opw_parameters_gp4.yaml')
            ok &= same
    if not quiet:
        print('\n'.join(lines))
    return ok, P


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('arm', choices=list(ARMS) + ['all'])
    ap.add_argument('--yaml', action='store_true', help='print the parameter file only')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    if a.yaml:
        ok, P = run(a.arm, rng, quiet=True)
        if not ok:
            sys.exit(f'{a.arm}: checks failed, run without --yaml')
        print(as_yaml(a.arm, P), end='')
        return
    if a.arm != 'all':
        sys.exit(0 if run(a.arm, rng)[0] else 1)
    results = {n: run(n, rng)[0] for n in ARMS}
    print()
    expected = {'gp4': True, 'gp7': True, 'gp25': True, 'hc10dt': False}
    for n, ok in results.items():
        print(f'{n:7s} {"OPW" if ok else "not OPW"}   (expected {"OPW" if expected[n] else "not OPW"})')
    sys.exit(0 if results == expected else 1)


if __name__ == '__main__':
    main()

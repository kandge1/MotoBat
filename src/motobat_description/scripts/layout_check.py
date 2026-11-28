#!/usr/bin/python3
"""
Line layout, first pass: robot placements by reach search, station spacing by swept envelope. (task 3.0)

Each floor robot (GP7 cells, GP4 carrier, GP25 weld, GP4 CMC + cover) gets the same kind
of placement search 2.5 ran for the HC10DT, but with closed-form IK (derive_opw.py), so a
whole grid takes seconds and needs no ROS:

  1. Task poses, in the pallet-datum frame (module centre on the pallet top, x along the
     line), from the CAD as exported (module_spec.md §7; bounding boxes of the part STLs):
       GP7   12 cell placements at x = (k - 5.5) * 28.0 mm, top 95 mm; end plates A, B
             (top centre, x = -/+172.375 mm, 105 mm); each from its magazine pick point
       GP4a  busbar carrier, top centre (0, 0, 102 mm)
       GP25  24 weld holds, scanner focus on each terminal centre (x_k, +-57.5 mm, 102.7 mm)
       GP4b  CMC (0, 0, 101.2 mm) then cover (0, 0, 107 mm)
     Every pick and place also has its approach pose 100 mm straight above (A). The tool
     points straight down. Gripper yaw is 0 or 180 deg (a symmetric two-finger or vacuum
     gripper; the 3P4S polarity flip is a 180 deg turn of joint 6); the welding scanner may
     take any yaw (30 deg steps).
  2. Base pose: on a pedestal on the -y side of the conveyor, facing it (yaw +90 deg),
     searched over lateral offset d and base height h (the base_link plane above the
     floor), centred on the pallet stop along the line. Kept only if every task pose and
     approach has an IK solution inside the URDF joint limits.
  3. Choice: a placement must leave every joint at least MARGIN (10 %) of its range from
     the nearer limit at every task state; of those, the one with the smallest swept
     footprint (length along the line x width across it) wins. Maximising the margin
     instead was tried first: it drove the GP4s onto 1.1 m pedestals above the pallet and
     the GP25 1.4 m from the line, for a margin nobody needs.
  4. Envelope: the task sequence flown as joint moves (MOVJ, nearest branch to the
     previous pose, 20 samples per move), vendor collision meshes + gripper box + carried
     part swept (line_envelope.py). Its extent along the line, plus a clearance, sets how
     far the next station's envelope must start.

What this does NOT check (MoveIt does, per station, next): collisions with the module,
the conveyor, fences and the magazines along a move; cycle time. Joint moves here are a
stand-in for the planner's moves, so envelopes are "as sequenced", not worst case: the
real guard is the robot controller's functional-safety range limit (Yaskawa FSU), set to
the box reported here plus the clearance.

Assumed values (A) are named in ASSUMED below; change them there.

    layout_check.py                 all four floor stations: placement, envelope, spacing
    layout_check.py --station gp7   one station, with the top placements
"""
import argparse
import itertools
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import derive_opw as D  # noqa: E402
import line_envelope as L  # noqa: E402

PALLET_TOP = 0.9             # m above the floor (line_layout.xacro)
CELL_PITCH = 0.028           # module_spec.md §5 (S, the carrier is built to it)
CELL_X = [(k - 5.5) * CELL_PITCH for k in range(12)]
END_PLATE_X = 0.172375      # m, end plate centre (CAD: x 167.25 .. 177.5 mm, 10.25 thick)

ASSUMED = dict(
    approach=0.10,           # m, straight above every pick and place
    gripper_len={'gp7': 0.15, 'gp4': 0.10},          # tool0 -> TCP along tool z
    # x y z (z along tool). GP7 24 mm thick: it places a cell between neighbours of the same
    # height on a 28 mm pitch (26 Nov: was 60 mm, which would hit them; the layout was spaced
    # with 60 mm, so its envelopes are conservative by up to 18 mm a side).
    # GP4 120 mm across, body ending 10 mm short of the TCP (suction cups, A): it places the carrier
    # between side plates 148 mm apart and the CMC beside the busbars (27 Nov: was 160, flush).
    gripper_box={'gp7': (0.024, 0.20, 0.15), 'gp4': (0.06, 0.12, 0.09)},
    # MOTOPAC-RL2D: 329.5 mm work distance is S (catalogue); flange to the exit window is not
    # published: 0.25 m assumed, so tool0 -> focus = 0.5795 m. Head box 0.20 x 0.25 x 0.25.
    scanner_len=0.25 + 0.3295,
    scanner_box=(0.20, 0.25, 0.25),
    # Magazine pick points, base-relative: along the line (upstream -), outward (-y), height
    # of the part's top above the pallet top. One feed per part type, beside the robot.
    magazine={'cell': (-0.45, -0.15, 0.095), 'end_plate': (+0.45, -0.15, 0.105),
              # GP4 feeds 0.36 m along the line: the carrier/cover magazines (part + 20 mm a side)
              # then clear the GP4 pedestal; at 0.30 they came within 5 mm of it.
              'carrier': (-0.36, -0.05, 0.102), 'cmc': (-0.36, -0.05, 0.101), 'cover': (+0.36, -0.05, 0.107)},
    clearance=0.10,          # m between neighbouring swept envelopes along the line
    wall=0.05,               # laser enclosure wall thickness (S4b)
)
CONVEYOR_HALF = 0.1405       # m: belt + frame half width (inspection_line.sdf, 2.5's placeholder)
FOOT_GAP = 0.03              # m (A): base and joint-1 housing at least this far outside the conveyor
MARGIN = 0.10                # joint-limit margin every task state must keep (fraction of range)
GATE_HALF = 0.03             # line-scan gate posts, half their length along the line (inspection_line.sdf)
# HC10DT at an inspection cell: swept along the line over every planned state of 2.5's base-A
# plan, all three stages (line_envelope.py hc10dt-s3): -1.168 .. +1.074 m from the pallet stop.
HC10DT_ENV = (-1.168, 1.074)
HC10DT_YZ = ((-1.635, 1.160), (0.821, 2.322))   # same sweep, world y and z
ORDER = ['S1 gp7', 'S2 gate', 'S3a hc10dt', 'S4a gp4a', 'S3b hc10dt', 'S4b gp25', 'S3c hc10dt', 'S5 gp4b']

PARTS = {  # carried part boxes (m) as placed: x along the line, y across, z up (STL bounds, module frame)
    'cell': (0.0265, 0.148, 0.091), 'end_plate': (0.01025, 0.148, 0.101),
    'carrier': (0.333, 0.146, 0.007), 'cmc': (0.250, 0.050, 0.003), 'cover': (0.355, 0.151, 0.002),
}


def down(yaw):
    """tool0 orientation, tool z straight down, rotated by yaw about world z.
    Columns are the tool axes in world: x = (cos, sin, 0), z = (0, 0, -1)."""
    return D.rz(yaw) @ np.diag([1.0, -1.0, -1.0])


def tasks(station):
    """[(name, tcp position in the pallet frame, yaw choices, carried part or None, kind)]"""
    yaw2 = [0.0, math.pi]
    t = []
    if station == 'gp7':
        t.append(('end_plate_A', (-END_PLATE_X, 0, 0.105), yaw2, 'end_plate', 'place'))
        for k, x in enumerate(CELL_X):
            t.append((f'cell_{k+1}', (x, 0, 0.095), yaw2, 'cell', 'place'))
        t.append(('end_plate_B', (+END_PLATE_X, 0, 0.105), yaw2, 'end_plate', 'place'))
    elif station == 'gp4a':
        t.append(('carrier', (0, 0, 0.102), yaw2, 'carrier', 'place'))
    elif station == 'gp4b':
        t.append(('cmc', (0, 0, 0.1012), yaw2, 'cmc', 'place'))
        t.append(('cover', (0, 0, 0.107), yaw2, 'cover', 'place'))
    elif station == 'gp25':
        yaws = [math.radians(a) for a in range(-180, 180, 30)]
        for k, x in enumerate(CELL_X):
            for s in (-1, +1):
                t.append((f'weld_{k+1}{"-+"[s > 0]}', (x, s * 0.0575, 0.1027), yaws, None, 'hold'))
    return t


ARM = {'gp7': 'gp7', 'gp4a': 'gp4', 'gp4b': 'gp4', 'gp25': 'gp25'}


CONTACT = 0.003   # m: two non-adjacent bodies closer than this count as touching (surface sampled at 4 mm)
CHAIN = ['base_link', 'link_1_s', 'link_2_l', 'link_3_u', 'link_4_r', 'link_5_b', 'link_6_t', 'tool', 'part']


class Station:
    def __init__(self, name):
        self.name, self.arm = name, ARM[name]
        self.macro = D.VENDOR / D.ARMS[self.arm][0]
        self._surf = None
        u = D.Urdf(self.macro, snap=True)
        self.urdf = u
        self.P, _ = D.derive(u)
        self.lo, self.hi = u.limits[:, 0], u.limits[:, 1]
        self.tcp = ASSUMED['scanner_len'] if name == 'gp25' else ASSUMED['gripper_len'][self.arm]
        self.tasks = tasks(name)

    def tool_box(self):
        """(size, centre) of the end effector in tool0, as in line_tools.xacro."""
        if self.name == 'gp25':
            sx, sy, sz = ASSUMED['scanner_box']
        else:
            sx, sy, sz = ASSUMED['gripper_box'][self.arm]
        return (sx, sy, sz), (0, 0, sz / 2)

    def self_collision(self, q, part):
        """Pairs of non-adjacent bodies (links, tool, carried part) closer than CONTACT at q, as
        [(a, b, distance)]. Surfaces sampled at 4 mm; MoveIt (FCL, exact) has the last word."""
        from scipy.spatial import cKDTree
        if self._surf is None:
            self._surf = L.link_surfaces(self.macro)
            size, c = self.tool_box()
            self._surf['tool'] = L.box_surface(size, c, 0.004)
            self._part = {k: L.box_surface((v[0] - 2 * 0.0005, v[1] - 2 * 0.0005, v[2] - 0.001),
                                           (0, 0, self.tcp + (v[2] - 0.001) / 2), 0.004) for k, v in PARTS.items()}
        fr = L.link_frames(self.urdf, q)
        p0, R0 = fr['tool0']
        world = {n: self._surf[n] @ fr[n][1].T + fr[n][0] for n in CHAIN[:7]}
        world['tool'] = self._surf['tool'] @ R0.T + p0
        if part is not None:
            world['part'] = self._part[part] @ R0.T + p0
        names = [n for n in CHAIN if n in world]
        hits = []
        for i, a in enumerate(names):
            for b in names[i + 2:]:                                   # skip the adjacent body
                A, B = world[a], world[b]
                if np.any(A.max(0) + CONTACT < B.min(0)) or np.any(B.max(0) + CONTACT < A.min(0)):
                    continue
                dist, _ = cKDTree(B).query(A, k=1, distance_upper_bound=CONTACT)
                m = dist.min()
                if m < CONTACT:
                    hits.append((a, b, float(m)))
        return hits

    def base_T(self, d, h):
        """world pose of base_link: (x = pallet stop = 0 here, y = -d, z = h), yaw +90 deg."""
        return np.array([0.0, -d, h]), D.rz(math.pi / 2)

    def ik(self, p_world, R_world, d, h):
        """All in-limit solutions for tool0 at (p, R) in world (relative to the pallet stop x)."""
        pb, Rb = self.base_T(d, h)
        p = Rb.T @ (p_world - pb)
        R = Rb.T @ R_world
        out = []
        for q in D.opw_ik(self.P, p, R):
            # shift by 2 pi into the limits where the range allows (joints 4/6 span > 360 deg)
            q = np.array(q)
            for i in range(6):
                for k in (-1, 1):
                    if not (self.lo[i] <= q[i] <= self.hi[i]) and self.lo[i] <= q[i] + k * 2 * math.pi <= self.hi[i]:
                        q[i] += k * 2 * math.pi
            if np.all(q >= self.lo - 1e-9) and np.all(q <= self.hi + 1e-9):
                out.append(q)
        return out

    def margin(self, q):
        r = self.hi - self.lo
        return float(np.min(np.minimum(q - self.lo, self.hi - q) / r))

    def poses(self, d, h):
        """Sequence of (label, list of (tool0 p, R) choices) in world, pallet-stop-relative."""
        seq = []
        top = PALLET_TOP
        for name, (x, y, z), yaws, part, kind in self.tasks:
            pts = []
            if part is not None:
                mx, my, mz = ASSUMED['magazine'][part]
                pick = np.array([mx, -d + my, top + mz])
                pts += [(f'{name}:pick_above', pick + [0, 0, ASSUMED['approach']]), (f'{name}:pick', pick)]
            place = np.array([x, y, top + z])
            if kind == 'hold':
                pts += [(f'{name}:hold', place)]
            else:
                pts += [(f'{name}:place_above', place + [0, 0, ASSUMED['approach']]), (f'{name}:place', place),
                        (f'{name}:retract', place + [0, 0, ASSUMED['approach']])]
            for label, tcp in pts:
                choices = []
                for yaw in yaws:
                    R = down(yaw)
                    held = part if label.endswith((':pick', ':place_above', ':place')) else None
                    choices.append((tcp - self.tcp * R[:, 2], R, held))
                seq.append((label, choices))
        return seq

    def solve(self, d, h, selfcheck=False):
        """Per pose: every (q) over all yaw choices; None if a pose has none.
        selfcheck: keep only branches free of self-collision (links, tool, carried part)."""
        sols = []
        for label, choices in self.poses(d, h):
            qs = [(q, held) for p, R, held in choices for q in self.ik(p, R, d, h)]
            if selfcheck:
                qs = [(q, held) for q, held in qs if not self.self_collision(q, held)]
            qs = [q for q, _ in qs]
            if not qs:
                return None, label
            sols.append((label, qs))
        return sols, None

    def sequence(self, sols, home=None):
        """Nearest-branch chain through the poses (greedy from the best-margin first pose)."""
        first = max(sols[0][1], key=self.margin)
        chain = [first]
        for _, qs in sols[1:]:
            chain.append(min(qs, key=lambda q: np.sum((q - chain[-1]) ** 2)))
        return np.array(chain)

    def evaluate(self, d, h, selfcheck=False):
        sols, missing = self.solve(d, h, selfcheck)
        if sols is None:
            return None, missing
        chain = self.sequence(sols)
        return dict(d=d, h=h, margin=min(self.margin(q) for q in chain), chain=chain,
                    labels=[s[0] for s in sols]), None

    def envelope(self, ev, samples=20):
        states = []
        for a, b in zip(ev['chain'][:-1], ev['chain'][1:]):
            for t in np.linspace(0, 1, samples, endpoint=False):
                states.append(a + t * (b - a))
        states.append(ev['chain'][-1])
        pb, _ = self.base_T(ev['d'], ev['h'])
        if self.name == 'gp25':
            sx, sy, sz = ASSUMED['scanner_box']
            boxes = [dict(xyz=(0, 0, sz / 2), size=(sx, sy, sz))]
        else:
            gx, gy, gz = ASSUMED['gripper_box'][self.arm]
            boxes = [dict(xyz=(0, 0, gz / 2), size=(gx, gy, gz))]
            # the largest part this robot carries, hanging under the TCP
            # tool down at yaw 0: tool x = world x, tool y = -world y, so the part's (x, y) sizes map
            # straight onto the tool's (yaw pi only flips signs, the box is symmetric).
            big = max((PARTS[t[3]] for t in self.tasks if t[3]), key=lambda s: s[0] * s[1])
            boxes.append(dict(xyz=(0, 0, self.tcp + big[2] / 2), size=big))
        lo, hi, per = L.sweep(self.arm, np.array(states), pb, (0, 0, math.pi / 2), boxes)
        self.last_per = per
        return lo, hi


def search(st, ds, hs):
    found, misses = [], {}
    for d, h in itertools.product(ds, hs):
        ev, missing = st.evaluate(d, h)
        if ev is None:
            misses[missing] = misses.get(missing, 0) + 1
        else:
            found.append(ev)
    found.sort(key=lambda e: -e['margin'])
    return found, misses


def choose(st, ds, hs):
    found, misses = search(st, ds, hs)
    ok = []
    for e in (e for e in found if e['margin'] >= MARGIN):
        e['env'] = st.envelope(e, samples=10)
        foot = max(st.last_per[n][1][1] for n in ('base_link', 'link_1_s'))
        if foot > -(CONVEYOR_HALF + FOOT_GAP):
            continue             # base would stand in (or against) the conveyor
        lo, hi = e['env']
        e['area'] = (hi[0] - lo[0]) * (hi[1] - lo[1])
        e['foot'] = foot
        ok.append(e)
    ok.sort(key=lambda e: (e['area'], e['d'], e['h']))
    # Self-collision (costly): test candidates in order of preference, keep the first that passes
    # at every task pose with some branch. (Added 26 Nov after MoveIt found the GP7 at d 0.30
    # folding its forearm into its upper arm at every place pose.)
    passed, tested = [], 0
    for e in ok:
        tested += 1
        ev, missing = st.evaluate(e['d'], e['h'], selfcheck=True)
        if ev is None:
            misses[f'self-collision at {missing}'] = misses.get(f'self-collision at {missing}', 0) + 1
            continue
        if ev['margin'] < MARGIN:
            continue
        ev['env'] = st.envelope(ev)          # final, at full sampling, on the collision-free branches
        lo, hi = ev['env']
        ev['area'], ev['foot'] = (hi[0] - lo[0]) * (hi[1] - lo[1]), e['foot']
        foot = max(st.last_per[n][1][1] for n in ('base_link', 'link_1_s'))
        ev['foot'] = foot
        passed.append(ev)
        if len(passed) == 3:
            break
    st.selfcheck_tested = tested
    return found, passed, misses


GRIDS = {  # d (m from the conveyor centreline), h (base_link above the floor)
    'gp7': (np.arange(0.30, 0.91, 0.05), np.arange(0.20, 1.01, 0.05)),
    'gp4a': (np.arange(0.20, 0.61, 0.025), np.arange(0.40, 0.91, 0.025)),
    'gp4b': (np.arange(0.20, 0.61, 0.025), np.arange(0.40, 0.91, 0.025)),
    'gp25': (np.arange(0.40, 1.41, 0.05), np.arange(0.00, 1.01, 0.05)),
}


def layout(env):
    """Pallet stops along the line, S1 at x = 0: each envelope starts `clearance` after the last."""
    c = ASSUMED['clearance']
    xs, prev_hi = {}, None
    walls = {}
    for item in ORDER:
        tag, what = item.split()
        if what == 'gate':
            lo, hi = -GATE_HALF, GATE_HALF
        elif what == 'hc10dt':
            lo, hi = HC10DT_ENV
        else:
            lo, hi = env[what]
        if what == 'gp25':   # walls of the laser enclosure: clearance inside and out
            lo, hi = lo - c - ASSUMED['wall'], hi + c + ASSUMED['wall']
        x = 0.0 if prev_hi is None else prev_hi + c - lo
        xs[tag] = (x, what, lo, hi)
        if what == 'gp25':
            walls = dict(upstream=x + lo + ASSUMED['wall'] / 2, downstream=x + hi - ASSUMED['wall'] / 2)
        prev_hi = x + hi
    return xs, walls


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--station', choices=list(ARM))
    ap.add_argument('--plot', help='write a top + side view of the layout to this PNG')
    ap.add_argument('--verify', metavar='URDF', help='check an expanded motobat_line URDF: no robot link '
                    'enters a fixture box (xacro motobat_line.urdf.xacro -o line.urdf)')
    ap.add_argument('--reach', default=str(L.REPO / 'data/reach/base_A/reach.json'))
    ap.add_argument('--stops', metavar='URDF', help='also check the pallet stops of an expanded line URDF '
                    'against these envelopes: every neighbouring pair at least the clearance apart')
    a = ap.parse_args()
    if a.verify:
        sys.exit(1 if verify(a.verify, a.reach) else 0)
    names = [a.station] if a.station else list(ARM)
    env, chosen = {}, {}
    for n in names:
        st = Station(n)
        found, ok, misses = choose(st, *GRIDS[n])
        n_grid = len(GRIDS[n][0]) * len(GRIDS[n][1])
        print(f'== {n} ({st.arm.upper()}, TCP {st.tcp*1e3:.0f} mm from tool0, {len(st.tasks)} tasks): '
              f'{len(found)} of {n_grid} placements reach every pose; self-collision tested on '
              f'{st.selfcheck_tested} (smallest footprint first), best {len(ok)} shown')
        sc = {k: v for k, v in misses.items() if k.startswith('self')}
        if sc:
            print('   rejected for self-collision:', sorted(sc.items(), key=lambda kv: -kv[1])[:4])
        if not ok:
            print('   most often unreachable:', sorted(misses.items(), key=lambda kv: -kv[1])[:3])
            continue
        for e in ok[:3]:
            lo, hi = e['env']
            print(f'   d {e["d"]:.3f}  h {e["h"]:.3f}  margin {e["margin"]:.3f}   envelope x {lo[0]:+.2f} .. {hi[0]:+.2f}, '
                  f'y {lo[1]:+.2f} .. {hi[1]:+.2f}, z {lo[2]:.2f} .. {hi[2]:.2f}  ({e["area"]:.2f} m2; base edge y {e["foot"]:+.3f})')
        e = ok[0]
        env[n], chosen[n] = (e['env'][0][0], e['env'][1][0]), e
    if len(env) < 4:
        return
    xs, walls = layout(env)
    print(f'\nLayout, pallet stops along the line (x, m), clearance {ASSUMED["clearance"]:.2f} m between envelopes:')
    for tag, (x, what, lo, hi) in xs.items():
        extra = ''
        if what in chosen:
            e = chosen[what]
            extra = f'  base (x {x:.3f}, y {-e["d"]:.3f}, z {e["h"]:.3f}), yaw +90'
        print(f'  {tag:4s} {what:7s} x {x:6.3f}   occupies {x+lo:6.3f} .. {x+hi:6.3f}{extra}')
    print(f'  S4b laser enclosure walls (centre planes) at x {walls["upstream"]:.3f} and {walls["downstream"]:.3f}')
    last = list(xs.values())[-1]
    print(f'  line length, first to last envelope: {last[0] + last[3] - (list(xs.values())[0][2]):.2f} m')
    if a.stops:
        at = check_stops(xs, a.stops)
        xs = {t: (at[t], what, lo, hi) for t, (_, what, lo, hi) in xs.items()}   # plot the URDF's stops
        x, _, lo, hi = xs['S4b']
        walls = dict(upstream=x + lo + ASSUMED['wall'] / 2, downstream=x + hi - ASSUMED['wall'] / 2)
        print(f'  S4b laser enclosure walls at these stops: x {walls["upstream"]:.3f} and {walls["downstream"]:.3f}')
    if a.plot:
        plot(xs, walls, chosen, a.plot)


def check_stops(xs, urdf_path):
    """Gaps between neighbouring envelopes at the stops an existing URDF uses."""
    import xml.etree.ElementTree as ET
    root = ET.parse(urdf_path).getroot()
    mount = {j.get('name')[:-len('mount')]: float(j.find('origin').get('xyz').split()[0])
             for j in root.iter('joint') if j.get('name', '').endswith('mount')}
    gate = [float(j.find('origin').get('xyz').split()[0]) for j in root.iter('joint')
            if j.get('name') == 'world-gate_beam']
    at = {'S1': mount['s1_'], 'S2': gate[0], 'S3a': mount['s3a_'], 'S4a': mount['s4a_'], 'S3b': mount['s3b_'],
          'S4b': mount['s4b_'], 'S3c': mount['s3c_'], 'S5': mount['s5_']}
    print(f'\nStops in {urdf_path}, gaps between neighbouring envelopes (need >= {ASSUMED["clearance"]:.2f} m):')
    tags = list(xs)
    bad = 0
    for t0, t1 in zip(tags[:-1], tags[1:]):
        gap = (at[t1] + xs[t1][2]) - (at[t0] + xs[t0][3])
        bad += gap < ASSUMED['clearance'] - 1e-6
        print(f'  {t0:4s} -> {t1:4s} {gap:6.3f} m  {"ok" if gap >= ASSUMED["clearance"] - 1e-6 else "TOO CLOSE"}')
    print(f'  {"all clear" if not bad else f"{bad} too close"}')
    return at


def plot(xs, walls, chosen, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, (top, side) = plt.subplots(2, 1, figsize=(15, 8.5), sharex=True,
                                    gridspec_kw=dict(height_ratios=[1.25, 1]))
    x0 = min(x + lo for x, _, lo, _ in xs.values()) - 0.3
    x1 = max(x + hi for x, _, _, hi in xs.values()) + 0.3
    colours = {'gp7': 'tab:orange', 'gp4a': 'tab:green', 'gp4b': 'tab:green', 'gp25': 'tab:red',
               'hc10dt': 'tab:blue', 'gate': 'black'}
    for ax in (top, side):
        ax.set_aspect('equal')
        ax.grid(alpha=0.25)
    # conveyor: top view strip, side view belt box
    top.add_patch(Rectangle((x0, -CONVEYOR_HALF), x1 - x0, 2 * CONVEYOR_HALF, color='0.85', zorder=0))
    side.add_patch(Rectangle((x0, 0.73), x1 - x0, 0.10, color='0.75', zorder=0))
    side.axhline(0, color='k', lw=1)
    for tag, (x, what, lo, hi) in xs.items():
        c = colours[what]
        if what != 'gate':   # pallet + module at the stop (the gate is passed through, not a stop)
            top.add_patch(Rectangle((x - 0.2025, -0.1005), 0.405, 0.201, fc='0.55', ec='k', lw=0.5, zorder=2))
            side.add_patch(Rectangle((x - 0.1775, 0.9), 0.355, 0.108, fc='0.55', ec='k', lw=0.5, zorder=2))
        if what in chosen:   # magazine pick points
            for part in {t[3] for t in tasks(what) if t[3]}:
                mx, my, mz = ASSUMED['magazine'][part]
                top.plot(x + mx, -chosen[what]['d'] + my, 's', color=colours[what], mfc='white', ms=6)
        if what == 'gate':
            for yy in (-0.30, 0.30):
                top.add_patch(Rectangle((x - 0.03, yy - 0.03), 0.06, 0.06, color='k'))
            side.add_patch(Rectangle((x - 0.03, 0), 0.06, 1.66, color='k'))
            top.text(x, 0.45, 'S2 gate', ha='center', fontsize=8)
            continue
        if what == 'hc10dt':
            (ylo, yhi), (zlo, zhi) = HC10DT_YZ
            base = (x, -0.30, PALLET_TOP + 1.10)
        else:
            e = chosen[what]
            ylo, yhi = e['env'][0][1], e['env'][1][1]
            zlo, zhi = e['env'][0][2], e['env'][1][2]
            base = (x, -e['d'], e['h'])
        envlo = lo + (0.0 if what != 'gp25' else ASSUMED['clearance'] + ASSUMED['wall'])
        envhi = hi - (0.0 if what != 'gp25' else ASSUMED['clearance'] + ASSUMED['wall'])
        top.add_patch(Rectangle((x + envlo, ylo), envhi - envlo, yhi - ylo, fc=c, alpha=0.15, ec=c, lw=1))
        side.add_patch(Rectangle((x + envlo, zlo), envhi - envlo, zhi - zlo, fc=c, alpha=0.15, ec=c, lw=1))
        top.plot(base[0], base[1], 'o', color=c, ms=7)
        side.plot(base[0], base[2], 'o', color=c, ms=7)
        label = {'gp7': 'GP7 cells', 'gp4a': 'GP4 carrier', 'gp4b': 'GP4 CMC + cover', 'gp25': 'GP25 weld',
                 'hc10dt': 'HC10DT (hung)'}[what]
        top.text(x, yhi + 0.06, f'{tag} {label}', ha='center', fontsize=8)
        top.text(x, -1.95, f'x {x:.2f}', ha='center', fontsize=7, color='0.3')
    for w in walls.values():   # laser enclosure walls
        top.axvline(w, color='tab:red', lw=2.5, ls='-')
        side.axvline(w, color='tab:red', lw=2.5, ls='-')
    top.set_ylim(-2.05, 1.5)
    side.set_ylim(-0.05, 2.5)
    top.set_ylabel('y, m (across the line)')
    side.set_ylabel('z, m (floor = 0)')
    side.set_xlabel('x, m along the line (S1 pallet stop = 0); dots = robot bases, boxes = swept envelopes, '
                    'squares = magazine pick points, red = laser enclosure walls')
    top.set_title('MotoBat line, first-pass layout (layout_check.py)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f'  plot: {path}')



# --- verification of the expanded line URDF ---------------------------------------------
def urdf_fixtures_and_mounts(urdf_path):
    """Fixture boxes (name -> lo, hi in world) and robot mounts (prefix -> xyz, rpy) from a URDF."""
    import xml.etree.ElementTree as ET
    root = ET.parse(urdf_path).getroot()
    sizes = {}
    for link in root.iter('link'):
        box = link.find('collision/geometry/box')
        if box is not None:
            sizes[link.get('name')] = np.array([float(v) for v in box.get('size').split()])
    boxes, mounts = {}, {}
    for j in root.iter('joint'):
        if j.find('parent').get('link') != 'world':
            continue
        child = j.find('child').get('link')
        o = j.find('origin')
        xyz = np.array([float(v) for v in o.get('xyz', '0 0 0').split()])
        rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
        if j.get('name').startswith('world-') and child in sizes:
            boxes[child] = (xyz - sizes[child] / 2, xyz + sizes[child] / 2)
        elif j.get('name').endswith('mount'):
            mounts[j.get('name')[:-len('mount')]] = (xyz, rpy)
    return boxes, mounts


def box_distance(pts, lo, hi):
    """Distance from each point to an axis-aligned box (0 inside)."""
    d = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
    return np.linalg.norm(d, axis=1)


def verify(urdf_path, reach_json):
    boxes, mounts = urdf_fixtures_and_mounts(urdf_path)
    sweeps = {}
    station_of = {'s1_': 'gp7', 's4a_': 'gp4a', 's4b_': 'gp25', 's5_': 'gp4b'}
    for prefix, name in station_of.items():
        xyz, rpy = mounts[prefix]
        st = Station(name)
        ev, missing = st.evaluate(-xyz[1], xyz[2], selfcheck=True)
        if ev is None:
            print(f'  {prefix}: unreachable at the URDF pose ({missing})')
            continue
        states = [a + t * (b - a) for a, b in zip(ev['chain'][:-1], ev['chain'][1:])
                  for t in np.linspace(0, 1, 20, endpoint=False)] + [ev['chain'][-1]]
        sweeps[prefix] = (st.arm, np.array(states), xyz, rpy)
    hc = L.hc10dt_states(reach_json)
    for prefix in ('s3a_', 's3b_', 's3c_'):
        xyz, rpy = mounts[prefix]
        sweeps[prefix] = ('hc10dt', hc[::2], xyz, rpy)
    worst = {}
    for prefix, (arm, states, xyz, rpy) in sweeps.items():
        macro = D.VENDOR / D.ARMS[arm][0]
        u = D.Urdf(macro)
        meshes = {n: v[::2] for n, v in L.link_meshes(macro).items()}
        Rm = D.rpy(*rpy)
        for q in states:
            fr = L.link_frames(u, q)
            for n, v in meshes.items():
                if n == 'base_link':
                    continue        # bolted to its own pedestal / drop column
                w = (v @ fr[n][1].T + fr[n][0]) @ Rm.T + xyz
                for b, (lo, hi) in boxes.items():
                    if np.any(w.max(0) < lo - 0.5) or np.any(w.min(0) > hi + 0.5):
                        continue
                    dmin = box_distance(w, lo, hi).min()
                    key = (prefix, b)
                    if dmin < worst.get(key, (np.inf,))[0]:
                        worst[key] = (dmin, n)
    print(f'Line URDF {urdf_path}: {len(boxes)} fixture boxes, {len(sweeps)} robots swept '
          f'(floor robots: their task sequence; inspectors: 2.5 base-A plan, every 2nd state)')
    print('  nearest approach of any moving link to any fixture, under 0.15 m (links only; tools and parts')
    print('  were in the envelope that spaced the stations, and are checked by MoveIt per station):')
    hits = 0
    for (prefix, b), (dmin, n) in sorted(worst.items(), key=lambda kv: kv[1][0]):
        if dmin < 0.15:
            flag = 'COLLIDES' if dmin <= 0 else ('< 5 cm' if dmin < 0.05 else '')
            hits += dmin <= 0
            print(f'    {prefix:5s} {n:9s} -> {b:22s} {dmin*1e3:7.1f} mm  {flag}')
    print(f'  {hits} collisions')
    return hits


if __name__ == '__main__':
    main()

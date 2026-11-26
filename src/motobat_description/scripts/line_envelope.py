#!/usr/bin/python3
"""
Swept envelope of a line robot in `world`: how far its links actually travel. (task 3.0)

The station layout (line_layout.xacro) needs to know how much of the line each robot
occupies, so neighbouring stations, fences and the laser enclosure can be placed
without guessing. This sweeps the vendor collision meshes (plus optional tool boxes
on tool0) over a set of joint states and reports the world-frame bounding box.

    line_envelope.py hc10dt-s3        the inspector over every planned state of 2.5's
                                      base-A plan (all legs and sweeps, 3 stages)

Library use (layout_check.py): `sweep(arm, states, mount, tool_boxes)`.

Pure numpy, no ROS: reads the vendor macro like derive_opw.py. Vertices are the mesh's
own (no hull, no padding): the box is where metal goes, before any clearance.
"""
import argparse
import json
import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import derive_opw as D  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
VENDOR = D.VENDOR


def read_stl(path):
    data = Path(path).read_bytes()
    n = struct.unpack_from('<I', data, 80)[0]
    rec = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]),
                        count=n, offset=84)
    v = rec['v'].reshape(-1, 3).astype(np.float64)
    return np.unique(np.round(v, 5), axis=0)


def link_meshes(macro):
    """link name -> vertices (link frame, m) of its collision mesh."""
    out = {}
    for link in ET.parse(macro).getroot().iter('link'):
        c = link.find('collision')
        if c is None or c.find('geometry/mesh') is None:
            continue
        uri = c.find('geometry/mesh').get('filename')
        pkg, rel = uri[len('package://'):].split('/', 1)
        v = read_stl(VENDOR / pkg / rel)
        o = c.find('origin')
        if o is not None:
            R = D.rpy(*D._num(o.get('rpy', '0 0 0')))
            v = v @ R.T + np.array(D._num(o.get('xyz', '0 0 0')))
        out[link.get('name').replace('${prefix}', '')] = v
    return out


def link_frames(urdf, q):
    """link name -> (p, R) in base_link, walking the chain the way URDF does."""
    p, R = np.zeros(3), np.eye(3)
    frames = {'base_link': (p, R)}
    i = 0
    for _, j in urdf.chain:
        p = p + R @ j['xyz']
        R = R @ j['R']
        if j['axis'] is not None:
            R = R @ D.rot_axis(j['axis'], q[i])
            i += 1
        frames[j['child']] = (p.copy(), R.copy())
    return frames


def box_corners(xyz, size):
    s = np.array(size) / 2
    return np.array([[sx, sy, sz] for sx in (-s[0], s[0]) for sy in (-s[1], s[1]) for sz in (-s[2], s[2])]) + xyz


def sweep(arm, states, mount_xyz, mount_rpy, tool_boxes=(), tool_frame=np.eye(4)):
    """World AABB (lo, hi) of every link (and tool box) over all states, plus per-link boxes."""
    macro = VENDOR / D.ARMS[arm][0]
    urdf = D.Urdf(macro)
    meshes = link_meshes(macro)
    tool = [box_corners(np.array(b['xyz']), b['size']) @ tool_frame[:3, :3].T + tool_frame[:3, 3]
            for b in tool_boxes]
    Rm, pm = D.rpy(*mount_rpy), np.asarray(mount_xyz, float)
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    per = {}
    for q in states:
        fr = link_frames(urdf, q)
        clouds = [(n, meshes[n] @ fr[n][1].T + fr[n][0]) for n in meshes if n in fr]
        clouds += [('tool', c @ fr['tool0'][1].T + fr['tool0'][0]) for c in tool]
        for name, c in clouds:
            w = c @ Rm.T + pm
            l, h = w.min(0), w.max(0)
            lo, hi = np.minimum(lo, l), np.maximum(hi, h)
            pl, ph = per.get(name, (np.full(3, np.inf), np.full(3, -np.inf)))
            per[name] = (np.minimum(pl, l), np.maximum(ph, h))
    return lo, hi, per


def hc10dt_states(reach_json):
    r = json.loads(Path(reach_json).read_text())['results'][0]
    qs = [r['home']]
    for st in r['sequences'].values():
        for leg in st['legs']:
            qs += leg['q']
        for path in st['paths'].values():
            qs += path
    return np.array(qs)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('what', choices=['hc10dt-s3'])
    ap.add_argument('--reach', default=str(REPO / 'data/reach/base_A/reach.json'))
    a = ap.parse_args()

    import yaml
    fl = yaml.safe_load((REPO / 'src/motobat_description/config/inspector_flange.yaml').read_text())['inspector_flange']
    T = np.eye(4)
    T[:3, :3] = D.rz(fl['flange_yaw'])          # tool0 -> flange
    states = hc10dt_states(a.reach)
    # s3 mount from line_layout.xacro (base A): (s3_datum_x, line_y - 0.30, 0.9 + 1.10), roll pi, yaw pi/2
    datum = np.array([1.8, 0.0, 0.9])
    lo, hi, per = sweep('hc10dt', states, datum + [0.0, -0.30, 1.10], (math.pi, 0, math.pi / 2),
                        fl['collision_boxes'].values(), T)
    print(f'HC10DT at S3 over {len(states)} planned states (2.5 base A, all stages), world m:')
    print(f'  all     x {lo[0]:.3f} .. {hi[0]:.3f}   y {lo[1]:.3f} .. {hi[1]:.3f}   z {lo[2]:.3f} .. {hi[2]:.3f}')
    print(f'  along the line from the pallet stop: {lo[0]-datum[0]:+.3f} .. {hi[0]-datum[0]:+.3f} m')
    for n, (l, h) in per.items():
        print(f'  {n:10s} x {l[0]-datum[0]:+.3f} .. {h[0]-datum[0]:+.3f}   y {l[1]:+.3f} .. {h[1]:+.3f}   '
              f'z {l[2]:.3f} .. {h[2]:.3f}')


if __name__ == '__main__':
    main()

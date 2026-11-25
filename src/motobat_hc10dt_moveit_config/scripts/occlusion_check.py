#!/usr/bin/env python3
"""
Does the robot block its own sensors? (task 2.5, step 2 of 2; venv: trimesh)

    mb-venv && python src/motobat_hc10dt_moveit_config/scripts/occlusion_check.py \
        data/reach/base_A/occlusion_poses.json --scans data/arm_scans/weld_walls/base_A

defectgen's scans (scan_view_plan.py) model the sensors' two occlusions against the module
only: a point counts if the light source reaches it and the camera sees it. The robot is not
in that scene. This adds it. For every capture, at the robot pose of that moment
(occlusion_poses.py), every measured point's two rays are tested against the arm:

  profilometer  laser emitter -> point, and point -> receiver camera (defectgen's geometry:
                emitter at the sensor origin, camera reference_distance * tan(triangulation
                angle) back along -y)
  snapshot      camera -> point, and point -> projector (baseline along the camera's -x)

The arm is the HC10DT's own collision meshes (links 1-6; the base on its mount plate too),
plus the flange's collision boxes: the bracket and the OTHER sensor's housing. The active
sensor's own box is left out: its rays leave through its own window, inside that box. The
boxes are bounding boxes, so a hit on one is an upper bound: look at it before believing it.

The points tested are the scans' display sample (clouds.npz), up to --points per robot pose:
a sweep's points measured near that pose's place along the sweep, a snapshot's all points.
Writes occlusion.json beside the poses file and prints a table per stage.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
import yaml

REPO = Path(__file__).resolve().parents[3]
MESHES = REPO / 'install' / 'motoman_hc10_support' / 'share' / 'motoman_hc10_support' / 'meshes'
LINK_MESH = {'s3_base_link': 'hc10/collision/base_link.stl', 's3_link_1_s': 'hc10/collision/link_1_s.stl',
             's3_link_2_l': 'hc10/collision/link_2_l.stl', 's3_link_3_u': 'hc10/collision/link_3_u.stl',
             's3_link_4_r': 'hc10/collision/link_4_r.stl', 's3_link_5_b': 'hc10/collision/link_5_b.stl',
             's3_link_6_t': 'hc10dt/collision/link_6_t.stl'}
OWN_BOX = {'profilometer': 'ljx', 'snapshot': 'zivid'}
END_TOL = 0.002    # m: a hit closer than this to the ray's far end is the end itself


def load_sensors():
    cfg = yaml.safe_load(open(REPO / 'defectgen' / 'src' / 'defectgen' / 'configs' / 'sensors.yaml'))
    return cfg['profilometer'], cfg['snapshot']


def flange_boxes():
    fl = yaml.safe_load(open(REPO / 'src' / 'motobat_description' / 'config' / 'inspector_flange.yaml'))
    return fl['inspector_flange']['collision_boxes']


def robot_meshes(links, skip_box):
    """(mesh, part name per face) for the arm at one pose, in the module frame."""
    parts, names = [], []
    for n, rel in LINK_MESH.items():
        m = trimesh.load(MESHES / rel, force='mesh').copy()
        m.apply_transform(np.array(links[n]))
        parts.append(m)
        names += [n.replace('s3_', '')] * len(m.faces)
    Tf = np.array(links['s3_flange'])
    for bn, b in flange_boxes().items():
        if bn == skip_box:
            continue
        m = trimesh.creation.box(extents=b['size'], transform=Tf @ trimesh.transformations.translation_matrix(b['xyz']))
        parts.append(m)
        names += [f'flange:{bn}'] * len(m.faces)
    return trimesh.util.concatenate(parts), np.array(names)


def blocked(mesh, names, start, end):
    """For segments start->end (N x 3): which are cut by the mesh before END_TOL of the end,
    and the part that cuts each."""
    d = end - start
    L = np.linalg.norm(d, axis=1)
    d = d / L[:, None]
    ray = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    tri, idx, loc = ray.intersects_id(start, d, multiple_hits=True, return_locations=True)
    hit = np.zeros(len(start), bool)
    by = np.full(len(start), '', dtype=object)
    if len(idx):
        t = np.einsum('ij,ij->i', loc - start[idx], d[idx])
        ok = (t > 1e-4) & (t < L[idx] - END_TOL)
        for i, f in zip(idx[ok], tri[ok]):
            if not hit[i]:
                hit[i], by[i] = True, names[f]
    return hit, by


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('poses', type=Path, help='occlusion_poses.json from occlusion_poses.py')
    ap.add_argument('--scans', type=Path, required=True, help='data/arm_scans/<name>/base_<B> (has <stage>/clouds.npz)')
    ap.add_argument('--coverage', type=Path, default=REPO / 'data' / 'coverage')
    ap.add_argument('--points', type=int, default=800, help='points tested per robot pose')
    a = ap.parse_args()

    prof, snap = load_sensors()
    back = prof['reference_distance'] * np.tan(np.radians(prof['triangulation_angle'])) / 1000.0
    base = snap['baseline'] / 1000.0
    P = json.load(open(a.poses))
    rng = np.random.default_rng(0)
    report = {}
    for stage, rows in P['stages'].items():
        plan = {v['id']: v for v in json.load(open(a.coverage / f'{stage}.plan.json'))['views']}
        z = np.load(a.scans / stage / 'clouds.npz')
        ids = list(z['ids'])
        clouds = {i: (z[f'p{k}'], z[f't{k}']) for k, i in enumerate(ids)}
        per_view = {}
        n_rows = {}
        for row in rows:
            n_rows[row['id']] = n_rows.get(row['id'], 0) + 1
        for row in rows:
            vid, sensor, w = row['id'], row['sensor'], row['when']
            pts, when = clouds[vid]
            if sensor == 'profilometer':
                half = 0.5 / n_rows[vid]
                sel = np.flatnonzero(np.abs(when - w) <= half)
            else:
                sel = np.arange(len(pts))
            if len(sel) == 0:
                continue
            sel = rng.choice(sel, min(len(sel), a.points), replace=False)
            p = pts[sel].astype(float)
            T = np.array(plan[vid]['pose_mm'], float)
            R, o = T[:3, :3], T[:3, 3] / 1000.0
            if sensor == 'profilometer':
                o = o + R[:, 1] * (w * plan[vid]['length_mm'] / 1000.0)
                ends = {'laser': o, 'receiver': o + R @ np.array([0.0, -back, 0.0])}
            else:
                ends = {'camera': o, 'projector': o - R[:, 0] * base}
            mesh, names = robot_meshes(row['links'], OWN_BOX[sensor])
            pv = per_view.setdefault(vid, {'sensor': sensor, 'points': 0, 'either': 0, 'by': {}, 'worst_at': None,
                                           'worst': 0.0, **{k: 0 for k in ends}})
            any_hit = np.zeros(len(p), bool)
            for k, e in ends.items():
                hit, by = blocked(mesh, names, p, np.broadcast_to(e, p.shape).copy())
                pv[k] += int(hit.sum())
                any_hit |= hit
                for b in by[hit]:
                    pv['by'][b] = pv['by'].get(b, 0) + 1
            pv['points'] += len(p)
            pv['either'] += int(any_hit.sum())
            frac = any_hit.mean()
            if frac > pv['worst']:
                pv['worst'], pv['worst_at'] = float(frac), w
        report[stage] = per_view
        print(f'\n{stage}: points whose light path the robot cuts (of those tested)')
        print(f"  {'view':30s} {'tested':>7s} {'blocked':>8s}  {'worst moment':>12s}  by")
        for vid, pv in per_view.items():
            share = pv['either'] / max(1, pv['points'])
            by = ', '.join(f'{k} {v}' for k, v in sorted(pv['by'].items(), key=lambda x: -x[1])) or '-'
            worst = (f"{pv['worst']:.0%} at {pv['worst_at']:.2f}"
                     if pv['sensor'] == 'profilometer' and pv['worst_at'] is not None
                     else f"{pv['worst']:.0%}")
            print(f"  {vid:30s} {pv['points']:7d} {share:8.1%}  {worst:>12s}  {by}")
        tot = sum(pv['either'] for pv in per_view.values())
        print(f"  total: {tot} of {sum(pv['points'] for pv in per_view.values())} tested points blocked")
    dst = a.poses.parent / 'occlusion.json'
    json.dump(report, open(dst, 'w'), indent=1)
    print(f'\nwrote {dst}')


if __name__ == '__main__':
    main()

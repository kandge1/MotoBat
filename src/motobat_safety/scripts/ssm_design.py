#!/usr/bin/python3
"""Protective fields for an HC10DT inspection cell: sizes, the fenceless footprint, and a guarded
opening's coverage and shadows. (task 3.5)

    ssm_design.py [--cell s3a] [--plot data/safety/ssm_design.png] [--K 1600]

Restricted space: the HC10DT's swept footprint (all states of 2.5's base-A plan, links and sensor
boxes; line_envelope.py) projected on the floor, its convex hull grown by Z_R (the FSU range limit).

(a) Fenceless: every point within D_core (ssm.Design.distance(1)) of the restricted space is guarded.
    Reported: which neighbouring stations that field would swallow.
(b) Guarded opening (the concept adopted): the line is fenced; each HC10DT cell has one opening on the
    operator side (+y), a corridor as wide as the restricted space and D deep, with side fences, so a
    person can reach the robot only by crossing the whole field. Two microScan3s at the corridor's
    inner corners, scan plane 300 mm. Checked: the 4 m range, and shadows: every fixture box that
    stands in the scan plane (line_fixtures.xacro, from the expanded line URDF) is cast against;
    a point is covered if either scanner sees it. Uncovered patches wider than the detection
    capability d are reported (someone could stand there unseen).
"""
import argparse
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / 'src/motobat_description/scripts'))
import derive_opw as D  # noqa: E402
import layout_check as LC  # noqa: E402
import line_envelope as L  # noqa: E402
import ssm  # noqa: E402


def swept_footprint(datum_x, base_xyz, reach_json, every=4):
    """Floor-projected points (m, world) of every link and the flange boxes over the plan."""
    import yaml
    macro = D.VENDOR / D.ARMS['hc10dt'][0]
    u = D.Urdf(macro)
    meshes = {n: v[::3] for n, v in L.link_meshes(macro).items()}
    fl = yaml.safe_load((REPO / 'src/motobat_description/config/inspector_flange.yaml').read_text())['inspector_flange']
    boxes = [L.box_corners(np.array(b['xyz']), b['size']) @ D.rz(fl['flange_yaw']).T for b in fl['collision_boxes'].values()]
    Rm = D.rpy(math.pi, 0, math.pi / 2)
    pts = []
    for q in L.hc10dt_states(reach_json)[::every]:
        fr = L.link_frames(u, q)
        for n, v in meshes.items():
            pts.append((v @ fr[n][1].T + fr[n][0]) @ Rm.T + base_xyz)
        p0, R0 = fr['tool0']
        for b in boxes:
            pts.append((b @ R0.T + p0) @ Rm.T + base_xyz)
    P = np.vstack(pts)
    return P[:, :2]


def hull(points):
    from scipy.spatial import ConvexHull
    h = ConvexHull(points)
    return points[h.vertices]


def dist_to_polygon(pts, poly):
    """Distance from points to a convex polygon (0 inside)."""
    n = len(poly)
    inside = np.ones(len(pts), bool)
    dmin = np.full(len(pts), np.inf)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ab = b - a
        t = np.clip(((pts - a) @ ab) / (ab @ ab), 0, 1)
        d = np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)
        dmin = np.minimum(dmin, d)
        cross = ab[0] * (pts[:, 1] - a[1]) - ab[1] * (pts[:, 0] - a[0])
        inside &= cross >= 0                     # hull vertices are counter-clockwise
    return np.where(inside, 0.0, dmin)


def segment_hits_box(p0, p1, lo, hi):
    """Do the 2-D segments p0 -> p1 (arrays) cross the axis-aligned box (lo, hi)? (slab test)"""
    d = p1 - p0
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = (lo - p0) / d
        t2 = (hi - p0) / d
    tmin = np.nanmax(np.where(np.isnan(t1), -np.inf, np.minimum(t1, t2)), axis=1)
    tmax = np.nanmin(np.where(np.isnan(t1), np.inf, np.maximum(t1, t2)), axis=1)
    # a coordinate parallel to the slab and outside it never hits
    par_out = np.any((d == 0) & ((p0 < lo) | (p0 > hi)), axis=1)
    return (tmax >= np.maximum(tmin, 0)) & (tmin <= 1) & ~par_out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--cell', default='s3a', choices=['s3a', 's3b', 's3c'])
    ap.add_argument('--K', type=float, default=ssm.K_APPROACH)
    ap.add_argument('--reach', default=str(REPO / 'data/reach/base_A/reach.json'))
    ap.add_argument('--plot', type=Path)
    a = ap.parse_args()

    from ament_index_python.packages import get_package_share_directory
    xacro_in = Path(get_package_share_directory('motobat_description')) / 'urdf/motobat_line.urdf.xacro'
    with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
        subprocess.run(['xacro', str(xacro_in), '-o', f.name], check=True)
        boxes, mounts = LC.urdf_fixtures_and_mounts(f.name)
    base, _ = mounts[f'{a.cell}_']
    datum_x = base[0]
    des = ssm.Design(K=a.K)
    D_core = des.distance(1.0) / 1e3
    tab = des.table(0.065)

    fp = swept_footprint(datum_x, base, a.reach)
    rs = hull(fp)                                           # restricted space before Z_R
    # grow the convex hull by Z_R: sample its offset boundary and re-hull
    ang = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    grown = (rs[:, None, :] + des.z_r / 1e3 * np.stack([np.cos(ang), np.sin(ang)], 1)[None]).reshape(-1, 2)
    rs = hull(grown)
    lo, hi = rs.min(0), rs.max(0)
    print(f'{a.cell.upper()}: restricted space (swept footprint + Z_R {des.z_r:.0f} mm): x {lo[0]:.2f} .. {hi[0]:.2f}, '
          f'y {lo[1]:.2f} .. {hi[1]:.2f} m')
    print(f'field sizes, K {a.K:.0f} mm/s: core (one safe field, any speed) {tab["core_stop"]:.0f} mm; network: '
          f'stop {tab["network_stop"]:.0f} mm, slowdown {tab["network_slowdown"]:.0f} mm  '
          f'(T_R {des.t_r * 1e3:.0f} ms, T_S {des.t_s_full:.2f} s, C {des.C:.0f}, Z_S {des.z_s:.0f}, Z_R {des.z_r:.0f})')

    # (a) fenceless: which robots' pedestals / stations fall inside D_core of the restricted space
    swallowed = []
    for name, (blo, bhi) in boxes.items():
        if blo[2] > 0.5 or not any(k in name for k in ('pedestal', 'post', 'mag', 'gate', 'wall')):
            continue
        c = np.array([[blo[0], blo[1]], [bhi[0], bhi[1]], [blo[0], bhi[1]], [bhi[0], blo[1]]])
        if dist_to_polygon(c, rs).min() < D_core:
            swallowed.append(name)
    print(f'(a) fenceless field ({D_core:.2f} m all round) would reach: {", ".join(sorted(swallowed)) or "nothing"}')

    # (b) guarded opening on +y
    y0 = hi[1]                                              # corridor starts at the restricted space
    corridor = (lo[0], hi[0], y0, y0 + D_core)
    scanners = [np.array([lo[0], y0]), np.array([hi[0], y0])]
    far = max(np.linalg.norm(np.array([x, corridor[3]]) - s) for s in scanners for x in (corridor[0], corridor[1]))
    near_far = max(min(np.linalg.norm(np.array([x, y]) - s) for s in scanners)
                   for x in np.linspace(*corridor[:2], 30) for y in np.linspace(*corridor[2:], 30))
    print(f'(b) guarded opening: corridor x {corridor[0]:.2f} .. {corridor[1]:.2f}, y {corridor[2]:.2f} .. '
          f'{corridor[3]:.2f} m ({corridor[1] - corridor[0]:.2f} x {D_core:.2f} m); scanners at its inner corners')
    print(f'    range: every corridor point within {near_far:.2f} m of a scanner (limit {ssm.SCANNER_RANGE / 1e3:.1f}); '
          f'farthest corner from either {far:.2f} m')
    H = des.H / 1e3
    obstacles = {n: (b[0][:2], b[1][:2]) for n, (b0, b1) in boxes.items() for b in [(b0, b1)]
                 if b0[2] <= H <= b1[2]}
    in_corr = {n: o for n, o in obstacles.items()
               if o[1][0] > corridor[0] - 0.5 and o[0][0] < corridor[1] + 0.5 and o[1][1] > corridor[2] - 0.5
               and o[0][1] < corridor[3] + 0.5}
    print(f'    fixtures in the scan plane near the corridor: {", ".join(sorted(in_corr)) or "none"}')
    gx = np.arange(corridor[0], corridor[1], 0.02)
    gy = np.arange(corridor[2], corridor[3], 0.02)
    G = np.array(np.meshgrid(gx, gy)).reshape(2, -1).T
    blocked_in = np.zeros(len(G), bool)
    for o in in_corr.values():
        blocked_in |= np.all((G >= o[0]) & (G <= o[1]), axis=1)
    seen = np.zeros(len(G), bool)
    for s in scanners:
        vis = np.linalg.norm(G - s, axis=1) <= ssm.SCANNER_RANGE / 1e3
        for o in in_corr.values():
            vis &= ~segment_hits_box(np.repeat(s[None], len(G), 0), G, o[0], o[1])
        seen |= vis
    hole = ~seen & ~blocked_in
    if hole.any():
        from scipy.ndimage import label
        mask = hole.reshape(len(gy), len(gx))
        lab, nlab = label(mask)
        sizes = [((lab == i).sum() * 0.02 * 0.02, (lab == i).any(0).sum() * 0.02, (lab == i).any(1).sum() * 0.02)
                 for i in range(1, nlab + 1)]
        big = [s for s in sizes if min(s[1], s[2]) > des.d / 1e3]
        print(f'    SHADOWS: {nlab} unseen patch(es), {len(big)} wider than d = {des.d:.0f} mm in both directions; largest '
              f'{max(s[0] for s in sizes):.3f} m2 ({max(sizes, key=lambda s: s[0])[1]:.2f} x {max(sizes, key=lambda s: s[0])[2]:.2f} m)')
    else:
        print('    shadows: none: every corridor point is seen by at least one scanner')

    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Rectangle
        fig, ax = plt.subplots(figsize=(13, 7))
        ax.set_aspect('equal')
        # fenceless field outline: offset the hull by D_core
        ring = hull((rs[:, None, :] + D_core * np.stack([np.cos(ang), np.sin(ang)], 1)[None]).reshape(-1, 2))
        ax.add_patch(Polygon(ring, fc='tab:red', alpha=0.06, ec='tab:red', ls='--', label=f'(a) fenceless field, {D_core:.2f} m'))
        ax.add_patch(Polygon(rs, fc='tab:blue', alpha=0.25, ec='tab:blue', label='restricted space (swept + Z_R)'))
        ax.add_patch(Rectangle((corridor[0], corridor[2]), corridor[1] - corridor[0], D_core, fc='tab:orange', alpha=0.2,
                               ec='tab:orange', label='(b) protective field in the guarded opening'))
        for x in corridor[:2]:
            ax.plot([x, x], [corridor[2], corridor[3]], color='k', lw=3)
        ax.plot([lo[0] - 3, corridor[0]], [y0, y0], color='k', lw=3, label='fence')
        ax.plot([corridor[1], hi[0] + 3], [y0, y0], color='k', lw=3)
        for n, (blo, bhi) in boxes.items():
            if blo[2] <= H <= bhi[2]:
                ax.add_patch(Rectangle(blo[:2], *(bhi[:2] - blo[:2]), fc='0.4', ec='k', lw=0.4))
        ax.add_patch(Rectangle((lo[0] - 3, -0.1405), hi[0] - lo[0] + 6, 0.281, fc='0.85', ec='none', label='conveyor'))
        for s in scanners:
            ax.plot(*s, 'v', color='tab:green', ms=10)
        if hole.any():
            ax.plot(G[hole, 0], G[hole, 1], '.', color='tab:red', ms=1.5, label='unseen by both scanners')
        ax.set_xlim(lo[0] - 2.6, hi[0] + 2.6)
        ax.set_ylim(lo[1] - 2.6, corridor[3] + 0.6)
        ax.set_xlabel('x, m (along the line)')
        ax.set_ylabel('y, m (operator side +)')
        ax.set_title(f'{a.cell.upper()} HC10DT: protective fields (microScan3, K {a.K:.0f} mm/s, scan plane {des.H:.0f} mm; '
                     f'grey = fixtures in the scan plane; green = scanners)')
        ax.legend(loc='lower left', fontsize=8)
        ax.grid(alpha=0.3)
        a.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.tight_layout()
        fig.savefig(a.plot, dpi=100)
        print(f'plot: {a.plot}')


if __name__ == '__main__':
    main()

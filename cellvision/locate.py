"""Locate a prismatic cell's top face in a calibrated overhead image: (x, y, yaw) and its size. (task 3.2)

Seen from above, a cell's top is a bright rectangle (aluminium can lid under the ring light)
whose four edges fall to something darker: the cell's side faces and the tray at the infeed;
the 1.5 mm foam pads between cells in a stack. The steps:

1. Coarse pose (if not given): threshold at Otsu's level, keep the bright 8-connected blob
   nearest the expected place, holes filled (terminals and vent), map its pixels to the plane
   with H^-1, take the centroid and the principal axis of the second moments. Good to a
   fraction of a millimetre and degree for a whole, unoccluded top.

2. Edge points. For each side, stations every `step` mm along it (ends kept 2 mm clear of the
   rounded corners). At each station a profile is sampled ALONG THE SIDE'S NORMAL IN THE
   PLANE, from -w to +w mm round the expected edge, at 1/4 pixel spacing: plane points are
   pushed through H to pixel positions and the image is read there by cubic B-spline
   interpolation (smooth first derivative; bilinear would make it flat across each pixel).
   The edge is the steepest fall of brightness going outward (the derivative's minimum),
   refined to sub-sample position by a parabola through the minimum and its two neighbours:
        t* = t_i + h (g_{i-1} - g_{i+1}) / (2 (g_{i-1} - 2 g_i + g_{i+1}))
   (h the sample spacing, g the derivative). Working in plane millimetres keeps every profile
   the same physical length whatever the perspective.

3. Rectangle fit. Unknowns: centre (cx, cy), yaw th, half sizes (a, b). With the side normals
   n = (cos th, sin th) for the +-a sides and m = (-sin th, cos th) for the +-b sides, a point q
   on side +a has residual  n.(q - c) - a,  on side -a  -n.(q - c) - a, and the same with m, b.
   Minimised by nonlinear least squares (scipy least_squares, soft-L1 loss with a 0.05 mm
   scale so a stray edge point cannot drag the fit), then points beyond 4 x the robust spread
   (1.4826 x MAD) are dropped and the fit repeated. Fitting the sizes too keeps a cell that is
   0.2 mm long from biasing the centre: the error splits evenly between opposite sides.

4. Final pass, edge-model fit (method='model', the default). The steepest-gradient point of
   step 2 is biased by where the edge falls on the pixel grid ("pixel locking"): measured on
   exactly formed pixels (continuous blur, then pixel integration) it is off by up to 0.10 px
   at a 0.6 px blur, and along an edge nearly parallel to the pixel rows every station shares
   that phase, so the bias does not average out (it showed as ~20 um on the stack camera).
   The cure is to fit the image formation itself. For each side, every pixel within 4 px of the
   edge line is fitted with
        I = A_out + (A_in - A_out) * P(x),      x = signed distance into the bright side (px)
   where P is a step blurred by a Gaussian of width s and integrated over the pixel. The pixel's
   footprint, projected on the edge normal n = (n_u, n_v), is a box of width |n_u| convolved
   with one of width |n_v|, so with G1(z) = z Phi(z) + phi(z) and
   G2(z) = ((z^2 + 1) Phi(z) + z phi(z)) / 2 (the first and second antiderivatives of Phi):
        P(x) = s^2 / (a b) [G2((x+a/2+b/2)/s) - G2((x+a/2-b/2)/s) - G2((x-a/2+b/2)/s) + G2((x-a/2-b/2)/s)]
   with a = |n_u|, b = |n_v| (and the one-box form (s/a)[G1((x+a/2)/s) - G1((x-a/2)/s)] as b -> 0).
   Unknowns per side: the line (angle, offset), A_in, A_out, s. On exact pixels it is unbiased
   to 1e-5 px (1-D check, 26 Nov). A homography maps lines to lines: the image line l becomes the
   plane line H^T l, which is sampled into points and handed to the rectangle fit of step 3.

Returned: pose, sizes, RMS residual and the number of edge points used. All in plane mm.
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import binary_fill_holes, label, map_coordinates, spline_filter
from scipy.optimize import least_squares
from scipy.special import ndtr

from .calib import apply


def otsu(img):
    h, edges = np.histogram(img, bins=256)
    c = (edges[:-1] + edges[1:]) / 2
    w0 = np.cumsum(h)
    w1 = w0[-1] - w0
    m0 = np.cumsum(h * c) / np.maximum(w0, 1)
    m1 = (np.sum(h * c) - np.cumsum(h * c)) / np.maximum(w1, 1)
    return c[np.argmax(w0 * w1 * (m0 - m1) ** 2)]


def coarse_pose(img, offset, H, near_xy):
    """(x, y, yaw) of the bright blob nearest near_xy (plane mm), from its moments."""
    Hinv = np.linalg.inv(H)
    mask = img > otsu(img)
    lab, n = label(mask, structure=np.ones((3, 3)))
    best, bestd = None, np.inf
    for i in range(1, n + 1):
        vv, uu = np.nonzero(lab == i)
        if len(uu) < 200:
            continue
        q = apply(Hinv, np.column_stack([uu + offset[0], vv + offset[1]]))
        d = np.linalg.norm(q.mean(0) - near_xy)
        if d < bestd:
            best, bestd = i, d
    blob = binary_fill_holes(lab == best)
    vv, uu = np.nonzero(blob)
    q = apply(Hinv, np.column_stack([uu + offset[0], vv + offset[1]]))
    c = q.mean(0)
    C = np.cov((q - c).T)
    w, V = np.linalg.eigh(C)
    major = V[:, 1]                                   # the long axis (the cell's own y)
    yaw = np.arctan2(major[1], major[0]) - np.pi / 2  # own x axis = long axis rotated -90 deg
    yaw = (yaw + np.pi / 2) % np.pi - np.pi / 2       # a rectangle: yaw is defined mod 180 deg
    return np.array([c[0], c[1], yaw])


def edge_points(img, offset, H, pose, size, step=1.0, w=1.5, corner=2.0, gsd=0.15):
    """Sub-sample edge points (plane mm) on the four sides of the expected rectangle."""
    x, y, th = pose
    a, b = size[0] / 2, size[1] / 2
    n = np.array([np.cos(th), np.sin(th)])
    m = np.array([-np.sin(th), np.cos(th)])
    c = np.array([x, y])
    h = gsd / 4
    t = np.arange(-w, w + h / 2, h)
    # Cubic B-spline interpolation (coefficients once per image): its derivative is smooth, so the
    # gradient has a real peak to fit. Bilinear interpolation (tried first, 26 Nov) is piecewise
    # linear between pixel centres: its derivative is flat across each pixel, the parabola had
    # nothing to fit, and edges snapped to the sample grid (bias up to ~44 um at 0.17 mm/px).
    coef = spline_filter(img.astype(float), order=3, mode='nearest')
    pts, side = [], []
    for k, (normal, along, half, span) in enumerate(((n, m, a, b), (-n, m, a, b), (m, n, b, a), (-m, n, b, a))):
        s = np.arange(-span + corner, span - corner + 1e-9, step)
        base = c + half * normal                                  # expected edge midpoint
        P = base[None, None, :] + s[:, None, None] * along + t[None, :, None] * normal
        uv = apply(H, P.reshape(-1, 2)) - np.asarray(offset)
        prof = map_coordinates(coef, [uv[:, 1], uv[:, 0]], order=3, mode='nearest', prefilter=False)
        prof = prof.reshape(len(s), len(t))
        g = np.gradient(prof, h, axis=1)
        i = np.clip(np.argmin(g, axis=1), 1, len(t) - 2)
        r = np.arange(len(s))
        gm, g0, gp = g[r, i - 1], g[r, i], g[r, i + 1]
        den = gm - 2 * g0 + gp
        with np.errstate(divide='ignore', invalid='ignore'):
            dt = np.where(np.abs(den) > 1e-12, h * (gm - gp) / (2 * den), 0.0)
        tt = t[i] + dt
        pts.append(base + s[:, None] * along + tt[:, None] * normal)
        side += [k] * len(s)
    return np.vstack(pts), np.array(side)


def _phi(z):
    return np.exp(-0.5 * z * z) / np.sqrt(2 * np.pi)


def _G1(z):
    return z * ndtr(z) + _phi(z)


def _G2(z):
    return 0.5 * ((z * z + 1) * ndtr(z) + z * _phi(z))


def pixel_step(x, s, a, b):
    """Mean over a pixel footprint (boxes a and b along the normal) of a step at x = 0 blurred by
    N(0, s^2): 1 deep inside (x >> 0), 0 outside. x in pixels."""
    s = max(s, 1e-3)
    if b < 0.02:
        return s / a * (_G1((x + a / 2) / s) - _G1((x - a / 2) / s))
    return s * s / (a * b) * (_G2((x + a / 2 + b / 2) / s) - _G2((x + a / 2 - b / 2) / s)
                              - _G2((x - a / 2 + b / 2) / s) + _G2((x - a / 2 - b / 2) / s))


class _SideFailed(Exception):
    def __init__(self, side):
        super().__init__(f'edge-model fit hit its bounds on side {side}')
        self.side = side


def model_edges(img, offset, H, pose, size, band=4.0, corner=2.0, per_side=24):
    """Per side: fit the pixel-integrated blurred-step model to the raw pixels near the expected
    edge, map the fitted line to the plane, sample it. Returns (points, side labels)."""
    x, y, th = pose
    hx, hy = size[0] / 2, size[1] / 2
    n = np.array([np.cos(th), np.sin(th)])
    m = np.array([-np.sin(th), np.cos(th)])
    c = np.array([x, y])
    V, U = np.mgrid[0:img.shape[0], 0:img.shape[1]]
    U = U + offset[0]
    V = V + offset[1]
    I = img.astype(float)
    pts, side = [], []
    for k, (normal, along, half, span) in enumerate(((n, m, hx, hy), (-n, m, hx, hy), (m, n, hy, hx), (-m, n, hy, hx))):
        e0 = c + half * normal - (span - corner) * along          # usable stretch of the edge, plane
        e1 = c + half * normal + (span - corner) * along
        p0, p1 = apply(H, np.array([e0, e1]))
        d = (p1 - p0) / np.linalg.norm(p1 - p0)                   # along the edge, image
        inward = apply(H, np.array([c]))[0] - (p0 + p1) / 2
        nrm = np.array([-d[1], d[0]])
        if nrm @ inward < 0:
            nrm = -nrm                                            # image normal pointing into the cell
        L = np.linalg.norm(p1 - p0)
        rel_u, rel_v = U - p0[0], V - p0[1]
        t = rel_u * d[0] + rel_v * d[1]
        dist = rel_u * nrm[0] + rel_v * nrm[1]                    # + inside
        sel = (t >= 0) & (t <= L) & (np.abs(dist) <= band)
        if sel.sum() < 50:
            raise ValueError(f'side {k}: only {sel.sum()} pixels in the edge band (outside the image?)')
        tu, du, val = t[sel], dist[sel], I[sel]
        a_, b_ = abs(nrm[0]), abs(nrm[1])
        a_, b_ = max(a_, b_), min(a_, b_)

        def f(p):
            ang, off, ain, aout, sg = p
            # the edge line, rotated by ang and shifted by off relative to the expected one
            xx = du * np.cos(ang) - (tu - L / 2) * np.sin(ang) - off
            return aout + (ain - aout) * pixel_step(xx, sg, a_, b_) - val

        p_init = [0.0, 0.0, np.percentile(val[du > 1.5], 75), np.percentile(val[du < -1.5], 25), 0.8]
        # Bounded to what the +-band pixels can support: with the offset free, a fit could slide off
        # the band where the model goes flat and stop on a neighbouring cell's edge one pitch away
        # (3 of 100 stack measurements, 27 Nov; the locator's sanity check rejected them).
        lo = [-0.05, -2.0, -np.inf, -np.inf, 0.1]
        hi = [0.05, 2.0, np.inf, np.inf, 3.0]
        r = least_squares(f, p_init, x_scale=[1e-3, 0.1, 100, 100, 0.1], bounds=(lo, hi))
        ang, off = r.x[0], r.x[1]
        if abs(off) > 1.95 or abs(ang) > 0.049:
            raise _SideFailed(k)                                  # locate() reports 'not measured'
        # fitted image line: points q with (q - p0 - (L/2) d) . nrm_rot = off, nrm_rot = nrm rotated by ang
        nr = np.array([nrm[0] * np.cos(ang) - d[0] * np.sin(ang), nrm[1] * np.cos(ang) - d[1] * np.sin(ang)])
        nr /= np.linalg.norm(nr)
        mid = p0 + (L / 2) * d + off * nr
        line = np.array([nr[0], nr[1], -(nr @ mid)])              # l . (u, v, 1) = 0
        lp = H.T @ line                                           # the same line in the plane
        # sample it: project the expected stretch onto the plane line
        ln = lp[:2] / np.linalg.norm(lp[:2])
        dd = lp[2] / np.linalg.norm(lp[:2])
        s_ = np.linspace(0, 1, per_side)[:, None]
        q = e0 + s_ * (e1 - e0)
        q = q - ((q @ ln) + dd)[:, None] * ln[None, :]
        pts.append(q)
        side += [k] * per_side
    return np.vstack(pts), np.array(side)


def _resid(p, q, side):
    cx, cy, th, a, b = p
    n = np.array([np.cos(th), np.sin(th)])
    m = np.array([-np.sin(th), np.cos(th)])
    d = q - np.array([cx, cy])
    pn, pm = d @ n, d @ m
    return np.select([side == 0, side == 1, side == 2], [pn - a, -pn - a, pm - b], -pm - b)


def fit_rectangle(q, side, pose, size):
    p0 = np.array([pose[0], pose[1], pose[2], size[0] / 2, size[1] / 2])
    r = least_squares(_resid, p0, args=(q, side), loss='soft_l1', f_scale=0.05)
    res = _resid(r.x, q, side)
    mad = 1.4826 * np.median(np.abs(res - np.median(res)))
    keep = np.abs(res) <= max(4 * mad, 1e-3)
    r = least_squares(_resid, r.x, args=(q[keep], side[keep]))
    res = _resid(r.x, q[keep], side[keep])
    return r.x, float(np.sqrt(np.mean(res ** 2))), int(keep.sum())


def locate(img, offset, H, size, near_pose=None, near_xy=None, gsd=0.15, passes=2, method='model'):
    """Pose (x, y, yaw) and size (sx, sy) of a cell top. Give near_pose if known to ~1 mm
    and ~0.5 deg (a placed cell); otherwise near_xy and the blob finds it (the infeed).
    method: 'model' (default) ends with the edge-model fit; 'peak' stops at the gradient peaks."""
    pose = np.asarray(near_pose, float) if near_pose is not None else coarse_pose(img, offset, H, near_xy)
    sz = np.asarray(size, float)
    w = 1.5 if near_pose is not None else 2.5
    start = pose.copy()
    for _ in range(passes):
        q, side = edge_points(img, offset, H, pose, sz, w=w, gsd=gsd)
        p, rms, n = fit_rectangle(q, side, pose, sz)
        pose, sz, w = p[:3], 2 * p[3:], 0.8
    if method == 'model':
        try:
            q, side = model_edges(img, offset, H, pose, sz)
        except _SideFailed as e:
            # A side whose model fit pins at its bounds is not measured: report that rather than a
            # number. (Falling back to the gradient-peak points was tried, 27 Nov: on 2 of the 3
            # such stacks they were ~0.1 mm off, half the tolerance, with no sign of it.)
            raise ValueError(f'not measured: {e}') from e
        p, rms, n = fit_rectangle(q, side, pose, sz)
        pose, sz = p[:3], 2 * p[3:]
    # A fit that wandered far from where it started latched onto something else: say so.
    if (np.hypot(*(pose[:2] - start[:2])) > 2.0 + w or abs(np.degrees(pose[2] - start[2])) > 2.0
            or np.any(np.abs(sz - np.asarray(size)) > 1.0)):
        raise ValueError(f'fit left the expected cell: moved {pose[:2] - start[:2]} mm, '
                         f'{np.degrees(pose[2] - start[2]):.2f} deg, sizes {sz}')
    return dict(x=pose[0], y=pose[1], yaw=pose[2], sx=sz[0], sy=sz[1], rms=rms, points=n)


def parallax_correct(result, cam, plane_z, dz):
    """Undo the parallax of a top dz above the calibrated plane (plane_z, world mm).

    The homography maps every image point onto the calibrated plane. A point dz higher, truly at
    horizontal distance r from the optical axis, is seen along a ray that reaches the plane at
    r' = r Z / (Z - dz)  (similar triangles; Z = camera height above the plane). So r = r' (Z - dz) / Z:
    each in-plane coordinate is pulled toward the axis by the factor (Z - dz) / Z. The yaw is
    unchanged; the sizes scale by the same factor. dz comes from the infeed point laser (task 3.4)."""
    Z = cam.t[2] - plane_z
    f = (Z - dz) / Z
    out = dict(result)
    out['x'] = cam.t[0] + (result['x'] - cam.t[0]) * f
    out['y'] = cam.t[1] + (result['y'] - cam.t[1]) * f
    out['sx'], out['sy'] = result['sx'] * f, result['sy'] * f
    return out

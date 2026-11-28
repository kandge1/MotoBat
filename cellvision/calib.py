"""Image-to-plane calibration with a dot-grid target: a homography per measurement plane. (task 3.2)

Why a homography. Every point of a plane Z = z0 maps to the image through the pinhole model
    s [u, v, 1]^T = K [r1 r2 (r3 z0 + t')] [X, Y, 1]^T  =  H [X, Y, 1]^T
(K the intrinsics, r_i the columns of the camera rotation, t' the translation): a 3 x 3 matrix
up to scale, 8 numbers. So a fixed camera that measures on a known plane needs only H, found
from points whose plane coordinates are known: a calibration target lying on that plane. The
cell tops (95.0 mm above the pallet datum) are one such plane; at the infeed, the tops of cells
standing on the infeed pad are another. Points OFF the plane are mapped as if they were on it:
that is the parallax error study.py measures (a point dz above the plane, r from the optical
axis, appears shifted outward by r * dz / (Z - dz): about 0.13 mm at r = 150 mm, Z = 600 mm,
dz = 0.5 mm; a telecentric lens would remove it).

Target (A): white dots on a dark plate, Ø 4 mm on a 10 mm grid, printed or laser-marked to
better than the dot-centre noise we fit (a real certified target: ~1-2 um). Laid on the
measurement plane at calibration time.

Dot centres: intensity-weighted centroid of (I - background) in a window round each dot's
expected position (the target is placed roughly, so windows come from a first coarse H, or
here from the nominal pose). For a small dot seen nearly face-on, perspective moves the
centroid off the true centre by ~ (radius^2 / Z) * tilt: sub-micron here, ignored.

Homography by the normalised DLT (Hartley & Zisserman, Multiple View Geometry, 2nd ed.,
Alg. 4.2). Each correspondence (X, Y) -> (u, v) gives two linear equations in the 9 entries of
H; stacked for n points, A h = 0, solved by the SVD (h = the right singular vector of the
smallest singular value). Normalising both point sets first (centroid to 0, mean distance
sqrt 2) makes A well conditioned; without it the answer depends on the units. Then
    H = T_img^-1  H_norm  T_plane.
Reported: the reprojection RMS (pixels) and, on points held out of the fit, the plane-mapping
error (mm) - that is the calibration's contribution to every measurement.
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import gaussian_filter

from .camera import NOISE, SENSOR, subsamples


def _normaliser(p):
    c = p.mean(0)
    s = np.sqrt(2) / np.mean(np.linalg.norm(p - c, axis=1))
    return np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1]])


def dlt(plane_xy, img_uv):
    """H with  s [u v 1]^T = H [X Y 1]^T, by the normalised DLT."""
    Tp, Ti = _normaliser(plane_xy), _normaliser(img_uv)
    P = (Tp @ np.column_stack([plane_xy, np.ones(len(plane_xy))]).T).T
    I = (Ti @ np.column_stack([img_uv, np.ones(len(img_uv))]).T).T
    rows = []
    for (X, Y, W), (u, v, w) in zip(P, I):
        rows.append([0, 0, 0, -w * X, -w * Y, -w * W, v * X, v * Y, v * W])
        rows.append([w * X, w * Y, w * W, 0, 0, 0, -u * X, -u * Y, -u * W])
    _, _, Vt = np.linalg.svd(np.asarray(rows))
    Hn = Vt[-1].reshape(3, 3)
    H = np.linalg.inv(Ti) @ Hn @ Tp
    return H / H[2, 2]


def apply(H, xy):
    p = np.column_stack([xy, np.ones(len(xy))]) @ H.T
    return p[:, :2] / p[:, 2:3]


def render_target(cam, z0, origin, pitch=10.0, dot=4.0, n=(21, 17), yaw=0.0, ss=4, rng=None,
                  noise=NOISE, white=0.85, dark=0.08):
    """12-bit image of a dot grid lying on the plane Z = z0, its grid centred on `origin`
    (x, y). Returns (image, dot centres in world (n, 2))."""
    rng = np.random.default_rng() if rng is None else rng
    c, s = np.cos(yaw), np.sin(yaw)
    gx, gy = np.meshgrid((np.arange(n[0]) - (n[0] - 1) / 2) * pitch, (np.arange(n[1]) - (n[1] - 1) / 2) * pitch)
    centres = np.column_stack([origin[0] + c * gx.ravel() - s * gy.ravel(), origin[1] + s * gx.ravel() + c * gy.ravel()])
    uv = cam.project(np.column_stack([centres, np.full(len(centres), z0)]))
    u0, v0 = np.floor(uv.min(0) - 40).astype(int)
    u1, v1 = np.ceil(uv.max(0) + 40).astype(int)
    u0, v0 = max(u0, 0), max(v0, 0)
    u1, v1 = min(u1, cam.width), min(v1, cam.height)
    uu, vv = subsamples(u0, v0, u1, v1, ss, rng)
    d = cam.rays(uu, vv)
    t = (z0 - cam.t[2]) / d[..., 2]
    X, Y = cam.t[0] + t * d[..., 0], cam.t[1] + t * d[..., 1]
    # nearest grid node in target coordinates
    a, b = c * (X - origin[0]) + s * (Y - origin[1]), -s * (X - origin[0]) + c * (Y - origin[1])
    ra = a - np.round(a / pitch) * pitch
    rb = b - np.round(b / pitch) * pitch
    inside_grid = (np.abs(a) <= (n[0] - 1) / 2 * pitch + pitch / 2) & (np.abs(b) <= (n[1] - 1) / 2 * pitch + pitch / 2)
    L = np.where((ra ** 2 + rb ** 2 <= (dot / 2) ** 2) & inside_grid, white, dark).mean(axis=(2, 3))
    L = gaussian_filter(L, noise['psf_px'])
    N = L * noise['exposure'] * noise['full_well']
    N = N + rng.normal(0, 1, N.shape) * np.sqrt(N + noise['read'] ** 2)
    img = np.clip(np.round(N / noise['full_well'] * (2 ** SENSOR['bits'] - 1)), 0, 2 ** SENSOR['bits'] - 1)
    return img.astype(np.uint16), (u0, v0), centres


def dot_centroids(img, offset, guesses, radius_px):
    """Intensity-weighted centroids (u, v) of bright dots near each guess."""
    u0, v0 = offset
    out = []
    bg = np.percentile(img, 20)
    r = int(np.ceil(radius_px * 1.8))
    for gu, gv in guesses:
        iu, iv = int(round(gu)) - u0, int(round(gv)) - v0
        w = img[iv - r:iv + r + 1, iu - r:iu + r + 1].astype(float) - bg
        w[w < 0] = 0
        vv, uu = np.mgrid[iv - r:iv + r + 1, iu - r:iu + r + 1]
        out.append([(w * uu).sum() / w.sum() + u0, (w * vv).sum() / w.sum() + v0])
    return np.array(out)


def calibrate(cam, z0, origin, rng=None, holdout=0.2, **target):
    """Render the target at Z = z0, find its dots, fit H on 80 % and check on the rest.
    Returns (H plane->image, report dict)."""
    rng = np.random.default_rng(0) if rng is None else rng
    img, off, centres = render_target(cam, z0, origin, rng=rng, **target)
    guess = cam.project(np.column_stack([centres, np.full(len(centres), z0)]))
    dot = target.get('dot', 4.0)
    uv = dot_centroids(img, off, guess, radius_px=dot / 2 / cam.gsd(cam.t[2] - z0))
    test = rng.random(len(centres)) < holdout
    H = dlt(centres[~test], uv[~test])
    rep = np.linalg.norm(apply(H, centres[~test]) - uv[~test], axis=1)
    Hinv = np.linalg.inv(H)
    back = apply(Hinv, uv[test])
    err = np.linalg.norm(back - centres[test], axis=1)
    return H, dict(dots=int(len(centres)), fit=int((~test).sum()), reproj_rms_px=float(np.sqrt(np.mean(rep ** 2))),
                   check_rms_mm=float(np.sqrt(np.mean(err ** 2))), check_max_mm=float(err.max()))

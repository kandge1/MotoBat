"""Pinhole camera for the S1 overhead midiCam2s, and the image renderer for box scenes. (task 3.2)

Camera (S: SICK I2D605C datasheet, the colour sibling of the mono I2D605M; same Sony IMX264):
    2448 x 2048 px, 3.45 um square pixels, 12 bit, global shutter. Lens: C-mount, focal
    length f (A: a stock 12 or 16 mm lens). No lens distortion is rendered: a calibrated
    system removes it, and what is left is folded into the calibration residual (calib.py).

Pinhole model. A camera frame C sits at position t in world with rotation R (columns = the
camera's x, y, z axes in world; z = viewing direction, x = image right, y = image down, the
OpenCV convention). A world point P has camera coordinates  Pc = R^T (P - t), and lands at
pixel
        u = cx + (f / p) * Pc_x / Pc_z,      v = cy + (f / p) * Pc_y / Pc_z
with p the pixel pitch and (cx, cy) the principal point (the sensor centre, A). The ground
sample distance at distance Z straight below is  GSD = p * Z / f.

Rendering is the inverse: each pixel (supersampled ss x ss times) sends a ray from t along
    d = R (( u - cx) p / f, (v - cy) p / f, 1)
and takes the brightness of the first surface it meets. The scene is axis-aligned-in-z boxes
(cells, pads, terminals, end plates, the tray): a ray meets a box's TOP if, at the top plane's
height, it lies inside the box's (rotated) footprint; a ray that misses every top but crosses a
box's footprint between its top and bottom planes has gone through a SIDE face. Tops are lit
by the ring light round the lens and return their reflectance; vertical sides are lit at
grazing incidence and return `side` (dark). The highest top hit wins (the camera looks down).

Image formation (A, a plain sensor model): the scene radiance (0..1) times an exposure that
puts a white top at `exposure` of full well gives photo-electrons N; shot noise ~ Poisson(N)
(Gaussian with variance N here), read noise sigma_r e-; blur by a Gaussian PSF of sigma_psf
pixels (lens + pixel aperture) before noise; quantised to 12 bit.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import gaussian_filter

SENSOR = dict(width=2448, height=2048, pitch=3.45e-3, bits=12)   # mm; S (SICK I2D605C datasheet)
NOISE = dict(full_well=10000.0, read=3.0, exposure=0.8, psf_px=0.6)  # A


def look_down(position, yaw=0.0):
    """Camera at `position` (mm) looking straight down; image x along world x rotated by yaw."""
    c, s = np.cos(yaw), np.sin(yaw)
    x = np.array([c, s, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    y = np.cross(z, x)
    return np.asarray(position, float), np.column_stack([x, y, z])


@dataclass
class Camera:
    f: float                                    # mm
    t: np.ndarray                               # mm, world
    R: np.ndarray                               # columns: camera axes in world
    width: int = SENSOR['width']
    height: int = SENSOR['height']
    pitch: float = SENSOR['pitch']
    cx: float = field(default=None)
    cy: float = field(default=None)

    def __post_init__(self):
        self.cx = (self.width - 1) / 2 if self.cx is None else self.cx
        self.cy = (self.height - 1) / 2 if self.cy is None else self.cy

    def gsd(self, Z):
        return self.pitch * Z / self.f

    def project(self, P):
        Pc = (np.asarray(P, float) - self.t) @ self.R
        k = self.f / self.pitch
        return np.column_stack([self.cx + k * Pc[:, 0] / Pc[:, 2], self.cy + k * Pc[:, 1] / Pc[:, 2]])

    def rays(self, u, v):
        """Unit-free ray directions (world) through pixel coordinates u, v (arrays)."""
        k = self.pitch / self.f
        dc = np.stack([(u - self.cx) * k, (v - self.cy) * k, np.ones_like(u)], axis=-1)
        return dc @ self.R.T


@dataclass
class Box:
    """A box standing on the plane z = bottom, footprint centred (x, y), sizes (sx along its own
    x, sy along its own y), rotated by yaw about z. Values in mm and rad."""
    x: float
    y: float
    yaw: float
    sx: float
    sy: float
    bottom: float
    top: float
    reflect: float = 0.7          # top brightness, 0..1


def _inside(box, px, py):
    c, s = np.cos(box.yaw), np.sin(box.yaw)
    dx, dy = px - box.x, py - box.y
    a = c * dx + s * dy
    b = -s * dx + c * dy
    return (np.abs(a) <= box.sx / 2) & (np.abs(b) <= box.sy / 2)


def radiance(cam, boxes, u, v, floor_z=-1e9, floor=0.25, side=0.08):
    """Scene brightness (0..1) seen along the rays through pixel coordinates u, v."""
    d = cam.rays(u, v)
    out = np.full(u.shape, floor, float)
    best = np.full(u.shape, floor_z)
    for b in sorted(boxes, key=lambda b: b.top):            # highest last, so it wins ties
        for z, val in ((b.top, b.reflect),):
            s = (z - cam.t[2]) / d[..., 2]
            hit = _inside(b, cam.t[0] + s * d[..., 0], cam.t[1] + s * d[..., 1]) & (z > best)
            out[hit], best[hit] = val, z
        # through a side: inside the footprint at the bottom plane, not at the top
        s0 = (b.bottom - cam.t[2]) / d[..., 2]
        through = _inside(b, cam.t[0] + s0 * d[..., 0], cam.t[1] + s0 * d[..., 1]) & (b.top > best)
        out[through], best[through] = side, b.top - 1e-6   # a side, below this top
    return out


def subsamples(u0, v0, u1, v1, ss, rng):
    """ss x ss jittered (stratified) sample positions in every pixel of the window.
    A pixel integrates light over its area. A FIXED sub-grid would quantise an edge's coverage
    in steps of 1/ss pixel: an edge nearly parallel to the pixel rows then lands up to 1/(2 ss)
    pixel off (29 um at 0.17 mm/px with ss = 3, measured 26 Nov before this fix). Jittering
    each sample inside its own sub-cell makes the coverage estimate unbiased."""
    base = np.arange(ss) / ss - 0.5
    shape = (v1 - v0, u1 - u0, ss, ss)
    uu = np.arange(u0, u1)[None, :, None, None] + base[None, None, None, :] + rng.random(shape) / ss
    vv = np.arange(v0, v1)[:, None, None, None] + base[None, None, :, None] + rng.random(shape) / ss
    return uu, vv


def render(cam, boxes, roi=None, ss=3, rng=None, noise=NOISE, floor=0.25, side=0.08):
    """12-bit image of the scene (or of roi = (u0, v0, u1, v1), half-open) and its offset."""
    rng = np.random.default_rng() if rng is None else rng
    u0, v0, u1, v1 = roi if roi is not None else (0, 0, cam.width, cam.height)
    uu, vv = subsamples(u0, v0, u1, v1, ss, rng)
    L = radiance(cam, boxes, uu, vv, floor=floor, side=side).mean(axis=(2, 3))
    L = gaussian_filter(L, noise['psf_px'])
    N = L * noise['exposure'] * noise['full_well']
    N = N + rng.normal(0.0, 1.0, N.shape) * np.sqrt(N + noise['read'] ** 2)
    dn = np.clip(np.round(N / noise['full_well'] * (2 ** SENSOR['bits'] - 1)), 0, 2 ** SENSOR['bits'] - 1)
    return dn.astype(np.uint16), (u0, v0)

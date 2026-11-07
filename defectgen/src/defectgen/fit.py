"""The geometric fits detection is built from (project_plan.md §4B.4).

Pure numpy and scipy, no Open3D, so each fit can be checked on its own
against a known answer. Millimetres, module frame, Z up.

* `fit_plane` -- trimmed least squares, z = a + b(x - x0) + c(y - y0).
* `min_area_rect` -- the smallest rectangle around a 2D point set (rotating
  calipers over the convex hull): centre, yaw, extents.
* `rigid_2d` -- the rotation and translation that best map one 2D point set
  onto another (Kabsch / Umeyama without scale).
* `fit_line` -- trimmed least squares, y = a + b x, for the row of terminal
  heights.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Plane:
    """z = a + b (x - x0) + c (y - y0), fitted to `n` points with `rms`
    residual after trimming."""

    x0: float
    y0: float
    a: float
    b: float
    c: float
    rms: float
    n: int

    def z(self, x, y):
        return self.a + self.b * (np.asarray(x) - self.x0) + self.c * (np.asarray(y) - self.y0)

    def residual(self, p: np.ndarray) -> np.ndarray:
        """Vertical distance of each point above the plane."""
        return p[:, 2] - self.z(p[:, 0], p[:, 1])

    @property
    def normal(self) -> np.ndarray:
        n = np.array([-self.b, -self.c, 1.0])
        return n / np.linalg.norm(n)

    @property
    def tilt_deg(self) -> float:
        """Angle between the plane normal and +Z."""
        return float(np.degrees(np.arccos(np.clip(self.normal[2], -1.0, 1.0))))


def fit_plane(p: np.ndarray, iters: int = 5, k: float = 3.0, floor: float = 1e-3) -> tuple[Plane, np.ndarray]:
    """Least-squares plane, refitted `iters` times on the points within
    max(k * rms, floor) of the last fit. Returns the plane and the inlier mask.

    Trimming is what lets a terminal top be fitted with a speck of FOD on it,
    or a busbar top with weld beads standing 0.3 mm proud of it. Raises if
    fewer than three points survive."""
    if len(p) < 3:
        raise ValueError(f"need at least 3 points for a plane, got {len(p)}")
    x0, y0 = float(p[:, 0].mean()), float(p[:, 1].mean())
    a_ = np.column_stack([np.ones(len(p)), p[:, 0] - x0, p[:, 1] - y0])
    keep = np.ones(len(p), bool)
    coef = np.zeros(3)
    for _ in range(iters):
        if keep.sum() < 3:
            raise ValueError("plane fit trimmed away all but two points")
        coef, *_ = np.linalg.lstsq(a_[keep], p[keep, 2], rcond=None)
        res = p[:, 2] - a_ @ coef
        rms = float(np.sqrt(np.mean(res[keep] ** 2)))
        new = np.abs(res) <= max(k * rms, floor)
        if np.array_equal(new, keep):
            break
        keep = new
    res = p[:, 2] - a_ @ coef
    rms = float(np.sqrt(np.mean(res[keep] ** 2))) if keep.any() else float("nan")
    return Plane(x0, y0, *map(float, coef), rms, int(keep.sum())), keep


def ransac_plane(p: np.ndarray, tol: float, trials: int = 200, sample: int = 5000,
                 max_tilt_deg: float = 15.0, seed: int = 0) -> np.ndarray:
    """Inlier mask of the near-horizontal plane with the most points within
    `tol` (Fischler & Bolles 1981). A seed for `fit_plane` where the dominant
    surface shares the crop with others -- a busbar top with weld beads 0.3 mm
    proud of it, a terminal pad beside a busbar. Planes tilted more than
    `max_tilt_deg` are not candidates: every face fitted here is a top face.
    Scored on a fixed-seed subsample, so it is deterministic."""
    rng = np.random.default_rng(seed)
    q = p if len(p) <= sample else p[rng.choice(len(p), sample, replace=False)]
    tri = q[rng.integers(0, len(q), size=(trials, 3))]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    length = np.linalg.norm(n, axis=1)
    ok = length > 1e-9
    n[ok] /= length[ok, None]
    ok &= np.abs(n[:, 2]) >= np.cos(np.radians(max_tilt_deg))
    if not ok.any():
        return np.ones(len(p), bool)
    n, a = n[ok], tri[ok, 0]
    d = np.abs((q[None, :, :] - a[:, None, :]) @ n[..., None])[..., 0] if len(n) * len(q) <= 2e6 else None
    if d is None:
        counts = np.array([(np.abs((q - ai) @ ni) < tol).sum() for ai, ni in zip(a, n)])
    else:
        counts = (d < tol).sum(axis=1)
    best = int(np.argmax(counts))
    return np.abs((p - a[best]) @ n[best]) < tol


def fit_line(x: np.ndarray, y: np.ndarray, iters: int = 5, k: float = 3.0, floor: float = 1e-3):
    """y = a + b x, trimmed on the median absolute deviation: keep points with
    |e - median(e)| <= max(k * 1.4826 * MAD, floor), refit. Returns (a, b,
    inlier mask).

    MAD rather than rms because a row of terminals is short -- six to twelve
    points -- and one raised terminal inflates the rms enough to keep itself,
    pulling the line toward it. The fit starts flat at the median for the
    same reason. Fewer than three points: the median, slope 0."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 3:
        return float(np.median(y)), 0.0, np.ones(len(x), bool)
    a_ = np.column_stack([np.ones(len(x)), x])
    coef = np.array([np.median(y), 0.0])
    keep = np.ones(len(x), bool)
    for _ in range(iters):
        res = y - a_ @ coef
        dev = np.abs(res - np.median(res))
        new = dev <= max(k * 1.4826 * float(np.median(dev)), floor)
        if new.sum() < 2:
            break
        keep = new
        coef, *_ = np.linalg.lstsq(a_[keep], y[keep], rcond=None)
    return float(coef[0]), float(coef[1]), keep


@dataclass(frozen=True)
class Rect:
    """A rectangle in the XY plane: centre, yaw of its first axis from +X
    (degrees, in (-45, 45]), and its extent along each axis."""

    cx: float
    cy: float
    yaw_deg: float
    ex: float
    ey: float

    def corners(self) -> np.ndarray:
        c, s = np.cos(np.radians(self.yaw_deg)), np.sin(np.radians(self.yaw_deg))
        u = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [self.ex / 2, self.ey / 2]
        return u @ np.array([[c, s], [-s, c]]) + [self.cx, self.cy]


def min_area_rect(xy: np.ndarray) -> Rect:
    """Smallest-area enclosing rectangle. One side of it is collinear with a
    convex hull edge (Freeman & Shapira 1975), so only the hull edge angles
    need trying.

    The yaw is folded into (-45, 45], so `ex` is the extent along the axis
    nearest +X. Callers whose part is near square must not rely on which
    axis is which beyond that."""
    from scipy.spatial import ConvexHull, QhullError

    xy = np.asarray(xy, float)
    if len(xy) < 3:
        raise ValueError(f"need at least 3 points for a rectangle, got {len(xy)}")
    try:
        hull = xy[ConvexHull(xy).vertices]
    except QhullError:
        hull = xy
    edges = np.diff(np.vstack([hull, hull[:1]]), axis=0)
    angles = np.unique(np.mod(np.arctan2(edges[:, 1], edges[:, 0]), np.pi / 2))
    best = None
    for th in angles:
        c, s = np.cos(th), np.sin(th)
        # Coordinates along the rotated axes u = (c, s), v = (-s, c).
        u = hull @ np.array([c, s])
        v = hull @ np.array([-s, c])
        area = (u.max() - u.min()) * (v.max() - v.min())
        if best is None or area < best[0]:
            best = (area, th, u, v)
    _, th, u, v = best
    uc, vc = (u.max() + u.min()) / 2, (v.max() + v.min()) / 2
    c, s = np.cos(th), np.sin(th)
    cx, cy = uc * c - vc * s, uc * s + vc * c
    ex, ey = u.max() - u.min(), v.max() - v.min()
    yaw = np.degrees(th)
    if yaw > 45.0:
        yaw -= 90.0
        ex, ey = ey, ex
    return Rect(float(cx), float(cy), float(yaw), float(ex), float(ey))


def rigid_2d(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """R (2x2), t with dst ~ R src + t in the least-squares sense (Arun et al.
    1987, the 2D case of plan §9.7). Two points are enough."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    ps, pd = src.mean(axis=0), dst.mean(axis=0)
    h = (src - ps).T @ (dst - pd)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T)) or 1.0
    r = vt.T @ np.diag([1.0, d]) @ u.T
    return r, pd - r @ ps

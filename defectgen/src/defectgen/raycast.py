"""Exact, vectorised ray casting along a coordinate axis.

Every ray `settle.py` casts runs along X, Y or Z: straight down onto supports,
along the stack between cells, outward toward a side plate. Along an axis a
ray is a 2D point-in-triangle test in the plane across that axis, plus one
interpolation for where along the axis it meets the triangle. Done in float64
numpy over a uniform bucket grid, that is both much faster than trimesh's
general caster (which visits an r-tree once per ray from Python) and exact.

Exactness is the point. Mating faces here touch at zero clearance, and
Embree/Open3D cast in float32 -- about 1 um of noise at 100 mm, enough to make
a clean module read as interpenetrating and nudge parts that should not move.
"""

from __future__ import annotations

import numpy as np
import trimesh

# A point on a shared edge or on the rim of a face counts as a hit: contact
# along an edge is still contact.
EDGE_TOL = 1e-9


class AxisCaster:
    """All the places a line parallel to `axis` meets a mesh."""

    def __init__(self, mesh: trimesh.Trimesh, axis: int, cell: float = 4.0):
        self.axis = axis
        across = [a for a in range(3) if a != axis]
        tri = mesh.triangles
        uv = tri[:, :, across]
        w = tri[:, :, axis]

        # Faces seen edge-on from this axis have no area to hit.
        e1, e2 = uv[:, 1] - uv[:, 0], uv[:, 2] - uv[:, 0]
        det = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
        keep = np.abs(det) > 1e-12
        self.uv0, self.e1, self.e2 = uv[keep, 0], e1[keep], e2[keep]
        self.det, self.w = det[keep], w[keep]

        self.cell = cell
        lo = uv[keep].min(axis=1)
        hi = uv[keep].max(axis=1)
        self.origin = lo.min(axis=0) if len(lo) else np.zeros(2)
        first = np.floor((lo - self.origin) / cell).astype(int)
        last = np.floor((hi - self.origin) / cell).astype(int)
        self.shape = (last.max(axis=0) + 1) if len(lo) else np.ones(2, int)

        # Every (cell, triangle) pair a triangle's bounding box covers, sorted
        # by cell so a query reads its candidates as one contiguous slice.
        span = last - first + 1
        count = span[:, 0] * span[:, 1]
        tri_id = np.repeat(np.arange(len(first)), count)
        offset = np.arange(count.sum()) - np.repeat(np.cumsum(count) - count, count)
        cu = first[tri_id, 0] + offset % span[tri_id, 0]
        cv = first[tri_id, 1] + offset // span[tri_id, 0]
        cell_id = cu * self.shape[1] + cv
        order = np.argsort(cell_id, kind="stable")
        self.tri_of = tri_id[order]
        self.start = np.searchsorted(cell_id[order], np.arange(self.shape[0] * self.shape[1] + 1))

    def hits(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(point index, coordinate along the axis) for every crossing."""
        across = [a for a in range(3) if a != self.axis]
        uv = np.asarray(points, dtype=float)[:, across]
        c = np.floor((uv - self.origin) / self.cell).astype(int)
        inside = np.all((c >= 0) & (c < self.shape), axis=1)
        idx = np.where(inside)[0]
        if not len(idx):
            return np.empty(0, int), np.empty(0)
        cid = c[idx, 0] * self.shape[1] + c[idx, 1]
        lo, hi = self.start[cid], self.start[cid + 1]
        n = hi - lo
        pt = np.repeat(idx, n)
        tri = self.tri_of[np.repeat(lo, n) + np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)]

        d = uv[pt] - self.uv0[tri]
        e1, e2, det = self.e1[tri], self.e2[tri], self.det[tri]
        b1 = (d[:, 0] * e2[:, 1] - d[:, 1] * e2[:, 0]) / det
        b2 = (e1[:, 0] * d[:, 1] - e1[:, 1] * d[:, 0]) / det
        ok = (b1 >= -EDGE_TOL) & (b2 >= -EDGE_TOL) & (b1 + b2 <= 1 + EDGE_TOL)
        tri, b1, b2 = tri[ok], b1[ok], b2[ok]
        w = self.w[tri]
        at = w[:, 0] + b1 * (w[:, 1] - w[:, 0]) + b2 * (w[:, 2] - w[:, 0])
        return pt[ok], at

    def extreme(self, points: np.ndarray, highest: bool) -> np.ndarray:
        """Highest (or lowest) crossing along the axis per point; NaN if none."""
        pt, at = self.hits(points)
        out = np.full(len(points), -np.inf if highest else np.inf)
        if highest:
            np.maximum.at(out, pt, at)
        else:
            np.minimum.at(out, pt, at)
        out[~np.isfinite(out)] = np.nan
        return out

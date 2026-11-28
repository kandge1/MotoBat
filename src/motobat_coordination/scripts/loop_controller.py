"""The cell-placement correction law, per axis (x, y, yaw), k = cell index. (task 3.4)

Run-to-run integral (I-form) control, as task 1.12 derived and tested in numpy (correction/README.md):

    e_k     = measured placement error of cell k (it already contains the offset u_k it was placed with)
    u_{k+1} = u_k - K e_k                       the offset the robot adds to cell k+1

With a plant that turns a commanded offset into g times as much movement, a constant disturbance d
and white placement noise w:  x_k = g u_k + d + w_k,  e_k = x_k + v_k (gauge noise v). Then
    e_{k+1} = (1 - g K) e_k + (noise terms)
one pole at 1 - gK: stable for 0 < gK < 2, monotone below 1, deadbeat at 1, alternating above.
Integral action drives the MEAN error to zero for any constant d (and to zero minus the gauge's
bias: a biased gauge is copied into the product). The price of a high K is noise: at steady
state  Var(x) = s_w^2 + K (s_w^2 + s_v^2) / (2 - K)  (g = 1), so deadbeat doubles the placement
variance (Deming's funnel). The gain is a trade between how fast an offset is removed (it decays
as (1 - gK)^k) and how much noise is fed back.

Anti-windup: |u| is clamped (u_max); a clamped axis stops integrating in the direction of the clamp.
An axis can be disabled (K = 0): it then stays at u = 0.
"""
from __future__ import annotations

import numpy as np


class ILoop:
    def __init__(self, gains, u_max):
        self.K = np.asarray(gains, float)          # per axis
        self.u_max = np.asarray(u_max, float)
        self.u = np.zeros_like(self.K)
        self.history = []

    def update(self, e):
        """Take cell k's measured error e (per axis), return u_{k+1}."""
        e = np.asarray(e, float)
        step = -self.K * e
        u = self.u + step
        clamped = np.abs(u) > self.u_max
        u = np.clip(u, -self.u_max, self.u_max)
        self.history.append(dict(e=e.tolist(), u_before=self.u.tolist(), u_after=u.tolist(),
                                 clamped=clamped.tolist()))
        self.u = u
        return self.u.copy()

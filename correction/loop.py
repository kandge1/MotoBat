"""Task 1.12: the cell-placement correction loop, in numpy (plan §4B.7, §9.9).

k is the cell index across a shift. Per axis (x mm, y mm, yaw deg):

    d_k  = offset + ramp·k (+ step after step_at)       systematic: gripper, fixture wear
    x_k  = g·u_k + d_k + w_k,   w ~ N(0, σ_place²)       where cell k actually lands (its error)
    e_k  = x_k + b_m + v_k,     v ~ N(0, σ_meas²)        what the station measures (b_m: gauge bias)
    S_k  = S_{k-1} + e_k
    u_{k+1} = −Kp·e_k − Ki·S_k                           the offset the robot adds to cell k+1

Three controllers fall out of (Kp, Ki):
    open loop      Kp = Ki = 0
    P              Kp = K, Ki = 0:   u_{k+1} = −K·e_k
    I (run-to-run) Kp = 0, Ki = K:   u_{k+1} = u_k − K·e_k

The plan writes the loop as u_{k+1} = −K·e_k with the stability band 0 < gK < 2. That band belongs
to the I form: there the error obeys e_{k+1} = (1 − gK)·e_k. The P form (offset recomputed from
scratch each cell) obeys e_{k+1} = −gK·e_k + d: stable only for gK < 1, and it never removes a
constant offset (it leaves d / (1 + gK)). The I form is what a real cell-placement loop runs.

Closed form for the general loop (a_p = g·Kp, a_i = g·Ki): the poles are the roots of
    z² + (a_p + a_i − 1)·z − a_p = 0,
and the loop is stable when both lie inside the unit circle (Jury: |a_p| < 1, a_i > 0,
a_i + 2·a_p < 2). Kp > 0 puts a pole on the negative real axis, i.e. cell-to-cell alternation;
the I form has the single pole 1 − a_i.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from scipy.linalg import solve_discrete_lyapunov
from scipy.stats import norm

ROOT = Path(__file__).resolve().parents[1]
CONFIG = Path(__file__).with_name("loop.yaml")
DETECT_YAML = ROOT / "defectgen/src/defectgen/configs/detect.yaml"
AXES = ("x", "y", "yaw")
# 1.9 results, newest first: the v3 dataset's, then the v1 dataset's (fixed only).
GRR_FILES = ("data/dataset/validation/grr.{setup}.json", "data/dataset_plan_v1/validation/grr.{setup}.json")


@dataclass
class Plant:
    g: float
    sigma_place: np.ndarray
    offset: np.ndarray
    ramp: np.ndarray
    step_at: int
    step: np.ndarray
    sigma_meas: np.ndarray
    bias_meas: np.ndarray

    def drift(self, k: np.ndarray) -> np.ndarray:
        """(n, 3) systematic error at cells k."""
        k = np.asarray(k)[:, None]
        return self.offset + self.ramp * k + (k >= self.step_at) * self.step


@dataclass
class Gains:
    name: str
    kp: float = 0.0
    ki: float = 0.0


OPEN = Gains("open loop")


def P(k: float) -> Gains:
    return Gains("P", kp=k)


def I(k: float) -> Gains:
    return Gains("I", ki=k)


# -- configuration -----------------------------------------------------------

@dataclass
class Setup:
    plant: Plant
    tol: np.ndarray            # pos_tol (radial, mm), pos_tol, yaw_tol (deg)
    height_tol: float
    sigma_height_meas: float
    grr_source: str
    cfg: dict = field(repr=False, default_factory=dict)


def find_grr(setup: str) -> Path | None:
    for f in GRR_FILES:
        p = ROOT / f.format(setup=setup)
        if p.is_file():
            return p
    return None


def load(setup: str = "fixed", config: Path = CONFIG, grr_path: Path | None = None) -> Setup:
    cfg = yaml.safe_load(Path(config).read_text())
    det = yaml.safe_load(DETECT_YAML.read_text())
    grr_path = grr_path or find_grr(setup)
    if grr_path is None:
        raise FileNotFoundError(f"no 1.9 %GRR result for the {setup} setup: run "
                                f"python -m defectgen.validate grr --setup {setup} first")
    grr = json.loads(Path(grr_path).read_text())["characteristics"]
    cp, th = grr["cell_position"], grr["terminal_height"]
    m = cfg["measure"]
    s_xy = float(cp["GRR"])
    s_yaw = math.degrees(math.sqrt(2) * s_xy / m["terminal_span_mm"])
    pl = cfg["plant"]
    plant = Plant(g=pl["g"], sigma_place=np.array(pl["sigma_place"], float),
                  offset=np.array(pl["offset"], float), ramp=np.array(pl["ramp"], float),
                  step_at=int(pl["step_at"]), step=np.array(pl["step"], float),
                  sigma_meas=np.array([s_xy, s_xy, s_yaw]),
                  bias_meas=np.array([float(cp["bias"]["mean_bias"]), m["bias_y"], m["bias_yaw"]]))
    tol = np.array([det["cell"]["pos_tol"], det["cell"]["pos_tol"], det["cell"]["yaw_tol"]], float)
    return Setup(plant, tol, float(det["terminal"]["height_tol"]), float(th["GRR"]),
                 str(Path(grr_path).relative_to(ROOT) if Path(grr_path).is_relative_to(ROOT) else grr_path),
                 cfg)


# -- simulation --------------------------------------------------------------

def simulate(plant: Plant, gains: Gains, cells: int, runs: int, seed: int = 0,
             g: float | None = None, noise: bool = True) -> dict:
    """Monte Carlo of the loop. Returns x (true error) and e (measured), each (runs, cells, 3).
    `g` overrides the plant gain (the controller's gains stay as designed for g = 1)."""
    g = plant.g if g is None else g
    rng = np.random.default_rng(seed)
    d = plant.drift(np.arange(cells))
    x = np.empty((runs, cells, 3))
    e = np.empty((runs, cells, 3))
    u = np.zeros((runs, 3))
    s = np.zeros((runs, 3))
    with np.errstate(over="ignore", invalid="ignore"):
        for k in range(cells):
            w = rng.standard_normal((runs, 3)) * plant.sigma_place if noise else 0.0
            v = rng.standard_normal((runs, 3)) * plant.sigma_meas if noise else 0.0
            x[:, k] = g * u + d[k] + w
            e[:, k] = x[:, k] + plant.bias_meas + v
            s += e[:, k]
            u = -gains.kp * e[:, k] - gains.ki * s
    return {"x": x, "e": e}


def out_of_tol(x: np.ndarray, tol: np.ndarray) -> np.ndarray:
    """Placement defect per cell: centre off by more than pos_tol (radially), or yaw past yaw_tol."""
    with np.errstate(invalid="ignore", over="ignore"):
        bad = (np.hypot(x[..., 0], x[..., 1]) > tol[0]) | (np.abs(x[..., 2]) > tol[2])
    return bad | ~np.isfinite(x).all(axis=-1)


def defect_rate(x: np.ndarray, tol: np.ndarray, skip: int = 0) -> dict:
    """Fraction of cells out of tolerance, with a 95 % CI taken across runs (cells within one
    run are correlated through the loop, so they are not independent trials)."""
    per_run = out_of_tol(x[:, skip:], tol).mean(axis=1)
    m = float(per_run.mean())
    half = 1.96 * float(per_run.std(ddof=1)) / math.sqrt(len(per_run)) if len(per_run) > 1 else 0.0
    return {"rate": m, "lo": max(0.0, m - half), "hi": min(1.0, m + half)}


# -- closed form -------------------------------------------------------------

def poles(a_p: float, a_i: float) -> np.ndarray:
    if a_i == 0:
        return np.array([-a_p])            # P only: x_{k+1} = −a_p·x_k + ...
    return np.roots([1.0, a_p + a_i - 1.0, -a_p])


def stable(a_p: float, a_i: float) -> bool:
    return bool(np.max(np.abs(poles(a_p, a_i))) < 1.0)


def steady_mean(a_p: float, a_i: float, offset: float, ramp: float, bias: float) -> float:
    """Steady-state mean of x. With integral action the loop drives the *measured* error to
    r / a_i, so x settles at r / a_i − b_m: a gauge bias passes straight into placement.
    P only (ramp must be 0): x = (d − a_p·b_m) / (1 + a_p)."""
    if a_i > 0:
        return ramp / a_i - bias
    if ramp:
        return math.copysign(math.inf, ramp)
    return (offset - a_p * bias) / (1.0 + a_p)


def steady_var(a_p: float, a_i: float, s_place: float, s_meas: float) -> float:
    """Steady-state variance of x for white placement and measurement noise.
    I only: σ_x² = σ_w² + a·(σ_w² + σ_v²)/(2 − a). At a = 1 (deadbeat) that is 2σ_w² + σ_v²:
    correcting every cell fully on one noisy reading doubles the placement variance."""
    if not stable(a_p, a_i):
        return math.inf
    n2 = s_place ** 2 + s_meas ** 2
    if a_i == 0:
        return s_place ** 2 + a_p ** 2 * n2 / (1.0 - a_p ** 2)
    # state (y_k, S_{k-1}), y = x − w; noise n = w + v enters y with −(a_p + a_i), S with +1
    A = np.array([[-(a_p + a_i), -a_i], [1.0, 1.0]])
    B = np.array([[-(a_p + a_i)], [1.0]])
    Pm = solve_discrete_lyapunov(A, B @ B.T * n2)
    return float(Pm[0, 0] + s_place ** 2)


def p_out(mean: float, sd: float, tol: float) -> float:
    """P(|N(mean, sd²)| > tol), one axis."""
    if not math.isfinite(mean) or not math.isfinite(sd):
        return 1.0
    return float(norm.cdf((-tol - mean) / sd) + norm.sf((tol - mean) / sd))


def settle_cells(a: float, frac: float = 0.05) -> float:
    """I form: cells until a step's error is down to `frac` of itself, |1 − a|^k = frac."""
    r = abs(1.0 - a)
    if r == 0:
        return 1.0
    return math.inf if r >= 1 else math.log(frac) / math.log(r)


def best_i_gain(ramp: float, bias: float, s_place: float, s_meas: float, tol: float,
                g: float = 1.0) -> float:
    """The I gain that minimises steady-state P(out of tolerance) on one axis: the trade between
    tracking drift (wants a large) and not chasing noise (wants a small)."""
    ks = np.linspace(0.005, 1.995 / g, 800)
    ps = [p_out(steady_mean(0, g * k, 0, ramp, bias), math.sqrt(steady_var(0, g * k, s_place, s_meas)), tol)
          for k in ks]
    return float(ks[int(np.argmin(ps))])


# -- terminal height: not correctable by placement -------------------------------

def guard_band_rates(sigma_process: float, sigma_meas: float, tol: float, guard: float) -> dict:
    """Terminal height cannot be corrected by moving the next cell, so it is a decision:
    rework the module before the carrier (and so before the irreversible weld) when the measured
    adjacent difference is past tol − guard. With true Δh ~ N(0, σ_p²) and reading Δh + N(0, σ_m²):
      escape        = P(|Δh| > tol  and |reading| ≤ tol − guard)   a bad module reaches the weld
      false rework  = P(|Δh| ≤ tol  and |reading| > tol − guard)   a good module is reworked
    Integrated over Δh on a fine grid."""
    lim = tol - guard
    edges = np.linspace(-8 * sigma_process, 8 * sigma_process, 40001)
    h = (edges[1:] + edges[:-1]) / 2       # midpoints: no grid point sits exactly on ±tol
    ph = np.diff(norm.cdf(edges, scale=sigma_process))
    sm = max(sigma_meas, 1e-12)
    accept = norm.cdf((lim - h) / sm) - norm.cdf((-lim - h) / sm)
    bad = np.abs(h) > tol
    return {"guard": guard, "escape": float((ph * bad * accept).sum()),
            "false_rework": float((ph * ~bad * (1 - accept)).sum()),
            "rework": float((ph * (1 - accept)).sum()), "truly_bad": float((ph * bad).sum())}


# -- what each finding means for the module ------------------------------------

# Reject or rework happens at stations 6 and 8, before the irreversible weld at 9 (module_spec
# §6). Placement is also corrected forward, by the loop, for the cells that follow.
DECISION = {
    "cell_placement": "rework before carrier: re-place the cell (the loop corrects the cells after it)",
    "terminal_height": "rework before carrier: reseat or swap the cell / pad (placement cannot fix height)",
    "terminal_tilt": "rework before carrier: reseat the cell (placement cannot fix tilt)",
    "missing_cell": "rework before carrier: place the cell",
    "reversed_polarity": "rework before carrier: turn the cell",
    "busbar_pose": "rework before weld: re-place the busbar carrier",
    "busbar_liftoff": "rework before weld: re-seat the carrier (the gap decides the weld)",
    "carrier_seating": "rework before weld: re-seat the carrier",
}
AFTER_WELD = "reject, or repair offline: the weld cannot be undone in line"


def decide(cls: str, stage: str) -> str:
    if stage == "post_weld":
        return AFTER_WELD
    return DECISION.get(cls, "hold for review: no rule for this class")

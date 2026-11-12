"""Detection: point clouds in, a DefectReport out (task 1.4, project_plan.md §4B.4).

    python -m defectgen.detect data/run --stage pre_busbar

The pipeline, per capture:

1. **Coarse align** -- the fixture pose. A module on its pallet sits at the
   3-2-1 datum (module_spec §10), so the sensor poses are already in the
   module frame to within the fixture's repeatability.
2. **Fine align, to the housing only** -- no defect moves it. Z, roll and
   pitch from point-to-plane ICP on the housing points against the golden
   capture (below); X, Y and yaw from the two end-plate outlines. Whole-module
   ICP was tried first and failed: a carrier seated high dragged it in Z, and
   from above it barely constrains X and Y (plan §4B.4, "As built").
3. **Golden capture** -- the *same rays* cast at the clean reference module for
   this stage, through the same frozen sensor model with the noise left off.
   It is what this capture would have returned from a perfect module. Every
   feature is measured on both and reported as the difference, f(scan) -
   f(golden), which cancels whatever the method itself gets wrong (chamfered
   edges, shadowed corners, sampling) because it gets it equally wrong on both.
4. **Per-feature measurement**, each within this one capture: terminal top
   planes (height, tilt) and outlines (cell pose), cell polarity, busbar top
   planes and outlines (pose), the busbar-to-pad gap at every terminal, the
   carrier top, and the bead profile along every weld stitch.
5. **Residual segmentation** -- ray by ray against the golden capture. Because
   the comparison is per ray, occlusion is handled exactly: a missing cell
   shows as rays that went 91 mm too far, not as points that happen to sit
   near some hidden CAD face.
6. **Classify** into the §4B.2 classes and emit a `DefectReport`.

Captures are then combined per feature by taking, for each feature, the
capture that saw it best *by design* -- the most golden points on it, which
does not depend on the defects. A feature no capture saw is listed as
uncovered rather than passed (that is the coverage question of task 1.5).

The detector never reads `ScanResult.truth` or `.material`: a `Capture` keeps
only what a real sensor returns. It does use the clean CAD (the golden
module) and the frozen sensor model, as a CAD-based inspection cell would.

Thresholds: configs/detect.yaml, fingerprinted by `detect_hash`.
"""

from __future__ import annotations

import hashlib
import json
import logging
import time
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np
import trimesh
import yaml

from .config import Config, load
from .fit import Plane, Rect, fit_line, fit_plane, min_area_rect, ransac_plane, rigid_2d
from .groundtruth import Labels, Stage
from .module import (
    busbar_length,
    cell_transform,
    collect_labels,
    injected_plan,
    realise_groups,
    terminal_local,
)
from .parts import load_all
from .plan import rotation_z
from .scan import ScanResult, Scene, line_scan, load_params, snapshot, without_groups

CONFIG = Path(__file__).parent / "configs" / "detect.yaml"
log = logging.getLogger("defectgen.detect")


def load_detect_params(path: Path | None = None) -> dict:
    return yaml.safe_load(Path(path or CONFIG).read_text())


def detect_hash(path: Path | None = None) -> str:
    canon = json.dumps(load_detect_params(path), sort_keys=True)
    return hashlib.sha256(canon.encode()).hexdigest()


# -- inputs ------------------------------------------------------------------

@dataclass
class Capture:
    """What one sensor returned in one capture, and nothing else: points in
    the module (pallet datum) frame, the organised ray index, the sensor poses
    and how it was driven. Intensity is kept because real sensors return it."""

    sensor: str
    points: np.ndarray
    index: np.ndarray
    grid: tuple
    poses: np.ndarray
    settings: dict
    signal: np.ndarray | None = None
    params_hash: str = ""
    name: str = ""
    source: dict = field(default_factory=dict)   # what was scanned (ScanResult.source)

    @classmethod
    def from_scan(cls, r: ScanResult, name: str = "") -> "Capture":
        if r.index is None or r.grid is None:
            raise ValueError(
                "this scan has no ray index (saved before detection existed); "
                "rescan it with python -m defectgen.scan"
            )
        return cls(r.sensor, r.points, r.index, tuple(r.grid), r.poses, dict(r.settings),
                   r.signal, r.params_hash, name or r.sensor, dict(r.source))

    @classmethod
    def load(cls, path: Path | str) -> "Capture":
        return cls.from_scan(ScanResult.load(path), name=Path(path).stem)

    def transformed(self, t: np.ndarray) -> "Capture":
        """The same capture with a rigid transform applied to points and poses."""
        pts = self.points @ t[:3, :3].T + t[:3, 3]
        return Capture(self.sensor, pts, self.index, self.grid, t @ self.poses, self.settings,
                       self.signal, self.params_hash, self.name, self.source)


@dataclass
class Golden:
    """The noise-free capture of the clean reference through the same rays."""

    points: np.ndarray
    index: np.ndarray
    material: np.ndarray
    material_names: list
    axis: np.ndarray        # (N, 3) unit measuring direction, sensor into scene
    footprint: np.ndarray   # (N,) mm^2 of surface each ray stands for
    # Typical distance between neighbouring samples on the module, in the
    # coarser direction (mm). An outline places an edge only to about half of
    # it, so it decides which positions this capture can measure (capable()).
    spacing: float = 0.0


class Reference:
    """The clean module for one stage: the golden plan, its labels (nominal
    feature positions) and a raycasting scene of its visible body groups."""

    def __init__(self, stage: Stage, cfg: Config | None = None, include=(), parts=None,
                 run: Path | None = None, max_cached_points: int = 12_000_000):
        self.stage = stage
        self.cfg = cfg or load()
        if run is not None:
            # The golden module is built from the config, so it must be the
            # config the run was generated from.
            f = Path(run) / f"{stage}.labels.json"
            name = json.loads(f.read_text()).get("config_name") if f.is_file() else None
            if name is not None and name != self.cfg.name:
                raise ValueError(f"{run} was generated from config {name!r}, "
                                 f"the reference is {self.cfg.name!r}")
        self.parts = parts if parts is not None else load_all(self.cfg)
        plan = injected_plan(stage, self.cfg, 0, (), self.parts)
        self.plan = plan
        self.labels: Labels = collect_labels(plan, 0, self.parts)
        drop = without_groups(stage, include)
        self.groups = {g: m for g, m in realise_groups(plan, self.parts).items() if g not in drop}
        self.scene = Scene.from_groups(self.groups)
        # Golden captures by pose, least recently used first, bounded by the
        # points they hold. Unbounded, it grew with every new pose -- and in
        # the 1.7 dataset every scan has a fresh fixture placement, so every
        # aligned capture is a new pose: 12 GB and an OOM kill by case two.
        from collections import OrderedDict

        self._golden: OrderedDict = OrderedDict()
        self.max_cached_points = max_cached_points

    def part_box(self, ref: tuple, grow: float = 0.5) -> tuple:
        """Box around a placed part as designed -- all of it, e.g. a negative
        terminal's fill plug as well as its pad."""
        pl = self.plan.find(ref)
        c = trimesh.transform_points(trimesh.bounds.corners(self.parts[pl.part].bounds), pl.transform)
        return _box(c.min(axis=0), c.max(axis=0), grow=grow)

    @staticmethod
    def _key(cap: Capture) -> tuple:
        return (cap.sensor, json.dumps(cap.settings, sort_keys=True, default=str),
                np.round(cap.poses[0], 9).tobytes(), len(cap.poses))

    def forget(self, cap: Capture) -> None:
        """Drop this capture's golden from the cache: an aligned pose carries
        the scan's own fixture error and never comes back."""
        self._golden.pop(self._key(cap), None)

    def golden(self, cap: Capture) -> Golden:
        key = self._key(cap)
        if key in self._golden:
            self._golden.move_to_end(key)
            return self._golden[key]
        gold = self._cast(cap)
        _trim()
        # Cache it only if it fits the budget on its own: a full-length Keyence
        # sweep's golden (~10 M points) would otherwise sit in memory for good.
        if len(gold.points) <= self.max_cached_points:
            self._golden[key] = gold
            while sum(len(g.points) for g in self._golden.values()) > self.max_cached_points:
                self._golden.popitem(last=False)
        return gold

    def as_measured_scene(self, motions: dict, beads: dict) -> Scene:
        """The clean module with every measured part moved to where it was
        measured (see `as_measured`). Built once per detection: every capture
        is cast against the same one."""
        placements = [replace(pl, transform=motions[pl.ref] @ pl.transform) if pl.ref in motions else pl
                      for pl in self.plan.placements]
        moved_beads = {i: (b.copy().apply_transform(beads[i]) if i in beads else b)
                       for i, b in self.plan.beads.items()}
        plan = replace(self.plan, placements=placements, beads=moved_beads)
        groups = {g: m for g, m in realise_groups(plan, self.parts).items() if g in self.groups}
        return Scene.from_groups(groups)

    def as_measured(self, cap: Capture, motions: dict, beads: dict, scene: Scene | None = None) -> Golden:
        """The same rays cast at the clean module with every part detection
        measured moved to where it was measured: `motions` maps a placement ref
        to a 4x4 correction in module coordinates, `beads` a seam index to one
        for its weld bead.

        The residual against this, not against the golden capture, is what no
        feature measurement explains. A part that moved within tolerance -- a
        carrier 0.1 mm high, a busbar on a 0.1 mm gap -- otherwise leaves its
        edges and shadows behind as residual regions (a 0.1 mm carrier left
        5,321 of them), since only a feature *finding* covered them."""
        return self._cast(cap, scene or self.as_measured_scene(motions, beads))

    def _cast(self, cap: Capture, scene: Scene | None = None) -> Golden:
        scene = scene or self.scene
        sensors, _ = load_params()
        cfg = sensors[cap.sensor]
        quiet = logging.getLogger("defectgen.scan")
        level = quiet.level
        quiet.setLevel(logging.WARNING)
        try:
            if cfg["kind"] == "line":
                s = cap.settings
                r = line_scan(scene, cap.sensor, cap.poses[0], s["length_mm"],
                              speed=s.get("speed_mm_s"), rate=s.get("rate_hz"))
            else:
                r = snapshot(scene, cap.poses[0], sensor=cap.sensor)
        finally:
            quiet.setLevel(level)
        if tuple(r.grid) != tuple(cap.grid):
            raise ValueError(f"golden grid {r.grid} does not match the capture's {cap.grid}")
        pts = r.truth
        if cfg["kind"] == "line":
            # Gather only the two pose columns each point needs: indexing the full
            # 4x4 per point built a 128 B/point copy (1.5 GB for a 12 M-point sweep).
            prof = r.index // r.grid[1]
            axis = r.poses[:, :3, 2][prof]
            depth = ((pts - r.poses[:, :3, 3][prof]) * axis).sum(axis=1)
            pitch = s.get("pitch_mm", cfg["profile_pitch"])
            spacing = cfg["line_width"] / (cfg["points"] - 1) * depth / cfg["reference_distance"]
            footprint = pitch * spacing
            coarse = max(pitch, float(np.median(spacing)))
        else:
            eye = r.poses[0][:3, 3]
            axis = pts - eye
            depth = np.linalg.norm(axis, axis=1)
            axis /= depth[:, None]
            h, w = r.grid
            fx = 2 * np.tan(np.radians(cfg["hfov"] / 2)) / (w - 1)
            fy = 2 * np.tan(np.radians(cfg["vfov"] / 2)) / (h - 1)
            footprint = (depth * fx) * (depth * fy)
            coarse = float(np.median(depth) * max(fx, fy))
        # Directions and footprints as float32 (positions stay float64): the
        # residual is a micron-scale difference projected on a unit vector.
        return Golden(pts, r.index, r.material.astype(np.int16), r.material_names, axis.astype(np.float32),
                      np.asarray(footprint, np.float32), coarse)


# -- alignment -----------------------------------------------------------------

# The module's own datum: the housing. Fixed parts no defect class moves, so
# registering to them measures everything else relative to the module (plan
# §9.7, last paragraph). Named by material, because that is what the golden
# capture knows about each ray: end plates and cooling plate are cast, the
# side plates are sheet (configs/materials.yaml).
DATUM_MATERIALS = ("aluminium_cast", "aluminium_sheet")


def icp(src_pts: np.ndarray, dst_pts: np.ndarray, ref: "Reference", p: dict) -> tuple[np.ndarray, dict]:
    """Point-to-plane ICP of `src_pts` onto `dst_pts` (plan §9.7): plain least
    squares to converge, then Tukey-weighted. Returns the 4x4 taking src to
    dst. Both are subsampled at random rather than voxel-averaged (which
    smears points across edges), and the target normals are the reference
    mesh's own face normals rather than estimates (which round edges off)."""
    import open3d as o3d

    q = p["icp"]
    rng = np.random.default_rng(0)
    src_pts = src_pts[rng.choice(len(src_pts), min(len(src_pts), q["points"]), replace=False)]
    dst_pts = dst_pts[rng.choice(len(dst_pts), min(len(dst_pts), q["points"]), replace=False)]
    near = ref.scene.raycaster.compute_closest_points(o3d.core.Tensor(dst_pts.astype(np.float32)))
    src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(src_pts))
    dst = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(dst_pts))
    dst.normals = o3d.utility.Vector3dVector(near["primitive_normals"].numpy().astype(float))
    reg = o3d.pipelines.registration
    crit = reg.ICPConvergenceCriteria(max_iteration=q["iters"])
    t = np.eye(4)
    out = None
    for corr, est in ((q["coarse_corr"], reg.TransformationEstimationPointToPlane()),
                      (q["max_corr"], reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=q["tukey"])))):
        out = reg.registration_icp(src, dst, corr, t, est, crit)
        t = np.asarray(out.transformation).copy()
    return t, {"fitness": float(out.fitness), "rmse_mm": float(out.inlier_rmse)}


def _in_plane(t: np.ndarray) -> tuple[np.ndarray, float]:
    """Split off the part of `t` that moves things within the XY plane:
    returns (the 4x4 with only Z, roll and pitch left, the yaw removed in deg)."""
    yaw = np.arctan2(t[1, 0], t[0, 0])
    keep = rotation_z(-np.degrees(yaw)) @ t
    keep[:2, 3] = 0.0
    return keep, float(np.degrees(yaw))


def datum_strips(cloud_scan: "Cloud", cloud_gold: "Cloud", cfg: Config, p: dict, band: float,
                 spacing: float) -> list[dict]:
    """The housing's top strips, fitted by outline on scan and golden alike:
    the two end-plate tops and the two side-plate tops, each over its middle
    span only. All four are flush at z_wall_top, and the side plates run the
    module's full length past the end plates, so an end-plate outline taken
    out to its corners runs on along the side plates to the edge of the search
    window -- an edge set by the window, fixed in the module frame, not by the
    part. That bias hid in 1.4 (an end plate read 0.045 of a 0.15 mm shift).

    Each strip counts across its own width only, where both edges are real
    drops: an end plate fixes X, a side plate Y, and either one's long edge
    yaw if `icp.datum_min_edge` or more of it is in view. A strip whose golden
    outline falls short of its design width by more than
    `fit.outline_slack` + 3 samples was cut by the view and counts for
    nothing."""
    end_x = cfg.module_l / 2 - cfg.end_plate_t / 2
    side_y = cfg.module_w / 2 - cfg.side_plate_t / 2
    keep_clear = 3.0                        # mm short of the corners, past the search margin
    strips = [("end", sign, (sign * end_x, 0.0), (cfg.end_plate_t / 2, cfg.end_plate_w / 2 - keep_clear))
              for sign in (-1, 1)]
    strips += [("side", sign, (0.0, sign * side_y),
                (cfg.module_l / 2 - cfg.end_plate_t - keep_clear, cfg.side_plate_t / 2)) for sign in (-1, 1)]
    slack = p["fit"]["outline_slack"] + 3 * spacing
    out = []
    for kind, sign, centre, half in strips:
        across = 0 if kind == "end" else 1          # module axis the strip's width runs along
        shrink = np.array([min(1.0, half[0] / 3), min(5.0, half[1] / 3)]) if kind == "end" else \
            np.array([5.0, min(1.0, half[1] / 3)])
        args = (centre, half, cfg.z_wall_top, (-1.0, 1.0), p, band)
        g = measure_face(cloud_gold, *args, shrink=shrink)
        sc = measure_face(cloud_scan, *args, shrink=shrink) if g else None
        if g is None or sc is None or sc.n < p["fit"]["min_fraction"] * g.n:
            continue
        square = abs(np.cos(np.radians(g.rect.yaw_deg))) > 0.7
        ext = (g.rect.ex, g.rect.ey) if square else (g.rect.ey, g.rect.ex)
        width = 2 * half[across]
        gc = np.array([g.rect.cx, g.rect.cy])
        line = _strip_offset(cloud_gold.src[g.idx, :2], cloud_scan.src[sc.idx, :2], across, gc[1 - across],
                             p["icp"]["datum_bin"], spacing)
        if line is None:
            continue
        offset, slope = line
        # A side strip's centreline is y(x), so its offset's slope is the yaw
        # between them; an end strip's is x(y), and a yaw tilts it the other way.
        dyaw = np.degrees(slope if kind == "side" else -slope)
        out.append({"strip": f"{kind} {sign:+d}", "axis": across, "at": gc.tolist(),
                    "offset": float(offset), "whole": bool(ext[across] >= width - slack),
                    "dyaw": float(dyaw), "edge": float(ext[1 - across]), "spacing": float(spacing)})
    return out


def _strip_offset(gold_xy: np.ndarray, scan_xy: np.ndarray, across: int, at: float, bin_w: float,
                  spacing: float) -> tuple[float, float] | None:
    """Offset (golden - scan) of a strip's centreline across its width, at
    `at` along it, and that offset's slope along it -- from a robust line
    through per-bin centrelines, not one outline round the whole strip.

    A side plate is 1.5 mm sheet, and a cell set off its pitch bows it out
    locally (settle.py). One rectangle round the strip moved with that bow:
    the datum then took up part of the cell's own shift and the cell read
    short (0.20 read 0.14 through a part view, 11 % lost in the fixed pass).
    Bins along the strip each give a centreline (midway between the two edges
    seen in the bin); a line is fitted to golden - scan, and bins further than
    3 robust sigma from it (at least a sample spacing) are dropped and the
    line refitted. A bow over a minority of the strip drops out. Returns None
    if fewer than 5 bins are seen on both."""
    along = 1 - across

    def centres(xy):
        k = np.floor(xy[:, along] / bin_w).astype(np.int64)
        order = np.argsort(k, kind="stable")
        k, v = k[order], xy[order, across]
        keys, start, count = np.unique(k, return_index=True, return_counts=True)
        lo = np.minimum.reduceat(v, start)
        hi = np.maximum.reduceat(v, start)
        ok = count >= 4
        return dict(zip(keys[ok].tolist(), ((lo + hi) / 2)[ok].tolist()))

    g, s = centres(gold_xy), centres(scan_xy)
    common = sorted(set(g) & set(s))
    if len(common) < 5:
        return None
    u = (np.array(common, float) + 0.5) * bin_w - at
    d = np.array([g[k] - s[k] for k in common])
    keep = np.ones(len(u), bool)
    for _ in range(3):
        a = np.c_[np.ones(keep.sum()), u[keep]]
        coef = np.linalg.lstsq(a, d[keep], rcond=None)[0]
        r = d - (coef[0] + coef[1] * u)
        mad = 1.4826 * np.median(np.abs(r[keep] - np.median(r[keep])))
        new = np.abs(r) <= max(3 * mad, spacing)
        if new.sum() < 5 or (new == keep).all():
            break
        keep = new
    return float(coef[0]), float(coef[1])


def solve_datum(obs: list[dict], p: dict) -> tuple[np.ndarray, list[str]] | None:
    """In-plane correction (X, Y, yaw) mapping scan to reference, by least
    squares over whole-strip observations, small-angle: an end strip at
    (x, y) says  offset_x = tx - yaw * y,  a side strip  offset_y = ty + yaw * x,
    and a long edge  dyaw = yaw  (weighted by its length, so in mm like the
    rest). Axes no observation constrains stay at the fixture pose.
    Returns (4x4, axes fixed) or None if nothing is fixed."""
    rows, rhs = [], []
    for o in obs:
        if not o["whole"]:
            continue
        x, y = o["at"]
        rows.append([1.0, 0.0, -y] if o["axis"] == 0 else [0.0, 1.0, x])
        rhs.append(o["offset"])
    for o in obs:
        if o["edge"] >= p["icp"]["datum_min_edge"]:
            rows.append([0.0, 0.0, o["edge"]])
            rhs.append(np.radians(o["dyaw"]) * o["edge"])
    if not rows:
        return None
    a, b = np.array(rows), np.array(rhs)
    names = ["x", "y", "yaw"]
    use = [i for i in range(3) if np.abs(a[:, i]).max() > 1e-9]
    sol = np.zeros(3)
    sol[use] = np.linalg.lstsq(a[:, use], b, rcond=None)[0]
    tx, ty, th = sol
    out = np.eye(4)
    out[:2, :2] = [[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]
    out[:2, 3] = tx, ty
    return out, [names[i] for i in use]


def joint_datum(per_capture: dict, p: dict) -> tuple[np.ndarray, list[str]] | None:
    """One in-plane correction for the whole load, from the housing strips
    any of its captures saw whole: for each strip, the finest such capture,
    then the one that saw most of it.
    Every capture of one load sees the same module in the same fixture
    placement, and the station knows where each capture was taken from, so
    the module is registered once -- as the fixed pass registers itself from
    all four strips in one sweep -- and views that see no strip whole (most
    arm views) inherit it. Assumes exact capture poses: robot pose error is
    not modelled yet. Returns (4x4, the captures used), or None unless X, Y
    and yaw are all fixed."""
    best: dict = {}
    for name, obs in per_capture.items():
        for o in obs:
            # Finest sampling first, then the most of the strip in view: a
            # square-on view of the whole strip, not a corner of it seen at 60 deg.
            if o["whole"] and (o["strip"] not in best or (o["spacing"], -o["edge"]) <
                               (best[o["strip"]][1]["spacing"], -best[o["strip"]][1]["edge"])):
                best[o["strip"]] = (name, o)
    d = solve_datum([o for _, o in best.values()], p)
    if d is None or set(d[1]) != {"x", "y", "yaw"}:
        return None
    return d[0], sorted({n for n, _ in best.values()})


def capable(gold: Golden, tol: float, p: dict) -> bool:
    """Can this capture place an edge well enough to judge a position against
    tolerance `tol`? Plan §3.7.3's rule of thumb: sample spacing <= tol / 2
    (`capability.spacing_per_tol`). Plane fits (heights, tilt, gap, carrier)
    average thousands of points and are not gated by this."""
    return gold.spacing <= p["capability"]["spacing_per_tol"] * tol + 1e-9


def housing_fit(cap: Capture, gold: Golden, ref: "Reference", p: dict) -> tuple[np.ndarray | None, dict]:
    """Z, roll and pitch from ICP on the housing only -- the rays the golden
    capture says land on it -- and this capture's housing strips
    (`datum_strips`), measured after that correction. Whole-module ICP was tried first and fails twice
    over: the dominant surface drags it (a carrier seated 0.3 mm high pulled
    the whole capture down 0.3 mm, hiding the defect and making every cap read
    0.3 mm low), and seen from above it barely constrains X, Y and yaw (a
    0.15 mm fixture shift came back 0.10 mm wrong). Returns (None, info) if
    the capture sees too little housing."""
    q = p["icp"]
    names = [i for i, n in enumerate(gold.material_names) if n in DATUM_MATERIALS]
    _, si, gi = np.intersect1d(cap.index, gold.index, assume_unique=True, return_indices=True)
    on = np.isin(gold.material[gi], names)
    info: dict = {"datum_points": int(on.sum()), "spacing_mm": gold.spacing}
    if on.sum() < q["min_datum_points"]:
        return None, info
    t3, fit = icp(cap.points[si[on]], gold.points[gi[on]], ref, p)
    info.update(fit)
    t, info["icp_yaw_dropped_deg"] = _in_plane(t3)
    # An outline places an edge to about half the sample spacing; a capture
    # too coarse for the tightest position tolerance (§3.7.3) registers no
    # in-plane axis from outlines.
    info["strips"] = []
    if capable(gold, min(p["cell"]["pos_tol"], p["busbar"]["pos_tol"]), p):
        moved = cap.transformed(t)
        info["strips"] = datum_strips(Cloud(moved.points), Cloud(gold.points), ref.cfg, p, p["fit"]["band"],
                                      gold.spacing)
    return t, info


def align(cap: Capture, gold: Golden | None, ref: "Reference", p: dict,
          fit: tuple | None = None, joint: tuple | None = None) -> tuple[np.ndarray, dict]:
    """Fine alignment of one capture to the module's datum: Z, roll and pitch
    from its own housing (`housing_fit`, or `fit` if already done); X, Y and
    yaw from the load's joint registration if there is one (`joint_datum`),
    else from this capture's own whole housing strips (`solve_datum`).

    Axes nothing registered keep the fixture (or robot) pose: absolute
    positions inherit that pose's error, relative ones -- height between
    adjacent terminals, busbar to pad, seam to busbar -- do not; the residual
    pass allows the fixture's tolerance there. Returns identity when the
    correction would move no point of the module more than `identity_tol`."""
    q = p["icp"]
    t, info = fit if fit is not None else housing_fit(cap, gold, ref, p)
    info = dict(info)
    strips = info.pop("strips", [])
    if t is None:
        info["datum"] = "housing not seen: fixture pose kept"
        info["registered_axes"] = []
        return np.eye(4), info
    own = solve_datum(strips, p)
    if joint is not None:
        t = joint[0] @ t
        info["datum"] = f"housing tops + strip outlines, registered once per load (from {', '.join(joint[1])})"
        info["registered_axes"] = ["x", "y", "yaw"]
    elif own is not None:
        t = own[0] @ t
        full = set(own[1]) == {"x", "y", "yaw"}
        info["datum"] = "housing tops + strip outlines" if full else \
            f"housing tops + strip outlines fixing only {', '.join(own[1])}; fixture pose kept on the rest"
        info["registered_axes"] = own[1]
    else:
        info["datum"] = "housing tops only: fixture pose kept in X, Y, yaw" if strips or \
            capable(gold, min(p["cell"]["pos_tol"], p["busbar"]["pos_tol"]), p) else \
            f"housing tops only: {info['spacing_mm']:.2f} mm sampling is too coarse for the strip outlines, " \
            "fixture pose kept in X, Y, yaw"
        info["registered_axes"] = []
    angle = np.arccos(np.clip((np.trace(t[:3, :3]) - 1) / 2, -1, 1))
    info.update(translation_mm=t[:3, 3].tolist(), rotation_mrad=float(angle * 1e3), transform=t.tolist())
    if np.linalg.norm(t[:3, 3]) + angle * ref.cfg.module_l / 2 < q["identity_tol"]:
        info["kept_fixture_pose"] = True
        return np.eye(4), info
    return t, info


# -- point access ----------------------------------------------------------------

class Cloud:
    """Points sorted by X so an axis-aligned box crop is a binary search plus
    a mask over the slice, not a pass over millions of points."""

    def __init__(self, points: np.ndarray):
        self.src = points                   # what `box`'s indices (and Face.idx) index into
        self.order = np.argsort(points[:, 0], kind="stable")
        self.p = points[self.order]
        self.x = self.p[:, 0]

    def box(self, lo, hi) -> tuple[np.ndarray, np.ndarray]:
        """Points with lo <= p <= hi, and their indices into the original array."""
        a, b = np.searchsorted(self.x, [lo[0], hi[0]], side="left")
        q = self.p[a:b]
        m = (q[:, 1] >= lo[1]) & (q[:, 1] <= hi[1]) & (q[:, 2] >= lo[2]) & (q[:, 2] <= hi[2])
        return q[m], self.order[a:b][m]


def _outside(pts: np.ndarray, boxes) -> np.ndarray:
    keep = np.ones(len(pts), bool)
    for (x0, y0, x1, y1) in boxes:
        keep &= ~((pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1))
    return keep


@dataclass
class Face:
    """A fitted top face: its plane, its outline (if asked for) and which
    points of the capture lie on it."""

    plane: Plane
    rect: Rect | None
    n: int
    idx: np.ndarray


def measure_face(cloud: Cloud, centre, half, z_nom: float, window, p: dict, band: float,
                 exclude=(), outline: bool = True, shrink=None) -> Face | None:
    """Fit the near-horizontal face nominally centred at `centre` with
    half-extents `half` at height `z_nom`.

    Seed a plane from the core (the face shrunk by `shrink`, so a displaced
    part still covers it) by RANSAC, refine it by trimmed least squares, then
    take every point within `band` of it in the face grown by `margin` --
    those points are the face, and their outline is the part's outline.
    `exclude` removes XY boxes first (a busbar lying over a pad)."""
    f = p["fit"]
    shrink = f["shrink"] if shrink is None else shrink
    sx, sy = np.asarray(half, float) - np.broadcast_to(np.asarray(shrink, float), 2)
    cx, cy = centre
    zl, zh = z_nom + window[0], z_nom + window[1]
    core, _ = cloud.box((cx - sx, cy - sy, zl), (cx + sx, cy + sy, zh))
    core = core[_outside(core, exclude)]
    if len(core) < f["min_points"]:
        return None
    seed = ransac_plane(core, tol=band)
    if seed.sum() < f["min_points"]:
        return None
    plane, _ = fit_plane(core[seed], floor=band / 2)
    mx, my = np.asarray(half) + f["margin"]
    pts, idx = cloud.box((cx - mx, cy - my, zl), (cx + mx, cy + my, zh))
    keep = _outside(pts, exclude)
    pts, idx = pts[keep], idx[keep]
    on = np.abs(plane.residual(pts)) <= max(band, 4 * plane.rms)
    if on.sum() < f["min_points"]:
        return None
    rect = min_area_rect(pts[on, :2]) if outline else None
    return Face(plane, rect, int(on.sum()), idx[on])


# -- per-feature measurement -------------------------------------------------------

@dataclass
class Pair:
    """One feature measured the same way on the scan and on the golden capture."""

    scan: object
    gold: object
    support: int   # golden points on the feature: how well this capture saw it, by design


def _face_ok(scan: Face | None, gold: Face | None, p: dict) -> bool:
    return scan is not None and scan.n >= p["fit"]["min_fraction"] * gold.n


def _trim() -> None:
    """Hand freed memory back to the OS. glibc keeps large freed blocks, so a
    detection over many big captures crept up ~0.5 GB per sweep without it."""
    try:
        import ctypes

        ctypes.CDLL("libc.so.6").malloc_trim(0)
    except OSError:
        pass


@dataclass
class _Spacing:
    """What a released Measurer keeps of its golden capture."""

    spacing: float


class Measurer:
    """Every feature this capture can see, measured on scan and golden alike."""

    def __init__(self, ref: Reference, cap: Capture, gold: Golden, p: dict, noise: float):
        self.ref, self.cfg, self.p = ref, ref.cfg, p
        self.cap, self.gold = cap, gold
        self.scan_cloud, self.gold_cloud = Cloud(cap.points), Cloud(gold.points)
        self.band = max(p["fit"]["band"], 4 * noise)
        self.terminals: dict = {}
        self.busbars: dict = {}
        self.pads: dict = {}
        self.seams: dict = {}
        self.polarity: dict = {}
        self.caps: dict = {}
        self.carrier: Pair | None = None

    def release(self) -> None:
        """Drop what only measuring needed: the two sorted point copies and the
        golden capture (its sample spacing stays; the residual pass recasts).
        The fitted features stay. A full-length Keyence sweep is ~15 M points,
        and holding every capture's copies at once did not fit in 16 GB."""
        self.scan_cloud = self.gold_cloud = None
        self.gold = _Spacing(self.gold.spacing)

    def both(self, fn, *args, **kw) -> Pair | None:
        g = fn(self.gold_cloud, *args, **kw)
        if g is None:
            return None
        return Pair(fn(self.scan_cloud, *args, **kw), g, g.n if hasattr(g, "n") else 0)

    # Terminals and cells ---------------------------------------------------

    def run_terminals(self) -> None:
        cfg = self.cfg
        half = (cfg.terminal_d / 2, cfg.terminal_w / 2)
        for t in self.ref.labels.terminals:
            pair = self.both(measure_face, t.centre[:2], half, t.top_z, (-0.75, 1.75), self.p, self.band)
            if pair is not None:
                self.terminals[(t.cell_index, t.polarity)] = pair

    def axes(self, idx: np.ndarray) -> np.ndarray:
        """Unit measuring direction (sensor into scene) of capture points `idx`."""
        cap = self.cap
        if len(cap.poses) > 1:
            return cap.poses[cap.index[idx] // cap.grid[1]][:, :3, 2]
        d = cap.points[idx] - cap.poses[0][:3, 3]
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def run_caps(self) -> None:
        """Points on each cell's cap, scan against golden: the evidence that a
        cell is there at all when a sweep covered only one of its terminals."""
        cfg = self.cfg
        for k in range(cfg.n_cells):
            cx = cfg.cell_centre_x(k)
            lo = (cx - cfg.cell_t / 2 + 1.0, -cfg.cell_w / 2 + 1.0, cfg.z_cell_top - 0.3)
            hi = (cx + cfg.cell_t / 2 - 1.0, cfg.cell_w / 2 - 1.0, cfg.z_cell_top + 0.3)
            n_gold = len(self.gold_cloud.box(lo, hi)[0])
            if n_gold >= self.p["fit"]["min_points"]:
                self.caps[k] = (len(self.scan_cloud.box(lo, hi)[0]), n_gold)

    def run_polarity(self) -> None:
        """Score the cap of every cell against two CAD hypotheses: as designed,
        and turned 180 degrees. The terminal pads land in the same places
        either way; the fill plug beside the negative pad does not.

        Each point is compared along its own measuring ray, not by distance
        to the nearest face: a cap point where a hypothesis puts the plug lies
        on that plug's bottom face, so nearest-face distance would charge
        nothing for a plug that is not there. Along the ray, the hypothesis
        says the surface should be 5 mm nearer, and that is charged."""
        import open3d as o3d

        cfg, q = self.cfg, self.p["polarity"]
        parts = self.ref.parts
        for k in range(cfg.n_cells):
            if not any(key[0] == k for key in self.terminals):
                continue
            cx = cfg.cell_centre_x(k)
            pts, idx = self.scan_cloud.box(
                (cx - cfg.cell_t / 2 + 0.5, -cfg.cell_w / 2 + 0.5, cfg.z_cell_top - 0.5),
                (cx + cfg.cell_t / 2 - 0.5, cfg.cell_w / 2 - 0.5, cfg.z_terminal_top + 1.0))
            if len(pts) < self.p["fit"]["min_points"]:
                continue
            if len(pts) > q["max_points"]:
                pick = np.random.default_rng(k).choice(len(pts), q["max_points"], replace=False)
                pts, idx = pts[pick], idx[pick]
            axis = self.axes(idx)
            back = 20.0
            rays = np.hstack([pts - axis * back, axis]).astype(np.float32)
            scores = []
            for turn in (0.0, 180.0):
                t = rotation_z(turn, about=(cx, 0.0, 0.0)) @ cell_transform(cfg, k)
                meshes = [parts["cell_can"].copy().apply_transform(t)]
                for pol in ("pos", "neg"):
                    meshes.append(parts[f"terminal_{pol}"].copy().apply_transform(t @ terminal_local(cfg, pol)))
                m = trimesh.util.concatenate(meshes)
                scene = o3d.t.geometry.RaycastingScene()
                scene.add_triangles(o3d.core.Tensor(m.vertices.astype(np.float32)),
                                    o3d.core.Tensor(m.faces.astype(np.uint32)))
                hit = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
                d = np.abs(np.where(np.isfinite(hit), hit, np.inf) - back)
                scores.append(float(np.minimum(d, q["truncate"]).sum()))
            self.polarity[k] = {"as_designed": scores[0], "turned": scores[1], "points": len(pts)}

    # Busbars, pads, gap, carrier ------------------------------------------

    def busbar_box(self, b, grow: float) -> tuple:
        length = busbar_length(self.cfg, b.kind)
        x, y = b.centre[:2]
        return (x - length / 2 - grow, y - self.cfg.busbar_w / 2 - grow,
                x + length / 2 + grow, y + self.cfg.busbar_w / 2 + grow)

    def run_busbars(self) -> None:
        cfg = self.cfg
        for b in self.ref.labels.busbars:
            half = (busbar_length(cfg, b.kind) / 2, cfg.busbar_w / 2)
            top = b.underside_z + cfg.busbar_t
            pair = self.both(measure_face, b.centre[:2], half, top, (-1.0, 3.0), self.p, self.band,
                             shrink=np.array([2.5, 1.0]))
            if pair is not None:
                self.busbars[b.index] = pair

    def run_pads(self) -> None:
        """Each terminal pad's top from the strips the busbar leaves exposed
        either side of it -- so the gap is measured inside one capture."""
        cfg = self.cfg
        half = (cfg.terminal_d / 2, cfg.terminal_w / 2)
        for b in self.ref.labels.busbars:
            if b.index not in self.busbars:
                continue
            cover = [self.busbar_box(b, 0.75)]
            for k, pol in b.terminals:
                t = next(t for t in self.ref.labels.terminals if t.cell_index == k and t.polarity == pol)
                pair = self.both(measure_face, t.centre[:2], half, t.top_z, (-0.75, 0.6), self.p,
                                 self.band, exclude=cover, outline=False, shrink=np.array([1.0, 0.5]))
                if pair is not None:
                    self.pads[(k, pol)] = pair

    def run_carrier(self) -> None:
        cfg = self.cfg
        cover = [self.busbar_box(b, 2.0) for b in self.ref.labels.busbars]
        half = (cfg.carrier_l / 2, cfg.carrier_w / 2)
        pair = self.both(measure_face, (0.0, 0.0), half, cfg.z_carrier_top, (-0.6, 6.0), self.p,
                         self.band, exclude=cover, outline=False, shrink=np.array([5.0, 5.0]))
        self.carrier = pair

    # Weld seams -------------------------------------------------------------

    def run_seams(self) -> None:
        for i, s in enumerate(self.ref.labels.seams):
            bus = self.busbars.get(s.busbar_index)
            if bus is None or bus.scan is None:
                continue
            g = measure_seam(self.gold_cloud, s, bus.gold.plane, bus.gold.rect, np.eye(2), np.zeros(2),
                             self.cfg, self.p)
            if g is None:
                continue
            r, t = _rect_motion(bus.gold.rect, bus.scan.rect)
            m = measure_seam(self.scan_cloud, s, bus.scan.plane, bus.scan.rect, r, t, self.cfg, self.p)
            self.seams[i] = Pair(m, g, g["n"])


def _in_rect(xy: np.ndarray, rect: Rect, grow: float = 0.0) -> np.ndarray:
    c, s = np.cos(np.radians(rect.yaw_deg)), np.sin(np.radians(rect.yaw_deg))
    d = xy - [rect.cx, rect.cy]
    return (np.abs(d @ [c, s]) <= rect.ex / 2 + grow) & (np.abs(d @ [-s, c]) <= rect.ey / 2 + grow)


def _gap(bus_s: Plane, bus_g: Plane, pad_s: Plane, pad_g: Plane, x: float, y: float, cfg: Config) -> float:
    """Busbar underside to pad top at (x, y), scan minus golden. The underside
    is the fitted top less the busbar's design thickness."""
    gs = (bus_s.z(x, y) - cfg.busbar_t) - pad_s.z(x, y)
    gg = (bus_g.z(x, y) - cfg.busbar_t) - pad_g.z(x, y)
    return float(gs - gg)


def _rect_motion(gold: Rect, scan: Rect) -> tuple[np.ndarray, np.ndarray]:
    """The 2D rigid motion taking the golden outline onto the scanned one."""
    th = np.radians(scan.yaw_deg - gold.yaw_deg)
    r = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return r, np.array([scan.cx, scan.cy]) - r @ np.array([gold.cx, gold.cy])


def measure_seam(cloud: Cloud, seam, plane: Plane, outline: Rect, r: np.ndarray, t: np.ndarray,
                 cfg: Config, p: dict):
    """The bead along one stitch, as heights off the busbar's fitted plane.

    The stitch frame (s along, t across) is the nominal one carried by the
    busbar's measured motion (r, t), since a weld is laid where the busbar
    is. Sampled in bins of `bin` along s; per bin: whether the surface was
    melted (|h| > bead_min_h), the bead's centre and width across, and its
    height. Returned as arrays plus whole-seam numbers.

    Only points on the busbar count (inside its measured `outline`): at an end
    terminal the strip would otherwise run off the busbar's end onto the pad
    1.5 mm below. "Melted" means changed near the surface, |h| above
    `bead_min_h` but not more than half the busbar thickness down; deeper
    than that, inside the stitch, is a hole."""
    q = p["seam"]
    a0, b0 = np.array(seam.start[:2]), np.array(seam.end[:2])
    c = r @ ((a0 + b0) / 2) + t
    u = r @ ((b0 - a0) / np.linalg.norm(b0 - a0))
    v = np.array([-u[1], u[0]])
    length = float(np.linalg.norm(b0 - a0))
    reach = length / 2 + q["along_margin"]
    ext = np.abs(u) * reach + np.abs(v) * q["across"]
    z = plane.z(c[0], c[1])
    pts, _ = cloud.box((c[0] - ext[0], c[1] - ext[1], z - 2.5), (c[0] + ext[0], c[1] + ext[1], z + 1.5))
    if len(pts) < p["fit"]["min_points"]:
        return None
    d = pts[:, :2] - c
    s, w = d @ u, d @ v
    inside = (np.abs(s) <= reach) & (np.abs(w) <= q["across"]) & _in_rect(pts[:, :2], outline, -0.3)
    s, w, pts = s[inside], w[inside], pts[inside]
    if len(pts) < p["fit"]["min_points"]:
        return None
    h = plane.residual(pts)
    floor = -cfg.busbar_t / 2
    melted = (np.abs(h) > q["bead_min_h"]) & (h > floor)
    edges = np.arange(-reach, reach + q["bin"], q["bin"])
    nb = len(edges) - 1
    which = np.clip(np.digitize(s, edges) - 1, 0, nb - 1)
    has = np.zeros(nb, bool)
    centre = np.full(nb, np.nan)
    width = np.full(nb, np.nan)
    crown = np.full(nb, np.nan)
    area = np.full(nb, np.nan)
    grid = np.arange(-q["across"], q["across"] + 1e-9, 0.02)
    toe_h = []
    for b in np.unique(which[melted]):
        sel = melted & (which == b)
        if sel.sum() < 3:
            continue
        has[b] = True
        centre[b] = float(np.median(w[sel]))
        # The toe: a band just outside the design melt width, either side of
        # where this bin's bead actually is. weld.py cuts undercut there.
        off = np.abs(w - centre[b])
        band = (which == b) & (off >= seam.width / 2 + 0.03) & (off <= seam.width / 2 + 0.3) & (h > floor)
        toe_h.append(h[band])
        lo, hi = np.percentile(w[sel], [2, 98])
        width[b] = float(hi - lo)
        crown[b] = float(np.median(h[sel]))
        # Cross-section above the busbar, every point of the bin, on a fixed
        # grid across (missing returns interpolated over): the flanks, where
        # grazing returns drop out, carry little of it.
        allb = (which == b) & (h > floor)
        o = np.argsort(w[allb])
        area[b] = float(np.trapz(np.interp(grid, w[allb][o], np.maximum(h[allb][o], 0.0), left=0.0, right=0.0),
                                grid))
    mids = (edges[:-1] + edges[1:]) / 2
    nominal = np.abs(mids) <= length / 2
    run = best = 0
    for flag in has:
        run = run + 1 if flag else 0
        best = max(best, run)
    span = np.abs(s) <= length / 2
    toe = np.concatenate(toe_h) if toe_h else np.zeros(0)
    return {
        "n": int(len(pts)),
        "mids": mids, "has": has, "centre": centre, "width": width, "crown": crown, "area": area,
        "coverage": float(has[nominal].mean()) if nominal.any() else 0.0,
        "length": best * q["bin"],
        "undercut": float(-np.percentile(toe, 2)) if len(toe) > 20 else 0.0,
        "deep": int((span & (h <= floor)).sum()),
        "nominal_length": length,
        "centre_xy": c.tolist(), "z": float(z),
        "u": u.tolist(), "v": v.tolist(),
    }


# -- residual segmentation ---------------------------------------------------------

@dataclass
class Region:
    kind: str                # "raised" or "missing"
    n: int
    area: float              # mm^2
    peak: float              # mm, largest |r| (nan where only missing data)
    material: str            # what the golden capture says the surface should be
    missing_fraction: float  # of the region's rays, how many returned nothing
    location: tuple
    bounds: tuple
    scan_idx: np.ndarray = field(repr=False, default=None)
    gold_pts: np.ndarray = field(repr=False, default=None)
    on_step: float = 0.0     # fraction of its rays near a discontinuity of the golden capture


def step_band(gold: Golden, grid: tuple, misreg: float, q: dict):
    """Rays (on the sensor grid) within reach of a discontinuity of the golden
    capture: a depth step between neighbouring rays along the measuring
    direction larger than step_slope x spacing + step_min (a surface tilted
    up to ~72 deg changes by less), or a shadow / silhouette boundary (one
    neighbour returned a point, the other did not). Widened by
    1 + ceil(misreg / spacing) rays: a capture registered to within `misreg`
    moves every such edge by up to that much, in any view, and the residual
    there is misregistration, not a defect. The outline band is the special
    case of this for parts detection measured."""
    from scipy import ndimage

    rows, cols = grid
    present = np.zeros(rows * cols, bool)
    present[gold.index] = True
    depth = np.full(rows * cols, np.nan)
    depth[gold.index] = (gold.points * gold.axis).sum(axis=1)
    present, depth = present.reshape(rows, cols), depth.reshape(rows, cols)
    thr = q["step_slope"] * gold.spacing + q["step_min"]
    edge = np.zeros((rows, cols), bool)
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = (slice(0, rows - dr), slice(max(0, -dc), cols - max(0, dc)))
        b = (slice(dr, rows), slice(max(0, dc), cols - max(0, -dc) if dc < 0 else cols))
        pa, pb = present[a], present[b]
        jump = (pa != pb) | (pa & pb & (np.abs(depth[a] - depth[b]) > thr))
        edge[a] |= jump & pa
        edge[b] |= jump & pb
    k = 1 + int(np.ceil(misreg / max(gold.spacing, 1e-6)))
    return ndimage.binary_dilation(edge, np.ones((3, 3), bool), iterations=k).ravel()


def sideways_gain(gold: Golden, grid: tuple, rays: np.ndarray, cap: float = 20.0) -> np.ndarray:
    """|n_h| / |n . a| for golden rays `rays` (indices into gold's arrays): how
    much a sideways shift of the surface under each ray changes the depth it
    reads. The normals come from the golden points of the neighbouring grid
    cells, one per quadrant (right/left x down/up), and the steepest counts: a
    ray at a crease (a wall's foot) lands on either face once the part moves,
    and averaging the two halved the bound there. A ray with no quadrant whole
    gets 0, leaving the other bounds. Capped where the ray grazes the surface
    (n . a -> 0)."""
    rows, cols = grid
    idx = gold.index
    order = None
    if len(idx) > 1 and np.any(np.diff(idx) <= 0):
        order = np.argsort(idx, kind="stable")
        idx = idx[order]
    cell = gold.index[rays]
    r0, c0 = cell // cols, cell % cols

    def neighbour(dr, dc):
        rr, cc = r0 + dr, c0 + dc
        inside = (rr >= 0) & (rr < rows) & (cc >= 0) & (cc < cols)
        want = np.where(inside, rr * cols + cc, -1)
        k = np.clip(np.searchsorted(idx, want), 0, len(idx) - 1)
        hit = inside & (idx[k] == want)
        if order is not None:
            k = order[k]
        return np.where(hit, k, -1)

    here = gold.points[rays]
    axis = gold.axis[rays]
    gain = np.zeros(len(rays))
    side = {}
    for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        k = neighbour(dr, dc)
        side[dr, dc] = ((gold.points[np.where(k >= 0, k, rays)] - here) * (dr + dc), k >= 0)
    for su in ((0, 1), (0, -1)):
        for sv in ((1, 0), (-1, 0)):
            (tu, ok_u), (tv, ok_v) = side[su], side[sv]
            n = np.cross(tu, tv)
            norm = np.linalg.norm(n, axis=1)
            ok = ok_u & ok_v & (norm > 0)
            n = n / np.where(ok, norm, 1.0)[:, None]
            facing = np.abs((n * axis).sum(axis=1))
            g = np.hypot(n[:, 0], n[:, 1]) / np.maximum(facing, 1.0 / cap)
            gain = np.maximum(gain, np.where(ok, g, 0.0))
    return gain


def residuals(cap: Capture, gold: Golden, p: dict, regions: bool = True,
              misreg: float | None = None) -> tuple[list[Region], dict]:
    """Ray-by-ray residual against the golden capture and its connected
    regions on the sensor's own grid.

    r = (golden - scan) . axis, positive where the surface came back nearer
    the sensor than designed (something raised or added). Noise is estimated
    robustly per nominal material, sigma_m = 1.4826 MAD, so bright copper and
    matte plastic each get their own threshold."""
    from scipy import ndimage

    q = p["residual"]
    common, si, gi = np.intersect1d(cap.index, gold.index, assume_unique=True, return_indices=True)
    r = ((gold.points[gi] - cap.points[si]) * gold.axis[gi]).sum(axis=1)
    mat = gold.material[gi]
    tau = np.zeros(len(r))
    sigma = {}
    for m in np.unique(mat):
        sel = mat == m
        mad = float(np.median(np.abs(r[sel] - np.median(r[sel]))))
        sigma[gold.material_names[m]] = 1.4826 * mad
        tau[sel] = max(q["k_sigma"] * 1.4826 * mad, q["floor"])
    noise = float(1.4826 * np.median(np.abs(r - np.median(r)))) if len(r) else 0.0
    if not regions:
        return [], {"rays_compared": int(len(r)), "noise_mm": noise, "sigma_by_material_mm": sigma}
    if misreg is not None:
        # X, Y and yaw come from the housing datum, good to about `misreg`; Z, roll
        # and pitch each capture registers from its own housing. A ray measuring
        # along a horizontal direction reads that X/Y error in full, so it cannot be
        # held to its noise alone. Side-on wall sweeps read the whole wall 7-13 um
        # off against a 5 um threshold: 3,592 false regions (12 Nov 2026). Looking
        # straight down this adds nothing.
        horiz = np.hypot(gold.axis[gi, 0], gold.axis[gi, 1]).astype(float)
        tau = np.maximum(tau, misreg * horiz)
        # The general form: a surface moved sideways by d reads d |n_h| / |n . a|
        # along the ray (n its normal). Zero on a level top, the term above on a
        # wall seen side-on, and several times d on a wall seen nearly edge-on from
        # above: a close-up snapshot read a cell cap's 2 mm recess walls ~0.08 mm
        # off from a 0.027 mm residual shift, 2 false regions (12 Nov 2026). Only
        # rays already over threshold need it.
        cand = np.flatnonzero(np.abs(r) > tau)
        if len(cand):
            tau[cand] = np.maximum(tau[cand], misreg * sideways_gain(gold, cap.grid, gi[cand]))

    rows, cols = cap.grid
    raised = np.zeros(rows * cols, bool)
    missing = np.zeros(rows * cols, bool)
    raised[common[r > tau]] = True
    missing[common[r < -tau]] = True
    lost = np.setdiff1d(gold.index, cap.index, assume_unique=True)
    missing[lost] = True

    # Per grid cell, where the evidence lives.
    gold_at = np.full(rows * cols, -1, np.int64)
    gold_at[gold.index] = np.arange(len(gold.index))
    scan_at = np.full(rows * cols, -1, np.int64)
    scan_at[cap.index] = np.arange(len(cap.index))
    r_at = np.full(rows * cols, np.nan)
    r_at[common] = r

    regions: list[Region] = []
    band = step_band(gold, cap.grid, misreg, q) if misreg is not None else None
    eight = np.ones((3, 3), bool)
    for kind, mask in (("raised", raised), ("missing", missing)):
        lab, n = ndimage.label(mask.reshape(rows, cols), structure=eight)
        if not n:
            continue
        flat = lab.ravel()
        cells = np.flatnonzero(flat)
        ids = flat[cells]
        order = np.argsort(ids, kind="stable")
        cells, ids = cells[order], ids[order]
        splits = np.flatnonzero(np.diff(ids)) + 1
        for group in np.split(cells, splits):
            g = gold_at[group]
            sidx = scan_at[group]
            area = float(gold.footprint[g[g >= 0]].sum())
            npts = len(group)
            if npts < q["min_points"]:
                continue
            # A ray or two wide along its whole length: a ray that grazed an
            # edge onto the other side of it, not something on the part.
            span = max(len(np.unique(group // cols)), len(np.unique(group % cols)))
            if span >= 3 * q["min_thickness"] and npts / span < q["min_thickness"]:
                continue
            gone = float((sidx < 0).mean())
            if area < (q["missing_min_area"] if kind == "missing" and gone > 0.5 else q["min_area"]):
                continue
            pts = [gold.points[g[g >= 0]], cap.points[sidx[sidx >= 0]]]
            pts = np.vstack([x for x in pts if len(x)])
            here = cap.points[sidx[sidx >= 0]] if kind == "raised" and (sidx >= 0).any() else gold.points[g[g >= 0]]
            if not len(here):
                continue
            mats = gold.material[g[g >= 0]]
            material = gold.material_names[int(np.bincount(mats).argmax())] if len(mats) else "none"
            rr = r_at[group]
            peak = float(np.nanmax(np.abs(rr))) if np.isfinite(rr).any() else float("nan")
            # Far deeper (or taller) than it is wide: a ray that crossed a
            # step edge onto the surface beside it, not a dent or a particle.
            if np.isfinite(peak) and peak > q["max_aspect"] * np.sqrt(max(area, 1e-12)):
                continue
            regions.append(Region(kind, npts, area, peak, material, gone,
                                  tuple(map(float, here.mean(axis=0))),
                                  (tuple(map(float, pts.min(axis=0))), tuple(map(float, pts.max(axis=0)))),
                                  sidx[sidx >= 0], gold.points[g[g >= 0]],
                                  float(band[group].mean()) if band is not None else 0.0))
    stats = {"rays_compared": int(len(r)), "noise_mm": noise,
             "sigma_by_material_mm": sigma, "missing_rays": int(len(lost)),
             "regions": len(regions)}
    return regions, stats


# -- the report -----------------------------------------------------------------

@dataclass
class Finding:
    cls: str
    magnitude: float
    location: tuple
    bounds: tuple
    capture: str
    feature: str
    measured: dict = field(default_factory=dict)


@dataclass
class DefectReport:
    stage: str
    findings: list
    measurements: dict
    uncovered: dict
    captures: list
    detect_hash: str
    timings_s: dict
    # What the station tells the line: NOK if anything was found; INCOMPLETE
    # if nothing was, but some feature was never seen (that is not a pass);
    # OK otherwise.
    result: str = ""

    def by_class(self) -> dict:
        out: dict = {}
        for f in self.findings:
            out.setdefault(f.cls, []).append(f)
        return out

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(asdict(self), indent=indent, default=_jsonable)


def _jsonable(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    raise TypeError(type(o))


def _box(*pts, grow: float = 0.0) -> tuple:
    a = np.array([np.asarray(p, float) for p in pts])
    return (tuple(map(float, a.min(axis=0) - grow)), tuple(map(float, a.max(axis=0) + grow)))


def _rect_box(rect: Rect, z0: float, z1: float) -> tuple:
    c = rect.corners()
    return ((float(c[:, 0].min()), float(c[:, 1].min()), z0), (float(c[:, 0].max()), float(c[:, 1].max()), z1))


def _union(*boxes) -> tuple:
    lo = np.min([b[0] for b in boxes], axis=0)
    hi = np.max([b[1] for b in boxes], axis=0)
    return (tuple(map(float, lo)), tuple(map(float, hi)))


# -- classification -------------------------------------------------------------

class Detector:
    def __init__(self, stage: Stage, ref: Reference | None = None, params: dict | None = None,
                 align: bool = True):
        self.stage = stage
        self.ref = ref or Reference(stage)
        self.cfg = self.ref.cfg
        self.p = params or load_detect_params()
        self.do_align = align

    def run(self, captures: list[Capture]) -> DefectReport:
        """Detect over all captures. Memory: each capture is measured and then
        released (Measurer.release), and an aligned capture replaces its
        unaligned original in `captures`, so the original can be freed -- the
        caller's list holds the aligned captures afterwards."""
        timings: dict = {}
        per: list[tuple[Capture, Measurer, list[Region], dict]] = []
        cap_info = []
        # Registration first, for the whole load: Z, roll and pitch per capture
        # from its housing, and the housing strips each capture saw whole.
        # With several captures, X, Y and yaw come once from the best whole
        # view of each strip (joint_datum) -- one module, one placement.
        fits: dict = {}
        joint = None
        if self.do_align:
            for ci, cap in enumerate(captures):
                t0 = time.perf_counter()
                gold = self.ref.golden(cap)
                timings[f"{cap.name}:golden"] = time.perf_counter() - t0
                t0 = time.perf_counter()
                fits[ci] = housing_fit(cap, gold, self.ref, self.p)
                timings[f"{cap.name}:align"] = time.perf_counter() - t0
                del gold
                _trim()
            if len(captures) > 1:
                joint = joint_datum({captures[ci].name: f[1].get("strips", []) for ci, f in fits.items()}, self.p)
        for ci in range(len(captures)):
            cap = captures[ci]
            info: dict = {"name": cap.name, "sensor": cap.sensor, "params_hash": cap.params_hash,
                          "points": int(len(cap.points)), "settings": cap.settings}
            aligned = False
            if self.do_align:
                t, info["icp"] = align(cap, None, self.ref, self.p, fit=fits[ci], joint=joint)
                if not info["icp"].get("kept_fixture_pose"):
                    cap = cap.transformed(t)
                    captures[ci] = cap
                    aligned = True
            # Timed apart: the golden cast is ray tracing the clean CAD, which a
            # real cell does once per station pose (or on a GPU), not per module.
            t0 = time.perf_counter()
            gold = self.ref.golden(cap)
            timings[f"{cap.name}:golden"] = timings.get(f"{cap.name}:golden", 0.0) + time.perf_counter() - t0
            t0 = time.perf_counter()
            _, rstats = residuals(cap, gold, self.p, regions=False)
            m = Measurer(self.ref, cap, gold, self.p, rstats["noise_mm"])
            if self.stage == "pre_busbar":
                m.run_terminals()
                m.run_caps()
                m.run_polarity()
            else:
                m.run_busbars()
                m.run_pads()
                m.run_carrier()
                if self.stage == "post_weld":
                    m.run_seams()
            timings[f"{cap.name}:measure"] = time.perf_counter() - t0
            m.release()
            del gold
            if aligned:
                self.ref.forget(cap)
            _trim()
            per.append((cap, m, [], info))
            cap_info.append(info)

        findings: list[Finding] = []
        measurements: dict = {}
        uncovered: dict = {}
        # Each feature method records the correction that moves its part from
        # where it was designed to where it was measured.
        self._motions, self._bead_motions = {}, {}
        if self.stage == "pre_busbar":
            self._terminals_and_cells(per, findings, measurements, uncovered)
        else:
            self._busbars(per, findings, measurements, uncovered)
            self._carrier(per, findings, measurements)
            if self.stage == "post_weld":
                self._seams(per, findings, measurements, uncovered)

        # Residuals against the module as measured: what is left is what no
        # feature measurement accounts for.
        moved = {k: v for k, v in self._motions.items() if _moves(v, self.p)}
        beads = {k: v for k, v in self._bead_motions.items() if _moves(v, self.p)}
        scene = self.ref.as_measured_scene(moved, beads) if moved or beads else None
        for i, (cap, m, _, info) in enumerate(per):
            t0 = time.perf_counter()
            pred = self.ref.as_measured(cap, moved, beads, scene) if scene else self.ref.golden(cap)
            # How far this capture may still sit from the module: half a sample
            # where the housing strips fixed X, Y and yaw (its own or the load's),
            # else the fixture's own tolerance (an axis kept the fixture pose).
            reg_axes = set(info.get("icp", {}).get("registered_axes", []))
            misreg = pred.spacing / 2 if {"x", "y", "yaw"} <= reg_axes else \
                max(pred.spacing / 2, self.p["residual"]["fixture_tol"])
            regions, info["residual"] = residuals(cap, pred, self.p, misreg=misreg)
            info["residual"]["misregistration_mm"] = misreg
            del pred
            if scene is None:
                self.ref.forget(cap)
            _trim()
            info["residual"]["against"] = (f"the module as measured ({len(moved)} parts, "
                                           f"{len(beads)} beads moved)" if moved or beads else "the golden capture")
            per[i] = (cap, m, regions, info)
            timings[f"{cap.name}:residual"] = time.perf_counter() - t0
        self._regions(per, findings)
        result = "NOK" if findings else ("INCOMPLETE" if uncovered else "OK")
        return DefectReport(self.stage, findings, measurements, uncovered, cap_info, detect_hash(),
                            {k: round(v, 2) for k, v in timings.items()}, result)

    # helpers ----------------------------------------------------------------

    def _best(self, per, attr: str, key, tol: float | None = None):
        """The capture that saw feature `key` best, by golden support; with
        `tol`, only among captures that can judge a position against it."""
        best = None
        for cap, m, _, _ in per:
            if tol is not None and not capable(m.gold, tol, self.p):
                continue
            pair = getattr(m, attr).get(key)
            if pair is not None and (best is None or pair.support > best[1].support):
                best = (cap, pair, m)
        return best

    # pre-busbar -------------------------------------------------------------

    def _terminals_and_cells(self, per, findings, meas, uncovered):
        cfg, p = self.cfg, self.p
        tq, cq = p["terminal"], p["cell"]
        sites = {(t.cell_index, t.polarity): t for t in self.ref.labels.terminals}
        term: dict = {}
        for key, t in sites.items():
            best = self._best(per, "terminals", key)
            if best is None:
                uncovered.setdefault("terminal", []).append(f"pin {t.pin}")
                continue
            cap, pair, m = best
            term[key] = _terminal_entry(cap, pair, t, p)
            term[key]["capable"] = capable(m.gold, cq["pos_tol"], p)
            if not term[key]["capable"]:
                # The best view of this pad is too coarse to place it; a finer
                # capture that also saw it places it instead.
                fine = self._best(per, "terminals", key, cq["pos_tol"])
                if fine is not None and _face_ok(fine[1].scan, fine[1].gold, p):
                    term[key] = dict(_terminal_entry(fine[0], fine[1], t, p), capable=True)
                    pair = fine[1]
            if term[key]["seen"]:
                g = pair.gold
                self._motions[("terminal", *key)] = _plane_motion(pair.scan.plane, g.plane, g.rect.cx, g.rect.cy)
        meas["terminals"] = {f"{k[0]}{k[1]}": v for k, v in term.items()}

        # Terminal height: each row's heights after removing what the whole
        # row shares (a straight line along it), then the 0.1 mm rule. The
        # line is fitted per capture, on every terminal of the row that capture
        # saw, so a height is always relative to its own capture's row: two
        # captures carry different alignment offsets and must not share a fit.
        rows: dict = {}
        for cap, m, _, _ in per:
            for side in (-1, 1):
                row = sorted((k for k, pair in m.terminals.items()
                              if k in sites and np.sign(sites[k].centre[1]) == side
                              and _face_ok(pair.scan, pair.gold, p)),
                             key=lambda k: sites[k].centre[0])
                if not row:
                    continue
                e = {k: (term[k] if term.get(k, {}).get("capture") == cap.name
                         else _terminal_entry(cap, m.terminals[k], sites[k], p)) for k in row}
                x = np.array([sites[k].centre[0] for k in row])
                dz = np.array([e[k]["dz"] for k in row])
                a, b, _ = fit_line(x, dz, floor=0.01)
                rows[cap.name, side] = {k: float(d - (a + b * xi)) for k, xi, d in zip(row, x, dz)}
                meas.setdefault("adjacent_dh", []).extend(
                    {"pins": (e[k1]["pin"], e[k2]["pin"]), "dh": e[k2]["dz"] - e[k1]["dz"],
                     "capture": cap.name}
                    for k1, k2 in zip(row, row[1:]))
        heights = {k: rows[e["capture"], np.sign(sites[k].centre[1])][k]
                   for k, e in term.items() if e["seen"]}
        for key, d in heights.items():
            e, t = term[key], sites[key]
            loc = (t.centre[0], t.centre[1], t.top_z)
            box = self.ref.part_box(("terminal", *key))
            tilted = e["tilt_deg"] > tq["tilt_tol"]
            if tilted:
                findings.append(Finding("terminal_tilt", e["tilt_deg"], loc, box, e["capture"],
                                        f"terminal pin {e['pin']}", {"tilt_deg": e["tilt_deg"], "dz": d}))
            rise = cfg.terminal_d / 2 * np.sin(np.radians(e["tilt_deg"])) if tilted else 0.0
            if abs(d) > tq["height_tol"] + rise:
                findings.append(Finding("terminal_height", d, loc, box, e["capture"],
                                        f"terminal pin {e['pin']}", {"dz_vs_row": d}))

        # Cells: pose from the terminal outlines seen in one capture.
        cells = {}
        for k in range(cfg.n_cells):
            keys = [(k, "pos"), (k, "neg")]
            seen = [key for key in keys if key in term]
            if not seen:
                uncovered.setdefault("cell", []).append(k)
                continue
            cx = cfg.cell_centre_x(k)
            # Out to the side plates: a cell pushed sideways bulges them.
            box = _box((cx - cfg.cell_t / 2, -cfg.cell_w / 2 - cfg.side_plate_t, cfg.z_cell_bottom),
                       (cx + cfg.cell_t / 2, cfg.cell_w / 2 + cfg.side_plate_t, cfg.z_wall_top))
            present = [key for key in seen if term[key]["seen"]]
            placeable = [key for key in present if term[key]["capable"]]
            if not present:
                if len(seen) == 2 or self._missing_evidence(per, k):
                    findings.append(Finding("missing_cell", 1.0, (cx, 0.0, cfg.z_cell_top), box,
                                            term[seen[0]]["capture"], f"cell {k}",
                                            {"terminals_seen": 0}))
                continue
            if not placeable:
                # Seen, but only by captures too coarse to place a pad against
                # pos_tol: its position is not measured, which is not a pass.
                uncovered.setdefault("cell position", []).append(k)
                continue
            by_cap: dict = {}
            for key in placeable:
                by_cap.setdefault(term[key]["capture"], []).append(key)
            cap_name, keys_here = max(by_cap.items(), key=lambda kv: len(kv[1]))
            src = np.array([term[key]["gold_xy"] for key in keys_here])
            dst = np.array([term[key]["xy"] for key in keys_here])
            if len(keys_here) == 2:
                r, tr = rigid_2d(src, dst)
                dth = float(np.degrees(np.arctan2(r[1, 0], r[0, 0])))
            else:
                dth = float(term[keys_here[0]]["dyaw_deg"])
                th = np.radians(dth)
                r = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
                tr = dst[0] - r @ src[0]
            c0 = np.array([cx, 0.0])
            dx, dy = (r @ c0 + tr) - c0
            # Only a shift the capture resolves moves the cell in the module
            # as measured; moving it by sampling noise would leave a residual.
            gold_k = next(m.gold for c, m, _, _ in per if c.name == cap_name)
            resolved = np.hypot(dx, dy) + np.radians(abs(dth)) * cfg.cell_w / 2 > gold_k.spacing / 2
            inplane = _planar(r, tr) if resolved else np.eye(4)
            self._motions[("cell", k)] = inplane
            for pol in ("pos", "neg"):
                ref_t = ("terminal", k, pol)
                self._motions[ref_t] = inplane @ self._motions.get(ref_t, np.eye(4))
            cells[k] = {"capture": cap_name, "dx": float(dx), "dy": float(dy), "dyaw_deg": dth,
                        "terminals": len(keys_here)}
            if np.hypot(dx, dy) > cq["pos_tol"] or abs(dth) > cq["yaw_tol"]:
                findings.append(Finding("cell_placement", float(np.hypot(dx, dy)),
                                        (cx, 0.0, cfg.z_cell_bottom), box, cap_name, f"cell {k}",
                                        cells[k]))
        meas["cells"] = cells

        # Polarity: whichever hypothesis the cap fits better.
        pol = {}
        for k in range(cfg.n_cells):
            best = max(((cap, m) for cap, m, _, _ in per if k in m.polarity),
                       key=lambda cm: cm[1].polarity[k]["points"], default=None)
            if best is None:
                continue
            cap, m = best
            sc = m.polarity[k]
            pol[k] = dict(sc, capture=cap.name)
            if sc["turned"] < sc["as_designed"]:
                cx = cfg.cell_centre_x(k)
                box = _box((cx - cfg.cell_t / 2, -cfg.cell_w / 2, cfg.z_cell_top),
                           (cx + cfg.cell_t / 2, cfg.cell_w / 2, cfg.z_terminal_top))
                findings.append(Finding("reversed_polarity", 180.0, (cx, 0.0, cfg.z_cell_top), box,
                                        cap.name, f"cell {k}", sc))
        meas["polarity"] = pol

    def _missing_evidence(self, per, k) -> bool:
        """Is the cap gone too, in some capture that should have seen it?"""
        frac = self.p["fit"]["min_fraction"]
        return any(k in m.caps and m.caps[k][0] < frac * m.caps[k][1] for _, m, _, _ in per)

    # pre-weld and post-weld ---------------------------------------------------

    def _busbars(self, per, findings, meas, uncovered):
        cfg, p = self.cfg, self.p
        bq = p["busbar"]
        out, gaps = {}, {}
        self._gap_planes = {}
        for b in self.ref.labels.busbars:
            best = self._best(per, "busbars", b.index)
            if best is None:
                uncovered.setdefault("busbar", []).append(b.index)
                continue
            cap, pair, m = best
            s, g = pair.scan, pair.gold
            top = b.underside_z + cfg.busbar_t
            if not _face_ok(s, g, p):
                out[b.index] = {"capture": cap.name, "seen": False}
                continue
            dx, dy = s.rect.cx - g.rect.cx, s.rect.cy - g.rect.cy
            dyaw = s.rect.yaw_deg - g.rect.yaw_deg
            e = {"capture": cap.name, "seen": True, "dx": dx, "dy": dy, "dyaw_deg": dyaw,
                 "dz": float(s.plane.z(b.centre[0], b.centre[1]) - g.plane.z(b.centre[0], b.centre[1])),
                 "tilt_deg": float(np.degrees(np.arccos(np.clip(s.plane.normal @ g.plane.normal, -1, 1)))),
                 "extent": (s.rect.ex, s.rect.ey)}
            out[b.index] = e
            box = _union(_rect_box(s.rect, top - 2, top + 3), _rect_box(g.rect, top - 2, top + 3))
            loc = (b.centre[0], b.centre[1], top)
            e["pose_measured"] = capable(m.gold, bq["pos_tol"], p)
            motion = _plane_motion(s.plane, g.plane, b.centre[0], b.centre[1])
            reach = np.hypot(dx, dy) + np.radians(abs(dyaw)) * max(g.rect.ex, g.rect.ey) / 2
            if e["pose_measured"] and reach > m.gold.spacing / 2:   # resolved, not sampling noise
                th = np.radians(dyaw)
                r2 = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
                gc, sc = np.array([g.rect.cx, g.rect.cy]), np.array([s.rect.cx, s.rect.cy])
                motion = _planar(r2, sc - r2 @ gc) @ motion
            self._motions[("busbar", b.index)] = motion
            if not e["pose_measured"]:
                uncovered.setdefault("busbar position", []).append(b.index)
            elif np.hypot(dx, dy) > bq["pos_tol"] or abs(dyaw) > bq["yaw_tol"]:
                findings.append(Finding("busbar_pose", float(np.hypot(dx, dy)), loc, box, cap.name,
                                        f"busbar {b.index}", e))
            # Gap at each terminal under it, from this same capture.
            worst = None
            for k, pol in b.terminals:
                pad = m.pads.get((k, pol))
                if pad is None or not _face_ok(pad.scan, pad.gold, p):
                    continue
                t = next(t for t in self.ref.labels.terminals if t.cell_index == k and t.polarity == pol)
                x, y = t.centre[:2]
                self._motions[("terminal", k, pol)] = _plane_motion(pad.scan.plane, pad.gold.plane, x, y)
                gap = _gap(s.plane, g.plane, pad.scan.plane, pad.gold.plane, x, y, cfg)
                self._gap_planes[(k, pol)] = (s.plane, g.plane, pad.scan.plane, pad.gold.plane)
                gaps[f"busbar {b.index} pin {t.pin}"] = {"capture": cap.name, "gap": gap}
                if worst is None or gap > worst[0]:
                    worst = (gap, t)
            if worst is not None and worst[0] > bq["gap_tol"]:
                gap, t = worst
                findings.append(Finding("busbar_liftoff", gap, (t.centre[0], t.centre[1], t.top_z), box,
                                        cap.name, f"busbar {b.index}", {"max_gap": gap, "at_pin": t.pin}))
        meas["busbars"] = out
        meas["gaps"] = gaps

    def _carrier(self, per, findings, meas):
        cfg, q = self.cfg, self.p["carrier"]
        best = max(((cap, m.carrier) for cap, m, _, _ in per if m.carrier is not None),
                   key=lambda cp: cp[1].support, default=None)
        if best is None:
            return
        cap, pair = best
        if pair.scan is None:
            meas["carrier"] = {"capture": cap.name, "seen": False}
            return
        s, g = pair.scan.plane, pair.gold.plane
        dz = float(s.z(0, 0) - g.z(0, 0))
        tilt = float(np.degrees(np.arccos(np.clip(s.normal @ g.normal, -1, 1))))
        meas["carrier"] = {"capture": cap.name, "dz": dz, "tilt_deg": tilt}
        self._motions[("carrier",)] = _plane_motion(s, g, 0.0, 0.0)
        if abs(dz) > q["dz_tol"] or tilt > q["tilt_tol"]:
            box = _box((-cfg.carrier_l / 2, -cfg.carrier_w / 2, cfg.z_cell_top),
                       (cfg.carrier_l / 2, cfg.carrier_w / 2, cfg.z_carrier_top + 6))
            findings.append(Finding("carrier_seating", dz, (0.0, 0.0, cfg.z_carrier_top), box, cap.name,
                                    "carrier", meas["carrier"]))

    def _seams(self, per, findings, meas, uncovered):
        cfg, q = self.cfg, self.p["seam"]
        planes = getattr(self, "_gap_planes", {})
        out = {}
        for i, seam in enumerate(self.ref.labels.seams):
            best = self._best(per, "seams", i, q["offset_tol"])
            if best is None:
                uncovered.setdefault("seam", []).append(i)
                continue
            cap, pair, _ = best
            g, s = pair.gold, pair.scan
            t = next(t for t in self.ref.labels.terminals
                     if t.cell_index == seam.cell_index and abs(t.centre[1] - seam.start[1]) < 15)
            name = f"seam {i} (busbar {seam.busbar_index}, pin {t.pin})"
            c = np.array(g["centre_xy"])
            half = np.abs(np.array(g["u"])) * (g["nominal_length"] / 2 + q["along_margin"]) + \
                np.abs(np.array(g["v"])) * q["across"]
            box = _box((*(c - half), g["z"] - 2.0), (*(c + half), g["z"] + 1.0))
            loc = (float(c[0]), float(c[1]), g["z"])
            if s is None:
                out[i] = {"capture": cap.name, "seen": False}
                continue
            e = {"capture": cap.name, "coverage": s["coverage"], "length": s["length"],
                 "gold_length": g["length"]}
            found = []
            # The largest busbar-to-pad gap under the stitch, from this same
            # capture: a busbar tilted on a chip opens toward one end, so its
            # ends, not its middle. Over a gap the melt runs down to bridge it
            # (weld.py) and that reshapes the whole bead, so the gap is then
            # the cause: report what it did (underfill, undercut, hole), not
            # every shape change it made.
            pl = planes.get((t.cell_index, t.polarity))
            gap = max(_gap(*pl, x, y, cfg) for x, y in (seam.start[:2], seam.end[:2])) if pl else 0.0
            e["gap_here"] = gap
            gapped = gap > self.p["busbar"]["gap_tol"]
            if g["coverage"] > 0.5 and s["coverage"] < q["missing_coverage"] and s["deep"] < q["hole_points"]:
                found.append(("missing_weld", 1.0))
            else:
                short = 1.0 - s["length"] / max(g["length"], 1e-9)
                e["short"] = short
                if short > q["short_tol"] and not gapped:
                    found.append(("short_weld", short))
                both = s["has"] & g["has"] & (np.abs(s["mids"]) <= g["nominal_length"] / 2)
                if both.sum() >= 5:
                    sm = s["mids"][both]
                    dc = s["centre"][both] - g["centre"][both]
                    basis = np.column_stack([np.ones(len(sm)), 1 - (2 * sm / g["nominal_length"]) ** 2])
                    coef, *_ = np.linalg.lstsq(basis, dc, rcond=None)
                    offset, sag = float(coef[0]), float(coef[1])
                    # Drift: how much more each centreline wanders about its own
                    # straight-plus-bow fit than the golden one does, not the
                    # wander of their bin-by-bin difference. The difference
                    # needs scan and golden bins to line up along the stitch;
                    # alignment leaves up to half a profile pitch along X, and
                    # bead texture then reads as drift on every clean stitch
                    # (0.12-0.45 under a 0.15 mm fixture error).
                    own = np.column_stack([basis, sm])
                    # Path drift wanders over millimetres; below `drift_window`
                    # the centreline holds only sampling artefacts. A bead's fine
                    # ripple sampled at the line-scan's 0.1 mm pitch aliased into a
                    # bin-by-bin alternation of +/-0.12 mm under some fixture
                    # offsets, read as 0.2 mm of drift on clean stitches. A running
                    # mean over the window removes it; a 6 mm-wavelength drift
                    # loses ~5% (sinc 1/6).
                    win = max(1, int(round(q["drift_window"] / q["bin"])))

                    def wander(c):
                        if win > 1 and len(c) > win:
                            c = np.convolve(c, np.ones(win) / win, mode="valid")
                            o = own[(win - 1) // 2:(win - 1) // 2 + len(c)]
                        else:
                            o = own
                        e = c - o @ np.linalg.lstsq(o, c, rcond=None)[0]
                        return 1.4826 * float(np.median(np.abs(e - np.median(e))))
                    ws, wg = wander(s["centre"][both]), wander(g["centre"][both])
                    drift = float(np.sqrt(2) * np.sqrt(max(ws ** 2 - wg ** 2, 0.0)))
                    width = float(np.nanmedian(s["width"][both]) / np.nanmedian(g["width"][both]) - 1)
                    # The same from cross-section over crown height (a bead's
                    # area is width x crown x a shape constant): recorded beside
                    # the edge-percentile width while the two are compared (1.8).
                    e["width_change_area"] = float(
                        np.nanmedian(s["area"][both] / s["crown"][both]) /
                        np.nanmedian(g["area"][both] / g["crown"][both]) - 1)
                    crown = float(np.nanmedian(s["crown"][both]) - np.nanmedian(g["crown"][both]))
                    e.update(offset=offset, sagitta=sag, drift=drift, width_change=width, crown_change=crown)
                    if not gapped:
                        if abs(sag) > q["arc_tol"]:
                            found.append(("weld_arc", abs(sag)))
                        if abs(offset) > q["offset_tol"]:
                            found.append(("seam_deviation", offset))
                        if drift > q["drift_tol"]:
                            found.append(("weld_path_drift", drift))
                        if abs(width) > q["width_tol"]:
                            found.append(("width_deviation", float(np.nanmedian(s["width"][both]))))
                    if crown < -q["crown_tol"]:
                        found.append(("weld_underfill", -crown) if gapped else ("reinforcement", crown))
                    elif crown > q["crown_tol"]:
                        found.append(("reinforcement", crown))
                e["undercut"] = s["undercut"] - g["undercut"]
                if e["undercut"] > q["undercut_tol"]:
                    found.append(("weld_undercut", e["undercut"]))
                e["deep"] = s["deep"] - g["deep"]
                if e["deep"] >= q["hole_points"]:
                    found.append(("burn_through", 1.0))
            out[i] = e
            bead = self._motions.get(("busbar", seam.busbar_index), np.eye(4)).copy()
            # Shift the bead by its measured offset only when the offset stands
            # clear of the centreline's own noise; shifting by noise would
            # itself leave a residual beside every clean stitch.
            if "offset" in e and not gapped and abs(e["offset"]) > q["bead_shift_min"]:
                bead[:2, 3] += e["offset"] * np.asarray(g["v"], float)[:2]
            self._bead_motions[i] = bead
            for cls, mag in found:
                findings.append(Finding(cls, float(mag), loc, box, cap.name, name, e))
        meas["seams"] = out

    # residual regions ---------------------------------------------------------

    def _regions(self, per, findings):
        """Turn residual regions into findings, after removing what a feature
        finding or a measured pose already accounts for."""
        q = self.p["residual"]
        absorbing = [f for f in findings]
        for cap, m, regions, _ in per:
            # What earlier captures already reported absorbs this one's regions
            # too: a drop both sensors saw is one defect, not two. Not within a
            # capture, where two regions are two separate components.
            absorbing = absorbing + [f for f in findings[len(absorbing):]]
            explained = np.zeros(len(cap.points), bool)
            # Each outline with the band its own motion needs: a part that
            # moved 0.3 mm leaves residuals up to 0.3 mm either side of its
            # edges; one that did not leaves only sampling-phase flips.
            outlines: list[tuple[Rect, float]] = []
            for pair in list(m.terminals.values()) + list(m.busbars.values()):
                if pair.scan is not None:
                    explained[pair.scan.idx] = True
                g = pair.gold.rect
                if g is None:
                    continue
                band = max(q["edge_band"], m.gold.spacing)
                sc = pair.scan.rect if pair.scan is not None else None
                if sc is not None:
                    band += np.hypot(sc.cx - g.cx, sc.cy - g.cy) + \
                        np.radians(abs(sc.yaw_deg - g.yaw_deg)) * max(g.ex, g.ey) / 2
                    outlines.append((sc, band))
                outlines.append((g, band))
            # Weld beads too: alignment leaves up to half a profile pitch, and a
            # bead's ends and toes then leave slivers just like a part's edges.
            for pair in m.seams.values():
                g = pair.gold
                if g is None or "centre_xy" not in g:
                    continue
                u = np.asarray(g["u"], float)
                outlines.append((Rect(float(g["centre_xy"][0]), float(g["centre_xy"][1]),
                                      float(np.degrees(np.arctan2(u[1], u[0]))),
                                      float(g["nominal_length"]), float(self.cfg.weld_stitch_w)),
                                 max(q["edge_band"], m.gold.spacing)))
            for pair in list(m.pads.values()) + ([m.carrier] if m.carrier else []):
                if pair.scan is not None:
                    explained[pair.scan.idx] = True
            # A bead this capture's seam measurement judged accounts for what is
            # left on it. The module as measured moves beads but does not reshape
            # them, so an in-spec crown or width change remains as residual, and the
            # profilometer (~1 um noise) sees it as a dent or spatter: 10/10 in-spec
            # reinforcement modules rejected (1.7 arm column, 12 Nov 2026). Out of
            # spec, the seam's own finding covers it. The bead is as wide as its
            # widest in-spec width. Cost: a drop landing on a bead is not reported
            # (the fixed pass already misses those, bead band). Reshaping the beads
            # instead is deferred (status_update.md, "Deferred to refine").
            judged = []
            for pair in m.seams.values():
                g = pair.gold
                if pair.scan is None or g is None or "centre_xy" not in g:
                    continue
                u = np.asarray(g["u"], float)
                judged.append(Rect(float(g["centre_xy"][0]), float(g["centre_xy"][1]),
                                   float(np.degrees(np.arctan2(u[1], u[0]))), float(g["nominal_length"]),
                                   float(self.cfg.weld_stitch_w) * (1 + self.p["seam"]["width_tol"])))
            bare = short_bead_stretches(m.seams, self.p["seam"], self.cfg.weld_stitch_w)
            for reg in regions:
                if self._absorbed(reg, absorbing):
                    continue
                pts = np.vstack([x for x in (cap.points[reg.scan_idx], reg.gold_pts) if len(x)])
                near_edge = _near_outline(pts[:, :2], outlines)
                for rect in bare + judged:
                    near_edge |= _in_rect(pts[:, :2], rect, q["edge_band"])
                frac = (explained[reg.scan_idx].sum() + near_edge.sum()) / max(len(pts), 1)
                if frac > 0.5 or reg.on_step > 0.5:
                    continue
                cls = self._region_class(reg)
                findings.append(Finding(cls, reg.peak if np.isfinite(reg.peak) else reg.area,
                                        reg.location, reg.bounds, cap.name, f"{reg.kind} region on {reg.material}",
                                        {"area_mm2": reg.area, "points": reg.n, "peak_mm": reg.peak,
                                         "missing_fraction": reg.missing_fraction}))

    @staticmethod
    def _absorbed(reg: Region, findings) -> bool:
        x, y, z = reg.location
        for f in findings:
            (x0, y0, z0), (x1, y1, z1) = f.bounds
            if x0 - 1 <= x <= x1 + 1 and y0 - 1 <= y <= y1 + 1 and z0 - 1 <= z <= z1 + 1:
                return True
        return False

    def _region_class(self, reg: Region) -> str:
        q = self.p["residual"]
        if reg.kind == "raised":
            if reg.area <= q["blob_max_area"]:
                return "spatter" if self.stage == "post_weld" else "fod"
            return "anomaly"
        if reg.material == "foam":
            return "missing_pad"
        if reg.area <= q["dent_max_area"] and reg.missing_fraction < 0.5:
            return "handling_damage"
        return "anomaly"


def _plane_motion(scan: Plane, gold: Plane, x: float, y: float) -> np.ndarray:
    """The rigid correction taking a part whose top was fitted as `gold` to
    one fitted as `scan`: tilt its normal onto the scan normal about the top's
    point above (x, y), then lift it by the height difference there."""
    ng, ns = gold.normal, scan.normal
    axis = np.cross(ng, ns)
    sin, cos = np.linalg.norm(axis), float(ng @ ns)
    out = np.eye(4)
    if sin > 1e-12:
        out = trimesh.transformations.rotation_matrix(np.arctan2(sin, cos), axis / sin,
                                                      (x, y, gold.z(x, y)))
    out[2, 3] += scan.z(x, y) - gold.z(x, y)
    return out


def _planar(r2: np.ndarray, t2) -> np.ndarray:
    """A 2D rigid motion (x' = R x + t) as a 4x4."""
    out = np.eye(4)
    out[:2, :2], out[:2, 3] = r2, t2
    return out


def _moves(t: np.ndarray, p: dict) -> bool:
    """Does correction `t` move a point a part's size from its origin by more
    than identity_tol? (Not worth a recast otherwise.)"""
    angle = np.arccos(np.clip((np.trace(t[:3, :3]) - 1) / 2, -1, 1))
    return np.linalg.norm(t[:3, 3]) + angle * 200.0 > p["icp"]["identity_tol"]


def _terminal_entry(cap: Capture, pair, t, p: dict) -> dict:
    """One terminal as one capture saw it: height, tilt and outline motion
    against the golden capture."""
    s, g = pair.scan, pair.gold
    entry = {"capture": cap.name, "pin": t.pin, "seen": _face_ok(s, g, p)}
    if entry["seen"]:
        zs = s.plane.z(s.rect.cx, s.rect.cy)
        zg = g.plane.z(g.rect.cx, g.rect.cy)
        tilt = float(np.degrees(np.arccos(np.clip(s.plane.normal @ g.plane.normal, -1, 1))))
        entry.update(dz=float(zs - zg), z=float(zs), tilt_deg=tilt,
                     dx=s.rect.cx - g.rect.cx, dy=s.rect.cy - g.rect.cy,
                     xy=(s.rect.cx, s.rect.cy), gold_xy=(g.rect.cx, g.rect.cy),
                     dyaw_deg=s.rect.yaw_deg - g.rect.yaw_deg)
    return entry


def _bead_span(prof: dict, bin_: float) -> tuple[float, float] | None:
    """Along-seam extent (from, to) of the longest melted run in a seam
    profile (`measure_seam`), or None if nothing melted."""
    best, start = (0, 0, 0), None
    for i, flag in enumerate(list(prof["has"]) + [False]):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            if i - start > best[0]:
                best = (i - start, start, i - 1)
            start = None
    if best[0] == 0:
        return None
    return float(prof["mids"][best[1]] - bin_ / 2), float(prof["mids"][best[2]] + bin_ / 2)


def short_bead_stretches(seams: dict, q: dict, stitch_w: float) -> list[Rect]:
    """Where a bead measured shorter than the golden one differs from it at
    its ends: per seam, the stretch from each golden bead end back to the
    measured one and on into the bead by its stop crater (one stitch width:
    weld.py sinks the crater over the pool's size, so an early stop moves it
    too), the stitch's width across, in the scan's stitch frame.

    The module as measured keeps every bead at its design length, so a bead
    that stopped short -- by less than `short_tol`, or a `short_weld` finding
    would cover it -- leaves a dip there the depth of the crown, read as a
    0.2 mm dent (`handling_damage`). The seam measurement already judged that
    length, so residuals in these stretches are explained by it -- while the
    shortfall is within `short_tol`; past it a finding has to cover it.
    Found by 1.8: half the in-spec short welds were rejected for 'dents'."""
    out = []
    bin_ = q["bin"]
    for pair in seams.values():
        sc, g = pair.scan, pair.gold
        if sc is None or g is None:
            continue
        ss, gs = _bead_span(sc, bin_), _bead_span(g, bin_)
        if ss is None or gs is None or 1 - (ss[1] - ss[0]) / (gs[1] - gs[0]) > q["short_tol"]:
            continue
        u, c = np.asarray(sc["u"], float), np.asarray(sc["centre_xy"], float)
        yaw = float(np.degrees(np.arctan2(u[1], u[0])))
        for lo, hi in ((gs[0], ss[0] + stitch_w), (ss[1] - stitch_w, gs[1])):
            if hi - lo > stitch_w + 1e-9:          # the bead receded at this end
                mid = c + u * (lo + hi) / 2
                out.append(Rect(float(mid[0]), float(mid[1]), yaw, float(hi - lo), float(stitch_w)))
    return out


def _near_outline(xy: np.ndarray, rects: list[tuple[Rect, float]]) -> np.ndarray:
    """True for points within each rectangle's band of its edge."""
    near = np.zeros(len(xy), bool)
    for rect, band in rects:
        c, s = np.cos(np.radians(rect.yaw_deg)), np.sin(np.radians(rect.yaw_deg))
        d = xy - [rect.cx, rect.cy]
        u, v = d @ [c, s], d @ [-s, c]
        du, dv = np.abs(u) - rect.ex / 2, np.abs(v) - rect.ey / 2
        inside_band = (np.abs(du) <= band) & (dv <= band) | (np.abs(dv) <= band) & (du <= band)
        near |= inside_band
    return near


def scan_problem(path: Path, run: Path, stage: Stage) -> tuple[str, set]:
    """Why a saved scan cannot be used for detecting `stage` on `run`, or ''.
    Also returns the groups it was scanned with put back (--include), which
    the golden capture has to have too."""
    from .scan import labels_fingerprint

    meta = ScanResult.read_meta(path)
    src = meta.get("source") or {}
    rescan = f"rescan it: python -m defectgen.scan {run} --stage {stage} --sensor {meta['sensor']}"
    if not meta["has_index"]:
        return f"saved before scans recorded their ray index -- {rescan}", set()
    if not src:
        return f"does not record which module it scanned -- {rescan}", set()
    if src.get("stage") != stage:
        return f"a scan of stage {src.get('stage')}, not {stage}", set()
    if src.get("labels") != labels_fingerprint(run, stage):
        return f"the module was regenerated after this scan -- {rescan}", set()
    return "", set(src.get("include", []))


def detect(captures: list[Capture], stage: Stage, ref: Reference | None = None, **kw) -> DefectReport:
    return Detector(stage, ref, **kw).run(captures)


# -- command line ----------------------------------------------------------------

def main() -> None:
    import argparse

    from .evaluate import evaluate, format_table, to_json

    ap = argparse.ArgumentParser(prog="python -m defectgen.detect", description=__doc__.split("\n")[0])
    ap.add_argument("run", type=Path, help="a defectgen --out directory")
    ap.add_argument("--stage", default="post_weld")
    ap.add_argument("--scans", nargs="+", type=Path, default=None,
                    help="scan .npz files (default: every <run>/scans/<stage>_*.npz)")
    ap.add_argument("--include", nargs="+", default=[], metavar="GROUP",
                    help="groups the scans included that the stage normally leaves out (post_weld: cmc cover)")
    ap.add_argument("--no-align", dest="align", action="store_false")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    named = args.scans is not None
    candidates = args.scans or sorted((args.run / "scans").glob(f"{args.stage}_*.npz"))
    scans, include = [], set(args.include)
    for path in candidates:
        why, inc = scan_problem(path, args.run, args.stage)
        if why and named:
            ap.error(f"{path}: {why}")
        if why:
            log.warning("skipping %s: %s", path.name, why)
            continue
        scans.append(path)
        include |= inc
    if not scans:
        ap.error(f"no usable {args.stage} scans in {args.run / 'scans'}; scan with "
                 f"python -m defectgen.scan {args.run} --stage {args.stage} --sensor line_scan")
    t0 = time.perf_counter()
    captures = [Capture.load(s) for s in scans]
    log.info("captures: %s", ", ".join(f"{c.name} ({len(c.points):,} pts)" for c in captures))
    ref = Reference(args.stage, include=sorted(include), run=args.run)
    report = detect(captures, args.stage, ref, align=args.align)
    out = args.out or args.run / "detections" / f"{args.stage}.report.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(report.to_json())
    log.info("%s: %d findings in %.1f s -> %s", report.result, len(report.findings),
             time.perf_counter() - t0, out)
    for f in report.findings:
        log.info("  %-18s %9.3f  %-34s  at (%.1f, %.1f, %.1f)  [%s]", f.cls, f.magnitude, f.feature,
                 *f.location, f.capture)
    if report.uncovered:
        log.info("not seen by any capture: %s",
                 "; ".join(f"{k}: {len(v)}" for k, v in report.uncovered.items()))
    labels = args.run / f"{args.stage}.labels.json"
    if labels.is_file():
        result = evaluate(report, json.loads(labels.read_text()))
        out.with_name(f"{args.stage}.evaluation.json").write_text(to_json(result))
        log.info("\n%s", format_table(result))


if __name__ == "__main__":
    main()

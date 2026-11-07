"""Coverage and view selection (task 1.5, project_plan.md §4B.5 "Design", §9.8).

    python -m defectgen.coverage --stage post_weld

Which required surfaces of the clean module each approach can observe well
enough to measure, *in a single capture*:

* the fixed Ranger3 line-scan, one pass (the baseline), and
* the HC10DT wrist sensors -- Zivid snapshots and Keyence sweeps -- picked by
  greedy set cover from candidate views.

The pieces:

1. **Elements.** The stage's clean module, the faces of the surfaces some
   defect class needs, subdivided into triangles no longer than
   `element_size`. One triangle, one element: exact area and normal.
2. **Requirement.** Each class names its surfaces and D, the smallest size
   that matters for it; it needs sample spacing d = D/2 there (§3.7.3).
3. **Observable at all.** An element counts only if some direction within
   `theta_max` of its normal, at or above horizontal, escapes the module.
4. **Covered.** A capture covers element j for class k when, in that one
   capture, at least max(1, a_j / d_k^2) of its points land on j at an
   incidence below `theta_max`. Counts come from the frozen sensor models, so
   shadowing and specular dropout count.
5. **Greedy set cover** over candidate views by mean class coverage gained
   per second of capture time.

Coverage is computed on the clean module: it is a property of where a
sensor looks, not of what is wrong with the part.
"""

from __future__ import annotations

import hashlib
import json
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh
import yaml

from .config import Config, load
from .scan import ScanResult, Scene, line_scan, load_params, looking_down, params_hash, snapshot

CONFIG = Path(__file__).parent / "configs" / "coverage.yaml"
log = logging.getLogger("defectgen.coverage")


def load_coverage_params(path: Path | None = None) -> dict:
    return yaml.safe_load(Path(path or CONFIG).read_text())


# -- elements -------------------------------------------------------------------

@dataclass
class Elements:
    """Surface elements of one stage's clean module: one per triangle."""

    centroid: np.ndarray       # (E, 3)
    normal: np.ndarray         # (E, 3) unit
    area: np.ndarray           # (E,) mm^2
    group: np.ndarray          # (E,) index into group_names
    group_names: list
    surfaces: dict             # surface name -> (E,) bool
    vertices: np.ndarray       # the element mesh, for exact point-to-triangle assignment
    faces: np.ndarray
    observable: np.ndarray = None   # (E,) bool, filled by `observe`
    _scene: object = field(default=None, repr=False)
    # Patches: what coverage is judged on. The CAD is finely meshed (median
    # triangle 0.06 mm^2), and "one point on a 0.06 mm^2 triangle" is luck,
    # not density. Observable triangles of one group, facing one way, in one
    # patch_size cell form a patch; a point counts for the patch of the
    # triangle it lies on.
    patch: np.ndarray = None        # (E,) patch index per triangle, -1 if unobservable
    p_area: np.ndarray = None       # (P,)
    p_centroid: np.ndarray = None
    p_normal: np.ndarray = None
    p_surfaces: dict = None         # surface name -> (P,) bool

    def __len__(self) -> int:
        return len(self.area)

    def make_patches(self, size: float) -> int:
        ok = self.observable
        key = np.column_stack([np.floor(self.centroid / size).astype(np.int64), self.group[:, None],
                               np.round(self.normal * 3).astype(np.int64)])
        _, inv = np.unique(key[ok], axis=0, return_inverse=True)
        inv = inv.ravel()
        self.patch = np.full(len(self), -1, np.int64)
        self.patch[ok] = inv
        n = int(inv.max()) + 1 if len(inv) else 0
        a = self.area[ok]
        self.p_area = np.bincount(inv, weights=a, minlength=n)
        self.p_centroid = np.column_stack([np.bincount(inv, weights=a * self.centroid[ok, k], minlength=n)
                                           for k in range(3)]) / self.p_area[:, None]
        nrm = np.column_stack([np.bincount(inv, weights=a * self.normal[ok, k], minlength=n) for k in range(3)])
        self.p_normal = nrm / np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
        # A patch belongs to a surface if most of its area does.
        self.p_surfaces = {s: np.bincount(inv, weights=a * m[ok], minlength=n) > 0.5 * self.p_area
                           for s, m in self.surfaces.items()}
        return n

    def closest(self, points: np.ndarray, chunk: int = 2_000_000) -> tuple[np.ndarray, np.ndarray]:
        """Index of the element each point lies on, and its distance to it."""
        import open3d as o3d

        if self._scene is None:
            sc = o3d.t.geometry.RaycastingScene()
            sc.add_triangles(o3d.core.Tensor(self.vertices.astype(np.float32)),
                             o3d.core.Tensor(self.faces.astype(np.uint32)))
            self._scene = sc
        ids = np.empty(len(points), np.int64)
        dist = np.empty(len(points))
        for a in range(0, len(points), chunk):
            q = points[a:a + chunk].astype(np.float32)
            ans = self._scene.compute_closest_points(o3d.core.Tensor(q))
            ids[a:a + chunk] = ans["primitive_ids"].numpy()
            dist[a:a + chunk] = np.linalg.norm(ans["points"].numpy() - q, axis=1)
        return ids, dist


def _facing(normal: np.ndarray, centroid: np.ndarray, facing: str, q: dict) -> np.ndarray:
    nz = normal[:, 2]
    if facing == "up":
        return nz >= q["up_cos"]
    if facing == "side":
        return np.abs(nz) < q["side_cos"]
    if facing == "outer":
        # A wall facing away from the module centre (the module is centred on its XY origin).
        return (np.abs(nz) < q["side_cos"]) & ((normal[:, :2] * centroid[:, :2]).sum(axis=1) > 0)
    if facing == "any":
        return np.ones(len(nz), bool)
    raise ValueError(f"unknown facing {facing!r}")


def build_elements(groups: dict[str, trimesh.Trimesh], q: dict) -> Elements:
    """Subdivide the faces any surface needs into elements, keeping for each
    element which group it came from and which surfaces it belongs to."""
    surf = q["surfaces"]
    names = sorted(groups)
    verts, faces, cen, nor, area, grp, off = [], [], [], [], [], [], 0
    labels: dict[str, list] = {s: [] for s in surf}
    for gi, g in enumerate(names):
        mesh = groups[g]
        wanted = [s for s, spec in surf.items() if g in spec["groups"]]
        if not wanted:
            continue
        fn, fc = mesh.face_normals, mesh.triangles_center
        per_face = {s: _facing(fn, fc, surf[s]["facing"], q) for s in wanted}
        keep = np.zeros(len(fn), bool)
        for m in per_face.values():
            keep |= m
        if not keep.any():
            continue
        v, f, idx = trimesh.remesh.subdivide_to_size(mesh.vertices, mesh.faces[keep],
                                                     max_edge=q["element_size"], return_index=True)
        src = np.flatnonzero(keep)[idx]          # original face of each new triangle
        tri = v[f]
        cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        a = 0.5 * np.linalg.norm(cr, axis=1)
        ok = a > 1e-9
        f, src, tri, a = f[ok], src[ok], tri[ok], a[ok]
        verts.append(v)
        faces.append(f + off)
        off += len(v)
        cen.append(tri.mean(axis=1))
        nor.append(fn[src])
        area.append(a)
        grp.append(np.full(len(f), gi))
        for s in surf:
            labels[s].append(per_face[s][src] if s in per_face else np.zeros(len(f), bool))
    el = Elements(np.vstack(cen), np.vstack(nor), np.concatenate(area), np.concatenate(grp), names,
                  {s: np.concatenate(v) for s, v in labels.items()}, np.vstack(verts), np.vstack(faces))
    # Surfaces restricted to the neighbourhood of another group (busbar top beside a bead).
    for s, spec in surf.items():
        near = spec.get("near")
        if near and near in names:
            from scipy.spatial import cKDTree

            other = el.group == names.index(near)
            if other.any():
                d, _ = cKDTree(el.centroid[other, :2]).query(el.centroid[:, :2])
                el.surfaces[s] &= d <= q["toe_band"]
            else:
                el.surfaces[s][:] = False
        elif near:
            el.surfaces[s][:] = False
    return el


def _cone(k: int, half_angle_deg: float) -> np.ndarray:
    """k unit vectors spread evenly over the cap within `half_angle_deg` of +Z
    (Fibonacci spiral), plus +Z itself."""
    i = np.arange(k) + 0.5
    z = 1 - (1 - np.cos(np.radians(half_angle_deg))) * i / k
    r = np.sqrt(1 - z ** 2)
    phi = i * np.pi * (3 - np.sqrt(5))
    return np.vstack([[0.0, 0.0, 1.0], np.column_stack([r * np.cos(phi), r * np.sin(phi), z])])


def _basis(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors completing each normal to an orthonormal frame."""
    ref = np.where(np.abs(n[:, 2:3]) < 0.9, [[0.0, 0.0, 1.0]], [[1.0, 0.0, 0.0]])
    u = np.cross(n, ref)
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    return u, np.cross(n, u)


def observe(el: Elements, scene: Scene, q: dict, chunk: int = 2_000_000) -> np.ndarray:
    """Can each element be seen at all, from above or the side, within
    theta_max of its normal? Faces in contact or enclosed cannot."""
    cone = _cone(q["observe_rays"], q["theta_max"])
    u, v = _basis(el.normal)
    seen = np.zeros(len(el), bool)
    for c in cone:
        d = c[0] * u + c[1] * v + c[2] * el.normal
        up = d[:, 2] >= -1e-9                       # nothing looks up through the pallet
        idx = np.flatnonzero(up & ~seen)
        for a in range(0, len(idx), chunk):
            j = idx[a:a + chunk]
            t, _, _ = scene.cast(el.centroid[j] + 0.02 * el.normal[j], d[j])
            seen[j[~np.isfinite(t)]] = True
    el.observable = seen
    return seen


# -- what each class needs ------------------------------------------------------

@dataclass
class Requirement:
    """One class family at one stage: its elements and the points each needs."""

    name: str
    D: float
    members: list
    elements: np.ndarray       # indices into the patches
    need: np.ndarray           # points needed per patch, in one capture
    areas: np.ndarray          # mm^2 per patch
    area: float                # total required area, mm^2
    level: int = 0             # row of the count array: points quiet enough for this D

    def covered(self, n: np.ndarray) -> np.ndarray:
        """Which of this requirement's patches count array `n` (levels x
        patches, see `counts`) covers."""
        return n[self.level, self.elements] >= self.need

    def fraction(self, ok: np.ndarray) -> float:
        return float(self.areas[ok].sum() / self.area) if self.area else float("nan")


def levels(q: dict) -> list[float]:
    """Every D any class uses: one row of counts each, since a point counts
    for class k only if its noise is at most D_k/3."""
    return sorted({spec["D"] for stage in q["stages"].values() for spec in stage.values()})


def requirements(el: Elements, stage: str, q: dict) -> list[Requirement]:
    lv = levels(q)
    out = []
    for name, spec in q["stages"][stage].items():
        mask = np.zeros(len(el.p_area), bool)
        for s in spec["surfaces"]:
            mask |= el.p_surfaces[s]
        j = np.flatnonzero(mask)
        d = spec["D"] / 2.0
        need = np.maximum(1.0, el.p_area[j] / d ** 2)
        out.append(Requirement(name, spec["D"], spec["members"], j, need, el.p_area[j],
                               float(el.p_area[j].sum()), lv.index(spec["D"])))
    return out


# -- counting a capture ------------------------------------------------------------

def counts(result: ScanResult, el: Elements, q: dict, scale: float = 1.0) -> np.ndarray:
    """Points per patch that can serve a measurement, scaled to the real
    sensor's density: one row per D in `levels(q)`. A point counts if it lies
    on the patch (its noise-free hit, `truth`, assigned to its triangle), came
    in at an incidence below theta_max, and -- §3.7.3's other half -- its
    noise is at most D/3 (`ScanResult.sigma`, the model's own per-point value)."""
    p = result.truth
    if result.grid is not None and result.index is not None and len(result.poses) > 1:
        origin = result.poses[result.index // result.grid[1]][:, :3, 3]
    else:
        origin = np.broadcast_to(result.poses[0][:3, 3], p.shape)
    ray = p - origin
    ray /= np.linalg.norm(ray, axis=1, keepdims=True)
    ids, dist = el.closest(p)
    on = dist < 1e-3          # on an element, not on a face no class needs
    cos_i = -(ray[on] * el.normal[ids[on]]).sum(axis=1)
    ok = cos_i > np.cos(np.radians(q["theta_max"]))
    sig = result.sigma[on] if result.sigma is not None else np.zeros(int(on.sum()))
    keep = el.patch >= 0
    rows = []
    for D in levels(q):
        good = ok & (sig <= q["noise_per_D"] * D)
        tri = np.bincount(ids[on][good], minlength=len(el))
        rows.append(np.bincount(el.patch[keep], weights=tri[keep], minlength=len(el.p_area)))
    return (np.array(rows) * scale).astype(np.float32)


def coverage(ns, reqs: list[Requirement]) -> dict:
    """Covered fraction of each requirement's area by one or more captures.
    An element is covered if *some single* capture covers it: counts are
    never summed across captures (the single-capture rule, §3.7.6)."""
    ns = [ns] if isinstance(ns, np.ndarray) else list(ns)
    out = {}
    for r in reqs:
        ok = np.zeros(len(r.elements), bool)
        for n in ns:
            ok |= r.covered(n)
        out[r.name] = r.fraction(ok)
    return out


# -- candidate views ---------------------------------------------------------------

@dataclass
class View:
    """One capture the arm could take: a snapshot pose, or a sweep (start pose,
    length; the sensor travels along its own +y)."""

    id: str
    sensor: str
    kind: str                  # "snapshot" or "sweep"
    pose: np.ndarray           # 4x4 sensor-to-module; for a sweep, its start
    length: float = 0.0        # mm, sweep only

    def seconds(self, q: dict) -> float:
        p = q["plan"]
        capture = p["t_snapshot"] if self.kind == "snapshot" else self.length / p["sweep_speed"]
        return p["t_move"] + capture

    def to_json(self) -> dict:
        return {"id": self.id, "sensor": self.sensor, "kind": self.kind,
                "pose_mm": np.round(self.pose, 6).tolist(), "length_mm": self.length}


def look_at(eye, target) -> np.ndarray:
    """Camera pose at `eye` with its +z towards `target`."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye
    z /= np.linalg.norm(z)
    ref = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.99 else np.array([1.0, 0.0, 0.0])
    x = np.cross(z, ref)
    x /= np.linalg.norm(x)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    pose[:3, 3] = eye
    return pose


def sweep_pose(target, along, look, standoff: float) -> np.ndarray:
    """Line-sensor start pose: travelling along `along`, looking along `look`
    (both unit, orthogonal), with `target` at `standoff` on its axis."""
    y = np.asarray(along, float) / np.linalg.norm(along)
    z = np.asarray(look, float)
    z = z - (z @ y) * y
    z /= np.linalg.norm(z)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([np.cross(y, z), y, z])
    pose[:3, 3] = np.asarray(target, float) - standoff * z
    return pose


def _rolled(along, roll_deg: float) -> np.ndarray:
    """Straight down, rolled about the sweep direction by `roll_deg`."""
    y = np.asarray(along, float)
    down = np.array([0.0, 0.0, -1.0])
    th = np.radians(roll_deg)
    # Rodrigues about y.
    return down * np.cos(th) + np.cross(y, down) * np.sin(th) + y * (y @ down) * (1 - np.cos(th))


def candidates(cfg: Config, q: dict) -> list[View]:
    sensors, _ = load_params()
    cq = q["candidates"]
    views: list[View] = []

    # Snapshots: a hemisphere about the module centre at the focus distance,
    # then close-ups of each quadrant.
    s = sensors["snapshot"]
    centre = np.array([0.0, 0.0, cfg.z_terminal_top])
    for el_deg in cq["snapshot"]["hemisphere_elevations"]:
        azs = [0.0] if el_deg >= 89.9 else np.linspace(0, 360, cq["snapshot"]["hemisphere_azimuths"],
                                                       endpoint=False)
        for az in azs:
            e, a = np.radians(el_deg), np.radians(az)
            eye = centre + s["focus_distance"] * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
            views.append(View(f"snap_e{el_deg:g}_a{az:g}", "snapshot", "snapshot", look_at(eye, centre)))
    near = cq["snapshot"]["near_distance"]
    for sx in (-1, 1):
        for sy in (-1, 1):
            t = np.array([sx * cfg.module_l / 4, sy * cfg.module_w / 4, cfg.z_terminal_top])
            views.append(View(f"snap_near_{sx:+d}{sy:+d}", "snapshot", "snapshot",
                              look_at(t + [0, 0, near], t)))
            out = np.array([sx, sy, 0.0]) / np.sqrt(2)
            views.append(View(f"snap_near_{sx:+d}{sy:+d}_tilt", "snapshot", "snapshot",
                              look_at(t + near * (0.5 * out + [0, 0, np.sqrt(0.75)]), t)))

    # Sweeps: lanes along X across the module, along Y over each end plate,
    # each level and rolled; and side-on along each housing wall.
    p = sensors["profilometer"]
    sw = cq["sweep"]
    ref, m = p["reference_distance"], sw["margin"]
    length_x, length_y = cfg.module_l + 2 * m, cfg.module_w + 2 * m
    for y0 in sw["lanes_y"]:
        z_t = cfg.z_wall_top if abs(y0) > cfg.cell_w / 2 else cfg.z_terminal_top
        for roll in sw["rolls"]:
            look = _rolled([1, 0, 0], roll)
            views.append(View(f"sweep_x_y{y0:+g}_r{roll:+d}", "profilometer", "sweep",
                              sweep_pose([-cfg.module_l / 2 - m, y0, z_t], [1, 0, 0], look, ref), length_x))
    end_x = cfg.module_l / 2 - cfg.end_plate_t / 2
    for x0 in (-end_x, end_x):
        for roll in sw["rolls"]:
            look = _rolled([0, 1, 0], roll)
            views.append(View(f"sweep_y_x{x0:+g}_r{roll:+d}", "profilometer", "sweep",
                              sweep_pose([x0, -cfg.module_w / 2 - m, cfg.z_wall_top], [0, 1, 0], look, ref),
                              length_y))
    if sw["side_on"]:
        z_mid = cfg.z_wall_top / 2
        for sy in (-1, 1):
            y_face = sy * cfg.module_w / 2
            views.append(View(f"sweep_side_y{sy:+d}", "profilometer", "sweep",
                              sweep_pose([-cfg.module_l / 2 - m, y_face, z_mid], [1, 0, 0], [0, -sy, 0], ref),
                              length_x))
        for sx in (-1, 1):
            x_face = sx * cfg.module_l / 2
            views.append(View(f"sweep_end_x{sx:+d}", "profilometer", "sweep",
                              sweep_pose([x_face, -cfg.module_w / 2 - m, z_mid], [0, 1, 0], [-sx, 0, 0], ref),
                              length_y))
    return views


def fixed_baseline(cfg: Config) -> View:
    """The Ranger3 station, exactly as scan.py and detection fly it."""
    sensors, _ = load_params()
    start = looking_down(-cfg.module_l / 2 - 5, 0.0, cfg.z_terminal_top + sensors["line_scan"]["reference_distance"])
    return View("fixed_line_scan", "line_scan", "sweep", start, cfg.module_l + 10)


def capture(view: View, scene: Scene, q: dict, cast_pitch: float | None) -> tuple[ScanResult, float]:
    """Simulate one view through the frozen sensor model; return the result
    and the factor that scales its counts to the real sensor's density."""
    sensors, _ = load_params()
    cfg = sensors[view.sensor]
    quiet = logging.getLogger("defectgen.scan")
    level = quiet.level
    quiet.setLevel(logging.WARNING)
    try:
        if view.kind == "snapshot":
            return snapshot(scene, view.pose, sensor=view.sensor), float(cfg["stride"] ** 2)
        if cast_pitch is None:
            return line_scan(scene, view.sensor, view.pose, view.length), 1.0
        rate = cfg["reference_rate"]
        r = line_scan(scene, view.sensor, view.pose, view.length, speed=cast_pitch * rate, rate=rate)
        return r, cast_pitch / cfg["profile_pitch"]
    finally:
        quiet.setLevel(level)


# -- evaluating candidates, with a cache -------------------------------------------

def _cache_key(stage: str, q: dict, el: Elements) -> str:
    h = hashlib.sha256()
    h.update(params_hash().encode())
    h.update(json.dumps(q, sort_keys=True).encode())
    h.update(stage.encode())
    h.update(np.round(el.area, 6).tobytes())
    return h.hexdigest()[:12]


def evaluate_views(views: list[View], el: Elements, scene: Scene, q: dict, cache: Path | None,
                   cast_pitch: float | None) -> dict:
    """Count vector per view, from disk when this exact setup was run before."""
    out = {}
    for i, v in enumerate(views, 1):
        f = cache / f"{v.id}.npy" if cache else None
        if f is not None and f.is_file():
            out[v.id] = np.load(f)
            continue
        t0 = time.perf_counter()
        r, scale = capture(v, scene, q, cast_pitch)
        out[v.id] = counts(r, el, q, scale)
        if f is not None:
            f.parent.mkdir(parents=True, exist_ok=True)
            np.save(f, out[v.id])
        log.info("  view %3d/%d %-26s %5.1f s  %8d points  on %6d patches",
                 i, len(views), v.id, time.perf_counter() - t0, len(r.points), int((out[v.id][-1] > 0).sum()))
    return out


# -- greedy set cover ----------------------------------------------------------------

@dataclass
class Step:
    view: View
    seconds: float
    gain: float               # mean class coverage this view added
    coverage: dict            # per class, cumulative after this view


def greedy(views: list[View], n: dict, reqs: list[Requirement], q: dict) -> list[Step]:
    """Repeatedly take the view that adds the most mean class coverage per
    second, until none adds more than min_gain. Each class weighs the same,
    however large its area."""
    live = [r for r in reqs if r.area > 0]
    cov = {v.id: [r.covered(n[v.id]) for r in live] for v in views}
    have = [np.zeros(len(r.elements), bool) for r in live]
    steps: list[Step] = []
    left = list(views)
    while left:
        best = None
        for v in left:
            gain = np.mean([r.fraction(c & ~h) for r, c, h in zip(live, cov[v.id], have)])
            rate = gain / v.seconds(q)
            if best is None or rate > best[0]:
                best = (rate, gain, v)
        _, gain, v = best
        if gain < q["plan"]["min_gain"]:
            break
        have = [h | c for h, c in zip(have, cov[v.id])]
        left.remove(v)
        steps.append(Step(v, v.seconds(q), float(gain), {r.name: r.fraction(h) for r, h in zip(live, have)}))
    return steps


# -- the whole stage -----------------------------------------------------------------

def run_stage(stage: str, out_dir: Path, q: dict | None = None, cfg: Config | None = None,
              use_cache: bool = True) -> dict:
    from .detect import Reference

    q = q or load_coverage_params()
    cfg = cfg or load()
    t0 = time.perf_counter()
    ref = Reference(stage, cfg)
    el = build_elements(ref.groups, q)
    observe(el, ref.scene, q)
    n_patch = el.make_patches(q["patch_size"])
    reqs = requirements(el, stage, q)
    log.info("%s: %d triangles, %.0f mm^2, %.0f mm^2 observable, in %d patches (%.0f s)", stage, len(el),
             el.area.sum(), el.area[el.observable].sum(), n_patch, time.perf_counter() - t0)
    for r in reqs:
        log.info("  required %-16s D %.2f mm -> spacing %.3f mm  %6d elements  %8.0f mm^2",
                 r.name, r.D, r.D / 2, len(r.elements), r.area)

    cache = out_dir / "cache" / f"{stage}_{_cache_key(stage, q, el)}" if use_cache else None
    fixed = fixed_baseline(cfg)
    log.info("fixed baseline")
    n_fixed = evaluate_views([fixed], el, ref.scene, q, cache, None)[fixed.id]
    views = candidates(cfg, q)
    log.info("%d candidate arm views", len(views))
    n = evaluate_views(views, el, ref.scene, q, cache, q["candidates"]["sweep"]["cast_pitch"])
    steps = greedy(views, n, reqs, q)

    result = {
        "stage": stage,
        "params_hash": params_hash(),
        "elements": int(len(el)), "observable": int(el.observable.sum()),
        "requirements": {r.name: {"D_mm": r.D, "spacing_mm": r.D / 2, "area_mm2": r.area,
                                  "members": r.members} for r in reqs},
        "fixed": coverage(n_fixed, reqs),
        "arm_plan": coverage([n[s.view.id] for s in steps], reqs),
        "arm_ceiling": coverage(list(n.values()), reqs),
        "arm_plus_fixed": coverage([n_fixed] + [n[s.view.id] for s in steps], reqs),
        "per_view_alone": {v.id: coverage(n[v.id], reqs) for v in views},
        "plan": [{"step": i + 1, **s.view.to_json(), "seconds": s.seconds, "gain": s.gain,
                  "coverage_after": s.coverage} for i, s in enumerate(steps)],
        "plan_seconds": float(sum(s.seconds for s in steps)),
        # The fixed pass costs only its own conveyor travel: profile pitch x rate.
        "fixed_seconds": float(fixed.length / (load_params()[0]["line_scan"]["profile_pitch"]
                                               * load_params()[0]["line_scan"]["reference_rate"])),
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / f"{stage}.json").write_text(json.dumps(result, indent=2))
    (out_dir / f"{stage}.plan.json").write_text(json.dumps(
        {"stage": stage, "frame": "module (pallet datum), mm", "params_hash": params_hash(),
         "note": "sensor poses: +z is the viewing direction; a sweep travels along its own +y",
         "views": result["plan"]}, indent=2))
    best = {r.name: max(n.items(), key=lambda kv: coverage(kv[1], [r])[r.name])[0] for r in reqs}
    P = len(el.p_area)

    def flags(sel):
        out = np.zeros((len(reqs), P), bool)
        for i, r in enumerate(reqs):
            out[i, r.elements[sel(r)]] = True
        return out
    plan_ns = [n[s.view.id] for s in steps]
    np.savez_compressed(out_dir / f"{stage}.patches.npz", centroid=el.p_centroid, normal=el.p_normal,
                        area=el.p_area, classes=np.array([r.name for r in reqs]),
                        required=flags(lambda r: np.ones(len(r.elements), bool)),
                        fixed=flags(lambda r: r.covered(n_fixed)),
                        arm=flags(lambda r: np.any([r.covered(x) for x in plan_ns], axis=0)
                                  if plan_ns else np.zeros(len(r.elements), bool)))
    result["best_single_view"] = best
    log.info("done in %.0f s", time.perf_counter() - t0)
    return result


def format_result(res: dict) -> str:
    rows = [f"coverage ({res['stage']}), % of required area seen well enough to measure, single capture",
            f"  {'class':16s} {'D mm':>5s} {'fixed':>7s} {'arm':>7s} {'ceiling':>8s} {'both':>7s}"]
    for name, rq in res["requirements"].items():
        def pc(k):
            v = res[k][name]
            return "    n/a" if v != v else f"{100 * v:6.1f}%"
        rows.append(f"  {name:16s} {rq['D_mm']:5.2f} {pc('fixed'):>7s} {pc('arm_plan'):>7s} "
                    f"{pc('arm_ceiling'):>8s} {pc('arm_plus_fixed'):>7s}")
    rows.append(f"  fixed pass: 1 sweep, {res['fixed_seconds']:.1f} s of conveyor travel")
    rows.append(f"  arm plan: {len(res['plan'])} captures, {res['plan_seconds']:.1f} s "
                f"(moves at an assumed fixed time; reachability and real motion are Phase 2)")
    for s in res["plan"]:
        rows.append(f"    {s['step']:2d}. {s['id']:26s} {s['seconds']:5.1f} s  +{100 * s['gain']:5.1f} % mean")
    return "\n".join(rows)


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(prog="python -m defectgen.coverage", description=__doc__.split("\n")[0])
    ap.add_argument("--stage", nargs="+", default=["pre_busbar", "pre_weld", "post_weld"])
    ap.add_argument("--out", type=Path, default=Path("data/coverage"))
    ap.add_argument("--no-cache", dest="cache", action="store_false")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s.%(msecs)03d %(message)s", datefmt="%H:%M:%S")
    for stage in args.stage:
        res = run_stage(stage, args.out, use_cache=args.cache)
        log.info("\n%s", format_result(res))
        log.info("-> %s/%s.json, %s.plan.json, %s.patches.npz", args.out, stage, stage, stage)


if __name__ == "__main__":
    main()

"""Ray-cast sensor models: what each sensor actually returns (task 1.3).

Three sensors, two models (project_plan.md §4B.3):

* **Laser-line triangulation** -- the wrist profilometer (LJ-X8200 class) and
  the fixed line-scan station (Ranger3 class). Each profile is a fan of rays
  from the laser; a lit point is kept only if the camera, offset along the scan
  direction by the triangulation angle, can also see it. A scan is profiles
  swept along the part, exactly as on a real cell.
* **Structured-light snapshot** -- the wrist area camera (Zivid 2+ M60 class).
  One ray per (strided) pixel from the camera; a point is kept only if the
  projector, `baseline` to the side, also reaches it and it lies inside the
  projector's field of view.

Then, for every point, what a bare ray cast leaves out:

* **Reflectance.** The light returned toward the camera is computed per
  material (configs/materials.yaml): a diffuse part plus a rough-metal specular
  lobe, at the sensor's wavelength band. Too little and the point drops (bare
  copper seen off the mirror angle); too much and the spot saturates and drops
  (seen on it). This is the specular dropout the plan calls the biggest
  sim-to-real gap in the project.
* **Grazing angle and range window**, from the sensor config.
* **Noise** from the datasheet, growing as the returned signal weakens, and
  **quantisation** of the height output.

Every parameter comes from configs/sensors.yaml and configs/materials.yaml,
tagged S/D/A, and `params_hash` fingerprints both. The hash is pinned in
tests/test_scan.py and written into every scan, so a result can always be
traced to the exact sensor model that produced it.

Open3D's RaycastingScene does the casting (float32 is ample here: sensor noise
is microns, float32 error at 400 mm is hundredths of a micron).
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

CONFIG_DIR = Path(__file__).parent / "configs"
log = logging.getLogger("defectgen.scan")


# -- parameters ------------------------------------------------------------

def load_params() -> tuple[dict, dict]:
    sensors = yaml.safe_load((CONFIG_DIR / "sensors.yaml").read_text())
    materials = yaml.safe_load((CONFIG_DIR / "materials.yaml").read_text())
    return sensors, materials


def params_hash() -> str:
    """SHA-256 over the parsed sensor and material parameters, so comments and
    formatting can change but no number can without changing the hash."""
    sensors, materials = load_params()
    canon = json.dumps({"sensors": sensors, "materials": materials}, sort_keys=True)
    return hashlib.sha256(canon.encode()).hexdigest()


# -- scene -----------------------------------------------------------------

@dataclass
class Scene:
    """The module to scan: one Open3D raycasting scene plus a material per
    triangle, so every hit knows what it landed on."""

    raycaster: object
    material_names: list[str]
    face_material: list[np.ndarray]  # per geometry id: material index per face

    @classmethod
    def from_groups(cls, groups: dict[str, trimesh.Trimesh], materials: dict | None = None):
        import open3d as o3d

        if materials is None:
            materials = load_params()[1]
        names = sorted(materials["materials"])
        index = {n: i for i, n in enumerate(names)}
        scene = o3d.t.geometry.RaycastingScene()
        face_material = []
        for group, mesh in sorted(groups.items()):
            rule = materials["parts"].get(group)
            if rule is None:
                raise KeyError(f"no material for body group {group!r} in materials.yaml")
            if isinstance(rule, dict):
                up = mesh.face_normals[:, 2] > 0.9
                mats = np.where(up, index[rule["top"]], index[rule["other"]])
            else:
                mats = np.full(len(mesh.faces), index[rule])
            gid = scene.add_triangles(
                o3d.core.Tensor(mesh.vertices.astype(np.float32)),
                o3d.core.Tensor(mesh.faces.astype(np.uint32)),
            )
            assert gid == len(face_material)
            face_material.append(mats)
        return cls(scene, names, face_material)

    @classmethod
    def from_run(cls, run_dir: Path | str, stage: str = "post_weld", without=None):
        """From what `python -m defectgen` wrote: the per-group STLs are the
        contract, so a scan never needs to regenerate the module.

        `without` drops body groups. By default the post-weld scan leaves out
        the CMC and cover: inspection (module_spec §8 step 10) happens before
        they are placed (step 11), so the sensors see the welds."""
        if without is None:
            without = without_groups(stage)
        groups = {
            p.stem: trimesh.load_mesh(p)
            for p in sorted((Path(run_dir) / stage).glob("*.stl"))
            if p.stem not in without
        }
        if not groups:
            raise FileNotFoundError(f"no body groups in {Path(run_dir) / stage}")
        return cls.from_groups(groups)

    def cast(self, origins: np.ndarray, directions: np.ndarray):
        """Hit distance (inf on a miss), unit normal and material per ray."""
        import open3d as o3d

        rays = np.hstack([origins, directions]).astype(np.float32)
        out = self.raycaster.cast_rays(o3d.core.Tensor(rays))
        t = out["t_hit"].numpy().astype(float)
        normal = out["primitive_normals"].numpy().astype(float)
        gid = out["geometry_ids"].numpy()
        pid = out["primitive_ids"].numpy()
        hit = np.isfinite(t)
        mat = np.full(len(t), -1)
        for g in np.unique(gid[hit]):
            sel = hit & (gid == g)
            mat[sel] = self.face_material[int(g)][pid[sel]]
        # Face the normal toward the incoming ray.
        flip = (normal * directions).sum(axis=1) > 0
        normal[flip] *= -1
        return t, normal, mat

    def visible(self, points: np.ndarray, eye: np.ndarray, normals: np.ndarray) -> np.ndarray:
        """True where nothing lies between each point and `eye` (one point or
        one per point). Starts a hair off the surface along the normal so a
        point does not occlude itself."""
        import open3d as o3d

        eye = np.broadcast_to(eye, points.shape)
        start = points + normals * 1e-3
        # Unnormalised direction: Open3D measures t in units of its length,
        # so t in [0, 1) is exactly "between the point and the eye".
        rays = np.hstack([start, eye - start]).astype(np.float32)
        blocked = self.raycaster.test_occlusions(o3d.core.Tensor(rays), tnear=0.0, tfar=1.0).numpy()
        return ~blocked


def without_groups(stage: str, include=()) -> tuple:
    """Body groups a scan of `stage` leaves out by default: after welding the
    CMC and cover are not on yet when inspection happens (module_spec §8,
    steps 10 and 11). `include` puts any of them back."""
    default = ("cmc", "cover") if stage == "post_weld" else ()
    return tuple(g for g in default if g not in include)


# -- reflectance -----------------------------------------------------------

def signal(materials: dict, names: list[str], mat: np.ndarray, normal, to_light, to_cam, band: str):
    """Light returned toward the camera, relative to a white diffuse surface
    seen square on (= 1). Lambert plus a Beckmann specular lobe with constant
    normal-incidence reflectance; the geometry term is left out (it matters
    only near grazing, which the incidence cut removes anyway)."""
    table = materials["materials"]
    kd = np.array([table[n]["kd"] for n in names])[mat]
    alpha = np.array([table[n]["alpha"] for n in names])[mat]
    r0 = np.array([table[n]["r0"][band] for n in names])[mat]

    cos_l = np.clip((normal * to_light).sum(axis=1), 1e-6, 1.0)
    cos_v = np.clip((normal * to_cam).sum(axis=1), 1e-6, 1.0)
    half = to_light + to_cam
    half /= np.linalg.norm(half, axis=1, keepdims=True)
    cos_h = np.clip((normal * half).sum(axis=1), 1e-6, 1.0)
    tan2 = (1.0 - cos_h**2) / cos_h**2
    beckmann = np.exp(-tan2 / alpha**2) / (np.pi * alpha**2 * cos_h**4)
    specular = r0 * beckmann / (4.0 * cos_l * cos_v)
    return np.pi * (kd / np.pi + specular) * cos_l


# -- output ----------------------------------------------------------------

@dataclass
class ScanResult:
    """A point cloud as the sensor returned it, plus what a detector is not
    allowed to see but an evaluation needs: the noise-free hit and the material
    it came from, and the reason every dropped ray was dropped."""

    sensor: str
    points: np.ndarray                 # (N, 3) module frame, with noise
    truth: np.ndarray                  # (N, 3) noise-free hits
    signal: np.ndarray                 # (N,) returned light, white-diffuse = 1
    material: np.ndarray               # (N,) material index
    material_names: list[str]
    poses: np.ndarray                  # (P, 4, 4) sensor-to-module, one per profile/shot
    dropped: dict = field(default_factory=dict)  # reason -> count
    params_hash: str = ""
    image: np.ndarray | None = None    # (H, W, 3) uint8, snapshot only: the 2D picture
    # reason -> rays dropped for it, per material index (misses hit nothing)
    dropped_by_material: dict = field(default_factory=dict)
    # How the sensor was used: speed, profile rate, pitch, length, scan time.
    # Recorded with every scan but NOT in params_hash -- the hash fixes what the
    # sensor is; these are how it was driven, and are meant to be varied.
    settings: dict = field(default_factory=dict)
    rays: int = 0
    # The organised layout a real sensor returns: which ray each point came
    # from, as a flat index into `grid` -- (profiles, points per profile) for a
    # line sensor, (rows, columns) of cast pixels for the snapshot. Detection
    # (detect.py) compares a scan ray by ray with the same rays cast at the
    # clean reference. Adds no information about the scene: an LJ-X profile
    # and a Zivid organised cloud both carry it.
    index: np.ndarray | None = None    # (N,) int64
    grid: tuple | None = None
    # What was scanned: run directory, stage, groups put back (--include) and
    # `labels_fingerprint` of the module at scan time. Detection checks it, so
    # a scan left over from before the module was regenerated is refused
    # instead of being scored against the new labels.
    source: dict = field(default_factory=dict)
    # The 1-sigma noise the model put on each point, along its measuring
    # direction (distance and returned light both count). In memory only, not
    # saved: coverage (coverage.py) uses it for §3.7.3's sigma <= D/3 half.
    sigma: np.ndarray | None = None

    def save(self, path: Path | str) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path, points=self.points, truth=self.truth, signal=self.signal,
            material=self.material, poses=self.poses,
            index=self.index if self.index is not None else np.zeros(0, np.int64),
            meta=json.dumps({
                "sensor": self.sensor, "material_names": self.material_names,
                "dropped": self.dropped, "params_hash": self.params_hash,
                "dropped_by_material": self.dropped_by_material,
                "settings": self.settings, "rays": self.rays,
                "grid": list(self.grid) if self.grid is not None else None,
                "source": self.source,
            }),
        )
        trimesh.PointCloud(self.points).export(path.with_suffix(".ply"))
        if self.image is not None:
            from PIL import Image

            Image.fromarray(self.image).save(path.with_name(path.stem + "_2d.png"))
        return path

    @classmethod
    def load(cls, path: Path | str) -> "ScanResult":
        """Read back what `save` wrote. Scans saved before the ray index was
        recorded come back with `index=None`; rescan them for detection."""
        z = np.load(path)
        meta = json.loads(str(z["meta"]))
        index = z["index"] if "index" in z.files and len(z["index"]) == len(z["points"]) else None
        grid = tuple(meta["grid"]) if meta.get("grid") else None
        return cls(meta["sensor"], z["points"], z["truth"], z["signal"], z["material"],
                   meta["material_names"], z["poses"], meta["dropped"], meta["params_hash"],
                   dropped_by_material=meta.get("dropped_by_material", {}),
                   settings=meta.get("settings", {}), rays=meta.get("rays", 0),
                   index=index, grid=grid, source=meta.get("source", {}))

    @staticmethod
    def read_meta(path: Path | str) -> dict:
        """Only the metadata of a saved scan, without loading the points
        (npz members load lazily), plus `has_index`."""
        with np.load(path) as z:
            meta = json.loads(str(z["meta"]))
            meta["has_index"] = bool(meta.get("grid")) and "index" in z.files
        return meta


def labels_fingerprint(run: Path | str, stage: str) -> str:
    """Fingerprint of a generated module at one stage: the hash of its labels
    file, which records every placed part and injected defect. Empty if the
    run has no labels for that stage."""
    f = Path(run) / f"{stage}.labels.json"
    return hashlib.sha256(f.read_bytes()).hexdigest()[:16] if f.is_file() else ""


def _drop(dropped: dict, by_mat: dict, reason: str, mask: np.ndarray, mat: np.ndarray, n_mat: int):
    """Count rays dropped for `reason`, in total and per material."""
    dropped[reason] = dropped.get(reason, 0) + int(mask.sum())
    m = mat[mask]
    counts = np.bincount(m[m >= 0], minlength=n_mat)
    by_mat[reason] = (np.asarray(by_mat.get(reason, np.zeros(n_mat, int))) + counts).tolist()


def _finish(name, cfg, materials, scene, pts, normal, mat, to_light, to_cam, along, normals,
            poses, dropped, sigma, sigma_x_dir=None, sigma_x=0.0, by_mat=None, index=None):
    """Shared tail: reflectance cut, noise, quantisation. `normals` is (2, N)
    standard-normal draws fixed per ray upstream, so the noise on a point does
    not depend on how a sweep was split into chunks."""
    # Exposure: a line sensor run faster than its reference rate integrates
    # each profile for proportionally less time, so less light per point --
    # more dark dropout and noisier points, less glare. 1.0 for the snapshot.
    s = signal(materials, scene.material_names, mat, normal, to_light, to_cam, cfg["band"])
    s = s * cfg.get("_exposure", 1.0)
    dark, glare = s < cfg["min_signal"], s > cfg["saturation"]
    by_mat = {} if by_mat is None else by_mat
    n_mat = len(scene.material_names)
    _drop(dropped, by_mat, "dark", dark, mat, n_mat)
    _drop(dropped, by_mat, "glare", glare, mat, n_mat)
    keep = ~dark & ~glare
    pts, s, mat, along, sigma = pts[keep], s[keep], mat[keep], along[keep], sigma[keep]
    if index is not None:
        index = index[keep]
    zn, xn = normals[0][keep], normals[1][keep]
    if sigma_x_dir is not None:
        sigma_x_dir = sigma_x_dir[keep]

    scale = np.sqrt(np.maximum(cfg["noise_ref_signal"] / s, 1.0))
    noisy = pts + (zn * sigma * np.minimum(scale, 10.0))[:, None] * along
    if sigma_x_dir is not None and sigma_x:
        noisy += (xn * sigma_x)[:, None] * sigma_x_dir
    # Quantise the measured range along the measurement direction.
    q = cfg["quantum_z"]
    depth = (noisy * along).sum(axis=1)
    noisy += ((np.round(depth / q) * q) - depth)[:, None] * along

    return ScanResult(name, noisy, pts, s, mat, scene.material_names, np.array(poses), dropped,
                      params_hash(), dropped_by_material=by_mat, index=index,
                      sigma=sigma * np.minimum(scale, 10.0))


# -- laser line ------------------------------------------------------------

def line_scan(scene: Scene, sensor: str, start: np.ndarray, length: float, seed: int = 0,
              chunk: int = 400, speed: float | None = None, rate: float | None = None) -> ScanResult:
    """Sweep a laser-line sensor `length` mm along its own +y from `start`
    (4x4 sensor-to-module). Each profile: a fan of `points` rays spanning
    `line_width` at the reference distance, in the sensor's x-z plane. The
    camera sits back along -y so it views the line at `triangulation_angle`.

    Motion: the sensor (or, for the fixed station, the conveyor) moves at
    `speed` mm/s while the sensor fires `rate` profiles per second, so
    profiles land speed/rate mm apart -- slower or faster sampling means
    denser points. Give both or neither; neither uses the frozen
    `profile_pitch`. The motion itself is ideal for now (no vibration).

    Profiles are processed `chunk` at a time to bound memory; noise is fixed
    per (seed, profile), so the result does not depend on the chunking."""
    sensors, materials = load_params()
    cfg = sensors[sensor]
    if (speed is None) != (rate is None):
        raise ValueError("give both speed (mm/s) and rate (profiles/s), or neither")
    if rate is not None and rate > cfg["max_rate"]:
        raise ValueError(f"{sensor} cannot fire {rate:g} profiles/s; its maximum is {cfg['max_rate']:g}")
    pitch = speed / rate if speed else cfg["profile_pitch"]
    exposure = cfg["reference_rate"] / rate if rate else 1.0
    cfg = dict(cfg, _exposure=exposure)
    steps = np.arange(0.0, length + 1e-9, pitch)
    poses = []
    for s in steps:
        pose = start.copy()
        pose[:3, 3] = start[:3, 3] + start[:3, 1] * s
        poses.append(pose)
    settings = {"length_mm": length, "pitch_mm": pitch, "profiles": len(poses),
                "points_per_profile": cfg["points"], "seed": seed}
    if speed:
        settings.update(speed_mm_s=speed, rate_hz=rate, scan_time_s=length / speed,
                        exposure_vs_reference=exposure)
    rays = len(poses) * cfg["points"]
    log.info("plan: %s sweep %.1f mm at %.3f mm pitch -> %d profiles x %d points = %.1f M rays%s",
             sensor, length, pitch, len(poses), cfg["points"], rays / 1e6,
             f", {speed:g} mm/s at {rate:g} Hz = {length / speed:.2f} s of sensor time, "
             f"exposure x{exposure:.2f} of the reference" if speed else "")

    parts, dropped, by_mat = [], {}, {}
    t0, kept = time.perf_counter(), 0
    for a in range(0, len(poses), chunk):
        part = _line_profiles(scene, cfg, materials, poses[a:a + chunk], seed, a, by_mat)
        parts.append(part)
        kept += len(part.points)
        for k, v in part.dropped.items():
            dropped[k] = dropped.get(k, 0) + v
        done = min(a + chunk, len(poses))
        cast = done * cfg["points"]
        log.info("  profiles %5d/%d (%3.0f%%) | rays %6.2f M | kept %6.2f M (%4.1f%%) | lost: %s | %.1f M rays/s",
                 done, len(poses), 100 * done / len(poses), cast / 1e6, kept / 1e6, 100 * kept / cast,
                 " ".join(f"{k} {v / 1e3:.1f}k" for k, v in dropped.items() if v),
                 cast / 1e6 / max(time.perf_counter() - t0, 1e-9))
    return ScanResult(
        sensor,
        np.vstack([p.points for p in parts]),
        np.vstack([p.truth for p in parts]),
        np.concatenate([p.signal for p in parts]),
        np.concatenate([p.material for p in parts]),
        scene.material_names,
        np.array(poses),
        dropped,
        params_hash(),
        dropped_by_material=by_mat,
        settings=settings,
        rays=rays,
        index=np.concatenate([p.index for p in parts]),
        grid=(len(poses), cfg["points"]),
        sigma=np.concatenate([p.sigma for p in parts]),
    )


def _line_profiles(scene: Scene, cfg: dict, materials: dict, poses: list, seed: int, first: int,
                   by_mat: dict) -> ScanResult:
    d0, n = cfg["reference_distance"], cfg["points"]
    xs = np.linspace(-cfg["line_width"] / 2, cfg["line_width"] / 2, n)
    fan = np.column_stack([xs, np.zeros(n), np.full(n, d0)])
    fan /= np.linalg.norm(fan, axis=1, keepdims=True)
    cam_local = np.array([0.0, -d0 * np.tan(np.radians(cfg["triangulation_angle"])), 0.0])

    rot = np.array([p[:3, :3] for p in poses])
    org = np.array([p[:3, 3] for p in poses])
    origins = np.repeat(org, n, axis=0)
    dirs = np.einsum("pij,nj->pni", rot, fan).reshape(-1, 3)
    cam = np.repeat(np.einsum("pij,j->pi", rot, cam_local) + org, n, axis=0)
    axis = np.repeat(rot[:, :, 2], n, axis=0)
    xdir = np.repeat(rot[:, :, 0], n, axis=0)

    t, normal, mat = scene.cast(origins, dirs)
    n_mat = len(scene.material_names)
    dropped = {"miss": int((~np.isfinite(t)).sum())}
    hit = np.isfinite(t)
    pts = origins + dirs * np.where(hit, t, 0.0)[:, None]
    depth = ((pts - origins) * axis).sum(axis=1)
    keep = hit & (np.abs(depth - d0) <= cfg["z_range"])
    _drop(dropped, by_mat, "range", hit & ~keep, mat, n_mat)

    to_light = -dirs
    to_cam = cam - pts
    to_cam /= np.linalg.norm(to_cam, axis=1, keepdims=True)
    cut = np.cos(np.radians(cfg["max_incidence"]))
    grazing = keep & (((normal * to_light).sum(1) < cut) | ((normal * to_cam).sum(1) < cut))
    _drop(dropped, by_mat, "grazing", grazing, mat, n_mat)
    keep &= ~grazing
    idx = np.where(keep)[0]
    seen = scene.visible(pts[idx], cam[idx], normal[idx])
    shadow = np.zeros(len(t), bool)
    shadow[idx[~seen]] = True
    _drop(dropped, by_mat, "shadow", shadow, mat, n_mat)
    idx = idx[seen]

    # Noise draws fixed per (seed, profile): independent of chunking.
    normals = np.concatenate(
        [np.random.default_rng([seed, first + k]).standard_normal((2, n)) for k in range(len(poses))],
        axis=1,
    )[:, idx]
    # Triangulation noise grows with range: sigma_z holds at the reference
    # distance and scales by (distance / reference) ** exponent.
    sigma = cfg["sigma_z"] * (depth[idx] / d0) ** cfg["noise_distance_exponent"]
    return _finish("", cfg, materials, scene, pts[idx], normal[idx], mat[idx], to_light[idx],
                   to_cam[idx], axis[idx], normals, poses, dropped, sigma, xdir[idx], cfg["sigma_x"],
                   by_mat=by_mat, index=idx.astype(np.int64) + first * n)


# -- structured light ------------------------------------------------------

def snapshot(scene: Scene, pose: np.ndarray, sensor: str = "snapshot", seed: int = 0) -> ScanResult:
    """One stop-and-shoot capture from `pose` (4x4 camera-to-module, z along
    the optical axis). The projector sits `baseline` along the camera's -x."""
    sensors, materials = load_params()
    cfg = sensors[sensor]
    w, h = (r // cfg["stride"] for r in cfg["resolution"])
    tx, ty = np.tan(np.radians(cfg["hfov"] / 2)), np.tan(np.radians(cfg["vfov"] / 2))
    u, v = np.meshgrid(np.linspace(-tx, tx, w), np.linspace(-ty, ty, h))
    local = np.column_stack([u.ravel(), v.ravel(), np.ones(u.size)])
    local /= np.linalg.norm(local, axis=1, keepdims=True)
    rot, eye = pose[:3, :3], pose[:3, 3]
    dirs = local @ rot.T
    origins = np.broadcast_to(eye, dirs.shape).copy()
    projector = eye - rot[:, 0] * cfg["baseline"]
    log.info("plan: %s one shot, %d x %d rays (stride %d of %d x %d), camera at (%.0f, %.0f, %.0f) mm",
             sensor, w, h, cfg["stride"], *cfg["resolution"], *eye)

    t0 = time.perf_counter()
    t, normal, mat = scene.cast(origins, dirs)
    log.info("  cast %.2f M rays in %.2f s", len(dirs) / 1e6, time.perf_counter() - t0)
    n_mat, by_mat = len(scene.material_names), {}
    dropped = {"miss": int((~np.isfinite(t)).sum())}
    hit = np.isfinite(t)
    pts = origins + dirs * np.where(hit, t, 0.0)[:, None]
    depth = (pts - eye) @ rot[:, 2]
    lo, hi = cfg["working_distance"]
    keep = hit & (depth >= lo) & (depth <= hi)
    _drop(dropped, by_mat, "range", hit & ~keep, mat, n_mat)

    to_cam = -dirs
    to_proj = projector - pts
    to_proj /= np.linalg.norm(to_proj, axis=1, keepdims=True)
    # Inside the projector's field of view (projector looks along the camera axis).
    # World to camera axes is `@ rot` for row vectors. This read `@ rot.T` until
    # 7 Nov 2026, which only mattered for tilted views: every snapshot until then
    # looked straight down, where `looking_down`'s rotation is its own transpose.
    in_proj = (pts - projector) @ rot
    px = np.abs(in_proj[:, 0] / np.maximum(in_proj[:, 2], 1e-9)) <= np.tan(np.radians(cfg["projector_hfov"] / 2))
    py = np.abs(in_proj[:, 1] / np.maximum(in_proj[:, 2], 1e-9)) <= np.tan(np.radians(cfg["projector_vfov"] / 2))
    _drop(dropped, by_mat, "outside_projector", keep & ~(px & py), mat, n_mat)
    keep &= px & py

    cut = np.cos(np.radians(cfg["max_incidence"]))
    grazing = keep & (((normal * to_proj).sum(1) < cut) | ((normal * to_cam).sum(1) < cut))
    _drop(dropped, by_mat, "grazing", grazing, mat, n_mat)
    keep &= ~grazing
    idx = np.where(keep)[0]
    lit = scene.visible(pts[idx], projector, normal[idx])
    shadow = np.zeros(len(t), bool)
    shadow[idx[~lit]] = True
    _drop(dropped, by_mat, "shadow", shadow, mat, n_mat)
    idx = idx[lit]

    sigma = cfg["point_precision"] * (depth[idx] / cfg["focus_distance"]) ** cfg["precision_exponent"]
    normals = np.random.default_rng(seed).standard_normal((2, len(dirs)))[:, idx]
    result = _finish(sensor, cfg, materials, scene, pts[idx], normal[idx], mat[idx], to_proj[idx],
                     to_cam[idx], -to_cam[idx], normals, [pose], dropped, sigma, by_mat=by_mat,
                     index=idx.astype(np.int64))
    result.grid = (h, w)
    result.rays = len(dirs)
    result.settings = {"pose_xyz_mm": [float(v) for v in eye], "rays": len(dirs), "seed": seed,
                       "stride": cfg["stride"]}
    log.info("  rendering the 2D picture")
    result.image = render_2d(scene, materials, cfg, hit, pts, normal, mat, to_proj, to_cam,
                             projector, (h, w))
    return result


def render_2d(scene, materials, cfg, hit, pts, normal, mat, to_light, to_cam, projector, shape,
              exposure: float | str = 1.0):
    """The snapshot camera's 2D picture: every pixel that hit something, lit
    by the projector (the Zivid uses it as the flash for its 2D capture), in
    the surface's display colour (configs/appearance.yaml), scaled by the light
    the reflectance model says comes back. Shadowed pixels get ambient only.
    Bright metal can clip to white -- the same glare that drops 3D points.

    `exposure` scales the light before display. "auto" sets it so the
    brightest 0.5% of lit pixels just reach white, as a camera's auto
    exposure would; 1.0 shows the raw model, glare clipping included."""
    look = yaml.safe_load((CONFIG_DIR / "appearance.yaml").read_text())
    colours = np.array([look["colours"][n] for n in scene.material_names])
    img = np.zeros((len(pts), 3))
    idx = np.where(hit)[0]
    lit = scene.visible(pts[idx], projector, normal[idx])
    light = signal(materials, scene.material_names, mat[idx], normal[idx], to_light[idx],
                   to_cam[idx], cfg["band"])
    light = np.where(lit, light, 0.0) + look["ambient"]
    if exposure == "auto":
        exposure = 1.0 / max(float(np.percentile(light, 99.5)), 1e-6)
    img[idx] = colours[mat[idx]] * (light * exposure)[:, None]
    # Linear to display: clip, then sRGB-like gamma.
    img = np.clip(img, 0.0, 1.0) ** (1 / 2.2)
    return (img.reshape(*shape, 3) * 255).astype(np.uint8)


# -- standard poses --------------------------------------------------------

def looking_down(x: float, y: float, z: float, scan_along_x: bool = True) -> np.ndarray:
    """A sensor at (x, y, z) looking straight down (-Z), scanning along module
    +X (or +Y). Sensor z = -module Z; sensor y = the scan direction."""
    pose = np.eye(4)
    ydir = np.array([1.0, 0.0, 0.0]) if scan_along_x else np.array([0.0, 1.0, 0.0])
    zdir = np.array([0.0, 0.0, -1.0])
    xdir = np.cross(ydir, zdir)
    pose[:3, :3] = np.column_stack([xdir, ydir, zdir])
    pose[:3, 3] = (x, y, z)
    return pose


def summary(result: ScanResult) -> dict:
    """What a run did, in numbers: where the rays went, per reason and per
    material, and how noisy the kept points really are."""
    names = result.material_names
    rays = result.rays or (len(result.points) + sum(result.dropped.values()))
    err = result.points - result.truth
    per_mat = {}
    for i, name in enumerate(names):
        kept = int((result.material == i).sum())
        lost = {r: int(c[i]) for r, c in result.dropped_by_material.items() if c[i]}
        if not kept and not lost:
            continue
        sel = result.material == i
        per_mat[name] = {
            "kept": kept,
            "lost": lost,
            "kept_pct": round(100 * kept / max(kept + sum(lost.values()), 1), 1),
            # Height error (module Z) -- what a height or gap measurement feels.
            "noise_um": round(float(err[sel][:, 2].std()) * 1000, 2) if kept else None,
            "median_signal": round(float(np.median(result.signal[sel])), 3) if kept else None,
        }
    return {
        "sensor": result.sensor,
        "params_hash": result.params_hash,
        "settings": result.settings,
        "rays": rays,
        "kept": len(result.points),
        "kept_pct": round(100 * len(result.points) / max(rays, 1), 1),
        "dropped": result.dropped,
        "by_material": per_mat,
    }


def _report(info: dict) -> None:
    log.info("result: %s kept %s of %s rays (%.1f%%)", info["sensor"], f"{info['kept']:,}",
             f"{info['rays']:,}", info["kept_pct"])
    for reason, n in info["dropped"].items():
        if n:
            log.info("  lost to %-18s %12s  (%4.1f%%)", reason, f"{n:,}", 100 * n / info["rays"])
    log.info("  %-20s %10s %7s %10s %8s  lost by reason", "material", "kept", "kept%", "Z noise um", "signal")
    for name, m in sorted(info["by_material"].items(), key=lambda kv: -kv[1]["kept"]):
        lost = ", ".join(f"{r} {n:,}" for r, n in m["lost"].items())
        log.info("  %-20s %10s %6.1f%% %10s %8s  %s", name, f"{m['kept']:,}", m["kept_pct"],
                 "-" if m["noise_um"] is None else f"{m['noise_um']:.2f}",
                 "-" if m["median_signal"] is None else f"{m['median_signal']:.3f}", lost)


def main() -> None:
    import argparse

    from . import scanlog
    from .config import load

    p = argparse.ArgumentParser(prog="python -m defectgen.scan", description=__doc__.split("\n")[0])
    p.add_argument("run", type=Path, help="a defectgen --out directory")
    p.add_argument("--stage", default="post_weld")
    p.add_argument("--sensor", choices=["line_scan", "profilometer", "snapshot", "photos"], default="line_scan",
                   help="photos = the four Zivid 2D quadrant photos (same as python -m defectgen.photos)")
    p.add_argument("--include", nargs="+", default=[], metavar="GROUP",
                   help="put back groups the stage leaves out for inspection (post_weld: cmc cover)")
    p.add_argument("--no-stitch", dest="stitch", action="store_false",
                   help="photos: skip the stitched whole-module image")
    p.add_argument("--speed", type=float, default=None,
                   help="line sensors: travel speed in mm/s (robot sweep, or conveyor for line_scan)")
    p.add_argument("--rate", type=float, default=None,
                   help="line sensors: profiles per second; pitch = speed / rate. Faster than the "
                        "sensor's reference_rate means a shorter exposure and less light per point")
    p.add_argument("--distance", type=float, default=None,
                   help="line sensors: height of the sensor above the terminal tops, mm "
                        "(default: its reference distance). Farther = wider line, coarser and noisier")
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--log-dir", type=Path, default=scanlog.LOG_ROOT,
                   help="each run gets its own folder here (scanlog.py)")
    p.add_argument("--quiet", action="store_true", help="only the final line")
    args = p.parse_args()
    if args.sensor != "snapshot" and (args.speed is None) != (args.rate is None):
        p.error("give both --speed and --rate, or neither (then the frozen profile_pitch is used)")

    folder = scanlog.start(args.run, args.stage, args.sensor, args.log_dir, args.quiet)
    if args.sensor == "photos":
        from .photos import capture

        log.info("defectgen.scan photos on %s / %s -> %s", args.run, args.stage, folder)
        t0 = time.perf_counter()
        info = capture(args.run, args.stage, folder, stitch=args.stitch, include=args.include)
        scanlog.write_summary(folder, info)
        log.warning("done in %.1f s: %d photos%s, %d defect close-ups -> %s", time.perf_counter() - t0,
                    len(info["shots"]), " + stitched" if args.stitch else "", len(info["close_ups"]), folder)
        return

    cfg = load()
    sensors, materials = load_params()
    sc = sensors[args.sensor]
    log.info("defectgen.scan %s on %s / %s", args.sensor, args.run, args.stage)
    log.info("sensor model: params %s (configs/sensors.yaml + materials.yaml, frozen)", params_hash()[:12])
    log.info("  %s", ", ".join(f"{k}={v}" for k, v in sc.items()))

    t0 = time.perf_counter()
    scene = Scene.from_run(args.run, args.stage, without=without_groups(args.stage, args.include))
    log.info("scene: loaded in %.1f s; materials in use: %s", time.perf_counter() - t0,
             ", ".join(scene.material_names))

    top = cfg.z_wall_top + 2.0
    t0 = time.perf_counter()
    if args.sensor == "snapshot":
        result = snapshot(scene, looking_down(0.0, 0.0, top + sc["focus_distance"]), seed=args.seed)
    else:
        d = args.distance if args.distance is not None else sc["reference_distance"]
        off = d - sc["reference_distance"]
        if abs(off) > sc["z_range"]:
            log.warning("distance %.0f mm puts the terminal tops %.0f mm from the reference distance, "
                        "outside the +/-%.0f mm range: they will drop out as 'range'", d, off, sc["z_range"])
        log.info("standoff: %.1f mm above the terminal tops (reference %.0f, %+.1f)",
                 d, sc["reference_distance"], off)
        if args.sensor == "line_scan":
            # The fixed station: the module passes under it end to end.
            start = looking_down(-cfg.module_l / 2 - 5, 0.0, cfg.z_terminal_top + d)
            length = cfg.module_l + 10
        else:
            # One wrist sweep along the +Y busbar row.
            start = looking_down(-cfg.module_l / 2 + 10, cfg.terminal_y, cfg.z_terminal_top + d)
            length = cfg.module_l - 20
        result = line_scan(scene, args.sensor, start, length, seed=args.seed, speed=args.speed, rate=args.rate)
        result.settings["distance_mm"] = d
    log.info("simulated in %.1f s", time.perf_counter() - t0)

    info = summary(result)
    _report(info)
    out = args.out or args.run / "scans" / f"{args.stage}_{args.sensor}.npz"
    result.source = {"run": str(args.run), "stage": args.stage, "include": sorted(args.include),
                     "labels": labels_fingerprint(args.run, args.stage)}
    t0 = time.perf_counter()
    result.save(out)
    info["files"] = [str(out), str(out.with_suffix(".ply"))] + (
        [str(out.with_name(out.stem + "_2d.png"))] if result.image is not None else [])
    info["run"], info["stage"] = str(args.run), args.stage
    scanlog.write_summary(folder, info)
    log.info("saved %s in %.1f s", ", ".join(info["files"]), time.perf_counter() - t0)
    log.warning("%s: %s points kept of %s rays, params %s -- log and summary in %s/",
                out, f"{info['kept']:,}", f"{info['rays']:,}", result.params_hash[:12], folder)


if __name__ == "__main__":
    main()

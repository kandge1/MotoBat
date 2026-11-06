"""Zivid 2D photos of the module: four close shots, one mosaic, defects boxed.

One shot from 600 mm wastes most of the frame on background. Instead the wrist
camera takes one shot per quadrant -- half the module's length on one terminal
row -- centred between terminals 3 and 4, 9 and 10, 15 and 16, 21 and 22
(DIP-chip pin numbering, module.terminal_pin), as close as it allows:

* **Height.** The distance that just fits one quadrant (plus a margin) in the
  2D field of view, clamped to the Zivid's working range. For this module the
  fit is ~217 mm, under the 300 mm minimum, so the camera sits at 300 mm above
  the highest part -- 0.12 mm per pixel, against 0.24 mm from 600 mm.
* **Full resolution.** 2448 x 2048, not the 3D cloud's stride, then each shot
  is cropped to its group so no frame is mostly background.
* **Mosaic.** An orthographic image of the whole module: for every mosaic
  pixel a ray straight down finds the real surface point, which is projected
  into the shot covering that group and coloured from it. Using the true
  surface height, not one plane, keeps busbars and cell caps -- 6 mm apart in
  height -- aligned across the seams.
* **Defects.** The run's labels (`<stage>.labels.json`) are drawn as boxes on
  every image, and each defect gets its own zoomed close-up.

Images are the display render from scan.render_2d -- colours from
configs/appearance.yaml, not a frozen sensor model (project_plan.md §4B.3
defers the 2D camera). Everything is written to one scan_logs folder.

    python -m defectgen.photos data/preview_welds
"""

from __future__ import annotations

import colorsys
import json
import logging
import time
from pathlib import Path

import numpy as np

import yaml

from .scan import CONFIG_DIR, Scene, load_params, looking_down, params_hash, signal, without_groups

log = logging.getLogger("defectgen.photos")

MARGIN = 6.0      # mm around each group and around the module
BACKGROUND = (40, 40, 44)


# -- one shot --------------------------------------------------------------

class Shot:
    """A full-resolution 2D capture from `pose`, with what is needed to
    project module points into it.

    Rendered as a photo, not a depth map: `aa` x `aa` rays per pixel averaged
    (a lens integrates light over each pixel, so edges come out smooth rather
    than stair-stepped), surface texture so flat CAD faces do not read as
    flat paint, one auto exposure, then mild vignetting and sensor noise. All
    of it is appearance only (configs/appearance.yaml), outside the frozen
    sensor model.

    Only `roi` (col0, row0, col1, row1) is rendered; the rest of the frame
    stays background. Ray casting itself is a small part of the cost -- the
    per-sample shading of 45 M samples at 9 rays/pixel is the bulk -- so
    rendering just the quadrant being kept is what makes a shot fast."""

    def __init__(self, pose: np.ndarray):
        sensors, _ = load_params()
        cfg = sensors["snapshot"]
        self.w, self.h = cfg["resolution"]
        self.tx = np.tan(np.radians(cfg["hfov"] / 2))
        self.ty = np.tan(np.radians(cfg["vfov"] / 2))
        self.rot, self.eye = pose[:3, :3], pose[:3, 3]
        self.image = None
        self.exposure = 1.0

    def project(self, points: np.ndarray) -> np.ndarray:
        """Module points -> (col, row) pixel coordinates (float)."""
        c = (points - self.eye) @ self.rot
        u, v = c[:, 0] / c[:, 2], c[:, 1] / c[:, 2]
        col = (u + self.tx) / (2 * self.tx) * (self.w - 1)
        row = (v + self.ty) / (2 * self.ty) * (self.h - 1)
        return np.column_stack([col, row])

    def window(self, points: np.ndarray) -> tuple[int, int, int, int]:
        """Pixel box (col0, row0, col1, row1) around the projection of `points`."""
        px = self.project(points)
        c0, r0 = np.floor(px.min(axis=0)).astype(int).clip(0)
        c1, r1 = np.ceil(px.max(axis=0)).astype(int)
        return int(c0), int(r0), int(min(c1, self.w)), int(min(r1, self.h))

    def render(self, scene: Scene, roi=None, aa: int = 3, seed: int = 0, rows_per_tile: int = 96):
        sensors, materials = load_params()
        cfg = sensors["snapshot"]
        look = yaml.safe_load((CONFIG_DIR / "appearance.yaml").read_text())
        projector = self.eye - self.rot[:, 0] * cfg["baseline"]
        colours = np.array([look["colours"][n] for n in scene.material_names], np.float32)
        textures = [look.get("texture", {}).get(n, {}) for n in scene.material_names]
        c0, r0, c1, r1 = roi or (0, 0, self.w, self.h)

        # Sub-pixel sample offsets, centred in each pixel.
        sub = ((np.arange(aa) + 0.5) / aa - 0.5).astype(np.float32)
        du = 2 * self.tx / (self.w - 1)
        dv = 2 * self.ty / (self.h - 1)
        cols = np.arange(c0, c1)
        linear = np.zeros((self.h, self.w, 3), np.float32)
        light_all = np.zeros((self.h, self.w), np.float32)
        hit_frac = np.zeros((self.h, self.w), np.float32)
        rot32 = self.rot.astype(np.float32)
        for t0 in range(r0, r1, rows_per_tile):
            rows = np.arange(t0, min(t0 + rows_per_tile, r1))
            # (rows, aa, cols, aa) sample grid -> flat
            v = (-self.ty + rows * dv)[:, None, None, None] + (sub * dv)[None, :, None, None]
            u = (-self.tx + cols * du)[None, None, :, None] + (sub * du)[None, None, None, :]
            v, u = np.broadcast_arrays(v.astype(np.float32), u.astype(np.float32))
            local = np.stack([u.ravel(), v.ravel(), np.ones(u.size, np.float32)], axis=1)
            local /= np.linalg.norm(local, axis=1, keepdims=True)
            dirs = local @ rot32.T
            t, normal, mat = scene.cast(np.broadcast_to(self.eye, dirs.shape), dirs)
            hit = np.isfinite(t)
            n_s = len(dirs)
            col = np.zeros((n_s, 3), np.float32)
            light = np.zeros(n_s, np.float32)
            idx = np.where(hit)[0]
            if len(idx):
                pts = self.eye + dirs[idx] * t[idx, None]
                to_proj = projector - pts
                to_proj /= np.linalg.norm(to_proj, axis=1, keepdims=True)
                lit = scene.visible(pts, projector, normal[idx])
                li = signal(materials, scene.material_names, mat[idx], normal[idx], to_proj,
                            -dirs[idx], cfg["band"])
                light[idx] = np.where(lit, li, 0.0) + look["ambient"]
                col[idx] = colours[mat[idx]] * _texture(textures, mat[idx], pts)[:, None]
            shape = (len(rows), aa, len(cols), aa)
            # Average the aa x aa samples of each pixel: colour weighted by light.
            linear[rows, c0:c1] = (col * light[:, None]).reshape(*shape, 3).mean(axis=(1, 3))
            light_all[rows, c0:c1] = light.reshape(shape).mean(axis=(1, 3))
            hit_frac[rows, c0:c1] = hit.reshape(shape).mean(axis=(1, 3))

        # One exposure for the rendered region, set on its median lit surface
        # (it lands at mid-grey), then a Reinhard highlight curve. Specular
        # metal under the projector returns ~15x the diffuse surfaces; exposing
        # for it leaves everything else black, clipping it leaves it white. The
        # curve compresses it instead, as a camera's tone mapping does.
        lit_px = light_all[hit_frac > 0.5]
        exposure = 0.18 / max(float(np.median(lit_px)), 1e-6) if lit_px.size else 1.0
        img = linear * exposure
        white = max(float(np.percentile(img[hit_frac > 0.5].max(axis=-1), 99.9)), 1.0) if lit_px.size else 1.0
        img = img * (1.0 + img / white**2) / (1.0 + img)
        # Background where nothing was hit, blended at partly covered edges.
        bg = np.array(BACKGROUND, np.float32) / 255.0
        img = img + (1.0 - hit_frac)[..., None] * bg ** 2.2
        # Lens vignetting (cos^4 falloff, softened) and sensor noise.
        uu, vv = np.meshgrid(np.linspace(-self.tx, self.tx, self.w, dtype=np.float32),
                             np.linspace(-self.ty, self.ty, self.h, dtype=np.float32))
        cos4 = (1.0 / (1.0 + uu**2 + vv**2)) ** 2
        img *= (0.75 + 0.25 * cos4)[..., None]
        rng = np.random.default_rng(seed)
        img += rng.normal(0.0, look.get("sensor_noise", 0.0), img.shape).astype(np.float32)
        img = np.clip(img, 0.0, 1.0) ** (1 / 2.2)
        self.image = (img * 255 + 0.5).astype(np.uint8)
        self.exposure = exposure
        return self


def _waves(coord: np.ndarray, period: float, seed: int) -> np.ndarray:
    """Smooth pseudo-random variation along one coordinate, -1..1: a few sines
    of nearby periods with fixed random phases. Deterministic per seed."""
    rng = np.random.default_rng(seed)
    out = np.zeros_like(coord)
    for k in range(5):
        f = 1.0 / (period * rng.uniform(0.5, 1.5))
        out += np.sin(2 * np.pi * f * coord + rng.uniform(0, 2 * np.pi)) / (k + 1)
    return out / 2.3


def _hash(ix: np.ndarray, iy: np.ndarray, seed: int) -> np.ndarray:
    """A fixed pseudo-random value in -1..1 per integer lattice point."""
    h = (ix.astype(np.int64) * 374761393 + iy.astype(np.int64) * 668265263 + seed * 2147483647) & 0xFFFFFFFF
    h = ((h ^ (h >> 13)) * 1274126177) & 0xFFFFFFFF
    return (h & 0xFFFF).astype(np.float32) / 32767.5 - 1.0


def _stipple(x: np.ndarray, y: np.ndarray, period: float, seed: int, octaves: int = 3) -> np.ndarray:
    """Isotropic speckle, -1..1: smoothed random values on a grid of the given
    period, plus finer octaves -- the look of a textured moulded plastic or a
    cast surface. (A product of 1D waves, used before, reads as a plaid.)"""
    out = np.zeros(len(x), np.float32)
    amp, total = 1.0, 0.0
    for o in range(octaves):
        fx, fy = x / period, y / period
        ix, iy = np.floor(fx), np.floor(fy)
        sx, sy = fx - ix, fy - iy
        sx, sy = sx * sx * (3 - 2 * sx), sy * sy * (3 - 2 * sy)
        a = _hash(ix, iy, seed + o)
        b = _hash(ix + 1, iy, seed + o)
        c = _hash(ix, iy + 1, seed + o)
        d = _hash(ix + 1, iy + 1, seed + o)
        out += amp * ((a * (1 - sx) + b * sx) * (1 - sy) + (c * (1 - sx) + d * sx) * sy)
        total += amp
        amp, period = amp * 0.5, period * 0.5
    return out / total


def _texture(textures: list[dict], mat: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Brightness factor per hit point, around 1. `streak`: rolling or brushing
    marks -- variation across the streak direction only, so lines run along
    it. `grain`: fine isotropic speckle (moulded plastic, cast metal, PCB)."""
    out = np.ones(len(mat), np.float32)
    for m in np.unique(mat):
        t = textures[m]
        if not t:
            continue
        sel = mat == m
        p = pts[sel]
        f = np.ones(sel.sum(), np.float32)
        if "streak" in t:
            across = p[:, 1] if t.get("along", "x") == "x" else p[:, 0]
            f += t["streak"] * _waves(across, t.get("streak_period", 0.4), int(m))
            f += 0.5 * t["streak"] * _waves(across, t.get("streak_period", 0.4) / 7, int(m) + 100)
        if "grain" in t:
            f += t["grain"] * _stipple(p[:, 0], p[:, 1], t.get("grain_period", 0.3), int(m) + 200)
        out[sel] = f
    return out


# -- layout ----------------------------------------------------------------

def shot_layout(cfg):
    """The four quadrant shots, named by terminal pins (module.terminal_pin).

    Each quadrant is half the module's length on one terminal row: pins 1-6,
    7-12 down the left (-Y) row, 13-18, 19-24 up the right (+Y). The camera
    sits over the middle of its quadrant -- between pins 3 and 4, 9 and 10,
    15 and 16, 21 and 22 -- looking straight down, image top toward -X and
    image left toward -Y, the same view the pin numbering is defined in.

    Returns the quadrants, the camera distance above the highest part, the
    distance that would just fit a quadrant, and mm per pixel."""
    from .module import terminal_pin, terminal_y

    sensors, _ = load_params()
    sc = sensors["snapshot"]
    n, half = cfg.n_cells, cfg.n_cells // 2
    row = abs(terminal_y(cfg, 0, "pos"))
    quads = []
    for side, y in ((-1, -row), (1, row)):
        for first in (0, half):
            cells = list(range(first, first + half))
            pins = sorted(terminal_pin(cfg, k, y) for k in cells)
            mid = half // 2
            cx = (cfg.cell_centre_x(first + mid - 1) + cfg.cell_centre_x(first + mid)) / 2
            x_lo = cfg.cell_centre_x(first) - cfg.cell_pitch / 2
            x_hi = cfg.cell_centre_x(first + half - 1) + cfg.cell_pitch / 2
            y_lo, y_hi = (-cfg.module_w / 2, 0.0) if side < 0 else (0.0, cfg.module_w / 2)
            quads.append({
                "pins": (pins[0], pins[-1]),
                "between": tuple(sorted((terminal_pin(cfg, first + mid - 1, y), terminal_pin(cfg, first + mid, y)))),
                "centre": (cx, y),
                "x": (x_lo - MARGIN, x_hi + MARGIN), "y": (y_lo - MARGIN, y_hi + MARGIN),
            })
    quads.sort(key=lambda q: q["pins"][0])
    # Image width runs along Y, height along X.
    need_w = max(q["y"][1] - q["y"][0] for q in quads)
    need_h = max(q["x"][1] - q["x"][0] for q in quads)
    fit = max(need_w / (2 * np.tan(np.radians(sc["hfov"] / 2))),
              need_h / (2 * np.tan(np.radians(sc["vfov"] / 2))))
    lo, hi = sc["working_distance"]
    distance = float(np.clip(fit, lo, hi))
    mm_per_px = 2 * distance * np.tan(np.radians(sc["hfov"] / 2)) / (sc["resolution"][0] - 1)
    return quads, distance, fit, mm_per_px


# -- drawing ---------------------------------------------------------------

def _colour(cls: str):
    h = (hash(cls) % 360) / 360.0
    return tuple(int(255 * c) for c in colorsys.hsv_to_rgb(h, 0.9, 1.0))


def _font(size: int):
    from PIL import ImageFont

    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


SEVERITY = ("burn_through", "lack_of_fusion", "weld_underfill", "weld_undercut")


def _severity(cls: str) -> int:
    return SEVERITY.index(cls) if cls in SEVERITY else len(SEVERITY)


def _boxes(defects):
    """Defects grouped by identical box: several consequences on one weld
    stitch become one box with one combined label."""
    groups = {}
    for d in defects:
        if not d.get("bounds"):
            continue
        key = tuple(round(v, 2) for v in d["bounds"][0] + d["bounds"][1])
        groups.setdefault(key, []).append(d)
    out = []
    for key, ds in groups.items():
        classes = sorted({d["cls"] for d in ds}, key=_severity)
        cause = ds[0].get("cause")
        text = "+".join(classes) + (f" <- {cause}" if cause else "")
        # On the images: the worst class only, +N for the rest. The full
        # list and the cause go in the close-up's name and summary.json.
        short = classes[0] + (f" +{len(classes) - 1}" if len(classes) > 1 else "")
        out.append((np.array(key[:3]), np.array(key[3:]), classes[0], text, short))
    return out


def _annotate(image: np.ndarray, rects, scale: int = 2, tags=()) -> np.ndarray:
    """Draw defect boxes with their labels, and `tags` -- (pixel, text) pairs,
    the terminal pin numbers -- in white with a dark outline."""
    from PIL import Image, ImageDraw

    im = Image.fromarray(image)
    draw = ImageDraw.Draw(im)
    font = _font(max(12, image.shape[1] // 120))
    for (x, y), text in tags:
        draw.text((x, y), text, fill=(255, 255, 255), font=font, anchor="mm",
                  stroke_width=2, stroke_fill=(0, 0, 0))
    for (x0, y0, x1, y1), cls, text in rects:
        c = _colour(cls)
        draw.rectangle([x0, y0, x1, y1], outline=c, width=scale)
        draw.text((x0, max(0, y0 - font.size - 2)), text, fill=c, font=font)
    return np.asarray(im)


# -- the capture -----------------------------------------------------------

def capture(run: Path, stage: str, folder: Path, stitch: bool = True, aa: int = 3, include=()) -> dict:
    from PIL import Image

    from .config import load

    cfg = load()
    scene = Scene.from_run(run, stage, without=without_groups(stage, include))
    labels_path = Path(run) / f"{stage}.labels.json"
    defects = json.loads(labels_path.read_text())["defects"] if labels_path.exists() else []
    boxes = _boxes(defects)

    top_z = cfg.z_wall_top
    quads, distance, fit, mm_px = shot_layout(cfg)
    log.info("layout: %d quadrant shots, camera %.0f mm above the top (z %.1f), looking down; "
             "image top = -X (cell 0 end), image left = -Y (pins 1-12)", len(quads), distance, top_z)
    for q in quads:
        log.info("  pins %2d-%2d: centred between pins %d and %d at (%.1f, %.1f) mm",
                 *q["pins"], *q["between"], *q["centre"])
    log.info("  distance that just fits a quadrant: %.0f mm; working range %s mm -> %.0f mm, %.3f mm per pixel",
             fit, load_params()[0]["snapshot"]["working_distance"], distance, mm_px)

    labels = json.loads(labels_path.read_text()) if labels_path.exists() else {"terminals": []}
    pins = [(t["pin"], np.array(t["centre"]) + [0, 0, cfg.terminal_h]) for t in labels.get("terminals", [])
            if t.get("pin")]

    for sub in ("photos", "annotated", "defects"):
        (folder / sub).mkdir(exist_ok=True)

    shots, files = [], []
    for i, q in enumerate(quads, 1):
        t0 = time.perf_counter()
        cx, cy = q["centre"]
        shot = Shot(looking_down(cx, cy, top_z + distance))
        # Crop to the quadrant: project its footprint at the cap and top
        # heights, and render only that window.
        corners = np.array([[x, y, z] for x in q["x"] for y in q["y"] for z in (cfg.z_cell_top, top_z)])
        c0, r0, c1, r1 = shot.window(corners)
        shot.render(scene, (c0, r0, c1, r1), aa=aa, seed=i)
        crop = shot.image[r0:r1, c0:c1]
        shots.append((shot, crop, c0, r0, q))
        name = f"q{i}_pins_{q['pins'][0]}-{q['pins'][1]}.png"
        path = folder / "photos" / name
        Image.fromarray(crop).save(path)
        files.append(str(path))

        inside = lambda x, y: q["x"][0] <= x <= q["x"][1] and q["y"][0] <= y <= q["y"][1]  # noqa: E731
        rects = []
        for lo, hi, cls, text, short in boxes:
            if not inside((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2):
                continue
            bc = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
            p = shot.project(bc) - [c0, r0]
            rects.append(((*p.min(axis=0), *p.max(axis=0)), cls, short))
        tags = [(shot.project(c[None])[0] - [c0, r0], f"T{n}") for n, c in pins if inside(c[0], c[1])]
        apath = folder / "annotated" / name
        Image.fromarray(_annotate(crop, rects, tags=tags)).save(apath)
        files.append(str(apath))
        log.info("  shot %d (pins %d-%d): %d x %d px x %d rays/px, exposure %.2f, cropped to %d x %d, "
                 "%d defects boxed (%.1f s)", i, *q["pins"], shot.w, shot.h, aa * aa, shot.exposure,
                 c1 - c0, r1 - r0, len(rects), time.perf_counter() - t0)

    # One close-up per defect, cut from the quarter shot that holds it.
    crops = []
    for n, (lo, hi, cls, text, short) in enumerate(boxes, 1):
        mid = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2])
        shot, crop, c0, r0, q = min(shots, key=lambda s: np.hypot(*(np.array(s[4]["centre"]) - mid)))
        pad = max(3.0, 0.25 * max(hi[0] - lo[0], hi[1] - lo[1]))
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        box = shot.project(corners)
        wide = np.array([[x, y, z] for x in (lo[0] - pad, hi[0] + pad) for y in (lo[1] - pad, hi[1] + pad)
                         for z in (lo[2], hi[2])])
        win = shot.project(wide)
        a, b = np.floor(win.min(axis=0)).astype(int).clip(0)
        c, d = np.ceil(win.max(axis=0)).astype(int)
        c, d = min(c, shot.w), min(d, shot.h)
        if c - a < 2 or d - b < 2:
            continue
        # The box sits a few pixels outside the defect so it frames it rather
        # than covering its edge.
        lo_px, hi_px = box.min(axis=0) - [a, b] - 4, box.max(axis=0) - [a, b] + 4
        piece = _annotate(shot.image[b:d, a:c], [((*lo_px, *hi_px), cls, "")], scale=1)
        zoom = max(1, int(np.ceil(400 / piece.shape[1])))
        img = Image.fromarray(piece).resize((piece.shape[1] * zoom, piece.shape[0] * zoom), Image.LANCZOS)
        name = "_".join(text.split(" <- ")[0].split("+")[:2])
        path = folder / "defects" / f"{n:02d}_{name}.png"
        img.save(path)
        crops.append({"file": path.name, "defect": text, "bounds_mm": [lo.tolist(), hi.tolist()]})
    log.info("defect close-ups: %d in %s", len(crops), folder / "defects")

    scans = Path(run) / "scans"
    scans.mkdir(exist_ok=True)
    info = {
        "sensor": "photos (Zivid 2D, display render)",
        "run": str(run), "stage": stage, "params_hash": params_hash(),
        "shots": [{"pins": list(q["pins"]), "between_pins": list(q["between"]),
                   "centre_mm": [float(v) for v in q["centre"]], "camera_z_mm": float(top_z + distance)}
                  for q in quads],
        "distance_mm": distance, "fit_distance_mm": float(fit), "mm_per_px": float(mm_px),
        "rays_per_pixel": aa * aa, "defects_boxed": len(boxes), "close_ups": crops, "files": files,
    }
    if not stitch:
        return info

    # Mosaic: orthographic, true-height projection into the covering shot,
    # in the same view as the photos and the pin numbering: rows run along
    # +X (top = cell 0 end), columns along +Y (left = pins 1-12).
    t0 = time.perf_counter()
    x0, x1 = -cfg.module_l / 2 - MARGIN, cfg.module_l / 2 + MARGIN
    y0, y1 = -cfg.module_w / 2 - MARGIN, cfg.module_w / 2 + MARGIN
    xs = np.arange(x0, x1, mm_px)
    ys = np.arange(y0, y1, mm_px)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    down = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, top_z + 100.0)])
    t, _, _ = scene.cast(down, np.tile([0.0, 0.0, -1.0], (len(down), 1)))
    hit = np.isfinite(t)
    surface = down + np.array([0.0, 0.0, -1.0]) * np.where(hit, t, 0.0)[:, None]
    centres = np.array([s[4]["centre"] for s in shots])
    which = np.argmin(((surface[:, None, :2] - centres[None]) ** 2).sum(axis=2), axis=1)
    mosaic = np.empty((len(surface), 3), np.uint8)
    mosaic[:] = BACKGROUND
    for k, (shot, *_) in enumerate(shots):
        sel = hit & (which == k)
        cr = np.rint(shot.project(surface[sel])).astype(int)
        cr[:, 0] = cr[:, 0].clip(0, shot.w - 1)
        cr[:, 1] = cr[:, 1].clip(0, shot.h - 1)
        mosaic[sel] = shot.image[cr[:, 1], cr[:, 0]]
    mosaic = mosaic.reshape(len(xs), len(ys), 3)
    mpath = folder / "photos" / "stitched.png"
    Image.fromarray(mosaic).save(mpath)
    files.append(str(mpath))

    to_px = lambda x, y: ((y - y0) / mm_px, (x - x0) / mm_px)  # noqa: E731  (col, row)
    rects = []
    for lo, hi, cls, text, short in boxes:
        (a, b), (c, d) = to_px(lo[0], lo[1]), to_px(hi[0], hi[1])
        rects.append(((a, b, c, d), cls, short))
    tags = [(np.array(to_px(c[0], c[1])), f"T{n}") for n, c in pins]
    apath = folder / "annotated" / "stitched.png"
    Image.fromarray(_annotate(mosaic, rects, tags=tags)).save(apath)
    files.append(str(apath))
    log.info("mosaic: %d x %d px at %.3f mm/px, %d defects boxed (%.1f s)",
             mosaic.shape[1], mosaic.shape[0], mm_px, len(rects), time.perf_counter() - t0)

    # The mosaic next to the run's scans, for show_scan.py.
    Image.fromarray(mosaic).save(scans / f"{stage}_photos_stitched.png")
    Image.open(apath).save(scans / f"{stage}_photos_annotated.png")
    return info


def main() -> None:
    import argparse

    from . import scanlog

    p = argparse.ArgumentParser(prog="python -m defectgen.photos", description=__doc__.split("\n")[0])
    p.add_argument("run", type=Path, help="a defectgen --out directory")
    p.add_argument("--stage", default="post_weld")
    p.add_argument("--log-dir", type=Path, default=scanlog.LOG_ROOT)
    p.add_argument("--no-stitch", dest="stitch", action="store_false",
                   help="skip stitching the four shots into one image of the whole module")
    p.add_argument("--include", nargs="+", default=[], metavar="GROUP",
                   help="put back groups the stage leaves out for inspection (post_weld: cmc cover)")
    p.add_argument("--aa", type=int, default=3, help="rays per pixel along each side (3 = 9 per pixel)")
    args = p.parse_args()

    folder = scanlog.start(args.run, args.stage, "photos", args.log_dir)
    log.info("defectgen.photos on %s / %s -> %s", args.run, args.stage, folder)
    t0 = time.perf_counter()
    info = capture(args.run, args.stage, folder, stitch=args.stitch, aa=args.aa, include=args.include)
    scanlog.write_summary(folder, info)
    log.warning("done in %.1f s: %d photos%s, %d defect close-ups -> %s", time.perf_counter() - t0,
                len(info["shots"]), " + stitched" if args.stitch else "", len(info["close_ups"]), folder)


if __name__ == "__main__":
    main()

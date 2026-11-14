"""The labelled dataset (task 1.7, project_plan.md §4B.6 "1.7 dataset").

    python -m defectgen.dataset --tier pod --pilot          # one case per family, to time it
    python -m defectgen.dataset --tier pod                  # the whole tier (resumes)
    python -m defectgen.dataset --tier clean
    python -m defectgen.dataset --tier grr
    python -m defectgen.dataset --tier pod --shard 3/14    # worker 3 of 14 (cloud/run_remote.sh)

A case is one module at the stage that inspects its class, with one injected
defect of one size, and a seed. The seed fixes where the defect goes, each
scan's fixture placement and each scan's noise, so the dataset is a list of
recipes, regenerable bit for bit. Each case is scanned by every setup (the
fixed Ranger3 pass, and the arm's task 1.5 view plan), detected, and scored
with the out-of-spec rule: a label below its tolerance is not a defect, and
flagging it is a false reject.

Only small files are kept, per case under data/dataset/<tier>/<family>/:
case.json (recipe and truth), labels.json, <setup>.report.json and
<setup>.score.json; and one line per finished case in <tier>/index.jsonl.
Point clouds (~150 MB each) come back from the seed. A finished case is
skipped on rerun, so a long run can be stopped and resumed.
"""

from __future__ import annotations

import json
import logging
import shutil
import time
import zlib
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml

from . import defects as D
from .config import Config, load
from .coverage import fixed_baseline
from .detect import Capture, Reference, detect, load_detect_params
from .evaluate import evaluate
from .module import busbar_plan, collect_labels, injected_plan, realise_groups
from .parts import load_all
from .scan import Scene, line_scan, snapshot, without_groups

CONFIG = Path(__file__).parent / "configs" / "dataset.yaml"
log = logging.getLogger("defectgen.dataset")


def load_dataset_params(path: Path | None = None) -> dict:
    return yaml.safe_load(Path(path or CONFIG).read_text())


def _seed(*parts) -> int:
    """A stable integer seed from any labels (crc32: same on every run and machine)."""
    return zlib.crc32("/".join(map(str, parts)).encode()) & 0x7FFFFFFF


def tolerance(fam: dict, detect_params: dict) -> float | None:
    tol = fam.get("tol")
    if isinstance(tol, str):
        section, key = tol.split(".")
        return float(detect_params[section][key])
    return None if tol is None else float(tol)


def levels(fam: dict, q: dict, detect_params: dict) -> list[float]:
    if fam["kind"] == "tolerance":
        tol = tolerance(fam, detect_params)
        return [round(f * tol, 6) for f in q["tolerance_factors"]]
    if fam["kind"] == "presence":
        return list(fam["levels"])
    return [1.0]


# -- recipes ---------------------------------------------------------------------

@dataclass
class Recipe:
    """Everything that defines one case. Two equal recipes give identical modules,
    scans and reports."""

    tier: str
    family: str
    cls: str
    stage: str
    kind: str
    magnitude: float
    seed: int
    name: str                       # case directory name
    params: dict = field(default_factory=dict)   # the injector's true parameters
    truth: str = ""                 # must_find | in_spec | dont_care | clean
    tol: float | None = None

    def to_json(self) -> dict:
        return asdict(self)


def _seam_pairs(cfg: Config) -> list[tuple[int, int]]:
    return [(b, c) for b, (_, terms) in enumerate(busbar_plan(cfg)) for c, _ in terms]


def _point_on(scene: Scene, rng, cfg: Config, material: str, avoid_terminals: bool = True):
    """A random point on an up-facing surface of `material` (a cap top, say),
    found by casting straight down at random XY. Avoids the terminal pads, so
    a particle does not end up inside one."""
    idx = scene.material_names.index(material)
    for _ in range(500):
        k = int(rng.integers(cfg.n_cells))
        x = cfg.cell_centre_x(k) + rng.uniform(-cfg.cell_t / 2 + 2, cfg.cell_t / 2 - 2)
        y = rng.uniform(-cfg.cell_w / 2 + 3, cfg.cell_w / 2 - 3)
        if avoid_terminals and abs(abs(y) - cfg.terminal_y) < 14:
            continue
        t, n, m = scene.cast(np.array([[x, y, 300.0]]), np.array([[0.0, 0.0, -1.0]]))
        if np.isfinite(t[0]) and m[0] == idx and n[0, 2] > 0.95:
            return k, np.array([x, y, 300.0 - t[0]])
    raise RuntimeError(f"no clear {material} point found")


def injector(recipe: Recipe, cfg: Config, ref: Reference, rng: np.random.Generator):
    """The defects.py injector for a recipe, drawing its location from `rng`.
    Fills recipe.params with the true parameters it chose."""
    f, m, p = recipe.family, recipe.magnitude, recipe.params
    sign = 1.0 if rng.random() < 0.5 else -1.0
    if f == "cell_shift":
        k, th = int(rng.integers(cfg.n_cells)), rng.uniform(0, 2 * np.pi)
        if p.get("along_x") is not None:            # GRR: a signed shift along X
            dx, dy = p["along_x"], 0.0
        else:
            dx, dy = m * np.cos(th), m * np.sin(th)
        p.update(k=k, dx=float(dx), dy=float(dy))
        return D.cell_placement(k, dx=dx, dy=dy)
    if f == "cell_yaw":
        k = int(rng.integers(cfg.n_cells))
        p.update(k=k, dtheta_deg=sign * m)
        return D.cell_placement(k, dtheta_deg=sign * m)
    if f == "terminal_height":
        k, pol = int(rng.integers(cfg.n_cells)), str(rng.choice(["pos", "neg"]))
        dz = p["dz"] if "dz" in p else sign * m
        p.update(k=k, polarity=pol, dz=float(dz))
        return D.terminal_height(k, pol, dz)
    if f == "terminal_tilt":
        k, pol, axis = int(rng.integers(cfg.n_cells)), str(rng.choice(["pos", "neg"])), str(rng.choice(["x", "y"]))
        p.update(k=k, polarity=pol, deg=m, axis=axis)
        return D.terminal_tilt(k, pol, m, axis=axis)
    if f == "missing_cell":
        k = int(rng.integers(cfg.n_cells))
        p.update(k=k)
        return D.missing_cell(k)
    if f == "reversed_polarity":
        k = int(rng.integers(cfg.n_cells))
        p.update(k=k)
        return D.reversed_polarity(k)
    if f == "missing_pad":
        k = int(rng.integers(cfg.n_cells - 1))
        p.update(k=k)
        return D.missing_pad(k)
    if f == "fod":
        _, at = _point_on(ref.scene, rng, cfg, "aluminium_cap")
        centre = at + [0.0, 0.0, m]
        p.update(centre=centre.tolist(), radius=m)
        return D.fod(tuple(centre), radius=m)
    if f == "handling_damage":
        fam_r = p.get("radius", 4.0)
        target = str(rng.choice(["cap", "end_plate_top", "side_plate_outer"]))
        if target == "cap":
            k, at = _point_on(ref.scene, rng, cfg, "aluminium_cap")
            ref_part, direction = ("cell", k), (0.0, 0.0, -1.0)
        elif target == "end_plate_top":
            side = str(rng.choice(["a", "b"]))
            x = (-1 if side == "a" else 1) * (cfg.module_l / 2 - cfg.end_plate_t / 2)
            at = np.array([x, rng.uniform(-cfg.module_w / 2 + 8, cfg.module_w / 2 - 8), cfg.z_wall_top])
            ref_part, direction = ("end_plate", side), (0.0, 0.0, -1.0)
        else:
            side = str(rng.choice(["left", "right"]))
            sy = -1 if side == "left" else 1
            at = np.array([rng.uniform(-cfg.module_l / 2 + 25, cfg.module_l / 2 - 25),
                           sy * (cfg.cell_w / 2 + cfg.side_plate_t), rng.uniform(20.0, cfg.z_wall_top - 15)])
            ref_part, direction = ("side_plate", side), (0.0, -sy, 0.0)
        p.update(target=target, ref=list(ref_part), centre=at.tolist(), depth=m, radius=fam_r)
        return D.handling_damage(ref_part, depth=m, radius=fam_r, centre=tuple(at), direction=direction)
    if f == "busbar_shift":
        b, th = int(rng.integers(len(busbar_plan(cfg)))), rng.uniform(0, 2 * np.pi)
        p.update(busbar=b, dx=float(m * np.cos(th)), dy=float(m * np.sin(th)))
        return D.busbar_pose(b, dx=p["dx"], dy=p["dy"])
    if f == "busbar_liftoff":
        b = int(rng.integers(len(busbar_plan(cfg))))
        gap = p.get("gap", m)
        p.update(busbar=b, gap=float(gap))
        return D.busbar_liftoff(b, gap=gap)
    if f == "carrier_seating":
        p.update(dz=m)
        return D.carrier_seating(m)
    if f == "spatter":
        p.update(count=p.get("count", 5), radius=m)
        return D.spatter(count=p["count"], radius=m)
    pairs = _seam_pairs(cfg)
    b, c = pairs[int(rng.integers(len(pairs)))]
    p.update(busbar=b, cell=c)
    if f == "missing_weld":
        return D.missing_weld(b, c)
    if f == "short_weld":
        p.update(fraction=1.0 - m)
        return D.short_weld(b, c, fraction=1.0 - m)
    if f == "seam_deviation":
        p.update(dy=sign * m)
        return D.seam_deviation(b, c, dy=sign * m)
    if f == "width_deviation":
        factor = max(1.0 + sign * m, 0.3)
        p.update(factor=factor)
        return D.width_deviation(b, c, factor=factor)
    if f == "reinforcement":
        p.update(dh=sign * m)
        return D.reinforcement(b, c, dh=sign * m)
    if f == "weld_path_drift":
        p.update(amp=m)
        return D.weld_path_drift(b, c, amp=m)
    if f == "weld_arc":
        p.update(sagitta=sign * m)
        return D.weld_arc(b, c, sagitta=sign * m)
    raise ValueError(f"no injector for family {f!r}")


def judge(recipe: Recipe) -> str:
    """Is the injected defect one the station must reject? The recipe's true
    parameters decide, not the label's single number (cell placement has a
    position and a yaw tolerance; the label holds only the shift)."""
    if recipe.kind == "binary":
        return "must_find"
    if recipe.kind == "presence":
        return "must_find" if recipe.magnitude >= recipe.params.get("D", np.inf) else "dont_care"
    p, tol = recipe.params, recipe.tol
    if recipe.family == "cell_shift":
        value = float(np.hypot(p["dx"], p["dy"]))
    elif recipe.family == "terminal_height" and "dz" in p:
        value = abs(p["dz"])
    elif recipe.family == "busbar_liftoff":
        value = p["gap"]
    else:
        value = abs(recipe.magnitude)
    return "must_find" if value > tol else "in_spec"


def pod_cases(q: dict, detect_params: dict, families: list[str] | None = None) -> list[Recipe]:
    out = []
    for name, fam in q["families"].items():
        if families and name not in families:
            continue
        tol = tolerance(fam, detect_params)
        for li, m in enumerate(levels(fam, q, detect_params)):
            for s in range(q["tiers"]["pod"]["seeds"]):
                params = {"D": fam["D"]} if "D" in fam else {}
                if "count" in fam:
                    params["count"] = fam["count"]
                if "radius" in fam:
                    params["radius"] = fam["radius"]
                out.append(Recipe("pod", name, fam["cls"], fam["stage"], fam["kind"], m,
                                  _seed("pod", name, li, s), f"L{li}_{m:g}_s{s}", params, tol=tol))
    return out


def clean_cases(q: dict) -> list[Recipe]:
    return [Recipe("clean", stage, "", stage, "clean", 0.0, _seed("clean", stage, i), f"m{i}", truth="clean")
            for stage in ("pre_busbar", "pre_weld", "post_weld")
            for i in range(q["tiers"]["clean"]["modules"])]


def grr_cases(q: dict, detect_params: dict) -> list[Recipe]:
    """AIAG crossed study: parts x loads x trials per characteristic. A part
    keeps its defect (location and true value) across all its loads and
    trials; a load is a fresh fixture placement; a trial repeats the scan
    on the same placement with new noise."""
    g = q["tiers"]["grr"]
    out = []
    for char, fam_name in g["characteristics"].items():
        fam = q["families"][fam_name]
        tol = tolerance(fam, detect_params)
        # A gap cannot be negative, and a zero gap is no chip at all.
        lo, hi = (0.1 * tol, 2 * g["spread"] * tol) if fam_name == "busbar_liftoff" else \
            (-g["spread"] * tol, g["spread"] * tol)
        values = np.linspace(lo, hi, g["parts"])
        for pi, v in enumerate(values):
            params = {"terminal_height": {"dz": float(v)}, "cell_shift": {"along_x": float(v)},
                      "busbar_liftoff": {"gap": float(v)}}[fam_name]
            for li in range(g["loads"]):
                for ti in range(g["trials"]):
                    r = Recipe("grr", fam_name, fam["cls"], fam["stage"], fam["kind"], float(abs(v)),
                               _seed("grr", char, pi), f"{char}_p{pi}_l{li}_t{ti}", dict(params), tol=tol)
                    r.params.update(part=pi, load=li, trial=ti)
                    out.append(r)
    return out


# -- scanning ---------------------------------------------------------------------

def fixture_error(rng: np.random.Generator, q: dict) -> np.ndarray:
    """A pallet reload: a small random rigid motion about the module centre."""
    f = q["fixture"]
    rz, rx, ry = np.radians(rng.normal(0, [f["sigma_yaw_deg"], f["sigma_tilt_deg"], f["sigma_tilt_deg"]]))
    cz, sz, cx, sx, cy, sy = np.cos(rz), np.sin(rz), np.cos(rx), np.sin(rx), np.cos(ry), np.sin(ry)
    r = (np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
         @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]))
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = [*rng.normal(0, f["sigma_xy"], 2), rng.normal(0, f["sigma_z"])]
    return t


def view_plan(stage: str, root: Path = Path("data/coverage")) -> list[dict]:
    f = root / f"{stage}.plan.json"
    if not f.is_file():
        raise FileNotFoundError(f"{f} not found: run python -m defectgen.coverage --stage {stage} (task 1.5)")
    return json.loads(f.read_text())["views"]


def _spill(cap: Capture, folder: Path | None) -> Capture:
    """Write a capture's big arrays to disk and map them back read-only, as a
    real cell streams captures to disk: detection then pages in only what it
    touches. Six full-length Keyence sweeps (~10 M points each) held in memory
    at once did not fit in 16 GB."""
    if folder is None:
        return cap
    folder.mkdir(parents=True, exist_ok=True)
    arrays = {}
    for name in ("points", "index", "signal"):
        a = getattr(cap, name)
        if a is None:
            continue
        f = folder / f"{cap.name}.{name}.npy"
        np.save(f, a)
        arrays[name] = np.load(f, mmap_mode="r")
    return Capture(cap.sensor, arrays["points"], arrays["index"], cap.grid, cap.poses, cap.settings,
                   arrays.get("signal"), cap.params_hash, cap.name, cap.source)


def scan_setup(setup: str, groups: dict, stage: str, cfg: Config, noise: int,
               spill: Path | None = None) -> list[Capture]:
    """What one station returns from one placed module."""
    scene = Scene.from_groups(groups)
    quiet = logging.getLogger("defectgen.scan")
    level = quiet.level
    quiet.setLevel(logging.WARNING)
    try:
        if setup == "fixed":
            v = fixed_baseline(cfg)
            return [_spill(Capture.from_scan(line_scan(scene, "line_scan", v.pose, v.length, seed=noise),
                                             name="fixed"), spill)]
        caps = []
        for i, v in enumerate(view_plan(stage)):
            pose = np.array(v["pose_mm"])
            if v["kind"] == "snapshot":
                r = snapshot(scene, pose, sensor=v["sensor"], seed=noise + i)
            else:
                # A plan sweep flown fast (coverage.yaml fast_speed) carries its speed and rate.
                r = line_scan(scene, v["sensor"], pose, v["length_mm"], seed=noise + i,
                              speed=v.get("speed_mm_s"), rate=v.get("rate_hz"))
            caps.append(_spill(Capture.from_scan(r, name=v["id"]), spill))
            del r
        return caps
    finally:
        quiet.setLevel(level)


# -- what a setup looks at, by design ----------------------------------------------------

class SetupCoverage:
    """Which surface patches each setup covers, per class family, as task 1.5
    worked it out (data/coverage/<stage>.patches.npz: patch centroids, which
    families require each patch, which the fixed pass and the arm plan cover).

    It is a property of the station, not of the part. A dent on a side plate's
    outer face is out of the fixed line-scan's reach whatever its size: that is
    a coverage gap, and counting it as a detection miss would flatten the POD
    curve into a plateau that says nothing about either. POD is conditional on
    the location having been inspected (MIL-HDBK-1823A); what a setup cannot
    reach is reported beside it, as `not_covered`."""

    def __init__(self, folder: Path | str = "data/coverage", q: dict | None = None):
        from .coverage import load_coverage_params

        self.folder = Path(folder)
        self.q = q or load_coverage_params()
        self._stages: dict = {}

    def _stage(self, stage: str) -> dict | None:
        if stage not in self._stages:
            f = self.folder / f"{stage}.patches.npz"
            self._stages[stage] = dict(np.load(f)) if f.is_file() else None
        return self._stages[stage]

    def seen(self, stage: str, setup: str, cls: str, bounds) -> bool | None:
        """Whether `setup` covers, by design, any patch inside the label's box
        (grown by half a patch, so a sub-patch particle still meets one) that the
        class's family requires. A dent whose edge reaches a covered face counts
        as seen: it deforms that face. None when task 1.5 has no map for this
        stage, setup or class, or no required patch lies in the box."""
        z = self._stage(stage)
        if z is None or setup not in z:
            return None
        fams = self.q["stages"].get(stage, {})
        rows = [i for i, name in enumerate(z["classes"]) if cls in fams.get(str(name), {}).get("members", [])]
        if not rows:
            return None
        pad = self.q["patch_size"] / 2
        lo, hi = np.asarray(bounds[0], float) - pad, np.asarray(bounds[1], float) + pad
        inside = np.all((z["centroid"] >= lo) & (z["centroid"] <= hi), axis=1)
        need = z["required"][rows].any(axis=0) & inside
        if not need.any():
            return None
        return bool((z[setup][rows].any(axis=0) & need).any())


# -- scoring ------------------------------------------------------------------------

def _field(report, key: str):
    return report[key] if isinstance(report, dict) else getattr(report, key)


def score(report, labels: dict, recipe: Recipe, seen=None) -> dict:
    """The case's outcome for one setup, under the out-of-spec rule. `report`
    is a DefectReport or its JSON. `seen(label) -> bool | None` says whether the
    setup looks at a label's place by design (SetupCoverage); a must-find target
    missed where the setup does not look is `not_covered`, not a miss."""
    ev = evaluate(report, labels)
    targets = [d for d in ev["labels"] if d["cls"] == recipe.cls]
    found = [d["verdict"] == "tp" for d in targets]
    looked = [seen(d) if seen else None for d in targets]
    out = {"result": _field(report, "result"),
           "false_alarms": int(sum(v == "fp" for v in ev["finding_verdicts"])),
           "other": int(sum(v == "other" for v in ev["finding_verdicts"])),
           "uncovered": {k: len(v) for k, v in _field(report, "uncovered").items()},
           "targets": len(targets), "found": int(sum(found)),
           "not_looked_at": int(sum(v is False for v in looked))}
    if recipe.truth == "clean":
        out["outcome"] = "clean_ok" if not _field(report, "findings") else "false_reject"
    elif recipe.truth == "must_find":
        missed_in_view = [not f and v is not False for f, v in zip(found, looked)]
        if targets and all(found):
            out["outcome"] = "hit"
        elif any(missed_in_view) or not targets:
            out["outcome"] = "partial" if any(found) else "miss"
        else:                            # every miss lies where this setup does not look
            out["outcome"] = "hit" if any(found) else "not_covered"
    elif recipe.truth == "in_spec":
        out["outcome"] = "false_reject" if any(found) else "pass"
    else:
        out["outcome"] = "found" if any(found) else "not_found"     # dont_care: for POD only
    out["found_there"] = sorted({c for d in targets for c in d.get("found_there", [])})
    return out


def measured(report, recipe: Recipe, labels: dict, cfg: Config) -> float | None:
    """The detector's measurement of the injected feature: the 'a-hat' of a
    POD signal-response analysis and the reading of a %GRR study."""
    m, p, f = report.measurements, recipe.params, recipe.family
    try:
        if f == "terminal_height":
            return float(m["terminals"][f"{p['k']}{p['polarity']}"]["dz"])
        if f == "terminal_tilt":
            return float(m["terminals"][f"{p['k']}{p['polarity']}"]["tilt_deg"])
        if f == "cell_shift":
            c = m["cells"][p["k"]]
            return float(c["dx"]) if "along_x" in p else float(np.hypot(c["dx"], c["dy"]))
        if f == "cell_yaw":
            return float(m["cells"][p["k"]]["dyaw_deg"])
        if f == "busbar_shift":
            b = m["busbars"][p["busbar"]]
            return float(np.hypot(b["dx"], b["dy"]))
        if f == "busbar_liftoff":
            gaps = [v["gap"] for k, v in m["gaps"].items() if k.startswith(f"busbar {p['busbar']} ")]
            return float(max(gaps)) if gaps else None
        if f == "carrier_seating":
            return float(m["carrier"]["dz"])
        key = {"short_weld": "short", "seam_deviation": "offset", "width_deviation": "width_change",
               "reinforcement": "crown_change", "weld_path_drift": "drift", "weld_arc": "sagitta"}.get(f)
        if key:
            idx = [i for i, s in enumerate(labels["seams"])
                   if s["busbar_index"] == p["busbar"] and s["cell_index"] == p["cell"]]
            vals = [m["seams"][i][key] for i in idx if key in m["seams"].get(i, {})]
            return float(max(vals, key=abs)) if vals else None
    except (KeyError, TypeError):
        return None
    return None


# -- running cases --------------------------------------------------------------------

class Runner:
    def __init__(self, q: dict | None = None, setups: list[str] | None = None, root: Path | None = None):
        self.q = q or load_dataset_params()
        self.cfg = load()
        self.parts = load_all(self.cfg)
        self.dp = load_detect_params()
        self.setups = setups or self.q["setups"]
        self.root = Path(root or self.q["out"])
        self.cover = SetupCoverage()
        self._refs: dict = {}

    def ref(self, stage: str) -> Reference:
        if stage not in self._refs:
            self._refs[stage] = Reference(stage, self.cfg, parts=self.parts)
        return self._refs[stage]

    def _seen(self, r: Recipe, setup: str):
        return lambda d: self.cover.seen(r.stage, setup, d["cls"], d["bounds"])

    def case_dir(self, r: Recipe) -> Path:
        return self.root / r.tier / r.family / r.name

    def done(self, r: Recipe) -> bool:
        d = self.case_dir(r)
        return all((d / f"{s}.score.json").is_file() for s in self.setups)

    def run(self, r: Recipe) -> dict:
        t0 = time.perf_counter()
        ref = self.ref(r.stage)
        rng = np.random.default_rng(r.seed)
        # A draw can be physically impossible -- the last cell shifted into its
        # end plate -- and the generator refuses it. Draw again from the same
        # stream: still deterministic, and the case records how many draws it took.
        base = dict(r.params)
        for attempt in range(20):
            r.params = dict(base)
            injectors = [] if r.truth == "clean" else [injector(r, self.cfg, ref, rng)]
            try:
                plan = injected_plan(r.stage, self.cfg, r.seed, injectors, self.parts)
                break
            except ValueError as e:
                log.info("  %s %s: draw %d impossible (%s), drawing again", r.family, r.name, attempt + 1, e)
        else:
            raise RuntimeError(f"{r.family} {r.name}: no physically possible draw in 20 tries")
        r.params["draws"] = attempt + 1
        if r.truth != "clean":
            r.truth = judge(r)
        labels = json.loads(collect_labels(plan, r.seed, self.parts).to_json())
        groups = {g: m for g, m in realise_groups(plan, self.parts).items() if g not in without_groups(r.stage)}
        d = self.case_dir(r)
        d.mkdir(parents=True, exist_ok=True)
        (d / "labels.json").write_text(json.dumps(labels, indent=1))
        line = {"tier": r.tier, "family": r.family, "cls": r.cls, "magnitude": r.magnitude, "truth": r.truth,
                "name": r.name, "seed": r.seed, "setups": {}}
        loads = {}
        for setup in self.setups:
            t1 = time.perf_counter()
            # A GRR trial repeats its load's placement with new noise; everything else
            # gets a fresh placement per setup (the two stations are separate loads).
            load_key = (r.params["part"], r.params["load"]) if r.tier == "grr" else None
            frng = np.random.default_rng(_seed(r.seed, setup, "fixture", *(load_key or ())))
            fx = fixture_error(frng, self.q)
            noise = _seed(r.seed, setup, "noise", r.params.get("trial", 0)) % 100000
            moved = {g: m.copy().apply_transform(fx) for g, m in groups.items()}
            spill = self.root / "_spill" / f"{r.family}_{r.name}_{setup}"
            caps = scan_setup(setup, moved, r.stage, self.cfg, noise, spill)
            del moved
            report = detect(caps, r.stage, ref)
            del caps
            shutil.rmtree(spill, ignore_errors=True)
            sc = score(report, labels, r, seen=self._seen(r, setup))
            sc["measured"] = measured(report, r, labels, self.cfg)
            sc["seconds"] = round(time.perf_counter() - t1, 1)
            sc["fixture"] = {"translation_mm": fx[:3, 3].round(5).tolist(),
                             "yaw_deg": float(np.degrees(np.arctan2(fx[1, 0], fx[0, 0])))}
            (d / f"{setup}.report.json").write_text(report.to_json())
            (d / f"{setup}.score.json").write_text(json.dumps(sc, indent=1))
            line["setups"][setup] = {k: sc[k] for k in INDEX_KEYS}
            loads[setup] = fx
        (d / "case.json").write_text(json.dumps(r.to_json(), indent=1))
        # A setup this run did not do (an earlier `--setups fixed`, say) keeps
        # its stored result in the case's newest index line.
        for setup in SETUPS:
            f = d / f"{setup}.score.json"
            if setup not in line["setups"] and f.is_file():
                sc = json.loads(f.read_text())
                line["setups"][setup] = {k: sc.get(k) for k in INDEX_KEYS}
        line["setups"] = {k: line["setups"][k] for k in sorted(line["setups"], key=SETUPS.index)}
        line["seconds"] = round(time.perf_counter() - t0, 1)
        with open(self.root / r.tier / "index.jsonl", "a") as fh:
            fh.write(json.dumps(line) + "\n")
        return line


SETUPS = ["fixed", "arm"]
INDEX_KEYS = ("outcome", "measured", "false_alarms", "other", "result", "seconds")


def read_index(f: Path) -> dict:
    """<tier>/index.jsonl as {(family, name): row}, one row per case, in the
    order cases last appeared. A case appears more than once when a run is
    resumed or a second setup is run later (`--setups arm` after
    `--setups fixed`): per setup, its latest result wins, so neither setup
    hides the other."""
    out: dict = {}
    for x in Path(f).read_text().splitlines():
        if not x.strip():
            continue
        row = json.loads(x)
        key = (row["family"], row["name"])
        prev = out.pop(key, None)
        if prev is not None:
            row["setups"] = {**prev["setups"], **row["setups"]}
        out[key] = row
    return out


def rescore(root: Path, tier: str, cover: SetupCoverage | None = None) -> int:
    """Score every finished case of `tier` again from its stored labels and
    reports -- no scanning, no detection -- and rewrite its score files and
    <tier>/index.jsonl (same order, one line per case). For when the scoring
    rule changes and the detector does not. Not while that tier is running."""
    cover = cover or SetupCoverage()
    f = Path(root) / tier / "index.jsonl"
    order = read_index(f)
    lines = []
    for (fam, name), row in order.items():
        d = Path(root) / tier / fam / name
        r = Recipe(**json.loads((d / "case.json").read_text()))
        labels = json.loads((d / "labels.json").read_text())
        for setup in row["setups"]:
            old = json.loads((d / f"{setup}.score.json").read_text())
            report = json.loads((d / f"{setup}.report.json").read_text())
            sc = score(report, labels, r,
                       seen=lambda lab, setup=setup: cover.seen(r.stage, setup, lab["cls"], lab["bounds"]))
            sc.update({k: old[k] for k in ("measured", "seconds", "fixture") if k in old})
            (d / f"{setup}.score.json").write_text(json.dumps(sc, indent=1))
            row["setups"][setup] = {k: sc[k] for k in INDEX_KEYS}
        lines.append(json.dumps(row))
    tmp = f.with_suffix(".jsonl.tmp")
    tmp.write_text("\n".join(lines) + "\n")
    tmp.replace(f)
    return len(lines)


def summary(root: Path, tier: str) -> str:
    """Outcomes per family and size, per setup, from <tier>/index.jsonl. No
    statistics here: intervals, POD and %GRR are tasks 1.8, 1.11 and 1.9."""
    from collections import Counter, defaultdict

    f = Path(root) / tier / "index.jsonl"
    if not f.is_file():
        return f"no {f}"
    seen = read_index(f)
    table: dict = defaultdict(lambda: defaultdict(Counter))
    meas: dict = defaultdict(lambda: defaultdict(list))
    fa: Counter = Counter()
    for r in seen.values():
        for setup, v in r["setups"].items():
            table[(r["family"], r["magnitude"], r["truth"])][setup][v["outcome"]] += 1
            if v.get("measured") is not None:
                meas[(r["family"], r["magnitude"], r["truth"])][setup].append(abs(v["measured"]))
            fa[setup] += v.get("false_alarms", 0)
    out = [f"{tier}: {len(seen)} cases; false alarms elsewhere, total: {dict(fa)}",
           f"  {'family':18s} {'size':>7s} {'truth':10s} setup  outcomes                         |measured| median"]
    for (fam, mag, truth), per in sorted(table.items()):
        for setup, c in per.items():
            m = meas[(fam, mag, truth)][setup]
            med = f"{np.median(m):.4f}" if m else "   -"
            out.append(f"  {fam:18s} {mag:7.3g} {truth:10s} {setup:6s} "
                       f"{', '.join(f'{k} {v}' for k, v in sorted(c.items())):32s} {med}")
    return "\n".join(out)


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(prog="python -m defectgen.dataset", description=__doc__.split("\n")[0])
    ap.add_argument("--tier", choices=["pod", "clean", "grr"], required=True)
    ap.add_argument("--family", nargs="+", default=None, help="only these families (pod tier)")
    ap.add_argument("--setups", nargs="+", default=None, choices=["fixed", "arm"])
    ap.add_argument("--pilot", action="store_true",
                    help="one case per family (pod), stage (clean) or characteristic (grr), to time a run")
    ap.add_argument("--list", action="store_true", help="print the cases and exit")
    ap.add_argument("--summary", action="store_true", help="tabulate what this tier has so far, and exit")
    ap.add_argument("--rescore", action="store_true",
                    help="score the finished cases again from their stored reports (no detection), and exit")
    ap.add_argument("--shard", default=None, metavar="I/N",
                    help="run only every N-th case starting at I (0 <= I < N), so N workers split a tier "
                         "with no case run twice (cloud/run_remote.sh)")
    args = ap.parse_args()
    shard = None
    if args.shard is not None:
        try:
            i, n = (int(x) for x in args.shard.split("/"))
        except ValueError:
            ap.error("--shard takes I/N, e.g. 0/14")
        if not 0 <= i < n:
            ap.error("--shard I/N needs 0 <= I < N")
        shard = (i, n)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    for noisy in ("defectgen.detect", "defectgen.scan"):
        logging.getLogger(noisy).setLevel(logging.WARNING)

    q = load_dataset_params()
    dp = load_detect_params()
    if args.summary:
        print(summary(Path(q["out"]), args.tier))
        return
    if args.rescore:
        print(f"{args.tier}: {rescore(Path(q['out']), args.tier)} cases rescored")
        print(summary(Path(q["out"]), args.tier))
        return
    cases = {"pod": lambda: pod_cases(q, dp, args.family), "clean": lambda: clean_cases(q),
             "grr": lambda: grr_cases(q, dp)}[args.tier]()
    if args.pilot:
        seen, pick = set(), []
        for c in cases:
            key = c.family
            # The pilot takes each family's largest size, where every class must work.
            if key not in seen:
                seen.add(key)
                pick.append(max((x for x in cases if x.family == key), key=lambda x: (x.magnitude, -len(x.name))))
        cases = pick
    if shard is not None:
        # Two workers on one case would share its scan scratch folder (SIGBUS, 11 Nov 2026).
        cases = cases[shard[0]::shard[1]]
    if args.list:
        for c in cases:
            print(c.tier, c.family, c.name, c.magnitude, c.stage)
        print(len(cases), "cases")
        return
    runner = Runner(q, args.setups)
    todo = [c for c in cases if not runner.done(c)]
    log.info("%s tier: %d cases, %d already done, %d to run, setups %s -> %s", args.tier, len(cases),
             len(cases) - len(todo), len(todo), runner.setups, runner.root / args.tier)
    t0 = time.perf_counter()
    for i, c in enumerate(todo, 1):
        line = runner.run(c)
        per = "  ".join(f"{s}: {v['outcome']:<12} meas {v['measured'] if v['measured'] is None else round(v['measured'], 4)}"
                        f" fa {v['false_alarms']} ({v['seconds']:.0f} s)" for s, v in line["setups"].items())
        log.info("[%d/%d %3.0f%%] %-18s %-14s %-9s %s", i, len(todo), 100 * i / len(todo), c.family, c.name,
                 line["truth"], per)
    log.info("done: %d cases in %.0f s", len(todo), time.perf_counter() - t0)


if __name__ == "__main__":
    main()

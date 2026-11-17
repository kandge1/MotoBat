"""1.11 (cut-down): how detection on the fixed line moves with the sensor's noise, sampling and
standoff, one factor at a time. Plan §4B.6: "this tells the flange which sensor class is
actually required".

    python -m analysis.sensitivity --setting pitch_0.2      # run one setting (heavy: ~30 min)
    python -m analysis.sensitivity --list                    # the settings and the case count
    python -m analysis.sensitivity --report                  # collate -> data/sensitivity/summary.md

**What changes.** The frozen sensor model (`configs/sensors.yaml`, hashed) is not edited. Each
setting overrides `scan.load_params()` in its own process, so the scan *and* its golden capture
are cast with the same sensor, and the results go to `data/sensitivity/<setting>/`. Each case is a
dataset 1 recipe (same seed, same defect, same fixture draw), so a setting differs from
`baseline` only in the sensor.

- **noise**: `sigma_z` (height repeatability at the reference distance), 0.5×, 2×, 4× of 10 µm.
- **spacing**: `profile_pitch` (conveyor travel per profile, the line scan's X spacing) 0.05 and
  0.2 against 0.1 mm; `points` per line 1280 against 2560 (Y spacing 0.14 against 0.07 mm).
- **standoff**: the *same* camera mounted farther away, at d = 700 or 1000 mm against 500. The fan
  widens with d (line width, so Y spacing, ∝ d), and triangulation noise grows with d² (sigma_z and
  sigma_x ∝ (d/500)², the model's own `noise_distance_exponent`). Closer than ~420 mm the 180 mm
  line no longer spans the 151 mm module, so nearer is not a setting.

**Cases.** The families a sensor class could move: cell shift and yaw, terminal height, busbar
lift-off, short weld, reinforcement (tolerance classes at 0.8, 1.1, 1.3 × tol: the last in-spec
size and the first two out), FOD and spatter (presence classes at the three sizes around D).
Seeds 0–2. 72 cases per setting, about 25 s each on the laptop.

**Read.** Per setting and family, found / n at each size. A class that loses its 1.1 × tol cases
when the noise doubles needs a sensor at least as good as the baseline; one that gains them at
0.05 mm pitch is sampling-limited. Three seeds per size is a screen, not a POD: it shows which
factor matters and roughly how much, and the full sweep (deferred) puts numbers on it.
"""
from __future__ import annotations

import argparse
import json
import logging
from collections import defaultdict
from pathlib import Path

OUT = Path("data/sensitivity")
BASE_D = 500.0

SETTINGS: dict[str, dict] = {
    "baseline": {},
    "noise_x0.5": {"sigma_z": 0.005},
    "noise_x2": {"sigma_z": 0.020},
    "noise_x4": {"sigma_z": 0.040},
    "pitch_0.05": {"profile_pitch": 0.05},
    "pitch_0.2": {"profile_pitch": 0.2},
    "points_1280": {"points": 1280},
    "standoff_700": "standoff:700",
    "standoff_1000": "standoff:1000",
}

FAMILIES = {   # family -> level prefixes (dataset.yaml order: L0 = smallest)
    "cell_shift": ("L1", "L2", "L3"), "cell_yaw": ("L1", "L2", "L3"),
    "terminal_height": ("L1", "L2", "L3"), "busbar_liftoff": ("L1", "L2", "L3"),
    "short_weld": ("L1", "L2", "L3"), "reinforcement": ("L1", "L2", "L3"),
    "fod": ("L2", "L3", "L4"), "spatter": ("L2", "L3", "L4"),
}
SEEDS = ("s0", "s1", "s2")


def override(setting: str) -> dict:
    """The line_scan parameters this setting replaces."""
    spec = SETTINGS[setting]
    if isinstance(spec, dict):
        return dict(spec)
    from defectgen.scan import load_params

    d = float(spec.split(":")[1])
    base = load_params()[0]["line_scan"]
    k = d / BASE_D
    return {"reference_distance": d, "line_width": base["line_width"] * k,
            "sigma_z": base["sigma_z"] * k ** base["noise_distance_exponent"],
            "sigma_x": base["sigma_x"] * k ** base["noise_distance_exponent"]}


def patch(setting: str) -> dict:
    """Make every reader of the sensor parameters see this setting's line scan."""
    from defectgen import coverage, scan

    over = override(setting)
    orig = scan.load_params

    def load_params():
        sensors, materials = orig()
        sensors = dict(sensors)
        sensors["line_scan"] = dict(sensors["line_scan"], **over)
        return sensors, materials

    scan.load_params = load_params
    coverage.load_params = load_params
    try:
        from defectgen import photos
        photos.load_params = load_params
    except ImportError:
        pass
    return over


def cases():
    from defectgen.dataset import load_dataset_params, pod_cases
    from defectgen.detect import load_detect_params

    out = []
    for r in pod_cases(load_dataset_params(), load_detect_params(), list(FAMILIES)):
        lv, seed = r.name.split("_")[0], r.name.rsplit("_", 1)[1]
        if lv in FAMILIES[r.family] and seed in SEEDS:
            out.append(r)
    return out


def run(setting: str) -> None:
    from defectgen.dataset import Runner, load_dataset_params

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    for noisy in ("defectgen.detect", "defectgen.scan"):
        logging.getLogger(noisy).setLevel(logging.WARNING)
    over = patch(setting)
    root = OUT / setting
    root.mkdir(parents=True, exist_ok=True)
    (root / "setting.json").write_text(json.dumps({"setting": setting, "line_scan": over}, indent=1))
    runner = Runner(load_dataset_params(), ["fixed"], root)
    todo = [c for c in cases() if not runner.done(c)]
    logging.info("%s: %d cases to run (%s)", setting, len(todo), over or "baseline")
    for i, c in enumerate(todo, 1):
        v = runner.run(c)["setups"]["fixed"]
        logging.info("[%d/%d] %s %s %s fa %s (%.0f s)", i, len(todo), c.family, c.name, v["outcome"],
                     v["false_alarms"], v["seconds"])


DETECTED = {"hit", "partial", "false_reject", "found"}


def report() -> str:
    from defectgen.validate import rows

    table = defaultdict(dict)   # family -> setting -> level -> [found, n]
    fa = {}
    sizes = {}
    for setting in SETTINGS:
        root = OUT / setting
        if not (root / "pod" / "index.jsonl").is_file():
            continue
        fa[setting] = 0
        for r in rows(root, "pod"):
            v = r["setups"].get("fixed")
            if v is None:
                continue
            lv = r["name"].split("_")[0]
            sizes[(r["family"], lv)] = r["magnitude"]
            cell = table[r["family"]].setdefault(setting, {}).setdefault(lv, [0, 0])
            cell[0] += v["outcome"] in DETECTED
            cell[1] += 1
            fa[setting] += v.get("false_alarms") or 0
    done = [s for s in SETTINGS if s in fa]
    head = "| Family | sizes | " + " | ".join(done) + " |\n|---|---|" + "---|" * len(done) + "\n"
    lines = []
    for fam, lvs in FAMILIES.items():
        sz = " / ".join(f"{sizes.get((fam, l), float('nan')):g}" for l in lvs)
        cells = []
        for s in done:
            c = table[fam].get(s, {})
            cells.append(" ".join(f"{c[l][0]}/{c[l][1]}" if l in c else "–" for l in lvs))
        lines.append(f"| {fam} | {sz} | " + " | ".join(cells) + " |")
    fa_line = "| false alarms | | " + " | ".join(str(fa[s]) for s in done) + " |"
    over = {s: json.loads((OUT / s / "setting.json").read_text())["line_scan"] for s in done}
    settings = "\n".join(f"- `{s}`: {over[s] or 'dataset 1 sensor'}" for s in done)
    return ("# 1.11 sensitivity, fixed line (cut-down)\n\n"
            "Found / n per size (seeds 0–2), one factor changed at a time. Found counts any finding "
            "of the class on it, so at the in-spec size (first) it is a false reject. Method: "
            "`analysis/sensitivity.py`.\n\n" + settings + "\n\n" + head + "\n".join(lines) + "\n" + fa_line + "\n")


def main() -> None:
    ap = argparse.ArgumentParser(prog="python -m analysis.sensitivity")
    ap.add_argument("--setting", choices=list(SETTINGS))
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args()
    if a.list:
        n = len(cases())
        for s in SETTINGS:
            print(s, override(s))
        print(f"{n} cases per setting, {n * len(SETTINGS)} in all")
    elif a.report:
        text = report()
        (OUT / "summary.md").write_text(text)
        print(text)
    elif a.setting:
        run(a.setting)
    else:
        ap.error("--setting, --list or --report")


if __name__ == "__main__":
    main()

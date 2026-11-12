"""Detection regression: re-verify the detector after any change to it.

    python -m defectgen.regress single --sensors line_scan
    python -m defectgen.regress single --sensors prof+ prof-
    python -m defectgen.regress fixture | seeds | subtol | snapshot | overlap
    python -m defectgen.regress arm [--stage post_weld]      # the arm's 1.5 view plan
    python -m defectgen.regress single --setup arm           # any group, through a dataset setup

Groups (task 1.4 and its reviews, project_plan.md §4B.4):

* single   one defect per module, every class, at a size above tolerance
* fixture  a 0.15 / -0.10 / 0.03 mm, 0.05 deg yaw, 0.02 deg roll fixture error
           with a defect; detection must still register to the housing
* seeds    clean modules at every stage, three sensor-noise seeds: zero findings
* subtol   defects just under tolerance: must pass without stray findings
* snapshot the Zivid alone
* overlap  line-scan and profilometer together: no double counting
* width    bead width as measured against as injected, on one stitch pair, for the
           edge-percentile width and the cross-section / crown one; a crown change
           must read as no width change (1.8 found narrow beads read ~1.4x)
* arm      clean modules through the arm's 1.5 view plan (tilted snapshots, rolled
           and side-on sweeps), placed exactly and with the fixture error: zero
           findings. `--setup arm|fixed` sends any other group through a dataset
           setup instead of `--sensors`.

Each case prints one line (TP / FN / FP / other) and its findings; FN lines say
what was found there instead. Lives in the package, not a scratch folder:
the harness that verified the detector must survive to verify it again.
"""

from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path

import numpy as np

from . import defects as D
from .config import load
from .detect import Capture, Reference, detect
from .evaluate import evaluate
from .module import collect_labels, injected_plan, realise_groups
from .parts import load_all
from .scan import Scene, line_scan, looking_down, snapshot, without_groups

CFG = load()
_parts = None
_refs: dict = {}


def parts():
    global _parts
    if _parts is None:
        _parts = load_all(CFG)
    return _parts


def ref(stage):
    """The clean reference for `stage`. Only one is kept: the post-weld one is
    large, and holding all three pushed a profilometer run past 7 GB."""
    if stage not in _refs:
        _refs.clear()
        import gc

        gc.collect()
        _refs[stage] = Reference(stage, CFG, parts=parts())
    return _refs[stage]


def _rot(deg_z=0.0, deg_x=0.0):
    cz, sz = np.cos(np.radians(deg_z)), np.sin(np.radians(deg_z))
    cx, sx = np.cos(np.radians(deg_x)), np.sin(np.radians(deg_x))
    return np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])


SPILL = Path("data/regress_spill")


def scans(groups, sensors, noise=0, setup=None, stage=None):
    """Captures, spilled to disk and memory-mapped back as the dataset does:
    two full-length Keyence sweeps held in RAM with the post-weld reference
    went past 7 GB. With `setup`, exactly what that dataset setup returns."""
    from .dataset import _spill, scan_setup

    shutil.rmtree(SPILL, ignore_errors=True)
    if setup is not None:
        return scan_setup(setup, groups, stage, CFG, noise, SPILL)
    sc = Scene.from_groups(groups)
    out = []
    for s in sensors:
        if s == "line_scan":
            r = line_scan(sc, "line_scan", looking_down(-CFG.module_l / 2 - 5, 0, CFG.z_terminal_top + 500),
                          CFG.module_l + 10, seed=noise)
        elif s.startswith("prof"):
            side = -1 if s.endswith("-") else 1
            r = line_scan(sc, "profilometer", looking_down(-CFG.module_l / 2 + 10, side * CFG.terminal_y,
                                                           CFG.z_terminal_top + 245), CFG.module_l - 20, seed=noise)
        elif s == "snapshot":
            r = snapshot(sc, looking_down(0, 0, CFG.z_wall_top + 2 + 600), seed=noise)
        else:
            raise ValueError(s)
        out.append(_spill(Capture.from_scan(r, name=s), SPILL))
        del r
    return out


def run(stage, defects, sensors, seed=0, noise=0, fixture=None, setup=None):
    plan = injected_plan(stage, CFG, seed, defects, parts())
    labels = json.loads(collect_labels(plan, seed, parts()).to_json())
    groups = {g: m for g, m in realise_groups(plan, parts()).items() if g not in without_groups(stage)}
    if fixture is not None:          # the module sits off its datum: move it, scan from the nominal poses
        t = np.eye(4)
        t[:3, :3] = _rot(fixture[3], fixture[4])
        t[:3, 3] = fixture[:3]
        groups = {g: m.copy().apply_transform(t) for g, m in groups.items()}
    caps = scans(groups, sensors, noise, setup, stage)
    t0 = time.time()
    rep = detect(caps, stage, ref(stage))
    return rep, evaluate(rep, labels), time.time() - t0


def line(tag, rep, ev, td, extra=""):
    t = ev["total"]
    print(f"{tag:46s} TP {t['tp']:2d} FN {t['fn']:2d} FP {t['fp']:2d} other {t['other']:2d} {td:5.1f}s {extra}",
          flush=True)
    for f, v in zip(rep.findings, ev["finding_verdicts"]):
        print(f"     {v:5s} {f.cls:17s} {f.magnitude:8.3f}  {f.feature[:40]:40s} "
              f"{tuple(round(x, 1) for x in f.location)} [{f.capture}]", flush=True)
    for c, s in ev["classes"].items():
        if s.fn:
            print(f"     FN {c} missed as {s.missed_as}", flush=True)


X = CFG.cell_centre_x
FIXTURE = (0.15, -0.10, 0.03, 0.05, 0.02)


def single_cases():
    return [
        ("pre_busbar", "clean", []),
        ("pre_busbar", "cell_placement", [D.cell_placement(4, dx=0.3, dy=0.2, dtheta_deg=0.15)]),
        ("pre_busbar", "terminal_height", [D.terminal_height(6, "pos", dz=0.15)]),
        ("pre_busbar", "terminal_tilt", [D.terminal_tilt(7, "neg", deg=0.5)]),
        ("pre_busbar", "missing_cell", [D.missing_cell(9)]),
        ("pre_busbar", "reversed_polarity", [D.reversed_polarity(1)]),
        ("pre_busbar", "missing_pad", [D.missing_pad(3)]),
        ("pre_busbar", "fod", [D.fod((X(5), 20.0, CFG.z_cell_top + 0.5), radius=0.5)]),
        ("pre_busbar", "handling_damage(top)", [D.handling_damage(("cell", 5), depth=0.3, radius=4.0,
                                                centre=(X(5), 20.0, CFG.z_cell_top), direction=(0, 0, -1))]),
        ("pre_weld", "clean", []),
        ("pre_weld", "busbar_pose", [D.busbar_pose(0, dx=0.3, dtheta_deg=0.1)]),
        ("pre_weld", "busbar_liftoff", [D.busbar_liftoff(2, gap=0.2)]),
        ("pre_weld", "carrier_seating", [D.carrier_seating(0.3)]),
        ("post_weld", "clean", []),
        ("post_weld", "missing_weld", [D.missing_weld(0, 1)]),
        ("post_weld", "short_weld", [D.short_weld(0, 4, fraction=0.8)]),
        ("post_weld", "weld_path_drift", [D.weld_path_drift(1, 5, amp=0.15, wavelength=6.0)]),
        ("post_weld", "weld_arc", [D.weld_arc(2, 8, sagitta=0.2)]),
        ("post_weld", "seam_deviation", [D.seam_deviation(1, 7, dy=0.4)]),
        ("post_weld", "width_deviation", [D.width_deviation(2, 10, factor=1.4)]),
        ("post_weld", "reinforcement", [D.reinforcement(3, 1, dh=-0.15)]),
        ("post_weld", "spatter", [D.spatter(count=10, radius=0.15)]),
        ("post_weld", "liftoff->weld", [D.busbar_liftoff(4, gap=0.5)]),
    ]


def main() -> None:
    ap = argparse.ArgumentParser(prog="python -m defectgen.regress", description=__doc__.split("\n")[0])
    ap.add_argument("group", choices=["single", "fixture", "seeds", "subtol", "snapshot", "overlap", "arm",
                                      "width"])
    ap.add_argument("--sensors", nargs="+", default=["line_scan"])
    ap.add_argument("--setup", choices=["fixed", "arm"], default=None,
                    help="scan through this dataset setup instead of --sensors")
    ap.add_argument("--stage", default=None, help="single: only this stage's cases")
    args = ap.parse_args()
    import logging

    logging.basicConfig(level=logging.WARNING)
    g, sens = args.group, args.sensors
    setup = "arm" if g == "arm" else args.setup
    print(f"group {g}, " + (f"setup {setup}" if setup else f"sensors {sens}"), flush=True)

    def run_(*a, **kw):
        return run(*a, setup=setup, **kw)
    if g == "single":
        for st, name, d in single_cases():
            if args.stage and st != args.stage:
                continue
            rep, ev, td = run_(st, d, sens)
            line(f"{st} {name}", rep, ev, td)
    elif g == "fixture":
        for st, name, d in [("pre_busbar", "fixture + cell_placement", [D.cell_placement(4, dx=0.3, dy=0.0)]),
                            ("pre_busbar", "fixture + terminal_height", [D.terminal_height(6, "pos", dz=0.15)]),
                            ("pre_weld", "fixture + carrier .3", [D.carrier_seating(0.3)]),
                            ("pre_weld", "fixture + busbar_pose", [D.busbar_pose(0, dx=0.3)]),
                            ("post_weld", "fixture + seam_deviation", [D.seam_deviation(1, 7, dy=0.4)]),
                            ("post_weld", "fixture + weld_path_drift .15",
                             [D.weld_path_drift(1, 5, amp=0.15, wavelength=6.0)]),
                            ("post_weld", "fixture clean", [])]:
            rep, ev, td = run_(st, d, sens, fixture=FIXTURE)
            line(f"{st} {name}", rep, ev, td, str(np.round(rep.captures[0].get("icp", {}).get("translation_mm", []), 4)))
    elif g == "seeds":
        for st in ("pre_busbar", "pre_weld", "post_weld"):
            for n in (1, 2, 3):
                rep, ev, td = run_(st, [], sens, noise=n)
                line(f"{st} clean noise={n}", rep, ev, td)
    elif g == "subtol":
        for st, name, d in [("pre_busbar", "terminal_height 0.08", [D.terminal_height(6, "pos", dz=0.08)]),
                            ("pre_busbar", "cell dy 0.15", [D.cell_placement(4, dy=0.15)]),
                            ("pre_weld", "busbar_liftoff 0.1", [D.busbar_liftoff(2, gap=0.1)]),
                            ("pre_weld", "carrier dz 0.1", [D.carrier_seating(0.1)]),
                            ("post_weld", "seam_deviation 0.15", [D.seam_deviation(1, 7, dy=0.15)]),
                            # 8 % short: the bare bead ends once read as dents (1.8)
                            ("post_weld", "short_weld 0.08", [D.short_weld(1, 7, fraction=0.92)])]:
            rep, ev, td = run_(st, d, sens)
            line(f"{st} {name} (under tolerance: expect no findings)", rep, ev, td)
    elif g == "snapshot":
        for st in ("pre_busbar", "pre_weld", "post_weld"):
            rep, ev, td = run_(st, [], ["snapshot"])
            line(f"{st} clean", rep, ev, td, str({k: len(v) for k, v in rep.uncovered.items()}))
    elif g == "arm":
        for st in ("pre_busbar", "pre_weld", "post_weld"):
            if args.stage and st != args.stage:
                continue
            for fx in (None, FIXTURE):
                rep, ev, td = run_(st, [], sens, fixture=fx)
                line(f"{st} clean {'fixture' if fx else 'placed'}", rep, ev, td,
                     str({k: len(v) for k, v in rep.uncovered.items()}))
    elif g == "width":
        idx = [i for i, sm in enumerate(ref("post_weld").labels.seams) if (sm.busbar_index, sm.cell_index) == (1, 7)]
        cases = [(f"width x{f}", [D.width_deviation(1, 7, factor=f)] if f != 1.0 else [], f - 1)
                 for f in (0.6, 0.84, 0.9, 1.0, 1.1, 1.16, 1.4)]
        cases += [(f"crown {dh:+}", [D.reinforcement(1, 7, dh=dh)], 0.0) for dh in (-0.1, 0.1)]
        for fx in (None, FIXTURE):
            for name, d, true in cases:
                rep, ev, td = run_("post_weld", d, sens, fixture=fx)
                m = [rep.measurements["seams"].get(i, {}) for i in idx]
                fmt = lambda k: " ".join(f"{e[k]:+.3f}" if k in e else "  -   " for e in m)  # noqa: E731
                print(f"{name:12s} {'fixture' if fx else 'placed ':7s} true {true:+.2f}  edge {fmt('width_change')}"
                      f"  area/crown {fmt('width_change_area')}  crown {fmt('crown_change')}  "
                      f"findings {sorted({f.cls for f in rep.findings})}", flush=True)
    elif g == "overlap":
        for st, name, d in [("pre_busbar", "clean", []), ("post_weld", "clean", []),
                            ("pre_busbar", "terminal_height .15", [D.terminal_height(6, "pos", dz=0.15)]),
                            ("pre_busbar", "fod on +Y row", [D.fod((X(5), CFG.terminal_y + 13.5,
                                                                    CFG.z_cell_top + 0.5), radius=0.5)]),
                            ("post_weld", "spatter r.3", [D.spatter(count=10, radius=0.3)])]:
            rep, ev, td = run_(st, d, ["line_scan", "prof+"])
            line(f"{st} {name} [line_scan+prof+]", rep, ev, td)


if __name__ == "__main__":
    main()

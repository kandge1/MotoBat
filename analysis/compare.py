"""1.10 (first pass): fixed line vs arm, side by side. Plan §4B.6, §3.7.6.

    python -m analysis.compare [--root data/dataset] [--out data/analysis]

Needs `python -m defectgen.validate pr --setup {fixed,arm}` and `python -m analysis.pod` first.
Every number is read from what those wrote and from 1.5's coverage maps; nothing is recomputed.

Columns, and where each comes from:
- **coverage**: the share of the class's required surface each setup samples finely enough
  (1.5, `data/coverage/<stage>.json`, `fixed` and `arm_plan`). The geometric ceiling: what the
  setup *could* see.
- **recall**: out-of-spec cases found where the setup looks (1.8, Wilson 95 %). POD is
  conditional on coverage, so the two are reported apart, not multiplied.
- **rejected**: the module decision on the same cases (a partial or a wrong-class finding
  still rejects it).
- **a90**: from 1.11 (`analysis/pod.py`), per family; a range when a class has several.
- **captures, scan s, detect s**: per module, by stage. Scan time is 1.5's estimate (the arm's
  planned motion and sweeps, the fixed line's pass); detection is the measured compute time per
  report on this laptop. Phase 2 replaces both with the simulated cell's cycle time.
"""
from __future__ import annotations

import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path

STAGES = ("pre_busbar", "pre_weld", "post_weld")
# FOD and handling damage are injected at pre-busbar only (dataset design, §4B.6).
ONE_STAGE = {"fod": "pre_busbar", "handling_damage": "pre_busbar"}


def coverage_by_class() -> dict[str, dict]:
    out = {}
    for st in STAGES:
        c = json.loads(Path(f"data/coverage/{st}.json").read_text())
        for req, spec in c["requirements"].items():
            for cls in spec["members"]:
                if ONE_STAGE.get(cls, st) != st:
                    continue
                out[cls] = {"stage": st, "requirement": req, "fixed": c["fixed"].get(req),
                            "arm": c["arm_plan"].get(req)}
    return out


def per_module(root: Path) -> dict:
    """setup -> stage -> {captures, detect_s} lists, over pod + clean reports."""
    out = defaultdict(lambda: defaultdict(lambda: {"captures": [], "detect_s": []}))
    for tier in ("pod", "clean"):
        for rep in (root / tier).glob("*/*/*.report.json"):
            setup = rep.name.split(".")[0]
            r = json.loads(rep.read_text())
            d = out[setup][r["stage"]]
            d["captures"].append(len(r["captures"]))
            d["detect_s"].append(sum(r["timings_s"].values()))
    return out


def scan_seconds() -> dict:
    out = {}
    for st in STAGES:
        c = json.loads(Path(f"data/coverage/{st}.json").read_text())
        out[st] = {"fixed": c.get("fixed_seconds"), "arm": c.get("plan_seconds")}
    return out


def _pct(x, ci=None):
    if x is None:
        return "—"
    return f"{100 * x:.0f}%" + (f" [{100 * ci[0]:.0f}–{100 * ci[1]:.0f}]" if ci else "")


def _a90(pod: dict, fams: list[str]) -> str:
    vals = []
    for f in fams:
        fit = pod.get(f, {}).get("fit")
        if fit and fit.get("a90") is not None:
            mark = "*" if pod[f]["non_monotone"] else ""
            vals.append(f"{fit['a90']:.3g}{mark}")
    return ", ".join(vals) or "—"


def build(root: Path, out: Path) -> str:
    pr = {s: json.loads((root / "validation" / f"pr.{s}.json").read_text()) for s in ("fixed", "arm")}
    pod = json.loads((out / "pod.json").read_text())
    cov = coverage_by_class()
    fam_of = defaultdict(list)
    for f, c in pr["fixed"]["family_class"].items():
        fam_of[c].append(f)

    lines = ["| Class | stage | coverage fixed / arm | recall fixed | recall arm | rejected fixed / arm "
             "| a90 fixed | a90 arm |", "|---|---|---|---|---|---|---|---|"]
    for cls in sorted(pr["fixed"]["classes"]):
        f, a = pr["fixed"]["classes"][cls], pr["arm"]["classes"].get(cls, {})
        cv = cov.get(cls, {})
        lines.append(
            f"| {cls} | {cv.get('stage', '—')} | {_pct(cv.get('fixed'))} / {_pct(cv.get('arm'))} | "
            f"{_pct(f.get('recall'), f.get('recall_ci'))} | {_pct(a.get('recall'), a.get('recall_ci'))} | "
            f"{_pct(f.get('rejected'))} / {_pct(a.get('rejected'))} | "
            f"{_a90(pod['fixed'], fam_of[cls])} | {_a90(pod['arm'], fam_of[cls])} |")

    pm, sc = per_module(root), scan_seconds()
    t = ["| stage | captures fixed / arm | scan s fixed / arm (1.5 estimate) | detect s fixed / arm (median, laptop) |",
         "|---|---|---|---|"]
    for st in STAGES:
        f, a = pm["fixed"][st], pm["arm"][st]
        med = lambda v: f"{statistics.median(v):.0f}" if v else "—"
        fs, as_ = sc[st]["fixed"], sc[st]["arm"]
        t.append(f"| {st} | {med(f['captures'])} / {med(a['captures'])} | "
                 f"{fs if fs is None else round(fs)} / {as_ if as_ is None else round(as_)} | "
                 f"{med(f['detect_s'])} / {med(a['detect_s'])} |")

    fa = {s: pr[s]["false_alarms"] for s in ("fixed", "arm")}
    fa_line = " ; ".join(f"{s}: {fa[s]['total']} in {fa[s]['total_modules']} modules "
                         f"(≤ {fa[s]['per_module_upper95']:.4f} per module, 95 %)" for s in fa)
    return ("# 1.10 fixed vs arm (first pass)\n\n"
            f"From `{root}`, 1.5 coverage, 1.8 (`validate pr`), 1.11 (`analysis/pod.py`). Method: "
            "`analysis/compare.py` docstring. a90 marked * is from a non-monotone family: read its "
            "per-size counts in `pod.md` instead. Ideal world (dataset 1): exact robot poses, assumed "
            "sensor models.\n\n" + "\n".join(lines) + "\n\n" + "\n".join(t) + "\n\n"
            f"False alarms (findings on no label): {fa_line}.\n")


def main() -> None:
    ap = argparse.ArgumentParser(prog="python -m analysis.compare")
    ap.add_argument("--root", default="data/dataset")
    ap.add_argument("--out", default="data/analysis")
    a = ap.parse_args()
    text = build(Path(a.root), Path(a.out))
    (Path(a.out) / "compare.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()

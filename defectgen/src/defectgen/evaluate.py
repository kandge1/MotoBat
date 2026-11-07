"""Score a DefectReport against the labels the generator wrote (task 1.4).

Matching rule: a finding and a label match when they are the same class and
their boxes overlap, with `margin` mm of slack (a spatter drop's label box is
a fraction of a millimetre). Each label is matched at most once.

Which labels count depends on the stage. A class is scored at the stage that
inspects it (`INSPECTED`). An earlier class still present later (a misplaced
cell under a welded busbar) is not expected again. A finding that lands on such a
label, or on a label of another class, is not a false positive. It is
recorded as "other". That covers the true downstream effects too: the carrier
a reversed cell lifts, or the weld a lifted busbar spoils.

Internal weld defects (`lack_of_fusion`, plan §4B.2 class 16) are never in
scope: no surface scan can see them, and the write-up says so.

Precision and recall here are plain counts for one module. Confidence
intervals over a seeded dataset are task 1.8.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

INSPECTED = {
    "pre_busbar": {"cell_placement", "terminal_height", "terminal_tilt", "missing_cell",
                   "reversed_polarity", "missing_pad", "fod", "handling_damage"},
    "pre_weld": {"busbar_pose", "busbar_liftoff", "carrier_seating", "fod", "handling_damage"},
    "post_weld": {"missing_weld", "short_weld", "seam_deviation", "width_deviation", "reinforcement",
                  "weld_path_drift", "weld_arc", "spatter", "weld_underfill", "weld_undercut",
                  "burn_through", "fod", "handling_damage"},
}
NOT_VISIBLE = {"lack_of_fusion"}


@dataclass
class ClassScore:
    tp: int = 0
    fn: int = 0
    fp: int = 0
    other: int = 0       # findings of this class that landed on a label of another class
    missed_as: list = field(default_factory=list)  # for each FN: what, if anything, was found there

    @property
    def precision(self) -> float:
        return self.tp / (self.tp + self.fp) if self.tp + self.fp else float("nan")

    @property
    def recall(self) -> float:
        return self.tp / (self.tp + self.fn) if self.tp + self.fn else float("nan")


def _overlap(a, b, margin: float) -> bool:
    (a0, a1), (b0, b1) = a, b
    return bool(np.all(np.asarray(a0) - margin <= np.asarray(b1)) and
                np.all(np.asarray(b0) - margin <= np.asarray(a1)))


def _label_box(d: dict):
    if d.get("bounds"):
        return d["bounds"]
    loc = np.asarray(d["location"], float)
    return (tuple(loc - 1.0), tuple(loc + 1.0))


def _fields(f):
    return (f["cls"], f["bounds"], f["location"]) if isinstance(f, dict) else (f.cls, f.bounds, f.location)


def evaluate(report, labels: dict, margin: float = 1.0) -> dict:
    """Per-class counts for one report against one labels dict (as JSON)."""
    stage = report["stage"] if isinstance(report, dict) else report.stage
    findings = report["findings"] if isinstance(report, dict) else report.findings
    scope = INSPECTED[stage]
    labs = [d for d in labels["defects"] if d["cls"] not in NOT_VISIBLE]
    boxes = [_label_box(d) for d in labs]
    scores: dict[str, ClassScore] = {}
    used = [False] * len(labs)

    finds = [_fields(f) for f in findings]
    # Same-class matches first, nearest label first.
    order = []
    for fi, (cls, fb, floc) in enumerate(finds):
        for li, d in enumerate(labs):
            if d["cls"] == cls and _overlap(fb, boxes[li], margin):
                c = (np.asarray(boxes[li][0]) + np.asarray(boxes[li][1])) / 2
                order.append((float(np.linalg.norm(np.asarray(floc) - c)), fi, li))
    matched_f = set()
    for _, fi, li in sorted(order):
        if fi in matched_f or used[li]:
            continue
        matched_f.add(fi)
        used[li] = True
        scores.setdefault(finds[fi][0], ClassScore()).tp += 1

    verdicts = []          # per finding, in report order: tp / other / fp
    for fi, (cls, fb, _) in enumerate(finds):
        if fi in matched_f:
            verdicts.append("tp")
            continue
        s = scores.setdefault(cls, ClassScore())
        if any(_overlap(fb, b, margin) for b in boxes):
            s.other += 1
            verdicts.append("other")
        else:
            s.fp += 1
            verdicts.append("fp")

    per_label = []         # per in-scope label: found or missed, and what was found there
    for li, d in enumerate(labs):
        if d["cls"] not in scope:
            continue
        there = sorted({finds[fi][0] for fi in range(len(finds)) if _overlap(finds[fi][1], boxes[li], margin)})
        per_label.append({"cls": d["cls"], "bounds": boxes[li], "location": d["location"],
                          "magnitude": d.get("magnitude"), "cause": d.get("cause"),
                          "verdict": "tp" if used[li] else "fn", "found_there": there})
        if used[li]:
            continue
        s = scores.setdefault(d["cls"], ClassScore())
        s.fn += 1
        s.missed_as.append(there or None)

    # TPs on out-of-scope labels are still right, but they are not this stage's job.
    tp = sum(s.tp for c, s in scores.items() if c in scope)
    fn = sum(s.fn for s in scores.values())
    fp = sum(s.fp for s in scores.values())
    return {"stage": stage, "classes": scores, "finding_verdicts": verdicts, "labels": per_label,
            "total": {"tp": tp, "fn": fn, "fp": fp,
                      "other": sum(s.other for s in scores.values()),
                      "out_of_scope_labels": sum(1 for d in labs if d["cls"] not in scope),
                      "not_visible_labels": sum(1 for d in labels["defects"] if d["cls"] in NOT_VISIBLE)}}


def to_json(result: dict) -> str:
    """The evaluation as JSON, for viewers outside the venv (show_detections.py)."""
    import json
    from dataclasses import asdict

    out = dict(result, classes={c: asdict(s) for c, s in result["classes"].items()})
    return json.dumps(out, indent=2, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))


def format_table(result: dict) -> str:
    rows = [f"evaluation ({result['stage']}): class              TP  FN  FP  other  missed as"]
    for cls, s in sorted(result["classes"].items()):
        missed = ", ".join("nothing" if m is None else "/".join(m) for m in s.missed_as)
        rows.append(f"  {cls:30s} {s.tp:3d} {s.fn:3d} {s.fp:3d} {s.other:5d}  {missed}")
    t = result["total"]
    rows.append(f"  {'total (in scope)':30s} {t['tp']:3d} {t['fn']:3d} {t['fp']:3d} {t['other']:5d}"
                f"  ({t['out_of_scope_labels']} labels belong to other stages, "
                f"{t['not_visible_labels']} are internal)")
    return "\n".join(rows)

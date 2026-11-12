"""The validation numbers over the stored 1.7 dataset (plan §4B.6): tasks 1.8
and 1.9.

    python -m defectgen.validate pr  [--setup fixed]   # 1.8: recall, false rejects, precision, false alarms
    python -m defectgen.validate grr [--setup fixed]   # 1.9: %GRR, AIAG crossed ANOVA method

Reads only what `dataset.py` stored per case and setup (index, recipe, labels,
report). Nothing is scanned or detected, so this is light and may run beside
a heavy job. Prints tables and writes `<root>/validation/<what>.<setup>.json`.

**1.8.** The unit is a case: one module, one injected defect, one setup.
- Recall is conditional on the defect lying where the setup looks by design
  (`not_covered` cases are counted, but outside it: MIL-HDBK-1823A).
- A case with several targets where only some were found (`partial`) is not
  a hit. The module was still rejected, so the decision column counts it.
- A false reject is a finding of the injected class on an in-spec part.
- A false alarm is a finding on no label. Every scanned module is an
  opportunity for one: `clean`, `pod` and `grr` alike.

Intervals are Wilson 95% for proportions, and an exact Poisson bound for
false alarms per module. Precision is per finding class, from `pod`: correct
findings against correct + false-reject + false-alarm findings. Findings of
another class on a label (`other`: knock-on effects, cross-talk) are counted
apart, as §4B.4's scoring caveats ask.

**1.9.** AIAG MSA (4th ed.) ANOVA method for a crossed study: parts ×
appraisers × trials, the appraiser being a pallet load (a fresh fixture
placement). Variance components follow from the mean squares. The part ×
load interaction is pooled into repeatability when its F-test p-value
exceeds `ALPHA_INTERACTION`.

%GRR is reported two ways:
- against tolerance: 6 σ_GRR / (USL − LSL);
- against study variation: σ_GRR / σ_total.

The study's parts were spread over ±1.5 tol by design rather than drawn
from a process. So σ_part, %GRR-to-study and ndc describe that design, and
%GRR to tolerance is the figure to judge the gauge by (AIAG's own advice
when the parts do not represent process variation). Bias and linearity
against the injected true value are reported beside it.
"""

from __future__ import annotations

import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

Z95 = 1.959963984540054
ALPHA_INTERACTION = 0.05     # A  pool part x load into repeatability above this p-value (practice: 0.05-0.25)
# Tolerance width per %GRR characteristic, from its detect.yaml tolerance t.
# A gap cannot be negative: its spec runs from 0 to t, one-sided.
TOL_WIDTH = {"terminal_height": lambda t: 2 * t, "cell_position": lambda t: 2 * t, "busbar_gap": lambda t: t}


# -- statistics ------------------------------------------------------------------

def wilson(k: int, n: int, z: float = Z95) -> tuple[float, float]:
    """Wilson score interval for k successes in n trials (95 % by default).
    Unlike the normal interval it stays inside [0, 1] and is not degenerate
    at k = 0 or k = n, which is where most of these cells sit."""
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (0.0 if k == 0 else max(0.0, c - h), 1.0 if k == n else min(1.0, c + h))


def poisson_upper(k: int, exposure: float, conf: float = 0.95) -> float:
    """One-sided upper confidence bound on a Poisson rate, from k events in
    `exposure` units (exact: chi-square). With k = 0 this is -ln(1 - conf) /
    exposure, the 'rule of three'."""
    from scipy.stats import chi2
    return float(chi2.ppf(conf, 2 * k + 2) / 2 / exposure)


def anova_grr(x: np.ndarray, tol_width: float | None = None, alpha: float = ALPHA_INTERACTION) -> dict:
    """AIAG ANOVA gauge R&R for a balanced crossed study.

    `x[i, j, k]` is the reading of part i by appraiser j, trial k. With p
    parts, o appraisers, r trials and grand mean m:

        SS_P  = o r Σ_i (x̄_i − m)²             df p − 1
        SS_O  = p r Σ_j (x̄_j − m)²             df o − 1
        SS_PO = r Σ_ij (x̄_ij − x̄_i − x̄_j + m)² df (p − 1)(o − 1)
        SS_E  = Σ_ijk (x_ijk − x̄_ij)²           df p o (r − 1)

    The variance components, each clamped at 0, are:
    - repeatability: σ²_E = MS_E;
    - interaction: σ²_PO = (MS_PO − MS_E) / r;
    - appraiser: σ²_O = (MS_O − MS_PO) / (p r);
    - part: σ²_P = (MS_P − MS_PO) / (o r).

    If the interaction is not significant (p-value > alpha), it is pooled:
    MS_E' = (SS_PO + SS_E) / (df_PO + df_E), then σ²_E = MS_E',
    σ²_O = (MS_O − MS_E') / (p r) and σ²_P = (MS_P − MS_E') / (o r).

    The study's figures follow from those components:
    - EV = σ_E;
    - AV = sqrt(σ²_O + σ²_PO);
    - GRR = sqrt(EV² + AV²);
    - PV = σ_P;
    - TV = sqrt(GRR² + PV²);
    - ndc = floor(1.41 PV / GRR).

    Standard deviations are in the readings' unit."""
    from scipy.stats import f as fdist

    x = np.asarray(x, dtype=float)
    p, o, r = x.shape
    if r < 2 or p < 2 or o < 2:
        raise ValueError(f"crossed study needs >= 2 parts, appraisers and trials, got {x.shape}")
    m = x.mean()
    xi, xj, xij = x.mean(axis=(1, 2)), x.mean(axis=(0, 2)), x.mean(axis=2)
    ss = {"part": o * r * ((xi - m) ** 2).sum(),
          "appraiser": p * r * ((xj - m) ** 2).sum(),
          "interaction": r * ((xij - xi[:, None] - xj[None, :] + m) ** 2).sum(),
          "repeatability": ((x - xij[:, :, None]) ** 2).sum()}
    df = {"part": p - 1, "appraiser": o - 1, "interaction": (p - 1) * (o - 1), "repeatability": p * o * (r - 1)}
    ms = {k: ss[k] / df[k] for k in ss}
    tiny = 1e-30
    f_int = ms["interaction"] / max(ms["repeatability"], tiny)
    p_int = float(fdist.sf(f_int, df["interaction"], df["repeatability"])) if ms["repeatability"] > tiny else \
        (0.0 if ms["interaction"] > tiny else 1.0)
    pooled = p_int > alpha
    if pooled:
        mse = (ss["interaction"] + ss["repeatability"]) / (df["interaction"] + df["repeatability"])
        var = {"repeatability": mse, "interaction": 0.0,
               "appraiser": max(0.0, (ms["appraiser"] - mse) / (p * r)),
               "part": max(0.0, (ms["part"] - mse) / (o * r))}
    else:
        var = {"repeatability": ms["repeatability"],
               "interaction": max(0.0, (ms["interaction"] - ms["repeatability"]) / r),
               "appraiser": max(0.0, (ms["appraiser"] - ms["interaction"]) / (p * r)),
               "part": max(0.0, (ms["part"] - ms["interaction"]) / (o * r))}
    ev = math.sqrt(var["repeatability"])
    av = math.sqrt(var["appraiser"] + var["interaction"])
    grr = math.hypot(ev, av)
    pv = math.sqrt(var["part"])
    tv = math.hypot(grr, pv)
    out = {"shape": [p, o, r], "ss": ss, "df": df, "ms": ms,
           "interaction_F": f_int, "interaction_p": p_int, "interaction_pooled": pooled,
           "var": var, "EV": ev, "AV": av, "GRR": grr, "PV": pv, "TV": tv,
           "pct_grr_study": 100 * grr / tv if tv > 0 else float("nan"),
           "pct_ev_study": 100 * ev / tv if tv > 0 else float("nan"),
           "pct_av_study": 100 * av / tv if tv > 0 else float("nan"),
           # A gauge that reads every part identically each time has no ndc
           # (rounding in the sums leaves a GRR of ~1e-17, not 0).
           "ndc": int(math.floor(1.41 * pv / grr)) if grr > 1e-9 * pv else None}
    if tol_width:
        out.update(tol_width=tol_width, pct_grr_tol=100 * 6 * grr / tol_width,
                   pct_ev_tol=100 * 6 * ev / tol_width, pct_av_tol=100 * 6 * av / tol_width)
    return out


def bias_linearity(true: np.ndarray, x: np.ndarray) -> dict:
    """Per part, mean reading minus true value (bias), and its straight-line
    trend over the true value (linearity, AIAG): bias = a + b · true. A gauge
    with b ≠ 0 reads big parts and small parts differently."""
    true = np.asarray(true, dtype=float)
    bias = np.asarray(x, dtype=float).reshape(len(true), -1).mean(axis=1) - true
    b, a = np.polyfit(true, bias, 1)
    return {"bias_per_part": bias.tolist(), "mean_bias": float(bias.mean()),
            "mean_bias_se": float(bias.std(ddof=1) / math.sqrt(len(bias))),
            "max_abs_bias": float(np.abs(bias).max()), "slope": float(b), "intercept": float(a)}


# -- reading the dataset ---------------------------------------------------------

def rows(root: Path, tier: str) -> list[dict]:
    """The tier's index, one row per case, every setup's latest result."""
    from .dataset import read_index

    f = Path(root) / tier / "index.jsonl"
    return list(read_index(f).values()) if f.is_file() else []


def _case_dir(root: Path, r: dict) -> Path:
    return Path(root) / r["tier"] / r["family"] / r["name"]


def _load(path: Path):
    return json.loads(path.read_text()) if path.is_file() else None


# -- 1.8 -------------------------------------------------------------------------

def pr(root: Path, setup: str = "fixed") -> dict:
    """Recall, false-reject rate and precision per class and family, and
    false alarms per module, for one setup."""
    from .evaluate import evaluate

    fam: dict = defaultdict(Counter)        # family -> outcome counts
    fam_cls: dict = {}
    decision: dict = defaultdict(Counter)   # family -> NOK among must_find (in view)
    finding: dict = defaultdict(Counter)    # finding class -> tp / false_reject / fp / other
    other_on: dict = defaultdict(Counter)   # finding class -> injected class of the case its 'other' sat in
    readings: dict = defaultdict(list)     # family -> (true size, |reading|)
    modules = Counter()
    false_alarms = Counter()
    missing = []
    for tier in ("pod", "clean", "grr"):
        for r in rows(root, tier):
            s = r["setups"].get(setup)
            if s is None:
                continue
            modules[tier] += 1
            false_alarms[tier] += s.get("false_alarms", 0)
            if tier == "clean":
                fam[f"clean {r['family']}"][s["outcome"]] += 1
                continue
            if tier != "pod":
                continue
            fam[r["family"]][s["outcome"]] += 1
            fam_cls[r["family"]] = r["cls"]
            if s.get("measured") is not None and r.get("magnitude") is not None:
                readings[r["family"]].append((float(r["magnitude"]), abs(float(s["measured"]))))
            if r["truth"] == "must_find" and s["outcome"] != "not_covered":
                decision[r["family"]][s.get("result")] += 1
            elif r["truth"] == "in_spec":
                decision[r["family"]][f"in_spec {s.get('result')}"] += 1
            d = _case_dir(root, r)
            rep, lab = _load(d / f"{setup}.report.json"), _load(d / "labels.json")
            if rep is None or lab is None:
                missing.append(r["name"])
                continue
            ev = evaluate(rep, lab)
            for fnd, v in zip(rep["findings"], ev["finding_verdicts"]):
                c = fnd["cls"]
                if v == "tp" and c == r["cls"]:
                    finding[c]["false_reject" if r["truth"] == "in_spec" else
                               "tp" if r["truth"] == "must_find" else "below_D"] += 1
                else:
                    finding[c][v] += 1
                    if v == "other" or (v == "tp" and c != r["cls"]):
                        other_on[c][r["cls"]] += 1

    out: dict = {"setup": setup, "families": {}, "classes": {}, "clean": {}, "missing_reports": missing,
                 "family_class": dict(fam_cls)}
    by_cls: dict = defaultdict(Counter)
    for f, c in sorted(fam.items()):
        if f.startswith("clean "):
            n = sum(c.values())
            bad = c.get("false_reject", 0)
            out["clean"][f[6:]] = {"modules": n, "with_findings": bad, "ci": wilson(bad, n)}
            continue
        by_cls[fam_cls[f]].update(c)
        out["families"][f] = _rates(c, decision.get(f, Counter()))
    for cls, c in sorted(by_cls.items()):
        dec = Counter()
        for f in fam_cls:
            if fam_cls[f] == cls:
                dec.update(decision.get(f, Counter()))
        row = _rates(c, dec)
        fc = finding.get(cls, Counter())
        denom = fc["tp"] + fc["false_reject"] + fc["fp"]
        row["findings"] = dict(fc)
        row["other_on"] = dict(other_on.get(cls, {}))
        row["precision"] = fc["tp"] / denom if denom else None
        row["precision_ci"] = wilson(fc["tp"], denom) if denom else None
        out["classes"][cls] = row
    out["readings"] = {f: reading_error(v) for f, v in sorted(readings.items()) if len(v) >= 5}
    total_modules = sum(modules.values())
    total_fa = sum(false_alarms.values())
    out["false_alarms"] = {"modules": dict(modules), "count": dict(false_alarms), "total_modules": total_modules,
                           "total": total_fa, "per_module": total_fa / total_modules if total_modules else None,
                           "per_module_upper95": poisson_upper(total_fa, total_modules) if total_modules else None}
    return out


def reading_error(pairs) -> dict:
    """How a family's reading compares with the size injected: |reading| -
    size, its mean and sd, and its slope over the size (with its standard
    error). Recall and decisions can hide a bias this shows at once: a datum
    that moved with the defect took 11 % off every cell shift without
    changing a single cell decision (plan §4B.6)."""
    a = np.asarray(pairs, float)
    true, err = a[:, 0], a[:, 1] - a[:, 0]
    out = {"n": len(a), "mean": float(err.mean()), "sd": float(err.std(ddof=1)),
           "slope": None, "slope_se": None}
    if np.ptp(true) > 0 and len(a) > 2:
        (b, c), cov = np.polyfit(true, err, 1, cov=True)
        out["slope"], out["slope_se"] = float(b), float(np.sqrt(cov[0, 0]))
    return out


def _rates(c: Counter, dec: Counter) -> dict:
    hit, part, miss, nc = c["hit"], c["partial"], c["miss"], c["not_covered"]
    n_in = hit + part + miss
    fr, ok = c["false_reject"], c["pass"]
    nok = dec.get("NOK", 0)
    in_nok, in_n = dec.get("in_spec NOK", 0), sum(v for k, v in dec.items() if k.startswith("in_spec"))
    return {"outcomes": dict(c), "in_view": n_in, "not_covered": nc,
            "recall": hit / n_in if n_in else None, "recall_ci": wilson(hit, n_in) if n_in else None,
            "rejected": nok / n_in if n_in else None, "rejected_ci": wilson(nok, n_in) if n_in else None,
            "in_spec": fr + ok, "false_reject": fr / (fr + ok) if fr + ok else None,
            "false_reject_ci": wilson(fr, fr + ok) if fr + ok else None,
            # The decision on in-spec parts: rejected for any finding, of any class.
            "in_spec_rejected": in_nok / in_n if in_n else None,
            "in_spec_rejected_ci": wilson(in_nok, in_n) if in_n else None}


def _pct(x, ci=None) -> str:
    if x is None:
        return "—"
    s = f"{100 * x:.0f}%"
    return s + (f" [{100 * ci[0]:.0f}–{100 * ci[1]:.0f}]" if ci else "")


def format_pr(res: dict) -> str:
    """Markdown tables, ready for the plan."""
    out = [f"setup `{res['setup']}`. Recall is of out-of-spec defects where the setup looks "
           f"(Wilson 95% in brackets); 'rejected' counts the module decision (NOK), which a partial also earns. "
           f"'False reject' is a finding of the injected class on an in-spec part; 'in spec rejected' is the "
           f"module decision on it, for a finding of any class.",
           "", "| Class | Family | in view | hit / partial / miss | not covered | recall | rejected | "
               "in spec | false reject | in spec rejected |", "|---|---|---|---|---|---|---|---|---|---|"]
    fam_rows = res["families"]
    for cls, row in res["classes"].items():
        fams = [f for f in fam_rows if res["family_class"].get(f) == cls]
        for f in fams:
            v = fam_rows[f]
            o = v["outcomes"]
            out.append(f"| {cls if f == fams[0] else ''} | {f} | {v['in_view']} | "
                       f"{o.get('hit', 0)} / {o.get('partial', 0)} / {o.get('miss', 0)} | {v['not_covered'] or ''} | "
                       f"{_pct(v['recall'], v['recall_ci'])} | {_pct(v['rejected'], v['rejected_ci'])} | "
                       f"{v['in_spec'] or ''} | {_pct(v['false_reject'], v['false_reject_ci']) if v['in_spec'] else ''} | "
                       f"{_pct(v['in_spec_rejected'], v['in_spec_rejected_ci']) if v['in_spec'] else ''} |")
    out += ["", "| Class | correct findings | false rejects | false alarms | other | precision |", "|---|---|---|---|---|---|"]
    for cls, row in res["classes"].items():
        fc = row["findings"]
        out.append(f"| {cls} | {fc.get('tp', 0)} | {fc.get('false_reject', 0)} | {fc.get('fp', 0)} | "
                   f"{fc.get('other', 0)} | {_pct(row['precision'], row['precision_ci'])} |")
    others = [f"{c} on {', '.join(f'{k} {n}' for k, n in sorted(r['other_on'].items()))}"
              for c, r in res["classes"].items() if r.get("other_on")]
    if others:
        out += ["", "Other findings, by the injected class of the case they sat in (knock-on or cross-talk): "
                + "; ".join(others) + "."]
    if res.get("readings"):
        out += ["", "Readings against the size injected: |reading| − size, ×1000 in the reading's unit (µm; m° for "
                    "yaw and tilt; ‰ of golden width for width), over every `pod` case "
                    "with a reading (in spec and out); slope is the error's trend over the size (0 = proportional "
                    "error free; −0.1 = reads 10 % short per unit size).", "",
                "| Family | n | mean error | sd | slope ± se |", "|---|---|---|---|---|"]
        for f, v in res["readings"].items():
            sl = f"{v['slope']:+.3f} ± {v['slope_se']:.3f}" if v["slope"] is not None else "—"
            out.append(f"| {f} | {v['n']} | {1000 * v['mean']:+.1f} | {1000 * v['sd']:.1f} | {sl} |")
    fa = res["false_alarms"]
    out += ["", f"False alarms (findings on no label): {fa['total']} in {fa['total_modules']} modules "
                f"({', '.join(f'{k} {v}' for k, v in fa['modules'].items())}); "
                f"rate ≤ {fa['per_module_upper95']:.4f} per module (95% upper bound)."]
    if res["clean"]:
        out.append("Clean modules with any finding: " + ", ".join(
            f"{k} {v['with_findings']}/{v['modules']}" for k, v in res["clean"].items()) + ".")
    if res["missing_reports"]:
        out.append(f"Cases without a stored report (left out of precision): {len(res['missing_reports'])}.")
    return "\n".join(out)


# -- 1.9 -------------------------------------------------------------------------

def grr(root: Path, setup: str = "fixed") -> dict:
    """%GRR per characteristic of the `grr` tier, for one setup."""
    by_char: dict = defaultdict(dict)
    for r in rows(root, "grr"):
        s = r["setups"].get(setup)
        case = _load(_case_dir(root, r) / "case.json")
        if s is None or case is None:
            continue
        char = r["name"].rsplit("_p", 1)[0]
        q = case["params"]
        true = {"terminal_height": q.get("dz"), "cell_shift": q.get("along_x"),
                "busbar_liftoff": q.get("gap")}[r["family"]]
        by_char[char][(q["part"], q["load"], q["trial"])] = (s.get("measured"), true, case["tol"])
    out = {"setup": setup, "characteristics": {}}
    for char, cells in sorted(by_char.items()):
        parts = sorted({k[0] for k in cells})
        loads = sorted({k[1] for k in cells})
        trials = sorted({k[2] for k in cells})
        # A balanced design: a part with any reading missing is left out whole.
        keep = [pi for pi in parts if all(cells.get((pi, li, ti), (None,))[0] is not None
                                          for li in loads for ti in trials)]
        x = np.array([[[cells[(pi, li, ti)][0] for ti in trials] for li in loads] for pi in keep])
        true = np.array([cells[(pi, loads[0], trials[0])][1] for pi in keep])
        tol = cells[(keep[0], loads[0], trials[0])][2]
        width = TOL_WIDTH[char](tol)
        res = anova_grr(x, width)
        res.update(characteristic=char, tol=tol, parts_dropped=[pi for pi in parts if pi not in keep],
                   true_values=true.tolist(), readings=x.tolist(), **{"bias": bias_linearity(true, x)})
        out["characteristics"][char] = res
    return out


def format_grr(res: dict) -> str:
    out = [f"setup `{res['setup']}`. AIAG ANOVA method; appraiser = pallet load. σ in µm. "
           "AIAG guide: %GRR < 10 % acceptable, 10–30 % conditional, > 30 % not acceptable; ndc ≥ 5.", "",
           "| Characteristic | parts × loads × trials | EV σ | AV σ | GRR σ | %GRR tol (width) | "
           "%GRR study | ndc | interaction p | mean bias | linearity slope |",
           "|---|---|---|---|---|---|---|---|---|---|---|"]
    for char, v in res["characteristics"].items():
        p, o, r = v["shape"]
        out.append(f"| {char} | {p} × {o} × {r} | {1e3 * v['EV']:.1f} | {1e3 * v['AV']:.1f} | {1e3 * v['GRR']:.1f} | "
                   f"{v['pct_grr_tol']:.1f} % ({v['tol_width']:g} mm) | {v['pct_grr_study']:.1f} % | {v['ndc']} | "
                   f"{v['interaction_p']:.3f}{' pooled' if v['interaction_pooled'] else ''} | "
                   f"{1e3 * v['bias']['mean_bias']:+.1f} ± {1e3 * v['bias']['mean_bias_se']:.1f} µm | "
                   f"{v['bias']['slope']:+.4f} |")
        if v["parts_dropped"]:
            out.append(f"|  | parts dropped (a reading missing): {v['parts_dropped']} |||||||||| ")
    return "\n".join(out)


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(prog="python -m defectgen.validate")
    ap.add_argument("what", choices=["pr", "grr"])
    ap.add_argument("--setup", default="fixed", choices=["fixed", "arm"])
    ap.add_argument("--root", default="data/dataset")
    args = ap.parse_args()
    root = Path(args.root)
    res = pr(root, args.setup) if args.what == "pr" else grr(root, args.setup)
    text = format_pr(res) if args.what == "pr" else format_grr(res)
    print(text)
    out = root / "validation" / f"{args.what}.{args.setup}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()

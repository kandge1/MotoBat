"""1.11 (first pass): probability of detection per defect family, both setups. Plan §4B.6.

    python -m analysis.pod [--root data/dataset] [--out data/analysis]

Reads the `pod` tier's stored scores. Nothing is scanned or detected (~seconds).

## What is fitted, from first principles

Each `pod` case is one injected defect of size a (its family's magnitude) and one Bernoulli
outcome y: did the setup report a finding of that class on it. Detected means a same-class
finding on the label, whatever the truth: `hit`, `partial` (some of several targets, e.g. one
spatter drop of five), `false_reject` (an in-spec part flagged) and `found` (a presence defect
below D) all count. Not detected: `miss`, `pass`, `not_found`. `not_covered` is left out: POD is
conditional on the location being inspected (MIL-HDBK-1823A, §1.10 reports coverage beside it).

The model is the standard hit/miss POD curve (MIL-HDBK-1823A §3): a logistic in log size,

    POD(a) = 1 / (1 + exp(−(b0 + b1·ln a)))

so POD(a50) = 0.5 at ln a50 = −b0/b1, and POD(a90) = 0.9 at ln a90 = (ln 9 − b0)/b1. Log size
because a detector's evidence grows with the defect multiplicatively (twice the size is the same
step at any scale), and it keeps a > 0.

**Why Firth.** Maximum likelihood fails on most families here. If every case below some size is
missed and every one above is found ("complete separation"), the likelihood keeps rising as b1 →
∞: the fitted curve becomes a step and the MLE does not exist. With 5 seeds per size that is the
usual case for a good detector. Firth (1993) maximises l(b) + ½·ln det I(b), the log-likelihood plus
a Jeffreys-prior penalty. The penalty grows without bound as the curve steepens, so the estimate is
always finite. It also removes the O(1/n) bias of the MLE, which matters at n ≈ 25–35. Without
separation it is close to the MLE.

**a90/95.** The size at which POD is 0.9 *with 95 % confidence*: the one-sided 95 % lower bound
of the curve reaches 0.9 there. The bound is Wald on the linear predictor η = b0 + b1·ln a:
η_low = η − 1.645·√(xᵀVx), x = (1, ln a), V = I(b)⁻¹ at the Firth estimate (the delta method,
exact for η because η is linear in b). MIL-HDBK-1823A prefers the likelihood-ratio bound; Wald is
the simpler first pass and is wider under near-separation, so it errs on the safe side.

a90/95 is reported only inside the sizes tested. Past the largest size it is "not shown": the
curve there is extrapolation, not evidence.

**The nonparametric check.** If every one of the n cases at sizes ≥ a was detected, then POD(a′)
≥ 0.05^(1/n) for every a′ ≥ a with 95 % confidence (the binomial bound for n of n; the "29 of 29"
rule is this with n = 29 giving 0.902). It needs no curve shape and is shown beside the fit.

**For tolerance classes** (cell shift, terminal height, welds, ...) the curve is the gauge's
operating characteristic: the ideal is a step at the tolerance t. a50 near t means the reading is
unbiased at the threshold; a90 − a50 is how much the gauge's noise spreads it. The two in-spec
sizes (0.5 t, 0.8 t) count: a detection there is a false reject and pulls a50 down.
"""
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

DETECTED = {"hit", "partial", "false_reject", "found"}
NOT_DETECTED = {"miss", "pass", "not_found"}
Z_ONE_SIDED_95 = 1.6448536269514722
LOGIT90 = math.log(9.0)
SETUPS = ("fixed", "arm")


# -- the fit ---------------------------------------------------------------------

def firth_logistic(x: np.ndarray, y: np.ndarray, iters: int = 100, tol: float = 1e-10):
    """Firth-penalised logistic regression of y (0/1) on x (one regressor plus intercept).
    Returns (b, V): coefficients [b0, b1] and the covariance I(b)^-1 at b.

    Newton on the modified score U*(b) = Xᵀ(y − p + h(½ − p)), h the diagonal of the hat
    matrix W^½ X (XᵀWX)^-1 Xᵀ W^½, W = diag p(1 − p) (Firth 1993; Heinze & Schemper 2002).
    Steps are halved while the penalised log-likelihood falls."""
    X = np.column_stack([np.ones_like(x), x])
    b = np.zeros(2)

    def info(b):
        p = 1.0 / (1.0 + np.exp(-X @ b))
        w = p * (1.0 - p)
        return p, w, X.T @ (X * w[:, None])

    def penalised(b):
        p, _, I = info(b)
        p = np.clip(p, 1e-300, 1 - 1e-16)
        sign, logdet = np.linalg.slogdet(I)
        return float(np.sum(y * np.log(p) + (1 - y) * np.log1p(-p)) + 0.5 * logdet) if sign > 0 else -np.inf

    cur = penalised(b)
    for _ in range(iters):
        p, w, I = info(b)
        Iinv = np.linalg.inv(I)
        h = w * np.einsum("ij,jk,ik->i", X, Iinv, X)
        step = Iinv @ (X.T @ (y - p + h * (0.5 - p)))
        t = 1.0
        while True:
            nb = b + t * step
            new = penalised(nb)
            if new >= cur - 1e-12 or t < 1e-6:
                break
            t /= 2
        b, cur = nb, new
        if np.max(np.abs(t * step)) < tol:
            break
    return b, np.linalg.inv(info(b)[2])


def separated(x: np.ndarray, y: np.ndarray) -> bool:
    """Complete or quasi-complete separation in one regressor: the MLE would not exist."""
    if y.min() == y.max():
        return True
    return bool(x[y == 0].max() <= x[y == 1].min() or x[y == 1].max() <= x[y == 0].min())


def size_at(b, logit: float) -> float:
    return math.exp((logit - b[0]) / b[1]) if b[1] > 0 else math.nan


def a90_95(b, V, lo: float, hi: float, n: int = 4000) -> float | None:
    """Smallest size in [lo, hi] where the one-sided 95 % Wald lower bound of POD is ≥ 0.9."""
    la = np.linspace(math.log(lo), math.log(hi), n)
    X = np.column_stack([np.ones_like(la), la])
    eta = X @ b
    se = np.sqrt(np.einsum("ij,jk,ik->i", X, V, X))
    ok = eta - Z_ONE_SIDED_95 * se >= LOGIT90
    if not ok.any():
        return None
    i = int(np.argmax(ok))
    return float(math.exp(la[i]))


def nonparametric(levels: list[dict]) -> dict | None:
    """The smallest tested size from which every larger case was detected, and the 95 %
    lower bound on POD that n of n gives (0.05^(1/n))."""
    best = None
    n = 0
    for lv in sorted(levels, key=lambda l: -l["size"]):
        if lv["detected"] < lv["n"]:
            break
        n += lv["n"]
        best = {"from_size": lv["size"], "n": n, "pod_lower95": 0.05 ** (1.0 / n)}
    return best


# -- the data --------------------------------------------------------------------

def cases(root: Path, setup: str) -> dict[str, list[dict]]:
    """family -> [{size, y, truth, outcome, name}] for one setup, not_covered left out."""
    from defectgen.validate import rows

    out = defaultdict(list)
    for r in rows(root, "pod"):
        s = r["setups"].get(setup)
        if s is None or s["outcome"] == "not_covered":
            continue
        o = s["outcome"]
        if o not in DETECTED and o not in NOT_DETECTED:
            raise ValueError(f"unknown outcome {o!r} in {r['family']}/{r['name']}")
        out[r["family"]].append({"size": float(r["magnitude"]), "y": int(o in DETECTED),
                                 "truth": r["truth"], "outcome": o, "name": r["name"], "cls": r["cls"]})
    return out


def threshold(cs: list[dict]) -> float | None:
    """The family's tolerance t or D: the levels are 0.5 t, 0.8 t | 1.1 t, ... for tolerance
    classes (dataset.yaml), so t is the smallest must_find size / 1.1; for presence classes the
    smallest must_find size is D itself."""
    must = sorted({c["size"] for c in cs if c["truth"] == "must_find"})
    if not must:
        return None
    kinds = {c["truth"] for c in cs}
    return must[0] / 1.1 if "in_spec" in kinds else must[0]


def analyse(cs: list[dict]) -> dict:
    by = defaultdict(lambda: [0, 0])
    for c in cs:
        by[c["size"]][0] += 1
        by[c["size"]][1] += c["y"]
    levels = [{"size": a, "n": n, "detected": d} for a, (n, d) in sorted(by.items())]
    t = threshold(cs)
    res = {"levels": levels, "threshold": t, "kind": "in_spec" in {c["truth"] for c in cs} and "tolerance"
           or "dont_care" in {c["truth"] for c in cs} and "presence" or "binary",
           "n": len(cs), "detected": sum(c["y"] for c in cs)}
    # A larger size found clearly less often than a smaller one: the curve's shape assumption fails.
    res["non_monotone"] = [(lo["size"], hi["size"]) for lo, hi in zip(levels, levels[1:])
                           if lo["detected"] / lo["n"] - hi["detected"] / hi["n"] >= 0.4]
    res["nonparametric"] = nonparametric(levels)
    if len(levels) < 2:
        return res
    x = np.log([c["size"] for c in cs])
    y = np.array([c["y"] for c in cs], float)
    if y.min() == y.max():
        res["fit"] = None
        res["note"] = "all detected" if y[0] else "none detected"
        return res
    b, V = firth_logistic(x, y)
    lo, hi = levels[0]["size"], levels[-1]["size"]
    res["fit"] = {"b": b.tolist(), "V": V.tolist(), "separated": separated(x, y),
                  "a50": size_at(b, 0.0), "a90": size_at(b, LOGIT90), "a90_95": a90_95(b, V, lo, hi)}
    return res


def pod_curve(b, a: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-(b[0] + b[1] * np.log(a))))


# -- output ----------------------------------------------------------------------

def _f(x, nd=3) -> str:
    return "—" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.{nd}f}"


def table(res: dict[str, dict[str, dict]]) -> str:
    fams = sorted({f for s in res.values() for f in s})
    head = ("| Family | kind | t or D | setup | per size: found / n | a50 | a90 | a90/95 | "
            "all found from (95 % lower POD) | flags |\n|---|---|---|---|---|---|---|---|---|---|\n")
    lines = []
    for f in fams:
        for s in SETUPS:
            r = res.get(s, {}).get(f)
            if r is None:
                continue
            lv = "  ".join(f"{l['size']:g}: {l['detected']}/{l['n']}" for l in r["levels"])
            fit = r.get("fit")
            npar = r["nonparametric"]
            np_s = f"{npar['from_size']:g} ({100 * npar['pod_lower95']:.0f} %, n={npar['n']})" if npar else "—"
            flags = []
            if fit and fit["separated"]:
                flags.append("separated (Firth)")
            if fit and fit["a90_95"] is None:
                flags.append("a90/95 not shown in range")
            if r["non_monotone"]:
                flags.append("**not monotone** " + ", ".join(f"{a:g}→{b:g}" for a, b in r["non_monotone"]))
            if r.get("note"):
                flags.append(r["note"])
            lines.append(f"| {f} | {r['kind']} | {_f(r['threshold'])} | {s} | {lv} | "
                         f"{_f(fit and fit['a50'])} | {_f(fit and fit['a90'])} | {_f(fit and fit['a90_95'])} | "
                         f"{np_s} | {'; '.join(flags)} |")
    return head + "\n".join(lines)


COLOR = {"fixed": "#2a78d6", "arm": "#eb6834"}   # dataviz default categorical, slots 1 and 2


def figure(res: dict, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fams = sorted(f for f in {f for s in res.values() for f in s}
                  if len(next(iter(res.values()))[f]["levels"]) > 1)
    ncol = 4
    nrow = math.ceil(len(fams) / ncol)
    fig, axes = plt.subplots(nrow, ncol, figsize=(13, 2.7 * nrow), squeeze=False)
    ink, muted, grid = "#1f2328", "#59636e", "#d8dee4"
    for ax, f in zip(axes.flat, fams):
        for i, s in enumerate(SETUPS):
            r = res[s].get(f)
            if r is None:
                continue
            sizes = np.array([l["size"] for l in r["levels"]])
            rate = np.array([l["detected"] / l["n"] for l in r["levels"]])
            jitter = 1.0 + (0.015 if i else -0.015)
            ax.plot(sizes * jitter, rate, "o", ms=6, color=COLOR[s], mec="white", mew=1.5, zorder=3,
                    label=s)
            fit = r.get("fit")
            if fit:
                a = np.geomspace(sizes.min(), sizes.max(), 200)
                # A curve that misdescribes its counts is drawn dotted: read the dots there.
                ax.plot(a, pod_curve(fit["b"], a), color=COLOR[s], lw=2, zorder=2,
                        ls=(0, (1, 2)) if r["non_monotone"] else "-")
        t = next(r for r in (res[s].get(f) for s in SETUPS) if r)["threshold"]
        if t:
            ax.axvline(t, color=muted, lw=1, ls=(0, (3, 3)), zorder=1)
        ax.set_xscale("log")
        ax.set_ylim(-0.05, 1.05)
        ax.set_yticks([0, 0.5, 0.9, 1])
        ax.grid(axis="y", color=grid, lw=0.8)
        ax.set_title(f, fontsize=10, color=ink, loc="left")
        ax.tick_params(colors=muted, labelsize=8)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            ax.spines[sp].set_color(grid)
        ax.xaxis.set_major_formatter(matplotlib.ticker.FormatStrFormatter("%g"))
        ax.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        ax.set_xticks(sorted({l["size"] for s in SETUPS if res[s].get(f) for l in res[s][f]["levels"]}))
        ax.tick_params(axis="x", labelrotation=45)
    for ax in list(axes.flat)[len(fams):]:
        ax.axis("off")
    h, l = axes.flat[0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper right", frameon=False, fontsize=10, ncol=2)
    fig.suptitle("POD per family: share of cases found per size (dots), Firth logistic in log size (lines); "
                 "dotted = not monotone, read the dots; dashed grey = tolerance t or D", x=0.01, ha="left", fontsize=11, color=ink)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(prog="python -m analysis.pod")
    ap.add_argument("--root", default="data/dataset")
    ap.add_argument("--out", default="data/analysis")
    a = ap.parse_args()
    root, out = Path(a.root), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    res = {s: {f: analyse(cs) for f, cs in sorted(cases(root, s).items())} for s in SETUPS}
    text = ("# 1.11 POD per family (first pass)\n\n"
            f"From `{root}/pod`. Method: `analysis/pod.py` docstring. Sizes in the family's unit (mm, "
            "deg for yaw and tilt, fraction of golden width for width). Flags: *separated* = every "
            "miss is below every find, the MLE does not exist and Firth's estimate stands in; *not "
            "monotone* = a larger size found ≥ 40 points less often than the one below it, so the "
            "curve misdescribes it and the per-size counts are the result.\n\n" + table(res) + "\n")
    (out / "pod.md").write_text(text)
    (out / "pod.json").write_text(json.dumps(res, indent=1))
    figure(res, out / "pod.png")
    print(text)
    print(f"wrote {out}/pod.md, pod.json, pod.png")


if __name__ == "__main__":
    main()

"""Task 1.12 results: K-sweep, open- vs closed-loop placement defect rate, reject/rework logic.

    python -m correction.run [--setup fixed|arm] [--runs N] [--out DIR]

Writes to data/correction/<setup>/: summary.md (the numbers), results.json, step.png, sweep.png,
spread.png. Light: ~1 min, < 1 GB. The measurement noise comes from the newest 1.9 %GRR result, so
rerun it after v3's validate grr to refresh the numbers.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from correction import loop as L  # noqa: E402

# Default dataviz palette, light surface: categorical slots in fixed order.
SURF, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]


def style(ax, title, xlabel, ylabel):
    ax.set_facecolor(SURF)
    ax.set_title(title, loc="left", color=INK, fontsize=11)
    ax.set_xlabel(xlabel, color=INK2)
    ax.set_ylabel(ylabel, color=INK2)
    ax.grid(True, color=GRID, lw=0.8)
    ax.tick_params(colors=INK2)
    for s in ax.spines.values():
        s.set_visible(False)


def figure(n=1, w=9, h=4.2):
    fig, axs = plt.subplots(1, n, figsize=(w, h), facecolor=SURF)
    return fig, np.atleast_1d(axs)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--setup", default="fixed", choices=["fixed", "arm"])
    ap.add_argument("--runs", type=int, default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    t0 = time.time()
    S = L.load(a.setup)
    p, cfg, tol = S.plant, S.cfg, S.tol
    cells, runs = cfg["cells"], a.runs or cfg["runs"]
    out = Path(a.out or L.ROOT / f"data/correction/{a.setup}")
    out.mkdir(parents=True, exist_ok=True)
    k0, k1, dk = cfg["sweep"]["k"]
    f0, f1, df = cfg["sweep"]["k_fine"]
    ks = np.unique(np.round(np.r_[np.arange(k0, k1 + dk / 2, dk), np.arange(f0, f1, df)], 6))
    res = {"setup": a.setup, "grr_source": S.grr_source, "cells": cells, "runs": runs,
           "tol": tol.tolist(), "sigma_place": p.sigma_place.tolist(), "sigma_meas": p.sigma_meas.tolist(),
           "bias_meas": p.bias_meas.tolist(), "offset": p.offset.tolist(), "ramp": p.ramp.tolist(),
           "step_at": p.step_at, "step": p.step.tolist()}

    # 1. step responses, noise off: what each gain does to one offset
    fig, (ax,) = figure()
    style(ax, f"x error after a 0.15 mm gripper offset, no noise (I form, u₊ = u − K·e);\n"
              f"it settles at {-p.bias_meas[0]:+.3f} mm, minus the gauge bias", "cell k", "x error (mm)")
    n = 14
    for c, k in zip(SERIES, cfg["sweep"]["show"]):
        x = L.simulate(p, L.I(k), n, 1, noise=False)["x"][0, :, 0]
        kind = ("monotone" if k < 1 else "deadbeat" if k == 1 else "oscillating" if k < 2 else "diverging")
        ax.plot(range(n), x, "-o", color=c, lw=2, ms=4, label=f"K = {k}  {kind}")
    xp = L.simulate(p, L.P(0.5), n, 1, noise=False)["x"][0, :, 0]
    ax.plot(range(n), xp, "--o", color=SERIES[4], lw=2, ms=4, label="P form, K = 0.5: keeps d/(1+K)")
    ax.axhspan(-tol[0], tol[0], color=GRID, alpha=0.5, lw=0)
    ax.set_ylim(-0.45, 0.45)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2, loc="lower left")
    fig.tight_layout(); fig.savefig(out / "step.png", dpi=130); plt.close(fig)

    # 2. the sweep: defect rate over the shift (drift, gripper change, noise)
    skip = 0
    rate = lambda g, gains: L.defect_rate(L.simulate(p, gains, cells, runs, seed=7, g=g)["x"], tol, skip)
    open_r = rate(p.g, L.OPEN)
    sweep = {"I": [], "P": []}
    for k in ks:
        sweep["I"].append(rate(p.g, L.I(k)))
        sweep["P"].append(rate(p.g, L.P(k)) if k < 1.0 else {"rate": 1.0, "lo": 1.0, "hi": 1.0})
    g_lo, g_hi = cfg["plant"]["g_range"]
    sweep[f"I, g={g_lo}"] = [rate(g_lo, L.I(k)) for k in ks]
    sweep[f"I, g={g_hi}"] = [rate(g_hi, L.I(k)) for k in ks]
    res["open_loop"] = open_r
    res["sweep"] = {"k": ks.tolist(), **{n_: [r["rate"] for r in v] for n_, v in sweep.items()}}
    iI = int(np.argmin([r["rate"] for r in sweep["I"]]))
    k_best = float(ks[iI])
    i1 = int(np.argmin(np.abs(ks - 1.0)))
    res["best_I"] = {"k": k_best, **sweep["I"][iI]}
    res["deadbeat_I"] = {"k": 1.0, **sweep["I"][i1]}

    fig, (ax,) = figure()
    style(ax, f"Placement defect rate over a {cells}-cell shift ({runs} runs, 95 % CI)",
          "loop gain K", "cells out of tolerance (log)")
    for c, (name, v) in zip(SERIES, sweep.items()):
        r = np.array([x["rate"] for x in v]); lo = np.array([x["lo"] for x in v]); hi = np.array([x["hi"] for x in v])
        m = r < 1.0
        ax.plot(ks[m], np.maximum(r[m], 1e-5), color=c, lw=2, label=f"{name} form" if "," not in name else name)
        ax.fill_between(ks[m], np.maximum(lo[m], 1e-5), np.maximum(hi[m], 1e-5), color=c, alpha=0.15, lw=0)
    ax.axhline(open_r["rate"], color=INK2, lw=1.5, ls=":", label=f"open loop {open_r['rate']:.1%}")
    ax.axvline(2.0, color=INK2, lw=0.8)
    ax.text(2.0, 0.5, " 2/g", color=INK2, fontsize=8)
    ax.set_yscale("log"); ax.set_ylim(1e-5, 1.2)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2, loc="lower right")
    fig.tight_layout(); fig.savefig(out / "sweep.png", dpi=130); plt.close(fig)

    # 3. noise only: spread of x vs K, closed form against Monte Carlo (Deming's funnel)
    pn = L.Plant(p.g, p.sigma_place, 0 * p.offset, 0 * p.ramp, p.step_at, 0 * p.step, p.sigma_meas, 0 * p.bias_meas)
    ksn = ks[(ks > 0) & (ks < 2)]
    mc = [float(L.simulate(pn, L.I(k), 1500, 40, seed=11)["x"][:, 300:, 0].std()) for k in ksn]
    an = [math.sqrt(L.steady_var(0, k, p.sigma_place[0], p.sigma_meas[0])) for k in ksn]
    res["spread_x"] = {"k": ksn.tolist(), "monte_carlo": mc, "closed_form": an}
    fig, (ax,) = figure()
    style(ax, "Spread of x with noise only: σ² = σ_w² + K(σ_w² + σ_v²)/(2 − K)", "loop gain K", "1σ of x (mm)")
    ax.plot(ksn, an, color=SERIES[0], lw=2, label="closed form")
    ax.plot(ksn, mc, "o", color=SERIES[1], ms=4, label="Monte Carlo")
    ax.axhline(p.sigma_place[0], color=INK2, ls=":", lw=1.5, label=f"open loop σ_w = {p.sigma_place[0]} mm")
    ax.set_ylim(0, min(max(an) * 1.1, 0.3))
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2, loc="upper left")
    fig.tight_layout(); fig.savefig(out / "spread.png", dpi=130); plt.close(fig)

    # 4. closed-form steady state per axis at the best gain
    axes = []
    for i, ax_ in enumerate(L.AXES):
        a_ = p.g * k_best
        mean = L.steady_mean(0, a_, 0, p.ramp[i], p.bias_meas[i])
        sd = math.sqrt(L.steady_var(0, a_, p.sigma_place[i], p.sigma_meas[i]))
        axes.append({"axis": ax_, "mean": mean, "sd": sd, "p_out": L.p_out(mean, sd, tol[i]),
                     "best_k_alone": L.best_i_gain(p.ramp[i], p.bias_meas[i], p.sigma_place[i], p.sigma_meas[i], tol[i])})
    res["steady_at_best"] = axes
    res["settle_cells"] = {str(k): L.settle_cells(k) for k in cfg["sweep"]["show"] + [k_best]}

    # 5. terminal height: the decision, not the loop
    sh = cfg["terminal_height"]["sigma_process"]
    gb = [L.guard_band_rates(sh, S.sigma_height_meas, S.height_tol, n_ * S.sigma_height_meas)
          for n_ in cfg["terminal_height"]["guard_sigmas"]]
    sm_grid = [0.001, 0.005, 0.01, 0.02, 0.03]
    gb_worse = [L.guard_band_rates(sh, s, S.height_tol, 0.0) for s in sm_grid]
    res["terminal_height"] = {"sigma_process": sh, "sigma_meas": S.sigma_height_meas, "tol": S.height_tol,
                              "guard": gb, "no_guard_vs_sigma_meas": dict(zip(map(str, sm_grid), gb_worse))}
    res["decisions"] = {c: L.decide(c, "pre_busbar" if c in ("cell_placement", "terminal_height", "terminal_tilt",
                                                              "missing_cell", "reversed_polarity") else "pre_weld")
                        for c in L.DECISION} | {"(any, post weld)": L.AFTER_WELD}
    (out / "results.json").write_text(json.dumps(res, indent=1, default=float))

    # summary
    pc = lambda r: f"{r['rate']:.2%} ({r['lo']:.2%}–{r['hi']:.2%})"
    lines = [
        f"# 1.12 correction loop, {a.setup} gauge ({time.strftime('%d %b %Y')})", "",
        f"Measurement from `{S.grr_source}`: σ_v = {p.sigma_meas[0]*1e3:.1f} µm (x, y), "
        f"{p.sigma_meas[2]:.4f}° (yaw, derived), gauge bias {p.bias_meas[0]*1e3:+.1f} µm in x.",
        f"Plant (assumed, `correction/loop.yaml`): σ_w = {p.sigma_place.tolist()}, offset {p.offset.tolist()}, "
        f"ramp {p.ramp.tolist()}/cell, step {p.step.tolist()} at cell {p.step_at}. "
        f"Tolerance: {tol[0]} mm radial, {tol[2]}°. {cells} cells × {runs} runs.", "",
        "## Open vs closed loop (the headline)", "",
        "| Loop | K | cells out of tolerance (95 % CI) |", "|---|---|---|",
        f"| open | – | {pc(open_r)} |",
        f"| I, deadbeat | 1.00 | {pc(res['deadbeat_I'])} |",
        f"| I, best | {k_best:.2f} | {pc(res['best_I'])} |", "",
        "## Stability", "",
        f"- I form: stable for 0 < gK < 2 (pole 1 − gK). P form: only gK < 1, and it keeps d/(1 + gK).",
        f"- Plant gain off by ±20 %: the band moves to 0 < K < {2/g_hi:.2f} (g = {g_hi}) / {2/g_lo:.2f} (g = {g_lo}).",
        "- Cells to settle to 5 %: " + ", ".join(f"K = {k}: {v:.1f}" for k, v in res["settle_cells"].items()), "",
        "## Steady state at the best gain (closed form, per axis)", "",
        "| axis | mean | 1σ | P(out, this axis) | best K for this axis alone |", "|---|---|---|---|---|",
        *[f"| {x['axis']} | {x['mean']:+.4f} | {x['sd']:.4f} | {x['p_out']:.2e} | {x['best_k_alone']:.2f} |" for x in axes], "",
        "A gauge bias lands in the placement one for one: with integral action the loop zeroes the "
        "*measured* error.", "",
        "## Terminal height: rework before the weld, not correction", "",
        f"σ_process {sh} mm (A), σ_meas {S.sigma_height_meas*1e3:.2f} µm (1.9), tol ±{S.height_tol} mm.", "",
        "| guard band | rework | escapes to the weld | false rework |", "|---|---|---|---|",
        *[f"| {g['guard']*1e3:.2f} µm | {g['rework']:.3%} | {g['escape']:.1e} | {g['false_rework']:.1e} |" for g in gb], "",
        "Without a guard band, if the gauge were worse:", "",
        "| σ_meas | escapes | false rework |", "|---|---|---|",
        *[f"| {s*1e3:.0f} µm | {g['escape']:.1e} | {g['false_rework']:.1e} |" for s, g in zip(sm_grid, gb_worse)], "",
        "## Decisions", "", "| finding | action |", "|---|---|",
        *[f"| {c} | {d} |" for c, d in res["decisions"].items()], "",
        f"_{time.time() - t0:.0f} s. Figures: step.png, sweep.png, spread.png._",
    ]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()

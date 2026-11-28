#!/usr/bin/python3
"""Plot and tabulate S1 correction-loop runs (data/loop/<label>.json, written by correction_node.py).
(task 3.4)

    loop_report.py data/loop/open.json data/loop/closed_k05.json data/loop/closed_k16.json \
        --plot data/loop/loop.png

One row of panels per run: the TRUE placement error of each cell (x, y, yaw; simulation truth,
what the product gets) against the cell index k across modules, with the +-tolerance band (A),
and the offset u the loop applied. Table: mean, sd and cells out of tolerance, over all cells and
after the first 6 (the loop's settling, at K = 0.5 the error is (1 - K)^6 = 1.6 % of its start).
"""
import argparse
import json
from pathlib import Path

import numpy as np

TOL = (0.20, 0.20, 0.10)          # mm, mm, deg (A)
AXES = ('x mm', 'y mm', 'yaw deg')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='+', type=Path)
    ap.add_argument('--plot', type=Path)
    a = ap.parse_args()
    runs = [(p.stem, json.loads(p.read_text())) for p in a.runs]
    print('| run | gains | cells | mean x / y / yaw | sd x / y / yaw | out of tol | after 6: mean | after 6: sd | after 6: out |')
    print('|---|---|---|---|---|---|---|---|---|')
    for name, r in runs:
        s, s6 = r['summary_all'], r['summary_after_6']
        f = lambda v: ' / '.join(f'{x:+.3f}' for x in v)  # noqa: E731
        g = lambda v: ' / '.join(f'{x:.3f}' for x in v)  # noqa: E731
        print(f'| {name} | {r["gains"]} | {s["cells"]} | {f(s["mean"])} | {g(s["sd"])} | {s["out_of_tol"]} | '
              f'{f(s6["mean"])} | {g(s6["sd"])} | {s6["out_of_tol"]} |')
    if not a.plot:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(len(runs), 3, figsize=(14, 3.0 * len(runs)), squeeze=False, sharex='col')
    for i, (name, r) in enumerate(runs):
        t = np.array([row['truth'] for row in r['rows']])
        u = np.array([row['u'] for row in r['rows']])
        k = np.arange(1, len(t) + 1)
        for j in range(3):
            A = ax[i, j]
            A.axhspan(-TOL[j], TOL[j], color='tab:green', alpha=0.08)
            A.axhline(0, color='k', lw=0.6)
            A.plot(k, t[:, j], 'o-', ms=3, lw=1, label='placement error (truth)')
            A.plot(k, u[:, j], 's--', ms=2.5, lw=0.8, color='tab:orange', label='offset u applied')
            for m in range(12, len(k), 12):
                A.axvline(m + 0.5, color='0.7', lw=0.6, ls=':')
            A.set_ylim(-0.25, 0.25)
            if i == 0:
                A.set_title(AXES[j])
            if j == 0:
                A.set_ylabel(f'{name}\nK = {r["gains"][0]}')
            if i == len(runs) - 1:
                A.set_xlabel('cell k (dotted: new module)')
            A.grid(alpha=0.3)
    ax[0, 0].legend(fontsize=7, loc='upper right')
    fig.suptitle('S1 cell placement, correction loop on mock hardware (green: +-tolerance, A)')
    fig.tight_layout()
    fig.savefig(a.plot, dpi=100)
    print(f'plot: {a.plot}')


if __name__ == '__main__':
    main()

# Known cases

Dataset cases that once went wrong, and a sweep near every tolerance, with the outcome each is
accepted to have. A detector change has to do at least as well on every one before it reaches a
dataset run. `data/chain_v3.sh` and `cloud/run_remote.sh` refuse to start until it has.

Two tiers keep a fix from trading one failure for another:

| Tier | What | Time | When |
|---|---|---|---|
| Mechanism | One small synthetic pytest per failure (e.g. `test_a_region_one_ray_short_at_5_sigma_...`) | seconds | every edit |
| Known cases | `cases.yaml`, real modules regenerated from their seeds | fixed ~30 min, arm ~3 h | before any dataset run |

## Run

From the repo root, memory-capped like any heavy job:

```bash
systemd-run --user --scope -q -p MemoryMax=14G -p MemorySwapMax=0 python defectgen/known_cases/run.py
```

- `--setup fixed|arm`: one setup only.
- `--src F7 ...`: only cases from those fix-list items.
- `--root DIR`: results folder. Reuse it to resume, or point it at an earlier run's output to
  re-judge that output without recomputing.
- `--list`: print the selected cases and stop.
- `--check`: check whether a full pass is recorded for the code as it is now. Runs nothing.

Each case prints `PASS`, `BETTER` or `FAIL` with the reason. The exit status is 0 only if
nothing fails.

A full run with no `--setup` or `--src` that passes writes `data/known_cases_runs/LAST_PASS.json`,
which holds a fingerprint of every `.py` and `.yaml` in `src/defectgen/` plus `cases.yaml`. Any edit
changes the fingerprint, so the known cases must pass again.

## The rule

- **Outcome is ranked.**
  - must_find: hit > partial > miss = not_covered.
  - in_spec: pass > false_reject.
  - dont_care and clean cases: only false alarms are judged.
- **`fa` is the most false alarms allowed.** Any rise fails.
- **`meas` moving by more than `meas_tol` (0.01) warns.** It does not fail: a fix may legitimately
  move a reading.
- **`BETTER` raises nothing by itself.** Review the improvement, then raise the accepted value in
  `cases.yaml` by hand.

## Adding a failure

When a run turns up something wrong:

1. Add its reproducing cases to `cases.yaml`, tagged with its fix-list item (`src: [F9]`), with the
   outcome they *should* have. The file now fails.
2. Fix it, and add a mechanism test.
3. The fix is done when the whole file passes, the new cases and every old one.

Accepted values as of 13 Nov 2026 come from the v3 gates (detector with F7 and F8): 138 cases,
64 fixed and 74 arm.

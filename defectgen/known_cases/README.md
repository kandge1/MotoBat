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
- `--judge-only`: judge the results already in `--root`; never compute. A missing case prints MISSING.
- `--check`: check whether a full pass is recorded for the code as it is now. Runs nothing.

It refuses to compute a case outside a memory cap (one arm case can take 14 GB), unless given
`--uncapped`. A results folder is stamped with the code that computed it (`CODE`); reusing one from
other code is refused, and a full pass is recorded only for a folder stamped with the current code.

Each case prints `PASS`, `BETTER` or `FAIL` with the reason. The exit status is 0 only if
nothing fails.

A full run with no `--setup` or `--src` that passes writes `data/known_cases_runs/LAST_PASS.json`,
which holds a fingerprint of every `.py` and `.yaml` in `src/defectgen/` plus `cases.yaml`. Any edit
changes the fingerprint, so the known cases must pass again.

## Two kinds of case

- **Dataset cases** (`tier: pod | grr | clean`): one module from the 1.7 dataset, regenerated from its
  seed, judged on its outcome and false alarms.
- **Regress cases** (`tier: regress`): one case of a `python -m defectgen.regress` group (`single`,
  `fixture`, `seeds`, `subtol`, `arm`, `fixtures`) through one setup, judged on its TP / FN / false
  alarm counts. A sub-tolerance case carries `tp_max: 0`: finding it is a false reject.

## The rule

- **Outcome is ranked.**
  - must_find: hit > partial > miss = not_covered.
  - in_spec: pass > false_reject.
  - dont_care and clean cases: only false alarms are judged.
- **`fa` is the most false alarms allowed.** Any rise fails.
- **`other`, where a case sets it, is the most wrong-class findings allowed** (a finding of another
  class than the case's: not a false alarm, but it still rejects the module). Any rise fails.
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

Accepted values (13–14 Nov 2026) come from the v3 gates (detector F7), raised where a fix made a case
what it should be (F8 spatter, F9 cell wrap), and zero findings for the new `fixtures` stress group:
243 cases, 118 fixed and 125 arm.

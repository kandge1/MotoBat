"""Run the known cases (cases.yaml) through the current detector and say, per
case, whether it did at least as well as accepted. Run from the repo root,
memory-capped like any heavy job:

    systemd-run --user --scope -q -p MemoryMax=14G -p MemorySwapMax=0 \\
        python defectgen/known_cases/run.py [--setup fixed|arm] [--src F7 ...] [--root DIR]

Exit status 0 only if every selected case passes. Results go under
data/known_cases_runs/<date_time>/ (or --root); rerunning with the same --root
resumes, reusing finished cases. A full run: fixed ~30 min, arm ~3 h.
"""
import argparse
import gc
import hashlib
import json
import sys
import time
from pathlib import Path

import yaml

from defectgen.dataset import Runner, clean_cases, grr_cases, load_dataset_params, pod_cases
from defectgen.detect import load_detect_params

HERE = Path(__file__).resolve().parent
CODE = HERE.parent / "src" / "defectgen"
MARK = Path("data/known_cases_runs/LAST_PASS.json")


def code_hash() -> str:
    """Fingerprint of everything that decides a result: the package's code and
    configs, and this list. Any edit means the known cases must pass again."""
    h = hashlib.sha256()
    files = sorted(p for p in CODE.rglob("*") if p.suffix in (".py", ".yaml") and "__pycache__" not in p.parts)
    for f in files + [HERE / "cases.yaml"]:
        h.update(str(f.relative_to(HERE.parent)).encode())
        h.update(f.read_bytes())
    return h.hexdigest()[:16]
STAGES = ["pre_busbar", "pre_weld", "post_weld"]
RANK = {"must_find": {"hit": 3, "partial": 2, "miss": 1, "not_covered": 1},
        "in_spec": {"pass": 2, "false_reject": 1}}


def judge(want: dict, got: dict, meas_tol: float) -> tuple[str, list[str]]:
    """PASS / BETTER / FAIL, and the reasons. The scale comes from the accepted
    outcome itself: a recipe's truth is only filled in while it runs, so a
    reused result has none (a first version ranked by it and passed a miss
    against an accepted partial)."""
    why, better = [], False
    rank = next((t for t in RANK.values() if want["outcome"] in t), None)
    if rank is not None:
        if got["outcome"] not in rank:
            why.append(f"outcome {got['outcome']} is not on the scale of accepted {want['outcome']}")
        else:
            g, w = rank[got["outcome"]], rank[want["outcome"]]
            if g < w:
                why.append(f"outcome {got['outcome']} < accepted {want['outcome']}")
            better |= g > w
    fa = got["false_alarms"] or 0
    if fa > want["fa"]:
        why.append(f"false alarms {fa} > accepted {want['fa']}")
    better |= fa < want["fa"]
    warn = []
    m, wm = got.get("measured"), want.get("meas")
    if m is not None and wm is not None and abs(m - wm) > meas_tol:
        warn.append(f"meas {m:.4f} vs accepted {wm:.4f}")
    return ("FAIL" if why else "BETTER" if better else "PASS"), why + warn


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--setup", choices=["fixed", "arm"], default=None, help="only this setup")
    ap.add_argument("--src", nargs="+", default=None, help="only cases tagged with one of these (F7, targeted, ...)")
    ap.add_argument("--root", default=None, help="results folder (default data/known_cases_runs/<now>); reuse to resume")
    ap.add_argument("--list", action="store_true", help="print the selected cases and exit")
    ap.add_argument("--check", action="store_true",
                    help="exit 0 only if a full pass is recorded for the code as it is now (no run)")
    args = ap.parse_args()
    if args.check:
        mark = json.loads(MARK.read_text()) if MARK.is_file() else {}
        now = code_hash()
        if mark.get("code") == now:
            print(f"known cases passed for this code ({now}) at {mark['when']} in {mark['root']}")
            return 0
        print(f"no full known-cases pass for this code ({now}); last pass: {mark.get('code', 'none')}. "
              f"Run: python defectgen/known_cases/run.py")
        return 1

    spec = yaml.safe_load((HERE / "cases.yaml").read_text())
    want = [c for c in spec["cases"]
            if (args.setup is None or c["setup"] == args.setup)
            and (args.src is None or set(c["src"]) & set(args.src))]
    if args.list:
        for c in want:
            print(c["setup"], c["tier"], c["family"], c["name"], c["outcome"], "fa<=", c["fa"], c["src"])
        print(len(want), "cases")
        return 0

    code_at_start = code_hash()
    q, dp = load_dataset_params(), load_detect_params()
    recipes = {"pod": pod_cases(q, dp), "grr": grr_cases(q, dp), "clean": clean_cases(q)}
    index = {(t, r.family, r.name): r for t, rs in recipes.items() for r in rs}
    missing = [c for c in want if (c["tier"], c["family"], c["name"]) not in index]
    if missing:
        sys.exit(f"cases.yaml names cases the dataset no longer has: {missing}")

    root = Path(args.root or f"data/known_cases_runs/{time.strftime('%Y%m%d_%H%M')}")
    root.mkdir(parents=True, exist_ok=True)
    # Results reused from --root must come from this code. A folder that does not
    # say (e.g. data/probes/run_case.py output) is taken on trust and stamped now.
    stamp = root / "CODE"
    if stamp.is_file() and stamp.read_text().strip() != code_at_start:
        sys.exit(f"{root} holds results from other code ({stamp.read_text().strip()} vs {code_at_start}); "
                 f"use a new --root")
    stamp.write_text(code_at_start + "\n")
    print(f"{len(want)} known cases -> {root}", flush=True)
    tally = {"PASS": 0, "BETTER": 0, "FAIL": 0}
    fails = []
    with open(root / "results.jsonl", "a") as log:
        for setup in ("fixed", "arm"):
            mine = [c for c in want if c["setup"] == setup]
            if not mine:
                continue
            run = None                  # built on the first case that needs computing
            # One stage's reference held at a time: three at once OOM-killed a 7 GB cap (13 Nov 2026).
            mine.sort(key=lambda c: STAGES.index(index[(c["tier"], c["family"], c["name"])].stage))
            for c in mine:
                r = index[(c["tier"], c["family"], c["name"])]
                t = time.time()
                f = root / r.tier / r.family / r.name / f"{setup}.score.json"
                if f.is_file():
                    got = json.loads(f.read_text())
                else:
                    run = run or Runner(q, [setup], root)
                    if r.stage not in run._refs:
                        run._refs.clear()
                        gc.collect()
                    got = run.run(r)["setups"][setup]
                verdict, why = judge(c, got, spec["meas_tol"])
                tally[verdict] += 1
                line = (f"{verdict:6s} {setup:5s} {r.tier:5s} {r.family:16s} {r.name:24s} {got['outcome']:12s} "
                        f"fa {got['false_alarms']}  [{','.join(c['src'])}]  ({time.time() - t:.0f} s)")
                if why:
                    line += "  " + "; ".join(why)
                print(line, flush=True)
                if verdict == "FAIL":
                    fails.append(line)
                log.write(json.dumps({"setup": setup, "tier": r.tier, "family": r.family, "name": r.name,
                                      "verdict": verdict, "why": why, "outcome": got["outcome"],
                                      "false_alarms": got["false_alarms"], "measured": got.get("measured")}) + "\n")
                log.flush()
    print(f"\n{tally['PASS']} pass, {tally['BETTER']} better, {tally['FAIL']} FAIL of {len(want)}")
    for line in fails:
        print("  " + line)
    if not fails and args.setup is None and args.src is None:
        if code_hash() != code_at_start:
            print("the code changed during this run: no pass recorded; run again")
            return 1
        MARK.parent.mkdir(parents=True, exist_ok=True)
        MARK.write_text(json.dumps({"code": code_at_start, "when": time.strftime("%Y-%m-%d %H:%M"),
                                    "root": str(root), "cases": len(want)}, indent=1))
        print(f"full pass recorded: {MARK}")
    if tally["BETTER"]:
        print("BETTER: review, then raise the accepted value in defectgen/known_cases/cases.yaml by hand.")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

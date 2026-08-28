# MotoBat — Repository Structure

How this repository is organised, and the reasoning behind each decision.
Companion to `project_start.md`, which covers the technical plan.

---

## 1. The branch question

### Prior convention (swarm_project)

`swarm_project` uses one long-lived branch per robot — `mycobot_main`,
`myagv_main` — currently 19 branches across 226 commits.

**That was defensible there.** The myCobot 280 Pi and the myAGV are physically
separate machines, deployed independently, with different dependencies and
different target hardware. The repo's `pi_setup/` and `workstation_setup/`
directories confirm it: two deployment targets, two environments. You never
needed both codebases checked out simultaneously.

### Why it breaks for MotoBat

MotoBat has four arms, but they are **not four deployments**. They are four
stations in **one Isaac Sim scene**, on **one ROS 2 graph**, on **one machine**
(or one cloud instance, §4.0b of `project_start.md`). The entire contribution is
that they interact — the Inspector's findings feed the Stacker's next placement.

> **Git checks out one branch at a time.**
> Per-robot branches would make it structurally impossible to run the line.

There is also no `pi_setup`/`workstation_setup` split here. Everything runs in
one place. The condition that justified branches in `swarm_project` is absent.

### What branches are actually for

Branches model **divergence over time** — a feature being built, a fix being
tested, an experiment that may be discarded. They are merged and deleted.

Branches do **not** model **variation across configuration** — which robot,
which sensor, which station. That belongs in packages, parameters, and launch
arguments.

Long-lived per-configuration branches produce a predictable failure: a fix to
shared code must be cherry-picked into every branch, they drift, and eventually
nobody knows which branch is authoritative. Nineteen branches is that pattern
beginning to bite.

### Decision

**One long-lived branch: `main`.**

Separation happens by **ROS 2 package**. Station identity comes from **launch
arguments and config files**, not from source divergence.

The instinct in "one branch, and a robot knowing itself picks what it needs" is
correct — with the refinement that a *node* does not decide who it is at
runtime. A **launch file** instantiates it with a namespace, a prefix, and a
config file:

```bash
ros2 launch motobat_bringup station.launch.py station:=inspector arm:=hc10dt
ros2 launch motobat_bringup line.launch.py          # all four at once
```

Same code, different parameters. This is the pattern the URDF already uses —
`prefix="s1_"`, `prefix="s3_"` — extended to the rest of the stack.

---

## 2. Layout

```
motobat/
├── README.md                  portfolio front door — structured as a paper
├── repo_structure.md          this file
├── project_start.md           technical plan, maths, timeline
├── LICENSE                    Apache-2.0
├── .gitignore
│
├── docs/
│   ├── mathematics.md         equations per subsystem (project_start.md §9)
│   ├── results/               plots, tables, measured numbers
│   └── media/                 diagrams, demo stills
│
├── defectgen/                 ★ standalone Python — NO ROS DEPENDENCY
│   ├── pyproject.toml
│   ├── src/defectgen/
│   │   ├── stack.py           procedural electrode stack generator
│   │   ├── defects.py         parameterised defect injection
│   │   ├── scan.py            simulated profilometer → point cloud
│   │   ├── detect.py          Open3D registration + envelope deviation
│   │   └── metrics.py         precision / recall / minimum detectable defect
│   └── tests/
│
├── scripts/
│   ├── setup_motobat.sh       environment installer
│   ├── bootstrap_rviz.sh      workspace + vendor deps + build
│   └── fix_torch.sh           PyTorch CUDA repair
│
└── ros2_ws/src/
    ├── motobat_msgs/          DefectReport, ScanResult, CorrectionCmd
    ├── motobat_description/   URDF/xacro, flange, meshes, RViz configs
    ├── motobat_bringup/       launch composition, scene setup
    ├── motobat_inspector/     HC10DT + sensor suite + detection node
    └── _vendor/               cloned deps — GITIGNORED, never committed
```

Station packages (`motobat_stacker`, `motobat_welder`, `motobat_sealer`) are
added **when they are built**, not before. Empty placeholder packages are
clutter and break `colcon build`.

### Why `defectgen` sits outside `ros2_ws`

This is the most consequential structural decision here.

The defect generation and detection pipeline (§4B) is **pure numpy and Open3D**.
It has no reason to depend on ROS 2, and keeping it independent buys three
things:

1. **Reviewers can reproduce the core result without installing ROS 2.**
   `pip install -e defectgen` and run. For a research artifact that lowers the
   barrier enormously — most people evaluating this will not set up a ROS 2
   workspace to check a claim.
2. **It is testable with plain `pytest`**, no colcon, no sourcing, no DDS.
3. **It works today**, on the 8 GB laptop, before any simulator exists — which
   is exactly why §4B is week 1–2 of the timeline.

The ROS 2 node in `motobat_inspector` becomes a thin wrapper: subscribe to
`PointCloud2`, convert to numpy, call `defectgen.detect`, publish a
`DefectReport`. **The algorithm is not entangled with the middleware.**

### Datasets are generated, not committed

`.gitignore` excludes `data/`, `*.ply`, `*.npy`. Commit the **generator and the
seed**; the dataset is reproducible output.

A seeded generator is stronger evidence than a committed dataset — anyone can
regenerate it, vary it, and check that results hold across seeds. It also keeps
the repository small enough to clone quickly.

---

## 3. Branch workflow

```
main ──●──────●──────●──────●──────●───▶   always green, always runnable
        \    /        \    /
         ●──●          ●──●                short-lived feature branches
      feat/defect-gen  fix/tf-frames
```

**Rules:**

- `main` always builds and runs. If it is broken, that is the priority.
- Feature branches live **days, not weeks**. Long branches diverge and become
  merge archaeology.
- Naming: `feat/`, `fix/`, `docs/`, `exp/` prefixes.
  - `exp/` marks experiments that may be abandoned — useful, and honest.
- Merge via PR **even when working solo.** The PR description is a free
  engineering log, and a reader can follow the reasoning later. `swarm_project`
  already does this (PR #26) — worth keeping.

**Tags mark results.** When a number goes into the write-up, tag the commit that
produced it:

```bash
git tag -a v0.1-detection-baseline -m "Precision/recall baseline, seed 0"
```

This is what makes results reproducible rather than merely claimed, and it costs
nothing at the time.

---

## 4. Commit practice

The commit history is evidence of the engineering process (proposal §5.7) and
will be read by people evaluating this work.

Write **why**, not **what** — the diff already shows what changed:

```
poor:   "fixed bug"
poor:   "updated detection.py"

good:   "Detect misalignment via envelope deviation, not height map

         Height-based detection was blind to lateral shift: a 0.8 mm
         layer offset produced only a 2.3 um max-Z change and zero
         footprint change. Envelope deviation gives a monotonic signal
         (+200 um at 0.2 mm shift). See docs/mathematics.md §9.8."
```

The second commit message is a finding. It survives being read a year later, and
it is the kind of thing that makes a repository read as research.

---

## 5. This repository vs `swarm_project`

**MotoBat is a new repository, not a branch in `swarm_project`.**

Different project, different robots, different domain. More practically: for
portfolio purposes a focused repository with one clear entry point reads far
better than a branch inside a repository about a different robot. Someone
landing on 19 branches with no obvious starting point will leave.

**Do not refactor `swarm_project` now.** It works, it is not the deliverable, and
the six-week timeline has no room for it. The branch structure there is
defensible for its deployment model anyway.

Worth carrying forward from it — these are good habits already in place:

- `dev_logs/` — an engineering journal is genuinely rare and worth keeping
- `WORKFLOW.md` and `git_user_guide.md` — documenting process, not just code
- `.gitignore` discipline
- PR-based merges even when solo

---

## 6. Making the repository readable to an evaluator

The audience is someone deciding whether the author knows what they are doing,
who will spend **about ninety seconds** before deciding. Optimise for that.

- **README answers "what is this and does it work?" above the fold.** Problem,
  one architecture image, headline result. Not a wall of setup instructions.
- **One demo asset near the top** — a GIF or short video of the arm scanning a
  defective stack. This is the single highest-return artifact in the project.
- **Results as tables and plots**, with the numbers stated plainly.
- **Limitations section**, honestly written. Reviewers look for it; its absence
  reads as inexperience.
- **A reproduce command that actually works** from a clean clone. Test it in a
  fresh container before publishing — a broken quick-start undoes everything
  above it.

---

## 7. Setup

```bash
./scaffold_repo.sh ~/motobat      # idempotent; never overwrites

cd ~/motobat
cp /path/to/project_start.md .
git add -A
git commit -m "Initial repository structure"

gh repo create motobat --public --source=. --remote=origin --push
```

Keep it **public from the first commit**. The history showing a project built
deliberately over weeks is itself part of the evidence.

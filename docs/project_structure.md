# MotoBat — Repository Structure

How this repository is organised, and the reasoning behind each decision.
Companion to `project_plan.md`, which covers the technical plan and the maths.

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
(or one cloud instance, `project_plan.md` §4.0b). The entire contribution is
that they interact — the Inspector's findings feed the Stacker's next placement.

> **Git checks out one branch at a time.**
> Per-robot branches would make it structurally impossible to run the line.

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

**One long-lived branch: `main`.** Short-lived feature branches off it.

Separation happens by **ROS 2 package**. Station identity comes from **launch
arguments and config files**, not from source divergence:

```bash
ros2 launch motobat_bringup station.launch.py station:=inspector arm:=hc10dt
ros2 launch motobat_bringup line.launch.py          # all four at once
```

Same code, different parameters. This is the pattern the URDF already uses —
`prefix="s1_"`, `prefix="s3_"` — extended to the rest of the stack.

---

## 2. Layout

**The repository root is the colcon workspace.** There is no nested `ros2_ws/`.
`colcon build` is run from the repo root and writes `build/`, `install/`, and
`log/` there, all gitignored.

```
MotoBat/                        ← git repo AND colcon workspace
├── README.md                   portfolio front door — structured as a paper
├── WORKFLOW.md                 how to work in this repo, day to day
├── LICENSE                     Apache-2.0
├── .gitignore
├── motobat_env.sh              sourced from ~/.bashrc; self-locating
│
├── docs/
│   ├── project_plan.md         technical plan, maths (§9), timeline
│   ├── project_structure.md    this file
│   └── results/                plots, tables, measured numbers
│
├── dev_logs/                   engineering journal
│
├── workstation_setup/
│   ├── setup_workstation.sh    environment installer
│   ├── bootstrap_rviz.sh       vendor deps + build + verify
│   └── fix_torch.sh            PyTorch CUDA repair
│
├── defectgen/                  ★ standalone Python — NO ROS DEPENDENCY
│   ├── pyproject.toml
│   ├── src/defectgen/
│   └── tests/
│
├── src/                        ← ROS 2 packages (see §3)
│   ├── motobat_description/
│   └── _vendor/                cloned deps — GITIGNORED, never committed
│
├── logs/                       installer logs        (gitignored)
└── build/  install/  log/      colcon output         (gitignored)
```

> **`log/` vs `logs/`.** Singular `log/` is colcon's build log — it owns that
> name and we cannot change it. Plural `logs/` is the installer's. Both are
> gitignored. Mildly unfortunate, worth knowing before you `rm -rf` the wrong one.

### Where the Python venv lives, and why it is not here

The venv stays at **`~/motobat/venv`**, deliberately outside the repository.

An activated venv changes which interpreter `colcon` and `ament_python` resolve,
and the failures that causes are indirect and slow to diagnose. Keeping the venv
physically outside the workspace makes "am I in the venv or the workspace?" a
question with an obvious answer. It is also ~6 GB of PyTorch wheels, which is
build output by any reasonable definition.

`~/motobat/` now contains nothing but `venv/`. Everything else is in the repo.

### Why `defectgen` sits outside `src/`

This is the most consequential structural decision here.

The defect generation and detection pipeline (`project_plan.md` §4B) is **pure
numpy and Open3D**. It has no reason to depend on ROS 2, and keeping it
independent buys three things:

1. **Reviewers can reproduce the core result without installing ROS 2.**
   `pip install -e defectgen` and run. For a research artifact that lowers the
   barrier enormously — most people evaluating this will not set up a ROS 2
   workspace to check a claim.
2. **It is testable with plain `pytest`** — no colcon, no sourcing, no DDS.
3. **It works today**, before any simulator exists — which is exactly why §4B
   is week 1–2 of the timeline.

`motobat_perception` becomes a thin wrapper: subscribe to `PointCloud2`,
convert to numpy, call `defectgen.detect`, publish a `DefectReport`.
**The algorithm is not entangled with the middleware.**

### Datasets are generated, not committed

`.gitignore` excludes `data/`, `*.ply`, `*.npy`, `*.pcd`. Commit the
**generator and the seed**; the dataset is reproducible output.

A seeded generator is stronger evidence than a committed dataset — anyone can
regenerate it, vary it, and check that results hold across seeds. It also keeps
the repository small enough to clone quickly.

---

## 3. The `src/` package structure

This section exists because the obvious decomposition is the wrong one, and the
reason is worth writing down.

### 3.1 The proposal, and what is right about it

The starting proposal was:

- fleet-wide concerns → `motobat_<type>`, mirroring `swarm_network` and
  `swarm_interfaces`
- anything specific to a robot → its own package (`HC10DT`, `Perception`)
- all URDFs and robot geometry → one package, `descriptions`

Two of those three instincts are correct:

- **The prefix is right** — and should be applied *more* aggressively than
  proposed (§3.2).
- **One description package is right** for MotoBat specifically, for a reason
  worth knowing (§3.5).
- **A standalone interfaces package is right**, and is the one package whose
  boundary is non-negotiable (§3.4).

### 3.2 Problem 1 — two of those are not legal package names

ROS 2 package names are governed by REP-144: they must start with a lowercase
letter and contain only lowercase letters, digits, and underscores.

`HC10DT` and `Perception` will be **rejected by ament at build time**. They
become `motobat_hc10dt` and `motobat_perception`.

### 3.3 Problem 2 — package names are one flat global namespace

This is the part that surprises people. A package name is not scoped to your
repository. It is scoped to **every workspace overlaid on your environment**,
including `/opt/ros/jazzy` and every vendor package you clone.

A package literally named `descriptions` or `perception` will collide with any
other package of that name anywhere on the ROS index. `ros2 pkg prefix
perception` becomes ambiguous, and overlay resolution silently picks one.

This is why every vendor prefixes everything: `ur_description`,
`franka_description`, `moveit_ros_planning`, `motoman_gp4_support`. The prefix
is not decoration — **it is the namespace**.

> **Every package gets the `motobat_` prefix**, not just the fleet-wide ones.

### 3.4 Problem 3 — "one package per robot" is right in `swarm_project` and wrong here

This is the interesting one, because the rule is not wrong in general — it is
wrong for *this* project, and knowing the difference is the actual skill.

**In `swarm_project` it was correct.** `mycobot_hardware` wraps a serial
protocol that only the myCobot speaks. It has its own dependencies, it deploys
to the Pi and not the workstation, and it is independently testable. That is a
genuine per-robot package and it earns its boundary.

**In MotoBat the condition is absent.** Four arms, one scene, one ROS 2 graph,
one machine. Station 1 (GP4 stacker) and Station 2 (GP4 welder) differ by:

- a TF prefix (`s1_` vs `s2_`)
- a pose in the world
- the task they perform

All three are **configuration**. None is source. A `motobat_gp4` package would
be empty, or a copy of its neighbour that drifts.

There is also a subtler tell in the original proposal: **`HC10DT` and
`Perception` were listed as the same kind of thing.** One is a piece of
hardware; the other is a capability. When a decomposition mixes axes like that,
it usually means the axis has not been chosen yet.

### 3.5 The test to apply

> **Would you ever build, test, or ship this package without the others?**
> If no, it is a directory or a config file — not a package.

| Candidate | Passes? | Why |
|---|---|---|
| `motobat_interfaces` | **emphatically** | A reviewer may want only the message definitions. Message codegen must never sit in a dependency cycle. |
| `motobat_description` | yes | Loadable by RViz, Gazebo, Isaac Sim, and MoveIt independently. |
| `defectgen` | so emphatically it should not be a ROS package at all | See §2. |
| `motobat_hc10dt` | **no** | Nothing about the HC10DT is independently shippable here. |
| `motobat_stacker` | not yet | Would be empty. Create it when it has a node in it. |

Applied honestly, this test deletes most packages people are tempted to create.
That is the point. **Empty packages are a promise of structure that does not
exist**, and `colcon build` on seven empty directories is pure noise. The
installer used to create exactly those seven; it no longer does.

### 3.6 The axis that works: dependency direction and rate of change

Decompose by **what depends on what**, and by **what changes together**.

```
   motobat_bringup            ← nothing depends on it. Launch + YAML only.
          │                      Knows "station 3 is an HC10DT at pose X".
          ▼
   motobat_perception         ← capability packages. Thin ROS wrappers
   motobat_coordination          around defectgen and MoveIt.
          │
          ▼
   motobat_*_moveit_config    ← GENERATED by Setup Assistant. Isolated so
          │                      regenerating churns nothing hand-written.
          ▼
   motobat_description        ← URDF/xacro, the flange, the line layout.
          │
          ▼
   motobat_interfaces         ← depends on NOTHING. DefectReport, ScanResult,
                                 CorrectionCmd.
```

Four rules fall out of that picture:

1. **Interfaces at the bottom, with zero dependencies.** This is the one
   boundary you cannot get wrong later — a cycle involving message generation
   is not fixable by refactoring, only by splitting the package.
2. **Generated content is isolated.** MoveIt configs are Setup Assistant
   output. Regenerating one must not touch a file you wrote by hand.
3. **Exactly one leaf at the top.** `motobat_bringup` depends on everything and
   nothing depends on it. It contains launch files and YAML — **no source**.
4. **Robot identity lives in launch arguments and YAML**, never in package
   boundaries.

### 3.7 The resulting layout

Packages are created **when they are written**, not in advance.

| Package | Contains | Status |
|---|---|---|
| `motobat_description` | line xacro, 4-arm layout, RViz configs, sensor flange (§4A.4) | **exists** |
| `motobat_interfaces` | `DefectReport.msg`, `ScanResult.msg`, `CorrectionCmd.action` | §4B.4 |
| `motobat_gp4_moveit_config` | SRDF, OPW kinematics, joint limits | §4A.1 |
| `motobat_hc10dt_moveit_config` | SRDF, KDL kinematics, joint limits | §4A.2 |
| `motobat_perception` | ROS wrapper over `defectgen`; publishes `DefectReport` | §4B.4 |
| `motobat_bringup` | station/line launch composition, Isaac Sim scene setup | §4A.3 |
| `motobat_coordination` | the correction loop (§9.9) | week 6 |
| `_vendor/` | `gavanderhoorn/motoman`, MotoROS2 client interfaces | gitignored |

Two MoveIt configs rather than one, because the two arms genuinely differ:
the GP4 has a spherical wrist and uses closed-form OPW IK; the HC10DT's wrist
axes do not intersect and it must use numerical KDL (`project_plan.md` §4A.2).
That is a real difference in generated content, not a naming preference.

### 3.8 `_msgs` or `_interfaces`?

**`motobat_interfaces`.** The convention is `_msgs` when a package holds only
messages, `_interfaces` when it holds messages *and* services *and* actions.
MotoBat needs all three — `DefectReport` is a message, `CorrectionCmd` is an
action with feedback. It also matches `swarm_interfaces`.

`project_plan.md` still says `motobat_msgs` in places; that is the older name.

### 3.9 What about `motobat_network`?

`swarm_network` exists because that fleet genuinely spans machines. MotoBat
runs on one host, and §4A.5 **explicitly deferred** the simulated-switch/DDS
work — the reasoning is recorded there so it is not revisited by accident.

Do not create the package until there is something to put in it.

### 3.10 Interface discipline — read before writing a control node

This is a structural rule, not a style preference. It is what makes the
sim-to-hardware claim in §5.1 of the proposal true rather than aspirational.

- **Arms are commanded ONLY through `control_msgs/FollowJointTrajectory`.**
- **State is read ONLY from `/joint_states`.**
- **Never call Isaac Sim's Python articulation API from a control node.**

```
sim:   MoveIt2 → FollowJointTrajectory → Isaac Sim controller
real:  MoveIt2 → FollowJointTrajectory → micro-ROS Agent → MotoROS2 → arm
```

MotoROS2 on a real YRC1000 exposes the same action server, so migrating means
swapping the hardware interface, not rewriting the stack. Isaac-Sim-specific
code belongs in `motobat_bringup` — scene setup, sensor spawning, defect
injection — and nowhere else.

---

## 4. Branch workflow

```
main ──●──────●──────●──────●──────●───▶   always green, always runnable
        \    /        \    /
         ●──●          ●──●                short-lived feature branches
   feat/defect-gen   fix/tf-frames
```

**Rules:**

- `main` always builds and runs. If it is broken, that is the priority.
- Feature branches live **days, not weeks**. Long branches diverge and become
  merge archaeology.
- Naming: `feat/`, `fix/`, `docs/`, `exp/` prefixes.
  - `exp/` marks experiments that may be abandoned — useful, and honest.
- Merge via PR **even when working solo.** The PR description is a free
  engineering log, and a reader can follow the reasoning later.

**Tags mark results.** When a number goes into the write-up, tag the commit that
produced it:

```bash
git tag -a v0.1-detection-baseline -m "Precision/recall baseline, seed 0"
```

This is what makes results reproducible rather than merely claimed, and it costs
nothing at the time.

---

## 5. Commit practice

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
         (+200 um at 0.2 mm shift). See project_plan.md §9.8."
```

The second commit message is a finding. It survives being read a year later, and
it is the kind of thing that makes a repository read as research.

---

## 6. This repository vs `swarm_project`

**MotoBat is a new repository, not a branch in `swarm_project`.**

Different project, different robots, different domain. More practically: for
portfolio purposes a focused repository with one clear entry point reads far
better than a branch inside a repository about a different robot. Someone
landing on 19 branches with no obvious starting point will leave.

**Do not refactor `swarm_project` now.** It works, it is not the deliverable, and
the six-week timeline has no room for it.

Carried forward from it deliberately:

- `dev_logs/` — an engineering journal is genuinely rare and worth keeping
- `WORKFLOW.md` — documenting process, not just code
- `workstation_setup/` — setup scripts in one obvious place
- a root env file (`swarm_env.sh` → `motobat_env.sh`)
- `.gitignore` discipline, and PR-based merges even when solo

---

## 7. Making the repository readable to an evaluator

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

## 8. Setup from a clean clone

```bash
git clone <repo> ~/projects/MotoBat
cd ~/projects/MotoBat
./workstation_setup/setup_workstation.sh    # apt, ROS 2, venv, tooling
./workstation_setup/bootstrap_rviz.sh       # vendor deps, colcon build, verify
source ~/.bashrc

ros2 launch motobat_description view_line.launch.py
```

Both scripts resolve the repo root from their own location, so the clone path
does not matter and neither script needs editing.

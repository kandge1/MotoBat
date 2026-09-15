# MotoBat — Status and Task Order

**One question this file answers: what do I do next?**

`project_plan.md` is the reference — the reasoning, the maths, the decisions.
It is long on purpose. This file is the checklist that runs off it. When they
disagree, `project_plan.md` is right about *why* and this file is right about
*when*.

Section references like §4B.1 point into `project_plan.md`.

*Last updated: 15 Sep 2026*

---

## ▶ CURRENT TASK

> **§4B.1 — Procedural battery-module generator, mesh-native.** Pure numpy +
> trimesh, no ROS, no simulator. A VDA 355 module (355 × 151 × 108 mm):
> twelve PHEV2 cells (148 × 91 × 26.5 mm) in 3P4S, laser-weld terminals,
> busbars, weld seams, end and side plates, cooling plate, carrier, CMC and
> cover. **Geometry reference: [`module_spec.md`](module_spec.md).** In
> parallel: measure the terminal spacing, pad size and vent from photos in
> Fusion (module_spec §6). It emits three stages
> (pre-busbar, pre-weld, post-weld) as watertight solids with STL/PLY export.
> The generator returns `(mesh, labels)`, and the same mesh is both the
> reference and the scan target.
>
> Lives in `defectgen/`, outside `src/`. It must not import ROS (§2 of
> `project_structure.md`).

**Scope changed on 15 Sep: from inside a single cell to a battery module
assembly line.** Cell-internal defects are buried under anode and separator,
so no external optical sensor can see them. Even an LJ-X8020 has the
resolution but not the line of sight. Module-level defects are on the surface,
come with real tolerances (a > 0.1 mm height difference between adjacent
terminals can cause a flawed weld), and give all four robots a job. The
question MotoBat now answers is **whether a robot-mounted multi-view sensor
finds what a fixed line-scan misses, and what it costs in cycle time.** Full
reasoning, numbers and sources: **`project_plan.md` §3.7.**

---

## How work is sequenced

**There is no fixed deadline (15 Sep).** Work is ordered by dependency, and
each phase unblocks the next:

1. **Phase 1, numpy:** generator → defects → three sensor models → detection →
   coverage and view selection → dataset → metrics → correction-loop K-sweep
2. **Phase 2, HC10DT in Gazebo:** MoveIt, flange, reachable view plan, real
   motion time, fixed vs arm with cycle time
3. **Phase 3, the line:** GP7 places cells, GP4 places busbars, the loop is
   closed in simulation, microScan3 safety zones
4. **Phase 4:** optional Isaac Sim fidelity, then the write-up and demo

**Never trade away** the Phase 1 metrics or the fixed-vs-arm comparison.
Everything else is presentation around them.

---

## Phase 0 — Setup, repair, restructure ✅ COMPLETE

Branch `feature/restructure`. This phase exists so the rest of the project is not
spent fighting the environment.

| # | Task | Status |
|---|---|---|
| 0.1 | Ubuntu 24.04 + ROS 2 Jazzy, MoveIt2, Gazebo, RViz2 | ✅ verified |
| 0.2 | Diagnose 8 GB ceiling → dead DIMM slot A, not bad RAM (§1.0) | ✅ |
| 0.3 | Dell warranty transfer + onsite motherboard replacement | ✅ 31 Aug |
| 0.4 | Verify 16 GB dual-channel (`free -h` → 15 Gi) | ✅ |
| 0.5 | Repair PyTorch CUDA stack (§1.1) | ✅ torch 2.6.0+cu124, matmul verified |
| 0.6 | Fix `fix_torch.sh` name-guessing bug that hid the CUDA 13 strays | ✅ |
| 0.7 | Restore numpy 1.26.4 pin after torch pulled 2.x (§1.3) | ✅ |
| 0.8 | Pin SB3 2.5.0 / gymnasium <1.1 to match torch 2.6 | ✅ |
| 0.9 | Repo root becomes the colcon workspace; `src/` holds packages | ✅ |
| 0.10 | Install logs → `logs/`, gitignored | ✅ |
| 0.11 | `motobat_env.sh` version-controlled at repo root, self-locating | ✅ |
| 0.12 | Installer stops generating the env file / empty packages | ✅ |
| 0.13 | Package naming + layout rules written (`project_structure.md`) | ✅ |
| 0.14 | `bootstrap_rviz.sh` builds from repo root; xacro verified (§1.2) | ✅ 39 links / 38 joints |
| 0.15 | Fix `robot_description` YAML-parse crash in both launch files (§1.4) | ✅ |
| 0.16 | Headless verify: `/joint_states` 50 Hz, 24 dynamic + 14 static TF frames | ✅ |
| 0.17 | Four arms in RViz, sliders drive all 24 joints independently | ✅ 4 Sep |
| 0.18 | `dmidecode` confirms DIMM A + B, distinct serials — slot A alive | ✅ 4 Sep |

**Phase 0 is complete. Nothing is deferred and nothing is outstanding.**

Closed out: `feature/restructure` merged to `main` (PR #1). Phase 1 runs on
`feature/defect_generation_and_detection`.

---

## Phase 1 — §4B: module generation, detection, coverage, correction ◀ NOW

**Pure numpy / trimesh / Open3D. No ROS, no simulator.** This is the core of
the contribution.

| # | Task | § | Output |
|---|---|---|---|
| 1.0 | Resolution gate + scope decision | 3.7 | ✅ 15 Sep, in `project_plan.md` |
| 1.1 | Mesh-native module generator, three stages, `(mesh, labels)` seam | 4B.1 | `defectgen/module.py` |
| 1.2 | Defect injection: cell pose, terminal height/tilt, missing cell, reversed polarity, busbar pose/gap, weld seam, adhesive bead, FOD | 4B.2 | `defects.py` |
| 1.3 | Three ray-cast sensor models: fixed Ranger3 line-scan, wrist Zivid-class snapshot, wrist LJ-X8200-class profilometer. Both occlusions, dropout, noise; parameters **frozen and hashed before any results**. | 4B.3 | `scan.py` |
| 1.4 | Detection: align → ICP → per-feature fits within one capture → residual segmentation → classify | 4B.4 | `detect.py` |
| 1.5 | Coverage metric + greedy view selection (no kinematics yet) | 4B.5 | `coverage.py` |
| 1.6 | `motobat_interfaces`: `DefectReport`, `ScanResult`, `CorrectionCmd` | §7 | first ROS package |
| 1.7 | Labelled dataset, **several seeds per (class × severity) cell** | 4B.6 | seeded, not committed |
| 1.8 | **Precision/recall per class, with confidence intervals** | 4B.6 | |
| 1.9 | **%GRR** for terminal height, cell position, busbar gap | 4B.6 | automotive MSA form |
| 1.10 | **Fixed vs arm: coverage % and recall per class** | 4B.6, 3.7.6 | the central comparison |
| 1.11 | Minimum detectable defect per class; sensitivity to noise, point spacing, standoff | 4B.6 | → which sensor class the flange needs |
| 1.12 | **Correction loop in numpy: K-sweep, open- vs closed-loop placement defect rate, reject/rework logic** | 4B.7, 9.9 | the headline control result |

Tag the commit that produces 1.8–1.12: `git tag -a v0.1-detection-baseline`.

**Source before claiming results** (§3.7.9): terminal and busbar dimensions,
the cell-placement / busbar-gap / weld-seam tolerances, and a module-line
cycle time. Confirm the 0.1 mm terminal figure from the MDPI paper itself.

**Do not** let this phase acquire a ROS dependency. A reviewer must be able to
`pip install -e defectgen && pytest` without touching ROS 2 (§2 of
`project_structure.md`).

---

## Phase 2 — HC10DT inspection in Gazebo

| # | Task | § | Notes |
|---|---|---|---|
| 2.1 | HC10DT MoveIt config — **KDL, not OPW** | 4A.2 | `joint_5_b` offset 162 mm in Y; wrist axes do not intersect. No OPW file exists. Do not hunt for one. |
| 2.2 | `ros2_control` + `joint_trajectory_controller`, HC10DT in Gazebo | 4A.3 | |
| 2.3 | Flange: snapshot 3D + profilometer + 2D camera, poses as variables | 4A.4, 3.7.3 | check field-of-view occlusion, standoff compatibility, ~3 kg payload |
| 2.4 | `inspector_flange.xacro` — flange + sensor frames on `s3_tool0` | 4A.4 | derive origins from CAD **numerically**; a 2° eyeball error corrupts every registration downstream and will look like an algorithm bug |
| 2.5 | Reachable view plan: Phase 1 viewpoints filtered by IK and collision, ordered to minimise motion | 4B.5 | the arm earns its place here |
| 2.6 | Module meshes in Gazebo; depth camera + narrow-fan `gpu_lidar` as the sensors; fixed line-scan station alongside | 4B.8 | |
| 2.7 | **Fixed vs arm with real cycle time** (motion + capture per module) | 3.7.6 | completes the central comparison |
| 2.8 | Verify TF tree: `ros2 run tf2_tools view_frames` | 4A.4 | |

---

## Phase 3 — The line: GP7 cells, GP4 busbars, closed loop, safety

| # | Task | § | Notes |
|---|---|---|---|
| 3.1 | GP7 MoveIt + OPW. **Derive the OPW parameters from the URDF** (none ship, verified) and check them against forward kinematics. | 4A.1 | |
| 3.2 | GP7 cell pick-and-place, located by a fixed overhead 2D camera | 3.7.3 | same pattern as the myCobot vision pick-and-place |
| 3.3 | GP4 MoveIt + OPW (vendor file exists); busbar placement + simulated weld pass | 4A.1 | |
| 3.4 | Close the loop in simulation: `DefectReport` → `CorrectionCmd` → next cell's placement | 9.9 | |
| 3.5 | microScan3 safety zones → HC10DT speed and separation monitoring | 3.7.3 | ISO/TS 15066; this is the controls-and-automation showpiece |
| 3.6 | Latency instrumentation: sensor publish → registration → correction, from rosbag2 timestamps | 4A.5 | |

---

## Phase 4 — Fidelity, results, write-up

| # | Task | § |
|---|---|---|
| 4.1 | Optional: Isaac Sim, only if Gazebo's rendering limits a result | 4 |
| 4.2 | README rewritten for the module line, structured as a paper: Problem → Method → Results → **Limitations** → Future work (cell internals with the collaborator; carbon fibre) | 3.6, 3.7.7 |
| 4.3 | Demo video: cells placed → HC10DT multi-view inspection → defect caught → next placement corrected | 3.6 |
| 4.4 | Verify quick-start from a clean clone in a fresh container | §7 of `project_structure.md` |
| 4.5 | arXiv preprint (cs.RO) — needs one endorsement, not necessarily the advisor | 3.6 |

> **The write-up's motivation section comes from §3.7.** Three measured
> findings about sensing justify the scope and the sensors before a single
> result is shown.

---

## Standing rules

These have each already cost time once.

- **Never `set -u` in a script that sources a ROS setup file.** §1.2.
- **Wrap every `robot_description` in `ParameterValue(..., value_type=str)`.**
  Otherwise launch YAML-parses your URDF and dies. Applies to the MoveIt
  configs and Gazebo bringup still to come. §1.4.
- **A clean `colcon build` does not mean the launch files run.** Launch each
  one once, headlessly, before calling a milestone done. §1.4.
- **Command arms only via `FollowJointTrajectory`; read state only from
  `/joint_states`.** Never call Isaac Sim's articulation API from a control
  node — that boundary is the entire sim-to-hardware claim. §6.
- **numpy stays at 1.26.4.** Open3D links the 1.x ABI. A `--force-reinstall`
  of torch will silently pull 2.x. §1.3.
- **Any `pip install` that can pull torch transitively needs the same
  `--index-url`, or `--no-deps`.** §1.1.
- **`pip uninstall` on a name that is not installed *succeeds*.** An uninstall
  list of guessed names looks identical to one that worked. §1.1.
- **Use `mb-venv` for Open3D/PyTorch.** Never pip into system Python — PEP 668
  blocks it and overriding breaks apt.
- **Every ROS package gets the `motobat_` prefix.** Package names are one flat
  global namespace across every overlaid workspace. `project_structure.md` §3.3.
- **Create packages when you write them**, never in advance.
- **If point clouds drop or truncate under load, switch to FastDDS before
  debugging anything else.** §5. MotoROS2 requires FastDDS anyway.

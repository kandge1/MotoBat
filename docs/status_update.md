# MotoBat — Status and Task Order

**One question this file answers: what do I do next?**

`project_plan.md` is the reference — the reasoning, the maths, the decisions.
It is long on purpose. This file is the checklist that runs off it. When they
disagree, `project_plan.md` is right about *why* and this file is right about
*when*.

Section references like §4B.1 point into `project_plan.md`.

*Last updated: 4 Sep 2026*

---

## ▶ CURRENT TASK

> **§4B.1 — Procedural stack generator.** Pure numpy, no ROS, no simulator.
> Parametric electrode stack: 66.5 × 48.0 mm footprint, 24 layer pairs,
> 90 µm anode / 75 µm cathode / 20 µm separator, 1.5 mm cathode inset.
> Everything a variable. This same generator is also the reference-geometry
> source, so "known good" and "scanned part" come from one model.
>
> Lives in `defectgen/`, outside `src/` — it must not import ROS (§2 of
> `project_structure.md`).

**Phase 0 is complete and verified** — environment repaired, repo
restructured, four arms confirmed moving in RViz, slot A confirmed alive.
Nothing is blocking this.

---

## Schedule reality check — read this once

The §3.5 plan allocated **Aug 21 – Oct 1**, six weeks. Weeks 1 and 2 went to
hardware diagnosis, the Dell warranty path, the motherboard repair, and the
environment/repo repair — none of it optional, none of it planned.

**As of 4 Sep 2026: 27 days remain and §4B has not started.**

That is a real slip, and it is worth stating plainly rather than quietly
re-baselining. Three things make it recoverable:

1. **§4B is not starting from zero.** The §4B.0 prototype already produced the
   headline negative result with measured numbers (height maps miss lateral
   misalignment; envelope deviation gives a clean monotonic signal). The
   hardest question — *what to measure* — is answered.
2. **§4B is the contribution.** If nothing else lands, §4B.1–4B.5 alone is a
   defensible result (§3.6). Protect it first.
3. **The fallbacks are pre-decided.** Gazebo instead of Isaac Sim (§4A.3),
   cloud instead of local RAM (§4.0b). Neither needs a new decision under time
   pressure.

**If time gets short, cut in this order:** the four-station line first (already
cut), then Isaac Sim → Gazebo, then §4A.4's full FOV study → a documented
sketch. **Never cut §4B.5** — the precision/recall numbers are the deliverable.

---

## Phase 0 — Setup, repair, restructure ✅ COMPLETE

Branch `feature/restructure`. This phase exists so the next five weeks are not
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

To close it out: PR `feature/restructure` → `main`, then branch
`feat/defect-gen` and start §4B.1.

---

## Phase 1 — §4B: defect generation and detection ◀ NEXT

**Weeks 1–2 of the plan. Pure numpy/Open3D. No ROS, no simulator, no RAM
pressure.** This is the research contribution — everything else supports it.

| # | Task | § | Output |
|---|---|---|---|
| 1.1 | Procedural stack generator, fully parametric | 4B.1 | `defectgen/stack.py` |
| 1.2 | Defect injection: misalignment, overhang, layer count, wrinkle — each with a severity parameter and a ground-truth label | 4B.2 | `defects.py` |
| 1.3 | Simulated profilometer scan: 50–200 µm pitch, ~5 µm Gaussian noise, scan window = part + tolerance band | 4B.3 | `scan.py` |
| 1.4 | Detection pipeline: coarse align → ICP → envelope deviation → classify | 4B.4 | `detect.py` |
| 1.5 | `motobat_interfaces` package: `DefectReport`, `ScanResult`, `CorrectionCmd` | §7 | first ROS package |
| 1.6 | Labelled dataset, few hundred stacks across type × severity | 4B.5 | seeded, not committed |
| 1.7 | **Precision/recall per defect class, with error bars** | 4B.5 | vs 62.07 % Hi-Pot, 93.82 % YOLOv4 |
| 1.8 | **Minimum detectable defect per class** | 4B.5 | vs 0.5 mm production threshold |
| 1.9 | Detection latency | 4B.5 | vs 1.5–2.5 s/unit cycle time |
| 1.10 | Sensitivity to scan pitch and noise | 4B.5 | → tells §2.4 which sensors are needed |

**1.7–1.9 are the numbers the write-up cannot be written without.** Tag the
commit that produces them: `git tag -a v0.1-detection-baseline`.

**Do not** let this phase acquire a ROS dependency. A reviewer must be able to
`pip install -e defectgen && pytest` without touching ROS 2 (§2 of
`project_structure.md`).

---

## Phase 2 — §4A: arms, IK, and the sensor flange

**Weeks 3–4. RViz2 + Gazebo, 16 GB is plenty.** None of this is throwaway —
the MoveIt configs and controller wiring carry straight into Isaac Sim.

| # | Task | § | Notes |
|---|---|---|---|
| 2.1 | GP4 MoveIt config via Setup Assistant | 4A.1 | → `src/motobat_gp4_moveit_config/` |
| 2.2 | Swap KDL → `moveit_opw_kinematics_plugin` for the GP4 | 4A.1 | vendor ships `opw_parameters_gp4.yaml`; closed-form, all 8 branches |
| 2.3 | Drag marker → Plan & Execute → GP4 follows in RViz | 4A.1 | first real milestone |
| 2.4 | HC10DT MoveIt config — **KDL, not OPW** | 4A.2 | `joint_5_b` offset 162 mm in Y; wrist axes do not intersect. No OPW file exists. Do not hunt for one. |
| 2.5 | `ros2_control` + `joint_trajectory_controller`, one arm in Gazebo | 4A.3 | **the rehearsal for Isaac Sim** — only the hardware plugin changes later |
| 2.6 | Parametric sensor flange CAD (FreeCAD/Blender), sensor poses as variables | 4A.4 | STEP + source |
| 2.7 | Fit / payload / FOV-occlusion study: do 4 sensors fit on one HC10DT wrist within 10 kg? | 4A.4 | **any answer is a result**; "they don't fit" is publishable |
| 2.8 | `inspector_flange.xacro` — flange + 4 sensor frames on `s3_tool0` | 4A.4 | derive origins from CAD **numerically**; a 2° eyeball error corrupts every registration downstream and will look like an algorithm bug |
| 2.9 | Verify TF tree: `ros2 run tf2_tools view_frames` | 4A.4 | |

> **Ordering note:** §4B.0 already derived the hard constraint for 2.6–2.7 —
> the scan window must cover **part + tolerance band**, not just the part.
> Material that moved outside the nominal footprint is invisible to a scan
> sized to the nominal footprint. That turns the flange from a modelling
> exercise into a requirement with a number behind it.

---

## Phase 3 — §4: Isaac Sim

**Week 5. Decide *where* to run it before installing (§4.0).**

16 GB is below the documented 32 GB minimum. §4.0b recommends renting an L4
(~$0.33–0.98/hr, 40–80 hrs ≈ $20–60) and running the **entire stack** on the
instance — Isaac Sim, ROS 2, MoveIt2, RViz2 — streaming only the display back.
Do **not** split ROS 2 across the internet: DDS discovery needs multicast, the
container requires `--network=host`, and WAN latency lands inside the
`FollowJointTrajectory` loop.

| # | Task | § |
|---|---|---|
| 3.0 | Decide: local 16 GB vs cloud L4. Rent for an hour first and prove it streams. | 4.0b |
| 3.1 | Install Isaac Sim 6.0 (~30 GB; first launch compiles shaders 10–30 min — **it is not hung**) | 4.3 |
| 3.2 | Mount persistent cache volumes — otherwise you pay shader compilation every session | 4.0b |
| 3.3 | Import HC10DT URDF → USD; replace visual meshes with convex hulls for collision | 4.3 |
| 3.4 | Build the OmniGraph: Subscribe JointState → Articulation Controller → Publish JointState | 4.3 |
| 3.5 | `ros2_control` block using `JointStateTopicSystem` (ros-controls, **not** the UNMAINTAINED PickNik `topic_based_ros2_control`) | 4.2 |
| 3.6 | RViz marker → Plan & Execute → HC10DT moves in Isaac Sim | 4.3 |
| 3.7 | Import a generated defective stack as scene geometry; scan it with RTX Lidar | 4B.6 |

**Budget debugging time for §4.4's four known failure modes** — joint-name
mismatch, drive gains too soft to track, ActionGraph not ticking, RMW mismatch.
These are documented, not hypothetical.

**Fallback, already decided:** if Isaac Sim is not cooperating by **~24 Sep**,
ship the Gazebo version. Depth camera on the HC10DT gives the same
demonstration at lower sensor fidelity. Swap the hardware interface, keep
everything else.

---

## Phase 4 — Integration, results, write-up

**Week 6. The highest-leverage hours in the project are here, not in code.**

| # | Task | § |
|---|---|---|
| 4.1 | Correction loop with derived stability bound `0 < K < 2/g` | 9.9 |
| 4.2 | **K-sweep: convergence → critical damping → divergence** | 3.6 |
| 4.3 | **Open-loop vs closed-loop defect rate** — the headline claim | 3.6 |
| 4.4 | Latency instrumentation: sensor publish → registration → correction, from rosbag2 timestamps | 4A.5 |
| 4.5 | README restructured as a paper: Problem → Method → Results → **Limitations** → Future work | 3.6 |
| 4.6 | Demo video / GIF of the arm scanning a defective stack | 3.6 |
| 4.7 | Verify quick-start from a clean clone in a fresh container | §7 of `project_structure.md` |
| 4.8 | arXiv preprint (cs.RO) — needs one endorsement, not necessarily the advisor | 3.6 |

> Most simulation projects show one tuned gain that works. **Showing the
> boundary** — where it converges, where it diverges — is what makes 4.2 a
> controls result rather than a demo.

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

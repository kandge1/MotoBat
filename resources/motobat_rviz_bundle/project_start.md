# MotoBat — Project Start

**Battery Cell Assembly Line: Multi-Robot Inspection & Correction System**
A simulated closed-loop defect detection and correction system for lithium-ion
battery cell manufacturing, built in NVIDIA Isaac Sim.

Rhishikesh Kandge — M.S. Electrical and Computer Engineering, The Ohio State University

---

## What this document is

The handoff from environment setup to actual development. It records what is
built and verified, the decisions made and why, and the concrete next milestone:
**one HC10DT arm, planned in RViz via MoveIt2, moving in Isaac Sim.**

Read this first if you are picking the project up cold — including Claude Code.

---

## 0. Current status

| Area | State |
|---|---|
| Ubuntu 24.04 LTS + ROS 2 Jazzy | Installed, verified (talker/listener) |
| MoveIt2, ros2_control, rqt, RViz2 | Installed |
| Open3D, PCL, OpenCV, Python 3.11 venv | Installed |
| PyTorch CUDA | **Broken** — see §1.1 |
| Yaskawa GP4/GP7/HC10DT descriptions | Cloned, verified |
| `motobat_description` (4-arm line) | Written, xacro compile-tested |
| Four arms visualized in RViz | Blocked on §1.2 (one-line fix) |
| Isaac Sim | **Not installed** — next major step |
| MoveIt config for HC10DT | Not started — §4 |

---

## 1. Known issues to clear first

### 1.1 PyTorch is broken

The installer ran `pip install torch --index-url .../cu124` (got `2.6.0+cu124`),
then ran `pip install stable-baselines3[extra]` **without** the index URL. Pip
resolved torch from PyPI, found `2.13.0`, and upgraded over the CUDA build.

Result: `torchvision 0.21.0` and `torchaudio 2.6.0` both pin `torch==2.6.0` and
are broken, plus ~4 GB of redundant CUDA 13 wheels alongside the cu124 ones.

```bash
./fix_torch.sh
```

Reinstalls all four together in one resolve, removes the stray CUDA 13 runtime,
verifies `torch.cuda.is_available()`. Does not block RViz or Isaac Sim work —
only the Section 5.4 RL stretch goal.

**Lesson worth keeping:** any `pip install` that can pull `torch` as a transitive
dependency must either carry the same `--index-url` or use `--no-deps`.

### 1.2 `bootstrap_rviz.sh` fails with "AMENT_TRACE_SETUP_FILES: unbound variable"

`set -u` collides with `/opt/ros/jazzy/setup.bash`, which reads undefined
variables intentionally. Fixed in the current bundle — the script now uses
`set -Eeo pipefail` without `-u`.

**This applies to every shell script in the project that sources ROS.** Do not
use `set -u` in anything that sources a ROS setup file.

### 1.3 Minor, already handled

- `ros-jazzy-industrial-msgs` does not exist in apt; vcs pulls it from source.
- protobuf version mismatch with `grpcio-tools` — only matters when generating
  gRPC stubs. `fix_torch.sh` pins it.
- numpy correctly held at `1.26.4`. **Do not upgrade to 2.x** — Open3D wheels
  and several Jazzy Python packages still link the 1.x ABI.

---

## 2. Environment

**Ubuntu 24.04 LTS + ROS 2 Jazzy.** Not 26.04/Lyrical: Isaac Sim 6.0 supports
22.04/24.04 only, and its ROS 2 bridge names Jazzy on 24.04 as the recommended
configuration. Jazzy is supported to May 2029 — past both project phases.

### Layout

```
~/motobat/
├── venv/                     Python 3.11 (matches Isaac Sim's interpreter)
├── motobat_env.sh            sourced from ~/.bashrc
└── ros2_ws/
    └── src/
        ├── motobat_description/   4-arm line, launch files, RViz config
        ├── motobat_bringup/       (empty) launch composition, Isaac scene setup
        ├── motobat_msgs/          (empty) DefectReport, ScanResult, CorrectionCmd
        ├── motobat_stacker/       (empty) Station 1 — GP4
        ├── motobat_welder/        (empty) Station 2 — GP4
        ├── motobat_inspector/     (empty) Station 3 — HC10DT
        ├── motobat_sealer/        (empty) Station 4 — GP7
        └── _vendor/               cloned deps — gitignore this
```

Aliases: `mb-venv` `mb-ws` `mb-build`

### Python: two interpreters, on purpose

System Python is 3.12. The venv is **3.11** because Isaac Sim 5.x/6.x embed
CPython 3.11 — importing Open3D from 3.12 into Isaac Sim's interpreter produces
ABI errors that look like Open3D bugs but aren't.

Use `mb-venv` for Open3D/PyTorch work. Never `pip install` into system Python;
24.04 blocks it via PEP 668 and overriding will eventually break apt.

### Hardware

RTX 4060 Laptop (8 GB VRAM), driver 595.84, 854 GB free — all pass.

**RAM is 7 GB. Isaac Sim wants 32 GB.** This is the binding constraint on
everything downstream. Plan to bring up **one station at a time**; the full
four-arm line with four active sensors is not realistic on this machine. This
is the main reason §4's milestone is a *single* HC10DT.

---

## 3. What is built and verified

### `motobat_description`

Four arms in one scene, each instantiated from the Yaskawa xacro macros with a
distinct prefix so they share one TF tree without name collisions.

| Station | Role | Arm | Prefix | Macro |
|---|---|---|---|---|
| 1 | Stacker | GP4 | `s1_` | `motoman_gp4` |
| 2 | Tab Welder | GP4 | `s2_` | `motoman_gp4` |
| 3 | Inspector | HC10DT | `s3_` | `motoman_hc10dt` |
| 4 | Can Sealer | GP7 | `s4_` | `motoman_gp7` |

**Verified by compile test:** 39 links, 38 joints, 24 movable (4 × 6), single
`world` root, no missing joint limits.

```bash
tar xzf motobat_rviz_bundle.tar.gz
./bootstrap_rviz.sh

# sliders for all 24 joints
ros2 launch motobat_description view_line.launch.py

# or scripted motion (terminal 1, then terminal 2)
ros2 launch motobat_description view_line.launch.py gui:=false
ros2 run motobat_description demo_line_motion.py
```

> **`demo_line_motion.py` is a smoke test, not architecture.** It publishes
> `/joint_states` directly — no physics, no collision checking, no controllers.
> It proves the descriptions, TF tree, and meshes are correct. Delete it once
> real controllers exist so it does not quietly become load-bearing.

### Robot descriptions

`ros-industrial/motoman` is **ROS 1 only**. ROS 2 descriptions come from the
[`gavanderhoorn/motoman`](https://github.com/gavanderhoorn/motoman/tree/ros2)
fork, `ros2` branch — ament_cmake packages with meshes.

**On the HC10:** the fork ships `hc10`, `hc10dt`, and `hc10dt_b10` — there is no
`hc10dtp`. DTP is the IP67-rated variant of the DT with identical kinematics and
reach, so `hc10dt.xacro` is correct. Worth one sentence in Section 2 of the
proposal so it does not read as a spec mismatch.

---

## 4. Next milestone — HC10DT in Isaac Sim, planned from RViz

The target: open RViz, drag the MoveIt interactive marker to a pose, hit Plan &
Execute, and watch the HC10DT move in Isaac Sim. This is the first real
closed-loop between planner and simulator, and every later station reuses it.

### 4.1 Architecture

```
RViz (MoveIt Motion Planning plugin)
        │  interactive marker → plan request
        ▼
   move_group  ──────────────────────────────┐
        │  FollowJointTrajectory action      │  planning scene,
        ▼                                     │  collision checking
   joint_trajectory_controller               │
        │  (ros2_control)                    │
        ▼                                     │
   TopicBasedSystem hardware interface       │
        │                                     │
        │  publishes /isaac_joint_commands   │
        │  subscribes /isaac_joint_states    │
        ▼                                     │
   Isaac Sim OmniGraph ActionGraph           │
        │  ROS2 Subscribe JointState  ───────┘
        │  Articulation Controller
        │  ROS2 Publish JointState
        ▼
   HC10DT articulation in the USD stage
```

The seam is deliberate. `joint_trajectory_controller` exposes exactly the
`control_msgs/FollowJointTrajectory` action that **MotoROS2 exposes on real
Yaskawa hardware** (see §6). Swapping Isaac Sim for a YRC1000 later means
changing the hardware interface, not the planning stack.

### 4.2 The bridge package

Two options, both released for Jazzy:

| | Package | Plugin class |
|---|---|---|
| **Recommended** | `topic_based_hardware_interfaces` (ros-controls) | `joint_state_topic_hardware_interface/JointStateTopicSystem` |
| Legacy | `topic_based_ros2_control` (PickNik) | `topic_based_ros2_control/TopicBasedSystem` |

Prefer the **ros-controls** one. The PickNik package is marked UNMAINTAINED on
the ROS index; the functionality moved upstream into `ros2_control` under
Simulator Integrations. Most tutorials still show the old name — translate as
you read them.

### 4.3 Steps

**1 — Install Isaac Sim 6.0.** ~30 GB. Not scripted deliberately: the installer
is version-sensitive enough that a wrapper rots faster than it helps.

```bash
# https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html
cd ~/isaacsim && ./post_install.sh && ./isaac-sim.selector.sh
```

First launch compiles shaders for 10–30 minutes. **It is not hung.** Then
uncomment `ISAAC_SIM_PATH` in `~/motobat/motobat_env.sh`.

**2 — Import the HC10DT into Isaac Sim.** Use the URDF Importer extension on
`hc10dt.xacro` (expand to plain URDF with `xacro` first). Save as USD.

Expect to fix after import: joint drive stiffness/damping, joint limits, and
collision meshes. **The visual meshes are too detailed to use as collision
geometry** — replace with convex hulls or primitives, or physics will crawl.

**3 — Build the OmniGraph.** Three nodes, wired to the articulation:

- `ROS2 Subscribe JointState` → topic `/isaac_joint_commands`
- `Articulation Controller` → targeting the HC10DT prim
- `ROS2 Publish JointState` → topic `/isaac_joint_states`

Follow the [Joint Control tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_manipulation.html)
— it covers this graph directly.

**4 — Generate the MoveIt config.**

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Load `hc10dt.xacro`. Define one planning group (`inspector_arm`, all six joints),
generate the self-collision matrix, add a home pose. Output goes to
`src/motobat_inspector/config/`.

**5 — Add the `ros2_control` block** to the URDF used by MoveIt:

```xml
<ros2_control name="HC10DTIsaac" type="system">
  <hardware>
    <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
    <param name="joint_commands_topic">/isaac_joint_commands</param>
    <param name="joint_states_topic">/isaac_joint_states</param>
  </hardware>
  <!-- one <joint> block per joint_1_s … joint_6_t -->
</ros2_control>
```

**6 — Launch and test.** `move_group` + `joint_trajectory_controller` + RViz.
Drag the marker, Plan & Execute, watch Isaac Sim.

### 4.4 Where this will actually go wrong

Budget debugging time for these — they are the known failure modes, not
hypotheticals:

- **Joint name mismatch.** MoveIt's names must match Isaac Sim's articulation
  DOF names exactly. Check `ros2 topic echo /isaac_joint_states` against
  `ros2 param get /move_group robot_description`.
- **Trajectory following is inconsistent** even when connected — a documented
  Isaac Sim + `topic_based_ros2_control` issue. Usually joint drive gains being
  too soft to track the commanded trajectory. Tune stiffness/damping on the
  articulation first.
- **Nothing moves, no error.** Almost always the ActionGraph is not ticking, or
  the Articulation Controller points at the wrong prim.
- **RMW mismatch.** See §5 — this one is silent and wastes hours.

---

## 5. Open decision: Cyclone DDS vs FastDDS

`motobat_env.sh` currently sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` per
Section 5.3 of the proposal. **There are now two independent arguments against
that choice:**

1. **NVIDIA recommends FastDDS** for single-machine Isaac Sim — it uses shared
   memory for large `PointCloud2` traffic, which is essentially everything the
   Inspector does.
2. **MotoROS2 requires FastDDS** and does not support Cyclone. Keeping Cyclone
   means the stated hardware migration path needs an RMW change to work.

**Recommendation:** switch to FastDDS (comment the line out — it is the default)
and note the reversal in the proposal as an evidence-driven change. That reads
better than a spec nobody revisited.

If point clouds arrive truncated or drop under load, this is the first thing to
check — before debugging registration code.

---

## 6. MotoROS2: what it is and is not

**MotoROS2 cannot run on this machine.** It is a MotoPlus application that runs
*on* a Yaskawa controller (DX200 / YRC1000 / YRC1000micro). No PC build, no
simulation mode.

The MotoPlus SDK is Yaskawa's paid, license-gated C SDK, needed only to *build*
MotoROS2 from source. Yaskawa publishes prebuilt `.out` binaries. **The SDK is
not the barrier — the controller is.**

### Why this barely matters

MotoROS2 exposes itself to ROS 2 as a standard
`control_msgs/FollowJointTrajectory` action server plus `sensor_msgs/JointState`
publishing — the same interface MoveIt2 drives in Isaac Sim.

```
Simulation:  MoveIt2 → FollowJointTrajectory → Isaac Sim controller
Hardware:    MoveIt2 → FollowJointTrajectory → micro-ROS Agent → MotoROS2 → arm
```

Planning, inspection, and the correction loop are unchanged across that swap.

### The discipline that keeps this true

- Command arms **only** via `FollowJointTrajectory` actions
- Read state **only** from `/joint_states`
- **Never** call Isaac Sim's Python articulation API from a control node
- Isaac-Sim-specific code lives in `motobat_bringup` only

Breaking this welds the project to the simulator and invalidates the
sim-to-hardware claim.

### Proposal wording

Section 5.1 currently reads as though MotoROS2 is part of the running stack.
Replace with:

> The system targets the standard ROS 2 manipulation interfaces
> (`control_msgs/FollowJointTrajectory`, `sensor_msgs/JointState`) that
> MotoROS2 implements on real Yaskawa controllers. Arms are driven through
> ros2_control and MoveIt2 against Isaac Sim's articulation controller, with the
> command interface built against the genuine MotoROS2 contract — making
> sim-to-hardware migration a controller swap rather than a rewrite.

Section 2.2's Yaskawa-over-FANUC argument survives intact and is now the basis
of the migration path.

Add to Section 6 (future work): real controllers enforce motion limits, speed
overrides, and safety interlocks that Isaac Sim ignores. Trajectories valid in
simulation may be rejected outright by a YRC1000.

---

## 7. Roadmap after the HC10DT milestone

1. **Sensor array on the HC10DT** — SICK microScan3 / multiScan136 / Inspector83x,
   ZED 2i. RTX Lidar OmniGraph → `sensor_msgs/PointCloud2`. Verify in RViz2.
2. **TF2 tree for the sensors** — each sensor localized to the end effector.
   Point clouds from different passes cannot fuse without this.
3. **`motobat_msgs`** — `DefectReport`, `ScanResult`, `CorrectionCmd`. Define
   these before writing nodes that pass them around.
4. **Open3D registration pipeline** — ICP scan-vs-reference. Runs in the venv,
   not system Python.
5. **Cell and fixture geometry** — Blender/FreeCAD, procedurally generated so
   defect variants can be scripted rather than modeled by hand.
6. **Defect injection** — geometric, per Section 3.1.
7. **Remaining three stations** — reuse the HC10DT pattern. Watch RAM.
8. **Correction loop** — the actual contribution. Everything above is scaffolding.

---

## 8. Repo conventions

**`.gitignore`:**

```gitignore
build/
install/
log/
src/_vendor/
venv/
*.pyc
__pycache__/
```

`_vendor/` is cloned by the bootstrap script — never commit it.

**Commits:** the history is evidence of the development process (Section 5.7 of
the proposal). Write messages that explain *why*, not *what*.

**Before writing a control node,** re-read §6. That boundary is the whole
sim-to-hardware argument.

---

## References

- [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim ROS 2 installation](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_ros.html)
- [Isaac Sim joint control tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_manipulation.html)
- [MoveIt: command a simulated Isaac robot](https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html)
- [topic_based_hardware_interfaces](https://control.ros.org/master/doc/topic_based_hardware_interfaces/doc/index.html)
- [Yaskawa MotoROS2](https://github.com/Yaskawa-Global/motoros2)
- [gavanderhoorn/motoman (ros2)](https://github.com/gavanderhoorn/motoman/tree/ros2)

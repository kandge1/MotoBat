# MotoBat: A Battery Cell Assembly Line — Multi-Robot Inspection & Correction Project

## Project Summary

A simulated four-station robotic battery cell assembly line built in Isaac Sim, modeling the real
industrial process for prismatic/cylindrical lithium-ion cell manufacturing: **stack → weld → inspect
→ seal**. The project's core technical contribution is a **closed-loop defect detection and correction
system** — an inspection robot scans the cell stack after assembly and welding, detects geometric
misalignment against the real documented fault mode (electrode overhang crossing the insulating
member boundary), and feeds a correction signal back into the line *before* the irreversible can-sealing
step, rather than catching defects only after the fact via post-seal X-ray (the real industry's current
approach).

The project is explicitly designed to demonstrate a **generalizable architecture**: the same
scan-detect-correct pipeline built here is intended to be reapplied in a follow-up project to a
different material domain (carbon fiber composite layup inspection), to prove the approach isn't
overfit to one material or defect type.

### Why this problem matters
A single defective cell reaching a finished battery pack can cost well over $10,000 in warranty and
recall exposure. Real production lines run extremely fast (roughly 1.5–2.5 seconds per unit on a
prismatic cell winder), meaning a positional error of even half a millimeter at that speed becomes a
serious quality risk. Catching misalignment *before* the can is sealed — rather than relying on
downstream X-ray and helium leak tests — is a genuine, underexplored opportunity: it prevents defective
cells from ever being sealed shut around a scrap electrode stack.

### The real process this project is modeled on
Industrial cell assembly generally follows this sequence:
1. **Stacking** — electrodes and separators are alternately stacked (or Z-folded) into a "jelly stack"
2. **Tab welding** — a metal strip is ultrasonically or laser-welded to the stacked foil layers,
   forming the current-collector tab for each electrode
3. **Insertion & sealing** — the stack is inserted into the can/casing, and the can-lid joint is sealed
   via ultrasonic or laser welding to make it airtight
4. **Inspection** — traditionally performed *after* sealing, via X-ray (checking stack position within
   the can) and a helium leak test (checking weld integrity)

This project moves inspection **earlier**, inserting it between welding and sealing, so defects are
caught while the stack is still accessible and correctable.

---

## The Four Robots

| Station | Robot | Payload | Reach (Vert / Horiz) | Class |
|---|---|---|---|---|
| 1. Stacker | Yaskawa **GP4** | 4 kg | 1008 mm / 550 mm | Industrial (GP-series) |
| 2. Tab Welder | Yaskawa **GP4** | 4 kg | 1008 mm / 550 mm | Industrial (GP-series) |
| 3. Inspector / Scanner | Yaskawa **HC10DTP** | 10 kg | 2400 mm / 1200 mm | Collaborative (HC-series) |
| 4. Can-Sealer | Yaskawa **GP7** | 7 kg | 1693 mm / 927 mm | Industrial (GP-series) |

### Why these specific robots

**GP4 (Stacker + Tab Welder) — precision over payload.** Battery cell components at 21700
cylindrical-cell scale are small and require tight, repeatable tolerances, not brute strength. The GP4's
4 kg payload and compact ~550 mm horizontal reach are well matched to tabletop-scale, high-precision,
programmed-path work. Two identical GP4 units also keep the ROS2 control and motion-planning code
consistent across both stations rather than juggling two different control paradigms.

**GP7 (Can-Sealer) — a little more working envelope.** Sealing requires manipulating the can and lid
together, which benefits from slightly more reach and payload headroom than pure stacking work, without
jumping to an oversized industrial arm. Staying in the same GP-series family keeps the same
programming/control approach as the other two industrial stations.

**HC10DTP (Inspector/Scanner) — a deliberately different robot class.** This was the key design
decision in the project: rather than using a fourth GP-series arm for inspection, the project uses a
**collaborative** robot with a much larger reach envelope (2400 mm vertical vs. the GP4's 1008 mm).
Traditional industrial arms like the GP-series are built for repeatable, pre-programmed paths — not for
the kind of flexible, exploratory, multi-angle sweeping motion a scanning/inspection task benefits from.
The HC10's larger reach also means it can physically oversee or scan across multiple stations from a
central position, rather than being confined to one tight cell like the GP4/GP7. This mirrors a real
integration pattern: choosing hardware by *behavioral fit to task* (repeatable path-following vs.
flexible sensing) rather than defaulting to identical arms everywhere.

**Explicitly not chosen: a palletizer.** Yaskawa's PL-series palletizers (80–800 kg payload) are built
for high-speed, heavy case/box stacking — the wrong tool entirely for sub-millimeter electrode
alignment work. Precision, not payload, is what this project needs.

### Rejected/considered alternative: FANUC
Considered FANUC (ROBOGUIDE + KAREL) as an alternative platform. Chose Yaskawa/Motoman instead because:
- Yaskawa publishes **MotoROS2**, a real open-source ROS2 driver, allowing control of a simulated
  Yaskawa controller through genuine ROS2 messages rather than staying inside a proprietary,
  non-ROS ecosystem
- FANUC's ROBOGUIDE remains a useful *secondary* tool to learn separately (see Software Tools below)
  for comparison/breadth, but isn't the primary platform for this build

---

## Project Architecture: Scan → Detect → Correct

1. **Stacker (GP4)** places alternating electrode/separator layers on a build stage, following the
   real documented geometry (overhang regions, insulating member placement) from prismatic cell patents
2. **Tab Welder (GP4)** performs a simulated ultrasonic weld pass, bonding the stacked foil layers into
   a current-collector tab
3. **Inspector (HC10DTP)** sweeps a simulated depth/vision sensor over the completed stack, builds a
   point cloud, and registers it against a known-good reference geometry
4. **Defect detection** — flags the real documented fault mode: positive electrode overlapping the
   insulating member/overhang region, or stack thickness deviating from expected tolerance
5. **Correction loop** — rather than blind open-loop stacking, the next layer's placement offset is
   adjusted based on the Inspector's feedback, closing the loop before the **Can-Sealer (GP7)**
   performs the final, irreversible seal
6. **Logging** — every defect (type, location, severity, corrective action taken) is logged like a real
   QA system, along with before/after point clouds

### Defect simulation approach
Defects are injected **geometrically**, not through real material physics simulation — this keeps the
project tractable while still being realistic, since the actual fault mode (layer misalignment/overhang
violation) is fundamentally a geometric deviation problem, which is exactly what point-cloud-vs-
reference-mesh comparison is built to catch.

### Performance targets to measure and report
- Detection + correction latency, benchmarked against real industry cycle time (~1.5–2.5 sec/unit)
- Defect rate: simulated open-loop (blind stacking) vs. closed-loop (corrected) misalignment rate
- Throughput under the full 4-station pipeline

---

## Software Tools

### Simulation & Robot Control
- **Isaac Sim** — primary simulation environment for all four robots, sensors, and the physical cell
- **Isaac Lab** — for the stretch-goal RL correction policy (built on Isaac Sim, adds RL training
  infrastructure like PPO out of the box)
- **ROS2** — core middleware for all inter-node and inter-robot communication
- **MotoROS2** — Yaskawa's official open-source ROS2 driver; study its source/docs to understand how
  ROS2 control commands map to real Motoman controllers, even when only controlling simulated arms
- **MoveIt2** — motion planning for each arm (trajectory generation, collision checking); directly
  reuses prior MoveIt experience
- **RViz2** — visualization of robot state, planned trajectories, and point cloud data during
  development/debugging
- **Gazebo** — optional secondary simulator; useful if any component benefits from Gazebo's physics
  engine or if comparing sim fidelity against Isaac Sim

### Point Cloud & Computer Vision
- **Open3D** — point cloud registration (ICP), filtering, downsampling; core library for the
  scan-vs-reference comparison pipeline. Also directly relevant since it's named explicitly in real
  robotics-company perception job postings
- **PCL (Point Cloud Library)** — alternative/complementary to Open3D, particularly if working in C++
  rather than Python for performance-critical portions
- **OpenCV** — general image processing, camera calibration utilities, useful alongside point cloud
  work for any RGB-based inspection steps

### Communication & Data
- **Cyclone DDS** — set as the ROS2 RMW implementation (`RMW_IMPLEMENTATION`) instead of defaulting to
  FastRTPS, and tune its QoS settings; directly closes a named skill gap
- **Protobuf / gRPC** — worth introducing for the Inspector→Correction data pipeline as a structured,
  versioned schema, mirroring how real perception platforms wrap ML/sensor outputs into fleet-deployable
  services rather than passing loosely-typed ROS messages everywhere

### Machine Learning / RL
- **PyTorch** — primary ML framework for any learned components (defect classification, RL policy)
- **Isaac Lab (RL)** — PPO/SAC training for the stretch-goal correction policy
- **Stable-Baselines3** — a simpler, well-documented alternative/supplement to Isaac Lab's built-in RL
  if a lighter-weight RL prototyping loop is useful before committing to full Isaac Lab training
  infrastructure

### CAD / Geometry Pipeline
- **URDF / Xacro** — robot description format for all four Yaskawa arms in Isaac Sim/ROS2
- **Blender** or **FreeCAD** — for building the custom battery cell, stack, and mold-surface geometry
  (electrode layers, can, lid) that doesn't come as an off-the-shelf robot description; free and
  scriptable, useful for procedurally generating defect geometry variations
- **OnShape** (free tier) — alternative to Blender/FreeCAD if a more traditional parametric CAD
  workflow is preferred for the cell casing/fixture geometry

### Logging, Experiment Tracking & MLOps
- **Weights & Biases (W&B)** or **MLflow** — track training runs if the RL/ML stretch goals are
  pursued; also useful for logging defect-rate metrics across pipeline runs in a structured,
  comparable way rather than ad hoc print statements
- **rosbag2** — record and replay full ROS2 sessions (sensor data, robot state, detected defects) for
  debugging and for generating demo material later without needing to re-run the full sim live
- **Foxglove Studio** — modern alternative/complement to RViz2 for visualizing rosbag2 recordings,
  particularly nice for producing clean demo videos and shareable visualizations of point cloud data

### Version Control & Documentation
- **Git / GitHub** — standard version control; commit history itself becomes evidence of the
  development and debugging process for portfolio purposes
- **GitHub Actions** — CI to catch build breakage across ROS2 nodes as the project grows, especially
  useful once four robots' worth of packages exist in one repo

### Secondary / Comparison Tools (breadth, not core path)
- **FANUC ROBOGUIDE** — free (with registration) offline programming/simulation tool; worth exploring
  separately from this project purely to build comparative fluency between Yaskawa's and FANUC's
  programming paradigms (INFORM/MotoROS2 vs. KAREL/ROBOGUIDE)
- **TwinCAT (Beckhoff)** — free EtherCAT master; useful for conceptual, hands-on exposure to industrial
  fieldbus configuration, even though this project's simulation doesn't require real EtherCAT hardware

---

## Relationship to Future Work

This project is the first of two planned builds. The scan→detect→correct architecture, Isaac Sim
setup, ROS2/MotoROS2 integration, and Open3D-based point cloud pipeline developed here are intended to
be directly reused — not rebuilt — for a follow-up **carbon fiber composite layup inspection and
correction project**, applying the same core architecture to a structurally different defect domain
(wrinkles, gaps, overlaps, and bridging in automated fiber placement) as a deliberate test of whether
the approach generalizes across materials and industries, rather than being a one-off, overfit solution.

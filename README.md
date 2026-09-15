# MotoBat: A Battery Module Assembly Line — Robotic Multi-View Inspection & Correction

## Project Summary

A simulated robotic battery **module** assembly line built around four Yaskawa arms and ROS 2,
modeled on the real industrial process: **place cells → place busbars → inspect → laser weld**.
The core technical contribution is a **closed-loop, multi-view inspection system**. A
collaborative HC10DT carries a 3D sensor suite around the module, measures documented defect
classes (cell placement, terminal height, busbar fit, weld seams) against reference geometry,
and corrects the next placement *before* the irreversible laser weld.

The project asks one question that practitioners actually argue about:

> **Does a robot-mounted, multi-view sensor find defects that a fixed line-scan sensor misses —
> and what does it cost in cycle time?**

Both approaches are built, both are measured by coverage, recall per defect class and seconds per
module, and the answer is reported whichever way it falls.

### Why this problem matters
EV battery modules are joined by laser-welding busbars to cell terminals. That weld is
**irreversible**, and it is sensitive to geometry: a height difference of more than about
**0.1 mm between adjacent terminals** can produce a flawed joint. A cell placed off-position
shifts where the weld lands. A busbar that lifts off leaves an air gap under the laser. Catching
these *before* welding — and correcting the next placement rather than scrapping a module — is
the difference between a rework and a reject.

### The real process this project is modeled on
A prismatic module line runs roughly: cell loading and electrical test → cleaning → pad and adhesive
application → **cell stacking into the module** → strapping → **terminal cleaning and camera
positioning** → **busbar placement** → **laser welding** → end-of-line test. MotoBat models the
stations in bold, with inspection placed between busbar placement and welding (and again after
welding), where defects are still correctable.

### Why the scope is the module, not the inside of the cell
MotoBat began as electrode-stack inspection inside a single cell. A feasibility study, run before
any code, showed why that could not work with external optical sensing: the critical defect
(a cathode shifting inside the anode's outline) is buried under anode and separator layers.
Even a sub-micron laser profilometer has the resolution but not the line of sight, which is why
industry inspects each sheet during stacking and X-rays the result. That study, and the two
other sensing findings it produced, are recorded in
[`docs/project_plan.md` §3.7](docs/project_plan.md) and form the motivation for this scope.

---

## The Four Robots

| Station | Robot | Payload | Class | Job |
|---|---|---|---|---|
| 1. Cell Placer | Yaskawa **GP7** | 7 kg | Industrial (GP-series) | Picks prismatic cells (vision-located) and places them into the module |
| 2. Busbar Placer / Welder | Yaskawa **GP4** | 4 kg | Industrial (GP-series) | Places busbars on terminals; performs the simulated laser weld pass |
| 3. Inspector | Yaskawa **HC10DTP** | 10 kg | Collaborative (HC-series) | Multi-view 3D inspection before and after welding |
| — | Fixed line-scan station | — | Sensor, no robot | The baseline the Inspector has to beat |

### Why these specific robots

**GP7 (Cell Placer) — payload and reach for the heaviest part.** Prismatic cells are the heaviest
items handled on the line. The GP7 gives payload headroom for a cell plus gripper and enough reach
to span a 12-cell module, while staying in the same GP-series programming model as the GP4.

**GP4 (Busbar Placer / Welder) — precision over payload.** Busbars are light, and placing them
is a short, repeatable, programmed-path task. The GP4's compact envelope and closed-form (OPW)
inverse kinematics suit exactly that.

**HC10DTP (Inspector) — a deliberately different robot class.** Traditional industrial arms are
built for repeatable, pre-programmed paths. Inspection of a large part with occluded features
benefits from flexible, multi-angle viewpoints — the same reason car-body lines use robot-mounted
3D sensors from ZEISS, Hexagon and Perceptron. As a collaborative arm, the HC10 can also share
space with people under speed-and-separation monitoring. Choosing hardware by *behavioral fit to
task* (repeatable path-following vs flexible sensing) is itself part of the design argument —
and the fixed line-scan baseline keeps that argument honest.

**Explicitly not chosen: a palletizer.** Yaskawa's PL-series palletizers (80–800 kg payload) are
built for heavy, high-speed case stacking — the wrong tool for sub-millimeter placement work.

### Rejected/considered alternative: FANUC
Considered FANUC (ROBOGUIDE + KAREL). Chose Yaskawa/Motoman because Yaskawa publishes
**MotoROS2**, a real open-source ROS 2 driver, so the simulated arms are commanded through genuine
ROS 2 interfaces rather than a proprietary ecosystem. ROBOGUIDE remains a useful secondary tool to
learn separately.

---

## Sensors — each one where it belongs

A four-sensor wrist was the original plan. Checking datasheets against the defect sizes showed that
most of those sensors are the wrong class for measurement, so each sensor now has the role its
specifications support (full analysis: `docs/project_plan.md` §3.7.3).

| Where | Sensor (class) | Why |
|---|---|---|
| HC10DT wrist | Structured-light snapshot 3D camera (Zivid 2+ M60 class) | Coverage: stop-and-shoot views of the whole module |
| HC10DT wrist | Laser line profilometer (Keyence LJ-X8200 class) | Precision: two adjacent terminals in one profile, µm-level height |
| HC10DT wrist | 2D camera (SICK midiCam2) | Polarity, weld appearance |
| Over the conveyor | 3D laser-triangulation camera (SICK Ranger3) | The fixed-sensor baseline |
| Overhead, Station 1 | 2D camera (SICK midiCam2) | Locates cells for the GP7 pick |
| Cell perimeter | Safety laser scanner (SICK microScan3) | Speed-and-separation monitoring for the collaborative Inspector |

All sensors are simulated, with parameters pinned to their datasheets.

---

## Project Architecture: Place → Inspect → Correct → Weld

1. **Cell Placer (GP7)** picks each cell, located by the overhead camera, and places it into the
   module housing
2. **Inspector (HC10DTP)** captures the module from a planned set of viewpoints; each critical
   measurement (e.g. adjacent terminal heights) is taken *within a single capture*, so the arm's
   path accuracy never enters a 0.1 mm measurement — **the arm provides coverage, the sensor
   provides metrology**
3. **Defect detection** — registration against the reference module, then per-feature measurement:
   cell pose, terminal height and tilt, missing or reversed cells, busbar pose and gap, weld seam
   continuity and width
4. **Correction loop** — the measured placement error of cell *k* offsets the placement of cell
   *k+1*, with a derived stability bound (`0 < K < 2/g`); defects that placement cannot fix
   (terminal height) trigger reject/rework *before* welding
5. **Busbar Placer / Welder (GP4)** places busbars; the Inspector checks them; the weld runs only on
   a module that passed
6. **Logging** — every defect (type, location, severity, action taken) is logged like a real QA
   system, with before/after point clouds

### Defect simulation approach
Defects are injected **geometrically** into a procedurally generated, mesh-native module (twelve
VDA PHEV2-format prismatic cells, terminals, busbars, weld seams, housing). Every generated module
carries ground-truth labels, which is what makes precision and recall measurable. The generator
lives in `defectgen/`, a standalone Python package with no ROS dependency.

### What gets measured
- **Precision / recall per defect class**, with confidence intervals
- **Gage R&R (%GRR)** for measured characteristics — terminal height, cell position, busbar gap
- **Fixed vs robot-mounted inspection** — coverage %, recall per class, cycle time per module
- **Minimum detectable defect** per class, and sensitivity to sensor noise, point spacing and standoff
- **Correction-loop stability** — a gain sweep showing convergence, critical damping and divergence;
  open-loop vs closed-loop placement defect rate
- **Latency** — sensor publish → registration → correction command

---

## Software Tools

### Simulation & Robot Control
- **Gazebo (Harmonic)** — primary simulator for the line, with `ros2_control`
- **Isaac Sim** — optional higher-fidelity path, adopted only if Gazebo's rendering limits a result
- **ROS 2 Jazzy** — middleware for all nodes and robots
- **MoveIt 2** — motion planning; closed-form OPW IK for the GP-series arms, KDL for the HC10DT
  (its wrist axes do not intersect)
- **MotoROS2** — Yaskawa's open-source ROS 2 driver; the control boundary is kept identical to it
  so the simulated line maps onto real controllers
- **RViz2** — visualization of robot state, plans and point clouds

### Geometry, Point Clouds & Vision
- **trimesh / numpy** — procedural, mesh-native module generation and defect injection
- **Open3D** — ray-cast sensor models, ICP registration, per-feature measurement
- **OpenCV** — 2D camera processing (cell location, polarity)
- **PCL** — C++ alternative where performance demands it

### Communication & Data
- **DDS** — Cyclone DDS vs FastDDS is an open decision (MotoROS2 requires FastDDS)
- **rosbag2** — record and replay sessions; latency measured from timestamps
- **Foxglove Studio** — visualization of recordings for demos

### Machine Learning (stretch)
- **PyTorch**, **Stable-Baselines3**, **Isaac Lab** — only if a learned component outperforms the
  geometric pipeline on a measured result

### CAD & Robot Description
- **URDF / Xacro** — all arms and the inspector flange; sensor frames derived numerically from CAD
- **FreeCAD / Blender** — parametric sensor flange and fixtures

### Version Control & Documentation
- **Git / GitHub**, **GitHub Actions** — history is part of the development record
- **`docs/`** — the plan, the mathematics behind each subsystem, and the current task list

---

## Relationship to Future Work

- **Cell internals, with a battery-technology collaborator.** The feasibility study left a concrete
  hypothesis: during stacking, the anode's edge may leave a measurable step in the separator laid
  over it, which would let a profilometer measure cathode-to-anode overhang *through* the separator.
  Testing it needs real materials and domain expertise.
- **Carbon-fibre composite layup.** The same scan → detect → correct architecture applied to a
  structurally different domain (gaps, overlaps, wrinkles in automated fibre placement), as a test of
  whether the approach generalizes across materials and industries.

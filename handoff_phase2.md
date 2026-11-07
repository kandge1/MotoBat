# Handoff: Phase 2 lane (6 Nov 2026)

*Written for the Claude Code session in `~/projects/MotoBat-arm`. Temporary: delete once read,
and never commit it.*

## Where you are

This folder is a **git worktree** of MotoBat: the same repository as `~/projects/MotoBat`, but a
second folder on its own branch.

| | This lane (ROS) | The other lane (numpy) |
|---|---|---|
| Folder | `~/projects/MotoBat-arm` | `~/projects/MotoBat` |
| Branch | `feature/hc10dt_gazebo`, from `315bf90` ("Task 1.3 complete") | `feature/defect_generation_and_detection` |
| Work | Phase 2 tasks with no Phase 1 dependency | Phase 1, `defectgen/` |

**The docs in this folder date from `315bf90`.** The current versions are in the other folder and
not committed yet. Before you start, read **`../MotoBat/docs/status_update.md`**, the "Two lanes"
section, and **`../MotoBat/docs/project_plan.md` §8**, "Parallel worktrees". Treat the other folder
as read-only.

Since `315bf90`, the other lane has also finished a first version of task 1.4 (detection:
`defectgen/detect.py`, `fit.py`, `evaluate.py`). It is staged there, not committed.

## Your tasks

They are listed in dependency order. The specs are in `status_update.md`, Phase 2 table, and
`project_plan.md` §4A.

1. **2.1: HC10DT MoveIt config, using KDL, not OPW** (§4A.2, §9.3). `joint_5_b` is offset 162 mm in Y,
   so the wrist axes do not intersect. No OPW file exists, so don't look for one.
2. **2.2: `ros2_control` + `joint_trajectory_controller`, HC10DT in Gazebo** (§4A.3).
3. **2.4: `inspector_flange.xacro`**, the flange and sensor frames on `s3_tool0` (§4A.4).
   Derive the frame origins **numerically** from `cad/inspector/inspector_flange.step`. A 2° error
   from eyeballing it corrupts every later registration and will look like an algorithm bug.
   Task 2.3, the flange CAD, is the user's Fusion work. It carries a Zivid 2+ M60 and a Keyence
   LJ-X8200, and the sensor poses are Fusion User Parameters.
4. **2.8: TF tree check** with `ros2 run tf2_tools view_frames`.
5. **1.6: `motobat_interfaces`** (`DefectReport`, `ScanResult`, `CorrectionCmd`). It's a ROS-only
   package. Take the field contents from `../MotoBat/defectgen/src/defectgen/detect.py`, and only
   read that file.

**Not yet:** 2.5 needs Phase 1's viewpoints (task 1.5). 2.6 needs a decimated mesh export from
`defectgen`, which the other lane does. 2.7 needs both.

## Rules for this lane

- **Never commit.** Stage the work and let the user write the commit. The history is a deliberate
  artifact.
- **Docs:** in `status_update.md`, edit only the Phase 2 rows. In `project_plan.md`, edit only §4A.
  The other lane owns everything else, including `WORKFLOW.md`. Record anything else in a short
  note for the user to fold in at merge.
- **Don't touch `defectgen/`.** The shared venv (`~/motobat/venv`) has it installed in editable
  mode from the *other* folder, so `python -m defectgen` here runs the other folder's code.
- **One simulation at a time.** Only one lane runs RViz, Gazebo or a heavy job at once (16 GB of
  RAM, and both lanes use `ROS_DOMAIN_ID=42`).
- **No timelines or duration estimates**, even if asked. Order work by dependency.
- **CAD is Fusion 360 only.** Never suggest other CAD tools.
- Your memory folder is shared with the other lane's session.

## Environment gotchas

- `~/.bashrc` sources the **other** folder's `motobat_env.sh` in every terminal. In each terminal
  here, run `source motobat_env.sh` from this folder so its `install/` sits on top. Check with
  `ros2 pkg prefix motobat_description`, which should print a path under `MotoBat-arm`.
- `src/_vendor` is a symlink to the other folder's copy. It's gitignored there and listed in
  `.git/info/exclude`.
- `install/setup.bash` here chains only onto `/opt/ros/jazzy`, and was checked clean. If you ever
  delete `install/`, rebuild from a clean shell, or the other folder's install becomes a permanent
  underlay:
  `env -i HOME=$HOME PATH=/usr/bin:/bin bash -c 'source motobat_env.sh && colcon build --symlink-install'`.
  After that, `grep MotoBat/install install/setup.bash` should print nothing.
- rclpy scripts run with `/usr/bin/python3` (3.12), never the venv (3.11). With ROS sourced, run
  pytest with `-p no:launch_testing`.
- `data/` is gitignored and does not exist here. Copy in whatever a task needs.

## Read first

1. `../MotoBat/docs/status_update.md`: "Two lanes", then the Phase 2 table.
2. `project_plan.md` §4A.2, §4A.3, §4A.4, §6 (the MotoROS2 boundary), §9.3 (IK).
3. `docs/module_spec.md` §5, for the standoffs and the ≈ 39° wall-occlusion limit that 2.5 will need.

Start with **2.1**.

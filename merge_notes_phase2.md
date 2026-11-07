# Merge notes from the Phase 2 lane

*For the user to fold in when `feature/hc10dt_gazebo` merges. These touch docs
the Phase 1 lane owns, so the ROS lane did not edit them. Never commit this file.*

## From task 2.1 (6 Nov 2026)

- **`docs/project_structure.md`, package table (§3):** `motobat_hc10dt_moveit_config`
  now **exists**. Its row says "SRDF, KDL kinematics, joint limits". That is still right,
  and KDL is now a measured choice (plan §4A.2 "As built").
- **`docs/project_structure.md`, `motobat_description` layout:** new files
  `urdf/line_layout.xacro` (all station poses, and `station_mount`),
  `urdf/inspector_station.xacro` (the macro: HC10DT plus mount, and later the flange) and
  `urdf/inspector_station.urdf.xacro` (the station alone, which MoveIt loads).
  `motobat_line.urdf.xacro` now includes the first two. Its expanded URDF is unchanged.
- **`WORKFLOW.md`, a terminal recipe:**
  ```bash
  source motobat_env.sh
  ros2 launch motobat_hc10dt_moveit_config demo.launch.py          # RViz, drag the marker, Plan & Execute
  ros2 run motobat_hc10dt_moveit_config compare_ik.py             # IK solver benchmark, headless
  ```
- **Task 3.0 (GP25 swap in the line xacro):** station poses are now in `line_layout.xacro`.
  Add the GP25's pose there, not inline in the line file.
- **For the other MoveIt configs (3.1, 3.3, 3.3b):** copy this package's pattern: a
  `<station>.urdf.xacro` per station, `regen_collision_matrix.sh`, and `compare_ik.py`.
  On the GP arms the comparison would include OPW.

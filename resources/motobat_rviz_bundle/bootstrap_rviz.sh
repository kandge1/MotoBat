#!/usr/bin/env bash
# =============================================================================
#  MotoBat — get four arms moving in RViz
#
#  Run from wherever you unpacked motobat_description/:
#      ./bootstrap_rviz.sh
#
#  Installs the description package into your workspace, resolves the Yaskawa
#  vendor deps, builds, and tells you what to run. Idempotent.
# =============================================================================
# NOTE: deliberately NOT using -u here.
# /opt/ros/jazzy/setup.bash reads AMENT_TRACE_SETUP_FILES and other variables
# without defaults. Under `set -u` that is a fatal "unbound variable" error.
# Every ROS 2 script that sources setup.bash has to live without -u.
set -Eeo pipefail

WS="${HOME}/motobat/ros2_ws"
SRC="${WS}/src"
VENDOR="${SRC}/_vendor"
PKG_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/motobat_description"

info() { printf '\033[34m::\033[0m %s\n' "$*"; }
ok()   { printf '\033[32m ok\033[0m %s\n' "$*"; }
warn() { printf '\033[33m  ! \033[0m%s\n' "$*"; }
die()  { printf '\033[31m  x \033[0m%s\n' "$*"; exit 1; }

[[ -d "${WS}" ]] || die "No workspace at ${WS}. Run setup_motobat.sh first."
[[ -d "${PKG_SRC}" ]] || die "motobat_description/ not found next to this script."

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

# --- 1. Vendor descriptions -------------------------------------------------
if [[ ! -d "${VENDOR}/motoman" ]]; then
  info "cloning Yaskawa descriptions"
  mkdir -p "${VENDOR}"
  git clone -q -b ros2 https://github.com/gavanderhoorn/motoman.git "${VENDOR}/motoman"
else
  ok "Yaskawa descriptions present"
fi

# The fork carries ~30 robot support packages plus a ROS1 'attic'. Building
# all of them wastes several minutes; ignore everything except the three arms
# in the proposal and the shared resources package.
info "marking unused vendor packages COLCON_IGNORE"
KEEP="motoman_gp4_support motoman_gp7_support motoman_hc10_support motoman_resources"
for d in "${VENDOR}/motoman"/*/; do
  name="$(basename "$d")"
  if [[ " ${KEEP} " != *" ${name} "* ]]; then
    touch "${d}/COLCON_IGNORE" 2>/dev/null || true
  else
    rm -f "${d}/COLCON_IGNORE" 2>/dev/null || true
  fi
done
[[ -d "${VENDOR}/motoman/attic" ]] && touch "${VENDOR}/motoman/attic/COLCON_IGNORE"

# micro-ROS Agent needs hardware to be useful and pulls a long build.
# Skip it until there is a controller to talk to.
for p in micro-ROS-Agent micro_ros_msgs; do
  [[ -d "${VENDOR}/${p}" ]] && touch "${VENDOR}/${p}/COLCON_IGNORE"
done
ok "vendor tree trimmed to the three arms"

# --- 2. Install our package -------------------------------------------------
info "installing motobat_description"
rm -rf "${SRC}/motobat_description"
cp -r "${PKG_SRC}" "${SRC}/motobat_description"
ok "package staged"

# --- 3. Deps and build ------------------------------------------------------
info "resolving dependencies (rosdep)"
rosdep install --from-paths "${SRC}" --ignore-src -r -y >/dev/null 2>&1 \
  || warn "rosdep reported issues — continuing; check output if the build fails"

info "building (first build takes a couple of minutes)"
cd "${WS}"
if colcon build --symlink-install --packages-up-to motobat_description; then
  ok "build succeeded"
else
  die "build failed — see the log above"
fi

# --- 4. Verify --------------------------------------------------------------
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

info "verifying the xacro expands"
if xacro "$(ros2 pkg prefix motobat_description)/share/motobat_description/urdf/motobat_line.urdf.xacro" \
     > /tmp/motobat_line.urdf 2>/tmp/motobat_xacro.err; then
  LINKS=$(grep -c '<link' /tmp/motobat_line.urdf)
  JOINTS=$(grep -c '<joint' /tmp/motobat_line.urdf)
  ok "URDF generated: ${LINKS} links, ${JOINTS} joints"
  [[ "${LINKS}" -ge 39 ]] || warn "expected >=39 links; descriptions may have changed"
else
  warn "xacro failed:"; sed 's/^/    /' /tmp/motobat_xacro.err
  die "fix the xacro error above before launching"
fi

cat <<'NEXT'

=============================================================
 Ready
=============================================================

Sliders (manual control of all 24 joints):

    ros2 launch motobat_description view_line.launch.py

Scripted motion (the line running on its own):

    # terminal 1
    ros2 launch motobat_description view_line.launch.py gui:=false
    # terminal 2
    ros2 run motobat_description demo_line_motion.py

One arm on its own:

    ros2 launch motobat_description view_single.launch.py arm:=hc10dt

If RViz shows "No transform from [...] to [world]", the robot_state_publisher
is not running or /joint_states is silent. Check with:

    ros2 topic hz /joint_states
    ros2 run tf2_tools view_frames

NEXT

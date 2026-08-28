#!/usr/bin/env bash
# =============================================================================
#  MotoBat — Development Environment Installer
#  Battery Cell Assembly Line: Multi-Robot Inspection & Correction System
#
#  Target:   Ubuntu 24.04 LTS (Noble Numbat) — x86_64
#  Stack:    ROS 2 Jazzy + Isaac Sim 6.0 + Open3D/PCL + PyTorch (CUDA)
#  Author:   Rhishikesh Kandge
#
#  Usage:
#      chmod +x setup_motobat.sh
#      ./setup_motobat.sh                 # full install
#      ./setup_motobat.sh --dry-run       # print actions, change nothing
#      ./setup_motobat.sh --skip-torch    # skip the large PyTorch download
#      ./setup_motobat.sh --only ros      # run a single stage (see --help)
#
#  Do NOT run this with sudo. It calls sudo itself where required, so that
#  files created in $HOME stay owned by you rather than by root.
# =============================================================================

set -Eeuo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
readonly ROS_DISTRO_NAME="jazzy"
readonly PROJECT_ROOT="${HOME}/motobat"
readonly WS_DIR="${PROJECT_ROOT}/ros2_ws"
readonly VENV_DIR="${PROJECT_ROOT}/venv"
readonly PY_VERSION="3.11"          # Isaac Sim 5.x/6.x embed CPython 3.11
readonly CUDA_WHEEL_INDEX="https://download.pytorch.org/whl/cu124"
LOG_FILE="${PROJECT_ROOT}/install_$(date +%Y%m%d_%H%M%S).log"
readonly LOG_FILE

DRY_RUN=0
SKIP_TORCH=0
ONLY_STAGE=""

# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------
if [[ -t 1 ]]; then
  C_RESET=$'\033[0m'; C_BOLD=$'\033[1m'; C_DIM=$'\033[2m'
  C_RED=$'\033[31m'; C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'; C_BLUE=$'\033[34m'
else
  C_RESET=""; C_BOLD=""; C_DIM=""; C_RED=""; C_GREEN=""; C_YELLOW=""; C_BLUE=""
fi

log()   { printf '%s\n' "$*" | tee -a "${LOG_FILE}" >&2; }
info()  { log "${C_BLUE}::${C_RESET} $*"; }
ok()    { log "${C_GREEN} ok${C_RESET} $*"; }
warn()  { log "${C_YELLOW}  ! ${C_RESET}$*"; }
err()   { log "${C_RED}  x ${C_RESET}$*"; }
skip()  { log "${C_DIM}  - $* (already present)${C_RESET}"; }

stage() {
  log ""
  log "${C_BOLD}=============================================================${C_RESET}"
  log "${C_BOLD} $* ${C_RESET}"
  log "${C_BOLD}=============================================================${C_RESET}"
}

run() {
  if (( DRY_RUN )); then
    log "${C_DIM}  would run: $*${C_RESET}"
    return 0
  fi
  "$@" >>"${LOG_FILE}" 2>&1
}

# Fail loudly with the line number rather than dying silently mid-apt.
trap 'err "Failed at line ${LINENO}: ${BASH_COMMAND}"; err "See log: ${LOG_FILE}"' ERR

have()      { command -v "$1" >/dev/null 2>&1; }
apt_has()   { dpkg -s "$1" >/dev/null 2>&1; }

# Install apt packages, but only the ones actually missing. Keeps re-runs fast
# and makes the output readable instead of 400 lines of "already newest version".
apt_ensure() {
  local -a missing=()
  local p
  for p in "$@"; do
    apt_has "$p" || missing+=("$p")
  done
  if (( ${#missing[@]} == 0 )); then
    skip "${#@} packages"
    return 0
  fi
  info "installing ${#missing[@]} package(s): ${missing[*]}"
  run sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${missing[@]}"
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
usage() {
  cat <<'USAGE'
MotoBat environment installer for Ubuntu 24.04 LTS.

  --dry-run          Show what would happen without changing the system
  --skip-torch       Skip the PyTorch CUDA download (~2.5 GB)
  --only STAGE       Run one stage only. Valid stages:
                       base    system packages, build tools, git
                       ros     ROS 2 Jazzy, MoveIt2, Cyclone DDS
                       moto    MotoROS2 client interfaces, Yaskawa URDFs
                       python  venv, Open3D, OpenCV, PyTorch, W&B
                       tools   VS Code, Gazebo, Foxglove
                       ws      colcon workspace scaffold
                       check   Isaac Sim GPU/driver preflight only
  -h, --help         This message
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)    DRY_RUN=1; shift ;;
    --skip-torch) SKIP_TORCH=1; shift ;;
    --only)       ONLY_STAGE="${2:-}"; shift 2 ;;
    -h|--help)    usage; exit 0 ;;
    *)            err "Unknown option: $1"; usage; exit 2 ;;
  esac
done

should_run() { [[ -z "${ONLY_STAGE}" || "${ONLY_STAGE}" == "$1" ]]; }

# ---------------------------------------------------------------------------
# Preflight
# ---------------------------------------------------------------------------
mkdir -p "${PROJECT_ROOT}"
: >"${LOG_FILE}"

stage "MotoBat Environment Setup"
log "Log file: ${LOG_FILE}"
(( DRY_RUN )) && warn "DRY RUN — no changes will be made"

if [[ ${EUID} -eq 0 ]]; then
  err "Do not run this script as root or with sudo."
  err "It escalates on its own where needed; running as root would leave"
  err "root-owned files in your home directory."
  exit 1
fi

# Verify we are actually on 24.04. Refusing early beats failing halfway through
# an apt transaction on the wrong release.
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  UBUNTU_CODE="${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}"
  if [[ "${UBUNTU_CODE}" != "noble" ]]; then
    err "This script targets Ubuntu 24.04 LTS (noble). Detected: ${PRETTY_NAME:-unknown}"
    err "ROS 2 Jazzy binaries and the Isaac Sim support matrix both assume noble."
    exit 1
  fi
  ok "Ubuntu 24.04 LTS (noble) confirmed"
else
  err "Cannot read /etc/os-release — unsupported system."
  exit 1
fi

if [[ "$(uname -m)" != "x86_64" ]]; then
  err "Isaac Sim requires x86_64. Detected: $(uname -m)"
  exit 1
fi

# Keep sudo alive for the whole run so the user is prompted once, not eight
# times across a 40-minute install.
if ! (( DRY_RUN )); then
  info "Requesting sudo (needed for apt). You will be prompted once."
  sudo -v
  ( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) &
  readonly SUDO_KEEPALIVE_PID=$!
  trap 'kill "${SUDO_KEEPALIVE_PID}" 2>/dev/null || true' EXIT
fi

# ---------------------------------------------------------------------------
# Stage: base
# ---------------------------------------------------------------------------
if should_run base; then
  stage "1/7  Base system, build tools, Python"

  info "refreshing apt index"
  run sudo apt-get update

  apt_ensure \
    build-essential cmake git curl wget gnupg lsb-release ca-certificates \
    software-properties-common apt-transport-https pkg-config \
    python3 python3-pip python3-venv python3-dev python3-argcomplete \
    htop tree unzip zip jq ripgrep tmux net-tools usbutils

  # Deadsnakes gives us 3.11 to match the interpreter Isaac Sim ships with.
  # Ubuntu 24.04's system Python is 3.12, which Isaac Sim's Python API will
  # not import against — this is the single most common setup failure.
  if ! have "python${PY_VERSION}"; then
    info "adding deadsnakes PPA for Python ${PY_VERSION} (Isaac Sim compatibility)"
    run sudo add-apt-repository -y ppa:deadsnakes/ppa
    run sudo apt-get update
    apt_ensure "python${PY_VERSION}" "python${PY_VERSION}-venv" "python${PY_VERSION}-dev"
  else
    skip "python${PY_VERSION}"
  fi

  ok "base system ready"
fi

# ---------------------------------------------------------------------------
# Stage: ros
# ---------------------------------------------------------------------------
if should_run ros; then
  stage "2/7  ROS 2 ${ROS_DISTRO_NAME^} + MoveIt2 + Cyclone DDS"

  # ROS 2 moved to a signed ros2-apt-source .deb in 2025. The old
  # raw.githubusercontent.com key + manual sources.list method is deprecated
  # and its key has been rotated — do not resurrect it from older guides.
  if ! apt_has ros2-apt-source; then
    info "enabling universe repository"
    run sudo add-apt-repository -y universe

    info "installing ros2-apt-source (official repository package)"
    if (( DRY_RUN )); then
      log "${C_DIM}  would fetch and install ros2-apt-source .deb${C_RESET}"
    else
      ROS_APT_VER="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
                     | jq -r '.tag_name')"
      if [[ -z "${ROS_APT_VER}" || "${ROS_APT_VER}" == "null" ]]; then
        err "Could not determine ros-apt-source release (GitHub API rate limit?)."
        err "Retry later, or install manually from:"
        err "  https://github.com/ros-infrastructure/ros-apt-source/releases"
        exit 1
      fi
      info "ros-apt-source version ${ROS_APT_VER}"
      curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VER}/ros2-apt-source_${ROS_APT_VER}.${UBUNTU_CODE}_all.deb"
      run sudo dpkg -i /tmp/ros2-apt-source.deb
      rm -f /tmp/ros2-apt-source.deb
    fi
    run sudo apt-get update
  else
    skip "ros2-apt-source"
  fi

  # Desktop metapackage pulls in RViz2, demo nodes, and the full message set.
  apt_ensure "ros-${ROS_DISTRO_NAME}-desktop" ros-dev-tools

  info "motion planning and control stack"
  apt_ensure \
    "ros-${ROS_DISTRO_NAME}-moveit" \
    "ros-${ROS_DISTRO_NAME}-moveit-servo" \
    "ros-${ROS_DISTRO_NAME}-moveit-visual-tools" \
    "ros-${ROS_DISTRO_NAME}-ros2-control" \
    "ros-${ROS_DISTRO_NAME}-ros2-controllers" \
    "ros-${ROS_DISTRO_NAME}-controller-manager" \
    "ros-${ROS_DISTRO_NAME}-joint-state-publisher-gui" \
    "ros-${ROS_DISTRO_NAME}-xacro" \
    "ros-${ROS_DISTRO_NAME}-urdf-tutorial" \
    "ros-${ROS_DISTRO_NAME}-joint-trajectory-controller" \
    "ros-${ROS_DISTRO_NAME}-joint-state-broadcaster" \
    "ros-${ROS_DISTRO_NAME}-force-torque-sensor-broadcaster" \
    "ros-${ROS_DISTRO_NAME}-hardware-interface" \
    "ros-${ROS_DISTRO_NAME}-controller-interface" \
    "ros-${ROS_DISTRO_NAME}-moveit-planners-chomp" \
    "ros-${ROS_DISTRO_NAME}-moveit-ros-perception" \
    "ros-${ROS_DISTRO_NAME}-moveit-setup-assistant" \
    "ros-${ROS_DISTRO_NAME}-ur-description"

  # rqt: the introspection toolset. ros-jazzy-desktop pulls a subset, but the
  # plugins that actually matter for debugging a four-robot line -- the graph
  # view, plot, TF tree, and reconfigure -- are separate packages.
  info "rqt introspection and debugging tools"
  apt_ensure \
    "ros-${ROS_DISTRO_NAME}-rqt" \
    "ros-${ROS_DISTRO_NAME}-rqt-common-plugins" \
    "ros-${ROS_DISTRO_NAME}-rqt-graph" \
    "ros-${ROS_DISTRO_NAME}-rqt-plot" \
    "ros-${ROS_DISTRO_NAME}-rqt-console" \
    "ros-${ROS_DISTRO_NAME}-rqt-tf-tree" \
    "ros-${ROS_DISTRO_NAME}-rqt-reconfigure" \
    "ros-${ROS_DISTRO_NAME}-rqt-topic" \
    "ros-${ROS_DISTRO_NAME}-rqt-service-caller" \
    "ros-${ROS_DISTRO_NAME}-rqt-publisher" \
    "ros-${ROS_DISTRO_NAME}-rqt-image-view" \
    "ros-${ROS_DISTRO_NAME}-rqt-robot-monitor" \
    "ros-${ROS_DISTRO_NAME}-rqt-controller-manager" \
    "ros-${ROS_DISTRO_NAME}-rqt-joint-trajectory-controller"

  info "diagnostics and RViz plugins"
  apt_ensure \
    "ros-${ROS_DISTRO_NAME}-diagnostic-updater" \
    "ros-${ROS_DISTRO_NAME}-diagnostic-aggregator" \
    "ros-${ROS_DISTRO_NAME}-rviz-visual-tools" \
    "ros-${ROS_DISTRO_NAME}-rviz-2d-overlay-plugins" || true

  info "perception, TF2, and point cloud interop"
  # perception-pcl bridges sensor_msgs/PointCloud2 <-> pcl::PointCloud, which is
  # the seam between the RTX Lidar output and the Open3D/PCL comparison pipeline.
  apt_ensure \
    "ros-${ROS_DISTRO_NAME}-perception-pcl" \
    "ros-${ROS_DISTRO_NAME}-pcl-conversions" \
    "ros-${ROS_DISTRO_NAME}-pcl-ros" \
    "ros-${ROS_DISTRO_NAME}-tf2-tools" \
    "ros-${ROS_DISTRO_NAME}-tf2-sensor-msgs" \
    "ros-${ROS_DISTRO_NAME}-tf2-geometry-msgs" \
    "ros-${ROS_DISTRO_NAME}-vision-msgs" \
    "ros-${ROS_DISTRO_NAME}-ackermann-msgs" \
    "ros-${ROS_DISTRO_NAME}-image-transport" \
    "ros-${ROS_DISTRO_NAME}-image-pipeline" \
    "ros-${ROS_DISTRO_NAME}-camera-info-manager" \
    "ros-${ROS_DISTRO_NAME}-cv-bridge"

  info "rosbag2 recording and storage backends"
  apt_ensure \
    "ros-${ROS_DISTRO_NAME}-rosbag2" \
    "ros-${ROS_DISTRO_NAME}-rosbag2-storage-mcap" \
    "ros-${ROS_DISTRO_NAME}-rosbag2-transport"

  info "Cyclone DDS RMW"
  apt_ensure "ros-${ROS_DISTRO_NAME}-rmw-cyclonedds-cpp"

  # Foxglove bridge lets Foxglove Studio attach over websocket rather than
  # requiring you to shuttle rosbag files around.
  apt_ensure "ros-${ROS_DISTRO_NAME}-foxglove-bridge" || \
    warn "foxglove-bridge unavailable in apt; install from source if needed"

  info "initialising rosdep"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    run sudo rosdep init || warn "rosdep already initialised"
  else
    skip "rosdep sources"
  fi
  run rosdep update || warn "rosdep update failed (network?) — re-run later"

  ok "ROS 2 ${ROS_DISTRO_NAME} installed"
fi

# ---------------------------------------------------------------------------
# Stage: moto — MotoROS2 client interfaces + Yaskawa descriptions
# ---------------------------------------------------------------------------
# IMPORTANT ARCHITECTURAL NOTE
#
# MotoROS2 itself does NOT run here. It is a MotoPlus application that runs ON
# a physical Yaskawa controller (DX200 / YRC1000 / YRC1000micro). There is no
# PC build of it and no pure-simulation mode. Without real controller hardware
# you cannot run MotoROS2 at all.
#
# This is NOT a blocker for the project. MotoROS2 speaks standard
# control_msgs/FollowJointTrajectory -- the same interface MoveIt2 uses against
# Isaac Sim. Build against that interface and the hardware migration is a
# controller swap, not a rewrite. See src/_vendor/README.md for the full
# reasoning and the one discipline that keeps it true.
#
# What a PC-side developer CAN have, and what this stage installs:
#   1. motoros2_client_interface_dependencies — the msg/srv/action definitions
#      a client uses to talk to a MotoROS2 server. Building against these means
#      your nodes speak the real MotoROS2 interface, so they would drop onto
#      actual hardware unchanged.
#   2. micro-ROS Agent — the UDP bridge that would sit between your PC and the
#      controller. Useless without hardware, but building it now documents the
#      integration path.
#   3. Yaskawa URDF descriptions from the community ROS 2 fork.
#
# For your Isaac Sim line, the arms are driven through ros2_control +
# MoveIt2 against Isaac Sim's articulation controller. MotoROS2 is the
# *interface contract* you design to, not the runtime.
# ---------------------------------------------------------------------------
if should_run moto; then
  stage "4/7  MotoROS2 client interfaces + Yaskawa descriptions"

  MOTO_WS="${WS_DIR}/src/_vendor"
  run mkdir -p "${MOTO_WS}"

  apt_ensure "ros-${ROS_DISTRO_NAME}-control-msgs" \
             "ros-${ROS_DISTRO_NAME}-industrial-msgs" || \
    warn "industrial-msgs not in apt for ${ROS_DISTRO_NAME} (expected; vcs pulls it)"

  # --- MotoROS2 client interface definitions --------------------------------
  if [[ ! -d "${MOTO_WS}/motoros2_client_interface_dependencies" ]]; then
    info "cloning motoros2_client_interface_dependencies"
    run git clone -q \
      https://github.com/Yaskawa-Global/motoros2_client_interface_dependencies.git \
      "${MOTO_WS}/motoros2_client_interface_dependencies" \
      || warn "clone failed — check network"

    # This repo is an empty shell; the real interface packages come in via vcs.
    if [[ -f "${MOTO_WS}/motoros2_client_interface_dependencies/source_deps.repos" ]] \
       && ! (( DRY_RUN )); then
      info "importing MotoROS2 interface dependencies via vcs"
      ( cd "${MOTO_WS}" && \
        vcs import < motoros2_client_interface_dependencies/source_deps.repos ) \
        >>"${LOG_FILE}" 2>&1 || warn "vcs import failed — run manually later"
    fi
  else
    skip "motoros2 client interfaces"
  fi

  # --- Yaskawa robot descriptions -------------------------------------------
  # ros-industrial/motoman is ROS 1 only. The ROS 2 work lives in
  # gavanderhoorn's fork, which upstream maintainers point to as the best
  # current source.
  #
  # VERIFIED (Aug 2026): all three of your arms are present on the ros2
  # branch, as ament_cmake packages with meshes:
  #     motoman_gp4_support   urdf/gp4.xacro     14 meshes
  #     motoman_gp7_support   urdf/gp7.xacro     14 meshes
  #     motoman_hc10_support  urdf/hc10dt.xacro  30 meshes
  #
  # Note on the HC10: the fork ships hc10.xacro, hc10dt.xacro, and
  # hc10dt_b10.xacro. There is no hc10dtp variant. DTP is the IP67
  # dust/liquid-protected version of the DT; kinematics and reach are
  # identical, so hc10dt.xacro is the correct model to use. Say so
  # explicitly in the proposal rather than letting a reader think the
  # spec drifted.
  if [[ ! -d "${MOTO_WS}/motoman" ]]; then
    info "cloning Yaskawa motoman descriptions (ros2 branch)"
    run git clone -q -b ros2 \
      https://github.com/gavanderhoorn/motoman.git "${MOTO_WS}/motoman" \
      || warn "clone failed — ros2 branch may have moved; check the repo"
  else
    skip "motoman descriptions"
  fi

  # --- micro-ROS Agent ------------------------------------------------------
  if [[ ! -d "${MOTO_WS}/micro-ROS-Agent" ]]; then
    info "cloning micro-ROS Agent (PC-side bridge to a real controller)"
    run git clone -q -b "${ROS_DISTRO_NAME}" \
      https://github.com/micro-ROS/micro-ROS-Agent.git \
      "${MOTO_WS}/micro-ROS-Agent" \
      || warn "no ${ROS_DISTRO_NAME} branch; try the humble branch"
    run git clone -q -b "${ROS_DISTRO_NAME}" \
      https://github.com/micro-ROS/micro_ros_msgs.git \
      "${MOTO_WS}/micro_ros_msgs" || true
  else
    skip "micro-ROS Agent"
  fi

  if ! (( DRY_RUN )); then
    cat > "${MOTO_WS}/README.md" <<'VENDOR_EOF'
# Vendored third-party sources

Cloned by the installer. Not your code — do not commit these into the project
repo; add `src/_vendor/` to .gitignore and let the script re-fetch them.

## motoros2_client_interface_dependencies
Message/service/action definitions for talking to a MotoROS2 server.

### Why MotoROS2 is not running on this machine

MotoROS2 is a MotoPlus application that runs ON a Yaskawa controller
(DX200 / YRC1000 / YRC1000micro). No PC build, no simulation mode.

The MotoPlus SDK is Yaskawa's paid, license-gated C SDK, needed only to BUILD
MotoROS2 from source. You do not need it: Yaskawa publishes prebuilt .out
binaries on the releases page, which a controller owner loads directly. The
SDK is not the barrier. The controller is.

### Why this barely matters for how you write code

MotoROS2 exposes itself to ROS 2 as a standard control_msgs/FollowJointTrajectory
action server plus sensor_msgs/JointState publishing. That is the same
interface MoveIt2 already speaks, and the same one you drive in Isaac Sim.

    Simulation:  MoveIt2 -> FollowJointTrajectory -> Isaac Sim controller
    Hardware:    MoveIt2 -> FollowJointTrajectory -> micro-ROS Agent
                                                  -> MotoROS2 -> arm

Planning, inspection, and the correction loop are unchanged across that swap.
Only the far end of the action interface differs.

### The discipline that keeps this true

Command arms ONLY through FollowJointTrajectory actions.
Read state ONLY from /joint_states.

Do NOT call Isaac Sim's Python articulation API from control nodes. It is
faster in the short term and it welds the project to the simulator. Keeping
this boundary clean is what makes "sim-to-hardware is a controller swap" an
honest claim rather than a hopeful one.

### Two hardware realities the simulator will not teach you

  * MotoROS2 requires FastDDS. It does not support Cyclone DDS. This is a
    second, independent argument against the Cyclone choice in Section 5.3.
  * Real controllers enforce motion limits, speed overrides, and safety
    interlocks that Isaac Sim ignores. Trajectories that run clean in sim can
    be rejected outright by a YRC1000.

## motoman (ros2 branch, gavanderhoorn fork)
Yaskawa URDF/xacro descriptions as ament_cmake packages.
ros-industrial/motoman is ROS 1 only — do not use it.

All three arms in the proposal are available and verified:

    Station 1,2  GP4      motoman_gp4_support/urdf/gp4.xacro
    Station 4    GP7      motoman_gp7_support/urdf/gp7.xacro
    Station 3    HC10DT   motoman_hc10_support/urdf/hc10dt.xacro

On the HC10: the fork has hc10, hc10dt, and hc10dt_b10 — no hc10dtp. DTP is
the IP67-rated variant of the DT with identical kinematics and reach, so
hc10dt.xacro is correct. Worth one sentence in Section 2 so a reader does not
read it as a spec mismatch.

Isaac Sim imports these via the URDF importer extension. Expect to fix up
joint limits, damping, and collision meshes after import — the visual meshes
are detailed enough to hurt physics performance if you use them as collision
geometry for all four arms at once.

## micro-ROS-Agent
UDP bridge between a PC and MotoROS2 on a controller. No use without hardware;
present to document the integration path.
VENDOR_EOF
  fi

  warn "MotoROS2 runs on physical Yaskawa controllers, not on this PC."
  warn "See ${MOTO_WS}/README.md — this matters for how you word Section 5.1."

  ok "MotoROS2 client-side sources staged (not yet built)"
fi

# ---------------------------------------------------------------------------
# Stage: python
# ---------------------------------------------------------------------------
if should_run python; then
  stage "5/7  Python venv, Open3D, OpenCV, PyTorch"

  # A venv on 3.11 (not system 3.12) keeps pip installs out of the system
  # site-packages, which Ubuntu 24.04 protects via PEP 668 anyway. It also
  # means Open3D and PyTorch are importable from Isaac Sim's own interpreter.
  if [[ ! -d "${VENV_DIR}" ]]; then
    info "creating venv at ${VENV_DIR} (Python ${PY_VERSION})"
    run "python${PY_VERSION}" -m venv "${VENV_DIR}"
  else
    skip "venv"
  fi

  PIP="${VENV_DIR}/bin/pip"
  if (( DRY_RUN )); then PIP="echo would-pip"; fi

  info "upgrading pip toolchain"
  run "${PIP}" install --upgrade pip setuptools wheel

  info "point cloud, vision, and scientific stack"
  run "${PIP}" install \
    "open3d>=0.18" \
    "opencv-python>=4.10" \
    "opencv-contrib-python>=4.10" \
    "numpy<2.0" \
    scipy matplotlib pandas \
    trimesh pyransac3d scikit-learn \
    pyyaml tqdm rich typer

  # numpy pinned below 2.0 deliberately: Open3D wheels and several ROS 2 Jazzy
  # Python packages still link against the 1.x ABI. Lift this pin only after
  # verifying open3d and cv_bridge both import cleanly.

  info "protobuf / gRPC for the inspector-to-correction pipeline"
  run "${PIP}" install grpcio grpcio-tools protobuf

  info "experiment tracking"
  run "${PIP}" install wandb mlflow tensorboard

  if (( SKIP_TORCH )); then
    warn "skipping PyTorch (--skip-torch)"
  else
    info "PyTorch with CUDA (this is a large download, be patient)"
    run "${PIP}" install torch torchvision torchaudio --index-url "${CUDA_WHEEL_INDEX}"
    info "Stable-Baselines3 for RL prototyping"
    run "${PIP}" install "stable-baselines3[extra]" gymnasium
  fi

  ok "Python environment ready"
fi

# ---------------------------------------------------------------------------
# Stage: tools
# ---------------------------------------------------------------------------
if should_run tools; then
  stage "6/7  VS Code, Gazebo, Foxglove"

  # --- VS Code -------------------------------------------------------------
  if ! have code; then
    info "adding Microsoft apt repository"
    if (( DRY_RUN )); then
      log "${C_DIM}  would add packages.microsoft.com repo${C_RESET}"
    else
      sudo install -d -m 0755 /etc/apt/keyrings
      curl -fsSL https://packages.microsoft.com/keys/microsoft.asc \
        | sudo gpg --dearmor -o /etc/apt/keyrings/packages.microsoft.gpg
      sudo chmod 0644 /etc/apt/keyrings/packages.microsoft.gpg
      printf 'deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main\n' \
        | sudo tee /etc/apt/sources.list.d/vscode.list >/dev/null
      sudo apt-get update
    fi
    apt_ensure code
  else
    skip "VS Code"
  fi

  if have code && ! (( DRY_RUN )); then
    info "installing VS Code extensions"
    # --force keeps this idempotent and non-interactive on re-runs.
    for ext in \
      ms-python.python \
      ms-python.vscode-pylance \
      ms-iot.vscode-ros \
      ms-vscode.cpptools-extension-pack \
      ms-vscode.cmake-tools \
      redhat.vscode-xml \
      redhat.vscode-yaml \
      zxh404.vscode-proto3 \
      eamodio.gitlens \
      charliermarsh.ruff
    do
      code --install-extension "$ext" --force >>"${LOG_FILE}" 2>&1 \
        || warn "extension ${ext} failed (non-fatal)"
    done
    ok "VS Code extensions installed"
  fi

  # --- Gazebo --------------------------------------------------------------
  # ros_gz is the Jazzy-era bridge to Gazebo Harmonic; installing the ros-*
  # metapackage pulls Gazebo itself as a dependency and guarantees an ABI match.
  info "Gazebo Harmonic + ros_gz bridge"
  apt_ensure "ros-${ROS_DISTRO_NAME}-ros-gz" || \
    warn "ros-gz unavailable; install Gazebo separately from gazebosim.org"

  # --- Foxglove ------------------------------------------------------------
  if ! have foxglove-studio && ! snap list foxglove-studio >/dev/null 2>&1; then
    info "Foxglove Studio (snap)"
    if have snap; then
      run sudo snap install foxglove-studio || \
        warn "snap install failed — download the .deb from https://foxglove.dev/download"
    else
      warn "snapd not present; get the .deb from https://foxglove.dev/download"
    fi
  else
    skip "Foxglove Studio"
  fi

  ok "tooling installed"
fi

# ---------------------------------------------------------------------------
# Stage: ws
# ---------------------------------------------------------------------------
if should_run ws; then
  stage "7/7  Workspace scaffold and shell configuration"

  info "creating colcon workspace at ${WS_DIR}"
  run mkdir -p "${WS_DIR}/src"

  # Station-per-package mirrors the four-station line from the proposal, so
  # each robot's nodes stay independently buildable and testable.
  for pkg in motobat_bringup motobat_stacker motobat_welder \
             motobat_inspector motobat_sealer motobat_msgs motobat_description; do
    run mkdir -p "${WS_DIR}/src/${pkg}"
  done

  if ! (( DRY_RUN )); then
    cat > "${WS_DIR}/README.md" <<'WS_EOF'
# MotoBat ROS 2 Workspace

Four-station simulated battery cell assembly line.

    motobat_msgs         Custom interfaces: DefectReport, ScanResult, CorrectionCmd
    motobat_description  URDF/Xacro for the four Yaskawa arms + cell/fixture geometry
    motobat_bringup      Launch files that compose the full line
    motobat_stacker      Station 1  — Yaskawa GP4
    motobat_welder       Station 2  — Yaskawa GP4
    motobat_inspector    Station 3  — Yaskawa HC10DT(P) + 4-sensor array
    motobat_sealer       Station 4  — Yaskawa GP7

Build:

    cd ~/motobat/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash

## Interface discipline (read this before writing a control node)

Arms are commanded ONLY through control_msgs/FollowJointTrajectory actions.
State is read ONLY from /joint_states.

Never call Isaac Sim's Python articulation API from a control node. That
boundary is the entire reason this project can claim a sim-to-hardware path:
MotoROS2 on a real YRC1000 exposes the same FollowJointTrajectory server, so
migrating means swapping the controller, not rewriting the stack.

    sim:   MoveIt2 -> FollowJointTrajectory -> Isaac Sim controller
    real:  MoveIt2 -> FollowJointTrajectory -> micro-ROS Agent
                                            -> MotoROS2 -> arm

Isaac-Sim-specific code belongs in motobat_bringup (scene setup, sensor
spawning, defect injection) -- never in the per-station packages.
WS_EOF
  fi

  # --- Environment file ----------------------------------------------------
  # Kept as a separate sourced file rather than dumped into .bashrc so it can be
  # edited, version-controlled, and disabled in one place.
  ENV_FILE="${PROJECT_ROOT}/motobat_env.sh"
  if ! (( DRY_RUN )); then
    cat > "${ENV_FILE}" <<ENV_EOF
# MotoBat environment — sourced from ~/.bashrc
# Edit here, not in .bashrc.

source /opt/ros/${ROS_DISTRO_NAME}/setup.bash

# Cyclone DDS per Section 5.3 of the proposal.
# NOTE: NVIDIA recommends FastDDS (the default) for single-machine Isaac Sim,
# because it uses shared memory for large PointCloud2 traffic. If you see
# dropped point clouds between Isaac Sim and your nodes, comment the next line
# out and re-test on FastDDS before debugging anything else.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Isolate this project's DDS traffic from anything else on the LAN.
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Colourised, timestamped node output.
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}]: {message}'

# Overlay workspace, once it has been built at least once.
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

# Python venv activation helper: run 'mb-venv' to enter it.
alias mb-venv='source ${VENV_DIR}/bin/activate'
alias mb-ws='cd ${WS_DIR}'
alias mb-build='cd ${WS_DIR} && colcon build --symlink-install && source install/setup.bash'

# Isaac Sim launcher — set ISAAC_SIM_PATH after installing Isaac Sim.
# export ISAAC_SIM_PATH="\${HOME}/isaacsim"
# alias isaac='\${ISAAC_SIM_PATH}/isaac-sim.sh'
ENV_EOF

    # Idempotent .bashrc hook — guarded by a marker so re-runs don't stack up.
    MARKER="# >>> motobat env >>>"
    if ! grep -qF "${MARKER}" "${HOME}/.bashrc" 2>/dev/null; then
      {
        printf '\n%s\n' "${MARKER}"
        printf 'if [ -f "%s" ]; then source "%s"; fi\n' "${ENV_FILE}" "${ENV_FILE}"
        printf '%s\n' "# <<< motobat env <<<"
      } >> "${HOME}/.bashrc"
      ok "hooked ${ENV_FILE} into ~/.bashrc"
    else
      skip "bashrc hook"
    fi
  fi

  # --- Git identity --------------------------------------------------------
  if ! git config --global user.email >/dev/null 2>&1; then
    warn "git identity not configured. Set it with:"
    warn "    git config --global user.name  \"Rhishikesh Kandge\""
    warn "    git config --global user.email \"you@example.com\""
  else
    skip "git identity"
  fi

  ok "workspace scaffolded"
fi

# ---------------------------------------------------------------------------
# Stage: check — Isaac Sim preflight
# ---------------------------------------------------------------------------
if should_run check || [[ -z "${ONLY_STAGE}" ]]; then
  stage "Isaac Sim preflight (no install — verification only)"

  ISAAC_READY=1

  # --- GPU -----------------------------------------------------------------
  if have nvidia-smi; then
    GPU_NAME="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)"
    DRIVER_VER="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)"
    VRAM_MB="$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1)"

    ok "GPU:    ${GPU_NAME}"
    ok "Driver: ${DRIVER_VER}"
    ok "VRAM:   ${VRAM_MB} MiB"

    # RTX is a hard requirement: Isaac Sim's sensor pipeline is RTX-raytracing
    # based, and the SICK/ZED lidar models in Section 4 render through it.
    if ! grep -Eqi 'RTX|Quadro|Tesla|A[0-9]{4}|L[0-9]{2}|H100' <<<"${GPU_NAME}"; then
      err "GPU does not appear to be RTX-class. Isaac Sim's RTX Lidar and"
      err "sensor rendering will not run on a non-RTX card."
      ISAAC_READY=0
    fi

    DRIVER_MAJOR="${DRIVER_VER%%.*}"
    if [[ -n "${DRIVER_MAJOR}" ]] && (( DRIVER_MAJOR < 535 )); then
      err "Driver ${DRIVER_VER} is too old. Isaac Sim 6.0 wants 535+."
      err "Fix: sudo ubuntu-drivers install    (then reboot)"
      ISAAC_READY=0
    fi

    if [[ -n "${VRAM_MB}" ]] && (( VRAM_MB < 8000 )); then
      warn "Under 8 GB VRAM. Isaac Sim will run but a four-robot scene with"
      warn "four active sensors will be tight. 12 GB+ is comfortable."
    fi
  else
    err "nvidia-smi not found — no NVIDIA driver installed."
    err "Fix: sudo ubuntu-drivers install    (then reboot)"
    ISAAC_READY=0
  fi

  # --- RAM -----------------------------------------------------------------
  RAM_GB=$(( $(awk '/MemTotal/ {print $2}' /proc/meminfo) / 1024 / 1024 ))
  if (( RAM_GB < 32 )); then
    warn "System RAM ${RAM_GB} GB — Isaac Sim recommends 32 GB, 64 GB ideal."
  else
    ok "RAM:    ${RAM_GB} GB"
  fi

  # --- Disk ----------------------------------------------------------------
  DISK_GB=$(( $(df --output=avail -k "${HOME}" | tail -1) / 1024 / 1024 ))
  if (( DISK_GB < 100 )); then
    warn "Only ${DISK_GB} GB free in \$HOME. Isaac Sim plus its asset cache"
    warn "wants 100 GB+; shader and USD caches grow steadily during use."
  else
    ok "Disk:   ${DISK_GB} GB free"
  fi

  if (( ISAAC_READY )); then
    ok "This machine meets the Isaac Sim requirements."
  else
    err "Resolve the errors above before installing Isaac Sim."
  fi
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
stage "Done"

cat <<'NEXT'
Next steps
----------

1.  Load the new environment (once per new terminal it is automatic):

        source ~/.bashrc

2.  Verify ROS 2:

        ros2 doctor --report
        ros2 run demo_nodes_cpp talker     # in one terminal
        ros2 run demo_nodes_py listener    # in another

3.  Install Isaac Sim 6.0 — NOT scripted, and deliberately so. The download is
    ~30 GB and the installer is version-sensitive enough that a wrapper script
    tends to rot faster than it helps.

        https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html

    Download the Linux zip, extract to ~/isaacsim, then:

        cd ~/isaacsim && ./post_install.sh && ./isaac-sim.selector.sh

    First launch compiles shaders and takes 10-30 minutes. It is not hung.

    Then uncomment ISAAC_SIM_PATH in ~/motobat/motobat_env.sh.

4.  Isaac Sim + ROS 2 bridge: enable the isaacsim.ros2.bridge extension and
    source /opt/ros/jazzy/setup.bash BEFORE launching isaac-sim.sh, so it uses
    your system ROS rather than its bundled internal libraries.

5.  Isaac Lab (only needed for the Section 5.4 RL stretch goal):

        git clone https://github.com/isaac-sim/IsaacLab.git ~/IsaacLab
        cd ~/IsaacLab && ./isaaclab.sh --install

Three things worth knowing
--------------------------

  * MotoROS2 does not run on this machine, and cannot. It is a MotoPlus
    application that runs ON a physical Yaskawa controller (DX200 / YRC1000).
    There is no PC build and no simulation mode. What is staged in
    ros2_ws/src/_vendor/ is the CLIENT side: the message/service/action
    definitions your nodes use to speak the real MotoROS2 interface, plus the
    micro-ROS Agent that would bridge to hardware.

    This is not a blocker. MotoROS2 presents a standard
    control_msgs/FollowJointTrajectory action server -- the same interface
    MoveIt2 drives in Isaac Sim. So the hardware path is:

        sim:   MoveIt2 -> FollowJointTrajectory -> Isaac Sim controller
        real:  MoveIt2 -> FollowJointTrajectory -> Agent -> MotoROS2 -> arm

    Same planning code, same inspection pipeline, same correction loop.

    The discipline that makes this real: command arms ONLY via
    FollowJointTrajectory, read state ONLY from /joint_states, and never call
    Isaac Sim's articulation API from a control node.

    For Section 5.1, this is a stronger claim than "we use MotoROS2":
    the system targets the standard ROS 2 manipulation interfaces that
    MotoROS2 implements, making sim-to-hardware a controller swap rather than
    a rewrite. Section 2.2's Yaskawa-over-FANUC reasoning survives intact.

    Note for Section 5.3: MotoROS2 requires FastDDS and does not support
    Cyclone DDS -- a second independent argument against the Cyclone choice.

Two more things worth knowing
-----------------------------

  * RMW choice. The env file sets Cyclone DDS per Section 5.3 of the proposal,
    but NVIDIA recommends FastDDS for single-machine Isaac Sim because it uses
    shared memory for large PointCloud2 messages. If point clouds arrive
    truncated or drop under load, switch back to FastDDS before debugging
    anything else — this is a known and easily misdiagnosed failure.

  * Python versions. System Python is 3.12; the venv is 3.11 to match Isaac
    Sim's embedded interpreter. Use the venv (mb-venv) for Open3D and PyTorch
    work. Do not pip install into system Python — 24.04 blocks it via PEP 668,
    and overriding that will eventually break apt.

Handy aliases: mb-venv  mb-ws  mb-build
NEXT

log ""
ok "Full log: ${LOG_FILE}"

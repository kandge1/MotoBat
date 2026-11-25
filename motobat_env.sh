# MotoBat environment — sourced from ~/.bashrc
# Edit here, not in .bashrc.
#
# NOTE: deliberately no `set -u` anywhere near this file.
# /opt/ros/jazzy/setup.bash reads AMENT_TRACE_SETUP_FILES and friends without
# defaults; under `set -u` that is a fatal "unbound variable". See project_plan
# §1.2 — this bit us once already.

# Resolve the repo root from this file's own location, so a re-clone to a
# different path needs no edits here.
MOTOBAT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MOTOBAT_ROOT
export MOTOBAT_VENV="${HOME}/motobat/venv"

source /opt/ros/jazzy/setup.bash

# Cyclone DDS per Section 5.3 of the proposal.
# NOTE: NVIDIA recommends FastDDS (the default) for single-machine Isaac Sim,
# because it uses shared memory for large PointCloud2 traffic. MotoROS2 also
# requires FastDDS and does not support Cyclone. If you see dropped point
# clouds between Isaac Sim and your nodes, comment the next line out and
# re-test on FastDDS before debugging anything else. See project_plan §5.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Isolate this project's DDS traffic from anything else on the LAN.
export ROS_DOMAIN_ID=42

# Which network DDS uses (18 Nov 2026). By default everything stays on this machine's
# loopback interface (127.0.0.1): Cyclone otherwise binds to the Wi-Fi address it finds
# when a node starts, and when that network drops or changes, every running node floods
# "ddsi_udp_conn_write ... failed" and stops talking (seen 18 Nov). Nothing in the
# project needs the LAN yet. Set MOTOBAT_DDS_LAN=1 before sourcing to let other machines
# (a real controller, a second PC) see these nodes: then the Wi-Fi caveat is back.
# Loopback has no multicast, so discovery goes by unicast to 127.0.0.1, with a
# participant index per process (up to 120 ROS processes at once).
unset ROS_LOCALHOST_ONLY            # deprecated in Jazzy; the setting below replaces it
if [ "${MOTOBAT_DDS_LAN:-0}" = "1" ]; then
    unset CYCLONEDDS_URI
else
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
fi

# Colourised, timestamped node output.
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}]: {message}'

# Overlay this workspace, once it has been built at least once.
if [ -f "${MOTOBAT_ROOT}/install/setup.bash" ]; then
  source "${MOTOBAT_ROOT}/install/setup.bash"
fi

# The repo IS the colcon workspace, so mb-ws and the repo root are the same
# place. The venv lives outside it on purpose — see project_structure.md.
alias mb-venv='source "${MOTOBAT_VENV}/bin/activate"'
alias mb-ws='cd "${MOTOBAT_ROOT}"'
alias mb-build='cd "${MOTOBAT_ROOT}" && colcon build --symlink-install && source install/setup.bash'

# Isaac Sim launcher — set ISAAC_SIM_PATH after installing Isaac Sim.
# export ISAAC_SIM_PATH="${HOME}/isaacsim"
# alias isaac='${ISAAC_SIM_PATH}/isaac-sim.sh'

#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WS_DIR}"

IFNAME="${IFNAME:-enp4s0}"
CAN_BAUD="${CAN_BAUD:-1M}"
SLAVE_HIGH_FRONT="${SLAVE_HIGH_FRONT:-2}"
SLAVE_HIGH_UP_DOWN="${SLAVE_HIGH_UP_DOWN:-1}"
SLAVE_LOW="${SLAVE_LOW:-0}"
SCAN_SECONDS="${SCAN_SECONDS:-8}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[ERROR] /opt/ros/humble/setup.bash not found" >&2
    exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/src/linkx_bringup/config/fastrtps_profiles.xml"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${WS_DIR}/var_data/ros_log}"

mkdir -p "${ROS_LOG_DIR}"

echo "[SCAN] ifname=${IFNAME} can_baud=${CAN_BAUD} high_front=${SLAVE_HIGH_FRONT} high_up_down=${SLAVE_HIGH_UP_DOWN} low=${SLAVE_LOW} seconds=${SCAN_SECONDS}"

sudo -v

sudo -E bash -lc "
  source /opt/ros/humble/setup.bash &&
  source '${WS_DIR}/install/setup.bash' &&
  export FASTRTPS_DEFAULT_PROFILES_FILE='${FASTRTPS_DEFAULT_PROFILES_FILE}' &&
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}' &&
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' &&
  export ROS_LOG_DIR='${ROS_LOG_DIR}' &&
  timeout '${SCAN_SECONDS}' '${WS_DIR}/install/tof/lib/tof/can_link_test' '${IFNAME}' '${CAN_BAUD}' '${SLAVE_HIGH_FRONT}' '${SLAVE_HIGH_UP_DOWN}' '${SLAVE_LOW}'
"

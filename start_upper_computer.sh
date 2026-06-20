#!/usr/bin/env bash
# 启动上位机 ROS 2 控制栈：joy → remote_node → topic_relay → linkx_soem_demo (EtherCAT)
#
# 使用方法:
#   ./start_upper_computer.sh                                 # 默认 ifname=enp4s0, sudo
#   ./start_upper_computer.sh --ifname eth0                   # 指定网卡
#   ./start_upper_computer.sh --no-vehicle                    # 仅启动 ROS 话题，不启动 EtherCAT 主控
#   ./start_upper_computer.sh --no-sudo                       # 已 setcap 时跳过 sudo
#   ./start_upper_computer.sh --gimbal                        # 同时启动云台话题转发
#
# 前置条件:
#   - 已安装 ROS 2 Humble (/opt/ros/humble/setup.bash)
#   - 已安装 ros-humble-joy
#   - F710 / 兼容手柄已通过 USB 接入
#   - EtherCAT 网卡已 UP 且未被其他进程占用

set -euo pipefail

WS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${WS_DIR}"

IFNAME="${IFNAME:-enp4s0}"
USE_SUDO="true"
START_VEHICLE="true"
GIMBAL="false"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ifname)      IFNAME="$2"; shift 2;;
        --no-sudo)     USE_SUDO="false"; shift;;
        --no-vehicle)  START_VEHICLE="false"; shift;;
        --gimbal)      GIMBAL="true"; shift;;
        *) echo "Unknown arg: $1" >&2; exit 1;;
    esac
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[ERROR] /opt/ros/humble/setup.bash not found. Install ROS 2 Humble first." >&2
    exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u

echo "[SETUP] Building workspace (colcon)..."
COLCON_CACHE_ARGS=()
CMAKE_CACHE_FILES=("${WS_DIR}"/build/*/CMakeCache.txt)
for CMAKE_CACHE_FILE in "${CMAKE_CACHE_FILES[@]}"; do
    [[ -f "${CMAKE_CACHE_FILE}" ]] || continue
    if grep -q '^CMAKE_INSTALL_PREFIX:PATH=/usr/local$' "${CMAKE_CACHE_FILE}"; then
        echo "[SETUP] stale CMake cache uses /usr/local install prefix: ${CMAKE_CACHE_FILE}"
        COLCON_CACHE_ARGS+=(--cmake-clean-cache)
        break
    fi
    if ! grep -q '^CMAKE_C_COMPILER:.*=/usr/bin/gcc$' "${CMAKE_CACHE_FILE}" ||
       ! grep -q '^CMAKE_CXX_COMPILER:.*=/usr/bin/g++$' "${CMAKE_CACHE_FILE}"; then
        echo "[SETUP] compiler in CMake cache changed: ${CMAKE_CACHE_FILE}"
        COLCON_CACHE_ARGS+=(--cmake-clean-cache)
        break
    fi
done
if [[ ${#COLCON_CACHE_ARGS[@]} -gt 0 ]]; then
    echo "[SETUP] cleaning CMake package caches before build..."
fi

CC=/usr/bin/gcc CXX=/usr/bin/g++ \
  colcon build "${COLCON_CACHE_ARGS[@]}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DCMAKE_C_COMPILER=/usr/bin/gcc \
                 -DCMAKE_CXX_COMPILER=/usr/bin/g++

set +u
source install/setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/src/linkx_bringup/config/fastrtps_profiles.xml"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ENABLE_TOF_PRINT="1"
export TOF_PRINT_STDOUT="${TOF_PRINT_STDOUT:-0}"
export TOF_PRINT_FILE="${TOF_PRINT_FILE:-${WS_DIR}/var_data/terminal/ops_terminal.log}"

# raw 以太网权限：默认 sudo 启动，需要保留 LD_LIBRARY_PATH
PREFIX=""
if [[ "${USE_SUDO}" == "true" ]]; then
    echo "[SETUP] checking sudo permission for EtherCAT raw socket..."
    sudo -v
    PREFIX="sudo -E env LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-} FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ENABLE_TOF_PRINT=${ENABLE_TOF_PRINT} TOF_PRINT_STDOUT=${TOF_PRINT_STDOUT} TOF_PRINT_FILE=${TOF_PRINT_FILE}"
fi

if ! ip link show "${IFNAME}" >/dev/null 2>&1; then
    echo "[ERROR] EtherCAT interface '${IFNAME}' not found. Use --ifname <netdev>." >&2
    ip -br link >&2
    exit 1
fi

if ! ip -br link show "${IFNAME}" | awk '{print $2}' | grep -qw "UP"; then
    if [[ "${USE_SUDO}" == "true" ]]; then
        echo "[SETUP] bringing EtherCAT interface ${IFNAME} up..."
        sudo ip link set "${IFNAME}" up
        sleep 1
    else
        echo "[WARN] EtherCAT interface ${IFNAME} is not UP. Run: sudo ip link set ${IFNAME} up" >&2
    fi
fi

echo "[START] ifname=${IFNAME} vehicle=${START_VEHICLE} gimbal=${GIMBAL} sudo=${USE_SUDO}"
TAIL_PID=""
if [[ "${START_VEHICLE}" == "true" ]]; then
    mkdir -p var_data/terminal var_data/tof var_data/chassis_trace var_data/omni var_data/lift var_data/calibration
    if [[ ! -w var_data ]]; then
        if [[ "${USE_SUDO}" == "true" ]]; then
            echo "[SETUP] fixing var_data ownership for runtime logs..."
            sudo chown -R "$(id -u):$(id -g)" var_data
        else
            echo "[ERROR] var_data is not writable. Run: sudo chown -R $(id -u):$(id -g) var_data" >&2
            exit 1
        fi
    fi
    : > "${TOF_PRINT_FILE}"
    echo "[MONITOR] tailing ToF/IMU/Lift feedback: ${TOF_PRINT_FILE}"
    tail -n 0 -F "${TOF_PRINT_FILE}" &
    TAIL_PID="$!"
    trap 'if [[ -n "${TAIL_PID}" ]]; then kill "${TAIL_PID}" >/dev/null 2>&1 || true; fi' EXIT INT TERM
fi

LAUNCH_ARGS=(
    "ifname:=${IFNAME}"
    "start_vehicle_control:=${START_VEHICLE}"
    "start_gimbal_bridge:=${GIMBAL}"
)
if [[ -n "${PREFIX}" ]]; then
    LAUNCH_ARGS+=("vehicle_prefix:=${PREFIX}")
fi

ros2 launch linkx_bringup full_system.launch.py "${LAUNCH_ARGS[@]}" 2>&1 | awk '
        /^\[INFO\] \[launch\]: All log files can be found below / { next }
        /^\[INFO\] \[launch\]: Default logging verbosity is set to INFO$/ { next }
        /^\[INFO\] \[[^]]+-[0-9]+\]: process started with pid \[[0-9]+\]$/ { next }
        { print; fflush(); }
    '

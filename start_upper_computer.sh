#!/usr/bin/env bash
# 启动上位机 ROS 2 控制栈：joy → remote_node → cmd_vel → chassis_relay → linkx_soem_demo (EtherCAT)
#
# 使用方法:
#   ./start_upper_computer.sh                                 # 默认 ifname=enxf01e341224fd (拓展坞), sudo, max_speed=1.5
#   ./start_upper_computer.sh --ifname eth0                   # 指定网卡
#   ./start_upper_computer.sh --no-vehicle                    # 仅启动 ROS 话题，不启动 EtherCAT 主控
#   ./start_upper_computer.sh --no-sudo                       # 已 setcap 时跳过 sudo
#   ./start_upper_computer.sh --gimbal                        # 同时启动云台话题转发
#   ./start_upper_computer.sh --max-speed 1.0
#
# 前置条件:
#   - 已安装 ROS 2 Humble (/opt/ros/humble/setup.bash)
#   - 已安装 ros-humble-joy
#   - F710 / 兼容手柄已通过 USB 接入
#   - EtherCAT 网卡已 UP 且未被其他进程占用

set -euo pipefail

WS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${WS_DIR}"

IFNAME="${IFNAME:-enxf01e341224fd}"
MAX_SPEED="${MAX_SPEED:-1.5}"
USE_SUDO="true"
START_VEHICLE="true"
GIMBAL="false"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ifname)      IFNAME="$2"; shift 2;;
        --max-speed)   MAX_SPEED="$2"; shift 2;;
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

if [[ ! -d build || ! -d install ]]; then
    echo "[INFO] First-time build (colcon)..."
    CC=/usr/bin/gcc CXX=/usr/bin/g++ \
      colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
                                -DCMAKE_C_COMPILER=/usr/bin/gcc \
                                -DCMAKE_CXX_COMPILER=/usr/bin/g++
fi
set +u
source install/setup.bash
set -u

# raw 以太网权限：默认 sudo 启动，需要保留 LD_LIBRARY_PATH
PREFIX=""
if [[ "${USE_SUDO}" == "true" ]]; then
    PREFIX='sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH'
fi

echo "[INFO] launching with ifname=${IFNAME} max_speed=${MAX_SPEED} vehicle=${START_VEHICLE} gimbal=${GIMBAL} sudo=${USE_SUDO}"

exec ros2 launch linkx_bringup full_system.launch.py \
    ifname:="${IFNAME}" \
    max_speed:="${MAX_SPEED}" \
    start_vehicle_control:="${START_VEHICLE}" \
    start_gimbal_bridge:="${GIMBAL}" \
    vehicle_prefix:="${PREFIX}"

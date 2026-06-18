# R2 EtherCAT 上位机控制栈

这是 R2 机器人上位机 ROS 2 工作区，核心链路为 **SOEM EtherCAT 主站 -> LinkX-4C -> CAN -> 电机/传感器**。工作区同时包含整车主控、F710 手柄遥控、HiPNUC IMU、ToF 测距和 bringup 启动配置。

- 系统平台：Ubuntu + ROS 2 Humble
- 主要语言：C / C++17
- 构建系统：colcon + ament_cmake
- 核心入口：`./start_upper_computer.sh`
- 主控包：`linkx_soem_demo`

> 当前源码中网卡默认值不完全一致：`start_upper_computer.sh` 实际默认 `enp4s0`，`full_system.launch.py` 和 `linkx_soem_demo` 可执行默认 `enp86s0`。联调时建议始终通过 `--ifname <网卡名>` 或 `ifname:=<网卡名>` 显式指定。

## 系统架构

```text
Logitech F710
    |
    v
joy_node -> remote_node_cpp -> /chassis/remote_cmd_vel ----+
                            -> /robot_buttons -------------+
上层规划 /cmd_vel -> topic_relay_cpp(chassis_relay) -------+
                            -> /chassis/cmd_vel             |
                            -> /chassis/buttons             |
HiPNUC IMU -> /IMU_data -----------------------------------+
                                                            v
linkx_soem_demo(vehicle_control)
    |
    v
SOEM EtherCAT Master -> LinkX-4C -> CAN0~CAN3 -> 全向轮/升降/夹爪/ToF
```

主控内部优先级：

1. `std_srvs/Trigger` 服务动作优先级最高，例如 `/vehicle/enable`、`/vehicle/stair/up_raise_8_0`。
2. `/chassis/cmd_vel` 为高优先级速度入口，通常由导航或规划经 `/cmd_vel` relay 转入。
3. `/chassis/remote_cmd_vel` 为低优先级遥控入口，高优先级速度 200 ms 超时后回落。
4. START/BACK 按键控制整车使能/失能；失能后速度话题不会驱动底盘。

## 包说明

| 包 | 作用 |
| --- | --- |
| `linkx_soem_demo` | 整车主控、SOEM/LinkX 通信、F710 遥控解算、话题转发和若干硬件测试工具 |
| `linkx_bringup` | `full_system.launch.py`、`teleop.launch.py` 和 FastRTPS 配置 |
| `hipnuc_imu` | HiPNUC 串口 IMU 发布节点，默认发布 `/IMU_data` |
| `hipnuc_lib_package` | HiPNUC 协议解析共享库 |
| `tof` | EtherCAT -> LinkX-4C -> CAN 测距节点，发布 `sensor_msgs/Range` |

目录结构：

```text
.
├── README.md
├── start_upper_computer.sh
└── src/
    ├── linkx_soem_demo/
    │   ├── src/vehicle_control/        # 整车主控
    │   │   ├── 1_Middleware/           # SOEM、LinkX、PID、ramp
    │   │   ├── 2_Device/               # EtherCAT、LinkX-4C、DM 电机、计时
    │   │   ├── 3_Chariot/              # 全向底盘、升降、夹爪、IMU 航向保持
    │   │   ├── 4_Interaction/robot/    # ROS 话题/服务桥
    │   │   └── 5_Task/task/            # 初始化和周期任务
    │   ├── src/remote/                 # F710 解算与话题转发
    │   └── src/extra_tests/            # 硬件联调工具
    ├── linkx_bringup/
    ├── hipnuc_imu/
    ├── hipnuc_lib_package/
    └── tof/
```

## 依赖安装

基础依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-joy \
  libpcap-dev
```

已假定 ROS 2 Humble 安装在 `/opt/ros/humble`。如果需要运行手柄链路，请确认 F710 或兼容手柄已接入，并能被 `/dev/input/js*` 识别。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

一键脚本会自动执行 Release 构建，并在检测到陈旧 CMake cache 时清理 package cache：

```bash
./start_upper_computer.sh --ifname enp4s0
```

构建产物主要位于：

| 路径 | 说明 |
| --- | --- |
| `build/` | colcon build 中间产物 |
| `install/<package>/lib/<package>/` | ROS 2 安装后的可执行文件 |
| `log/` | colcon 构建日志 |
| `var_data/ops_terminal.log` | 主控运行时 ToF/升降相关输出，脚本启动时会 tail |

## 运行权限

SOEM 需要直接打开网卡收发 raw Ethernet frame，因此整车主控和 ToF EtherCAT 节点需要 root 或 capability。

默认使用 `sudo`：

```bash
sudo ./start_upper_computer.sh --ifname enp4s0
```

也可以给可执行文件授予 capability：

```bash
sudo setcap cap_net_raw,cap_net_admin+ep \
  install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo

./start_upper_computer.sh --ifname enp4s0 --no-sudo
```

每次重新构建后，可执行文件可能被覆盖，需要重新执行 `setcap`。ToF 节点同理：

```bash
sudo setcap cap_net_raw,cap_net_admin+ep \
  install/tof/lib/tof/can_link_test
```

运行前检查网卡：

```bash
ip -br link
sudo ip link set enp4s0 up
```

EtherCAT 网卡建议专用，不要同时承担上网、SSH、远程桌面等普通网络流量。

## 启动整车

推荐使用一键脚本：

```bash
# 启动完整系统：IMU + joy + remote + relay + EtherCAT 主控
./start_upper_computer.sh --ifname enp4s0

# 仅启动 ROS 话题链路，不启动 EtherCAT 主控
./start_upper_computer.sh --ifname enp4s0 --no-vehicle

# 已配置 setcap 时跳过 sudo
./start_upper_computer.sh --ifname enp4s0 --no-sudo

# 同时启动云台话题 relay
./start_upper_computer.sh --ifname enp4s0 --gimbal
```

也可以直接使用 launch：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch linkx_bringup full_system.launch.py \
  ifname:=enp4s0 \
  start_vehicle_control:=true \
  start_gimbal_bridge:=false \
  start_imu:=true
```

`full_system.launch.py` 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `ifname` | `enp86s0` | 传给 `linkx_soem_demo` 的 EtherCAT 网卡名 |
| `start_vehicle_control` | `true` | 是否启动 EtherCAT 整车主控 |
| `start_gimbal_bridge` | `false` | 是否启动云台 relay |
| `start_imu` | `true` | 是否启动 HiPNUC IMU 发布节点 |
| `vehicle_prefix` | 空 | 主控启动前缀，常用于 `sudo -E env LD_LIBRARY_PATH=...` |
| `ros_nodes_prefix` | 空 | ROS 节点启动前缀，可用于 gdb/valgrind |

仅检查遥控链路：

```bash
ros2 launch linkx_bringup teleop.launch.py
```

## 话题约定

| 话题 | 类型 | 发布者 | 订阅者/用途 |
| --- | --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `remote_node_cpp` |
| `/cmd_vel` | `geometry_msgs/Twist` | 上层规划/导航 | `chassis_relay` |
| `/chassis/cmd_vel` | `geometry_msgs/Twist` | `chassis_relay` 或外部节点 | 主控高优先级速度入口 |
| `/chassis/remote_cmd_vel` | `geometry_msgs/Twist` | `remote_node_cpp` | 主控低优先级遥控速度入口 |
| `/robot_buttons` | `std_msgs/UInt16` | `remote_node_cpp` | `chassis_relay`、可选云台 relay |
| `/chassis/buttons` | `std_msgs/UInt16` | `chassis_relay` | 主控按键入口 |
| `/chassis/odom_twist` | `geometry_msgs/Twist` | `r2_vehicle_bridge` | 底盘实际速度反馈 |
| `/IMU_data` | `sensor_msgs/Imu` | `hipnuc_imu/talker` | 主控 IMU 航向保持 |
| `/gimbal/cmd_vel` | `geometry_msgs/Twist` | `gimbal_relay` | 可选云台入口 |
| `/gimbal/buttons` | `std_msgs/UInt16` | `gimbal_relay` | 可选云台入口 |

速度字段约定：

| 字段 | 含义 |
| --- | --- |
| `linear.x` | 底盘前后速度 |
| `linear.y` | 全向底盘横移速度 |
| `angular.z` | 底盘旋转角速度 |
| `angular.x` | 遥控右摇杆 Y 值，当前由 `remote_node_cpp` 透传 |

全向底盘限幅常量在 `crt_chassis_omni.h` 中定义：`MAX_OMNI_CHASSIS_SPEED=5.0`，`MAX_OMNI_CHASSIS_OMEGA=10.0`。

## 服务约定

整车主控启动后创建节点 `r2_vehicle_bridge`，并提供以下 `std_srvs/srv/Trigger` 服务：

| 服务 | 动作 |
| --- | --- |
| `/vehicle/enable` | 整车使能，等价 START |
| `/vehicle/disable` | 整车失能，等价 BACK，停止底盘/升降/辅助电机 |
| `/vehicle/stair/up_raise_8_0` | 以 `-8.0` 抬升角执行上台阶流程 |
| `/vehicle/stair/down_raise_8_0` | 以 `-8.0` 抬升角执行下台阶流程 |
| `/vehicle/stair/up_raise_14_3` | 以 `-14.3` 抬升角执行上台阶流程 |
| `/vehicle/stair/down_raise_14_3` | 以 `-14.3` 抬升角执行下台阶流程 |
| `/vehicle/lift_aux/raise` | 两侧升降到 `-39.0`，再将辅助电机控到抬起位置 |
| `/vehicle/lift_aux/home` | 辅助电机回零，再将升降收到 `-0.01` |
| `/vehicle/gripper/grab` | 向夹爪/下位机发送抓取命令 `0x01` |
| `/vehicle/gripper/release` | 向夹爪/下位机发送释放命令 `0x02` |

调用示例：

```bash
ros2 service call /vehicle/enable std_srvs/srv/Trigger {}
ros2 service call /vehicle/disable std_srvs/srv/Trigger {}
```

## HiPNUC IMU

`full_system.launch.py` 默认启动 `hipnuc_imu/talker`，配置文件为 `src/hipnuc_imu/config/hipnuc_config.yaml`。

默认参数：

| 参数 | 默认值 |
| --- | --- |
| `serial_port` | `/dev/ttyUSB0` |
| `baud_rate` | `921600` |
| `frame_id` | `imu_link` |
| `imu_topic` | `/IMU_data` |
| `hardware_attitude_reset_on_start` | `true` |
| `reset_attitude_on_start` | `true` |

单独启动：

```bash
ros2 launch hipnuc_imu imu_spec_msg.launch.py
ros2 topic echo /IMU_data
```

`hipnuc_imu` 还安装 `listener` 示例节点；`hipnuc_lib_package` 提供底层协议解析共享库。

## ToF 测距

`tof` 包提供 `can_link_test` 和 `up_range_subscriber`：

```bash
ros2 launch tof radar_launch.py ifname:=enp4s0 can_baud:=1M slave_high:=1 slave_low:=0

# 不启动四路 up/down 订阅打印节点
ros2 launch tof radar_launch.py ifname:=enp4s0 start_up_range_subscriber:=false

# 直接运行：网卡 波特率 高频从站 低频从站
ros2 run tof can_link_test enp4s0 1M 1 0
```

`radar_launch.py` 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `ifname` | `enp86s0` | EtherCAT 网卡 |
| `can_baud` | `1M` | CAN 波特率，支持 `1M`、`500k`、`250k` |
| `slave_high` | `1` | 高频 TFmini-S 所在 LinkX-4C 从站 ID |
| `slave_low` | `0` | 低频传感器从站 ID，`0` 表示禁用 |
| `start_up_range_subscriber` | `true` | 是否启动四路上/下高频测距订阅打印 |

主要 Range 话题：

| 话题 | 说明 |
| --- | --- |
| `/high/front/range`、`/high/left/range`、`/high/right/range`、`/high/back/range` | 高频前/左/右/后 |
| `/high/up_front/range`、`/high/up_back/range` | 高频上前/上后 |
| `/high/down_front/range`、`/high/down_back/range` | 高频下前/下后 |
| `/low/front/range`、`/low/left/range`、`/low/right/range`、`/low/back/range` | 低频前/左/右/后 |

更详细的 ToF CAN ID、帧格式和解析逻辑见 `src/tof/README.md`。

## 可执行文件

`linkx_soem_demo` 安装的主要可执行：

| 可执行 | 作用 |
| --- | --- |
| `linkx_soem_demo` | 整车主控：SOEM 主站 + LinkX-4C CAN 桥 + 底盘/升降/夹爪调度 |
| `remote_node_cpp` | `/joy` -> `/chassis/remote_cmd_vel` + `/robot_buttons` |
| `topic_relay_cpp` | 参数化话题转发，可用于 chassis/gimbal |
| `dm_motor_param_calib` | DM 电机参数标定工具 |
| `omni_motion_record` | 全向底盘运动记录工具 |
| `omni_accel_response` | 全向底盘加速度响应测试 |
| `imu_heading_hold_sweep` | IMU 航向保持参数扫描 |
| `lift_3519_feedback` | 升降 3519 反馈测试 |
| `lift_raise_drive_test` | 升降驱动测试 |
| `lift_tof_trigger_test` | 升降 ToF 触发测试 |

额外测试工具大多需要真实硬件、正确网卡和 raw socket 权限。运行前先查看对应 `src/extra_tests/*_main.cpp` 文件头部用法。

## EtherCAT 说明

本项目使用 PC 作为 EtherCAT 主站，LinkX-4C 作为 EtherCAT 从站。SOEM 不通过 IP 连接设备，而是直接在专用网卡上收发原始以太网帧。

| 概念 | 项目内含义 |
| --- | --- |
| Master | 上位机 `linkx_soem_demo` / `tof can_link_test` |
| Slave | LinkX-4C |
| PDO | 周期过程数据，用于 CAN 桥接收发 |
| SDO | 非周期参数读写和诊断 |
| Station Alias | 从站别名，建议联调时保持固定 |
| OP | EtherCAT 正常工作态，主控必须进入 OP 后才稳定周期通信 |
| WKC | Working Counter，连续异常通常意味着从站掉线、状态不对、网线或映射问题 |

启动路径：

```text
source ROS/install -> ros2 launch -> 打开 EtherCAT 网卡
  -> 扫描从站 -> 配置 PDO -> SAFE-OP -> 启动周期收发
  -> 请求 OP -> WKC 检查 -> 整车周期控制
```

## 常用检查

检查网卡：

```bash
ip -br link
ip link show enp4s0
```

检查手柄：

```bash
ls /dev/input/js*
ros2 run joy joy_node
ros2 topic echo /joy
```

检查 ROS 遥控链路：

```bash
./start_upper_computer.sh --ifname enp4s0 --no-vehicle
ros2 topic echo /chassis/remote_cmd_vel
ros2 topic echo /chassis/cmd_vel
ros2 topic echo /robot_buttons
```

检查 IMU：

```bash
ros2 topic hz /IMU_data
ros2 topic echo /IMU_data --once
```

检查整车服务：

```bash
ros2 service list | grep /vehicle
ros2 service call /vehicle/enable std_srvs/srv/Trigger {}
```

## 排错速查

| 现象 | 排查方向 |
| --- | --- |
| `Failed to open interface` | 网卡名错误、网卡未 UP、没有 `sudo`/`setcap` raw 权限 |
| `slave count = 0` | LinkX-4C 未上电、网线错误、不是专用 EtherCAT 网口、从站状态异常 |
| WKC 连续异常 | 检查线缆、从站供电、PDO 映射、LinkX 状态和 EtherCAT 状态机 |
| 手柄无输出 | 确认 F710 接收器、D/X 档位、`/joy` 是否有数据 |
| `/chassis/cmd_vel` 有数据但底盘不动 | 确认已 START 或调用 `/vehicle/enable`，并检查主控是否进入 OP |
| `setcap` 后仍权限不足 | 重新构建后 capability 丢失，重新对 install 下可执行执行 `setcap` |
| 找不到 ROS 包 | 确认当前目录是工作区根目录，并执行 `source install/setup.bash` |
| `/IMU_data` 没有数据 | 检查 `/dev/ttyUSB0`、波特率、串口权限和 `hipnuc_config.yaml` |

## 安全注意事项

- 调试底盘、升降、夹爪前，确认急停、支撑和限位有效。
- 首次联调建议先 `--no-vehicle` 验证 ROS 链路，再上 EtherCAT 主控。
- 修改 PID、速度上限、摩擦补偿、升降目标角后，先低速空载验证。
- EtherCAT 主控运行时不要让同一网卡承担普通网络流量。
- 额外测试工具会直接驱动硬件，运行前确认电机 ID、CAN 通道和机械状态。

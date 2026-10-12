# R2 EtherCAT 上位机控制栈

这是 R2 机器人上位机 ROS 2 工作区，核心链路为 **SOEM EtherCAT 主站 -> LinkX-4C -> CAN -> 电机/传感器**。工作区同时包含整车主控、F710 手柄遥控、HiPNUC IMU、ToF 测距和 bringup 启动配置。

- 系统平台：Ubuntu + ROS 2 Humble
- 主要语言：C / C++17
- 构建系统：colcon + ament_cmake
- 核心入口：`./start_upper_computer.sh`
- 主控包：`linkx_soem_demo`

> 网卡默认值已统一为 `enp4s0`：`start_upper_computer.sh`、`full_system.launch.py`、主控及各工具一致；优先级为 `--ifname` > `IFNAME` 环境变量 > `enp4s0`。换机器或换网口时建议显式指定。

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
| `linkx_soem_demo` | 整车主控、SOEM/LinkX 通信、F710 遥控解算、话题转发、TFmini-S ToF 发布和硬件联调工具 |
| `linkx_bringup` | `full_system.launch.py`、`teleop.launch.py` 和 FastRTPS 配置 |
| `hipnuc_imu` | HiPNUC 串口 IMU 发布节点（`hipnuc_imu_node`），默认发布 `/IMU_data` |
| `hipnuc_lib_package` | HiPNUC 协议解析共享库 |

目录结构：

```text
.
├── README.md
├── start_upper_computer.sh
├── scripts/
│   └── watch_tof_ranges.py            # ToF 话题实时监控脚本
└── src/
    ├── linkx_soem_demo/
    │   ├── include/linkx_soem_demo/    # 与 src 同构的头文件树
    │   │   ├── vehicle_control/        # 1_Middleware / 2_Device / 3_Chariot / 4_Interaction / 5_Task 分层
    │   │   ├── remote/                 # F710 手柄解算与遥控节点头文件
    │   │   ├── diagnostics/            # 运行时诊断辅助头文件
    │   │   └── tof/                    # TFmini-S Range 发布头文件
    │   └── src/
    │       ├── vehicle_control/        # 整车主控（分层同上）
    │       │   ├── 1_Middleware/       # SOEM、LinkX、PID、ramp
    │       │   ├── 2_Device/           # EtherCAT、LinkX-4C、DM 电机、计时
    │       │   ├── 3_Chariot/          # 全向底盘、升降、夹爪、IMU 航向保持
    │       │   ├── 4_Interaction/      # ROS 话题/服务桥
    │       │   └── 5_Task/             # 初始化和周期任务
    │       ├── remote/                 # F710 解算与话题转发
    │       ├── diagnostics/            # 运行时诊断辅助代码
    │       ├── tof/                    # TFmini-S Range 发布
    │       └── tools/                  # 硬件联调/标定/诊断工具（原 extra_tests）
    ├── linkx_bringup/
    ├── hipnuc_imu/
    └── hipnuc_lib_package/
```

## 依赖安装

基础依赖：

```bash
sudo apt update
sudo apt install -y ros-humble-joy
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

每次重新构建后，可执行文件可能被覆盖，需要重新执行 `setcap`。ToF 测距由主控进程内发布，主控具备 raw 权限即可；`src/linkx_soem_demo/src/tools/` 下的联调工具（如 `tfmini_s_can_read`）独立运行时同样需要 raw 权限。

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
| `ifname` | `IFNAME` 环境变量或 `enp4s0` | 传给 `linkx_soem_demo` 的 EtherCAT 网卡名 |
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
| `/IMU_data` | `sensor_msgs/Imu` | `hipnuc_imu/hipnuc_imu_node` | 主控 IMU 航向保持 |
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

`full_system.launch.py` 默认启动 `hipnuc_imu/hipnuc_imu_node`，配置文件为 `src/hipnuc_imu/config/hipnuc_config.yaml`。

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
ros2 launch hipnuc_imu hipnuc_imu.launch.py
ros2 topic echo /IMU_data
```

`hipnuc_imu` 还安装 `hipnuc_imu_listener` 示例节点（订阅并打印 IMU 消息）；`hipnuc_lib_package` 提供底层协议解析共享库。

## ToF 测距

ToF 已并入 `linkx_soem_demo`：主控进程内直接配置 LinkX-4C 从站并解析 TFmini-S 帧，以 `sensor_msgs/Range`（SensorData QoS）发布 12 路话题：

```bash
# 12 路 Range 话题实时可视化（前/后/左/右各左右两路 + 上/下各两路）
python3 scripts/watch_tof_ranges.py

# 直接监控 CAN 上的 TFmini-S 原始帧（建议显式指定网卡）
sudo IFNAME=enp4s0 ros2 run linkx_soem_demo tfmini_s_can_read --ifname enp4s0
```

`tfmini_s_can_read` 常用参数：`--ifname <网卡>`（默认取 `IFNAME` 环境变量或 `enp4s0`）、`--duration <秒>`（`0` 表示持续运行）；其余参数（`--can-rate`、`--print-raw` 等）见工具 `--help` 输出。

主控发布的 Range 话题：

| 话题 | 说明 |
| --- | --- |
| `/high/front_left/range`、`/high/front_right/range` | 高频前左/前右 |
| `/high/left_left/range`、`/high/left_right/range` | 高频左前/左后 |
| `/high/right_left/range`、`/high/right_right/range` | 高频右前/右后 |
| `/high/back_left/range`、`/high/back_right/range` | 高频后左/后右 |
| `/high/up_front/range`、`/high/up_back/range` | 高频上前/上后 |
| `/high/down_front/range`、`/high/down_back/range` | 高频下前/下后 |

ToF CAN ID、帧格式和解析逻辑见 `src/linkx_soem_demo/src/tof/tfmini_s_range_publisher.cpp`。

## 可执行文件

`linkx_soem_demo` 安装的主要可执行（联调工具源码位于 `src/linkx_soem_demo/src/tools/`）：

| 可执行 | 作用 |
| --- | --- |
| `linkx_soem_demo` | 整车主控：SOEM 主站 + LinkX-4C CAN 桥 + 底盘/升降/夹爪调度 |
| `remote_node_cpp` | `/joy` -> `/chassis/remote_cmd_vel` + `/robot_buttons` |
| `topic_relay_cpp` | 参数化话题转发，可用于 chassis/gimbal |
| `dm_motor_param_calib` | DM 电机参数标定工具 |
| `omni_motion_record` | 全向底盘运动记录工具 |
| `omni_accel_response` | 全向底盘加速度响应测试 |
| `remote_chassis_yaw_tune` | 遥控底盘偏航角调参工具 |
| `imu_heading_hold_sweep` | IMU 航向保持参数扫描 |
| `lift_3519_feedback` | 升降 3519 反馈测试 |
| `lift_3519_readonly` | 升降 3519 只读反馈诊断（不发送控制帧） |
| `lift_raise_drive_test` | 升降驱动测试 |
| `lift_tof_trigger_test` | 升降 ToF 触发测试 |
| `tfmini_s_can_read` | TFmini-S CAN 原始帧监控工具 |

联调工具大多需要真实硬件、正确网卡和 raw socket 权限。运行前先查看对应 `src/tools/*_main.cpp` 文件头部用法。

## EtherCAT 说明

本项目使用 PC 作为 EtherCAT 主站，LinkX-4C 作为 EtherCAT 从站。SOEM 不通过 IP 连接设备，而是直接在专用网卡上收发原始以太网帧。

| 概念 | 项目内含义 |
| --- | --- |
| Master | 上位机 `linkx_soem_demo`（联调工具如 `tfmini_s_can_read` 也是 EtherCAT 主站） |
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

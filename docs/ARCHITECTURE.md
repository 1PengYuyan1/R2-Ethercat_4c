# 架构说明

本文档描述 R2-Ethercat_4c 工作区的链路结构、包划分、内部代码分层与目录命名规则。
构建、运行与改动流程见 [MAINTENANCE.md](MAINTENANCE.md)。

## 1. 总览

- 系统平台：Ubuntu + ROS 2 Humble
- 主要语言：C / C++17
- 构建系统：colcon + ament_cmake
- 核心入口：`./start_upper_computer.sh`
- 主控包：`linkx_soem_demo`（可执行 `linkx_soem_demo`，节点名 `r2_vehicle_bridge`）

### 数据链路

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

### 主控内部优先级

1. `std_srvs/Trigger` 服务动作优先级最高（如 `/vehicle/enable`、`/vehicle/stair/up_raise_8_0`）。
2. `/chassis/cmd_vel` 为高优先级速度入口，通常由导航/规划经 `/cmd_vel` relay 转入。
3. `/chassis/remote_cmd_vel` 为低优先级遥控入口，高优先级速度 200 ms 超时后回落。
4. START/BACK 按键控制整车使能/失能；失能后速度话题不会驱动底盘。

## 2. 包划分

| 包 | 作用 |
| --- | --- |
| `linkx_soem_demo` | 整车主控、SOEM/LinkX 通信、F710 遥控解算、话题转发、TFmini-S ToF 发布与硬件联调工具 |
| `linkx_bringup` | `full_system.launch.py`、`teleop.launch.py` 与 FastRTPS 配置 |
| `hipnuc_imu` | HiPNUC 串口 IMU 发布节点（`hipnuc_imu_node`），默认发布 `/IMU_data` |
| `hipnuc_lib_package` | HiPNUC 协议解析共享库（Apache-2.0） |

`full_system.launch.py` 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `ifname` | `IFNAME` 环境变量或 `enp4s0` | 传给 `linkx_soem_demo` 的 EtherCAT 网卡名 |
| `start_vehicle_control` | `true` | 是否启动 EtherCAT 整车主控 |
| `start_gimbal_bridge` | `false` | 是否启动云台 relay |
| `start_imu` | `true` | 是否启动 HiPNUC IMU 发布节点 |
| `vehicle_prefix` | 空 | 主控启动前缀，常用于 `sudo -E env LD_LIBRARY_PATH=...` |
| `ros_nodes_prefix` | 空 | ROS 节点启动前缀，可用于 gdb/valgrind |

## 3. linkx_soem_demo 内部分层

`include/linkx_soem_demo/` 与 `src/linkx_soem_demo/src/` 保持同构，两侧目录一一对应：

| 层 | 目录 | 内容 |
| --- | --- | --- |
| 1 | `1_Middleware/` | SOEM EtherCAT 主站（`soem/`）、LinkX 协议（`linkx/`）、PID/斜坡算法（`algorithm/`）、数学工具（`math/`） |
| 2 | `2_Device/` | EtherCAT 从站管理（`ethercat/`）、LinkX-4C 处理、DM 电机（`motor/`）、实时计时（`rt_timing/`） |
| 3 | `3_Chariot/` | 全向底盘（`chassis/omni_wheel/`）、升降（`lift/`）、夹爪（`gripper/`）、IMU 航向保持（`imu_heading_hold/`） |
| 4 | `4_Interaction/` | ROS 话题/服务桥（`robot/`） |
| 5 | `5_Task/` | 初始化与周期任务（`task/`） |

主控之外的横向模块：

| 目录 | 内容 |
| --- | --- |
| `remote/` | F710 手柄解算（`device/dvc_log_f710.*`）与遥控/转发节点（`ros2/`） |
| `diagnostics/` | 运行时诊断辅助（can/chassis/task/timing terminal） |
| `tof/` | TFmini-S Range 发布 |
| `tools/` | 硬件联调、标定、诊断工具的可执行入口（`*_main.cpp`） |

## 4. 对外接口

### 话题

| 话题 | 类型 | 发布者 | 订阅者/用途 |
| --- | --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `remote_node_cpp` |
| `/cmd_vel` | `geometry_msgs/Twist` | 上层规划/导航 | `chassis_relay` |
| `/chassis/cmd_vel` | `geometry_msgs/Twist` | `chassis_relay` 或外部节点 | 主控高优先级速度入口 |
| `/chassis/remote_cmd_vel` | `geometry_msgs/Twist` | `remote_node_cpp` | 主控低优先级遥控速度入口 |
| `/robot_buttons` | `std_msgs/UInt16` | `remote_node_cpp` | `chassis_relay`、可选云台 relay |
| `/chassis/buttons` | `std_msgs/UInt16` | `chassis_relay` | 主控按键入口 |
| `/chassis/odom_twist` | `geometry_msgs/Twist` | `r2_vehicle_bridge` | 底盘实际速度反馈 |
| `/IMU_data` | `sensor_msgs/Imu` | `hipnuc_imu_node` | 主控 IMU 航向保持 |
| `/gimbal/cmd_vel` | `geometry_msgs/Twist` | `gimbal_relay` | 可选云台入口 |
| `/gimbal/buttons` | `std_msgs/UInt16` | `gimbal_relay` | 可选云台入口 |

速度字段约定：`linear.x` 前后、`linear.y` 横移、`angular.z` 旋转；`angular.x` 由遥控节点透传右摇杆 Y 值。
全向底盘限幅常量位于 `crt_chassis_omni.h`：`MAX_OMNI_CHASSIS_SPEED=5.0`、`MAX_OMNI_CHASSIS_OMEGA=10.0`。

### 服务

主控启动后创建节点 `r2_vehicle_bridge`，提供以下 `std_srvs/srv/Trigger` 服务：

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

### ToF

ToF 已并入 `linkx_soem_demo`：主控进程内直接配置 LinkX-4C 从站并解析 TFmini-S 帧，以
`sensor_msgs/Range`（SensorData QoS）发布 12 路 `/high/<位置>/range` 话题（前/后/左/右各两路，上/下各两路）。
CAN ID、帧格式与解析逻辑见 `src/linkx_soem_demo/src/tof/tfmini_s_range_publisher.cpp`。

### 可执行文件

`linkx_soem_demo` 安装的主要可执行：`linkx_soem_demo`（整车主控）、`remote_node_cpp`、`topic_relay_cpp`，
以及 `dm_motor_param_calib`、`omni_motion_record`、`omni_accel_response`、`remote_chassis_yaw_tune`、
`imu_heading_hold_sweep`、`lift_3519_feedback`、`lift_3519_readonly`、`lift_raise_drive_test`、
`lift_tof_trigger_test`、`tfmini_s_can_read` 等联调工具。

## 5. 目录布局与命名规则

顶层结构：

```text
.
├── README.md
├── LICENSE
├── THIRD_PARTY_NOTICES.md
├── start_upper_computer.sh
├── .github/workflows/ci.yml
├── docs/
├── scripts/
└── src/
    ├── linkx_soem_demo/
    ├── linkx_bringup/
    ├── hipnuc_imu/
    └── hipnuc_lib_package/
```

命名规则：

- `include/linkx_soem_demo/` 必须与 `src/linkx_soem_demo/src/` 同构，头文件路径可预测。
- 项目内文件统一「小写 + 下划线」；专有名词也随规则写作 `f710`、`hipnuc`，不在文件名中混用大小写。
- 前缀约定：`ec_*`/`soem` 为 EtherCAT 中间件，`dvc_*` 为设备驱动，`crt_*` 为底盘与执行机构，
  `alg_*` 为算法，`*_terminal` 为诊断终端，`*_main.cpp` 为工具可执行入口。
- 项目头文件不得与 C 标准库头同名（例如禁止 `math.h`、`string.h`），避免在 `-I` 顺序下覆盖系统头；
  项目自带数学工具因此命名为 `r2_math.h`。
- 联调工具源码只放在 `src/linkx_soem_demo/src/tools/`，且必须在 `CMakeLists.txt` 中被引用。

## 6. 静态布局校验

仓库自带只读校验脚本（不需要编译，也不需要硬件）：

```bash
python3 scripts/check_repo_layout.py .
```

检查项：

- A：历史遗留字符串（已删除的目录/话题/文件引用）不得出现；
- B：`CMakeLists.txt` 引用的 `src/...` 路径必须存在；
- C：`#include "..."` 的项目头路径必须按大小写精确解析；
- D：`linkx_soem_demo/include` 下不得出现无人引用的孤儿头文件；
- E：`linkx_soem_demo/src` 下的源文件必须被 CMake 引用；
- F：launch 文件中的 `executable=` 必须是同包 CMake 目标；
- G：项目头文件名不得与 C 标准库头同名。

退出码非 0 表示存在 `PROBLEM`，CI 会因此失败；`KNOWN` 为已知历史遗留（当前为空）。

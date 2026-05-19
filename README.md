# Ethercat-R2 上位机控制栈

基于 **SOEM / EtherCAT 主站 + LinkX-4C CAN 桥** 的 R2 整车 ROS 2 控制工作空间。
PC 通过一张普通以太网卡跑 EtherCAT,经由 LinkX-4C 把 4 路经典 CAN 桥到舵向(DM6225)/ 轮向(DM3519)/ 升降(Gantry)/ 机械臂(Arm)/ 真空(Suction) 等设备。
手柄(Logitech F710)通过 `joy` 解算成 `/cmd_vel` 与按键话题,转发给整车主控。

- 平台:Ubuntu + ROS 2 Humble
- 主语言:C / C++17

---

## 目录结构

```
Ethercat-R2/
├── README.md
└── ros2_ws/
    ├── start_upper_computer.sh         # 一键启动脚本(joy + remote + relay + 主控)
    └── src/
        ├── linkx_bringup/              # launch / 参数包
        │   ├── launch/
        │   │   ├── full_system.launch.py   # 完整系统(默认入口)
        │   │   └── teleop.launch.py        # 仅遥控数据通路(无主控)
        │   └── config/
        │       ├── teleop.params.yaml
        │       └── fastrtps_profiles.xml
        └── linkx_soem_demo/            # 主控 + 工具集 + ROS 桥
            ├── src/
            │   ├── vehicle_control/    # 整车主控可执行
            │   │   ├── main.cpp
            │   │   ├── middleware/     # 1) SOEM / LinkX / Algorithm (PID, ramp)
            │   │   ├── device/         # 2) Motor / Buzzer / Suction / OPS / ecat_manager / linkx4c_handler / rt_timing
            │   │   ├── chariot/        # 3) chassis / gantry / arm / navigation
            │   │   ├── interaction/    # 4) robot
            │   │   └── task/           # 5) task 顶层调度
            │   ├── remote/             # 手柄解算 + 串口/话题转发
            │   │   ├── ros2/           # remote_node / stm32_node / joystick_mapper
            │   │   └── device/Remote/  # F710 手柄驱动
            │   └── test_mains/         # 独立工具(标定 / 调参 / 链路测试)
            ├── include/
            └── CMakeLists.txt
```

主控源码遵循 `1_Middleware → 2_Device → 3_Chariot → 4_Interaction → 5_Task` 五层分层(命名沿用 R1 框架)。

---

## 依赖

- **ROS 2 Humble** (`/opt/ros/humble/setup.bash`)
- `ros-humble-joy`(手柄驱动)
- `libpcap-dev`(SOEM 原始以太网)
- 编译器:GCC ≥ 9 (C++17)
- 硬件:Logitech F710 / 兼容手柄,LinkX-4C CAN 桥设备,一张专用 EtherCAT 网卡

---

## 构建

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> 首次运行 `start_upper_computer.sh` 时会自动 `colcon build`。

---

## 网卡权限

SOEM 需要 raw 以太网权限,二选一(均假设当前目录在 `ros2_ws/`):

```bash
# 方案 A:每次 sudo(默认脚本走这条)
sudo ./start_upper_computer.sh

# 方案 B:给可执行文件一次性赋能力,之后无需 sudo
sudo setcap cap_net_raw,cap_net_admin+ep \
    install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo
./start_upper_computer.sh --no-sudo
```

---

## 启动

### 一键脚本

```bash
cd ros2_ws

# 默认:网卡 enp86s0,sudo 启动,max_speed=1.5
./start_upper_computer.sh

# 指定网卡
./start_upper_computer.sh --ifname eth0

# 仅启动 ROS 话题层,不启动 EtherCAT 主控(适合远程调试 / 单测)
./start_upper_computer.sh --no-vehicle

# 同时启动云台话题转发
./start_upper_computer.sh --gimbal

# 限速
./start_upper_computer.sh --max-speed 1.0
```

### 直接 launch

```bash
ros2 launch linkx_bringup full_system.launch.py \
    ifname:=enp86s0 \
    max_speed:=1.5 \
    start_vehicle_control:=true \
    start_gimbal_bridge:=false
```

启动后的节点拓扑:

```
joy_node ──► /joy ──► remote_node ──► /cmd_vel ─────┐
                                  └─► /robot_buttons┤
                                                     ▼
                                          chassis_relay ──► /chassis/cmd_vel
                                                        └─► /chassis/buttons
                                                                │
                                                                ▼
                                                  vehicle_control(EtherCAT 主控)
```

---

## 可执行清单

`linkx_soem_demo` 包构建以下可执行(`install/linkx_soem_demo/lib/linkx_soem_demo/` 下):

| 可执行 | 用途 |
| --- | --- |
| `linkx_soem_demo` | **整车主控**:SOEM 主站 + LinkX-4C CAN 桥 + chassis/gantry/arm/navigation 调度 |
| `remote_node_cpp` | 手柄解算:`/joy` → `/cmd_vel` + `/robot_buttons` |
| `stm32_node_cpp` | 通用话题转发(参数化输入/输出话题,可复用做 chassis_relay / gimbal_relay) |
| `linkx_set_alias` | LinkX EEPROM Station Alias 写入工具(脱离物理串接顺序) |
| `can_link_test` | 4 通道经典 CAN 1Mbps 链路冒烟测试 |
| `motor_calib` | DM6225 / DM3519 电机参数标定(动/静摩擦、惯量) |
| `steer_tuning` | 舵向 PID 网格自动扫参,输出 `var_data/steer_tuning_results.csv` |
| `robot_test` | 整车回归(Init / EtherCAT / TIM / ROS 桥 / 限速短转 / Gantry / Arm / Suction) |

---

## 常用工具用法

### 写入 LinkX Station Alias(R2 建议 `alias=2`)
```bash
sudo ./linkx_set_alias enp86s0 show
sudo ./linkx_set_alias enp86s0 set <slave_idx> <alias>
# ⚠ 写完必须给 LinkX 断电再上电,aliasadr 才生效
```

### CAN 链路冒烟
```bash
sudo ./can_link_test enp86s0
```

### DM 电机标定
```bash
# 舵向 DM6225:不需要架空,但解除负载/机械臂归位
sudo IFNAME=enp86s0 ./motor_calib --motor steer --wheel 0 --test all

# 轮向 DM3519:必须把目标轮架空
sudo IFNAME=enp86s0 ./motor_calib --motor wheel --wheel 0 --test all
```

### 舵向 PID 扫参
```bash
sudo ./steer_tuning enp86s0
# 进入交互后按 'a' 进入 AUTO_SWEEP,或 export TUNE_AUTO=1
```

### 整车回归
```bash
sudo ./robot_test enp86s0
```

---

## 话题约定

| 话题 | 类型 | 方向 |
| --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | `joy_node` 发布 |
| `/cmd_vel` | `geometry_msgs/Twist` | `remote_node` 发布,任意上层节点也可发布 |
| `/robot_buttons` | `std_msgs/...` | `remote_node` 发布 |
| `/chassis/cmd_vel` | `geometry_msgs/Twist` | `chassis_relay` 转发,主控订阅 |
| `/chassis/buttons` | 同上 | 同上 |
| `/gimbal/cmd_vel`、`/gimbal/buttons` | 同上 | 可选(`--gimbal`) |

> 任何外部规划/导航节点可以直接发布 `/cmd_vel`(与手柄共用入口),或绕过手柄直接发布 `/chassis/cmd_vel`。

---

## 配置入口

- 默认网卡:`start_upper_computer.sh` 中 `IFNAME=enp86s0`,或 `--ifname` / 环境变量 `IFNAME` 覆盖
- 手柄死区 / 自动重复率:`full_system.launch.py` 中 `joy_node` 参数
- 最大线速度:`remote_node` 的 `max_speed` 参数,或 `--max-speed`
- FastRTPS profile:`src/linkx_bringup/config/fastrtps_profiles.xml`

---

## 排错速查

| 现象 | 排查方向 |
| --- | --- |
| 启动报 `Failed to open interface` | 网卡名错 / 被其他进程占用 / 没有 raw 权限(`sudo` 或 `setcap`) |
| `slave count = 0` | LinkX 没上电、网线串接顺序异常、或 alias 未写入 |
| 手柄不动 | `ros2 topic echo /joy` 看 `joy_node` 是否在发,确认 F710 拨到 `D` 档 |
| 主控启动但底盘无响应 | `ros2 topic echo /chassis/cmd_vel`,检查 `chassis_relay` 是否在转发 |
| DM 电机抖动 / 过冲 | 用 `steer_tuning` 重新扫参,先 `motor_calib` 标定摩擦/惯量 |

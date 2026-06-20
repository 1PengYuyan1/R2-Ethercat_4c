# tof — 激光位移传感器 ROS 2 驱动包

通过 **EtherCAT → DM-LinKX-4c → CAN** 链路接入两类传感器，
统一发布 `sensor_msgs/Range` 话题。

| 类型 | 数量 | 区分方式 | DM-LinKX-4c 从站 |
|------|------|----------|-----------------|
| 高频（TFmini-S） | 8 路 | 每路独立 CAN ID | 从站 1 |
| 低频 | 4 路 | 共用一个 CAN ID，数据帧 Byte0 为设备地址 | 从站 2 |

---

## 硬件拓扑

```
上位机 (enp86s0)
  └── EtherCAT
        ├── DM-LinKX-4c #1（slave_id=1）  ← 高频传感器
        │     ├── CAN CH3
        │     │     ├── TFmini-S  CAN_ID=0x04  /high/front/range
        │     │     ├── TFmini-S  CAN_ID=0x03  /high/left/range
        │     │     ├── TFmini-S  CAN_ID=0x02  /high/right/range
        │     │     └── TFmini-S  CAN_ID=0x01  /high/back/range
        │     └── CAN CH2
        │           ├── TFmini-S  CAN_ID=0x01  /high/up_front/range
        │           ├── TFmini-S  CAN_ID=0x02  /high/up_back/range
        │           ├── TFmini-S  CAN_ID=0x03  /high/down_front/range
        │           └── TFmini-S  CAN_ID=0x04  /high/down_back/range
        └── DM-LinKX-4c #2（slave_id=2）  ← 低频传感器
              └── CAN CH3  （所有传感器共用 CAN_ID=0x05）
                    ├── 低频传感器  地址=0x04  /low/front/range
                    ├── 低频传感器  地址=0x03  /low/left/range
                    ├── 低频传感器  地址=0x02  /low/right/range
                    └── 低频传感器  地址=0x01  /low/back/range
```

> 若只有一台 DM-LinKX-4c，将低频从站接在不同 CAN 通道，
> 并修改 `can_link_test.cpp` 中 `slave_low` 为同一从站 ID。

---

## 依赖

| 依赖 | 说明 |
|------|------|
| `rclcpp` | ROS 2 C++ 客户端库 |
| `sensor_msgs` | `sensor_msgs/msg/Range` |
| `linkx_soem_demo` | 当前 Ethercat-R2 工作区内的 SOEM + LinkX-4C 源码 |

---

## 编译

```bash
cd /home/rc/R2/Ethercat-R2
colcon build --packages-select tof
source install/setup.bash
```

---

## 运行

```bash
# 使用 launch 文件（推荐）
ros2 launch tof radar_launch.py

# 默认会启动四路上/下高频 Range 订阅节点，可关闭
ros2 launch tof radar_launch.py start_up_range_subscriber:=false

# 指定网卡 / 波特率 / 从站 ID；slave_low:=0 表示不启用低频从站
ros2 launch tof radar_launch.py ifname:=enp3s0 can_baud:=500k slave_high:=1 slave_low:=2

# 高频 TFmini-S 透明传输链路走 CAN-FD 1M/5M
ros2 launch tof radar_launch.py ifname:=enp4s0 can_baud:=1m-5m slave_high:=1 slave_low:=0

# 直接运行（4 个可选参数：网卡  波特率  slave_high  slave_low）
ros2 run tof can_link_test enp86s0 1M 1 0

# 直接运行 CAN-FD 1M/5M
ros2 run tof can_link_test enp4s0 1m-5m 1 0

# 只启动四路上/下高频测距订阅节点
ros2 run tof up_range_subscriber
```

> **注意**：需要 root 权限或已配置网卡的 `CAP_NET_RAW` 权限。
> 每次重新 `colcon build` 后，`install/tof/lib/tof/can_link_test` 可能被覆盖，需要重新执行：
>
> ```bash
> sudo setcap cap_net_raw,cap_net_admin=eip /home/rc/R2/Ethercat-R2/install/tof/lib/tof/can_link_test
> getcap /home/rc/R2/Ethercat-R2/install/tof/lib/tof/can_link_test
> ```

---

## 发布的话题

### 高频传感器（TFmini-S，slave 1）

| 话题 | 通道 | CAN ID | 位置 |
|------|------|--------|------|
| `/high/front/range`      | CH3 | 0x04 | 前 |
| `/high/left/range`       | CH3 | 0x03 | 左 |
| `/high/right/range`      | CH3 | 0x02 | 右 |
| `/high/back/range`       | CH3 | 0x01 | 后 |
| `/high/up_front/range`   | CH2 | 0x01 | 上前 |
| `/high/up_back/range`    | CH2 | 0x02 | 上后 |
| `/high/down_front/range` | CH2 | 0x03 | 下前 |
| `/high/down_back/range`  | CH2 | 0x04 | 下后 |

Range 字段：`radiation_type=INFRARED`，`fov=0.0349 rad`，`min=0.1 m`，`max=12.0 m`

### 低频传感器（slave 2）

| 话题 | 共用 CAN ID | 地址字节 (Byte0) | 位置 |
|------|------------|-----------------|------|
| `/low/front/range` | 0x05 | 0x04 | 前 |
| `/low/left/range`  | 0x05 | 0x03 | 左 |
| `/low/right/range` | 0x05 | 0x02 | 右 |
| `/low/back/range`  | 0x05 | 0x01 | 后 |

Range 字段：`radiation_type=INFRARED`，`fov=0.0 rad`，`min=0.05 m`，`max=3.0 m`

---

## 低频传感器 CAN 帧格式

```
Byte 0 : 设备地址（0x01-0x04）
Byte 1 : 距离低字节（mm，小端 uint16）
Byte 2 : 距离高字节
Byte 3 : 状态 / 保留（可选）
```

> **按实际传感器手册修改** `dvc_low_freq.cpp` 中 `OnFrame()` 的解析逻辑，
> 以及 `ConfigMaxRate()` 中的帧率配置命令。

### 帧率最大化

`ConfigMaxRate()` 在节点启动时向每个传感器发送"设置最高输出频率"命令。
目前为占位符注释，填写方法：

```cpp
// dvc_low_freq.cpp → Dvc_LowFreq_Array::ConfigMaxRate()
const uint8_t hz_l = 100 & 0xFF;
const uint8_t hz_h = (100 >> 8) & 0xFF;
const uint8_t cmd[] = {0x06, hz_l, hz_h};   // 命令字按手册替换
SendCommand(addr, cmd, sizeof(cmd));
```

EtherCAT 主循环本身**无 sleep、全速轮询**，帧率上限由传感器硬件决定。

---

## 高频传感器 TFmini-S 数据帧格式

```
Byte:  0     1     2     3     4     5     6     7     8
       0x59  0x59  DL    DH    SL    SH    TL    TH    SUM
```

| 字段 | 说明 |
|------|------|
| `DH:DL` | 距离 cm；0=太近，65532=太远 |
| `SH:SL` | 信号强度；< 100 或 = 65535 表示无效 |
| `TH:TL` | 温度原始值，实际温度 = raw/8 − 256 (°C) |
| `SUM`   | 前 8 字节之和的低 8 位 |

传感器帧率在 `Init()` 时通过命令配置为 **1000 Hz**。

---

## 源文件说明

```
src/
├── can_link_test.cpp   主程序：EtherCAT 初始化、双从站管理、主循环
├── dvc_tfmini_s.cpp    高频传感器驱动：TFmini-S 协议解析、ROS 2 发布
├── dvc_low_freq.cpp    低频传感器驱动：地址路由、协议解析、ROS 2 发布
└── up_range_subscriber.cpp 订阅 /high/up_front、/high/up_back、/high/down_front、/high/down_back
```

### can_link_test.cpp — 主循环架构

- 双线程：**EtherCAT 主循环**（全速，无 sleep）+ **ROS 2 spin 线程**
- 两个 `linkx_t` 实例分别对应高频从站 / 低频从站
- 单次 `ecat_master_sync` 覆盖 EtherCAT 总线上所有从站
- 每 1 秒调用 `Tick_Alive_Check` 更新在线状态
- 每 5 秒打印各通道帧计数统计

### dvc_tfmini_s.cpp — 高频驱动

- 每个传感器独立 CAN ID，在 `CAN_RxCpltCallback` 中按 `msg.id` 路由
- 9 字节帧逐字节解析 + 校验和验证
- 启动时配置传感器输出帧率为 1000 Hz

### dvc_low_freq.cpp — 低频驱动

- 4 个传感器共用 `LOW_DATA_CAN_ID`（默认 `0x05`），按 `Byte0` 地址路由
- O(1) 地址查表（`unordered_map`），零额外循环开销
- 启动时调用 `ConfigMaxRate()` 配置最高输出频率（需按手册填写命令）

---

## 在线状态检测

`Tick_Alive_Check` / `Tick_Alive_Check_Low` 每秒被主循环调用，
对比前后帧计数：有变化 = 在线，无变化 = 离线。

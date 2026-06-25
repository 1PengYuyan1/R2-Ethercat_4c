# 全向轮底盘电机参数 & IMU 纠偏 —— 全量重测大纲

> 适用对象：`src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.cpp`
> 配套头文件：`include/linkx_soem_demo/.../omni_wheel/crt_chassis_omni.h`
> IMU 纠偏：`src/vehicle_control/3_Chariot/imu_heading_hold/imu_heading_hold.cpp`
> 编制日期：2026-06-25
> 目标：从零重测全部电机参数与底盘 / IMU 配合，**反复大量测试**，直到跑出一版直线良好、起步不偏的效果。

---

## 0. 本次重测要解决的现象（必须复现 → 量化 → 消除）

遥控操控时实测到的问题，全部要在测试中量化并逐项归零：

1. **底盘偏得很离谱，完全走不了直线**。
2. **车尾两个电机比车头转得快、响应也比车头快**。
3. **左右平移时，总是车头先动**，破坏整车姿态。
4. **前后移动也是，起步阶段总会偏离**。

### 0.1 现象与现有参数的对应关系（测试时重点验证的假设）

轮位映射（俯视图，新车头 +X，左 +Y；index → CAN → 方位角）：

| index | CAN/ID | 位置 | 方位角 θ | 当前 kd | 当前 correction | 当前 breakaway | 车头/车尾 |
|------|--------|------|---------|--------|----------------|---------------|----------|
| 0 | CAN1 ID2 | 右后 | 225° | **3.5** | 0.849 | 0.85 | 车尾 |
| 1 | CAN0 ID2 | 右前 | 315° | **1.2** | 1.275 | 1.10 | 车头 |
| 2 | CAN0 ID1 | 左前 | 45°  | **0.5** | 1.147 | 0.45 | 车头 |
| 3 | CAN1 ID1 | 左后 | 135° | **3.5** | 0.871 | 0.60 | 车尾 |

**需要在测试中证实/证伪的根因假设：**

- **H1（响应快慢不对称）**：车尾 kd=3.5，车头 kd=0.5/1.2。Kd 越大速度跟踪越紧、响应越快 → 直接解释"车尾比车头转得快、反应快"。重测要把同侧前后轮的速度阶跃响应（上升时间、超调）调到一致。
- **H2（稳态速度不对称）**：`wheel_speed_correction` 跨度 0.849~1.275（≈50%），远超注释建议的 0.90~1.10。correction 直接乘在目标轮速上，左右/前后不一致就会走斜线。
- **H3（起步先偏）**：`wheel_breakaway_torque` 四轮 0.45~1.10 各不相同，起步瞬间四轮"破静摩擦冲量"不一致 → 必然某个轮先动、整车姿态被先破坏。`wheel_stiction_torque`/`wheel_dynamic_friction`/`wheel_rotor_inertia`/`wheel_feedforward_scale` 也四轮各异，叠加放大起步偏移。
- **H4（IMU 纠偏补不回来）**：起步偏移发生在 IMU 锁航向之前/纠偏带宽之外，`out_limit_scale=0.6`、`kp=3.5` 可能不足以在 5m 行程内拉回。

> 测试策略：**先把机械/电机层面做到对称（H1~H3），再用 IMU（H4）做最后的闭环兜底**。不要指望 IMU 掩盖电机不对称。

---

## 1. 被测参数清单（全部要重新标定/验证）

### 1.1 单轮电机参数 `wheel_params_[i]`（`Init_Motor_Params()`）

| 参数 | 含义 | 当前值范围 | 重测关注点 |
|------|------|-----------|-----------|
| `wheel_kp` | MIT 位置环 Kp | 全 0 | 保持 0（速度模式），仅确认 |
| `wheel_kd` | MIT 速度环 Kd / 粘性阻尼 | 0.5 / 1.2 / 3.5 / 3.5 | **重点**：四轮响应一致性，消除车头/车尾快慢差 |
| `wheel_direction` | 轮向符号 ±1 | 全 +1 | 先验证方向正确 |
| `wheel_speed_correction` | 直行偏转微调 | 0.849~1.275 | **重点**：收敛到对称、接近 1.0 |
| `wheel_omega_deadzone` | 轮速死区 | 全 0.05 | 验证不漏触发/不抖动 |
| `wheel_stiction_torque` | 静摩擦补偿 | 0.13~0.48 | 起步力矩标定 |
| `wheel_dynamic_friction` | 动摩擦补偿 | 0.28~0.43 | 中低速力矩标定 |
| `wheel_rotor_inertia` | 等效转动惯量 | 0.0114~0.0176 | 惯量前馈标定 |
| `wheel_feedforward_scale` | 前馈比例 | 0.547~1.109 | 与摩擦/惯量联动 |
| `wheel_breakaway_torque` | 起步附加力矩 | 0.45~1.10 | **重点**：四轮起步冲量一致 |
| `wheel_accel_limit` | 轮速梯形加速上限 | 400 rad/s² | 与底盘加速度配合 |
| `wheel_decel_limit` | 轮速梯形减速上限 | 650 rad/s² | 与底盘减速配合 |

### 1.2 底盘级常量（`crt_chassis_omni.h`）

| 宏 | 当前值 | 重测关注点 |
|----|-------|-----------|
| `MAX_OMNI_CHASSIS_SPEED` | 5.0 m/s | 本次测试速度区间上限参考 |
| `MAX_OMNI_CHASSIS_OMEGA` | 10.0 rad/s | — |
| `OMNI_CHASSIS_LINEAR_ACCEL_LIMIT_M_S2` | 8.0 | **重点**：起步加速度 vs 起步偏移 |
| `OMNI_CHASSIS_LINEAR_DECEL_LIMIT_M_S2` | 12.0 | 停车/换向跟手 |
| `OMNI_CHASSIS_ANG_ACCEL/DECEL_LIMIT` | 24 / 32 | 平移阶段角速度纠偏裕度 |
| `OMNI_WHEEL_ACCEL_LIMIT_RAD_S2` | 400 | **重点**：四轮同步加速 |
| `OMNI_WHEEL_DECEL_LIMIT_RAD_S2` | 650 | — |
| `OMNI_WHEEL_ACCEL_FILTER_ALPHA` | 0.25 | 惯量前馈平滑 |
| `OMNI_WHEEL_TORQUE_FF_LIMIT_NM` | 1.5 | 前馈饱和上限 |
| `OMNI_WHEEL_RELIABLE_OMEGA_LIMIT` | 80 | 归一化轮速上限 |
| `OMNI_WHEEL_BREAKAWAY_OMEGA_RAD_S` | 2.0 | 起步力矩作用窗口 |
| `OMNI_WHEEL_BREAKAWAY_RATIO` | 0.90 | 跟踪不足触发阈 |
| `Wheel_Radius` / `Wheel_To_Core_Distance` | 0.0761 / 0.24621 | 机械量，先核实 |

### 1.3 IMU 纠偏参数（`imu_heading_hold.h::Config`，ROS 参数前缀 `imu_heading_hold.`）

| 参数 | 默认 | 重测关注点 |
|------|------|-----------|
| `enable` | true | 分两组：关闭 / 开启 对比 |
| `topic` | `/IMU_data` | 仅启动时读，确认有数据 |
| `timeout_ms` | 100 | IMU 失活保护 |
| `turn_eps` | 0.05 | 判定"用户在主动转向"阈 |
| `move_eps` | 0.02 | 判定"在平移"阈 |
| `kp` | 3.5 | **重点**：纠偏带宽 |
| `ki` | 0.0 | 稳态残差，谨慎引入 |
| `kd` | 0.03 | 抑制超调 |
| `kf` | 0.0 | — |
| `i_out_max` | 0.0 | 积分限幅 |
| `out_limit_scale` | 0.6 | **重点**：纠偏角速度上限占比 |
| `dt` | 0.001 | — |
| `dead_zone` | 0.01 | 航向死区 |

---

## 2. 测试前提与环境

- **场地**：≥ 6m 直线净空（5m 行程 + 起停余量），地面平整一致；地面贴 5m 直线基准带 + 起点/终点线，沿线每 1m 标尺。
- **安全**：急停可达；前几轮测试在低速档先确认方向与急停。重物/人员避让。
- **网卡 / 从站**：确认 `IFNAME`（示例 `enp4s0`/`enp86s0`，按本机为准）与电机所在 EtherCAT 从站号 `R2_MOTOR_SLAVE`（默认 2）。
- **上电核对**：4 个 DM3519 上电打印应为 PMAX=12.5、VMAX=395、TMAX=5、IMAX=12，与 `Init()` 传参一致；任一不符先停止排查接线/ID。
- **IMU 核对**：`ros2 topic hz /IMU_data` 有稳定频率，`Get_Snapshot().fresh==true`；静置时 yaw 漂移在可接受范围。

### 2.1 可复用的现有测试可执行程序

| 程序 | 用途 | 关键可调命令行 |
|------|------|---------------|
| `omni_accel_response` | 单轮/同步**速度阶跃响应**，量化上升时间、超调、跟踪 | `--speeds`、`--single 1`、`--sync 1`、`--kds`、`--kd`、`--ff-scales`、`--profile 1` |
| `omni_motion_record` | +X/-X/+Y/-Y **直线/偏移测量**，逐轮实测/目标速度比 | `--speed`、`--duration`、`--directions`、`--kds`、`--corrections`、`--ff-scales`、`--wheel-directions`、`--breakaway-scale` |
| `omni_straight_line_autotune` | 直线**自动调参**（ROS 链路） | 见程序内 `--help` |
| `omni_accel_response` (`--profile 1`) | 验证轮速梯形 profile 加速度配合 | `--profile-accel`、`--profile-decel` |
| `omni_chassis_full_retest` | 多阶段重测编排，记录到 `var_data/omni/` | `--phases`、`--speed-list`、`--distance-m`、`--repeats`、`--execute` |

> 这些程序支持用命令行覆盖 `kd / correction / ff-scale / breakaway` 等，**先用命令行扫参找到好值，再回填到 `Init_Motor_Params()` 源码**，避免反复改源码重编译。所有 CSV/summary 默认落在 `var_data/omni/`。

---

## 3. 测试方法论（贯穿全程）

1. **反复、大量**：每个工况 ≥ 8~10 次重复；记录均值 + 最大值 + 标准差，不取单次"运气好"的结果。
2. **单变量**：一次只动一个参数（或一组对称参数），记录前后对比。
3. **先对称、再闭环**：先在架空/低速把四轮做对称（开环可直），再开 IMU 闭环兜底。
4. **量化优先于主观**：偏移用"终点横向偏差 / 行程"（%）、初始 1m 偏航角、yaw 曲线峰峰值衡量，禁止只凭手感。
5. **每阶段设"准入门槛"**：达不到门槛不进入下一阶段（见各阶段验收标准）。
6. **全程留痕**：每次跑保存 CSV/summary + 参数快照 + 一句话结论，命名含时间戳与参数标签。

### 3.1 统一评价指标

| 指标 | 定义 | 目标 |
|------|------|------|
| 终点横向偏差率 e_lat | 终点偏离基准线横向距离 / 5m | 见各阶段 |
| 起步偏航 Δψ₀ | 起步前 1m 内 IMU yaw 最大偏离 | 越小越好，逐步收紧 |
| 四轮响应一致性 | `omni_accel_response` 各轮上升时间/超调极差 | 极差 ≤ 10% |
| 实测/目标轮速比 | `omni_motion_record` 各轮 actual/target | 四轮接近、≈1.0 |
| yaw 峰峰值 | 全程 IMU yaw 峰峰 | 逐阶段收紧 |

---

## 4. 测试阶段（按依赖顺序执行；先前后移动）

### 阶段 A —— 基线复现与机械核实（开环）

目的：量化"坏"的起点，确认方向/机械量无误。

- A1 核实 `wheel_direction` 四轮符号：低速 +X 指令，确认四轮转向与车体前进一致（必要时用 `omni_motion_record --wheel-directions` 验证）。
- A2 核实 `Wheel_Radius`、`Wheel_To_Core_Distance` 与实物。
- A3 **基线偏移测量**：用当前源码参数，遥控 + `omni_motion_record`，跑 +X/-X 各 ≥10 次，速度 0.5、行程 5m，记录 e_lat、Δψ₀、四轮 actual/target 比。→ 形成"问题基线"数据。

**准入门槛**：方向全部正确；基线数据完整存档。

### 阶段 B —— 单轮/同步速度阶跃响应一致性（解决 H1：车尾比车头快）★核心

目的：让四轮（尤其同侧前后轮）速度响应一致，消除"车尾转得快、反应快"。

- B1 用 `omni_accel_response --single 1 --speeds 10,30,60`，逐轮测上升时间、超调、稳态误差。量化当前 kd(0.5/1.2/3.5/3.5) 造成的差异。
- B2 扫 `--kds a,b,c,d`：以"四轮上升时间/超调极差最小"为目标收敛 kd。**重点把车头(idx1/idx2)与车尾(idx0/idx3)的响应对齐**（很可能需要把车尾 kd 降下来或把车头 kd 提上去）。
- B3 `omni_accel_response --sync 1` 验证四轮同步阶跃：四轮同时到达、无某轮抢先。
- B4 反复 ≥10 组，确定一组对称 kd，回填源码。

**验收**：四轮上升时间极差 ≤ 10%，同步阶跃无明显抢先轮。

### 阶段 C —— 稳态轮速一致性 / correction 标定（解决 H2：走斜线）★核心

目的：消除稳态阶段的左右/前后速度不对称。

- C1 `omni_motion_record --speed 0.5 --directions x+,x-`，读各轮 actual/target 比。
- C2 调 `--corrections`：使四轮实测线速度一致、整车直行。把 0.849~1.275 的大跨度收敛到尽量接近、对称（理想 0.90~1.10）。
- C3 +Y/-Y 同样校一遍（`--directions y+,y-`），确认左右对称。
- C4 反复 ≥10 次，回填 `wheel_speed_correction`。

**验收**：开环（IMU 关）下 +X/-X 终点横向偏差率 e_lat ≤ 2%。

### 阶段 D —— 起步力矩/摩擦/惯量标定（解决 H3：起步先偏）★核心

目的：四轮起步冲量一致，整车起步不甩头。

- D1 `omni_motion_record`，看 `--startup-window`（默认 0.6s）内各轮起步先后与 Δψ₀。
- D2 标定 `wheel_stiction_torque` / `wheel_dynamic_friction`：从能稳定起步的最小值起调，避免过补偿冲。
- D3 调 `wheel_breakaway_torque`（当前 0.45~1.10 差异过大）使四轮"破静摩擦"同步；用 `--breakaway-scale` 整体缩放后再逐轮微调。
- D4 标定 `wheel_rotor_inertia` / `wheel_feedforward_scale`（惯量前馈），减小加速段跟踪误差。
- D5 反复 ≥10 次起步测试，量化 Δψ₀ 收敛。

**验收**：开环下起步 1m 内 Δψ₀ 显著下降、四轮起步先后 < 一个采样窗内可分辨差。

### 阶段 E —— 前后直线主测（速度 ≥ 0.5、行程 5m、反复大量）★首要工况

> **按用户要求：先测前后移动，速度设在 0.5 以上，行程 5m，反复大量测试。**

- E1 速度档：0.5 / 0.8 / 1.0 / 1.5 m/s（≥0.5 起，逐级加），每档 +X、-X 各 ≥10 次。
- E2 每次记录 e_lat、Δψ₀、yaw 峰峰、四轮 actual/target。
- E3 先 **IMU 关**（纯开环）跑，确认 A~D 的标定让开环已基本直；再 **IMU 开** 对比。
- E4 出现偏移立刻回到 B/C/D 对应参数微调，再回 E 复测（迭代闭环）。

**验收**：各速度档（IMU 开）e_lat ≤ 1%、Δψ₀ 小且一致，10 次重复标准差小。

### 阶段 F —— 加速度与底盘配合（与电机配合也要测）★

目的：标定底盘级与轮级加减速限幅的配合，保证加速段不偏、跟手。

- F1 `omni_accel_response --profile 1 --profile-accel A --profile-decel D` 验证轮速 profile 下四轮同步加速（对应源码 `Apply_Wheel_Trapezoid_Profile` 用同一 progress 比例）。
- F2 扫 `OMNI_CHASSIS_LINEAR_ACCEL_LIMIT_M_S2`（基线 8.0）：在 5m 行程内做"急起步/急停"，观察加速段 Δψ₀ 与停车跟手；过大→起步甩头，过小→不跟手。
- F3 配 `OMNI_WHEEL_ACCEL_LIMIT_RAD_S2`(400) / `OMNI_WHEEL_DECEL_LIMIT_RAD_S2`(650) / `OMNI_WHEEL_ACCEL_FILTER_ALPHA`(0.25)，使底盘加速度与轮级加速度不打架（底盘加速被轮级限幅"吃掉"会导致比例失真先偏）。
- F4 急起步 / 急停 / 急换向（前→后）各 ≥10 次。

**验收**：加速/减速/换向段无可见甩头，加速段 Δψ₀ 与匀速段同量级。

### 阶段 G —— 左右平移（解决"左右移动车头先动"）★

- G1 重复 C/D/E 思路于 +Y/-Y：速度 ≥0.5、5m、各 ≥10 次。
- G2 重点观察起步是否仍"车头先动"；若有，回 D（起步力矩对称）+ B（响应对称）继续调。

**验收**：+Y/-Y e_lat ≤ 1%，起步不甩头。

### 阶段 H —— IMU 纠偏闭环整定（解决 H4：兜底拉回）

> 注意逻辑：`Correct_Omega` 仅在 `enable && imu_fresh && !lift_diff_mode && 用户未主动转向(|omega_cmd|<turn_eps) && 正在平移` 时接管，首次平移锁存当前 yaw 为目标。

- H1 确认锁航向时机：平移瞬间 `heading_locked_` 置位、目标=当前 yaw；`move_eps`(0.02)/`turn_eps`(0.05) 是否合适（太大起步段不纠偏 → 与"起步先偏"叠加）。
- H2 整定 `kp`(3.5) / `kd`(0.03) / `out_limit_scale`(0.6)：在 5m 行程内能把残余偏移拉回且不振荡。可用 `omni_straight_line_autotune` 或运行时 `ros2 param set`。
- H3 谨慎引入 `ki`/`i_out_max` 消稳态残差（先 0，确有残差再加）。
- H4 验证 IMU 失活/超时（`timeout_ms`）保护：拔 IMU 话题，确认退回不发散。
- H5 IMU 开 vs 关，在 0.5/0.8/1.0 各档对比，确认 IMU 是"锦上添花"而非"遮丑"。

**验收**：IMU 开后 yaw 峰峰、e_lat 较开环再降一档且无振荡；失活保护正确。

### 阶段 I —— 遥控综合复跑（最终确认）

- I1 用遥控器复刻最初问题场景：前后、左右、加减速、换向连续操控，≥30 分钟累计。
- I2 主观跟手 + 客观 e_lat/Δψ₀/yaw 全部达标，且 10+ 次重复稳定。
- I3 固化"好的一版"参数到源码，打标签存档（参数快照 + 数据 + 结论）。

**最终验收（达成即结束迭代）**：
- 前后/左右 5m、速度 ≥0.5，e_lat ≤ 1%；
- 起步不甩头（车头/车尾无明显先后），Δψ₀ 收敛且一致；
- 四轮响应一致性极差 ≤ 10%；
- 加速/减速/换向无甩头、跟手；
- 多次重复（≥10）标准差小，遥控主观直、稳。

---

## 5. 迭代回路（直到测出好效果）

```
A 基线 ─► B 响应对称 ─► C 稳态对称 ─► D 起步对称 ─► E 前后直线(主) ─► F 加速度配合 ─► G 左右 ─► H IMU 闭环 ─► I 遥控综合
   ▲                                                        │
   └──────── 任一阶段未达门槛 → 回对应阶段微调，重测 ◄────────┘
```

- 每轮迭代只改一组参数，跑满重复次数，对比指标，决定保留/回退。
- 把每次"参数 → 指标"记成一行表格，形成可追溯的调参轨迹。
- 命令行扫参定好值后，统一回填 `Init_Motor_Params()` 与 `crt_chassis_omni.h` 宏 / IMU 默认 Config，再整机复测一遍确认源码与扫参一致。

---

## 6. 数据记录与可分析性

### 6.1 现有程序"自动产出 → 可直接分析"的字段（已具备）

下列数据由现有可执行程序自动写入 `var_data/omni/`，可直接喂入分析，无需手填：

**`omni_accel_response`**（→ 支撑阶段 B 响应一致性、F 加速）

- 每轮每档：`reach_time`（上升/到达时间）、`overshoot_pct`（超调）、是否 `reached`、稳态误差。
- 直接算："四轮响应一致性极差" = max/min reach_time、overshoot 极差。

**`omni_motion_record`** CSV 逐采样列（→ 支撑阶段 C 稳态、D 起步力矩、F 加速度配合）：

```
segment, t_s, cmd_vx, cmd_vy, cmd_omega,
profiled_vx, profiled_vy, profiled_omega, odom_vx, odom_vy, odom_omega,
<wheel>_ideal_omega, _raw_omega, _profile_omega, _actual_omega,
_raw_error, _profile_error, _cmd_accel, _accel_filtered, _measured_accel,
_cmd_torque, _actual_torque, _status, _ctrl_status      (× 4 轮)
```

summary 另给每轮 **actual/target ratio（mean ± stddev）**。直接算：

- 阶段 C 稳态对称：四轮 `actual/target ratio` 是否一致、是否≈1.0。
- 阶段 D 起步：`--startup-window` 内 `_actual_omega`/`_actual_torque` 的逐轮起步先后。
- 阶段 F：`_cmd_accel`/`_accel_filtered`/`_measured_accel` 对比，看底盘加速度是否被轮级限幅吃掉。

### 6.2 ⚠️ 缺口：整车真实偏移与 IMU 航向（需补采集，否则 E/G/H 无法直接分析）

现有 `omni_motion_record` **未订阅 `/IMU_data`，CSV 无 yaw 列**；其 `odom_*` 是**轮速积分里程计，全向轮打滑时不可信**，不能作为真实横向偏移。因此本大纲的两个核心整车指标缺数据源，必须按下面任一方式补齐：

| 指标 | 缺口 | 补采集方式（择一/并用） |
|------|------|----------------------|
| **e_lat 真实终点横向偏差** | odom 不可信、无真值 | ① 卷尺/激光：终点实测偏离基准线横向距离 ÷ 5m，手填本表；② 地面贴线 + 手机俯拍轨迹后量取 |
| **Δψ₀ 起步偏航 / yaw 峰峰** | CSV 无 IMU yaw | ① 测试期间 `ros2 bag record /IMU_data /chassis/cmd_vel`，事后从 bag 取 yaw 算 Δψ₀、峰峰；② （推荐）给 `omni_motion_record` 增订 `/IMU_data` 并把 yaw 写入 CSV，一次跑齐电机+航向数据 |

> 建议优先做"②给 omni_motion_record 增订 IMU 写 yaw 列"，这样 E/G/H 的真值与电机数据同 CSV、同时间轴，可直接做相关性分析（如起步 yaw 突变 vs 哪个轮先动）。需要的话我可以直接改这个程序加上 yaw 列。

### 6.3 手填记录模板（整车真值 + 结论，配合 6.1 的自动 CSV）

| 时间戳 | 阶段 | 工况(方向/速度) | 参数标签(kd/corr/breakaway/...) | IMU开关 | 重复次数 | e_lat 来源 | e_lat(均/最大/σ) | Δψ₀ 来源 | Δψ₀ | yaw 峰峰 | 四轮actual/target | CSV 文件名 | 结论 |
|--------|------|----------------|-------------------------------|---------|---------|-----------|-----------------|---------|------|---------|------------------|-----------|------|

> 每行务必填 `CSV 文件名` 与参数快照，建立"自动数据 ↔ 手填真值 ↔ 参数"三者可追溯链路；CSV/summary 路径统一 `var_data/omni/`。

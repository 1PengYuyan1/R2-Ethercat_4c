# omni_straight_retest 运行 & 分析手册（给 codex 用）

配套程序：`omni_straight_retest_main.cpp`（大纲阶段 E/G + IMU 真值）。
本手册让 codex（或人）能**直接编译、运行、解析 JSON 并给出调参建议**。

---

## 1. 这个程序产出什么（可直接分析）

单进程同时驱动 EtherCAT 电机 + 订阅 `/IMU_data`，按"前后 5m、速度≥0.5、反复 N 次"协议跑，输出三份文件到 `var_data/omni/`：

| 文件 | 内容 | 用途 |
|------|------|------|
| `omni_straight_retest_<ts>.json` | **机器可读**：每趟 + 聚合指标 | codex 直接 parse 分析 |
| `omni_straight_retest_<ts>.csv` | 逐样本（电机 + IMU yaw 同时间轴） | 画曲线 / 深挖 |
| `omni_straight_retest_<ts>.txt` | 人读摘要 | 快速浏览 |

每趟（一次 5m）自动算出的指标：

- `delta_psi0_deg`：起步前 1m 内 IMU yaw 最大偏离 →「起步甩头」程度。
- `yaw_pp_deg`：全程 yaw 峰峰。
- `yaw_final_dev_deg`：终点 yaw 相对起点偏离。
- `lateral_m` / `e_lat_pct`：IMU 航向估算的横向漂移 `∫ v·sin(Δyaw) dt`，及其相对 5m 的百分比（**不依赖打滑后不可信的里程计**）。
- `wheel_ratio[4]`：四轮 actual/target 稳态比 → 对称性。
- `reach_time_s[4]` / `reach_spread_s` / `fastest_wheel` / `slowest_wheel`：四轮到达目标轮速时间与极差 →「车尾比车头快」的量化。

> IMU 不在线时，电机数据照常记录，IMU 相关字段为 `null` 并打印 `[IMU MISSING]`。

---

## 2. 编译

```bash
cd /home/rcr2/R2/src/R2-Ethercat_4c
source /opt/ros/humble/setup.bash
colcon build --packages-select linkx_soem_demo --cmake-target omni_straight_retest
```

二进制：`install/linkx_soem_demo/lib/linkx_soem_demo/omni_straight_retest`

## 3. 运行（需 root 跑 EtherCAT；需 IMU 在发 /IMU_data）

```bash
cd /home/rcr2/R2/src/R2-Ethercat_4c
source install/setup.bash
# 确认 IMU 在线（另一终端）：
ros2 topic hz /IMU_data

# 正式跑：前后来回(x+去 x-回)，速度 0.5/0.8/1.0，每档重复 10 次，单趟 5m
sudo -E env "PATH=$PATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" \
  IFNAME=enp4s0 R2_MOTOR_SLAVE=2 \
  ./install/linkx_soem_demo/lib/linkx_soem_demo/omni_straight_retest \
  --speeds 0.5,0.8,1.0 --distance 5.0 --repeats 10 --directions x+,x-
```

常用参数：`--imu-topic`、`--first-meter`(默认1.0)、`--analysis-delay`、`--sample-hz`、
扫参覆盖 `--kds a,b,c,d` / `--corrections a,b,c,d` / `--ff-scales a,b,c,d` /
`--kd K` / `--wheel-directions a,b,c,d` / `--breakaway-scale S`、
**软件速度环 `--vel-kp K --vel-ki K --vel-i-limit NM`**、
`--torque-ff-limit NM`、
输出路径 `--csv/--summary/--json`。`--help` 查看全部。

### 3.1 ★当前首要任务：先用软件速度环把 wheel_ratio 拉到 ≈1

诊断已确认：MIT 用 Kp=0、速度环只有比例(Kd)+前馈、**无积分**，地面负载下稳态垂降
`ω_actual ≈ ω_target − (T_load−τ_ff)/Kd` → ratio≈0.5。已在 `crt_chassis_omni.cpp`
的 `Output_To_Motor()` 加了**逐轮软件速度 PI**（默认 kp=ki=0 关闭，不改原行为；起步未越过
breakaway 速度时冻结积分防饱和；带积分力矩限幅；停车清零）。用 `--vel-ki` 打开扫参：

```bash
# 架空先验证不发散、积分收敛 (看 CSV 的 *_vel_i 列是否稳定到某常值)
sudo -E ... omni_straight_retest --distance 0.5 --repeats 1 --speeds 0.5 \
  --vel-kp 0.0 --vel-ki 4.0 --vel-i-limit 1.5
# 落地扫 ki: 目标 wheel_ratio_mean 四轮都 → ≈1.0、std 小、不抖
#   ki 太小: ratio 仍<1; ki 太大: *_vel_i 抖/饱和到 ±i_limit、yaw 变差
# 起步可少量 kp 提响应(可选), 但主力是 ki。
```

判读：JSON `aggregate[].wheel_ratio_mean` 四轮都接近 1 即达标；CSV `*_vel_i` 应在
起步后平滑爬升并稳定（不顶满 ±i_limit、不振荡）。**ratio≈1 之前不要碰 correction，
不要跑 5m。** 达标后再回 §1 流程在 2m→5m 上对称重标 correction。

#### 给 codex 的 `--vel-ki` 扫参清单（按顺序执行，每步看完再下一步）

> 全程固定原始 correction（`0.849,1.275,1.147,0.871`）、kd 用 Kd=5（短扫里最好）、
> 不要改 ff/breakaway。只动 `--vel-ki`。每步跑完读对应 JSON 的 `wheel_ratio_mean`
> 与 CSV 的 `*_vel_i`，按"判定"决定下一步。命令前缀统一：
> `sudo -E env "PATH=$PATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" IFNAME=enp4s0 R2_MOTOR_SLAVE=2 ./install/linkx_soem_demo/lib/linkx_soem_demo/omni_straight_retest`

| 步 | 目的 | 命令尾参 | 判定 / 下一步 |
|----|------|---------|--------------|
| S0 | **架空**验不发散、积分收敛 | `--distance 0.5 --repeats 1 --speeds 0.5 --kd 5 --vel-ki 4 --vel-i-limit 1.5` | `*_vel_i` 平滑稳定→进 S1；若抖/顶满→ki 减半再试 |
| S1 | 落地 ki=2（保守） | `--distance 2.0 --repeats 2 --speeds 0.5 --kd 5 --corrections 0.849,1.275,1.147,0.871 --vel-ki 2` | 记 ratio。多半仍<1→进 S2 |
| S2 | 落地 ki=4 | 同上但 `--vel-ki 4` | ratio 更接近 1 且不抖→进 S3；已≈1→进 S4 |
| S3 | 落地 ki=8（激进） | 同上但 `--vel-ki 8` | 若 ratio→1 且 `*_vel_i` 不饱和不振荡→选它；若振荡/yaw 变差→回退到 S2 的 ki |
| S4 | 选定 ki 后**升速验证** | `--distance 2.0 --repeats 3 --speeds 0.5,0.8,1.0 --kd 5 --corrections 0.849,1.275,1.147,0.871 --vel-ki <选定>` | 各速度档 ratio 都≈1、std 小→进 §1 的 correction 重标 |
| S5 | i_limit 边界检查（可选） | 在选定 ki 上对比 `--vel-i-limit 1.0` vs `1.5` | 若常顶满 1.5→说明负载需要更大力矩，先查机械/饱和，别单纯放大 |

二分建议：ki 在 [2,8] 内按 2→4→8 找到"ratio≈1 且 `*_vel_i` 不饱和不振荡"的最大可用值，
再取其约 0.7~0.8 倍作为留裕度的工作值。选定后把 `wheel_ratio_mean`、`*_vel_i` 峰值、
`delta_psi0`、`e_lat` 记入调参轨迹表。

> 注意：速度环是**对称**机制，只消除整体垂降；**不会**抹平 W3 与其它轮的机械/负载差。
> 若 ratio≈1 后 W3 仍系统性不同，查 CSV 各轮 `_actual_torque`：W3 明显更低=负载轻(机械)，
> 这时才回到 correction 做对称微调，而不是反过来用 correction 掩盖。

> 架空联调先用小行程验证：`--distance 0.5 --repeats 2 --speeds 0.5`。
> 左右平移测试：`--directions y+,y-`。

#### W0_RB 已知反馈项

2026-06-25 人工数圈确认 W0 实际转速正常，但 `Get_Now_Omega()` 反馈约低报到
`0.78`。已在电机反馈出口对 W0 加 `OMNI_WHEEL_W0_FEEDBACK_OMEGA_SCALE=1.282`
（只修反馈、不动命令；固件根因查清后应回到 `1.0`）。架空 A/B 验证：
`wheel_ratio_mean` x+ = `[0.886,0.935,1.004,0.992]`，x- =
`[1.011,0.995,0.945,0.912]`，W0 `vel_i` 与 `cmd_torque` 已恢复正常。

注意：W0 `_actual_torque` 反馈仍与控制侧信号矛盾，疑似同一电机反馈标定问题。
后续不要用 W0 `_actual_torque` 判断机械负载；优先看 `wheel_ratio`、`*_vel_i`、
`*_cmd_torque`。其它轮的 `_actual_torque` 仍可作为辅助参考，但不能替代电流/电压遥测。

⚠️ 安全：需 ≥6m 净空走廊；`x+,x-` 会原地来回（去 5m、回 5m）。Ctrl+C 会先发 DM exit。

## 4. 验收阈值（来自大纲）

| 指标 | 目标 |
|------|------|
| `e_lat_pct`（聚合 mean/max） | ≤ 1% |
| `delta_psi0_deg` | 小且各趟一致（std 小） |
| `reach_spread_s` | 四轮响应一致，极差 ≤ 上升时间的 10% |
| `wheel_ratio_mean[4]` | 四轮接近、≈1.0 |
| 重复稳定性 | 各指标 std 小 |

---

## 5. 给 codex 的分析任务提示词（可直接粘贴）

> 读取最新的 `var_data/omni/omni_straight_retest_*.json`，对每个 `aggregate` 分组（speed×direction）：
> 1. 判定是否达标：`e_lat_pct.mean ≤ 1`、`reach_spread_s.max` 是否过大、`wheel_ratio_mean` 四轮是否对称(极差<10%)。
> 2. 定位根因：
>    - 若某 `slowest_wheel`/`fastest_wheel` 固定是车头(W1/W2)或车尾(W0/W3) → kd 不对称，建议把慢轮 kd 提高/快轮 kd 降低，给出新的 `--kds`。
>    - 若 `wheel_ratio_mean` 某轮偏离 1 → 调该轮 `--corrections`（ratio>1 说明实际超目标，应调小该轮 correction，反之调大）。
>    - 若 `delta_psi0_deg` 大且起步阶段 CSV 中某轮先冲 → 该轮 `breakaway/stiction` 偏大，建议降低或用 `--breakaway-scale`。
> 3. 给出下一轮**单变量**实验命令（只改一组参数），并预测期望指标变化。
> 4. 把"参数→指标"追加到调参轨迹表，对比上一轮决定保留/回退。
>
> 注意：`lateral_m`/`e_lat_pct` 是 IMU 航向积分估计，方向正负代表偏左/偏右；CSV 里 `imu_dyaw_deg` 是逐样本航向偏差，可画出"起步 yaw 突变 vs 哪个轮先动"的相关性。最终把好的一版回填到 `crt_chassis_omni.cpp::Init_Motor_Params()`。

---

## 6. JSON 结构速查

```jsonc
{
  "meta": { "distance_m":5, "repeats":10, "directions":"x+,x-",
            "chassis_linear_accel_limit":8,
            "vel_loop_kp":0, "vel_loop_ki":0, "vel_loop_i_limit":1.5,
            "wheel_params":[{wheel,kd,correction,breakaway,...}] },
  "runs": [ { "speed":0.5, "direction":"x+", "repeat":1, "imu_valid":true,
              "delta_psi0_deg":.., "yaw_pp_deg":.., "yaw_final_dev_deg":..,
              "lateral_m":.., "e_lat_pct":.., "reach_spread_s":..,
              "fastest_wheel":i, "slowest_wheel":j,
              "wheel_ratio":[..×4], "reach_time_s":[..×4] }, ... ],
  "aggregate": [ { "speed":0.5, "direction":"x+", "n":10, "n_imu":10,
                   "delta_psi0_deg":{mean,max,std}, "yaw_pp_deg":{mean,max},
                   "e_lat_pct":{mean,max,std}, "reach_spread_s":{mean,max},
                   "wheel_ratio_mean":[..×4] }, ... ]
}
```

CSV 列：`speed,direction,repeat,t_s,cmd_vx,cmd_vy,profiled_vx,profiled_vy,odom_vx,odom_vy,odom_omega,imu_valid,imu_yaw_deg,imu_dyaw_deg,imu_omega_z,imu_age_ms,est_dist_m,est_lat_m,<W?>_ideal/_raw/_profile/_actual/_ratio/_cmd_accel/_cmd_torque/_actual_torque/_vel_i ×4`

> `*_vel_i` = 该轮软件速度环积分(力矩 N·m)；`*_actual_torque` 对照 TMAX=7.8 可辅助判饱和。
> W0 `_actual_torque` 已知不可信，见 §3.1 的 W0_RB 已知反馈项。

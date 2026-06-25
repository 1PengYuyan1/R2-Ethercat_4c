# 全向轮底盘重测总结（2026-06-25）

> 减速比修正后的整车重测全过程与当前定稿。供后续（尤其修完 W1/W2 硬件后）接续。

---

## 0. 一句话现状

底盘已能在 **cmd=1.0（实际 ~0.55 m/s）** 带 IMU 闭环纠偏 + slow-only 跑直，3m 前后 yaw_rms ~2.3° 稳定；
**残留横漂 ~20cm/3m 是 W1/W2 受载力矩饥饿导致的系统性蟹行，软件压不掉，只能修硬件。**

---

## 1. 减速比

- 原 `crt_chassis_omni` 有减速比软件补偿（实际 268:17，固件曾误按 19:1），`COMMAND/FEEDBACK_GEAR_SCALE`。
- 固件改回 268:17 后 **删除该补偿层**。基线回到 git `f77b4cc`（候选 D 参数），candidate K 已回退。

## 2. 诊断链（逐项排除）

### W0_RB —— 反馈假报低（已软件修复）
- 架空 A/B 排除“全局减速比回归”。
- 实测：命令正确转 6.25 圈、反馈只报 4.86 圈（稳态比 **0.78**）→ W0 速度反馈系统性低报，控制器误判其慢而堆力矩。
- 修复：电机层 `Set_Feedback_Omega_Scale`，W0 = **1.282**（只改反馈，不动命令）。
- 附带：**W0 的 `actual_torque` 反馈也不可信**，不作机械负载判据。

### W1/W2 —— 受载力矩饥饿（根因=受载机械，未修）
- 架空正常（~0.95）、落地崩（0.4~0.7），W1/W2=CAN0 这一对同步崩。
- 逐项排除：热降额、趟内时间衰减、CAN0 通讯掉帧、电池 SoC（满电仍崩）、host/固件电流限幅（四轮一致）、空载手转（正常）。
- 结论（by elimination）：**受载才发作的机械阻力**（系统性蟹行的物理根源）。物理修复未最终落地。

## 3. IMU 闭环链路
- 病因：`r2_vehicle_bridge` 以 sudo/root 跑，与用户态 IMU **DDS 隔离** → 收不到 `/IMU_data`。
- 修复：改 **setcap**（`cap_net_raw,cap_net_admin+ep`）让 bridge 以普通用户跑 + ldconfig 修 LD_LIBRARY_PATH。IMU 1000Hz 入闭环，QoS best_effort 匹配。
- 每次 rebuild 后需重做 setcap：
  ```
  sudo setcap cap_net_raw,cap_net_admin+ep install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo
  ```

## 4. slow-only 纠偏（核心方案）
- 洞察：弱轮已饱和，标准纠偏想给它“提速”没余量 → **改成只压快轮、不要求任何轮提速**。
- 实现：底盘 `Set_Yaw_Correction_Slow_Only`（逆解里把被 omega 加速的轮夹回平移幅值）+ heading-hold `Was_Correcting()` + robot.cpp 纠偏激活时置位。默认不影响用户主动转向。

## 5. 调参 / 提速
- 指令速度扫描：**cmd=1.0（实际 ~0.55 m/s）= 硬件不修下最高可用档**；cmd≥1.2 失稳。
- retune 收敛：**kp=7.5 / kd=0.12 / out_limit_scale=0.6 / ki=0**；3m 前后 4/4 稳定，yaw_rms ~2.3°。
- corrections 维持原值 `0.849,1.275,1.147,0.871`（开环配平候选均更差）。

## 6. 代码状态

提交 `9b3f4a7`（分支 feat/odom-tof-yaw-aux-hold）：
- `dvc_motor_dm.h` — `Set_Feedback_Omega_Scale`
- `crt_chassis_omni.h/.cpp` — 去减速比补偿、W0 反馈缩放 1.282、slow-only 逻辑
- `imu_heading_hold.h/.cpp` — `Was_Correcting()`、固化参数（后续 retune 到 kp7.5/kd0.12）

未提交工作区：
- `robot.cpp` 的 slow-only 4 行接线（混在该文件 617 行无关分支 WIP 里，待一起提）。
- ⚠️ 因接线未提交，**单独 checkout `9b3f4a7` 时 slow-only 不会激活**；当前工作区完整可用。

## 7. 当前能力与边界

| 项 | 现状 |
|---|---|
| 工作点 | cmd=1.0，实际 ~0.55 m/s |
| 直行品质 | 3m 前后 yaw_rms ~2.3°，稳定 |
| 残留横漂 | ~20cm/3m（系统性蟹行） |
| 硬上限 | cmd≥1.2 失稳；横漂随距离线性增长（5m 预期 ~30~40cm） |
| 提升出路 | **只剩修 W1/W2 受载机械**——速度与横漂都卡在这条线 |

## 8. 下一步（修硬件后）
1. 修 W1/W2 受载机械（轮系阻滞/蹭/轴承等），先做受载手感对比 + 顶墙堵转测受载电压兜底。
2. 修好后重跑速度阶梯（应能上 ≥1.0 实际、横漂大幅下降）。
3. 在更高实际速度上重调 heading-hold，再固化。
4. 把 robot.cpp 的 slow-only 接线随分支一起提交，使 slow-only 在版本历史里闭环。

## 9. 数据产物
- `var_data/omni/`、`var_data/omni_straight_line_autotune_*/`（CSV/JSON）。
- 工具/文档：`extra_tests/` 下 `omni_straight_retest_*`、`omni_chassis_full_retest_SPEC.md`、`omni_straight_retest_RUNBOOK.md`。

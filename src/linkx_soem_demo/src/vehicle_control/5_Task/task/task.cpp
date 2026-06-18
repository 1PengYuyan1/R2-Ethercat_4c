/**
 * @file task.cpp
 * @brief R2 机器人主控循环 —— 参考 R1 task.cpp 框架，按 R2 硬件实际剪裁。
 *
 *  R2 与 R1 的关键差异：
 *    - 单 LinkX 适配器（无 FD 第二片）
 *    - 全 DM 电机（DM3519 全向轮 + 升降），无 ODrive
 *    - 无外部 BRT 编码器，无编码器多圈持久化
 *    - 无夹爪 Clamp，新增升降 Lift
 *
 *  本文件做的事：
 *    1. EtherCAT 主站 + 单 LinkX 4 通道 CAN-FD（nominal 1Mbps / data 5Mbps）初始化
 *    2. 1ms 节拍主循环：CAN 收 → Robot 周期任务 → CAN 发 → 诊断
 *    3. 周期诊断：CAN 收发统计（5s）、LIVE 仪表盘（200ms）、周期漂移检测
 *    4. 退出钩子：所有 DM 执行器在 ~50ms 内强制失能（deadline-bounded）
 *
 *  明确不做（用户已确认）：
 *    - 舵向零点抓取 / 编码器多圈持久化 / 断电异常检测
 *    - ODrive 速度 CSV / BRT 编码器仪表盘
 */

#include "task.h"

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "robot.h"
#include "task_terminal_diagnostics.h"
#include "timing_terminal_printer.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "math.h"

// ============================================================================
//   全局对象 —— 与 main.cpp 共享（main.cpp 通过 extern 引用 master.is_running）
// ============================================================================

Class_Robot   robot;        ///< 机器人逻辑入口
ecat_master_t master;       ///< EtherCAT 主站
linkx_t       linkx_dev;    ///< LinkX 4 通道 CAN-FD 适配器（nominal 1Mbps / data 5Mbps）

// ============================================================================
//                             编译期可调参数
// ============================================================================
namespace
{

constexpr int      kChannelCount             = 4;        ///< CAN 通道数
constexpr uint32_t kControlLoopPeriodMs      = 1;        ///< 主循环周期 1ms (1kHz)
constexpr uint32_t kTaskPeriod2Ms            = 2;        ///< 2ms 任务节拍
constexpr uint32_t kTaskPeriod100Ms          = 100;      ///< 100ms 任务节拍

constexpr uint8_t kCanFdEnable = 1;
constexpr uint8_t kCanNominalPrescaler = 2;  ///< 80MHz / (2 * (1 + 31 + 8)) = 1Mbps
constexpr uint8_t kCanNominalSeg1 = 31;
constexpr uint8_t kCanNominalSeg2 = 8;
constexpr uint8_t kCanNominalSjw = 8;
constexpr uint8_t kCanDataPrescaler = 1;     ///< 80MHz / (1 * (1 + 12 + 3)) = 5Mbps
constexpr uint8_t kCanDataSeg1 = 12;
constexpr uint8_t kCanDataSeg2 = 3;
constexpr uint8_t kCanDataSjw = 3;

}  // namespace

// ============================================================================
//                                初始化
// ============================================================================

/**
 * @brief EtherCAT 主站 + 单 LinkX 4 通道 CAN-FD 1M/5M 初始化。
 *        slave_id 固定为 1（R2 只有一片 LinkX）。
 *
 * @return true 成功；false 表示初始化失败，调用方应直接返回。
 */
static bool Init_Ethercat_And_Linkx(const char *ifname)
{
    if (!ecat_master_init(&master, ifname))
    {
        return false;
    }

    if (master.ctx.slavecount < 1)
    {
        return false;
    }

    linkx_init(&linkx_dev, 1, &master.ctx);

    bool wakeup_ok[kChannelCount] = {false};
    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        wakeup_ok[ch] = linkx_switch_can_channel(&linkx_dev, static_cast<uint8_t>(ch), true);
    }

    // LinkX 4 路通道统一使用 CAN-FD + BRS:
    // nominal 1Mbps = 80MHz / (2 * (1 + 31 + 8))
    // data    5Mbps = 80MHz / (1 * (1 + 12 + 3))
    bool timing_ok[kChannelCount] = {false};
    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        timing_ok[ch] = linkx_set_can_baudrate(&linkx_dev,
                                               static_cast<uint8_t>(ch),
                                               kCanFdEnable,
                                               kCanNominalPrescaler,
                                               kCanNominalSeg1,
                                               kCanNominalSeg2,
                                               kCanNominalSjw,
                                               kCanDataPrescaler,
                                               kCanDataSeg1,
                                               kCanDataSeg2,
                                               kCanDataSjw);
        if (!timing_ok[ch])
        {
            return false;
        }
    }

    bool reenable_ok[kChannelCount] = {false};
    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        reenable_ok[ch] = linkx_switch_can_channel(&linkx_dev, static_cast<uint8_t>(ch), true);
    }

    return ecat_master_bring_online(&master);
}

static void Init_Robot_Logic()
{
    robot.Init(&linkx_dev);
    robot.Start_ROS2_Bridge();
}

// ============================================================================
//                        主循环每帧子步骤
// ============================================================================

/**
 * @brief 抽干 4 个 CAN 通道的接收队列，逐帧分发给 Robot。
 */
static void Pump_CAN_Receive()
{
    can_msg_t msg;
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
        while (linkx_quick_recv(&linkx_dev, ch, &msg))
            robot.CAN_Rx_Callback(ch, msg.id, msg.data, msg.dlen);
}

/**
 * @brief 按 1ms / 2ms / 100ms 节拍触发 Robot 周期任务。
 */
static void Dispatch_Robot_Tick(uint32_t tick)
{
    robot.TIM_1ms_Calculate_Callback();
    if ((tick % kTaskPeriod2Ms) == 0)
        robot.TIM_2ms_Calculate_PeriodElapsedCallback();
    if ((tick % kTaskPeriod100Ms) == 0)
        robot.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 退出时强制失能所有 DM 执行器。
 *
 *  对象（共 11 个执行器）：
 *    - 4× DM3519 全向轮（ch0: ID 0x01/0x02, ch1: ID 0x01/0x02）
 *    - 6× 前/后抬升（ch0/ch1: ID 0x03/0x04/0x05）
 *    - 1× 辅助 DM 电机（ch0: ID 0x07）
 *
 *  每个 tick 重新入队一次 CAN_Send_Exit；linkx_send_pdos 每 ch 出 1 帧，
 *  共跑 50 cycle ≈ 50ms（实际加 PDO sync 略多），保证多 ID 全部到总线。
 *  期间继续 ecat_master_sync + linkx_recv_pdos，保持 EtherCAT 活动。
 */
static void Disable_All_Devices()
{
    constexpr int kDisableSendCycles = 50;  ///< ≈ 50ms 总预算
    constexpr int kHardDeadlineMs    = 200; ///< 硬上限：超过即放弃

    const auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::milliseconds(kHardDeadlineMs);

    for (int cycle = 0; cycle < kDisableSendCycles; ++cycle)
    {
        if (std::chrono::steady_clock::now() >= deadline)
        {
            break;
        }

        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev);
        Pump_CAN_Receive();

        for (int i = 0; i < kChannelCount; ++i)
        {
            robot.Chassis.Motor_Wheel[i].CAN_Send_Exit();
        }
        for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
        {
            robot.Lift.Motor_Drive_Left[i].CAN_Send_Exit();
            robot.Lift.Motor_Drive_Right[i].CAN_Send_Exit();
            robot.Lift.Motor_Lift[i].CAN_Send_Exit();
        }
        robot.Auxiliary_Motor.CAN_Send_Exit();
        // 软件级状态字也置 DISABLE，防止 1ms 回调再次入队 MIT 帧
        robot.Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);
        robot.Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_DISABLE);
        robot.Auxiliary_Motor.Set_Control_Status(Motor_DM_Status_DISABLE);

        linkx_send_pdos(&linkx_dev);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

/**
 * @brief CAN 统计 / LIVE 仪表盘的周期触发。各自带开关与节拍，互不影响。
 */
static void Dispatch_Periodic_Diagnostics(uint32_t tick)
{
    task_terminal::DispatchPeriodic(tick, robot, &linkx_dev);
}

/**
 * @brief 周期漂移检测：若 sleep_until 之后实际时间已远超目标，说明上一帧超时。
 *        前 kPeriodDriftWarnSkipFirst 个 cycle 静默，避开冷启动 jitter。
 */
static void Check_Period_Drift(uint32_t tick,
                               std::chrono::steady_clock::time_point target)
{
    timing_terminal::WarnIfPeriodDrift(tick, target);
}

static void Flush_Diagnostic_Streams()
{
    task_terminal::FlushStreams();
}

// ============================================================================
//                                主入口
// ============================================================================

void Robot_Control_Loop(const char *ifname)
{
    task_terminal::ParseEnvFlags();

    if (!Init_Ethercat_And_Linkx(ifname))
        return;
    Init_Robot_Logic();

    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;

    while (master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kControlLoopPeriodMs);

        // 进站：EtherCAT 同步 + CAN 接收分发
        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev);
        Pump_CAN_Receive();

        // 计算：Robot 周期任务
        Dispatch_Robot_Tick(tick);

        // 出站：提交 CAN 发送
        linkx_send_pdos(&linkx_dev);

        // 旁路：诊断（不影响 1ms 节拍）
        Dispatch_Periodic_Diagnostics(tick);

        ++tick;
        std::this_thread::sleep_until(next_wakeup);

        Check_Period_Drift(tick, next_wakeup);
    }

    // 退出钩子：强制失能 → 关 ROS 桥
    Flush_Diagnostic_Streams();
    Disable_All_Devices();
    Flush_Diagnostic_Streams();
    robot.Stop_ROS2_Bridge();
}

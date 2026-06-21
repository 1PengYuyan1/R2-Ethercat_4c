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
 *    2. 生产者/消费者双线程模型（参考 tfmini_s_can_read_main.cpp）：
 *         - RT 优先级 IO 线程（EcatIoThread）全速轮询 EtherCAT：sync → recv_pdos
 *           → 去重读取所有 CAN 帧入队 → send_pdos（刷出 TX）
 *         - 1ms 节拍控制线程（本文件主循环）：取走原始帧分发给 Robot/ToF →
 *           Robot 周期任务（电机 TX 经 linkx_send_can 入队，由 IO 线程刷出）→ 诊断
 *       说明：SOEM 的 sync 收发一体且双从站共用网卡，无法按从站拆分，故整条
 *       EtherCAT 周期交换都在 IO 线程；电机 TX 由严格 1ms 改为 IO 线程全速异步刷出。
 *    3. 周期诊断：CAN 收发统计（5s）、LIVE 仪表盘（200ms）、周期漂移检测
 *    4. 退出钩子：所有 DM 执行器在 ~50ms 内强制失能（deadline-bounded）
 *
 *  明确不做（用户已确认）：
 *    - 舵向零点抓取 / 编码器多圈持久化 / 断电异常检测
 *    - ODrive 速度 CSV / BRT 编码器仪表盘
 */

#include "task.h"

#include "ecat_io_thread.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "can_terminal_printer.h"
#include "robot.h"
#include "task_terminal_diagnostics.h"
#include "tfmini_s_range_publisher.h"
#include "timing_terminal_printer.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "math.h"

// ============================================================================
//   全局对象 —— 与 main.cpp 共享（main.cpp 通过 extern 引用 master.is_running）
// ============================================================================

Class_Robot   robot;        ///< 机器人逻辑入口
ecat_master_t master;       ///< EtherCAT 主站
linkx_t       linkx_dev;    ///< LinkX 4 通道 CAN-FD 适配器（nominal 1Mbps / data 5Mbps）
linkx_t       tof_linkx_dev; ///< 可选第二片 LinkX：TFmini-S 上/下方向测距

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
constexpr uint32_t kVehicleSlaveId = 1U;   // EtherCAT slave 1（linkx_dev）
constexpr uint32_t kTofSlave2Id = 2U;      // EtherCAT slave 2（tof_linkx_dev）

// 电机实际所在的 EtherCAT 从站（其 CAN0/CAN1 接整车电机）。
// 本车实测：电机与 slave2 的 up/down ToF 在同一 LinkX 模块上，故默认 2。
// 若硬件接线不同，可用环境变量 R2_MOTOR_SLAVE=1/2 覆盖，无需重编译。
uint32_t g_motor_slave_id = 2U;

bool tof_slave2_enabled = false;
std::shared_ptr<rclcpp::Node> tof_node;
TfminiSRangePublisher tof_range_publisher;

// 返回电机所在从站对应的 LinkX 句柄；若该从站不可用则回退到 slave1。
linkx_t *Motor_Linkx()
{
    if (g_motor_slave_id == kTofSlave2Id)
        return tof_slave2_enabled ? &tof_linkx_dev : &linkx_dev;
    return &linkx_dev;
}

// EtherCAT 全速 IO 线程（生产者）。CAN 帧去重读取/入队、TX 刷出都在它内部完成；
// 控制线程通过 DrainRx 取帧消费。详见 ecat_io_thread.h。
EcatIoThread ecat_io;

// 控制线程取帧用的临时缓冲，复用以避免每帧分配。
std::vector<EcatRawCanMsg> rx_batch;

}  // namespace

// ============================================================================
//                                初始化
// ============================================================================

static bool Configure_Linkx_Channel_1M5M(linkx_t *linkx, uint8_t ch)
{
    if (!linkx_switch_can_channel(linkx, ch, true))
    {
        std::fprintf(stderr,
                     "[R2] LinkX slave%u CAN%d initial enable failed; continuing to baud config\n",
                     linkx ? linkx->slave_id : 0U,
                     ch);
    }

    if (!linkx_set_can_baudrate(linkx,
                                ch,
                                kCanFdEnable,
                                kCanNominalPrescaler,
                                kCanNominalSeg1,
                                kCanNominalSeg2,
                                kCanNominalSjw,
                                kCanDataPrescaler,
                                kCanDataSeg1,
                                kCanDataSeg2,
                                kCanDataSjw))
    {
        std::fprintf(stderr,
                     "[R2] LinkX slave%u CAN%d baudrate 1M/5M SDO config failed\n",
                     linkx ? linkx->slave_id : 0U,
                     ch);
        return false;
    }

    if (!linkx_switch_can_channel(linkx, ch, true))
    {
        std::fprintf(stderr,
                     "[R2] LinkX slave%u CAN%d final enable failed after baud config; continuing\n",
                     linkx ? linkx->slave_id : 0U,
                     ch);
    }

    return true;
}

// 带重试地把某个 CAN 通道使能（SDO 0x8001），返回是否成功（WKC>0）。
static bool Reenable_Linkx_Channel(linkx_t *linkx, uint8_t ch, int max_retries)
{
    for (int attempt = 0; attempt < max_retries; ++attempt)
    {
        if (linkx_switch_can_channel(linkx, ch, true))
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

// 进入 OP 之后重新断言所有 CAN 通道使能。
//
// 背景：部分 LinkX 固件在写波特率（0x8002 触发 0x0B）重配 FDCAN 外设、或在
// SAFE_OP→OP 跃迁时，会复位通道使能位（0x8001）。结果是 SAFE_OP 阶段写过的
// 使能丢失，该通道既收不到也发不出——电机所在的 slave1 CAN0/CAN1 一旦如此，
// 表现就是“按 START 使能电机毫无反应”。SDO 在 OP 态经邮箱仍可写，这里在启动
// IO 线程之前（无并发 EtherCAT 访问）逐通道带重试地重新使能，并打印结果便于排查。
// 使能一块从站的全部 4 个 CAN 通道，把结果填进 ok[] 并整体返回是否全成功。
static void Enable_All_Channels(linkx_t *linkx, bool ok[kChannelCount])
{
    for (int ch = 0; ch < kChannelCount; ++ch)
        ok[ch] = Reenable_Linkx_Channel(linkx, static_cast<uint8_t>(ch), 5);
}

static void Reassert_Can_Channels_After_Op()
{
    bool s1[kChannelCount] = {false, false, false, false};
    bool s2[kChannelCount] = {false, false, false, false};

    Enable_All_Channels(&linkx_dev, s1);
    if (tof_slave2_enabled)
        Enable_All_Channels(&tof_linkx_dev, s2);

    // ===== 大终端打印：两块 EtherCAT 模块全部 CAN 通道的使能状态 =====
    auto cell = [](bool ok) { return ok ? " OK " : "FAIL"; };
    std::fprintf(stderr,
        "\n"
        "================== CAN CHANNEL ENABLE (after OP) ==================\n"
        "  slave  |  CAN0  |  CAN1  |  CAN2  |  CAN3  |  note\n"
        "  -------+--------+--------+--------+--------+----------------------\n"
        "  1 (id=%u)|  %s  |  %s  |  %s  |  %s  | vehicle (motors expected)\n",
        kVehicleSlaveId, cell(s1[0]), cell(s1[1]), cell(s1[2]), cell(s1[3]));

    if (tof_slave2_enabled)
        std::fprintf(stderr,
            "  2 (id=%u)|  %s  |  %s  |  %s  |  %s  | second module\n",
            kTofSlave2Id, cell(s2[0]), cell(s2[1]), cell(s2[2]), cell(s2[3]));
    else
        std::fprintf(stderr,
            "  2       |   --   |   --   |   --   |   --   | not present\n");

    std::fprintf(stderr,
        "==================================================================\n"
        "  提示：电机所在通道必须为 OK。若电机仍无反应，对照本表确认电机实际\n"
        "  接在哪个 slave/CAN 上；若与 robot 发送的 slave(=%u) 不一致，即“从站搞错”。\n"
        "==================================================================\n\n",
        kVehicleSlaveId);
}

/**
 * @brief EtherCAT 主站 + 单 LinkX 4 通道 CAN-FD 1M/5M 初始化。
 *        slave1 作为整车主控；若总线上存在 slave2，则作为 TFmini-S 测距从站接入。
 *
 * @return true 成功；false 表示初始化失败，调用方应直接返回。
 */
static bool Init_Ethercat_And_Linkx(const char *ifname)
{
    if (!ecat_master_init(&master, ifname))
    {
        std::fprintf(stderr, "[R2] EtherCAT master init failed on %s\n", ifname);
        return false;
    }

    if (master.ctx.slavecount < 1)
    {
        std::fprintf(stderr, "[R2] no LinkX slave available for vehicle control\n");
        return false;
    }

    linkx_init(&linkx_dev, kVehicleSlaveId, &master.ctx);

    // LinkX 4 路通道统一使用 CAN-FD + BRS:
    // nominal 1Mbps = 80MHz / (2 * (1 + 31 + 8))
    // data    5Mbps = 80MHz / (1 * (1 + 12 + 3))
    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        if (!Configure_Linkx_Channel_1M5M(&linkx_dev, static_cast<uint8_t>(ch)))
        {
            std::fprintf(stderr, "[R2] vehicle LinkX slave1 CAN%d 1M/5M config failed\n", ch);
            return false;
        }
    }

    tof_slave2_enabled = false;
    if (master.ctx.slavecount >= static_cast<int>(kTofSlave2Id))
    {
        linkx_init(&tof_linkx_dev, kTofSlave2Id, &master.ctx);
        // 诊断：两块模块的全部 4 路通道都配 1M/5M（不再只 CAN2/3），这样不论电机
        // 实际接在哪个从站/通道，都已按正确波特率配置。单通道失败只告警不禁用。
        for (int ch = 0; ch < kChannelCount; ++ch)
        {
            if (!Configure_Linkx_Channel_1M5M(&tof_linkx_dev, static_cast<uint8_t>(ch)))
                std::fprintf(stderr, "[R2] slave2 CAN%d 1M/5M config failed (continuing)\n", ch);
        }
        tof_slave2_enabled = true;
        std::fprintf(stderr, "[R2] slave2 present: configured CAN0-3 @1M/5M\n");
    }
    else
    {
        std::fprintf(stderr,
                     "[R2] EtherCAT slave2 not found; slave1 only\n");
    }

    if (!ecat_master_bring_online(&master))
    {
        std::fprintf(stderr, "[R2] EtherCAT slaves failed to enter OP state\n");
        return false;
    }

    // 进入 OP 后重新断言通道使能，确保 SAFE_OP 阶段写的使能不会因波特率重配或
    // OP 跃迁被复位（尤其是电机所在的 slave1 CAN0/CAN1）。
    Reassert_Can_Channels_After_Op();

    return true;
}

static void Init_Robot_Logic()
{
    // 电机绑定到其实际所在的从站（默认 slave2 的 CAN0/CAN1）。
    linkx_t *motor_linkx = Motor_Linkx();
    std::fprintf(stderr,
                 "[R2] ===== MOTOR bus = EtherCAT slave %u (CAN0/CAN1) =====\n"
                 "[R2]       (覆盖用 R2_MOTOR_SLAVE=1/2；电机收/发都走该从站)\n",
                 g_motor_slave_id);
    robot.Init(motor_linkx);
    robot.Start_ROS2_Bridge();

    if (rclcpp::ok())
    {
        tof_node = std::make_shared<rclcpp::Node>("r2_tof_bridge");
        tof_range_publisher.Start(tof_node);
        tof_range_publisher.ConfigureReset(&linkx_dev,
                                           &tof_linkx_dev,
                                           tof_slave2_enabled);
    }
}

// ============================================================================
//                        主循环每帧子步骤
// ============================================================================

/**
 * @brief 消费捕获的原始 CAN 帧，逐帧分发给 ToF 与 Robot。
 *
 *  分发规则：
 *    - 所有从站所有通道：ToF.HandleCanFrame（按 kSensorMap 的 slave/通道/ID 自行匹配）
 *    - 电机所在从站（g_motor_slave_id）：robot.CAN_Rx_Callback（电机反馈）
 *    - slave2：robot.Lift.CAN_Rx_ToF_Frame（升降用的 up/down ToF）
 *
 *  电机反馈只从电机实际所在的从站投递给电机对象，避免把另一模块上同 ID 的
 *  ToF 帧（id 0x01/0x02）误当成电机反馈。
 */
static void Consume_CAN_Rx()
{
    rx_batch.clear();
    ecat_io.DrainRx(rx_batch);

    for (EcatRawCanMsg &raw : rx_batch)
    {
        tof_range_publisher.HandleCanFrame(raw.slave_id,
                                           raw.channel,
                                           raw.msg.id,
                                           raw.msg.data,
                                           raw.msg.dlen,
                                           raw.msg.timestamp);

        if (raw.slave_id == g_motor_slave_id)
        {
            robot.CAN_Rx_Callback(raw.channel, raw.msg.id, raw.msg.data, raw.msg.dlen);
        }

        if (raw.slave_id == kTofSlave2Id)
        {
            robot.Lift.CAN_Rx_ToF_Frame(raw.slave_id, raw.channel, raw.msg.id, raw.msg.data, raw.msg.dlen);
        }
    }
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
    {
        robot.TIM_100ms_Alive_PeriodElapsedCallback();
        tof_range_publisher.Tick100ms();
    }
}

/**
 * @brief 退出时强制失能所有 DM 执行器。
 *
 *  对象（共 11 个执行器）：
 *    - 4× DM3519 全向轮（ch0: ID 0x01/0x02, ch1: ID 0x01/0x02）
 *    - 6× 前/后抬升（ch0/ch1: ID 0x03/0x04/0x05）
 *    - 1× 辅助 DM 电机（ch0: ID 0x07）
 *
 *  每个 cycle 重新入队一次 CAN_Send_Exit（经 linkx_send_can 进入 tx_queues），
 *  实际刷到总线由仍在运行的 IO 线程 linkx_send_pdos 完成（每 ch 每周期出 1 帧）；
 *  共跑 50 cycle ≈ 50ms，保证多 ID 全部到总线。须在 ecat_io.Stop() 之前调用。
 */
static void Disable_All_Devices(bool use_io_thread)
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

        // 同步模式：在本线程刷出；异步模式：由仍在运行的 IO 线程刷出。
        if (!use_io_thread)
        {
            ecat_io.PollReceiveOnce();
            ecat_io.FlushSendOnce();
        }

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

// 诊断探针：设 R2_CAN_PROBE=1 时，每 1s 打印两块从站每通道的 tx/rx 计数。
// 用于定位电机实际接在哪个 slave/CAN——哪个通道的 RX 在持续增长，电机反馈就在那。
static void Probe_Both_Slaves_Can(uint32_t tick)
{
    static const bool enabled = (std::getenv("R2_CAN_PROBE") != nullptr);
    if (!enabled || tick == 0 || (tick % 1000U) != 0U)
        return;

    std::printf("\n[CAN-PROBE] ===== slave1 (id=%u) =====", kVehicleSlaveId);
    can_terminal::PrintStats(&linkx_dev);
    if (tof_slave2_enabled)
    {
        std::printf("[CAN-PROBE] ===== slave2 (id=%u) =====", kTofSlave2Id);
        can_terminal::PrintStats(&tof_linkx_dev);
    }
    std::fflush(stdout);
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

    // 电机所在从站（默认 2）：可用 R2_MOTOR_SLAVE 覆盖。须在 Init_Robot_Logic
    // （robot.Init 绑定电机 LinkX）之前解析。
    if (const char *ms = std::getenv("R2_MOTOR_SLAVE"))
    {
        const unsigned v = static_cast<unsigned>(std::strtoul(ms, nullptr, 10));
        if (v == kVehicleSlaveId || v == kTofSlave2Id)
            g_motor_slave_id = v;
        else
            std::fprintf(stderr, "[R2] ignoring invalid R2_MOTOR_SLAVE=%s (expect 1 or 2)\n", ms);
    }

    if (!Init_Ethercat_And_Linkx(ifname))
        return;
    Init_Robot_Logic();

    // EtherCAT 收发模式：
    //   默认（异步）：RT 优先级 IO 线程全速 sync/recv/send_pdos，电机 TX 异步刷出。
    //   R2_ECAT_IO_THREAD=0/false/off：回退到同步 in-loop（sync/recv/send 全在
    //                 控制线程，与原始 busy-poll 一致，电机 TX 与 1ms tick 同步）。
    bool use_io_thread = true;
    if (const char *e = std::getenv("R2_ECAT_IO_THREAD"))
    {
        use_io_thread = !(std::strcmp(e, "0") == 0 ||
                          std::strcmp(e, "false") == 0 ||
                          std::strcmp(e, "off") == 0);
    }

    ecat_io.Configure(&master);
    ecat_io.AddLinkx(&linkx_dev, kVehicleSlaveId, true);
    ecat_io.AddLinkx(&tof_linkx_dev, kTofSlave2Id, tof_slave2_enabled);
    // 复位帧的发送与清理围绕 send_pdos 顺序执行，保证只发一次
    // （参考 tfmini_s_can_read_main.cpp）：之前入队复位帧，之后清掉它。
    ecat_io.SetPreSendHook([]() { tof_range_publisher.ServiceResetRequests(); });
    ecat_io.SetPostSendHook([]() { tof_range_publisher.ClearOneShotResetPdos(); });

    std::fprintf(stderr, "[R2] EtherCAT IO mode = %s\n",
                 use_io_thread ? "ASYNC IO thread (default; set R2_ECAT_IO_THREAD=0 for sync)"
                               : "SYNCHRONOUS in-loop (R2_ECAT_IO_THREAD=0)");

    if (use_io_thread)
        ecat_io.Start();

    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;

    while (master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kControlLoopPeriodMs);

        // 进站：同步模式在本线程收一拍；异步模式由 IO 线程已收好。
        if (!use_io_thread)
            ecat_io.PollReceiveOnce();
        Consume_CAN_Rx();

        // 计算：Robot 周期任务（电机 TX 经 linkx_send_can 入队）
        Dispatch_Robot_Tick(tick);

        // 出站：同步模式在本线程发一拍（与本次 tick 入队的 TX 确定性同步）。
        if (!use_io_thread)
            ecat_io.FlushSendOnce();

        // 旁路：诊断（不影响 1ms 节拍）
        Dispatch_Periodic_Diagnostics(tick);
        Probe_Both_Slaves_Can(tick);

        ++tick;

        if (use_io_thread)
        {
            // CAN 收发已由 IO 线程持续进行，控制线程按 1ms 节拍休眠让出 CPU。
            std::this_thread::sleep_until(next_wakeup);
        }
        else
        {
            // 同步模式：忙轮询 EtherCAT 直到下一个 1ms，保持 CAN 收发高频
            //（与原始 Poll_CAN_Until 一致，避免单槽 RX PDO 丢 ToF/反馈帧）。
            while (std::chrono::steady_clock::now() < next_wakeup)
            {
                ecat_io.PollReceiveOnce();
                Consume_CAN_Rx();
                ecat_io.FlushSendOnce();
            }
        }

        Check_Period_Drift(tick, next_wakeup);
    }

    // 退出钩子：强制失能 → （异步模式）停 IO 线程 → 关 ROS 桥
    Flush_Diagnostic_Streams();
    Disable_All_Devices(use_io_thread);
    if (use_io_thread)
        ecat_io.Stop();
    Flush_Diagnostic_Streams();
    tof_range_publisher.Stop();
    tof_node.reset();
    robot.Stop_ROS2_Bridge();
}

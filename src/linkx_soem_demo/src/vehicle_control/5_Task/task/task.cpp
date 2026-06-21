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
constexpr uint32_t kVehicleSlaveId = 1U;
constexpr uint32_t kTofSlave2Id = 2U;

bool tof_slave2_enabled = false;
std::shared_ptr<rclcpp::Node> tof_node;
TfminiSRangePublisher tof_range_publisher;

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
        if (!Configure_Linkx_Channel_1M5M(&tof_linkx_dev, 2U) ||
            !Configure_Linkx_Channel_1M5M(&tof_linkx_dev, 3U))
        {
            std::fprintf(stderr,
                         "[TFMINI-S] slave2 CAN2/CAN3 config failed; ToF publishing disabled, vehicle continues\n");
            tof_slave2_enabled = false;
        }
        else
        {
            tof_slave2_enabled = true;
            std::fprintf(stderr,
                         "[TFMINI-S] slave2 enabled for CAN2/CAN3 TFmini-S publishing\n");
        }
    }
    else
    {
        std::fprintf(stderr,
                     "[TFMINI-S] EtherCAT slave2 not found; publishing only slave1 ToF topics\n");
    }

    if (!ecat_master_bring_online(&master))
    {
        std::fprintf(stderr, "[R2] EtherCAT slaves failed to enter OP state\n");
        return false;
    }

    return true;
}

static void Init_Robot_Logic()
{
    robot.Init(&linkx_dev);
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
 * @brief 消费 IO 线程捕获的原始 CAN 帧，逐帧分发给 ToF 与 Robot。
 *
 *  分发规则与原 Pump_CAN_Receive 一致：
 *    - slave1（整车）所有通道：ToF.HandleCanFrame + robot.CAN_Rx_Callback
 *    - slave2（ToF）所有通道：ToF.HandleCanFrame + robot.Lift.CAN_Rx_ToF_Frame
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

        if (raw.slave_id == kVehicleSlaveId)
        {
            robot.CAN_Rx_Callback(raw.channel, raw.msg.id, raw.msg.data, raw.msg.dlen);
        }
        else if (raw.slave_id == kTofSlave2Id)
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

        // EtherCAT 周期交换由 IO 线程持续进行；这里只负责按节拍重新入队失能帧。
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

    // 生产者：RT 优先级 IO 线程全速轮询 EtherCAT。须在 bring_online 与
    // Init_Robot_Logic（ToF/复位配置就绪）之后启动。
    ecat_io.Configure(&master);
    ecat_io.AddLinkx(&linkx_dev, kVehicleSlaveId, true);
    ecat_io.AddLinkx(&tof_linkx_dev, kTofSlave2Id, tof_slave2_enabled);
    // 复位帧的发送与清理都放到 IO 线程、围绕 send_pdos 顺序执行，保证只发一次
    // （参考 tfmini_s_can_read_main.cpp）：之前入队复位帧，之后清掉它。
    ecat_io.SetPreSendHook([]() { tof_range_publisher.ServiceResetRequests(); });
    ecat_io.SetPostSendHook([]() { tof_range_publisher.ClearOneShotResetPdos(); });
    ecat_io.Start();

    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;

    while (master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kControlLoopPeriodMs);

        // 进站：取走 IO 线程已捕获的原始帧并分发
        Consume_CAN_Rx();

        // 计算：Robot 周期任务（电机 TX 经 linkx_send_can 入队，由 IO 线程刷出）
        Dispatch_Robot_Tick(tick);

        // 旁路：诊断（不影响 1ms 节拍）
        Dispatch_Periodic_Diagnostics(tick);

        ++tick;

        // 控制线程按 1ms 节拍休眠让出 CPU；CAN 收发已由 IO 线程持续进行。
        std::this_thread::sleep_until(next_wakeup);

        Check_Period_Drift(tick, next_wakeup);
    }

    // 退出钩子：强制失能（IO 线程仍在运行以刷出失能帧）→ 停 IO 线程 → 关 ROS 桥
    Flush_Diagnostic_Streams();
    Disable_All_Devices();
    ecat_io.Stop();
    Flush_Diagnostic_Streams();
    tof_range_publisher.Stop();
    tof_node.reset();
    robot.Stop_ROS2_Bridge();
}

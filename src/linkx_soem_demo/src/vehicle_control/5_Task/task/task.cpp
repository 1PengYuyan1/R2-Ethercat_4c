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

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
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
constexpr uint32_t kCanStatPrintPeriodMs     = 5000;     ///< CAN 统计打印周期 5s
constexpr uint32_t kLiveDashboardPeriodMs    = 200;      ///< LIVE 仪表盘刷新周期
constexpr uint32_t kOpsPrintPeriodMs         = 1000;     ///< OPS 解算数据打印周期 1s
constexpr uint32_t kPeriodDriftWarnSkipFirst = 500;      ///< 前 500 cycle 静默漂移告警
constexpr int64_t  kPeriodDriftWarnUs        = 1500;     ///< 周期漂移阈值 1.5ms

constexpr uint8_t kCanFdEnable = 1;
constexpr uint8_t kCanNominalPrescaler = 2;  ///< 80MHz / (2 * (1 + 31 + 8)) = 1Mbps
constexpr uint8_t kCanNominalSeg1 = 31;
constexpr uint8_t kCanNominalSeg2 = 8;
constexpr uint8_t kCanNominalSjw = 8;
constexpr uint8_t kCanDataPrescaler = 1;     ///< 80MHz / (1 * (1 + 12 + 3)) = 5Mbps
constexpr uint8_t kCanDataSeg1 = 12;
constexpr uint8_t kCanDataSeg2 = 3;
constexpr uint8_t kCanDataSjw = 3;

// R1 实测：iostream 高频刷新会引起 stdout 背压。两个开关默认 false，
// 需要时通过环境变量 ENABLE_DASHBOARD=1 / ENABLE_CAN_STATS=1 启用。
bool g_enable_can_stat_print  = false;
bool g_enable_live_dashboard  = false;
bool g_enable_ops_print       = true;

constexpr const char *kDefaultVarDataFile = "var_data/live_variables.log";
constexpr const char *kDefaultOpsPrintFile = "var_data/ops_terminal.log";

std::ofstream g_var_data_stream;       ///< 仪表盘镜像日志（懒打开）
bool          g_var_data_stream_inited = false;
std::ofstream g_ops_print_stream;
bool          g_ops_print_stream_inited = false;

static const char *Ok_Fail(bool ok)
{
    return ok ? "OK" : "FAIL";
}

}  // namespace

// ============================================================================
//                            初始化辅助
// ============================================================================

static void Parse_Diagnostic_Env_Flags()
{
    const char *cs = std::getenv("ENABLE_CAN_STATS");
    if (cs != nullptr && cs[0] == '1')
        g_enable_can_stat_print = true;

    const char *dash = std::getenv("ENABLE_DASHBOARD");
    if (dash != nullptr && dash[0] == '1')
        g_enable_live_dashboard = true;

    const char *ops = std::getenv("ENABLE_OPS_PRINT");
    if (ops != nullptr && ops[0] == '0')
        g_enable_ops_print = false;
}

static void Ensure_Var_Data_Stream()
{
    if (g_var_data_stream_inited)
        return;

    const char *path = std::getenv("VAR_DATA_FILE");
    if (path == nullptr || path[0] == '\0')
        path = kDefaultVarDataFile;

    g_var_data_stream.open(path, std::ios::app);
    g_var_data_stream_inited = true;
    if (!g_var_data_stream.is_open())
        std::cerr << "[WARN] Failed to open variable data file: " << path << std::endl;
}

static void Ensure_OPS_Print_Stream()
{
    if (g_ops_print_stream_inited)
        return;

    const char *path = std::getenv("OPS_PRINT_FILE");
    if (path == nullptr || path[0] == '\0')
        path = kDefaultOpsPrintFile;

    g_ops_print_stream.open(path, std::ios::app);
    g_ops_print_stream_inited = true;
    if (!g_ops_print_stream.is_open())
        std::cerr << "[WARN] Failed to open OPS print file: " << path << std::endl;
}

// ============================================================================
//                         诊断打印 —— CAN 统计
// ============================================================================

/**
 * @brief 打印 4 个 CAN 通道的累计 TX/RX/丢帧/丢包率与全局合计。
 *        每 kCanStatPrintPeriodMs 调用一次。用 printf 避免 iostream 背压。
 */
static void Print_CAN_Stats(linkx_t *linkx)
{
    std::printf("\n[CAN-TOTAL]\n");

    uint64_t sum_tx_req_f = 0, sum_tx_req_b = 0;
    uint64_t sum_tx_sent_f = 0, sum_tx_sent_b = 0;
    uint64_t sum_tx_drop_f = 0;
    uint64_t sum_rx_f = 0, sum_rx_b = 0;

    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        const auto &s = linkx->can_stats[ch];
        const uint16_t q_fill = linkx->tx_queues[ch].size;
        const uint64_t loss_f = (s.tx_frames > s.rx_frames) ? (s.tx_frames - s.rx_frames) : 0;
        const double loss_rt = (s.tx_frames > 0) ? (100.0 * (double)loss_f / (double)s.tx_frames) : 0.0;

        std::printf("  CH%d TX_REQ=%llu (%llu B) TX_SENT=%llu (%llu B) TX_DROP=%llu Q=%u"
                    " RX=%llu (%llu B) LOSS=%llu (%.2f%%)\n",
                    ch,
                    (unsigned long long)s.tx_enqueued_frames,
                    (unsigned long long)s.tx_enqueued_bytes,
                    (unsigned long long)s.tx_frames,
                    (unsigned long long)s.tx_bytes,
                    (unsigned long long)s.tx_dropped_frames,
                    (unsigned)q_fill,
                    (unsigned long long)s.rx_frames,
                    (unsigned long long)s.rx_bytes,
                    (unsigned long long)loss_f,
                    loss_rt);

        sum_tx_req_f  += s.tx_enqueued_frames;
        sum_tx_req_b  += s.tx_enqueued_bytes;
        sum_tx_sent_f += s.tx_frames;
        sum_tx_sent_b += s.tx_bytes;
        sum_tx_drop_f += s.tx_dropped_frames;
        sum_rx_f      += s.rx_frames;
        sum_rx_b      += s.rx_bytes;
    }

    const uint64_t sum_loss_f = (sum_tx_sent_f > sum_rx_f) ? (sum_tx_sent_f - sum_rx_f) : 0;
    const double sum_loss_rt = (sum_tx_sent_f > 0) ? (100.0 * (double)sum_loss_f / (double)sum_tx_sent_f) : 0.0;
    std::printf("  ALL TX_REQ=%llu (%llu B) TX_SENT=%llu (%llu B) TX_DROP=%llu RX=%llu (%llu B) LOSS=%llu (%.2f%%)\n",
                (unsigned long long)sum_tx_req_f,
                (unsigned long long)sum_tx_req_b,
                (unsigned long long)sum_tx_sent_f,
                (unsigned long long)sum_tx_sent_b,
                (unsigned long long)sum_tx_drop_f,
                (unsigned long long)sum_rx_f,
                (unsigned long long)sum_rx_b,
                (unsigned long long)sum_loss_f,
                sum_loss_rt);
}

// ============================================================================
//                      诊断打印 —— LIVE 仪表盘
// ============================================================================

/**
 * @brief 单缓冲 snprintf 渲染 LIVE 仪表盘，一次 fwrite 出栈。
 *        避免 iostream 多次 flush 造成 1-10ms 阻塞。
 */
static void Print_Live_Dashboard()
{
    Ensure_Var_Data_Stream();

    constexpr size_t kBufSize = 8192;
    char buf[kBufSize];
    int n = 0;

    // 清屏 + 头
    n += std::snprintf(buf + n, kBufSize - n,
                       "\033[2J\033[H[LIVE-DASHBOARD] refresh=%ums\n",
                       (unsigned)kLiveDashboardPeriodMs);

    // 底盘整体状态
    n += std::snprintf(buf + n, kBufSize - n,
                       "\n[CHASSIS]\n  ctrl=%d  tgt_vx=%.3f tgt_vy=%.3f"
                       "  now_vx=%.3f now_vy=%.3f now_omega=%.3f\n",
                       (int)robot.Chassis.Get_Chassis_Control_Type(),
                       robot.Chassis.Get_Target_Velocity_X(),
                       robot.Chassis.Get_Target_Velocity_Y(),
                       robot.Chassis.Get_Now_Velocity_X(),
                       robot.Chassis.Get_Now_Velocity_Y(),
                       robot.Chassis.Get_Now_Omega());

    // 4 个轮向 DM3519（全向轮）
    n += std::snprintf(buf + n, kBufSize - n, "\n[OMNI-WHEEL-DM3519]\n");
    for (int i = 0; i < kChannelCount && n < (int)kBufSize - 256; ++i)
    {
        auto &dm = robot.Chassis.Motor_Wheel[i];
        n += std::snprintf(buf + n, kBufSize - n,
                           "  W%d tx=0x%02X rx=0x%02X omega=%+8.3f torque=%+5.2f"
                           " T_mos=%3.0f T_rot=%3.0f status=%d\n",
                           i, (unsigned)dm.DM_CAN_Tx_ID, (unsigned)dm.DM_CAN_Rx_ID,
                           dm.Get_Now_Omega(), dm.Get_Now_Torque(),
                           dm.Get_Now_MOS_Temperature(), dm.Get_Now_Rotor_Temperature(),
                           (int)dm.Get_Status());
    }

    // 前/后抬升与差速驱动
    if (n < (int)kBufSize - 768)
    {
        n += std::snprintf(buf + n, kBufSize - n,
                           "\n[LIFT]\n  ctrl=%d diff=%d diff_cmd=(%.3f, %.3f)\n",
                           (int)robot.Lift.Get_Control_Type(),
                           robot.Lift.Get_Diff_Drive_Enable() ? 1 : 0,
                           robot.Lift.Get_Target_Diff_Forward(),
                           robot.Lift.Get_Target_Diff_Yaw());
        for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM && n < (int)kBufSize - 256; ++i)
        {
            auto &dl = robot.Lift.Motor_Drive_Left[i];
            auto &dr = robot.Lift.Motor_Drive_Right[i];
            auto &lm = robot.Lift.Motor_Lift[i];
            n += std::snprintf(buf + n, kBufSize - n,
                               "  %s DL tx=0x%02X w=%+7.3f DR tx=0x%02X w=%+7.3f"
                               " LIFT tx=0x%02X rad=%+7.3f alive=(%d,%d,%d) ctrlst=(%d,%d,%d)\n",
                               (i == CHARIOT_LIFT_MODULE_FRONT) ? "FRONT" : "REAR ",
                               (unsigned)dl.DM_CAN_Tx_ID, dl.Get_Now_Omega(),
                               (unsigned)dr.DM_CAN_Tx_ID, dr.Get_Now_Omega(),
                               (unsigned)lm.DM_CAN_Tx_ID, lm.Get_Now_Radian(),
                               (int)dl.Get_Status(), (int)dr.Get_Status(), (int)lm.Get_Status(),
                               (int)dl.Get_Now_Control_Status(),
                               (int)dr.Get_Now_Control_Status(),
                               (int)lm.Get_Now_Control_Status());
        }
    }

    if (n > 0)
    {
        std::fwrite(buf, 1, (size_t)n, stdout);
        std::fflush(stdout);
        if (g_var_data_stream.is_open())
        {
            g_var_data_stream.write(buf, n);
            g_var_data_stream.flush();
        }
    }
}

static void Print_OPS_Data()
{
    Ensure_OPS_Print_Stream();

    const Struct_OPS_Rx_Data data = robot.ops.getData();
    const Struct_OPS_Raw_CAN_Frame raw = robot.ops.getRawCANFrame();
    const Struct_OPS_Decoded_CAN_Frame decoded = robot.ops.getLastDecodedFrame();
    const bool connected = robot.ops.isConnected();
    const bool recent_frame = robot.ops.hasRecentFrame();
    const char *status = connected ? "OK" : (recent_frame ? "BAD_NAN" : "LOST");

    char line[512];
    int n = std::snprintf(line, sizeof(line),
                          "[OPS] %s  pos_x=%+.2f mm  pos_y=%+.2f mm"
                          "  yaw=%+.4f deg  pitch=%+.4f deg  roll=%+.4f deg"
                          "  omega_z=%+.4f deg/s  decoded=%u invalid=%u  can2_id=0x01 dlc=%u data=",
                          status,
                          data.Pos_X,
                          data.Pos_Y,
                          data.Yaw,
                          data.Pitch,
                          data.Roll,
                          data.Omega_Z,
                          decoded.count,
                          decoded.invalid_count,
                          (unsigned)raw.dlen);
    for (uint8_t i = 0; i < raw.dlen; ++i)
    {
        if (n < (int)sizeof(line))
        {
            n += std::snprintf(line + n, sizeof(line) - n,
                               "%02X%s", raw.data[i], (i + 1U < raw.dlen) ? " " : "");
        }
    }
    if (n < (int)sizeof(line))
    {
        n += std::snprintf(line + n, sizeof(line) - n, " frame28=");
    }
    for (uint8_t i = 0; i < decoded.data.size(); ++i)
    {
        if (n < (int)sizeof(line))
        {
            n += std::snprintf(line + n, sizeof(line) - n,
                               "%02X%s", decoded.data[i], (i + 1U < decoded.data.size()) ? " " : "");
        }
    }
    if (n < (int)sizeof(line))
    {
        n += std::snprintf(line + n, sizeof(line) - n, "\n");
    }
    const size_t len = (n > 0 && n < (int)sizeof(line)) ? (size_t)n : std::strlen(line);
    std::fwrite(line, 1, len, stdout);
    if (g_ops_print_stream.is_open())
    {
        g_ops_print_stream.write(line, len);
        g_ops_print_stream.flush();
    }
    std::fflush(stdout);
}

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
        std::cerr << "[TASK] ecat_master_init failed for ifname=" << ifname << std::endl;
        return false;
    }

    if (master.ctx.slavecount < 1)
    {
        std::cerr << "[TASK] FATAL: no EtherCAT slave detected. Check cabling." << std::endl;
        return false;
    }

    linkx_init(&linkx_dev, 1, &master.ctx);

    bool wakeup_ok[kChannelCount] = {false};
    std::cout << "[LinkX] wakeup: enabling CAN channels" << std::endl;
    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        wakeup_ok[ch] = linkx_switch_can_channel(&linkx_dev, static_cast<uint8_t>(ch), true);
    }
    std::cout << "[LinkX] wakeup result: CAN0=" << Ok_Fail(wakeup_ok[0])
              << " CAN1=" << Ok_Fail(wakeup_ok[1])
              << " CAN2=" << Ok_Fail(wakeup_ok[2])
              << " CAN3=" << Ok_Fail(wakeup_ok[3]) << std::endl;
    if (!wakeup_ok[0])
    {
        std::cerr << "[LinkX][WARN] CAN0 wakeup SDO failed; continuing because CAN0 TX works on this hardware." << std::endl;
    }
    if (!wakeup_ok[1])
    {
        std::cerr << "[LinkX][WARN] CAN1 wakeup SDO failed; watch CAN1 tx/rx." << std::endl;
    }

    // LinkX 4 路通道统一使用 CAN-FD + BRS:
    // nominal 1Mbps = 80MHz / (2 * (1 + 31 + 8))
    // data    5Mbps = 80MHz / (1 * (1 + 12 + 3))
    bool timing_ok[kChannelCount] = {false};
    std::cout << "[LinkX] timing: configuring CAN baudrates" << std::endl;
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
            std::cerr << "[LinkX][FATAL] CAN" << ch << " FDCAN 1M/5M config failed." << std::endl;
            return false;
        }
    }
    std::cout << "[LinkX] timing result: CAN0=FD1M/5M " << Ok_Fail(timing_ok[0])
              << " CAN1=FD1M/5M " << Ok_Fail(timing_ok[1])
              << " CAN2=FD1M/5M " << Ok_Fail(timing_ok[2])
              << " CAN3=FD1M/5M " << Ok_Fail(timing_ok[3]) << std::endl;

    bool reenable_ok[kChannelCount] = {false};
    std::cout << "[LinkX] post-config: re-enabling CAN channels" << std::endl;
    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        reenable_ok[ch] = linkx_switch_can_channel(&linkx_dev, static_cast<uint8_t>(ch), true);
    }
    std::cout << "[LinkX] post-config result: CAN0=" << Ok_Fail(reenable_ok[0])
              << " CAN1=" << Ok_Fail(reenable_ok[1])
              << " CAN2=" << Ok_Fail(reenable_ok[2])
              << " CAN3=" << Ok_Fail(reenable_ok[3]) << std::endl;
    if (!reenable_ok[0])
    {
        std::cerr << "[LinkX][WARN] CAN0 post-config wakeup failed; watch CAN0 tx/rx." << std::endl;
    }
    if (!reenable_ok[1])
    {
        std::cerr << "[LinkX][WARN] CAN1 post-config wakeup failed; watch CAN1 tx/rx." << std::endl;
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
 *  对象（共 10 个执行器）：
 *    - 4× DM3519 全向轮（ch0: ID 0x01/0x02, ch1: ID 0x01/0x02）
 *    - 6× 前/后抬升（ch0/ch1: ID 0x03/0x04/0x05）
 *
 *  每个 tick 重新入队一次 CAN_Send_Exit；linkx_send_pdos 每 ch 出 1 帧，
 *  共跑 50 cycle ≈ 50ms（实际加 PDO sync 略多），保证多 ID 全部到总线。
 *  期间继续 ecat_master_sync + linkx_recv_pdos，保持 EtherCAT 活动。
 */
static void Disable_All_Devices()
{
    constexpr int kDisableSendCycles = 50;  ///< ≈ 50ms 总预算
    constexpr int kHardDeadlineMs    = 200; ///< 硬上限：超过即放弃

    std::cout << "\n[TASK] Disabling all devices (Ctrl+C safety)..." << std::endl;

    const auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::milliseconds(kHardDeadlineMs);

    for (int cycle = 0; cycle < kDisableSendCycles; ++cycle)
    {
        if (std::chrono::steady_clock::now() >= deadline)
        {
            std::cerr << "[TASK] Disable_All_Devices hit hard deadline; bailing out." << std::endl;
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
        // 软件级状态字也置 DISABLE，防止 1ms 回调再次入队 MIT 帧
        robot.Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);
        robot.Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_DISABLE);

        linkx_send_pdos(&linkx_dev);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "[TASK] All devices disabled." << std::endl;
}

/**
 * @brief CAN 统计 / LIVE 仪表盘的周期触发。各自带开关与节拍，互不影响。
 */
static void Dispatch_Periodic_Diagnostics(uint32_t tick)
{
    if (g_enable_ops_print && tick != 0 && (tick % kOpsPrintPeriodMs) == 0)
        Print_OPS_Data();

    if (g_enable_can_stat_print && tick != 0 && (tick % kCanStatPrintPeriodMs) == 0)
    {
        std::printf("\n[CAN-STATS] === LinkX (slave_id=1) ===");
        Print_CAN_Stats(&linkx_dev);
    }

    if (g_enable_live_dashboard && (tick % kLiveDashboardPeriodMs) == 0)
        Print_Live_Dashboard();
}

/**
 * @brief 周期漂移检测：若 sleep_until 之后实际时间已远超目标，说明上一帧超时。
 *        前 kPeriodDriftWarnSkipFirst 个 cycle 静默，避开冷启动 jitter。
 */
static void Check_Period_Drift(uint32_t tick,
                               std::chrono::steady_clock::time_point target)
{
    if (tick < kPeriodDriftWarnSkipFirst)
        return;

    const auto now = std::chrono::steady_clock::now();
    const auto drift_us = std::chrono::duration_cast<std::chrono::microseconds>(now - target).count();
    if (drift_us > kPeriodDriftWarnUs)
    {
        std::fprintf(stderr, "[TASK][WARN] tick=%u drift=%lldus\n",
                     (unsigned)tick, (long long)drift_us);
    }
}

// ============================================================================
//                                主入口
// ============================================================================

void Robot_Control_Loop(const char *ifname)
{
    Parse_Diagnostic_Env_Flags();

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
    Disable_All_Devices();
    robot.Stop_ROS2_Bridge();
}

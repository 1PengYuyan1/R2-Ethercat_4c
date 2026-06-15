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
constexpr uint32_t kCanStatPrintPeriodMs     = 5000;     ///< CAN 统计打印周期 5s
constexpr uint32_t kLiveDashboardPeriodMs    = 200;      ///< LIVE 仪表盘刷新周期
constexpr uint32_t kToFPrintPeriodMs         = 200;      ///< ToF 终端打印周期
constexpr uint32_t kDefaultToFButtonLogPeriodMs = 20;    ///< ToF+按钮 CSV 记录周期
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
bool g_enable_tof_print       = true;
bool g_tof_print_stdout       = false;
bool g_enable_tof_button_log  = true;
uint32_t g_tof_button_log_period_ms = kDefaultToFButtonLogPeriodMs;

constexpr const char *kDefaultVarDataFile = "var_data/live_variables.log";
constexpr const char *kDefaultToFPrintFile = "var_data/ops_terminal.log";

std::ofstream g_var_data_stream;       ///< 仪表盘镜像日志（懒打开）
bool          g_var_data_stream_inited = false;
std::ofstream g_tof_print_stream;      ///< ToF 简洁终端输出，start 脚本会 tail
bool          g_tof_print_stream_inited = false;
std::ofstream g_tof_button_log_stream; ///< ToF+按钮 CSV 数据记录
bool          g_tof_button_log_stream_inited = false;
std::string   g_tof_button_log_path;
uint32_t      g_tof_button_log_rows_since_flush = 0;

static const char *Ok_Fail(bool ok)
{
    return ok ? "OK" : "FAIL";
}

static const char *Lift_ToF_Name(int index)
{
    switch (index)
    {
        case CHARIOT_LIFT_TOF_UP_FRONT: return "up_front";
        case CHARIOT_LIFT_TOF_UP_BACK: return "up_back";
        case CHARIOT_LIFT_TOF_DOWN_FRONT: return "down_front";
        case CHARIOT_LIFT_TOF_DOWN_BACK: return "down_back";
        default: return "unknown";
    }
}

static const char *Lift_ToF_Topic(int index)
{
    switch (index)
    {
        case CHARIOT_LIFT_TOF_UP_FRONT: return "/high/up_front/range";
        case CHARIOT_LIFT_TOF_UP_BACK: return "/high/up_back/range";
        case CHARIOT_LIFT_TOF_DOWN_FRONT: return "/high/down_front/range";
        case CHARIOT_LIFT_TOF_DOWN_BACK: return "/high/down_back/range";
        default: return "/high/unknown/range";
    }
}

static const char *Button_Name(uint16_t code)
{
    switch (code)
    {
        case LogF710_Key_IDLE: return "IDLE";
        case LogF710_Key_X: return "X";
        case LogF710_Key_A: return "A";
        case LogF710_Key_B: return "B";
        case LogF710_Key_Y: return "Y";
        case LogF710_Key_LB: return "LB";
        case LogF710_Key_LB_X: return "LB+X";
        case LogF710_Key_LB_Y: return "LB+Y";
        case LogF710_Key_LT: return "LT";
        case LogF710_Key_RB: return "RB";
        case LogF710_Key_RT: return "RT";
        case LogF710_Key_Back: return "Back";
        case LogF710_Key_Start: return "Start";
        case LogF710_Key_Right: return "Right";
        case LogF710_Key_Left: return "Left";
        case LogF710_Key_Up: return "Up";
        case LogF710_Key_Down: return "Down";
        default: return "Unknown";
    }
}

static void Format_ToF_Range(float range_m, char *buf, size_t len)
{
    if (std::isnan(range_m))
    {
        std::snprintf(buf, len, "nan");
    }
    else if (!std::isfinite(range_m))
    {
        std::snprintf(buf, len, "%sinf", std::signbit(range_m) ? "-" : "+");
    }
    else
    {
        std::snprintf(buf, len, "%.3fm", range_m);
    }
}

static void Format_Csv_Float(float value, char *buf, size_t len)
{
    if (std::isnan(value))
    {
        std::snprintf(buf, len, "nan");
    }
    else if (!std::isfinite(value))
    {
        std::snprintf(buf, len, "%sinf", std::signbit(value) ? "-" : "+");
    }
    else
    {
        std::snprintf(buf, len, "%.5f", value);
    }
}

static void Format_Table_Float(bool valid,
                               float value,
                               float scale,
                               int precision,
                               char *buf,
                               size_t len)
{
    if (!valid)
    {
        std::snprintf(buf, len, "no_data");
    }
    else if (std::isnan(value))
    {
        std::snprintf(buf, len, "nan");
    }
    else if (!std::isfinite(value))
    {
        std::snprintf(buf, len, "%sinf", std::signbit(value) ? "-" : "+");
    }
    else
    {
        std::snprintf(buf, len, "%.*f", precision, value * scale);
    }
}

static const char *Imu_State_Name(const Class_Chariot_Imu_Heading_Hold::Snapshot &imu)
{
    if (!imu.valid)
        return "NO_DATA";
    return imu.fresh ? "OK" : "STALE";
}

static std::string Make_Default_ToF_Button_Log_Path()
{
    std::time_t now = std::time(nullptr);
    std::tm tm_now {};
    localtime_r(&now, &tm_now);

    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm_now);
    return std::string("var_data/tof_button_log_") + ts + ".csv";
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

    const char *tof = std::getenv("ENABLE_TOF_PRINT");
    if (tof != nullptr && tof[0] == '0')
        g_enable_tof_print = false;

    const char *tof_stdout = std::getenv("TOF_PRINT_STDOUT");
    if (tof_stdout != nullptr && tof_stdout[0] == '1')
        g_tof_print_stdout = true;

    const char *tof_button_log = std::getenv("ENABLE_TOF_BUTTON_LOG");
    if (tof_button_log != nullptr && tof_button_log[0] == '0')
        g_enable_tof_button_log = false;

    const char *tof_button_period = std::getenv("TOF_BUTTON_LOG_PERIOD_MS");
    if (tof_button_period != nullptr && tof_button_period[0] != '\0')
    {
        char *end = nullptr;
        const unsigned long parsed = std::strtoul(tof_button_period, &end, 10);
        if (end != tof_button_period && parsed > 0UL && parsed <= 10000UL)
            g_tof_button_log_period_ms = static_cast<uint32_t>(parsed);
    }
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

static void Ensure_ToF_Print_Stream()
{
    if (g_tof_print_stream_inited)
        return;

    const char *path = std::getenv("TOF_PRINT_FILE");
    if (path == nullptr || path[0] == '\0')
        path = kDefaultToFPrintFile;

    g_tof_print_stream.open(path, std::ios::app);
    g_tof_print_stream_inited = true;
    if (!g_tof_print_stream.is_open())
        std::cerr << "[WARN] Failed to open ToF print file: " << path << std::endl;
}

static void Ensure_ToF_Button_Log_Stream()
{
    if (g_tof_button_log_stream_inited)
        return;

    const char *path = std::getenv("TOF_BUTTON_LOG_FILE");
    g_tof_button_log_path =
        (path != nullptr && path[0] != '\0') ? std::string(path) :
        Make_Default_ToF_Button_Log_Path();

    g_tof_button_log_stream.open(g_tof_button_log_path, std::ios::out);
    g_tof_button_log_stream_inited = true;
    if (!g_tof_button_log_stream.is_open())
    {
        std::cerr << "[WARN] Failed to open ToF/button log file: "
                  << g_tof_button_log_path << std::endl;
        return;
    }

    g_tof_button_log_stream
        << "tick_ms,t_s,"
        << "button_code_dec,button_code_hex,button_name,button_has,button_recent,button_age_ms,"
        << "stair_state,stair_name,stair_chassis_mps";
    for (int i = 0; i < CHARIOT_LIFT_TOF_NUM; ++i)
    {
        const char *name = Lift_ToF_Name(i);
        g_tof_button_log_stream
            << "," << name << "_online"
            << "," << name << "_valid"
            << "," << name << "_cm"
            << "," << name << "_m"
            << "," << name << "_strength"
            << "," << name << "_frames";
    }
    g_tof_button_log_stream << "\n";
    g_tof_button_log_stream.flush();

    std::cout << "[LOG] ToF/button CSV: " << g_tof_button_log_path
              << " period=" << g_tof_button_log_period_ms << "ms" << std::endl;
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
                           "\n[LIFT]\n  ctrl=%d diff=%d diff_cmd=(%.3f, %.3f)"
                           " stair=%s(%d) stair_chassis=%.3f\n",
                           (int)robot.Lift.Get_Control_Type(),
                           robot.Lift.Get_Diff_Drive_Enable() ? 1 : 0,
                           robot.Lift.Get_Target_Diff_Forward(),
                           robot.Lift.Get_Target_Diff_Yaw(),
                           robot.Lift.Get_Stair_State_Name(),
                           (int)robot.Lift.Get_Stair_State(),
                           robot.Lift.Get_Stair_Chassis_Forward());
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

    if (n < (int)kBufSize - 512)
    {
        n += std::snprintf(buf + n, kBufSize - n, "\n[LIFT-TOF]\n");
        for (int i = 0; i < CHARIOT_LIFT_TOF_NUM && n < (int)kBufSize - 128; ++i)
        {
            const auto sensor = static_cast<Enum_Chariot_Lift_ToF_Sensor>(i);
            const ChariotLiftToFData &tof = robot.Lift.Get_ToF_Data(sensor);
            char range_buf[24];
            Format_ToF_Range(tof.range_m, range_buf, sizeof(range_buf));
            n += std::snprintf(buf + n, kBufSize - n,
                               "  %-10s %s range=%s raw=%ucm strength=%u frames=%u\n",
                               Lift_ToF_Name(i),
                               tof.online ? (tof.valid ? "OK " : "BAD") : "OFF",
                               range_buf,
                               (unsigned)tof.distance_cm,
                               (unsigned)tof.strength,
                               (unsigned)tof.frame_count);
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

static void Print_ToF_Terminal_Data()
{
    Ensure_ToF_Print_Stream();

    const auto imu = robot.Get_Imu_Snapshot();
    char roll_deg[24];
    char pitch_deg[24];
    char yaw_deg[24];
    char gyro_x[24];
    char gyro_y[24];
    char gyro_z[24];
    char accel_x[24];
    char accel_y[24];
    char accel_z[24];
    char imu_age[24];

    Format_Table_Float(imu.valid, imu.roll_rad, RAD_TO_DEG, 2, roll_deg, sizeof(roll_deg));
    Format_Table_Float(imu.valid, imu.pitch_rad, RAD_TO_DEG, 2, pitch_deg, sizeof(pitch_deg));
    Format_Table_Float(imu.valid, imu.yaw_rad, RAD_TO_DEG, 2, yaw_deg, sizeof(yaw_deg));
    Format_Table_Float(imu.valid, imu.angular_velocity_x, 1.0f, 3, gyro_x, sizeof(gyro_x));
    Format_Table_Float(imu.valid, imu.angular_velocity_y, 1.0f, 3, gyro_y, sizeof(gyro_y));
    Format_Table_Float(imu.valid, imu.angular_velocity_z, 1.0f, 3, gyro_z, sizeof(gyro_z));
    Format_Table_Float(imu.valid, imu.linear_acceleration_x, 1.0f, 3, accel_x, sizeof(accel_x));
    Format_Table_Float(imu.valid, imu.linear_acceleration_y, 1.0f, 3, accel_y, sizeof(accel_y));
    Format_Table_Float(imu.valid, imu.linear_acceleration_z, 1.0f, 3, accel_z, sizeof(accel_z));
    if (imu.valid)
        std::snprintf(imu_age, sizeof(imu_age), "%lld", (long long)imu.age_ms);
    else
        std::snprintf(imu_age, sizeof(imu_age), "no_data");

    char frame[2048];
    int n = std::snprintf(frame, sizeof(frame),
                          "\033[2J\033[H[LIFT-TOF+IMU] refresh=%ums\n\n"
                          "+------------------------+--------+----------+--------+----------+----------+\n"
                          "| ToF topic              | status | range    | raw_cm | strength | frames   |\n"
                          "+------------------------+--------+----------+--------+----------+----------+\n",
                          (unsigned)kToFPrintPeriodMs);

    for (int i = 0; i < CHARIOT_LIFT_TOF_NUM && n < (int)sizeof(frame) - 160; ++i)
    {
        const auto sensor = static_cast<Enum_Chariot_Lift_ToF_Sensor>(i);
        const ChariotLiftToFData &tof = robot.Lift.Get_ToF_Data(sensor);
        char range_buf[24];
        Format_ToF_Range(tof.range_m, range_buf, sizeof(range_buf));

        const char *value = tof.online ? range_buf : "no_data";
        const char *status = tof.online ? (tof.valid ? "OK" : "BAD") : "OFF";
        n += std::snprintf(frame + n, sizeof(frame) - n,
                           "| %-22s | %-6s | %-8s | %6u | %8u | %8u |\n",
                           Lift_ToF_Topic(i),
                           status,
                           value,
                           (unsigned)tof.distance_cm,
                           (unsigned)tof.strength,
                           (unsigned)tof.frame_count);
    }

    if (n < (int)sizeof(frame) - 512)
    {
        n += std::snprintf(frame + n, sizeof(frame) - n,
                           "+------------------------+--------+----------+--------+----------+----------+\n\n"
                           "+---------+----------+-----------+-----------+-----------+-----------+-----------+-----------+\n"
                           "| IMU     | age_ms   | roll_deg  | pitch_deg | yaw_deg   | gyro_x    | gyro_y    | gyro_z    |\n"
                           "+---------+----------+-----------+-----------+-----------+-----------+-----------+-----------+\n"
                           "| %-7s | %-8s | %9s | %9s | %9s | %9s | %9s | %9s |\n"
                           "+---------+----------+-----------+-----------+-----------+-----------+-----------+-----------+\n"
                           "| IMU     | accel_x  | accel_y   | accel_z   | units                                  |\n"
                           "+---------+----------+-----------+-----------+----------------------------------------+\n"
                           "| %-7s | %8s | %9s | %9s | gyro=rad/s, accel=m/s^2              |\n"
                           "+---------+----------+-----------+-----------+----------------------------------------+\n",
                           Imu_State_Name(imu),
                           imu_age,
                           roll_deg,
                           pitch_deg,
                           yaw_deg,
                           gyro_x,
                           gyro_y,
                           gyro_z,
                           Imu_State_Name(imu),
                           accel_x,
                           accel_y,
                           accel_z);
    }

    if (n < (int)sizeof(frame))
        n += std::snprintf(frame + n, sizeof(frame) - n, "\n");
    else
        frame[sizeof(frame) - 1] = '\0';

    const size_t len = (n > 0 && n < (int)sizeof(frame)) ?
        static_cast<size_t>(n) :
        sizeof(frame) - 1U;

    bool wrote_file = false;
    if (g_tof_print_stream.is_open())
    {
        g_tof_print_stream.write(frame, len);
        g_tof_print_stream.flush();
        wrote_file = true;
    }

    if (g_tof_print_stdout || !wrote_file)
    {
        std::fwrite(frame, 1, len, stdout);
        std::fflush(stdout);
    }
}

static void Write_ToF_Button_Log(uint32_t tick)
{
    Ensure_ToF_Button_Log_Stream();
    if (!g_tof_button_log_stream.is_open())
        return;

    const Class_Robot::ButtonSnapshot button = robot.Get_Button_Snapshot();

    char button_hex[8];
    std::snprintf(button_hex, sizeof(button_hex), "0x%04X", (unsigned)button.code);

    g_tof_button_log_stream
        << tick << ','
        << (static_cast<double>(tick) * 0.001) << ','
        << (unsigned)button.code << ','
        << button_hex << ','
        << Button_Name(button.code) << ','
        << (button.has_buttons ? 1 : 0) << ','
        << (button.recent ? 1 : 0) << ','
        << button.age_ms << ','
        << (int)robot.Lift.Get_Stair_State() << ','
        << robot.Lift.Get_Stair_State_Name() << ','
        << robot.Lift.Get_Stair_Chassis_Forward();

    for (int i = 0; i < CHARIOT_LIFT_TOF_NUM; ++i)
    {
        const auto sensor = static_cast<Enum_Chariot_Lift_ToF_Sensor>(i);
        const ChariotLiftToFData &tof = robot.Lift.Get_ToF_Data(sensor);
        char range_m[24];
        Format_Csv_Float(tof.range_m, range_m, sizeof(range_m));

        g_tof_button_log_stream
            << ',' << (tof.online ? 1 : 0)
            << ',' << (tof.valid ? 1 : 0)
            << ',' << (unsigned)tof.distance_cm
            << ',' << range_m
            << ',' << (unsigned)tof.strength
            << ',' << (unsigned)tof.frame_count;
    }

    g_tof_button_log_stream << '\n';

    ++g_tof_button_log_rows_since_flush;
    if (g_tof_button_log_rows_since_flush >= 50U)
    {
        g_tof_button_log_stream.flush();
        g_tof_button_log_rows_since_flush = 0U;
    }
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
        robot.Auxiliary_Motor.CAN_Send_Exit();
        // 软件级状态字也置 DISABLE，防止 1ms 回调再次入队 MIT 帧
        robot.Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);
        robot.Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_DISABLE);
        robot.Auxiliary_Motor.Set_Control_Status(Motor_DM_Status_DISABLE);

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
    if (g_enable_tof_button_log &&
        tick != 0 &&
        (tick % g_tof_button_log_period_ms) == 0)
    {
        Write_ToF_Button_Log(tick);
    }

    if (g_enable_tof_print && tick != 0 && (tick % kToFPrintPeriodMs) == 0)
        Print_ToF_Terminal_Data();

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

static void Flush_Diagnostic_Streams()
{
    if (g_tof_button_log_stream.is_open())
        g_tof_button_log_stream.flush();
    if (g_tof_print_stream.is_open())
        g_tof_print_stream.flush();
    if (g_var_data_stream.is_open())
        g_var_data_stream.flush();
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
    Flush_Diagnostic_Streams();
    Disable_All_Devices();
    Flush_Diagnostic_Streams();
    robot.Stop_ROS2_Bridge();
}

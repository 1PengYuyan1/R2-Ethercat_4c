#include "task_terminal_diagnostics.h"

#include "can_terminal_printer.h"
#include "math.h"
#include "robot.h"

#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

namespace
{
constexpr int kWheelCount = 4;
constexpr uint32_t kCanStatPrintPeriodMs = 5;
constexpr uint32_t kLiveDashboardPeriodMs = 5;
constexpr uint32_t kToFPrintPeriodMs = 5;
constexpr uint32_t kDefaultToFButtonLogPeriodMs = 5;
constexpr float kLiftMotorToRodRatio = 3.0f;

bool g_enable_can_stat_print = false;
bool g_enable_live_dashboard = false;
bool g_enable_tof_print = true;
bool g_tof_print_stdout = false;
bool g_enable_tof_button_log = true;
uint32_t g_tof_button_log_period_ms = kDefaultToFButtonLogPeriodMs;

constexpr const char *kDefaultVarDataFile = "var_data/terminal/live_variables.log";
constexpr const char *kDefaultToFPrintFile = "var_data/terminal/ops_terminal.log";

std::ofstream g_var_data_stream;
bool g_var_data_stream_inited = false;
std::ofstream g_tof_print_stream;
bool g_tof_print_stream_inited = false;
std::ofstream g_tof_button_log_stream;
bool g_tof_button_log_stream_inited = false;
std::string g_tof_button_log_path;
uint32_t g_tof_button_log_rows_since_flush = 0;

bool EnsureDirectory(const char *path)
{
    if (::mkdir(path, 0775) == 0)
        return true;
    return errno == EEXIST;
}

void EnsureDefaultDirectories()
{
    EnsureDirectory("var_data");
    EnsureDirectory("var_data/terminal");
    EnsureDirectory("var_data/tof");
    EnsureDirectory("var_data/chassis_trace");
    EnsureDirectory("var_data/omni");
    EnsureDirectory("var_data/lift");
    EnsureDirectory("var_data/calibration");
}

const char *LiftToFName(int index)
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

const char *LiftToFTopic(int index)
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

const char *ButtonName(uint16_t code)
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

void FormatRange(float range_m, char *buf, size_t len)
{
    if (std::isnan(range_m))
        std::snprintf(buf, len, "nan");
    else if (!std::isfinite(range_m))
        std::snprintf(buf, len, "%sinf", std::signbit(range_m) ? "-" : "+");
    else
        std::snprintf(buf, len, "%.3fm", range_m);
}

void FormatCsvFloat(float value, char *buf, size_t len)
{
    if (std::isnan(value))
        std::snprintf(buf, len, "nan");
    else if (!std::isfinite(value))
        std::snprintf(buf, len, "%sinf", std::signbit(value) ? "-" : "+");
    else
        std::snprintf(buf, len, "%.5f", value);
}

void FormatTableFloat(bool valid,
                      float value,
                      float scale,
                      int precision,
                      char *buf,
                      size_t len)
{
    if (!valid)
        std::snprintf(buf, len, "no_data");
    else if (std::isnan(value))
        std::snprintf(buf, len, "nan");
    else if (!std::isfinite(value))
        std::snprintf(buf, len, "%sinf", std::signbit(value) ? "-" : "+");
    else
        std::snprintf(buf, len, "%.*f", precision, value * scale);
}

const char *ImuStateName(const Class_Chariot_Imu_Heading_Hold::Snapshot &imu)
{
    if (!imu.valid)
        return "NO_DATA";
    return imu.fresh ? "OK" : "STALE";
}

const char *LiftModuleName(int index)
{
    return (index == CHARIOT_LIFT_MODULE_FRONT) ? "front" : "rear";
}

uint8_t LiftModuleCanChannel(int index)
{
    return (index == CHARIOT_LIFT_MODULE_FRONT) ? 0U : 1U;
}

const char *DmStatusName(Enum_Motor_DM_Status status)
{
    return (status == Motor_DM_Status_ENABLE) ? "ONLINE" : "OFFLINE";
}

const char *DmControlStatusName(Enum_Motor_DM_Control_Status_Normal status)
{
    switch (status)
    {
    case Motor_DM_Control_Status_DISABLE: return "DISABLE";
    case Motor_DM_Control_Status_ENABLE: return "ENABLE";
    case Motor_DM_Control_Status_UNDERVOLTAGE: return "UNDERVOLT";
    case Motor_DM_Control_Status_OVERCURRENT: return "OVERCUR";
    case Motor_DM_Control_Status_MOS_OVERTEMPERATURE: return "MOS_OT";
    case Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE: return "ROTOR_OT";
    case Motor_DM_Control_Status_LOSE_CONNECTION: return "LOST";
    case Motor_DM_Control_Status_MOS_OVERLOAD: return "MOS_OL";
    case Motor_DM_Control_Status_OVERVOLTAGE: return "OVERVOLT";
    default: return "UNKNOWN";
    }
}

std::string MakeDefaultToFButtonLogPath()
{
    std::time_t now = std::time(nullptr);
    std::tm tm_now {};
    localtime_r(&now, &tm_now);

    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm_now);
    return std::string("var_data/tof/tof_button_log_") + ts + ".csv";
}

void EnsureVarDataStream()
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

void EnsureToFPrintStream()
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

void EnsureToFButtonLogStream()
{
    if (g_tof_button_log_stream_inited)
        return;

    const char *path = std::getenv("TOF_BUTTON_LOG_FILE");
    g_tof_button_log_path =
        (path != nullptr && path[0] != '\0') ? std::string(path) :
        MakeDefaultToFButtonLogPath();

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
        const char *name = LiftToFName(i);
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

void PrintLiveDashboard(Class_Robot &robot)
{
    EnsureVarDataStream();

    constexpr size_t kBufSize = 8192;
    char buf[kBufSize];
    int n = 0;

    n += std::snprintf(buf + n, kBufSize - n,
                       "\033[2J\033[H[LIVE-DASHBOARD] refresh=%ums\n",
                       static_cast<unsigned>(kLiveDashboardPeriodMs));

    n += std::snprintf(buf + n, kBufSize - n,
                       "\n[CHASSIS]\n  ctrl=%d tgt_vx=%.3f tgt_vy=%.3f tgt_wz=%.3f"
                       " now_vx=%.3f now_vy=%.3f now_wz=%.3f\n",
                       static_cast<int>(robot.Chassis.Get_Chassis_Control_Type()),
                       robot.Chassis.Get_Target_Velocity_X(),
                       robot.Chassis.Get_Target_Velocity_Y(),
                       robot.Chassis.Get_Target_Omega(),
                       robot.Chassis.Get_Now_Velocity_X(),
                       robot.Chassis.Get_Now_Velocity_Y(),
                       robot.Chassis.Get_Now_Omega());

    n += std::snprintf(buf + n, kBufSize - n, "\n[OMNI-WHEEL-DM3519]\n");
    for (int i = 0; i < kWheelCount && n < static_cast<int>(kBufSize) - 256; ++i)
    {
        auto &dm = robot.Chassis.Motor_Wheel[i];
        n += std::snprintf(buf + n, kBufSize - n,
                           "  W%d tx=0x%02X rx=0x%02X omega=%+8.3f torque=%+5.2f"
                           " T_mos=%3.0f T_rot=%3.0f status=%d ctrlst=%d\n",
                           i,
                           static_cast<unsigned>(dm.DM_CAN_Tx_ID),
                           static_cast<unsigned>(dm.DM_CAN_Rx_ID),
                           dm.Get_Now_Omega(),
                           dm.Get_Now_Torque(),
                           dm.Get_Now_MOS_Temperature(),
                           dm.Get_Now_Rotor_Temperature(),
                           static_cast<int>(dm.Get_Status()),
                           static_cast<int>(dm.Get_Now_Control_Status()));
    }

    if (n < static_cast<int>(kBufSize) - 768)
    {
        n += std::snprintf(buf + n, kBufSize - n,
                           "\n[LIFT]\n  ctrl=%d diff=%d diff_cmd=(%.3f, %.3f)"
                           " stair=%s(%d) stair_chassis=%.3f\n",
                           static_cast<int>(robot.Lift.Get_Control_Type()),
                           robot.Lift.Get_Diff_Drive_Enable() ? 1 : 0,
                           robot.Lift.Get_Target_Diff_Forward(),
                           robot.Lift.Get_Target_Diff_Yaw(),
                           robot.Lift.Get_Stair_State_Name(),
                           static_cast<int>(robot.Lift.Get_Stair_State()),
                           robot.Lift.Get_Stair_Chassis_Forward());
        for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM &&
                        n < static_cast<int>(kBufSize) - 256; ++i)
        {
            auto &dl = robot.Lift.Motor_Drive_Left[i];
            auto &dr = robot.Lift.Motor_Drive_Right[i];
            auto &lm = robot.Lift.Motor_Lift[i];
            n += std::snprintf(buf + n, kBufSize - n,
                               "  %s DL tx=0x%02X w=%+7.3f DR tx=0x%02X w=%+7.3f"
                               " LIFT tx=0x%02X rad=%+7.3f alive=(%d,%d,%d) ctrlst=(%d,%d,%d)\n",
                               (i == CHARIOT_LIFT_MODULE_FRONT) ? "FRONT" : "REAR ",
                               static_cast<unsigned>(dl.DM_CAN_Tx_ID),
                               dl.Get_Now_Omega(),
                               static_cast<unsigned>(dr.DM_CAN_Tx_ID),
                               dr.Get_Now_Omega(),
                               static_cast<unsigned>(lm.DM_CAN_Tx_ID),
                               lm.Get_Now_Radian(),
                               static_cast<int>(dl.Get_Status()),
                               static_cast<int>(dr.Get_Status()),
                               static_cast<int>(lm.Get_Status()),
                               static_cast<int>(dl.Get_Now_Control_Status()),
                               static_cast<int>(dr.Get_Now_Control_Status()),
                               static_cast<int>(lm.Get_Now_Control_Status()));
        }
    }

    if (n < static_cast<int>(kBufSize) - 512)
    {
        n += std::snprintf(buf + n, kBufSize - n, "\n[LIFT-TOF]\n");
        for (int i = 0; i < CHARIOT_LIFT_TOF_NUM &&
                        n < static_cast<int>(kBufSize) - 128; ++i)
        {
            const auto sensor = static_cast<Enum_Chariot_Lift_ToF_Sensor>(i);
            const ChariotLiftToFData &tof = robot.Lift.Get_ToF_Data(sensor);
            char range_buf[24];
            FormatRange(tof.range_m, range_buf, sizeof(range_buf));
            n += std::snprintf(buf + n, kBufSize - n,
                               "  %-10s %-22s %s range=%s raw=%ucm strength=%u frames=%u\n",
                               LiftToFName(i),
                               LiftToFTopic(i),
                               tof.online ? (tof.valid ? "OK " : "BAD") : "OFF",
                               range_buf,
                               static_cast<unsigned>(tof.distance_cm),
                               static_cast<unsigned>(tof.strength),
                               static_cast<unsigned>(tof.frame_count));
        }
    }

    if (n > 0)
    {
        std::fwrite(buf, 1, static_cast<size_t>(n), stdout);
        std::fflush(stdout);
        if (g_var_data_stream.is_open())
        {
            g_var_data_stream.write(buf, n);
            g_var_data_stream.flush();
        }
    }
}

void PrintToFTerminalData(Class_Robot &robot)
{
    EnsureToFPrintStream();

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

    FormatTableFloat(imu.valid, imu.roll_rad, RAD_TO_DEG, 2, roll_deg, sizeof(roll_deg));
    FormatTableFloat(imu.valid, imu.pitch_rad, RAD_TO_DEG, 2, pitch_deg, sizeof(pitch_deg));
    FormatTableFloat(imu.valid, imu.yaw_rad, RAD_TO_DEG, 2, yaw_deg, sizeof(yaw_deg));
    FormatTableFloat(imu.valid, imu.angular_velocity_x, 1.0f, 3, gyro_x, sizeof(gyro_x));
    FormatTableFloat(imu.valid, imu.angular_velocity_y, 1.0f, 3, gyro_y, sizeof(gyro_y));
    FormatTableFloat(imu.valid, imu.angular_velocity_z, 1.0f, 3, gyro_z, sizeof(gyro_z));
    FormatTableFloat(imu.valid, imu.linear_acceleration_x, 1.0f, 3, accel_x, sizeof(accel_x));
    FormatTableFloat(imu.valid, imu.linear_acceleration_y, 1.0f, 3, accel_y, sizeof(accel_y));
    FormatTableFloat(imu.valid, imu.linear_acceleration_z, 1.0f, 3, accel_z, sizeof(accel_z));
    if (imu.valid)
        std::snprintf(imu_age, sizeof(imu_age), "%lld", static_cast<long long>(imu.age_ms));
    else
        std::snprintf(imu_age, sizeof(imu_age), "no_data");

    char frame[8192];
    int n = std::snprintf(frame, sizeof(frame),
                          "\033[2J\033[H[LIFT-TOF-IMU] refresh=%ums (200Hz)\n\n"
                          "+------------+------------------------+--------+----------+--------+----------+----------+\n"
                          "| ToF        | topic                  | status | range    | raw_cm | strength | frames   |\n"
                          "+------------+------------------------+--------+----------+--------+----------+----------+\n",
                          static_cast<unsigned>(kToFPrintPeriodMs));

    for (int i = 0; i < CHARIOT_LIFT_TOF_NUM && n < static_cast<int>(sizeof(frame)) - 192; ++i)
    {
        const auto sensor = static_cast<Enum_Chariot_Lift_ToF_Sensor>(i);
        const ChariotLiftToFData &tof = robot.Lift.Get_ToF_Data(sensor);
        char range_buf[24];
        FormatRange(tof.range_m, range_buf, sizeof(range_buf));

        const char *value = tof.online ? range_buf : "no_data";
        const char *status = tof.online ? (tof.valid ? "OK" : "BAD") : "OFF";
        n += std::snprintf(frame + n, sizeof(frame) - n,
                           "| %-10s | %-22s | %-6s | %-8s | %6u | %8u | %8u |\n",
                           LiftToFName(i),
                           LiftToFTopic(i),
                           status,
                           value,
                           static_cast<unsigned>(tof.distance_cm),
                           static_cast<unsigned>(tof.strength),
                           static_cast<unsigned>(tof.frame_count));
    }

    if (n < static_cast<int>(sizeof(frame)) - 512)
    {
        n += std::snprintf(frame + n, sizeof(frame) - n,
                           "+------------+------------------------+--------+----------+--------+----------+----------+\n"
                           "ToF topics: /high/up_front/range, /high/down_front/range, "
                           "/high/up_back/range, /high/down_back/range\n\n"
                           "+--------+----+-------------+---------+----------+\n"
                           "| Lift   | ch | ids(rx/tx)  | online  | ctrl     |\n"
                           "+--------+----+-------------+---------+----------+\n");

        for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM &&
                        n < static_cast<int>(sizeof(frame)) - 160; ++i)
        {
            auto &motor = robot.Lift.Motor_Lift[i];

            n += std::snprintf(frame + n, sizeof(frame) - n,
                               "| %-6s | %2u | 0x%02X/0x%02X   | %-7s | %-8s |\n",
                               LiftModuleName(i),
                               static_cast<unsigned>(LiftModuleCanChannel(i)),
                               static_cast<unsigned>(motor.DM_CAN_Rx_ID),
                               static_cast<unsigned>(motor.DM_CAN_Tx_ID),
                               DmStatusName(motor.Get_Status()),
                               DmControlStatusName(motor.Get_Now_Control_Status()));
        }
    }

    if (n < static_cast<int>(sizeof(frame)) - 768)
    {
        n += std::snprintf(frame + n, sizeof(frame) - n,
                           "+--------+----+-------------+---------+----------+\n\n"
                           "+--------+-----------+-----------+-----------+-----------+----------+\n"
                           "| Lift   | mpos_rad  | mvel_rad  | rpos_rad  | rvel_rad  | torqueNm |\n"
                           "+--------+-----------+-----------+-----------+-----------+----------+\n");

        for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM &&
                        n < static_cast<int>(sizeof(frame)) - 192; ++i)
        {
            auto &motor = robot.Lift.Motor_Lift[i];
            const float motor_pos = motor.Get_Now_Radian();
            const float motor_vel = motor.Get_Now_Omega();
            const float rod_pos = motor_pos / kLiftMotorToRodRatio;
            const float rod_vel = motor_vel / kLiftMotorToRodRatio;

            n += std::snprintf(frame + n, sizeof(frame) - n,
                               "| %-6s | %9.3f | %9.3f | %9.3f | %9.3f | %8.3f |\n",
                               LiftModuleName(i),
                               motor_pos,
                               motor_vel,
                               rod_pos,
                               rod_vel,
                               motor.Get_Now_Torque());
        }

        n += std::snprintf(frame + n, sizeof(frame) - n,
                           "+--------+-----------+-----------+-----------+-----------+----------+\n"
                           "Lift target: CAN0/CAN1 tx=0x05 rx=0x15, rod=motor/3\n\n"
                           "+---------+----------+-----------+-----------+-----------+\n"
                           "| IMU     | age_ms   | roll_deg  | pitch_deg | yaw_deg   |\n"
                           "+---------+----------+-----------+-----------+-----------+\n"
                           "| %-7s | %-8s | %9s | %9s | %9s |\n"
                           "+---------+----------+-----------+-----------+-----------+\n\n"
                           "+---------+-----------+-----------+-----------+----------+\n"
                           "| IMU     | gyro_x    | gyro_y    | gyro_z    | units    |\n"
                           "+---------+-----------+-----------+-----------+----------+\n"
                           "| %-7s | %9s | %9s | %9s | rad/s    |\n"
                           "+---------+-----------+-----------+-----------+----------+\n\n"
                           "+---------+-----------+-----------+-----------+----------+\n"
                           "| IMU     | accel_x   | accel_y   | accel_z   | units    |\n"
                           "+---------+-----------+-----------+-----------+----------+\n"
                           "| %-7s | %9s | %9s | %9s | m/s^2    |\n"
                           "+---------+-----------+-----------+-----------+----------+\n",
                           ImuStateName(imu),
                           imu_age,
                           roll_deg,
                           pitch_deg,
                           yaw_deg,
                           ImuStateName(imu),
                           gyro_x,
                           gyro_y,
                           gyro_z,
                           ImuStateName(imu),
                           accel_x,
                           accel_y,
                           accel_z);
    }

    if (n < static_cast<int>(sizeof(frame)))
        n += std::snprintf(frame + n, sizeof(frame) - n, "\n");
    else
        frame[sizeof(frame) - 1] = '\0';

    const size_t len = (n > 0 && n < static_cast<int>(sizeof(frame))) ?
        static_cast<size_t>(n) :
        sizeof(frame) - 1U;

    bool wrote_file = false;
    if (!g_tof_print_stdout && g_tof_print_stream.is_open())
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

void WriteToFButtonLog(uint32_t tick, Class_Robot &robot)
{
    EnsureToFButtonLogStream();
    if (!g_tof_button_log_stream.is_open())
        return;

    const Class_Robot::ButtonSnapshot button = robot.Get_Button_Snapshot();

    char button_hex[8];
    std::snprintf(button_hex, sizeof(button_hex), "0x%04X", static_cast<unsigned>(button.code));

    g_tof_button_log_stream
        << tick << ','
        << (static_cast<double>(tick) * 0.001) << ','
        << static_cast<unsigned>(button.code) << ','
        << button_hex << ','
        << ButtonName(button.code) << ','
        << (button.has_buttons ? 1 : 0) << ','
        << (button.recent ? 1 : 0) << ','
        << button.age_ms << ','
        << static_cast<int>(robot.Lift.Get_Stair_State()) << ','
        << robot.Lift.Get_Stair_State_Name() << ','
        << robot.Lift.Get_Stair_Chassis_Forward();

    for (int i = 0; i < CHARIOT_LIFT_TOF_NUM; ++i)
    {
        const auto sensor = static_cast<Enum_Chariot_Lift_ToF_Sensor>(i);
        const ChariotLiftToFData &tof = robot.Lift.Get_ToF_Data(sensor);
        char range_m[24];
        FormatCsvFloat(tof.range_m, range_m, sizeof(range_m));

        g_tof_button_log_stream
            << ',' << (tof.online ? 1 : 0)
            << ',' << (tof.valid ? 1 : 0)
            << ',' << static_cast<unsigned>(tof.distance_cm)
            << ',' << range_m
            << ',' << static_cast<unsigned>(tof.strength)
            << ',' << static_cast<unsigned>(tof.frame_count);
    }

    g_tof_button_log_stream << '\n';

    ++g_tof_button_log_rows_since_flush;
    if (g_tof_button_log_rows_since_flush >= 50U)
    {
        g_tof_button_log_stream.flush();
        g_tof_button_log_rows_since_flush = 0U;
    }
}
} // namespace

void task_terminal::ParseEnvFlags()
{
    EnsureDefaultDirectories();

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

void task_terminal::DispatchPeriodic(uint32_t tick, Class_Robot &robot, linkx_t *linkx)
{
    if (g_enable_tof_button_log &&
        tick != 0 &&
        (tick % g_tof_button_log_period_ms) == 0)
    {
        WriteToFButtonLog(tick, robot);
    }

    if (g_enable_tof_print && tick != 0 && (tick % kToFPrintPeriodMs) == 0)
        PrintToFTerminalData(robot);

    if (g_enable_can_stat_print && tick != 0 && (tick % kCanStatPrintPeriodMs) == 0)
    {
        std::printf("\n[CAN-STATS] === LinkX (slave_id=1) ===");
        can_terminal::PrintStats(linkx);
    }

    if (g_enable_live_dashboard && (tick % kLiveDashboardPeriodMs) == 0)
        PrintLiveDashboard(robot);
}

void task_terminal::FlushStreams()
{
    if (g_tof_button_log_stream.is_open())
        g_tof_button_log_stream.flush();
    if (g_tof_print_stream.is_open())
        g_tof_print_stream.flush();
    if (g_var_data_stream.is_open())
        g_var_data_stream.flush();
}

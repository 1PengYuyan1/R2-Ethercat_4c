// lift_3519_feedback_main.cpp
//
// Terminal feedback monitor for the lift DM3519 motors.
//
// Hardware mapping follows Class_Chariot_Lift:
//   front lift: LinkX CAN0, DM3519 Tx 0x05, feedback Rx 0x15
//   rear  lift: LinkX CAN1, DM3519 Tx 0x05, feedback Rx 0x15
//
// Example:
//   sudo IFNAME=enp86s0 build/linkx_soem_demo/lift_3519_feedback --module both
//   sudo IFNAME=enp86s0 build/linkx_soem_demo/lift_3519_feedback --module front --duration 10

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

#include <sys/stat.h>
#include <sys/types.h>

#include "crt_lift.h"
#include "dvc_motor_dm.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"

namespace
{
constexpr int kChannelCount = 4;
constexpr uint32_t kControlPeriodMs = 1;
constexpr uint32_t kAlivePeriodMs = 100;
constexpr float kLiftMotorToRodRatio = 3.0f;
constexpr float kRadPerSecToRpm = 60.0f / (2.0f * 3.14159265358979323846f);
constexpr float kCelsiusToKelvin = 273.15f;

struct ModuleInfo
{
    Enum_Chariot_Lift_Module module;
    const char *name;
    uint8_t can_channel;
};

enum class ModuleSelection
{
    Rear,
    Front,
    Both,
};

const std::array<ModuleInfo, CHARIOT_LIFT_MODULE_NUM> kModuleInfo {{
    {CHARIOT_LIFT_MODULE_FRONT, "front", 0},
    {CHARIOT_LIFT_MODULE_REAR,  "rear",  1},
}};

struct LiftFeedbackSample
{
    const ModuleInfo *info;
    uint16_t rx_id;
    uint16_t tx_id;
    const char *online_text;
    const char *ctrl_text;
    float motor_pos_rad;
    float motor_vel_rad_s;
    float motor_rpm;
    float torque_nm;
    float mos_c;
    float rotor_c;
    float rod_pos_rad;
    float rod_vel_rad_s;
    float rod_rpm;
};

ecat_master_t st_master {};
linkx_t st_linkx {};
Class_Chariot_Lift st_lift;
std::atomic<bool> st_running {true};

void on_signal(int)
{
    st_running.store(false);
}

bool has_arg(int argc, char **argv, const std::string &key)
{
    const std::string flag = "--" + key;
    for (int i = 1; i < argc; ++i)
    {
        if (argv[i] == flag)
            return true;
    }
    return false;
}

std::string cli_value(int argc, char **argv, const std::string &key, const std::string &fallback)
{
    const std::string flag = "--" + key;
    const std::string prefix = flag + "=";

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == flag && (i + 1) < argc)
            return argv[i + 1];
        if (arg.compare(0, prefix.size(), prefix) == 0)
            return arg.substr(prefix.size());
    }

    return fallback;
}

bool parse_bool_text(const std::string &value, bool fallback)
{
    if (value == "1" || value == "true" || value == "TRUE" ||
        value == "yes" || value == "YES" || value == "on" || value == "ON")
        return true;
    if (value == "0" || value == "false" || value == "FALSE" ||
        value == "no" || value == "NO" || value == "off" || value == "OFF")
        return false;
    return fallback;
}

bool cli_bool(int argc, char **argv, const std::string &key, const char *env_name, bool fallback)
{
    const std::string flag = "--" + key;
    const std::string no_flag = "--no-" + key;
    const std::string prefix = flag + "=";

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == no_flag)
            return false;
        if (arg == flag)
        {
            if ((i + 1) < argc)
            {
                const std::string next = argv[i + 1];
                if (next.compare(0, 2, "--") != 0)
                    return parse_bool_text(next, true);
            }
            return true;
        }
        if (arg.compare(0, prefix.size(), prefix) == 0)
            return parse_bool_text(arg.substr(prefix.size()), fallback);
    }

    const char *env = std::getenv(env_name);
    return (env != nullptr && env[0] != '\0') ? parse_bool_text(env, fallback) : fallback;
}

float parse_float(const std::string &value, float fallback)
{
    char *end = nullptr;
    const float parsed = std::strtof(value.c_str(), &end);
    return (end == value.c_str()) ? fallback : parsed;
}

ModuleSelection parse_module_selection(const std::string &value, bool &ok)
{
    ok = true;
    if (value == "rear" || value == "0")
        return ModuleSelection::Rear;
    if (value == "front" || value == "1")
        return ModuleSelection::Front;
    if (value == "both" || value == "all")
        return ModuleSelection::Both;

    ok = false;
    return ModuleSelection::Both;
}

bool module_selected(ModuleSelection selection, Enum_Chariot_Lift_Module module)
{
    return selection == ModuleSelection::Both ||
           (selection == ModuleSelection::Rear && module == CHARIOT_LIFT_MODULE_REAR) ||
           (selection == ModuleSelection::Front && module == CHARIOT_LIFT_MODULE_FRONT);
}

Class_Motor_DM_Normal &lift_motor(Enum_Chariot_Lift_Module module)
{
    return st_lift.Motor_Lift[static_cast<int>(module)];
}

const char *dm_status_name(Enum_Motor_DM_Status status)
{
    return (status == Motor_DM_Status_ENABLE) ? "ONLINE" : "OFFLINE";
}

const char *dm_control_status_name(Enum_Motor_DM_Control_Status_Normal status)
{
    switch (status)
    {
        case Motor_DM_Control_Status_DISABLE:              return "DISABLE";
        case Motor_DM_Control_Status_ENABLE:               return "ENABLE";
        case Motor_DM_Control_Status_UNDERVOLTAGE:         return "UNDERVOLT";
        case Motor_DM_Control_Status_OVERCURRENT:          return "OVERCUR";
        case Motor_DM_Control_Status_MOS_OVERTEMPERATURE:  return "MOS_OT";
        case Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE:return "ROTOR_OT";
        case Motor_DM_Control_Status_LOSE_CONNECTION:      return "LOST";
        case Motor_DM_Control_Status_MOS_OVERLOAD:         return "MOS_OL";
        case Motor_DM_Control_Status_OVERVOLTAGE:          return "OVERVOLT";
        default:                                           return "UNKNOWN";
    }
}

float temp_k_to_c_or_nan(float temp_k)
{
    if (temp_k < 200.0f)
        return std::numeric_limits<float>::quiet_NaN();
    return temp_k - kCelsiusToKelvin;
}

std::string read_first_token(const std::string &path)
{
    std::ifstream file(path);
    std::string value;
    file >> value;
    return value;
}

void print_interface_preflight(const std::string &ifname)
{
    const std::string base = "/sys/class/net/" + ifname;
    const std::string operstate = read_first_token(base + "/operstate");
    const std::string carrier = read_first_token(base + "/carrier");

    if (operstate.empty())
    {
        std::cerr << "[LIFT3519][NET] interface '" << ifname
                  << "' not found in /sys/class/net. Check --ifname.\n";
        return;
    }

    std::cout << "[LIFT3519][NET] " << ifname
              << " operstate=" << operstate;
    if (!carrier.empty())
        std::cout << " carrier=" << carrier;
    std::cout << "\n";

    if (operstate == "down")
    {
        std::cerr << "[LIFT3519][NET][WARN] " << ifname
                  << " is DOWN. Run: sudo ip link set " << ifname << " up\n";
    }
    else if (carrier == "0")
    {
        std::cerr << "[LIFT3519][NET][WARN] " << ifname
                  << " has no carrier. Check LinkX power, EtherCAT cable, and port.\n";
    }
}

bool ec_receive_once(bool allow_stopping = false)
{
    if ((!allow_stopping && !st_running.load()) || !st_master.is_running)
        return false;

    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);

    can_msg_t msg {};
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
    {
        while (linkx_quick_recv(&st_linkx, ch, &msg))
        {
            st_lift.CAN_Rx_Callback(ch, msg.id, msg.data);
        }
    }

    return true;
}

void alive_selected_lift_motors(ModuleSelection selection)
{
    for (const auto &info : kModuleInfo)
    {
        if (module_selected(selection, info.module))
            lift_motor(info.module).TIM_Alive_PeriodElapsedCallback();
    }
}

void enable_selected_lift_motors(ModuleSelection selection)
{
    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(selection, info.module))
            continue;

        auto &motor = lift_motor(info.module);
        const auto ctrl = motor.Get_Now_Control_Status();

        if (ctrl != Motor_DM_Control_Status_ENABLE)
        {
            motor.CAN_Send_Clear_Error();
            motor.CAN_Send_Enter();
        }
    }
}

void exit_selected_lift_motors(ModuleSelection selection)
{
    for (const auto &info : kModuleInfo)
    {
        if (module_selected(selection, info.module))
            lift_motor(info.module).CAN_Send_Exit();
    }
}

const char *module_selection_name(ModuleSelection selection)
{
    switch (selection)
    {
        case ModuleSelection::Rear:  return "rear";
        case ModuleSelection::Front: return "front";
        case ModuleSelection::Both:  return "both";
        default:                     return "unknown";
    }
}

std::string hex_id(uint16_t id)
{
    std::ostringstream ss;
    ss << "0x" << std::uppercase << std::hex << id;
    return ss.str();
}

bool ensure_directory_exists(const std::string &dir)
{
    if (dir.empty() || dir == ".")
        return true;

    struct stat st {};
    if (stat(dir.c_str(), &st) == 0)
        return S_ISDIR(st.st_mode);

    const std::size_t slash = dir.find_last_of('/');
    if (slash != std::string::npos && slash > 0)
    {
        if (!ensure_directory_exists(dir.substr(0, slash)))
            return false;
    }

    if (mkdir(dir.c_str(), 0755) == 0)
        return true;

    if (errno == EEXIST && stat(dir.c_str(), &st) == 0)
        return S_ISDIR(st.st_mode);

    return false;
}

bool ensure_parent_directory_exists(const std::string &path)
{
    const std::size_t slash = path.find_last_of('/');
    if (slash == std::string::npos)
        return true;
    if (slash == 0)
        return true;
    return ensure_directory_exists(path.substr(0, slash));
}

LiftFeedbackSample read_lift_feedback_sample(const ModuleInfo &info)
{
    auto &motor = lift_motor(info.module);
    const float motor_pos_rad = motor.Get_Now_Radian();
    const float motor_vel_rad_s = motor.Get_Now_Omega();
    const float rod_pos_rad = motor_pos_rad / kLiftMotorToRodRatio;
    const float rod_vel_rad_s = motor_vel_rad_s / kLiftMotorToRodRatio;

    LiftFeedbackSample sample {};
    sample.info = &info;
    sample.rx_id = motor.DM_CAN_Rx_ID;
    sample.tx_id = motor.DM_CAN_Tx_ID;
    sample.online_text = dm_status_name(motor.Get_Status());
    sample.ctrl_text = dm_control_status_name(motor.Get_Now_Control_Status());
    sample.motor_pos_rad = motor_pos_rad;
    sample.motor_vel_rad_s = motor_vel_rad_s;
    sample.motor_rpm = motor_vel_rad_s * kRadPerSecToRpm;
    sample.torque_nm = motor.Get_Now_Torque();
    sample.mos_c = temp_k_to_c_or_nan(motor.Get_Now_MOS_Temperature());
    sample.rotor_c = temp_k_to_c_or_nan(motor.Get_Now_Rotor_Temperature());
    sample.rod_pos_rad = rod_pos_rad;
    sample.rod_vel_rad_s = rod_vel_rad_s;
    sample.rod_rpm = rod_vel_rad_s * kRadPerSecToRpm;
    return sample;
}

class CsvRecorder
{
public:
    bool Open(const std::string &path)
    {
        if (!ensure_parent_directory_exists(path))
        {
            std::cerr << "[LIFT3519][RECORD] cannot create parent directory for "
                      << path << ": " << std::strerror(errno) << "\n";
            return false;
        }

        file_.open(path, std::ios::out | std::ios::trunc);
        if (!file_.is_open())
        {
            std::cerr << "[LIFT3519][RECORD] cannot open " << path << "\n";
            return false;
        }

        path_ = path;
        rows_ = 0;
        file_ << "time_s,module,ch,rx,tx,online,ctrl,"
              << "mpos_rad,mvel_rad_s,mrpm,torque_nm,mos_c,rotor_c,"
              << "rpos_rad,rvel_rad_s,rrpm\n";
        file_.flush();
        return true;
    }

    void Write(ModuleSelection selection, double elapsed_s)
    {
        if (!file_.is_open())
            return;

        file_ << std::fixed << std::setprecision(6);
        for (const auto &info : kModuleInfo)
        {
            if (!module_selected(selection, info.module))
                continue;

            const LiftFeedbackSample sample = read_lift_feedback_sample(info);
            file_ << elapsed_s << ','
                  << sample.info->name << ','
                  << static_cast<int>(sample.info->can_channel) << ','
                  << "0x" << std::hex << std::uppercase << sample.rx_id << std::dec << ','
                  << "0x" << std::hex << std::uppercase << sample.tx_id << std::dec << ','
                  << sample.online_text << ','
                  << sample.ctrl_text << ','
                  << sample.motor_pos_rad << ','
                  << sample.motor_vel_rad_s << ','
                  << sample.motor_rpm << ','
                  << sample.torque_nm << ','
                  << sample.mos_c << ','
                  << sample.rotor_c << ','
                  << sample.rod_pos_rad << ','
                  << sample.rod_vel_rad_s << ','
                  << sample.rod_rpm << '\n';
            ++rows_;
        }
        file_.flush();
    }

    const std::string &Path() const { return path_; }
    uint64_t Rows() const { return rows_; }
    bool IsOpen() const { return file_.is_open(); }

private:
    std::ofstream file_;
    std::string path_;
    uint64_t rows_ = 0;
};

void print_feedback_table(ModuleSelection selection,
                          double elapsed_s,
                          bool enable_motors,
                          const CsvRecorder *recorder,
                          float record_hz)
{
    std::cout << "\033[H\033[J";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "R2 Lift DM3519 Feedback Monitor"
              << " | t=" << elapsed_s << "s"
              << " | motor:rod=3:1"
              << " | enable_enter=" << (enable_motors ? "ON" : "OFF")
              << " | MIT command frames=OFF"
              << "\n\n";

    std::cout << std::left
              << std::setw(7)  << "Module"
              << std::setw(4)  << "Ch"
              << std::setw(6)  << "Rx"
              << std::setw(6)  << "Tx"
              << std::setw(8)  << "Online"
              << std::setw(10) << "Ctrl"
              << std::right
              << std::setw(10) << "MPos"
              << std::setw(10) << "MVel"
              << std::setw(10) << "MRPM"
              << std::setw(10) << "Torque"
              << std::setw(9)  << "MOS"
              << std::setw(9)  << "Rotor"
              << std::setw(10) << "RPos"
              << std::setw(10) << "RVel"
              << std::setw(10) << "RRPM"
              << "\n";

    std::cout << std::string(129, '-') << "\n";

    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(selection, info.module))
            continue;

        const LiftFeedbackSample sample = read_lift_feedback_sample(info);

        std::cout << std::left
                  << std::setw(7)  << sample.info->name
                  << std::setw(4)  << static_cast<int>(sample.info->can_channel)
                  << std::setw(6)  << hex_id(sample.rx_id)
                  << std::setw(6)  << hex_id(sample.tx_id)
                  << std::setw(8)  << sample.online_text
                  << std::setw(10) << sample.ctrl_text
                  << std::right
                  << std::setw(10) << sample.motor_pos_rad
                  << std::setw(10) << sample.motor_vel_rad_s
                  << std::setw(10) << sample.motor_rpm
                  << std::setw(10) << sample.torque_nm
                  << std::setw(9)  << sample.mos_c
                  << std::setw(9)  << sample.rotor_c
                  << std::setw(10) << sample.rod_pos_rad
                  << std::setw(10) << sample.rod_vel_rad_s
                  << std::setw(10) << sample.rod_rpm
                  << "\n";
    }

    std::cout << "\nUnits: MPos/RPos=rad, MVel/RVel=rad/s, MRPM/RRPM=rpm, Torque=Nm, MOS/Rotor=C\n";
    std::cout << "Limits: pmax=" << lift_motor(CHARIOT_LIFT_MODULE_FRONT).Get_Radian_Max()
              << " rad, vmax=" << lift_motor(CHARIOT_LIFT_MODULE_FRONT).Get_Omega_Max()
              << " rad/s, tmax=" << lift_motor(CHARIOT_LIFT_MODULE_FRONT).Get_Torque_Max()
              << " Nm, imax=" << lift_motor(CHARIOT_LIFT_MODULE_FRONT).Get_Current_Max()
              << " A\n";
    if (recorder != nullptr && recorder->IsOpen())
    {
        std::cout << "Record: " << recorder->Path()
                  << " @ " << record_hz << " Hz, rows=" << recorder->Rows()
                  << "\n";
    }
    std::cout << "Ctrl-C to quit. Exit command is sent on stop when --exit-on-stop=1.\n";
    std::cout.flush();
}

void print_usage(const char *argv0)
{
    std::cerr
        << "Usage:\n"
        << "  " << argv0 << " [--ifname enp86s0] [--module rear|front|both]\n"
        << "        [--print-hz 10] [--duration 0] [--enable 1] [--exit-on-stop 1]\n"
        << "        [--record 0|1] [--record-path var_data/lift_3519_feedback_log.csv]\n"
        << "        [--record-hz 50]\n\n"
        << "Notes:\n"
        << "  duration=0 keeps running until Ctrl-C.\n"
        << "  enable=1 sends DM clear-error/enter frames so feedback is live.\n"
        << "  It does not send MIT position/velocity/torque command frames.\n"
        << "  rod feedback is calculated as rod = motor / 3.\n";
}

bool configure_linkx_can()
{
    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);

    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        // Runtime stack uses CAN-FD+BRS, nominal 1 Mbps and data 5 Mbps.
        if (!linkx_set_can_baudrate(&st_linkx,
                                     static_cast<uint8_t>(ch),
                                     1, 2, 31, 8, 8,
                                     1, 12, 3, 3))
        {
            std::cerr << "[LIFT3519] CAN" << ch << " FDCAN 1M/5M config failed\n";
            return false;
        }
    }

    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);

    return true;
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    if (has_arg(argc, argv, "help") || has_arg(argc, argv, "h"))
    {
        print_usage(argv[0]);
        return 0;
    }

    const std::string ifname = cli_value(
        argc,
        argv,
        "ifname",
        std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp86s0");

    bool module_ok = false;
    const ModuleSelection module_selection = parse_module_selection(
        cli_value(argc,
                  argv,
                  "module",
                  std::getenv("LIFT3519_MODULE") ? std::getenv("LIFT3519_MODULE") : "both"),
        module_ok);
    if (!module_ok)
    {
        std::cerr << "[LIFT3519] invalid --module, expected rear|front|both\n";
        print_usage(argv[0]);
        return 1;
    }

    float print_hz = parse_float(
        cli_value(argc,
                  argv,
                  "print-hz",
                  std::getenv("LIFT3519_PRINT_HZ") ? std::getenv("LIFT3519_PRINT_HZ") : "10"),
        10.0f);
    print_hz = std::clamp(print_hz, 0.5f, 100.0f);

    const float duration_s = std::max(
        0.0f,
        parse_float(cli_value(argc,
                              argv,
                              "duration",
                              std::getenv("LIFT3519_DURATION") ? std::getenv("LIFT3519_DURATION") : "0"),
                    0.0f));
    const bool enable_motors = cli_bool(argc, argv, "enable", "LIFT3519_ENABLE", true);
    const bool exit_on_stop = cli_bool(argc, argv, "exit-on-stop", "LIFT3519_EXIT_ON_STOP", true);
    const bool record_enabled = cli_bool(argc, argv, "record", "LIFT3519_RECORD", false);
    const std::string record_path = cli_value(
        argc,
        argv,
        "record-path",
        std::getenv("LIFT3519_RECORD_PATH") ? std::getenv("LIFT3519_RECORD_PATH") :
                                              "var_data/lift_3519_feedback_log.csv");
    float record_hz = parse_float(
        cli_value(argc,
                  argv,
                  "record-hz",
                  std::getenv("LIFT3519_RECORD_HZ") ? std::getenv("LIFT3519_RECORD_HZ") : "50"),
        50.0f);
    record_hz = std::clamp(record_hz, 1.0f, 1000.0f);

    std::cout << "===============================================\n"
              << "  R2 Lift DM3519 Feedback Monitor\n"
              << "  IFNAME        : " << ifname << "\n"
              << "  MODULE        : " << module_selection_name(module_selection) << "\n"
              << "  ENABLE        : " << (enable_motors ? "enter only" : "off") << "\n"
              << "  EXIT_ON_STOP  : " << (exit_on_stop ? "1" : "0") << "\n"
              << "  MIT COMMAND   : off\n"
              << "  RECORD        : " << (record_enabled ? "on" : "off") << "\n";
    if (record_enabled)
    {
        std::cout << "  RECORD_PATH   : " << record_path << "\n"
                  << "  RECORD_HZ     : " << record_hz << "\n";
    }
    std::cout << "  GEAR          : motor:rod = 3:1, rod = motor / 3\n"
              << "===============================================\n";

    print_interface_preflight(ifname);

    if (!ecat_master_init(&st_master, ifname.c_str()))
    {
        std::cerr << "[LIFT3519] ecat_master_init failed. If the adapter opened but no slaves were found,"
                  << " first check: sudo ip link set " << ifname
                  << " up, LinkX power, and EtherCAT cable.\n";
        return 2;
    }

    linkx_init(&st_linkx, 1, &st_master.ctx);

    if (!configure_linkx_can())
        return 2;

    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[LIFT3519] ecat_master_bring_online failed\n";
        return 2;
    }

    st_lift.Init(&st_linkx);

    CsvRecorder recorder;
    if (record_enabled && !recorder.Open(record_path))
        return 2;
    if (record_enabled)
        std::cout << "[LIFT3519][RECORD] writing " << record_path << "\n";

    const uint64_t print_period_ticks = std::max<uint64_t>(
        1U,
        static_cast<uint64_t>(std::lround(1000.0f / print_hz)));
    const uint64_t record_period_ticks = std::max<uint64_t>(
        1U,
        static_cast<uint64_t>(std::lround(1000.0f / record_hz)));
    const uint64_t duration_ticks =
        (duration_s > 0.0f) ? static_cast<uint64_t>(duration_s * 1000.0f) : 0U;

    auto next_wakeup = std::chrono::steady_clock::now();
    uint64_t tick = 0;
    std::cout << "\033[?25l";

    while (st_running.load() && st_master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kControlPeriodMs);

        if (!ec_receive_once())
            break;

        if ((tick % kAlivePeriodMs) == 0U)
        {
            alive_selected_lift_motors(module_selection);
            if (enable_motors)
                enable_selected_lift_motors(module_selection);
        }

        linkx_send_pdos(&st_linkx);

        if (record_enabled && (tick % record_period_ticks) == 0U)
            recorder.Write(module_selection, static_cast<double>(tick) * 0.001);

        if ((tick % print_period_ticks) == 0U)
            print_feedback_table(module_selection,
                                 static_cast<double>(tick) * 0.001,
                                 enable_motors,
                                 record_enabled ? &recorder : nullptr,
                                 record_hz);

        if (duration_ticks > 0U && tick >= duration_ticks)
            break;

        ++tick;
        std::this_thread::sleep_until(next_wakeup);
    }

    if (enable_motors && exit_on_stop)
    {
        for (int i = 0; i < 50; ++i)
        {
            if (!ec_receive_once(true))
                break;
            exit_selected_lift_motors(module_selection);
            linkx_send_pdos(&st_linkx);
            std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
        }
    }

    std::cout << "\033[?25h";
    std::cout << "[LIFT3519] done.\n";
    return 0;
}

// lift_3519_readonly_main.cpp
//
// Read-only terminal monitor for the two lift DM3519 motors:
//   front lift: LinkX CAN0, DM3519 Tx 0x05, feedback Rx 0x15
//   rear  lift: LinkX CAN1, DM3519 Tx 0x05, feedback Rx 0x15
//
// This tool does not send DM clear-error, enter, exit, or MIT command frames.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

#include "dvc_motor_dm.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"

namespace
{
constexpr uint32_t kControlPeriodMs = 1;
constexpr uint32_t kAlivePeriodMs = 100;
constexpr float kLiftMotorToRodRatio = 3.0f;
constexpr float kRadPerSecToRpm = 60.0f / (2.0f * 3.14159265358979323846f);
constexpr float kCelsiusToKelvin = 273.15f;
constexpr uint8_t kFeedbackRxId = 0x15;
constexpr uint8_t kCommandTxId = 0x05;
constexpr float kRadianMax = 62.8f;
constexpr float kOmegaMax = 395.0f;
constexpr float kTorqueMax = 7.8f;
constexpr float kCurrentMax = 9.2f;

struct MotorInfo
{
    const char *name;
    uint8_t can_channel;
};

struct Sample
{
    const char *name;
    uint8_t can_channel;
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

constexpr std::array<MotorInfo, 2> kMotorInfo {{
    {"front", 0},
    {"rear", 1},
}};

ecat_master_t st_master {};
linkx_t st_linkx {};
std::array<Class_Motor_DM_Normal, kMotorInfo.size()> st_motors {};
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

float parse_float(const std::string &value, float fallback)
{
    char *end = nullptr;
    const float parsed = std::strtof(value.c_str(), &end);
    return (end == value.c_str()) ? fallback : parsed;
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
        std::cerr << "[LIFT3519-RO][NET] interface '" << ifname
                  << "' not found in /sys/class/net. Check --ifname.\n";
        return;
    }

    std::cout << "[LIFT3519-RO][NET] " << ifname << " operstate=" << operstate;
    if (!carrier.empty())
        std::cout << " carrier=" << carrier;
    std::cout << "\n";

    if (operstate == "down")
    {
        std::cerr << "[LIFT3519-RO][NET][WARN] " << ifname
                  << " is DOWN. Run: sudo ip link set " << ifname << " up\n";
    }
    else if (carrier == "0")
    {
        std::cerr << "[LIFT3519-RO][NET][WARN] " << ifname
                  << " has no carrier. Check LinkX power, EtherCAT cable, and port.\n";
    }
}

const char *dm_status_name(Enum_Motor_DM_Status status)
{
    return (status == Motor_DM_Status_ENABLE) ? "ONLINE" : "OFFLINE";
}

const char *dm_control_status_name(Enum_Motor_DM_Control_Status_Normal status)
{
    switch (status)
    {
        case Motor_DM_Control_Status_DISABLE:               return "DISABLE";
        case Motor_DM_Control_Status_ENABLE:                return "ENABLE";
        case Motor_DM_Control_Status_UNDERVOLTAGE:          return "UNDERVOLT";
        case Motor_DM_Control_Status_OVERCURRENT:           return "OVERCUR";
        case Motor_DM_Control_Status_MOS_OVERTEMPERATURE:   return "MOS_OT";
        case Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE: return "ROTOR_OT";
        case Motor_DM_Control_Status_LOSE_CONNECTION:       return "LOST";
        case Motor_DM_Control_Status_MOS_OVERLOAD:          return "MOS_OL";
        case Motor_DM_Control_Status_OVERVOLTAGE:           return "OVERVOLT";
        default:                                            return "UNKNOWN";
    }
}

float temp_k_to_c_or_nan(float temp_k)
{
    if (temp_k < 200.0f)
        return std::numeric_limits<float>::quiet_NaN();
    return temp_k - kCelsiusToKelvin;
}

std::string hex_id(uint16_t id)
{
    std::ostringstream ss;
    ss << "0x" << std::uppercase << std::hex << id;
    return ss.str();
}

bool configure_linkx_can()
{
    for (const auto &info : kMotorInfo)
        linkx_switch_can_channel(&st_linkx, info.can_channel, true);

    for (const auto &info : kMotorInfo)
    {
        if (!linkx_set_can_baudrate(&st_linkx,
                                     info.can_channel,
                                     1, 2, 31, 8, 8,
                                     1, 12, 3, 3))
        {
            std::cerr << "[LIFT3519-RO] CAN" << static_cast<int>(info.can_channel)
                      << " FDCAN 1M/5M config failed\n";
            return false;
        }
    }

    for (const auto &info : kMotorInfo)
        linkx_switch_can_channel(&st_linkx, info.can_channel, true);

    return true;
}

void init_motors()
{
    for (std::size_t i = 0; i < kMotorInfo.size(); ++i)
    {
        st_motors[i].Init(&st_linkx,
                          kMotorInfo[i].can_channel,
                          kFeedbackRxId,
                          kCommandTxId,
                          Motor_DM_Control_Method_NORMAL_MIT,
                          kRadianMax,
                          kOmegaMax,
                          kTorqueMax,
                          kCurrentMax);
        st_motors[i].Set_Use_FDCAN(true);
        st_motors[i].Set_Force_Output_Without_Feedback(false);
    }
}

void receive_once()
{
    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);

    can_msg_t msg {};
    for (std::size_t i = 0; i < kMotorInfo.size(); ++i)
    {
        const uint8_t ch = kMotorInfo[i].can_channel;
        while (linkx_quick_recv(&st_linkx, ch, &msg))
        {
            const uint32_t can_id_std = msg.id & 0x7FFU;
            if (can_id_std == st_motors[i].DM_CAN_Rx_ID)
                st_motors[i].CAN_RxCpltCallback(msg.data);
        }
    }
}

void update_alive_status()
{
    for (auto &motor : st_motors)
        motor.TIM_Alive_PeriodElapsedCallback();
}

Sample read_sample(std::size_t i)
{
    auto &motor = st_motors[i];
    const float motor_pos_rad = motor.Get_Now_Radian();
    const float motor_vel_rad_s = motor.Get_Now_Omega();
    const float rod_pos_rad = motor_pos_rad / kLiftMotorToRodRatio;
    const float rod_vel_rad_s = motor_vel_rad_s / kLiftMotorToRodRatio;

    Sample sample {};
    sample.name = kMotorInfo[i].name;
    sample.can_channel = kMotorInfo[i].can_channel;
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

void print_table(double elapsed_s)
{
    std::cout << "\033[H\033[J";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "R2 Lift DM3519 Readonly Monitor"
              << " | t=" << elapsed_s << "s"
              << " | target=CAN0/CAN1 Tx 0x05 Rx 0x15"
              << " | sends_dm_frames=NO"
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

    for (std::size_t i = 0; i < kMotorInfo.size(); ++i)
    {
        const Sample sample = read_sample(i);
        std::cout << std::left
                  << std::setw(7)  << sample.name
                  << std::setw(4)  << static_cast<int>(sample.can_channel)
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
    std::cout << "No DM command frames are sent by this tool. Ctrl-C to quit.\n";
}

void print_usage(const char *argv0)
{
    std::cout
        << "Usage:\n"
        << "  " << argv0 << " [--ifname enp4s0] [--duration 0] [--print-hz 10]\n\n"
        << "Reads only:\n"
        << "  front CAN0 Tx 0x05 feedback Rx 0x15\n"
        << "  rear  CAN1 Tx 0x05 feedback Rx 0x15\n\n"
        << "This tool never sends DM clear-error, enter, exit, or MIT command frames.\n";
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
        std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp4s0");

    float print_hz = parse_float(
        cli_value(argc, argv, "print-hz", std::getenv("LIFT3519_RO_PRINT_HZ") ?
                  std::getenv("LIFT3519_RO_PRINT_HZ") : "10"),
        10.0f);
    print_hz = std::clamp(print_hz, 0.5f, 100.0f);

    const float duration_s = std::max(
        0.0f,
        parse_float(cli_value(argc, argv, "duration", std::getenv("LIFT3519_RO_DURATION") ?
                              std::getenv("LIFT3519_RO_DURATION") : "0"),
                    0.0f));

    std::cout << "===============================================\n"
              << "  R2 Lift DM3519 Readonly Monitor\n"
              << "  IFNAME        : " << ifname << "\n"
              << "  TARGET MOTORS : front CAN0 Tx 0x05 Rx 0x15; rear CAN1 Tx 0x05 Rx 0x15\n"
              << "  SEND DM FRAMES: no\n"
              << "  DURATION      : " << duration_s << " s\n"
              << "  PRINT_HZ      : " << print_hz << "\n"
              << "===============================================\n";

    print_interface_preflight(ifname);

    if (!ecat_master_init(&st_master, ifname.c_str()))
    {
        std::cerr << "[LIFT3519-RO] ecat_master_init failed. Check the NIC, LinkX power, and EtherCAT cable.\n";
        return 2;
    }

    linkx_init(&st_linkx, 1, &st_master.ctx);

    if (!configure_linkx_can())
        return 2;

    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[LIFT3519-RO] ecat_master_bring_online failed\n";
        return 2;
    }

    init_motors();

    const uint64_t print_period_ticks = std::max<uint64_t>(
        1U,
        static_cast<uint64_t>(1000.0f / print_hz));
    const uint64_t duration_ticks =
        (duration_s > 0.0f) ? static_cast<uint64_t>(duration_s * 1000.0f) : 0U;

    uint64_t tick = 0;
    auto next_wakeup = std::chrono::steady_clock::now();
    while (st_running.load())
    {
        receive_once();

        if ((tick % kAlivePeriodMs) == 0U)
            update_alive_status();

        if ((tick % print_period_ticks) == 0U)
            print_table(static_cast<double>(tick) * 0.001);

        if (duration_ticks > 0U && tick >= duration_ticks)
            break;

        ++tick;
        next_wakeup += std::chrono::milliseconds(kControlPeriodMs);
        std::this_thread::sleep_until(next_wakeup);
    }

    std::cout << "[LIFT3519-RO] done.\n";
    return 0;
}

// lift_tof_trigger_test_main.cpp
//
// Standalone ToF trigger test for the lift rods.
//
// Watches:
//   /high/up_front/range   (CAN2 ID 0x01) -> front lift rod raise/deploy
//   /high/down_back/range  (CAN2 ID 0x04) -> rear lift rod raise/deploy
//
// Do not run this while start_upper_computer.sh or any other EtherCAT master is
// using the same adapter.
//
// Example:
//   sudo IFNAME=enp86s0 ros2 run linkx_soem_demo lift_tof_trigger_test
//   sudo ./install/linkx_soem_demo/lib/linkx_soem_demo/lift_tof_trigger_test --ifname enp86s0 --raise-angle -14.3

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "crt_lift.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"

namespace
{
constexpr int kChannelCount = 4;
constexpr uint32_t kControlPeriodMs = 1;
constexpr uint32_t kTaskPeriod2Ms = 2;
constexpr uint32_t kTaskPeriod100Ms = 100;
constexpr uint16_t kToFTooFarCm = 65532U;

ecat_master_t st_master {};
linkx_t st_linkx {};
Class_Chariot_Lift st_lift;
std::atomic<bool> st_running {true};

struct Options
{
    std::string ifname = "enp86s0";
    float raise_angle = -8.0f;
    uint16_t near_cm = 10U;
    uint16_t jump_cm = 5U;
    uint32_t print_ms = 200U;
    float duration_s = 0.0f;
};

struct SensorTrigger
{
    Enum_Chariot_Lift_ToF_Sensor sensor;
    Enum_Chariot_Lift_Module module;
    const char *topic;
    const char *module_name;
    bool reference_valid = false;
    uint16_t reference_cm = 0U;
    bool triggered = false;
};

SensorTrigger st_triggers[] = {
    {CHARIOT_LIFT_TOF_UP_FRONT, CHARIOT_LIFT_MODULE_FRONT, "/high/up_front/range", "front"},
    {CHARIOT_LIFT_TOF_DOWN_BACK, CHARIOT_LIFT_MODULE_REAR, "/high/down_back/range", "rear"},
};

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

uint16_t parse_u16(const std::string &value, uint16_t fallback)
{
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(value.c_str(), &end, 10);
    if (end == value.c_str() || parsed > 65535UL)
        return fallback;
    return static_cast<uint16_t>(parsed);
}

uint32_t parse_u32(const std::string &value, uint32_t fallback)
{
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(value.c_str(), &end, 10);
    if (end == value.c_str())
        return fallback;
    return static_cast<uint32_t>(parsed);
}

void print_usage(const char *argv0)
{
    std::cerr
        << "Usage:\n"
        << "  " << argv0 << " [--ifname enp86s0] [--raise-angle -8.0]\n"
        << "        [--near-cm 10] [--jump-cm 5] [--print-ms 200] [--duration 0]\n\n"
        << "Behavior:\n"
        << "  /high/up_front/range  condition -> front lift RAISE/deploy\n"
        << "  /high/down_back/range condition -> rear  lift RAISE/deploy\n"
        << "  condition = distance <= near-cm OR abs(current_cm - reference_cm) >= jump-cm\n"
        << "  reference_cm is captured from the first usable frame for each sensor.\n";
}

bool configure_linkx_can()
{
    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);

    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        if (!linkx_set_can_baudrate(&st_linkx,
                                    static_cast<uint8_t>(ch),
                                    1, 2, 31, 8, 8,
                                    1, 12, 3, 3))
        {
            std::cerr << "[LIFT-TOF-TEST] CAN" << ch << " FDCAN 1M/5M config failed\n";
            return false;
        }
    }

    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);

    return true;
}

bool init_ethercat_linkx(const Options &opt)
{
    if (!ecat_master_init(&st_master, opt.ifname.c_str()))
    {
        std::cerr << "[LIFT-TOF-TEST] ecat_master_init failed for " << opt.ifname << "\n";
        return false;
    }

    linkx_init(&st_linkx, 1, &st_master.ctx);
    if (!configure_linkx_can())
        return false;

    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[LIFT-TOF-TEST] ecat_master_bring_online failed\n";
        return false;
    }

    return true;
}

void pump_can_receive()
{
    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);

    can_msg_t msg {};
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
    {
        while (linkx_quick_recv(&st_linkx, ch, &msg))
            st_lift.CAN_Rx_Callback(ch, msg.id, msg.data, msg.dlen);
    }
}

bool tof_usable(const ChariotLiftToFData &tof)
{
    return tof.online &&
           tof.valid &&
           tof.distance_cm > 0U &&
           tof.distance_cm != kToFTooFarCm &&
           std::isfinite(tof.range_m);
}

bool trigger_condition_met(SensorTrigger &trigger, const Options &opt)
{
    const ChariotLiftToFData &tof = st_lift.Get_ToF_Data(trigger.sensor);
    if (!tof_usable(tof))
        return false;

    if (!trigger.reference_valid)
    {
        trigger.reference_cm = tof.distance_cm;
        trigger.reference_valid = true;
    }

    const int delta_cm = static_cast<int>(tof.distance_cm) -
                         static_cast<int>(trigger.reference_cm);
    return tof.distance_cm <= opt.near_cm ||
           std::abs(delta_cm) >= static_cast<int>(opt.jump_cm);
}

void deploy_module(Enum_Chariot_Lift_Module module)
{
    if (module == CHARIOT_LIFT_MODULE_FRONT)
        st_lift.Set_Front_Lift_State(CHARIOT_LIFT_POSITION_RAISE);
    else
        st_lift.Set_Rear_Lift_State(CHARIOT_LIFT_POSITION_RAISE);
}

void update_triggers(const Options &opt)
{
    for (auto &trigger : st_triggers)
    {
        if (trigger.triggered)
            continue;

        if (trigger_condition_met(trigger, opt))
        {
            trigger.triggered = true;
            deploy_module(trigger.module);
            const ChariotLiftToFData &tof = st_lift.Get_ToF_Data(trigger.sensor);
            const int delta_cm = static_cast<int>(tof.distance_cm) -
                                 static_cast<int>(trigger.reference_cm);
            std::cout << "[LIFT-TOF-TEST][TRIGGER] " << trigger.topic
                      << " -> " << trigger.module_name << " lift RAISE"
                      << " current=" << tof.distance_cm << "cm"
                      << " ref=" << trigger.reference_cm << "cm"
                      << " delta=" << delta_cm << "cm\n";
        }
    }
}

void run_lift_control_tick(uint64_t tick)
{
    if ((tick % kTaskPeriod100Ms) == 0U)
        st_lift.TIM_100ms_Alive_PeriodElapsedCallback();
    if ((tick % kTaskPeriod2Ms) == 0U)
        st_lift.TIM_2ms_Control_PeriodElapsedCallback();
}

void print_status(double t_s)
{
    std::cout << std::fixed << std::setprecision(3)
              << "[LIFT-TOF-TEST] t=" << t_s << "s";
    for (const auto &trigger : st_triggers)
    {
        const ChariotLiftToFData &tof = st_lift.Get_ToF_Data(trigger.sensor);
        const char *status = tof.online ? (tof.valid ? "OK" : "BAD") : "OFF";
        int delta_cm = 0;
        if (trigger.reference_valid && tof_usable(tof))
        {
            delta_cm = static_cast<int>(tof.distance_cm) -
                       static_cast<int>(trigger.reference_cm);
        }

        std::cout << " | " << trigger.topic
                  << " " << status
                  << " cur=" << tof.distance_cm << "cm"
                  << " ref=";
        if (trigger.reference_valid)
            std::cout << trigger.reference_cm << "cm";
        else
            std::cout << "--";
        std::cout << " delta=" << delta_cm << "cm"
                  << " trig=" << (trigger.triggered ? 1 : 0);
    }
    std::cout << "\n";
}

void disable_all()
{
    st_lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_DISABLE);
    for (int i = 0; i < 80 && st_master.is_running; ++i)
    {
        pump_can_receive();
        st_lift.TIM_2ms_Control_PeriodElapsedCallback();
        linkx_send_pdos(&st_linkx);
        std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
    }
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);

    if (has_arg(argc, argv, "help") || has_arg(argc, argv, "h"))
    {
        print_usage(argv[0]);
        return 0;
    }

    Options opt;
    opt.ifname = cli_value(argc,
                           argv,
                           "ifname",
                           std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp86s0");
    opt.raise_angle = parse_float(cli_value(argc, argv, "raise-angle", "-8.0"), -8.0f);
    opt.near_cm = parse_u16(cli_value(argc, argv, "near-cm", "10"), 10U);
    opt.jump_cm = parse_u16(cli_value(argc, argv, "jump-cm", "5"), 5U);
    opt.print_ms = std::max<uint32_t>(20U, parse_u32(cli_value(argc, argv, "print-ms", "200"), 200U));
    opt.duration_s = std::max(0.0f, parse_float(cli_value(argc, argv, "duration", "0"), 0.0f));

    std::cout << "===============================================\n"
              << "  R2 Lift ToF Trigger Test\n"
              << "  IFNAME      : " << opt.ifname << "\n"
              << "  RAISE_ANGLE : " << opt.raise_angle << "\n"
              << "  NEAR_CM     : " << opt.near_cm << "\n"
              << "  JUMP_CM     : " << opt.jump_cm << "\n"
              << "  WATCH       : /high/up_front/range -> front lift\n"
              << "                /high/down_back/range -> rear lift\n"
              << "===============================================\n";

    if (!init_ethercat_linkx(opt))
        return 2;

    st_lift.Init(&st_linkx);
    st_lift.Set_Raise_Angle(opt.raise_angle);
    st_lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    st_lift.Set_Front_Lift_State(CHARIOT_LIFT_POSITION_RETRACT);
    st_lift.Set_Rear_Lift_State(CHARIOT_LIFT_POSITION_RETRACT);

    const uint64_t print_period_ticks = std::max<uint64_t>(1U, opt.print_ms / kControlPeriodMs);
    const uint64_t duration_ticks =
        (opt.duration_s > 0.0f) ? static_cast<uint64_t>(opt.duration_s * 1000.0f) : 0U;

    auto next_wakeup = std::chrono::steady_clock::now();
    uint64_t tick = 0;
    while (st_running.load() && st_master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kControlPeriodMs);

        pump_can_receive();
        update_triggers(opt);
        run_lift_control_tick(tick);
        linkx_send_pdos(&st_linkx);

        if ((tick % print_period_ticks) == 0U)
            print_status(static_cast<double>(tick) * 0.001);

        if (duration_ticks > 0U && tick >= duration_ticks)
            break;

        ++tick;
        std::this_thread::sleep_until(next_wakeup);
    }

    disable_all();
    std::cout << "[LIFT-TOF-TEST] done.\n";
    return 0;
}

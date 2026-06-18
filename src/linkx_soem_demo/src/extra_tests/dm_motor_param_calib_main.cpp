// dm_motor_param_calib_main.cpp
//
// R2 single DM motor parameter calibration tool:
//   1) friction torque
//   2) stiction / breakaway torque
//   3) equivalent rotor/load inertia
//
// Supported targets:
//   --motor wheel|omni : current omni chassis DM3519 wheels on LinkX channel 0
//   --wheel 0..3  : target wheel index
//
// The calibration algorithms are implemented by Class_Motor_DM_Normal and run
// in MIT mode. This file only owns EtherCAT/LinkX setup, feedback dispatch,
// command-line parsing, safety checks, and result printing.
//
// Usage:
//   sudo IFNAME=enp86s0 ros2 run linkx_soem_demo dm_motor_param_calib --motor wheel --wheel 0 --test all
//   sudo IFNAME=enp86s0 ./install/linkx_soem_demo/lib/linkx_soem_demo/dm_motor_param_calib --motor wheel --wheel 0 --test friction
//
// Environment variables, equivalent to CLI defaults:
//   CALIB_MOTOR=wheel|omni
//   CALIB_WHEEL=0..3
//   CALIB_TEST=friction|stiction|inertia|all
//   CALIB_OMEGA=2.0
//   CALIB_KD=2.0
//   CALIB_WARMUP=1.5
//   CALIB_MEASURE=2.0
//   CALIB_TSTEP=0.01
//   CALIB_DWELL=0.10
//   CALIB_THRESH=0.3
//   CALIB_TMAX=1.0
//   CALIB_INERTIA_T=0.5
//   CALIB_INERTIA_DUR=0.5
//   CALIB_FRICTION=0.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <thread>

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "crt_chassis_omni.h"
#include "dvc_motor_dm.h"
#include "math.h"

namespace
{
constexpr int kChannelCount = 4;
constexpr uint32_t kCtrlPeriodMs = 1;
constexpr uint32_t k100msPeriod = 100;
constexpr float kThermalLimitC = 70.0f;

ecat_master_t st_master {};
linkx_t st_linkx {};
Class_Chassis_Omni st_chassis;
std::atomic<bool> st_running {true};

int st_wheel = 0;

Class_Motor_DM_Normal &target_motor()
{
    return st_chassis.Motor_Wheel[st_wheel];
}

void on_signal(int)
{
    st_running.store(false);
    st_master.is_running = false;
}

float env_f(const char *name, float fallback)
{
    const char *v = std::getenv(name);
    if (v == nullptr || v[0] == '\0')
    {
        return fallback;
    }

    char *end = nullptr;
    const float parsed = std::strtof(v, &end);
    return (end == v) ? fallback : parsed;
}

int env_i(const char *name, int fallback)
{
    const char *v = std::getenv(name);
    if (v == nullptr || v[0] == '\0')
    {
        return fallback;
    }
    return std::atoi(v);
}

const char *cli_get(int argc, char **argv, const char *key, const char *fallback)
{
    const std::string k1 = std::string("--") + key;
    const std::string k2 = std::string("--") + key + "=";

    for (int i = 1; i < argc; ++i)
    {
        if (k1 == argv[i] && (i + 1) < argc)
        {
            return argv[i + 1];
        }

        const std::string arg = argv[i];
        if (arg.compare(0, k2.size(), k2) == 0)
        {
            return argv[i] + k2.size();
        }
    }

    return fallback;
}

void can_dispatch(uint8_t ch, uint32_t can_id, uint8_t *data)
{
    const uint32_t id_std = can_id & 0x7FFU;

    auto dispatch = [&](const int *indices, int n) -> bool {
        for (int k = 0; k < n; ++k)
        {
            const int i = indices[k];
            if (id_std == st_chassis.Motor_Wheel[i].DM_CAN_Rx_ID)
            {
                st_chassis.Motor_Wheel[i].CAN_RxCpltCallback(data);
                return true;
            }
        }
        return false;
    };

    if (ch == 0)
    {
        const int indices[] = {1, 2};
        dispatch(indices, 2);
    }
    else if (ch == 1)
    {
        const int indices[] = {0, 3};
        dispatch(indices, 2);
    }
}

bool temperature_safe()
{
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const float mos_c = st_chassis.Motor_Wheel[i].Get_Now_MOS_Temperature() - CELSIUS_TO_KELVIN;
        const float rotor_c = st_chassis.Motor_Wheel[i].Get_Now_Rotor_Temperature() - CELSIUS_TO_KELVIN;

        const bool mos_valid = mos_c > -50.0f && mos_c < 200.0f;
        const bool rotor_valid = rotor_c > -50.0f && rotor_c < 200.0f;
        if ((mos_valid && mos_c >= kThermalLimitC) ||
            (rotor_valid && rotor_c >= kThermalLimitC))
        {
            std::cerr << "\n[CALIB][THERMAL] motor_index=" << i
                      << " MOS=" << mos_c
                      << "C Rotor=" << rotor_c
                      << "C limit=" << kThermalLimitC
                      << "C, abort.\n";
            return false;
        }
    }

    return true;
}

void hold_other_motors_neutral()
{
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        if (i != st_wheel)
        {
            st_chassis.Motor_Wheel[i].Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
            st_chassis.Motor_Wheel[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
}

bool ec_step(uint32_t tick)
{
    if (!st_running.load() || !st_master.is_running)
    {
        return false;
    }

    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);

    can_msg_t msg {};
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
    {
        while (linkx_quick_recv(&st_linkx, ch, &msg))
        {
            can_dispatch(ch, msg.id, msg.data);
        }
    }

    if ((tick % k100msPeriod) == 0U)
    {
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            st_chassis.Motor_Wheel[i].TIM_Alive_PeriodElapsedCallback();

            if (st_chassis.Motor_Wheel[i].Get_Status() != Motor_DM_Status_ENABLE)
            {
                st_chassis.Motor_Wheel[i].CAN_Send_Enter();
            }
        }
    }

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        st_chassis.Motor_Wheel[i].TIM_Send_PeriodElapsedCallback();
    }

    linkx_send_pdos(&st_linkx);
    return true;
}

void ec_busywait_ms(uint32_t &tick, uint32_t ms)
{
    auto next_wakeup = std::chrono::steady_clock::now();

    for (uint32_t i = 0; i < ms; ++i)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        if (!ec_step(tick++))
        {
            return;
        }
        std::this_thread::sleep_until(next_wakeup);
    }
}

Struct_Motor_DM_Calib_Result run_calibration_blocking(float timeout_s)
{
    auto &mot = target_motor();
    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;
    uint32_t print_div = 0;
    float elapsed = 0.0f;
    const float dt = static_cast<float>(kCtrlPeriodMs) / 1000.0f;

    while (st_running.load() && st_master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);

        if (!ec_step(tick))
        {
            break;
        }

        hold_other_motors_neutral();
        mot.Calibration_Tick(dt);

        if (mot.Is_Calibration_Finished())
        {
            break;
        }

        if ((print_div++ % 200U) == 0U)
        {
            const auto r = mot.Get_Calibration_Result();
            std::cout << "  [t=" << std::fixed << std::setprecision(2) << elapsed
                      << "s phase=" << r.phase
                      << " status=" << static_cast<int>(mot.Get_Status())
                      << " ctrl=" << static_cast<int>(mot.Get_Now_Control_Status())
                      << " rad=" << mot.Get_Now_Radian()
                      << " omega=" << mot.Get_Now_Omega()
                      << " torque=" << mot.Get_Now_Torque()
                      << "]\r" << std::flush;
        }

        if (!temperature_safe())
        {
            mot.Stop_Calibration();
            break;
        }

        elapsed += dt;
        if (elapsed > timeout_s)
        {
            std::cerr << "\n[CALIB][TIMEOUT] " << timeout_s << "s reached, abort.\n";
            mot.Stop_Calibration();
            break;
        }

        ++tick;
        std::this_thread::sleep_until(next_wakeup);
    }

    std::cout << "\n";
    return mot.Get_Calibration_Result();
}

void print_usage(const char *argv0)
{
    std::cerr
        << "Usage:\n"
        << "  " << argv0 << " [--motor wheel|omni] [--wheel 0..3]\n"
        << "        [--test friction|stiction|inertia|all]\n"
        << "        [--omega 2.0] [--kd 2.0] [--warmup 1.5] [--measure 2.0]\n"
        << "        [--tstep 0.01] [--dwell 0.10] [--thresh 0.3] [--tmax 1.0]\n"
        << "        [--inertia_t 0.5] [--inertia_dur 0.5] [--friction 0.0]\n";
}

void append_summary_header(std::ofstream &summary,
                           const std::string &motor,
                           int wheel,
                           const std::string &test)
{
    if (!summary.is_open())
    {
        return;
    }

    std::time_t now = std::time(nullptr);
    summary << "\n===== "
            << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S")
            << " motor=" << motor
            << " wheel=" << wheel
            << " test=" << test
            << " =====\n";
}

void disable_all_motors(uint32_t &tick)
{
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        st_chassis.Motor_Wheel[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        st_chassis.Motor_Wheel[i].CAN_Send_Exit();
    }

    linkx_send_pdos(&st_linkx);
    ec_busywait_ms(tick, 100);
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    if (std::string(cli_get(argc, argv, "help", "")) == "1")
    {
        print_usage(argv[0]);
        return 0;
    }

    const std::string ifname = std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp86s0";

    const std::string motor_str = cli_get(
        argc,
        argv,
        "motor",
        std::getenv("CALIB_MOTOR") ? std::getenv("CALIB_MOTOR") : "wheel");

    if (motor_str != "wheel" && motor_str != "omni")
    {
        std::cerr << "[CALIB] invalid --motor '" << motor_str << "'; expected wheel|omni\n";
        print_usage(argv[0]);
        return 1;
    }

    st_wheel = std::atoi(cli_get(
        argc,
        argv,
        "wheel",
        std::to_string(env_i("CALIB_WHEEL", 0)).c_str()));
    if (st_wheel < 0 || st_wheel >= OMNI_WHEEL_NUM)
    {
        std::cerr << "[CALIB] invalid --wheel " << st_wheel
                  << "; expected 0.." << (OMNI_WHEEL_NUM - 1) << "\n";
        return 1;
    }

    const std::string test = cli_get(
        argc,
        argv,
        "test",
        std::getenv("CALIB_TEST") ? std::getenv("CALIB_TEST") : "all");
    if (test != "friction" && test != "stiction" && test != "inertia" && test != "all")
    {
        std::cerr << "[CALIB] invalid --test '" << test
                  << "'; expected friction|stiction|inertia|all\n";
        return 1;
    }

    const float omega_target = std::strtof(cli_get(argc, argv, "omega",
                                      std::to_string(env_f("CALIB_OMEGA", 2.0f)).c_str()), nullptr);
    const float kd_loop = std::strtof(cli_get(argc, argv, "kd",
                                 std::to_string(env_f("CALIB_KD", 2.0f)).c_str()), nullptr);
    const float warmup = std::strtof(cli_get(argc, argv, "warmup",
                                std::to_string(env_f("CALIB_WARMUP", 1.5f)).c_str()), nullptr);
    const float measure = std::strtof(cli_get(argc, argv, "measure",
                                 std::to_string(env_f("CALIB_MEASURE", 2.0f)).c_str()), nullptr);

    const float t_step = std::strtof(cli_get(argc, argv, "tstep",
                                std::to_string(env_f("CALIB_TSTEP", 0.01f)).c_str()), nullptr);
    const float dwell = std::strtof(cli_get(argc, argv, "dwell",
                               std::to_string(env_f("CALIB_DWELL", 0.10f)).c_str()), nullptr);
    const float thresh = std::strtof(cli_get(argc, argv, "thresh",
                                std::to_string(env_f("CALIB_THRESH", 0.3f)).c_str()), nullptr);
    const float t_max = std::strtof(cli_get(argc, argv, "tmax",
                               std::to_string(env_f("CALIB_TMAX", 1.0f)).c_str()), nullptr);

    const float inertia_t = std::strtof(cli_get(argc, argv, "inertia_t",
                                   std::to_string(env_f("CALIB_INERTIA_T", 0.5f)).c_str()), nullptr);
    const float inertia_dur = std::strtof(cli_get(argc, argv, "inertia_dur",
                                     std::to_string(env_f("CALIB_INERTIA_DUR", 0.5f)).c_str()), nullptr);
    float friction_known = std::strtof(cli_get(argc, argv, "friction",
                                  std::to_string(env_f("CALIB_FRICTION", 0.0f)).c_str()), nullptr);

    std::cout << "===============================================\n"
              << "  R2 DM Motor Parameter Calibration\n"
              << "  IFNAME : " << ifname << "\n"
              << "  MOTOR  : DM3519 omni wheel on LinkX channel 0\n"
              << "  WHEEL  : " << st_wheel << "\n"
              << "  TEST   : " << test << "\n"
              << "===============================================\n";

    std::cout << "[SAFETY] Wheel motor calibration requires the full chassis to be suspended.\n";

    if (!ecat_master_init(&st_master, ifname.c_str()))
    {
        std::cerr << "[CALIB] ecat_master_init failed\n";
        return 2;
    }

    linkx_init(&st_linkx, 1, &st_master.ctx);

    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);
    }

    // Match the runtime LinkX setup: all four channels use CAN-FD+BRS, nominal 1 Mbps and data 5 Mbps.
    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        if (!linkx_set_can_baudrate(&st_linkx, static_cast<uint8_t>(ch), 1, 2, 31, 8, 8, 1, 12, 3, 3))
        {
            std::cerr << "[CALIB] CAN" << ch << " FDCAN 1M/5M config failed\n";
            return 2;
        }
    }
    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);
    }

    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[CALIB] ecat_master_bring_online failed\n";
        return 2;
    }

    st_chassis.Init(&st_linkx);
    st_chassis.Init_Motor_Params();
    st_chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);

    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;
    while (st_running.load() && st_master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        ec_step(tick);
        if ((tick % 100U) == 0U && target_motor().Get_Status() == Motor_DM_Status_ENABLE)
        {
            break;
        }
        if (tick > 3000U)
        {
            break;
        }
        ++tick;
        std::this_thread::sleep_until(next_wakeup);
    }

    std::cout << "[CALIB] enabling DM motors...\n";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        st_chassis.Motor_Wheel[i].CAN_Send_Enter();
    }
    linkx_send_pdos(&st_linkx);
    ec_busywait_ms(tick, 150);

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        st_chassis.Motor_Wheel[i].Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        st_chassis.Motor_Wheel[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    std::cout << "\n>>> Confirm omni wheel " << st_wheel
              << " is safe. Test starts in 3 seconds...\n";
    ec_busywait_ms(tick, 3000);

    mkdir("var_data", 0755);
    std::ofstream summary("var_data/calibration/dm_motor_param_calib_result.txt", std::ios::app);
    append_summary_header(summary, motor_str, st_wheel, test);

    auto &mot = target_motor();

    if (test == "friction" || test == "all")
    {
        std::cout << "\n--- [1/3] FRICTION calibration ---\n";
        mot.Begin_Friction_Calibration(omega_target, kd_loop, warmup, measure);
        const float timeout = (warmup + measure) * 2.0f + 2.0f;
        const auto r = run_calibration_blocking(timeout);

        std::cout << std::fixed << std::setprecision(4)
                  << "  friction_torque_pos_nm=" << r.friction_torque_pos_nm << "\n"
                  << "  friction_torque_neg_nm=" << r.friction_torque_neg_nm << "\n"
                  << "  friction_torque_avg_nm=" << r.friction_torque_avg_nm << "\n"
                  << "  friction_omega_pos_actual=" << r.friction_omega_pos_actual << "\n"
                  << "  friction_omega_neg_actual=" << r.friction_omega_neg_actual << "\n"
                  << "  success=" << (r.success ? "YES" : "NO") << "\n"
                  << std::defaultfloat;

        if (summary.is_open())
        {
            summary << std::fixed << std::setprecision(4)
                    << "FRICTION friction_torque_pos_nm=" << r.friction_torque_pos_nm
                    << " friction_torque_neg_nm=" << r.friction_torque_neg_nm
                    << " friction_torque_avg_nm=" << r.friction_torque_avg_nm
                    << " friction_omega_pos_actual=" << r.friction_omega_pos_actual
                    << " friction_omega_neg_actual=" << r.friction_omega_neg_actual
                    << " success=" << r.success << "\n"
                    << std::defaultfloat;
        }

        if (r.success && test == "all" && friction_known < 1e-6f)
        {
            friction_known = r.friction_torque_avg_nm;
        }
        ec_busywait_ms(tick, 800);
    }

    if ((test == "stiction" || test == "all") && st_running.load())
    {
        std::cout << "\n--- [2/3] STICTION calibration ---\n";
        mot.Begin_Stiction_Calibration(t_step, dwell, thresh, t_max);
        const float timeout = (t_max / std::max(1e-3f, t_step)) * dwell + 2.0f;
        const auto r = run_calibration_blocking(timeout);

        std::cout << std::fixed << std::setprecision(4)
                  << "  stiction_torque_nm=" << r.stiction_torque_nm << "\n"
                  << "  stiction_breakaway_omega=" << r.stiction_breakaway_omega << "\n"
                  << "  success=" << (r.success ? "YES" : "NO") << "\n"
                  << std::defaultfloat;

        if (summary.is_open())
        {
            summary << std::fixed << std::setprecision(4)
                    << "STICTION stiction_torque_nm=" << r.stiction_torque_nm
                    << " stiction_breakaway_omega=" << r.stiction_breakaway_omega
                    << " success=" << r.success << "\n"
                    << std::defaultfloat;
        }
        ec_busywait_ms(tick, 800);
    }

    if ((test == "inertia" || test == "all") && st_running.load())
    {
        std::cout << "\n--- [3/3] INERTIA calibration ---\n";
        std::cout << "  using friction_known=" << friction_known << " Nm\n";
        mot.Begin_Inertia_Calibration(inertia_t, friction_known, 0.5f, inertia_dur);
        const float timeout = inertia_dur + 1.5f;
        const auto r = run_calibration_blocking(timeout);

        std::cout << std::fixed << std::setprecision(6)
                  << "  inertia_kgm2=" << r.inertia_kgm2 << "\n"
                  << "  inertia_alpha_meas=" << r.inertia_alpha_meas << "\n"
                  << "  inertia_torque_step_nm=" << r.inertia_torque_step_nm << "\n"
                  << "  inertia_friction_used_nm=" << r.inertia_friction_used_nm << "\n"
                  << "  success=" << (r.success ? "YES" : "NO") << "\n"
                  << std::defaultfloat;

        if (summary.is_open())
        {
            summary << std::fixed << std::setprecision(6)
                    << "INERTIA inertia_kgm2=" << r.inertia_kgm2
                    << " inertia_alpha_meas=" << r.inertia_alpha_meas
                    << " inertia_torque_step_nm=" << r.inertia_torque_step_nm
                    << " inertia_friction_used_nm=" << r.inertia_friction_used_nm
                    << " success=" << r.success << "\n"
                    << std::defaultfloat;
        }
    }

    std::cout << "\n[CALIB] disabling DM motors...\n";
    disable_all_motors(tick);

    std::cout << "[CALIB] result appended to var_data/calibration/dm_motor_param_calib_result.txt\n";
    std::cout << "[CALIB] done.\n";
    return 0;
}

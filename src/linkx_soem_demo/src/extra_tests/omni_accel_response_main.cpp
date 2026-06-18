// omni_accel_response_main.cpp
//
// R2 omni chassis wheel acceleration response tool.
//
// Test flow:
//   1) For low/mid/high target wheel speeds, run each wheel alone and measure
//      the time to reach the target speed.
//   2) For the same target speeds, run all four wheels together and measure
//      each wheel's reach time in the same step command.
//   3) Write raw CSV data and a summary with timing/acceleration spread plus
//      small-step wheel_feedforward_scale suggestions.
//
// Usage:
//   sudo IFNAME=enp86s0 ros2 run linkx_soem_demo omni_accel_response
//   sudo IFNAME=enp86s0 ./install/linkx_soem_demo/lib/linkx_soem_demo/omni_accel_response \
//        --speeds 10,30,60 --duration 2.5 --settle 1.0 --reach-frac 0.95
//
// Output:
//   var_data/omni/omni_accel_response_<timestamp>.csv
//   var_data/omni/omni_accel_summary_<timestamp>.txt

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include "crt_chassis_omni.h"
#include "dvc_motor_dm.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "math.h"

namespace
{
constexpr int kChannelCount = 4;
constexpr uint32_t kEcPeriodMs = 1;
constexpr uint32_t kMotorCommandPeriodTicks = 2;
constexpr uint32_t kAlivePeriodTicks = 100;
constexpr float kMotorCommandDtS = OMNI_WHEEL_PROFILE_DT;
constexpr float kAccelFilterAlpha = OMNI_WHEEL_ACCEL_FILTER_ALPHA;
constexpr float kTorqueFfLimitNm = OMNI_WHEEL_TORQUE_FF_LIMIT_NM;
constexpr float kThermalLimitC = 70.0f;

constexpr std::array<const char *, OMNI_WHEEL_NUM> kWheelName = {
    "W0_RB_ch1_id2_old_LF",
    "W1_RF_ch0_id2_old_LB",
    "W2_LF_ch0_id1_old_RB",
    "W3_LB_ch1_id1_old_RF",
};

ecat_master_t st_master {};
linkx_t st_linkx {};
Class_Chassis_Omni st_chassis;
std::atomic<bool> st_running {true};

struct Options
{
    std::string ifname = "enp86s0";
    std::vector<float> speeds_rad_s {10.0f, 30.0f, 60.0f};
    float direction = 1.0f;
    float duration_s = 2.5f;
    float settle_s = 1.0f;
    float reach_frac = 0.95f;
    float reach_abs_tol = 1.0f;
    float reach_hold_s = 0.05f;
    int sample_hz = 200;
    bool run_single = true;
    bool run_sync = true;
    bool use_profile = false;
    float profile_accel_limit = OMNI_WHEEL_ACCEL_LIMIT_RAD_S2;
    float profile_decel_limit = OMNI_WHEEL_DECEL_LIMIT_RAD_S2;
    bool use_feedforward = true;
    bool has_ff_scale_override = false;
    std::array<float, OMNI_WHEEL_NUM> ff_scale_override {};
    bool has_kd_override_list = false;
    std::array<float, OMNI_WHEEL_NUM> kd_override_list {};
    float kp_override = std::numeric_limits<float>::quiet_NaN();
    float kd_override = std::numeric_limits<float>::quiet_NaN();
    std::string csv_path;
    std::string summary_path;
};

struct DirectCommandState
{
    std::array<float, OMNI_WHEEL_NUM> raw_target {};
    std::array<float, OMNI_WHEEL_NUM> target {};
    std::array<float, OMNI_WHEEL_NUM> accel_filtered {};
    bool use_profile = false;
    float profile_accel_limit = OMNI_WHEEL_ACCEL_LIMIT_RAD_S2;
    float profile_decel_limit = OMNI_WHEEL_DECEL_LIMIT_RAD_S2;
    bool use_feedforward = true;
    float kp_override = std::numeric_limits<float>::quiet_NaN();
    float kd_override = std::numeric_limits<float>::quiet_NaN();
};

DirectCommandState st_cmd;

struct Stats
{
    double sum = 0.0;
    double sum_sq = 0.0;
    double min = std::numeric_limits<double>::infinity();
    double max = -std::numeric_limits<double>::infinity();
    uint64_t n = 0;

    void add(double value)
    {
        if (!std::isfinite(value))
            return;
        sum += value;
        sum_sq += value * value;
        min = std::min(min, value);
        max = std::max(max, value);
        n++;
    }

    double mean() const
    {
        return n > 0 ? sum / static_cast<double>(n) : 0.0;
    }

    double stddev() const
    {
        if (n < 2)
            return 0.0;
        const double m = mean();
        const double var = sum_sq / static_cast<double>(n) - m * m;
        return std::sqrt(std::max(0.0, var));
    }
};

struct WheelTrialResult
{
    bool commanded = false;
    bool reached = false;
    float target_omega = 0.0f;
    float reach_time_s = std::numeric_limits<float>::quiet_NaN();
    float final_omega = 0.0f;
    float max_signed_omega = 0.0f;
    float overshoot_pct = 0.0f;
    Stats omega_signed;
    Stats accel_signed;
    Stats torque_signed;
};

struct TrialResult
{
    std::string mode;
    std::string speed_label;
    float target_abs_omega = 0.0f;
    int active_wheel = -1;
    std::array<WheelTrialResult, OMNI_WHEEL_NUM> wheel;
};

struct WheelAggregate
{
    Stats reach_time;
    Stats accel_mean;
};

void on_signal(int)
{
    st_running.store(false);
    st_master.is_running = false;
}

float actual_wheel_omega(int wheel)
{
    return st_chassis.Motor_Wheel[wheel].Get_Now_Omega() *
           OMNI_WHEEL_FEEDBACK_GEAR_SCALE;
}

float firmware_command_omega(float wheel_omega)
{
    return wheel_omega * OMNI_WHEEL_COMMAND_GEAR_SCALE;
}

float env_f(const char *name, float fallback)
{
    const char *v = std::getenv(name);
    if (v == nullptr || v[0] == '\0')
        return fallback;

    char *end = nullptr;
    const float parsed = std::strtof(v, &end);
    return (end == v) ? fallback : parsed;
}

int env_i(const char *name, int fallback)
{
    const char *v = std::getenv(name);
    if (v == nullptr || v[0] == '\0')
        return fallback;
    return std::atoi(v);
}

const char *cli_get(int argc, char **argv, const char *key, const char *fallback)
{
    const std::string k1 = std::string("--") + key;
    const std::string k2 = k1 + "=";

    for (int i = 1; i < argc; ++i)
    {
        if (k1 == argv[i] && (i + 1) < argc)
            return argv[i + 1];

        const std::string arg = argv[i];
        if (arg.compare(0, k2.size(), k2) == 0)
            return argv[i] + k2.size();
    }

    return fallback;
}

bool cli_has(int argc, char **argv, const char *key)
{
    const std::string k = std::string("--") + key;
    for (int i = 1; i < argc; ++i)
        if (k == argv[i])
            return true;
    return false;
}

std::string timestamp_string()
{
    const std::time_t now = std::time(nullptr);
    std::tm tm_now {};
    localtime_r(&now, &tm_now);
    char buf[32] {};
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_now);
    return std::string(buf);
}

std::vector<std::string> split_csv(const std::string &text)
{
    std::vector<std::string> out;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
            out.push_back(item);
    }
    return out;
}

std::vector<float> parse_float_list(const std::string &text, const std::vector<float> &fallback)
{
    std::vector<float> values;
    for (const auto &item : split_csv(text))
    {
        char *end = nullptr;
        const float v = std::strtof(item.c_str(), &end);
        if (end != item.c_str() && std::isfinite(v) && std::fabs(v) > 1e-4f)
            values.push_back(std::fabs(v));
    }
    return values.empty() ? fallback : values;
}

bool parse_float_array4(const std::string &text, std::array<float, OMNI_WHEEL_NUM> &out)
{
    const auto items = split_csv(text);
    if (items.size() != OMNI_WHEEL_NUM)
        return false;

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        char *end = nullptr;
        const float v = std::strtof(items[i].c_str(), &end);
        if (end == items[i].c_str() || !std::isfinite(v) || v < 0.0f)
            return false;
        out[i] = v;
    }
    return true;
}

std::string speed_label(size_t index)
{
    static constexpr std::array<const char *, 3> kLabels = {"low", "mid", "high"};
    if (index < kLabels.size())
        return kLabels[index];
    return "speed" + std::to_string(index);
}

float clamp_float(float value, float lo, float hi)
{
    if (value < lo)
        return lo;
    if (value > hi)
        return hi;
    return value;
}

void print_usage(const char *argv0)
{
    std::cerr
        << "Usage:\n"
        << "  sudo IFNAME=enp86s0 " << argv0 << " [options]\n\n"
        << "Options:\n"
        << "  --ifname IFACE        EtherCAT NIC, default IFNAME env or enp86s0\n"
        << "  --speeds LIST         target wheel speeds rad/s, default 10,30,60\n"
        << "  --direction 1|-1      command direction, default 1\n"
        << "  --duration SEC        max duration per step, default 2.5\n"
        << "  --settle SEC          zero-command settle before each step, default 1.0\n"
        << "  --reach-frac F        reach threshold as fraction of target, default 0.95\n"
        << "  --reach-abs-tol RADS  absolute target tolerance rad/s, default 1.0\n"
        << "  --reach-hold SEC      threshold hold time, default 0.05\n"
        << "  --sample-hz HZ        CSV sample rate, default 200\n"
        << "  --single 0|1          run single-wheel tests, default 1\n"
        << "  --sync 0|1            run all-wheel synchronous tests, default 1\n"
        << "  --profile 0|1         command with project trapezoid profile, default 0\n"
        << "  --profile-accel A     profile acceleration rad/s^2, default project 550\n"
        << "  --profile-decel A     profile deceleration rad/s^2, default project 700\n"
        << "  --ff 0|1              use chassis feedforward model, default 1\n"
        << "  --ff-scales LIST      override wheel feedforward scales, e.g. 0.84,0.38,0.77,0.84\n"
        << "  --kds LIST            override per-wheel MIT Kd, e.g. 3.0,1.5,1.5,5.0\n"
        << "  --kp K                override MIT Kp for all wheels\n"
        << "  --kd K                override MIT Kd for all wheels\n"
        << "  --csv PATH            CSV output path\n"
        << "  --summary PATH        summary output path\n";
}

Options parse_options(int argc, char **argv)
{
    Options opt;
    if (const char *env = std::getenv("IFNAME"))
        opt.ifname = env;

    opt.ifname = cli_get(argc, argv, "ifname", opt.ifname.c_str());
    opt.speeds_rad_s = parse_float_list(
        cli_get(argc, argv, "speeds",
                std::getenv("OMNI_ACCEL_SPEEDS") ? std::getenv("OMNI_ACCEL_SPEEDS") : "10,30,60"),
        opt.speeds_rad_s);
    opt.direction = std::strtof(cli_get(argc, argv, "direction",
                                        std::to_string(env_f("OMNI_ACCEL_DIRECTION", opt.direction)).c_str()),
                                nullptr);
    opt.direction = (opt.direction < 0.0f) ? -1.0f : 1.0f;
    opt.duration_s = std::strtof(cli_get(argc, argv, "duration",
                                         std::to_string(env_f("OMNI_ACCEL_DURATION", opt.duration_s)).c_str()),
                                 nullptr);
    opt.settle_s = std::strtof(cli_get(argc, argv, "settle",
                                       std::to_string(env_f("OMNI_ACCEL_SETTLE", opt.settle_s)).c_str()),
                               nullptr);
    opt.reach_frac = std::strtof(cli_get(argc, argv, "reach-frac",
                                         std::to_string(env_f("OMNI_ACCEL_REACH_FRAC", opt.reach_frac)).c_str()),
                                 nullptr);
    opt.reach_abs_tol = std::strtof(cli_get(argc, argv, "reach-abs-tol",
                                            std::to_string(env_f("OMNI_ACCEL_REACH_ABS_TOL", opt.reach_abs_tol)).c_str()),
                                    nullptr);
    opt.reach_hold_s = std::strtof(cli_get(argc, argv, "reach-hold",
                                           std::to_string(env_f("OMNI_ACCEL_REACH_HOLD", opt.reach_hold_s)).c_str()),
                                   nullptr);
    opt.sample_hz = std::atoi(cli_get(argc, argv, "sample-hz",
                                      std::to_string(env_i("OMNI_ACCEL_SAMPLE_HZ", opt.sample_hz)).c_str()));
    opt.run_single = std::atoi(cli_get(argc, argv, "single",
                                       std::to_string(env_i("OMNI_ACCEL_SINGLE", opt.run_single ? 1 : 0)).c_str())) != 0;
    opt.run_sync = std::atoi(cli_get(argc, argv, "sync",
                                     std::to_string(env_i("OMNI_ACCEL_SYNC", opt.run_sync ? 1 : 0)).c_str())) != 0;
    opt.use_profile = std::atoi(cli_get(argc, argv, "profile",
                                        std::to_string(env_i("OMNI_ACCEL_PROFILE", opt.use_profile ? 1 : 0)).c_str())) != 0;
    opt.profile_accel_limit = std::strtof(
        cli_get(argc, argv, "profile-accel",
                std::to_string(env_f("OMNI_ACCEL_PROFILE_ACCEL", opt.profile_accel_limit)).c_str()),
        nullptr);
    opt.profile_decel_limit = std::strtof(
        cli_get(argc, argv, "profile-decel",
                std::to_string(env_f("OMNI_ACCEL_PROFILE_DECEL", opt.profile_decel_limit)).c_str()),
        nullptr);
    opt.use_feedforward = std::atoi(cli_get(argc, argv, "ff",
                                            std::to_string(env_i("OMNI_ACCEL_FF", opt.use_feedforward ? 1 : 0)).c_str())) != 0;
    if (cli_has(argc, argv, "ff-scales"))
    {
        const std::string scales = cli_get(argc, argv, "ff-scales", "");
        opt.has_ff_scale_override = parse_float_array4(scales, opt.ff_scale_override);
        if (!opt.has_ff_scale_override)
        {
            std::cerr << "[OMNI-ACCEL][WARN] invalid --ff-scales '" << scales
                      << "', expected four non-negative comma-separated numbers.\n";
        }
    }
    if (cli_has(argc, argv, "kds"))
    {
        const std::string kds = cli_get(argc, argv, "kds", "");
        opt.has_kd_override_list = parse_float_array4(kds, opt.kd_override_list);
        if (!opt.has_kd_override_list)
        {
            std::cerr << "[OMNI-ACCEL][WARN] invalid --kds '" << kds
                      << "', expected four non-negative comma-separated numbers.\n";
        }
    }

    if (cli_has(argc, argv, "kp"))
        opt.kp_override = std::strtof(cli_get(argc, argv, "kp", "0"), nullptr);
    if (cli_has(argc, argv, "kd"))
        opt.kd_override = std::strtof(cli_get(argc, argv, "kd", "0"), nullptr);

    opt.duration_s = std::max(0.2f, opt.duration_s);
    opt.settle_s = std::max(0.0f, opt.settle_s);
    opt.reach_frac = clamp_float(opt.reach_frac, 0.5f, 1.2f);
    opt.reach_abs_tol = std::max(0.0f, opt.reach_abs_tol);
    opt.reach_hold_s = std::max(0.0f, opt.reach_hold_s);
    opt.profile_accel_limit = std::max(1.0f, opt.profile_accel_limit);
    opt.profile_decel_limit = std::max(1.0f, opt.profile_decel_limit);
    opt.sample_hz = std::max(1, opt.sample_hz);

    mkdir("var_data", 0755);
    mkdir("var_data/omni", 0755);
    const std::string ts = timestamp_string();
    const std::string default_csv = "var_data/omni/omni_accel_response_" + ts + ".csv";
    const std::string default_summary = "var_data/omni/omni_accel_summary_" + ts + ".txt";
    opt.csv_path = cli_get(argc, argv, "csv", default_csv.c_str());
    opt.summary_path = cli_get(argc, argv, "summary", default_summary.c_str());
    return opt;
}

void dispatch_wheel_feedback(uint8_t ch, uint32_t can_id, uint8_t *data)
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
            std::cerr << "\n[OMNI-ACCEL][THERMAL] " << kWheelName[i]
                      << " MOS=" << mos_c
                      << "C Rotor=" << rotor_c
                      << "C limit=" << kThermalLimitC << "C\n";
            return false;
        }
    }

    return true;
}

void reset_direct_command_state()
{
    st_cmd.raw_target.fill(0.0f);
    st_cmd.target.fill(0.0f);
    st_cmd.accel_filtered.fill(0.0f);
}

void set_direct_targets(const std::array<float, OMNI_WHEEL_NUM> &targets)
{
    st_cmd.raw_target = targets;
}

void apply_direct_targets()
{
    std::array<float, OMNI_WHEEL_NUM> profiled_targets {};
    std::array<float, OMNI_WHEEL_NUM> command_accel {};

    if (st_cmd.use_profile)
    {
        float progress = 1.0f;
        bool has_delta = false;

        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const auto &p = st_chassis.wheel_params_[i];
            const float current = st_cmd.target[i];
            float raw_target = st_cmd.raw_target[i];
            if (std::fabs(raw_target) < p.wheel_omega_deadzone)
                raw_target = 0.0f;

            const float delta = raw_target - current;
            const float abs_delta = std::fabs(delta);
            if (abs_delta < 1e-5f)
                continue;

            const bool changing_direction =
                (current * raw_target < 0.0f) &&
                (std::fabs(current) > p.wheel_omega_deadzone);
            const bool reducing_speed =
                std::fabs(raw_target) < std::fabs(current) ||
                std::fabs(raw_target) < p.wheel_omega_deadzone;
            const float limit = (changing_direction || reducing_speed) ?
                st_cmd.profile_decel_limit :
                st_cmd.profile_accel_limit;
            const float wheel_progress = (limit * kMotorCommandDtS) / abs_delta;

            if (wheel_progress < progress)
                progress = wheel_progress;
            has_delta = true;
        }

        if (!has_delta)
            progress = 0.0f;
        if (progress > 1.0f)
            progress = 1.0f;
        if (progress < 0.0f)
            progress = 0.0f;

        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const auto &p = st_chassis.wheel_params_[i];
            float raw_target = st_cmd.raw_target[i];
            if (std::fabs(raw_target) < p.wheel_omega_deadzone)
                raw_target = 0.0f;

            const float current = st_cmd.target[i];
            float target = current + (raw_target - current) * progress;
            if (raw_target == 0.0f && std::fabs(target) < p.wheel_omega_deadzone)
                target = 0.0f;

            profiled_targets[i] = target;
            command_accel[i] = (target - current) / kMotorCommandDtS;
        }
    }
    else
    {
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const auto &p = st_chassis.wheel_params_[i];
            float target = st_cmd.raw_target[i];
            if (std::fabs(target) < p.wheel_omega_deadzone)
                target = 0.0f;

            profiled_targets[i] = target;
            command_accel[i] = (target - st_cmd.target[i]) / kMotorCommandDtS;
        }
    }

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        auto &motor = st_chassis.Motor_Wheel[i];
        const auto &p = st_chassis.wheel_params_[i];
        const float target = profiled_targets[i];

        float torque_ff = 0.0f;
        if (st_cmd.use_feedforward)
        {
            float stiction_ff = 0.0f;
            if (std::fabs(target) > p.wheel_omega_deadzone)
            {
                const float abs_omega = std::fabs(target);
                float dynamic_blend = abs_omega / 2.0f;
                if (dynamic_blend > 1.0f)
                    dynamic_blend = 1.0f;

                const float friction_torque =
                    p.wheel_stiction_torque * (1.0f - dynamic_blend) +
                    p.wheel_dynamic_friction * dynamic_blend;
                stiction_ff = p.wheel_feedforward_scale *
                              friction_torque *
                              std::tanh(target);
            }

            st_cmd.accel_filtered[i] =
                kAccelFilterAlpha * command_accel[i] +
                (1.0f - kAccelFilterAlpha) * st_cmd.accel_filtered[i];
            torque_ff = stiction_ff +
                        p.wheel_feedforward_scale *
                        p.wheel_rotor_inertia *
                        st_cmd.accel_filtered[i];
            torque_ff = clamp_float(torque_ff, -kTorqueFfLimitNm, kTorqueFfLimitNm);
        }
        st_cmd.target[i] = target;

        const float kp = std::isfinite(st_cmd.kp_override) ? st_cmd.kp_override : p.wheel_kp;
        const float kd = std::isfinite(st_cmd.kd_override) ? st_cmd.kd_override : p.wheel_kd;
        motor.Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        motor.Set_Control_Maintain_Postion(0.0f, firmware_command_omega(target), torque_ff, kp, kd);
    }

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        st_chassis.Motor_Wheel[i].TIM_Send_PeriodElapsedCallback();
}

bool ec_step(uint32_t tick)
{
    if (!st_running.load() || !st_master.is_running)
        return false;

    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);

    can_msg_t msg {};
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
    {
        while (linkx_quick_recv(&st_linkx, ch, &msg))
            dispatch_wheel_feedback(ch, msg.id, msg.data);
    }

    if ((tick % kAlivePeriodTicks) == 0U)
    {
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            st_chassis.Motor_Wheel[i].TIM_Alive_PeriodElapsedCallback();
            if (st_chassis.Motor_Wheel[i].Get_Status() != Motor_DM_Status_ENABLE)
                st_chassis.Motor_Wheel[i].CAN_Send_Enter();
        }
    }

    if ((tick % kMotorCommandPeriodTicks) == 0U)
        apply_direct_targets();

    linkx_send_pdos(&st_linkx);
    return true;
}

void run_for_ms(uint32_t &tick, uint32_t ms)
{
    auto next_wakeup = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < ms && st_running.load(); ++i)
    {
        next_wakeup += std::chrono::milliseconds(kEcPeriodMs);
        if (!ec_step(tick++))
            return;
        std::this_thread::sleep_until(next_wakeup);
    }
}

bool init_ethercat_linkx(const std::string &ifname)
{
    if (!ecat_master_init(&st_master, ifname.c_str()))
    {
        std::cerr << "[OMNI-ACCEL] ecat_master_init failed for " << ifname << "\n";
        return false;
    }

    linkx_init(&st_linkx, 1, &st_master.ctx);

    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);

    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        if (!linkx_set_can_baudrate(&st_linkx,
                                    static_cast<uint8_t>(ch),
                                    1, 2, 31, 8, 8, 1, 12, 3, 3))
        {
            std::cerr << "[OMNI-ACCEL] CAN" << ch << " FDCAN 1M/5M config failed\n";
            return false;
        }
    }

    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);

    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[OMNI-ACCEL] ecat_master_bring_online failed\n";
        return false;
    }

    return true;
}

void write_csv_header(std::ofstream &csv)
{
    csv << "mode,speed_label,active_wheel,t_s";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        csv << "," << kWheelName[i] << "_target_omega";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        csv << "," << kWheelName[i] << "_actual_omega";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        csv << "," << kWheelName[i] << "_accel";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        csv << "," << kWheelName[i] << "_torque";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        csv << "," << kWheelName[i] << "_status"
            << "," << kWheelName[i] << "_ctrl_status";
    csv << "\n";
}

void write_csv_sample(std::ofstream &csv,
                      const TrialResult &trial,
                      float t_s,
                      const std::array<float, OMNI_WHEEL_NUM> &accel)
{
    csv << trial.mode << ","
        << trial.speed_label << ","
        << trial.active_wheel << ","
        << std::fixed << std::setprecision(4) << t_s;

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        csv << "," << trial.wheel[i].target_omega;
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        csv << "," << actual_wheel_omega(i);
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        csv << "," << accel[i];
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        csv << "," << st_chassis.Motor_Wheel[i].Get_Now_Torque();
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        csv << "," << static_cast<int>(st_chassis.Motor_Wheel[i].Get_Status())
            << "," << static_cast<int>(st_chassis.Motor_Wheel[i].Get_Now_Control_Status());
    }
    csv << "\n";
}

std::array<float, OMNI_WHEEL_NUM> zero_targets()
{
    return {0.0f, 0.0f, 0.0f, 0.0f};
}

TrialResult run_trial(const Options &opt,
                      const std::string &mode,
                      const std::string &label,
                      float target_abs_omega,
                      int active_wheel,
                      uint32_t &tick,
                      std::ofstream &csv)
{
    TrialResult trial;
    trial.mode = mode;
    trial.speed_label = label;
    trial.target_abs_omega = target_abs_omega;
    trial.active_wheel = active_wheel;

    std::array<float, OMNI_WHEEL_NUM> targets {};
    targets.fill(0.0f);
    if (active_wheel >= 0)
        targets[active_wheel] = opt.direction * target_abs_omega;
    else
        targets.fill(opt.direction * target_abs_omega);

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        trial.wheel[i].target_omega = targets[i];
        trial.wheel[i].commanded = std::fabs(targets[i]) > 1e-4f;
    }

    std::cout << "\n[OMNI-ACCEL] " << mode
              << " speed=" << label
              << " target_abs=" << target_abs_omega
              << " rad/s";
    if (active_wheel >= 0)
        std::cout << " active=" << kWheelName[active_wheel];
    else
        std::cout << " active=ALL";
    std::cout << "\n";

    set_direct_targets(zero_targets());
    run_for_ms(tick, static_cast<uint32_t>(opt.settle_s * 1000.0f));
    reset_direct_command_state();

    std::array<float, OMNI_WHEEL_NUM> prev_omega {};
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        prev_omega[i] = actual_wheel_omega(i);

    std::array<int, OMNI_WHEEL_NUM> hold_count {};
    std::array<float, OMNI_WHEEL_NUM> candidate_time {};
    hold_count.fill(0);
    candidate_time.fill(0.0f);

    const uint32_t total_ms = static_cast<uint32_t>(opt.duration_s * 1000.0f);
    const int sample_div = std::max(1, 1000 / std::max(1, opt.sample_hz));
    const int reach_hold_ticks = std::max(1, static_cast<int>(std::ceil(opt.reach_hold_s * 1000.0f)));
    const float reach_threshold =
        std::max(0.5f * target_abs_omega,
                 std::min(opt.reach_frac * target_abs_omega,
                          target_abs_omega - opt.reach_abs_tol));

    set_direct_targets(targets);
    auto next_wakeup = std::chrono::steady_clock::now();
    for (uint32_t ms = 0; ms < total_ms && st_running.load(); ++ms)
    {
        next_wakeup += std::chrono::milliseconds(kEcPeriodMs);
        if (!ec_step(tick++))
            break;

        const float t_s = static_cast<float>(ms + 1U) * 0.001f;
        std::array<float, OMNI_WHEEL_NUM> accel {};

        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const float omega = actual_wheel_omega(i);
            accel[i] = (omega - prev_omega[i]) * 1000.0f;
            prev_omega[i] = omega;

            auto &wr = trial.wheel[i];
            wr.final_omega = omega;
            if (!wr.commanded)
                continue;

            const float sign = (wr.target_omega >= 0.0f) ? 1.0f : -1.0f;
            const float signed_omega = sign * omega;
            const float signed_accel = sign * accel[i];
            const float signed_torque = sign * st_chassis.Motor_Wheel[i].Get_Now_Torque();

            wr.omega_signed.add(signed_omega);
            wr.torque_signed.add(signed_torque);
            wr.max_signed_omega = std::max(wr.max_signed_omega, signed_omega);

            if (!wr.reached &&
                signed_omega >= 0.10f * target_abs_omega &&
                signed_omega <= 1.10f * target_abs_omega)
            {
                wr.accel_signed.add(signed_accel);
            }

            if (!wr.reached)
            {
                if (signed_omega >= reach_threshold)
                {
                    if (hold_count[i] == 0)
                        candidate_time[i] = t_s;
                    hold_count[i]++;
                    if (hold_count[i] >= reach_hold_ticks)
                    {
                        wr.reached = true;
                        wr.reach_time_s = candidate_time[i];
                    }
                }
                else
                {
                    hold_count[i] = 0;
                }
            }
        }

        if ((ms % static_cast<uint32_t>(sample_div)) == 0U)
            write_csv_sample(csv, trial, t_s, accel);

        if ((ms % 200U) == 0U)
        {
            std::cout << "  t=" << std::fixed << std::setprecision(2) << t_s << "s";
            for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            {
                if (!trial.wheel[i].commanded)
                    continue;
                std::cout << " W" << i << "=" << actual_wheel_omega(i);
                if (trial.wheel[i].reached)
                    std::cout << "(hit)";
            }
            std::cout << "\r" << std::flush;
        }

        if (!temperature_safe())
        {
            st_running.store(false);
            break;
        }

        std::this_thread::sleep_until(next_wakeup);
    }

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        auto &wr = trial.wheel[i];
        if (wr.commanded && target_abs_omega > 1e-4f)
        {
            wr.overshoot_pct =
                100.0f * std::max(0.0f, wr.max_signed_omega - target_abs_omega) / target_abs_omega;
        }
    }

    std::cout << "\n  result:";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const auto &wr = trial.wheel[i];
        if (!wr.commanded)
            continue;
        std::cout << " W" << i << "_reach=";
        if (wr.reached)
            std::cout << std::fixed << std::setprecision(3) << wr.reach_time_s << "s";
        else
            std::cout << "MISS";
        std::cout << " accel_mean=" << std::fixed << std::setprecision(1)
                  << wr.accel_signed.mean();
    }
    std::cout << std::defaultfloat << "\n";

    set_direct_targets(zero_targets());
    run_for_ms(tick, 300);
    return trial;
}

void disable_all(uint32_t &tick)
{
    set_direct_targets(zero_targets());
    run_for_ms(tick, 300);

    for (int cycle = 0; cycle < 50; ++cycle)
    {
        ecat_master_sync(&st_master);
        linkx_recv_pdos(&st_linkx);
        can_msg_t msg {};
        for (uint8_t ch = 0; ch < kChannelCount; ++ch)
        {
            while (linkx_quick_recv(&st_linkx, ch, &msg))
                dispatch_wheel_feedback(ch, msg.id, msg.data);
        }
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            st_chassis.Motor_Wheel[i].CAN_Send_Exit();
        linkx_send_pdos(&st_linkx);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        tick++;
    }
}

void print_status_table()
{
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        std::cout << "  " << kWheelName[i]
                  << " tx=0x" << std::hex << st_chassis.Motor_Wheel[i].DM_CAN_Tx_ID
                  << " rx=0x" << st_chassis.Motor_Wheel[i].DM_CAN_Rx_ID << std::dec
                  << " status=" << static_cast<int>(st_chassis.Motor_Wheel[i].Get_Status())
                  << " ctrl=" << static_cast<int>(st_chassis.Motor_Wheel[i].Get_Now_Control_Status())
                  << " kd=" << st_chassis.wheel_params_[i].wheel_kd
                  << " ff_scale=" << st_chassis.wheel_params_[i].wheel_feedforward_scale
                  << " accel=" << st_chassis.wheel_params_[i].wheel_accel_limit
                  << " decel=" << st_chassis.wheel_params_[i].wheel_decel_limit
                  << "\n";
    }
}

void add_trial_to_aggregate(const TrialResult &trial,
                            std::array<WheelAggregate, OMNI_WHEEL_NUM> &agg)
{
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const auto &wr = trial.wheel[i];
        if (!wr.commanded)
            continue;
        if (wr.reached)
            agg[i].reach_time.add(wr.reach_time_s);
        const double accel_mean = wr.accel_signed.mean();
        if (wr.accel_signed.n > 0 && accel_mean > 1e-3)
            agg[i].accel_mean.add(accel_mean);
    }
}

double valid_mean_across_wheels(const std::array<WheelAggregate, OMNI_WHEEL_NUM> &agg,
                                bool use_time)
{
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const Stats &s = use_time ? agg[i].reach_time : agg[i].accel_mean;
        if (s.n > 0)
        {
            sum += s.mean();
            n++;
        }
    }
    return n > 0 ? sum / static_cast<double>(n) : 0.0;
}

double suggested_multiplier(const WheelAggregate &wheel, double mean_time, double mean_accel)
{
    double factor_sum = 0.0;
    int factor_n = 0;

    if (wheel.reach_time.n > 0 && mean_time > 1e-4)
    {
        factor_sum += wheel.reach_time.mean() / mean_time;
        factor_n++;
    }

    if (wheel.accel_mean.n > 0 && wheel.accel_mean.mean() > 1e-4 && mean_accel > 1e-4)
    {
        factor_sum += mean_accel / wheel.accel_mean.mean();
        factor_n++;
    }

    if (factor_n == 0)
        return 1.0;

    const double raw = factor_sum / static_cast<double>(factor_n);
    const double damped = 1.0 + 0.35 * (raw - 1.0);
    return std::max(0.80, std::min(1.20, damped));
}

void write_trial_table(std::ostream &os,
                       const std::string &title,
                       const std::vector<TrialResult> &trials)
{
    os << title << "\n";
    for (const auto &trial : trials)
    {
        os << "  [" << trial.mode
           << " " << trial.speed_label
           << " target_abs=" << trial.target_abs_omega
           << " active=" << trial.active_wheel << "]\n";
        os << "    reach_time_s:";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const auto &wr = trial.wheel[i];
            if (!wr.commanded)
                continue;
            os << " W" << i << "=";
            if (wr.reached)
                os << wr.reach_time_s;
            else
                os << "MISS";
        }
        os << "\n";

        os << "    accel_mean_rad_s2:";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const auto &wr = trial.wheel[i];
            if (!wr.commanded)
                continue;
            os << " W" << i << "=" << wr.accel_signed.mean()
               << "+/-" << wr.accel_signed.stddev()
               << " peak=" << (wr.accel_signed.n > 0 ? wr.accel_signed.max : 0.0);
        }
        os << "\n";

        os << "    overshoot_pct:";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const auto &wr = trial.wheel[i];
            if (!wr.commanded)
                continue;
            os << " W" << i << "=" << wr.overshoot_pct;
        }
        os << "\n";
    }
}

void write_summary(const Options &opt, const std::vector<TrialResult> &results)
{
    std::ofstream out(opt.summary_path);
    std::ostream &os = out.is_open() ? out : std::cout;

    os << std::fixed << std::setprecision(4);
    os << "R2 omni wheel acceleration response summary\n"
       << "speeds_rad_s=";
    for (size_t i = 0; i < opt.speeds_rad_s.size(); ++i)
    {
        if (i > 0)
            os << ",";
        os << opt.speeds_rad_s[i];
    }
    os << " direction=" << opt.direction
       << " duration_s=" << opt.duration_s
       << " settle_s=" << opt.settle_s
       << " reach_frac=" << opt.reach_frac
       << " reach_abs_tol=" << opt.reach_abs_tol
       << " reach_hold_s=" << opt.reach_hold_s
       << " profile=" << (opt.use_profile ? 1 : 0)
       << " profile_accel=" << opt.profile_accel_limit
       << " profile_decel=" << opt.profile_decel_limit
       << " use_feedforward=" << (opt.use_feedforward ? 1 : 0)
       << "\n\n";

    os << "Current wheel params:\n";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const auto &p = st_chassis.wheel_params_[i];
        os << "  W" << i
           << " name=" << kWheelName[i]
           << " kp=" << p.wheel_kp
           << " kd=" << p.wheel_kd
           << " stiction=" << p.wheel_stiction_torque
           << " dynamic_friction=" << p.wheel_dynamic_friction
           << " inertia=" << p.wheel_rotor_inertia
           << " ff_scale=" << p.wheel_feedforward_scale
           << " accel_limit=" << p.wheel_accel_limit
           << " decel_limit=" << p.wheel_decel_limit
           << "\n";
    }

    std::vector<TrialResult> single_trials;
    std::vector<TrialResult> sync_trials;
    for (const auto &r : results)
    {
        if (r.mode == "single")
            single_trials.push_back(r);
        else if (r.mode == "sync")
            sync_trials.push_back(r);
    }

    os << "\nRaw trial metrics:\n";
    write_trial_table(os, "Single-wheel tests:", single_trials);
    write_trial_table(os, "Four-wheel synchronous tests:", sync_trials);

    os << "\nPer-speed consistency analysis:\n";
    std::array<WheelAggregate, OMNI_WHEEL_NUM> global_agg {};
    for (size_t s = 0; s < opt.speeds_rad_s.size(); ++s)
    {
        const std::string label = speed_label(s);
        std::array<WheelAggregate, OMNI_WHEEL_NUM> agg {};

        for (const auto &r : results)
        {
            if (r.speed_label == label)
            {
                add_trial_to_aggregate(r, agg);
                add_trial_to_aggregate(r, global_agg);
            }
        }

        const double mean_time = valid_mean_across_wheels(agg, true);
        const double mean_accel = valid_mean_across_wheels(agg, false);
        double min_time = std::numeric_limits<double>::infinity();
        double max_time = -std::numeric_limits<double>::infinity();
        double min_accel = std::numeric_limits<double>::infinity();
        double max_accel = -std::numeric_limits<double>::infinity();
        int time_n = 0;
        int accel_n = 0;

        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            if (agg[i].reach_time.n > 0)
            {
                min_time = std::min(min_time, agg[i].reach_time.mean());
                max_time = std::max(max_time, agg[i].reach_time.mean());
                time_n++;
            }
            if (agg[i].accel_mean.n > 0)
            {
                min_accel = std::min(min_accel, agg[i].accel_mean.mean());
                max_accel = std::max(max_accel, agg[i].accel_mean.mean());
                accel_n++;
            }
        }

        os << "  [" << label << " target_abs=" << opt.speeds_rad_s[s] << "]\n";
        os << "    mean_reach_time_s=" << mean_time;
        if (time_n > 1)
            os << " spread_s=" << (max_time - min_time);
        else
            os << " spread_s=NA";
        os << " mean_process_accel_rad_s2=" << mean_accel;
        if (accel_n > 1)
            os << " spread_accel=" << (max_accel - min_accel);
        else
            os << " spread_accel=NA";
        os << "\n";

        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const double multiplier = suggested_multiplier(agg[i], mean_time, mean_accel);
            const double current = st_chassis.wheel_params_[i].wheel_feedforward_scale;
            os << "    W" << i
               << " reach_mean=";
            if (agg[i].reach_time.n > 0)
                os << agg[i].reach_time.mean();
            else
                os << "NA";
            os << " accel_mean=";
            if (agg[i].accel_mean.n > 0)
                os << agg[i].accel_mean.mean();
            else
                os << "NA";
            os << " suggested_ff_multiplier=" << multiplier
               << " suggested_ff_scale=" << (current * multiplier)
               << "\n";
        }
    }

    const double global_mean_time = valid_mean_across_wheels(global_agg, true);
    const double global_mean_accel = valid_mean_across_wheels(global_agg, false);
    os << "\nCombined recommendation:\n";
    os << "  The multiplier is damped and clamped to 0.80..1.20; apply in small steps and rerun.\n";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const double multiplier = suggested_multiplier(global_agg[i], global_mean_time, global_mean_accel);
        const double current = st_chassis.wheel_params_[i].wheel_feedforward_scale;
        os << "  W" << i
           << " current_ff_scale=" << current
           << " suggested_ff_scale=" << (current * multiplier)
           << " multiplier=" << multiplier;
        if (global_agg[i].reach_time.n == 0)
            os << "  NO_REACH_DATA";
        os << "\n";
    }

    os << "\nInterpretation:\n"
       << "  1) A larger reach_time means that wheel is slower to accelerate to the same target speed.\n"
       << "  2) A smaller process_accel mean means the wheel accelerates more weakly during the ramp.\n"
       << "  3) If a wheel is slow in both single and sync tests, tune its own wheel_feedforward_scale/Kd first.\n"
       << "  4) If sync differs from single, check bus timing, power supply sag, or shared current limits.\n"
       << "  5) Large overshoot means the wheel may need less feedforward or Kd before increasing speed.\n"
       << "  6) At low speed, derivative acceleration is noisy; trust reach_time more than accel_mean.\n";

    if (out.is_open())
        std::cout << "[OMNI-ACCEL] summary written to " << opt.summary_path << "\n";
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    if (cli_has(argc, argv, "help"))
    {
        print_usage(argv[0]);
        return 0;
    }

    Options opt = parse_options(argc, argv);
    if (!opt.run_single && !opt.run_sync)
    {
        std::cerr << "[OMNI-ACCEL] both --single and --sync are disabled.\n";
        return 1;
    }

    st_cmd.use_feedforward = opt.use_feedforward;
    st_cmd.use_profile = opt.use_profile;
    st_cmd.profile_accel_limit = opt.profile_accel_limit;
    st_cmd.profile_decel_limit = opt.profile_decel_limit;
    st_cmd.kp_override = opt.kp_override;
    st_cmd.kd_override = opt.kd_override;

    std::cout << "===============================================\n"
              << "  R2 Omni Wheel Acceleration Response Test\n"
              << "  IFNAME     : " << opt.ifname << "\n"
              << "  speeds     : ";
    for (size_t i = 0; i < opt.speeds_rad_s.size(); ++i)
    {
        if (i > 0)
            std::cout << ",";
        std::cout << opt.speeds_rad_s[i];
    }
    std::cout << " rad/s\n"
              << "  duration   : " << opt.duration_s << " s per step\n"
              << "  settle     : " << opt.settle_s << " s before each step\n"
              << "  reach      : " << opt.reach_frac
              << " or abs_tol=" << opt.reach_abs_tol
              << " for " << opt.reach_hold_s << " s\n"
              << "  profile    : " << (opt.use_profile ? "ON" : "OFF")
              << " accel=" << opt.profile_accel_limit
              << " decel=" << opt.profile_decel_limit << "\n"
              << "  feedforward: " << (opt.use_feedforward ? "ON" : "OFF") << "\n"
              << "  csv        : " << opt.csv_path << "\n"
              << "===============================================\n"
              << "[SAFETY] Wheels must be suspended and clear. Press Ctrl+C to stop.\n";

    if (!init_ethercat_linkx(opt.ifname))
        return 2;

    st_chassis.Init(&st_linkx);
    st_chassis.Init_Motor_Params();
    if (opt.has_ff_scale_override)
    {
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            st_chassis.wheel_params_[i].wheel_feedforward_scale = opt.ff_scale_override[i];
    }
    if (opt.has_kd_override_list)
    {
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            st_chassis.wheel_params_[i].wheel_kd = opt.kd_override_list[i];
    }
    if (opt.use_profile)
    {
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            st_chassis.wheel_params_[i].wheel_accel_limit = opt.profile_accel_limit;
            st_chassis.wheel_params_[i].wheel_decel_limit = opt.profile_decel_limit;
        }
    }
    st_chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        st_chassis.Motor_Wheel[i].Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        st_chassis.Motor_Wheel[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    uint32_t tick = 0;
    std::cout << "[OMNI-ACCEL] enabling wheel motors and waiting for feedback...\n";
    run_for_ms(tick, 1500);
    print_status_table();

    std::ofstream csv(opt.csv_path);
    if (!csv.is_open())
    {
        std::cerr << "[OMNI-ACCEL] failed to open CSV: " << opt.csv_path << "\n";
        disable_all(tick);
        return 3;
    }
    write_csv_header(csv);

    std::vector<TrialResult> results;
    for (size_t s = 0; s < opt.speeds_rad_s.size() && st_running.load(); ++s)
    {
        const std::string label = speed_label(s);
        const float target = opt.speeds_rad_s[s];

        if (opt.run_single)
        {
            for (int wheel = 0; wheel < OMNI_WHEEL_NUM && st_running.load(); ++wheel)
            {
                results.push_back(run_trial(opt, "single", label, target, wheel, tick, csv));
            }
        }

        if (opt.run_sync && st_running.load())
            results.push_back(run_trial(opt, "sync", label, target, -1, tick, csv));
    }

    std::cout << "\n[OMNI-ACCEL] stopping motors...\n";
    disable_all(tick);
    csv.flush();
    csv.close();

    write_summary(opt, results);
    std::cout << "[OMNI-ACCEL] csv written to " << opt.csv_path << "\n";
    std::cout << "[OMNI-ACCEL] done.\n";
    return 0;
}

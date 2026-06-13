// lift_raise_drive_test_main.cpp
//
// R2 lift combined test:
//   - DM3519 lift motor position sweep on the measured rod side range
//     raise_angle = [-1, -15] rad. Motor-side target = rod_angle * 3.
//   - DM2325 lift drive motors run a normal differential-drive velocity test
//     at the same time, with the same acceleration/deceleration limits used by
//     Class_Chariot_Lift.
//
// Usage:
//   sudo IFNAME=enp86s0 ros2 run linkx_soem_demo lift_raise_drive_test --module both
//   sudo IFNAME=enp86s0 ./install/linkx_soem_demo/lib/linkx_soem_demo/lift_raise_drive_test \
//        --module front --cycles 1 --forward 0.25 --rod-start -1 --rod-end -15
//
// Output:
//   var_data/lift_raise_drive_test_<timestamp>.csv

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
#include <map>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include "crt_lift.h"
#include "dvc_motor_dm.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "math.h"

namespace
{
constexpr int kChannelCount = 4;
constexpr uint32_t kEcPeriodMs = 1;
constexpr uint32_t kCommandPeriodTicks = 2;
constexpr uint32_t kAlivePeriodTicks = 100;
constexpr float kCommandDtS = 0.002f;
constexpr float kThermalLimitC = 70.0f;

// Same lift-side defaults as Class_Chariot_Lift::Init_Motor_Params().
constexpr float kLiftMotorToRodRatio = 3.0f;
constexpr float kRodRangeHigh = -1.0f;
constexpr float kRodRangeLow = -15.0f;
constexpr float kLiftRodMaxSpeed = 8.0f;
constexpr float kLiftKp = 15.0f;
constexpr float kLiftKd = 1.0f;
constexpr float kIdentifyLiftKp = 20.0f;
constexpr float kIdentifyLiftKd = 1.2f;
constexpr float kFeedforwardLiftSpeed = 5.0f;
constexpr float kFeedforwardTorqueLimitNm = 3.0f;
constexpr float kSCurvePeakVelocityScale = 1.875f;
constexpr float kIdentifyDynamicVelFrac = 0.35f;
constexpr float kIdentifyMinMotorAccelRadS2 = 5.0f;

// Same DM2325 differential-drive defaults as Class_Chariot_Lift.
constexpr float kDriveWheelRadius = 0.0761f;
constexpr float kDriveTrackWidth = 0.420f;
constexpr float kDriveMaxForward = 1.0f;
constexpr float kDriveMaxYaw = 2.0f;
constexpr float kDriveMaxWheelOmega = 80.0f;
constexpr float kDriveKp = 0.0f;
constexpr float kDriveKd = 1.2f;
constexpr float kDriveDeadzone = 0.05f;
constexpr float kDriveAccel = 300.0f;
constexpr float kDriveDecel = 600.0f;
constexpr float kDriveReachFrac = 0.95f;

ecat_master_t st_master {};
linkx_t st_linkx {};
Class_Chariot_Lift st_lift;
std::atomic<bool> st_running {true};

enum class ModuleSelection
{
    Front,
    Rear,
    Both,
};

enum class TestSuite
{
    Single,
    Param,
    Lift,
    Identify,
    Feedforward,
    SCurve,
    Fast,
    Contact,
};

enum class FeedforwardMode
{
    None,
    Hold,
    HoldAndFriction,
};

enum class LiftCommandProfile
{
    Trapezoid,
    SCurve,
};

struct LiftFeedforwardTable
{
    std::array<float, 5> angles;
    std::array<float, 5> hold_torque_nm;
    float raise_dynamic_nm;
    float retract_dynamic_nm;
};

struct ModuleInfo
{
    Enum_Chariot_Lift_Module module;
    const char *name;
    uint8_t can_channel;
};

const std::array<ModuleInfo, CHARIOT_LIFT_MODULE_NUM> kModuleInfo {{
    {CHARIOT_LIFT_MODULE_FRONT, "front", 0},
    {CHARIOT_LIFT_MODULE_REAR,  "rear",  1},
}};

// Candidate DM3519 motor-side feedforward values measured by --suite identify on 2026-06-12.
const std::array<LiftFeedforwardTable, CHARIOT_LIFT_MODULE_NUM> kLiftFeedforward {{
    LiftFeedforwardTable {
        std::array<float, 5> {{-15.0f, -11.5f, -8.0f, -4.5f, -1.0f}},
        std::array<float, 5> {{-1.0905f, -0.5794f, -0.5545f, -0.6979f, 0.3695f}},
        -0.4478f,
        0.3218f,
    },
    LiftFeedforwardTable {
        std::array<float, 5> {{-15.0f, -11.5f, -8.0f, -4.5f, -1.0f}},
        std::array<float, 5> {{-0.9960f, -0.6412f, -0.5545f, -0.5803f, 0.4385f}},
        -0.3775f,
        0.4027f,
    },
}};

struct Options
{
    std::string ifname = "enp86s0";
    ModuleSelection module = ModuleSelection::Both;
    TestSuite suite = TestSuite::Lift;
    int cycles = 1;
    float rod_start = kRodRangeHigh;
    float rod_end = kRodRangeLow;
    float settle_s = 1.0f;
    float hold_s = 0.75f;
    float lift_speed = kLiftRodMaxSpeed;
    float lift_kp = kLiftKp;
    float lift_kd = kLiftKd;
    bool lift_velocity_ff = false;
    float forward_m_s = 0.25f;
    float yaw_rad_s = 0.0f;
    float drive_kp = kDriveKp;
    float drive_kd = kDriveKd;
    float drive_accel = kDriveAccel;
    float drive_decel = kDriveDecel;
    bool drive_enable = false;
    FeedforwardMode feedforward_mode = FeedforwardMode::None;
    LiftCommandProfile lift_profile = LiftCommandProfile::Trapezoid;
    float identify_speed = 3.0f;
    float identify_hold_s = 1.2f;
    float identify_breakaway_vel = 0.20f;
    std::vector<float> identify_angles {-1.0f, -4.5f, -8.0f, -11.5f, -15.0f};
    float contact_speed = 0.8f;
    float contact_threshold_nm = 0.18f;
    float contact_ignore_s = 0.5f;
    float contact_min_travel = 6.0f;
    float contact_hold_s = 0.0f;
    float contact_preload_rad = 0.0f;
    int contact_confirm_ms = 120;
    int sample_hz = 100;
    bool record = true;
    bool exit_on_stop = true;
    std::string csv_path;
    std::string summary_path;
};

struct TestCase
{
    std::string label;
    float lift_speed = kLiftRodMaxSpeed;
    float lift_kp = kLiftKp;
    float lift_kd = kLiftKd;
    float forward_m_s = 0.25f;
    float yaw_rad_s = 0.0f;
    float drive_kp = kDriveKp;
    float drive_kd = kDriveKd;
    float drive_accel = kDriveAccel;
    float drive_decel = kDriveDecel;
    bool drive_enable = true;
    FeedforwardMode feedforward_mode = FeedforwardMode::None;
    LiftCommandProfile lift_profile = LiftCommandProfile::Trapezoid;
    bool lift_velocity_ff = false;
};

struct ModuleCommand
{
    float lift_rod_target = kRodRangeHigh;
    float lift_rod_cmd = kRodRangeHigh;
    float lift_rod_omega_cmd = 0.0f;
    float left_target = 0.0f;
    float right_target = 0.0f;
    float left_cmd = 0.0f;
    float right_cmd = 0.0f;
    float lift_torque_ff = 0.0f;
};

std::array<ModuleCommand, CHARIOT_LIFT_MODULE_NUM> st_cmd {};
bool st_command_enabled = false;

struct SCurveState
{
    bool active = false;
    float start = kRodRangeHigh;
    float target = kRodRangeHigh;
    float duration_s = 0.0f;
    float elapsed_s = 0.0f;
};

std::array<SCurveState, CHARIOT_LIFT_MODULE_NUM> st_scurve {};

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
        ++n;
    }

    double mean() const
    {
        return n > 0 ? sum / static_cast<double>(n) : 0.0;
    }

    double rms() const
    {
        return n > 0 ? std::sqrt(sum_sq / static_cast<double>(n)) : 0.0;
    }

    double stddev() const
    {
        if (n < 2)
            return 0.0;
        const double m = mean();
        return std::sqrt(std::max(0.0, sum_sq / static_cast<double>(n) - m * m));
    }

    double max_abs() const
    {
        if (n == 0)
            return 0.0;
        return std::max(std::fabs(min), std::fabs(max));
    }
};

struct PhaseModuleMetrics
{
    std::string phase;
    Enum_Chariot_Lift_Module module = CHARIOT_LIFT_MODULE_FRONT;
    const char *module_name = "";
    uint8_t can_channel = 0;

    float final_t_s = 0.0f;
    float final_rod_target = 0.0f;
    float final_rod_cmd = 0.0f;
    float final_rod_actual = 0.0f;
    float final_left_target = 0.0f;
    float final_left_cmd = 0.0f;
    float final_left_actual = 0.0f;
    float final_right_target = 0.0f;
    float final_right_cmd = 0.0f;
    float final_right_actual = 0.0f;

    Stats rod_actual;
    Stats rod_error;
    Stats rod_cmd_velocity;
    Stats rod_velocity_from_position;
    Stats rod_accel_from_position;
    Stats lift_torque;
    Stats lift_torque_ff;
    Stats left_actual;
    Stats left_error;
    Stats left_accel;
    Stats left_torque;
    Stats right_actual;
    Stats right_error;
    Stats right_accel;
    Stats right_torque;
    Stats temperature_c;

    bool has_prev = false;
    float prev_t_s = 0.0f;
    float prev_rod_actual = 0.0f;
    float prev_rod_velocity = 0.0f;
    float prev_left_actual = 0.0f;
    float prev_right_actual = 0.0f;

    bool left_reached = false;
    bool right_reached = false;
    float left_reach_time_s = std::numeric_limits<float>::quiet_NaN();
    float right_reach_time_s = std::numeric_limits<float>::quiet_NaN();

    float lift_speed = 0.0f;
    float lift_kp = 0.0f;
    float lift_kd = 0.0f;
    bool lift_velocity_ff = false;
    float forward_m_s = 0.0f;
    float yaw_rad_s = 0.0f;
    float drive_kp = 0.0f;
    float drive_kd = 0.0f;
    float drive_accel = 0.0f;
    float drive_decel = 0.0f;
    bool drive_enable = false;
    FeedforwardMode feedforward_mode = FeedforwardMode::None;
    LiftCommandProfile lift_profile = LiftCommandProfile::Trapezoid;
};

std::vector<PhaseModuleMetrics> st_metrics;

struct IdentifySample
{
    std::string phase;
    Enum_Chariot_Lift_Module module = CHARIOT_LIFT_MODULE_FRONT;
    const char *module_name = "";
    uint8_t can_channel = 0;
    float t_s = 0.0f;
    float rod_target = 0.0f;
    float rod_cmd = 0.0f;
    float rod_actual = 0.0f;
    float rod_error = 0.0f;
    float rod_velocity = 0.0f;
    float rod_accel = 0.0f;
    float torque_nm = 0.0f;
    float lift_kp = 0.0f;
    float lift_kd = 0.0f;
    float lift_speed = 0.0f;
};

struct IdentifyHoldPoint
{
    Enum_Chariot_Lift_Module module = CHARIOT_LIFT_MODULE_FRONT;
    const char *module_name = "";
    float angle = 0.0f;
    Stats torque;
    Stats rod_error;
};

std::vector<IdentifySample> st_identify_samples;

struct IdentifyPrevState
{
    bool valid = false;
    bool has_velocity = false;
    std::string phase;
    float t_s = 0.0f;
    float rod_actual = 0.0f;
    float rod_velocity = 0.0f;
};

std::array<IdentifyPrevState, CHARIOT_LIFT_MODULE_NUM> st_identify_prev {};

struct ContactResult
{
    std::string phase;
    Enum_Chariot_Lift_Module module = CHARIOT_LIFT_MODULE_FRONT;
    const char *module_name = "";
    uint8_t can_channel = 0;
    bool detected = false;
    bool timeout = false;
    float detect_t_s = 0.0f;
    float detect_rod_cmd = 0.0f;
    float detect_rod_actual = 0.0f;
    float detect_rod_error = 0.0f;
    float detect_torque_nm = 0.0f;
    float detect_hold_torque_nm = 0.0f;
    float detect_residual_nm = 0.0f;
    float detect_lowering_load_nm = 0.0f;
    float detect_motor_omega = 0.0f;
    Stats torque;
    Stats hold_torque;
    Stats residual;
    Stats lowering_load;
    Stats rod_error;
    Stats rod_velocity;
    Stats hold_residual;
    Stats hold_lowering_load;
};

std::vector<ContactResult> st_contact_results;

void on_signal(int)
{
    st_running.store(false);
    st_master.is_running = false;
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
    {
        if (k == argv[i])
            return true;
    }
    return false;
}

bool cli_specified(int argc, char **argv, const char *key)
{
    const std::string k1 = std::string("--") + key;
    const std::string k2 = k1 + "=";

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == k1 || arg.compare(0, k2.size(), k2) == 0)
            return true;
    }
    return false;
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

bool parse_bool(const char *text, bool fallback)
{
    if (text == nullptr)
        return fallback;
    const std::string v(text);
    if (v == "1" || v == "true" || v == "TRUE" || v == "on" || v == "ON" ||
        v == "yes" || v == "YES")
        return true;
    if (v == "0" || v == "false" || v == "FALSE" || v == "off" || v == "OFF" ||
        v == "no" || v == "NO")
        return false;
    return fallback;
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

float clamp_float(float value, float lo, float hi)
{
    if (value < lo)
        return lo;
    if (value > hi)
        return hi;
    return value;
}

bool starts_with(const std::string &text, const char *prefix)
{
    const std::string p(prefix);
    return text.compare(0, p.size(), p) == 0;
}

std::string angle_token(float angle)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(1) << std::fabs(angle);
    std::string token = os.str();
    std::replace(token.begin(), token.end(), '.', 'p');
    return std::string(angle < 0.0f ? "m" : "p") + token;
}

std::string angle_list_text(const std::vector<float> &angles)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < angles.size(); ++i)
    {
        if (i > 0)
            os << ",";
        os << angles[i];
    }
    return os.str();
}

std::vector<float> parse_float_list(const std::string &text,
                                    const std::vector<float> &fallback)
{
    std::vector<float> values;
    std::stringstream ss(text);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        char *end = nullptr;
        const float value = std::strtof(item.c_str(), &end);
        if (end != item.c_str() && std::isfinite(value))
            values.push_back(value);
    }

    return values.empty() ? fallback : values;
}

std::vector<float> sanitize_identify_angles(std::vector<float> angles,
                                            Options const &opt)
{
    std::vector<float> result;
    result.reserve(angles.size());
    for (float angle : angles)
        result.push_back(clamp_float(angle, kRodRangeLow, kRodRangeHigh));

    std::sort(result.begin(), result.end(), [](float a, float b) {
        return a > b;
    });

    std::vector<float> unique;
    for (float angle : result)
    {
        if (unique.empty() || std::fabs(unique.back() - angle) > 0.05f)
            unique.push_back(angle);
    }

    if (unique.size() < 2)
    {
        const float high = std::max(opt.rod_start, opt.rod_end);
        const float low = std::min(opt.rod_start, opt.rod_end);
        unique = {high, low};
    }

    return unique;
}

float ramp_to(float current, float target, float accel_limit, float decel_limit, float dt)
{
    const float delta = target - current;
    const float abs_delta = std::fabs(delta);
    if (abs_delta < 1e-5f)
        return target;

    const bool changing_direction =
        (current * target < 0.0f) && (std::fabs(current) > 1e-4f);
    const bool reducing_speed = std::fabs(target) < std::fabs(current);
    const float limit = (changing_direction || reducing_speed) ? decel_limit : accel_limit;
    const float max_step = limit * dt;

    if (abs_delta <= max_step)
        return target;

    return current + ((delta > 0.0f) ? max_step : -max_step);
}

float scurve_fraction(float u)
{
    u = clamp_float(u, 0.0f, 1.0f);
    return (10.0f * u * u * u) -
           (15.0f * u * u * u * u) +
           (6.0f * u * u * u * u * u);
}

float scurve_fraction_derivative(float u)
{
    u = clamp_float(u, 0.0f, 1.0f);
    return (30.0f * u * u) -
           (60.0f * u * u * u) +
           (30.0f * u * u * u * u);
}

float profile_distance_duration_s(float distance, Options const &opt)
{
    const float safe_speed = std::max(0.1f, opt.lift_speed);
    const float abs_distance = std::fabs(distance);
    if (abs_distance < 1e-4f)
        return 0.0f;

    if (opt.lift_profile == LiftCommandProfile::SCurve)
        return kSCurvePeakVelocityScale * abs_distance / safe_speed;

    return abs_distance / safe_speed;
}

float profile_move_duration_s(float from_angle, float to_angle, Options const &opt)
{
    return profile_distance_duration_s(to_angle - from_angle, opt);
}

bool parse_module_selection(const std::string &text, ModuleSelection &selection)
{
    if (text == "front" || text == "ch0" || text == "0")
    {
        selection = ModuleSelection::Front;
        return true;
    }
    if (text == "rear" || text == "ch1" || text == "1")
    {
        selection = ModuleSelection::Rear;
        return true;
    }
    if (text == "both" || text == "all")
    {
        selection = ModuleSelection::Both;
        return true;
    }
    return false;
}

bool parse_test_suite(const std::string &text, TestSuite &suite)
{
    if (text == "single" || text == "one" || text == "debug")
    {
        suite = TestSuite::Single;
        return true;
    }
    if (text == "param" || text == "params" || text == "sweep" || text == "full")
    {
        suite = TestSuite::Param;
        return true;
    }
    if (text == "lift" || text == "raise" || text == "ground")
    {
        suite = TestSuite::Lift;
        return true;
    }
    if (text == "identify" || text == "id" || text == "friction")
    {
        suite = TestSuite::Identify;
        return true;
    }
    if (text == "ff" || text == "feedforward" || text == "validate" || text == "validation")
    {
        suite = TestSuite::Feedforward;
        return true;
    }
    if (text == "scurve" || text == "s_curve" || text == "s-curve" || text == "s")
    {
        suite = TestSuite::SCurve;
        return true;
    }
    if (text == "fast" || text == "highspeed" || text == "high-speed" || text == "hs")
    {
        suite = TestSuite::Fast;
        return true;
    }
    if (text == "contact" || text == "touch" || text == "force" || text == "load")
    {
        suite = TestSuite::Contact;
        return true;
    }
    return false;
}

bool module_selected(ModuleSelection selection, Enum_Chariot_Lift_Module module)
{
    return selection == ModuleSelection::Both ||
           (selection == ModuleSelection::Front && module == CHARIOT_LIFT_MODULE_FRONT) ||
           (selection == ModuleSelection::Rear && module == CHARIOT_LIFT_MODULE_REAR);
}

const char *module_selection_name(ModuleSelection selection)
{
    switch (selection)
    {
        case ModuleSelection::Front: return "front";
        case ModuleSelection::Rear:  return "rear";
        case ModuleSelection::Both:  return "both";
        default:                     return "unknown";
    }
}

const char *test_suite_name(TestSuite suite)
{
    switch (suite)
    {
        case TestSuite::Single: return "single";
        case TestSuite::Param:  return "param";
        case TestSuite::Lift:   return "lift";
        case TestSuite::Identify: return "identify";
        case TestSuite::Feedforward: return "ff";
        case TestSuite::SCurve: return "scurve";
        case TestSuite::Fast:   return "fast";
        case TestSuite::Contact: return "contact";
        default:                return "unknown";
    }
}

const char *feedforward_mode_name(FeedforwardMode mode)
{
    switch (mode)
    {
        case FeedforwardMode::None:            return "pd_only";
        case FeedforwardMode::Hold:            return "hold";
        case FeedforwardMode::HoldAndFriction: return "hold_friction";
        default:                               return "unknown";
    }
}

const char *lift_profile_name(LiftCommandProfile profile)
{
    switch (profile)
    {
        case LiftCommandProfile::Trapezoid: return "trapezoid";
        case LiftCommandProfile::SCurve:    return "scurve";
        default:                            return "unknown";
    }
}

std::vector<TestCase> build_test_cases(Options const &opt)
{
    if (opt.suite == TestSuite::Identify ||
        opt.suite == TestSuite::Contact)
        return {};

    if (opt.suite == TestSuite::Single)
    {
        return {{
            "single",
            opt.lift_speed,
            opt.lift_kp,
            opt.lift_kd,
            opt.forward_m_s,
            opt.yaw_rad_s,
            opt.drive_kp,
            opt.drive_kd,
            opt.drive_accel,
            opt.drive_decel,
            opt.drive_enable,
            opt.feedforward_mode,
            opt.lift_profile,
            opt.lift_velocity_ff,
        }};
    }

    if (opt.suite == TestSuite::Feedforward)
    {
        return {
            {"ff_pd_only_s05",       kFeedforwardLiftSpeed, kIdentifyLiftKp, kIdentifyLiftKd, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
            {"ff_hold_s05",          kFeedforwardLiftSpeed, kIdentifyLiftKp, kIdentifyLiftKd, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::Trapezoid},
            {"ff_hold_friction_s05", kFeedforwardLiftSpeed, kIdentifyLiftKp, kIdentifyLiftKd, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::HoldAndFriction, LiftCommandProfile::Trapezoid},
        };
    }

    if (opt.suite == TestSuite::SCurve)
    {
        return {
            {"scurve_trap_hold_s05_posonly",     5.0f, kIdentifyLiftKp, kIdentifyLiftKd, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::Trapezoid, false},
            {"scurve_minjerk_hold_s08_posonly",  8.0f, kIdentifyLiftKp, kIdentifyLiftKd, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, false},
            {"scurve_minjerk_hold_s08_posvel",   8.0f, kIdentifyLiftKp, kIdentifyLiftKd, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"scurve_minjerk_hold_s09_posvel",   9.0f, kIdentifyLiftKp, kIdentifyLiftKd, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"scurve_minjerk_hold_s10_posonly", 10.0f, kIdentifyLiftKp, kIdentifyLiftKd, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, false},
            {"scurve_minjerk_hold_s10_posvel",  10.0f, kIdentifyLiftKp, kIdentifyLiftKd, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"scurve_minjerk_hold_s12_posvel",  12.0f, kIdentifyLiftKp, kIdentifyLiftKd, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
        };
    }

    if (opt.suite == TestSuite::Fast)
    {
        return {
            {"fast_s12_kp20_kd12_hold",      12.0f, 20.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s13_kp20_kd12_hold",      13.0f, 20.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s14_kp20_kd12_hold",      14.0f, 20.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s15_kp20_kd12_hold",      15.0f, 20.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s18_kp20_kd12_hold",      18.0f, 20.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s20_kp20_kd12_hold",      20.0f, 20.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s13_kp25_kd12_hold",      13.0f, 25.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s14_kp25_kd12_hold",      14.0f, 25.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s20_kp20_kd16_hold",      20.0f, 20.0f, 1.6f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s20_kp25_kd12_hold",      20.0f, 25.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s20_kp25_kd16_hold",      20.0f, 25.0f, 1.6f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s20_kp30_kd16_hold",      20.0f, 30.0f, 1.6f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::Hold, LiftCommandProfile::SCurve, true},
            {"fast_s20_kp25_kd16_friction",  20.0f, 25.0f, 1.6f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::HoldAndFriction, LiftCommandProfile::SCurve, true},
        };
    }

    if (opt.suite == TestSuite::Lift)
    {
        return {
            {"lift_kp15_kd10_s03", 3.0f, 15.0f, 1.0f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
            {"lift_kp15_kd10_s05", 5.0f, 15.0f, 1.0f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
            {"lift_kp20_kd12_s03", 3.0f, 20.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
            {"lift_kp20_kd12_s05", 5.0f, 20.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
            {"lift_kp20_kd12_s08", 8.0f, 20.0f, 1.2f, 0.0f, 0.0f, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, false, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
        };
    }

    return {
        {"kp10_kd10_f025",  opt.lift_speed, 10.0f, 1.0f, 0.25f, opt.yaw_rad_s, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, true, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
        {"kp15_kd10_f025",  opt.lift_speed, 15.0f, 1.0f, 0.25f, opt.yaw_rad_s, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, true, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
        {"kp20_kd12_f025",  opt.lift_speed, 20.0f, 1.2f, 0.25f, opt.yaw_rad_s, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, true, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
        {"kp15_kd10_f050",  opt.lift_speed, 15.0f, 1.0f, 0.50f, opt.yaw_rad_s, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, true, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
        {"kp15_kd10_f080",  opt.lift_speed, 15.0f, 1.0f, 0.80f, opt.yaw_rad_s, opt.drive_kp, opt.drive_kd, opt.drive_accel, opt.drive_decel, true, FeedforwardMode::None, LiftCommandProfile::Trapezoid},
    };
}

const char *dm_status_name(Enum_Motor_DM_Status status)
{
    return (status == Motor_DM_Status_ENABLE) ? "ONLINE" : "OFFLINE";
}

const char *dm_ctrl_name(Enum_Motor_DM_Control_Status_Normal status)
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

Class_Motor_DM_Normal &lift_motor(Enum_Chariot_Lift_Module module)
{
    return st_lift.Motor_Lift[static_cast<int>(module)];
}

Class_Motor_DM_Normal &drive_left_motor(Enum_Chariot_Lift_Module module)
{
    return st_lift.Motor_Drive_Left[static_cast<int>(module)];
}

Class_Motor_DM_Normal &drive_right_motor(Enum_Chariot_Lift_Module module)
{
    return st_lift.Motor_Drive_Right[static_cast<int>(module)];
}

float temp_k_to_c(float temp_k)
{
    if (temp_k < 200.0f)
        return std::numeric_limits<float>::quiet_NaN();
    return temp_k - CELSIUS_TO_KELVIN;
}

void ensure_motor_enabled(Class_Motor_DM_Normal &motor)
{
    const auto ctrl = motor.Get_Now_Control_Status();
    if (ctrl == Motor_DM_Control_Status_ENABLE)
        return;

    if (ctrl == Motor_DM_Control_Status_DISABLE)
    {
        motor.CAN_Send_Clear_Error();
        motor.CAN_Send_Enter();
    }
    else
    {
        motor.CAN_Send_Clear_Error();
    }
}

void alive_and_enable_selected(Options const &opt)
{
    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        lift_motor(info.module).TIM_Alive_PeriodElapsedCallback();
        drive_left_motor(info.module).TIM_Alive_PeriodElapsedCallback();
        drive_right_motor(info.module).TIM_Alive_PeriodElapsedCallback();

        ensure_motor_enabled(lift_motor(info.module));
        if (opt.drive_enable)
        {
            ensure_motor_enabled(drive_left_motor(info.module));
            ensure_motor_enabled(drive_right_motor(info.module));
        }
        else
        {
            drive_left_motor(info.module).CAN_Send_Exit();
            drive_right_motor(info.module).CAN_Send_Exit();
        }
    }
}

void compute_drive_targets_for_module(Options const &opt,
                                      float forward_m_s,
                                      float yaw_rad_s,
                                      ModuleCommand &cmd)
{
    if (!opt.drive_enable)
    {
        cmd.left_target = 0.0f;
        cmd.right_target = 0.0f;
        return;
    }

    const float forward_cmd = clamp_float(forward_m_s, -kDriveMaxForward, kDriveMaxForward);
    const float yaw_cmd = clamp_float(yaw_rad_s, -kDriveMaxYaw, kDriveMaxYaw);

    // Match Class_Chariot_Lift::Kinematics_Diff_Resolution().
    const float forward = -forward_cmd;
    const float left_linear = forward - yaw_cmd * kDriveTrackWidth * 0.5f;
    const float right_linear = forward + yaw_cmd * kDriveTrackWidth * 0.5f;

    cmd.left_target = left_linear / kDriveWheelRadius;
    cmd.right_target = right_linear / kDriveWheelRadius;

    const float max_abs = std::max(std::fabs(cmd.left_target), std::fabs(cmd.right_target));
    if (max_abs > kDriveMaxWheelOmega && max_abs > 1e-4f)
    {
        const float scale = kDriveMaxWheelOmega / max_abs;
        cmd.left_target *= scale;
        cmd.right_target *= scale;
    }

    if (std::fabs(cmd.left_target) < kDriveDeadzone)
        cmd.left_target = 0.0f;
    if (std::fabs(cmd.right_target) < kDriveDeadzone)
        cmd.right_target = 0.0f;
}

void set_phase_targets(Options const &opt,
                       float rod_target,
                       float forward_m_s,
                       float yaw_rad_s)
{
    const float safe_rod_target = clamp_float(rod_target, kRodRangeLow, kRodRangeHigh);

    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        ModuleCommand &cmd = st_cmd[static_cast<int>(info.module)];
        cmd.lift_rod_target = safe_rod_target;
        cmd.lift_rod_omega_cmd = 0.0f;
        if (opt.lift_profile == LiftCommandProfile::SCurve)
        {
            SCurveState &profile = st_scurve[static_cast<int>(info.module)];
            profile.start = cmd.lift_rod_cmd;
            profile.target = safe_rod_target;
            profile.elapsed_s = 0.0f;
            profile.duration_s = profile_move_duration_s(profile.start, profile.target, opt);
            profile.active = profile.duration_s > 1e-4f;
            if (!profile.active)
                cmd.lift_rod_cmd = safe_rod_target;
        }
        compute_drive_targets_for_module(opt, forward_m_s, yaw_rad_s, cmd);
    }
}

float interpolate_array_table(const std::array<float, 5> &xs,
                              const std::array<float, 5> &ys,
                              float x)
{
    if (x <= xs.front())
        return ys.front();
    if (x >= xs.back())
        return ys.back();

    for (size_t i = 1; i < xs.size(); ++i)
    {
        if (x <= xs[i])
        {
            const float span = xs[i] - xs[i - 1U];
            const float u = (std::fabs(span) > 1e-5f) ? (x - xs[i - 1U]) / span : 0.0f;
            return ys[i - 1U] + u * (ys[i] - ys[i - 1U]);
        }
    }

    return ys.back();
}

float lift_hold_torque(Enum_Chariot_Lift_Module module, float rod_angle)
{
    const auto &table = kLiftFeedforward[static_cast<int>(module)];
    return interpolate_array_table(table.angles, table.hold_torque_nm, rod_angle);
}

float lift_feedforward_torque(Options const &opt,
                              Enum_Chariot_Lift_Module module,
                              float rod_cmd,
                              float rod_delta_before_ramp)
{
    if (opt.feedforward_mode == FeedforwardMode::None)
        return 0.0f;

    const auto &table = kLiftFeedforward[static_cast<int>(module)];
    float torque_ff = lift_hold_torque(module, rod_cmd);

    if (opt.feedforward_mode == FeedforwardMode::HoldAndFriction)
    {
        if (rod_delta_before_ramp < -1e-3f)
            torque_ff += table.raise_dynamic_nm;
        else if (rod_delta_before_ramp > 1e-3f)
            torque_ff += table.retract_dynamic_nm;
    }

    return clamp_float(torque_ff, -kFeedforwardTorqueLimitNm, kFeedforwardTorqueLimitNm);
}

void apply_commands(Options const &opt)
{
    if (!st_command_enabled)
        return;

    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        const int idx = static_cast<int>(info.module);
        ModuleCommand &cmd = st_cmd[idx];
        const float lift_delta_before_ramp = cmd.lift_rod_target - cmd.lift_rod_cmd;
        cmd.lift_rod_omega_cmd = 0.0f;
        if (opt.lift_profile == LiftCommandProfile::SCurve)
        {
            SCurveState &profile = st_scurve[idx];
            if (profile.active)
            {
                profile.elapsed_s += kCommandDtS;
                const float u = (profile.duration_s > 1e-5f) ?
                    (profile.elapsed_s / profile.duration_s) :
                    1.0f;
                if (u >= 1.0f)
                {
                    cmd.lift_rod_cmd = profile.target;
                    cmd.lift_rod_omega_cmd = 0.0f;
                    profile.active = false;
                }
                else
                {
                    const float delta = profile.target - profile.start;
                    cmd.lift_rod_cmd =
                        profile.start +
                        delta * scurve_fraction(u);
                    if (opt.lift_velocity_ff)
                        cmd.lift_rod_omega_cmd =
                            delta * scurve_fraction_derivative(u) / profile.duration_s;
                }
            }
            else
            {
                cmd.lift_rod_cmd = cmd.lift_rod_target;
                cmd.lift_rod_omega_cmd = 0.0f;
            }
        }
        else
        {
            const float prev_lift_rod_cmd = cmd.lift_rod_cmd;
            cmd.lift_rod_cmd = ramp_to(cmd.lift_rod_cmd,
                                       cmd.lift_rod_target,
                                       opt.lift_speed,
                                       opt.lift_speed,
                                       kCommandDtS);
            if (opt.lift_velocity_ff)
                cmd.lift_rod_omega_cmd = (cmd.lift_rod_cmd - prev_lift_rod_cmd) / kCommandDtS;
        }
        cmd.left_cmd = ramp_to(cmd.left_cmd,
                               cmd.left_target,
                               opt.drive_accel,
                               opt.drive_decel,
                               kCommandDtS);
        cmd.right_cmd = ramp_to(cmd.right_cmd,
                                cmd.right_target,
                                opt.drive_accel,
                                opt.drive_decel,
                                kCommandDtS);

        auto &lift = lift_motor(info.module);
        cmd.lift_torque_ff = lift_feedforward_torque(opt,
                                                     info.module,
                                                     cmd.lift_rod_cmd,
                                                     lift_delta_before_ramp);
        lift.Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        lift.Set_Control_Maintain_Postion(cmd.lift_rod_cmd * kLiftMotorToRodRatio,
                                          cmd.lift_rod_omega_cmd * kLiftMotorToRodRatio,
                                          cmd.lift_torque_ff,
                                          opt.lift_kp,
                                          opt.lift_kd);
        lift.TIM_Send_PeriodElapsedCallback();

        if (!opt.drive_enable)
        {
            cmd.left_target = 0.0f;
            cmd.right_target = 0.0f;
            cmd.left_cmd = 0.0f;
            cmd.right_cmd = 0.0f;
            continue;
        }

        auto &left = drive_left_motor(info.module);
        left.Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        left.Set_Control_Maintain_Postion(0.0f,
                                          cmd.left_cmd,
                                          0.0f,
                                          opt.drive_kp,
                                          opt.drive_kd);
        left.TIM_Send_PeriodElapsedCallback();

        auto &right = drive_right_motor(info.module);
        right.Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        right.Set_Control_Maintain_Postion(0.0f,
                                           cmd.right_cmd,
                                           0.0f,
                                           opt.drive_kp,
                                           opt.drive_kd);
        right.TIM_Send_PeriodElapsedCallback();
    }
}

bool temperature_safe(Options const &opt)
{
    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        struct NamedMotor
        {
            const char *name;
            Class_Motor_DM_Normal *motor;
        };

        NamedMotor motors[] = {
            {"lift3519", &lift_motor(info.module)},
            {"drive_left2325", &drive_left_motor(info.module)},
            {"drive_right2325", &drive_right_motor(info.module)},
        };

        for (const auto &entry : motors)
        {
            const float mos_c = temp_k_to_c(entry.motor->Get_Now_MOS_Temperature());
            const float rotor_c = temp_k_to_c(entry.motor->Get_Now_Rotor_Temperature());
            const bool mos_valid = std::isfinite(mos_c) && mos_c > -50.0f && mos_c < 200.0f;
            const bool rotor_valid = std::isfinite(rotor_c) && rotor_c > -50.0f && rotor_c < 200.0f;

            if ((mos_valid && mos_c >= kThermalLimitC) ||
                (rotor_valid && rotor_c >= kThermalLimitC))
            {
                std::cerr << "\n[LIFT-TEST][THERMAL] module=" << info.name
                          << " motor=" << entry.name
                          << " MOS=" << mos_c
                          << "C Rotor=" << rotor_c
                          << "C limit=" << kThermalLimitC
                          << "C\n";
                return false;
            }
        }
    }

    return true;
}

bool ec_step(uint32_t tick, Options const &opt)
{
    if (!st_running.load() || !st_master.is_running)
        return false;

    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);

    can_msg_t msg {};
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
    {
        while (linkx_quick_recv(&st_linkx, ch, &msg))
            st_lift.CAN_Rx_Callback(ch, msg.id, msg.data);
    }

    if ((tick % kAlivePeriodTicks) == 0U)
        alive_and_enable_selected(opt);

    if ((tick % kCommandPeriodTicks) == 0U)
        apply_commands(opt);

    linkx_send_pdos(&st_linkx);
    return true;
}

void run_for_ms(uint32_t &tick, uint32_t ms, Options const &opt)
{
    auto next_wakeup = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < ms && st_running.load(); ++i)
    {
        next_wakeup += std::chrono::milliseconds(kEcPeriodMs);
        if (!ec_step(tick++, opt))
            return;
        std::this_thread::sleep_until(next_wakeup);
    }
}

void init_command_state_from_feedback(Options const &opt)
{
    for (const auto &info : kModuleInfo)
    {
        const int idx = static_cast<int>(info.module);
        ModuleCommand &cmd = st_cmd[idx];
        cmd = {};

        float rod_now = lift_motor(info.module).Get_Now_Radian() / kLiftMotorToRodRatio;
        if (!std::isfinite(rod_now))
            rod_now = opt.rod_start;
        rod_now = clamp_float(rod_now, kRodRangeLow, kRodRangeHigh);

        cmd.lift_rod_cmd = rod_now;
        cmd.lift_rod_target = rod_now;
        st_scurve[idx] = {};
        st_scurve[idx].start = rod_now;
        st_scurve[idx].target = rod_now;
    }
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
            std::cerr << "[LIFT-TEST] CAN" << ch << " FDCAN 1M/5M config failed\n";
            return false;
        }
    }

    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);

    return true;
}

bool init_ethercat_linkx(Options const &opt)
{
    if (!ecat_master_init(&st_master, opt.ifname.c_str()))
    {
        std::cerr << "[LIFT-TEST] ecat_master_init failed for " << opt.ifname << "\n";
        return false;
    }

    linkx_init(&st_linkx, 1, &st_master.ctx);

    if (!configure_linkx_can())
        return false;

    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[LIFT-TEST] ecat_master_bring_online failed\n";
        return false;
    }

    return true;
}

void write_csv_header(std::ofstream &csv)
{
    csv << "phase,t_s,module,ch,"
        << "rod_target_rad,rod_cmd_rad,rod_cmd_omega_rad_s,rod_actual_rad,"
        << "motor_target_rad,motor_target_omega_rad_s,motor_actual_rad,motor_omega_rad_s,lift_torque_nm,lift_torque_ff_nm,lift_torque_residual_nm,"
        << "left_target_rad_s,left_cmd_rad_s,left_actual_rad_s,left_torque_nm,"
        << "right_target_rad_s,right_cmd_rad_s,right_actual_rad_s,right_torque_nm,"
        << "lift_status,lift_ctrl,left_status,left_ctrl,right_status,right_ctrl,"
        << "lift_mos_c,lift_rotor_c,left_mos_c,left_rotor_c,right_mos_c,right_rotor_c\n";
}

void write_csv_sample(std::ofstream &csv,
                      Options const &opt,
                      const std::string &phase,
                      float t_s)
{
    if (!csv.is_open())
        return;

    csv << std::fixed << std::setprecision(6);
    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        const ModuleCommand &cmd = st_cmd[static_cast<int>(info.module)];
        auto &lift = lift_motor(info.module);
        auto &left = drive_left_motor(info.module);
        auto &right = drive_right_motor(info.module);
        const float rod_actual = lift.Get_Now_Radian() / kLiftMotorToRodRatio;
        const float lift_torque = lift.Get_Now_Torque();
        const float residual_nm = lift_torque - lift_hold_torque(info.module, rod_actual);

        csv << phase << ","
            << t_s << ","
            << info.name << ","
            << static_cast<int>(info.can_channel) << ","
            << cmd.lift_rod_target << ","
            << cmd.lift_rod_cmd << ","
            << cmd.lift_rod_omega_cmd << ","
            << rod_actual << ","
            << (cmd.lift_rod_cmd * kLiftMotorToRodRatio) << ","
            << (cmd.lift_rod_omega_cmd * kLiftMotorToRodRatio) << ","
            << lift.Get_Now_Radian() << ","
            << lift.Get_Now_Omega() << ","
            << lift_torque << ","
            << cmd.lift_torque_ff << ","
            << residual_nm << ","
            << cmd.left_target << ","
            << cmd.left_cmd << ","
            << left.Get_Now_Omega() << ","
            << left.Get_Now_Torque() << ","
            << cmd.right_target << ","
            << cmd.right_cmd << ","
            << right.Get_Now_Omega() << ","
            << right.Get_Now_Torque() << ","
            << dm_status_name(lift.Get_Status()) << ","
            << dm_ctrl_name(lift.Get_Now_Control_Status()) << ","
            << dm_status_name(left.Get_Status()) << ","
            << dm_ctrl_name(left.Get_Now_Control_Status()) << ","
            << dm_status_name(right.Get_Status()) << ","
            << dm_ctrl_name(right.Get_Now_Control_Status()) << ","
            << temp_k_to_c(lift.Get_Now_MOS_Temperature()) << ","
            << temp_k_to_c(lift.Get_Now_Rotor_Temperature()) << ","
            << temp_k_to_c(left.Get_Now_MOS_Temperature()) << ","
            << temp_k_to_c(left.Get_Now_Rotor_Temperature()) << ","
            << temp_k_to_c(right.Get_Now_MOS_Temperature()) << ","
            << temp_k_to_c(right.Get_Now_Rotor_Temperature()) << "\n";
    }
}

PhaseModuleMetrics &metrics_for(const std::string &phase, const ModuleInfo &info)
{
    for (auto &metrics : st_metrics)
    {
        if (metrics.phase == phase && metrics.module == info.module)
            return metrics;
    }

    PhaseModuleMetrics metrics {};
    metrics.phase = phase;
    metrics.module = info.module;
    metrics.module_name = info.name;
    metrics.can_channel = info.can_channel;
    st_metrics.push_back(metrics);
    return st_metrics.back();
}

void record_metrics_sample(Options const &opt,
                           const std::string &phase,
                           float t_s)
{
    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        const ModuleCommand &cmd = st_cmd[static_cast<int>(info.module)];
        auto &lift = lift_motor(info.module);
        auto &left = drive_left_motor(info.module);
        auto &right = drive_right_motor(info.module);
        PhaseModuleMetrics &metrics = metrics_for(phase, info);

        const float rod_actual = lift.Get_Now_Radian() / kLiftMotorToRodRatio;
        const float left_actual = left.Get_Now_Omega();
        const float right_actual = right.Get_Now_Omega();
        const float rod_error = rod_actual - cmd.lift_rod_cmd;
        const float left_error = left_actual - cmd.left_cmd;
        const float right_error = right_actual - cmd.right_cmd;

        metrics.final_t_s = t_s;
        metrics.final_rod_target = cmd.lift_rod_target;
        metrics.final_rod_cmd = cmd.lift_rod_cmd;
        metrics.final_rod_actual = rod_actual;
        metrics.final_left_target = cmd.left_target;
        metrics.final_left_cmd = cmd.left_cmd;
        metrics.final_left_actual = left_actual;
        metrics.final_right_target = cmd.right_target;
        metrics.final_right_cmd = cmd.right_cmd;
        metrics.final_right_actual = right_actual;
        metrics.lift_speed = opt.lift_speed;
        metrics.lift_kp = opt.lift_kp;
        metrics.lift_kd = opt.lift_kd;
        metrics.lift_velocity_ff = opt.lift_velocity_ff;
        metrics.forward_m_s = opt.forward_m_s;
        metrics.yaw_rad_s = opt.yaw_rad_s;
        metrics.drive_kp = opt.drive_kp;
        metrics.drive_kd = opt.drive_kd;
        metrics.drive_accel = opt.drive_accel;
        metrics.drive_decel = opt.drive_decel;
        metrics.drive_enable = opt.drive_enable;
        metrics.feedforward_mode = opt.feedforward_mode;
        metrics.lift_profile = opt.lift_profile;

        metrics.rod_actual.add(rod_actual);
        metrics.rod_error.add(rod_error);
        metrics.rod_cmd_velocity.add(cmd.lift_rod_omega_cmd);
        metrics.lift_torque.add(lift.Get_Now_Torque());
        metrics.lift_torque_ff.add(cmd.lift_torque_ff);
        metrics.left_actual.add(left_actual);
        metrics.left_error.add(left_error);
        metrics.left_torque.add(left.Get_Now_Torque());
        metrics.right_actual.add(right_actual);
        metrics.right_error.add(right_error);
        metrics.right_torque.add(right.Get_Now_Torque());

        metrics.temperature_c.add(temp_k_to_c(lift.Get_Now_MOS_Temperature()));
        metrics.temperature_c.add(temp_k_to_c(lift.Get_Now_Rotor_Temperature()));
        metrics.temperature_c.add(temp_k_to_c(left.Get_Now_MOS_Temperature()));
        metrics.temperature_c.add(temp_k_to_c(left.Get_Now_Rotor_Temperature()));
        metrics.temperature_c.add(temp_k_to_c(right.Get_Now_MOS_Temperature()));
        metrics.temperature_c.add(temp_k_to_c(right.Get_Now_Rotor_Temperature()));

        if (metrics.has_prev)
        {
            const float dt = t_s - metrics.prev_t_s;
            if (dt > 1e-5f)
            {
                const float rod_velocity = (rod_actual - metrics.prev_rod_actual) / dt;
                if (metrics.rod_velocity_from_position.n > 0)
                    metrics.rod_accel_from_position.add((rod_velocity - metrics.prev_rod_velocity) / dt);
                metrics.rod_velocity_from_position.add(rod_velocity);
                metrics.prev_rod_velocity = rod_velocity;

                metrics.left_accel.add((left_actual - metrics.prev_left_actual) / dt);
                metrics.right_accel.add((right_actual - metrics.prev_right_actual) / dt);
            }
        }

        metrics.has_prev = true;
        metrics.prev_t_s = t_s;
        metrics.prev_rod_actual = rod_actual;
        metrics.prev_left_actual = left_actual;
        metrics.prev_right_actual = right_actual;

        const float left_target_abs = std::fabs(cmd.left_target);
        if (!metrics.left_reached && left_target_abs > kDriveDeadzone)
        {
            const float sign = (cmd.left_target >= 0.0f) ? 1.0f : -1.0f;
            if (sign * left_actual >= kDriveReachFrac * left_target_abs)
            {
                metrics.left_reached = true;
                metrics.left_reach_time_s = t_s;
            }
        }

        const float right_target_abs = std::fabs(cmd.right_target);
        if (!metrics.right_reached && right_target_abs > kDriveDeadzone)
        {
            const float sign = (cmd.right_target >= 0.0f) ? 1.0f : -1.0f;
            if (sign * right_actual >= kDriveReachFrac * right_target_abs)
            {
                metrics.right_reached = true;
                metrics.right_reach_time_s = t_s;
            }
        }
    }
}

void reset_identify_sample_state()
{
    st_identify_samples.clear();
    for (auto &prev : st_identify_prev)
        prev = {};
}

void record_identify_sample(Options const &opt,
                            const std::string &phase,
                            float t_s)
{
    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        const int idx = static_cast<int>(info.module);
        const ModuleCommand &cmd = st_cmd[idx];
        auto &lift = lift_motor(info.module);

        const float rod_actual = lift.Get_Now_Radian() / kLiftMotorToRodRatio;
        float rod_velocity = 0.0f;
        float rod_accel = 0.0f;

        IdentifyPrevState &prev = st_identify_prev[idx];
        if (prev.valid && prev.phase == phase)
        {
            const float dt = t_s - prev.t_s;
            if (dt > 1e-5f)
            {
                rod_velocity = (rod_actual - prev.rod_actual) / dt;
                if (prev.has_velocity)
                    rod_accel = (rod_velocity - prev.rod_velocity) / dt;
            }
        }

        IdentifySample sample {};
        sample.phase = phase;
        sample.module = info.module;
        sample.module_name = info.name;
        sample.can_channel = info.can_channel;
        sample.t_s = t_s;
        sample.rod_target = cmd.lift_rod_target;
        sample.rod_cmd = cmd.lift_rod_cmd;
        sample.rod_actual = rod_actual;
        sample.rod_error = rod_actual - cmd.lift_rod_cmd;
        sample.rod_velocity = rod_velocity;
        sample.rod_accel = rod_accel;
        sample.torque_nm = lift.Get_Now_Torque();
        sample.lift_kp = opt.lift_kp;
        sample.lift_kd = opt.lift_kd;
        sample.lift_speed = opt.lift_speed;
        st_identify_samples.push_back(sample);

        prev.valid = true;
        prev.has_velocity = true;
        prev.phase = phase;
        prev.t_s = t_s;
        prev.rod_actual = rod_actual;
        prev.rod_velocity = rod_velocity;
    }
}

struct IdentifyDirectionStats
{
    Stats dynamic_residual_signed;
    Stats dynamic_residual_abs;
    Stats dynamic_velocity;
    Stats dynamic_accel;
    Stats breakaway_residual_signed;
    Stats breakaway_residual_abs;
    Stats inertia_motor;
};

bool interpolate_hold_torque(const std::vector<IdentifyHoldPoint> &hold_points,
                             float angle,
                             float &torque_nm)
{
    struct Point
    {
        float angle = 0.0f;
        float torque = 0.0f;
    };

    std::vector<Point> points;
    for (const auto &point : hold_points)
    {
        if (point.torque.n > 0)
            points.push_back({point.angle, static_cast<float>(point.torque.mean())});
    }

    if (points.empty())
        return false;

    std::sort(points.begin(), points.end(), [](const Point &a, const Point &b) {
        return a.angle < b.angle;
    });

    if (angle <= points.front().angle)
    {
        torque_nm = points.front().torque;
        return true;
    }
    if (angle >= points.back().angle)
    {
        torque_nm = points.back().torque;
        return true;
    }

    for (size_t i = 1; i < points.size(); ++i)
    {
        const Point &lo = points[i - 1];
        const Point &hi = points[i];
        if (angle >= lo.angle && angle <= hi.angle)
        {
            const float span = hi.angle - lo.angle;
            const float u = (std::fabs(span) > 1e-5f) ? (angle - lo.angle) / span : 0.0f;
            torque_nm = lo.torque + u * (hi.torque - lo.torque);
            return true;
        }
    }

    torque_nm = points.back().torque;
    return true;
}

void write_identify_direction_summary(std::ostream &os,
                                      const char *label,
                                      const IdentifyDirectionStats &stats)
{
    os << "  " << label;
    if (stats.dynamic_residual_signed.n > 0)
    {
        os << " dynamic_samples=" << stats.dynamic_residual_signed.n
           << " dynamic_residual_signed_mean_nm=" << stats.dynamic_residual_signed.mean()
           << " dynamic_residual_abs_mean_nm=" << stats.dynamic_residual_abs.mean()
           << " dynamic_residual_abs_max_nm=" << stats.dynamic_residual_abs.max
           << " rod_velocity_mean_rad_s=" << stats.dynamic_velocity.mean()
           << " rod_accel_max_abs_rad_s2=" << stats.dynamic_accel.max_abs();
    }
    else
    {
        os << " dynamic_samples=0 dynamic_residual=NA";
    }

    if (stats.breakaway_residual_signed.n > 0)
    {
        os << " breakaway_samples=" << stats.breakaway_residual_signed.n
           << " breakaway_residual_signed_mean_nm=" << stats.breakaway_residual_signed.mean()
           << " breakaway_residual_abs_mean_nm=" << stats.breakaway_residual_abs.mean()
           << " breakaway_residual_abs_max_nm=" << stats.breakaway_residual_abs.max;
    }
    else
    {
        os << " breakaway_samples=0 breakaway_residual=NA";
    }

    if (stats.inertia_motor.n >= 3)
    {
        const double inertia_motor = stats.inertia_motor.mean();
        os << " inertia_rough_samples=" << stats.inertia_motor.n
           << " inertia_rough_motor_kgm2=" << inertia_motor
           << " inertia_rough_rod_equiv_kgm2="
           << (inertia_motor * kLiftMotorToRodRatio * kLiftMotorToRodRatio);
    }
    else
    {
        os << " inertia_rough_samples=" << stats.inertia_motor.n
           << " inertia_rough=NA";
    }
    os << "\n";
}

void write_identify_summary(std::ostream &os, Options const &opt)
{
    if (opt.suite != TestSuite::Identify)
        return;

    os << "\nLift DM3519 parameter identification\n"
       << "identify_speed_rod_rad_s=" << opt.identify_speed
       << " hold_s=" << opt.identify_hold_s
       << " breakaway_vel_rod_rad_s=" << opt.identify_breakaway_vel
       << " lift_kp=" << opt.lift_kp
       << " lift_kd=" << opt.lift_kd
       << " angle_points=" << angle_list_text(opt.identify_angles) << "\n"
       << "note: torque values are DM3519 motor-side Nm; rod_equiv torque multiplies by "
       << kLiftMotorToRodRatio << ".\n"
       << "note: friction residual = measured motor torque - interpolated hold torque at the same rod angle.\n"
       << "note: inertia is a rough closed-loop estimate from position-derived acceleration.\n\n";

    if (st_identify_samples.empty())
    {
        os << "identify_samples=0\n";
        return;
    }

    const float hold_discard_s =
        std::min(0.4f, std::max(0.0f, opt.identify_hold_s * 0.35f));
    const float dynamic_min_vel =
        std::max(opt.identify_breakaway_vel, opt.identify_speed * kIdentifyDynamicVelFrac);

    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        std::vector<IdentifyHoldPoint> hold_points;
        hold_points.reserve(opt.identify_angles.size());
        for (float angle : opt.identify_angles)
        {
            IdentifyHoldPoint point {};
            point.module = info.module;
            point.module_name = info.name;
            point.angle = angle;

            for (const auto &sample : st_identify_samples)
            {
                if (sample.module != info.module)
                    continue;
                if (!starts_with(sample.phase, "id_hold"))
                    continue;
                if (sample.t_s < hold_discard_s)
                    continue;
                if (std::fabs(sample.rod_target - angle) > 0.08f)
                    continue;

                point.torque.add(sample.torque_nm);
                point.rod_error.add(sample.rod_error);
            }

            hold_points.push_back(point);
        }

        IdentifyDirectionStats raise_stats {};
        IdentifyDirectionStats retract_stats {};
        std::map<std::string, bool> breakaway_seen;

        for (const auto &sample : st_identify_samples)
        {
            if (sample.module != info.module)
                continue;

            const bool is_raise = starts_with(sample.phase, "id_raise");
            const bool is_retract = starts_with(sample.phase, "id_retract");
            if (!is_raise && !is_retract)
                continue;

            float hold_torque = 0.0f;
            if (!interpolate_hold_torque(hold_points, sample.rod_actual, hold_torque))
                continue;

            const float residual = sample.torque_nm - hold_torque;
            IdentifyDirectionStats &stats = is_raise ? raise_stats : retract_stats;

            if (std::fabs(sample.rod_velocity) >= dynamic_min_vel && sample.t_s > 0.05f)
            {
                stats.dynamic_residual_signed.add(residual);
                stats.dynamic_residual_abs.add(std::fabs(residual));
                stats.dynamic_velocity.add(sample.rod_velocity);
                stats.dynamic_accel.add(sample.rod_accel);
            }

            if (std::fabs(sample.rod_velocity) >= opt.identify_breakaway_vel)
            {
                std::ostringstream key;
                key << static_cast<int>(sample.module) << "|" << sample.phase;
                if (!breakaway_seen[key.str()])
                {
                    stats.breakaway_residual_signed.add(residual);
                    stats.breakaway_residual_abs.add(std::fabs(residual));
                    breakaway_seen[key.str()] = true;
                }
            }
        }

        for (const auto &sample : st_identify_samples)
        {
            if (sample.module != info.module)
                continue;

            const bool is_raise = starts_with(sample.phase, "id_raise");
            const bool is_retract = starts_with(sample.phase, "id_retract");
            if (!is_raise && !is_retract)
                continue;

            IdentifyDirectionStats &stats = is_raise ? raise_stats : retract_stats;
            if (stats.dynamic_residual_signed.n == 0)
                continue;

            float hold_torque = 0.0f;
            if (!interpolate_hold_torque(hold_points, sample.rod_actual, hold_torque))
                continue;

            const float residual = sample.torque_nm - hold_torque;
            const float friction_est = static_cast<float>(stats.dynamic_residual_signed.mean());
            const float accel_motor = sample.rod_accel * kLiftMotorToRodRatio;
            const float net_accel_torque = residual - friction_est;
            if (std::fabs(accel_motor) < kIdentifyMinMotorAccelRadS2)
                continue;
            if (net_accel_torque * accel_motor <= 0.0f)
                continue;

            const float inertia_motor = net_accel_torque / accel_motor;
            if (std::isfinite(inertia_motor) && inertia_motor > 0.0f && inertia_motor < 100.0f)
                stats.inertia_motor.add(inertia_motor);
        }

        os << "[IDENTIFY " << info.name
           << " ch" << static_cast<int>(info.can_channel) << "]\n";
        os << "  hold_torque_table discard_first_s=" << hold_discard_s
           << " dynamic_min_vel_rod_rad_s=" << dynamic_min_vel << "\n";
        for (const auto &point : hold_points)
        {
            os << "    angle_rad=" << point.angle
               << " motor_angle_rad=" << (point.angle * kLiftMotorToRodRatio);
            if (point.torque.n > 0)
            {
                os << " samples=" << point.torque.n
                   << " hold_torque_motor_mean_nm=" << point.torque.mean()
                   << " hold_torque_motor_std_nm=" << point.torque.stddev()
                   << " hold_torque_rod_equiv_mean_nm="
                   << (point.torque.mean() * kLiftMotorToRodRatio)
                   << " rod_error_mean_rad=" << point.rod_error.mean()
                   << " rod_error_rms_rad=" << point.rod_error.rms();
            }
            else
            {
                os << " samples=0 hold_torque=NA";
            }
            os << "\n";
        }

        write_identify_direction_summary(os, "raise_toward_-15", raise_stats);
        write_identify_direction_summary(os, "retract_toward_-1", retract_stats);
        os << "  feedforward_hint_motor_nm=hold_torque_interp(rod_angle)"
           << " + direction_dynamic_residual_signed_mean\n\n";
    }
}

std::string feedforward_case_label(const std::string &phase)
{
    const std::array<const char *, 3> labels {{
        "ff_hold_friction_s05",
        "ff_pd_only_s05",
        "ff_hold_s05",
    }};

    for (const char *label : labels)
    {
        const std::string prefix = std::string(label) + "_";
        if (starts_with(phase, prefix.c_str()))
            return label;
    }
    return "";
}

bool feedforward_motion_phase(const std::string &phase)
{
    return phase.find("_raise") != std::string::npos ||
           phase.find("_retract") != std::string::npos;
}

struct FeedforwardAggregate
{
    std::string label;
    const char *module_name = "";
    uint64_t samples = 0;
    double rod_error_sum = 0.0;
    double rod_error_sum_sq = 0.0;
    double rod_error_max_abs = 0.0;
    double final_error_max_abs = 0.0;
    double lift_torque_max_abs = 0.0;
    double lift_torque_ff_max_abs = 0.0;
};

void write_feedforward_summary(std::ostream &os, Options const &opt)
{
    if (opt.suite != TestSuite::Feedforward)
        return;

    std::map<std::string, FeedforwardAggregate> aggregates;
    for (const auto &metrics : st_metrics)
    {
        if (!feedforward_motion_phase(metrics.phase))
            continue;

        const std::string label = feedforward_case_label(metrics.phase);
        if (label.empty())
            continue;

        const std::string key = label + "|" + metrics.module_name;
        auto &agg = aggregates[key];
        agg.label = label;
        agg.module_name = metrics.module_name;
        agg.samples += metrics.rod_error.n;
        agg.rod_error_sum += metrics.rod_error.sum;
        agg.rod_error_sum_sq += metrics.rod_error.sum_sq;
        agg.rod_error_max_abs = std::max(agg.rod_error_max_abs, metrics.rod_error.max_abs());
        agg.final_error_max_abs = std::max(
            agg.final_error_max_abs,
            static_cast<double>(std::fabs(metrics.final_rod_actual - metrics.final_rod_cmd)));
        agg.lift_torque_max_abs = std::max(agg.lift_torque_max_abs, metrics.lift_torque.max_abs());
        agg.lift_torque_ff_max_abs = std::max(agg.lift_torque_ff_max_abs, metrics.lift_torque_ff.max_abs());
    }

    os << "\nFeedforward validation comparison\n"
       << "note: comparison uses raise/retract motion phases only; preposition/stop phases are excluded.\n"
       << "candidate_ff_source=identify_20260612_014823"
       << " torque_limit_nm=" << kFeedforwardTorqueLimitNm << "\n";

    const std::array<const char *, 3> labels {{
        "ff_pd_only_s05",
        "ff_hold_s05",
        "ff_hold_friction_s05",
    }};

    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        os << "[FF " << info.name << "]\n";
        for (const char *label : labels)
        {
            const std::string key = std::string(label) + "|" + info.name;
            const auto it = aggregates.find(key);
            os << "  case=" << label;
            if (it == aggregates.end() || it->second.samples == 0)
            {
                os << " samples=0 result=NA\n";
                continue;
            }

            const auto &agg = it->second;
            os << " samples=" << agg.samples
               << " rod_error_mean=" << (agg.rod_error_sum / static_cast<double>(agg.samples))
               << " rod_error_rms=" << std::sqrt(agg.rod_error_sum_sq / static_cast<double>(agg.samples))
               << " rod_error_max_abs=" << agg.rod_error_max_abs
               << " final_error_max_abs=" << agg.final_error_max_abs
               << " lift_torque_max_abs_nm=" << agg.lift_torque_max_abs
               << " lift_torque_ff_max_abs_nm=" << agg.lift_torque_ff_max_abs
               << "\n";
        }
        os << "\n";
    }
}

std::string scurve_case_label(const std::string &phase)
{
    const std::array<const char *, 20> labels {{
        "scurve_trap_hold_s05_posonly",
        "scurve_minjerk_hold_s08_posonly",
        "scurve_minjerk_hold_s08_posvel",
        "scurve_minjerk_hold_s09_posvel",
        "scurve_minjerk_hold_s10_posonly",
        "scurve_minjerk_hold_s10_posvel",
        "scurve_minjerk_hold_s12_posvel",
        "fast_s12_kp20_kd12_hold",
        "fast_s13_kp20_kd12_hold",
        "fast_s14_kp20_kd12_hold",
        "fast_s15_kp20_kd12_hold",
        "fast_s18_kp20_kd12_hold",
        "fast_s20_kp20_kd12_hold",
        "fast_s13_kp25_kd12_hold",
        "fast_s14_kp25_kd12_hold",
        "fast_s20_kp20_kd16_hold",
        "fast_s20_kp25_kd12_hold",
        "fast_s20_kp25_kd16_hold",
        "fast_s20_kp30_kd16_hold",
        "fast_s20_kp25_kd16_friction",
    }};

    for (const char *label : labels)
    {
        const std::string prefix = std::string(label) + "_";
        if (starts_with(phase, prefix.c_str()))
            return label;
    }
    return "";
}

struct SCurveAggregate
{
    std::string label;
    const char *module_name = "";
    uint64_t samples = 0;
    double motion_time_s = 0.0;
    double rod_error_sum = 0.0;
    double rod_error_sum_sq = 0.0;
    double rod_error_max_abs = 0.0;
    double final_error_max_abs = 0.0;
    double rod_cmd_velocity_max_abs = 0.0;
    double rod_velocity_max_abs = 0.0;
    double rod_accel_max_abs = 0.0;
    double lift_torque_max_abs = 0.0;
    double lift_torque_ff_max_abs = 0.0;
};

void write_scurve_summary(std::ostream &os, Options const &opt)
{
    if (opt.suite != TestSuite::SCurve && opt.suite != TestSuite::Fast)
        return;

    std::map<std::string, SCurveAggregate> aggregates;
    for (const auto &metrics : st_metrics)
    {
        if (!feedforward_motion_phase(metrics.phase))
            continue;

        const std::string label = scurve_case_label(metrics.phase);
        if (label.empty())
            continue;

        const std::string key = label + "|" + metrics.module_name;
        auto &agg = aggregates[key];
        agg.label = label;
        agg.module_name = metrics.module_name;
        agg.samples += metrics.rod_error.n;
        agg.motion_time_s += metrics.final_t_s;
        agg.rod_error_sum += metrics.rod_error.sum;
        agg.rod_error_sum_sq += metrics.rod_error.sum_sq;
        agg.rod_error_max_abs = std::max(agg.rod_error_max_abs, metrics.rod_error.max_abs());
        agg.final_error_max_abs = std::max(
            agg.final_error_max_abs,
            static_cast<double>(std::fabs(metrics.final_rod_actual - metrics.final_rod_cmd)));
        agg.rod_cmd_velocity_max_abs = std::max(agg.rod_cmd_velocity_max_abs,
                                                metrics.rod_cmd_velocity.max_abs());
        agg.rod_velocity_max_abs = std::max(agg.rod_velocity_max_abs,
                                            metrics.rod_velocity_from_position.max_abs());
        agg.rod_accel_max_abs = std::max(agg.rod_accel_max_abs,
                                         metrics.rod_accel_from_position.max_abs());
        agg.lift_torque_max_abs = std::max(agg.lift_torque_max_abs,
                                           metrics.lift_torque.max_abs());
        agg.lift_torque_ff_max_abs = std::max(agg.lift_torque_ff_max_abs,
                                              metrics.lift_torque_ff.max_abs());
    }

    os << "\n" << (opt.suite == TestSuite::Fast ?
                   "Fast S-curve tuning comparison" :
                   "S-curve validation comparison") << "\n"
       << "note: comparison uses raise/retract motion phases only; preposition/stop phases are excluded.\n"
       << "note: minjerk S-curve duration is "
       << kSCurvePeakVelocityScale
       << " * distance / lift_speed, so lift_speed is peak rod speed.\n"
       << "note: posvel cases command MIT omega target = rod_cmd_velocity * "
       << kLiftMotorToRodRatio << ".\n";

    const std::vector<const char *> labels =
        (opt.suite == TestSuite::Fast) ?
        std::vector<const char *> {
            "fast_s12_kp20_kd12_hold",
            "fast_s13_kp20_kd12_hold",
            "fast_s14_kp20_kd12_hold",
            "fast_s15_kp20_kd12_hold",
            "fast_s18_kp20_kd12_hold",
            "fast_s20_kp20_kd12_hold",
            "fast_s13_kp25_kd12_hold",
            "fast_s14_kp25_kd12_hold",
            "fast_s20_kp20_kd16_hold",
            "fast_s20_kp25_kd12_hold",
            "fast_s20_kp25_kd16_hold",
            "fast_s20_kp30_kd16_hold",
            "fast_s20_kp25_kd16_friction",
        } :
        std::vector<const char *> {
            "scurve_trap_hold_s05_posonly",
            "scurve_minjerk_hold_s08_posonly",
            "scurve_minjerk_hold_s08_posvel",
            "scurve_minjerk_hold_s09_posvel",
            "scurve_minjerk_hold_s10_posonly",
            "scurve_minjerk_hold_s10_posvel",
            "scurve_minjerk_hold_s12_posvel",
        };

    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        os << "[SCURVE " << info.name << "]\n";
        for (const char *label : labels)
        {
            const std::string key = std::string(label) + "|" + info.name;
            const auto it = aggregates.find(key);
            os << "  case=" << label;
            if (it == aggregates.end() || it->second.samples == 0)
            {
                os << " samples=0 result=NA\n";
                continue;
            }

            const auto &agg = it->second;
            os << " samples=" << agg.samples
               << " motion_time_s=" << agg.motion_time_s
               << " rod_error_mean=" << (agg.rod_error_sum / static_cast<double>(agg.samples))
               << " rod_error_rms=" << std::sqrt(agg.rod_error_sum_sq / static_cast<double>(agg.samples))
               << " rod_error_max_abs=" << agg.rod_error_max_abs
               << " final_error_max_abs=" << agg.final_error_max_abs
               << " rod_cmd_velocity_max_abs=" << agg.rod_cmd_velocity_max_abs
               << " rod_velocity_max_abs=" << agg.rod_velocity_max_abs
               << " rod_accel_max_abs=" << agg.rod_accel_max_abs
               << " lift_torque_max_abs_nm=" << agg.lift_torque_max_abs
               << " lift_torque_ff_max_abs_nm=" << agg.lift_torque_ff_max_abs
               << "\n";
        }
        os << "\n";
    }
}

void write_contact_summary(std::ostream &os, Options const &opt)
{
    if (opt.suite != TestSuite::Contact)
        return;

    os << "\nLift contact / load feedback probe\n"
       << "contact_speed_rod_rad_s=" << opt.contact_speed
       << " contact_threshold_lowering_load_nm=" << opt.contact_threshold_nm
       << " contact_ignore_s=" << opt.contact_ignore_s
       << " contact_min_travel_rad=" << opt.contact_min_travel
       << " contact_confirm_ms=" << opt.contact_confirm_ms
       << " contact_hold_s=" << opt.contact_hold_s
       << " contact_preload_rad=" << opt.contact_preload_rad << "\n"
       << "note: residual_nm = measured_motor_torque - hold_torque_interp(rod_actual).\n"
       << "note: lowering_load_nm = hold_torque_interp(rod_actual) - measured_motor_torque; positive means more load against lowering toward -15.\n"
       << "note: contact is detected only when positive lowering_load_nm stays above threshold after the minimum travel gate.\n\n";

    if (st_contact_results.empty())
    {
        os << "contact_results=0\n";
        return;
    }

    for (const auto &result : st_contact_results)
    {
        os << "[CONTACT " << result.module_name
           << " ch" << static_cast<int>(result.can_channel)
           << " phase=" << result.phase << "]\n"
           << "  detected=" << (result.detected ? 1 : 0)
           << " timeout=" << (result.timeout ? 1 : 0)
           << " detect_t_s=" << result.detect_t_s
           << " detect_rod_cmd=" << result.detect_rod_cmd
           << " detect_rod_actual=" << result.detect_rod_actual
           << " detect_motor_actual=" << (result.detect_rod_actual * kLiftMotorToRodRatio)
           << " detect_rod_error=" << result.detect_rod_error
           << " detect_torque_nm=" << result.detect_torque_nm
           << " detect_hold_torque_nm=" << result.detect_hold_torque_nm
           << " detect_residual_nm=" << result.detect_residual_nm
           << " detect_lowering_load_nm=" << result.detect_lowering_load_nm
           << " detect_motor_omega_rad_s=" << result.detect_motor_omega << "\n"
           << "  motion_stats"
           << " samples=" << result.residual.n
           << " residual_mean_nm=" << result.residual.mean()
           << " residual_min_nm=" << result.residual.min
           << " residual_max_nm=" << result.residual.max
           << " lowering_load_mean_nm=" << result.lowering_load.mean()
           << " lowering_load_max_nm=" << result.lowering_load.max
           << " rod_error_max_abs=" << result.rod_error.max_abs()
           << " rod_velocity_max_abs=" << result.rod_velocity.max_abs()
           << " torque_max_abs_nm=" << result.torque.max_abs() << "\n"
           << "  hold_stats"
           << " samples=" << result.hold_residual.n
           << " residual_mean_nm=" << result.hold_residual.mean()
           << " residual_std_nm=" << result.hold_residual.stddev()
           << " lowering_load_mean_nm=" << result.hold_lowering_load.mean()
           << " lowering_load_std_nm=" << result.hold_lowering_load.stddev()
           << "\n\n";
    }
}

void write_summary(Options const &opt)
{
    std::ofstream file(opt.summary_path, std::ios::out | std::ios::trunc);
    std::ostream &os = file.is_open() ? file : std::cout;

    if (!file.is_open())
        std::cerr << "[LIFT-TEST] failed to open summary: " << opt.summary_path << "\n";

    os << std::fixed << std::setprecision(4);
    os << "R2 lift motor parameter summary\n"
       << "suite=" << test_suite_name(opt.suite)
       << " module=" << module_selection_name(opt.module)
       << " rod_range_rad=[" << opt.rod_start << "," << opt.rod_end << "]"
       << " motor_range_rad=[" << (opt.rod_start * kLiftMotorToRodRatio)
       << "," << (opt.rod_end * kLiftMotorToRodRatio) << "]"
       << " motor_to_rod_ratio=" << kLiftMotorToRodRatio << "\n"
       << "cycles_per_case=" << opt.cycles
       << " reach_frac=" << kDriveReachFrac << "\n"
       << "note: lift rod velocity/acceleration below are derived from position samples.\n\n";

    if (opt.suite == TestSuite::Identify)
    {
        os << "ordinary per-phase metrics omitted for identify suite; raw phase samples are in CSV.\n";
        os << "identify_samples=" << st_identify_samples.size() << "\n";
    }
    else
    {
        for (const auto &metrics : st_metrics)
        {
            os << "[" << metrics.phase << " " << metrics.module_name
               << " ch" << static_cast<int>(metrics.can_channel) << "]\n";

            os << "  params"
               << " lift_speed_rod_rad_s=" << metrics.lift_speed
               << " lift_kp=" << metrics.lift_kp
               << " lift_kd=" << metrics.lift_kd
               << " lift_velocity_ff=" << (metrics.lift_velocity_ff ? 1 : 0)
               << " forward_m_s=" << metrics.forward_m_s
               << " yaw_rad_s=" << metrics.yaw_rad_s
               << " drive_kp=" << metrics.drive_kp
               << " drive_kd=" << metrics.drive_kd
               << " drive_accel_limit_rad_s2=" << metrics.drive_accel
               << " drive_decel_limit_rad_s2=" << metrics.drive_decel
               << " drive_enable=" << (metrics.drive_enable ? 1 : 0)
               << " feedforward=" << feedforward_mode_name(metrics.feedforward_mode)
               << " lift_profile=" << lift_profile_name(metrics.lift_profile)
               << "\n";

            os << "  DM3519_lift"
               << " final_rod_target=" << metrics.final_rod_target
               << " final_rod_cmd=" << metrics.final_rod_cmd
               << " final_rod_actual=" << metrics.final_rod_actual
               << " rod_actual_min=" << metrics.rod_actual.min
               << " rod_actual_max=" << metrics.rod_actual.max
               << " rod_error_mean=" << metrics.rod_error.mean()
               << " rod_error_rms=" << metrics.rod_error.rms()
               << " rod_error_max_abs=" << metrics.rod_error.max_abs()
               << " rod_cmd_velocity_max_abs=" << metrics.rod_cmd_velocity.max_abs()
               << " rod_velocity_mean=" << metrics.rod_velocity_from_position.mean()
               << " rod_velocity_max_abs=" << metrics.rod_velocity_from_position.max_abs()
               << " rod_accel_max_abs=" << metrics.rod_accel_from_position.max_abs()
               << " lift_torque_mean_nm=" << metrics.lift_torque.mean()
               << " lift_torque_max_abs_nm=" << metrics.lift_torque.max_abs()
               << " lift_torque_ff_mean_nm=" << metrics.lift_torque_ff.mean()
               << " lift_torque_ff_max_abs_nm=" << metrics.lift_torque_ff.max_abs()
               << "\n";

            os << "  DM2325_left"
               << " final_target=" << metrics.final_left_target
               << " final_cmd=" << metrics.final_left_cmd
               << " final_actual=" << metrics.final_left_actual
               << " actual_mean=" << metrics.left_actual.mean()
               << " velocity_error_rms=" << metrics.left_error.rms()
               << " velocity_error_max_abs=" << metrics.left_error.max_abs()
               << " reach95_s=";
            if (metrics.left_reached)
                os << metrics.left_reach_time_s;
            else
                os << "NA";
            os << " accel_max_abs_rad_s2=" << metrics.left_accel.max_abs()
               << " torque_mean_nm=" << metrics.left_torque.mean()
               << " torque_max_abs_nm=" << metrics.left_torque.max_abs()
               << "\n";

            os << "  DM2325_right"
               << " final_target=" << metrics.final_right_target
               << " final_cmd=" << metrics.final_right_cmd
               << " final_actual=" << metrics.final_right_actual
               << " actual_mean=" << metrics.right_actual.mean()
               << " velocity_error_rms=" << metrics.right_error.rms()
               << " velocity_error_max_abs=" << metrics.right_error.max_abs()
               << " reach95_s=";
            if (metrics.right_reached)
                os << metrics.right_reach_time_s;
            else
                os << "NA";
            os << " accel_max_abs_rad_s2=" << metrics.right_accel.max_abs()
               << " torque_mean_nm=" << metrics.right_torque.mean()
               << " torque_max_abs_nm=" << metrics.right_torque.max_abs()
               << "\n";

            os << "  max_temperature_c=" << metrics.temperature_c.max << "\n\n";
        }
    }

    write_identify_summary(os, opt);
    write_feedforward_summary(os, opt);
    write_scurve_summary(os, opt);
    write_contact_summary(os, opt);

    if (file.is_open())
        std::cout << "[LIFT-TEST] summary written to " << opt.summary_path << "\n";
}

float max_lift_distance_to(float rod_target, Options const &opt)
{
    float max_distance = 0.0f;
    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        const ModuleCommand &cmd = st_cmd[static_cast<int>(info.module)];
        max_distance = std::max(max_distance, std::fabs(cmd.lift_rod_cmd - rod_target));
    }
    return max_distance;
}

void print_phase_status(const std::string &phase, float elapsed_s, Options const &opt)
{
    std::cout << "[LIFT-TEST] " << phase
              << " t=" << std::fixed << std::setprecision(2) << elapsed_s << "s";

    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        const ModuleCommand &cmd = st_cmd[static_cast<int>(info.module)];
        auto &lift = lift_motor(info.module);
        auto &left = drive_left_motor(info.module);
        auto &right = drive_right_motor(info.module);

        std::cout << " | " << info.name
                  << " rod=" << (lift.Get_Now_Radian() / kLiftMotorToRodRatio)
                  << "/" << cmd.lift_rod_target
                  << " L=" << left.Get_Now_Omega()
                  << "/" << cmd.left_cmd
                  << " R=" << right.Get_Now_Omega()
                  << "/" << cmd.right_cmd;
    }

    std::cout << "\r" << std::flush;
}

void run_phase(Options const &opt,
               const std::string &phase,
               float rod_target,
               float forward_m_s,
               float yaw_rad_s,
               float duration_s,
               uint32_t &tick,
               std::ofstream &csv)
{
    set_phase_targets(opt, rod_target, forward_m_s, yaw_rad_s);

    const uint32_t total_ms = std::max<uint32_t>(1U, static_cast<uint32_t>(duration_s * 1000.0f));
    const uint32_t sample_period_ms =
        std::max<uint32_t>(1U, static_cast<uint32_t>(std::lround(1000.0f / std::max(1, opt.sample_hz))));

    auto next_wakeup = std::chrono::steady_clock::now();
    for (uint32_t ms = 0; ms < total_ms && st_running.load(); ++ms)
    {
        next_wakeup += std::chrono::milliseconds(kEcPeriodMs);

        if (!ec_step(tick++, opt))
            break;

        const float t_s = static_cast<float>(ms + 1U) * 0.001f;
        if ((ms % sample_period_ms) == 0U)
        {
            record_metrics_sample(opt, phase, t_s);
            if (opt.suite == TestSuite::Identify)
                record_identify_sample(opt, phase, t_s);
            if (opt.record)
                write_csv_sample(csv, opt, phase, t_s);
        }

        if ((ms % 200U) == 0U)
            print_phase_status(phase, t_s, opt);

        if (!temperature_safe(opt))
        {
            st_running.store(false);
            st_master.is_running = false;
            break;
        }

        std::this_thread::sleep_until(next_wakeup);
    }

    std::cout << "\n";
}

void stop_and_exit_selected(uint32_t &tick, Options const &opt)
{
    set_phase_targets(opt, opt.rod_start, 0.0f, 0.0f);
    run_for_ms(tick, 300, opt);

    if (!opt.exit_on_stop)
        return;

    st_command_enabled = false;
    for (int cycle = 0; cycle < 50; ++cycle)
    {
        ecat_master_sync(&st_master);
        linkx_recv_pdos(&st_linkx);

        can_msg_t msg {};
        for (uint8_t ch = 0; ch < kChannelCount; ++ch)
        {
            while (linkx_quick_recv(&st_linkx, ch, &msg))
                st_lift.CAN_Rx_Callback(ch, msg.id, msg.data);
        }

        for (const auto &info : kModuleInfo)
        {
            if (!module_selected(opt.module, info.module))
                continue;
            lift_motor(info.module).CAN_Send_Exit();
            drive_left_motor(info.module).CAN_Send_Exit();
            drive_right_motor(info.module).CAN_Send_Exit();
        }

        linkx_send_pdos(&st_linkx);
        std::this_thread::sleep_for(std::chrono::milliseconds(kEcPeriodMs));
        ++tick;
    }
}

float identify_move_duration_s(float from_angle, float to_angle, Options const &opt)
{
    return std::fabs(from_angle - to_angle) / std::max(0.1f, opt.lift_speed) + 0.35f;
}

void run_identify_suite(Options const &base_opt,
                        uint32_t &tick,
                        std::ofstream &csv)
{
    Options opt = base_opt;
    opt.suite = TestSuite::Identify;
    opt.drive_enable = false;
    opt.forward_m_s = 0.0f;
    opt.yaw_rad_s = 0.0f;
    opt.lift_speed = opt.identify_speed;
    opt.identify_angles = sanitize_identify_angles(opt.identify_angles, opt);

    reset_identify_sample_state();

    std::cout << "\n[LIFT-TEST][IDENTIFY]"
              << " lift_kp=" << opt.lift_kp
              << " lift_kd=" << opt.lift_kd
              << " identify_speed=" << opt.identify_speed
              << " hold_s=" << opt.identify_hold_s
              << " breakaway_vel=" << opt.identify_breakaway_vel
              << " angles=" << angle_list_text(opt.identify_angles)
              << " drive_enable=0\n";

    const float start_angle = opt.identify_angles.front();
    const float preposition_duration_s =
        std::max(opt.settle_s,
                 max_lift_distance_to(start_angle, opt) /
                     std::max(0.1f, opt.lift_speed) +
                 0.5f);

    run_phase(opt,
              "id_preposition_" + angle_token(start_angle),
              start_angle,
              0.0f,
              0.0f,
              preposition_duration_s,
              tick,
              csv);

    for (size_t i = 0; i < opt.identify_angles.size() && st_running.load(); ++i)
    {
        const float angle = opt.identify_angles[i];
        run_phase(opt,
                  "id_hold_" + angle_token(angle),
                  angle,
                  0.0f,
                  0.0f,
                  opt.identify_hold_s,
                  tick,
                  csv);

        if (!st_running.load() || i + 1U >= opt.identify_angles.size())
            break;

        const float next_angle = opt.identify_angles[i + 1U];
        run_phase(opt,
                  "id_raise_" + angle_token(angle) + "_to_" + angle_token(next_angle),
                  next_angle,
                  0.0f,
                  0.0f,
                  identify_move_duration_s(angle, next_angle, opt),
                  tick,
                  csv);
    }

    if (!st_running.load())
        return;

    for (size_t i = opt.identify_angles.size() - 1U; i > 0U && st_running.load(); --i)
    {
        const float from_angle = opt.identify_angles[i];
        const float to_angle = opt.identify_angles[i - 1U];
        run_phase(opt,
                  "id_retract_" + angle_token(from_angle) + "_to_" + angle_token(to_angle),
                  to_angle,
                  0.0f,
                  0.0f,
                  identify_move_duration_s(from_angle, to_angle, opt),
                  tick,
                  csv);

        if (!st_running.load())
            break;

        run_phase(opt,
                  "id_hold_return_" + angle_token(to_angle),
                  to_angle,
                  0.0f,
                  0.0f,
                  opt.identify_hold_s,
                  tick,
                  csv);
    }

    if (!st_running.load())
        return;

    run_phase(opt,
              "id_stop",
              opt.rod_start,
              0.0f,
              0.0f,
              opt.settle_s,
              tick,
              csv);
}

void set_contact_probe_targets(Options const &opt,
                               Enum_Chariot_Lift_Module contact_module,
                               float contact_target)
{
    const float safe_target = clamp_float(contact_target, kRodRangeLow, kRodRangeHigh);
    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        const int idx = static_cast<int>(info.module);
        ModuleCommand &cmd = st_cmd[idx];
        cmd.lift_rod_target =
            (info.module == contact_module) ? safe_target : opt.rod_start;
        cmd.lift_rod_omega_cmd = 0.0f;
        st_scurve[idx] = {};
        compute_drive_targets_for_module(opt, 0.0f, 0.0f, cmd);
    }
}

ContactResult run_contact_probe(Options const &opt,
                                const ModuleInfo &contact_info,
                                uint32_t &tick,
                                std::ofstream &csv)
{
    const std::string phase = std::string("contact_") + contact_info.name + "_lower";
    const int idx = static_cast<int>(contact_info.module);
    ContactResult result {};
    result.phase = phase;
    result.module = contact_info.module;
    result.module_name = contact_info.name;
    result.can_channel = contact_info.can_channel;

    set_contact_probe_targets(opt, contact_info.module, opt.rod_end);

    const float travel = std::fabs(opt.rod_start - opt.rod_end);
    const float duration_s =
        travel / std::max(0.05f, opt.contact_speed) +
        opt.contact_hold_s +
        1.5f;
    const uint32_t total_ms =
        std::max<uint32_t>(1U, static_cast<uint32_t>(duration_s * 1000.0f));
    const uint32_t sample_period_ms =
        std::max<uint32_t>(1U, static_cast<uint32_t>(std::lround(1000.0f / std::max(1, opt.sample_hz))));
    const uint32_t hold_ms =
        std::max<uint32_t>(1U, static_cast<uint32_t>(opt.contact_hold_s * 1000.0f));

    std::cout << "\n[LIFT-TEST][CONTACT] module=" << contact_info.name
              << " speed=" << opt.contact_speed
              << " threshold_lowering_load_nm=" << opt.contact_threshold_nm
              << " ignore_s=" << opt.contact_ignore_s
              << " min_travel=" << opt.contact_min_travel
              << " confirm_ms=" << opt.contact_confirm_ms
              << " hold_s=" << opt.contact_hold_s
              << " preload_rad=" << opt.contact_preload_rad << "\n";

    uint32_t confirm_ms = 0U;
    uint32_t detected_hold_ms = 0U;
    auto next_wakeup = std::chrono::steady_clock::now();

    for (uint32_t ms = 0; ms < total_ms && st_running.load(); ++ms)
    {
        next_wakeup += std::chrono::milliseconds(kEcPeriodMs);

        if (!ec_step(tick++, opt))
            break;

        auto &lift = lift_motor(contact_info.module);
        const ModuleCommand &cmd = st_cmd[idx];
        const float t_s = static_cast<float>(ms + 1U) * 0.001f;
        const float rod_actual = lift.Get_Now_Radian() / kLiftMotorToRodRatio;
        const float rod_error = rod_actual - cmd.lift_rod_cmd;
        const float torque_nm = lift.Get_Now_Torque();
        const float hold_torque_nm = lift_hold_torque(contact_info.module, rod_actual);
        const float residual_nm = torque_nm - hold_torque_nm;
        const float lowering_load_nm = hold_torque_nm - torque_nm;
        const float rod_velocity = lift.Get_Now_Omega() / kLiftMotorToRodRatio;

        result.torque.add(torque_nm);
        result.hold_torque.add(hold_torque_nm);
        result.residual.add(residual_nm);
        result.lowering_load.add(lowering_load_nm);
        result.rod_error.add(rod_error);
        result.rod_velocity.add(rod_velocity);

        if (result.detected)
        {
            result.hold_residual.add(residual_nm);
            result.hold_lowering_load.add(lowering_load_nm);
            ++detected_hold_ms;
        }
        else
        {
            const float traveled = std::fabs(rod_actual - opt.rod_start);
            const bool contact_candidate =
                t_s >= opt.contact_ignore_s &&
                traveled >= opt.contact_min_travel &&
                lowering_load_nm >= opt.contact_threshold_nm;
            if (contact_candidate)
                ++confirm_ms;
            else
                confirm_ms = 0U;

            if (confirm_ms >= static_cast<uint32_t>(std::max(1, opt.contact_confirm_ms)))
            {
                result.detected = true;
                result.detect_t_s = t_s;
                result.detect_rod_cmd = cmd.lift_rod_cmd;
                result.detect_rod_actual = rod_actual;
                result.detect_rod_error = rod_error;
                result.detect_torque_nm = torque_nm;
                result.detect_hold_torque_nm = hold_torque_nm;
                result.detect_residual_nm = residual_nm;
                result.detect_lowering_load_nm = lowering_load_nm;
                result.detect_motor_omega = lift.Get_Now_Omega();

                ModuleCommand &mutable_cmd = st_cmd[idx];
                const float hold_target =
                    clamp_float(rod_actual - opt.contact_preload_rad,
                                kRodRangeLow,
                                kRodRangeHigh);
                mutable_cmd.lift_rod_target = hold_target;
                mutable_cmd.lift_rod_cmd = hold_target;
                mutable_cmd.lift_rod_omega_cmd = 0.0f;
                st_scurve[idx] = {};

                std::cout << "\n[LIFT-TEST][CONTACT] detected module="
                          << contact_info.name
                          << " t=" << result.detect_t_s
                          << " rod_actual=" << result.detect_rod_actual
                          << " threshold_lowering_load_nm=" << opt.contact_threshold_nm
                          << " lowering_load_nm=" << result.detect_lowering_load_nm
                          << " residual_nm=" << result.detect_residual_nm << "\n";
            }
        }

        if ((ms % sample_period_ms) == 0U)
        {
            record_metrics_sample(opt, phase, t_s);
            if (opt.record)
                write_csv_sample(csv, opt, phase, t_s);
        }

        if ((ms % 200U) == 0U)
            print_phase_status(phase, t_s, opt);

        if (!temperature_safe(opt))
        {
            st_running.store(false);
            st_master.is_running = false;
            break;
        }

        if (result.detected && detected_hold_ms >= hold_ms)
            break;

        std::this_thread::sleep_until(next_wakeup);
    }

    if (!result.detected)
    {
        result.timeout = true;
        std::cout << "\n[LIFT-TEST][CONTACT] timeout module="
                  << contact_info.name
                  << " threshold_lowering_load_nm=" << opt.contact_threshold_nm << "\n";
    }

    std::cout << "\n";
    return result;
}

void run_contact_suite(Options const &base_opt,
                       uint32_t &tick,
                       std::ofstream &csv)
{
    Options opt = base_opt;
    opt.suite = TestSuite::Contact;
    opt.drive_enable = false;
    opt.forward_m_s = 0.0f;
    opt.yaw_rad_s = 0.0f;
    opt.lift_speed = opt.contact_speed;
    opt.feedforward_mode = FeedforwardMode::Hold;
    opt.lift_profile = LiftCommandProfile::Trapezoid;
    opt.lift_velocity_ff = false;

    st_contact_results.clear();

    const float preposition_duration_s =
        std::max(opt.settle_s,
                 max_lift_distance_to(opt.rod_start, opt) /
                     std::max(0.05f, opt.contact_speed) +
                 0.5f);
    run_phase(opt,
                 "contact_preposition",
                 opt.rod_start,
                 0.0f,
                 0.0f,
                 preposition_duration_s,
              tick,
              csv);

    for (const auto &info : kModuleInfo)
    {
        if (!st_running.load())
            break;
        if (!module_selected(opt.module, info.module))
            continue;

        st_contact_results.push_back(run_contact_probe(opt, info, tick, csv));

        if (!st_running.load())
            break;

        const float retract_duration_s =
            std::max(opt.settle_s,
                     max_lift_distance_to(opt.rod_start, opt) /
                         std::max(0.05f, opt.contact_speed) +
                     0.5f);
        run_phase(opt,
                  std::string("contact_") + info.name + "_retract",
                  opt.rod_start,
                  0.0f,
                  0.0f,
                  retract_duration_s,
                  tick,
                  csv);
    }
}

void run_test_case(Options const &base_opt,
                   TestCase const &test_case,
                   uint32_t &tick,
                   std::ofstream &csv)
{
    Options opt = base_opt;
    opt.lift_speed = test_case.lift_speed;
    opt.lift_kp = test_case.lift_kp;
    opt.lift_kd = test_case.lift_kd;
    opt.forward_m_s = test_case.forward_m_s;
    opt.yaw_rad_s = test_case.yaw_rad_s;
    opt.drive_kp = test_case.drive_kp;
    opt.drive_kd = test_case.drive_kd;
    opt.drive_accel = test_case.drive_accel;
    opt.drive_decel = test_case.drive_decel;
    opt.drive_enable = test_case.drive_enable;
    opt.feedforward_mode = test_case.feedforward_mode;
    opt.lift_profile = test_case.lift_profile;
    opt.lift_velocity_ff = test_case.lift_velocity_ff;

    const float rod_range = std::fabs(opt.rod_start - opt.rod_end);
    const float sweep_duration_s =
        profile_distance_duration_s(rod_range, opt) + opt.hold_s;
    const float preposition_duration_s =
        std::max(opt.settle_s,
                 profile_distance_duration_s(max_lift_distance_to(opt.rod_start, opt), opt) +
                 0.5f);

    std::cout << "\n[LIFT-TEST][CASE] " << test_case.label
              << " lift_kp=" << opt.lift_kp
              << " lift_kd=" << opt.lift_kd
              << " lift_speed=" << opt.lift_speed
              << " forward=" << opt.forward_m_s
              << " yaw=" << opt.yaw_rad_s
              << " drive_enable=" << (opt.drive_enable ? 1 : 0)
              << " drive_kd=" << opt.drive_kd
              << " accel=" << opt.drive_accel
              << " decel=" << opt.drive_decel
              << " feedforward=" << feedforward_mode_name(opt.feedforward_mode)
              << " profile=" << lift_profile_name(opt.lift_profile)
              << " lift_velocity_ff=" << (opt.lift_velocity_ff ? 1 : 0)
              << " cycles=" << opt.cycles << "\n";

    const std::string prefix = test_case.label + "_";

    run_phase(opt,
              prefix + "preposition",
              opt.rod_start,
              0.0f,
              0.0f,
              preposition_duration_s,
              tick,
              csv);

    for (int cycle = 0; cycle < opt.cycles && st_running.load(); ++cycle)
    {
        const std::string suffix = "_c" + std::to_string(cycle + 1);
        run_phase(opt,
                  prefix + "raise" + suffix,
                  opt.rod_end,
                  opt.forward_m_s,
                  opt.yaw_rad_s,
                  sweep_duration_s,
                  tick,
                  csv);

        if (!st_running.load())
            break;

        run_phase(opt,
                  prefix + "retract" + suffix,
                  opt.rod_start,
                  -opt.forward_m_s,
                  -opt.yaw_rad_s,
                  sweep_duration_s,
                  tick,
                  csv);
    }

    if (!st_running.load())
        return;

    run_phase(opt,
              prefix + "stop",
              opt.rod_start,
              0.0f,
              0.0f,
              opt.settle_s,
              tick,
              csv);
}

void print_usage(const char *argv0)
{
    std::cerr
        << "Usage:\n"
        << "  sudo IFNAME=enp86s0 " << argv0 << " [options]\n\n"
        << "Options:\n"
        << "  --ifname IFACE       EtherCAT NIC, default IFNAME env or enp86s0\n"
        << "  --module M           front|rear|both, default both\n"
        << "  --suite S            lift|identify|ff|scurve|fast|contact|param|single, default lift\n"
        << "  --cycles N           raise/retract cycles per case, default 1\n"
        << "  --rod-start RAD      lift rod start angle, default -1\n"
        << "  --rod-end RAD        lift rod raised angle, default -15\n"
        << "  --settle SEC         preposition/stop settle time, default 1.0\n"
        << "  --hold SEC           extra time at each sweep endpoint, default 0.75\n"
        << "  --lift-speed RADS    lift rod profile speed, capped at 8.0, default 8.0\n"
        << "  --lift-kp K          DM3519 MIT Kp, default 15\n"
        << "  --lift-kd K          DM3519 MIT Kd, default 1\n"
        << "  --lift-velocity-ff 0|1  command DM3519 MIT velocity target in single suite, default 0\n"
        << "  --forward MPS        DM2325 diff-drive forward command, default 0.25\n"
        << "  --yaw RADS           DM2325 diff-drive yaw command, default 0\n"
        << "  --drive-kp K         DM2325 MIT Kp, default 0\n"
        << "  --drive-kd K         DM2325 MIT Kd, default 1.2\n"
        << "  --drive-accel A      DM2325 accel limit rad/s^2, default 300\n"
        << "  --drive-decel A      DM2325 decel limit rad/s^2, default 600\n"
        << "  --drive-enable 0|1   command DM2325 drive motors in single suite, default 0\n"
        << "  --identify-speed V   identify rod speed rad/s, default 3.0\n"
        << "  --identify-hold SEC  identify hold duration at each angle, default 1.2\n"
        << "  --identify-angles A  comma list of rod angles, default -1,-4.5,-8,-11.5,-15\n"
        << "  --identify-breakaway-vel V  rod velocity threshold for stiction residual, default 0.20\n"
        << "  --contact-speed V    contact probe lowering speed rad/s, default 0.8\n"
        << "  --contact-threshold T  positive lowering-load threshold Nm, default 0.18\n"
        << "  --contact-ignore SEC ignore early motion for contact, default 0.5\n"
        << "  --contact-min-travel RAD  minimum travel before contact detect, default 6.0\n"
        << "  --contact-confirm-ms MS  consecutive contact time, default 120\n"
        << "  --contact-hold SEC   hold after contact detection before retract, default 0.0\n"
        << "  --contact-preload RAD  extra downward command after contact, default 0.0\n"
        << "                       identify suite defaults to lift-kp=20 lift-kd=1.2 and disables DM2325\n"
        << "                       ff suite runs PD-only, hold-ff, hold+friction-ff at speed 5\n"
        << "                       scurve suite compares pos-only and pos+velocity S-curve profiles at speed 8/9/10/12\n"
        << "                       fast suite tunes high-speed pos+velocity S-curve at speed 12/13/14/15/18/20\n"
        << "  --sample-hz HZ       CSV sample rate, default 100\n"
        << "  --record 0|1         write CSV, default 1\n"
        << "  --csv PATH           CSV path\n"
        << "  --summary PATH       parameter summary path\n"
        << "  --exit-on-stop 0|1   send DM exit frames before return, default 1\n";
}

Options parse_options(int argc, char **argv)
{
    Options opt;
    if (const char *env = std::getenv("IFNAME"))
        opt.ifname = env;

    opt.ifname = cli_get(argc, argv, "ifname", opt.ifname.c_str());

    ModuleSelection selection = ModuleSelection::Both;
    const std::string module_text = cli_get(
        argc,
        argv,
        "module",
        std::getenv("LIFT_TEST_MODULE") ? std::getenv("LIFT_TEST_MODULE") : "both");
    if (!parse_module_selection(module_text, selection))
    {
        std::cerr << "[LIFT-TEST] invalid --module '" << module_text
                  << "', expected front|rear|both\n";
        print_usage(argv[0]);
        std::exit(1);
    }
    opt.module = selection;

    TestSuite suite = TestSuite::Lift;
    const std::string suite_text = cli_get(
        argc,
        argv,
        "suite",
        std::getenv("LIFT_TEST_SUITE") ? std::getenv("LIFT_TEST_SUITE") : "lift");
    if (!parse_test_suite(suite_text, suite))
    {
        std::cerr << "[LIFT-TEST] invalid --suite '" << suite_text
                  << "', expected lift|identify|ff|scurve|fast|contact|param|single\n";
        print_usage(argv[0]);
        std::exit(1);
    }
    opt.suite = suite;

    const bool lift_kp_override =
        cli_specified(argc, argv, "lift-kp") || std::getenv("LIFT_TEST_LIFT_KP") != nullptr;
    const bool lift_kd_override =
        cli_specified(argc, argv, "lift-kd") || std::getenv("LIFT_TEST_LIFT_KD") != nullptr;

    opt.cycles = std::atoi(cli_get(argc, argv, "cycles",
                                   std::to_string(env_i("LIFT_TEST_CYCLES", opt.cycles)).c_str()));
    opt.rod_start = std::strtof(cli_get(argc, argv, "rod-start",
                                        std::to_string(env_f("LIFT_TEST_ROD_START", opt.rod_start)).c_str()),
                                nullptr);
    opt.rod_end = std::strtof(cli_get(argc, argv, "rod-end",
                                      std::to_string(env_f("LIFT_TEST_ROD_END", opt.rod_end)).c_str()),
                              nullptr);
    opt.settle_s = std::strtof(cli_get(argc, argv, "settle",
                                       std::to_string(env_f("LIFT_TEST_SETTLE", opt.settle_s)).c_str()),
                               nullptr);
    opt.hold_s = std::strtof(cli_get(argc, argv, "hold",
                                     std::to_string(env_f("LIFT_TEST_HOLD", opt.hold_s)).c_str()),
                             nullptr);
    opt.lift_speed = std::strtof(cli_get(argc, argv, "lift-speed",
                                         std::to_string(env_f("LIFT_TEST_LIFT_SPEED", opt.lift_speed)).c_str()),
                                 nullptr);
    opt.lift_kp = std::strtof(cli_get(argc, argv, "lift-kp",
                                      std::to_string(env_f("LIFT_TEST_LIFT_KP", opt.lift_kp)).c_str()),
                              nullptr);
    opt.lift_kd = std::strtof(cli_get(argc, argv, "lift-kd",
                                      std::to_string(env_f("LIFT_TEST_LIFT_KD", opt.lift_kd)).c_str()),
                              nullptr);
    opt.lift_velocity_ff = parse_bool(cli_get(argc, argv, "lift-velocity-ff",
                                              std::to_string(env_i("LIFT_TEST_LIFT_VELOCITY_FF", opt.lift_velocity_ff ? 1 : 0)).c_str()),
                                      opt.lift_velocity_ff);
    opt.forward_m_s = std::strtof(cli_get(argc, argv, "forward",
                                          std::to_string(env_f("LIFT_TEST_FORWARD", opt.forward_m_s)).c_str()),
                                  nullptr);
    opt.yaw_rad_s = std::strtof(cli_get(argc, argv, "yaw",
                                        std::to_string(env_f("LIFT_TEST_YAW", opt.yaw_rad_s)).c_str()),
                                nullptr);
    opt.drive_kp = std::strtof(cli_get(argc, argv, "drive-kp",
                                       std::to_string(env_f("LIFT_TEST_DRIVE_KP", opt.drive_kp)).c_str()),
                               nullptr);
    opt.drive_kd = std::strtof(cli_get(argc, argv, "drive-kd",
                                       std::to_string(env_f("LIFT_TEST_DRIVE_KD", opt.drive_kd)).c_str()),
                               nullptr);
    opt.drive_accel = std::strtof(cli_get(argc, argv, "drive-accel",
                                          std::to_string(env_f("LIFT_TEST_DRIVE_ACCEL", opt.drive_accel)).c_str()),
                                  nullptr);
    opt.drive_decel = std::strtof(cli_get(argc, argv, "drive-decel",
                                          std::to_string(env_f("LIFT_TEST_DRIVE_DECEL", opt.drive_decel)).c_str()),
                                  nullptr);
    opt.drive_enable = parse_bool(cli_get(argc, argv, "drive-enable",
                                          std::to_string(env_i("LIFT_TEST_DRIVE_ENABLE", opt.drive_enable ? 1 : 0)).c_str()),
                                  opt.drive_enable);
    opt.identify_speed = std::strtof(cli_get(argc, argv, "identify-speed",
                                             std::to_string(env_f("LIFT_TEST_IDENTIFY_SPEED", opt.identify_speed)).c_str()),
                                     nullptr);
    opt.identify_hold_s = std::strtof(cli_get(argc, argv, "identify-hold",
                                              std::to_string(env_f("LIFT_TEST_IDENTIFY_HOLD", opt.identify_hold_s)).c_str()),
                                      nullptr);
    opt.identify_breakaway_vel = std::strtof(cli_get(argc, argv, "identify-breakaway-vel",
                                                     std::to_string(env_f("LIFT_TEST_IDENTIFY_BREAKAWAY_VEL", opt.identify_breakaway_vel)).c_str()),
                                             nullptr);
    opt.contact_speed = std::strtof(cli_get(argc, argv, "contact-speed",
                                            std::to_string(env_f("LIFT_TEST_CONTACT_SPEED", opt.contact_speed)).c_str()),
                                    nullptr);
    opt.contact_threshold_nm = std::strtof(cli_get(argc, argv, "contact-threshold",
                                                   std::to_string(env_f("LIFT_TEST_CONTACT_THRESHOLD", opt.contact_threshold_nm)).c_str()),
                                           nullptr);
    opt.contact_ignore_s = std::strtof(cli_get(argc, argv, "contact-ignore",
                                               std::to_string(env_f("LIFT_TEST_CONTACT_IGNORE", opt.contact_ignore_s)).c_str()),
                                       nullptr);
    opt.contact_min_travel = std::strtof(cli_get(argc, argv, "contact-min-travel",
                                                 std::to_string(env_f("LIFT_TEST_CONTACT_MIN_TRAVEL", opt.contact_min_travel)).c_str()),
                                         nullptr);
    opt.contact_confirm_ms = std::atoi(cli_get(argc, argv, "contact-confirm-ms",
                                               std::to_string(env_i("LIFT_TEST_CONTACT_CONFIRM_MS", opt.contact_confirm_ms)).c_str()));
    opt.contact_hold_s = std::strtof(cli_get(argc, argv, "contact-hold",
                                             std::to_string(env_f("LIFT_TEST_CONTACT_HOLD", opt.contact_hold_s)).c_str()),
                                     nullptr);
    opt.contact_preload_rad = std::strtof(cli_get(argc, argv, "contact-preload",
                                                  std::to_string(env_f("LIFT_TEST_CONTACT_PRELOAD", opt.contact_preload_rad)).c_str()),
                                          nullptr);
    {
        const char *angles_env = std::getenv("LIFT_TEST_IDENTIFY_ANGLES");
        const std::string default_angles = angle_list_text(opt.identify_angles);
        const std::string angles_text = cli_get(argc,
                                                argv,
                                                "identify-angles",
                                                angles_env ? angles_env : default_angles.c_str());
        opt.identify_angles = parse_float_list(angles_text, opt.identify_angles);
    }
    opt.sample_hz = std::atoi(cli_get(argc, argv, "sample-hz",
                                      std::to_string(env_i("LIFT_TEST_SAMPLE_HZ", opt.sample_hz)).c_str()));
    opt.record = parse_bool(cli_get(argc, argv, "record",
                                    std::to_string(env_i("LIFT_TEST_RECORD", opt.record ? 1 : 0)).c_str()),
                            opt.record);
    opt.exit_on_stop = parse_bool(cli_get(argc, argv, "exit-on-stop",
                                          std::to_string(env_i("LIFT_TEST_EXIT_ON_STOP", opt.exit_on_stop ? 1 : 0)).c_str()),
                                  opt.exit_on_stop);

    if (opt.suite == TestSuite::Identify ||
        opt.suite == TestSuite::Feedforward ||
        opt.suite == TestSuite::SCurve ||
        opt.suite == TestSuite::Fast ||
        opt.suite == TestSuite::Contact)
    {
        if (!lift_kp_override)
            opt.lift_kp = kIdentifyLiftKp;
        if (!lift_kd_override)
            opt.lift_kd = kIdentifyLiftKd;
        opt.drive_enable = false;
        opt.forward_m_s = 0.0f;
        opt.yaw_rad_s = 0.0f;
    }

    opt.cycles = std::max(1, opt.cycles);
    opt.rod_start = clamp_float(opt.rod_start, kRodRangeLow, kRodRangeHigh);
    opt.rod_end = clamp_float(opt.rod_end, kRodRangeLow, kRodRangeHigh);
    if (std::fabs(opt.rod_start - opt.rod_end) < 1e-4f)
    {
        std::cerr << "[LIFT-TEST] rod-start and rod-end are equal after safety clamp.\n";
        std::exit(1);
    }

    opt.settle_s = std::max(0.0f, opt.settle_s);
    opt.hold_s = std::max(0.0f, opt.hold_s);
    opt.lift_speed = clamp_float(opt.lift_speed, 0.1f, kLiftRodMaxSpeed);
    opt.lift_kp = clamp_float(opt.lift_kp, 0.0f, 500.0f);
    opt.lift_kd = clamp_float(opt.lift_kd, 0.0f, 5.0f);
    opt.forward_m_s = clamp_float(opt.forward_m_s, -kDriveMaxForward, kDriveMaxForward);
    opt.yaw_rad_s = clamp_float(opt.yaw_rad_s, -kDriveMaxYaw, kDriveMaxYaw);
    opt.drive_kp = clamp_float(opt.drive_kp, 0.0f, 500.0f);
    opt.drive_kd = clamp_float(opt.drive_kd, 0.0f, 5.0f);
    opt.drive_accel = std::max(1.0f, opt.drive_accel);
    opt.drive_decel = std::max(1.0f, opt.drive_decel);
    opt.identify_speed = clamp_float(opt.identify_speed, 0.1f, kLiftRodMaxSpeed);
    opt.identify_hold_s = std::max(0.3f, opt.identify_hold_s);
    opt.identify_breakaway_vel = clamp_float(opt.identify_breakaway_vel, 0.01f, kLiftRodMaxSpeed);
    opt.identify_angles = sanitize_identify_angles(opt.identify_angles, opt);
    opt.contact_speed = clamp_float(opt.contact_speed, 0.05f, 3.0f);
    opt.contact_threshold_nm = clamp_float(opt.contact_threshold_nm, 0.05f, 6.0f);
    opt.contact_ignore_s = clamp_float(opt.contact_ignore_s, 0.0f, 3.0f);
    opt.contact_min_travel = clamp_float(opt.contact_min_travel, 0.0f, 13.5f);
    opt.contact_confirm_ms = std::max(1, std::min(1000, opt.contact_confirm_ms));
    opt.contact_hold_s = clamp_float(opt.contact_hold_s, 0.0f, 5.0f);
    opt.contact_preload_rad = clamp_float(opt.contact_preload_rad, 0.0f, 1.0f);
    opt.sample_hz = std::max(1, std::min(500, opt.sample_hz));

    mkdir("var_data", 0755);
    const std::string ts = timestamp_string();
    const std::string default_csv =
        "var_data/lift_raise_drive_test_" + ts + ".csv";
    const std::string default_summary =
        "var_data/lift_raise_drive_test_" + ts + "_summary.txt";
    opt.csv_path = cli_get(argc, argv, "csv", default_csv.c_str());
    opt.summary_path = cli_get(argc, argv, "summary", default_summary.c_str());
    return opt;
}

void print_motor_table(Options const &opt)
{
    std::cout << std::fixed << std::setprecision(3);
    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;

        auto &lift = lift_motor(info.module);
        auto &left = drive_left_motor(info.module);
        auto &right = drive_right_motor(info.module);

        std::cout << "  " << info.name
                  << " ch" << static_cast<int>(info.can_channel)
                  << " lift3519 tx=0x" << std::hex << lift.DM_CAN_Tx_ID
                  << " rx=0x" << lift.DM_CAN_Rx_ID << std::dec
                  << " status=" << dm_status_name(lift.Get_Status())
                  << " ctrl=" << dm_ctrl_name(lift.Get_Now_Control_Status())
                  << " rod_now=" << lift.Get_Now_Radian() / kLiftMotorToRodRatio
                  << " motor_now=" << lift.Get_Now_Radian()
                  << "\n"
                  << "      drive_left2325 tx=0x" << std::hex << left.DM_CAN_Tx_ID
                  << " rx=0x" << left.DM_CAN_Rx_ID << std::dec
                  << " status=" << dm_status_name(left.Get_Status())
                  << " ctrl=" << dm_ctrl_name(left.Get_Now_Control_Status())
                  << " omega=" << left.Get_Now_Omega()
                  << "\n"
                  << "      drive_right2325 tx=0x" << std::hex << right.DM_CAN_Tx_ID
                  << " rx=0x" << right.DM_CAN_Rx_ID << std::dec
                  << " status=" << dm_status_name(right.Get_Status())
                  << " ctrl=" << dm_ctrl_name(right.Get_Now_Control_Status())
                  << " omega=" << right.Get_Now_Omega()
                  << "\n";
    }
    std::cout << std::defaultfloat;
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    if (cli_has(argc, argv, "help") || cli_has(argc, argv, "h"))
    {
        print_usage(argv[0]);
        return 0;
    }

    Options opt = parse_options(argc, argv);
    const std::vector<TestCase> test_cases = build_test_cases(opt);
    const size_t displayed_case_count =
        (opt.suite == TestSuite::Identify) ? 1U :
        (opt.suite == TestSuite::Contact) ?
            ((opt.module == ModuleSelection::Both) ? 2U : 1U) :
        test_cases.size();

    const float motor_start = opt.rod_start * kLiftMotorToRodRatio;
    const float motor_end = opt.rod_end * kLiftMotorToRodRatio;

    std::cout << "===============================================\n"
              << "  R2 Lift Raise + Drive Combined Test\n"
              << "  IFNAME     : " << opt.ifname << "\n"
              << "  MODULE     : " << module_selection_name(opt.module) << "\n"
              << "  SUITE      : " << test_suite_name(opt.suite) << "\n"
              << "  ROD RANGE  : [" << opt.rod_start << ", " << opt.rod_end
              << "] rad\n"
              << "  MOTOR RANGE: [" << motor_start << ", " << motor_end
              << "] rad (rod * 3)\n"
              << "  CASES      : " << displayed_case_count << "\n"
              << "  CYCLES     : " << opt.cycles << " per case\n"
              << "  CSV        : " << (opt.record ? opt.csv_path : "disabled") << "\n"
              << "  SUMMARY    : " << opt.summary_path << "\n"
              << "===============================================\n"
              << "[SAFETY] Keep the lift mechanism clear and support the drive wheels.\n"
              << "[SAFETY] Ctrl+C stops commands and sends DM exit frames when exit-on-stop=1.\n";

    for (const auto &test_case : test_cases)
    {
        std::cout << "  case=" << test_case.label
                  << " lift_kp=" << test_case.lift_kp
                  << " lift_kd=" << test_case.lift_kd
                  << " lift_speed=" << test_case.lift_speed
                  << " forward=" << test_case.forward_m_s
                  << " drive_enable=" << (test_case.drive_enable ? 1 : 0)
                  << " drive_kd=" << test_case.drive_kd
                  << " feedforward=" << feedforward_mode_name(test_case.feedforward_mode)
                  << " profile=" << lift_profile_name(test_case.lift_profile)
                  << " lift_velocity_ff=" << (test_case.lift_velocity_ff ? 1 : 0)
                  << "\n";
    }

    if (opt.suite == TestSuite::Identify)
    {
        std::cout << "  identify"
                  << " lift_kp=" << opt.lift_kp
                  << " lift_kd=" << opt.lift_kd
                  << " speed=" << opt.identify_speed
                  << " hold_s=" << opt.identify_hold_s
                  << " breakaway_vel=" << opt.identify_breakaway_vel
                  << " angles=" << angle_list_text(opt.identify_angles)
                  << " drive_enable=0\n"
                  << "[SAFETY] identify suite disables DM2325 drive motors.\n";
    }

    if (opt.suite == TestSuite::Contact)
    {
        std::cout << "  contact"
                  << " lift_kp=" << opt.lift_kp
                  << " lift_kd=" << opt.lift_kd
                  << " speed=" << opt.contact_speed
                  << " threshold_lowering_load_nm=" << opt.contact_threshold_nm
                  << " ignore_s=" << opt.contact_ignore_s
                  << " min_travel=" << opt.contact_min_travel
                  << " confirm_ms=" << opt.contact_confirm_ms
                  << " hold_s=" << opt.contact_hold_s
                  << " preload_rad=" << opt.contact_preload_rad
                  << " drive_enable=0\n"
                  << "[SAFETY] contact suite slowly lowers one lift at a time and freezes command when contact is detected.\n";
    }

    if (!init_ethercat_linkx(opt))
        return 2;

    st_lift.Init(&st_linkx);

    for (const auto &info : kModuleInfo)
    {
        if (!module_selected(opt.module, info.module))
            continue;
        lift_motor(info.module).Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        drive_left_motor(info.module).Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
        drive_right_motor(info.module).Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
    }

    uint32_t tick = 0;
    std::cout << "[LIFT-TEST] enabling selected motors and waiting for feedback...\n";
    run_for_ms(tick, 1500, opt);
    print_motor_table(opt);

    init_command_state_from_feedback(opt);
    st_command_enabled = true;

    std::ofstream csv;
    if (opt.record)
    {
        csv.open(opt.csv_path, std::ios::out | std::ios::trunc);
        if (!csv.is_open())
        {
            std::cerr << "[LIFT-TEST] failed to open CSV: " << opt.csv_path << "\n";
            stop_and_exit_selected(tick, opt);
            return 3;
        }
        write_csv_header(csv);
    }

    if (opt.suite == TestSuite::Identify)
    {
        run_identify_suite(opt, tick, csv);
    }
    else if (opt.suite == TestSuite::Contact)
    {
        run_contact_suite(opt, tick, csv);
    }
    else
    {
        for (const auto &test_case : test_cases)
        {
            if (!st_running.load())
                break;
            run_test_case(opt, test_case, tick, csv);
        }
    }

    write_summary(opt);
    stop_and_exit_selected(tick, opt);
    print_motor_table(opt);

    if (opt.record)
        std::cout << "[LIFT-TEST] CSV written to " << opt.csv_path << "\n";
    std::cout << "[LIFT-TEST] done.\n";
    return 0;
}

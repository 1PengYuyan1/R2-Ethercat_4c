// Lift-raised straight-line IMU autotune.
//
// Direct EtherCAT test for Class_Chariot_Lift:
//   1. raise front/rear lift motors to (-24,+24) by default;
//   2. drive the four DM2325 friction wheels in straight-line segments;
//   3. optionally close IMU yaw hold in the test process;
//   4. score wheel balance and yaw quality, then write CSV/YAML/report artifacts.

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
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "crt_lift.h"
#include "dvc_motor_dm.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"

namespace
{
constexpr int kChannelCount = 4;
constexpr uint32_t kEcPeriodMs = 1;
constexpr uint32_t kControlPeriodTicks = 2;
constexpr uint32_t kAlivePeriodTicks = 100;
constexpr uint32_t kEcatStableRequiredMs = 500;
constexpr float kLiftMotorToRodRatio = 3.0f;
constexpr double kPi = 3.14159265358979323846;
constexpr double kWheelRadiusM = 0.0761;
constexpr double kTrackWidthM = 0.420;
constexpr double kMaxForwardMps = 2.0;
constexpr double kMaxYawRadS = 2.0;

ecat_master_t g_master {};
linkx_t g_linkx {};
Class_Chariot_Lift g_lift;
std::atomic<bool> g_running {true};
uint32_t g_last_wkc = 0;
bool g_ecat_op = false;
uint32_t g_ecat_good_ms = 0;
uint32_t g_ecat_bad_ms = 0;

enum class Mode
{
    WheelAlign,
    ImuHold,
    StraightEval,
    OnlineTune,
};

enum class DirectionMode
{
    Forward,
    Back,
    ForwardBack,
};

struct Options
{
    Mode mode = Mode::WheelAlign;
    std::string ifname = "enp4s0";
    int slave_id = 2;
    float front_raise = kLiftFrontRaiseMotorAngle;
    float rear_raise = kLiftRearRaiseMotorAngle;
    std::string imu_topic = "/IMU_data";
    DirectionMode direction = DirectionMode::ForwardBack;
    double speed = 0.6;
    double distance = 2.0;
    double linear_accel = 0.6;
    double linear_decel = 0.6;
    double yaw_kp = 3.0;
    double yaw_ki = 0.0;
    double yaw_kd = 0.05;
    double yaw_out_limit = 1.0;
    double drive_kp = 0.0;
    double drive_kd = 1.2;
    double front_left_corr = 1.0;
    double front_right_corr = 1.0;
    double rear_left_corr = 1.0;
    double rear_right_corr = 1.0;
    int repeats = 3;
    double settle_s = 1.0;
    int max_rounds = 30;
    int sample_hz = 100;
    double imu_wait_s = 1.0;
    double imu_stale_s = 0.25;
    double lift_timeout_s = 12.0;
    std::string output_dir = "var_data/lift_straight";
    bool yes = false;
    bool dry_run = false;
};

struct ImuSample
{
    bool valid = false;
    int64_t ns = 0;
    double yaw_rad = 0.0;
    double gyro_z = 0.0;
};

struct TuneParams
{
    double front_left_corr = 1.0;
    double front_right_corr = 1.0;
    double rear_left_corr = 1.0;
    double rear_right_corr = 1.0;
    double yaw_kp = 3.0;
    double yaw_ki = 0.0;
    double yaw_kd = 0.05;
    double yaw_out_limit = 1.0;
    double drive_kp = 0.0;
    double drive_kd = 1.2;
};

struct Stats
{
    uint64_t n = 0;
    double sum = 0.0;
    double sum_sq = 0.0;
    double min = std::numeric_limits<double>::infinity();
    double max = -std::numeric_limits<double>::infinity();

    void add(double v)
    {
        if (!std::isfinite(v))
            return;
        ++n;
        sum += v;
        sum_sq += v * v;
        min = std::min(min, v);
        max = std::max(max, v);
    }
    double mean() const { return n ? sum / static_cast<double>(n) : 0.0; }
    double rms() const { return n ? std::sqrt(sum_sq / static_cast<double>(n)) : 0.0; }
    double stddev() const
    {
        if (n < 2)
            return 0.0;
        const double m = mean();
        return std::sqrt(std::max(0.0, sum_sq / static_cast<double>(n) - m * m));
    }
    double max_abs() const
    {
        if (!n)
            return 0.0;
        return std::max(std::fabs(min), std::fabs(max));
    }
};

struct SegmentResult
{
    int param_id = 0;
    int repeat = 0;
    std::string direction;
    uint64_t samples = 0;
    uint64_t used_samples = 0;
    bool success = false;
    std::string failure_reason;
    double duration_s = 0.0;
    double distance_m = 0.0;
    double lateral_offset_m = 0.0;
    double lateral_offset_m_per_m = 0.0;
    double yaw_rms_rad = 0.0;
    double yaw_final_abs_rad = 0.0;
    double wheel_speed_imbalance = 0.0;
    double overshoot_rad = 0.0;
    double omega_rms_rad_s = 0.0;
    double score = std::numeric_limits<double>::infinity();
};

struct Evaluation
{
    int round = 0;
    std::string phase;
    int param_id = 0;
    TuneParams params;
    Stats score_stats;
    Stats yaw_rms_deg;
    Stats yaw_final_deg;
    Stats wheel_imbalance;
    int failures = 0;
    bool success = false;
    double aggregate_score = std::numeric_limits<double>::infinity();
};

std::mutex g_imu_mutex;
ImuSample g_latest_imu;

void on_signal(int)
{
    g_running.store(false);
}

int64_t now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

double clamp(double value, double lo, double hi)
{
    return std::min(hi, std::max(lo, value));
}

double normalize_angle(double value)
{
    while (value > kPi) value -= 2.0 * kPi;
    while (value < -kPi) value += 2.0 * kPi;
    return value;
}

double rad_to_deg(double value)
{
    return value * 180.0 / kPi;
}

std::string timestamp_string()
{
    const std::time_t t = std::time(nullptr);
    std::tm tm_now {};
    localtime_r(&t, &tm_now);
    char buf[32] {};
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_now);
    return std::string(buf);
}

bool path_is_dir(const std::string &path)
{
    struct stat st {};
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool ensure_dir(const std::string &path)
{
    if (path.empty())
        return false;
    if (path_is_dir(path))
        return true;
    return mkdir(path.c_str(), 0755) == 0 || path_is_dir(path);
}

bool ensure_dir_recursive(const std::string &path)
{
    if (path.empty())
        return false;
    if (path_is_dir(path))
        return true;
    std::string current;
    size_t start = 0;
    if (path[0] == '/')
    {
        current = "/";
        start = 1;
    }
    while (start <= path.size())
    {
        const size_t slash = path.find('/', start);
        const std::string part = path.substr(start, slash == std::string::npos ?
                                             std::string::npos : slash - start);
        if (!part.empty())
        {
            if (!current.empty() && current.back() != '/')
                current += "/";
            current += part;
            if (!ensure_dir(current))
                return false;
        }
        if (slash == std::string::npos)
            break;
        start = slash + 1;
    }
    return path_is_dir(path);
}

const char *cli_get(int argc, char **argv, const char *key, const char *fallback)
{
    const std::string k1 = std::string("--") + key;
    const std::string k2 = k1 + "=";
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == k1 && i + 1 < argc)
            return argv[i + 1];
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

Mode parse_mode(const std::string &value)
{
    if (value == "wheel-align")
        return Mode::WheelAlign;
    if (value == "imu-hold")
        return Mode::ImuHold;
    if (value == "straight-eval")
        return Mode::StraightEval;
    if (value == "online-tune")
        return Mode::OnlineTune;
    throw std::runtime_error("unknown --mode: " + value);
}

const char *mode_name(Mode mode)
{
    switch (mode)
    {
        case Mode::WheelAlign: return "wheel-align";
        case Mode::ImuHold: return "imu-hold";
        case Mode::StraightEval: return "straight-eval";
        case Mode::OnlineTune: return "online-tune";
    }
    return "unknown";
}

DirectionMode parse_direction(const std::string &value)
{
    if (value == "fwd")
        return DirectionMode::Forward;
    if (value == "back")
        return DirectionMode::Back;
    if (value == "fb")
        return DirectionMode::ForwardBack;
    throw std::runtime_error("unknown --direction: " + value);
}

std::vector<int> directions(DirectionMode mode)
{
    if (mode == DirectionMode::Forward)
        return {1};
    if (mode == DirectionMode::Back)
        return {-1};
    return {1, -1};
}

std::string direction_name(int sign)
{
    return sign >= 0 ? "fwd" : "back";
}

Options parse_options(int argc, char **argv)
{
    Options opt;
    opt.mode = parse_mode(cli_get(argc, argv, "mode", "wheel-align"));
    opt.ifname = cli_get(argc, argv, "ifname", opt.ifname.c_str());
    opt.slave_id = std::atoi(cli_get(argc, argv, "slave-id", "2"));
    opt.front_raise = std::strtof(cli_get(argc, argv, "front-raise", "-24.0"), nullptr);
    opt.rear_raise = std::strtof(cli_get(argc, argv, "rear-raise", "24.0"), nullptr);
    opt.imu_topic = cli_get(argc, argv, "imu-topic", opt.imu_topic.c_str());
    opt.direction = parse_direction(cli_get(argc, argv, "direction", "fb"));
    opt.speed = std::strtod(cli_get(argc, argv, "speed", "0.6"), nullptr);
    opt.distance = std::strtod(cli_get(argc, argv, "distance", "2.0"), nullptr);
    opt.linear_accel = std::strtod(cli_get(argc, argv, "linear-accel", "0.6"), nullptr);
    opt.linear_decel = std::strtod(cli_get(argc, argv, "linear-decel", "0.6"), nullptr);
    opt.yaw_kp = std::strtod(cli_get(argc, argv, "yaw-kp", "3.0"), nullptr);
    opt.yaw_ki = std::strtod(cli_get(argc, argv, "yaw-ki", "0.0"), nullptr);
    opt.yaw_kd = std::strtod(cli_get(argc, argv, "yaw-kd", "0.05"), nullptr);
    opt.yaw_out_limit = std::strtod(cli_get(argc, argv, "yaw-out-limit", "1.0"), nullptr);
    opt.drive_kp = std::strtod(cli_get(argc, argv, "drive-kp", "0.0"), nullptr);
    opt.drive_kd = std::strtod(cli_get(argc, argv, "drive-kd", "1.2"), nullptr);
    opt.front_left_corr = std::strtod(cli_get(argc, argv, "front-left-corr", "1.0"), nullptr);
    opt.front_right_corr = std::strtod(cli_get(argc, argv, "front-right-corr", "1.0"), nullptr);
    opt.rear_left_corr = std::strtod(cli_get(argc, argv, "rear-left-corr", "1.0"), nullptr);
    opt.rear_right_corr = std::strtod(cli_get(argc, argv, "rear-right-corr", "1.0"), nullptr);
    opt.repeats = std::atoi(cli_get(argc, argv, "repeats", "3"));
    opt.settle_s = std::strtod(cli_get(argc, argv, "settle", "1.0"), nullptr);
    opt.max_rounds = std::atoi(cli_get(argc, argv, "max-rounds", "30"));
    opt.sample_hz = std::atoi(cli_get(argc, argv, "sample-hz", "100"));
    opt.imu_wait_s = std::strtod(cli_get(argc, argv, "imu-wait", "1.0"), nullptr);
    opt.imu_stale_s = std::strtod(cli_get(argc, argv, "imu-stale", "0.25"), nullptr);
    opt.output_dir = cli_get(argc, argv, "output-dir", opt.output_dir.c_str());
    opt.yes = cli_has(argc, argv, "yes");
    opt.dry_run = cli_has(argc, argv, "dry-run");

    opt.speed = clamp(std::fabs(opt.speed), 0.02, kMaxForwardMps);
    opt.distance = clamp(opt.distance, 0.05, 20.0);
    opt.linear_accel = clamp(opt.linear_accel, 0.05, 5.0);
    opt.linear_decel = clamp(opt.linear_decel, 0.05, 5.0);
    opt.yaw_out_limit = clamp(std::fabs(opt.yaw_out_limit), 0.0, kMaxYawRadS);
    opt.repeats = std::max(1, std::min(100, opt.repeats));
    opt.max_rounds = std::max(1, std::min(500, opt.max_rounds));
    opt.sample_hz = std::max(10, std::min(500, opt.sample_hz));
    opt.slave_id = std::max(1, std::min(16, opt.slave_id));
    opt.settle_s = clamp(opt.settle_s, 0.0, 30.0);
    opt.imu_wait_s = clamp(opt.imu_wait_s, 0.0, 10.0);
    opt.imu_stale_s = clamp(opt.imu_stale_s, 0.02, 5.0);
    return opt;
}

double yaw_from_imu(const sensor_msgs::msg::Imu &msg)
{
    const double w = msg.orientation.w;
    const double x = msg.orientation.x;
    const double y = msg.orientation.y;
    const double z = msg.orientation.z;
    return normalize_angle(std::atan2(2.0 * (w * z + x * y),
                                      1.0 - 2.0 * (y * y + z * z)));
}

class ImuNode : public rclcpp::Node
{
public:
    explicit ImuNode(const std::string &topic)
        : Node("lift_raise_straight_line_imu_autotune")
    {
        sub_ = create_subscription<sensor_msgs::msg::Imu>(
            topic, rclcpp::SensorDataQoS(),
            [](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                ImuSample sample;
                sample.valid = true;
                sample.ns = now_ns();
                sample.yaw_rad = yaw_from_imu(*msg);
                sample.gyro_z = msg->angular_velocity.z;
                std::lock_guard<std::mutex> lock(g_imu_mutex);
                g_latest_imu = sample;
            });
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

class ImuRuntime
{
public:
    ~ImuRuntime() { stop(); }

    bool start(const Options &opt)
    {
        if (opt.dry_run)
            return true;
        ensure_dir_recursive("var_data/ros_log");
        if (std::getenv("ROS_LOG_DIR") == nullptr)
            setenv("ROS_LOG_DIR", "var_data/ros_log", 0);
        {
            std::lock_guard<std::mutex> lock(g_imu_mutex);
            g_latest_imu = ImuSample{};
        }
        if (!rclcpp::ok())
        {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
            own_rclcpp_ = true;
        }
        node_ = std::make_shared<ImuNode>(opt.imu_topic);
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        spin_thread_ = std::thread([this]() { executor_->spin(); });

        const int64_t start_ns = now_ns();
        while (g_running.load() && (now_ns() - start_ns) < static_cast<int64_t>(opt.imu_wait_s * 1e9))
        {
            if (latest().valid)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::cerr << "[lift_autotune] warning: no fresh IMU before start; yaw loop will stay open until data arrives\n";
        return true;
    }

    void stop()
    {
        if (executor_)
            executor_->cancel();
        if (spin_thread_.joinable())
            spin_thread_.join();
        if (executor_ && node_)
            executor_->remove_node(node_);
        executor_.reset();
        node_.reset();
        if (own_rclcpp_ && rclcpp::ok())
            rclcpp::shutdown();
    }

    static ImuSample latest()
    {
        std::lock_guard<std::mutex> lock(g_imu_mutex);
        return g_latest_imu;
    }

private:
    bool own_rclcpp_ = false;
    std::shared_ptr<ImuNode> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
};

void write_samples_header(std::ofstream &csv)
{
    csv << "mode,dir,param_id,repeat,t,forward_cmd,yaw_cmd,imu_yaw,yaw_err,"
        << "front_left_target,front_right_target,rear_left_target,rear_right_target,"
        << "front_left_now,front_right_now,rear_left_now,rear_right_now,"
        << "front_lift_motor,rear_lift_motor,lifts_reached,imu_age_ms,saturated,used_for_score\n";
}

void write_runs_header(std::ofstream &csv)
{
    csv << "mode,dir,param_id,repeat,samples,used_samples,success,failure_reason,"
        << "duration_s,distance_m,lateral_offset_m,lateral_offset_m_per_m,"
        << "yaw_rms_deg,yaw_final_abs_deg,wheel_speed_imbalance,overshoot_deg,omega_rms_rad_s,score\n";
}

void write_history_header(std::ofstream &csv)
{
    csv << "round,phase,param_id,aggregate_score,score_mean,score_std,segments,failures,"
        << "yaw_rms_deg_mean,yaw_final_deg_mean,wheel_imbalance_mean,"
        << "front_left_corr,front_right_corr,rear_left_corr,rear_right_corr,"
        << "yaw_kp,yaw_ki,yaw_kd,yaw_out_limit,drive_kp,drive_kd\n";
}

void write_run_row(std::ofstream &csv, Mode mode, const SegmentResult &r)
{
    csv << mode_name(mode) << ',' << r.direction << ',' << r.param_id << ',' << r.repeat << ','
        << r.samples << ',' << r.used_samples << ','
        << (r.success ? 1 : 0) << ',' << r.failure_reason << ','
        << r.duration_s << ',' << r.distance_m << ','
        << r.lateral_offset_m << ',' << r.lateral_offset_m_per_m << ','
        << rad_to_deg(r.yaw_rms_rad) << ',' << rad_to_deg(r.yaw_final_abs_rad) << ','
        << r.wheel_speed_imbalance << ',' << rad_to_deg(r.overshoot_rad) << ','
        << r.omega_rms_rad_s << ',' << r.score << '\n';
}

void write_history_row(std::ofstream &csv, const Evaluation &e)
{
    csv << e.round << ',' << e.phase << ',' << e.param_id << ','
        << e.aggregate_score << ',' << e.score_stats.mean() << ',' << e.score_stats.stddev() << ','
        << e.score_stats.n << ',' << e.failures << ','
        << e.yaw_rms_deg.mean() << ',' << e.yaw_final_deg.mean() << ','
        << e.wheel_imbalance.mean() << ','
        << e.params.front_left_corr << ',' << e.params.front_right_corr << ','
        << e.params.rear_left_corr << ',' << e.params.rear_right_corr << ','
        << e.params.yaw_kp << ',' << e.params.yaw_ki << ',' << e.params.yaw_kd << ','
        << e.params.yaw_out_limit << ',' << e.params.drive_kp << ',' << e.params.drive_kd << '\n';
}

bool configure_linkx_can()
{
    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&g_linkx, static_cast<uint8_t>(ch), true);
    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        if (!linkx_set_can_baudrate(&g_linkx,
                                    static_cast<uint8_t>(ch),
                                    1, 2, 31, 8, 8,
                                    1, 12, 3, 3))
        {
            std::cerr << "[lift_autotune] CAN" << ch << " FDCAN 1M/5M config failed\n";
            return false;
        }
    }
    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&g_linkx, static_cast<uint8_t>(ch), true);
    return true;
}

bool ecat_all_operational()
{
    if (!g_master.is_running)
        return false;
    ecx_readstate(&g_master.ctx);
    g_ecat_op =
        g_master.ctx.slavecount > 0 &&
        g_master.ctx.slavelist[0].state == EC_STATE_OPERATIONAL;
    return g_ecat_op;
}

void request_all_motor_enable();

bool init_hardware(const Options &opt)
{
    if (opt.dry_run)
        return true;
    if (!ecat_master_init(&g_master, opt.ifname.c_str()))
    {
        std::cerr << "[lift_autotune] ecat_master_init failed for " << opt.ifname << "\n";
        return false;
    }
    linkx_init(&g_linkx, static_cast<uint32_t>(opt.slave_id), &g_master.ctx);
    if (!configure_linkx_can())
        return false;
    if (!ecat_master_bring_online(&g_master))
    {
        std::cerr << "[lift_autotune] ecat_master_bring_online failed\n";
        return false;
    }
    g_lift.Init(&g_linkx);
    g_lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    g_lift.Send_Enable_Burst();
    request_all_motor_enable();
    return true;
}

bool lift_motors_enabled()
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        const auto module = static_cast<Enum_Chariot_Lift_Module>(i);
        if (g_lift.Motor_Lift[module].Get_Status() != Motor_DM_Status_ENABLE ||
            g_lift.Motor_Lift[module].Get_Now_Control_Status() != Motor_DM_Control_Status_ENABLE)
            return false;
    }
    return true;
}

bool drive_motors_enabled()
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        const auto module = static_cast<Enum_Chariot_Lift_Module>(i);
        if (g_lift.Motor_Drive_Left[module].Get_Status() != Motor_DM_Status_ENABLE ||
            g_lift.Motor_Drive_Left[module].Get_Now_Control_Status() != Motor_DM_Control_Status_ENABLE ||
            g_lift.Motor_Drive_Right[module].Get_Status() != Motor_DM_Status_ENABLE ||
            g_lift.Motor_Drive_Right[module].Get_Now_Control_Status() != Motor_DM_Control_Status_ENABLE)
            return false;
    }
    return true;
}

void request_motor_enable(Class_Motor_DM_Normal &motor)
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

void request_all_motor_enable()
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        const auto module = static_cast<Enum_Chariot_Lift_Module>(i);
        request_motor_enable(g_lift.Motor_Lift[module]);
        request_motor_enable(g_lift.Motor_Drive_Left[module]);
        request_motor_enable(g_lift.Motor_Drive_Right[module]);
    }
}

void print_ready_status()
{
    std::cerr << "[lift_autotune] ready status:"
              << " wkc=" << g_last_wkc
              << " expected=" << g_master.expected_wkc
              << " op=" << (g_ecat_op ? 1 : 0)
              << " good_ms=" << g_ecat_good_ms
              << " bad_ms=" << g_ecat_bad_ms << "\n";
    if (g_master.is_running)
    {
        ecx_readstate(&g_master.ctx);
        for (int i = 1; i <= g_master.ctx.slavecount; ++i)
        {
            std::cerr << "  slave=" << i
                      << " state=0x" << std::hex << g_master.ctx.slavelist[i].state
                      << " al=0x" << g_master.ctx.slavelist[i].ALstatuscode
                      << std::dec << "\n";
        }
    }
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        const auto module = static_cast<Enum_Chariot_Lift_Module>(i);
        std::cerr << "  module=" << (i == 0 ? "front" : "rear")
                  << " lift_status=" << static_cast<int>(g_lift.Motor_Lift[module].Get_Status())
                  << " lift_ctrl=" << static_cast<int>(g_lift.Motor_Lift[module].Get_Now_Control_Status())
                  << " left_status=" << static_cast<int>(g_lift.Motor_Drive_Left[module].Get_Status())
                  << " left_ctrl=" << static_cast<int>(g_lift.Motor_Drive_Left[module].Get_Now_Control_Status())
                  << " right_status=" << static_cast<int>(g_lift.Motor_Drive_Right[module].Get_Status())
                  << " right_ctrl=" << static_cast<int>(g_lift.Motor_Drive_Right[module].Get_Now_Control_Status())
                  << "\n";
    }
}

bool ec_step(uint32_t tick, const Options &opt)
{
    if (opt.dry_run)
        return true;
    if (!g_running.load() || !g_master.is_running)
        return false;

    g_last_wkc = static_cast<uint32_t>(std::max(0, ecat_master_sync(&g_master)));
    if ((tick % 50U) == 0U)
        (void)ecat_all_operational();
    if (g_master.expected_wkc > 0 && g_last_wkc >= static_cast<uint32_t>(g_master.expected_wkc) && g_ecat_op)
    {
        g_ecat_good_ms += kEcPeriodMs;
        g_ecat_bad_ms = 0;
    }
    else
    {
        g_ecat_good_ms = 0;
        g_ecat_bad_ms += kEcPeriodMs;
    }

    linkx_recv_pdos(&g_linkx);
    can_msg_t msg {};
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
    {
        while (linkx_quick_recv(&g_linkx, ch, &msg))
            g_lift.CAN_Rx_Callback(ch, msg.id, msg.data);
    }

    if ((tick % kAlivePeriodTicks) == 0U)
    {
        g_lift.TIM_100ms_Alive_PeriodElapsedCallback();
        g_lift.Send_Enable_Burst();
        request_all_motor_enable();
    }
    if ((tick % kControlPeriodTicks) == 0U)
        g_lift.TIM_2ms_Control_PeriodElapsedCallback();

    linkx_send_pdos(&g_linkx);
    return true;
}

void run_for_ms(uint32_t &tick, const Options &opt, uint32_t ms)
{
    auto next = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < ms && g_running.load(); ++i)
    {
        next += std::chrono::milliseconds(1);
        ec_step(tick++, opt);
        if (!opt.dry_run)
            std::this_thread::sleep_until(next);
    }
}

bool wait_hardware_ready(uint32_t &tick, const Options &opt)
{
    if (opt.dry_run)
        return true;
    const uint32_t timeout_ms = 4000;
    for (uint32_t ms = 0; ms < timeout_ms && g_running.load(); ++ms)
    {
        ec_step(tick++, opt);
        if ((ms % 250U) == 0U && !ecat_all_operational())
            (void)ecat_master_bring_online(&g_master);
        if ((ms % 100U) == 0U)
            request_all_motor_enable();
        if (g_ecat_good_ms >= kEcatStableRequiredMs && lift_motors_enabled())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    print_ready_status();
    return g_ecat_good_ms >= kEcatStableRequiredMs && lift_motors_enabled();
}

bool wait_drive_motors_ready(uint32_t &tick, const Options &opt, uint32_t timeout_ms)
{
    if (opt.dry_run)
        return true;
    for (uint32_t ms = 0; ms < timeout_ms && g_running.load(); ++ms)
    {
        ec_step(tick++, opt);
        if ((ms % 100U) == 0U)
            request_all_motor_enable();
        if (drive_motors_enabled())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    print_ready_status();
    return drive_motors_enabled();
}

void apply_params_to_lift(const TuneParams &p)
{
    auto front = g_lift.Get_Diff_Drive_Params(CHARIOT_LIFT_MODULE_FRONT);
    auto rear = g_lift.Get_Diff_Drive_Params(CHARIOT_LIFT_MODULE_REAR);
    front.left_speed_correction = static_cast<float>(p.front_left_corr);
    front.right_speed_correction = static_cast<float>(p.front_right_corr);
    front.wheel_kp = static_cast<float>(p.drive_kp);
    front.wheel_kd = static_cast<float>(p.drive_kd);
    rear.left_speed_correction = static_cast<float>(p.rear_left_corr);
    rear.right_speed_correction = static_cast<float>(p.rear_right_corr);
    rear.wheel_kp = static_cast<float>(p.drive_kp);
    rear.wheel_kd = static_cast<float>(p.drive_kd);
    g_lift.Set_Diff_Drive_Params(CHARIOT_LIFT_MODULE_FRONT, front);
    g_lift.Set_Diff_Drive_Params(CHARIOT_LIFT_MODULE_REAR, rear);
}

bool raise_and_wait(uint32_t &tick, const Options &opt)
{
    if (opt.dry_run)
        return true;
    g_lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    g_lift.Set_Both_Lift_Raise(opt.front_raise, opt.rear_raise);
    const uint32_t timeout_ms = static_cast<uint32_t>(opt.lift_timeout_s * 1000.0);
    uint32_t stable_ms = 0;
    for (uint32_t ms = 0; ms < timeout_ms && g_running.load(); ++ms)
    {
        ec_step(tick++, opt);
        const bool reached = g_lift.Are_Both_Lifts_Reached(CHARIOT_LIFT_POSITION_RAISE);
        stable_ms = reached ? stable_ms + 1 : 0;
        if (stable_ms >= 300)
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

void stop_drive(uint32_t &tick, const Options &opt)
{
    g_lift.Set_Target_Diff_Command_For_Module(CHARIOT_LIFT_MODULE_FRONT, 0.0f, 0.0f);
    g_lift.Set_Target_Diff_Command_For_Module(CHARIOT_LIFT_MODULE_REAR, 0.0f, 0.0f);
    g_lift.Set_Diff_Drive_Enable(true);
    run_for_ms(tick, opt, 350);
    g_lift.Set_Diff_Drive_Enable(false);
    g_lift.Clear_Target_Diff_Command_For_Modules();
}

void shutdown_hardware(uint32_t &tick, const Options &opt)
{
    if (opt.dry_run)
        return;
    stop_drive(tick, opt);
    g_lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_DISABLE);
    run_for_ms(tick, opt, 200);
    for (int i = 0; i < 30; ++i)
    {
        g_lift.TIM_2ms_Control_PeriodElapsedCallback();
        linkx_send_pdos(&g_linkx);
        ecat_master_sync(&g_master);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

double profile_velocity(double traveled, double distance, double vmax, double accel, double decel)
{
    const double up = std::sqrt(std::max(0.0, 2.0 * accel * traveled));
    const double down = std::sqrt(std::max(0.0, 2.0 * decel * std::max(0.0, distance - traveled)));
    return std::min(vmax, std::min(up, down));
}

std::array<double, 4> actual_wheel_omega(const Options &opt,
                                         const TuneParams &p,
                                         double forward,
                                         double yaw_cmd)
{
    if (!opt.dry_run)
    {
        return {
            g_lift.Motor_Drive_Left[CHARIOT_LIFT_MODULE_FRONT].Get_Now_Omega(),
            g_lift.Motor_Drive_Right[CHARIOT_LIFT_MODULE_FRONT].Get_Now_Omega(),
            g_lift.Motor_Drive_Left[CHARIOT_LIFT_MODULE_REAR].Get_Now_Omega(),
            g_lift.Motor_Drive_Right[CHARIOT_LIFT_MODULE_REAR].Get_Now_Omega(),
        };
    }

    const double base = -forward / kWheelRadiusM;
    const double yaw = yaw_cmd * kTrackWidthM * 0.5 / kWheelRadiusM;
    const std::array<double, 4> hidden_bias = {0.94, 1.06, 0.98, 1.03};
    return {
        (base - yaw) * p.front_left_corr * hidden_bias[0],
        -(base + yaw) * p.front_right_corr * hidden_bias[1],
        (base + yaw) * p.rear_left_corr * hidden_bias[2],
        -(base - yaw) * p.rear_right_corr * hidden_bias[3],
    };
}

double wheel_imbalance_from_omega(const std::array<double, 4> &omega)
{
    Stats speeds;
    for (double w : omega)
        speeds.add(std::fabs(w * kWheelRadiusM));
    const double mean = std::max(0.02, speeds.mean());
    return speeds.stddev() / mean;
}

SegmentResult run_segment(const Options &opt,
                          const TuneParams &params,
                          Mode mode,
                          int param_id,
                          int repeat,
                          int direction,
                          uint32_t &tick,
                          std::ofstream &samples_csv)
{
    SegmentResult result;
    result.param_id = param_id;
    result.repeat = repeat;
    result.direction = direction_name(direction);

    if (!raise_and_wait(tick, opt))
    {
        result.failure_reason = "lift_not_reached";
        return result;
    }

    const bool yaw_closed_loop = mode != Mode::WheelAlign;
    const double dt = 1.0 / static_cast<double>(opt.sample_hz);
    const int sample_ms = std::max(1, static_cast<int>(std::round(1000.0 * dt)));
    const double hard_timeout = std::max(3.0, opt.distance / std::max(0.05, opt.speed) + 8.0);
    double traveled = 0.0;
    double lateral = 0.0;
    double t = 0.0;
    double yaw_integral = 0.0;
    double prev_yaw_err = 0.0;
    bool have_prev_yaw_err = false;
    bool saturated = false;
    double sim_yaw = 0.0;

    ImuSample imu0 = ImuRuntime::latest();
    const double yaw_target = imu0.valid ? imu0.yaw_rad : 0.0;
    double final_yaw_err = 0.0;
    Stats yaw_err_stat;
    Stats yaw_cmd_stat;
    Stats wheel_imbalance_stat;
    Stats overshoot_stat;

    g_lift.Set_Diff_Drive_Enable(true);
    if (!wait_drive_motors_ready(tick, opt, 2000))
    {
        result.failure_reason = "drive_not_enabled";
        stop_drive(tick, opt);
        return result;
    }

    while (g_running.load() && traveled < opt.distance && t < hard_timeout)
    {
        double speed_abs = profile_velocity(traveled, opt.distance, opt.speed,
                                            opt.linear_accel, opt.linear_decel);
        if (speed_abs < 1e-4 && traveled < opt.distance)
            speed_abs = std::min(opt.speed, std::max(0.01, opt.linear_accel * dt));
        const double forward = static_cast<double>(direction) * speed_abs;

        ImuSample imu = ImuRuntime::latest();
        const double imu_age_ms = imu.valid ? (now_ns() - imu.ns) / 1.0e6 :
                                  std::numeric_limits<double>::infinity();
        if (opt.dry_run)
        {
            imu.valid = true;
            imu.yaw_rad = sim_yaw;
            imu.gyro_z = 0.0;
        }

        const bool imu_fresh = imu.valid && (opt.dry_run || imu_age_ms <= opt.imu_stale_s * 1000.0);
        double yaw_err = 0.0;
        double yaw_cmd = 0.0;
        if (yaw_closed_loop && imu_fresh)
        {
            yaw_err = normalize_angle(yaw_target - imu.yaw_rad);
            yaw_integral = clamp(yaw_integral + yaw_err * dt, -1.0, 1.0);
            const double derivative = have_prev_yaw_err ? (yaw_err - prev_yaw_err) / dt : 0.0;
            yaw_cmd = params.yaw_kp * yaw_err + params.yaw_ki * yaw_integral + params.yaw_kd * derivative;
            yaw_cmd = clamp(yaw_cmd, -params.yaw_out_limit, params.yaw_out_limit);
            saturated = saturated || std::fabs(yaw_cmd) >= params.yaw_out_limit - 1e-6;
            prev_yaw_err = yaw_err;
            have_prev_yaw_err = true;
        }
        else if (yaw_closed_loop && !imu_fresh)
        {
            yaw_cmd = 0.0;
        }

        g_lift.Set_Target_Diff_Command_For_Module(CHARIOT_LIFT_MODULE_FRONT,
                                                  static_cast<float>(forward),
                                                  static_cast<float>(yaw_cmd));
        g_lift.Set_Target_Diff_Command_For_Module(CHARIOT_LIFT_MODULE_REAR,
                                                  static_cast<float>(forward),
                                                  static_cast<float>(-yaw_cmd));

        if (opt.dry_run)
        {
            const double imbalance_yaw = 0.20 * ((params.front_right_corr + params.rear_right_corr) -
                                                 (params.front_left_corr + params.rear_left_corr));
            sim_yaw = normalize_angle(sim_yaw + (imbalance_yaw - 0.75 * yaw_cmd) * dt);
        }
        else
        {
            run_for_ms(tick, opt, static_cast<uint32_t>(sample_ms));
        }

        const auto wheel_now = actual_wheel_omega(opt, params, forward, yaw_cmd);
        const double imbalance = wheel_imbalance_from_omega(wheel_now);
        const double yaw_for_score = opt.dry_run ? sim_yaw : imu.yaw_rad;
        yaw_err = yaw_closed_loop ? normalize_angle(yaw_target - yaw_for_score) :
                  normalize_angle(yaw_for_score - yaw_target);
        final_yaw_err = yaw_err;
        const bool lifts_reached = opt.dry_run ||
            g_lift.Are_Both_Lifts_Reached(CHARIOT_LIFT_POSITION_RAISE);
        const bool used = lifts_reached && (!yaw_closed_loop || imu_fresh);
        ++result.samples;
        if (used)
        {
            ++result.used_samples;
            yaw_err_stat.add(yaw_err);
            yaw_cmd_stat.add(yaw_cmd);
            wheel_imbalance_stat.add(imbalance);
            overshoot_stat.add(std::max(0.0, std::fabs(yaw_err) - std::fabs(prev_yaw_err)));
        }

        lateral += std::sin(yaw_err) * std::fabs(forward) * dt;
        traveled += std::fabs(forward) * dt;
        t += dt;

        const double fl_target = g_lift.Get_Target_Left_Omega(CHARIOT_LIFT_MODULE_FRONT);
        const double fr_target = g_lift.Get_Target_Right_Omega(CHARIOT_LIFT_MODULE_FRONT);
        const double rl_target = g_lift.Get_Target_Left_Omega(CHARIOT_LIFT_MODULE_REAR);
        const double rr_target = g_lift.Get_Target_Right_Omega(CHARIOT_LIFT_MODULE_REAR);
        const double front_lift = opt.dry_run ? opt.front_raise :
            g_lift.Motor_Lift[CHARIOT_LIFT_MODULE_FRONT].Get_Now_Radian();
        const double rear_lift = opt.dry_run ? opt.rear_raise :
            g_lift.Motor_Lift[CHARIOT_LIFT_MODULE_REAR].Get_Now_Radian();

        samples_csv << mode_name(mode) << ',' << result.direction << ',' << param_id << ','
                    << repeat << ',' << t << ',' << forward << ',' << yaw_cmd << ','
                    << yaw_for_score << ',' << yaw_err << ','
                    << fl_target << ',' << fr_target << ',' << rl_target << ',' << rr_target << ','
                    << wheel_now[0] << ',' << wheel_now[1] << ',' << wheel_now[2] << ',' << wheel_now[3] << ','
                    << front_lift << ',' << rear_lift << ',' << (lifts_reached ? 1 : 0) << ','
                    << imu_age_ms << ',' << (saturated ? 1 : 0) << ',' << (used ? 1 : 0) << '\n';

        if (!lifts_reached)
        {
            result.failure_reason = "lift_lost_reached";
            break;
        }
        if (yaw_closed_loop && !opt.dry_run && !imu_fresh)
        {
            result.failure_reason = "imu_stale";
            break;
        }
        if (std::fabs(yaw_err) > 35.0 * kPi / 180.0)
        {
            result.failure_reason = "yaw_error_too_large";
            break;
        }
    }

    stop_drive(tick, opt);
    if (opt.settle_s > 0.0)
        run_for_ms(tick, opt, static_cast<uint32_t>(opt.settle_s * 1000.0));

    result.duration_s = t;
    result.distance_m = traveled;
    result.lateral_offset_m = lateral;
    result.lateral_offset_m_per_m = (traveled > 1e-6) ? std::fabs(lateral) / traveled : 0.0;
    result.yaw_rms_rad = yaw_err_stat.rms();
    result.yaw_final_abs_rad = std::fabs(final_yaw_err);
    result.wheel_speed_imbalance = wheel_imbalance_stat.mean();
    result.overshoot_rad = overshoot_stat.max_abs();
    result.omega_rms_rad_s = yaw_cmd_stat.rms();
    if (result.failure_reason.empty() && result.used_samples == 0)
        result.failure_reason = "no_scored_samples";
    if (result.failure_reason.empty() && t >= hard_timeout)
        result.failure_reason = "timeout";

    if (result.failure_reason.empty())
    {
        result.score = 50.0 * result.yaw_final_abs_rad +
                       30.0 * result.yaw_rms_rad +
                       20.0 * result.wheel_speed_imbalance +
                       10.0 * result.lateral_offset_m_per_m +
                       5.0 * result.overshoot_rad +
                       5.0 * result.omega_rms_rad_s;
        if (saturated)
            result.score += 2.0;
        result.success = std::isfinite(result.score);
    }
    return result;
}

Evaluation evaluate_params(const Options &opt,
                           const TuneParams &params,
                           Mode mode,
                           int round,
                           const std::string &phase,
                           int param_id,
                           uint32_t &tick,
                           std::ofstream &samples_csv,
                           std::ofstream &runs_csv)
{
    Evaluation eval;
    eval.round = round;
    eval.phase = phase;
    eval.param_id = param_id;
    eval.params = params;
    apply_params_to_lift(params);
    for (int repeat = 1; repeat <= opt.repeats && g_running.load(); ++repeat)
    {
        for (int dir : directions(opt.direction))
        {
            SegmentResult r = run_segment(opt, params, mode, param_id, repeat, dir, tick, samples_csv);
            write_run_row(runs_csv, mode, r);
            runs_csv.flush();
            samples_csv.flush();
            if (r.success && std::isfinite(r.score))
            {
                eval.score_stats.add(r.score);
                eval.yaw_rms_deg.add(rad_to_deg(r.yaw_rms_rad));
                eval.yaw_final_deg.add(rad_to_deg(r.yaw_final_abs_rad));
                eval.wheel_imbalance.add(r.wheel_speed_imbalance);
            }
            else
            {
                ++eval.failures;
            }
        }
    }
    if (eval.score_stats.n > 0)
    {
        eval.aggregate_score = eval.score_stats.mean() + 0.5 * eval.score_stats.stddev();
        eval.success = std::isfinite(eval.aggregate_score);
    }
    return eval;
}

TuneParams params_from_options(const Options &opt)
{
    TuneParams p;
    p.front_left_corr = opt.front_left_corr;
    p.front_right_corr = opt.front_right_corr;
    p.rear_left_corr = opt.rear_left_corr;
    p.rear_right_corr = opt.rear_right_corr;
    p.yaw_kp = opt.yaw_kp;
    p.yaw_ki = opt.yaw_ki;
    p.yaw_kd = opt.yaw_kd;
    p.yaw_out_limit = opt.yaw_out_limit;
    p.drive_kp = opt.drive_kp;
    p.drive_kd = opt.drive_kd;
    return p;
}

void append_unique(std::vector<TuneParams> &list, const TuneParams &p)
{
    auto same = [](double a, double b) { return std::fabs(a - b) < 1e-9; };
    for (const auto &x : list)
    {
        if (same(x.front_left_corr, p.front_left_corr) &&
            same(x.front_right_corr, p.front_right_corr) &&
            same(x.rear_left_corr, p.rear_left_corr) &&
            same(x.rear_right_corr, p.rear_right_corr) &&
            same(x.yaw_kp, p.yaw_kp) &&
            same(x.yaw_kd, p.yaw_kd) &&
            same(x.drive_kp, p.drive_kp) &&
            same(x.drive_kd, p.drive_kd))
            return;
    }
    list.push_back(p);
}

std::vector<TuneParams> build_candidates(const Options &opt, const TuneParams &base)
{
    std::vector<TuneParams> candidates;
    if (opt.mode != Mode::OnlineTune)
    {
        candidates.push_back(base);
        return candidates;
    }

    const std::array<double, 3> corr_scale = {0.9, 1.0, 1.1};
    int added = 0;
    for (double fl : corr_scale)
    {
        for (double fr : corr_scale)
        {
            TuneParams p = base;
            p.front_left_corr = clamp(base.front_left_corr * fl, 0.5, 1.5);
            p.front_right_corr = clamp(base.front_right_corr * fr, 0.5, 1.5);
            p.rear_left_corr = clamp(base.rear_left_corr * fl, 0.5, 1.5);
            p.rear_right_corr = clamp(base.rear_right_corr * fr, 0.5, 1.5);
            append_unique(candidates, p);
            if (++added >= 9)
                break;
        }
        if (added >= 9)
            break;
    }

    const std::array<double, 3> kp_list = {2.0, 3.0, 5.0};
    const std::array<double, 3> kd_list = {0.0, 0.05, 0.1};
    for (double kp : kp_list)
    {
        for (double kd : kd_list)
        {
            TuneParams p = base;
            p.yaw_kp = kp;
            p.yaw_kd = kd;
            append_unique(candidates, p);
        }
    }
    return candidates;
}

std::vector<TuneParams> local_candidates(const TuneParams &best)
{
    std::vector<TuneParams> out;
    const std::array<double TuneParams::*, 4> corr_fields = {
        &TuneParams::front_left_corr,
        &TuneParams::front_right_corr,
        &TuneParams::rear_left_corr,
        &TuneParams::rear_right_corr,
    };
    for (auto field : corr_fields)
    {
        for (double scale : {0.9, 1.1})
        {
            TuneParams p = best;
            p.*field = clamp(p.*field * scale, 0.5, 1.5);
            append_unique(out, p);
        }
    }
    for (double scale : {0.8, 1.2})
    {
        TuneParams p = best;
        p.yaw_kp = clamp(p.yaw_kp * scale, 0.0, 20.0);
        append_unique(out, p);
    }
    for (double delta : {-0.03, 0.03})
    {
        TuneParams p = best;
        p.yaw_kd = clamp(p.yaw_kd + delta, 0.0, 1.0);
        append_unique(out, p);
    }
    return out;
}

void write_yaml(const std::string &path, const Options &opt, const TuneParams &p)
{
    std::ofstream out(path);
    out << std::fixed << std::setprecision(6);
    out << "lift_raise_straight_line_imu_autotune:\n";
    out << "  ifname: " << opt.ifname << "\n";
    out << "  slave_id: " << opt.slave_id << "\n";
    out << "  front_raise_motor_angle: " << opt.front_raise << "\n";
    out << "  rear_raise_motor_angle: " << opt.rear_raise << "\n";
    out << "  front_left_speed_correction: " << p.front_left_corr << "\n";
    out << "  front_right_speed_correction: " << p.front_right_corr << "\n";
    out << "  rear_left_speed_correction: " << p.rear_left_corr << "\n";
    out << "  rear_right_speed_correction: " << p.rear_right_corr << "\n";
    out << "  yaw_kp: " << p.yaw_kp << "\n";
    out << "  yaw_ki: " << p.yaw_ki << "\n";
    out << "  yaw_kd: " << p.yaw_kd << "\n";
    out << "  yaw_out_limit_rad_s: " << p.yaw_out_limit << "\n";
    out << "  drive_kp: " << p.drive_kp << "\n";
    out << "  drive_kd: " << p.drive_kd << "\n";
    out << "  note: front module uses +yaw correction; rear module uses -yaw correction.\n";
}

void write_report(const std::string &path,
                  const Options &opt,
                  const Evaluation *best,
                  const std::vector<Evaluation> &history,
                  const std::string &samples,
                  const std::string &runs,
                  const std::string &hist,
                  const std::string &yaml)
{
    std::ofstream out(path);
    out << "# Lift Raise Straight-Line IMU Autotune Report\n\n";
    out << "- mode: " << mode_name(opt.mode) << "\n";
    out << "- dry_run: " << (opt.dry_run ? 1 : 0) << "\n";
    out << "- lift target: front " << opt.front_raise << " rad, rear " << opt.rear_raise << " rad\n";
    out << "- distance: " << opt.distance << " m, speed: " << opt.speed << " m/s\n";
    out << "- score: 50*yaw_final + 30*yaw_rms + 20*wheel_imbalance + 10*lateral_per_m + 5*overshoot + 5*omega_rms\n";
    out << "- samples_csv: " << samples << "\n";
    out << "- runs_csv: " << runs << "\n";
    out << "- search_history_csv: " << hist << "\n";
    out << "- best_params_yaml: " << yaml << "\n\n";
    if (best)
    {
        out << "## Best Parameters\n\n";
        out << "- aggregate_score: " << best->aggregate_score << "\n";
        out << "- yaw_rms_deg_mean: " << best->yaw_rms_deg.mean() << "\n";
        out << "- yaw_final_deg_mean: " << best->yaw_final_deg.mean() << "\n";
        out << "- wheel_imbalance_mean: " << best->wheel_imbalance.mean() << "\n";
        out << "- front_left/right_corr: " << best->params.front_left_corr << " / "
            << best->params.front_right_corr << "\n";
        out << "- rear_left/right_corr: " << best->params.rear_left_corr << " / "
            << best->params.rear_right_corr << "\n";
        out << "- yaw_pid: kp=" << best->params.yaw_kp
            << " ki=" << best->params.yaw_ki
            << " kd=" << best->params.yaw_kd
            << " out_limit=" << best->params.yaw_out_limit << "\n\n";
    }
    out << "## Safety And Sign Notes\n\n";
    out << "- This test locks current IMU yaw at segment start; it does not use the stair-mode fixed -90 deg target.\n";
    out << "- Yaw correction is injected as front +yaw and rear -yaw because the front/rear lift modules face opposite physical directions.\n";
    out << "- Lift reach is checked before and during every segment; loss of reach stops scoring and stops drive.\n";
    out << "- IMU stale disables/aborts closed-loop yaw instead of reusing old yaw.\n";
    out << "- Manual recheck: after applying best params, raise the lift and drive one short forward and one short back segment at low speed before increasing distance.\n\n";
    out << "## History\n\n";
    out << "round,phase,param_id,aggregate_score,segments,failures\n";
    for (const auto &e : history)
        out << e.round << ',' << e.phase << ',' << e.param_id << ','
            << e.aggregate_score << ',' << e.score_stats.n << ',' << e.failures << "\n";
}

void print_usage(const char *argv0)
{
    std::cout << "Usage: " << argv0 << " [options]\n"
              << "  --mode wheel-align|imu-hold|straight-eval|online-tune\n"
              << "  --ifname enp4s0 --slave-id 2 --direction fb|fwd|back --speed 0.6 --distance 2.0\n"
              << "  --front-raise -24 --rear-raise 24 --imu-topic /IMU_data\n"
              << "  --yaw-kp 3 --yaw-ki 0 --yaw-kd 0.05 --yaw-out-limit 1.0\n"
              << "  --front-left-corr 1 --front-right-corr 1 --rear-left-corr 1 --rear-right-corr 1\n"
              << "  --repeats 3 --max-rounds 30 --output-dir var_data/lift_straight --yes --dry-run\n";
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

    Options opt;
    try
    {
        opt = parse_options(argc, argv);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[lift_autotune] " << e.what() << "\n";
        print_usage(argv[0]);
        return 2;
    }

    if (!opt.yes && !opt.dry_run)
    {
        std::cout << "[SAFETY] Need clear floor in front/back: at least distance + 1.5 m each way.\n"
                  << "[SAFETY] First real run should use low speed and short distance to verify wheel and yaw signs.\n"
                  << "Type YES to continue: ";
        std::string answer;
        std::getline(std::cin, answer);
        if (answer != "YES")
        {
            std::cerr << "[lift_autotune] cancelled by user\n";
            return 1;
        }
    }

    const std::string stamp = timestamp_string();
    const std::string run_dir = opt.output_dir + "/" + stamp;
    if (!ensure_dir_recursive(run_dir))
    {
        std::cerr << "[lift_autotune] failed to create output dir: " << run_dir << "\n";
        return 3;
    }

    const std::string samples_path = run_dir + "/samples_" + stamp + ".csv";
    const std::string runs_path = run_dir + "/runs_" + stamp + ".csv";
    const std::string history_path = run_dir + "/search_history_" + stamp + ".csv";
    const std::string yaml_path = run_dir + "/best_params_" + stamp + ".yaml";
    const std::string report_path = run_dir + "/report_" + stamp + ".md";

    std::ofstream samples_csv(samples_path);
    std::ofstream runs_csv(runs_path);
    std::ofstream history_csv(history_path);
    if (!samples_csv || !runs_csv || !history_csv)
    {
        std::cerr << "[lift_autotune] failed to open output CSV files\n";
        return 3;
    }
    samples_csv << std::fixed << std::setprecision(9);
    runs_csv << std::fixed << std::setprecision(9);
    history_csv << std::fixed << std::setprecision(9);
    write_samples_header(samples_csv);
    write_runs_header(runs_csv);
    write_history_header(history_csv);

    std::cout << "[lift_autotune] mode=" << mode_name(opt.mode)
              << " slave_id=" << opt.slave_id
              << " dry_run=" << (opt.dry_run ? 1 : 0)
              << " output=" << run_dir << "\n";

    ImuRuntime imu;
    if (!imu.start(opt))
        return 4;
    if (!init_hardware(opt))
        return 5;

    uint32_t tick = 0;
    if (!wait_hardware_ready(tick, opt))
    {
        std::cerr << "[lift_autotune] hardware did not become ready; refusing to move\n";
        shutdown_hardware(tick, opt);
        return 6;
    }

    TuneParams base = params_from_options(opt);
    std::vector<Evaluation> history;
    Evaluation best;
    bool have_best = false;
    int param_id = 0;

    auto consider = [&](Evaluation &&eval)
    {
        write_history_row(history_csv, eval);
        history_csv.flush();
        const bool better = eval.success &&
            (!have_best || eval.aggregate_score < best.aggregate_score);
        if (better)
        {
            best = eval;
            have_best = true;
            write_yaml(yaml_path, opt, best.params);
            std::cout << "[lift_autotune] new best param_id=" << best.param_id
                      << " score=" << best.aggregate_score << "\n";
        }
        history.push_back(eval);
    };

    if (opt.mode == Mode::OnlineTune)
    {
        const std::vector<TuneParams> coarse = build_candidates(opt, base);
        int round = 0;
        for (const TuneParams &p : coarse)
        {
            if (!g_running.load() || round >= opt.max_rounds)
                break;
            ++round;
            consider(evaluate_params(opt, p, Mode::WheelAlign, round, "coarse-wheel-align",
                                     ++param_id, tick, samples_csv, runs_csv));
        }
        while (g_running.load() && round < opt.max_rounds && have_best)
        {
            bool improved = false;
            const double before = best.aggregate_score;
            const std::vector<TuneParams> locals = local_candidates(best.params);
            for (const TuneParams &p : locals)
            {
                if (!g_running.load() || round >= opt.max_rounds)
                    break;
                ++round;
                consider(evaluate_params(opt, p, Mode::ImuHold, round, "local-refine",
                                         ++param_id, tick, samples_csv, runs_csv));
                improved = have_best && best.aggregate_score + 1e-4 < before;
            }
            if (!improved)
                break;
        }
    }
    else
    {
        consider(evaluate_params(opt, base, opt.mode, 1, mode_name(opt.mode),
                                 ++param_id, tick, samples_csv, runs_csv));
    }

    if (!have_best)
        write_yaml(yaml_path, opt, base);
    write_report(report_path, opt, have_best ? &best : nullptr, history,
                 samples_path, runs_path, history_path, yaml_path);

    shutdown_hardware(tick, opt);
    imu.stop();

    std::cout << "[lift_autotune] samples=" << samples_path << "\n"
              << "[lift_autotune] runs=" << runs_path << "\n"
              << "[lift_autotune] history=" << history_path << "\n"
              << "[lift_autotune] best_params=" << yaml_path << "\n"
              << "[lift_autotune] report=" << report_path << "\n";
    return have_best || opt.dry_run ? 0 : 7;
}

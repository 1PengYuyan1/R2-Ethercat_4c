// Remote-like chassis acceleration/yaw tuning test.
//
// This tool reproduces the bad hand-feel cases from remote driving:
//   1) straight acceleration/stop yaw kick and residual path offset,
//   2) heading-hold parameter sensitivity while IMU runs at control rate,
//   3) manual turn response lag.
//
// It publishes shaped velocity commands, records IMU + chassis odom, scores each
// parameter set, and writes CSV reports with the best observed combination.
//
// Typical usage while vehicle_control is running and chassis is enabled:
//   ros2 run linkx_soem_demo remote_chassis_yaw_tune
//   ros2 run linkx_soem_demo remote_chassis_yaw_tune --cmd-topic /chassis/cmd_vel --max-combos 6

#include <algorithm>
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
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;
std::atomic<bool> g_running {true};

void on_signal(int)
{
    g_running.store(false);
}

int64_t now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

double normalize_angle(double angle)
{
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

double rad_to_deg(double value)
{
    return value * 180.0 / kPi;
}

double deg_s(double rad_s)
{
    return rad_to_deg(rad_s);
}

double yaw_from_imu(const sensor_msgs::msg::Imu &msg)
{
    const double w = msg.orientation.w;
    const double x = msg.orientation.x;
    const double y = msg.orientation.y;
    const double z = msg.orientation.z;
    return std::atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z));
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

bool ensure_dir(const std::string &path)
{
    struct stat st {};
    if (stat(path.c_str(), &st) == 0)
        return S_ISDIR(st.st_mode);
    return mkdir(path.c_str(), 0755) == 0;
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

std::vector<double> parse_double_list(const std::string &text)
{
    std::vector<double> values;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
            values.push_back(std::stod(item));
    }
    return values;
}

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

    double rms() const
    {
        return n > 0 ? std::sqrt(sum_sq / static_cast<double>(n)) : 0.0;
    }
};

struct Latest
{
    bool imu_valid = false;
    int64_t imu_ns = 0;
    double yaw = 0.0;
    double gyro_z = 0.0;
    double accel_x = 0.0;
    double accel_y = 0.0;
    double accel_z = 0.0;

    bool odom_valid = false;
    int64_t odom_ns = 0;
    double odom_vx = 0.0;
    double odom_vy = 0.0;
    double odom_omega = 0.0;
};

struct Options
{
    std::string cmd_topic = "/chassis/remote_cmd_vel";
    std::string imu_topic = "/IMU_data";
    std::string odom_topic = "/chassis/odom_twist";
    std::string bridge_node = "/r2_vehicle_bridge";
    std::string output_dir = "var_data/omni";

    std::vector<double> speed_list {0.35, 0.60};
    std::vector<double> kp_list {2.5, 3.5};
    std::vector<double> kd_list {0.03, 0.08};
    std::vector<double> out_scale_list {0.60};
    std::vector<double> accel_list {0.8, 1.2};
    std::vector<double> yaw_accel_list {4.0, 6.0};

    double ki = 0.0;
    double move_eps = 0.02;
    double turn_eps = 0.05;
    double dead_zone = 0.01;
    double timeout_ms = 100.0;
    double settle_s = 0.70;
    double drive_s = 2.50;
    double stop_s = 1.20;
    double turn_s = 1.20;
    double turn_stop_s = 0.80;
    double turn_omega = 0.80;
    double imu_wait_s = 3.0;
    int sample_hz = 200;
    int max_combos = 0;
    bool set_params = true;
    bool require_imu = true;
    bool straight_only = false;
};

struct ParamCombo
{
    int index = 0;
    double kp = 0.0;
    double kd = 0.0;
    double out_scale = 0.0;
    double accel = 0.0;
    double yaw_accel = 0.0;
};

struct CommandState
{
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
};

struct PoseEstimate
{
    bool initialized = false;
    int64_t last_ns = 0;
    double x = 0.0;
    double y = 0.0;

    void reset(int64_t stamp_ns)
    {
        initialized = true;
        last_ns = stamp_ns;
        x = 0.0;
        y = 0.0;
    }

    void update(const Latest &latest, int64_t stamp_ns)
    {
        if (!initialized)
        {
            reset(stamp_ns);
            return;
        }

        const double dt = std::max(0.0, static_cast<double>(stamp_ns - last_ns) * 1.0e-9);
        last_ns = stamp_ns;
        if (!latest.odom_valid || !latest.imu_valid || dt <= 0.0 || dt > 0.2)
            return;

        const double c = std::cos(latest.yaw);
        const double s = std::sin(latest.yaw);
        const double world_vx = c * latest.odom_vx - s * latest.odom_vy;
        const double world_vy = s * latest.odom_vx + c * latest.odom_vy;
        x += world_vx * dt;
        y += world_vy * dt;
    }
};

struct TrialResult
{
    ParamCombo param;
    std::string test_name;
    double command_speed = 0.0;
    double command_omega = 0.0;
    uint64_t samples = 0;
    uint64_t used_samples = 0;
    double score = std::numeric_limits<double>::infinity();
    double final_cross_m = 0.0;
    double final_yaw_abs_deg = 0.0;
    double response_t80_s = std::numeric_limits<double>::quiet_NaN();
    double stop_residual_omega_deg_s = 0.0;

    Stats yaw_abs_deg;
    Stats yaw_error_deg;
    Stats cross_abs_m;
    Stats cross_speed_mps;
    Stats speed_abs_error_mps;
    Stats omega_abs_error_deg_s;
    Stats omega_abs_deg_s;
    Stats imu_gyro_abs_deg_s;

    bool is_turn() const
    {
        return std::fabs(command_omega) > 1e-5;
    }

    void finalize()
    {
        if (used_samples == 0)
        {
            score = std::numeric_limits<double>::infinity();
            return;
        }

        if (is_turn())
        {
            const double response_penalty =
                std::isfinite(response_t80_s) ? response_t80_s : 2.0;
            score = 3.0 * response_penalty +
                    0.025 * omega_abs_error_deg_s.mean() +
                    0.02 * stop_residual_omega_deg_s;
        }
        else
        {
            score = yaw_error_deg.rms() +
                    0.35 * yaw_abs_deg.max +
                    1.10 * final_yaw_abs_deg +
                    25.0 * std::fabs(final_cross_m) +
                    8.0 * cross_abs_m.mean() +
                    4.0 * speed_abs_error_mps.mean();
        }
    }
};

struct ComboSummary
{
    ParamCombo param;
    Stats score;
    Stats straight_score;
    Stats turn_score;
    Stats yaw_rms_deg;
    Stats yaw_max_deg;
    Stats final_cross_m;
    Stats response_t80_s;
};

class TuneNode : public rclcpp::Node
{
public:
    explicit TuneNode(const Options &options)
        : Node("remote_chassis_yaw_tune"), options_(options)
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(options_.cmd_topic, 10);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            options_.imu_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_.imu_valid = true;
                latest_.imu_ns = now_ns();
                latest_.yaw = normalize_angle(yaw_from_imu(*msg));
                latest_.gyro_z = msg->angular_velocity.z;
                latest_.accel_x = msg->linear_acceleration.x;
                latest_.accel_y = msg->linear_acceleration.y;
                latest_.accel_z = msg->linear_acceleration.z;
            });

        odom_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            options_.odom_topic, 50,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_.odom_valid = true;
                latest_.odom_ns = now_ns();
                latest_.odom_vx = msg->linear.x;
                latest_.odom_vy = msg->linear.y;
                latest_.odom_omega = msg->angular.z;
            });
    }

    void publish_command(double vx, double vy, double omega)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.angular.z = omega;
        cmd_pub_->publish(msg);
    }

    void publish_zero()
    {
        publish_command(0.0, 0.0, 0.0);
    }

    Latest latest() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return latest_;
    }

private:
    Options options_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr odom_sub_;
    mutable std::mutex mtx_;
    Latest latest_;
};

Options parse_options(int argc, char **argv)
{
    Options options;
    options.cmd_topic = cli_get(argc, argv, "cmd-topic", options.cmd_topic.c_str());
    options.imu_topic = cli_get(argc, argv, "imu-topic", options.imu_topic.c_str());
    options.odom_topic = cli_get(argc, argv, "odom-topic", options.odom_topic.c_str());
    options.bridge_node = cli_get(argc, argv, "bridge-node", options.bridge_node.c_str());
    options.output_dir = cli_get(argc, argv, "output-dir", options.output_dir.c_str());

    options.speed_list = parse_double_list(cli_get(argc, argv, "speeds", "0.35,0.60"));
    options.kp_list = parse_double_list(cli_get(argc, argv, "kp-list", "2.5,3.5"));
    options.kd_list = parse_double_list(cli_get(argc, argv, "kd-list", "0.03,0.08"));
    options.out_scale_list = parse_double_list(cli_get(argc, argv, "out-scale-list", "0.60"));
    options.accel_list = parse_double_list(cli_get(argc, argv, "accel-list", "0.8,1.2"));
    options.yaw_accel_list = parse_double_list(cli_get(argc, argv, "yaw-accel-list", "4.0,6.0"));

    options.ki = std::stod(cli_get(argc, argv, "ki", "0.0"));
    options.move_eps = std::stod(cli_get(argc, argv, "move-eps", "0.02"));
    options.turn_eps = std::stod(cli_get(argc, argv, "turn-eps", "0.05"));
    options.dead_zone = std::stod(cli_get(argc, argv, "dead-zone", "0.01"));
    options.timeout_ms = std::stod(cli_get(argc, argv, "timeout-ms", "100.0"));
    options.settle_s = std::stod(cli_get(argc, argv, "settle", "0.70"));
    options.drive_s = std::stod(cli_get(argc, argv, "drive", "2.50"));
    options.stop_s = std::stod(cli_get(argc, argv, "stop", "1.20"));
    options.turn_s = std::stod(cli_get(argc, argv, "turn", "1.20"));
    options.turn_stop_s = std::stod(cli_get(argc, argv, "turn-stop", "0.80"));
    options.turn_omega = std::stod(cli_get(argc, argv, "turn-omega", "0.80"));
    options.imu_wait_s = std::stod(cli_get(argc, argv, "imu-wait", "3.0"));
    options.sample_hz = std::max(20, std::atoi(cli_get(argc, argv, "sample-hz", "200")));
    options.max_combos = std::max(0, std::atoi(cli_get(argc, argv, "max-combos", "0")));
    options.set_params = !cli_has(argc, argv, "no-param-set");
    options.require_imu = !cli_has(argc, argv, "allow-missing-imu");
    options.straight_only = cli_has(argc, argv, "straight-only");

    if (options.speed_list.empty())
        options.speed_list = {0.35, 0.60};
    return options;
}

void print_help()
{
    std::cout
        << "remote_chassis_yaw_tune options:\n"
        << "  --cmd-topic /chassis/remote_cmd_vel   command topic; use /chassis/cmd_vel to bypass remote priority\n"
        << "  --speeds 0.35,0.60                    straight speed list in m/s\n"
        << "  --kp-list 2.5,3.5                     IMU heading-hold Kp candidates\n"
        << "  --kd-list 0.03,0.08                   IMU heading-hold Kd candidates\n"
        << "  --out-scale-list 0.60                 heading output-limit scale candidates\n"
        << "  --accel-list 0.8,1.2                  remote-like linear accel/decel candidates in m/s^2\n"
        << "  --yaw-accel-list 4.0,6.0              remote-like yaw accel candidates in rad/s^2\n"
        << "  --drive 2.5 --stop 1.2                straight drive/stop durations\n"
        << "  --turn 1.2 --turn-omega 0.8           turn response test command\n"
        << "  --max-combos N                        cap sweep size for quick testing\n"
        << "  --straight-only                       skip turn response tests\n"
        << "  --no-param-set                        only record current controller params\n";
}

std::vector<ParamCombo> build_combos(const Options &options)
{
    std::vector<ParamCombo> combos;
    for (double kp : options.kp_list)
        for (double kd : options.kd_list)
            for (double out_scale : options.out_scale_list)
                for (double accel : options.accel_list)
                    for (double yaw_accel : options.yaw_accel_list)
                    {
                        ParamCombo combo;
                        combo.index = static_cast<int>(combos.size());
                        combo.kp = kp;
                        combo.kd = kd;
                        combo.out_scale = out_scale;
                        combo.accel = accel;
                        combo.yaw_accel = yaw_accel;
                        combos.push_back(combo);
                        if (options.max_combos > 0 &&
                            static_cast<int>(combos.size()) >= options.max_combos)
                            return combos;
                    }
    return combos;
}

bool wait_for_imu(const std::shared_ptr<TuneNode> &node, double wait_s)
{
    const int64_t start = now_ns();
    while (g_running.load() && rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (node->latest().imu_valid)
            return true;
        if ((now_ns() - start) > static_cast<int64_t>(wait_s * 1.0e9))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

double slew_axis(double current, double target, double accel, double decel, double dt)
{
    const double limit = (std::fabs(target) < std::fabs(current)) ? decel : accel;
    const double max_step = std::max(0.0, limit) * dt;
    const double delta = target - current;
    if (std::fabs(delta) <= max_step)
        return target;
    return current + std::copysign(max_step, delta);
}

void hold_zero(const std::shared_ptr<TuneNode> &node, const Options &options, double duration_s)
{
    const int64_t start = now_ns();
    const int64_t duration_ns = static_cast<int64_t>(duration_s * 1.0e9);
    const auto sleep_dt = std::chrono::duration<double>(1.0 / static_cast<double>(options.sample_hz));

    while (g_running.load() && rclcpp::ok() && (now_ns() - start) < duration_ns)
    {
        node->publish_zero();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(sleep_dt);
    }
    node->publish_zero();
}

bool set_heading_params(const std::shared_ptr<TuneNode> &node,
                        const std::shared_ptr<rclcpp::AsyncParametersClient> &client,
                        const Options &options,
                        const ParamCombo &combo)
{
    if (!options.set_params)
        return true;

    std::vector<rclcpp::Parameter> params;
    params.emplace_back("imu_heading_hold.enable", true);
    params.emplace_back("imu_heading_hold.kp", combo.kp);
    params.emplace_back("imu_heading_hold.ki", options.ki);
    params.emplace_back("imu_heading_hold.kd", combo.kd);
    params.emplace_back("imu_heading_hold.out_limit_scale", combo.out_scale);
    params.emplace_back("imu_heading_hold.move_eps", options.move_eps);
    params.emplace_back("imu_heading_hold.turn_eps", options.turn_eps);
    params.emplace_back("imu_heading_hold.dead_zone", options.dead_zone);
    params.emplace_back("imu_heading_hold.timeout_ms", options.timeout_ms);

    auto future = client->set_parameters(params);
    const auto rc = rclcpp::spin_until_future_complete(
        node, future, std::chrono::seconds(3));
    if (rc != rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cerr << "[remote_chassis_yaw_tune] set_parameters timed out for combo "
                  << combo.index << std::endl;
        return false;
    }

    const auto results = future.get();
    for (const auto &result : results)
    {
        if (!result.successful)
        {
            std::cerr << "[remote_chassis_yaw_tune] parameter rejected: "
                      << result.reason << std::endl;
            return false;
        }
    }
    return true;
}

void write_raw_header(std::ofstream &csv)
{
    csv << "timestamp_ns,elapsed_s,param_index,kp,ki,kd,out_scale,linear_accel,yaw_accel,"
        << "test_name,phase,command_speed,command_omega,"
        << "raw_vx,raw_vy,raw_omega,cmd_vx,cmd_vy,cmd_omega,"
        << "imu_valid,imu_age_ms,yaw_rad,yaw_ref_rad,yaw_error_deg,gyro_z,accel_x,accel_y,accel_z,"
        << "odom_valid,odom_vx,odom_vy,odom_omega,forward_speed,cross_speed,speed_error,omega_error,"
        << "world_x,world_y,cross_track_m,used_for_score\n";
}

void write_trial_header(std::ofstream &csv)
{
    csv << "param_index,kp,ki,kd,out_scale,linear_accel,yaw_accel,test_name,"
        << "command_speed,command_omega,samples,used_samples,"
        << "yaw_rms_deg,yaw_max_abs_deg,final_yaw_abs_deg,final_cross_m,cross_abs_mean_m,"
        << "cross_speed_rms_mps,speed_abs_error_mean_mps,omega_abs_error_mean_deg_s,"
        << "response_t80_s,stop_residual_omega_deg_s,score\n";
}

void write_trial_row(std::ofstream &csv, const Options &options, const TrialResult &r)
{
    csv << r.param.index << ','
        << r.param.kp << ','
        << options.ki << ','
        << r.param.kd << ','
        << r.param.out_scale << ','
        << r.param.accel << ','
        << r.param.yaw_accel << ','
        << r.test_name << ','
        << r.command_speed << ','
        << r.command_omega << ','
        << r.samples << ','
        << r.used_samples << ','
        << r.yaw_error_deg.rms() << ','
        << (r.yaw_abs_deg.n > 0 ? r.yaw_abs_deg.max : 0.0) << ','
        << r.final_yaw_abs_deg << ','
        << r.final_cross_m << ','
        << r.cross_abs_m.mean() << ','
        << r.cross_speed_mps.rms() << ','
        << r.speed_abs_error_mps.mean() << ','
        << r.omega_abs_error_deg_s.mean() << ','
        << r.response_t80_s << ','
        << r.stop_residual_omega_deg_s << ','
        << r.score << '\n';
}

TrialResult run_straight_trial(const std::shared_ptr<TuneNode> &node,
                               std::ofstream &raw_csv,
                               const Options &options,
                               const ParamCombo &combo,
                               const std::string &name,
                               double vx_target,
                               double vy_target)
{
    TrialResult result;
    result.param = combo;
    result.test_name = name;
    result.command_speed = std::hypot(vx_target, vy_target);

    CommandState cmd;
    PoseEstimate pose;
    const double dir = std::atan2(vy_target, vx_target);
    const double sleep_s = 1.0 / static_cast<double>(options.sample_hz);
    const auto sleep_dt = std::chrono::duration<double>(sleep_s);
    const int64_t start = now_ns();
    const int64_t total_ns = static_cast<int64_t>(
        (options.settle_s + options.drive_s + options.stop_s) * 1.0e9);

    double yaw_ref = 0.0;
    bool yaw_ref_set = false;

    while (g_running.load() && rclcpp::ok() && (now_ns() - start) < total_ns)
    {
        const int64_t ts = now_ns();
        const double elapsed = static_cast<double>(ts - start) * 1.0e-9;
        const bool in_settle = elapsed < options.settle_s;
        const bool in_drive = elapsed >= options.settle_s &&
                              elapsed < (options.settle_s + options.drive_s);
        const bool used = elapsed >= (options.settle_s + 0.25) &&
                          elapsed < (options.settle_s + options.drive_s + options.stop_s);
        const char *phase = in_settle ? "settle" : (in_drive ? "drive" : "stop");
        const double raw_vx = in_drive ? vx_target : 0.0;
        const double raw_vy = in_drive ? vy_target : 0.0;

        cmd.vx = slew_axis(cmd.vx, raw_vx, combo.accel, combo.accel * 1.5, sleep_s);
        cmd.vy = slew_axis(cmd.vy, raw_vy, combo.accel, combo.accel * 1.5, sleep_s);
        cmd.omega = slew_axis(cmd.omega, 0.0, combo.yaw_accel, combo.yaw_accel * 1.5, sleep_s);
        node->publish_command(cmd.vx, cmd.vy, cmd.omega);
        rclcpp::spin_some(node);

        const Latest latest = node->latest();
        if (latest.imu_valid && in_drive && !yaw_ref_set)
        {
            yaw_ref = latest.yaw;
            yaw_ref_set = true;
            pose.reset(ts);
        }
        pose.update(latest, ts);

        const double yaw_error = latest.imu_valid && yaw_ref_set ?
            normalize_angle(latest.yaw - yaw_ref) : 0.0;
        const double forward_speed =
            std::cos(dir) * latest.odom_vx + std::sin(dir) * latest.odom_vy;
        const double cross_speed =
            -std::sin(dir) * latest.odom_vx + std::cos(dir) * latest.odom_vy;
        const double speed_error = forward_speed - result.command_speed;
        const double path_dir = yaw_ref + dir;
        const double cross_track =
            -std::sin(path_dir) * pose.x + std::cos(path_dir) * pose.y;

        result.samples++;
        if (used && latest.imu_valid)
        {
            result.used_samples++;
            result.yaw_error_deg.add(rad_to_deg(yaw_error));
            result.yaw_abs_deg.add(std::fabs(rad_to_deg(yaw_error)));
            result.cross_speed_mps.add(cross_speed);
            result.cross_abs_m.add(std::fabs(cross_track));
            result.speed_abs_error_mps.add(std::fabs(speed_error));
            result.imu_gyro_abs_deg_s.add(std::fabs(deg_s(latest.gyro_z)));
            result.final_yaw_abs_deg = std::fabs(rad_to_deg(yaw_error));
            result.final_cross_m = cross_track;
        }

        raw_csv << ts << ','
                << elapsed << ','
                << combo.index << ','
                << combo.kp << ','
                << options.ki << ','
                << combo.kd << ','
                << combo.out_scale << ','
                << combo.accel << ','
                << combo.yaw_accel << ','
                << name << ','
                << phase << ','
                << result.command_speed << ','
                << 0.0 << ','
                << raw_vx << ','
                << raw_vy << ','
                << 0.0 << ','
                << cmd.vx << ','
                << cmd.vy << ','
                << cmd.omega << ','
                << latest.imu_valid << ','
                << (latest.imu_valid ? static_cast<double>(ts - latest.imu_ns) * 1.0e-6 : -1.0) << ','
                << latest.yaw << ','
                << yaw_ref << ','
                << rad_to_deg(yaw_error) << ','
                << latest.gyro_z << ','
                << latest.accel_x << ','
                << latest.accel_y << ','
                << latest.accel_z << ','
                << latest.odom_valid << ','
                << latest.odom_vx << ','
                << latest.odom_vy << ','
                << latest.odom_omega << ','
                << forward_speed << ','
                << cross_speed << ','
                << speed_error << ','
                << -latest.odom_omega << ','
                << pose.x << ','
                << pose.y << ','
                << cross_track << ','
                << used << '\n';

        std::this_thread::sleep_for(sleep_dt);
    }

    node->publish_zero();
    result.finalize();
    return result;
}

TrialResult run_turn_trial(const std::shared_ptr<TuneNode> &node,
                           std::ofstream &raw_csv,
                           const Options &options,
                           const ParamCombo &combo,
                           const std::string &name,
                           double omega_target)
{
    TrialResult result;
    result.param = combo;
    result.test_name = name;
    result.command_omega = omega_target;

    CommandState cmd;
    const double sleep_s = 1.0 / static_cast<double>(options.sample_hz);
    const auto sleep_dt = std::chrono::duration<double>(sleep_s);
    const int64_t start = now_ns();
    const int64_t total_ns = static_cast<int64_t>(
        (options.settle_s + options.turn_s + options.turn_stop_s) * 1.0e9);

    double yaw_ref = 0.0;
    bool yaw_ref_set = false;
    bool reached80 = false;

    while (g_running.load() && rclcpp::ok() && (now_ns() - start) < total_ns)
    {
        const int64_t ts = now_ns();
        const double elapsed = static_cast<double>(ts - start) * 1.0e-9;
        const bool in_settle = elapsed < options.settle_s;
        const bool in_turn = elapsed >= options.settle_s &&
                             elapsed < (options.settle_s + options.turn_s);
        const bool used = elapsed >= options.settle_s &&
                          elapsed < (options.settle_s + options.turn_s + options.turn_stop_s);
        const char *phase = in_settle ? "settle" : (in_turn ? "turn" : "stop");
        const double raw_omega = in_turn ? omega_target : 0.0;

        cmd.vx = slew_axis(cmd.vx, 0.0, combo.accel, combo.accel * 1.5, sleep_s);
        cmd.vy = slew_axis(cmd.vy, 0.0, combo.accel, combo.accel * 1.5, sleep_s);
        cmd.omega = slew_axis(cmd.omega, raw_omega, combo.yaw_accel, combo.yaw_accel * 1.5, sleep_s);
        node->publish_command(cmd.vx, cmd.vy, cmd.omega);
        rclcpp::spin_some(node);

        const Latest latest = node->latest();
        if (latest.imu_valid && in_turn && !yaw_ref_set)
        {
            yaw_ref = latest.yaw;
            yaw_ref_set = true;
        }
        const double yaw_error = latest.imu_valid && yaw_ref_set ?
            normalize_angle(latest.yaw - yaw_ref) : 0.0;
        const double omega_error = latest.odom_omega - cmd.omega;

        if (in_turn && latest.odom_valid && !reached80 &&
            std::fabs(latest.odom_omega) >= 0.8 * std::fabs(omega_target))
        {
            result.response_t80_s = elapsed - options.settle_s;
            reached80 = true;
        }
        if (!in_turn && !in_settle && latest.odom_valid)
            result.stop_residual_omega_deg_s = std::fabs(deg_s(latest.odom_omega));

        result.samples++;
        if (used && latest.imu_valid)
        {
            result.used_samples++;
            result.yaw_error_deg.add(rad_to_deg(yaw_error));
            result.yaw_abs_deg.add(std::fabs(rad_to_deg(yaw_error)));
            result.omega_abs_error_deg_s.add(std::fabs(deg_s(omega_error)));
            result.omega_abs_deg_s.add(std::fabs(deg_s(latest.odom_omega)));
            result.imu_gyro_abs_deg_s.add(std::fabs(deg_s(latest.gyro_z)));
        }

        raw_csv << ts << ','
                << elapsed << ','
                << combo.index << ','
                << combo.kp << ','
                << options.ki << ','
                << combo.kd << ','
                << combo.out_scale << ','
                << combo.accel << ','
                << combo.yaw_accel << ','
                << name << ','
                << phase << ','
                << 0.0 << ','
                << omega_target << ','
                << 0.0 << ','
                << 0.0 << ','
                << raw_omega << ','
                << cmd.vx << ','
                << cmd.vy << ','
                << cmd.omega << ','
                << latest.imu_valid << ','
                << (latest.imu_valid ? static_cast<double>(ts - latest.imu_ns) * 1.0e-6 : -1.0) << ','
                << latest.yaw << ','
                << yaw_ref << ','
                << rad_to_deg(yaw_error) << ','
                << latest.gyro_z << ','
                << latest.accel_x << ','
                << latest.accel_y << ','
                << latest.accel_z << ','
                << latest.odom_valid << ','
                << latest.odom_vx << ','
                << latest.odom_vy << ','
                << latest.odom_omega << ','
                << 0.0 << ','
                << 0.0 << ','
                << 0.0 << ','
                << omega_error << ','
                << 0.0 << ','
                << 0.0 << ','
                << 0.0 << ','
                << used << '\n';

        std::this_thread::sleep_for(sleep_dt);
    }

    node->publish_zero();
    result.finalize();
    return result;
}

void add_to_summary(ComboSummary &summary, const TrialResult &trial)
{
    summary.score.add(trial.score);
    if (trial.is_turn())
    {
        summary.turn_score.add(trial.score);
        if (std::isfinite(trial.response_t80_s))
            summary.response_t80_s.add(trial.response_t80_s);
    }
    else
    {
        summary.straight_score.add(trial.score);
        summary.yaw_rms_deg.add(trial.yaw_error_deg.rms());
        summary.yaw_max_deg.add(trial.yaw_abs_deg.n > 0 ? trial.yaw_abs_deg.max : 0.0);
        summary.final_cross_m.add(std::fabs(trial.final_cross_m));
    }
}

void write_report(const std::string &path,
                  const Options &options,
                  const std::vector<ComboSummary> &summaries,
                  const std::vector<TrialResult> &trials)
{
    std::ofstream out(path);
    out << std::fixed << std::setprecision(6);
    out << "remote_chassis_yaw_tune report\n";
    out << "cmd_topic: " << options.cmd_topic << "\n";
    out << "imu_topic: " << options.imu_topic << "\n";
    out << "odom_topic: " << options.odom_topic << "\n";
    out << "score: lower is better; straight emphasizes yaw drift and cross-track error, turn emphasizes t80 response\n\n";

    if (!summaries.empty())
    {
        const auto best_it = std::min_element(
            summaries.begin(), summaries.end(),
            [](const ComboSummary &a, const ComboSummary &b) {
                return a.score.mean() < b.score.mean();
            });

        const ComboSummary &best = *best_it;
        out << "BEST_COMBO\n";
        out << "  index: " << best.param.index << "\n";
        out << "  imu_heading_hold.kp: " << best.param.kp << "\n";
        out << "  imu_heading_hold.ki: " << options.ki << "\n";
        out << "  imu_heading_hold.kd: " << best.param.kd << "\n";
        out << "  imu_heading_hold.out_limit_scale: " << best.param.out_scale << "\n";
        out << "  remote_linear_accel_m_s2: " << best.param.accel << "\n";
        out << "  remote_yaw_accel_rad_s2: " << best.param.yaw_accel << "\n";
        out << "  mean_score: " << best.score.mean() << "\n";
        out << "  straight_score: " << best.straight_score.mean() << "\n";
        out << "  turn_score: " << best.turn_score.mean() << "\n";
        out << "  yaw_rms_deg: " << best.yaw_rms_deg.mean() << "\n";
        out << "  yaw_max_deg: " << best.yaw_max_deg.mean() << "\n";
        out << "  final_cross_m: " << best.final_cross_m.mean() << "\n";
        out << "  turn_t80_s: " << best.response_t80_s.mean() << "\n\n";

        out << "Apply heading params:\n";
        out << "  ros2 param set " << options.bridge_node << " imu_heading_hold.kp " << best.param.kp << "\n";
        out << "  ros2 param set " << options.bridge_node << " imu_heading_hold.kd " << best.param.kd << "\n";
        out << "  ros2 param set " << options.bridge_node << " imu_heading_hold.out_limit_scale " << best.param.out_scale << "\n";
        out << "Use remote command shaping near linear_accel=" << best.param.accel
            << " m/s^2, yaw_accel=" << best.param.yaw_accel << " rad/s^2.\n\n";
    }

    out << "COMBO_SUMMARY\n";
    out << "index,kp,ki,kd,out_scale,linear_accel,yaw_accel,mean_score,straight_score,turn_score,"
        << "yaw_rms_deg,yaw_max_deg,final_cross_m,turn_t80_s\n";
    for (const auto &s : summaries)
    {
        out << s.param.index << ','
            << s.param.kp << ','
            << options.ki << ','
            << s.param.kd << ','
            << s.param.out_scale << ','
            << s.param.accel << ','
            << s.param.yaw_accel << ','
            << s.score.mean() << ','
            << s.straight_score.mean() << ','
            << s.turn_score.mean() << ','
            << s.yaw_rms_deg.mean() << ','
            << s.yaw_max_deg.mean() << ','
            << s.final_cross_m.mean() << ','
            << s.response_t80_s.mean() << "\n";
    }

    out << "\nTRIAL_SUMMARY\n";
    out << "combo,test,score,yaw_rms_deg,yaw_max_deg,final_yaw_deg,final_cross_m,response_t80_s\n";
    for (const auto &t : trials)
    {
        out << t.param.index << ','
            << t.test_name << ','
            << t.score << ','
            << t.yaw_error_deg.rms() << ','
            << (t.yaw_abs_deg.n > 0 ? t.yaw_abs_deg.max : 0.0) << ','
            << t.final_yaw_abs_deg << ','
            << t.final_cross_m << ','
        << t.response_t80_s << "\n";
    }
}

} // namespace

int main(int argc, char **argv)
{
    if (cli_has(argc, argv, "help") || cli_has(argc, argv, "h"))
    {
        print_help();
        return 0;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    Options options = parse_options(argc, argv);
    const std::vector<ParamCombo> combos = build_combos(options);
    if (combos.empty())
    {
        std::cerr << "[remote_chassis_yaw_tune] no parameter combinations\n";
        return 1;
    }

    ensure_dir("var_data");
    ensure_dir(options.output_dir);
    const std::string stamp = timestamp_string();
    const std::string raw_path = options.output_dir + "/remote_chassis_yaw_tune_raw_" + stamp + ".csv";
    const std::string trial_path = options.output_dir + "/remote_chassis_yaw_tune_trials_" + stamp + ".csv";
    const std::string report_path = options.output_dir + "/remote_chassis_yaw_tune_report_" + stamp + ".txt";

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TuneNode>(options);
    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, options.bridge_node);

    if (options.require_imu && !wait_for_imu(node, options.imu_wait_s))
    {
        std::cerr << "[remote_chassis_yaw_tune] no IMU data on " << options.imu_topic
                  << "; use --allow-missing-imu only for wiring/debug runs\n";
        rclcpp::shutdown();
        return 2;
    }

    if (options.set_params)
    {
        const auto start_wait = now_ns();
        while (g_running.load() && rclcpp::ok() && !param_client->service_is_ready())
        {
            if ((now_ns() - start_wait) > 3LL * 1000LL * 1000LL * 1000LL)
            {
                std::cerr << "[remote_chassis_yaw_tune] parameter service not ready: "
                          << options.bridge_node << "\n";
                rclcpp::shutdown();
                return 3;
            }
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    std::ofstream raw_csv(raw_path);
    std::ofstream trial_csv(trial_path);
    raw_csv << std::fixed << std::setprecision(9);
    trial_csv << std::fixed << std::setprecision(9);
    write_raw_header(raw_csv);
    write_trial_header(trial_csv);

    std::vector<TrialResult> all_trials;
    std::vector<ComboSummary> summaries;

    std::cout << "[remote_chassis_yaw_tune] combos=" << combos.size()
              << " raw=" << raw_path << "\n";

    for (const ParamCombo &combo : combos)
    {
        if (!g_running.load() || !rclcpp::ok())
            break;

        std::cout << "[remote_chassis_yaw_tune] combo " << combo.index
                  << " kp=" << combo.kp
                  << " kd=" << combo.kd
                  << " out=" << combo.out_scale
                  << " accel=" << combo.accel
                  << " yaw_accel=" << combo.yaw_accel << "\n";

        if (!set_heading_params(node, param_client, options, combo))
            continue;

        ComboSummary summary;
        summary.param = combo;
        hold_zero(node, options, options.settle_s);

        for (double speed : options.speed_list)
        {
            const std::vector<std::pair<std::string, std::pair<double, double>>> straight_tests {
                {"x_forward", {speed, 0.0}},
                {"x_backward", {-speed, 0.0}},
                {"y_left", {0.0, speed}},
                {"y_right", {0.0, -speed}},
            };

            for (const auto &test : straight_tests)
            {
                TrialResult trial = run_straight_trial(node,
                                                       raw_csv,
                                                       options,
                                                       combo,
                                                       test.first,
                                                       test.second.first,
                                                       test.second.second);
                write_trial_row(trial_csv, options, trial);
                add_to_summary(summary, trial);
                all_trials.push_back(trial);
                hold_zero(node, options, options.turn_stop_s);
            }
        }

        if (!options.straight_only)
        {
            TrialResult turn_l = run_turn_trial(node,
                                                raw_csv,
                                                options,
                                                combo,
                                                "turn_left",
                                                options.turn_omega);
            write_trial_row(trial_csv, options, turn_l);
            add_to_summary(summary, turn_l);
            all_trials.push_back(turn_l);
            hold_zero(node, options, options.turn_stop_s);

            TrialResult turn_r = run_turn_trial(node,
                                                raw_csv,
                                                options,
                                                combo,
                                                "turn_right",
                                                -options.turn_omega);
            write_trial_row(trial_csv, options, turn_r);
            add_to_summary(summary, turn_r);
            all_trials.push_back(turn_r);
            hold_zero(node, options, options.turn_stop_s);
        }

        summaries.push_back(summary);
        raw_csv.flush();
        trial_csv.flush();
    }

    node->publish_zero();
    hold_zero(node, options, 0.2);
    write_report(report_path, options, summaries, all_trials);

    std::cout << "[remote_chassis_yaw_tune] trial_csv=" << trial_path << "\n";
    std::cout << "[remote_chassis_yaw_tune] report=" << report_path << "\n";

    rclcpp::shutdown();
    return 0;
}

// IMU heading-hold parameter sweep for the omni chassis.
//
// It drives +X/-X first, then +Y/-Y, each at low/medium/high speed.
// For each heading-hold parameter set it records raw samples and writes a
// per-segment and per-parameter score summary.
//
// Typical usage while vehicle_control is running:
//   ros2 run linkx_soem_demo imu_heading_hold_sweep
//   ros2 run linkx_soem_demo imu_heading_hold_sweep --duration 3.0 --speeds 0.15,0.35,0.60

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

void handle_signal(int)
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
        if (item.empty())
            continue;
        values.push_back(std::stod(item));
    }
    return values;
}

std::vector<std::string> parse_string_list(const std::string &text)
{
    std::vector<std::string> values;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
            values.push_back(item);
    }
    return values;
}

struct Options
{
    std::string cmd_topic = "/chassis/cmd_vel";
    std::string imu_topic = "/IMU_data";
    std::string odom_topic = "/chassis/odom_twist";
    std::string bridge_node = "/r2_vehicle_bridge";
    std::string output_dir = "var_data";
    std::string directions = "all";

    std::vector<double> speeds {0.15, 0.35, 0.60};
    std::vector<std::string> speed_labels {"low", "medium", "high"};
    std::vector<double> kp_list {1.5, 2.0, 2.5};
    std::vector<double> ki_list {0.0};
    std::vector<double> kd_list {0.05, 0.10, 0.15};
    std::vector<double> out_scale_list {0.6};
    std::vector<double> move_eps_list {0.02};
    std::vector<double> turn_eps_list {0.05};
    std::vector<double> dead_zone_list {0.01};

    double duration_s = 3.0;
    double distance_m = 0.0;
    double settle_s = 0.60;
    double stop_s = 0.80;
    double imu_wait_s = 3.0;
    double timeout_ms = 100.0;
    int sample_hz = 50;
    int max_combos = 0;
    bool set_params = true;
    bool require_imu = true;
};

struct ParamCombo
{
    int index = 0;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double out_scale = 0.0;
    double move_eps = 0.0;
    double turn_eps = 0.0;
    double dead_zone = 0.0;
};

struct Segment
{
    int index = 0;
    std::string group;
    std::string direction;
    std::string speed_label;
    double speed = 0.0;
    double vx = 0.0;
    double vy = 0.0;
};

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
    double imu_omega_z = 0.0;

    bool odom_valid = false;
    int64_t odom_ns = 0;
    double odom_vx = 0.0;
    double odom_vy = 0.0;
    double odom_omega = 0.0;
};

struct SegmentResult
{
    ParamCombo param;
    Segment segment;
    uint64_t samples = 0;
    uint64_t used_samples = 0;
    double yaw_final_abs_deg = 0.0;
    double global_yaw_final_abs_deg = 0.0;
    double score = std::numeric_limits<double>::infinity();

    Stats yaw_error_deg;
    Stats yaw_abs_deg;
    Stats global_yaw_error_deg;
    Stats global_yaw_abs_deg;
    Stats cross_speed_mps;
    Stats cross_abs_mps;
    Stats speed_error_mps;
    Stats speed_abs_error_mps;
    Stats odom_omega_deg_s;

    void finalize()
    {
        if (used_samples == 0 || yaw_abs_deg.n == 0)
        {
            score = std::numeric_limits<double>::infinity();
            return;
        }

        const double yaw_rms = global_yaw_error_deg.n > 0 ? global_yaw_error_deg.rms() : yaw_error_deg.rms();
        const double yaw_max = global_yaw_abs_deg.n > 0 ? global_yaw_abs_deg.max : yaw_abs_deg.max;
        const double cross_rms = cross_speed_mps.rms();
        const double omega_rms = odom_omega_deg_s.rms();

        // Lower is better. Use global heading error so stop/start between segments cannot hide
        // cumulative yaw drift. Cross velocity and yaw-rate smoothness are secondary penalties.
        score = yaw_rms + 0.25 * yaw_max + 20.0 * cross_rms + 0.05 * omega_rms;
    }
};

struct ComboSummary
{
    ParamCombo param;
    Stats score;
    Stats yaw_rms_deg;
    Stats yaw_max_deg;
    Stats global_yaw_rms_deg;
    Stats global_yaw_max_deg;
    Stats cross_rms_mps;
    Stats omega_rms_deg_s;
};

class SweepNode : public rclcpp::Node
{
public:
    explicit SweepNode(const Options &options)
        : Node("imu_heading_hold_sweep"), options_(options)
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(options_.cmd_topic, 10);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            options_.imu_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                Latest next;
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    next = latest_;
                }
                next.imu_valid = true;
                next.imu_ns = now_ns();
                next.yaw = normalize_angle(yaw_from_imu(*msg));
                next.imu_omega_z = msg->angular_velocity.z;
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    latest_.imu_valid = next.imu_valid;
                    latest_.imu_ns = next.imu_ns;
                    latest_.yaw = next.yaw;
                    latest_.imu_omega_z = next.imu_omega_z;
                }
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

protected:
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
    options.directions = cli_get(argc, argv, "directions", options.directions.c_str());

    options.speeds = parse_double_list(cli_get(argc, argv, "speeds", "0.15,0.35,0.60"));
    options.speed_labels = parse_string_list(cli_get(argc, argv, "speed-labels", "low,medium,high"));
    options.kp_list = parse_double_list(cli_get(argc, argv, "kp-list", "1.5,2.0,2.5"));
    options.ki_list = parse_double_list(cli_get(argc, argv, "ki-list", "0.0"));
    options.kd_list = parse_double_list(cli_get(argc, argv, "kd-list", "0.05,0.10,0.15"));
    options.out_scale_list = parse_double_list(cli_get(argc, argv, "out-scale-list", "0.6"));
    options.move_eps_list = parse_double_list(cli_get(argc, argv, "move-eps-list", "0.02"));
    options.turn_eps_list = parse_double_list(cli_get(argc, argv, "turn-eps-list", "0.05"));
    options.dead_zone_list = parse_double_list(cli_get(argc, argv, "dead-zone-list", "0.01"));

    options.duration_s = std::stod(cli_get(argc, argv, "duration", "3.0"));
    options.distance_m = std::stod(cli_get(argc, argv, "distance", "0.0"));
    options.settle_s = std::stod(cli_get(argc, argv, "settle", "0.60"));
    options.stop_s = std::stod(cli_get(argc, argv, "stop", "0.80"));
    options.imu_wait_s = std::stod(cli_get(argc, argv, "imu-wait", "3.0"));
    options.timeout_ms = std::stod(cli_get(argc, argv, "timeout-ms", "100.0"));
    options.sample_hz = std::max(5, std::atoi(cli_get(argc, argv, "sample-hz", "50")));
    options.max_combos = std::max(0, std::atoi(cli_get(argc, argv, "max-combos", "0")));
    options.set_params = !cli_has(argc, argv, "no-param-set");
    options.require_imu = !cli_has(argc, argv, "allow-missing-imu");

    if (options.speeds.empty())
        options.speeds = {0.15, 0.35, 0.60};
    while (options.speed_labels.size() < options.speeds.size())
        options.speed_labels.push_back("speed" + std::to_string(options.speed_labels.size()));

    return options;
}

void print_help()
{
    std::cout
        << "imu_heading_hold_sweep options:\n"
        << "  --speeds 0.15,0.35,0.60          low/medium/high command speeds (m/s)\n"
        << "  --kp-list 1.5,2.0,2.5            heading PID Kp candidates\n"
        << "  --ki-list 0.0                    heading PID Ki candidates\n"
        << "  --kd-list 0.05,0.10,0.15         heading PID Kd candidates\n"
        << "  --out-scale-list 0.6             output limit as chassis omega scale\n"
        << "  --move-eps-list 0.02             movement threshold candidates\n"
        << "  --turn-eps-list 0.05             manual-turn threshold candidates\n"
        << "  --dead-zone-list 0.01            PID dead-zone candidates\n"
        << "  --duration 3.0                   drive time per segment\n"
        << "  --distance 0.0                   if >0, drive this distance per segment\n"
        << "  --settle 0.60                    initial drive time ignored in scoring\n"
        << "  --stop 0.80                      zero-command time between segments\n"
        << "  --directions all|fb|lr           all, front/back only, or left/right only\n"
        << "  --output-dir var_data            CSV/report directory\n"
        << "  --no-param-set                   record current controller params only\n";
}

std::vector<ParamCombo> build_param_combos(const Options &options)
{
    std::vector<ParamCombo> combos;
    for (double kp : options.kp_list)
        for (double ki : options.ki_list)
            for (double kd : options.kd_list)
                for (double out_scale : options.out_scale_list)
                    for (double move_eps : options.move_eps_list)
                        for (double turn_eps : options.turn_eps_list)
                            for (double dead_zone : options.dead_zone_list)
                            {
                                ParamCombo combo;
                                combo.index = static_cast<int>(combos.size());
                                combo.kp = kp;
                                combo.ki = ki;
                                combo.kd = kd;
                                combo.out_scale = out_scale;
                                combo.move_eps = move_eps;
                                combo.turn_eps = turn_eps;
                                combo.dead_zone = dead_zone;
                                combos.push_back(combo);
                                if (options.max_combos > 0 &&
                                    static_cast<int>(combos.size()) >= options.max_combos)
                                    return combos;
                            }
    return combos;
}

std::vector<Segment> build_segments(const Options &options)
{
    std::vector<Segment> segments;
    const bool do_fb = options.directions == "all" || options.directions == "fb";
    const bool do_lr = options.directions == "all" || options.directions == "lr";

    auto add_segment = [&](const std::string &group,
                           const std::string &direction,
                           const std::string &speed_label,
                           double speed,
                           double vx,
                           double vy) {
        Segment segment;
        segment.index = static_cast<int>(segments.size());
        segment.group = group;
        segment.direction = direction;
        segment.speed_label = speed_label;
        segment.speed = speed;
        segment.vx = vx;
        segment.vy = vy;
        segments.push_back(segment);
    };

    if (do_fb)
    {
        for (size_t i = 0; i < options.speeds.size(); ++i)
        {
            const double speed = options.speeds[i];
            const std::string &label = options.speed_labels[i];
            add_segment("front_back", "forward", label, speed, speed, 0.0);
            add_segment("front_back", "backward", label, speed, -speed, 0.0);
        }
    }

    if (do_lr)
    {
        for (size_t i = 0; i < options.speeds.size(); ++i)
        {
            const double speed = options.speeds[i];
            const std::string &label = options.speed_labels[i];
            add_segment("left_right", "left", label, speed, 0.0, speed);
            add_segment("left_right", "right", label, speed, 0.0, -speed);
        }
    }

    return segments;
}

bool wait_for_imu(const std::shared_ptr<SweepNode> &node, double wait_s)
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

void hold_zero(const std::shared_ptr<SweepNode> &node, const Options &options, double duration_s)
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

bool set_heading_params(const std::shared_ptr<SweepNode> &node,
                        const std::shared_ptr<rclcpp::AsyncParametersClient> &client,
                        const Options &options,
                        const ParamCombo &combo)
{
    if (!options.set_params)
        return true;

    std::vector<rclcpp::Parameter> params;
    params.emplace_back("imu_heading_hold.enable", true);
    params.emplace_back("imu_heading_hold.kp", combo.kp);
    params.emplace_back("imu_heading_hold.ki", combo.ki);
    params.emplace_back("imu_heading_hold.kd", combo.kd);
    params.emplace_back("imu_heading_hold.out_limit_scale", combo.out_scale);
    params.emplace_back("imu_heading_hold.move_eps", combo.move_eps);
    params.emplace_back("imu_heading_hold.turn_eps", combo.turn_eps);
    params.emplace_back("imu_heading_hold.dead_zone", combo.dead_zone);
    params.emplace_back("imu_heading_hold.timeout_ms", options.timeout_ms);

    auto future = client->set_parameters(params);
    const auto rc = rclcpp::spin_until_future_complete(
        node, future, std::chrono::seconds(3));
    if (rc != rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cerr << "[imu_heading_hold_sweep] set_parameters timed out for combo "
                  << combo.index << std::endl;
        return false;
    }

    const auto results = future.get();
    for (const auto &result : results)
    {
        if (!result.successful)
        {
            std::cerr << "[imu_heading_hold_sweep] parameter rejected: "
                      << result.reason << std::endl;
            return false;
        }
    }
    return true;
}

void write_raw_header(std::ofstream &csv)
{
    csv << "timestamp_ns,elapsed_s,param_index,kp,ki,kd,out_scale,move_eps,turn_eps,dead_zone,"
        << "segment_index,group,direction,speed_label,speed_mps,"
        << "vx_cmd,vy_cmd,omega_cmd,used_for_score,"
        << "imu_valid,imu_age_ms,yaw_rad,yaw_ref_rad,yaw_error_deg,"
        << "combo_yaw_ref_rad,combo_yaw_error_deg,imu_omega_z,"
        << "odom_valid,odom_vx,odom_vy,odom_omega,forward_speed,cross_speed,speed_error\n";
}

void write_segment_header(std::ofstream &csv)
{
    csv << "param_index,kp,ki,kd,out_scale,move_eps,turn_eps,dead_zone,"
        << "segment_index,group,direction,speed_label,speed_mps,vx_cmd,vy_cmd,"
        << "samples,used_samples,yaw_rms_deg,yaw_abs_mean_deg,yaw_max_abs_deg,yaw_final_abs_deg,"
        << "combo_yaw_rms_deg,combo_yaw_abs_mean_deg,combo_yaw_max_abs_deg,combo_yaw_final_abs_deg,"
        << "cross_rms_mps,cross_abs_mean_mps,speed_abs_error_mean_mps,odom_omega_rms_deg_s,score\n";
}

void write_segment_row(std::ofstream &csv, const SegmentResult &result)
{
    csv << result.param.index << ','
        << result.param.kp << ','
        << result.param.ki << ','
        << result.param.kd << ','
        << result.param.out_scale << ','
        << result.param.move_eps << ','
        << result.param.turn_eps << ','
        << result.param.dead_zone << ','
        << result.segment.index << ','
        << result.segment.group << ','
        << result.segment.direction << ','
        << result.segment.speed_label << ','
        << result.segment.speed << ','
        << result.segment.vx << ','
        << result.segment.vy << ','
        << result.samples << ','
        << result.used_samples << ','
        << result.yaw_error_deg.rms() << ','
        << result.yaw_abs_deg.mean() << ','
        << (result.yaw_abs_deg.n > 0 ? result.yaw_abs_deg.max : 0.0) << ','
        << result.yaw_final_abs_deg << ','
        << result.global_yaw_error_deg.rms() << ','
        << result.global_yaw_abs_deg.mean() << ','
        << (result.global_yaw_abs_deg.n > 0 ? result.global_yaw_abs_deg.max : 0.0) << ','
        << result.global_yaw_final_abs_deg << ','
        << result.cross_speed_mps.rms() << ','
        << result.cross_abs_mps.mean() << ','
        << result.speed_abs_error_mps.mean() << ','
        << result.odom_omega_deg_s.rms() << ','
        << result.score << '\n';
}

SegmentResult run_segment(const std::shared_ptr<SweepNode> &node,
                          const Options &options,
                          const ParamCombo &combo,
                          const Segment &segment,
                          double combo_yaw_ref,
                          std::ofstream &raw_csv)
{
    hold_zero(node, options, options.stop_s);

    const Latest start_latest = node->latest();
    const double yaw_ref = start_latest.imu_valid ? start_latest.yaw : 0.0;

    SegmentResult result;
    result.param = combo;
    result.segment = segment;

    const int64_t start = now_ns();
    const double segment_duration_s =
        (options.distance_m > 0.0 && segment.speed > 1e-6) ?
        (options.distance_m / segment.speed) :
        options.duration_s;
    const int64_t duration_ns = static_cast<int64_t>(segment_duration_s * 1.0e9);
    const auto sleep_dt = std::chrono::duration<double>(1.0 / static_cast<double>(options.sample_hz));

    const double cmd_speed = std::sqrt(segment.vx * segment.vx + segment.vy * segment.vy);
    const double unit_x = (cmd_speed > 1e-6) ? segment.vx / cmd_speed : 1.0;
    const double unit_y = (cmd_speed > 1e-6) ? segment.vy / cmd_speed : 0.0;

    while (g_running.load() && rclcpp::ok() && (now_ns() - start) < duration_ns)
    {
        node->publish_command(segment.vx, segment.vy, 0.0);
        rclcpp::spin_some(node);

        const int64_t sample_ns = now_ns();
        const double elapsed_s = static_cast<double>(sample_ns - start) / 1.0e9;
        const bool used = elapsed_s >= options.settle_s;
        const Latest latest = node->latest();

        const bool imu_valid = latest.imu_valid;
        const double yaw_error_rad = imu_valid ? normalize_angle(latest.yaw - yaw_ref) : 0.0;
        const double yaw_error_deg = rad_to_deg(yaw_error_rad);
        const double combo_yaw_error_deg =
            imu_valid ? rad_to_deg(normalize_angle(latest.yaw - combo_yaw_ref)) : 0.0;
        const double imu_age_ms = imu_valid ? static_cast<double>(sample_ns - latest.imu_ns) / 1.0e6 : -1.0;

        double forward_speed = 0.0;
        double cross_speed = 0.0;
        double speed_error = 0.0;
        if (latest.odom_valid)
        {
            forward_speed = latest.odom_vx * unit_x + latest.odom_vy * unit_y;
            cross_speed = -latest.odom_vx * unit_y + latest.odom_vy * unit_x;
            speed_error = forward_speed - cmd_speed;
        }

        result.samples++;
        if (used && imu_valid)
        {
            result.used_samples++;
            result.yaw_error_deg.add(yaw_error_deg);
            result.yaw_abs_deg.add(std::fabs(yaw_error_deg));
            result.yaw_final_abs_deg = std::fabs(yaw_error_deg);
            result.global_yaw_error_deg.add(combo_yaw_error_deg);
            result.global_yaw_abs_deg.add(std::fabs(combo_yaw_error_deg));
            result.global_yaw_final_abs_deg = std::fabs(combo_yaw_error_deg);
            if (latest.odom_valid)
            {
                result.cross_speed_mps.add(cross_speed);
                result.cross_abs_mps.add(std::fabs(cross_speed));
                result.speed_error_mps.add(speed_error);
                result.speed_abs_error_mps.add(std::fabs(speed_error));
                result.odom_omega_deg_s.add(rad_to_deg(latest.odom_omega));
            }
        }

        raw_csv << sample_ns << ','
                << elapsed_s << ','
                << combo.index << ','
                << combo.kp << ','
                << combo.ki << ','
                << combo.kd << ','
                << combo.out_scale << ','
                << combo.move_eps << ','
                << combo.turn_eps << ','
                << combo.dead_zone << ','
                << segment.index << ','
                << segment.group << ','
                << segment.direction << ','
                << segment.speed_label << ','
                << segment.speed << ','
                << segment.vx << ','
                << segment.vy << ','
                << 0.0 << ','
                << (used ? 1 : 0) << ','
                << (imu_valid ? 1 : 0) << ','
                << imu_age_ms << ','
                << (imu_valid ? latest.yaw : 0.0) << ','
                << yaw_ref << ','
                << yaw_error_deg << ','
                << combo_yaw_ref << ','
                << combo_yaw_error_deg << ','
                << (imu_valid ? latest.imu_omega_z : 0.0) << ','
                << (latest.odom_valid ? 1 : 0) << ','
                << latest.odom_vx << ','
                << latest.odom_vy << ','
                << latest.odom_omega << ','
                << forward_speed << ','
                << cross_speed << ','
                << speed_error << '\n';

        std::this_thread::sleep_for(sleep_dt);
    }

    node->publish_zero();
    result.finalize();
    return result;
}

std::vector<ComboSummary> summarize_combos(const std::vector<SegmentResult> &segments,
                                           const std::vector<ParamCombo> &combos)
{
    std::vector<ComboSummary> summaries;
    summaries.reserve(combos.size());
    for (const auto &combo : combos)
    {
        ComboSummary summary;
        summary.param = combo;
        for (const auto &segment : segments)
        {
            if (segment.param.index != combo.index || !std::isfinite(segment.score))
                continue;
            summary.score.add(segment.score);
            summary.yaw_rms_deg.add(segment.yaw_error_deg.rms());
            summary.yaw_max_deg.add(segment.yaw_abs_deg.n > 0 ? segment.yaw_abs_deg.max : 0.0);
            summary.global_yaw_rms_deg.add(segment.global_yaw_error_deg.rms());
            summary.global_yaw_max_deg.add(segment.global_yaw_abs_deg.n > 0 ? segment.global_yaw_abs_deg.max : 0.0);
            summary.cross_rms_mps.add(segment.cross_speed_mps.rms());
            summary.omega_rms_deg_s.add(segment.odom_omega_deg_s.rms());
        }
        summaries.push_back(summary);
    }

    std::sort(summaries.begin(), summaries.end(),
              [](const ComboSummary &a, const ComboSummary &b) {
                  const bool a_ok = a.score.n > 0;
                  const bool b_ok = b.score.n > 0;
                  if (a_ok != b_ok)
                      return a_ok;
                  return a.score.mean() < b.score.mean();
              });
    return summaries;
}

void write_best_report(const std::string &path,
                       const Options &options,
                       const std::vector<ComboSummary> &summaries,
                       double global_start_yaw,
                       double global_end_yaw,
                       const std::string &raw_path,
                       const std::string &segment_path)
{
    std::ofstream out(path);
    out << "IMU heading hold sweep report\n";
    out << "raw_csv: " << raw_path << "\n";
    out << "segment_csv: " << segment_path << "\n";
    out << "score = combo_yaw_rms_deg + 0.25*combo_yaw_max_deg"
        << " + 20*cross_rms_mps + 0.05*odom_omega_rms_deg_s\n";
    out << "duration_s: " << options.duration_s << ", distance_m: " << options.distance_m
        << ", settle_s: " << options.settle_s
        << ", stop_s: " << options.stop_s << "\n\n";
    out << "Cumulative yaw over this run:\n";
    out << "  start_yaw_deg: " << rad_to_deg(global_start_yaw) << "\n";
    out << "  end_yaw_deg: " << rad_to_deg(global_end_yaw) << "\n";
    out << "  delta_yaw_deg: " << rad_to_deg(normalize_angle(global_end_yaw - global_start_yaw)) << "\n\n";

    if (summaries.empty() || summaries.front().score.n == 0)
    {
        out << "No valid scored samples. Check /IMU_data and /chassis/odom_twist.\n";
        return;
    }

    const auto &best = summaries.front();
    out << "Best parameter set:\n";
    out << "  param_index: " << best.param.index << "\n";
    out << "  kp: " << best.param.kp << "\n";
    out << "  ki: " << best.param.ki << "\n";
    out << "  kd: " << best.param.kd << "\n";
    out << "  out_limit_scale: " << best.param.out_scale << "\n";
    out << "  move_eps: " << best.param.move_eps << "\n";
    out << "  turn_eps: " << best.param.turn_eps << "\n";
    out << "  dead_zone: " << best.param.dead_zone << "\n";
    out << "  mean_score: " << best.score.mean() << "\n";
    out << "  mean_yaw_rms_deg: " << best.yaw_rms_deg.mean() << "\n";
    out << "  mean_yaw_max_deg: " << best.yaw_max_deg.mean() << "\n";
    out << "  mean_combo_yaw_rms_deg: " << best.global_yaw_rms_deg.mean() << "\n";
    out << "  mean_combo_yaw_max_deg: " << best.global_yaw_max_deg.mean() << "\n";
    out << "  mean_cross_rms_mps: " << best.cross_rms_mps.mean() << "\n";
    out << "  mean_odom_omega_rms_deg_s: " << best.omega_rms_deg_s.mean() << "\n\n";

    out << "Top parameter sets:\n";
    out << "rank,param_index,kp,ki,kd,out_scale,move_eps,turn_eps,dead_zone,"
        << "mean_score,mean_yaw_rms_deg,mean_yaw_max_deg,"
        << "mean_combo_yaw_rms_deg,mean_combo_yaw_max_deg,mean_cross_rms_mps,segments\n";
    const size_t limit = std::min<size_t>(10, summaries.size());
    for (size_t i = 0; i < limit; ++i)
    {
        const auto &s = summaries[i];
        out << (i + 1) << ','
            << s.param.index << ','
            << s.param.kp << ','
            << s.param.ki << ','
            << s.param.kd << ','
            << s.param.out_scale << ','
            << s.param.move_eps << ','
            << s.param.turn_eps << ','
            << s.param.dead_zone << ','
            << s.score.mean() << ','
            << s.yaw_rms_deg.mean() << ','
            << s.yaw_max_deg.mean() << ','
            << s.global_yaw_rms_deg.mean() << ','
            << s.global_yaw_max_deg.mean() << ','
            << s.cross_rms_mps.mean() << ','
            << s.score.n << '\n';
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

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const Options options = parse_options(argc, argv);
    if (!ensure_dir(options.output_dir))
    {
        std::cerr << "Cannot create output directory: " << options.output_dir << std::endl;
        return 1;
    }

    const std::string stamp = timestamp_string();
    const std::string raw_path = options.output_dir + "/imu_heading_hold_sweep_" + stamp + ".csv";
    const std::string segment_path = options.output_dir + "/imu_heading_hold_segments_" + stamp + ".csv";
    const std::string report_path = options.output_dir + "/imu_heading_hold_best_" + stamp + ".txt";

    std::ofstream raw_csv(raw_path);
    std::ofstream segment_csv(segment_path);
    if (!raw_csv || !segment_csv)
    {
        std::cerr << "Cannot open output CSV files." << std::endl;
        return 1;
    }
    raw_csv << std::fixed << std::setprecision(6);
    segment_csv << std::fixed << std::setprecision(6);
    write_raw_header(raw_csv);
    write_segment_header(segment_csv);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SweepNode>(options);
    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, options.bridge_node);

    if (options.set_params)
    {
        std::cout << "Waiting for parameter service on " << options.bridge_node << " ..." << std::endl;
        if (!param_client->wait_for_service(std::chrono::seconds(8)))
        {
            std::cerr << "Parameter service unavailable. Is vehicle_control running?" << std::endl;
            node->publish_zero();
            rclcpp::shutdown();
            return 2;
        }
    }

    std::cout << "Waiting for IMU topic " << options.imu_topic << " ..." << std::endl;
    const bool has_imu = wait_for_imu(node, options.imu_wait_s);
    if (!has_imu && options.require_imu)
    {
        std::cerr << "No IMU samples received. Use --allow-missing-imu only for dry logging." << std::endl;
        node->publish_zero();
        rclcpp::shutdown();
        return 3;
    }
    const double global_yaw_ref = node->latest().imu_valid ? node->latest().yaw : 0.0;

    const auto combos = build_param_combos(options);
    const auto segments = build_segments(options);
    std::vector<SegmentResult> results;
    results.reserve(combos.size() * segments.size());

    std::cout << "Running " << combos.size() << " parameter combos and "
              << segments.size() << " motion segments per combo." << std::endl;

    for (const auto &combo : combos)
    {
        if (!g_running.load() || !rclcpp::ok())
            break;

        std::cout << "\n[param " << combo.index << "]"
                  << " kp=" << combo.kp
                  << " ki=" << combo.ki
                  << " kd=" << combo.kd
                  << " out_scale=" << combo.out_scale
                  << " move_eps=" << combo.move_eps
                  << " turn_eps=" << combo.turn_eps
                  << " dead_zone=" << combo.dead_zone
                  << std::endl;

        if (!set_heading_params(node, param_client, options, combo))
            break;

        hold_zero(node, options, options.stop_s);
        const double combo_yaw_ref = node->latest().imu_valid ? node->latest().yaw : global_yaw_ref;

        for (const auto &segment : segments)
        {
            if (!g_running.load() || !rclcpp::ok())
                break;

            std::cout << "  segment " << segment.index
                      << " " << segment.direction
                      << " " << segment.speed_label
                      << " vx=" << segment.vx
                      << " vy=" << segment.vy;
            if (options.distance_m > 0.0 && segment.speed > 1e-6)
            {
                std::cout << " distance=" << options.distance_m
                          << "m duration=" << (options.distance_m / segment.speed)
                          << "s";
            }
            std::cout << std::endl;

            SegmentResult result = run_segment(node, options, combo, segment, combo_yaw_ref, raw_csv);
            write_segment_row(segment_csv, result);
            raw_csv.flush();
            segment_csv.flush();
            results.push_back(result);
        }
    }

    hold_zero(node, options, std::max(0.5, options.stop_s));
    node->publish_zero();

    const auto summaries = summarize_combos(results, combos);
    const double global_yaw_end = node->latest().imu_valid ? node->latest().yaw : global_yaw_ref;
    write_best_report(report_path, options, summaries, global_yaw_ref, global_yaw_end, raw_path, segment_path);

    if (!summaries.empty() && summaries.front().score.n > 0)
    {
        const auto &best = summaries.front();
        std::cout << "\nBest param_index=" << best.param.index
                  << " kp=" << best.param.kp
                  << " ki=" << best.param.ki
                  << " kd=" << best.param.kd
                  << " out_scale=" << best.param.out_scale
                  << " score=" << best.score.mean()
                  << std::endl;
    }

    std::cout << "Raw CSV: " << raw_path << std::endl;
    std::cout << "Segment CSV: " << segment_path << std::endl;
    std::cout << "Report: " << report_path << std::endl;
    std::cout << "Cumulative yaw delta: "
              << rad_to_deg(normalize_angle(global_yaw_end - global_yaw_ref))
              << " deg" << std::endl;

    rclcpp::shutdown();
    return 0;
}

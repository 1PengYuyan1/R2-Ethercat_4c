// Omni straight-line autotune for the 4-wheel omni chassis.
//
// This is the B-layer tuner described in omni_straight_line_autotune_SPEC.md:
// it tunes online imu_heading_hold.* parameters, runs repeated 2 m straight
// segments, optionally restarts the IMU node before each segment to re-zero
// attitude, scores yaw/cross-track quality, and writes CSV/YAML/report outputs.
//
// The A-layer wheel speed-correction search is intentionally not implemented
// here because wheel_speed_correction is still compile-time vehicle code state.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
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

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int64_t kNsPerSecond = 1000LL * 1000LL * 1000LL;

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

double deg_to_rad(double value)
{
    return value * kPi / 180.0;
}

double clamp(double value, double lo, double hi)
{
    return std::min(hi, std::max(lo, value));
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
        const std::string part = path.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
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

std::string shell_quote(const std::string &value)
{
    std::string quoted = "'";
    for (char c : value)
    {
        if (c == '\'')
            quoted += "'\\''";
        else
            quoted += c;
    }
    quoted += "'";
    return quoted;
}

std::vector<int> pgrep_f(const std::string &pattern)
{
    std::vector<int> pids;
    const std::string cmd = "pgrep -f " + shell_quote(pattern);
    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
        return pids;

    char line[64] {};
    while (fgets(line, sizeof(line), pipe))
    {
        char *end = nullptr;
        const long pid = std::strtol(line, &end, 10);
        if (pid > 1 && pid <= std::numeric_limits<int>::max())
            pids.push_back(static_cast<int>(pid));
    }
    pclose(pipe);
    return pids;
}

bool pid_alive(int pid)
{
    return pid > 1 && kill(pid, 0) == 0;
}

int terminate_matching_processes(const std::string &pattern)
{
    int count = 0;
    const int self = static_cast<int>(getpid());
    const int parent = static_cast<int>(getppid());
    std::vector<int> pids = pgrep_f(pattern);

    for (int pid : pids)
    {
        if (pid == self || pid == parent)
            continue;
        if (kill(pid, SIGTERM) == 0)
            count++;
    }

    for (int i = 0; i < 20; ++i)
    {
        bool any_alive = false;
        for (int pid : pids)
        {
            if (pid != self && pid != parent && pid_alive(pid))
            {
                any_alive = true;
                break;
            }
        }
        if (!any_alive)
            return count;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    for (int pid : pids)
    {
        if (pid != self && pid != parent && pid_alive(pid))
            kill(pid, SIGKILL);
    }
    return count;
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

    double stddev() const
    {
        if (n < 2)
            return 0.0;
        const double m = mean();
        return std::sqrt(std::max(0.0, sum_sq / static_cast<double>(n) - m * m));
    }
};

struct Latest
{
    bool imu_valid = false;
    int64_t imu_ns = 0;
    double yaw = 0.0;
    double gyro_z = 0.0;

    bool odom_valid = false;
    int64_t odom_ns = 0;
    double odom_vx = 0.0;
    double odom_vy = 0.0;
    double odom_omega = 0.0;
};

struct Options
{
    std::string cmd_topic = "/chassis/cmd_vel";
    std::string imu_topic = "/IMU_data";
    std::string odom_topic = "/chassis/odom_twist";
    std::string bridge_node = "/r2_vehicle_bridge";
    std::string output_dir = "var_data";
    std::string directions = "fb";
    std::string imu_reset = "each";
    std::string imu_node = "IMU_publisher";
    std::string imu_launch = "ros2 launch hipnuc_imu imu_spec_msg.launch.py";
    std::string search = "coarse2fine";

    std::vector<double> kp_list {1.5, 2.5, 3.5};
    std::vector<double> kd_list {0.03, 0.08, 0.15};
    std::vector<double> out_scale_list {0.6, 0.8};
    std::vector<double> dead_zone_list {0.01};
    std::vector<double> ki_list {0.0};

    double distance_m = 2.0;
    double speed_mps = 0.9;
    double linear_accel_mps2 = 0.0;
    double linear_decel_mps2 = 0.0;
    double accel_settle_s = 0.5;
    double stop_s = 0.8;
    double imu_wait_s = 4.0;
    double imu_stable_s = 1.0;
    double reset_attitude_delay_s = 0.5;
    double timeout_scale = 1.6;
    double timeout_ms = 100.0;
    double move_eps = 0.02;
    double turn_eps = 0.05;
    double kf = 0.0;
    double i_out_max = 0.0;
    double measured_offset_m = std::numeric_limits<double>::quiet_NaN();
    double max_imu_age_ms = 500.0;
    double max_odom_age_ms = 500.0;
    int repeats = 2;
    int max_rounds = 40;
    int sample_hz = 100;
    bool dry_run = false;
    bool yes = false;
    bool set_params = true;
};

struct ParamSet
{
    int id = -1;
    double kp = 3.5;
    double ki = 0.0;
    double kd = 0.03;
    double kf = 0.0;
    double out_limit_scale = 0.8;
    double dead_zone = 0.01;
    double move_eps = 0.02;
    double turn_eps = 0.05;
    double i_out_max = 0.0;
    double timeout_ms = 100.0;
};

struct MotionSegment
{
    int id = 0;
    std::string group;
    std::string direction;
    double vx = 0.0;
    double vy = 0.0;
};

struct SegmentResult
{
    ParamSet param;
    MotionSegment segment;
    int repeat = 0;
    bool success = false;
    std::string failure_reason;
    uint64_t samples = 0;
    uint64_t used_samples = 0;
    double duration_s = 0.0;
    double distance_m = 0.0;
    double lateral_offset_m = 0.0;
    double yaw_final_abs_deg = 0.0;
    double overshoot_deg = 0.0;
    double score = std::numeric_limits<double>::infinity();
    double max_yaw_deg = -std::numeric_limits<double>::infinity();
    double min_yaw_deg = std::numeric_limits<double>::infinity();

    Stats yaw_error_deg;
    Stats yaw_abs_deg;
    Stats cross_speed_mps;
    Stats omega_deg_s;

    void finalize()
    {
        if (used_samples == 0 || yaw_error_deg.n == 0)
        {
            success = false;
            if (failure_reason.empty())
                failure_reason = "no_scored_samples";
            score = std::numeric_limits<double>::infinity();
            return;
        }
        if (!failure_reason.empty())
        {
            success = false;
            score = std::numeric_limits<double>::infinity();
            return;
        }

        if (max_yaw_deg > 0.10 && min_yaw_deg < -0.10)
            overshoot_deg = std::min(max_yaw_deg, -min_yaw_deg);
        else
            overshoot_deg = 0.0;

        const double yaw_rms_deg = yaw_error_deg.rms();
        const double cross_rms_mps = cross_speed_mps.rms();
        const double omega_rms_deg_s = omega_deg_s.rms();
        const double lateral_abs_m = std::fabs(lateral_offset_m);

        score = yaw_rms_deg
              + 0.5 * yaw_final_abs_deg
              + 40.0 * lateral_abs_m
              + 15.0 * cross_rms_mps
              + 0.05 * omega_rms_deg_s
              + 0.3 * overshoot_deg;
        success = std::isfinite(score);
    }
};

struct Evaluation
{
    int round = 0;
    std::string phase;
    ParamSet param;
    bool success = false;
    double aggregate_score = std::numeric_limits<double>::infinity();
    Stats segment_score;
    Stats yaw_rms_deg;
    Stats yaw_final_abs_deg;
    Stats lateral_abs_m;
    Stats cross_rms_mps;
    Stats omega_rms_deg_s;
    Stats overshoot_deg;
    int failures = 0;
};

class AutotuneNode : public rclcpp::Node
{
public:
    explicit AutotuneNode(const Options &options)
        : Node("omni_straight_line_autotune"), options_(options)
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

    void clear_imu()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_.imu_valid = false;
        latest_.imu_ns = 0;
        latest_.yaw = 0.0;
        latest_.gyro_z = 0.0;
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
    options.directions = cli_get(argc, argv, "directions", options.directions.c_str());
    options.imu_reset = cli_get(argc, argv, "imu-reset", options.imu_reset.c_str());
    options.imu_node = cli_get(argc, argv, "imu-node", options.imu_node.c_str());
    options.imu_launch = cli_get(argc, argv, "imu-launch", options.imu_launch.c_str());
    options.search = cli_get(argc, argv, "search", options.search.c_str());

    options.kp_list = parse_double_list(cli_get(argc, argv, "kp-list", "1.5,2.5,3.5"));
    options.kd_list = parse_double_list(cli_get(argc, argv, "kd-list", "0.03,0.08,0.15"));
    options.out_scale_list = parse_double_list(cli_get(argc, argv, "out-scale-list", "0.6,0.8"));
    options.dead_zone_list = parse_double_list(cli_get(argc, argv, "dead-zone-list", "0.01"));
    options.ki_list = parse_double_list(cli_get(argc, argv, "ki-list", "0.0"));

    options.distance_m = std::stod(cli_get(argc, argv, "distance", "2.0"));
    options.speed_mps = std::stod(cli_get(argc, argv, "speed", "0.9"));
    options.linear_accel_mps2 = std::stod(cli_get(argc, argv, "linear-accel", "0.0"));
    options.linear_decel_mps2 = std::stod(cli_get(argc, argv, "linear-decel", "0.0"));
    options.accel_settle_s = std::stod(cli_get(argc, argv, "accel-settle", "0.5"));
    options.stop_s = std::stod(cli_get(argc, argv, "stop", "0.8"));
    options.imu_wait_s = std::stod(cli_get(argc, argv, "imu-wait", "4.0"));
    options.imu_stable_s = std::stod(cli_get(argc, argv, "imu-stable", "1.0"));
    options.reset_attitude_delay_s = std::stod(cli_get(argc, argv, "reset-attitude-delay", "0.5"));
    options.timeout_scale = std::stod(cli_get(argc, argv, "timeout-scale", "1.6"));
    options.timeout_ms = std::stod(cli_get(argc, argv, "timeout-ms", "100.0"));
    options.move_eps = std::stod(cli_get(argc, argv, "move-eps", "0.02"));
    options.turn_eps = std::stod(cli_get(argc, argv, "turn-eps", "0.05"));
    options.kf = std::stod(cli_get(argc, argv, "kf", "0.0"));
    options.i_out_max = std::stod(cli_get(argc, argv, "i-out-max", "0.0"));
    options.max_imu_age_ms = std::stod(cli_get(argc, argv, "max-imu-age-ms", "500.0"));
    options.max_odom_age_ms = std::stod(cli_get(argc, argv, "max-odom-age-ms", "500.0"));
    options.repeats = std::max(1, std::atoi(cli_get(argc, argv, "repeats", "2")));
    options.max_rounds = std::max(1, std::atoi(cli_get(argc, argv, "max-rounds", "40")));
    options.sample_hz = std::max(10, std::atoi(cli_get(argc, argv, "sample-hz", "100")));
    options.dry_run = cli_has(argc, argv, "dry-run");
    options.yes = cli_has(argc, argv, "yes") || cli_has(argc, argv, "assume-yes");
    options.set_params = !cli_has(argc, argv, "no-param-set");

    if (cli_has(argc, argv, "measured-offset"))
        options.measured_offset_m = std::stod(cli_get(argc, argv, "measured-offset", "nan"));

    if (options.kp_list.empty())
        options.kp_list = {1.5, 2.5, 3.5};
    if (options.kd_list.empty())
        options.kd_list = {0.03, 0.08, 0.15};
    if (options.out_scale_list.empty())
        options.out_scale_list = {0.6, 0.8};
    if (options.dead_zone_list.empty())
        options.dead_zone_list = {0.01};
    if (options.ki_list.empty())
        options.ki_list = {0.0};

    return options;
}

void print_help()
{
    std::cout
        << "omni_straight_line_autotune options:\n"
        << "  --distance 2.0                  straight segment distance in meters\n"
        << "  --speed 0.9                     straight command speed in m/s\n"
        << "  --linear-accel 0.0              command ramp accel limit in m/s^2; 0 keeps step command\n"
        << "  --linear-decel 0.0              command ramp decel limit in m/s^2; defaults to accel when 0\n"
        << "  --accel-settle 0.5              initial drive seconds ignored for scoring\n"
        << "  --directions fb|lr|all          default fb\n"
        << "  --repeats 2                     repeats per parameter/direction\n"
        << "  --imu-reset each|once|none      restart IMU node before each segment by default\n"
        << "  --imu-node IMU_publisher        process/node pattern used by pkill -f\n"
        << "  --imu-launch CMD                command relaunched with nohup after kill\n"
        << "  --bridge-node /r2_vehicle_bridge\n"
        << "  --cmd-topic /chassis/cmd_vel\n"
        << "  --imu-topic /IMU_data\n"
        << "  --odom-topic /chassis/odom_twist\n"
        << "  --output-dir var_data           creates timestamped child directory\n"
        << "  --search grid|coarse2fine       default coarse2fine\n"
        << "  --max-rounds 40                 candidate parameter evaluations limit\n"
        << "  --sample-hz 100                 sampling/publish rate\n"
        << "  --dry-run                       simulate IMU/odom and never touch the chassis\n"
        << "  --yes                           skip real-vehicle safety prompt\n"
        << "  --no-param-set                  do not write /r2_vehicle_bridge parameters\n"
        << "  --measured-offset M             optional tape-measured final offset for report\n";
}

std::vector<MotionSegment> build_segments(const Options &options)
{
    std::vector<MotionSegment> segments;
    const bool fb = options.directions == "fb" || options.directions == "all";
    const bool lr = options.directions == "lr" || options.directions == "all";

    auto add = [&](const std::string &group, const std::string &direction, double vx, double vy) {
        MotionSegment segment;
        segment.id = static_cast<int>(segments.size());
        segment.group = group;
        segment.direction = direction;
        segment.vx = vx;
        segment.vy = vy;
        segments.push_back(segment);
    };

    if (fb)
    {
        add("front_back", "forward", options.speed_mps, 0.0);
        add("front_back", "backward", -options.speed_mps, 0.0);
    }
    if (lr)
    {
        add("left_right", "left", 0.0, options.speed_mps);
        add("left_right", "right", 0.0, -options.speed_mps);
    }
    return segments;
}

ParamSet make_param(const Options &options,
                    double kp,
                    double ki,
                    double kd,
                    double out_scale,
                    double dead_zone)
{
    ParamSet param;
    param.kp = kp;
    param.ki = ki;
    param.kd = kd;
    param.kf = options.kf;
    param.out_limit_scale = out_scale;
    param.dead_zone = dead_zone;
    param.move_eps = options.move_eps;
    param.turn_eps = options.turn_eps;
    param.i_out_max = options.i_out_max;
    param.timeout_ms = options.timeout_ms;
    return param;
}

std::vector<ParamSet> build_grid_params(const Options &options)
{
    std::vector<ParamSet> params;
    for (double kp : options.kp_list)
        for (double ki : options.ki_list)
            for (double kd : options.kd_list)
                for (double out_scale : options.out_scale_list)
                    for (double dead_zone : options.dead_zone_list)
                        params.push_back(make_param(options, kp, ki, kd, out_scale, dead_zone));
    return params;
}

bool same_param(const ParamSet &a, const ParamSet &b)
{
    auto close = [](double x, double y) {
        return std::fabs(x - y) < 1e-9;
    };
    return close(a.kp, b.kp) &&
           close(a.ki, b.ki) &&
           close(a.kd, b.kd) &&
           close(a.out_limit_scale, b.out_limit_scale) &&
           close(a.dead_zone, b.dead_zone);
}

bool wait_for_fresh_imu(const std::shared_ptr<AutotuneNode> &node,
                        double wait_s,
                        int64_t newer_than_ns,
                        double max_age_ms)
{
    const int64_t start = now_ns();
    while (g_running.load() && rclcpp::ok())
    {
        rclcpp::spin_some(node);
        const int64_t ts = now_ns();
        const Latest latest = node->latest();
        if (latest.imu_valid &&
            latest.imu_ns > newer_than_ns &&
            static_cast<double>(ts - latest.imu_ns) / 1.0e6 <= max_age_ms)
        {
            return true;
        }
        if ((ts - start) > static_cast<int64_t>(wait_s * 1.0e9))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

void sleep_or_spin(const std::shared_ptr<AutotuneNode> &node,
                   double duration_s,
                   int sample_hz,
                   bool dry_run)
{
    if (dry_run || duration_s <= 0.0)
        return;

    const int64_t start = now_ns();
    const int64_t duration_ns = static_cast<int64_t>(duration_s * 1.0e9);
    const auto sleep_dt = std::chrono::duration<double>(1.0 / static_cast<double>(sample_hz));
    while (g_running.load() && rclcpp::ok() && (now_ns() - start) < duration_ns)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(sleep_dt);
    }
}

void hold_zero(const std::shared_ptr<AutotuneNode> &node,
               const Options &options,
               double duration_s)
{
    if (options.dry_run)
        return;

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

bool wait_still(const std::shared_ptr<AutotuneNode> &node,
                const Options &options,
                double max_wait_s)
{
    if (options.dry_run)
        return true;

    const int required_frames = std::max(5, options.sample_hz / 5);
    int still_frames = 0;
    const int64_t start = now_ns();
    const int64_t max_wait_ns = static_cast<int64_t>(max_wait_s * 1.0e9);
    const auto sleep_dt = std::chrono::duration<double>(1.0 / static_cast<double>(options.sample_hz));

    while (g_running.load() && rclcpp::ok() && (now_ns() - start) < max_wait_ns)
    {
        node->publish_zero();
        rclcpp::spin_some(node);
        const Latest latest = node->latest();
        if (latest.odom_valid)
        {
            const double speed = std::hypot(latest.odom_vx, latest.odom_vy);
            if (speed < 0.025 && std::fabs(latest.odom_omega) < 0.04)
                still_frames++;
            else
                still_frames = 0;

            if (still_frames >= required_frames)
                return true;
        }
        std::this_thread::sleep_for(sleep_dt);
    }
    return false;
}

bool restart_imu(const std::shared_ptr<AutotuneNode> &node,
                 const Options &options,
                 const std::string &run_dir,
                 int restart_index)
{
    if (options.dry_run || options.imu_reset == "none")
        return true;

    std::cout << "[omni_autotune] IMU reset " << restart_index
              << ": stopping chassis and restarting " << options.imu_node << "\n";

    hold_zero(node, options, options.stop_s);
    wait_still(node, options, std::max(0.5, options.stop_s + 0.7));

    const int64_t reset_start_ns = now_ns();
    node->clear_imu();

    const int killed = terminate_matching_processes(options.imu_node);
    std::cout << "[omni_autotune] terminated " << killed
              << " process(es) matching " << options.imu_node << "\n";

    sleep_or_spin(node, 0.7, options.sample_hz, false);

    const std::string log_path = run_dir + "/imu_restart_" + std::to_string(restart_index) + ".log";
    const std::string launch_cmd = "nohup " + options.imu_launch + " > " +
                                   shell_quote(log_path) + " 2>&1 &";
    const int launch_rc = std::system(launch_cmd.c_str());
    if (launch_rc != 0)
    {
        std::cerr << "[omni_autotune] failed to launch IMU command, rc=" << launch_rc
                  << " command=" << options.imu_launch << "\n";
        return false;
    }

    if (!wait_for_fresh_imu(node, options.imu_wait_s, reset_start_ns, options.max_imu_age_ms))
    {
        std::cerr << "[omni_autotune] no fresh IMU data after restart; see " << log_path << "\n";
        return false;
    }

    sleep_or_spin(node, options.reset_attitude_delay_s + options.imu_stable_s,
                  options.sample_hz, false);

    if (!wait_for_fresh_imu(node, 0.5, reset_start_ns, options.max_imu_age_ms))
    {
        std::cerr << "[omni_autotune] IMU became stale after reset settle\n";
        return false;
    }
    return true;
}

bool set_heading_params(const std::shared_ptr<AutotuneNode> &node,
                        const std::shared_ptr<rclcpp::AsyncParametersClient> &client,
                        const Options &options,
                        const ParamSet &param)
{
    if (options.dry_run || !options.set_params)
        return true;

    std::vector<rclcpp::Parameter> params;
    params.emplace_back("imu_heading_hold.enable", true);
    params.emplace_back("imu_heading_hold.kp", param.kp);
    params.emplace_back("imu_heading_hold.ki", param.ki);
    params.emplace_back("imu_heading_hold.kd", param.kd);
    params.emplace_back("imu_heading_hold.kf", param.kf);
    params.emplace_back("imu_heading_hold.i_out_max", param.i_out_max);
    params.emplace_back("imu_heading_hold.out_limit_scale", param.out_limit_scale);
    params.emplace_back("imu_heading_hold.move_eps", param.move_eps);
    params.emplace_back("imu_heading_hold.turn_eps", param.turn_eps);
    params.emplace_back("imu_heading_hold.dead_zone", param.dead_zone);
    params.emplace_back("imu_heading_hold.timeout_ms", param.timeout_ms);

    auto future = client->set_parameters(params);
    const auto rc = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(3));
    if (rc != rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cerr << "[omni_autotune] set_parameters timed out for param_id="
                  << param.id << "\n";
        return false;
    }

    const auto results = future.get();
    for (const auto &result : results)
    {
        if (!result.successful)
        {
            std::cerr << "[omni_autotune] parameter rejected: " << result.reason << "\n";
            return false;
        }
    }
    return true;
}

Latest dry_latest(const Options &options,
                  const ParamSet &param,
                  const MotionSegment &segment,
                  int64_t stamp_ns,
                  double elapsed_s,
                  double distance_m,
                  double commanded_speed_mps)
{
    const double speed = std::hypot(segment.vx, segment.vy);
    const double unit_x = speed > 1e-6 ? segment.vx / speed : 1.0;
    const double unit_y = speed > 1e-6 ? segment.vy / speed : 0.0;

    const double kp_err = param.kp - 3.20;
    const double kd_err = param.kd - 0.075;
    const double out_err = param.out_limit_scale - 0.76;
    const double dz_err = param.dead_zone - 0.008;
    const double param_penalty = 0.55 * std::fabs(kp_err) +
                                 5.0 * std::fabs(kd_err) +
                                 1.0 * std::fabs(out_err) +
                                 20.0 * std::fabs(dz_err);

    const double dir_sign = (segment.direction == "backward" || segment.direction == "right") ? -1.0 : 1.0;
    const double yaw_drift_deg = dir_sign * (0.10 + param_penalty) * distance_m;
    const double oscillation_deg =
        std::max(0.0, param.kp - 3.2) * 0.20 * std::sin(2.0 * kPi * 1.2 * elapsed_s) +
        std::max(0.0, param.kd - 0.10) * 3.0 * std::sin(2.0 * kPi * 2.1 * elapsed_s);
    const double yaw_deg = yaw_drift_deg + oscillation_deg;

    const double cross_speed = dir_sign * (0.003 + 0.006 * param_penalty);
    const double forward_speed = commanded_speed_mps;

    Latest latest;
    latest.imu_valid = true;
    latest.imu_ns = stamp_ns;
    latest.yaw = deg_to_rad(yaw_deg);
    latest.gyro_z = deg_to_rad(dir_sign * (0.10 + param_penalty) * forward_speed +
                                1.5 * oscillation_deg);
    latest.odom_valid = true;
    latest.odom_ns = stamp_ns;
    latest.odom_vx = forward_speed * unit_x - cross_speed * unit_y;
    latest.odom_vy = forward_speed * unit_y + cross_speed * unit_x;
    latest.odom_omega = latest.gyro_z;
    return latest;
}

void write_samples_header(std::ofstream &csv)
{
    csv << "timestamp_ns,segment_id,param_id,repeat,direction,elapsed_s,"
        << "kp,ki,kd,kf,out_limit_scale,dead_zone,move_eps,turn_eps,i_out_max,"
        << "vx_target,vy_target,vx_cmd,vy_cmd,cmd_speed_mps,yaw_rad,yaw_rel_deg,omega_z_rad_s,omega_z_deg_s,"
        << "odom_vx,odom_vy,odom_omega,forward_speed,cross_speed,s_m,lateral_m,"
        << "used_for_score,imu_age_ms,odom_age_ms\n";
}

void write_segments_header(std::ofstream &csv)
{
    csv << "segment_id,param_id,repeat,group,direction,vx_cmd,vy_cmd,"
        << "kp,ki,kd,kf,out_limit_scale,dead_zone,move_eps,turn_eps,i_out_max,"
        << "samples,used_samples,success,failure_reason,duration_s,distance_m,lateral_offset_m,"
        << "yaw_rms_deg,yaw_final_abs_deg,cross_rms_mps,omega_rms_deg_s,overshoot_deg,score\n";
}

void write_segment_row(std::ofstream &csv, const SegmentResult &result)
{
    csv << result.segment.id << ','
        << result.param.id << ','
        << result.repeat << ','
        << result.segment.group << ','
        << result.segment.direction << ','
        << result.segment.vx << ','
        << result.segment.vy << ','
        << result.param.kp << ','
        << result.param.ki << ','
        << result.param.kd << ','
        << result.param.kf << ','
        << result.param.out_limit_scale << ','
        << result.param.dead_zone << ','
        << result.param.move_eps << ','
        << result.param.turn_eps << ','
        << result.param.i_out_max << ','
        << result.samples << ','
        << result.used_samples << ','
        << (result.success ? 1 : 0) << ','
        << result.failure_reason << ','
        << result.duration_s << ','
        << result.distance_m << ','
        << result.lateral_offset_m << ','
        << result.yaw_error_deg.rms() << ','
        << result.yaw_final_abs_deg << ','
        << result.cross_speed_mps.rms() << ','
        << result.omega_deg_s.rms() << ','
        << result.overshoot_deg << ','
        << result.score << '\n';
}

void write_history_header(std::ofstream &csv)
{
    csv << "round,phase,param_id,kp,ki,kd,kf,out_limit_scale,dead_zone,move_eps,turn_eps,i_out_max,"
        << "aggregate_score,segment_score_mean,segment_score_std,segments_scored,failures,"
        << "yaw_rms_deg_mean,yaw_final_abs_deg_mean,lateral_abs_m_mean,cross_rms_mps_mean,"
        << "omega_rms_deg_s_mean,overshoot_deg_mean,current_best_id,current_best_score,"
        << "current_best_kp,current_best_ki,current_best_kd,current_best_out_limit_scale,"
        << "current_best_dead_zone\n";
}

void write_history_row(std::ofstream &csv,
                       const Evaluation &eval,
                       const Evaluation *best)
{
    csv << eval.round << ','
        << eval.phase << ','
        << eval.param.id << ','
        << eval.param.kp << ','
        << eval.param.ki << ','
        << eval.param.kd << ','
        << eval.param.kf << ','
        << eval.param.out_limit_scale << ','
        << eval.param.dead_zone << ','
        << eval.param.move_eps << ','
        << eval.param.turn_eps << ','
        << eval.param.i_out_max << ','
        << eval.aggregate_score << ','
        << eval.segment_score.mean() << ','
        << eval.segment_score.stddev() << ','
        << eval.segment_score.n << ','
        << eval.failures << ','
        << eval.yaw_rms_deg.mean() << ','
        << eval.yaw_final_abs_deg.mean() << ','
        << eval.lateral_abs_m.mean() << ','
        << eval.cross_rms_mps.mean() << ','
        << eval.omega_rms_deg_s.mean() << ','
        << eval.overshoot_deg.mean() << ',';

    if (best)
    {
        csv << best->param.id << ','
            << best->aggregate_score << ','
            << best->param.kp << ','
            << best->param.ki << ','
            << best->param.kd << ','
            << best->param.out_limit_scale << ','
            << best->param.dead_zone << '\n';
    }
    else
    {
        csv << "-1,inf,0,0,0,0,0\n";
    }
}

SegmentResult run_segment(const std::shared_ptr<AutotuneNode> &node,
                          std::ofstream &samples_csv,
                          const Options &options,
                          const ParamSet &param,
                          const MotionSegment &segment,
                          int repeat)
{
    SegmentResult result;
    result.param = param;
    result.segment = segment;
    result.repeat = repeat;

    const double command_speed = std::hypot(segment.vx, segment.vy);
    const double unit_x = command_speed > 1e-6 ? segment.vx / command_speed : 1.0;
    const double unit_y = command_speed > 1e-6 ? segment.vy / command_speed : 0.0;
    const double dt_s = 1.0 / static_cast<double>(options.sample_hz);
    const auto sleep_dt = std::chrono::duration<double>(dt_s);
    const double accel_limit = std::max(0.0, options.linear_accel_mps2);
    const double decel_limit =
        options.linear_decel_mps2 > 0.0 ? options.linear_decel_mps2 : accel_limit;
    const double accel_time_allowance = accel_limit > 1e-6 ? command_speed / accel_limit : 0.0;
    const double decel_time_allowance = decel_limit > 1e-6 ? command_speed / decel_limit : 0.0;
    const double hard_timeout_s =
        std::max(0.2,
                 options.distance_m / std::max(0.05, command_speed) * options.timeout_scale +
                 accel_time_allowance + decel_time_allowance);
    const int64_t hard_timeout_ns = static_cast<int64_t>(hard_timeout_s * 1.0e9);

    if (!options.dry_run)
        hold_zero(node, options, std::min(0.25, options.stop_s));

    const int64_t start_ns = now_ns();
    const Latest start_latest = options.dry_run ? Latest{} : node->latest();
    const double yaw_ref = (options.dry_run || !start_latest.imu_valid) ? 0.0 : start_latest.yaw;

    int64_t last_sample_ns = start_ns;
    int64_t last_command_ns = start_ns;
    double distance = 0.0;
    double lateral = 0.0;
    double final_yaw_rel_deg = 0.0;
    double cmd_speed = accel_limit > 1e-6 ? 0.0 : command_speed;
    double cmd_vx = accel_limit > 1e-6 ? 0.0 : segment.vx;
    double cmd_vy = accel_limit > 1e-6 ? 0.0 : segment.vy;
    bool timeout = false;
    bool stale_imu = false;

    for (int dry_i = 0; g_running.load() && rclcpp::ok(); ++dry_i)
    {
        int64_t sample_ns = now_ns();
        double elapsed_s = static_cast<double>(sample_ns - start_ns) / 1.0e9;
        if (options.dry_run)
        {
            sample_ns = start_ns + static_cast<int64_t>(dry_i * dt_s * 1.0e9);
            elapsed_s = dry_i * dt_s;
        }

        if (!options.dry_run && (sample_ns - start_ns) > hard_timeout_ns)
        {
            timeout = true;
            break;
        }
        if (options.dry_run && elapsed_s > hard_timeout_s)
        {
            timeout = true;
            break;
        }

        const double command_dt = options.dry_run ?
            dt_s : std::max(0.0, static_cast<double>(sample_ns - last_command_ns) / 1.0e9);
        last_command_ns = sample_ns;

        double target_speed = command_speed;
        if (decel_limit > 1e-6)
        {
            const double remaining = std::max(0.0, options.distance_m - distance);
            target_speed = std::min(target_speed, std::sqrt(2.0 * decel_limit * remaining));
        }

        if (accel_limit > 1e-6 || decel_limit > 1e-6)
        {
            const double limit = target_speed >= cmd_speed ?
                std::max(accel_limit, 1e-6) : std::max(decel_limit, 1e-6);
            const double max_step = limit * command_dt;
            const double delta = target_speed - cmd_speed;
            if (std::fabs(delta) <= max_step)
                cmd_speed = target_speed;
            else
                cmd_speed += std::copysign(max_step, delta);
            cmd_speed = clamp(cmd_speed, 0.0, command_speed);
            cmd_vx = unit_x * cmd_speed;
            cmd_vy = unit_y * cmd_speed;
        }

        node->publish_command(options.dry_run ? 0.0 : cmd_vx,
                              options.dry_run ? 0.0 : cmd_vy,
                              0.0);
        if (!options.dry_run)
            rclcpp::spin_some(node);

        if (!options.dry_run)
        {
            sample_ns = now_ns();
            elapsed_s = static_cast<double>(sample_ns - start_ns) / 1.0e9;
        }

        const double local_distance = options.dry_run ?
            std::min(options.distance_m, distance) : distance;
        const Latest latest = options.dry_run ?
            dry_latest(options, param, segment, sample_ns, elapsed_s, local_distance, cmd_speed) :
            node->latest();

        const double dt = options.dry_run ?
            dt_s : std::max(0.0, static_cast<double>(sample_ns - last_sample_ns) / 1.0e9);
        last_sample_ns = sample_ns;

        const double imu_age_ms = latest.imu_valid ?
            static_cast<double>(sample_ns - latest.imu_ns) / 1.0e6 : -1.0;
        const double odom_age_ms = latest.odom_valid ?
            static_cast<double>(sample_ns - latest.odom_ns) / 1.0e6 : -1.0;
        const bool imu_fresh = latest.imu_valid && imu_age_ms >= 0.0 && imu_age_ms <= options.max_imu_age_ms;
        const bool odom_fresh = latest.odom_valid && odom_age_ms >= 0.0 && odom_age_ms <= options.max_odom_age_ms;

        if (!imu_fresh && elapsed_s > 0.2)
        {
            stale_imu = true;
            break;
        }

        double forward_speed = 0.0;
        double cross_speed = 0.0;
        if (odom_fresh)
        {
            forward_speed = latest.odom_vx * unit_x + latest.odom_vy * unit_y;
            cross_speed = -latest.odom_vx * unit_y + latest.odom_vy * unit_x;
            distance += std::max(0.0, forward_speed) * dt;
            lateral += cross_speed * dt;
        }

        const double yaw_rel_rad = imu_fresh ? normalize_angle(latest.yaw - yaw_ref) : 0.0;
        const double yaw_rel_deg = rad_to_deg(yaw_rel_rad);
        final_yaw_rel_deg = yaw_rel_deg;

        const bool used = imu_fresh &&
                          odom_fresh &&
                          elapsed_s >= options.accel_settle_s &&
                          distance <= options.distance_m * 0.98;

        result.samples++;
        if (used)
        {
            result.used_samples++;
            result.yaw_error_deg.add(yaw_rel_deg);
            result.yaw_abs_deg.add(std::fabs(yaw_rel_deg));
            result.cross_speed_mps.add(cross_speed);
            result.omega_deg_s.add(std::fabs(rad_to_deg(latest.gyro_z)));
            result.max_yaw_deg = std::max(result.max_yaw_deg, yaw_rel_deg);
            result.min_yaw_deg = std::min(result.min_yaw_deg, yaw_rel_deg);
        }

        samples_csv << sample_ns << ','
                    << segment.id << ','
                    << param.id << ','
                    << repeat << ','
                    << segment.direction << ','
                    << elapsed_s << ','
                    << param.kp << ','
                    << param.ki << ','
                    << param.kd << ','
                    << param.kf << ','
                    << param.out_limit_scale << ','
                    << param.dead_zone << ','
                    << param.move_eps << ','
                    << param.turn_eps << ','
                    << param.i_out_max << ','
                    << segment.vx << ','
                    << segment.vy << ','
                    << cmd_vx << ','
                    << cmd_vy << ','
                    << cmd_speed << ','
                    << (imu_fresh ? latest.yaw : 0.0) << ','
                    << yaw_rel_deg << ','
                    << (imu_fresh ? latest.gyro_z : 0.0) << ','
                    << (imu_fresh ? rad_to_deg(latest.gyro_z) : 0.0) << ','
                    << (odom_fresh ? latest.odom_vx : 0.0) << ','
                    << (odom_fresh ? latest.odom_vy : 0.0) << ','
                    << (odom_fresh ? latest.odom_omega : 0.0) << ','
                    << forward_speed << ','
                    << cross_speed << ','
                    << distance << ','
                    << lateral << ','
                    << (used ? 1 : 0) << ','
                    << imu_age_ms << ','
                    << odom_age_ms << '\n';

        if (distance >= options.distance_m &&
            (decel_limit <= 1e-6 || cmd_speed <= std::max(0.05, command_speed * 0.05)))
            break;

        if (!options.dry_run)
            std::this_thread::sleep_for(sleep_dt);
    }

    node->publish_zero();
    if (!options.dry_run)
        hold_zero(node, options, options.stop_s);

    result.duration_s = options.dry_run ?
        result.samples * dt_s : static_cast<double>(now_ns() - start_ns) / 1.0e9;
    result.distance_m = distance;
    result.lateral_offset_m = lateral;
    result.yaw_final_abs_deg = std::fabs(final_yaw_rel_deg);
    if (timeout)
        result.failure_reason = "hard_timeout";
    if (stale_imu)
        result.failure_reason = "imu_stale";
    if (distance < options.distance_m * 0.90 && result.failure_reason.empty())
        result.failure_reason = "distance_not_reached";

    result.finalize();
    return result;
}

Evaluation evaluate_param(const std::shared_ptr<AutotuneNode> &node,
                          const std::shared_ptr<rclcpp::AsyncParametersClient> &param_client,
                          std::ofstream &samples_csv,
                          std::ofstream &segments_csv,
                          const Options &options,
                          const std::vector<MotionSegment> &segments,
                          const std::string &run_dir,
                          ParamSet param,
                          int round,
                          const std::string &phase,
                          int &restart_count)
{
    Evaluation eval;
    eval.round = round;
    eval.phase = phase;
    eval.param = param;

    int consecutive_failures = 0;
    for (const MotionSegment &segment : segments)
    {
        for (int repeat = 0; repeat < options.repeats; ++repeat)
        {
            if (!g_running.load() || !rclcpp::ok())
                break;

            if (options.imu_reset == "each")
            {
                restart_count++;
                if (!restart_imu(node, options, run_dir, restart_count))
                {
                    eval.failures++;
                    consecutive_failures++;
                    if (consecutive_failures >= 3)
                        return eval;
                    continue;
                }
            }
            else
            {
                hold_zero(node, options, options.stop_s);
            }

            if (!set_heading_params(node, param_client, options, param))
            {
                eval.failures++;
                consecutive_failures++;
                if (consecutive_failures >= 3)
                    return eval;
                continue;
            }

            std::cout << "[omni_autotune] round=" << round
                      << " phase=" << phase
                      << " param_id=" << param.id
                      << " dir=" << segment.direction
                      << " repeat=" << (repeat + 1) << "/" << options.repeats
                      << " kp=" << param.kp
                      << " kd=" << param.kd
                      << " out=" << param.out_limit_scale
                      << " dead=" << param.dead_zone
                      << "\n";

            SegmentResult result = run_segment(node, samples_csv, options, param, segment, repeat);
            write_segment_row(segments_csv, result);
            samples_csv.flush();
            segments_csv.flush();

            if (result.success && std::isfinite(result.score))
            {
                consecutive_failures = 0;
                eval.segment_score.add(result.score);
                eval.yaw_rms_deg.add(result.yaw_error_deg.rms());
                eval.yaw_final_abs_deg.add(result.yaw_final_abs_deg);
                eval.lateral_abs_m.add(std::fabs(result.lateral_offset_m));
                eval.cross_rms_mps.add(result.cross_speed_mps.rms());
                eval.omega_rms_deg_s.add(result.omega_deg_s.rms());
                eval.overshoot_deg.add(result.overshoot_deg);
            }
            else
            {
                eval.failures++;
                consecutive_failures++;
                if (consecutive_failures >= 3)
                    return eval;
            }
        }
    }

    if (eval.segment_score.n > 0)
    {
        eval.aggregate_score = eval.segment_score.mean() + 0.5 * eval.segment_score.stddev();
        eval.success = std::isfinite(eval.aggregate_score);
    }
    return eval;
}

void write_best_yaml(const std::string &path, const ParamSet &param)
{
    std::ofstream out(path);
    out << std::fixed << std::setprecision(6);
    out << "r2_vehicle_bridge:\n";
    out << "  ros__parameters:\n";
    out << "    imu_heading_hold.enable: true\n";
    out << "    imu_heading_hold.kp: " << param.kp << "\n";
    out << "    imu_heading_hold.ki: " << param.ki << "\n";
    out << "    imu_heading_hold.kd: " << param.kd << "\n";
    out << "    imu_heading_hold.kf: " << param.kf << "\n";
    out << "    imu_heading_hold.i_out_max: " << param.i_out_max << "\n";
    out << "    imu_heading_hold.out_limit_scale: " << param.out_limit_scale << "\n";
    out << "    imu_heading_hold.dead_zone: " << param.dead_zone << "\n";
    out << "    imu_heading_hold.move_eps: " << param.move_eps << "\n";
    out << "    imu_heading_hold.turn_eps: " << param.turn_eps << "\n";
    out << "    imu_heading_hold.timeout_ms: " << param.timeout_ms << "\n";
}

void write_report(const std::string &path,
                  const Options &options,
                  const Evaluation *best,
                  const std::vector<Evaluation> &history,
                  const std::string &samples_path,
                  const std::string &segments_path,
                  const std::string &history_path,
                  const std::string &yaml_path)
{
    std::ofstream out(path);
    out << std::fixed << std::setprecision(6);
    out << "# Omni Straight-Line Autotune Report\n\n";
    out << "This run tunes the B-layer `imu_heading_hold.*` controller only. "
        << "A-layer per-wheel speed correction is still compile-time state and was not changed.\n\n";
    out << "## Inputs\n\n";
    out << "- distance_m: " << options.distance_m << "\n";
    out << "- speed_mps: " << options.speed_mps << "\n";
    out << "- linear_accel_mps2: " << options.linear_accel_mps2 << "\n";
    out << "- linear_decel_mps2: "
        << (options.linear_decel_mps2 > 0.0 ? options.linear_decel_mps2 : options.linear_accel_mps2)
        << "\n";
    out << "- directions: " << options.directions << "\n";
    out << "- repeats: " << options.repeats << "\n";
    out << "- imu_reset: " << options.imu_reset << "\n";
    out << "- search: " << options.search << "\n";
    out << "- max_rounds: " << options.max_rounds << "\n";
    out << "- score: yaw_rms_deg + 0.5*yaw_final_abs_deg + 40*abs(lateral_offset_m) "
        << "+ 15*cross_rms_mps + 0.05*omega_rms_deg_s + 0.3*overshoot_deg\n\n";
    out << "## Outputs\n\n";
    out << "- samples_csv: " << samples_path << "\n";
    out << "- segments_csv: " << segments_path << "\n";
    out << "- search_history_csv: " << history_path << "\n";
    out << "- best_params_yaml: " << yaml_path << "\n\n";

    if (best && best->success)
    {
        out << "## Best Parameters\n\n";
        out << "- param_id: " << best->param.id << "\n";
        out << "- aggregate_score: " << best->aggregate_score << "\n";
        out << "- imu_heading_hold.kp: " << best->param.kp << "\n";
        out << "- imu_heading_hold.ki: " << best->param.ki << "\n";
        out << "- imu_heading_hold.kd: " << best->param.kd << "\n";
        out << "- imu_heading_hold.kf: " << best->param.kf << "\n";
        out << "- imu_heading_hold.i_out_max: " << best->param.i_out_max << "\n";
        out << "- imu_heading_hold.out_limit_scale: " << best->param.out_limit_scale << "\n";
        out << "- imu_heading_hold.dead_zone: " << best->param.dead_zone << "\n";
        out << "- imu_heading_hold.move_eps: " << best->param.move_eps << "\n";
        out << "- imu_heading_hold.turn_eps: " << best->param.turn_eps << "\n";
        out << "- imu_heading_hold.timeout_ms: " << best->param.timeout_ms << "\n\n";

        out << "## Best Metrics\n\n";
        out << "- segment_score_mean: " << best->segment_score.mean() << "\n";
        out << "- segment_score_std: " << best->segment_score.stddev() << "\n";
        out << "- yaw_rms_deg_mean: " << best->yaw_rms_deg.mean() << "\n";
        out << "- yaw_final_abs_deg_mean: " << best->yaw_final_abs_deg.mean() << "\n";
        out << "- lateral_abs_m_mean: " << best->lateral_abs_m.mean() << "\n";
        out << "- cross_rms_mps_mean: " << best->cross_rms_mps.mean() << "\n";
        out << "- omega_rms_deg_s_mean: " << best->omega_rms_deg_s.mean() << "\n";
        out << "- overshoot_deg_mean: " << best->overshoot_deg.mean() << "\n\n";
    }
    else
    {
        out << "No valid best parameter set was found. Check IMU, odom, parameter service, and safety aborts.\n\n";
    }

    if (std::isfinite(options.measured_offset_m))
    {
        out << "## Measured Offset\n\n";
        out << "Operator measured final lateral offset: " << options.measured_offset_m << " m\n\n";
    }

    out << "## Notes\n\n";
    out << "- IMU zeroing is implemented by stopping the chassis, terminating processes matching "
        << "the configured IMU pattern, and relaunching `--imu-launch`; there is no runtime IMU reset service in this stack.\n";
    out << "- The lateral offset is integrated from `/chassis/odom_twist`; verify the final 2 m endpoint with tape "
        << "or a wall reference before adopting the YAML permanently.\n";
    out << "- If closed-loop heading hold still needs large correction, expose and tune per-wheel "
        << "`wheel_speed_correction` as the A-layer fix described in the SPEC.\n\n";

    out << "## Search History Summary\n\n";
    out << "round,phase,param_id,aggregate_score,kp,ki,kd,out_limit_scale,dead_zone,segments,failures\n";
    for (const Evaluation &eval : history)
    {
        out << eval.round << ','
            << eval.phase << ','
            << eval.param.id << ','
            << eval.aggregate_score << ','
            << eval.param.kp << ','
            << eval.param.ki << ','
            << eval.param.kd << ','
            << eval.param.out_limit_scale << ','
            << eval.param.dead_zone << ','
            << eval.segment_score.n << ','
            << eval.failures << "\n";
    }
}

bool ask_safety_confirmation(const Options &options)
{
    if (options.dry_run || options.yes)
        return true;

    std::cout << "\n[omni_autotune] SAFETY CHECK\n"
              << "  Required clear space: straight line >= 2.5 m plus side clearance.\n"
              << "  The tool will command automatic 2 m moves and may restart the IMU node.\n"
              << "  Keep E-stop/disable ready. Type YES to continue: ";
    std::string answer;
    std::getline(std::cin, answer);
    return answer == "YES";
}

bool better_than(const Evaluation &candidate, const Evaluation *best)
{
    return candidate.success &&
           std::isfinite(candidate.aggregate_score) &&
           (!best || !best->success || candidate.aggregate_score < best->aggregate_score);
}

ParamSet adjusted_param(const ParamSet &base, const std::string &dim, double delta)
{
    ParamSet next = base;
    if (dim == "kp")
        next.kp = clamp(next.kp + delta, 0.0, 8.0);
    else if (dim == "kd")
        next.kd = clamp(next.kd + delta, 0.0, 0.5);
    else if (dim == "out")
        next.out_limit_scale = clamp(next.out_limit_scale + delta, 0.1, 1.2);
    else if (dim == "dead")
        next.dead_zone = clamp(next.dead_zone + delta, 0.0, 0.08);
    else if (dim == "ki")
    {
        next.ki = clamp(next.ki + delta, 0.0, 0.20);
        if (next.ki > 0.0 && next.i_out_max <= 0.0)
            next.i_out_max = 0.20;
    }
    return next;
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
    if (options.distance_m <= 0.0 || options.speed_mps <= 0.0)
    {
        std::cerr << "[omni_autotune] --distance and --speed must be > 0\n";
        return 1;
    }
    if (options.directions != "fb" && options.directions != "lr" && options.directions != "all")
    {
        std::cerr << "[omni_autotune] --directions must be fb, lr, or all\n";
        return 1;
    }
    if (options.imu_reset != "each" && options.imu_reset != "once" && options.imu_reset != "none")
    {
        std::cerr << "[omni_autotune] --imu-reset must be each, once, or none\n";
        return 1;
    }
    if (options.search != "grid" && options.search != "coarse2fine")
    {
        std::cerr << "[omni_autotune] --search must be grid or coarse2fine\n";
        return 1;
    }
    if (!ask_safety_confirmation(options))
    {
        std::cerr << "[omni_autotune] aborted by operator\n";
        return 2;
    }

    const std::vector<MotionSegment> segments = build_segments(options);
    if (segments.empty())
    {
        std::cerr << "[omni_autotune] no motion segments selected\n";
        return 1;
    }

    const std::string stamp = timestamp_string();
    if (!ensure_dir_recursive(options.output_dir))
    {
        std::cerr << "[omni_autotune] cannot create output dir: " << options.output_dir << "\n";
        return 1;
    }
    const std::string run_dir = options.output_dir + "/omni_straight_line_autotune_" + stamp;
    if (!ensure_dir_recursive(run_dir))
    {
        std::cerr << "[omni_autotune] cannot create run dir: " << run_dir << "\n";
        return 1;
    }

    const std::string samples_path = run_dir + "/samples_" + stamp + ".csv";
    const std::string segments_path = run_dir + "/segments_" + stamp + ".csv";
    const std::string history_path = run_dir + "/search_history_" + stamp + ".csv";
    const std::string yaml_path = run_dir + "/best_params_" + stamp + ".yaml";
    const std::string report_path = run_dir + "/report_" + stamp + ".md";

    std::ofstream samples_csv(samples_path);
    std::ofstream segments_csv(segments_path);
    std::ofstream history_csv(history_path);
    if (!samples_csv || !segments_csv || !history_csv)
    {
        std::cerr << "[omni_autotune] cannot open output files in " << run_dir << "\n";
        return 1;
    }
    samples_csv << std::fixed << std::setprecision(9);
    segments_csv << std::fixed << std::setprecision(9);
    history_csv << std::fixed << std::setprecision(9);
    write_samples_header(samples_csv);
    write_segments_header(segments_csv);
    write_history_header(history_csv);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutotuneNode>(options);
    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, options.bridge_node);

    if (!options.dry_run)
    {
        std::cout << "[omni_autotune] waiting for IMU topic " << options.imu_topic << "\n";
        if (!wait_for_fresh_imu(node, options.imu_wait_s, 0, options.max_imu_age_ms))
        {
            std::cerr << "[omni_autotune] no fresh IMU samples; use --dry-run for logic-only testing\n";
            node->publish_zero();
            rclcpp::shutdown();
            return 3;
        }

        if (options.set_params)
        {
            std::cout << "[omni_autotune] waiting for parameter service " << options.bridge_node << "\n";
            if (!param_client->wait_for_service(std::chrono::seconds(8)))
            {
                std::cerr << "[omni_autotune] parameter service unavailable\n";
                node->publish_zero();
                rclcpp::shutdown();
                return 4;
            }
        }
    }

    int restart_count = 0;
    if (!options.dry_run && options.imu_reset == "once")
    {
        restart_count++;
        if (!restart_imu(node, options, run_dir, restart_count))
        {
            node->publish_zero();
            rclcpp::shutdown();
            return 5;
        }
    }

    std::vector<Evaluation> history;
    Evaluation best;
    Evaluation *best_ptr = nullptr;
    int round = 0;
    int next_param_id = 0;

    auto evaluate_and_record = [&](ParamSet param, const std::string &phase) -> bool {
        if (!g_running.load() || !rclcpp::ok())
            return false;
        if (round >= options.max_rounds)
            return false;
        for (const Evaluation &existing : history)
        {
            if (same_param(existing.param, param))
                return true;
        }

        param.id = next_param_id++;
        round++;
        Evaluation eval = evaluate_param(node, param_client, samples_csv, segments_csv,
                                         options, segments, run_dir, param, round, phase,
                                         restart_count);
        history.push_back(eval);
        if (better_than(history.back(), best_ptr))
        {
            best = history.back();
            best_ptr = &best;
        }
        write_history_row(history_csv, history.back(), best_ptr);
        history_csv.flush();

        if (best_ptr)
        {
            std::cout << "[omni_autotune] current best param_id=" << best.param.id
                      << " score=" << best.aggregate_score
                      << " kp=" << best.param.kp
                      << " ki=" << best.param.ki
                      << " kd=" << best.param.kd
                      << " out=" << best.param.out_limit_scale
                      << " dead=" << best.param.dead_zone
                      << "\n";
        }
        return true;
    };

    std::cout << "[omni_autotune] output_dir=" << run_dir << "\n";
    std::cout << "[omni_autotune] search=" << options.search
              << " max_rounds=" << options.max_rounds
              << " dry_run=" << (options.dry_run ? 1 : 0)
              << "\n";

    const std::vector<ParamSet> coarse_params = build_grid_params(options);
    for (const ParamSet &param : coarse_params)
    {
        if (!evaluate_and_record(param, "coarse"))
            break;
    }

    if (options.search == "coarse2fine" && best_ptr && round < options.max_rounds)
    {
        double kp_step = 0.50;
        double kd_step = 0.04;
        double out_step = 0.10;
        double dead_step = 0.005;

        for (int pass = 0; pass < 8 && round < options.max_rounds && g_running.load(); ++pass)
        {
            bool improved_this_pass = false;
            const std::vector<std::pair<std::string, double>> dims {
                {"kp", kp_step},
                {"kd", kd_step},
                {"out", out_step},
                {"dead", dead_step},
            };

            for (const auto &dim : dims)
            {
                if (round >= options.max_rounds)
                    break;

                const double before = best_ptr ? best_ptr->aggregate_score : std::numeric_limits<double>::infinity();
                ParamSet plus = adjusted_param(best.param, dim.first, dim.second);
                ParamSet minus = adjusted_param(best.param, dim.first, -dim.second);
                evaluate_and_record(plus, "fine_" + dim.first);
                if (round >= options.max_rounds)
                    break;
                evaluate_and_record(minus, "fine_" + dim.first);

                if (best_ptr && best_ptr->aggregate_score + 0.03 < before)
                    improved_this_pass = true;
            }

            if (!improved_this_pass)
            {
                kp_step *= 0.5;
                kd_step *= 0.5;
                out_step *= 0.5;
                dead_step *= 0.5;
                if (kp_step < 0.08 && kd_step < 0.006 && out_step < 0.02 && dead_step < 0.001)
                    break;
            }
        }

        if (best_ptr && round < options.max_rounds &&
            best.yaw_final_abs_deg.mean() > 1.0 &&
            best.yaw_final_abs_deg.mean() > best.yaw_rms_deg.mean() * 1.2)
        {
            ParamSet with_ki = best.param;
            with_ki.ki = std::max(0.01, with_ki.ki + 0.02);
            with_ki.i_out_max = std::max(0.10, with_ki.i_out_max);
            evaluate_and_record(with_ki, "ki_probe");
        }
    }

    node->publish_zero();
    hold_zero(node, options, std::max(0.3, options.stop_s));

    if (best_ptr && best.success)
        write_best_yaml(yaml_path, best.param);
    else
        write_best_yaml(yaml_path, make_param(options, 3.5, 0.0, 0.03, 0.8, 0.01));
    write_report(report_path, options, best_ptr, history,
                 samples_path, segments_path, history_path, yaml_path);

    std::cout << "[omni_autotune] complete\n";
    if (best_ptr && best.success)
    {
        std::cout << "[omni_autotune] best param_id=" << best.param.id
                  << " score=" << best.aggregate_score
                  << " kp=" << best.param.kp
                  << " ki=" << best.param.ki
                  << " kd=" << best.param.kd
                  << " out=" << best.param.out_limit_scale
                  << " dead=" << best.param.dead_zone
                  << "\n";
    }
    else
    {
        std::cout << "[omni_autotune] no valid best parameter set\n";
    }
    std::cout << "[omni_autotune] samples=" << samples_path << "\n";
    std::cout << "[omni_autotune] segments=" << segments_path << "\n";
    std::cout << "[omni_autotune] history=" << history_path << "\n";
    std::cout << "[omni_autotune] yaml=" << yaml_path << "\n";
    std::cout << "[omni_autotune] report=" << report_path << "\n";

    rclcpp::shutdown();
    return best_ptr && best.success ? 0 : 6;
}

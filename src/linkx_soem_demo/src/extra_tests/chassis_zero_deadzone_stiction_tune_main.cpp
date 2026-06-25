// Chassis zero-deadzone stiction tuning tool.
//
// This is a ROS2 topic-level test harness for the omni chassis deadzone and
// breakaway problem. It publishes small velocity steps, records chassis odom
// feedback, detects the first observable movement, and writes reproducible CSV,
// YAML, and Markdown artifacts. The online tuning mode can set the stage-two
// per-wheel parameters when r2_vehicle_bridge exposes them.

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
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace
{
constexpr int kWheelCount = 4;
constexpr double kNsPerS = 1.0e9;
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
    if (path.empty())
        return false;

    struct stat st {};
    if (stat(path.c_str(), &st) == 0)
        return S_ISDIR(st.st_mode);
    return mkdir(path.c_str(), 0755) == 0;
}

bool ensure_dir_recursive(const std::string &path)
{
    if (path.empty())
        return false;

    std::string current;
    if (path[0] == '/')
        current = "/";

    std::stringstream ss(path);
    std::string part;
    while (std::getline(ss, part, '/'))
    {
        if (part.empty())
            continue;
        if (!current.empty() && current.back() != '/')
            current += '/';
        current += part;
        if (!ensure_dir(current))
            return false;
    }
    return true;
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

double parse_double(const char *text, double fallback)
{
    if (text == nullptr || text[0] == '\0')
        return fallback;
    char *end = nullptr;
    const double value = std::strtod(text, &end);
    return (end == text || !std::isfinite(value)) ? fallback : value;
}

int parse_int(const char *text, int fallback)
{
    if (text == nullptr || text[0] == '\0')
        return fallback;
    char *end = nullptr;
    const long value = std::strtol(text, &end, 10);
    return (end == text) ? fallback : static_cast<int>(value);
}

std::vector<double> parse_double_list(const std::string &text,
                                      const std::vector<double> &fallback)
{
    std::vector<double> values;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        char *end = nullptr;
        const double value = std::strtod(item.c_str(), &end);
        if (end != item.c_str() && std::isfinite(value))
            values.push_back(value);
    }
    return values.empty() ? fallback : values;
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
        const double var = sum_sq / static_cast<double>(n) - m * m;
        return std::sqrt(std::max(0.0, var));
    }
};

struct ParameterSet
{
    double joystick_deadzone = 0.01;
    std::array<double, kWheelCount> omega_deadzone {0.05, 0.05, 0.05, 0.05};
    std::array<double, kWheelCount> stiction_torque {0.1300, 0.2400, 0.4800, 0.3000};
    std::array<double, kWheelCount> dynamic_friction {0.2960, 0.3962, 0.4301, 0.2767};
    std::array<double, kWheelCount> feedforward_scale {0.5842, 0.5472, 1.1088, 0.5769};
    std::array<double, kWheelCount> breakaway_torque {0.85, 1.10, 0.45, 0.60};
};

struct Options
{
    std::string mode = "joystick-floor";
    std::string cmd_topic = "/chassis/cmd_vel";
    std::string odom_topic = "/chassis/odom_twist";
    std::string joy_topic = "/joy";
    std::string wheel_topic = "/chassis/wheel_omega";
    std::string directions = "fb";
    std::string bridge_node = "/r2_vehicle_bridge";
    std::string output_dir = "var_data";

    double vmin = 0.02;
    double vmax = 0.30;
    double vstep = 0.01;
    double hold_s = 1.2;
    double move_eps = 0.02;
    double settle_s = 0.8;
    double standstill_s = 10.0;
    double odom_wait_s = 2.0;
    double joy_idle_s = 3.0;
    double joy_nudge_s = 3.0;
    double drift_limit = 0.01;
    double jitter_limit = 0.03;
    double safety_max_linear = 1.0;
    double safety_max_yaw = 2.0;
    double dry_breakaway_mps = 0.05;
    double dry_response_tau_s = 0.18;

    int repeats = 3;
    int sample_hz = 100;
    int max_rounds = 40;
    bool yes = false;
    bool dry_run = false;
    bool set_params = true;

    std::vector<double> omega_deadzone_list {0.02, 0.03, 0.05};
    std::vector<double> stiction_scale_list {0.70, 1.00, 1.30};
    std::vector<double> feedforward_scale_list {0.80, 1.00, 1.20};
    std::vector<double> breakaway_scale_list {0.70, 1.00, 1.30};
};

struct OutputPaths
{
    std::string stamp;
    std::string dir;
    std::string samples;
    std::string steps;
    std::string search_history;
    std::string best_params;
    std::string report;
};

struct Latest
{
    bool odom_valid = false;
    int64_t odom_ns = 0;
    double odom_vx = 0.0;
    double odom_vy = 0.0;
    double odom_omega = 0.0;

    bool joy_valid = false;
    int64_t joy_ns = 0;
    std::vector<float> joy_axes;

    bool wheel_valid = false;
    int64_t wheel_ns = 0;
    std::vector<double> wheel_omega;
};

struct Direction
{
    std::string name;
    double vx_unit = 0.0;
    double vy_unit = 0.0;
    double omega_unit = 0.0;

    bool is_yaw() const
    {
        return std::fabs(omega_unit) > 1e-9;
    }

    double measured_scalar(const Latest &latest) const
    {
        if (is_yaw())
            return latest.odom_omega * omega_unit;
        return latest.odom_vx * vx_unit + latest.odom_vy * vy_unit;
    }
};

struct Command
{
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
};

struct StepResult
{
    std::string mode;
    std::string direction;
    std::string param_id = "baseline";
    int repeat = 0;
    int step_index = 0;
    double cmd_value = 0.0;
    bool moved = false;
    bool pass = true;
    uint64_t samples = 0;
    double start_latency_s = std::numeric_limits<double>::quiet_NaN();
    double peak_speed = 0.0;
    double steady_speed = 0.0;
    double overshoot = 0.0;
    double score = std::numeric_limits<double>::infinity();
    double standstill_drift_rms = 0.0;
    double standstill_jitter_rms = 0.0;
    std::string notes;
};

struct ScoreMetrics
{
    double v_breakaway = std::numeric_limits<double>::infinity();
    double start_latency = 0.0;
    double start_miss_rate = 1.0;
    double overshoot = 0.0;
    double wheel_consistency = 0.0;
    double score = std::numeric_limits<double>::infinity();
};

struct RunSummary
{
    bool ok = true;
    bool params_supported = true;
    std::string notes;
    ParameterSet best_params;
    ScoreMetrics metrics;
    std::vector<StepResult> steps;
};

class TuneNode : public rclcpp::Node
{
public:
    explicit TuneNode(const Options &options)
        : Node("chassis_zero_deadzone_stiction_tune"), options_(options)
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(options_.cmd_topic, 10);

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

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            options_.joy_topic, 20,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_.joy_valid = true;
                latest_.joy_ns = now_ns();
                latest_.joy_axes = msg->axes;
            });

        wheel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            options_.wheel_topic, 20,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_.wheel_valid = true;
                latest_.wheel_ns = now_ns();
                latest_.wheel_omega = msg->data;
            });
    }

    void publish_command(double vx, double vy, double omega)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.angular.z = omega;
        cmd_pub_->publish(msg);
        last_cmd_ = {vx, vy, omega};
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

    void set_dry_breakaway(double value)
    {
        dry_breakaway_mps_ = std::max(0.0, value);
    }

    void step_dry_sim(double dt_s)
    {
        if (!options_.dry_run)
            return;

        auto target_axis = [&](double cmd) {
            if (std::fabs(cmd) < dry_breakaway_mps_)
                return 0.0;
            return cmd * 0.92;
        };

        const double tau = std::max(0.02, options_.dry_response_tau_s);
        const double alpha = std::min(1.0, std::max(0.0, dt_s / tau));
        dry_vx_ += (target_axis(last_cmd_.vx) - dry_vx_) * alpha;
        dry_vy_ += (target_axis(last_cmd_.vy) - dry_vy_) * alpha;
        dry_omega_ += (target_axis(last_cmd_.omega) - dry_omega_) * alpha;

        std::lock_guard<std::mutex> lock(mtx_);
        latest_.odom_valid = true;
        latest_.odom_ns = now_ns();
        latest_.odom_vx = dry_vx_;
        latest_.odom_vy = dry_vy_;
        latest_.odom_omega = dry_omega_;
        latest_.wheel_valid = true;
        latest_.wheel_ns = latest_.odom_ns;
        latest_.wheel_omega = {
            dry_vx_ * 8.0 - dry_vy_ * 8.0 - dry_omega_ * 0.7,
            dry_vx_ * 8.0 + dry_vy_ * 8.0 + dry_omega_ * 0.7,
            dry_vx_ * 8.0 + dry_vy_ * 8.0 - dry_omega_ * 0.7,
            dry_vx_ * 8.0 - dry_vy_ * 8.0 + dry_omega_ * 0.7,
        };
    }

    void step_dry_joy(bool nudge_phase, double elapsed_s, double duration_s)
    {
        if (!options_.dry_run)
            return;

        std::vector<float> axes(4, 0.0f);
        axes[0] = static_cast<float>(0.004 * std::sin(elapsed_s * 11.0));
        axes[1] = static_cast<float>(0.006 * std::cos(elapsed_s * 7.0));
        axes[2] = static_cast<float>(0.003 * std::sin(elapsed_s * 5.0));
        axes[3] = static_cast<float>(0.004 * std::cos(elapsed_s * 3.0));
        if (nudge_phase)
        {
            const double ramp = std::min(1.0, elapsed_s / std::max(0.1, duration_s));
            axes[1] += static_cast<float>(0.010 + 0.030 * ramp);
        }

        std::lock_guard<std::mutex> lock(mtx_);
        latest_.joy_valid = true;
        latest_.joy_ns = now_ns();
        latest_.joy_axes = axes;
    }

private:
    Options options_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_sub_;
    mutable std::mutex mtx_;
    Latest latest_;
    Command last_cmd_;
    double dry_breakaway_mps_ = 0.05;
    double dry_vx_ = 0.0;
    double dry_vy_ = 0.0;
    double dry_omega_ = 0.0;
};

Options parse_options(int argc, char **argv)
{
    Options options;
    options.mode = cli_get(argc, argv, "mode", options.mode.c_str());
    options.cmd_topic = cli_get(argc, argv, "cmd-topic", options.cmd_topic.c_str());
    options.odom_topic = cli_get(argc, argv, "odom-topic", options.odom_topic.c_str());
    options.joy_topic = cli_get(argc, argv, "joy-topic", options.joy_topic.c_str());
    options.wheel_topic = cli_get(argc, argv, "wheel-topic", options.wheel_topic.c_str());
    options.directions = cli_get(argc, argv, "directions", options.directions.c_str());
    options.bridge_node = cli_get(argc, argv, "bridge-node", options.bridge_node.c_str());
    options.output_dir = cli_get(argc, argv, "output-dir", options.output_dir.c_str());

    options.vmin = parse_double(cli_get(argc, argv, "vmin", "0.02"), options.vmin);
    options.vmax = parse_double(cli_get(argc, argv, "vmax", "0.30"), options.vmax);
    options.vstep = parse_double(cli_get(argc, argv, "vstep", "0.01"), options.vstep);
    options.hold_s = parse_double(cli_get(argc, argv, "hold", "1.2"), options.hold_s);
    options.move_eps = parse_double(cli_get(argc, argv, "move-eps", "0.02"), options.move_eps);
    options.settle_s = parse_double(cli_get(argc, argv, "settle", "0.8"), options.settle_s);
    options.standstill_s = parse_double(cli_get(argc, argv, "standstill", "10.0"), options.standstill_s);
    options.odom_wait_s = parse_double(cli_get(argc, argv, "odom-wait", "2.0"), options.odom_wait_s);
    options.joy_idle_s = parse_double(cli_get(argc, argv, "joy-idle", "3.0"), options.joy_idle_s);
    options.joy_nudge_s = parse_double(cli_get(argc, argv, "joy-nudge", "3.0"), options.joy_nudge_s);
    options.drift_limit = parse_double(cli_get(argc, argv, "drift-limit", "0.01"), options.drift_limit);
    options.jitter_limit = parse_double(cli_get(argc, argv, "jitter-limit", "0.03"), options.jitter_limit);
    options.safety_max_linear = parse_double(cli_get(argc, argv, "safety-max-linear", "1.0"), options.safety_max_linear);
    options.safety_max_yaw = parse_double(cli_get(argc, argv, "safety-max-yaw", "2.0"), options.safety_max_yaw);
    options.dry_breakaway_mps = parse_double(cli_get(argc, argv, "dry-breakaway", "0.05"), options.dry_breakaway_mps);
    options.dry_response_tau_s = parse_double(cli_get(argc, argv, "dry-response-tau", "0.18"), options.dry_response_tau_s);

    options.repeats = parse_int(cli_get(argc, argv, "repeats", "3"), options.repeats);
    options.sample_hz = parse_int(cli_get(argc, argv, "sample-hz", "100"), options.sample_hz);
    options.max_rounds = parse_int(cli_get(argc, argv, "max-rounds", "40"), options.max_rounds);
    options.yes = cli_has(argc, argv, "yes");
    options.dry_run = cli_has(argc, argv, "dry-run");
    options.set_params = !cli_has(argc, argv, "no-param-set");

    options.omega_deadzone_list = parse_double_list(
        cli_get(argc, argv, "omega-deadzone-list", "0.02,0.03,0.05"),
        options.omega_deadzone_list);
    options.stiction_scale_list = parse_double_list(
        cli_get(argc, argv, "stiction-scale-list", "0.70,1.00,1.30"),
        options.stiction_scale_list);
    options.feedforward_scale_list = parse_double_list(
        cli_get(argc, argv, "feedforward-scale-list", "0.80,1.00,1.20"),
        options.feedforward_scale_list);
    options.breakaway_scale_list = parse_double_list(
        cli_get(argc, argv, "breakaway-scale-list", "0.70,1.00,1.30"),
        options.breakaway_scale_list);

    options.vmin = std::max(0.0, options.vmin);
    options.vmax = std::max(options.vmin, options.vmax);
    options.vstep = std::max(0.001, options.vstep);
    options.hold_s = std::max(0.05, options.hold_s);
    options.move_eps = std::max(0.0, options.move_eps);
    options.settle_s = std::max(0.0, options.settle_s);
    options.standstill_s = std::max(0.1, options.standstill_s);
    options.odom_wait_s = std::max(0.0, options.odom_wait_s);
    options.joy_idle_s = std::max(0.1, options.joy_idle_s);
    options.joy_nudge_s = std::max(0.1, options.joy_nudge_s);
    options.repeats = std::max(1, options.repeats);
    options.sample_hz = std::max(5, options.sample_hz);
    options.max_rounds = std::max(1, options.max_rounds);
    return options;
}

void print_help()
{
    std::cout
        << "chassis_zero_deadzone_stiction_tune options:\n"
        << "  --mode joystick-floor|wheel-stiction|min-cmd-step|standstill|online-tune\n"
        << "  --cmd-topic /chassis/cmd_vel       velocity command topic\n"
        << "  --odom-topic /chassis/odom_twist   chassis odom Twist feedback\n"
        << "  --joy-topic /joy                   joystick source for joystick-floor\n"
        << "  --directions fb|lr|yaw|all         scan directions, default fb\n"
        << "  --vmin 0.02 --vmax 0.30 --vstep 0.01\n"
        << "  --hold 1.2 --settle 0.8 --move-eps 0.02 --repeats 3\n"
        << "  --bridge-node /r2_vehicle_bridge   parameter node for online-tune\n"
        << "  --output-dir var_data              timestamped output root\n"
        << "  --max-rounds 40                    cap online-tune candidates\n"
        << "  --yes                              skip safety prompt\n"
        << "  --dry-run                          use synthetic odom/joy feedback\n"
        << "  --no-param-set                     do not call bridge parameter service\n";
}

OutputPaths make_output_paths(const Options &options)
{
    OutputPaths paths;
    paths.stamp = timestamp_string();
    paths.dir = options.output_dir + "/chassis_zero_deadzone_stiction_tune_" + paths.stamp;
    paths.samples = paths.dir + "/samples_" + paths.stamp + ".csv";
    paths.steps = paths.dir + "/steps_" + paths.stamp + ".csv";
    paths.search_history = paths.dir + "/search_history_" + paths.stamp + ".csv";
    paths.best_params = paths.dir + "/best_params_" + paths.stamp + ".yaml";
    paths.report = paths.dir + "/report_" + paths.stamp + ".md";
    return paths;
}

std::vector<Direction> build_directions(const Options &options)
{
    std::vector<Direction> dirs;
    const bool fb = options.directions == "fb" || options.directions == "all";
    const bool lr = options.directions == "lr" || options.directions == "all";
    const bool yaw = options.directions == "yaw" || options.directions == "all";

    if (fb)
    {
        dirs.push_back({"forward", 1.0, 0.0, 0.0});
        dirs.push_back({"backward", -1.0, 0.0, 0.0});
    }
    if (lr)
    {
        dirs.push_back({"left", 0.0, 1.0, 0.0});
        dirs.push_back({"right", 0.0, -1.0, 0.0});
    }
    if (yaw)
    {
        dirs.push_back({"yaw_ccw", 0.0, 0.0, 1.0});
        dirs.push_back({"yaw_cw", 0.0, 0.0, -1.0});
    }
    if (dirs.empty())
        dirs.push_back({"forward", 1.0, 0.0, 0.0});
    return dirs;
}

Command command_for(const Direction &direction, double value)
{
    return {direction.vx_unit * value,
            direction.vy_unit * value,
            direction.omega_unit * value};
}

double joy_max_abs(const Latest &latest)
{
    double value = 0.0;
    for (float axis : latest.joy_axes)
        value = std::max(value, std::fabs(static_cast<double>(axis)));
    return value;
}

bool odom_sample_safe(const Options &options, const Latest &latest)
{
    if (!latest.odom_valid)
        return true;
    if (!std::isfinite(latest.odom_vx) ||
        !std::isfinite(latest.odom_vy) ||
        !std::isfinite(latest.odom_omega))
        return false;
    const double linear = std::hypot(latest.odom_vx, latest.odom_vy);
    return linear <= options.safety_max_linear &&
           std::fabs(latest.odom_omega) <= options.safety_max_yaw;
}

void write_sample_header(std::ofstream &csv)
{
    csv << "mode,direction,repeat,step_index,param_id,timestamp_ns,elapsed_s,phase,"
        << "cmd_vx,cmd_vy,cmd_omega,odom_valid,odom_age_ms,odom_vx,odom_vy,odom_omega,"
        << "measured_speed,joy_valid,joy_max_abs,wheel_valid,wheel0,wheel1,wheel2,wheel3\n";
}

void write_step_header(std::ofstream &csv)
{
    csv << "mode,direction,repeat,step_index,param_id,cmd_value,moved,start_latency_s,"
        << "peak_speed,steady_speed,overshoot,samples,score,standstill_drift_rms,"
        << "standstill_jitter_rms,pass,notes\n";
}

void write_search_header(std::ofstream &csv)
{
    csv << "round,param_id,omega_deadzone,stiction_scale,feedforward_scale,breakaway_scale,"
        << "params_applied,standstill_pass,v_breakaway,start_latency,start_miss_rate,"
        << "overshoot,score,best_score\n";
}

void write_sample(std::ofstream &csv,
                  const std::string &mode,
                  const std::string &direction,
                  int repeat,
                  int step_index,
                  const std::string &param_id,
                  int64_t start_ns,
                  const std::string &phase,
                  const Command &cmd,
                  const Latest &latest,
                  double measured_speed)
{
    const int64_t ts = now_ns();
    const double elapsed_s = static_cast<double>(ts - start_ns) / kNsPerS;
    csv << mode << ','
        << direction << ','
        << repeat << ','
        << step_index << ','
        << param_id << ','
        << ts << ','
        << elapsed_s << ','
        << phase << ','
        << cmd.vx << ','
        << cmd.vy << ','
        << cmd.omega << ','
        << latest.odom_valid << ','
        << (latest.odom_valid ? static_cast<double>(ts - latest.odom_ns) / 1.0e6 : -1.0) << ','
        << latest.odom_vx << ','
        << latest.odom_vy << ','
        << latest.odom_omega << ','
        << measured_speed << ','
        << latest.joy_valid << ','
        << joy_max_abs(latest) << ','
        << latest.wheel_valid;
    for (int i = 0; i < kWheelCount; ++i)
    {
        const double wheel = (i < static_cast<int>(latest.wheel_omega.size())) ?
            latest.wheel_omega[i] : 0.0;
        csv << ',' << wheel;
    }
    csv << '\n';
}

void write_step_row(std::ofstream &csv, const StepResult &r)
{
    csv << r.mode << ','
        << r.direction << ','
        << r.repeat << ','
        << r.step_index << ','
        << r.param_id << ','
        << r.cmd_value << ','
        << r.moved << ','
        << r.start_latency_s << ','
        << r.peak_speed << ','
        << r.steady_speed << ','
        << r.overshoot << ','
        << r.samples << ','
        << r.score << ','
        << r.standstill_drift_rms << ','
        << r.standstill_jitter_rms << ','
        << r.pass << ','
        << r.notes << '\n';
}

bool prompt_safety_if_needed(const Options &options)
{
    if (options.dry_run || options.yes || options.mode == "joystick-floor")
        return true;

    std::cout
        << "[chassis_zero_deadzone_stiction_tune] Safety check:\n"
        << "  - Leave at least 1.5 m around the chassis.\n"
        << "  - First run wheel-stiction and standstill with wheels lifted.\n"
        << "  - Keep an E-stop or power cut within reach.\n"
        << "Type YES to continue: ";
    std::string line;
    std::getline(std::cin, line);
    return line == "YES" || line == "yes";
}

void publish_zero_burst(const std::shared_ptr<TuneNode> &node)
{
    for (int i = 0; i < 5; ++i)
    {
        node->publish_zero();
        rclcpp::spin_some(node);
        node->step_dry_sim(0.02);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

bool wait_for_odom(const std::shared_ptr<TuneNode> &node, const Options &options)
{
    if (options.dry_run)
    {
        node->publish_zero();
        node->step_dry_sim(0.02);
        return true;
    }

    const int64_t start = now_ns();
    while (g_running.load() && rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (node->latest().odom_valid)
            return true;
        if ((now_ns() - start) > static_cast<int64_t>(options.odom_wait_s * kNsPerS))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

bool wait_for_joy(const std::shared_ptr<TuneNode> &node, const Options &options)
{
    if (options.dry_run)
    {
        node->step_dry_joy(false, 0.0, 1.0);
        return true;
    }

    const int64_t start = now_ns();
    while (g_running.load() && rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (node->latest().joy_valid)
            return true;
        if ((now_ns() - start) > static_cast<int64_t>(options.odom_wait_s * kNsPerS))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

void hold_zero(const std::shared_ptr<TuneNode> &node,
               const Options &options,
               const std::string &mode,
               const std::string &direction,
               int repeat,
               int step_index,
               const std::string &param_id,
               double duration_s,
               std::ofstream &samples_csv)
{
    const int64_t start = now_ns();
    const int64_t duration_ns = static_cast<int64_t>(duration_s * kNsPerS);
    const double dt_s = 1.0 / static_cast<double>(options.sample_hz);
    const auto sleep_dt = std::chrono::duration<double>(dt_s);
    const Command zero {};

    while (g_running.load() && rclcpp::ok() && (now_ns() - start) < duration_ns)
    {
        node->publish_zero();
        rclcpp::spin_some(node);
        node->step_dry_sim(dt_s);
        const Latest latest = node->latest();
        write_sample(samples_csv, mode, direction, repeat, step_index, param_id,
                     start, "settle", zero, latest, 0.0);
        if (!odom_sample_safe(options, latest))
        {
            g_running.store(false);
            break;
        }
        std::this_thread::sleep_for(sleep_dt);
    }
    node->publish_zero();
}

StepResult run_command_step(const std::shared_ptr<TuneNode> &node,
                            const Options &options,
                            const std::string &mode,
                            const Direction &direction,
                            int repeat,
                            int step_index,
                            const std::string &param_id,
                            double cmd_value,
                            std::ofstream &samples_csv)
{
    hold_zero(node, options, mode, direction.name, repeat, step_index, param_id,
              options.settle_s, samples_csv);

    StepResult result;
    result.mode = mode;
    result.direction = direction.name;
    result.repeat = repeat;
    result.step_index = step_index;
    result.param_id = param_id;
    result.cmd_value = cmd_value;

    const Command cmd = command_for(direction, cmd_value);
    const int64_t start = now_ns();
    const int64_t duration_ns = static_cast<int64_t>(options.hold_s * kNsPerS);
    const double dt_s = 1.0 / static_cast<double>(options.sample_hz);
    const auto sleep_dt = std::chrono::duration<double>(dt_s);
    Stats steady_stats;
    double peak_abs = 0.0;

    while (g_running.load() && rclcpp::ok() && (now_ns() - start) < duration_ns)
    {
        node->publish_command(cmd.vx, cmd.vy, cmd.omega);
        rclcpp::spin_some(node);
        node->step_dry_sim(dt_s);

        const int64_t ts = now_ns();
        const double elapsed = static_cast<double>(ts - start) / kNsPerS;
        const Latest latest = node->latest();
        const double measured = direction.measured_scalar(latest);
        const double measured_abs = std::fabs(measured);

        result.samples++;
        peak_abs = std::max(peak_abs, measured_abs);
        if (!result.moved && measured_abs >= options.move_eps)
        {
            result.moved = true;
            result.start_latency_s = elapsed;
        }
        if (elapsed >= options.hold_s * 0.55)
            steady_stats.add(measured);

        write_sample(samples_csv, mode, direction.name, repeat, step_index, param_id,
                     start, "drive", cmd, latest, measured);

        if (!odom_sample_safe(options, latest))
        {
            result.pass = false;
            result.notes = "safety_limit";
            g_running.store(false);
            break;
        }
        std::this_thread::sleep_for(sleep_dt);
    }

    node->publish_zero();
    result.peak_speed = peak_abs;
    result.steady_speed = steady_stats.mean();
    result.overshoot = std::max(0.0, peak_abs - cmd_value);
    result.score = result.moved ?
        (5.0 * result.start_latency_s + 10.0 * result.overshoot) :
        std::numeric_limits<double>::infinity();
    if (result.notes.empty())
        result.notes = result.moved ? "moved" : "no_start";
    return result;
}

StepResult run_standstill_once(const std::shared_ptr<TuneNode> &node,
                               const Options &options,
                               const std::string &param_id,
                               double duration_s,
                               std::ofstream &samples_csv)
{
    StepResult result;
    result.mode = "standstill";
    result.direction = "zero";
    result.param_id = param_id;
    result.cmd_value = 0.0;

    const int64_t start = now_ns();
    const int64_t duration_ns = static_cast<int64_t>(duration_s * kNsPerS);
    const double dt_s = 1.0 / static_cast<double>(options.sample_hz);
    const auto sleep_dt = std::chrono::duration<double>(dt_s);
    const Command zero {};
    Stats linear_speed;
    Stats yaw_speed;
    Stats jitter;
    double prev_scalar = 0.0;
    bool have_prev = false;

    while (g_running.load() && rclcpp::ok() && (now_ns() - start) < duration_ns)
    {
        node->publish_zero();
        rclcpp::spin_some(node);
        node->step_dry_sim(dt_s);

        const Latest latest = node->latest();
        const double linear = std::hypot(latest.odom_vx, latest.odom_vy);
        const double scalar = std::max(linear, std::fabs(latest.odom_omega));
        linear_speed.add(linear);
        yaw_speed.add(latest.odom_omega);
        if (have_prev)
            jitter.add((scalar - prev_scalar) / dt_s);
        prev_scalar = scalar;
        have_prev = true;

        result.samples++;
        result.peak_speed = std::max(result.peak_speed, scalar);
        if (scalar >= options.move_eps)
            result.moved = true;

        write_sample(samples_csv, "standstill", "zero", 0, 0, param_id,
                     start, "zero", zero, latest, scalar);

        if (!odom_sample_safe(options, latest))
        {
            result.pass = false;
            result.notes = "safety_limit";
            g_running.store(false);
            break;
        }
        std::this_thread::sleep_for(sleep_dt);
    }

    node->publish_zero();
    result.standstill_drift_rms = linear_speed.rms();
    result.standstill_jitter_rms =
        std::max(std::fabs(yaw_speed.rms()), std::min(1.0, jitter.rms()));
    result.pass = result.pass &&
                  result.standstill_drift_rms <= options.drift_limit &&
                  result.standstill_jitter_rms <= options.jitter_limit;
    result.score = result.pass ? result.standstill_drift_rms + result.standstill_jitter_rms :
        std::numeric_limits<double>::infinity();
    if (result.notes.empty())
        result.notes = result.pass ? "standstill_ok" : "standstill_fail";
    return result;
}

std::vector<double> build_speed_steps(const Options &options)
{
    std::vector<double> speeds;
    for (double value = options.vmin;
         value <= options.vmax + options.vstep * 0.25;
         value += options.vstep)
    {
        speeds.push_back(value);
        if (speeds.size() > 1000)
            break;
    }
    return speeds;
}

std::vector<StepResult> run_scan(const std::shared_ptr<TuneNode> &node,
                                 const Options &options,
                                 const std::string &mode,
                                 const std::string &param_id,
                                 std::ofstream &samples_csv,
                                 std::ofstream &steps_csv)
{
    std::vector<StepResult> results;
    const std::vector<Direction> dirs = build_directions(options);
    const std::vector<double> speeds = build_speed_steps(options);
    int step_index = 0;

    for (const Direction &dir : dirs)
    {
        for (double speed : speeds)
        {
            for (int repeat = 0; repeat < options.repeats; ++repeat)
            {
                if (!g_running.load() || !rclcpp::ok())
                    return results;
                StepResult result = run_command_step(
                    node, options, mode, dir, repeat, step_index, param_id,
                    speed, samples_csv);
                write_step_row(steps_csv, result);
                results.push_back(result);
            }
            step_index++;
        }
    }
    return results;
}

ScoreMetrics summarize_scan(const std::vector<StepResult> &steps)
{
    ScoreMetrics metrics;
    Stats latency;
    Stats overshoot;
    int small_total = 0;
    int small_miss = 0;
    double first_cmd = std::numeric_limits<double>::infinity();
    double second_cmd = std::numeric_limits<double>::infinity();
    double max_cmd = 0.0;

    for (const StepResult &step : steps)
    {
        if (step.cmd_value <= 0.0)
            continue;
        first_cmd = std::min(first_cmd, step.cmd_value);
        max_cmd = std::max(max_cmd, step.cmd_value);
    }
    for (const StepResult &step : steps)
    {
        if (step.cmd_value > first_cmd + 1e-9)
            second_cmd = std::min(second_cmd, step.cmd_value);
    }
    const double small_limit = std::isfinite(second_cmd) ? second_cmd : first_cmd;

    for (const StepResult &step : steps)
    {
        if (step.cmd_value <= 0.0)
            continue;
        if (step.moved)
        {
            metrics.v_breakaway = std::min(metrics.v_breakaway, step.cmd_value);
            latency.add(step.start_latency_s);
        }
        if (step.cmd_value <= small_limit + 1e-9)
        {
            small_total++;
            if (!step.moved)
                small_miss++;
        }
        overshoot.add(step.overshoot);
    }

    if (!std::isfinite(metrics.v_breakaway))
    {
        const double inferred_step = std::isfinite(second_cmd) ?
            std::max(0.0, second_cmd - first_cmd) : 0.0;
        metrics.v_breakaway = steps.empty() ? std::numeric_limits<double>::infinity() :
            max_cmd + inferred_step;
    }
    metrics.start_latency = latency.mean();
    metrics.start_miss_rate = small_total > 0 ?
        static_cast<double>(small_miss) / static_cast<double>(small_total) : 1.0;
    metrics.overshoot = overshoot.mean();
    metrics.score = 50.0 * metrics.v_breakaway +
                    5.0 * metrics.start_latency +
                    20.0 * metrics.start_miss_rate +
                    10.0 * metrics.overshoot +
                    5.0 * metrics.wheel_consistency;
    return metrics;
}

RunSummary run_joystick_floor(const std::shared_ptr<TuneNode> &node,
                              const Options &options,
                              std::ofstream &samples_csv,
                              std::ofstream &steps_csv)
{
    RunSummary summary;
    if (!wait_for_joy(node, options))
    {
        summary.ok = false;
        summary.notes = "No joystick data. Start the joystick driver or pass --joy-topic for sensor_msgs/msg/Joy.";
        return summary;
    }

    if (!options.dry_run && !options.yes)
    {
        std::cout << "Release the joystick, then press Enter for idle noise capture.";
        std::string line;
        std::getline(std::cin, line);
    }

    const double dt_s = 1.0 / static_cast<double>(options.sample_hz);
    const auto sleep_dt = std::chrono::duration<double>(dt_s);
    Stats idle_max;
    const int64_t idle_start = now_ns();
    while (g_running.load() && rclcpp::ok() &&
           (now_ns() - idle_start) < static_cast<int64_t>(options.joy_idle_s * kNsPerS))
    {
        const double elapsed = static_cast<double>(now_ns() - idle_start) / kNsPerS;
        node->step_dry_joy(false, elapsed, options.joy_idle_s);
        rclcpp::spin_some(node);
        const Latest latest = node->latest();
        const double max_axis = joy_max_abs(latest);
        idle_max.add(max_axis);
        write_sample(samples_csv, "joystick-floor", "idle", 0, 0, "joystick",
                     idle_start, "idle", {}, latest, max_axis);
        std::this_thread::sleep_for(sleep_dt);
    }

    if (!options.dry_run && !options.yes)
    {
        std::cout << "Push one stick very slowly just above the physical noise, then press Enter.";
        std::string line;
        std::getline(std::cin, line);
    }

    Stats nudge_max;
    const int64_t nudge_start = now_ns();
    while (g_running.load() && rclcpp::ok() &&
           (now_ns() - nudge_start) < static_cast<int64_t>(options.joy_nudge_s * kNsPerS))
    {
        const double elapsed = static_cast<double>(now_ns() - nudge_start) / kNsPerS;
        node->step_dry_joy(true, elapsed, options.joy_nudge_s);
        rclcpp::spin_some(node);
        const Latest latest = node->latest();
        const double max_axis = joy_max_abs(latest);
        nudge_max.add(max_axis);
        write_sample(samples_csv, "joystick-floor", "nudge", 0, 1, "joystick",
                     nudge_start, "nudge", {}, latest, max_axis);
        std::this_thread::sleep_for(sleep_dt);
    }

    const double noise_amp = idle_max.n > 0 ? idle_max.max : 0.0;
    const double suggested = std::max(0.002, noise_amp * 1.2);
    summary.best_params.joystick_deadzone = suggested;
    summary.metrics.v_breakaway = suggested;
    summary.metrics.score = suggested;

    StepResult idle;
    idle.mode = "joystick-floor";
    idle.direction = "idle";
    idle.param_id = "joystick";
    idle.cmd_value = 0.0;
    idle.samples = idle_max.n;
    idle.peak_speed = noise_amp;
    idle.score = suggested;
    idle.notes = "suggested_deadzone";
    write_step_row(steps_csv, idle);
    summary.steps.push_back(idle);

    StepResult nudge;
    nudge.mode = "joystick-floor";
    nudge.direction = "nudge";
    nudge.param_id = "joystick";
    nudge.step_index = 1;
    nudge.cmd_value = nudge_max.mean();
    nudge.samples = nudge_max.n;
    nudge.peak_speed = nudge_max.max;
    nudge.score = nudge_max.mean();
    nudge.notes = "operator_min_nudge";
    write_step_row(steps_csv, nudge);
    summary.steps.push_back(nudge);

    std::ostringstream note;
    note << "joystick_noise_max=" << noise_amp
         << " suggested_joystick.deadzone=" << suggested;
    summary.notes = note.str();
    return summary;
}

ParameterSet scaled_params(double omega_deadzone,
                           double stiction_scale,
                           double feedforward_scale,
                           double breakaway_scale,
                           double joystick_deadzone)
{
    ParameterSet params;
    params.joystick_deadzone = joystick_deadzone;
    for (int i = 0; i < kWheelCount; ++i)
    {
        params.omega_deadzone[i] = omega_deadzone;
        params.stiction_torque[i] *= stiction_scale;
        params.feedforward_scale[i] *= feedforward_scale;
        params.breakaway_torque[i] *= breakaway_scale;
    }
    return params;
}

bool apply_params(const std::shared_ptr<TuneNode> &node,
                  const std::shared_ptr<rclcpp::AsyncParametersClient> &client,
                  const Options &options,
                  const ParameterSet &params)
{
    if (options.dry_run || !options.set_params)
        return true;

    const int64_t start = now_ns();
    while (g_running.load() && rclcpp::ok() && !client->service_is_ready())
    {
        if ((now_ns() - start) > static_cast<int64_t>(3.0 * kNsPerS))
        {
            std::cerr << "[chassis_zero_deadzone_stiction_tune] parameter service not ready: "
                      << options.bridge_node << "\n";
            return false;
        }
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::vector<rclcpp::Parameter> ros_params;
    ros_params.emplace_back("joystick.deadzone", params.joystick_deadzone);
    for (int i = 0; i < kWheelCount; ++i)
    {
        ros_params.emplace_back("omni_wheel.omega_deadzone_" + std::to_string(i), params.omega_deadzone[i]);
        ros_params.emplace_back("omni_wheel.stiction_torque_" + std::to_string(i), params.stiction_torque[i]);
        ros_params.emplace_back("omni_wheel.dynamic_friction_" + std::to_string(i), params.dynamic_friction[i]);
        ros_params.emplace_back("omni_wheel.feedforward_scale_" + std::to_string(i), params.feedforward_scale[i]);
        ros_params.emplace_back("omni_wheel.breakaway_torque_" + std::to_string(i), params.breakaway_torque[i]);
    }

    auto future = client->set_parameters(ros_params);
    const auto rc = rclcpp::spin_until_future_complete(
        node, future, std::chrono::seconds(3));
    if (rc != rclcpp::FutureReturnCode::SUCCESS)
        return false;

    const auto results = future.get();
    for (const auto &result : results)
    {
        if (!result.successful)
        {
            std::cerr << "[chassis_zero_deadzone_stiction_tune] parameter rejected: "
                      << result.reason << "\n";
            return false;
        }
    }
    return true;
}

RunSummary run_online_tune(const std::shared_ptr<TuneNode> &node,
                           const std::shared_ptr<rclcpp::AsyncParametersClient> &client,
                           const Options &options,
                           std::ofstream &samples_csv,
                           std::ofstream &steps_csv,
                           std::ofstream &search_csv)
{
    RunSummary summary;
    double best_score = std::numeric_limits<double>::infinity();
    int round = 0;

    for (double omega_deadzone : options.omega_deadzone_list)
    {
        for (double stiction_scale : options.stiction_scale_list)
        {
            for (double ff_scale : options.feedforward_scale_list)
            {
                for (double breakaway_scale : options.breakaway_scale_list)
                {
                    if (!g_running.load() || !rclcpp::ok() || round >= options.max_rounds)
                        return summary;

                    const std::string param_id = "p" + std::to_string(round);
                    ParameterSet params = scaled_params(omega_deadzone, stiction_scale,
                                                        ff_scale, breakaway_scale,
                                                        summary.best_params.joystick_deadzone);
                    const bool applied = apply_params(node, client, options, params);
                    summary.params_supported = summary.params_supported && applied;

                    const double dry_factor = std::max(0.20,
                        0.35 * stiction_scale + 0.25 * ff_scale + 0.20 * breakaway_scale +
                        0.20 * (0.05 / std::max(0.005, omega_deadzone)));
                    node->set_dry_breakaway(options.dry_breakaway_mps / dry_factor);

                    const double standstill_duration = std::min(options.standstill_s, 2.0);
                    StepResult standstill = run_standstill_once(
                        node, options, param_id, standstill_duration, samples_csv);
                    write_step_row(steps_csv, standstill);
                    summary.steps.push_back(standstill);

                    ScoreMetrics metrics;
                    if (applied && standstill.pass)
                    {
                        std::vector<StepResult> scan = run_scan(
                            node, options, "online-tune", param_id, samples_csv, steps_csv);
                        metrics = summarize_scan(scan);
                        summary.steps.insert(summary.steps.end(), scan.begin(), scan.end());
                    }
                    else
                    {
                        metrics.score = std::numeric_limits<double>::infinity();
                    }

                    if (!standstill.pass)
                        metrics.score = std::numeric_limits<double>::infinity();

                    if (metrics.score < best_score)
                    {
                        best_score = metrics.score;
                        summary.best_params = params;
                        summary.metrics = metrics;
                    }

                    search_csv << round << ','
                               << param_id << ','
                               << omega_deadzone << ','
                               << stiction_scale << ','
                               << ff_scale << ','
                               << breakaway_scale << ','
                               << applied << ','
                               << standstill.pass << ','
                               << metrics.v_breakaway << ','
                               << metrics.start_latency << ','
                               << metrics.start_miss_rate << ','
                               << metrics.overshoot << ','
                               << metrics.score << ','
                               << best_score << '\n';

                    std::cout << "[chassis_zero_deadzone_stiction_tune] round " << round
                              << " score=" << metrics.score
                              << " best=" << best_score
                              << " params=" << param_id << "\n";
                    round++;
                }
            }
        }
    }

    if (!summary.params_supported)
        summary.notes = "Some parameter sets were not applied. Stage-two bridge parameter exposure is required for real online tuning.";
    return summary;
}

void write_best_params_yaml(const std::string &path, const ParameterSet &params)
{
    std::ofstream out(path);
    out << std::fixed << std::setprecision(6);
    out << "r2_vehicle_bridge:\n";
    out << "  ros__parameters:\n";
    out << "    joystick.deadzone: " << params.joystick_deadzone << "\n";
    for (int i = 0; i < kWheelCount; ++i)
        out << "    omni_wheel.omega_deadzone_" << i << ": " << params.omega_deadzone[i] << "\n";
    for (int i = 0; i < kWheelCount; ++i)
        out << "    omni_wheel.stiction_torque_" << i << ": " << params.stiction_torque[i] << "\n";
    for (int i = 0; i < kWheelCount; ++i)
        out << "    omni_wheel.dynamic_friction_" << i << ": " << params.dynamic_friction[i] << "\n";
    for (int i = 0; i < kWheelCount; ++i)
        out << "    omni_wheel.feedforward_scale_" << i << ": " << params.feedforward_scale[i] << "\n";
    for (int i = 0; i < kWheelCount; ++i)
        out << "    omni_wheel.breakaway_torque_" << i << ": " << params.breakaway_torque[i] << "\n";
}

void write_report(const std::string &path,
                  const Options &options,
                  const OutputPaths &paths,
                  const RunSummary &summary)
{
    std::ofstream out(path);
    out << std::fixed << std::setprecision(6);
    out << "# chassis_zero_deadzone_stiction_tune report\n\n";
    out << "- mode: " << options.mode << "\n";
    out << "- cmd_topic: " << options.cmd_topic << "\n";
    out << "- odom_topic: " << options.odom_topic << "\n";
    out << "- directions: " << options.directions << "\n";
    out << "- dry_run: " << options.dry_run << "\n";
    out << "- samples: " << paths.samples << "\n";
    out << "- steps: " << paths.steps << "\n";
    out << "- search_history: " << paths.search_history << "\n";
    out << "- best_params: " << paths.best_params << "\n\n";

    out << "## Result\n\n";
    out << "- ok: " << summary.ok << "\n";
    out << "- params_supported: " << summary.params_supported << "\n";
    out << "- notes: " << (summary.notes.empty() ? "none" : summary.notes) << "\n";
    out << "- v_breakaway: " << summary.metrics.v_breakaway << "\n";
    out << "- start_latency: " << summary.metrics.start_latency << "\n";
    out << "- start_miss_rate: " << summary.metrics.start_miss_rate << "\n";
    out << "- overshoot: " << summary.metrics.overshoot << "\n";
    out << "- score: " << summary.metrics.score << "\n\n";

    out << "## Suggested Parameters\n\n";
    out << "```yaml\n";
    out << "r2_vehicle_bridge:\n";
    out << "  ros__parameters:\n";
    out << "    joystick.deadzone: " << summary.best_params.joystick_deadzone << "\n";
    for (int i = 0; i < kWheelCount; ++i)
        out << "    omni_wheel.omega_deadzone_" << i << ": " << summary.best_params.omega_deadzone[i] << "\n";
    for (int i = 0; i < kWheelCount; ++i)
        out << "    omni_wheel.stiction_torque_" << i << ": " << summary.best_params.stiction_torque[i] << "\n";
    for (int i = 0; i < kWheelCount; ++i)
        out << "    omni_wheel.dynamic_friction_" << i << ": " << summary.best_params.dynamic_friction[i] << "\n";
    for (int i = 0; i < kWheelCount; ++i)
        out << "    omni_wheel.feedforward_scale_" << i << ": " << summary.best_params.feedforward_scale[i] << "\n";
    for (int i = 0; i < kWheelCount; ++i)
        out << "    omni_wheel.breakaway_torque_" << i << ": " << summary.best_params.breakaway_torque[i] << "\n";
    out << "```\n\n";

    out << "## Acceptance Checklist\n\n";
    out << "- Zero input: no crawl and no visible shake.\n";
    out << "- Small nonzero input above joystick noise: observable start.\n";
    out << "- Repeat wheel-stiction after changing compiled constants or bridge parameters.\n";
    out << "- Online tuning requires bridge support for omni_wheel.* and joystick.deadzone parameters.\n\n";

    out << "## Step Preview\n\n";
    out << "mode,direction,cmd,moved,latency,peak,notes\n";
    const size_t limit = std::min<size_t>(summary.steps.size(), 80);
    for (size_t i = 0; i < limit; ++i)
    {
        const StepResult &s = summary.steps[i];
        out << s.mode << ','
            << s.direction << ','
            << s.cmd_value << ','
            << s.moved << ','
            << s.start_latency_s << ','
            << s.peak_speed << ','
            << s.notes << "\n";
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
    OutputPaths paths = make_output_paths(options);
    if (!ensure_dir_recursive(paths.dir))
    {
        std::cerr << "[chassis_zero_deadzone_stiction_tune] cannot create output dir: "
                  << paths.dir << "\n";
        return 1;
    }

    std::ofstream samples_csv(paths.samples);
    std::ofstream steps_csv(paths.steps);
    std::ofstream search_csv(paths.search_history);
    if (!samples_csv || !steps_csv || !search_csv)
    {
        std::cerr << "[chassis_zero_deadzone_stiction_tune] cannot open output files\n";
        return 1;
    }
    samples_csv << std::fixed << std::setprecision(9);
    steps_csv << std::fixed << std::setprecision(9);
    search_csv << std::fixed << std::setprecision(9);
    write_sample_header(samples_csv);
    write_step_header(steps_csv);
    write_search_header(search_csv);

    if (!prompt_safety_if_needed(options))
    {
        std::cerr << "[chassis_zero_deadzone_stiction_tune] aborted by safety prompt\n";
        return 2;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TuneNode>(options);
    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, options.bridge_node);
    node->set_dry_breakaway(options.dry_breakaway_mps);

    RunSummary summary;

    if (options.mode == "joystick-floor")
    {
        summary = run_joystick_floor(node, options, samples_csv, steps_csv);
    }
    else if (options.mode == "standstill")
    {
        if (!wait_for_odom(node, options))
        {
            summary.ok = false;
            summary.notes = "No odom Twist data on " + options.odom_topic;
        }
        else
        {
            StepResult standstill = run_standstill_once(
                node, options, "baseline", options.standstill_s, samples_csv);
            write_step_row(steps_csv, standstill);
            summary.steps.push_back(standstill);
            summary.metrics.score = standstill.score;
            summary.ok = standstill.pass;
        }
    }
    else if (options.mode == "wheel-stiction" || options.mode == "min-cmd-step")
    {
        if (!wait_for_odom(node, options))
        {
            summary.ok = false;
            summary.notes = "No odom Twist data on " + options.odom_topic;
        }
        else
        {
            summary.steps = run_scan(node, options, options.mode, "baseline",
                                     samples_csv, steps_csv);
            summary.metrics = summarize_scan(summary.steps);
        }
    }
    else if (options.mode == "online-tune")
    {
        if (!wait_for_odom(node, options))
        {
            summary.ok = false;
            summary.notes = "No odom Twist data on " + options.odom_topic;
        }
        else
        {
            summary = run_online_tune(node, param_client, options,
                                      samples_csv, steps_csv, search_csv);
        }
    }
    else
    {
        summary.ok = false;
        summary.notes = "Unknown mode: " + options.mode;
        std::cerr << "[chassis_zero_deadzone_stiction_tune] " << summary.notes << "\n";
    }

    publish_zero_burst(node);
    write_best_params_yaml(paths.best_params, summary.best_params);
    write_report(paths.report, options, paths, summary);

    rclcpp::shutdown();

    std::cout << "[chassis_zero_deadzone_stiction_tune] output_dir=" << paths.dir << "\n"
              << "[chassis_zero_deadzone_stiction_tune] samples=" << paths.samples << "\n"
              << "[chassis_zero_deadzone_stiction_tune] steps=" << paths.steps << "\n"
              << "[chassis_zero_deadzone_stiction_tune] best_params=" << paths.best_params << "\n"
              << "[chassis_zero_deadzone_stiction_tune] report=" << paths.report << "\n";

    if (!summary.ok)
    {
        std::cerr << "[chassis_zero_deadzone_stiction_tune] failed: "
                  << (summary.notes.empty() ? "test did not pass" : summary.notes)
                  << "\n";
    }

    return summary.ok ? 0 : 3;
}

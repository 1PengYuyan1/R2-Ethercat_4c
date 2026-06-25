#include "imu_heading_hold/imu_heading_hold.h"

#include "math.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace
{
constexpr const char *kParamPrefix = "imu_heading_hold.";

bool parameter_to_double(const rclcpp::Parameter &parameter, double &value)
{
    switch (parameter.get_type())
    {
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            value = parameter.as_double();
            return true;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            value = static_cast<double>(parameter.as_int());
            return true;
        default:
            return false;
    }
}
}

void Class_Chariot_Imu_Heading_Hold::Init(float max_chassis_omega)
{
    max_chassis_omega_ = max_chassis_omega;

    std::lock_guard<std::mutex> lock(config_mtx_);
    Reinit_PID_Locked();
}

void Class_Chariot_Imu_Heading_Hold::Start(const std::shared_ptr<rclcpp::Node> &node)
{
    if (!node) return;

    Declare_Parameters(node);
    Load_Parameters(node);

    param_cb_handle_ = node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters)
        {
            return On_Set_Parameters(parameters);
        });

    Config config_snapshot;
    {
        std::lock_guard<std::mutex> lock(config_mtx_);
        config_snapshot = config_;
    }

    sub_imu_ = node->create_subscription<sensor_msgs::msg::Imu>(
        config_snapshot.topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            Handle_Imu(msg);
        });
}

void Class_Chariot_Imu_Heading_Hold::Stop()
{
    sub_imu_.reset();
    param_cb_handle_.reset();
    imu_valid_.store(false);
    latest_imu_ns_.store(0);
    Reset_Heading_Lock();
}

Class_Chariot_Imu_Heading_Hold::Snapshot Class_Chariot_Imu_Heading_Hold::Get_Snapshot()
{
    Snapshot snapshot;
    snapshot.valid = imu_valid_.load();
    snapshot.roll_rad = latest_imu_roll_.load();
    snapshot.pitch_rad = latest_imu_pitch_.load();
    snapshot.yaw_rad = latest_imu_yaw_.load();
    snapshot.angular_velocity_x = latest_imu_omega_x_.load();
    snapshot.angular_velocity_y = latest_imu_omega_y_.load();
    snapshot.angular_velocity_z = latest_imu_omega_z_.load();
    snapshot.linear_acceleration_x = latest_imu_accel_x_.load();
    snapshot.linear_acceleration_y = latest_imu_accel_y_.load();
    snapshot.linear_acceleration_z = latest_imu_accel_z_.load();

    const int64_t stamp_ns = latest_imu_ns_.load();
    if (snapshot.valid && stamp_ns > 0)
    {
        const int64_t age_ns = now_ns() - stamp_ns;
        snapshot.age_ms = age_ns / (1000LL * 1000LL);

        std::lock_guard<std::mutex> lock(config_mtx_);
        snapshot.fresh = age_ns <= config_.timeout_ns;
    }

    return snapshot;
}

float Class_Chariot_Imu_Heading_Hold::Correct_Omega(float vx_cmd,
                                                    float vy_cmd,
                                                    float omega_cmd,
                                                    bool lift_diff_mode,
                                                    int64_t now)
{
    std::lock_guard<std::mutex> lock(config_mtx_);

    float omega_out = omega_cmd;
    const bool imu_fresh = imu_valid_.load() &&
                           (now - latest_imu_ns_.load()) <= config_.timeout_ns;
    const bool user_turning = std::fabs(omega_cmd) > config_.turn_eps;
    const bool moving = (vx_cmd * vx_cmd + vy_cmd * vy_cmd) >
                        (config_.move_eps * config_.move_eps);
    const bool heading_allowed =
        config_.enable && imu_fresh && !lift_diff_mode && !user_turning;

    last_correcting_.store(heading_allowed && moving);

    if (heading_allowed && moving)
    {
        const float yaw_now = latest_imu_yaw_.load();
        if (!heading_locked_)
        {
            heading_target_ = yaw_now;
            heading_locked_ = true;
            heading_pid_.Set_Integral_Error(0.0f);
        }

        // 归一化航向误差喂入 PID(规避 ±pi 跳变): 令 PID 内部 error = Target-Now = err
        const float err = Math_Modulus_Normalization(heading_target_ - yaw_now, 2.0f * PI);
        heading_pid_.Set_Target(0.0f);
        heading_pid_.Set_Now(-err);
        heading_pid_.TIM_Calculate_PeriodElapsedCallback();
        omega_out = heading_pid_.Get_Out();
    }
    else if (heading_allowed)
    {
        // 静止时保持已锁定航向，下一次平移继续拉回同一目标；只清积分避免停顿期积累。
        heading_pid_.Set_Integral_Error(0.0f);
    }
    else
    {
        Reset_Heading_Lock_Locked();
    }

    if (omega_out >  max_chassis_omega_) omega_out =  max_chassis_omega_;
    if (omega_out < -max_chassis_omega_) omega_out = -max_chassis_omega_;
    return omega_out;
}

void Class_Chariot_Imu_Heading_Hold::Reset_Heading_Lock()
{
    std::lock_guard<std::mutex> lock(config_mtx_);
    Reset_Heading_Lock_Locked();
}

void Class_Chariot_Imu_Heading_Hold::Reset_Heading_Lock_Locked()
{
    heading_locked_ = false;
    heading_pid_.Set_Integral_Error(0.0f);
}

void Class_Chariot_Imu_Heading_Hold::Handle_Imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // 四元数 -> RPY, 归一化到 [-pi, pi]; HiPNUC 发布端为 best_effort, 必须用 SensorDataQoS
    const float w = static_cast<float>(msg->orientation.w);
    const float x = static_cast<float>(msg->orientation.x);
    const float y = static_cast<float>(msg->orientation.y);
    const float z = static_cast<float>(msg->orientation.z);
    const float roll = atan2f(2.0f * (w * x + y * z),
                              1.0f - 2.0f * (x * x + y * y));
    const float pitch_sin = 2.0f * (w * y - z * x);
    const float pitch = (std::fabs(pitch_sin) >= 1.0f) ?
                        std::copysign(PI * 0.5f, pitch_sin) :
                        asinf(pitch_sin);
    const float yaw = atan2f(2.0f * (w * z + x * y),
                             1.0f - 2.0f * (y * y + z * z));

    latest_imu_roll_.store(Math_Modulus_Normalization(roll, 2.0f * PI));
    latest_imu_pitch_.store(Math_Modulus_Normalization(pitch, 2.0f * PI));
    latest_imu_yaw_.store(Math_Modulus_Normalization(yaw, 2.0f * PI));
    latest_imu_omega_x_.store(static_cast<float>(msg->angular_velocity.x));
    latest_imu_omega_y_.store(static_cast<float>(msg->angular_velocity.y));
    latest_imu_omega_z_.store(static_cast<float>(msg->angular_velocity.z));
    latest_imu_accel_x_.store(static_cast<float>(msg->linear_acceleration.x));
    latest_imu_accel_y_.store(static_cast<float>(msg->linear_acceleration.y));
    latest_imu_accel_z_.store(static_cast<float>(msg->linear_acceleration.z));
    latest_imu_ns_.store(now_ns());
    imu_valid_.store(true);
}

void Class_Chariot_Imu_Heading_Hold::Declare_Parameters(const std::shared_ptr<rclcpp::Node> &node)
{
    auto declare_bool = [&](const char *name, bool value) {
        const std::string full_name = std::string(kParamPrefix) + name;
        if (!node->has_parameter(full_name))
            node->declare_parameter<bool>(full_name, value);
    };
    auto declare_double = [&](const char *name, double value) {
        const std::string full_name = std::string(kParamPrefix) + name;
        if (!node->has_parameter(full_name))
            node->declare_parameter<double>(full_name, value);
    };
    auto declare_string = [&](const char *name, const std::string &value) {
        const std::string full_name = std::string(kParamPrefix) + name;
        if (!node->has_parameter(full_name))
            node->declare_parameter<std::string>(full_name, value);
    };

    Config defaults;
    declare_bool("enable", defaults.enable);
    declare_string("topic", defaults.topic);
    declare_double("timeout_ms", static_cast<double>(defaults.timeout_ns) / 1.0e6);
    declare_double("turn_eps", defaults.turn_eps);
    declare_double("move_eps", defaults.move_eps);
    declare_double("kp", defaults.kp);
    declare_double("ki", defaults.ki);
    declare_double("kd", defaults.kd);
    declare_double("kf", defaults.kf);
    declare_double("i_out_max", defaults.i_out_max);
    declare_double("out_limit_scale", defaults.out_limit_scale);
    declare_double("dt", defaults.dt);
    declare_double("dead_zone", defaults.dead_zone);
}

void Class_Chariot_Imu_Heading_Hold::Load_Parameters(const std::shared_ptr<rclcpp::Node> &node)
{
    Config next;
    next.enable = node->get_parameter(std::string(kParamPrefix) + "enable").as_bool();
    next.topic = node->get_parameter(std::string(kParamPrefix) + "topic").as_string();
    next.timeout_ns = static_cast<int64_t>(
        node->get_parameter(std::string(kParamPrefix) + "timeout_ms").as_double() * 1.0e6);
    next.turn_eps = static_cast<float>(node->get_parameter(std::string(kParamPrefix) + "turn_eps").as_double());
    next.move_eps = static_cast<float>(node->get_parameter(std::string(kParamPrefix) + "move_eps").as_double());
    next.kp = static_cast<float>(node->get_parameter(std::string(kParamPrefix) + "kp").as_double());
    next.ki = static_cast<float>(node->get_parameter(std::string(kParamPrefix) + "ki").as_double());
    next.kd = static_cast<float>(node->get_parameter(std::string(kParamPrefix) + "kd").as_double());
    next.kf = static_cast<float>(node->get_parameter(std::string(kParamPrefix) + "kf").as_double());
    next.i_out_max = static_cast<float>(node->get_parameter(std::string(kParamPrefix) + "i_out_max").as_double());
    next.out_limit_scale = static_cast<float>(node->get_parameter(std::string(kParamPrefix) + "out_limit_scale").as_double());
    next.dt = static_cast<float>(node->get_parameter(std::string(kParamPrefix) + "dt").as_double());
    next.dead_zone = static_cast<float>(node->get_parameter(std::string(kParamPrefix) + "dead_zone").as_double());

    std::lock_guard<std::mutex> lock(config_mtx_);
    config_ = next;
    Reinit_PID_Locked();
    heading_locked_ = false;
}

void Class_Chariot_Imu_Heading_Hold::Reinit_PID_Locked()
{
    const float out_max = std::max(0.0f, config_.out_limit_scale) * max_chassis_omega_;
    heading_pid_.Init(config_.kp,
                      config_.ki,
                      config_.kd,
                      config_.kf,
                      config_.i_out_max,
                      out_max,
                      config_.dt,
                      config_.dead_zone);
}

rcl_interfaces::msg::SetParametersResult Class_Chariot_Imu_Heading_Hold::On_Set_Parameters(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::lock_guard<std::mutex> lock(config_mtx_);
    Config next = config_;
    bool pid_changed = false;

    for (const auto &parameter : parameters)
    {
        const std::string &name = parameter.get_name();
        if (name.rfind(kParamPrefix, 0) != 0)
            continue;

        const std::string key = name.substr(std::string(kParamPrefix).size());
        double value = 0.0;

        if (key == "enable")
        {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                result.successful = false;
                result.reason = name + " must be bool";
                return result;
            }
            next.enable = parameter.as_bool();
        }
        else if (key == "topic")
        {
            result.successful = false;
            result.reason = name + " is only read at module start";
            return result;
        }
        else if (!parameter_to_double(parameter, value))
        {
            result.successful = false;
            result.reason = name + " must be numeric";
            return result;
        }
        else if (key == "timeout_ms")
        {
            if (value <= 0.0)
            {
                result.successful = false;
                result.reason = name + " must be > 0";
                return result;
            }
            next.timeout_ns = static_cast<int64_t>(value * 1.0e6);
        }
        else if (key == "turn_eps")
        {
            next.turn_eps = std::max(0.0f, static_cast<float>(value));
        }
        else if (key == "move_eps")
        {
            next.move_eps = std::max(0.0f, static_cast<float>(value));
        }
        else if (key == "kp")
        {
            next.kp = static_cast<float>(value);
            pid_changed = true;
        }
        else if (key == "ki")
        {
            next.ki = static_cast<float>(value);
            pid_changed = true;
        }
        else if (key == "kd")
        {
            next.kd = static_cast<float>(value);
            pid_changed = true;
        }
        else if (key == "kf")
        {
            next.kf = static_cast<float>(value);
            pid_changed = true;
        }
        else if (key == "i_out_max")
        {
            next.i_out_max = std::max(0.0f, static_cast<float>(value));
            pid_changed = true;
        }
        else if (key == "out_limit_scale")
        {
            next.out_limit_scale = std::max(0.0f, static_cast<float>(value));
            pid_changed = true;
        }
        else if (key == "dt")
        {
            if (value <= 0.0)
            {
                result.successful = false;
                result.reason = name + " must be > 0";
                return result;
            }
            next.dt = static_cast<float>(value);
            pid_changed = true;
        }
        else if (key == "dead_zone")
        {
            next.dead_zone = std::max(0.0f, static_cast<float>(value));
            pid_changed = true;
        }
    }

    config_ = next;
    if (pid_changed)
    {
        Reinit_PID_Locked();
        heading_locked_ = false;
    }

    return result;
}

int64_t Class_Chariot_Imu_Heading_Hold::now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

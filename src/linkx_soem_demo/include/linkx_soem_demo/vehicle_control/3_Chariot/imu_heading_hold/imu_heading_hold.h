#ifndef IMU_HEADING_HOLD_H
#define IMU_HEADING_HOLD_H

#include "alg_pid.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class Class_Chariot_Imu_Heading_Hold
{
public:
    struct Snapshot
    {
        bool valid = false;
        bool fresh = false;
        int64_t age_ms = -1;
        float roll_rad = 0.0f;
        float pitch_rad = 0.0f;
        float yaw_rad = 0.0f;
        float angular_velocity_x = 0.0f;
        float angular_velocity_y = 0.0f;
        float angular_velocity_z = 0.0f;
        float linear_acceleration_x = 0.0f;
        float linear_acceleration_y = 0.0f;
        float linear_acceleration_z = 0.0f;
    };

    void Init(float max_chassis_omega);
    void Start(const std::shared_ptr<rclcpp::Node> &node);
    void Stop();
    Snapshot Get_Snapshot();

    float Correct_Omega(float vx_cmd,
                        float vy_cmd,
                        float omega_cmd,
                        bool lift_diff_mode,
                        int64_t now_ns);

protected:
    struct Config
    {
        bool enable = true;
        std::string topic = "/IMU_data";
        int64_t timeout_ns = 100LL * 1000LL * 1000LL;
        float turn_eps = 0.05f;
        float move_eps = 0.02f;
        float kp = 3.5f;
        float ki = 0.0f;
        float kd = 0.03f;
        float kf = 0.0f;
        float i_out_max = 0.0f;
        float out_limit_scale = 0.8f;
        float dt = 0.001f;
        float dead_zone = 0.01f;
    };

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    Class_PID heading_pid_;
    std::atomic<float> latest_imu_roll_{0.0f};
    std::atomic<float> latest_imu_pitch_{0.0f};
    std::atomic<float> latest_imu_yaw_{0.0f};
    std::atomic<float> latest_imu_omega_x_{0.0f};
    std::atomic<float> latest_imu_omega_y_{0.0f};
    std::atomic<float> latest_imu_omega_z_{0.0f};
    std::atomic<float> latest_imu_accel_x_{0.0f};
    std::atomic<float> latest_imu_accel_y_{0.0f};
    std::atomic<float> latest_imu_accel_z_{0.0f};
    std::atomic<int64_t> latest_imu_ns_{0};
    std::atomic<bool> imu_valid_{false};

    float heading_target_ = 0.0f;
    bool heading_locked_ = false;
    float max_chassis_omega_ = 0.0f;
    Config config_;
    std::mutex config_mtx_;

    void Reset_Heading_Lock();
    void Reset_Heading_Lock_Locked();
    void Handle_Imu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void Declare_Parameters(const std::shared_ptr<rclcpp::Node> &node);
    void Load_Parameters(const std::shared_ptr<rclcpp::Node> &node);
    void Reinit_PID_Locked();
    rcl_interfaces::msg::SetParametersResult On_Set_Parameters(
        const std::vector<rclcpp::Parameter> &parameters);

    static int64_t now_ns();
};

#endif // IMU_HEADING_HOLD_H

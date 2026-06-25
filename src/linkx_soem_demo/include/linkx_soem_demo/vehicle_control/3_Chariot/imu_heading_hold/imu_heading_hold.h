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

    /* 上一次 Correct_Omega 是否处于“航向纠偏激活”分支（即返回的 omega 是 PID 纠偏量，
     * 而非用户主动转向透传）。供底盘据此启用“只降速纠偏”。 */
    bool Was_Correcting() const { return last_correcting_.load(); }

protected:
    struct Config
    {
        bool enable = true;
        std::string topic = "/IMU_data";
        int64_t timeout_ns = 100LL * 1000LL * 1000LL;
        float turn_eps = 0.05f;
        float move_eps = 0.02f;
        // 2026-06-25 slow-only 纠偏直行调参收敛值（cmd=1.0 工作点 retune：kp→7.5、kd→0.12）：
        // W1/W2 受载力矩饥饿未修，配 slow-only 纠偏，cmd=1.0(实际 ~0.55 m/s)下 3m 前后可跑直，
        // yaw_rms ~2.3°；残留横漂 ~20cm/3m 是 W1/W2 弱导致的系统性蟹行(航向闭环纠不了横向)。
        // corrections 维持原值（配平候选均更差）。cmd≥1.2 失稳，1.0 为硬件不修下的最高可用档。
        float kp = 7.5f;
        float ki = 0.0f;
        float kd = 0.12f;
        float kf = 0.0f;
        float i_out_max = 0.0f;
        float out_limit_scale = 0.6f;  // autotune 20260624 直行调参推荐值（原 0.8）
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
    std::atomic<bool> last_correcting_{false};

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

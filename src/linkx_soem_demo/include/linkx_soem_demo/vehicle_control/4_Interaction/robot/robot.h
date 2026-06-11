#ifndef ROBOT_H
#define ROBOT_H

// 上位机版机器人编排器 (Class_Robot)：
//   - 串口/EtherCAT 控制：通过 linkx_t 句柄经 LinkX-4C → CAN → DM 电机
//   - ROS 接口：订阅 /chassis/cmd_vel(高优先级) + /chassis/remote_cmd_vel(遥控)
//              + /chassis/buttons + /imu，发布 /chassis/odom_twist
//   - 周期：每 1 ms 由 task.cpp 调用 TIM_1ms_Calculate_Callback / Loop / CAN_Rx_Callback

#include "crt_chassis_omni.h"
#include "crt_lift.h"
#include "crt_navigation.h"
#include "dvc_ops.h"
#include "linkx_soem_demo/remote/device/dvc_logF710.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int16.hpp>

class Class_Robot {
public:
    Class_Chassis_Omni Chassis;
    Class_Chariot_Lift Lift;
    Class_Navigation   Navigation;
    Class_OPS          ops;

    void Init(linkx_t *__LinkX_Handler);
    void Loop();
    void CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data, uint8_t CAN_Dlen = 8);

    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_2ms_Calculate_PeriodElapsedCallback();
    void TIM_1ms_Calculate_Callback();

    void Start_ROS2_Bridge();
    void Stop_ROS2_Bridge();

protected:
    enum class ChassisCommandSource {
        NONE,
        ROS2,
        REMOTE
    };

    struct Velocity_Command {
        float    vx            = 0.0f;
        float    vy            = 0.0f;
        float    omega         = 0.0f;
        float    right_y       = 0.0f;
        bool     has_twist     = false;
        int64_t  last_twist_ns = 0;
    };

    struct Ros_Command {
        Velocity_Command ros_twist;
        Velocity_Command remote_twist;
        uint16_t button_code   = LogF710_Key_IDLE;
        bool     has_buttons   = false;
        int64_t  last_button_ns = 0;
    };

    linkx_t *LinkX_Handler = nullptr;

    std::shared_ptr<rclcpp::Node>                                          bridge_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr             sub_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr             sub_remote_cmd_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr                 sub_buttons_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                 sub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                pub_odom_;
    std::thread                                                            ros_spin_thread_;
    std::atomic<bool>                                                      ros_bridge_running_{false};
    bool                                                                   ros_initialized_here_ = false;

    std::mutex                                                             cmd_mtx_;
    Ros_Command                                                            ros_cmd_;
    std::atomic<float>                                                     latest_imu_omega_z_{0.0f};

    int         odom_pub_divider_     = 0;

    float AHRS_Chassis_Omega_Feedforward = 0.07f;

    bool    chassis_remote_enabled_      = false;
    uint16_t last_chassis_button_code_   = LogF710_Key_IDLE;
    uint16_t last_chassis_rx_button_code_ = 0xFFFFU;
    uint16_t last_lift_button_code_      = LogF710_Key_IDLE;
    int64_t last_chassis_diag_ns_        = 0;
    bool    ops_yaw_hold_active_         = false;
    float   ops_yaw_hold_target_         = 0.0f;  // deg
    float   ops_yaw_hold_last_output_    = 0.0f;
    float   ops_yaw_hold_last_error_     = 0.0f;  // deg
    bool    ops_lateral_hold_active_     = false;
    float   ops_lateral_hold_yaw_target_ = 0.0f;  // deg
    float   ops_lateral_hold_start_x_    = 0.0f;
    float   ops_lateral_hold_start_y_    = 0.0f;
    float   ops_lateral_hold_body_dir_x_ = 1.0f;
    float   ops_lateral_hold_body_dir_y_ = 0.0f;
    float   ops_lateral_hold_last_error_mm_ = 0.0f;
    float   ops_lateral_hold_last_output_   = 0.0f;

    // 诊断：被 CAN_Rx_Callback 丢弃的未识别帧累计（通道/ID 都不匹配）
    std::atomic<uint64_t> unhandled_can_frames_{0};

    void _Chassis_Control();
    void _Lift_Control();
    void _Send_Odometry();
    void _Apply_OPS_Lateral_Correction(float &vx, float &vy, float omega_cmd);
    float _Apply_OPS_Yaw_Hold(float vx, float vy, float omega_cmd);
    void _Reset_OPS_Lateral_Hold();
    void _ROS2_Spin_Loop();
    void _Update_Twist(Velocity_Command &target, float vx, float vy, float omega, float right_y);
    void _Update_Ros_Twist(float vx, float vy, float omega, float right_y);
    void _Update_Remote_Twist(float vx, float vy, float omega, float right_y);
    void _Update_Button_Code(uint16_t button_code);
    void _Update_Chassis_Remote_Gate(bool buttons_recent, uint16_t button_code);
    void _Log_Chassis_Start_Gate(const char *msg);
    void _Log_Chassis_Diagnostic(bool ros_cmd_recent,
                                 bool remote_cmd_recent,
                                 ChassisCommandSource source);

    // 启动期检查：同一通道内 DM_CAN_Rx_ID 不能重号
    void _Verify_Motor_ID_Uniqueness();

    static int64_t now_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }
};

extern Class_Robot robot;

#endif

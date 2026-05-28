#ifndef ROBOT_H
#define ROBOT_H

// 上位机版机器人编排器 (Class_Robot)：
//   - 串口/EtherCAT 控制：通过 linkx_t 句柄经 LinkX-4C → CAN → DM 电机
//   - ROS 接口：订阅 /chassis/cmd_vel + /chassis/gantry_state + /imu，发布 /chassis/odom_twist
//   - 周期：每 1 ms 由 task.cpp 调用 TIM_1ms_Calculate_Callback / Loop / CAN_Rx_Callback

#include "crt_chassis_omni.h"
#include "crt_gantry.h"
#include "crt_arm.h"
#include "crt_navigation.h"
#include "dvc_ops.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int8.hpp>

class Class_Robot {
public:
    Class_Chassis_Omni Chassis;
    Class_Gantry       Gantry;
    Class_Arm          Arm;
    Class_Navigation   Navigation;
    Class_OPS          ops;

    void Init(linkx_t *__LinkX_Handler);
    void Loop();
    void CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data);

    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_2ms_Calculate_PeriodElapsedCallback();
    void TIM_1ms_Calculate_Callback();

    void Start_ROS2_Bridge();
    void Stop_ROS2_Bridge();

protected:
    enum GantryState {
        GANTRY_STATE_DISABLE,
        GANTRY_STATE_LOW_POS,
        GANTRY_STATE_HIGH_POS
    };

    struct Ros_Command {
        float    vx            = 0.0f;
        float    vy            = 0.0f;
        float    omega         = 0.0f;
        uint8_t  gantry_state  = 0xFF;
        bool     has_twist     = false;
        bool     has_gantry    = false;
        int64_t  last_twist_ns = 0;
    };

    linkx_t *LinkX_Handler = nullptr;

    std::shared_ptr<rclcpp::Node>                                          bridge_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr             sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr                  sub_gantry_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                 sub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                pub_odom_;
    std::thread                                                            ros_spin_thread_;
    std::atomic<bool>                                                      ros_bridge_running_{false};
    bool                                                                   ros_initialized_here_ = false;

    std::mutex                                                             cmd_mtx_;
    Ros_Command                                                            ros_cmd_;
    std::atomic<float>                                                     latest_imu_omega_z_{0.0f};

    GantryState current_gantry_state_ = GANTRY_STATE_DISABLE;
    uint32_t    Arm_timer_            = 0;
    uint32_t    gantry_timer_         = 0;
    int         odom_pub_divider_     = 0;

    float AHRS_Chassis_Omega_Feedforward = 0.07f;

    // 诊断：被 CAN_Rx_Callback 丢弃的未识别帧累计（通道/ID 都不匹配）
    std::atomic<uint64_t> unhandled_can_frames_{0};

    void _Chassis_Control();
    void _Gantry_control();
    void _Send_Odometry();
    void _ROS2_Spin_Loop();
    void _Update_Twist(float vx, float vy, float omega);
    void _Update_Gantry_State(uint8_t state);

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

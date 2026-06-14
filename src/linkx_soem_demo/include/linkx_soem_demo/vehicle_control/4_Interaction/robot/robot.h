#ifndef ROBOT_H
#define ROBOT_H

// 上位机版机器人编排器 (Class_Robot)：
//   - 串口/EtherCAT 控制：通过 linkx_t 句柄经 LinkX-4C → CAN → DM 电机
//   - ROS 接口：订阅 /chassis/cmd_vel(高优先级) + /chassis/remote_cmd_vel(遥控)
//              + /chassis/buttons，IMU 航向保持模块订阅 /IMU_data，发布 /chassis/odom_twist
//   - 周期：每 1 ms 由 task.cpp 调用 TIM_1ms_Calculate_Callback / Loop / CAN_Rx_Callback

#include "crt_chassis_omni.h"
#include "crt_gripper.h"
#include "crt_lift.h"
#include "dvc_motor_dm.h"
#include "imu_heading_hold/imu_heading_hold.h"
#include "linkx_soem_demo/remote/device/dvc_logF710.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/u_int16.hpp>

class Class_Robot {
public:
    struct ButtonSnapshot {
        uint16_t code = LogF710_Key_IDLE;
        bool has_buttons = false;
        bool recent = false;
        int64_t age_ms = -1;
        int64_t last_button_ns = 0;
    };

    Class_Chassis_Omni Chassis;
    Class_Chariot_Lift Lift;
    Class_Chariot_Gripper Gripper;
    Class_Motor_DM_Normal Auxiliary_Motor;

    void Init(linkx_t *__LinkX_Handler);
    void Loop();
    void CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data, uint8_t CAN_Dlen = 8);

    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_2ms_Calculate_PeriodElapsedCallback();
    void TIM_1ms_Calculate_Callback();

    void Start_ROS2_Bridge();
    void Stop_ROS2_Bridge();
    ButtonSnapshot Get_Button_Snapshot();

protected:
    enum class ChassisCommandSource {
        NONE,
        ROS2,
        REMOTE
    };

    enum class LiftAuxSequenceState {
        IDLE,
        RAISE_WAIT_LIFT_REACHED,
        RAISE_MOVE_AUXILIARY,
        RAISE_DONE,
        HOME_WAIT_LIFT_REACHED,
        HOME_MOVE_AUXILIARY,
        HOME_DONE,
    };

    enum class HighPriorityAction {
        VEHICLE_ENABLE,
        VEHICLE_DISABLE,
        STAIR_UP_RAISE_8_0,
        STAIR_DOWN_RAISE_8_0,
        STAIR_UP_RAISE_14_3,
        STAIR_DOWN_RAISE_14_3,
        LIFT_AUX_RAISE,
        LIFT_AUX_HOME,
        GRIPPER_GRAB,
        GRIPPER_RELEASE,
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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                pub_odom_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                     srv_vehicle_enable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                     srv_vehicle_disable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                     srv_stair_up_raise_8_0_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                     srv_stair_down_raise_8_0_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                     srv_stair_up_raise_14_3_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                     srv_stair_down_raise_14_3_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                     srv_lift_aux_raise_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                     srv_lift_aux_home_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                     srv_gripper_grab_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                     srv_gripper_release_;
    std::thread                                                            ros_spin_thread_;
    std::atomic<bool>                                                      ros_bridge_running_{false};
    bool                                                                   ros_initialized_here_ = false;

    std::mutex                                                             cmd_mtx_;
    Ros_Command                                                            ros_cmd_;
    std::deque<HighPriorityAction>                                         high_priority_actions_;
    Class_Chariot_Imu_Heading_Hold                                         imu_heading_hold_;

    int         odom_pub_divider_     = 0;

    float AHRS_Chassis_Omega_Feedforward = 0.07f;

    bool    chassis_remote_enabled_      = false;
    uint16_t last_chassis_button_code_   = LogF710_Key_IDLE;
    uint16_t last_chassis_rx_button_code_ = 0xFFFFU;
    uint16_t last_lift_button_code_      = LogF710_Key_IDLE;
    uint16_t last_gripper_button_code_   = LogF710_Key_IDLE;
    int64_t last_chassis_diag_ns_        = 0;
    bool suppress_remote_buttons_this_tick_ = false;
    LiftAuxSequenceState lift_aux_sequence_state_ = LiftAuxSequenceState::IDLE;
    float auxiliary_motor_target_angle_ = 0.0f;
    bool auxiliary_motor_command_enable_ = false;

    // 诊断：被 CAN_Rx_Callback 丢弃的未识别帧累计（通道/ID 都不匹配）
    std::atomic<uint64_t> unhandled_can_frames_{0};

    void _Chassis_Control();
    void _Lift_Control();
    void _Gripper_Control(bool buttons_recent, uint16_t button_code);
    void _Send_Odometry();
    void _ROS2_Spin_Loop();
    void _Register_Action_Services();
    bool _Enqueue_High_Priority_Action(HighPriorityAction action, const char *name);
    bool _Process_High_Priority_Actions();
    void _Execute_High_Priority_Action(HighPriorityAction action);
    void _Enable_Vehicle_Control(const char *reason);
    void _Disable_Vehicle_Control(const char *reason);
    void _Update_Twist(Velocity_Command &target, float vx, float vy, float omega, float right_y);
    void _Update_Ros_Twist(float vx, float vy, float omega, float right_y);
    void _Update_Remote_Twist(float vx, float vy, float omega, float right_y);
    void _Update_Button_Code(uint16_t button_code);
    void _Update_Chassis_Remote_Gate(bool buttons_recent, uint16_t button_code);
    void _Enable_Auxiliary_Motor();
    void _Disable_Auxiliary_Motor();
    void _Cancel_Lift_Aux_Sequence();
    void _Start_Lift_Aux_Raise_Sequence();
    void _Start_Lift_Aux_Home_Sequence();
    void _Update_Lift_Aux_Sequence();
    void _Output_Auxiliary_Motor();
    bool _Is_Auxiliary_Motor_Reached(float target_angle);
    void _Log_Chassis_Start_Gate(const char *msg);
    void _Log_Gripper_Action(const char *msg, bool ok);
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

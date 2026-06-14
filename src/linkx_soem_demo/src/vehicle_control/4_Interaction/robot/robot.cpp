// 上位机版 Class_Robot 实现：把 ROS /chassis/cmd_vel(高优先级)
// + /chassis/remote_cmd_vel(遥控低优先级) + /chassis/buttons
// 转化为底盘速度命令；IMU 航向保持模块订阅 /IMU_data；
// 通过 linkx_t 推送给 EtherCAT-CAN 桥；
// 把 chassis 实际速度回报到 /chassis/odom_twist。
//
// 工况：全向轮底盘（4 × DM3519 MIT，分布在 LinkX channel 0/1）
//      + 前/后抬升机构（新车头为原车尾：front=ch0，rear=ch1）
//      + 气夹爪下位机（默认 LinkX CAN0，ID 0x06）
//      + 辅助 DM 电机（LinkX CAN0，Tx ID 0x07，Rx ID 0x17）
//
// 调度:
//   robot.Init(linkx) 由 task::Robot_Control_Loop 在 SOEM 主站启动后调用一次
//   robot.CAN_Rx_Callback(ch, id, data) 由主循环 1ms 内 drain 每帧调用
//   robot.Loop() / TIM_1ms_Calculate_Callback() 每 1ms 调一次
//   robot.TIM_2ms_Calculate_PeriodElapsedCallback() 每 2ms 调一次
//   robot.TIM_100ms_Alive_PeriodElapsedCallback() 每 100ms 调一次

#include "robot.h"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>

namespace
{
constexpr int64_t kCmdTimeoutNs = 200LL * 1000LL * 1000LL;  // 200 ms
constexpr int64_t kButtonTimeoutNs = 500LL * 1000LL * 1000LL;  // 500 ms
constexpr const char *kRosCmdTopic = "/chassis/cmd_vel";
constexpr const char *kRemoteCmdTopic = "/chassis/remote_cmd_vel";
constexpr float kManualBothLiftMotorRaiseAngle = -39.0f;
constexpr float kManualBothLiftRetractAngle = -0.01f;
constexpr float kAuxiliaryMotorRaisedAngle = 1.5f;
constexpr float kAuxiliaryMotorHomeAngle = 0.1f;
constexpr float kAuxiliaryMotorReachedTolerance = 0.05f;
constexpr float kAuxiliaryMotorKp = 20.0f;
constexpr float kAuxiliaryMotorKd = 1.2f;
constexpr uint8_t kAuxiliaryMotorCanChannel = 0U;
constexpr uint8_t kAuxiliaryMotorRxId = 0x17U;
constexpr uint8_t kAuxiliaryMotorTxId = 0x07U;
constexpr size_t kHighPriorityActionQueueMax = 16U;
}

void Class_Robot::Init(linkx_t *__LinkX_Handler)
{
    LinkX_Handler = __LinkX_Handler;

    Chassis.Init(LinkX_Handler);
    Chassis.Init_Motor_Params();
    imu_heading_hold_.Init(MAX_OMNI_CHASSIS_OMEGA);

    Lift.Init(LinkX_Handler);
    Gripper.Init(LinkX_Handler);
    Auxiliary_Motor.Init(LinkX_Handler,
                         kAuxiliaryMotorCanChannel,
                         kAuxiliaryMotorRxId,
                         kAuxiliaryMotorTxId,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f,
                         395.0f,
                         7.8f,
                         9.2f);
    Auxiliary_Motor.Set_Use_FDCAN(true);
    Auxiliary_Motor.Set_Force_Output_Without_Feedback(false);

    _Verify_Motor_ID_Uniqueness();
}

void Class_Robot::_Verify_Motor_ID_Uniqueness()
{
    // R2 CAN 通道分布（新车头为原车尾）：
    //   ch0：index 1/2 全向轮（Tx 0x02/0x01）+ 前抬升（Tx 0x03/0x04/0x05）+ 辅助电机（Tx 0x07）
    //   ch1：index 0/3 全向轮（Tx 0x02/0x01）+ 后抬升（Tx 0x03/0x04/0x05）
    // 同通道内 Rx ID 必须互不相同。

    auto warn_dup = [](const char *channel_name, int a, int b, uint16_t id) {
        std::cerr << "[ROBOT][WARN] Same-channel DM CAN Rx ID collision on "
                  << channel_name << " between motor[" << a << "] and motor[" << b
                  << "] (Rx_ID=0x" << std::hex << id << std::dec << ")." << std::endl;
    };

    auto check_pair = [&](const char *channel_name, uint16_t ids[], int n) {
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
                if (ids[i] == ids[j]) warn_dup(channel_name, i, j, ids[i]);
    };

    uint16_t ch0_ids[6] = {
        Chassis.Motor_Wheel[1].DM_CAN_Rx_ID,
        Chassis.Motor_Wheel[2].DM_CAN_Rx_ID,
        Lift.Motor_Drive_Left[CHARIOT_LIFT_MODULE_FRONT].DM_CAN_Rx_ID,
        Lift.Motor_Drive_Right[CHARIOT_LIFT_MODULE_FRONT].DM_CAN_Rx_ID,
        Lift.Motor_Lift[CHARIOT_LIFT_MODULE_FRONT].DM_CAN_Rx_ID,
        Auxiliary_Motor.DM_CAN_Rx_ID,
    };
    check_pair("ch0 (omni+front lift+aux motor)", ch0_ids, 6);

    uint16_t ch1_ids[5] = {
        Chassis.Motor_Wheel[0].DM_CAN_Rx_ID,
        Chassis.Motor_Wheel[3].DM_CAN_Rx_ID,
        Lift.Motor_Drive_Left[CHARIOT_LIFT_MODULE_REAR].DM_CAN_Rx_ID,
        Lift.Motor_Drive_Right[CHARIOT_LIFT_MODULE_REAR].DM_CAN_Rx_ID,
        Lift.Motor_Lift[CHARIOT_LIFT_MODULE_REAR].DM_CAN_Rx_ID,
    };
    check_pair("ch1 (omni+rear lift)", ch1_ids, 5);

}

void Class_Robot::Loop()
{
    _Chassis_Control();
    _Lift_Control();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
    Lift.TIM_2ms_Control_PeriodElapsedCallback();
}

void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
    Lift.TIM_100ms_Alive_PeriodElapsedCallback();
    Auxiliary_Motor.TIM_Alive_PeriodElapsedCallback();
}

void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
    Lift.TIM_2ms_Control_PeriodElapsedCallback();
}

void Class_Robot::TIM_1ms_Calculate_Callback()
{
    suppress_remote_buttons_this_tick_ = _Process_High_Priority_Actions();
    _Chassis_Control();
    _Lift_Control();
    _Send_Odometry();
    suppress_remote_buttons_this_tick_ = false;
}

// --- ROS 桥 ----------------------------------------------------------------

void Class_Robot::Start_ROS2_Bridge()
{
    if (ros_bridge_running_.load()) return;

    if (!rclcpp::ok())
    {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
        ros_initialized_here_ = true;
    }

    bridge_node_ = std::make_shared<rclcpp::Node>("r2_vehicle_bridge");

    sub_cmd_ = bridge_node_->create_subscription<geometry_msgs::msg::Twist>(
        kRosCmdTopic, 20,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            _Update_Ros_Twist(static_cast<float>(msg->linear.x),
                              static_cast<float>(msg->linear.y),
                              static_cast<float>(msg->angular.z),
                              static_cast<float>(msg->angular.x));
        });

    sub_remote_cmd_ = bridge_node_->create_subscription<geometry_msgs::msg::Twist>(
        kRemoteCmdTopic, 20,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            _Update_Remote_Twist(static_cast<float>(msg->linear.x),
                                 static_cast<float>(msg->linear.y),
                                 static_cast<float>(msg->angular.z),
                                 static_cast<float>(msg->angular.x));
        });

    sub_buttons_ = bridge_node_->create_subscription<std_msgs::msg::UInt16>(
        "/chassis/buttons", 20,
        [this](const std_msgs::msg::UInt16::SharedPtr msg) { _Update_Button_Code(msg->data); });

    imu_heading_hold_.Start(bridge_node_);

    pub_odom_ = bridge_node_->create_publisher<geometry_msgs::msg::Twist>("/chassis/odom_twist", 50);
    _Register_Action_Services();

    RCLCPP_INFO(bridge_node_->get_logger(),
                "command priority: action services > ROS2 %s > remote %s",
                kRosCmdTopic,
                kRemoteCmdTopic);

    ros_bridge_running_.store(true);
    ros_spin_thread_ = std::thread(&Class_Robot::_ROS2_Spin_Loop, this);
}

void Class_Robot::Stop_ROS2_Bridge()
{
    if (!ros_bridge_running_.load()) return;
    ros_bridge_running_.store(false);
    if (ros_spin_thread_.joinable()) ros_spin_thread_.join();

    srv_gripper_release_.reset();
    srv_gripper_grab_.reset();
    srv_lift_aux_home_.reset();
    srv_lift_aux_raise_.reset();
    srv_stair_down_raise_14_3_.reset();
    srv_stair_up_raise_14_3_.reset();
    srv_stair_down_raise_8_0_.reset();
    srv_stair_up_raise_8_0_.reset();
    srv_vehicle_disable_.reset();
    srv_vehicle_enable_.reset();
    pub_odom_.reset();
    imu_heading_hold_.Stop();
    sub_buttons_.reset();
    sub_remote_cmd_.reset();
    sub_cmd_.reset();
    bridge_node_.reset();

    if (ros_initialized_here_ && rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    ros_initialized_here_ = false;
}

void Class_Robot::_ROS2_Spin_Loop()
{
    rclcpp::WallRate rate(200);
    while (ros_bridge_running_.load() && rclcpp::ok())
    {
        rclcpp::spin_some(bridge_node_);
        rate.sleep();
    }
}

void Class_Robot::_Register_Action_Services()
{
    if (!bridge_node_)
        return;

    auto make_trigger_service =
        [this](const char *service_name, HighPriorityAction action)
        -> rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        {
            return bridge_node_->create_service<std_srvs::srv::Trigger>(
                service_name,
                [this, action, service_name](
                    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
                {
                    (void)request;
                    const bool queued = _Enqueue_High_Priority_Action(action, service_name);
                    response->success = queued;
                    response->message = queued ?
                        (std::string("accepted: ") + service_name) :
                        (std::string("rejected, action queue full: ") + service_name);
                });
        };

    srv_vehicle_enable_ =
        make_trigger_service("/vehicle/enable", HighPriorityAction::VEHICLE_ENABLE);
    srv_vehicle_disable_ =
        make_trigger_service("/vehicle/disable", HighPriorityAction::VEHICLE_DISABLE);
    srv_stair_up_raise_8_0_ =
        make_trigger_service("/vehicle/stair/up_raise_8_0",
                             HighPriorityAction::STAIR_UP_RAISE_8_0);
    srv_stair_down_raise_8_0_ =
        make_trigger_service("/vehicle/stair/down_raise_8_0",
                             HighPriorityAction::STAIR_DOWN_RAISE_8_0);
    srv_stair_up_raise_14_3_ =
        make_trigger_service("/vehicle/stair/up_raise_14_3",
                             HighPriorityAction::STAIR_UP_RAISE_14_3);
    srv_stair_down_raise_14_3_ =
        make_trigger_service("/vehicle/stair/down_raise_14_3",
                             HighPriorityAction::STAIR_DOWN_RAISE_14_3);
    srv_lift_aux_raise_ =
        make_trigger_service("/vehicle/lift_aux/raise", HighPriorityAction::LIFT_AUX_RAISE);
    srv_lift_aux_home_ =
        make_trigger_service("/vehicle/lift_aux/home", HighPriorityAction::LIFT_AUX_HOME);
    srv_gripper_grab_ =
        make_trigger_service("/vehicle/gripper/grab", HighPriorityAction::GRIPPER_GRAB);
    srv_gripper_release_ =
        make_trigger_service("/vehicle/gripper/release", HighPriorityAction::GRIPPER_RELEASE);
}

bool Class_Robot::_Enqueue_High_Priority_Action(HighPriorityAction action, const char *name)
{
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    if (high_priority_actions_.size() >= kHighPriorityActionQueueMax)
    {
        if (bridge_node_)
        {
            RCLCPP_ERROR(bridge_node_->get_logger(),
                         "high-priority action queue full, reject %s",
                         name);
        }
        return false;
    }

    high_priority_actions_.push_back(action);
    if (bridge_node_)
    {
        RCLCPP_WARN(bridge_node_->get_logger(),
                    "high-priority action queued: %s",
                    name);
    }
    return true;
}

bool Class_Robot::_Process_High_Priority_Actions()
{
    std::deque<HighPriorityAction> actions;
    {
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        actions.swap(high_priority_actions_);
    }

    const bool had_actions = !actions.empty();
    while (!actions.empty())
    {
        _Execute_High_Priority_Action(actions.front());
        actions.pop_front();
    }

    return had_actions;
}

void Class_Robot::_Enable_Vehicle_Control(const char *reason)
{
    chassis_remote_enabled_ = true;
    Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    Lift.Send_Enable_Burst();
    _Enable_Auxiliary_Motor();
    _Log_Chassis_Start_Gate(reason);
}

void Class_Robot::_Disable_Vehicle_Control(const char *reason)
{
    chassis_remote_enabled_ = false;
    _Cancel_Lift_Aux_Sequence();
    Lift.Stop_Stair_Auto();
    Lift.Set_Diff_Drive_Enable(false);
    Lift.Set_Target_Diff_Command(0.0f, 0.0f);
    Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_DISABLE);
    Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);
    Chassis.Set_Target_Velocity_X(0.0f);
    Chassis.Set_Target_Velocity_Y(0.0f);
    Chassis.Set_Target_Omega(0.0f);
    _Disable_Auxiliary_Motor();

    {
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        ros_cmd_.ros_twist = Velocity_Command{};
        ros_cmd_.remote_twist = Velocity_Command{};
    }

    _Log_Chassis_Start_Gate(reason);
}

void Class_Robot::_Execute_High_Priority_Action(HighPriorityAction action)
{
    switch (action)
    {
        case HighPriorityAction::VEHICLE_ENABLE:
            _Enable_Vehicle_Control("service /vehicle/enable: vehicle ENABLED");
            break;

        case HighPriorityAction::VEHICLE_DISABLE:
            _Disable_Vehicle_Control("service /vehicle/disable: vehicle DISABLED");
            break;

        case HighPriorityAction::STAIR_UP_RAISE_8_0:
            _Enable_Vehicle_Control("service action: vehicle enabled for stair UP raise_angle=-8.0");
            _Cancel_Lift_Aux_Sequence();
            Lift.Start_Stair_Up(-8.0f);
            _Log_Chassis_Start_Gate("service /vehicle/stair/up_raise_8_0: stair UP auto start");
            break;

        case HighPriorityAction::STAIR_DOWN_RAISE_8_0:
            _Enable_Vehicle_Control("service action: vehicle enabled for stair DOWN raise_angle=-8.0");
            _Cancel_Lift_Aux_Sequence();
            Lift.Start_Stair_Down(-8.0f);
            _Log_Chassis_Start_Gate("service /vehicle/stair/down_raise_8_0: stair DOWN auto start");
            break;

        case HighPriorityAction::STAIR_UP_RAISE_14_3:
            _Enable_Vehicle_Control("service action: vehicle enabled for stair UP raise_angle=-14.3");
            _Cancel_Lift_Aux_Sequence();
            Lift.Start_Stair_Up(-14.3f);
            _Log_Chassis_Start_Gate("service /vehicle/stair/up_raise_14_3: stair UP auto start");
            break;

        case HighPriorityAction::STAIR_DOWN_RAISE_14_3:
            _Enable_Vehicle_Control("service action: vehicle enabled for stair DOWN raise_angle=-14.3");
            _Cancel_Lift_Aux_Sequence();
            Lift.Start_Stair_Down(-14.3f);
            _Log_Chassis_Start_Gate("service /vehicle/stair/down_raise_14_3: stair DOWN auto start");
            break;

        case HighPriorityAction::LIFT_AUX_RAISE:
            _Enable_Vehicle_Control("service action: vehicle enabled for lift+aux raise");
            _Start_Lift_Aux_Raise_Sequence();
            _Log_Chassis_Start_Gate("service /vehicle/lift_aux/raise: lift target=-39.0, then aux 0x07 target=1.5");
            break;

        case HighPriorityAction::LIFT_AUX_HOME:
            _Enable_Vehicle_Control("service action: vehicle enabled for lift+aux home");
            _Start_Lift_Aux_Home_Sequence();
            _Log_Chassis_Start_Gate("service /vehicle/lift_aux/home: aux 0x07 target=0.1, then lift target=-0.01");
            break;

        case HighPriorityAction::GRIPPER_GRAB: {
            const bool ok = Gripper.Grab();
            _Log_Gripper_Action(ok ? "service /vehicle/gripper/grab: gripper GRAB sent (0x01)"
                                   : "service /vehicle/gripper/grab: gripper GRAB send failed (0x01)",
                                ok);
            break;
        }

        case HighPriorityAction::GRIPPER_RELEASE: {
            const bool ok = Gripper.Release();
            _Log_Gripper_Action(ok ? "service /vehicle/gripper/release: gripper RELEASE sent (0x02)"
                                   : "service /vehicle/gripper/release: gripper RELEASE send failed (0x02)",
                                ok);
            break;
        }
    }
}

void Class_Robot::_Update_Twist(Velocity_Command &target,
                                float vx,
                                float vy,
                                float omega,
                                float right_y)
{
    target.vx = vx;
    target.vy = vy;
    target.omega = omega;
    target.right_y = right_y;
    target.has_twist = true;
    target.last_twist_ns = now_ns();
}

void Class_Robot::_Update_Ros_Twist(float vx, float vy, float omega, float right_y)
{
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    _Update_Twist(ros_cmd_.ros_twist, vx, vy, omega, right_y);
}

void Class_Robot::_Update_Remote_Twist(float vx, float vy, float omega, float right_y)
{
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    _Update_Twist(ros_cmd_.remote_twist, vx, vy, omega, right_y);
}

void Class_Robot::_Update_Button_Code(uint16_t button_code)
{
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    ros_cmd_.button_code = button_code;
    ros_cmd_.has_buttons = true;
    ros_cmd_.last_button_ns = now_ns();

    if (button_code != last_chassis_rx_button_code_)
    {
        last_chassis_rx_button_code_ = button_code;
        if (bridge_node_)
        {
            RCLCPP_WARN(bridge_node_->get_logger(),
                        "chassis button rx: code=0x%04X",
                        static_cast<unsigned>(button_code));
        }
    }
}

Class_Robot::ButtonSnapshot Class_Robot::Get_Button_Snapshot()
{
    std::lock_guard<std::mutex> lock(cmd_mtx_);

    ButtonSnapshot snapshot;
    snapshot.code = ros_cmd_.button_code;
    snapshot.has_buttons = ros_cmd_.has_buttons;
    snapshot.last_button_ns = ros_cmd_.last_button_ns;

    const int64_t now = now_ns();
    if (snapshot.has_buttons && snapshot.last_button_ns > 0)
    {
        const int64_t age_ns = now - snapshot.last_button_ns;
        snapshot.age_ms = age_ns / (1000LL * 1000LL);
        snapshot.recent = age_ns <= kButtonTimeoutNs;
    }

    return snapshot;
}

// --- CAN 路由 --------------------------------------------------------------
//
// 全向轮工况分发策略（新车头为原车尾）：
//   - ch0 → index 1/2 全向轮 + 前抬升 + 辅助电机，通过 Rx_ID 区分（0x11..0x17）
//   - ch1 → index 0/3 全向轮 + 后抬升，通过 Rx_ID 区分（0x11..0x15）
//   - ch2 → ToF(CH2 ID 0x01..0x04)
//   - 其它通道 / 通道+ID 不匹配 → 累计到 unhandled_can_frames_

void Class_Robot::CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data, uint8_t CAN_Dlen)
{
    const uint32_t can_id_std = (CAN_ID & 0x7FFU);

    auto dispatch_wheel = [&](const int *indices, int n) -> bool {
        for (int idx = 0; idx < n; ++idx)
        {
            const int i = indices[idx];
            if (can_id_std == Chassis.Motor_Wheel[i].DM_CAN_Rx_ID)
            {
                Chassis.Motor_Wheel[i].CAN_RxCpltCallback(CAN_Data);
                return true;
            }
        }
        return false;
    };

    bool handled = false;
    switch (CAN_Channel)
    {
        case 0: {
            const int indices[] = {1, 2};
            handled = dispatch_wheel(indices, 2);
            if (!handled)
                handled = Lift.CAN_Rx_Callback(CAN_Channel, CAN_ID, CAN_Data, CAN_Dlen);
            if (!handled && can_id_std == Auxiliary_Motor.DM_CAN_Rx_ID)
            {
                Auxiliary_Motor.CAN_RxCpltCallback(CAN_Data);
                handled = true;
            }
            break;
        }
        case 1: {
            const int indices[] = {0, 3};
            handled = dispatch_wheel(indices, 2);
            if (!handled)
                handled = Lift.CAN_Rx_Callback(CAN_Channel, CAN_ID, CAN_Data, CAN_Dlen);
            break;
        }
        case 2:
            handled = Lift.CAN_Rx_Callback(CAN_Channel, CAN_ID, CAN_Data, CAN_Dlen);
            break;
        default: /* ch3 未配置任何执行器 */  break;
    }

    if (!handled)
        unhandled_can_frames_.fetch_add(1, std::memory_order_relaxed);
}

// --- 控制策略 --------------------------------------------------------------

void Class_Robot::_Chassis_Control()
{
    Ros_Command snapshot;
    {
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        snapshot = ros_cmd_;
    }

    const int64_t now = now_ns();
    const bool ros_cmd_recent =
        snapshot.ros_twist.has_twist && snapshot.ros_twist.last_twist_ns > 0 &&
        (now - snapshot.ros_twist.last_twist_ns) <= kCmdTimeoutNs;
    const bool remote_cmd_recent =
        snapshot.remote_twist.has_twist && snapshot.remote_twist.last_twist_ns > 0 &&
        (now - snapshot.remote_twist.last_twist_ns) <= kCmdTimeoutNs;
    const bool buttons_recent_raw =
        snapshot.has_buttons && snapshot.last_button_ns > 0 &&
        (now - snapshot.last_button_ns) <= kButtonTimeoutNs;
    if (suppress_remote_buttons_this_tick_)
    {
        last_chassis_button_code_ = buttons_recent_raw ?
            snapshot.button_code :
            LogF710_Key_IDLE;
    }
    else
    {
        _Update_Chassis_Remote_Gate(buttons_recent_raw, snapshot.button_code);
    }

    const Velocity_Command *selected_cmd = nullptr;
    ChassisCommandSource selected_source = ChassisCommandSource::NONE;
    if (chassis_remote_enabled_ && ros_cmd_recent)
    {
        selected_cmd = &snapshot.ros_twist;
        selected_source = ChassisCommandSource::ROS2;
    }
    else if (chassis_remote_enabled_ && remote_cmd_recent)
    {
        selected_cmd = &snapshot.remote_twist;
        selected_source = ChassisCommandSource::REMOTE;
    }

    if (selected_cmd != nullptr)
    {
        float vx_cmd = selected_cmd->vx;
        float vy_cmd = selected_cmd->vy;
        const float linear_speed = std::sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
        if (linear_speed > MAX_OMNI_CHASSIS_SPEED && linear_speed > 1e-4f)
        {
            const float scale = MAX_OMNI_CHASSIS_SPEED / linear_speed;
            vx_cmd *= scale;
            vy_cmd *= scale;
        }

        const bool lift_diff_mode = Lift.Get_Diff_Drive_Enable();
        float omega_cmd = lift_diff_mode ? 0.0f : selected_cmd->omega;
        if (omega_cmd >  MAX_OMNI_CHASSIS_OMEGA) omega_cmd =  MAX_OMNI_CHASSIS_OMEGA;
        if (omega_cmd < -MAX_OMNI_CHASSIS_OMEGA) omega_cmd = -MAX_OMNI_CHASSIS_OMEGA;

        const float omega_out =
            imu_heading_hold_.Correct_Omega(vx_cmd, vy_cmd, omega_cmd, lift_diff_mode, now);

        Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_ENABLE);
        Chassis.Set_Target_Velocity_X(vx_cmd);
        Chassis.Set_Target_Velocity_Y(vy_cmd);
        Chassis.Set_Target_Omega(lift_diff_mode ? 0.0f : omega_out);
        if ((now - last_chassis_diag_ns_) >= 1000LL * 1000LL * 1000LL)
        {
            last_chassis_diag_ns_ = now;
            _Log_Chassis_Diagnostic(ros_cmd_recent, remote_cmd_recent, selected_source);
        }
    }
    else
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);
        Chassis.Set_Target_Velocity_X(0.0f);
        Chassis.Set_Target_Velocity_Y(0.0f);
        Chassis.Set_Target_Omega(0.0f);
    }
}

void Class_Robot::_Update_Chassis_Remote_Gate(bool buttons_recent, uint16_t button_code)
{
    if (!buttons_recent)
    {
        last_chassis_button_code_ = LogF710_Key_IDLE;
        return;
    }

    const bool start_pressed = (button_code == LogF710_Key_Start);
    const bool back_pressed  = (button_code == LogF710_Key_Back);
    const bool start_rising  = start_pressed && (last_chassis_button_code_ != LogF710_Key_Start);
    const bool back_rising   = back_pressed  && (last_chassis_button_code_ != LogF710_Key_Back);

    if (back_pressed)
    {
        if (chassis_remote_enabled_ || back_rising)
        {
            _Disable_Vehicle_Control("BACK pressed: vehicle DISABLED");
        }
        last_chassis_button_code_ = button_code;
        return;
    }

    if (start_pressed && start_rising)
    {
        _Enable_Vehicle_Control("START pressed: vehicle ENABLED");
    }

    last_chassis_button_code_ = button_code;
}

void Class_Robot::_Enable_Auxiliary_Motor()
{
    Auxiliary_Motor.CAN_Send_Enter();
}

void Class_Robot::_Disable_Auxiliary_Motor()
{
    _Cancel_Lift_Aux_Sequence();
    Auxiliary_Motor.Set_Control_Status(Motor_DM_Status_DISABLE);
    Auxiliary_Motor.Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Auxiliary_Motor.CAN_Send_Exit();
}

void Class_Robot::_Cancel_Lift_Aux_Sequence()
{
    lift_aux_sequence_state_ = LiftAuxSequenceState::IDLE;
    auxiliary_motor_command_enable_ = false;
}

void Class_Robot::_Start_Lift_Aux_Raise_Sequence()
{
    Lift.Set_Both_Lift_Raise_By_Motor_Angle(kManualBothLiftMotorRaiseAngle);
    auxiliary_motor_target_angle_ = kAuxiliaryMotorRaisedAngle;
    auxiliary_motor_command_enable_ = false;
    lift_aux_sequence_state_ = LiftAuxSequenceState::RAISE_WAIT_LIFT_REACHED;
}

void Class_Robot::_Start_Lift_Aux_Home_Sequence()
{
    Lift.Stop_Stair_Auto();
    auxiliary_motor_target_angle_ = kAuxiliaryMotorHomeAngle;
    auxiliary_motor_command_enable_ = true;
    lift_aux_sequence_state_ = LiftAuxSequenceState::HOME_MOVE_AUXILIARY;
}

void Class_Robot::_Update_Lift_Aux_Sequence()
{
    switch (lift_aux_sequence_state_)
    {
        case LiftAuxSequenceState::RAISE_WAIT_LIFT_REACHED:
            if (Lift.Are_Both_Lifts_Reached(CHARIOT_LIFT_POSITION_RAISE))
            {
                auxiliary_motor_target_angle_ = kAuxiliaryMotorRaisedAngle;
                auxiliary_motor_command_enable_ = true;
                lift_aux_sequence_state_ = LiftAuxSequenceState::RAISE_MOVE_AUXILIARY;
            }
            break;

        case LiftAuxSequenceState::RAISE_MOVE_AUXILIARY:
            if (_Is_Auxiliary_Motor_Reached(kAuxiliaryMotorRaisedAngle))
                lift_aux_sequence_state_ = LiftAuxSequenceState::RAISE_DONE;
            break;

        case LiftAuxSequenceState::HOME_WAIT_LIFT_REACHED:
            if (Lift.Are_Both_Lifts_Reached(CHARIOT_LIFT_POSITION_RETRACT))
                lift_aux_sequence_state_ = LiftAuxSequenceState::HOME_DONE;
            break;

        case LiftAuxSequenceState::HOME_MOVE_AUXILIARY:
            if (_Is_Auxiliary_Motor_Reached(kAuxiliaryMotorHomeAngle))
            {
                Lift.Set_Both_Lift_Retract_To(kManualBothLiftRetractAngle);
                lift_aux_sequence_state_ = LiftAuxSequenceState::HOME_WAIT_LIFT_REACHED;
            }
            break;

        case LiftAuxSequenceState::RAISE_DONE:
        case LiftAuxSequenceState::HOME_DONE:
        case LiftAuxSequenceState::IDLE:
        default:
            break;
    }

    _Output_Auxiliary_Motor();
}

void Class_Robot::_Output_Auxiliary_Motor()
{
    if (!auxiliary_motor_command_enable_)
        return;

    if (Auxiliary_Motor.Get_Status() != Motor_DM_Status_ENABLE)
    {
        Auxiliary_Motor.CAN_Send_Enter();
    }
    else
    {
        Auxiliary_Motor.Set_Control_Maintain_Postion(auxiliary_motor_target_angle_,
                                                     0.0f,
                                                     0.0f,
                                                     kAuxiliaryMotorKp,
                                                     kAuxiliaryMotorKd);
    }

    Auxiliary_Motor.TIM_Send_PeriodElapsedCallback();
}

bool Class_Robot::_Is_Auxiliary_Motor_Reached(float target_angle)
{
    return Auxiliary_Motor.Get_Status() == Motor_DM_Status_ENABLE &&
           std::fabs(Auxiliary_Motor.Get_Now_Radian() - target_angle) <=
               kAuxiliaryMotorReachedTolerance;
}

void Class_Robot::_Log_Chassis_Start_Gate(const char *msg)
{
    if (bridge_node_)
        RCLCPP_WARN(bridge_node_->get_logger(), "%s", msg);
    else
        std::cerr << "[ROBOT][WARN] " << msg << std::endl;
}

void Class_Robot::_Log_Chassis_Diagnostic(bool ros_cmd_recent,
                                          bool remote_cmd_recent,
                                          ChassisCommandSource source)
{
    const uint64_t unhandled = unhandled_can_frames_.load(std::memory_order_relaxed);
    const char *source_name = "none";
    switch (source)
    {
    case ChassisCommandSource::ROS2:   source_name = "ros2"; break;
    case ChassisCommandSource::REMOTE: source_name = "remote"; break;
    case ChassisCommandSource::NONE:   source_name = "none"; break;
    }

    if (bridge_node_)
    {
        RCLCPP_INFO(bridge_node_->get_logger(),
                    "chassis diag: source=%s remote=%d ros_cmd_recent=%d remote_cmd_recent=%d"
                    " target=(%.3f, %.3f, %.3f)"
                    " now=(%.3f, %.3f, %.3f) unhandled_can=%llu",
                    source_name,
                    chassis_remote_enabled_ ? 1 : 0,
                    ros_cmd_recent ? 1 : 0,
                    remote_cmd_recent ? 1 : 0,
                    Chassis.Get_Target_Velocity_X(),
                    Chassis.Get_Target_Velocity_Y(),
                    Chassis.Get_Target_Omega(),
                    Chassis.Get_Now_Velocity_X(),
                    Chassis.Get_Now_Velocity_Y(),
                    Chassis.Get_Now_Omega(),
                    static_cast<unsigned long long>(unhandled));
    }
    else
    {
        std::cout << "[ROBOT] chassis diag"
                  << " source=" << source_name
                  << " remote=" << (chassis_remote_enabled_ ? 1 : 0)
                  << " ros_cmd_recent=" << (ros_cmd_recent ? 1 : 0)
                  << " remote_cmd_recent=" << (remote_cmd_recent ? 1 : 0)
                  << " target=(" << Chassis.Get_Target_Velocity_X()
                  << ", " << Chassis.Get_Target_Velocity_Y()
                  << ", " << Chassis.Get_Target_Omega() << ")"
                  << " now=(" << Chassis.Get_Now_Velocity_X()
                  << ", " << Chassis.Get_Now_Velocity_Y()
                  << ", " << Chassis.Get_Now_Omega() << ")"
                  << " unhandled_can=" << unhandled
                  << std::endl;
    }
}

void Class_Robot::_Lift_Control()
{
    Ros_Command snapshot;
    {
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        snapshot = ros_cmd_;
    }

    const int64_t now = now_ns();
    const bool remote_cmd_recent =
        snapshot.remote_twist.has_twist && snapshot.remote_twist.last_twist_ns > 0 &&
        (now - snapshot.remote_twist.last_twist_ns) <= kCmdTimeoutNs;
    const bool buttons_recent_raw =
        snapshot.has_buttons && snapshot.last_button_ns > 0 &&
        (now - snapshot.last_button_ns) <= kButtonTimeoutNs;
    const bool buttons_recent =
        !suppress_remote_buttons_this_tick_ && buttons_recent_raw;

    if (!chassis_remote_enabled_)
    {
        _Cancel_Lift_Aux_Sequence();
        Lift.Stop_Stair_Auto();
        Lift.Set_Diff_Drive_Enable(false);
        Lift.Set_Target_Diff_Command(0.0f, 0.0f);
        Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_DISABLE);
        last_lift_button_code_ = buttons_recent_raw ? snapshot.button_code : LogF710_Key_IDLE;
        last_gripper_button_code_ = buttons_recent_raw ? snapshot.button_code : LogF710_Key_IDLE;
        return;
    }

    Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    if (suppress_remote_buttons_this_tick_)
    {
        last_lift_button_code_ = buttons_recent_raw ? snapshot.button_code : LogF710_Key_IDLE;
        last_gripper_button_code_ = buttons_recent_raw ? snapshot.button_code : LogF710_Key_IDLE;
    }
    else
    {
        _Gripper_Control(buttons_recent, snapshot.button_code);
    }

    auto log_lift_action = [this](const char *msg) {
        if (bridge_node_)
            RCLCPP_WARN(bridge_node_->get_logger(), "%s", msg);
        else
            std::cerr << "[ROBOT][WARN] " << msg << std::endl;
    };

    if (buttons_recent)
    {
        const bool last_lift_was_gripper_combo =
            (last_lift_button_code_ == LogF710_Key_LB_X) ||
            (last_lift_button_code_ == LogF710_Key_LB_Y);
        const bool a_rising =
            (snapshot.button_code == LogF710_Key_A) &&
            (last_lift_button_code_ != LogF710_Key_A);
        const bool b_rising =
            (snapshot.button_code == LogF710_Key_B) &&
            (last_lift_button_code_ != LogF710_Key_B);
        const bool x_rising =
            (snapshot.button_code == LogF710_Key_X) &&
            (last_lift_button_code_ != LogF710_Key_X) &&
            !last_lift_was_gripper_combo;
        const bool y_rising =
            (snapshot.button_code == LogF710_Key_Y) &&
            (last_lift_button_code_ != LogF710_Key_Y) &&
            !last_lift_was_gripper_combo;
        const bool up_rising =
            (snapshot.button_code == LogF710_Key_Up) &&
            (last_lift_button_code_ != LogF710_Key_Up);
        const bool down_rising =
            (snapshot.button_code == LogF710_Key_Down) &&
            (last_lift_button_code_ != LogF710_Key_Down);

        if (x_rising)
        {
            _Cancel_Lift_Aux_Sequence();
            Lift.Start_Stair_Up(-8.0f);
            log_lift_action("X pressed: stair UP auto start, raise_angle=-8.0");
        }
        else if (y_rising)
        {
            _Cancel_Lift_Aux_Sequence();
            Lift.Start_Stair_Down(-8.0f);
            log_lift_action("Y pressed: stair DOWN auto start, raise_angle=-8.0");
        }
        else if (a_rising)
        {
            _Cancel_Lift_Aux_Sequence();
            Lift.Start_Stair_Up(-14.3f);
            log_lift_action("A pressed: stair UP auto start, raise_angle=-14.3");
        }
        else if (b_rising)
        {
            _Cancel_Lift_Aux_Sequence();
            Lift.Start_Stair_Down(-14.3f);
            log_lift_action("B pressed: stair DOWN auto start, raise_angle=-14.3");
        }
        else if (up_rising)
        {
            _Start_Lift_Aux_Home_Sequence();
            log_lift_action("Up pressed: aux 0x07 target=0.1, then lift target=-0.01");
        }
        else if (down_rising)
        {
            _Start_Lift_Aux_Raise_Sequence();
            log_lift_action("Down pressed: lift motor target=-39.0, then aux 0x07 target=1.5");
        }

        last_lift_button_code_ = snapshot.button_code;
    }
    else if (!suppress_remote_buttons_this_tick_)
    {
        last_lift_button_code_ = LogF710_Key_IDLE;
        last_gripper_button_code_ = LogF710_Key_IDLE;
    }

    _Update_Lift_Aux_Sequence();

    if (Lift.Is_Stair_Auto_Active())
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_ENABLE);
        Chassis.Set_Target_Velocity_X(Lift.Get_Stair_Chassis_Forward());
        Chassis.Set_Target_Velocity_Y(0.0f);
        Chassis.Set_Target_Omega(0.0f);
        return;
    }

    if (Lift.Get_Diff_Drive_Enable() && chassis_remote_enabled_ && remote_cmd_recent)
    {
        Lift.Set_Target_Diff_Command(snapshot.remote_twist.right_y, snapshot.remote_twist.omega);
    }
    else
    {
        Lift.Set_Target_Diff_Command(0.0f, 0.0f);
    }
}

void Class_Robot::_Gripper_Control(bool buttons_recent, uint16_t button_code)
{
    if (!buttons_recent)
    {
        last_gripper_button_code_ = LogF710_Key_IDLE;
        return;
    }

    const bool grab_rising =
        (button_code == LogF710_Key_LB_X) &&
        (last_gripper_button_code_ != LogF710_Key_LB_X);
    const bool release_rising =
        (button_code == LogF710_Key_LB_Y) &&
        (last_gripper_button_code_ != LogF710_Key_LB_Y);

    if (grab_rising)
    {
        const bool ok = Gripper.Grab();
        _Log_Gripper_Action(ok ? "LB+X pressed: gripper GRAB sent (0x01)"
                               : "LB+X pressed: gripper GRAB send failed (0x01)",
                            ok);
    }
    else if (release_rising)
    {
        const bool ok = Gripper.Release();
        _Log_Gripper_Action(ok ? "LB+Y pressed: gripper RELEASE sent (0x02)"
                               : "LB+Y pressed: gripper RELEASE send failed (0x02)",
                            ok);
    }

    last_gripper_button_code_ = button_code;
}

void Class_Robot::_Log_Gripper_Action(const char *msg, bool ok)
{
    if (bridge_node_)
    {
        if (ok)
            RCLCPP_WARN(bridge_node_->get_logger(), "%s", msg);
        else
            RCLCPP_ERROR(bridge_node_->get_logger(), "%s", msg);
    }
    else
    {
        std::cerr << "[ROBOT][" << (ok ? "WARN" : "ERROR") << "] " << msg << std::endl;
    }
}

void Class_Robot::_Send_Odometry()
{
    if (!pub_odom_) return;
    // 1ms 调用一次，限制在 100Hz 发布
    if (++odom_pub_divider_ < 10) return;
    odom_pub_divider_ = 0;

    geometry_msgs::msg::Twist msg;
    msg.linear.x  = Chassis.Get_Now_Velocity_X();
    msg.linear.y  = Chassis.Get_Now_Velocity_Y();
    msg.angular.z = Chassis.Get_Now_Omega();
    pub_odom_->publish(msg);
}

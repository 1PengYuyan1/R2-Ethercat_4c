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
#include "chassis_trace_logger.h"

#include <cmath>
#include <cstddef>
#include <string>

namespace
{
constexpr int64_t kCmdFreshTimeoutNs = 200LL * 1000LL * 1000LL;  // 200 ms
constexpr int64_t kCmdDisableTimeoutNs = 2LL * 1000LL * 1000LL * 1000LL;  // 2 s
constexpr int64_t kButtonTimeoutNs = 500LL * 1000LL * 1000LL;  // 500 ms
constexpr int kRosSpinRateHz = 1000;
constexpr const char *kRosCmdTopic = "/chassis/cmd_vel";
constexpr const char *kRemoteCmdTopic = "/chassis/remote_cmd_vel";
constexpr float kManualBothLiftMotorRaiseAngle = -39.0f;
constexpr float kManualBothLiftRetractAngle = -0.01f;
constexpr float kAuxiliaryMotorRaisedAngle = 1.5f;
constexpr float kAuxiliaryMotorHomeAngle = -0.05f;
constexpr float kAuxiliaryMotorReachedTolerance = 0.05f;
constexpr float kAuxiliaryMotorHomeReachedTolerance = 0.12f;
constexpr float kAuxiliaryMotorControlDtS = 0.002f;

// ---- 运动段：梯形速度规划（accel → cruise → decel），抬升/回零各一套限幅 ----
constexpr float kAuxiliaryMotorProfileMaxSpeed = 5.8f;
constexpr float kAuxiliaryMotorHomeProfileMaxSpeed = 4.0f;
constexpr float kAuxiliaryMotorProfileMaxAccel = 22.0f;
constexpr float kAuxiliaryMotorHomeProfileMaxAccel = 18.0f;
constexpr float kAuxiliaryMotorProfileMaxDecel = 28.0f;
constexpr float kAuxiliaryMotorHomeProfileMaxDecel = 28.0f;
constexpr float kAuxiliaryMotorHomeProfileSnapDistance = 0.06f;

// ---- 运动段 MIT 增益：跟随规划轨迹，偏软以保证平滑、不抖 ----
constexpr float kAuxiliaryMotorMoveTorqueNm = 0.0f;
constexpr float kAuxiliaryMotorMoveKp = 12.0f;
constexpr float kAuxiliaryMotorMoveKd = 2.0f;
constexpr float kAuxiliaryMotorHomeMoveKp = 14.0f;
constexpr float kAuxiliaryMotorHomeMoveKd = 2.0f;

// ---- 保持段：刚性 + 近临界阻尼 PD ----
// kp 提供随偏差线性增大的恢复力矩（即“到位后的大扭矩”，最终被电机 7.8 N·m 上限钳位），
// kd 抑制振荡，deadband 只在编码器量化噪声范围内关掉恢复力矩以免极限环抖动。
// 抗冲击不再用“松手 + 纯阻尼(kp=0)”——那样没有平衡点，一碰就漂/震荡；
// 改为始终用这套刚性 PD 主动把电机顶回目标角，碰撞/震动被自然抑制。
// home(0.0) 处刚度从原来的 0 提升到与抬升位一致，解决“0.0 无保持力矩”的问题。
// 调参：若仍有振荡，先增大 kd（上限 5.0），再适当下调 kp；kp 越大“到位扭矩”越硬。
constexpr float kAuxiliaryMotorHoldTorqueNm = 0.0f;
constexpr float kAuxiliaryMotorHoldKp = 20.0f;
constexpr float kAuxiliaryMotorHoldKd = 2.0f;
constexpr float kAuxiliaryMotorHoldDeadband = 0.015f;
constexpr float kAuxiliaryMotorHomeHoldKp = 20.0f;
constexpr float kAuxiliaryMotorHomeHoldKd = 2.0f;
constexpr float kAuxiliaryMotorHomeHoldDeadband = 0.015f;

// ---- 进入/退出保持的判据 ----
constexpr float kAuxiliaryMotorHoldEnterTolerance = 0.04f;
constexpr float kAuxiliaryMotorHomeHoldEnterTolerance = 0.12f;
constexpr float kAuxiliaryMotorHoldEnterOmega = 0.25f;
constexpr uint32_t kAuxiliaryMotorHoldEnterStableTicks = 25U;
constexpr float kAuxiliaryMotorHoldExitTolerance = 0.20f;
constexpr float kAuxiliaryMotorHoldGainRampTimeS = 0.25f;
constexpr float kAuxiliaryMotorHomeHoldGainRampTimeS = 0.20f;
constexpr uint8_t kAuxiliaryMotorCanChannel = 0U;
constexpr uint8_t kAuxiliaryMotorRxId = 0x17U;
constexpr uint8_t kAuxiliaryMotorTxId = 0x07U;
constexpr size_t kHighPriorityActionQueueMax = 16U;

bool Auxiliary_Is_Home_Target(float target_angle)
{
    return std::fabs(target_angle - kAuxiliaryMotorHomeAngle) <= 1e-3f;
}
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
    _Integrate_Odometry();
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
    pub_odometry_ = bridge_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    odom_state_ = Odom_State{};
    _Register_Action_Services();

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
    pub_odometry_.reset();
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
    rclcpp::WallRate rate(kRosSpinRateHz);
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
    (void)name;
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    if (high_priority_actions_.size() >= kHighPriorityActionQueueMax)
    {
        return false;
    }

    high_priority_actions_.push_back(action);
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
            _Start_Stair_Down(-8.0f);
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
            _Start_Stair_Down(-14.3f);
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
            _Log_Chassis_Start_Gate("service /vehicle/lift_aux/home: aux 0x07 target=-0.05, then lift target=-0.01");
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

Class_Chariot_Imu_Heading_Hold::Snapshot Class_Robot::Get_Imu_Snapshot()
{
    return imu_heading_hold_.Get_Snapshot();
}

// --- CAN 路由 --------------------------------------------------------------
//
// 全向轮工况分发策略（新车头为原车尾）：
//   - ch0 → index 1/2 全向轮 + 前抬升 + 辅助电机，通过 Rx_ID 区分（0x11..0x17）
//   - ch1 → index 0/3 全向轮 + 后抬升，通过 Rx_ID 区分（0x11..0x15）
//   - ToF 由 task.cpp 的 slave2 CAN2/CAN3 接收循环直接送入 Lift
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
    const bool ros_cmd_known =
        snapshot.ros_twist.has_twist && snapshot.ros_twist.last_twist_ns > 0;
    const bool remote_cmd_known =
        snapshot.remote_twist.has_twist && snapshot.remote_twist.last_twist_ns > 0;
    const bool ros_cmd_recent =
        ros_cmd_known && (now - snapshot.ros_twist.last_twist_ns) <= kCmdFreshTimeoutNs;
    const bool remote_cmd_recent =
        remote_cmd_known && (now - snapshot.remote_twist.last_twist_ns) <= kCmdFreshTimeoutNs;
    const bool cmd_disable_watchdog_alive =
        (ros_cmd_known && (now - snapshot.ros_twist.last_twist_ns) <= kCmdDisableTimeoutNs) ||
        (remote_cmd_known && (now - snapshot.remote_twist.last_twist_ns) <= kCmdDisableTimeoutNs);
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
        /* 航向纠偏激活时启用“只降速纠偏”：不向已饱和的弱轮要提速，只压快轮。 */
        Chassis.Set_Yaw_Correction_Slow_Only(imu_heading_hold_.Was_Correcting());
        Chassis.Set_Target_Omega(lift_diff_mode ? 0.0f : omega_out);
        if ((now - last_chassis_diag_ns_) >= 1000LL * 1000LL * 1000LL)
        {
            last_chassis_diag_ns_ = now;
            _Log_Chassis_Diagnostic(ros_cmd_recent, remote_cmd_recent, selected_source);
        }
    }
    else if (chassis_remote_enabled_ && cmd_disable_watchdog_alive)
    {
        selected_source = ChassisCommandSource::HOLD_ZERO;
        const bool lift_diff_mode = Lift.Get_Diff_Drive_Enable();
        (void)imu_heading_hold_.Correct_Omega(0.0f, 0.0f, 0.0f, lift_diff_mode, now);

        Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_ENABLE);
        Chassis.Set_Target_Velocity_X(0.0f);
        Chassis.Set_Target_Velocity_Y(0.0f);
        Chassis.Set_Yaw_Correction_Slow_Only(false);
        Chassis.Set_Target_Omega(0.0f);
        if ((now - last_chassis_diag_ns_) >= 1000LL * 1000LL * 1000LL)
        {
            last_chassis_diag_ns_ = now;
            _Log_Chassis_Diagnostic(ros_cmd_recent,
                                    remote_cmd_recent,
                                    ChassisCommandSource::HOLD_ZERO);
        }
    }
    else
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);
        Chassis.Set_Target_Velocity_X(0.0f);
        Chassis.Set_Target_Velocity_Y(0.0f);
        Chassis.Set_Target_Omega(0.0f);
    }

    _Trace_Chassis_Command(snapshot,
                           ros_cmd_recent,
                           remote_cmd_recent,
                           cmd_disable_watchdog_alive,
                           selected_source,
                           now);
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

void Class_Robot::_Update_Lift_Attitude_Yaw()
{
    const auto imu = imu_heading_hold_.Get_Snapshot();
    Lift.Set_Stair_Attitude_Yaw(imu.yaw_rad, imu.valid && imu.fresh);
}

void Class_Robot::_Start_Stair_Down(float raise_angle)
{
    _Update_Lift_Attitude_Yaw();
    Lift.Start_Stair_Down(raise_angle);
}

void Class_Robot::_Enable_Auxiliary_Motor()
{
    lift_aux_sequence_state_ = LiftAuxSequenceState::IDLE;
    auxiliary_motor_target_angle_ = kAuxiliaryMotorHomeAngle;
    auxiliary_motor_command_enable_ = true;
    auxiliary_motor_profile_initialized_ = false;
    auxiliary_motor_hold_active_ = false;
    auxiliary_motor_hold_blend_ = 0.0f;
    auxiliary_motor_hold_ready_ticks_ = 0U;
    Auxiliary_Motor.CAN_Send_Enter();
}

void Class_Robot::_Disable_Auxiliary_Motor()
{
    _Cancel_Lift_Aux_Sequence();
    auxiliary_motor_profile_initialized_ = false;
    auxiliary_motor_profile_active_ = false;
    auxiliary_motor_hold_active_ = false;
    auxiliary_motor_hold_blend_ = 0.0f;
    auxiliary_motor_hold_ready_ticks_ = 0U;
    Auxiliary_Motor.Set_Control_Status(Motor_DM_Status_DISABLE);
    Auxiliary_Motor.Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Auxiliary_Motor.CAN_Send_Exit();
}

void Class_Robot::_Cancel_Lift_Aux_Sequence()
{
    lift_aux_sequence_state_ = LiftAuxSequenceState::IDLE;
    if (chassis_remote_enabled_)
    {
        auxiliary_motor_target_angle_ = kAuxiliaryMotorHomeAngle;
        auxiliary_motor_command_enable_ = true;
    }
    else
    {
        auxiliary_motor_command_enable_ = false;
    }
}

void Class_Robot::_Start_Lift_Aux_Raise_Sequence()
{
    Lift.Set_Both_Lift_Raise_By_Motor_Angle(kManualBothLiftMotorRaiseAngle);
    auxiliary_motor_target_angle_ = kAuxiliaryMotorHomeAngle;
    auxiliary_motor_command_enable_ = true;
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

void Class_Robot::_Reset_Auxiliary_Motor_Profile(float angle)
{
    auxiliary_motor_smooth_angle_ = angle;
    auxiliary_motor_profile_start_angle_ = angle;
    auxiliary_motor_profile_target_angle_ = angle;
    auxiliary_motor_profile_elapsed_ = 0.0f;
    auxiliary_motor_profile_duration_ = 0.0f;
    auxiliary_motor_profile_accel_time_ = 0.0f;
    auxiliary_motor_profile_decel_time_ = 0.0f;
    auxiliary_motor_profile_cruise_time_ = 0.0f;
    auxiliary_motor_profile_peak_speed_ = 0.0f;
    auxiliary_motor_profile_distance_ = 0.0f;
    auxiliary_motor_profile_direction_ = 1.0f;
    auxiliary_motor_target_omega_ = 0.0f;
    auxiliary_motor_profile_active_ = false;
    auxiliary_motor_profile_initialized_ = true;
    auxiliary_motor_hold_active_ = false;
    auxiliary_motor_hold_blend_ = 0.0f;
    auxiliary_motor_hold_ready_ticks_ = 0U;
}

void Class_Robot::_Start_Auxiliary_Motor_Profile(float target_angle)
{
    const float start_angle = auxiliary_motor_smooth_angle_;
    const float distance = target_angle - start_angle;
    const float abs_distance = std::fabs(distance);
    const bool home_target = Auxiliary_Is_Home_Target(target_angle);
    const float max_speed = home_target ?
        kAuxiliaryMotorHomeProfileMaxSpeed :
        kAuxiliaryMotorProfileMaxSpeed;
    const float safe_peak_speed =
        (max_speed > 0.05f) ? max_speed : 0.05f;
    const float max_accel = home_target ?
        kAuxiliaryMotorHomeProfileMaxAccel :
        kAuxiliaryMotorProfileMaxAccel;
    const float max_decel = home_target ?
        kAuxiliaryMotorHomeProfileMaxDecel :
        kAuxiliaryMotorProfileMaxDecel;
    const float safe_peak_accel =
        (max_accel > 0.5f) ? max_accel : 0.5f;
    const float safe_peak_decel =
        (max_decel > 0.5f) ? max_decel : 0.5f;

    auxiliary_motor_profile_start_angle_ = start_angle;
    auxiliary_motor_profile_target_angle_ = target_angle;
    auxiliary_motor_profile_elapsed_ = 0.0f;
    auxiliary_motor_profile_distance_ = abs_distance;
    auxiliary_motor_profile_direction_ = (distance >= 0.0f) ? 1.0f : -1.0f;

    const float accel_time_to_max = safe_peak_speed / safe_peak_accel;
    const float decel_time_to_max = safe_peak_speed / safe_peak_decel;
    const float accel_decel_distance_to_max =
        0.5f * safe_peak_speed * safe_peak_speed *
        ((1.0f / safe_peak_accel) + (1.0f / safe_peak_decel));
    if (abs_distance <= accel_decel_distance_to_max)
    {
        auxiliary_motor_profile_peak_speed_ =
            std::sqrt((2.0f * abs_distance * safe_peak_accel * safe_peak_decel) /
                      (safe_peak_accel + safe_peak_decel));
        auxiliary_motor_profile_accel_time_ =
            auxiliary_motor_profile_peak_speed_ / safe_peak_accel;
        auxiliary_motor_profile_decel_time_ =
            auxiliary_motor_profile_peak_speed_ / safe_peak_decel;
        auxiliary_motor_profile_cruise_time_ = 0.0f;
    }
    else
    {
        auxiliary_motor_profile_peak_speed_ = safe_peak_speed;
        auxiliary_motor_profile_accel_time_ = accel_time_to_max;
        auxiliary_motor_profile_decel_time_ = decel_time_to_max;
        auxiliary_motor_profile_cruise_time_ =
            (abs_distance - accel_decel_distance_to_max) / safe_peak_speed;
    }

    auxiliary_motor_profile_duration_ =
        auxiliary_motor_profile_accel_time_ +
        auxiliary_motor_profile_decel_time_ +
        auxiliary_motor_profile_cruise_time_;
    auxiliary_motor_target_omega_ = 0.0f;
    auxiliary_motor_profile_active_ = auxiliary_motor_profile_duration_ > 1e-4f;
    auxiliary_motor_hold_active_ = false;
    auxiliary_motor_hold_blend_ = 0.0f;
    auxiliary_motor_hold_ready_ticks_ = 0U;

    if (!auxiliary_motor_profile_active_)
        _Reset_Auxiliary_Motor_Profile(target_angle);
}

float Class_Robot::_Update_Auxiliary_Motor_Profile(float target_angle)
{
    if (!auxiliary_motor_profile_initialized_)
        _Reset_Auxiliary_Motor_Profile(Auxiliary_Motor.Get_Now_Radian());

    if (std::fabs(target_angle - auxiliary_motor_profile_target_angle_) > 1e-4f)
        _Start_Auxiliary_Motor_Profile(target_angle);

    if (!auxiliary_motor_profile_active_)
    {
        auxiliary_motor_smooth_angle_ = target_angle;
        auxiliary_motor_target_omega_ = 0.0f;
        return auxiliary_motor_target_omega_;
    }

    auxiliary_motor_profile_elapsed_ += kAuxiliaryMotorControlDtS;

    if (auxiliary_motor_profile_elapsed_ >= auxiliary_motor_profile_duration_)
    {
        auxiliary_motor_smooth_angle_ = auxiliary_motor_profile_target_angle_;
        auxiliary_motor_target_omega_ = 0.0f;
        auxiliary_motor_profile_active_ = false;
        return auxiliary_motor_target_omega_;
    }

    const float accel_time = auxiliary_motor_profile_accel_time_;
    const float decel_time = auxiliary_motor_profile_decel_time_;
    const float cruise_time = auxiliary_motor_profile_cruise_time_;
    const float peak_speed = auxiliary_motor_profile_peak_speed_;
    const float accel = (accel_time > 1e-5f) ? (peak_speed / accel_time) : 0.0f;
    const float decel = (decel_time > 1e-5f) ? (peak_speed / decel_time) : 0.0f;
    const float accel_distance = 0.5f * accel * accel_time * accel_time;
    const float cruise_distance = peak_speed * cruise_time;
    float profile_distance = 0.0f;
    float profile_speed = 0.0f;

    if (auxiliary_motor_profile_elapsed_ < accel_time)
    {
        const float t = auxiliary_motor_profile_elapsed_;
        profile_distance = 0.5f * accel * t * t;
        profile_speed = accel * t;
    }
    else if (auxiliary_motor_profile_elapsed_ < accel_time + cruise_time)
    {
        const float t = auxiliary_motor_profile_elapsed_ - accel_time;
        profile_distance = accel_distance + peak_speed * t;
        profile_speed = peak_speed;
    }
    else
    {
        const float t = auxiliary_motor_profile_elapsed_ - accel_time - cruise_time;
        profile_distance =
            accel_distance +
            cruise_distance +
            peak_speed * t -
            0.5f * decel * t * t;
        profile_speed = peak_speed - decel * t;
        if (profile_speed < 0.0f)
            profile_speed = 0.0f;
    }

    if (profile_distance > auxiliary_motor_profile_distance_)
        profile_distance = auxiliary_motor_profile_distance_;

    const float remaining_distance =
        auxiliary_motor_profile_distance_ - profile_distance;
    if (Auxiliary_Is_Home_Target(auxiliary_motor_profile_target_angle_) &&
        remaining_distance <= kAuxiliaryMotorHomeProfileSnapDistance)
    {
        auxiliary_motor_smooth_angle_ = auxiliary_motor_profile_target_angle_;
        auxiliary_motor_target_omega_ = 0.0f;
        auxiliary_motor_profile_active_ = false;
        return auxiliary_motor_target_omega_;
    }

    auxiliary_motor_smooth_angle_ =
        auxiliary_motor_profile_start_angle_ +
        auxiliary_motor_profile_direction_ * profile_distance;
    auxiliary_motor_target_omega_ =
        auxiliary_motor_profile_direction_ * profile_speed;
    return auxiliary_motor_target_omega_;
}

bool Class_Robot::_Is_Auxiliary_Motor_Profile_At_Target() const
{
    return auxiliary_motor_profile_initialized_ &&
           !auxiliary_motor_profile_active_ &&
           std::fabs(auxiliary_motor_smooth_angle_ - auxiliary_motor_target_angle_) <= 1e-4f;
}

void Class_Robot::_Output_Auxiliary_Motor()
{
    if (!auxiliary_motor_command_enable_)
        return;

    if (Auxiliary_Motor.Get_Status() != Motor_DM_Status_ENABLE)
    {
        auxiliary_motor_profile_initialized_ = false;
        auxiliary_motor_hold_active_ = false;
        auxiliary_motor_hold_blend_ = 0.0f;
        auxiliary_motor_hold_ready_ticks_ = 0U;
        Auxiliary_Motor.CAN_Send_Enter();
        Auxiliary_Motor.TIM_Send_PeriodElapsedCallback();
        return;
    }

    // 1) 运动段：梯形速度规划生成平滑目标角 + 前馈角速度
    const float target_omega =
        _Update_Auxiliary_Motor_Profile(auxiliary_motor_target_angle_);
    const bool home_target = Auxiliary_Is_Home_Target(auxiliary_motor_target_angle_);
    const float now_angle = Auxiliary_Motor.Get_Now_Radian();
    const float feedback_error = std::fabs(now_angle - auxiliary_motor_target_angle_);
    const float feedback_omega = std::fabs(Auxiliary_Motor.Get_Now_Omega());

    // 2) 保持段状态机：规划到位且稳定 → 进入保持；被外力推出退出阈值 → 退回规划平滑滑回。
    //    不再有 impact / home / settling 等“松手纯阻尼”分支——抗冲击交给保持段的刚性 PD。
    const float hold_enter_tolerance = home_target ?
        kAuxiliaryMotorHomeHoldEnterTolerance :
        kAuxiliaryMotorHoldEnterTolerance;

    if (auxiliary_motor_hold_active_ &&
        feedback_error > kAuxiliaryMotorHoldExitTolerance)
    {
        auxiliary_motor_hold_active_ = false;
        auxiliary_motor_hold_blend_ = 0.0f;
        auxiliary_motor_hold_ready_ticks_ = 0U;
    }

    if (!auxiliary_motor_hold_active_)
    {
        if (_Is_Auxiliary_Motor_Profile_At_Target() &&
            feedback_error <= hold_enter_tolerance &&
            feedback_omega <= kAuxiliaryMotorHoldEnterOmega)
        {
            if (auxiliary_motor_hold_ready_ticks_ < kAuxiliaryMotorHoldEnterStableTicks)
                ++auxiliary_motor_hold_ready_ticks_;
        }
        else
        {
            auxiliary_motor_hold_ready_ticks_ = 0U;
        }

        if (auxiliary_motor_hold_ready_ticks_ >= kAuxiliaryMotorHoldEnterStableTicks)
        {
            auxiliary_motor_hold_active_ = true;
            auxiliary_motor_hold_blend_ = 0.0f;
        }
    }

    // 3) 增益选择
    const float move_kp = home_target ? kAuxiliaryMotorHomeMoveKp : kAuxiliaryMotorMoveKp;
    const float move_kd = home_target ? kAuxiliaryMotorHomeMoveKd : kAuxiliaryMotorMoveKd;

    float control_angle;
    float control_omega;
    float control_torque;
    float control_kp;
    float control_kd;

    if (auxiliary_motor_hold_active_)
    {
        // move → hold 增益渐变，避免切入瞬间的力矩台阶
        const float hold_kp = home_target ? kAuxiliaryMotorHomeHoldKp : kAuxiliaryMotorHoldKp;
        const float hold_kd = home_target ? kAuxiliaryMotorHomeHoldKd : kAuxiliaryMotorHoldKd;
        const float hold_deadband = home_target ?
            kAuxiliaryMotorHomeHoldDeadband : kAuxiliaryMotorHoldDeadband;
        const float hold_gain_ramp_time = home_target ?
            kAuxiliaryMotorHomeHoldGainRampTimeS : kAuxiliaryMotorHoldGainRampTimeS;
        const float blend_step = (hold_gain_ramp_time > 1e-4f) ?
            (kAuxiliaryMotorControlDtS / hold_gain_ramp_time) : 1.0f;
        auxiliary_motor_hold_blend_ =
            std::fmin(1.0f, auxiliary_motor_hold_blend_ + blend_step);

        const float blend = auxiliary_motor_hold_blend_;
        const float target_error = auxiliary_motor_target_angle_ - now_angle;

        // deadband 内：锁定当前角、不施加恢复力矩，避免编码器量化噪声引起的极限环抖动。
        // deadband 外：锁定到固定目标角，由 kp 生成正比于偏差的恢复力矩（到位/0.0 处的“大扭矩”），
        //              震动/碰撞会被这套刚性 PD 主动顶回，而不是漂走。
        control_angle = (std::fabs(target_error) <= hold_deadband) ?
            now_angle : auxiliary_motor_target_angle_;
        control_omega = 0.0f;
        control_torque = kAuxiliaryMotorHoldTorqueNm;
        control_kp = move_kp + blend * (hold_kp - move_kp);
        control_kd = move_kd + blend * (hold_kd - move_kd);
    }
    else
    {
        // 运动段：跟随规划轨迹（偏软增益 + 角速度前馈），又快又平滑地逼近目标
        auxiliary_motor_hold_blend_ = 0.0f;
        control_angle = auxiliary_motor_smooth_angle_;
        control_omega = target_omega;
        control_torque = kAuxiliaryMotorMoveTorqueNm;
        control_kp = move_kp;
        control_kd = move_kd;
    }

    Auxiliary_Motor.Set_Control_Maintain_Postion(
        control_angle,
        control_omega,
        control_torque,
        control_kp,
        control_kd);
    Auxiliary_Motor.TIM_Send_PeriodElapsedCallback();
}

bool Class_Robot::_Is_Auxiliary_Motor_Reached(float target_angle)
{
    const float reached_tolerance = Auxiliary_Is_Home_Target(target_angle) ?
        kAuxiliaryMotorHomeReachedTolerance :
        kAuxiliaryMotorReachedTolerance;
    return Auxiliary_Motor.Get_Status() == Motor_DM_Status_ENABLE &&
           _Is_Auxiliary_Motor_Profile_At_Target() &&
           auxiliary_motor_hold_active_ &&
           std::fabs(Auxiliary_Motor.Get_Now_Radian() - target_angle) <=
               reached_tolerance;
}

void Class_Robot::_Log_Chassis_Start_Gate(const char *msg)
{
    (void)msg;
}

void Class_Robot::_Trace_Chassis_Command(const Ros_Command &snapshot,
                                         bool ros_cmd_recent,
                                         bool remote_cmd_recent,
                                         bool cmd_disable_watchdog_alive,
                                         ChassisCommandSource source,
                                         int64_t now)
{
    chassis_trace::Sample sample;
    sample.now_ns = now;
    sample.source = static_cast<int>(source);
    sample.chassis_enabled =
        (Chassis.Get_Chassis_Control_Type() == Chassis_Omni_Control_Type_ENABLE) ? 1 : 0;

    const bool ros_known =
        snapshot.ros_twist.has_twist && snapshot.ros_twist.last_twist_ns > 0;
    const bool remote_known =
        snapshot.remote_twist.has_twist && snapshot.remote_twist.last_twist_ns > 0;

    sample.ros_known = ros_known ? 1 : 0;
    sample.remote_known = remote_known ? 1 : 0;
    sample.ros_recent = ros_cmd_recent ? 1 : 0;
    sample.remote_recent = remote_cmd_recent ? 1 : 0;
    sample.disable_watchdog_alive = cmd_disable_watchdog_alive ? 1 : 0;
    sample.ros_age_ms =
        ros_known ? (now - snapshot.ros_twist.last_twist_ns) / (1000LL * 1000LL) : -1;
    sample.remote_age_ms =
        remote_known ? (now - snapshot.remote_twist.last_twist_ns) / (1000LL * 1000LL) : -1;

    sample.ros_vx = snapshot.ros_twist.vx;
    sample.ros_vy = snapshot.ros_twist.vy;
    sample.ros_omega = snapshot.ros_twist.omega;
    sample.remote_vx = snapshot.remote_twist.vx;
    sample.remote_vy = snapshot.remote_twist.vy;
    sample.remote_omega = snapshot.remote_twist.omega;

    sample.target_vx = Chassis.Get_Target_Velocity_X();
    sample.target_vy = Chassis.Get_Target_Velocity_Y();
    sample.target_omega = Chassis.Get_Target_Omega();
    sample.profiled_vx = Chassis.Get_Profiled_Target_Velocity_X();
    sample.profiled_vy = Chassis.Get_Profiled_Target_Velocity_Y();
    sample.profiled_omega = Chassis.Get_Profiled_Target_Omega();
    sample.now_vx = Chassis.Get_Now_Velocity_X();
    sample.now_vy = Chassis.Get_Now_Velocity_Y();
    sample.now_omega = Chassis.Get_Now_Omega();

    for (int i = 0; i < chassis_trace::kWheelCount; ++i)
    {
        sample.wheels[i].raw_target_omega = Chassis.Get_Raw_Target_Wheel_Omega(i);
        sample.wheels[i].target_omega = Chassis.Get_Target_Wheel_Omega(i);
        sample.wheels[i].control_omega = Chassis.Motor_Wheel[i].Get_Control_Omega();
        sample.wheels[i].now_omega = Chassis.Motor_Wheel[i].Get_Now_Omega();
        sample.wheels[i].now_torque = Chassis.Motor_Wheel[i].Get_Now_Torque();
        sample.wheels[i].motor_status = static_cast<int>(Chassis.Motor_Wheel[i].Get_Status());
        sample.wheels[i].control_status =
            static_cast<int>(Chassis.Motor_Wheel[i].Get_Now_Control_Status());
    }

    chassis_trace::Record(sample);
}

void Class_Robot::_Log_Chassis_Diagnostic(bool ros_cmd_recent,
                                          bool remote_cmd_recent,
                                          ChassisCommandSource source)
{
    (void)ros_cmd_recent;
    (void)remote_cmd_recent;
    (void)source;
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
        (now - snapshot.remote_twist.last_twist_ns) <= kCmdFreshTimeoutNs;
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
    _Update_Lift_Attitude_Yaw();
    if (suppress_remote_buttons_this_tick_)
    {
        last_lift_button_code_ = buttons_recent_raw ? snapshot.button_code : LogF710_Key_IDLE;
        last_gripper_button_code_ = buttons_recent_raw ? snapshot.button_code : LogF710_Key_IDLE;
    }
    else
    {
        _Gripper_Control(buttons_recent, snapshot.button_code);
    }

    auto log_lift_action = [](const char *msg) {
        (void)msg;
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
            _Start_Stair_Down(-8.0f);
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
            _Start_Stair_Down(-14.3f);
            log_lift_action("B pressed: stair DOWN auto start, raise_angle=-14.3");
        }
        else if (up_rising)
        {
            _Start_Lift_Aux_Home_Sequence();
            log_lift_action("Up pressed: aux 0x07 target=-0.05, then lift target=-0.01");
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
        Chassis.Set_Target_Omega(Lift.Get_Stair_Chassis_Omega());
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
    (void)msg;
    (void)ok;
}

void Class_Robot::_Integrate_Odometry()
{
    if (!pub_odometry_) return;

    const int64_t now = now_ns();

    // 首帧只记录时间基准，下一帧才开始积分（避免用一个超大 dt 积分出跳变）
    if (!odom_state_.initialized)
    {
        odom_state_.last_ns = now;
        odom_state_.initialized = true;
        return;
    }

    const double dt = static_cast<double>(now - odom_state_.last_ns) * 1e-9;
    odom_state_.last_ns = now;
    if (dt <= 0.0 || dt > 0.1) return;  // 调度抖动/异常 dt 跳过，不污染积分

    // 数据源：机体瞬时速度（轮速正解）+ IMU 角速度。均为瞬时值，无累积状态。
    const float vx = Chassis.Get_Now_Velocity_X();
    const float vy = Chassis.Get_Now_Velocity_Y();
    const float wz_wheel = Chassis.Get_Now_Omega();

    // IMU 失效（超时/未连接）时降级为纯轮速，避免 IMU 断线导致 yaw 卡死或读到刷新值
    const auto imu = imu_heading_hold_.Get_Snapshot();
    const float wz_imu = imu.valid ? imu.angular_velocity_z : wz_wheel;

    // --- yaw：互补滤波（轮速里程基准 + IMU 短期预测 + 缓慢拉回纠正 IMU 漂移）---
    odom_state_.wheel_yaw += static_cast<double>(wz_wheel) * dt;

    const float speed = std::sqrt(vx * vx + vy * vy);
    const float alpha = (speed >= Odom_Speed_Threshold_MPS) ?
        Odom_Yaw_Alpha_Fast : Odom_Yaw_Alpha_Slow;

    odom_state_.yaw_fused += static_cast<double>(wz_imu) * dt;              // ① IMU 角速度积分
    odom_state_.yaw_fused +=
        static_cast<double>(alpha) * (odom_state_.wheel_yaw - odom_state_.yaw_fused);  // ② 拉回轮速基准

    while (odom_state_.yaw_fused >  M_PI) odom_state_.yaw_fused -= 2.0 * M_PI;
    while (odom_state_.yaw_fused < -M_PI) odom_state_.yaw_fused += 2.0 * M_PI;

    // --- x/y：纯轮速积分（机体坐标系 → 世界坐标系），无绝对参考，长期会漂（设计接受）---
    const double cos_yaw = std::cos(odom_state_.yaw_fused);
    const double sin_yaw = std::sin(odom_state_.yaw_fused);
    odom_state_.x += (static_cast<double>(vx) * cos_yaw - static_cast<double>(vy) * sin_yaw) * dt;
    odom_state_.y += (static_cast<double>(vx) * sin_yaw + static_cast<double>(vy) * cos_yaw) * dt;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = bridge_node_->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = odom_state_.x;
    msg.pose.pose.position.y = odom_state_.y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(odom_state_.yaw_fused * 0.5);
    msg.pose.pose.orientation.w = std::cos(odom_state_.yaw_fused * 0.5);
    msg.twist.twist.linear.x = vx;   // 瞬时值，直接转报，非积分用的历史值
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.angular.z = wz_imu;  // 瞬时角速度，无漂移问题，无需融合
    pub_odometry_->publish(msg);
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

// 上位机版 Class_Robot 实现：把 ROS /chassis/cmd_vel(高优先级)
// + /chassis/remote_cmd_vel(遥控低优先级) + /chassis/buttons
// + /imu 转化为底盘速度命令；通过 linkx_t 推送给 EtherCAT-CAN 桥；
// 把 chassis 实际速度回报到 /chassis/odom_twist。
//
// 工况：全向轮底盘（4 × DM3519 MIT，分布在 LinkX channel 0/1）
//      + 前/后抬升机构（新车头为原车尾：front=ch0，rear=ch1）
//
// 调度:
//   robot.Init(linkx) 由 task::Robot_Control_Loop 在 SOEM 主站启动后调用一次
//   robot.CAN_Rx_Callback(ch, id, data) 由主循环 1ms 内 drain 每帧调用
//   robot.Loop() / TIM_1ms_Calculate_Callback() 每 1ms 调一次
//   robot.TIM_2ms_Calculate_PeriodElapsedCallback() 每 2ms 调一次
//   robot.TIM_100ms_Alive_PeriodElapsedCallback() 每 100ms 调一次

#include "robot.h"

#include <cmath>
#include <iostream>

namespace
{
constexpr int64_t kCmdTimeoutNs = 200LL * 1000LL * 1000LL;  // 200 ms
constexpr int64_t kButtonTimeoutNs = 500LL * 1000LL * 1000LL;  // 500 ms
constexpr const char *kRosCmdTopic = "/chassis/cmd_vel";
constexpr const char *kRemoteCmdTopic = "/chassis/remote_cmd_vel";

constexpr float kOpsYawHoldMinLinearSpeed = 0.08f;      // m/s
constexpr float kOpsYawHoldManualOmegaDeadband = 0.05f; // rad/s
constexpr float kDegToRad = 0.017453292519943295769f;
constexpr float kOpsYawHoldKp = 1.20f * kDegToRad;      // rad/s per deg
constexpr float kOpsYawHoldKd = 0.08f * kDegToRad;      // rad/s per deg/s
constexpr float kOpsYawHoldMaxCorrection = 0.35f;       // rad/s

constexpr float kOpsLateralHoldMinLinearSpeed = 0.08f;       // m/s
constexpr float kOpsLateralHoldManualOmegaDeadband = 0.05f;  // rad/s
constexpr float kOpsLateralLowSpeedThreshold = 0.14f;        // m/s
constexpr float kOpsLateralKpLowSpeed = 1.50f;               // m/s per m error
constexpr float kOpsLateralKpHighSpeed = 1.00f;              // m/s per m error
constexpr float kOpsLateralSpeedLimitLow = 0.10f;            // m/s
constexpr float kOpsLateralSpeedLimitHigh = 0.12f;           // m/s
constexpr float kOpsLateralDirectionResetDot = 0.94f;        // about 20 deg
constexpr float kOpsYawToBodyFrameOffsetDeg = 90.0f;         // OPS yaw to new chassis body yaw

struct Vec2
{
    float x = 0.0f;
    float y = 0.0f;
};

float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

float normalize_angle_deg(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float dot(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

Vec2 normalize_or_forward(Vec2 v)
{
    const float n = std::sqrt(dot(v, v));
    if (n < 1e-4f)
    {
        return {1.0f, 0.0f};
    }
    return {v.x / n, v.y / n};
}

Vec2 body_to_world(Vec2 body, float yaw_rad)
{
    const float c = std::cos(yaw_rad);
    const float s = std::sin(yaw_rad);
    return {
        body.x * c - body.y * s,
        body.x * s + body.y * c,
    };
}

Vec2 world_to_body(Vec2 world, float yaw_rad)
{
    const float c = std::cos(yaw_rad);
    const float s = std::sin(yaw_rad);
    return {
        world.x * c + world.y * s,
       -world.x * s + world.y * c,
    };
}

}

void Class_Robot::Init(linkx_t *__LinkX_Handler)
{
    LinkX_Handler = __LinkX_Handler;

    Chassis.Init(LinkX_Handler);
    Chassis.Init_Motor_Params();

    Lift.Init(LinkX_Handler);

    Navigation.Init();

    _Verify_Motor_ID_Uniqueness();
}

void Class_Robot::_Verify_Motor_ID_Uniqueness()
{
    // R2 CAN 通道分布（新车头为原车尾）：
    //   ch0：index 1/2 全向轮（Tx 0x02/0x01）+ 前抬升（Tx 0x03/0x04/0x05）
    //   ch1：index 0/3 全向轮（Tx 0x02/0x01）+ 后抬升（Tx 0x03/0x04/0x05）
    //   ch2：OPS（非 DM）
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

    uint16_t ch0_ids[5] = {
        Chassis.Motor_Wheel[1].DM_CAN_Rx_ID,
        Chassis.Motor_Wheel[2].DM_CAN_Rx_ID,
        Lift.Motor_Drive_Left[CHARIOT_LIFT_MODULE_FRONT].DM_CAN_Rx_ID,
        Lift.Motor_Drive_Right[CHARIOT_LIFT_MODULE_FRONT].DM_CAN_Rx_ID,
        Lift.Motor_Lift[CHARIOT_LIFT_MODULE_FRONT].DM_CAN_Rx_ID,
    };
    check_pair("ch0 (omni+front lift)", ch0_ids, 5);

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
}

void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
    Lift.TIM_2ms_Control_PeriodElapsedCallback();
}

void Class_Robot::TIM_1ms_Calculate_Callback()
{
    _Chassis_Control();
    _Lift_Control();
    _Send_Odometry();
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

    sub_imu_ = bridge_node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 20,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            latest_imu_omega_z_.store(static_cast<float>(msg->angular_velocity.z));
        });

    pub_odom_ = bridge_node_->create_publisher<geometry_msgs::msg::Twist>("/chassis/odom_twist", 50);

    ops.init(bridge_node_.get());

    RCLCPP_INFO(bridge_node_->get_logger(),
                "chassis command priority: ROS2 %s > remote %s",
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

    ops.shutdown();
    pub_odom_.reset();
    sub_imu_.reset();
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

// --- CAN 路由 --------------------------------------------------------------
//
// 全向轮工况分发策略（新车头为原车尾）：
//   - ch0 → index 1/2 全向轮 + 前抬升，通过 Rx_ID 区分（0x11..0x15）
//   - ch1 → index 0/3 全向轮 + 后抬升，通过 Rx_ID 区分（0x11..0x15）
//   - ch2 → OPS(0x01)
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
                handled = Lift.CAN_Rx_Callback(CAN_Channel, CAN_ID, CAN_Data);
            break;
        }
        case 1: {
            const int indices[] = {0, 3};
            handled = dispatch_wheel(indices, 2);
            if (!handled)
                handled = Lift.CAN_Rx_Callback(CAN_Channel, CAN_ID, CAN_Data);
            break;
        }
        case 2:
            if (can_id_std == 0x01U)
            {
                ops.CAN_RxCpltCallback(CAN_Data, CAN_Dlen);
                handled = true;
            }
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
    const bool buttons_recent = snapshot.has_buttons && snapshot.last_button_ns > 0 &&
                                (now - snapshot.last_button_ns) <= kButtonTimeoutNs;
    _Update_Chassis_Remote_Gate(buttons_recent, snapshot.button_code);

    const Velocity_Command *selected_cmd = nullptr;
    ChassisCommandSource selected_source = ChassisCommandSource::NONE;
    if (ros_cmd_recent)
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

        _Apply_OPS_Lateral_Correction(vx_cmd, vy_cmd, omega_cmd);

        float corrected_omega = 0.0f;
        if (lift_diff_mode)
        {
            ops_yaw_hold_active_ = false;
            ops_yaw_hold_last_output_ = 0.0f;
            ops_yaw_hold_last_error_ = 0.0f;
        }
        else
        {
            corrected_omega = _Apply_OPS_Yaw_Hold(vx_cmd, vy_cmd, omega_cmd);
        }

        Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_ENABLE);
        Chassis.Set_Target_Velocity_X(vx_cmd);
        Chassis.Set_Target_Velocity_Y(vy_cmd);
        Chassis.Set_Target_Omega(corrected_omega);
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
        ops_yaw_hold_active_ = false;
        ops_yaw_hold_last_output_ = 0.0f;
        ops_yaw_hold_last_error_ = 0.0f;
        _Reset_OPS_Lateral_Hold();
    }
}

void Class_Robot::_Apply_OPS_Lateral_Correction(float &vx, float &vy, float omega_cmd)
{
    const float linear_speed = std::sqrt(vx * vx + vy * vy);
    const bool manual_rotate = std::fabs(omega_cmd) > kOpsLateralHoldManualOmegaDeadband;
    const bool should_hold_line = linear_speed > kOpsLateralHoldMinLinearSpeed && !manual_rotate;

    if (!should_hold_line || !ops.isConnected())
    {
        _Reset_OPS_Lateral_Hold();
        return;
    }

    const Struct_OPS_Rx_Data ops_data = ops.getData();
    if (!std::isfinite(ops_data.Yaw) ||
        !std::isfinite(ops_data.Pos_X) ||
        !std::isfinite(ops_data.Pos_Y))
    {
        _Reset_OPS_Lateral_Hold();
        return;
    }

    const Vec2 body_dir = normalize_or_forward({vx, vy});
    if (ops_lateral_hold_active_)
    {
        const Vec2 locked_dir = normalize_or_forward({
            ops_lateral_hold_body_dir_x_,
            ops_lateral_hold_body_dir_y_,
        });
        if (dot(body_dir, locked_dir) < kOpsLateralDirectionResetDot)
        {
            _Reset_OPS_Lateral_Hold();
        }
    }

    if (!ops_lateral_hold_active_)
    {
        ops_lateral_hold_yaw_target_ =
            ops_yaw_hold_active_ ? ops_yaw_hold_target_ : normalize_angle_deg(ops_data.Yaw);
        ops_lateral_hold_start_x_ = ops_data.Pos_X;
        ops_lateral_hold_start_y_ = ops_data.Pos_Y;
        ops_lateral_hold_body_dir_x_ = body_dir.x;
        ops_lateral_hold_body_dir_y_ = body_dir.y;
        ops_lateral_hold_active_ = true;
    }

    const Vec2 locked_body_dir = normalize_or_forward({
        ops_lateral_hold_body_dir_x_,
        ops_lateral_hold_body_dir_y_,
    });
    const float yaw_target_deg = normalize_angle_deg(ops_lateral_hold_yaw_target_ +
                                                    kOpsYawToBodyFrameOffsetDeg);
    const Vec2 world_dir = normalize_or_forward(body_to_world(locked_body_dir,
                                                             yaw_target_deg * kDegToRad));
    const Vec2 world_lat {-world_dir.y, world_dir.x};
    const Vec2 delta_pos {
        ops_data.Pos_X - ops_lateral_hold_start_x_,
        ops_data.Pos_Y - ops_lateral_hold_start_y_,
    };
    const float lateral_error_mm = dot(delta_pos, world_lat);

    const bool low_speed = linear_speed < kOpsLateralLowSpeedThreshold;
    const float lateral_kp = low_speed ? kOpsLateralKpLowSpeed : kOpsLateralKpHighSpeed;
    const float lateral_limit = low_speed ? kOpsLateralSpeedLimitLow : kOpsLateralSpeedLimitHigh;
    const float lateral_correction =
        clampf(-lateral_kp * lateral_error_mm * 0.001f,
               -lateral_limit,
                lateral_limit);

    const Vec2 desired_world {
        world_dir.x * linear_speed + world_lat.x * lateral_correction,
        world_dir.y * linear_speed + world_lat.y * lateral_correction,
    };
    const Vec2 desired_body =
        world_to_body(desired_world,
                      normalize_angle_deg(ops_data.Yaw + kOpsYawToBodyFrameOffsetDeg) * kDegToRad);

    vx = desired_body.x;
    vy = desired_body.y;

    const float corrected_speed = std::sqrt(vx * vx + vy * vy);
    if (corrected_speed > MAX_OMNI_CHASSIS_SPEED && corrected_speed > 1e-4f)
    {
        const float scale = MAX_OMNI_CHASSIS_SPEED / corrected_speed;
        vx *= scale;
        vy *= scale;
    }

    ops_lateral_hold_last_error_mm_ = lateral_error_mm;
    ops_lateral_hold_last_output_ = lateral_correction;
}

float Class_Robot::_Apply_OPS_Yaw_Hold(float vx, float vy, float omega_cmd)
{
    const float linear_speed = std::sqrt(vx * vx + vy * vy);
    const bool manual_rotate = std::fabs(omega_cmd) > kOpsYawHoldManualOmegaDeadband;
    const bool should_hold_yaw = linear_speed > kOpsYawHoldMinLinearSpeed && !manual_rotate;

    if (!should_hold_yaw || !ops.isConnected())
    {
        ops_yaw_hold_active_ = false;
        ops_yaw_hold_last_output_ = 0.0f;
        ops_yaw_hold_last_error_ = 0.0f;
        return omega_cmd;
    }

    const Struct_OPS_Rx_Data ops_data = ops.getData();
    if (!std::isfinite(ops_data.Yaw))
    {
        ops_yaw_hold_active_ = false;
        ops_yaw_hold_last_output_ = 0.0f;
        ops_yaw_hold_last_error_ = 0.0f;
        return omega_cmd;
    }

    if (!ops_yaw_hold_active_)
    {
        ops_yaw_hold_target_ = normalize_angle_deg(ops_data.Yaw);
        ops_yaw_hold_active_ = true;
    }

    const float yaw_now = normalize_angle_deg(ops_data.Yaw);
    const float yaw_error = normalize_angle_deg(ops_yaw_hold_target_ - yaw_now);
    const float yaw_rate = std::isfinite(ops_data.Omega_Z) ? ops_data.Omega_Z : 0.0f;
    const float correction =
        clampf(kOpsYawHoldKp * yaw_error - kOpsYawHoldKd * yaw_rate,
               -kOpsYawHoldMaxCorrection,
                kOpsYawHoldMaxCorrection);

    ops_yaw_hold_last_error_ = yaw_error;
    ops_yaw_hold_last_output_ = correction;
    return correction;
}

void Class_Robot::_Reset_OPS_Lateral_Hold()
{
    ops_lateral_hold_active_ = false;
    ops_lateral_hold_last_error_mm_ = 0.0f;
    ops_lateral_hold_last_output_ = 0.0f;
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
            chassis_remote_enabled_ = false;
            _Log_Chassis_Start_Gate("BACK pressed: chassis remote DISABLED");
        }
        last_chassis_button_code_ = button_code;
        return;
    }

    if (start_pressed && start_rising)
    {
        chassis_remote_enabled_ = true;
        Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
        Lift.Send_Enable_Burst();
        _Log_Chassis_Start_Gate("START pressed: chassis remote ENABLED");
    }

    last_chassis_button_code_ = button_code;
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
    const bool ops_connected = ops.isConnected();
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
                    "chassis diag: source=%s remote=%d ros_cmd_recent=%d remote_cmd_recent=%d ops=%d"
                    " target=(%.3f, %.3f, %.3f)"
                    " now=(%.3f, %.3f, %.3f) yaw_hold=%d yaw_err=%.2fdeg yaw_out=%.3f"
                    " lat_hold=%d lat_err=%.1fmm lat_out=%.3f unhandled_can=%llu",
                    source_name,
                    chassis_remote_enabled_ ? 1 : 0,
                    ros_cmd_recent ? 1 : 0,
                    remote_cmd_recent ? 1 : 0,
                    ops_connected ? 1 : 0,
                    Chassis.Get_Target_Velocity_X(),
                    Chassis.Get_Target_Velocity_Y(),
                    Chassis.Get_Target_Omega(),
                    Chassis.Get_Now_Velocity_X(),
                    Chassis.Get_Now_Velocity_Y(),
                    Chassis.Get_Now_Omega(),
                    ops_yaw_hold_active_ ? 1 : 0,
                    ops_yaw_hold_last_error_,
                    ops_yaw_hold_last_output_,
                    ops_lateral_hold_active_ ? 1 : 0,
                    ops_lateral_hold_last_error_mm_,
                    ops_lateral_hold_last_output_,
                    static_cast<unsigned long long>(unhandled));
    }
    else
    {
        std::cout << "[ROBOT] chassis diag"
                  << " source=" << source_name
                  << " remote=" << (chassis_remote_enabled_ ? 1 : 0)
                  << " ros_cmd_recent=" << (ros_cmd_recent ? 1 : 0)
                  << " remote_cmd_recent=" << (remote_cmd_recent ? 1 : 0)
                  << " ops=" << (ops_connected ? 1 : 0)
                  << " target=(" << Chassis.Get_Target_Velocity_X()
                  << ", " << Chassis.Get_Target_Velocity_Y()
                  << ", " << Chassis.Get_Target_Omega() << ")"
                  << " now=(" << Chassis.Get_Now_Velocity_X()
                  << ", " << Chassis.Get_Now_Velocity_Y()
                  << ", " << Chassis.Get_Now_Omega() << ")"
                  << " yaw_hold=" << (ops_yaw_hold_active_ ? 1 : 0)
                  << " yaw_err_deg=" << ops_yaw_hold_last_error_
                  << " yaw_out=" << ops_yaw_hold_last_output_
                  << " lat_hold=" << (ops_lateral_hold_active_ ? 1 : 0)
                  << " lat_err_mm=" << ops_lateral_hold_last_error_mm_
                  << " lat_out=" << ops_lateral_hold_last_output_
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
    const bool buttons_recent = snapshot.has_buttons && snapshot.last_button_ns > 0 &&
                                (now - snapshot.last_button_ns) <= kButtonTimeoutNs;

    if (!chassis_remote_enabled_)
    {
        Lift.Set_Diff_Drive_Enable(false);
        Lift.Set_Target_Diff_Command(0.0f, 0.0f);
        Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_DISABLE);
        last_lift_button_code_ = buttons_recent ? snapshot.button_code : LogF710_Key_IDLE;
        return;
    }

    Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);

    auto log_lift_action = [this](const char *msg) {
        if (bridge_node_)
            RCLCPP_WARN(bridge_node_->get_logger(), "%s", msg);
        else
            std::cerr << "[ROBOT][WARN] " << msg << std::endl;
    };

    if (buttons_recent)
    {
        const bool a_rising =
            (snapshot.button_code == LogF710_Key_A) &&
            (last_lift_button_code_ != LogF710_Key_A);
        const bool b_rising =
            (snapshot.button_code == LogF710_Key_B) &&
            (last_lift_button_code_ != LogF710_Key_B);
        const bool x_rising =
            (snapshot.button_code == LogF710_Key_X) &&
            (last_lift_button_code_ != LogF710_Key_X);

        if (a_rising)
        {
            Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
            Lift.Set_Front_Lift_State(CHARIOT_LIFT_POSITION_RAISE);
            Lift.Set_Rear_Lift_State(CHARIOT_LIFT_POSITION_RAISE);
            Lift.Set_Diff_Drive_Module(CHARIOT_LIFT_MODULE_FRONT);
            Lift.Set_Diff_Drive_Enable(true);
            log_lift_action("A pressed: front+rear lift RAISE, right stick controls front differential drive");
        }
        else if (b_rising)
        {
            Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
            Lift.Set_Front_Lift_State(CHARIOT_LIFT_POSITION_RETRACT);
            log_lift_action("B pressed: front lift RETRACT, stick mapping unchanged");
        }
        else if (x_rising)
        {
            Lift.Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
            Lift.Set_Rear_Lift_State(CHARIOT_LIFT_POSITION_RETRACT);
            Lift.Set_Diff_Drive_Enable(false);
            log_lift_action("X pressed: rear lift RETRACT, right stick returns to omni yaw");
        }

        last_lift_button_code_ = snapshot.button_code;
    }
    else
    {
        last_lift_button_code_ = LogF710_Key_IDLE;
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

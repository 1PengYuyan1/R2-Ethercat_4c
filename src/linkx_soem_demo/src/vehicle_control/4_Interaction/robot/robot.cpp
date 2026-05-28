// 上位机版 Class_Robot 实现：把 ROS /chassis/cmd_vel + /chassis/gantry_state + /imu
// 转化为底盘速度 / 龙门状态命令；通过 linkx_t 推送给 EtherCAT-CAN 桥；
// 把 chassis 实际速度回报到 /chassis/odom_twist。
//
// 工况：全向轮底盘（4 × DM3519 MIT，全部布置在 LinkX channel 0）
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
}

void Class_Robot::Init(linkx_t *__LinkX_Handler)
{
    LinkX_Handler = __LinkX_Handler;

    Chassis.Init(LinkX_Handler);
    Chassis.Init_Motor_Params();

    Gantry.Init(LinkX_Handler);
    Arm.Init(LinkX_Handler);

    Navigation.Init();

    _Verify_Motor_ID_Uniqueness();
}

void Class_Robot::_Verify_Motor_ID_Uniqueness()
{
    // R2 全向轮 CAN 通道分布：
    //   ch0：4× DM3519 全向轮（Rx 0x11-0x14）
    //   ch1：未使用（原舵向 CAN 总线，现保留以兼容硬件）
    //   ch2：2× Gantry DM + 1× Arm DM
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

    uint16_t wheel_ids[OMNI_WHEEL_NUM];
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        wheel_ids[i] = Chassis.Motor_Wheel[i].DM_CAN_Rx_ID;
    }
    check_pair("ch0 (omni wheel)", wheel_ids, OMNI_WHEEL_NUM);

    uint16_t ch2_ids[3] = {
        Gantry.Motor_Lift_Right.DM_CAN_Rx_ID,
        Gantry.Motor_Lift_Left .DM_CAN_Rx_ID,
        Arm   .Arm             .DM_CAN_Rx_ID,
    };
    check_pair("ch2 (gantry+arm)", ch2_ids, 3);
}

void Class_Robot::Loop()
{
    _Chassis_Control();
    _Gantry_control();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
}

void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
    Gantry.TIM_100ms_Alive_PeriodElapsedCallback();
    Arm.TIM_100ms_Alive_PeriodElapsedCallback();
}

void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
    Gantry.TIM_Calculate_PeriodElapsedCallback();
    Arm.TIM_Calculate_PeriodElapsedCallback();
}

void Class_Robot::TIM_1ms_Calculate_Callback()
{
    _Chassis_Control();
    _Gantry_control();
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
        rclcpp::init(argc, argv);
        ros_initialized_here_ = true;
    }

    bridge_node_ = std::make_shared<rclcpp::Node>("r2_vehicle_bridge");

    sub_cmd_ = bridge_node_->create_subscription<geometry_msgs::msg::Twist>(
        "/chassis/cmd_vel", 20,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            _Update_Twist(static_cast<float>(msg->linear.x),
                          static_cast<float>(msg->linear.y),
                          static_cast<float>(msg->angular.z));
        });

    sub_gantry_ = bridge_node_->create_subscription<std_msgs::msg::UInt8>(
        "/chassis/gantry_state", 20,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) { _Update_Gantry_State(msg->data); });

    sub_imu_ = bridge_node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 20,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            latest_imu_omega_z_.store(static_cast<float>(msg->angular_velocity.z));
        });

    pub_odom_ = bridge_node_->create_publisher<geometry_msgs::msg::Twist>("/chassis/odom_twist", 50);

    ops.init(bridge_node_.get());

    ros_bridge_running_.store(true);
    ros_spin_thread_ = std::thread(&Class_Robot::_ROS2_Spin_Loop, this);

    RCLCPP_INFO(bridge_node_->get_logger(),
                "ROS2 bridge up: subs=/chassis/cmd_vel,/chassis/gantry_state,/imu  pub=/chassis/odom_twist");
}

void Class_Robot::Stop_ROS2_Bridge()
{
    if (!ros_bridge_running_.load()) return;
    ros_bridge_running_.store(false);
    if (ros_spin_thread_.joinable()) ros_spin_thread_.join();

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

void Class_Robot::_Update_Twist(float vx, float vy, float omega)
{
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    ros_cmd_.vx = vx;
    ros_cmd_.vy = vy;
    ros_cmd_.omega = omega;
    ros_cmd_.has_twist = true;
    ros_cmd_.last_twist_ns = now_ns();
}

void Class_Robot::_Update_Gantry_State(uint8_t state)
{
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    ros_cmd_.gantry_state = state;
    ros_cmd_.has_gantry = true;
}

// --- CAN 路由 --------------------------------------------------------------
//
// 全向轮工况分发策略：
//   - ch0 → 4× DM3519 全向轮，通过 Rx_ID 区分（0x11..0x14）
//   - ch1 → 未使用（原舵向通道，保留兼容硬件）
//   - ch2 → Gantry/Arm，通过 Rx_ID 区分（0x11/0x12/0x13）
//   - 其它通道 / 通道+ID 不匹配 → 累计到 unhandled_can_frames_

void Class_Robot::CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data)
{
    const uint32_t can_id_std = (CAN_ID & 0x7FFU);

    auto dispatch_wheel = [&]() -> bool {
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            if (can_id_std == Chassis.Motor_Wheel[i].DM_CAN_Rx_ID)
            {
                Chassis.Motor_Wheel[i].CAN_RxCpltCallback(CAN_Data);
                return true;
            }
        }
        return false;
    };

    auto dispatch_gantry_arm = [&]() -> bool {
        if (can_id_std == Gantry.Motor_Lift_Right.DM_CAN_Rx_ID)
        { Gantry.Motor_Lift_Right.CAN_RxCpltCallback(CAN_Data); return true; }
        if (can_id_std == Gantry.Motor_Lift_Left .DM_CAN_Rx_ID)
        { Gantry.Motor_Lift_Left .CAN_RxCpltCallback(CAN_Data); return true; }
        if (can_id_std == Arm.Arm                .DM_CAN_Rx_ID)
        { Arm.Arm                .CAN_RxCpltCallback(CAN_Data); return true; }
        return false;
    };

    bool handled = false;
    switch (CAN_Channel)
    {
        case 0: handled = dispatch_wheel();      break;
        case 2: handled = dispatch_gantry_arm(); break;
        default: /* ch1/ch3 未配置任何执行器 */  break;
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

    const bool is_recent = snapshot.has_twist && snapshot.last_twist_ns > 0 &&
                           (now_ns() - snapshot.last_twist_ns) <= kCmdTimeoutNs;

    if (is_recent)
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_ENABLE);
        Chassis.Set_Target_Velocity_X(snapshot.vx);
        Chassis.Set_Target_Velocity_Y(snapshot.vy);
        Chassis.Set_Target_Omega(snapshot.omega);
    }
    else
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);
        Chassis.Set_Target_Velocity_X(0.0f);
        Chassis.Set_Target_Velocity_Y(0.0f);
        Chassis.Set_Target_Omega(0.0f);
    }
}

void Class_Robot::_Gantry_control()
{
    Ros_Command snapshot;
    {
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        snapshot = ros_cmd_;
    }

    if (snapshot.has_gantry)
    {
        if      (snapshot.gantry_state == 0) current_gantry_state_ = GANTRY_STATE_LOW_POS;
        else if (snapshot.gantry_state == 1) current_gantry_state_ = GANTRY_STATE_HIGH_POS;
        // 其它值（含 0xFF）保持当前状态
    }
    else
    {
        current_gantry_state_ = GANTRY_STATE_DISABLE;
    }

    switch (current_gantry_state_)
    {
    case GANTRY_STATE_DISABLE:
        Gantry.Set_Gantry_Control_Type(GANTRY_CONTROL_DISABLE);
        Arm.Set_Arm_Control_Type(ARM_CONTROL_DISABLE);
        Arm_timer_ = 0;
        gantry_timer_ = 0;
        break;

    case GANTRY_STATE_LOW_POS:
        Gantry.Set_Gantry_Control_Type(GANTRY_CONTROL_ENABLE);
        Arm.Set_Arm_Control_Type(ARM_CONTROL_ENABLE);
        Gantry.Set_Lift_State(LIFT_POS2);
        Arm.Set_Arm_State(Arm_grab);
        Arm_timer_ = 0;
        gantry_timer_ = 0;
        break;

    case GANTRY_STATE_HIGH_POS:
        Gantry.Set_Gantry_Control_Type(GANTRY_CONTROL_ENABLE);
        Arm.Set_Arm_Control_Type(ARM_CONTROL_ENABLE);
        if (Arm_timer_ < 1000) Arm_timer_++;
        if (Arm_timer_ >= 500)
        {
            Arm.Set_Arm_State(Arm_zero);
            if (gantry_timer_ < 2000) gantry_timer_++;
            if (gantry_timer_ >= 1000) Gantry.Set_Lift_State(LIFT_POS1);
        }
        break;
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

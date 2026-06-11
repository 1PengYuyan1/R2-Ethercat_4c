#include "crt_lift.h"

#include <cmath>

/**
 * @brief 初始化车头和车尾两个抬升模块，并绑定 LinkX CAN 通道。
 *
 * 新车头为原车尾，因此 CAN0 是车头抬升模块，CAN1 是车尾抬升模块。
 */
void Class_Chariot_Lift::Init(linkx_t *__LinkX_Handler)
{
    Init_Motor_Params();

    Motor_Drive_Left[CHARIOT_LIFT_MODULE_FRONT].Init(__LinkX_Handler, 0, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Drive_Right[CHARIOT_LIFT_MODULE_FRONT].Init(__LinkX_Handler, 0, 0x14, 0x04, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Lift[CHARIOT_LIFT_MODULE_FRONT].Init(__LinkX_Handler, 0, 0x15, 0x05, Motor_DM_Control_Method_NORMAL_MIT, 62.8f, 395.0f, 7.8f, 9.2f);

    Motor_Drive_Left[CHARIOT_LIFT_MODULE_REAR].Init(__LinkX_Handler, 1, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Drive_Right[CHARIOT_LIFT_MODULE_REAR].Init(__LinkX_Handler, 1, 0x14, 0x04, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Lift[CHARIOT_LIFT_MODULE_REAR].Init(__LinkX_Handler, 1, 0x15, 0x05, Motor_DM_Control_Method_NORMAL_MIT, 62.8f, 395.0f, 7.8f, 9.2f);

    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; i++)
    {
        Motor_Drive_Left[i].Set_Use_FDCAN(true);
        Motor_Drive_Right[i].Set_Use_FDCAN(true);
        Motor_Lift[i].Set_Use_FDCAN(true);

        Motor_Drive_Left[i].Set_Force_Output_Without_Feedback(false);
        Motor_Drive_Right[i].Set_Force_Output_Without_Feedback(false);
        Motor_Lift[i].Set_Force_Output_Without_Feedback(false);
    }
}

/**
 * @brief 抬升模块参数初始化（默认值，可在外部覆盖）
 */
void Class_Chariot_Lift::Init_Motor_Params()
{
    Drive_Params[CHARIOT_LIFT_MODULE_FRONT] = {
        .wheel_radius           = 0.0761f,
        .track_width            = 0.420f,
        .max_forward_speed      = 1.0f,
        .max_yaw_omega          = 2.0f,
        .max_wheel_omega        = 80.0f,
        .wheel_kp               = 0.0f,
        .wheel_kd               = 1.2f,
        .wheel_deadzone         = 0.05f,
        .left_direction         = 1.0f,
        .right_direction        = 1.0f,
        .left_speed_correction  = 1.0f,
        .right_speed_correction = 1.0f,
        .accel_limit            = 300.0f,
        .decel_limit            = 600.0f,
    };

    Drive_Params[CHARIOT_LIFT_MODULE_REAR] = {
        .wheel_radius           = 0.0761f,
        .track_width            = 0.420f,
        .max_forward_speed      = 1.0f,
        .max_yaw_omega          = 2.0f,
        .max_wheel_omega        = 80.0f,
        .wheel_kp               = 0.0f,
        .wheel_kd               = 1.2f,
        .wheel_deadzone         = 0.05f,
        .left_direction         = 1.0f,
        .right_direction        = 1.0f,
        .left_speed_correction  = 1.0f,
        .right_speed_correction = 1.0f,
        .accel_limit            = 300.0f,
        .decel_limit            = 600.0f,
    };

    Lift_Params[CHARIOT_LIFT_MODULE_FRONT] = {
        .retract_angle = 0.0f,
        .raise_angle   = -20.0f,
        .max_speed     = 5.0f,
        .kp            = 15.0f,
        .kd            = 1.0f,
    };

    Lift_Params[CHARIOT_LIFT_MODULE_REAR] = {
        .retract_angle = 0.0f,
        .raise_angle   = -20.0f,
        .max_speed     = 5.0f,
        .kp            = 15.0f,
        .kd            = 1.0f,
    };

    Smooth_Lift_Angle[CHARIOT_LIFT_MODULE_FRONT] =
        Lift_Params[CHARIOT_LIFT_MODULE_FRONT].retract_angle;
    Smooth_Lift_Angle[CHARIOT_LIFT_MODULE_REAR] =
        Lift_Params[CHARIOT_LIFT_MODULE_REAR].retract_angle;
}

/**
 * @brief 将收到的一帧 CAN 报文路由到对应模块内匹配的电机。
 */
bool Class_Chariot_Lift::CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data)
{
    const uint32_t can_id_std = CAN_ID & 0x7FFU;
    Enum_Chariot_Lift_Module module;

    /* 新车头为原车尾：CAN0=车头抬升模块，CAN1=车尾抬升模块 */
    if (CAN_Channel == 0U)
    {
        module = CHARIOT_LIFT_MODULE_FRONT;
    }
    else if (CAN_Channel == 1U)
    {
        module = CHARIOT_LIFT_MODULE_REAR;
    }
    else
    {
        return false;
    }

    if (can_id_std == Motor_Drive_Left[module].DM_CAN_Rx_ID)
    {
        Motor_Drive_Left[module].CAN_RxCpltCallback(CAN_Data);
        return true;
    }
    if (can_id_std == Motor_Drive_Right[module].DM_CAN_Rx_ID)
    {
        Motor_Drive_Right[module].CAN_RxCpltCallback(CAN_Data);
        return true;
    }
    if (can_id_std == Motor_Lift[module].DM_CAN_Rx_ID)
    {
        Motor_Lift[module].CAN_RxCpltCallback(CAN_Data);
        return true;
    }

    return false;
}

/**
 * @brief 刷新电机在线状态，并在抬升控制启用时保持电机使能。
 */
void Class_Chariot_Lift::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Motor_Drive_Left[i].TIM_Alive_PeriodElapsedCallback();
        Motor_Drive_Right[i].TIM_Alive_PeriodElapsedCallback();
        Motor_Lift[i].TIM_Alive_PeriodElapsedCallback();
    }

    if (Control_Type == CHARIOT_LIFT_CONTROL_ENABLE)
    {
        Ensure_All_Motors_Enabled();
    }
}

/**
 * @brief 2 ms 抬升主循环：START 后保持电机使能，机构按钮按下后才进入动作控制。
 */
void Class_Chariot_Lift::TIM_2ms_Control_PeriodElapsedCallback()
{
    switch (Control_Type)
    {
        case CHARIOT_LIFT_CONTROL_DISABLE:
        {
            Disable_All_Motors(control_disable_exit_burst_ticks_ > 0);
            if (control_disable_exit_burst_ticks_ > 0)
                --control_disable_exit_burst_ticks_;
            Reset_Drive_State();
            return;
        }

        case CHARIOT_LIFT_CONTROL_ENABLE:
        {
            Ensure_All_Motors_Enabled();

            for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
            {
                if (Lift_Module_Enable[i])
                    Output_Lift_Motor(static_cast<Enum_Chariot_Lift_Module>(i));
            }

            Kinematics_Diff_Resolution();
            Apply_Drive_Trapezoid_Profile();

            if (Diff_Drive_Enable || drive_disable_exit_burst_ticks_ > 0)
            {
                for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
                    Output_Drive_Motor(static_cast<Enum_Chariot_Lift_Module>(i));
            }
            else
            {
                Reset_Drive_State();
            }

            if (drive_disable_exit_burst_ticks_ > 0)
                --drive_disable_exit_burst_ticks_;
            break;
        }
    }
}

/**
 * @brief 让单台 DM 保持 Enter/使能态；不下发 MIT 控制量。
 */
void Class_Chariot_Lift::Ensure_Motor_Enabled(Class_Motor_DM_Normal &motor, bool clear_disable_state)
{
    const auto control_status = motor.Get_Now_Control_Status();
    if (control_status == Motor_DM_Control_Status_ENABLE)
        return;

    if (control_status == Motor_DM_Control_Status_DISABLE)
    {
        if (clear_disable_state)
            motor.CAN_Send_Clear_Error();
        else
            motor.CAN_Send_Enter();
    }
    else
    {
        motor.CAN_Send_Clear_Error();
    }
}

/**
 * @brief START 后整车使能：六台抬升相关电机都先进入电机使能态。
 */
void Class_Chariot_Lift::Ensure_All_Motors_Enabled()
{
    const int motor_slot = enable_service_tick_ % 3;
    const bool clear_disable_state = ((enable_service_tick_ / 3) % 2) == 0;

    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        switch (motor_slot)
        {
            case 0:
                Ensure_Motor_Enabled(Motor_Drive_Left[i], clear_disable_state);
                break;
            case 1:
                Ensure_Motor_Enabled(Motor_Drive_Right[i], clear_disable_state);
                break;
            default:
                Ensure_Motor_Enabled(Motor_Lift[i], clear_disable_state);
                break;
        }
    }

    ++enable_service_tick_;
}

/**
 * @brief START 上升沿立即发送一次六电机 Enter，不进入任何动作控制。
 */
void Class_Chariot_Lift::Send_Enable_Burst()
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Motor_Drive_Left[i].CAN_Send_Enter();
        Motor_Drive_Right[i].CAN_Send_Enter();
        Motor_Lift[i].CAN_Send_Enter();
    }
}

/**
 * @brief 更新单个抬升电机目标位置，包含抬升杆侧平滑和齿轮比换算。
 */
void Class_Chariot_Lift::Output_Lift_Motor(Enum_Chariot_Lift_Module module)
{
    const ChariotLiftPositionParams &params = Lift_Params[module];
    const float target_lift_rod_angle =
        !Lift_Module_Enable[module] ? Smooth_Lift_Angle[module] :
        (Lift_State[module] == CHARIOT_LIFT_POSITION_RAISE) ?
        params.raise_angle :
        params.retract_angle;

    // Smooth_Lift_Angle 始终表示抬升杆侧角度，仅在发送给电机前换算。
    Smooth_Lift_Angle[module] = Ramp_To(Smooth_Lift_Angle[module],
                                        target_lift_rod_angle,
                                        params.max_speed,
                                        params.max_speed,
                                        0.002f);

    if (Motor_Lift[module].Get_Status() != Motor_DM_Status_ENABLE)
    {
        Motor_Lift[module].CAN_Send_Enter();
    }
    else
    {
        const float motor_target_angle = Lift_Rod_To_Motor_Angle(Smooth_Lift_Angle[module]);
        Motor_Lift[module].Set_Control_Maintain_Postion(motor_target_angle,
                                                        0.0f,
                                                        0.0f,
                                                        params.kp,
                                                        params.kd);
    }
    Motor_Lift[module].TIM_Send_PeriodElapsedCallback();
}

/**
 * @brief 将临时底盘前进和偏航命令换算为左右抬升驱动轮角速度。
 */
void Class_Chariot_Lift::Kinematics_Diff_Resolution()
{
    /* 1. 只有被选中的抬升模块参与临时差速驱动 */
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Raw_Target_Left_Omega[i] = 0.0f;
        Raw_Target_Right_Omega[i] = 0.0f;
    }

    if (!Diff_Drive_Enable)
        return;

    const int module = Diff_Drive_Module;
    const ChariotLiftDriveParams &params = Drive_Params[module];

    /* 2. 遥控前进速度和偏航角速度限幅 */
    const float forward_cmd = Clamp(Target_Diff_Forward,
                                    -params.max_forward_speed,
                                     params.max_forward_speed);
    const float yaw = Clamp(Target_Diff_Yaw,
                            -params.max_yaw_omega,
                             params.max_yaw_omega);

    // 抬升驱动轮的左右电机方向仍按原车体系标定；新车头为原车尾，所以前进分量取反。
    const float forward = -forward_cmd;

    /* 3. 差速运动学分解 */
    const float left_linear = forward - yaw * params.track_width * 0.5f;
    const float right_linear = forward + yaw * params.track_width * 0.5f;

    // 先将车体速度换算为轮轴角速度，再叠加安装方向和速度修正。
    Raw_Target_Left_Omega[module] =
        (left_linear / params.wheel_radius) *
        params.left_direction *
        params.left_speed_correction;
    Raw_Target_Right_Omega[module] =
        (right_linear / params.wheel_radius) *
        params.right_direction *
        params.right_speed_correction;

    /* 4. 轮速归一化，保持左右轮比例不变 */
    const float max_abs = fmaxf(fabsf(Raw_Target_Left_Omega[module]),
                                fabsf(Raw_Target_Right_Omega[module]));
    if (max_abs > params.max_wheel_omega && max_abs > 1e-4f)
    {
        const float scale = params.max_wheel_omega / max_abs;
        Raw_Target_Left_Omega[module] *= scale;
        Raw_Target_Right_Omega[module] *= scale;
    }

    /* 5. 死区处理 */
    if (fabsf(Raw_Target_Left_Omega[module]) < params.wheel_deadzone)
        Raw_Target_Left_Omega[module] = 0.0f;
    if (fabsf(Raw_Target_Right_Omega[module]) < params.wheel_deadzone)
        Raw_Target_Right_Omega[module] = 0.0f;
}

/**
 * @brief 对抬升驱动轮目标角速度施加加减速限制。
 */
void Class_Chariot_Lift::Apply_Drive_Trapezoid_Profile()
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        const ChariotLiftDriveParams &params = Drive_Params[i];
        Target_Left_Omega[i] = Ramp_To(Target_Left_Omega[i],
                                       Raw_Target_Left_Omega[i],
                                       params.accel_limit,
                                       params.decel_limit,
                                       0.002f);
        Target_Right_Omega[i] = Ramp_To(Target_Right_Omega[i],
                                        Raw_Target_Right_Omega[i],
                                        params.accel_limit,
                                        params.decel_limit,
                                        0.002f);
    }
}

/**
 * @brief 向单个模块的左右驱动轮电机发送 MIT 速度命令。
 */
void Class_Chariot_Lift::Output_Drive_Motor(Enum_Chariot_Lift_Module module)
{
    const ChariotLiftDriveParams &params = Drive_Params[module];

    Motor_Drive_Left[module].Set_Control_Maintain_Postion(0.0f,
                                                          Target_Left_Omega[module],
                                                          0.0f,
                                                          params.wheel_kp,
                                                          params.wheel_kd);
    Motor_Drive_Left[module].TIM_Send_PeriodElapsedCallback();

    Motor_Drive_Right[module].Set_Control_Maintain_Postion(0.0f,
                                                           Target_Right_Omega[module],
                                                           0.0f,
                                                           params.wheel_kp,
                                                           params.wheel_kd);
    Motor_Drive_Right[module].TIM_Send_PeriodElapsedCallback();
}

/**
 * @brief 失能单个模块的两台抬升驱动轮电机，并重置轮速目标。
 */
void Class_Chariot_Lift::Disable_Drive_Motor(Enum_Chariot_Lift_Module module, bool force_exit)
{
    Target_Left_Omega[module] = 0.0f;
    Target_Right_Omega[module] = 0.0f;
    Raw_Target_Left_Omega[module] = 0.0f;
    Raw_Target_Right_Omega[module] = 0.0f;

    Motor_Drive_Left[module].Set_Control_Status(Motor_DM_Status_DISABLE);
    Motor_Drive_Left[module].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (force_exit ||
        Motor_Drive_Left[module].Get_Now_Control_Status() != Motor_DM_Control_Status_DISABLE)
    {
        Motor_Drive_Left[module].CAN_Send_Exit();
    }

    Motor_Drive_Right[module].Set_Control_Status(Motor_DM_Status_DISABLE);
    Motor_Drive_Right[module].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (force_exit ||
        Motor_Drive_Right[module].Get_Now_Control_Status() != Motor_DM_Control_Status_DISABLE)
    {
        Motor_Drive_Right[module].CAN_Send_Exit();
    }
}

/**
 * @brief 失能所有电机，并让抬升平滑状态对齐当前电机反馈。
 */
void Class_Chariot_Lift::Disable_All_Motors(bool force_exit)
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Disable_Drive_Motor(static_cast<Enum_Chariot_Lift_Module>(i), force_exit);

        Motor_Lift[i].Set_Control_Status(Motor_DM_Status_DISABLE);
        Motor_Lift[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        if (force_exit ||
            Motor_Lift[i].Get_Now_Control_Status() != Motor_DM_Control_Status_DISABLE)
        {
            Motor_Lift[i].CAN_Send_Exit();
        }
        Smooth_Lift_Angle[i] = Motor_To_Lift_Rod_Angle(Motor_Lift[i].Get_Now_Radian());
        Lift_Module_Enable[i] = false;
    }
}

/**
 * @brief 清除临时差速驱动命令和轮速状态。
 */
void Class_Chariot_Lift::Reset_Drive_State()
{
    Target_Diff_Forward = 0.0f;
    Target_Diff_Yaw = 0.0f;
    Diff_Drive_Enable = false;
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Raw_Target_Left_Omega[i] = 0.0f;
        Raw_Target_Right_Omega[i] = 0.0f;
        Target_Left_Omega[i] = 0.0f;
        Target_Right_Omega[i] = 0.0f;
    }
}

/**
 * @brief 将数值限制在闭区间范围内。
 */
float Class_Chariot_Lift::Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/**
 * @brief 按加速度限制将当前值向目标值推进一个控制周期。
 */
float Class_Chariot_Lift::Ramp_To(float current,
                                  float target,
                                  float accel_limit,
                                  float decel_limit,
                                  float dt)
{
    const float delta = target - current;
    const float abs_delta = fabsf(delta);
    if (abs_delta < 1e-5f)
        return target;

    const bool changing_direction =
        (current * target < 0.0f) && (fabsf(current) > 1e-4f);
    const bool reducing_speed = fabsf(target) < fabsf(current);
    const float limit = (changing_direction || reducing_speed) ? decel_limit : accel_limit;
    const float max_step = limit * dt;

    if (abs_delta <= max_step)
        return target;

    return current + ((delta > 0.0f) ? max_step : -max_step);
}

/**
 * @brief 通过 1:3 齿轮比，将抬升杆侧角度换算为 DM3519 电机轴角度。
 */
float Class_Chariot_Lift::Lift_Rod_To_Motor_Angle(float lift_rod_angle)
{
    return lift_rod_angle * 3.0f;
}

/**
 * @brief 通过 1:3 齿轮比，将 DM3519 电机轴角度换算回抬升杆侧角度。
 */
float Class_Chariot_Lift::Motor_To_Lift_Rod_Angle(float motor_angle)
{
    return motor_angle / 3.0f;
}

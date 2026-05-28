#include "crt_chassis_omni.h"

/**
 * @brief 全向轮底盘硬件初始化
 *
 * 4 个 DM3519 全部布置在 LinkX channel 0：
 *   Rx_ID 0x11..0x14, Tx_ID 0x01..0x04
 *   MIT 模式：PMAX=12.5 rad（实际本算法不用位置环），VMAX=150 rad/s，
 *             TMAX=5 N·m，IMAX=12 A（与电机出厂上电打印一致）
 */
void Class_Chassis_Omni::Init(linkx_t *__LinkX_Handler)
{
    Motor_Wheel[0].Init(__LinkX_Handler, 0, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Wheel[1].Init(__LinkX_Handler, 0, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Wheel[2].Init(__LinkX_Handler, 0, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Wheel[3].Init(__LinkX_Handler, 0, 0x14, 0x04, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
}

/**
 * @brief 全向轮参数初始化（默认值，可在外部覆盖）
 */
void Class_Chassis_Omni::Init_Motor_Params()
{
    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        wheel_params_[i] = {
            .wheel_kd               = 5.0f,
            .wheel_direction        = 1.0f,
            .wheel_speed_correction = 1.0f,
            .wheel_omega_deadzone   = 0.05f,
        };
    }
}

/**
 * @brief 100ms 周期回调：检测电机存活、必要时重新使能
 */
void Class_Chassis_Omni::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        Motor_Wheel[i].TIM_Alive_PeriodElapsedCallback();
    }

    if (Chassis_Control_Type == Chassis_Omni_Control_Type_ENABLE)
    {
        for (int i = 0; i < OMNI_WHEEL_NUM; i++)
        {
            if (Motor_Wheel[i].Get_Status() != Motor_DM_Status_ENABLE)
            {
                Motor_Wheel[i].CAN_Send_Clear_Error();
                Motor_Wheel[i].CAN_Send_Enter();
            }
        }
    }
}

/**
 * @brief 2ms 周期回调：自身姿态/速度解算（里程计正解）
 */
void Class_Chassis_Omni::TIM_2ms_Resolution_PeriodElapsedCallback()
{
    Self_Resolution();
}

/**
 * @brief 2ms 周期回调：底盘控制主循环
 */
void Class_Chassis_Omni::TIM_2ms_Control_PeriodElapsedCallback()
{
    switch (Chassis_Control_Type)
    {
        case Chassis_Omni_Control_Type_DISABLE:
        {
            for (int i = 0; i < OMNI_WHEEL_NUM; i++)
            {
                Motor_Wheel[i].Set_Control_Status(Motor_DM_Status_DISABLE);
                Motor_Wheel[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                if (Motor_Wheel[i].Get_Now_Control_Status() != Motor_DM_Control_Status_DISABLE)
                {
                    Motor_Wheel[i].CAN_Send_Exit();
                }
                Target_Wheel_Omega[i] = 0.0f;
            }

            Target_Velocity_X = 0.0f;
            Target_Velocity_Y = 0.0f;
            Target_Omega      = 0.0f;

            _Reset_State();
            return;
        }

        case Chassis_Omni_Control_Type_ENABLE:
        {
            /* 命令斜坡平滑：避免使能瞬间速度跳变 */
            Ramped_Velocity_X = f_Ramp_Calc(Ramped_Velocity_X, Target_Velocity_X, OMNI_ACC_LIN_RAMP);
            Ramped_Velocity_Y = f_Ramp_Calc(Ramped_Velocity_Y, Target_Velocity_Y, OMNI_ACC_LIN_RAMP);
            Ramped_Omega      = f_Ramp_Calc(Ramped_Omega,      Target_Omega,      OMNI_ACC_ANG_RAMP);

            Kinematics_Inverse_Resolution();
            Output_To_Motor();
            break;
        }
    }
}

/**
 * @brief 里程计正解
 *
 * 数学推导（4 轮 X-布局，全向轮，对称安装）：
 *   每个轮子在底盘系下的滚动单位方向 t_i = (-sinθ_i, cosθ_i)
 *   轮的线速度 v_i = v_chassis · t_i + ω · L
 *                 = -v_x sinθ_i + v_y cosθ_i + ω · L
 *
 *   对称性： Σsinθ_i = 0, Σcosθ_i = 0, Σsinθ_i cosθ_i = 0,
 *           Σsin²θ_i = 2,  Σcos²θ_i = 2  (4 轮等间隔)
 *
 *   反解（最小二乘）：
 *     v_x =  Σ v_i · (-sinθ_i) / 2 = (-v1 - v2 + v3 + v4) / (2√2)
 *     v_y =  Σ v_i ·  cosθ_i  / 2 = ( v1 - v2 - v3 + v4) / (2√2)
 *     ω   =  Σ v_i / (4L)         = ( v1 + v2 + v3 + v4) / (4L)
 */
void Class_Chassis_Omni::Self_Resolution()
{
    Now_Velocity_X = 0.0f;
    Now_Velocity_Y = 0.0f;
    Now_Omega      = 0.0f;

    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        float wheel_speed = Motor_Wheel[i].Get_Now_Omega() *
                            Wheel_Radius *
                            wheel_params_[i].wheel_direction;

        Now_Velocity_X += wheel_speed * (-sinf(Wheel_Azimuth[i])) * 2.0f / (float)OMNI_WHEEL_NUM;
        Now_Velocity_Y += wheel_speed *   cosf(Wheel_Azimuth[i])  * 2.0f / (float)OMNI_WHEEL_NUM;
        Now_Omega      += wheel_speed / Wheel_To_Core_Distance / (float)OMNI_WHEEL_NUM;
    }
}

/**
 * @brief 运动学逆解
 *
 *   v_i = -v_x sinθ_i + v_y cosθ_i + ω · L
 *   ω_motor_i = (v_i / R_wheel) · direction · correction
 *
 * 处理流程：
 *   1) 整车线速度限幅 + 角速度限幅
 *   2) 逐轮分解，记录最大轮速
 *   3) 若超过电机额定，按比例缩放（保持矢量方向）
 *   4) 死区处理
 */
void Class_Chassis_Omni::Kinematics_Inverse_Resolution()
{
    /* 1. 整车线速度限幅 */
    float chassis_speed = sqrtf(Ramped_Velocity_X * Ramped_Velocity_X +
                                Ramped_Velocity_Y * Ramped_Velocity_Y);

    float vx_cmd = Ramped_Velocity_X;
    float vy_cmd = Ramped_Velocity_Y;
    if (chassis_speed > MAX_OMNI_CHASSIS_SPEED && chassis_speed > 1e-4f)
    {
        float scale = MAX_OMNI_CHASSIS_SPEED / chassis_speed;
        vx_cmd *= scale;
        vy_cmd *= scale;
    }

    /* 整车角速度限幅 */
    float omega_cmd = Ramped_Omega;
    if (omega_cmd >  MAX_OMNI_CHASSIS_OMEGA) omega_cmd =  MAX_OMNI_CHASSIS_OMEGA;
    if (omega_cmd < -MAX_OMNI_CHASSIS_OMEGA) omega_cmd = -MAX_OMNI_CHASSIS_OMEGA;

    /* 2. 逐轮分解 */
    float max_wheel_omega = 0.0f;
    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        float v_wheel = -vx_cmd * sinf(Wheel_Azimuth[i]) +
                         vy_cmd * cosf(Wheel_Azimuth[i]) +
                         omega_cmd * Wheel_To_Core_Distance;

        Target_Wheel_Omega[i] = (v_wheel / Wheel_Radius) *
                                wheel_params_[i].wheel_direction *
                                wheel_params_[i].wheel_speed_correction;

        if (fabsf(Target_Wheel_Omega[i]) > max_wheel_omega)
            max_wheel_omega = fabsf(Target_Wheel_Omega[i]);
    }

    /* 3. 轮速归一化（保持矢量方向不变） */
    if (max_wheel_omega > MAX_WHEEL_OMEGA && max_wheel_omega > 1e-4f)
    {
        float scale = MAX_WHEEL_OMEGA / max_wheel_omega;
        for (int i = 0; i < OMNI_WHEEL_NUM; i++)
            Target_Wheel_Omega[i] *= scale;
    }

    /* 4. 死区处理 */
    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        if (fabsf(Target_Wheel_Omega[i]) < wheel_params_[i].wheel_omega_deadzone)
            Target_Wheel_Omega[i] = 0.0f;
    }
}

/**
 * @brief 下发 MIT 报文
 *
 * MIT 速度模式（无位置环）：
 *   target_pos = 0, target_omega = Target_Wheel_Omega[i],
 *   target_torque = 0, Kp = 0, Kd = wheel_kd
 *
 * 与原舵轮代码 DRIVE 状态下的轮电机配置保持一致：
 *   Set_Control_Maintain_Postion(0.0f, Target_Wheel_Omega[i], 0.0f, 0.0f, kd)
 */
void Class_Chassis_Omni::Output_To_Motor()
{
    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        Motor_Wheel[i].Set_Control_Maintain_Postion(
            0.0f,
            Target_Wheel_Omega[i],
            0.0f,
            0.0f,
            wheel_params_[i].wheel_kd);
    }

    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        Motor_Wheel[i].TIM_Send_PeriodElapsedCallback();
    }
}

/**
 * @brief 内部状态清零（DISABLE 时调用，防止重新使能时冲击）
 */
void Class_Chassis_Omni::_Reset_State()
{
    Ramped_Velocity_X = 0.0f;
    Ramped_Velocity_Y = 0.0f;
    Ramped_Omega      = 0.0f;
    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
        Target_Wheel_Omega[i] = 0.0f;
}

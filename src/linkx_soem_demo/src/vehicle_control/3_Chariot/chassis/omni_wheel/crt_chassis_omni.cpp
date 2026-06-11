#include "crt_chassis_omni.h"

/**
 * @brief 全向轮底盘硬件初始化
 *
 * 4 个 DM3519 分布在 LinkX channel 0/1（新车头为原车尾）：
 *   channel 0: 原 ID 0x02 保持 0x02 (Rx 0x12)，原 ID 0x03 改为 0x01 (Rx 0x11)
 *   channel 1: 原 ID 0x01 改为 0x02 (Rx 0x12)，原 ID 0x04 改为 0x01 (Rx 0x11)
 *   CAN-FD 1M/5M + MIT 模式：PMAX=12.5 rad（实际本算法不用位置环），VMAX=395 rad/s，
 *             TMAX=5 N·m，IMAX=12 A（与电机出厂上电打印一致）
 */
void Class_Chassis_Omni::Init(linkx_t *__LinkX_Handler)
{
    Motor_Wheel[0].Init(__LinkX_Handler, 1, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 395.0f, 7.8f, 9.2f);
    Motor_Wheel[1].Init(__LinkX_Handler, 0, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 395.0f, 7.8f, 9.2f);
    Motor_Wheel[2].Init(__LinkX_Handler, 0, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 395.0f, 7.8f, 9.2f);
    Motor_Wheel[3].Init(__LinkX_Handler, 1, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 395.0f, 7.8f, 9.2f);

    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        Motor_Wheel[i].Set_Use_FDCAN(true);
        Motor_Wheel[i].Set_Force_Output_Without_Feedback(false);
    }
}

/**
 * @brief 全向轮参数初始化（默认值，可在外部覆盖）
 */
void Class_Chassis_Omni::Init_Motor_Params()
{
    wheel_params_[0] = {
        .wheel_kp                = 0.0f,
        .wheel_kd                = 3.5f,
        .wheel_direction         = 1.0f,
        .wheel_speed_correction  = 0.849f,
        .wheel_omega_deadzone    = 0.05f,
        .wheel_stiction_torque   = 0.1300f,
        .wheel_dynamic_friction  = 0.2960f,
        .wheel_rotor_inertia     = 0.017600f,
        .wheel_feedforward_scale = 0.84f,
        .wheel_accel_limit       = OMNI_WHEEL_ACCEL_LIMIT_RAD_S2,
        .wheel_decel_limit       = OMNI_WHEEL_DECEL_LIMIT_RAD_S2,
    };

    wheel_params_[1] = {
        .wheel_kp                = 0.0f,
        .wheel_kd                = 0.7f,
        .wheel_direction         = 1.0f,
        .wheel_speed_correction  = 1.275f,
        .wheel_omega_deadzone    = 0.05f,
        .wheel_stiction_torque   = 0.2400f,
        .wheel_dynamic_friction  = 0.3962f,
        .wheel_rotor_inertia     = 0.011424f,
        .wheel_feedforward_scale = 0.38f,
        .wheel_accel_limit       = OMNI_WHEEL_ACCEL_LIMIT_RAD_S2,
        .wheel_decel_limit       = OMNI_WHEEL_DECEL_LIMIT_RAD_S2,
    };

    wheel_params_[2] = {
        .wheel_kp                = 0.0f,
        .wheel_kd                = 0.5f,
        .wheel_direction         = 1.0f,
        .wheel_speed_correction  = 1.147f,
        .wheel_omega_deadzone    = 0.05f,
        .wheel_stiction_torque   = 0.4800f,
        .wheel_dynamic_friction  = 0.4301f,
        .wheel_rotor_inertia     = 0.013110f,
        .wheel_feedforward_scale = 0.77f,
        .wheel_accel_limit       = OMNI_WHEEL_ACCEL_LIMIT_RAD_S2,
        .wheel_decel_limit       = OMNI_WHEEL_DECEL_LIMIT_RAD_S2,
    };

    wheel_params_[3] = {
        .wheel_kp                = 0.0f,
        .wheel_kd                = 3.5f,
        .wheel_direction         = 1.0f,
        .wheel_speed_correction  = 0.871f,
        .wheel_omega_deadzone    = 0.05f,
        .wheel_stiction_torque   = 0.3000f,
        .wheel_dynamic_friction  = 0.2767f,
        .wheel_rotor_inertia     = 0.014040f,
        .wheel_feedforward_scale = 0.84f,
        .wheel_accel_limit       = OMNI_WHEEL_ACCEL_LIMIT_RAD_S2,
        .wheel_decel_limit       = OMNI_WHEEL_DECEL_LIMIT_RAD_S2,
    };
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
            if (was_enabled_)
            {
                disable_exit_burst_ticks_ = 50;
                was_enabled_ = false;
            }

            for (int i = 0; i < OMNI_WHEEL_NUM; i++)
            {
                Motor_Wheel[i].Set_Control_Status(Motor_DM_Status_DISABLE);
                Motor_Wheel[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                if (disable_exit_burst_ticks_ > 0 ||
                    Motor_Wheel[i].Get_Now_Control_Status() != Motor_DM_Control_Status_DISABLE)
                {
                    Motor_Wheel[i].CAN_Send_Exit();
                }
                Raw_Target_Wheel_Omega[i] = 0.0f;
                Target_Wheel_Omega[i] = 0.0f;
            }
            if (disable_exit_burst_ticks_ > 0)
                disable_exit_burst_ticks_--;

            Target_Velocity_X = 0.0f;
            Target_Velocity_Y = 0.0f;
            Target_Omega      = 0.0f;

            _Reset_State();
            return;
        }

        case Chassis_Omni_Control_Type_ENABLE:
        {
            was_enabled_ = true;
            disable_exit_burst_ticks_ = 0;

            Apply_Chassis_Trapezoid_Profile();
            Kinematics_Inverse_Resolution();
            Apply_Wheel_Trapezoid_Profile();
            Output_To_Motor();
            break;
        }
    }
}

/**
 * @brief 里程计正解
 *
 * 数学推导（4 轮 X-布局，全向轮，对称安装，坐标系按新车头定义）：
 *   每个轮子在底盘系下的滚动单位方向 t_i = (-sinθ_i, cosθ_i)
 *   轮的线速度 v_i = v_chassis · t_i + ω · L
 *                 = -v_x sinθ_i + v_y cosθ_i + ω · L
 *
 *   对称性： Σsinθ_i = 0, Σcosθ_i = 0, Σsinθ_i cosθ_i = 0,
 *           Σsin²θ_i = 2,  Σcos²θ_i = 2  (4 轮等间隔)
 *
 *   反解（最小二乘）：
 *     v_x =  Σ v_i · (-sinθ_i) / 2
 *     v_y =  Σ v_i ·  cosθ_i  / 2
 *     ω   =  Σ v_i / (4L)
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
 * @brief 底盘速度矢量同步斜坡
 *
 * 轮速 profile 会限制高轮速目标，但低速小命令的目标轮速只有几 rad/s，
 * 单靠轮速限幅会在十几毫秒内完成启动。这里先限制底盘 vx/vy/omega。
 * 平移矢量整体限幅，保持方向；松杆、减速、反向使用更高限幅以保证遥控跟手。
 */
void Class_Chassis_Omni::Apply_Chassis_Trapezoid_Profile()
{
    constexpr float kDt = OMNI_WHEEL_PROFILE_DT;

    const float delta_vx = Target_Velocity_X - Ramped_Velocity_X;
    const float delta_vy = Target_Velocity_Y - Ramped_Velocity_Y;
    const float delta_v = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
    const float current_v = sqrtf(Ramped_Velocity_X * Ramped_Velocity_X +
                                  Ramped_Velocity_Y * Ramped_Velocity_Y);
    const float target_v = sqrtf(Target_Velocity_X * Target_Velocity_X +
                                 Target_Velocity_Y * Target_Velocity_Y);
    const float dot_v = Ramped_Velocity_X * Target_Velocity_X +
                        Ramped_Velocity_Y * Target_Velocity_Y;
    const bool stopping_or_slowing = target_v < current_v;
    const bool changing_direction =
        current_v > 0.05f &&
        target_v > 0.05f &&
        dot_v < 0.5f * current_v * target_v;
    const float linear_limit =
        (stopping_or_slowing || changing_direction) ?
        OMNI_CHASSIS_LINEAR_DECEL_LIMIT_M_S2 :
        OMNI_CHASSIS_LINEAR_ACCEL_LIMIT_M_S2;
    const float max_linear_step = linear_limit * kDt;

    if (delta_v <= max_linear_step || delta_v < 1e-6f)
    {
        Ramped_Velocity_X = Target_Velocity_X;
        Ramped_Velocity_Y = Target_Velocity_Y;
    }
    else
    {
        const float scale = max_linear_step / delta_v;
        Ramped_Velocity_X += delta_vx * scale;
        Ramped_Velocity_Y += delta_vy * scale;
    }

    const float delta_omega = Target_Omega - Ramped_Omega;
    const bool omega_slowing =
        fabsf(Target_Omega) < fabsf(Ramped_Omega) ||
        (Target_Omega * Ramped_Omega < 0.0f);
    const float omega_limit =
        omega_slowing ?
        OMNI_CHASSIS_ANG_DECEL_LIMIT_RAD_S2 :
        OMNI_CHASSIS_ANG_ACCEL_LIMIT_RAD_S2;
    const float max_omega_step = omega_limit * kDt;
    if (fabsf(delta_omega) <= max_omega_step)
    {
        Ramped_Omega = Target_Omega;
    }
    else
    {
        Ramped_Omega += (delta_omega > 0.0f) ? max_omega_step : -max_omega_step;
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
 *   3) 若超过实测可靠轮速上限，按比例缩放（保持矢量方向）
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

        Raw_Target_Wheel_Omega[i] = (v_wheel / Wheel_Radius) *
                                    wheel_params_[i].wheel_direction *
                                    wheel_params_[i].wheel_speed_correction;

        if (fabsf(Raw_Target_Wheel_Omega[i]) > max_wheel_omega)
            max_wheel_omega = fabsf(Raw_Target_Wheel_Omega[i]);
    }

    /* 3. 轮速归一化（保持矢量方向不变） */
    if (max_wheel_omega > MAX_WHEEL_OMEGA && max_wheel_omega > 1e-4f)
    {
        float scale = MAX_WHEEL_OMEGA / max_wheel_omega;
        for (int i = 0; i < OMNI_WHEEL_NUM; i++)
            Raw_Target_Wheel_Omega[i] *= scale;
    }

    /* 4. 死区处理 */
    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        if (fabsf(Raw_Target_Wheel_Omega[i]) < wheel_params_[i].wheel_omega_deadzone)
            Raw_Target_Wheel_Omega[i] = 0.0f;
    }
}

/**
 * @brief 轮速向量同步梯形加减速限幅
 *
 * 逆解得到的是最终目标轮速 Raw_Target_Wheel_Omega。
 * 这里把它转换成每 2ms 更新一次的梯形轮速 Target_Wheel_Omega。
 * 四个轮子使用同一个进度比例从当前轮速向目标轮速靠近，避免启动阶段
 * 轮速比例被独立限幅破坏，导致底盘姿态先偏移再恢复。
 *   - 同向提速使用 wheel_accel_limit；
 *   - 降速、停车、换向使用 wheel_decel_limit；
 *   - Wheel_Command_Accel 作为惯量前馈的加速度来源。
 */
void Class_Chassis_Omni::Apply_Wheel_Trapezoid_Profile()
{
    constexpr float kDt = OMNI_WHEEL_PROFILE_DT;

    float progress = 1.0f;
    bool has_delta = false;

    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        const float current = Target_Wheel_Omega[i];
        const float target = Raw_Target_Wheel_Omega[i];
        const float delta = target - current;
        const float abs_delta = fabsf(delta);
        if (abs_delta < 1e-5f)
            continue;

        const bool changing_direction =
            (current * target < 0.0f) &&
            (fabsf(current) > wheel_params_[i].wheel_omega_deadzone);
        const bool reducing_speed =
            fabsf(target) < fabsf(current) ||
            fabsf(target) < wheel_params_[i].wheel_omega_deadzone;

        const float accel_limit =
            (changing_direction || reducing_speed) ?
            wheel_params_[i].wheel_decel_limit :
            wheel_params_[i].wheel_accel_limit;
        const float max_step = accel_limit * kDt;
        const float wheel_progress = max_step / abs_delta;

        if (wheel_progress < progress)
            progress = wheel_progress;
        has_delta = true;
    }

    if (!has_delta)
    {
        for (int i = 0; i < OMNI_WHEEL_NUM; i++)
        {
            Last_Target_Wheel_Omega[i] = Target_Wheel_Omega[i];
            Wheel_Command_Accel[i] = 0.0f;
        }
        return;
    }

    if (progress > 1.0f)
        progress = 1.0f;
    if (progress < 0.0f)
        progress = 0.0f;

    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        const float current = Target_Wheel_Omega[i];
        const float target = Raw_Target_Wheel_Omega[i];
        const float next = current + (target - current) * progress;

        if (target == 0.0f && fabsf(next) < wheel_params_[i].wheel_omega_deadzone)
        {
            Target_Wheel_Omega[i] = 0.0f;
        }
        else
        {
            Target_Wheel_Omega[i] = next;
        }

        Last_Target_Wheel_Omega[i] = current;
        Wheel_Command_Accel[i] = (Target_Wheel_Omega[i] - current) / kDt;
    }
}

/**
 * @brief 下发 MIT 报文
 *
 * MIT 速度模式（无位置环）：
 *   target_pos = 0, target_omega = Target_Wheel_Omega[i],
 *   target_torque = stiction + inertia feedforward, Kp = 0, Kd = wheel_kd
 *
 * 与原舵轮代码 DRIVE 状态下的轮电机配置保持一致：
 *   Set_Control_Maintain_Postion(0.0f, Target_Wheel_Omega[i], torque_ff, 0.0f, kd)
 */
void Class_Chassis_Omni::Output_To_Motor()
{
    for (int i = 0; i < OMNI_WHEEL_NUM; i++)
    {
        const float target_omega = Target_Wheel_Omega[i];
        const float deadzone = wheel_params_[i].wheel_omega_deadzone;

        float stiction_ff = 0.0f;
        if (fabsf(target_omega) > deadzone)
        {
            const float abs_omega = fabsf(target_omega);
            float dynamic_blend = abs_omega / 2.0f;
            if (dynamic_blend > 1.0f) dynamic_blend = 1.0f;
            const float friction_torque =
                wheel_params_[i].wheel_stiction_torque * (1.0f - dynamic_blend) +
                wheel_params_[i].wheel_dynamic_friction * dynamic_blend;
            stiction_ff = wheel_params_[i].wheel_feedforward_scale *
                          friction_torque *
                          tanhf(target_omega);
        }

        Wheel_Accel_Filtered[i] = OMNI_WHEEL_ACCEL_FILTER_ALPHA * Wheel_Command_Accel[i] +
                                  (1.0f - OMNI_WHEEL_ACCEL_FILTER_ALPHA) * Wheel_Accel_Filtered[i];

        float torque_ff = stiction_ff +
                          wheel_params_[i].wheel_feedforward_scale *
                          wheel_params_[i].wheel_rotor_inertia *
                          Wheel_Accel_Filtered[i];
        if (torque_ff >  OMNI_WHEEL_TORQUE_FF_LIMIT_NM) torque_ff =  OMNI_WHEEL_TORQUE_FF_LIMIT_NM;
        if (torque_ff < -OMNI_WHEEL_TORQUE_FF_LIMIT_NM) torque_ff = -OMNI_WHEEL_TORQUE_FF_LIMIT_NM;

        Motor_Wheel[i].Set_Control_Maintain_Postion(
            0.0f,
            target_omega,
            torque_ff,
            wheel_params_[i].wheel_kp,
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
    {
        Raw_Target_Wheel_Omega[i] = 0.0f;
        Target_Wheel_Omega[i] = 0.0f;
        Last_Target_Wheel_Omega[i] = 0.0f;
        Wheel_Command_Accel[i] = 0.0f;
        Wheel_Accel_Filtered[i] = 0.0f;
    }
}

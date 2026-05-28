#include "crt_chassis.h"

/**
 * @brief 初始化硬件
 *
 * 上位机版本：所有 CAN 帧通过 EtherCAT-CAN 桥（LinkX-4C）转发，
 *   - channel 0：舵向电机所在 CAN 总线
 *   - channel 1：轮向电机所在 CAN 总线
 */
void Class_Chassis::Init(linkx_t *__LinkX_Handler)
{
    //舵向电机初始化（LinkX channel 0）
    Motor_Steer[0].Init(__LinkX_Handler, 0, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 125.0f, 150.0f, 5.0f, 20.0f);
    Motor_Steer[1].Init(__LinkX_Handler, 0, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_MIT, 125.0f, 150.0f, 5.0f, 20.0f);
    Motor_Steer[2].Init(__LinkX_Handler, 0, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 125.0f, 150.0f, 5.0f, 20.0f);
    Motor_Steer[3].Init(__LinkX_Handler, 0, 0x14, 0x04, Motor_DM_Control_Method_NORMAL_MIT, 125.0f, 150.0f, 5.0f, 20.0f);

    //轮向电机初始化（LinkX channel 1）
    Motor_Wheel[0].Init(__LinkX_Handler, 1, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Wheel[1].Init(__LinkX_Handler, 1, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Wheel[2].Init(__LinkX_Handler, 1, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Wheel[3].Init(__LinkX_Handler, 1, 0x14, 0x04, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
}

/**
 * @brief 初始化参数
 */
void Class_Chassis::Init_Motor_Params()
{
    {
        steer_wheel_params_[0] = {
            .steer_kp = 10.0f, // 舵向角速度P增益
            .steer_kd = 0.14f, // 舵向角速度D增益

            .steer_friction_torque = 0.0f, // 替换为你实际测量的 N·m 值
            .steer_torque_deadzone = 0.03f,// 约 1.7°，防止在目标点附近因固定力矩抽搐

            .wheel_omega_deadzone = 0.05f, // 轮向最大角速度
            .wheel_feedforward = 0.5, // 轮向前馈系数

            .wheel_direction = 1, // 轮向方向(+1或-1)

            // 动力学参数（暂时保持为0，等效纯速度控制）
            .wheel_coulomb_Tc   = 0.0f,
            .wheel_viscous_B    = 0.0f,
            .wheel_rotor_inertia = 0.0f,
            .steer_gyro_coeff   = 0.0f,
            .steer_rotor_inertia = 0.0f,

            .flip_speed_threshold = 0.5f, // 翻轮速度阈值
            .flip_drive_scale = 0.4, // 翻轮速度缩放

            .wheel_speed_correction = 1.0f   // 调参：直行偏转时微调（0.90~1.10）
        };


        steer_wheel_params_[1] = {
            .steer_kp = 10.0f, // 舵向角速度P增益
            .steer_kd = 0.20f, // 舵向角速度D增益

            .steer_friction_torque = 0.0f, // 替换为你实际测量的 N·m 值
            .steer_torque_deadzone = 0.03f,// 约 1.7°，防止在目标点附近因固定力矩抽搐

            .wheel_omega_deadzone = 0.05f, // 轮向最大角速度
            .wheel_feedforward = 0.5, // 轮向前馈系数

            .wheel_direction = 1, // 轮向方向(+1或-1)

            // 动力学参数（暂时保持为0，等效纯速度控制）
            .wheel_coulomb_Tc   = 0.0f,
            .wheel_viscous_B    = 0.0f,
            .wheel_rotor_inertia = 0.0f,
            .steer_gyro_coeff   = 0.0f,
            .steer_rotor_inertia = 0.0f,

            .flip_speed_threshold = 0.5f, // 翻轮速度阈值
            .flip_drive_scale = 0.4, // 翻轮速度缩放

            .wheel_speed_correction = 1.0f   // 调参：直行偏转时微调（0.90~1.10）
        };

        steer_wheel_params_[2] = {
            .steer_kp = 10.0f, // 舵向角速度P增益
            .steer_kd = 0.14f, // 舵向角速度D增益

            .steer_friction_torque = 0.0f, // 替换为你实际测量的 N·m 值
            .steer_torque_deadzone = 0.03f,// 约 1.7°，防止在目标点附近因固定力矩抽搐

            .wheel_omega_deadzone = 0.05f, // 轮向最大角速度
            .wheel_feedforward = 0.5, // 轮向前馈系数

            .wheel_direction = 1, // 轮向方向(+1或-1)

             // 动力学参数（暂时保持为0，等效纯速度控制）
            .wheel_coulomb_Tc   = 0.0f,
            .wheel_viscous_B    = 0.0f,
            .wheel_rotor_inertia = 0.0f,
            .steer_gyro_coeff   = 0.0f,
            .steer_rotor_inertia = 0.0f,

            .flip_speed_threshold = 0.5f, // 翻轮速度阈值
            .flip_drive_scale = 0.4, // 翻轮速度缩放

            .wheel_speed_correction = 1.0f   // 调参：直行偏转时微调（0.90~1.10）
        };

        steer_wheel_params_[3] = {
            .steer_kp = 10.0f, // 舵向角速度P增益
            .steer_kd = 0.16f, // 舵向角速度D增益

            .steer_friction_torque = 0.0f, // 替换为你实际测量的 N·m 值
            .steer_torque_deadzone = 0.03f,// 约 1.7°，防止在目标点附近因固定力矩抽搐

            .wheel_omega_deadzone = 0.05f, // 轮向最大角速度
            .wheel_feedforward = 0.5, // 轮向前馈系数

            .wheel_direction = 1, // 轮向方向(+1或-1)

             // 动力学参数（暂时保持为0，等效纯速度控制）
            .wheel_coulomb_Tc   = 0.0f,
            .wheel_viscous_B    = 0.0f,
            .wheel_rotor_inertia = 0.0f,
            .steer_gyro_coeff   = 0.0f,
            .steer_rotor_inertia = 0.0f,

            .flip_speed_threshold = 0.5f, // 翻轮速度阈值
            .flip_drive_scale = 0.4, // 翻轮速度缩放

            .wheel_speed_correction = 1.0f   // 调参：直行偏转时微调（0.90~1.10）
        };
    }
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < STEER_NUM; i++)
    {
        Motor_Steer[i].TIM_Alive_PeriodElapsedCallback();
        Motor_Wheel[i].TIM_Alive_PeriodElapsedCallback();
    }

    if (Chassis_Control_Type == Chassis_Control_Type_ENABLE)
    {
        for (int i = 0; i < STEER_NUM; i++)
        {
            // --- 达妙电机检查 (保持不变) ---
            if (Motor_Steer[i].Get_Status() != Motor_DM_Status_ENABLE)
            {
                Motor_Steer[i].CAN_Send_Clear_Error();
                Motor_Steer[i].CAN_Send_Enter();
            }

            if (Motor_Wheel[i].Get_Status() != Motor_DM_Status_ENABLE)
            {
                Motor_Wheel[i].CAN_Send_Clear_Error();
                Motor_Wheel[i].CAN_Send_Enter();
            }
        }
    }
}

/**
 * @brief TIM定时器中断自身姿态，速度解算回调函数
 *
 */
void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback()
{
    Self_Resolution();
}

/**
 * @brief TIM定时器中断底盘控制回调函数
 *
 */
void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback()
{
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
        {
            // --- 失能状态处理 ---
            for (int i = 0; i < STEER_NUM; i++)
            {
                /* -------- 舵向 G6220 -------- */
                Motor_Steer[i].Set_Control_Status(Motor_DM_Status_DISABLE);
                Motor_Steer[i].Set_Control_Parameter_MIT(0.0f, 0.0f);
                if (Motor_Steer[i].Get_Now_Control_Status() != Motor_DM_Control_Status_DISABLE)
                {
                    Motor_Steer[i].CAN_Send_Exit();
                }

                /* ---------- 轮向 DM3519 ---------- */
                Motor_Wheel[i].Set_Control_Status(Motor_DM_Status_DISABLE);
                Motor_Wheel[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                // 【修复点】：原来这里写错了，应该是 Motor_Wheel 发送 Exit
                if (Motor_Wheel[i].Get_Now_Control_Status() != Motor_DM_Control_Status_DISABLE)
                {
                    Motor_Wheel[i].CAN_Send_Exit();
                }

                Target_Wheel_Omega[i] = 0.0f;
                Target_Wheel_torque[i] = 0.0f;
            } // 【修复点】：正常的 for 循环必须在这里闭合！

            // 清除解算过程量，防止切回使能瞬间猛冲
            Target_Velocity_X = 0.0f;
            Target_Velocity_Y = 0.0f;
            Target_Omega = 0.0f;


            _Reset_Dynamics_State();
            // 【修复点】：return 移出循环体，确保 3 个轮组都能被正确清理
            return;
        }

        case (Chassis_Control_Type_ENABLE):
        {
                Kinematics_Inverse_Resolution();

                // Output_To_Dynamics();

                // Dynamics_Inverse_Resolution();

                Output_To_Motor();
        }
            break;
    }
}

/**
 * @brief 获取当前舵向累积角度（rad），不归一化
 *
 */
float Class_Chassis::Get_Now_Steer_Radian(int index)
{
    return Motor_Steer[index].Get_Now_Radian() / REDUCTION_RATIO;
}

/**
 * @brief 将舵向"累积目标弧度"转换为 DM 电机 MIT 位置指令
 *
 */
float Class_Chassis::Steer_To_Motor_Position(float target_accumulated_rad, int index)
{
    float motor_cmd = target_accumulated_rad * REDUCTION_RATIO;

    float rmax = Motor_Steer[index].Get_Radian_Max();
    if (motor_cmd >  rmax) motor_cmd =  rmax;
    if (motor_cmd < -rmax) motor_cmd = -rmax;

    return motor_cmd;
}

/**
 * @brief 状态清零
 */
void Class_Chassis::_Reset_Dynamics_State()
{
    // 加速度估算状态清零
    prev_target_vx        = 0.0f;
    prev_target_vy        = 0.0f;
    prev_target_omega_dyn = 0.0f;
    chassis_acc_x         = 0.0f;
    chassis_acc_y         = 0.0f;
    chassis_alpha         = 0.0f;

    // 逐轮状态清零
    for (int i = 0; i < STEER_NUM; i++)
    {
        wheel_acc_filtered[i]      = 0.0f;
        steer_vel_filtered[i]      = 0.0f;
        prev_target_wheel_omega[i] = 0.0f;
        prev_target_steer[i]       = Target_Steer_Rad[i];  // 保留舵向位置，速度清零
        contact_force[i]           = 0.0f;
        contact_force_filtered[i]  = 0.0f;
        Target_Wheel_Omega[i]      = 0.0f;
        Target_Wheel_torque[i]     = 0.0f;
        steer_ref_filtered[i]      = 0.0f; // 清空滤波值
        steer_filter_init[i]       = 0;    // 清除初始化标志 (核心)
    }
}

/**
 * @brief 自身姿态，速度解算
 *
 */
void Class_Chassis::Self_Resolution()
{
    Now_Velocity_X = 0.0f;
    Now_Velocity_Y = 0.0f;
    Now_Omega      = 0.0f;

    for (int i = 0; i < STEER_NUM; i++)
    {
        float wheel_speed = Motor_Wheel[i].Get_Now_Omega() *
                            Wheel_Radius *
                            steer_wheel_params_[i].wheel_direction;

        /* 折叠到 [-π,π] 仅供 sin/cos 使用，不影响累积值 */
        float angle = Get_Now_Steer_Radian(i);
        angle = fmodf(angle, 2.0f * (float)M_PI);
        if
            (angle >  (float)M_PI) angle -= 2.0f * (float)M_PI;
        else if
            (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;

        Now_Velocity_X += wheel_speed * cosf(angle) / (float)STEER_NUM;
        Now_Velocity_Y += wheel_speed * sinf(angle) / (float)STEER_NUM;
        Now_Omega      += (wheel_speed * sinf(angle - Wheel_Azimuth[i])
                           / Wheel_To_Core_Distance[i]) / (float)STEER_NUM;
    }
}

/**
 * @brief 运动学逆解算
 *
 * 将底盘目标速度（Target_Velocity_X/Y, Target_Omega）分解到各轮：
 *   第一步：计算每轮的速度矢量 → atan2 得到绝对方向角，存入 Target_Steer_Rad[]
 *   第二步：调用 _Steer_Motor_Kinematics_Nearest_Transposition()
 *           将 Target_Steer_Rad[] 从"绝对方向角"转换为"累积目标角"
 */
void Class_Chassis::Kinematics_Inverse_Resolution()
{
    /* 1. 整车限速 */
    float chassis_speed = sqrtf(Target_Velocity_X * Target_Velocity_X +
                                Target_Velocity_Y * Target_Velocity_Y);

    if (chassis_speed > MAX_CHASSIS_SPEED)
    {
        float scale = MAX_CHASSIS_SPEED / chassis_speed;
        Target_Velocity_X *= scale;
        Target_Velocity_Y *= scale;
    }

    float max_wheel_omega = 0.0f;

    /* 2. 逐轮分解 */
    for (int i = 0; i < STEER_NUM; i++)
    {
        float sin_theta = sinf(Wheel_Azimuth[i]);
        float cos_theta = cosf(Wheel_Azimuth[i]);

        float tmp_velocity_x = Target_Velocity_X - Target_Omega * Wheel_To_Core_Distance[i] * sin_theta;
        float tmp_velocity_y = Target_Velocity_Y + Target_Omega * Wheel_To_Core_Distance[i] * cos_theta;

        float v_mod = sqrtf(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y);

        if (v_mod < 1e-4f)
        {
            /* 速度极低：轮停，舵向目标保持上一帧累积值不更新 */
            Target_Wheel_Omega[i] = 0.0f;
            continue;
        }

        /* atan2 结果为绝对方向角 [-π,π]，后续由就近转位转换为累积目标 */
        Target_Steer_Rad[i]   = atan2f(tmp_velocity_y, tmp_velocity_x);
        Target_Wheel_Omega[i] = (v_mod / Wheel_Radius) *
                                steer_wheel_params_[i].wheel_direction *
                                steer_wheel_params_[i].wheel_speed_correction;

        if (fabsf(Target_Wheel_Omega[i]) > max_wheel_omega)
            max_wheel_omega = fabsf(Target_Wheel_Omega[i]);
    }

    /* 3. 轮速归一化 */
    const float MAX_WHEEL_OMEGA = MAX_CHASSIS_SPEED / Wheel_Radius;
    if (max_wheel_omega > MAX_WHEEL_OMEGA && max_wheel_omega > 1e-4f)
    {
        float scale = MAX_WHEEL_OMEGA / max_wheel_omega;
        for (int i = 0; i < STEER_NUM; i++)
            Target_Wheel_Omega[i] *= scale;
    }

    /* 4. 就近转位：将 Target_Steer_Rad[] 转为累积目标值 */
    _Steer_Motor_Kinematics_Nearest_Transposition();
}

/**
 * @brief 就近转位（Nearest Transposition）
 */
void Class_Chassis::_Steer_Motor_Kinematics_Nearest_Transposition()
{
    for (int i = 0; i < STEER_NUM; i++)
    {
        /* 轮速极低：冻结舵向，目标设为当前位置 */
        if (fabsf(Target_Wheel_Omega[i]) < steer_wheel_params_[i].wheel_omega_deadzone)
        {
            // 速度极低时，目标角度锁死在当前滤波后的平滑位置
            Target_Steer_Rad[i]  = Last_Target_Steer_Rad[i];
            Target_Wheel_Omega[i] = 0.0f;
            wheel_speed_scale[i] = 1.0f;
            Target_Steer_torque[i] = 0.0f;
            continue;
        }

        /* 当前累积角 */
        float current_reference = Last_Target_Steer_Rad[i];

        /* Target_Steer_Rad[i] 此时是 atan2 的绝对方向角 [-π, π] */
        float abs_dir = Target_Steer_Rad[i];

        /*
         * 将 abs_dir 展开到与 current_accumulated 最近的等效角。
         * delta ∈ [-π, π]（最短圆弧路径）。
         * 使用 floor 折叠公式，比 if-else 更精确：
         *   delta -= 2π × floor((delta + π) / 2π)
         */
        float delta = abs_dir - current_reference;
        delta -= 2.0f * (float)M_PI * floorf((delta + (float)M_PI) / (2.0f * (float)M_PI));

        /* 就近翻转：|delta| > π/2 时反转轮子比转舵更近 */
        if (delta > (float)M_PI / 2.0f)
        {
            delta -= (float)M_PI;
            Target_Wheel_Omega[i] = -Target_Wheel_Omega[i];
        }
        else if (delta < -(float)M_PI / 2.0f)
        {
            delta += (float)M_PI;
            Target_Wheel_Omega[i] = -Target_Wheel_Omega[i];
        }
        /* 生成新目标，并覆盖历史记录 */
        Target_Steer_Rad[i] = current_reference + delta;
        Last_Target_Steer_Rad[i] = Target_Steer_Rad[i];

        // 在确定了最终的 delta（实际转动方向）后，判定摩擦力矩方向
        if (delta > steer_wheel_params_[i].steer_torque_deadzone)
        {
            Target_Steer_torque[i] = steer_wheel_params_[i].steer_friction_torque;
        }
        else if (delta < -steer_wheel_params_[i].steer_torque_deadzone)
        {
            Target_Steer_torque[i] = -steer_wheel_params_[i].steer_friction_torque;
        }
        else
        {
            // 在死区范围内（已接近目标点），撤去推力防止反复震荡
            Target_Steer_torque[i] = 0.0f;
        }

        /* 4. 【余弦平滑】：用余弦函数处理未对齐时的轮速，杜绝断崖式抽搐 */
        float cos_scale = cosf(delta);
        wheel_speed_scale[i] = cos_scale;
    }
}

/**
 * @brief 动力学解算
 * @prif 输出转换到动力学状态
 */
void Class_Chassis::Output_To_Dynamics()
{

}

/**
 * @brief 动力学逆解算
 */
void Class_Chassis::Dynamics_Inverse_Resolution()
{
//     const float M  = chassis_physical_params_.mass;
//     const float J  = chassis_physical_params_.inertia;
//     const float dt = 0.002f;
//
//     /* ================================================================
//      * Part A：底盘加速度估算（速度指令差分 + 低通 + 限幅）
//      *
//      * 注：经 Ramp 整形后 Target_Velocity 每帧变化量最大为 DEC_RAMP=0.04 m/s，
//      *   对应 ax_raw = 0.04/0.002 = 20 m/s²，乘以 M=10kg = 200N，
//      *   折算到轮轴 × 0.076m = 15.2 N·m，依然可能偏大。
//      *   硬限幅 ±8 m/s² 进一步保护，确保前馈力矩不超过电机额定。
//      * ================================================================ */
//     const float ALPHA_ACC   = 0.40f;
//     const float MAX_LIN_ACC = 8.0f;    // m/s²
//     const float MAX_ANG_ACC = 40.0f;   // rad/s²
//
//     float ax_raw    = (Target_Velocity_X - prev_target_vx)        / dt;
//     float ay_raw    = (Target_Velocity_Y - prev_target_vy)        / dt;
//     float alpha_raw = (Target_Omega      - prev_target_omega_dyn) / dt;
//
//     // 限幅：仅截断数值噪声峰值，不影响正常加速度范围
//     if (ax_raw    >  MAX_LIN_ACC) ax_raw    =  MAX_LIN_ACC;
//     if (ax_raw    < -MAX_LIN_ACC) ax_raw    = -MAX_LIN_ACC;
//     if (ay_raw    >  MAX_LIN_ACC) ay_raw    =  MAX_LIN_ACC;
//     if (ay_raw    < -MAX_LIN_ACC) ay_raw    = -MAX_LIN_ACC;
//     if (alpha_raw >  MAX_ANG_ACC) alpha_raw =  MAX_ANG_ACC;
//     if (alpha_raw < -MAX_ANG_ACC) alpha_raw = -MAX_ANG_ACC;
//
//     chassis_acc_x = ALPHA_ACC * ax_raw    + (1.0f - ALPHA_ACC) * chassis_acc_x;
//     chassis_acc_y = ALPHA_ACC * ay_raw    + (1.0f - ALPHA_ACC) * chassis_acc_y;
//     chassis_alpha = ALPHA_ACC * alpha_raw + (1.0f - ALPHA_ACC) * chassis_alpha;
//
//     prev_target_vx        = Target_Velocity_X;
//     prev_target_vy        = Target_Velocity_Y;
//     prev_target_omega_dyn = Target_Omega;
//
//     /* ================================================================
//      * Part B：逆动力学求解接触力 F_i（克莱姆法则），并低通滤波
//      *
//      * 接触力低通（ALPHA_FORCE=0.35）：
//      *   消除舵向电机反馈角噪声（约 ±0.01 rad）经 A_mat 矩阵放大后
//      *   引起的逐帧力矩跳变（约 ±3 N·m），抑制中高速行驶的"抖动感"。
//      *
//      * 低速禁用接触力（< 0.15 m/s）：
//      *   低速时加速度本就很小，接触力贡献可忽略，且差分噪声相对更大，
//      *   关闭后退化为纯 kd 速度阻尼，更平稳。
//      * ================================================================ */
//     const float ALPHA_FORCE = 0.35f;
//
//     float A_mat[3][3], b_vec[3], F_vec[3];
//
//     b_vec[0] = M * chassis_acc_x;
//     b_vec[1] = M * chassis_acc_y;
//     b_vec[2] = J * chassis_alpha;
//
//     for (int i = 0; i < STEER_NUM; i++)
//     {
//         float phi = Get_Now_Steer_Radian(i);
//         phi = fmodf(phi, 2.0f * (float)M_PI);
//         if      (phi >  (float)M_PI) phi -= 2.0f * (float)M_PI;
//         else if (phi < -(float)M_PI) phi += 2.0f * (float)M_PI;
//
//         A_mat[0][i] = cosf(phi);
//         A_mat[1][i] = sinf(phi);
//         A_mat[2][i] = Wheel_To_Core_Distance[i] * sinf(phi - Wheel_Azimuth[i]);
//     }
//
//     if (_Solve_3x3(A_mat, b_vec, F_vec))
//     {
//         for (int i = 0; i < STEER_NUM; i++)
//         {
//             float f = F_vec[i];
//             if (f >  60.0f) f =  60.0f;
//             if (f < -60.0f) f = -60.0f;
//             contact_force_filtered[i] = ALPHA_FORCE * f
//                                       + (1.0f - ALPHA_FORCE) * contact_force_filtered[i];
//             contact_force[i] = contact_force_filtered[i];
//         }
//     }
//     else
//     {
//         for (int i = 0; i < STEER_NUM; i++)
//         {
//             contact_force_filtered[i] = 0.0f;
//             contact_force[i]          = 0.0f;
//         }
//     }
//
//     // 低速禁用接触力前馈
//     float eff_speed = sqrtf(Target_Velocity_X * Target_Velocity_X +
//                             Target_Velocity_Y * Target_Velocity_Y);
//     if (eff_speed < 0.15f)
//     {
//         for (int i = 0; i < STEER_NUM; i++)
//         {
//             contact_force_filtered[i] = 0.0f;
//             contact_force[i]          = 0.0f;
//         }
//     }
//
//     /* ================================================================
//      * Part C：轮向力矩合成
//      *   T_total = T_drive (接触力折算) + T_friction (摩擦前馈) + T_inertia (惯性前馈)
//      *   当 Tc/B/Jw = 0 时退化为纯速度控制
//      * ================================================================ */
//     const float ALPHA_WHEEL = 0.25f;
//     const float T_WHEEL_MAX = 4.0f;
//
//     for (int i = 0; i < STEER_NUM; i++)
//     {
//         float v_des = Target_Wheel_Omega[i];
//         float Tc    = steer_wheel_params_[i].wheel_coulomb_Tc;
//         float B     = steer_wheel_params_[i].wheel_viscous_B;
//         float Jw    = steer_wheel_params_[i].wheel_rotor_inertia;
//         float dead  = steer_wheel_params_[i].wheel_omega_deadzone * 2.0f;
//
//         float T_drive = contact_force[i] * Wheel_Radius;
//
//         float T_friction;
//         if      (v_des >  dead)  T_friction =  Tc + B * v_des;
//         else if (v_des < -dead)  T_friction = -Tc + B * v_des;
//         else if (dead  >  1e-6f) T_friction = (Tc / dead + B) * v_des;
//         else                     T_friction = 0.0f;
//
//         float alpha_raw_w = (v_des - prev_target_wheel_omega[i]) / dt;
//         wheel_acc_filtered[i] = ALPHA_WHEEL * alpha_raw_w
//                               + (1.0f - ALPHA_WHEEL) * wheel_acc_filtered[i];
//         prev_target_wheel_omega[i] = v_des;
//
//         float T_ff = T_drive + T_friction + Jw * wheel_acc_filtered[i];
//         if (T_ff >  T_WHEEL_MAX) T_ff =  T_WHEEL_MAX;
//         if (T_ff < -T_WHEEL_MAX) T_ff = -T_WHEEL_MAX;
//
//         Target_Wheel_torque[i] = T_ff;
//     }
//
//     /* ================================================================
//      * Part D：舵向力矩合成（惯量前馈 + 陀螺力矩前馈）
//      * ================================================================ */
//     const float ALPHA_STEER = 0.20f;
//     const float T_STEER_MAX = 5.0f;
//     float omega_chassis = Now_Omega;
//
//     for (int i = 0; i < STEER_NUM; i++)
//     {
//         float Js    = steer_wheel_params_[i].steer_rotor_inertia;
//         float Kgyro = steer_wheel_params_[i].steer_gyro_coeff;
//
//         float steer_vel_raw = (Target_Steer_Rad[i] - prev_target_steer[i]) / dt;
//         steer_vel_filtered[i] = ALPHA_STEER * steer_vel_raw
//                               + (1.0f - ALPHA_STEER) * steer_vel_filtered[i];
//         prev_target_steer[i]  = Target_Steer_Rad[i];
//
//         float omega_wheel_actual = Motor_Wheel[i].Get_Now_Omega();
//         float T_ff = Js * steer_vel_filtered[i] + Kgyro * omega_chassis * omega_wheel_actual;
//
//         if (T_ff >  T_STEER_MAX) T_ff =  T_STEER_MAX;
//         if (T_ff < -T_STEER_MAX) T_ff = -T_STEER_MAX;
//
//         Target_Steer_torque[i] = T_ff;
//     }
}

/**
 * @brief 更新舵轮状态机与轨迹平滑
 * @param i 轮子索引
 */
void Class_Chassis::Update_Steer_State(int i)
{
    float current_accumulated = Get_Now_Steer_Radian(i);
    float delta = Target_Steer_Rad[i] - current_accumulated;

    //  状态机转移逻辑
    switch (steer_state[i])
    {
    case STEER_STATE_IDLE:
        if (fabsf(Target_Wheel_Omega[i]) > steer_wheel_params_[i].wheel_omega_deadzone)
            steer_state[i] = STEER_STATE_ALIGN;
        break;

    case STEER_STATE_ALIGN:
        if (fabsf(delta) < STEER_ALIGN_THRESHOLD)
            steer_state[i] = STEER_STATE_DRIVE;
        break;

    case STEER_STATE_DRIVE:
        if (fabsf(delta) > STEER_ALIGN_THRESHOLD &&
            fabsf(Target_Wheel_Omega[i]) > steer_wheel_params_[i].wheel_omega_deadzone)
        {
            steer_state[i] = STEER_STATE_ALIGN;
            steer_flip_locked[i] = false;
        }
        else if (fabsf(Target_Wheel_Omega[i]) < steer_wheel_params_[i].wheel_omega_deadzone)
        {
            steer_state[i] = STEER_STATE_IDLE;
            steer_flip_locked[i] = false;
        }
        break;

    default:
        steer_state[i] = STEER_STATE_IDLE;
        break;
    }

    // 舵向目标角度斜坡滤波 (解决 MIT 发硬的核心)
    if (steer_filter_init[i] == 0)
    {
        steer_ref_filtered[i] = current_accumulated; // 首次以当前物理角度为起点，防突变
        steer_filter_init[i] = 1;
    }

    // 计算 2ms 控制周期下的最大允许转动步长
    float steer_step = MAX_STEER_OMEGA * 0.002f;

    // 斜坡逼近 (Ramp)
    if (steer_ref_filtered[i] < Target_Steer_Rad[i] - steer_step)
    {
        steer_ref_filtered[i] += steer_step;
    }
    else if (steer_ref_filtered[i] > Target_Steer_Rad[i] + steer_step)
    {
        steer_ref_filtered[i] -= steer_step;
    }
    else
    {
        steer_ref_filtered[i] = Target_Steer_Rad[i];
    }
}

/**
 * @brief 执行电机控制指令下发
 * @param i 轮子索引
 */
void Class_Chassis::Execute_Steer_State(int i)
{
    // 【核心】：这里传入的是平滑滤波后的角度 steer_ref_filtered[i]，而不是突变的 Target_Steer_Rad
    float steer_cmd = Steer_To_Motor_Position(steer_ref_filtered[i], i);
    float motor_torque_ff = Target_Steer_torque[i] / REDUCTION_RATIO;

    switch (steer_state[i])
    {
    case STEER_STATE_IDLE:
        {
            Motor_Steer[i].Set_Control_Maintain_Postion(
                steer_cmd, 0.0f, 0.0f,
                steer_wheel_params_[i].steer_kp,
                steer_wheel_params_[i].steer_kd);

            Motor_Wheel[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            break;
        }

    case STEER_STATE_ALIGN:
        {
            Motor_Steer[i].Set_Control_Maintain_Postion(
                steer_cmd, 0.0f, motor_torque_ff,
                steer_wheel_params_[i].steer_kp,
                steer_wheel_params_[i].steer_kd);

            Motor_Wheel[i].Set_Control_Maintain_Postion(
                0.0f,
                Target_Wheel_Omega[i] * wheel_speed_scale[i], // 使用原本的余弦缩放系数
                0.0f, 0.0f, 5.0f);
            break;
        }

    case STEER_STATE_DRIVE:
        {
            Motor_Steer[i].Set_Control_Maintain_Postion(
                steer_cmd, 0.0f, motor_torque_ff,
                steer_wheel_params_[i].steer_kp,
                steer_wheel_params_[i].steer_kd);

            Motor_Wheel[i].Set_Control_Maintain_Postion(
                0.0f,
                Target_Wheel_Omega[i] * wheel_speed_scale[i],
                Target_Wheel_torque[i], 0.0f, 5.0f);
            break;
        }
    }
}

void Class_Chassis::Output_To_Motor()
{
    for (int i = 0; i < STEER_NUM; i++)
    {
        Update_Steer_State(i);
        Execute_Steer_State(i);
    }

    for (int i = 0; i < STEER_NUM; i++)
    {
        Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
        Motor_Wheel[i].TIM_Send_PeriodElapsedCallback();
    }
}

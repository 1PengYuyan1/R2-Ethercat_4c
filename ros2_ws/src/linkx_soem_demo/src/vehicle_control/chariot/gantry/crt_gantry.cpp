#include "crt_gantry.h"

void Class_Gantry::Init(linkx_t *__LinkX_Handler) {
    // 龙门架抬升电机走 LinkX channel 2
    Motor_Lift_Right.Init(__LinkX_Handler, 2, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 31.4f, 45.0f, 15.0f, 30.0f);
    Motor_Lift_Left.Init( __LinkX_Handler, 2, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_MIT, 31.4f, 45.0f, 15.0f, 30.0f);

    // 应用独立的刚度和阻尼
    Motor_Lift_Right.Set_Control_Torque_P_D_MIT(0.0f, lift_kp_right, lift_kd_right);
    Motor_Lift_Left.Set_Control_Torque_P_D_MIT(0.0f, lift_kp_left, lift_kd_left);

    smooth_lift_left_angle = lift_left_pos1_angle;
    smooth_lift_right_angle = lift_right_pos1_angle;
}

void Class_Gantry::TIM_100ms_Alive_PeriodElapsedCallback() {
    Motor_Lift_Right.TIM_Alive_PeriodElapsedCallback();
    Motor_Lift_Left.TIM_Alive_PeriodElapsedCallback();

    if (Gantry_Control_Type == GANTRY_CONTROL_ENABLE) {
        if (Motor_Lift_Right.Get_Status() != Motor_DM_Control_Status_ENABLE) Motor_Lift_Right.CAN_Send_Enter();
        if (Motor_Lift_Left.Get_Status() != Motor_DM_Control_Status_ENABLE) Motor_Lift_Left.CAN_Send_Enter();
    }
}

void Class_Gantry::TIM_Calculate_PeriodElapsedCallback() {
    switch (Gantry_Control_Type) {
        case GANTRY_CONTROL_DISABLE: {
            Motor_Lift_Right.Set_Control_Status(Motor_DM_Status_DISABLE);
            Motor_Lift_Left.Set_Control_Status(Motor_DM_Status_DISABLE);

            if (Motor_Lift_Right.Get_Now_Control_Status() != Motor_DM_Status_DISABLE) Motor_Lift_Right.CAN_Send_Exit();
            if (Motor_Lift_Left.Get_Now_Control_Status() != Motor_DM_Status_DISABLE) Motor_Lift_Left.CAN_Send_Exit();

            // 掉线重置平滑点 (左右独立读取真实位置)
            smooth_lift_left_angle = Motor_Lift_Left.Get_Now_Radian();
            smooth_lift_right_angle = -Motor_Lift_Right.Get_Now_Radian();
            return;
        }

        case GANTRY_CONTROL_ENABLE: {
            // 获取左右各自的最终目标点
            float final_lift_left_target = (current_lift_state == LIFT_POS1) ? lift_left_pos1_angle : lift_left_pos2_angle;
            float final_lift_right_target = (current_lift_state == LIFT_POS1) ? lift_right_pos1_angle : lift_right_pos2_angle;

            // 速度步长彻底解耦
            float lift_step_left = max_speed_lift_left * 0.002f;
            float lift_step_right = max_speed_lift_right * 0.002f;

            // === 左侧电机斜坡滤波 (使用左侧独立步长) ===
            if (smooth_lift_left_angle < final_lift_left_target - lift_step_left) smooth_lift_left_angle += lift_step_left;
            else if (smooth_lift_left_angle > final_lift_left_target + lift_step_left) smooth_lift_left_angle -= lift_step_left;
            else smooth_lift_left_angle = final_lift_left_target;

            // === 右侧电机斜坡滤波 (使用右侧独立步长) ===
            if (smooth_lift_right_angle < final_lift_right_target - lift_step_right) smooth_lift_right_angle += lift_step_right;
            else if (smooth_lift_right_angle > final_lift_right_target + lift_step_right) smooth_lift_right_angle -= lift_step_right;
            else smooth_lift_right_angle = final_lift_right_target;

            // === 1. 下发右侧电机 (保留负号，实现镜像反转) ===
            if (Motor_Lift_Right.Get_Status() != Motor_DM_Control_Status_ENABLE) {
                Motor_Lift_Right.CAN_Send_Enter();
            } else {
                Motor_Lift_Right.Set_Control_Parameter_MIT(-smooth_lift_right_angle, 0.0f);
            }
            Motor_Lift_Right.TIM_Send_PeriodElapsedCallback();

            // === 2. 下发左侧电机 (正常正向) ===
            if (Motor_Lift_Left.Get_Status() != Motor_DM_Control_Status_ENABLE) {
                Motor_Lift_Left.CAN_Send_Enter();
            } else {
                Motor_Lift_Left.Set_Control_Parameter_MIT(smooth_lift_left_angle, 0.0f);
            }
            Motor_Lift_Left.TIM_Send_PeriodElapsedCallback();

            break;
        }
    }
}

void Class_Gantry::Set_Lift_State(Enum_Gantry_Lift_State state) { current_lift_state = state; }
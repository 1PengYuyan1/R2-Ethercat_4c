#include "crt_arm.h"

void Class_Arm::Init(linkx_t *__LinkX_Handler) {

    // 机械臂电机走 LinkX channel 2（与龙门架同总线）
    Arm.Init(__LinkX_Handler, 2, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 20.0f, 15.0f, 20.0f);

    Arm.Set_Control_Torque_P_D_MIT(0.0f, arm_kp, arm_kd);

    arm_smooth_angle = arm_start_angle;
}

void Class_Arm::TIM_100ms_Alive_PeriodElapsedCallback() {

    Arm.TIM_Alive_PeriodElapsedCallback();

    if (Arm_Control_Type == ARM_CONTROL_ENABLE) {
        if (Arm.Get_Status() != Motor_DM_Control_Status_ENABLE) Arm.CAN_Send_Enter();
    }
}

void Class_Arm::TIM_Calculate_PeriodElapsedCallback() {
    switch (Arm_Control_Type) {
        case ARM_CONTROL_DISABLE: {
            Arm.Set_Control_Status(Motor_DM_Status_DISABLE);

            if (Arm.Get_Now_Control_Status() != Motor_DM_Status_DISABLE) Arm.CAN_Send_Exit();

            // 掉线重置平滑点 (注意这里要减去 offset 转换为相对角度)
            arm_smooth_angle = Arm.Get_Now_Radian() - zero_offset;
            return;
        }

        case ARM_CONTROL_ENABLE: {

                float final_arm_target;
                switch (current_state) {
                case Arm_start:
                    final_arm_target = arm_start_angle;
                    break;
                case Arm_grab:
                    final_arm_target = arm_grab_angle;
                    break;
                case Arm_zero:
                    final_arm_target = arm_zero_angle;
                    break;
                default:
                    final_arm_target = arm_start_angle;
                    break;
                }

            float arm_step = arm_max_speed * 0.002f;

            // 斜坡滤波
            if (arm_smooth_angle < final_arm_target - arm_step) arm_smooth_angle += arm_step;
            else if (arm_smooth_angle > final_arm_target + arm_step) arm_smooth_angle -= arm_step;
            else arm_smooth_angle = final_arm_target;

            // 下发平滑后的目标位置 (加上物理零点偏置)
            if (Arm.Get_Status() != Motor_DM_Control_Status_ENABLE) {
                Arm.CAN_Send_Enter();
            } else {
                Arm.Set_Control_Parameter_MIT(arm_smooth_angle + zero_offset, 0.0f);
            }
            Arm.TIM_Send_PeriodElapsedCallback();

            break;
        }
    }
}

void Class_Arm::Set_Arm_State(Enum_Arm_State state) { current_state = state; }
#ifndef R1_ROBOT_1_CRT_ARM_H
#define R1_ROBOT_1_CRT_ARM_H

#include "dvc_motor_dm.h"

enum Enum_Arm_Control_Type { ARM_CONTROL_DISABLE = 0, ARM_CONTROL_ENABLE };
enum Enum_Arm_State { Arm_start = 0, Arm_grab, Arm_zero };

class Class_Arm {
public:
    Class_Motor_DM_Normal Arm;

    Enum_Arm_Control_Type Arm_Control_Type = ARM_CONTROL_DISABLE;

    void Init(linkx_t *__LinkX_Handler);
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_Calculate_PeriodElapsedCallback();

    void Set_Arm_Control_Type(Enum_Arm_Control_Type type) { Arm_Control_Type = type; }
    void Set_Arm_State(Enum_Arm_State state);

private:
    Enum_Arm_State current_state = Arm_start;

    // === 零点偏置与终点目标 ===
    float zero_offset = 0.0f;
    float arm_start_angle = -0.5f;     // 位置 1 (对应龙门架初始)
    float arm_grab_angle = -1.5875f;  // 位置 2 (对应龙门架夹取)
    float arm_zero_angle = 0.0f;      // 位置 3 (对应龙门架对接)

    float arm_smooth_angle = 0.0f;


    // 默认限速：1.0 rad/s
    float arm_max_speed = 3.0f;


    // === MIT 刚度与阻尼 ===
    float arm_kp = 7.0f;
    float arm_kd = 1.5f;
};

#endif //R1_ROBOT_1_CRT_ARM_H
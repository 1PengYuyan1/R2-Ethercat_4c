#ifndef R1_ROBOT_1_CRT_GANTRY_H
#define R1_ROBOT_1_CRT_GANTRY_H

#include "dvc_motor_dm.h"

enum Enum_Gantry_Control_Type { GANTRY_CONTROL_DISABLE = 0, GANTRY_CONTROL_ENABLE };
enum Enum_Gantry_Lift_State { LIFT_POS1 = 0, LIFT_POS2 };
enum Enum_Gantry_Extend_State { EXTEND_POS1 = 0, EXTEND_POS2 };

class Class_Gantry {
public:
    Class_Motor_DM_Normal Motor_Lift_Right; // 右侧电机 (ID: 0x11)
    Class_Motor_DM_Normal Motor_Lift_Left;  // 左侧电机 (ID: 0x12)

    Enum_Gantry_Control_Type Gantry_Control_Type = GANTRY_CONTROL_DISABLE;

    void Init(linkx_t *__LinkX_Handler);
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_Calculate_PeriodElapsedCallback();

    void Set_Gantry_Control_Type(Enum_Gantry_Control_Type type) { Gantry_Control_Type = type; }
    void Set_Lift_State(Enum_Gantry_Lift_State state);

private:
    Enum_Gantry_Lift_State current_lift_state = LIFT_POS1;

    // ==========================================
    // 🛠️ 调参区 4：龙门架上下限位 (左右独立)
    // ==========================================


    float lift_left_pos1_angle = 0.0f;  // 左侧最低点
    float lift_left_pos2_angle = -12.5f;  // 左侧最高点

    float lift_right_pos1_angle = 0.0f; // 右侧最低点
    float lift_right_pos2_angle = -12.5f; // 右侧最高点

    // ==========================================

    // 🛠️ 调参区 5：运行速度限制 (左右独立)
    // ==========================================
    float smooth_lift_left_angle = 10.0f;
    float smooth_lift_right_angle = 10.0f;

    float max_speed_lift_left = 5.0f;  // 左侧抬升速度
    float max_speed_lift_right =5.0f; // 右侧抬升速度

    // ==========================================
    // 🛠️ 调参区 6：MIT 刚度与阻尼 (左右独立)
    // ==========================================
    float lift_kp_left = 15.0f;
    float lift_kd_left = 1.0f;

    float lift_kp_right = 15.0f;
    float lift_kd_right = 1.0f;
};

#endif //R1_ROBOT_1_CRT_GANTRY_H
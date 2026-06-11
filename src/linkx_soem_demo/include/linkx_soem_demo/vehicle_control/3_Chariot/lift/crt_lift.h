#ifndef CRT_LIFT_H
#define CRT_LIFT_H

#include "dvc_motor_dm.h"

#include <cstdint>

#define CHARIOT_LIFT_MODULE_NUM 2

/* 新车头为原车尾：CAN0 对应车头抬升模块，CAN1 对应车尾抬升模块。 */
enum Enum_Chariot_Lift_Module
{
    CHARIOT_LIFT_MODULE_FRONT = 0,
    CHARIOT_LIFT_MODULE_REAR = 1,
};

enum Enum_Chariot_Lift_Control_Type
{
    CHARIOT_LIFT_CONTROL_DISABLE = 0,
    CHARIOT_LIFT_CONTROL_ENABLE,
};

enum Enum_Chariot_Lift_Position_State
{
    CHARIOT_LIFT_POSITION_RETRACT = 0,
    CHARIOT_LIFT_POSITION_RAISE,
};

struct ChariotLiftDriveParams
{
    float wheel_radius;           // 摩擦轮半径，单位 m
    float track_width;            // 左右轮间距，单位 m
    float max_forward_speed;      // 最大前进速度，单位 m/s
    float max_yaw_omega;          // 最大偏航角速度，单位 rad/s
    float max_wheel_omega;        // 电机轴最大角速度，单位 rad/s
    float wheel_kp;               // MIT 速度环默认使用 kp=0
    float wheel_kd;               // MIT 速度阻尼
    float wheel_deadzone;         // 轮速死区，单位 rad/s
    float left_direction;         // 左轮安装方向修正
    float right_direction;        // 右轮安装方向修正
    float left_speed_correction;
    float right_speed_correction;
    float accel_limit;            // 加速度限制，单位 rad/s^2
    float decel_limit;            // 减速度限制，单位 rad/s^2
};

struct ChariotLiftPositionParams
{
    float retract_angle;          // 抬升杆侧回收角度，单位 rad
    float raise_angle;            // 抬升杆侧抬升角度，单位 rad
    float max_speed;              // 抬升杆侧最大速度，单位 rad/s
    float kp;                     // MIT 位置刚度
    float kd;                     // MIT 位置阻尼
};

class Class_Chariot_Lift
{
public:
    Class_Motor_DM_Normal Motor_Drive_Left[CHARIOT_LIFT_MODULE_NUM];   // 左侧驱动轮电机，CAN ID 0x03
    Class_Motor_DM_Normal Motor_Drive_Right[CHARIOT_LIFT_MODULE_NUM];  // 右侧驱动轮电机，CAN ID 0x04
    Class_Motor_DM_Normal Motor_Lift[CHARIOT_LIFT_MODULE_NUM];         // 抬升 3519 电机，CAN ID 0x05

    void Init(linkx_t *__LinkX_Handler);
    bool CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data);

    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_2ms_Control_PeriodElapsedCallback();
    void Send_Enable_Burst();

    inline void Set_Control_Type(Enum_Chariot_Lift_Control_Type type);
    inline void Set_Front_Lift_State(Enum_Chariot_Lift_Position_State state);
    inline void Set_Rear_Lift_State(Enum_Chariot_Lift_Position_State state);
    inline void Set_Diff_Drive_Enable(bool enable);
    inline void Set_Diff_Drive_Module(Enum_Chariot_Lift_Module module);
    inline void Set_Target_Diff_Command(float forward_m_s, float yaw_rad_s);

    inline Enum_Chariot_Lift_Control_Type Get_Control_Type();
    inline bool Get_Diff_Drive_Enable();
    inline float Get_Target_Diff_Forward();
    inline float Get_Target_Diff_Yaw();
    inline float Get_Target_Left_Omega(Enum_Chariot_Lift_Module module);
    inline float Get_Target_Right_Omega(Enum_Chariot_Lift_Module module);

protected:
    Enum_Chariot_Lift_Control_Type Control_Type = CHARIOT_LIFT_CONTROL_DISABLE;
    Enum_Chariot_Lift_Position_State Lift_State[CHARIOT_LIFT_MODULE_NUM] = {
        CHARIOT_LIFT_POSITION_RETRACT,
        CHARIOT_LIFT_POSITION_RETRACT,
    };
    bool Lift_Module_Enable[CHARIOT_LIFT_MODULE_NUM] = {false, false};

    ChariotLiftDriveParams Drive_Params[CHARIOT_LIFT_MODULE_NUM];
    ChariotLiftPositionParams Lift_Params[CHARIOT_LIFT_MODULE_NUM];

    float Smooth_Lift_Angle[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Raw_Target_Left_Omega[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Raw_Target_Right_Omega[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Target_Left_Omega[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Target_Right_Omega[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};

    bool Diff_Drive_Enable = false;
    Enum_Chariot_Lift_Module Diff_Drive_Module = CHARIOT_LIFT_MODULE_FRONT;
    float Target_Diff_Forward = 0.0f;
    float Target_Diff_Yaw = 0.0f;
    uint16_t drive_disable_exit_burst_ticks_ = 0;
    uint16_t control_disable_exit_burst_ticks_ = 0;
    uint16_t enable_service_tick_ = 0;

    void Init_Motor_Params();
    void Ensure_Motor_Enabled(Class_Motor_DM_Normal &motor, bool clear_disable_state);
    void Ensure_All_Motors_Enabled();
    void Output_Lift_Motor(Enum_Chariot_Lift_Module module);
    void Kinematics_Diff_Resolution();
    void Apply_Drive_Trapezoid_Profile();
    void Output_Drive_Motor(Enum_Chariot_Lift_Module module);
    void Disable_Drive_Motor(Enum_Chariot_Lift_Module module, bool force_exit);
    void Disable_All_Motors(bool force_exit);
    void Reset_Drive_State();

    static float Clamp(float value, float min_value, float max_value);
    static float Ramp_To(float current, float target, float accel_limit, float decel_limit, float dt);
    static float Lift_Rod_To_Motor_Angle(float lift_rod_angle);
    static float Motor_To_Lift_Rod_Angle(float motor_angle);
};

inline void Class_Chariot_Lift::Set_Control_Type(Enum_Chariot_Lift_Control_Type type)
{
    if (Control_Type != type && type == CHARIOT_LIFT_CONTROL_DISABLE)
    {
        control_disable_exit_burst_ticks_ = 50;
        drive_disable_exit_burst_ticks_ = 50;
    }
    else if (Control_Type != type && type == CHARIOT_LIFT_CONTROL_ENABLE)
    {
        enable_service_tick_ = 0;
        for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
            Smooth_Lift_Angle[i] = Motor_To_Lift_Rod_Angle(Motor_Lift[i].Get_Now_Radian());
    }
    Control_Type = type;
}

inline void Class_Chariot_Lift::Set_Front_Lift_State(Enum_Chariot_Lift_Position_State state)
{
    Lift_State[CHARIOT_LIFT_MODULE_FRONT] = state;
    Lift_Module_Enable[CHARIOT_LIFT_MODULE_FRONT] = true;
}

inline void Class_Chariot_Lift::Set_Rear_Lift_State(Enum_Chariot_Lift_Position_State state)
{
    Lift_State[CHARIOT_LIFT_MODULE_REAR] = state;
    Lift_Module_Enable[CHARIOT_LIFT_MODULE_REAR] = true;
}

inline void Class_Chariot_Lift::Set_Diff_Drive_Enable(bool enable)
{
    if (Diff_Drive_Enable && !enable)
        drive_disable_exit_burst_ticks_ = 50;
    Diff_Drive_Enable = enable;
}

inline void Class_Chariot_Lift::Set_Diff_Drive_Module(Enum_Chariot_Lift_Module module)
{
    Diff_Drive_Module = module;
}

inline void Class_Chariot_Lift::Set_Target_Diff_Command(float forward_m_s, float yaw_rad_s)
{
    Target_Diff_Forward = forward_m_s;
    Target_Diff_Yaw = yaw_rad_s;
}

inline Enum_Chariot_Lift_Control_Type Class_Chariot_Lift::Get_Control_Type()
{
    return Control_Type;
}

inline bool Class_Chariot_Lift::Get_Diff_Drive_Enable()
{
    return Diff_Drive_Enable;
}

inline float Class_Chariot_Lift::Get_Target_Diff_Forward()
{
    return Target_Diff_Forward;
}

inline float Class_Chariot_Lift::Get_Target_Diff_Yaw()
{
    return Target_Diff_Yaw;
}

inline float Class_Chariot_Lift::Get_Target_Left_Omega(Enum_Chariot_Lift_Module module)
{
    return Target_Left_Omega[module];
}

inline float Class_Chariot_Lift::Get_Target_Right_Omega(Enum_Chariot_Lift_Module module)
{
    return Target_Right_Omega[module];
}

#endif // CRT_LIFT_H

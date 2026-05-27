#ifndef CRT_CHASSIS_OMNI_H
#define CRT_CHASSIS_OMNI_H

#include "dvc_motor_dm.h"

#include "math.h"
#include <cmath>

extern "C" {
#include "ramp.h"
}

#define OMNI_WHEEL_NUM 4

/* 轮子半径（m）= 152.2mm / 2 */
#define Omni_Wheel_Radius_Define            0.0761f

/* 轮毂中心到底盘旋转中心距离（m）= 246.21mm */
#define Omni_Wheel_To_Core_Distance_Define  0.24621f

/* 整车最大线/角速度 */
#define MAX_OMNI_CHASSIS_SPEED              2.0f
#define MAX_OMNI_CHASSIS_OMEGA              6.0f

/* 速度命令斜坡（单位 m/s/ctrl-tick，2ms 控制周期下 0.01 ≈ 5 m/s²） */
#define OMNI_ACC_LIN_RAMP                   0.01f
#define OMNI_ACC_ANG_RAMP                   0.04f

/**
 * @brief 全向轮单轮参数
 */
struct OmniWheelParams
{
    float wheel_kd;                // MIT 模式速度环 Kd（粘性阻尼，越大跟得越紧、噪声越大）
    float wheel_direction;         // 轮向方向 (+1 或 -1)
    float wheel_speed_correction;  // 直行偏转微调系数（0.90 ~ 1.10）
    float wheel_omega_deadzone;    // 轮向速度死区
};

/**
 * @brief 底盘控制状态
 */
enum Enum_Chassis_Omni_Control_Type
{
    Chassis_Omni_Control_Type_DISABLE = 0,
    Chassis_Omni_Control_Type_ENABLE,
};

/**
 * @brief 全向轮底盘类（4 轮 X-布局）
 *
 * 轮位约定（俯视图，车头 +X，左侧 +Y）：
 *   index 0 → ID 1 → 左前  45°
 *   index 1 → ID 2 → 左后 135°
 *   index 2 → ID 3 → 右后 225°
 *   index 3 → ID 4 → 右前 315°
 *
 * 全部 4 个 DM3519 在 LinkX channel 0，MIT 速度模式（位置 0、Kp 0、Kd 阻尼）。
 */
class Class_Chassis_Omni
{
public:
    Class_Motor_DM_Normal Motor_Wheel[OMNI_WHEEL_NUM];
    OmniWheelParams       wheel_params_[OMNI_WHEEL_NUM];

    void Init(linkx_t *__LinkX_Handler);
    void Init_Motor_Params();

    inline float Get_Now_Velocity_X();
    inline float Get_Now_Velocity_Y();
    inline float Get_Now_Omega();
    inline Enum_Chassis_Omni_Control_Type Get_Chassis_Control_Type();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Omni_Control_Type __type);
    inline void Set_Target_Velocity_X(float __vx);
    inline void Set_Target_Velocity_Y(float __vy);
    inline void Set_Target_Omega(float __omega);

    void TIM_2ms_Resolution_PeriodElapsedCallback();
    void TIM_2ms_Control_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();

protected:
    const float Wheel_Radius           = Omni_Wheel_Radius_Define;
    const float Wheel_To_Core_Distance = Omni_Wheel_To_Core_Distance_Define;

    /* X-布局：45° / 135° / 225° / 315° */
    const float Wheel_Azimuth[OMNI_WHEEL_NUM] = {
        (PI / 4.0f),         // ID 1 左前 45°
        (3.0f * PI / 4.0f),  // ID 2 左后 135°
        (5.0f * PI / 4.0f),  // ID 3 右后 225°
        (7.0f * PI / 4.0f)   // ID 4 右前 315°
    };

    /* DM3519 输出轴最大角速度（rad/s） */
    float MAX_WHEEL_OMEGA = 120.0f;

    float Now_Velocity_X = 0.0f;
    float Now_Velocity_Y = 0.0f;
    float Now_Omega      = 0.0f;

    float Target_Velocity_X = 0.0f;
    float Target_Velocity_Y = 0.0f;
    float Target_Omega      = 0.0f;

    /* 命令斜坡平滑（避免使能瞬间冲击） */
    float Ramped_Velocity_X = 0.0f;
    float Ramped_Velocity_Y = 0.0f;
    float Ramped_Omega      = 0.0f;

    float Target_Wheel_Omega[OMNI_WHEEL_NUM] = {0.0f};

    Enum_Chassis_Omni_Control_Type Chassis_Control_Type = Chassis_Omni_Control_Type_DISABLE;

    void Self_Resolution();
    void Kinematics_Inverse_Resolution();
    void Output_To_Motor();
    void _Reset_State();
};

/* ============================ inline 实现 ============================ */

inline float Class_Chassis_Omni::Get_Now_Velocity_X() { return Now_Velocity_X; }
inline float Class_Chassis_Omni::Get_Now_Velocity_Y() { return Now_Velocity_Y; }
inline float Class_Chassis_Omni::Get_Now_Omega()      { return Now_Omega; }
inline Enum_Chassis_Omni_Control_Type Class_Chassis_Omni::Get_Chassis_Control_Type()
{
    return Chassis_Control_Type;
}
inline float Class_Chassis_Omni::Get_Target_Velocity_X() { return Target_Velocity_X; }
inline float Class_Chassis_Omni::Get_Target_Velocity_Y() { return Target_Velocity_Y; }
inline float Class_Chassis_Omni::Get_Target_Omega()      { return Target_Omega; }

inline void Class_Chassis_Omni::Set_Chassis_Control_Type(Enum_Chassis_Omni_Control_Type __type)
{
    Chassis_Control_Type = __type;
}
inline void Class_Chassis_Omni::Set_Target_Velocity_X(float __vx)    { Target_Velocity_X = __vx; }
inline void Class_Chassis_Omni::Set_Target_Velocity_Y(float __vy)    { Target_Velocity_Y = __vy; }
inline void Class_Chassis_Omni::Set_Target_Omega(float __omega)      { Target_Omega      = __omega; }

#endif // CRT_CHASSIS_OMNI_H

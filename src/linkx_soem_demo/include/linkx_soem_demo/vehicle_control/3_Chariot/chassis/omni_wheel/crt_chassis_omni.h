#ifndef CRT_CHASSIS_OMNI_H
#define CRT_CHASSIS_OMNI_H

#include "dvc_motor_dm.h"

#include "math.h"
#include <cmath>

#define OMNI_WHEEL_NUM 4

/* 轮子半径（m）= 152.2mm / 2 */
#define Omni_Wheel_Radius_Define            0.0761f

/* 轮毂中心到底盘旋转中心距离（m）= 246.21mm */
#define Omni_Wheel_To_Core_Distance_Define  0.24621f

/* 整车最大线/角速度 */
#define MAX_OMNI_CHASSIS_SPEED              5.0f
#define MAX_OMNI_CHASSIS_OMEGA              10.0f

/* 底盘速度矢量斜坡：起步适中，停车/换向更快以保证遥控跟手 */
#define OMNI_CHASSIS_LINEAR_ACCEL_LIMIT_M_S2 8.0f
#define OMNI_CHASSIS_LINEAR_DECEL_LIMIT_M_S2 12.0f
#define OMNI_CHASSIS_ANG_ACCEL_LIMIT_RAD_S2  24.0f
#define OMNI_CHASSIS_ANG_DECEL_LIMIT_RAD_S2  32.0f

/* 轮速梯形加减速参数（由 2026-06-08/09 架空测试结果收敛得到） */
#define OMNI_WHEEL_PROFILE_DT               0.002f
#define OMNI_WHEEL_ACCEL_LIMIT_RAD_S2       400.0f
#define OMNI_WHEEL_DECEL_LIMIT_RAD_S2       650.0f
#define OMNI_WHEEL_ACCEL_FILTER_ALPHA       0.25f
#define OMNI_WHEEL_TORQUE_FF_LIMIT_NM       1.5f
#define OMNI_WHEEL_RELIABLE_OMEGA_LIMIT     80.0f
#define OMNI_WHEEL_BREAKAWAY_OMEGA_RAD_S    2.0f
#define OMNI_WHEEL_BREAKAWAY_RATIO          0.90f

/* W0_RB 速度反馈低报修正（2026-06-25 架空实测：命令正确转 6.28 圈，反馈仅报 4.86 圈，
 * 稳态反馈比 0.780）。仅修正反馈，不动命令；scale = 1/0.780 ≈ 1.282，待固件查清后归 1。 */
#define OMNI_WHEEL_W0_FEEDBACK_OMEGA_SCALE  1.282f

/**
 * @brief 全向轮单轮参数
 */
struct OmniWheelParams
{
    float wheel_kp;                // MIT 模式位置环 Kp（当前扫参最佳为 0）
    float wheel_kd;                // MIT 模式速度环 Kd（粘性阻尼，越大跟得越紧、噪声越大）
    float wheel_direction;         // 轮向方向 (+1 或 -1)
    float wheel_speed_correction;  // 直行偏转微调系数（0.90 ~ 1.10）
    float wheel_omega_deadzone;    // 轮向速度死区
    float wheel_stiction_torque;   // 起动/静摩擦补偿力矩 (N·m)
    float wheel_dynamic_friction;  // 中低速动摩擦补偿力矩 (N·m)
    float wheel_rotor_inertia;     // 轮向等效转动惯量 (kg·m²)
    float wheel_feedforward_scale; // 静摩擦/惯量前馈比例
    float wheel_breakaway_torque;  // 地面低速起步附加力矩 (N·m)
    float wheel_accel_limit;       // 轮速梯形加速上限 (rad/s²)
    float wheel_decel_limit;       // 轮速梯形减速/换向上限 (rad/s²)
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
 * 轮位约定（俯视图，新车头 +X，左侧 +Y；新车头为原车尾）：
 *   index 0 → CAN1 ID 2 → 右后 225°（原左前）
 *   index 1 → CAN0 ID 2 → 右前 315°（原左后）
 *   index 2 → CAN0 ID 1 → 左前  45°（原右后）
 *   index 3 → CAN1 ID 1 → 左后 135°（原右前）
 *
 * 4 个 DM3519 分布在 LinkX channel 0/1，MIT 速度模式（位置 0、Kp 0、Kd 阻尼）。
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
    inline float Get_Profiled_Target_Velocity_X();
    inline float Get_Profiled_Target_Velocity_Y();
    inline float Get_Profiled_Target_Omega();
    inline float Get_Raw_Target_Wheel_Omega(int index);
    inline float Get_Target_Wheel_Omega(int index);
    inline float Get_Wheel_Command_Accel(int index);
    inline float Get_Wheel_Accel_Filtered(int index);

    inline void Set_Chassis_Control_Type(Enum_Chassis_Omni_Control_Type __type);
    inline void Set_Target_Velocity_X(float __vx);
    inline void Set_Target_Velocity_Y(float __vy);
    inline void Set_Target_Omega(float __omega);
    /* 只降速纠偏开关：true 时逆解里 omega（航向纠偏）只许降低各轮幅值、不许提速。
     * 由上层在“航向纠偏激活”时每周期置位；默认 false，不影响用户主动转向。 */
    inline void Set_Yaw_Correction_Slow_Only(bool __enable);

    /* 逐轮软件速度环（补 MIT Kp=0 缺的积分，消除地面负载稳态垂降）。
     * 默认 kp=ki=0 → 完全不改变原行为。i_limit 为积分力矩限幅(N·m)。 */
    void Set_Velocity_Loop(float __kp, float __ki, float __i_limit);
    void Set_Torque_FF_Limit(float __limit_nm);
    inline float Get_Wheel_Vel_Integral(int index);

    void TIM_2ms_Resolution_PeriodElapsedCallback();
    void TIM_2ms_Control_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();

protected:
    const float Wheel_Radius           = Omni_Wheel_Radius_Define;
    const float Wheel_To_Core_Distance = Omni_Wheel_To_Core_Distance_Define;

    /* 新车体系 X-布局：原车体系整体旋转 180° */
    const float Wheel_Azimuth[OMNI_WHEEL_NUM] = {
        (5.0f * PI / 4.0f),  // index 0 右后 225°（原左前）
        (7.0f * PI / 4.0f),  // index 1 右前 315°（原左后）
        (PI / 4.0f),         // index 2 左前 45°（原右后）
        (3.0f * PI / 4.0f)   // index 3 左后 135°（原右前）
    };

    /* 实测可靠轮速上限（rad/s），高于该值 W0/W3 会先进入跟踪不足 */
    float MAX_WHEEL_OMEGA = OMNI_WHEEL_RELIABLE_OMEGA_LIMIT;

    float Now_Velocity_X = 0.0f;
    float Now_Velocity_Y = 0.0f;
    float Now_Omega      = 0.0f;

    float Target_Velocity_X = 0.0f;
    float Target_Velocity_Y = 0.0f;
    float Target_Omega      = 0.0f;

    float Ramped_Velocity_X = 0.0f;
    float Ramped_Velocity_Y = 0.0f;
    float Ramped_Omega      = 0.0f;

    float Raw_Target_Wheel_Omega[OMNI_WHEEL_NUM] = {0.0f};
    float Target_Wheel_Omega[OMNI_WHEEL_NUM] = {0.0f};
    float Last_Target_Wheel_Omega[OMNI_WHEEL_NUM] = {0.0f};
    float Wheel_Command_Accel[OMNI_WHEEL_NUM] = {0.0f};
    float Wheel_Accel_Filtered[OMNI_WHEEL_NUM] = {0.0f};

    /* 软件速度环状态/增益。落地负载下 MIT Kp=0 会稳态欠跟踪；
     * 默认开启低增益积分，和 omni_straight_retest 当前验证配置一致。 */
    float Wheel_Vel_Integral[OMNI_WHEEL_NUM] = {0.0f};
    float Vel_Loop_Kp = 0.0f;
    float Vel_Loop_Ki = 2.0f;
    float Vel_Loop_I_Limit = 3.0f;
    float Torque_FF_Limit = 3.0f;
    bool was_enabled_ = false;
    bool yaw_correction_slow_only_ = false;
    uint16_t disable_exit_burst_ticks_ = 0;

    Enum_Chassis_Omni_Control_Type Chassis_Control_Type = Chassis_Omni_Control_Type_DISABLE;

    void Self_Resolution();
    void Apply_Chassis_Trapezoid_Profile();
    void Kinematics_Inverse_Resolution();
    void Apply_Wheel_Trapezoid_Profile();
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
inline float Class_Chassis_Omni::Get_Profiled_Target_Velocity_X() { return Ramped_Velocity_X; }
inline float Class_Chassis_Omni::Get_Profiled_Target_Velocity_Y() { return Ramped_Velocity_Y; }
inline float Class_Chassis_Omni::Get_Profiled_Target_Omega()      { return Ramped_Omega; }
inline float Class_Chassis_Omni::Get_Raw_Target_Wheel_Omega(int index)
{
    return (index >= 0 && index < OMNI_WHEEL_NUM) ? Raw_Target_Wheel_Omega[index] : 0.0f;
}
inline float Class_Chassis_Omni::Get_Target_Wheel_Omega(int index)
{
    return (index >= 0 && index < OMNI_WHEEL_NUM) ? Target_Wheel_Omega[index] : 0.0f;
}
inline float Class_Chassis_Omni::Get_Wheel_Command_Accel(int index)
{
    return (index >= 0 && index < OMNI_WHEEL_NUM) ? Wheel_Command_Accel[index] : 0.0f;
}
inline float Class_Chassis_Omni::Get_Wheel_Accel_Filtered(int index)
{
    return (index >= 0 && index < OMNI_WHEEL_NUM) ? Wheel_Accel_Filtered[index] : 0.0f;
}
inline float Class_Chassis_Omni::Get_Wheel_Vel_Integral(int index)
{
    return (index >= 0 && index < OMNI_WHEEL_NUM) ? Wheel_Vel_Integral[index] : 0.0f;
}

inline void Class_Chassis_Omni::Set_Chassis_Control_Type(Enum_Chassis_Omni_Control_Type __type)
{
    Chassis_Control_Type = __type;
}
inline void Class_Chassis_Omni::Set_Target_Velocity_X(float __vx)    { Target_Velocity_X = __vx; }
inline void Class_Chassis_Omni::Set_Target_Velocity_Y(float __vy)    { Target_Velocity_Y = __vy; }
inline void Class_Chassis_Omni::Set_Target_Omega(float __omega)      { Target_Omega      = __omega; }
inline void Class_Chassis_Omni::Set_Yaw_Correction_Slow_Only(bool __enable) { yaw_correction_slow_only_ = __enable; }

#endif // CRT_CHASSIS_OMNI_H

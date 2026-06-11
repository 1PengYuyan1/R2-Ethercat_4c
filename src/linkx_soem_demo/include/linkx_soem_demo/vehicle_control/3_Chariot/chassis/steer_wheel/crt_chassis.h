#ifndef USTC_STEERING_1_CRT_CHASSIS_H
#define USTC_STEERING_1_CRT_CHASSIS_H


#include "dvc_motor_dm.h"

#include "math.h"
#include <cmath>

#define STEER_NUM 4

/* 轮子半径（m） */
#define Wheel_Radius_Define     0.076f

/* 轮毂中心到底盘旋转中心的距离（m） */
#define Wheel_To_Core_Distance_Define   0.25f

#define REDUCTION_RATIO_Define  1.0f    /* 舵向减速比（电机轴:舵向轴），直驱=1.0 */


#define MAX_CHASSIS_SPEED       1.5f    /* 底盘最大线速度（m/s） */

#define STEER_ALIGN_THRESHOLD  (15.0f * PI / 180.0f)    /*放宽对齐阈值到 15°，配合余弦缩放，避免高频抽搐 */
struct SteerWheelParams
{
    // 舵向参数
    float steer_kp;                    // 舵向角速度P增益
    float steer_kd;                    // 舵向角速度D增益

    float steer_friction_torque;       // 舵向克服静摩擦的固定力矩
    float steer_torque_deadzone;       // 固定力矩的判定死区

    // 轮向参数
    float wheel_omega_deadzone;        // 轮向死区
    float wheel_feedforward;           // 轮向前馈系数
    float wheel_direction;             // 轮向方向(+1或-1)

    //动力学物理参数
    float wheel_coulomb_Tc;            // 库伦摩擦力矩 (N·m)
    float wheel_viscous_B;             // 粘性摩擦系数 (N·m·s/rad)
    float wheel_rotor_inertia;         // 轮向等效转动惯量 (kg·m²)

    float steer_gyro_coeff;            // 陀螺力矩系数 (kg·m²)
    float steer_rotor_inertia;         // 舵向等效转动惯量 (kg·m²)

    // 翻轮参数
    float flip_speed_threshold;        // 翻轮速度阈值
    float flip_drive_scale;            // 翻轮期间速度缩放

    float wheel_speed_correction;

};

/**
 * @brief 底盘参数
 *
 */
struct ChassisPhysicalParams
{
    float mass = 10.0f;      // 总质量 (kg)
    float inertia = 2.0f;   // 转动惯量 (kg·m²)
};

/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type {
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_ENABLE,
};

/**
 * @brief 舵向电机状态判断
 *
 */
typedef enum
{
    STEER_STATE_IDLE = 0,     // 无运动 / 轮速极低
    STEER_STATE_ALIGN,        // 舵向对准中
    STEER_STATE_DRIVE         // 舵向已对准，允许驱动
} SteerDriveState_e;

/**
 * @brief 舵轮状态判断
 *
 */
typedef struct {
    SteerDriveState_e state;
    uint8_t flip_locked;      // 翻轮锁（只允许一次）
    uint8_t angle_reached;    // 舵向到位锁存
    uint8_t request_drive_slowdown; //翻轮发生的那一刻
} SteerCtrl_t;

/**
 * @brief Specialized, 舵轮底盘类
 *
 */
class Class_Chassis
{
public:

    ChassisPhysicalParams chassis_physical_params_;
    //舵轮底盘变量
    SteerWheelParams steer_wheel_params_[STEER_NUM];
    //舵向电机
    Class_Motor_DM_Normal Motor_Steer[STEER_NUM];
    //轮向电机
    Class_Motor_DM_Normal Motor_Wheel[STEER_NUM];

    //初始化参数
    void Init(linkx_t *__LinkX_Handler);
    void Init_Motor_Params();

    // 获取信息
    inline float Get_Now_Velocity_X();
    inline float Get_Now_Velocity_Y();
    inline float Get_Now_Omega();
    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();

    // 设立信息
    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);

    // 周期回调
    void TIM_2ms_Resolution_PeriodElapsedCallback();
    void TIM_2ms_Control_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();

    bool _Solve_3x3(const float A_mat[3][3], const float b_vec[3], float x_vec[3]);

protected:
    // 轮组半径
    const float Wheel_Radius = Wheel_Radius_Define;

    // 轮距中心长度
    const float Wheel_To_Core_Distance[STEER_NUM] = {
        Wheel_To_Core_Distance_Define,
        Wheel_To_Core_Distance_Define,
        Wheel_To_Core_Distance_Define,
        Wheel_To_Core_Distance_Define
    };

    const float Wheel_Azimuth[STEER_NUM] = {
        (7.0f * PI / 4.0f),  // ID 1 (Index 0) 实际是 右前 315°
        (5.0f * PI / 4.0f),  // ID 2 (Index 1) 实际是 右后 225° (原为45°，翻转180°)
        (3.0f * PI / 4.0f),  // ID 3 (Index 2) 实际是 左后 135°
        (PI / 4.0f)          // ID 4 (Index 3) 实际是 左前 45°  (原为225°，翻转180°)
    };
    uint8_t steer_flipped[STEER_NUM] = {0}; //轮子状态位
    uint8_t steer_filter_init[STEER_NUM] = {0}; // 某个舵轮的滤波器是否已经初始化
    float steer_ref_filtered[STEER_NUM] = {0}; // 存放 舵向参考值的滤波后结果
    uint8_t steer_flip_locked[STEER_NUM] = {0}; //翻轮锁存标志

    //舵向状态判断
    SteerDriveState_e steer_state[STEER_NUM]= {STEER_STATE_IDLE};
    // 轮速缩放系数
    float wheel_speed_scale[STEER_NUM]= {1.0f};


    // 轮向电机角速度目标值
    float Target_Wheel_Omega[STEER_NUM] = {0};
    // 轮向电机力矩
    float Target_Wheel_torque[STEER_NUM] = {0};
    // 轮向最大角速度 (rad/s, DM-3519)
    float MAX_WHEEL_OMEGA = 120.0f;

    //舵向电机力矩
    float Target_Steer_torque[STEER_NUM] = {0};  // 舵向前馈力矩 (N·m)
    // 舵向电机角度目标值
    float Target_Steer_Rad[STEER_NUM]   = {0.0f};

    float Last_Target_Steer_Rad[STEER_NUM] = {0.0f};
    // 舵向目标角速度前馈
    float Target_Steer_Omega[STEER_NUM] = {0.0f};
    // 舵向减速比
    const float REDUCTION_RATIO = REDUCTION_RATIO_Define;
    // 限制舵向最大转向速度
    float MAX_STEER_OMEGA = 30.0f;


    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标角速度
    float Target_Omega = 0.0f;

    // 逆动力学接触力（N）
    float contact_force[STEER_NUM] = {0.0f};

    // 【FIX-C】接触力低通滤波状态量
    // 对克莱姆解算后的原始接触力做低通，消除舵向反馈角噪声
    // 通过 A_mat 放大后引起的力矩跳变（ALPHA_FORCE=0.35 @ 500Hz ≈ 55Hz 截止）
    float contact_force_filtered[STEER_NUM]  = {0.0f};

    // 轮向加速度（差分+低通）
    float prev_target_wheel_omega[STEER_NUM] = {0.0f};
    float wheel_acc_filtered[STEER_NUM]      = {0.0f};

    // 舵向角速度（差分+低通）
    float prev_target_steer[STEER_NUM]       = {0.0f};
    float steer_vel_filtered[STEER_NUM]      = {0.0f};

    // 底盘加速度估算（速度指令差分+低通）
    float prev_target_vx          = 0.0f;
    float prev_target_vy          = 0.0f;
    float prev_target_omega_dyn   = 0.0f;
    float chassis_acc_x           = 0.0f;
    float chassis_acc_y           = 0.0f;
    float chassis_alpha           = 0.0f;

    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;

    // 获取当前舵向角度 (通过达妙反馈 + 减速比换算)
    float Get_Now_Steer_Radian(int index);

    // 舵向目标角度 → 达妙位置指令
    float Steer_To_Motor_Position(float target_steer, int index);

    //舵向电机就近解算
    void _Steer_Motor_Kinematics_Nearest_Transposition();

    void Self_Resolution();    //自身姿态，速度解算

    void Kinematics_Inverse_Resolution();    //舵轮运动学逆结算

    void Output_To_Dynamics();    //输出动力学

    void Dynamics_Inverse_Resolution();    //动力学逆解算

    void Update_Steer_State(int i);

    void Execute_Steer_State(int i);

    void Output_To_Motor();    //输出到电机
    void _Reset_Dynamics_State(); // 清零所有动力学状态量（DISABLE 时调用，防止重新使能时产生冲击力矩）

};

/**
 * @brief 获取当前速度X
 *
 * @return float 当前速度X
 */
inline float Class_Chassis::Get_Now_Velocity_X()
{
    return (Now_Velocity_X);
}
/**
 * @brief 获取当前速度Y
 *
 * @return float 当前速度Y
 */
inline float Class_Chassis::Get_Now_Velocity_Y()
{
    return (Now_Velocity_Y);
}
/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Chassis::Get_Now_Omega()
{
    return (Now_Omega);
}
/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
inline Enum_Chassis_Control_Type Class_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}
/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
inline float Class_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}
/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
inline float Class_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
inline void Class_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}
/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}
/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}
/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

#endif

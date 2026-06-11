//
// Created by pzx on 2025/12/15.
//

#ifndef DVC_MOTOR_DM_H
#define DVC_MOTOR_DM_H
#include <cstdint>
#include <cmath>
#include "linkx.h"
#include "linkx4c_handler.h"

/**
 * @brief 达妙电机状态
 *
 */
enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
};

/**
 * @brief 达妙电机控制状态, 传统模式有效
 *
 */
enum Enum_Motor_DM_Control_Status_Normal
{
    Motor_DM_Control_Status_DISABLE = 0x00,
    Motor_DM_Control_Status_ENABLE,
    Motor_DM_Control_Status_UNDERVOLTAGE,
    Motor_DM_Control_Status_OVERCURRENT,
    Motor_DM_Control_Status_MOS_OVERTEMPERATURE,
    Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE,
    Motor_DM_Control_Status_LOSE_CONNECTION,
    Motor_DM_Control_Status_MOS_OVERLOAD,
    Motor_DM_Control_Status_OVERVOLTAGE = 0x08,

};

/**
 * @brief 达妙电机控制方式
 *
 */
enum Enum_Motor_DM_Control_Method
{
    Motor_DM_Control_Method_NORMAL_MIT = 0,
    Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
    Motor_DM_Control_Method_NORMAL_OMEGA,
    Motor_DM_Control_Method_NORMAL_EMIT,
    Motor_DM_Control_Method_1_TO_4_CURRENT,
    Motor_DM_Control_Method_1_TO_4_OMEGA,
    Motor_DM_Control_Method_1_TO_4_ANGLE,
};

/**
 * @brief 标定模式（参数辨识：动摩擦力 / 静摩擦力 / 转动惯量）
 *
 *   - FRICTION : 恒定速度驱动 → 角加速度=0 → 反馈力矩 ≈ 系统动摩擦力
 *   - STICTION : MIT(kp=kd=0) 阶梯增大 torque → 速度首次突破阈值时的 torque ≈ 静摩擦力
 *   - INERTIA  : MIT(kp=kd=0) 给 torque 阶跃 → 由 ω 拟合得 α，J = (T-T_friction)/α
 *
 * 三个测试都通过 MIT 模式实现（不切换 CAN_Tx_ID），需要：
 *   - 车轮悬空（避免地面摩擦干扰）
 *   - 电机已使能（Motor_DM_Status_ENABLE）
 */
enum Enum_Motor_DM_Calib_Mode
{
    Motor_DM_Calib_Mode_NONE = 0,
    Motor_DM_Calib_Mode_FRICTION,
    Motor_DM_Calib_Mode_STICTION,
    Motor_DM_Calib_Mode_INERTIA,
};

/**
 * @brief 标定结果（外部读取）
 */
struct Struct_Motor_DM_Calib_Result
{
    Enum_Motor_DM_Calib_Mode mode;   // 本次执行的模式
    bool finished;                   // 是否完成（含失败）
    bool success;                    // 是否得到有效结果

    int phase;                       // 当前阶段（debug 用）
    float total_elapsed_s;           // 已运行时长（s）

    // ---- FRICTION ----
    float friction_torque_pos_nm;    // 正转平均反馈力矩 (Nm)
    float friction_torque_neg_nm;    // 反转平均反馈力矩 (Nm，绝对值)
    float friction_torque_avg_nm;    // 正反转平均
    float friction_omega_pos_actual; // 实测正转稳态速度
    float friction_omega_neg_actual; // 实测反转稳态速度

    // ---- STICTION ----
    float stiction_torque_nm;        // 突破时刻施加的 torque
    float stiction_breakaway_omega;  // 突破时刻测得的 ω

    // ---- INERTIA ----
    float inertia_kgm2;              // J，单位 kg·m²
    float inertia_alpha_meas;        // 测得的 α (rad/s²)
    float inertia_torque_step_nm;    // 阶跃 torque
    float inertia_friction_used_nm;  // 减除的动摩擦力（外部传入）
};

/**
 * @brief 达妙电机传统模式源数据
 *
 */
struct Struct_Motor_DM_CAN_Rx_Data_Normal
{
    uint8_t CAN_ID : 4;
    uint8_t Control_Status_Enum : 4;
    uint16_t Angle_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t MOS_Temperature;
    uint8_t Rotor_Temperature;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, MIT控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_MIT
{
    uint16_t Control_Angle_Reverse;
    uint8_t Control_Omega_11_4;
    uint8_t Control_Omega_3_0_K_P_11_8;
    uint8_t K_P_7_0;
    uint8_t K_D_11_4;
    uint8_t K_D_3_0_Control_Torque_11_8;
    uint8_t Control_Torque_7_0;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, 位置速度控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega
{
    float Control_Angle;
    float Control_Omega;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, 速度控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_Omega
{
    float Control_Omega;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, EMIT控制报文
 *
 */
struct Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT
{
    float Control_Angle;
    // 限定速度用, rad/s的100倍
    uint16_t Control_Omega;
    // 限定电流用, 电流最大值的10000倍
    uint16_t Control_Current;
} __attribute__((packed));

/**
 * @brief 达妙电机经过处理的数据, 传统模式有效
 *
 */
struct Struct_Motor_DM_Rx_Data_Normal
{
    Enum_Motor_DM_Control_Status_Normal Control_Status;
    float Now_state;
    float Now_Rad;
    float current_single_rad;
    float Now_Wheel_Rad;
    float Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;

    float Now_Steer_Rad;
    float Now_Steer_Omega;

    float target_omage;
    float target_position;
    float target_torque;

    uint16_t Raw_Angle_Encoder;
    uint16_t Raw_Omega_Encoder;
    uint16_t Raw_Torque_Encoder;

    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
    uint8_t First_Update_Flag;
};

/**
 * @brief Reusable, 达妙电机, 传统模式
 * 没有零点, 可在上位机调零点
 * 初始化的角度, 角速度, 扭矩, 电流等参数是J4310电机默认值
 *
 */
class Class_Motor_DM_Normal
{
public:

    uint16_t DM_CAN_Rx_ID;
    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t DM_CAN_Tx_ID;

    void Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID,
              Enum_Motor_DM_Control_Method __Motor_DM_Control_Method , float __Angle_Max , float __Omega_Max , float __Torque_Max , float __Current_Max );

    inline float Get_Radian_Max();
    inline float Get_Omega_Max();
    inline float Get_Torque_Max();
    inline float Get_Current_Max();
    inline Enum_Motor_DM_Status Get_Status();
    inline Enum_Motor_DM_Control_Status_Normal Get_Now_Control_Status();
    inline float Get_Now_Radian();
    inline float Get_Current_Single_Radian();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline float Get_Now_MOS_Temperature();
    inline float Get_Now_Rotor_Temperature();
    inline uint16_t Get_Raw_Angle_Encoder();
    inline uint16_t Get_Raw_Omega_Encoder();
    inline uint16_t Get_Raw_Torque_Encoder();
    inline float Get_Now_Steer_Radian();
    inline float Get_Now_Steer_Omega();
    inline Enum_Motor_DM_Control_Method Get_Control_Method();
    inline float Get_Control_Radian();
    inline float Get_Control_Omega();
    inline float Get_Control_Torque();
    inline float Get_Control_Current();
    inline float Get_K_P();
    inline float Get_K_D();

    inline void Set_Control_Status(Enum_Motor_DM_Status __Motor_DM_Status);
    inline void Set_Control_Method(Enum_Motor_DM_Control_Method __Motor_DM_Control_Method);
    inline void Set_Control_Radian(float __Control_Radain);
    inline void Set_Control_Omega(float __Control_Omega);
    inline void Set_Control_Torque(float __Control_Torque);
    inline void Set_Control_Current(float __Control_Current);
    inline void Set_K_P(float __K_P);
    inline void Set_K_D(float __K_D);
    inline void Set_Control_Maintain_Postion(float __Control_Radain,
                                             float __Control_Omega,
                                             float __Control_Torque,
                                             float __K_P,
                                             float __K_D);
    inline void Set_Control_Parameter_MIT(float __Control_Angle, float __Control_Omega);
    inline void Set_Control_Parameter_MIT(float __Control_Torque, float __Control_Angle, float __Control_Omega);
    inline void Set_Control_Torque_P_D_MIT(float __Control_Torque, float __K_P, float __K_D);
    inline void Set_Control_Torque_P_D_MIT(float __K_P, float __K_D);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    void CAN_Send_Clear_Error();
    void CAN_Send_Enter();
    void CAN_Send_Exit();
    void CAN_Send_Save_Zero();

    // 选择发送帧类型: true=CAN-FD(带 BRS, 切高速数据段), false=经典 CAN(默认)。需在 Init 之后调用。
    void Set_Use_FDCAN(bool __enable) { Use_FDCAN_ = __enable; }
    // 调试模式: 无反馈状态时也发送当前控制帧。仅用于链路/底盘架空联调。
    void Set_Force_Output_Without_Feedback(bool __enable) { Force_Output_Without_Feedback_ = __enable; }
    // 位置反馈默认直接使用 DM 协议的 [-Pmax,+Pmax] 位置值；需要跨边界连续展开时显式开启。
    void Set_Position_Unwrap(bool __enable) { Position_Unwrap_Enable_ = __enable; }
    // 反馈位置 16-bit 半区间缩放，默认等于控制 Pmax；部分减速电机反馈单圈范围与控制 Pmax 不一致。
    void Set_Feedback_Radian_Max(float __Feedback_Radian_Max) { Feedback_Radian_Max = __Feedback_Radian_Max; }

    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Send_PeriodElapsedCallback();

    // ============================================================
    //   参数标定（动摩擦 / 静摩擦 / 转动惯量），全程基于 MIT 模式
    //   使用方式：
    //     1) 启动其中一项 Begin_*_Calibration(...)
    //     2) 主循环每个控制周期调一次 Calibration_Tick(dt_s)（典型 dt=0.002）
    //     3) 通过 Is_Calibration_Running() / Get_Calibration_Result() 查状态
    //     4) 需要中止时调 Stop_Calibration()
    //   注意：开始前 ★ 必须把车轮架空 ★（避免地面摩擦混入测量）
    // ============================================================

    /** 启动动摩擦标定：用 MIT 的 D 项当速度环跟住 omega_target，稳态 torque ≈ 动摩擦
     *  @param omega_target_rad_s   目标速度（电机轴），建议 1.0~3.0
     *  @param kd_velocity_loop     MIT D 增益，建议 1.5~3.0（越大跟得越紧但越粗噪）
     *  @param warmup_s             单方向加速到位的等待时间（默认 1.5s）
     *  @param measure_s            单方向取样时长（默认 2.0s）*/
    void Begin_Friction_Calibration(float omega_target_rad_s,
                                    float kd_velocity_loop = 2.0f,
                                    float warmup_s = 1.5f,
                                    float measure_s = 2.0f);

    /** 启动静摩擦标定：MIT(kp=0,kd=0)，torque 从 0 阶梯递增直到 |omega| 突破阈值
     *  @param torque_step_nm       每步增量（建议 0.005~0.02）
     *  @param dwell_s              每步保持时长（建议 0.10~0.20）
     *  @param omega_threshold_rad_s 突破判定的 ω 阈值（建议 0.3）
     *  @param torque_max_nm        上限保护，超出仍未动则失败（建议 1.0）*/
    void Begin_Stiction_Calibration(float torque_step_nm = 0.01f,
                                    float dwell_s = 0.10f,
                                    float omega_threshold_rad_s = 0.3f,
                                    float torque_max_nm = 1.0f);

    /** 启动转动惯量标定：MIT(kp=0,kd=0) 给一个 torque 阶跃，由 ω-t 一次拟合得 α
     *  @param torque_step_nm           阶跃力矩，建议 0.3~0.8（足够使转子明显加速）
     *  @param friction_torque_known_nm 减除的动摩擦力（先跑 Friction 标定得到，可填 0）
     *  @param warmup_s                 阶跃前让转子静止的等待时间（默认 0.5s）
     *  @param accel_duration_s         阶跃持续/采样时长（默认 0.5s，避免 ω 撞速度上限）*/
    void Begin_Inertia_Calibration(float torque_step_nm,
                                   float friction_torque_known_nm = 0.0f,
                                   float warmup_s = 0.5f,
                                   float accel_duration_s = 0.5f);

    /** 主循环每个控制周期调一次，推进标定状态机 */
    void Calibration_Tick(float dt_s);

    /** 立即停止标定，恢复 0 力矩、0 增益（不会失能电机） */
    void Stop_Calibration();

    /** 是否处于标定中（已启动但未完成） */
    bool Is_Calibration_Running() const;

    /** 是否完成（成功或失败均算 finished） */
    bool Is_Calibration_Finished() const;

    /** 获取最近一次标定结果（运行中也可读，phase/total_elapsed_s 实时更新） */
    Struct_Motor_DM_Calib_Result Get_Calibration_Result() const;

protected:
    // 初始化相关变量

    // 绑定的CAN
    linkx_t *LinkX_Handler; 
    uint8_t CAN_Channel;    

    // 最大位置, 与上位机控制幅值PMAX保持一致
    float Radian_Max;
    // 反馈位置 16-bit 半区间缩放；默认与 Radian_Max 相同，可按具体电机反馈编码单独设置
    float Feedback_Radian_Max;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float Omega_Max;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float Torque_Max;
    // 最大电流, 与上位机串口中上电打印电流保持一致
    float Current_Max;

    // 常量

    // 内部变量

    // 发送缓冲区
    uint8_t Tx_Data[8];

    // 发送是否使用 CAN-FD(带 BRS, 切高速数据段); 默认经典 CAN, 仅特定测试显式开启
    bool Use_FDCAN_ = false;
    bool Force_Output_Without_Feedback_ = false;
    bool Position_Unwrap_Enable_ = false;

    // 读变量

    // 电机状态
    Enum_Motor_DM_Status Motor_DM_Status = Motor_DM_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_DM_Rx_Data_Normal data;

    // 写变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    // 读写变量

    // 电机控制方式
    Enum_Motor_DM_Control_Method Motor_DM_Control_Method = Motor_DM_Control_Method_NORMAL_MIT;

    // 角度, rad, 目标角度
    float Control_Radian = 0.0f;
    // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
    float Control_Omega = 0.0f;
    // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
    float Control_Torque = 0.0f;
    // 电流, A, EMIT模式是限幅, 其余模式无效
    float Control_Current = 0.0f;
    // K_P, 0~500, MIT模式有效
    float K_P = 0.0f;
    // K_D, 0~5, MIT模式有效
    float K_D = 0.0f;

    // ============ 标定相关内部状态 ============
    Enum_Motor_DM_Calib_Mode calib_mode_ = Motor_DM_Calib_Mode_NONE;
    int   calib_phase_ = 0;          // 状态机阶段
    float calib_phase_elapsed_s_ = 0.0f;
    float calib_total_elapsed_s_ = 0.0f;

    // FRICTION 参数 + 累加器
    float friction_omega_target_ = 0.0f;
    float friction_kd_ = 2.0f;
    float friction_warmup_s_ = 1.5f;
    float friction_measure_s_ = 2.0f;
    double friction_acc_torque_pos_ = 0.0;
    double friction_acc_torque_neg_ = 0.0;
    double friction_acc_omega_pos_ = 0.0;
    double friction_acc_omega_neg_ = 0.0;
    uint32_t friction_n_pos_ = 0;
    uint32_t friction_n_neg_ = 0;

    // STICTION 参数
    float stiction_torque_step_ = 0.01f;
    float stiction_dwell_s_ = 0.10f;
    float stiction_omega_thresh_ = 0.3f;
    float stiction_torque_max_ = 1.0f;
    float stiction_current_torque_ = 0.0f;

    // INERTIA 参数 + 最小二乘累加器（拟合 ω = ω0 + α·t）
    float inertia_torque_step_ = 0.5f;
    float inertia_warmup_s_ = 0.5f;
    float inertia_duration_s_ = 0.5f;
    float inertia_friction_known_ = 0.0f;
    double inertia_t_sum_ = 0.0;
    double inertia_omega_sum_ = 0.0;
    double inertia_t2_sum_ = 0.0;
    double inertia_t_omega_sum_ = 0.0;
    uint32_t inertia_n_ = 0;

    // 结果
    Struct_Motor_DM_Calib_Result calib_result_ = {};

    // 内部函数

    void Data_Process(uint8_t *__Data);

    void Output();

    // 按 Use_FDCAN_ 选择经典/FD(BRS) 帧, 发送 8 字节到 DM_CAN_Tx_ID
    void CAN_Send_Frame(const uint8_t *__data);
};

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DM_Status 电机状态
 */
inline Enum_Motor_DM_Status Class_Motor_DM_Normal::Get_Status()
{
    return (Motor_DM_Status);
}

/**
 * @brief 获取角度最大值
 *
 * @return float 角度最大值
 */
inline float Class_Motor_DM_Normal::Get_Radian_Max()
{
    return (Radian_Max);
}

/**
 * @brief 获取角速度最大值
 *
 * @return float 角速度最大值
 */
inline float Class_Motor_DM_Normal::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取扭矩最大值
 *
 * @return float 扭矩最大值
 */
inline float Class_Motor_DM_Normal::Get_Torque_Max()
{
    return (Torque_Max);
}

/**
 * @brief 获取电流最大值
 *
 * @return float 电流最大值
 */
inline float Class_Motor_DM_Normal::Get_Current_Max()
{
    return (Current_Max);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_DM_Control_Method 电机控制方式
 */
inline Enum_Motor_DM_Control_Method Class_Motor_DM_Normal::Get_Control_Method()
{
    return (Motor_DM_Control_Method);
}

/**
 * @brief 获取角度, rad, 目标角度
 *
 * @return float 角度, rad, 目标角度
 */
inline float Class_Motor_DM_Normal::Get_Control_Radian()
{
    return (Control_Radian);
}

/**
 * @brief 获取角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @return float 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
inline float Class_Motor_DM_Normal::Get_Control_Omega()
{
    return (Control_Omega);
}

/**
 * @brief 获取扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @return float 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
inline float Class_Motor_DM_Normal::Get_Control_Torque()
{
    return (Control_Torque);
}

/**
 * @brief 获取电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @return float 电流, A, EMIT模式是限幅, 其余模式无效
 */
inline float Class_Motor_DM_Normal::Get_Control_Current()
{
    return (Control_Current);
}

/**
 * @brief 获取K_P, 0~500, MIT模式有效
 *
 * @return float K_P, 0~500, MIT模式有效
 */
inline float Class_Motor_DM_Normal::Get_K_P()
{
    return (K_P);
}

/**
 * @brief 获取K_D, 0~5, MIT模式有效
 *
 * @return float K_D, 0~5, MIT模式有效
 */
inline float Class_Motor_DM_Normal::Get_K_D()
{
    return (K_D);
}

/**
 * @brief 获取舵向当前角速度
 *
 * @return float 当前舵向角速度
 */
inline Enum_Motor_DM_Control_Status_Normal Class_Motor_DM_Normal::Get_Now_Control_Status()
{
    return (data.Control_Status);
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度
 */
inline float Class_Motor_DM_Normal::Get_Now_Radian()
{
    return (data.Now_Rad);
}

/**
 * @brief 获取单圈协议角度
 *
 * @return float 单圈协议角度
 */
inline float Class_Motor_DM_Normal::Get_Current_Single_Radian()
{
    return (data.current_single_rad);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Motor_DM_Normal::Get_Now_Omega()
{
    return (data.Now_Omega);
}

/**
 * @brief 获取当前扭矩
 *
 * @return float 当前扭矩
 */
inline float Class_Motor_DM_Normal::Get_Now_Torque()
{
    return (data.Now_Torque);
}

/**
 * @brief 获取当前MOS温度
 *
 * @return float 当前MOS温度
 */
inline float Class_Motor_DM_Normal::Get_Now_MOS_Temperature()
{
    return (data.Now_MOS_Temperature);
}

/**
 * @brief 获取当前转子温度
 *
 * @return float 当前转子温度
 */
inline float Class_Motor_DM_Normal::Get_Now_Rotor_Temperature()
{
    return (data.Now_Rotor_Temperature);
}

/**
 * @brief 获取 DM 反馈原始位置编码值
 *
 * @return uint16_t 16-bit raw angle
 */
inline uint16_t Class_Motor_DM_Normal::Get_Raw_Angle_Encoder()
{
    return (data.Raw_Angle_Encoder);
}

/**
 * @brief 获取 DM 反馈原始速度编码值
 *
 * @return uint16_t 12-bit raw omega
 */
inline uint16_t Class_Motor_DM_Normal::Get_Raw_Omega_Encoder()
{
    return (data.Raw_Omega_Encoder);
}

/**
 * @brief 获取 DM 反馈原始力矩编码值
 *
 * @return uint16_t 12-bit raw torque
 */
inline uint16_t Class_Motor_DM_Normal::Get_Raw_Torque_Encoder()
{
    return (data.Raw_Torque_Encoder);
}

/**
 * @brief 获取舵向当前角度
 *
 * @return float 当前舵向角度
 */
inline float Class_Motor_DM_Normal::Get_Now_Steer_Radian()
{
    return (data.Now_Steer_Rad);
}

/**
 * @brief 获取舵向当前角速度
 *
 * @return float 当前舵向角速度
 */
inline float Class_Motor_DM_Normal::Get_Now_Steer_Omega()
{
    return (data.Now_Steer_Omega);
}

/**
 * @brief 设置电机控制模式
 *
 * @return
 */
inline void Class_Motor_DM_Normal::Set_Control_Method(Enum_Motor_DM_Control_Method __Motor_DM_Control_Method)
{
    Motor_DM_Control_Method = __Motor_DM_Control_Method;
}

/**
 * @brief 设置电机失能，使能状态
 *
 * @return
 */
inline void Class_Motor_DM_Normal::Set_Control_Status(Enum_Motor_DM_Status __Motor_DM_Status)
{
    Motor_DM_Status = __Motor_DM_Status;
}

/**
 * @brief 设定角度, rad, 目标角度
 *
 * @param __Control_Angle 角度, rad, 目标角度
 */
inline void Class_Motor_DM_Normal::Set_Control_Radian(float __Control_Radain)
{
    Control_Radian = __Control_Radain;
}

/**
 * @brief 设定角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @param __Control_Omega 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
inline void Class_Motor_DM_Normal::Set_Control_Omega(float __Control_Omega)
{
    Control_Omega = __Control_Omega;
}

/**
 * @brief 设定扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @param __Control_Torque 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
inline void Class_Motor_DM_Normal::Set_Control_Torque(float __Control_Torque)
{
    Control_Torque = __Control_Torque;
}

/**
 * @brief 设定K_P, 0~500, MIT模式有效
 *
 * @param __K_P K_P, 0~500, MIT模式有效
 */
inline void Class_Motor_DM_Normal::Set_K_P(float __K_P)
{
    K_P = __K_P;
}

/**
 * @brief 设定K_D, 0~5, MIT模式有效
 *
 * @param __K_D K_D, 0~5, MIT模式有效
 */
inline void Class_Motor_DM_Normal::Set_K_D(float __K_D)
{
    K_D = __K_D;
}

/**
 * @brief 设定K_D, 0~5, MIT模式有效
 * @brief 设定K_P, 0~500, MIT模式有效
 *
 */
inline void Class_Motor_DM_Normal::Set_Control_Torque_P_D_MIT(float __Control_Torque, float __K_P, float __K_D)
{
    Set_Control_Torque(__Control_Torque);
    Set_K_P(__K_P);
    Set_K_D(__K_D);
}

/**
 * @brief 设定__Control_AngleMIT模式有效
 * @brief 设定__Control_OmegaMIT模式有效
 *
 */
inline void Class_Motor_DM_Normal::Set_Control_Parameter_MIT(float __Control_Angle, float __Control_Omega)
{
    Set_Control_Radian(__Control_Angle);
    Set_Control_Omega(__Control_Omega);
}

/**
 * @brief 兼容旧版接口：按 (torque, angle, omega) 一次写入 MIT 三参数
 */
inline void Class_Motor_DM_Normal::Set_Control_Parameter_MIT(float __Control_Torque, float __Control_Angle, float __Control_Omega)
{
    Set_Control_Torque(__Control_Torque);
    Set_Control_Radian(__Control_Angle);
    Set_Control_Omega(__Control_Omega);
}

/**
 * @brief 兼容旧版接口：仅写 Kp/Kd，不改 torque
 */
inline void Class_Motor_DM_Normal::Set_Control_Torque_P_D_MIT(float __K_P, float __K_D)
{
    Set_K_P(__K_P);
    Set_K_D(__K_D);
}

/**
 * @brief 设定MIT参数
 *
 */
inline void Class_Motor_DM_Normal::Set_Control_Maintain_Postion(float __Control_Angle, float __Control_Omega, float __Control_Torque, float __K_P, float __K_D)
{
    Set_Control_Radian(__Control_Angle);
    Set_Control_Omega(__Control_Omega);
    Set_Control_Torque(__Control_Torque);
    Set_K_P(__K_P);
    Set_K_D(__K_D);
}

/**
 * @brief 设定电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @param __Control_Current 电流, A, EMIT模式是限幅, 其余模式无效
 */
inline void Class_Motor_DM_Normal::Set_Control_Current(float __Control_Current)
{
    Control_Current = __Control_Current;
}

#endif // USTC_STREETING_DM_H

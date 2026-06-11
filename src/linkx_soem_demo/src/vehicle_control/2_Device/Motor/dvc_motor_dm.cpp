//
// Created by pzx on 2025/12/15.
//
#include "dvc_motor_dm.h"
#include "math.h"

uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfb
};
// 使能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Enter[8] = {
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc
};
// 失能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Exit[8] = {
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd
};
// 保存当前电机位置为零点, 传统模式有效
uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe
};

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param __CAN_Tx_ID 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param __Motor_DM_Control_Method 电机控制方式
 * @param __Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param __Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param __Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 */
void Class_Motor_DM_Normal::Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID,
                                 Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, float __Angle_Max,
                                 float __Omega_Max, float __Torque_Max, float __Current_Max) {
    
    // 保存 EtherCAT LinkX 句柄和通道
    LinkX_Handler = __LinkX_Handler;
    CAN_Channel = __CAN_Channel;

    // 处理控制模式对应的 ID 偏移
    switch (__Motor_DM_Control_Method) {
        case (Motor_DM_Control_Method_NORMAL_MIT): {
            DM_CAN_Tx_ID = __CAN_Tx_ID;
            break;
        }
        case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA): {
            DM_CAN_Tx_ID = __CAN_Tx_ID + 0x100;
            break;
        }
        case (Motor_DM_Control_Method_NORMAL_OMEGA): {
            DM_CAN_Tx_ID = __CAN_Tx_ID + 0x200;
            break;
        }
        case (Motor_DM_Control_Method_NORMAL_EMIT): {
            DM_CAN_Tx_ID = __CAN_Tx_ID + 0x300;
            break;
        }
    }

    DM_CAN_Rx_ID = __CAN_Rx_ID;
    Motor_DM_Control_Method = __Motor_DM_Control_Method;
    Radian_Max = __Angle_Max;
    Feedback_Radian_Max = __Angle_Max;
    Omega_Max = __Omega_Max;
    Torque_Max = __Torque_Max;
    Current_Max = __Current_Max;

    data = {};
    Flag = 0;
    Pre_Flag = 0;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Motor_DM_Normal::CAN_RxCpltCallback(uint8_t *Rx_Data) {
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process(Rx_Data);
}

/**
 * @brief 按 Use_FDCAN_ 选择 FD(带 BRS) 或经典 CAN, 发送 8 字节到 DM_CAN_Tx_ID
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Frame(const uint8_t *__data) {
    if (Use_FDCAN_) {
        linkx_quick_FDcan_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, __data);
    } else {
        linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, __data);
    }
}

/**
 * @brief 发送清除错误信息
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Clear_Error() {
CAN_Send_Frame(DM_Motor_CAN_Message_Clear_Error);
}

/**
 * @brief 发送使能电机
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Enter() {

    CAN_Send_Frame(DM_Motor_CAN_Message_Enter);
}

/**
 * @brief 发送失能电机
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Exit() {
    CAN_Send_Frame(DM_Motor_CAN_Message_Exit);
}

/**
 * @brief 发送保存当前位置为零点
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Save_Zero() {
    CAN_Send_Frame(DM_Motor_CAN_Message_Save_Zero);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期取决于电机掉线时长
 *
 */
void Class_Motor_DM_Normal::TIM_Alive_PeriodElapsedCallback() {

    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag) {
        // 电机断开连接
        Motor_DM_Status = Motor_DM_Status_DISABLE;
    } else {
        // 电机保持连接
        Motor_DM_Status = Motor_DM_Status_ENABLE;
    }

    Pre_Flag = Flag;

}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 */
void Class_Motor_DM_Normal::TIM_Send_PeriodElapsedCallback() {

    auto send_control_frame = [&]() {
        Math_Constrain(&Control_Radian, -Radian_Max, Radian_Max);
        Math_Constrain(&Control_Omega, -Omega_Max, Omega_Max);
        Math_Constrain(&Control_Torque, -Torque_Max, Torque_Max);
        Math_Constrain(&Control_Current, -Current_Max, Current_Max);
        Math_Constrain(&K_P, 0.0f, 500.0f);
        Math_Constrain(&K_D, 0.0f, 5.0f);

        Output();
    };

    if (Force_Output_Without_Feedback_) {
        send_control_frame();
    } else if (data.Control_Status == Motor_DM_Control_Status_ENABLE) {
        // 电机在线, 正常控制
        send_control_frame();
    } else if (data.Control_Status == Motor_DM_Control_Status_DISABLE) {
        // 电机可能掉线, 使能电机
        CAN_Send_Enter();
    } else {
        // 电机错误, 发送清除错误帧
        CAN_Send_Clear_Error();
    }
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Motor_DM_Normal::Data_Process(uint8_t *rx_data) {
    // 原始解析
    int32_t delta_encoder;
    uint16_t tmp_encoder = 0, tmp_omega = 0, tmp_torque = 0;
    Struct_Motor_DM_CAN_Rx_Data_Normal *tmp_buffer =
        (Struct_Motor_DM_CAN_Rx_Data_Normal *)rx_data;

    // 电机ID不匹配, 则不进行处理
    if (tmp_buffer->CAN_ID != (DM_CAN_Tx_ID & 0x0f)) {
        return;
    }

    // 大小端处理
    Math_Endian_Reverse_16((void *)&tmp_buffer->Angle_Reverse, &tmp_encoder);
    tmp_omega = (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | tmp_buffer->Torque_7_0;
    data.Control_Status = static_cast<Enum_Motor_DM_Control_Status_Normal>(tmp_buffer->Control_Status_Enum);
    data.Raw_Angle_Encoder = tmp_encoder;
    data.Raw_Omega_Encoder = tmp_omega;
    data.Raw_Torque_Encoder = tmp_torque;

    // DM 反馈位置按 16-bit 映射到反馈半区间 [-Feedback_Radian_Max,+Feedback_Radian_Max]。
    // Feedback_Radian_Max 默认等于控制 Pmax；减速电机反馈单圈范围与控制 Pmax 不一致时需单独设置。
    const float range_rad = 2.0f * Feedback_Radian_Max;
    data.current_single_rad = -Feedback_Radian_Max + (tmp_encoder / 65535.0f) * range_rad;

    if (data.First_Update_Flag == 0) {
        data.Pre_Encoder = tmp_encoder;
        data.Total_Round = 0;
        data.First_Update_Flag = 1;
    }

    if (Position_Unwrap_Enable_) {
        // 可选连续展开：仅用于确实需要跨 Pmax 边界累计位置的场景。
        delta_encoder = static_cast<int32_t>(tmp_encoder) - static_cast<int32_t>(data.Pre_Encoder);
        if (delta_encoder < -(1 << 15)) {
            data.Total_Round++;
        } else if (delta_encoder > (1 << 15)) {
            data.Total_Round--;
        }
        data.Total_Encoder =
            data.Total_Round * (1 << 16) +
            static_cast<int32_t>(tmp_encoder) -
            ((1 << 15) - 1);
        data.Now_Rad = (data.Total_Round * range_rad) + data.current_single_rad;
    } else {
        data.Total_Round = 0;
        data.Total_Encoder = static_cast<int32_t>(tmp_encoder) - ((1 << 15) - 1);
        data.Now_Rad = data.current_single_rad;
    }

    // 速度 & 力矩  & 温度
    // 修复：反馈协议是双向 12-bit (0..4095 映射 -Vmax..+Vmax)，
    //       原 (0x7ff,0xFFF→0,Vmax) 会把负向截断为 0。
    data.Now_Omega = Math_Int_To_Float(tmp_omega, 0, (1 << 12) - 1, -Omega_Max, Omega_Max);
    data.Now_Torque = Math_Int_To_Float(tmp_torque, 0, (1 << 12) - 1, -Torque_Max, Torque_Max);
    data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + CELSIUS_TO_KELVIN;
    data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + CELSIUS_TO_KELVIN;
    data.Pre_Encoder = tmp_encoder;

    float steering_rad = data.Now_Rad / 3.5f;
    data.Now_Wheel_Rad = Math_Modulus_Normalization(steering_rad, 2.0f * PI);
    data.Now_Steer_Rad = data.Now_Wheel_Rad;
}

/**
 * @brief 电机数据输出到CAN总线
 *
 */
void Class_Motor_DM_Normal::Output() {
    // 电机控制
    switch (Motor_DM_Control_Method) {
        case (Motor_DM_Control_Method_NORMAL_MIT): {
            Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *) Tx_Data;

            uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

            tmp_angle = Math_Float_To_Int(Control_Radian, -Radian_Max, Radian_Max, 0, (1 << 16) - 1);
            tmp_omega = Math_Float_To_Int(Control_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
            tmp_torque = Math_Float_To_Int(Control_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);
            tmp_k_p = Math_Float_To_Int(K_P, 0, 500.0f, 0, (1 << 12) - 1);
            tmp_k_d = Math_Float_To_Int(K_D, 0, 5.0f, 0, (1 << 12) - 1);

            tmp_buffer->Control_Angle_Reverse = Math_Endian_Reverse_16(&tmp_angle, nullptr);
            tmp_buffer->Control_Omega_11_4 = tmp_omega >> 4;
            tmp_buffer->Control_Omega_3_0_K_P_11_8 = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
            tmp_buffer->K_P_7_0 = tmp_k_p & 0xff;
            tmp_buffer->K_D_11_4 = tmp_k_d >> 4;
            tmp_buffer->K_D_3_0_Control_Torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
            tmp_buffer->Control_Torque_7_0 = tmp_torque & 0xff;

            CAN_Send_Frame(Tx_Data);

            break;
        }
        case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA): {
            Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *) Tx_Data;

            tmp_buffer->Control_Angle = Control_Radian;
            tmp_buffer->Control_Omega = Control_Omega;

                CAN_Send_Frame(Tx_Data);

            break;
        }
        case (Motor_DM_Control_Method_NORMAL_OMEGA): {
            Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *) Tx_Data;

            tmp_buffer->Control_Omega = Control_Omega;

                CAN_Send_Frame(Tx_Data);

            break;
        }
        case (Motor_DM_Control_Method_NORMAL_EMIT): {
            Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *) Tx_Data;

            tmp_buffer->Control_Angle = Control_Radian;
            tmp_buffer->Control_Omega = (uint16_t) (Control_Omega * 100.0f);
            tmp_buffer->Control_Current = (uint16_t) (Control_Current / Current_Max * 10000.0f);

            CAN_Send_Frame(Tx_Data);

            break;
        }
    }
}

/* ============================================================================
 *   参数标定实现：动摩擦 / 静摩擦 / 转动惯量
 *   - 全部走 MIT 模式（不切 CAN_Tx_ID）；外部主循环每 tick 调 Calibration_Tick(dt)
 *   - 启动前应保证电机已 ENABLE、车轮悬空
 * ========================================================================== */

void Class_Motor_DM_Normal::Begin_Friction_Calibration(float omega_target_rad_s,
                                                      float kd_velocity_loop,
                                                      float warmup_s,
                                                      float measure_s)
{
    calib_mode_ = Motor_DM_Calib_Mode_FRICTION;
    calib_phase_ = 0;
    calib_phase_elapsed_s_ = 0.0f;
    calib_total_elapsed_s_ = 0.0f;

    friction_omega_target_ = omega_target_rad_s;
    friction_kd_ = kd_velocity_loop;
    friction_warmup_s_ = warmup_s;
    friction_measure_s_ = measure_s;
    friction_acc_torque_pos_ = friction_acc_torque_neg_ = 0.0;
    friction_acc_omega_pos_ = friction_acc_omega_neg_ = 0.0;
    friction_n_pos_ = friction_n_neg_ = 0;

    calib_result_ = {};
    calib_result_.mode = Motor_DM_Calib_Mode_FRICTION;

    // 立即让电机以正向速度跟随：MIT(kp=0, kd=kd_vel, omega=+target, torque=0)
    Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
    Set_Control_Maintain_Postion(/*angle=*/0.0f,
                                 /*omega=*/+friction_omega_target_,
                                 /*torque=*/0.0f,
                                 /*kp=*/0.0f,
                                 /*kd=*/friction_kd_);

}

void Class_Motor_DM_Normal::Begin_Stiction_Calibration(float torque_step_nm,
                                                      float dwell_s,
                                                      float omega_threshold_rad_s,
                                                      float torque_max_nm)
{
    calib_mode_ = Motor_DM_Calib_Mode_STICTION;
    calib_phase_ = 0;
    calib_phase_elapsed_s_ = 0.0f;
    calib_total_elapsed_s_ = 0.0f;

    stiction_torque_step_ = torque_step_nm;
    stiction_dwell_s_ = dwell_s;
    stiction_omega_thresh_ = omega_threshold_rad_s;
    stiction_torque_max_ = torque_max_nm;
    stiction_current_torque_ = 0.0f;

    calib_result_ = {};
    calib_result_.mode = Motor_DM_Calib_Mode_STICTION;

    // 第一步：MIT 模式，全 0（让电机进入“松软”状态）
    Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
    Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

}

void Class_Motor_DM_Normal::Begin_Inertia_Calibration(float torque_step_nm,
                                                     float friction_torque_known_nm,
                                                     float warmup_s,
                                                     float accel_duration_s)
{
    calib_mode_ = Motor_DM_Calib_Mode_INERTIA;
    calib_phase_ = 0;
    calib_phase_elapsed_s_ = 0.0f;
    calib_total_elapsed_s_ = 0.0f;

    inertia_torque_step_ = torque_step_nm;
    inertia_friction_known_ = friction_torque_known_nm;
    inertia_warmup_s_ = warmup_s;
    inertia_duration_s_ = accel_duration_s;

    inertia_t_sum_ = inertia_omega_sum_ = inertia_t2_sum_ = inertia_t_omega_sum_ = 0.0;
    inertia_n_ = 0;

    calib_result_ = {};
    calib_result_.mode = Motor_DM_Calib_Mode_INERTIA;

    // 第一步：让电机静止（MIT 全 0）
    Set_Control_Method(Motor_DM_Control_Method_NORMAL_MIT);
    Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

}

void Class_Motor_DM_Normal::Stop_Calibration()
{
    calib_mode_ = Motor_DM_Calib_Mode_NONE;
    calib_result_.finished = true;
    Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

bool Class_Motor_DM_Normal::Is_Calibration_Running() const
{
    return (calib_mode_ != Motor_DM_Calib_Mode_NONE) && !calib_result_.finished;
}

bool Class_Motor_DM_Normal::Is_Calibration_Finished() const
{
    return calib_result_.finished;
}

Struct_Motor_DM_Calib_Result Class_Motor_DM_Normal::Get_Calibration_Result() const
{
    return calib_result_;
}

/**
 * @brief 主循环每 tick 调用一次（dt_s = 控制周期，典型 0.002）
 *        将状态机推进到下一阶段；不会阻塞。
 */
void Class_Motor_DM_Normal::Calibration_Tick(float dt_s)
{
    if (calib_mode_ == Motor_DM_Calib_Mode_NONE || calib_result_.finished)
        return;

    calib_phase_elapsed_s_ += dt_s;
    calib_total_elapsed_s_ += dt_s;
    calib_result_.phase = calib_phase_;
    calib_result_.total_elapsed_s = calib_total_elapsed_s_;

    switch (calib_mode_)
    {
        // ---------------------------------------------------------------
        case Motor_DM_Calib_Mode_FRICTION:
        {
            // phase 0: 正向 warmup（等加速到目标速度）
            // phase 1: 正向 measure（累加 torque/omega）
            // phase 2: 反向 warmup
            // phase 3: 反向 measure
            // phase 4: done
            switch (calib_phase_)
            {
                case 0:
                    Set_Control_Maintain_Postion(0.0f, +friction_omega_target_, 0.0f, 0.0f, friction_kd_);
                    if (calib_phase_elapsed_s_ >= friction_warmup_s_)
                    {
                        calib_phase_ = 1;
                        calib_phase_elapsed_s_ = 0.0f;
                    }
                    break;
                case 1:
                    Set_Control_Maintain_Postion(0.0f, +friction_omega_target_, 0.0f, 0.0f, friction_kd_);
                    friction_acc_torque_pos_ += data.Now_Torque;
                    friction_acc_omega_pos_  += data.Now_Omega;
                    friction_n_pos_++;
                    if (calib_phase_elapsed_s_ >= friction_measure_s_)
                    {
                        calib_phase_ = 2;
                        calib_phase_elapsed_s_ = 0.0f;
                    }
                    break;
                case 2:
                    Set_Control_Maintain_Postion(0.0f, -friction_omega_target_, 0.0f, 0.0f, friction_kd_);
                    if (calib_phase_elapsed_s_ >= friction_warmup_s_)
                    {
                        calib_phase_ = 3;
                        calib_phase_elapsed_s_ = 0.0f;
                    }
                    break;
                case 3:
                    Set_Control_Maintain_Postion(0.0f, -friction_omega_target_, 0.0f, 0.0f, friction_kd_);
                    friction_acc_torque_neg_ += data.Now_Torque;
                    friction_acc_omega_neg_  += data.Now_Omega;
                    friction_n_neg_++;
                    if (calib_phase_elapsed_s_ >= friction_measure_s_)
                    {
                        // ---- 计算结果 ----
                        const float t_pos = (friction_n_pos_ > 0)
                            ? static_cast<float>(friction_acc_torque_pos_ / friction_n_pos_) : 0.0f;
                        const float t_neg = (friction_n_neg_ > 0)
                            ? static_cast<float>(friction_acc_torque_neg_ / friction_n_neg_) : 0.0f;
                        const float w_pos = (friction_n_pos_ > 0)
                            ? static_cast<float>(friction_acc_omega_pos_ / friction_n_pos_) : 0.0f;
                        const float w_neg = (friction_n_neg_ > 0)
                            ? static_cast<float>(friction_acc_omega_neg_ / friction_n_neg_) : 0.0f;
                        calib_result_.friction_torque_pos_nm = t_pos;
                        calib_result_.friction_torque_neg_nm = std::fabs(t_neg);
                        calib_result_.friction_torque_avg_nm =
                            0.5f * (std::fabs(t_pos) + std::fabs(t_neg));
                        calib_result_.friction_omega_pos_actual = w_pos;
                        calib_result_.friction_omega_neg_actual = w_neg;
                        calib_result_.success = (friction_n_pos_ > 50 && friction_n_neg_ > 50);
                        calib_result_.finished = true;
                        // 停转
                        Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    }
                    break;
                default:
                    Stop_Calibration();
                    break;
            }
            break;
        }

        // ---------------------------------------------------------------
        case Motor_DM_Calib_Mode_STICTION:
        {
            // phase 0: 在当前 dwell 周期中保持 stiction_current_torque_
            //          每 dwell_s 加 step；监控 |omega| > threshold 立即结束
            //          torque > torque_max 视为失败
            Set_Control_Maintain_Postion(0.0f, 0.0f, stiction_current_torque_, 0.0f, 0.0f);

            if (std::fabs(data.Now_Omega) > stiction_omega_thresh_ && stiction_current_torque_ > 1e-6f)
            {
                calib_result_.stiction_torque_nm = stiction_current_torque_;
                calib_result_.stiction_breakaway_omega = data.Now_Omega;
                calib_result_.success = true;
                calib_result_.finished = true;
                Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                break;
            }

            if (calib_phase_elapsed_s_ >= stiction_dwell_s_)
            {
                stiction_current_torque_ += stiction_torque_step_;
                calib_phase_elapsed_s_ = 0.0f;
                if (stiction_current_torque_ > stiction_torque_max_)
                {
                    calib_result_.stiction_torque_nm = stiction_torque_max_;
                    calib_result_.success = false;
                    calib_result_.finished = true;
                    Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                }
            }
            break;
        }

        // ---------------------------------------------------------------
        case Motor_DM_Calib_Mode_INERTIA:
        {
            // phase 0: warmup, 让转子静止
            // phase 1: 阶跃 torque, 累加 (t, ω) 做最小二乘求 α
            // phase 2: done
            switch (calib_phase_)
            {
                case 0:
                    Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    if (calib_phase_elapsed_s_ >= inertia_warmup_s_)
                    {
                        calib_phase_ = 1;
                        calib_phase_elapsed_s_ = 0.0f;
                    }
                    break;
                case 1:
                {
                    Set_Control_Maintain_Postion(0.0f, 0.0f, inertia_torque_step_, 0.0f, 0.0f);
                    const double t = static_cast<double>(calib_phase_elapsed_s_);
                    const double w = static_cast<double>(data.Now_Omega);
                    inertia_t_sum_ += t;
                    inertia_omega_sum_ += w;
                    inertia_t2_sum_ += t * t;
                    inertia_t_omega_sum_ += t * w;
                    inertia_n_++;
                    if (calib_phase_elapsed_s_ >= inertia_duration_s_)
                    {
                        // 一次最小二乘： ω = a + α·t  →  α = (n·Σtω - Σt·Σω) / (n·Σt² - (Σt)²)
                        const double n = static_cast<double>(inertia_n_);
                        const double denom = n * inertia_t2_sum_ - inertia_t_sum_ * inertia_t_sum_;
                        float alpha = 0.0f;
                        if (denom > 1e-12 && inertia_n_ > 20)
                        {
                            alpha = static_cast<float>(
                                (n * inertia_t_omega_sum_ - inertia_t_sum_ * inertia_omega_sum_) / denom);
                        }
                        const float net_torque = inertia_torque_step_ - inertia_friction_known_;
                        float J = 0.0f;
                        if (std::fabs(alpha) > 1e-3f)
                            J = net_torque / alpha;

                        calib_result_.inertia_kgm2 = J;
                        calib_result_.inertia_alpha_meas = alpha;
                        calib_result_.inertia_torque_step_nm = inertia_torque_step_;
                        calib_result_.inertia_friction_used_nm = inertia_friction_known_;
                        calib_result_.success = (J > 0.0f && std::isfinite(J));
                        calib_result_.finished = true;
                        // 停力矩
                        Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    }
                    break;
                }
                default:
                    Stop_Calibration();
                    break;
            }
            break;
        }

        case Motor_DM_Calib_Mode_NONE:
        default:
            break;
    }
}

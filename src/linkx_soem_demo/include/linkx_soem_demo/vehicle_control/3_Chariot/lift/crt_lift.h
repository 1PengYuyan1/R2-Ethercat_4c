#ifndef CRT_LIFT_H
#define CRT_LIFT_H

#include "dvc_motor_dm.h"

#include <cstdint>

#define CHARIOT_LIFT_MODULE_NUM 2
#define CHARIOT_LIFT_TOF_NUM 7

// 抬升目标位置：统一以「电机角度」(DM3519 编码器 rad) 定义，作为唯一对外配置来源。
// 杆角 = 电机角 / kLiftMotorToRodRatio(=3，定义于 crt_lift.cpp)，仅为内部实现细节。
// 改抬升高度只需改这里的电机角。车头电机为负、车尾电机反向安装故取正，幅值一致。
constexpr float kLiftFrontRetractMotorAngle = -0.1f;
constexpr float kLiftRearRetractMotorAngle = 0.1f;
constexpr float kLiftFrontRaiseMotorAngle = -24.5f;
constexpr float kLiftRearRaiseMotorAngle = 24.5f;
constexpr float kLiftFrontDeepRaiseMotorAngle = -46.0f;
constexpr float kLiftRearDeepRaiseMotorAngle = 46.0f;

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

enum Enum_Chariot_Lift_ToF_Sensor
{
    CHARIOT_LIFT_TOF_UP_FRONT = 0,
    CHARIOT_LIFT_TOF_UP_BACK,
    CHARIOT_LIFT_TOF_DOWN_FRONT,
    CHARIOT_LIFT_TOF_DOWN_BACK,
    // 车尾第二路下视 TFmini-S（slave1 / CAN2 / id 0x02，话题 /high/down_back2/range）。
    // 与 DOWN_BACK 一同用于下/上台阶车尾触发：需两路同时突变才动作/进状态。
    CHARIOT_LIFT_TOF_DOWN_BACK2,
    // 整车正方形底盘车头面的两个 TFmini-S（slave1 / CAN1 / id 0x01、0x02）。
    // 注：左右差分偏航纠偏已移除，姿态纠正改为纯 IMU 吸附最近正交航向；
    // 这两路仍保留采集，暂未参与爬梯纠偏。
    CHARIOT_LIFT_TOF_CHASSIS_FRONT_LEFT,
    CHARIOT_LIFT_TOF_CHASSIS_FRONT_RIGHT,
};

enum Enum_Chariot_Lift_Stair_State
{
    CHARIOT_LIFT_STAIR_IDLE = 0,
    CHARIOT_LIFT_STAIR_UP_RAISE_ALL,
    CHARIOT_LIFT_STAIR_UP_DRIVE_ALL_WAIT_UP_FRONT,
    CHARIOT_LIFT_STAIR_UP_RETRACT_FRONT,
    CHARIOT_LIFT_STAIR_UP_REAR_DRIVE_WAIT_DOWN_FRONT,
    CHARIOT_LIFT_STAIR_UP_RETRACT_REAR,
    CHARIOT_LIFT_STAIR_UP_CHASSIS_WAIT_DOWN_BACK,
    CHARIOT_LIFT_STAIR_DOWN_CHASSIS_WAIT_UP_BACK,
    CHARIOT_LIFT_STAIR_DOWN_RAISE_FRONT,
    CHARIOT_LIFT_STAIR_DOWN_FRONT_DRIVE_WAIT_DOWN_BACK,
    CHARIOT_LIFT_STAIR_DOWN_RAISE_REAR,
    CHARIOT_LIFT_STAIR_DOWN_DRIVE_ALL_EXTRA,
    CHARIOT_LIFT_STAIR_DOWN_RETRACT_ALL,
    CHARIOT_LIFT_STAIR_ABORT,
    CHARIOT_LIFT_STAIR_DOWN_ATTITUDE_CORRECT,
    CHARIOT_LIFT_STAIR_UP_PRE_ATTITUDE_CORRECT,   // 上台阶前：先对正要爬的台阶立面
    CHARIOT_LIFT_STAIR_UP_ATTITUDE_CORRECT,       // 上台阶后：再校一次（与下台阶对称）
    CHARIOT_LIFT_STAIR_DOWN_PRE_ATTITUDE_CORRECT, // 下台阶前：先把车身摆正再开始
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

struct ChariotLiftToFData
{
    bool online = false;          // 最近 500ms 内是否解析到完整 TFmini-S 帧
    bool valid = false;           // 信号强度是否有效，距离 0/65532 仍通过 range_m 表示特殊状态
    float range_m = 0.0f;         // m；0cm=-inf，65532cm=+inf，信号无效=NaN
    uint16_t distance_cm = 0;
    uint16_t strength = 0;
    uint16_t temperature_raw = 0;
    uint32_t frame_count = 0;     // 成功校验的 TFmini-S 帧累计数
};

class Class_Chariot_Lift
{
public:
    Class_Motor_DM_Normal Motor_Drive_Left[CHARIOT_LIFT_MODULE_NUM];   // 左侧驱动轮电机，CAN ID 0x03
    Class_Motor_DM_Normal Motor_Drive_Right[CHARIOT_LIFT_MODULE_NUM];  // 右侧驱动轮电机，CAN ID 0x04
    Class_Motor_DM_Normal Motor_Lift[CHARIOT_LIFT_MODULE_NUM];         // 抬升 3519 电机，CAN ID 0x05
    Class_Motor_DM_Normal Auxiliary_Motor;                             // 抬升伴随辅助 DM 电机，CAN0 Tx 0x07 / Rx 0x17

    void Init(linkx_t *__LinkX_Handler);
    bool CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data, uint8_t CAN_Dlen = 8);
    bool CAN_Rx_ToF_Frame(uint32_t slave_id,
                          uint8_t CAN_Channel,
                          uint32_t CAN_ID,
                          const uint8_t *CAN_Data,
                          uint8_t CAN_Dlen = 8);

    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_2ms_Control_PeriodElapsedCallback();
    void Send_Enable_Burst();
    void Start_Stair_Up(float raise_motor_angle);
    void Start_Stair_Down(float raise_motor_angle);
    void Stop_Stair_Auto();
    void Set_Stair_Attitude_Yaw(float yaw_rad, bool valid);
    // 抬升 API 角度参数均为「电机角度」(rad)。单参版前后用同一电机角并自动镜像
    // (车尾取反)；双参版分别指定前/后电机角。内部统一换算为杆角。
    void Set_Both_Lift_Raise(float raise_motor_angle);
    void Set_Both_Lift_Raise(float front_raise_motor_angle, float rear_raise_motor_angle);
    void Set_Both_Lift_Retract();
    void Set_Both_Lift_Retract_To(float retract_motor_angle);
    void Set_Both_Lift_Retract_To(float front_retract_motor_angle, float rear_retract_motor_angle);
    bool Are_Both_Lifts_Reached(Enum_Chariot_Lift_Position_State state);

    // 辅助 DM 电机（0x07）+ 抬升协同动作（遥控 Up/Down、服务 lift_aux/raise|home）。
    // Down=抬升伸出后辅助臂外伸；Up=辅助臂归位后抬升回收。整车使能/失能时调用 Enable/Disable。
    void Enable_Auxiliary_Motor();
    void Disable_Auxiliary_Motor();
    void Cancel_Lift_Aux_Sequence();
    void Start_Lift_Aux_Raise_Sequence();
    void Start_Lift_Aux_Home_Sequence();
    void TIM_1ms_Aux_PeriodElapsedCallback();   // 1ms 节拍推进序列 + 输出辅助电机

    inline void Set_Control_Type(Enum_Chariot_Lift_Control_Type type);
    inline void Set_Front_Lift_State(Enum_Chariot_Lift_Position_State state);
    inline void Set_Rear_Lift_State(Enum_Chariot_Lift_Position_State state);
    void Set_Raise_Angle(float raise_motor_angle);
    void Set_Raise_Angle(float front_raise_motor_angle, float rear_raise_motor_angle);
    inline void Set_Diff_Drive_Enable(bool enable);
    inline void Set_Diff_Drive_Module(Enum_Chariot_Lift_Module module);
    inline void Set_Target_Diff_Command(float forward_m_s, float yaw_rad_s);
    inline void Set_Target_Diff_Command_For_Module(Enum_Chariot_Lift_Module module,
                                                   float forward_m_s,
                                                   float yaw_rad_s);
    inline void Clear_Target_Diff_Command_For_Modules();
    inline void Set_Diff_Drive_Params(Enum_Chariot_Lift_Module module,
                                      const ChariotLiftDriveParams &params);

    inline Enum_Chariot_Lift_Control_Type Get_Control_Type();
    inline bool Get_Diff_Drive_Enable();
    inline float Get_Target_Diff_Forward();
    inline float Get_Target_Diff_Yaw();
    inline ChariotLiftDriveParams Get_Diff_Drive_Params(Enum_Chariot_Lift_Module module);
    inline float Get_Target_Left_Omega(Enum_Chariot_Lift_Module module);
    inline float Get_Target_Right_Omega(Enum_Chariot_Lift_Module module);
    inline const ChariotLiftToFData &Get_ToF_Data(Enum_Chariot_Lift_ToF_Sensor sensor);
    inline bool Get_ToF_Online(Enum_Chariot_Lift_ToF_Sensor sensor);
    inline bool Get_ToF_Valid(Enum_Chariot_Lift_ToF_Sensor sensor);
    inline float Get_ToF_Range_M(Enum_Chariot_Lift_ToF_Sensor sensor);
    inline uint16_t Get_ToF_Distance_Cm(Enum_Chariot_Lift_ToF_Sensor sensor);
    inline bool Is_Stair_Auto_Active();
    inline float Get_Stair_Chassis_Forward();
    inline float Get_Stair_Chassis_Omega();
    inline Enum_Chariot_Lift_Stair_State Get_Stair_State();
    const char *Get_Stair_State_Name();
    // 下台阶姿态诊断读出：IMU 偏航、锁定目标、有效位、归一化误差(target-now)。
    inline float Get_Stair_Attitude_Yaw();
    inline float Get_Stair_Attitude_Target_Yaw();
    inline bool Get_Stair_Attitude_Yaw_Valid();
    inline bool Get_Stair_Attitude_Target_Valid();
    float Get_Stair_Yaw_Error();

protected:
    // 辅助电机 + 抬升协同动作的序列状态机。
    enum class LiftAuxSequenceState {
        IDLE,
        RAISE_WAIT_LIFT_REACHED,
        RAISE_MOVE_AUXILIARY,
        RAISE_DONE,
        HOME_WAIT_LIFT_REACHED,
        HOME_MOVE_AUXILIARY,
        HOME_DONE,
    };

    Enum_Chariot_Lift_Control_Type Control_Type = CHARIOT_LIFT_CONTROL_DISABLE;
    Enum_Chariot_Lift_Position_State Lift_State[CHARIOT_LIFT_MODULE_NUM] = {
        CHARIOT_LIFT_POSITION_RETRACT,
        CHARIOT_LIFT_POSITION_RETRACT,
    };
    bool Lift_Module_Enable[CHARIOT_LIFT_MODULE_NUM] = {false, false};

    ChariotLiftDriveParams Drive_Params[CHARIOT_LIFT_MODULE_NUM];
    ChariotLiftPositionParams Lift_Params[CHARIOT_LIFT_MODULE_NUM];

    float Smooth_Lift_Angle[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Lift_Profile_Start_Angle[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Lift_Profile_Target_Angle[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Lift_Profile_Elapsed[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Lift_Profile_Duration[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Target_Lift_Rod_Omega[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    bool Lift_Profile_Active[CHARIOT_LIFT_MODULE_NUM] = {false, false};
    float Raw_Target_Left_Omega[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Raw_Target_Right_Omega[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Target_Left_Omega[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Target_Right_Omega[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};

    bool Diff_Drive_Enable = false;
    Enum_Chariot_Lift_Module Diff_Drive_Module = CHARIOT_LIFT_MODULE_FRONT;
    float Target_Diff_Forward = 0.0f;
    float Target_Diff_Yaw = 0.0f;
    bool Module_Diff_Target_Enable[CHARIOT_LIFT_MODULE_NUM] = {false, false};
    float Module_Target_Diff_Forward[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Module_Target_Diff_Yaw[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    uint16_t drive_disable_exit_burst_ticks_ = 0;
    uint16_t control_disable_exit_burst_ticks_ = 0;
    uint16_t enable_service_tick_ = 0;

    ChariotLiftToFData ToF_Data[CHARIOT_LIFT_TOF_NUM];
    uint8_t ToF_Rx_Index[CHARIOT_LIFT_TOF_NUM] = {0, 0, 0, 0};
    uint8_t ToF_Rx_Buffer[CHARIOT_LIFT_TOF_NUM][9] = {};
    uint32_t ToF_Pre_Frame_Count[CHARIOT_LIFT_TOF_NUM] = {0, 0, 0, 0};
    uint8_t ToF_Offline_Ticks[CHARIOT_LIFT_TOF_NUM] = {0, 0, 0, 0};

    Enum_Chariot_Lift_Stair_State Stair_State = CHARIOT_LIFT_STAIR_IDLE;
    float Stair_Raise_Angle[CHARIOT_LIFT_MODULE_NUM] = {0.0f, 0.0f};
    float Stair_Chassis_Forward = 0.0f;
    float Stair_Chassis_Omega = 0.0f;
    uint32_t Stair_State_Ticks = 0;
    bool Stair_Drive_Module_Enable[CHARIOT_LIFT_MODULE_NUM] = {false, false};
    bool Stair_ToF_Reference_Valid[CHARIOT_LIFT_TOF_NUM] = {false, false, false, false};
    uint16_t Stair_ToF_Reference_Cm[CHARIOT_LIFT_TOF_NUM] = {0, 0, 0, 0};
    bool Stair_Attitude_Yaw_Valid = false;
    float Stair_Attitude_Yaw = 0.0f;
    bool Stair_Attitude_Target_Valid = false;
    float Stair_Attitude_Target_Yaw = 0.0f;
    uint32_t Stair_Attitude_Stable_Ticks = 0;

    // ---- 辅助电机（0x07）+ 抬升协同动作状态 ----
    LiftAuxSequenceState lift_aux_sequence_state_ = LiftAuxSequenceState::IDLE;
    bool aux_vehicle_enabled_ = false;          // 整车是否使能（替代原 chassis_remote_enabled_ 门控）
    float auxiliary_motor_target_angle_ = 0.0f;
    float auxiliary_motor_smooth_angle_ = 0.0f;
    float auxiliary_motor_profile_start_angle_ = 0.0f;
    float auxiliary_motor_profile_target_angle_ = 0.0f;
    float auxiliary_motor_profile_elapsed_ = 0.0f;
    float auxiliary_motor_profile_duration_ = 0.0f;
    float auxiliary_motor_profile_accel_time_ = 0.0f;
    float auxiliary_motor_profile_decel_time_ = 0.0f;
    float auxiliary_motor_profile_cruise_time_ = 0.0f;
    float auxiliary_motor_profile_peak_speed_ = 0.0f;
    float auxiliary_motor_profile_distance_ = 0.0f;
    float auxiliary_motor_profile_direction_ = 1.0f;
    float auxiliary_motor_target_omega_ = 0.0f;
    float auxiliary_motor_hold_blend_ = 0.0f;
    uint32_t auxiliary_motor_hold_ready_ticks_ = 0U;
    bool auxiliary_motor_command_enable_ = false;
    bool auxiliary_motor_profile_active_ = false;
    bool auxiliary_motor_profile_initialized_ = false;
    bool auxiliary_motor_hold_active_ = false;

    void Init_Motor_Params();
    void Parse_ToF_Byte(Enum_Chariot_Lift_ToF_Sensor sensor, uint8_t byte);
    void Update_Stair_Auto();
    void Enter_Stair_State(Enum_Chariot_Lift_Stair_State state);
    void Finish_Stair_Auto(bool retract_lift);
    void Set_Stair_Lift_Command(bool front_enable,
                                Enum_Chariot_Lift_Position_State front_state,
                                bool rear_enable,
                                Enum_Chariot_Lift_Position_State rear_state);
    void Set_Stair_Drive_Command(bool front_enable, bool rear_enable, float forward_m_s);
    void Capture_Stair_ToF_Reference(Enum_Chariot_Lift_ToF_Sensor sensor);
    bool Is_Stair_ToF_Usable(Enum_Chariot_Lift_ToF_Sensor sensor);
    bool Is_Stair_ToF_Near(Enum_Chariot_Lift_ToF_Sensor sensor);
    bool Is_Stair_ToF_Jumped(Enum_Chariot_Lift_ToF_Sensor sensor);
    bool Is_Lift_Profile_Reached(Enum_Chariot_Lift_Module module, Enum_Chariot_Lift_Position_State state);
    bool Is_Lift_Feedback_Reached(Enum_Chariot_Lift_Module module, Enum_Chariot_Lift_Position_State state);
    bool Is_Stair_State_Timed_Out(uint32_t timeout_ticks);
    void Capture_Stair_Attitude_Target();
    void Reset_Stair_Attitude_Correction();
    bool Is_Stair_Attitude_Corrected(uint32_t timeout_ticks);
    // 上/下台阶过程中的闭环偏航保持：返回把 IMU yaw 拉回锁定目标的车体偏航角速度(rad/s)。
    // 无参版用默认过程增益；带参版用指定增益/上限（混合支撑阶段用更强一组）。
    float Compute_Stair_Yaw_Hold_Omega();
    float Compute_Stair_Yaw_Hold_Omega(float kp, float max_omega);
    void Ensure_Motor_Enabled(Class_Motor_DM_Normal &motor, bool clear_disable_state);
    void Ensure_All_Motors_Enabled();
    void Set_Raise_Angle_Rod(float front_raise_rod_angle, float rear_raise_rod_angle);
    // ---- 辅助电机（0x07）协同动作内部实现 ----
    void Update_Lift_Aux_Sequence();
    void Reset_Auxiliary_Motor_Profile(float angle);
    void Start_Auxiliary_Motor_Profile(float target_angle);
    float Update_Auxiliary_Motor_Profile(float target_angle);
    bool Is_Auxiliary_Motor_Profile_At_Target() const;
    void Output_Auxiliary_Motor();
    bool Is_Auxiliary_Motor_Reached(float target_angle);
    void Reset_Lift_Profile(Enum_Chariot_Lift_Module module, float lift_rod_angle);
    void Start_Lift_Profile(Enum_Chariot_Lift_Module module, float target_lift_rod_angle);
    float Update_Lift_Profile(Enum_Chariot_Lift_Module module, float target_lift_rod_angle);
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
        {
            Smooth_Lift_Angle[i] = Motor_To_Lift_Rod_Angle(Motor_Lift[i].Get_Now_Radian());
            Lift_Profile_Start_Angle[i] = Smooth_Lift_Angle[i];
            Lift_Profile_Target_Angle[i] = Smooth_Lift_Angle[i];
            Lift_Profile_Elapsed[i] = 0.0f;
            Lift_Profile_Duration[i] = 0.0f;
            Target_Lift_Rod_Omega[i] = 0.0f;
            Lift_Profile_Active[i] = false;
        }
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
    Clear_Target_Diff_Command_For_Modules();
}

inline void Class_Chariot_Lift::Set_Target_Diff_Command_For_Module(Enum_Chariot_Lift_Module module,
                                                                   float forward_m_s,
                                                                   float yaw_rad_s)
{
    const int idx = static_cast<int>(module);
    Module_Diff_Target_Enable[idx] = true;
    Module_Target_Diff_Forward[idx] = forward_m_s;
    Module_Target_Diff_Yaw[idx] = yaw_rad_s;
}

inline void Class_Chariot_Lift::Clear_Target_Diff_Command_For_Modules()
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Module_Diff_Target_Enable[i] = false;
        Module_Target_Diff_Forward[i] = 0.0f;
        Module_Target_Diff_Yaw[i] = 0.0f;
    }
}

inline void Class_Chariot_Lift::Set_Diff_Drive_Params(Enum_Chariot_Lift_Module module,
                                                      const ChariotLiftDriveParams &params)
{
    Drive_Params[static_cast<int>(module)] = params;
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

inline ChariotLiftDriveParams Class_Chariot_Lift::Get_Diff_Drive_Params(Enum_Chariot_Lift_Module module)
{
    return Drive_Params[static_cast<int>(module)];
}

inline float Class_Chariot_Lift::Get_Target_Left_Omega(Enum_Chariot_Lift_Module module)
{
    return Target_Left_Omega[module];
}

inline float Class_Chariot_Lift::Get_Target_Right_Omega(Enum_Chariot_Lift_Module module)
{
    return Target_Right_Omega[module];
}

inline const ChariotLiftToFData &Class_Chariot_Lift::Get_ToF_Data(Enum_Chariot_Lift_ToF_Sensor sensor)
{
    return ToF_Data[static_cast<int>(sensor)];
}

inline bool Class_Chariot_Lift::Get_ToF_Online(Enum_Chariot_Lift_ToF_Sensor sensor)
{
    return ToF_Data[static_cast<int>(sensor)].online;
}

inline bool Class_Chariot_Lift::Get_ToF_Valid(Enum_Chariot_Lift_ToF_Sensor sensor)
{
    return ToF_Data[static_cast<int>(sensor)].valid;
}

inline float Class_Chariot_Lift::Get_ToF_Range_M(Enum_Chariot_Lift_ToF_Sensor sensor)
{
    return ToF_Data[static_cast<int>(sensor)].range_m;
}

inline uint16_t Class_Chariot_Lift::Get_ToF_Distance_Cm(Enum_Chariot_Lift_ToF_Sensor sensor)
{
    return ToF_Data[static_cast<int>(sensor)].distance_cm;
}

inline bool Class_Chariot_Lift::Is_Stair_Auto_Active()
{
    return Stair_State != CHARIOT_LIFT_STAIR_IDLE &&
           Stair_State != CHARIOT_LIFT_STAIR_ABORT;
}

inline float Class_Chariot_Lift::Get_Stair_Chassis_Forward()
{
    return Stair_Chassis_Forward;
}

inline float Class_Chariot_Lift::Get_Stair_Chassis_Omega()
{
    return Stair_Chassis_Omega;
}

inline float Class_Chariot_Lift::Get_Stair_Attitude_Yaw()
{
    return Stair_Attitude_Yaw;
}

inline float Class_Chariot_Lift::Get_Stair_Attitude_Target_Yaw()
{
    return Stair_Attitude_Target_Yaw;
}

inline bool Class_Chariot_Lift::Get_Stair_Attitude_Yaw_Valid()
{
    return Stair_Attitude_Yaw_Valid;
}

inline bool Class_Chariot_Lift::Get_Stair_Attitude_Target_Valid()
{
    return Stair_Attitude_Target_Valid;
}

inline Enum_Chariot_Lift_Stair_State Class_Chariot_Lift::Get_Stair_State()
{
    return Stair_State;
}

#endif // CRT_LIFT_H

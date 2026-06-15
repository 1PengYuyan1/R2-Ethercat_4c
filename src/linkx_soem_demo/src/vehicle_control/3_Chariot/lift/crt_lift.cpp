#include "crt_lift.h"

#include "math.h"

#include <cmath>
#include <limits>

namespace
{
constexpr int kLiftHoldPointCount = 5;
constexpr float kLiftControlDtS = 0.002f;
constexpr float kLiftMotorToRodRatio = 3.0f;
constexpr float kLiftSCurvePeakVelocityScale = 1.875f;
constexpr uint8_t kLiftToFCanChannel = 2;
constexpr uint8_t kLiftToFOfflineTicks = 5;  // 5 * 100ms
constexpr uint16_t kLiftToFInvalidStrength = 65535U;
constexpr uint16_t kLiftToFTooFarCm = 65532U;
constexpr uint16_t kStairToFNearCm = 10U;
constexpr uint16_t kStairToFJumpCm = 5U;
constexpr float kStairLiftReachedTolerance = 0.10f;
constexpr float kStairLiftDriveForwardMps = 1.50f;
constexpr float kStairChassisForwardMps = 1.2f;
constexpr uint32_t kStairLiftTimeoutTicks = 2500U;     // 5s at 2ms
constexpr uint32_t kStairSensorTimeoutTicks = 5000U;   // 10s at 2ms
constexpr uint32_t kStairExtraDriveTicks = 500U;       // 1s at 2ms
constexpr uint32_t kStairDownAttitudeTimeoutTicks = 2500U;  // 5s at 2ms
constexpr uint32_t kStairDownAttitudeStableTicks = 150U;     // 0.3s at 2ms
constexpr float kStairDownAttitudeToleranceRad = 0.02f;
constexpr float kStairDownAttitudeKp = 3.6f;
constexpr float kStairDownAttitudeMaxYawRadS = 0.90f;
constexpr float kStairDownAttitudeMinYawRadS = 0.16f;
constexpr float kStairDownAttitudeMinYawErrorRad = 0.06f;

struct LiftHoldFeedforwardTable
{
    float rod_angle[kLiftHoldPointCount];
    float motor_torque_nm[kLiftHoldPointCount];
};

struct LiftToFCanMap
{
    uint32_t id;
    Enum_Chariot_Lift_ToF_Sensor sensor;
};

enum StairTransition
{
    STAIR_TRANSITION_NONE = 0,
    STAIR_TRANSITION_FRONT_RAISED,
    STAIR_TRANSITION_REAR_RAISED,
    STAIR_TRANSITION_BOTH_RAISED,
    STAIR_TRANSITION_FRONT_RETRACTED,
    STAIR_TRANSITION_REAR_RETRACTED,
    STAIR_TRANSITION_BOTH_RETRACTED,
    STAIR_TRANSITION_TOF_NEAR_OR_JUMP,
    STAIR_TRANSITION_TOF_JUMP,
    STAIR_TRANSITION_ATTITUDE_CORRECTED,
    STAIR_TRANSITION_TIMER,
};

struct StairStateConfig
{
    Enum_Chariot_Lift_Stair_State state;
    const char *name;
    bool front_lift_enable;
    Enum_Chariot_Lift_Position_State front_lift_state;
    bool rear_lift_enable;
    Enum_Chariot_Lift_Position_State rear_lift_state;
    bool front_drive_enable;
    bool rear_drive_enable;
    float lift_drive_forward_m_s;
    float chassis_forward_m_s;
    StairTransition transition;
    Enum_Chariot_Lift_ToF_Sensor sensor;
    Enum_Chariot_Lift_Stair_State next_state;
    uint32_t timeout_ticks;
    bool capture_sensor_reference;
    bool finish_on_transition;
};

// DM3519 motor-side hold torque feedforward measured by lift_raise_drive_test --suite identify.
constexpr LiftHoldFeedforwardTable kLiftHoldFeedforward[CHARIOT_LIFT_MODULE_NUM] = {
    {
        {-15.0f, -11.5f, -8.0f, -4.5f, -1.0f},
        {-1.0905f, -0.5794f, -0.5545f, -0.6979f, 0.3695f},
    },
    {
        {-15.0f, -11.5f, -8.0f, -4.5f, -1.0f},
        {-0.9960f, -0.6412f, -0.5545f, -0.5803f, 0.4385f},
    },
};

constexpr LiftToFCanMap kLiftToFCanMap[CHARIOT_LIFT_TOF_NUM] = {
    {0x001U, CHARIOT_LIFT_TOF_UP_FRONT},
    {0x002U, CHARIOT_LIFT_TOF_UP_BACK},
    {0x003U, CHARIOT_LIFT_TOF_DOWN_FRONT},
    {0x004U, CHARIOT_LIFT_TOF_DOWN_BACK},
};

constexpr StairStateConfig kStairStateConfigs[] = {
    {
        CHARIOT_LIFT_STAIR_UP_RAISE_ALL,
        "UP_RAISE_ALL",
        true, CHARIOT_LIFT_POSITION_RAISE,
        true, CHARIOT_LIFT_POSITION_RAISE,
        false, false, 0.0f, 0.0f,
        STAIR_TRANSITION_BOTH_RAISED,
        CHARIOT_LIFT_TOF_UP_FRONT,
        CHARIOT_LIFT_STAIR_UP_DRIVE_ALL_WAIT_UP_FRONT,
        kStairLiftTimeoutTicks,
        false,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_UP_DRIVE_ALL_WAIT_UP_FRONT,
        "UP_DRIVE_ALL_WAIT_UP_FRONT",
        true, CHARIOT_LIFT_POSITION_RAISE,
        true, CHARIOT_LIFT_POSITION_RAISE,
        true, true, kStairLiftDriveForwardMps, 0.0f,
        STAIR_TRANSITION_TOF_NEAR_OR_JUMP,
        CHARIOT_LIFT_TOF_UP_FRONT,
        CHARIOT_LIFT_STAIR_UP_RETRACT_FRONT,
        kStairSensorTimeoutTicks,
        true,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_UP_RETRACT_FRONT,
        "UP_RETRACT_FRONT",
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, CHARIOT_LIFT_POSITION_RAISE,
        false, false, 0.0f, 0.0f,
        STAIR_TRANSITION_FRONT_RETRACTED,
        CHARIOT_LIFT_TOF_UP_FRONT,
        CHARIOT_LIFT_STAIR_UP_REAR_DRIVE_WAIT_DOWN_FRONT,
        kStairLiftTimeoutTicks,
        false,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_UP_REAR_DRIVE_WAIT_DOWN_FRONT,
        "UP_REAR_DRIVE_WAIT_DOWN_FRONT",
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, CHARIOT_LIFT_POSITION_RAISE,
        false, true, kStairLiftDriveForwardMps, kStairChassisForwardMps,
        STAIR_TRANSITION_TOF_NEAR_OR_JUMP,
        CHARIOT_LIFT_TOF_DOWN_FRONT,
        CHARIOT_LIFT_STAIR_UP_RETRACT_REAR,
        kStairSensorTimeoutTicks,
        true,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_UP_RETRACT_REAR,
        "UP_RETRACT_REAR",
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, CHARIOT_LIFT_POSITION_RETRACT,
        false, false, 0.0f, 0.0f,
        STAIR_TRANSITION_REAR_RETRACTED,
        CHARIOT_LIFT_TOF_DOWN_FRONT,
        CHARIOT_LIFT_STAIR_UP_CHASSIS_WAIT_DOWN_BACK,
        kStairLiftTimeoutTicks,
        false,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_UP_CHASSIS_WAIT_DOWN_BACK,
        "UP_CHASSIS_WAIT_DOWN_BACK",
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, CHARIOT_LIFT_POSITION_RETRACT,
        false, false, 0.0f, kStairChassisForwardMps,
        STAIR_TRANSITION_TOF_NEAR_OR_JUMP,
        CHARIOT_LIFT_TOF_DOWN_BACK,
        CHARIOT_LIFT_STAIR_IDLE,
        kStairSensorTimeoutTicks,
        true,
        true,
    },
    {
        CHARIOT_LIFT_STAIR_DOWN_CHASSIS_WAIT_UP_BACK,
        "DOWN_CHASSIS_WAIT_UP_BACK",
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, CHARIOT_LIFT_POSITION_RETRACT,
        false, false, 0.0f, kStairChassisForwardMps,
        STAIR_TRANSITION_TOF_JUMP,
        CHARIOT_LIFT_TOF_UP_BACK,
        CHARIOT_LIFT_STAIR_DOWN_RAISE_FRONT,
        kStairSensorTimeoutTicks,
        true,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_DOWN_RAISE_FRONT,
        "DOWN_RAISE_FRONT",
        true, CHARIOT_LIFT_POSITION_RAISE,
        true, CHARIOT_LIFT_POSITION_RETRACT,
        false, false, 0.0f, 0.0f,
        STAIR_TRANSITION_FRONT_RAISED,
        CHARIOT_LIFT_TOF_UP_BACK,
        CHARIOT_LIFT_STAIR_DOWN_FRONT_DRIVE_WAIT_DOWN_BACK,
        kStairLiftTimeoutTicks,
        false,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_DOWN_FRONT_DRIVE_WAIT_DOWN_BACK,
        "DOWN_FRONT_DRIVE_WAIT_DOWN_BACK",
        true, CHARIOT_LIFT_POSITION_RAISE,
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, false, kStairLiftDriveForwardMps, kStairChassisForwardMps,
        STAIR_TRANSITION_TOF_JUMP,
        CHARIOT_LIFT_TOF_DOWN_BACK,
        CHARIOT_LIFT_STAIR_DOWN_RAISE_REAR,
        kStairSensorTimeoutTicks,
        true,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_DOWN_RAISE_REAR,
        "DOWN_RAISE_REAR",
        true, CHARIOT_LIFT_POSITION_RAISE,
        true, CHARIOT_LIFT_POSITION_RAISE,
        false, false, 0.0f, 0.0f,
        STAIR_TRANSITION_REAR_RAISED,
        CHARIOT_LIFT_TOF_DOWN_BACK,
        CHARIOT_LIFT_STAIR_DOWN_DRIVE_ALL_EXTRA,
        kStairLiftTimeoutTicks,
        false,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_DOWN_DRIVE_ALL_EXTRA,
        "DOWN_DRIVE_ALL_EXTRA",
        true, CHARIOT_LIFT_POSITION_RAISE,
        true, CHARIOT_LIFT_POSITION_RAISE,
        true, true, kStairLiftDriveForwardMps, 0.0f,
        STAIR_TRANSITION_TIMER,
        CHARIOT_LIFT_TOF_DOWN_BACK,
        CHARIOT_LIFT_STAIR_DOWN_RETRACT_ALL,
        kStairExtraDriveTicks,
        false,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_DOWN_RETRACT_ALL,
        "DOWN_RETRACT_ALL",
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, CHARIOT_LIFT_POSITION_RETRACT,
        false, false, 0.0f, 0.0f,
        STAIR_TRANSITION_BOTH_RETRACTED,
        CHARIOT_LIFT_TOF_DOWN_BACK,
        CHARIOT_LIFT_STAIR_DOWN_ATTITUDE_CORRECT,
        kStairLiftTimeoutTicks,
        false,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_DOWN_ATTITUDE_CORRECT,
        "DOWN_ATTITUDE_CORRECT",
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, CHARIOT_LIFT_POSITION_RETRACT,
        false, false, 0.0f, 0.0f,
        STAIR_TRANSITION_ATTITUDE_CORRECTED,
        CHARIOT_LIFT_TOF_DOWN_BACK,
        CHARIOT_LIFT_STAIR_IDLE,
        kStairDownAttitudeTimeoutTicks,
        false,
        true,
    },
    {
        CHARIOT_LIFT_STAIR_ABORT,
        "ABORT",
        false, CHARIOT_LIFT_POSITION_RETRACT,
        false, CHARIOT_LIFT_POSITION_RETRACT,
        false, false, 0.0f, 0.0f,
        STAIR_TRANSITION_NONE,
        CHARIOT_LIFT_TOF_UP_FRONT,
        CHARIOT_LIFT_STAIR_ABORT,
        0U,
        false,
        false,
    },
};

const StairStateConfig *Find_Stair_State_Config(Enum_Chariot_Lift_Stair_State state)
{
    for (const auto &config : kStairStateConfigs)
    {
        if (config.state == state)
            return &config;
    }
    return nullptr;
}

bool Match_Lift_ToF_Sensor(uint32_t can_id_std, Enum_Chariot_Lift_ToF_Sensor &sensor)
{
    for (const auto &item : kLiftToFCanMap)
    {
        if (item.id == can_id_std)
        {
            sensor = item.sensor;
            return true;
        }
    }
    return false;
}

float Interpolate_Lift_Hold_Torque(Enum_Chariot_Lift_Module module, float rod_angle)
{
    const LiftHoldFeedforwardTable &table = kLiftHoldFeedforward[static_cast<int>(module)];

    if (rod_angle <= table.rod_angle[0])
        return table.motor_torque_nm[0];
    if (rod_angle >= table.rod_angle[kLiftHoldPointCount - 1])
        return table.motor_torque_nm[kLiftHoldPointCount - 1];

    for (int i = 1; i < kLiftHoldPointCount; ++i)
    {
        if (rod_angle <= table.rod_angle[i])
        {
            const float span = table.rod_angle[i] - table.rod_angle[i - 1];
            const float ratio = (fabsf(span) > 1e-5f) ?
                (rod_angle - table.rod_angle[i - 1]) / span :
                0.0f;
            return table.motor_torque_nm[i - 1] +
                   ratio * (table.motor_torque_nm[i] - table.motor_torque_nm[i - 1]);
        }
    }

    return table.motor_torque_nm[kLiftHoldPointCount - 1];
}

float SCurve_Fraction(float u)
{
    if (u < 0.0f)
        u = 0.0f;
    if (u > 1.0f)
        u = 1.0f;

    return (10.0f * u * u * u) -
           (15.0f * u * u * u * u) +
           (6.0f * u * u * u * u * u);
}

float SCurve_Fraction_Derivative(float u)
{
    if (u < 0.0f)
        u = 0.0f;
    if (u > 1.0f)
        u = 1.0f;

    return (30.0f * u * u) -
           (60.0f * u * u * u) +
           (30.0f * u * u * u * u);
}
}  // namespace

/**
 * @brief 初始化车头和车尾两个抬升模块，并绑定 LinkX CAN 通道。
 *
 * 新车头为原车尾，因此 CAN0 是车头抬升模块，CAN1 是车尾抬升模块。
 */
void Class_Chariot_Lift::Init(linkx_t *__LinkX_Handler)
{
    Init_Motor_Params();

    Motor_Drive_Left[CHARIOT_LIFT_MODULE_FRONT].Init(__LinkX_Handler, 0, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Drive_Right[CHARIOT_LIFT_MODULE_FRONT].Init(__LinkX_Handler, 0, 0x14, 0x04, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Lift[CHARIOT_LIFT_MODULE_FRONT].Init(__LinkX_Handler, 0, 0x15, 0x05, Motor_DM_Control_Method_NORMAL_MIT, 62.8f, 395.0f, 7.8f, 9.2f);

    Motor_Drive_Left[CHARIOT_LIFT_MODULE_REAR].Init(__LinkX_Handler, 1, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Drive_Right[CHARIOT_LIFT_MODULE_REAR].Init(__LinkX_Handler, 1, 0x14, 0x04, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 150.0f, 5.0f, 12.0f);
    Motor_Lift[CHARIOT_LIFT_MODULE_REAR].Init(__LinkX_Handler, 1, 0x15, 0x05, Motor_DM_Control_Method_NORMAL_MIT, 62.8f, 395.0f, 7.8f, 9.2f);

    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; i++)
    {
        Motor_Drive_Left[i].Set_Use_FDCAN(true);
        Motor_Drive_Right[i].Set_Use_FDCAN(true);
        Motor_Lift[i].Set_Use_FDCAN(true);

        Motor_Drive_Left[i].Set_Force_Output_Without_Feedback(false);
        Motor_Drive_Right[i].Set_Force_Output_Without_Feedback(false);
        Motor_Lift[i].Set_Force_Output_Without_Feedback(false);
    }

    for (int i = 0; i < CHARIOT_LIFT_TOF_NUM; ++i)
    {
        ToF_Data[i] = ChariotLiftToFData{};
        ToF_Rx_Index[i] = 0;
        ToF_Pre_Frame_Count[i] = 0;
        ToF_Offline_Ticks[i] = kLiftToFOfflineTicks;
    }
}

/**
 * @brief 抬升模块参数初始化（默认值，可在外部覆盖）
 */
void Class_Chariot_Lift::Init_Motor_Params()
{
    Drive_Params[CHARIOT_LIFT_MODULE_FRONT] = {
        .wheel_radius           = 0.0761f,
        .track_width            = 0.420f,
        .max_forward_speed      = 1.0f,
        .max_yaw_omega          = 2.0f,
        .max_wheel_omega        = 80.0f,
        .wheel_kp               = 0.0f,
        .wheel_kd               = 1.2f,
        .wheel_deadzone         = 0.05f,
        .left_direction         = 1.0f,
        .right_direction        = -1.0f,
        .left_speed_correction  = 1.0f,
        .right_speed_correction = 1.0f,
        .accel_limit            = 300.0f,
        .decel_limit            = 600.0f,
    };

    Drive_Params[CHARIOT_LIFT_MODULE_REAR] = {
        .wheel_radius           = 0.0761f,
        .track_width            = 0.420f,
        .max_forward_speed      = 1.0f,
        .max_yaw_omega          = 2.0f,
        .max_wheel_omega        = 80.0f,
        .wheel_kp               = 0.0f,
        .wheel_kd               = 1.2f,
        .wheel_deadzone         = 0.05f,
        .left_direction         = 1.0f,
        .right_direction        = -1.0f,
        .left_speed_correction  = 1.0f,
        .right_speed_correction = 1.0f,
        .accel_limit            = 300.0f,
        .decel_limit            = 600.0f,
    };

    Lift_Params[CHARIOT_LIFT_MODULE_FRONT] = {
        .retract_angle = -0.1f,
        .raise_angle   = -8.0f,
        .max_speed     = 13.0f,
        .kp            = 20.0f,
        .kd            = 1.2f,
    };

    Lift_Params[CHARIOT_LIFT_MODULE_REAR] = {
        .retract_angle = -0.1f,
        .raise_angle   = -8.0f,
        .max_speed     = 13.0f,
        .kp            = 20.0f,
        .kd            = 1.2f,
    };

    Smooth_Lift_Angle[CHARIOT_LIFT_MODULE_FRONT] =
        Lift_Params[CHARIOT_LIFT_MODULE_FRONT].retract_angle;
    Smooth_Lift_Angle[CHARIOT_LIFT_MODULE_REAR] =
        Lift_Params[CHARIOT_LIFT_MODULE_REAR].retract_angle;

    Reset_Lift_Profile(CHARIOT_LIFT_MODULE_FRONT,
                       Smooth_Lift_Angle[CHARIOT_LIFT_MODULE_FRONT]);
    Reset_Lift_Profile(CHARIOT_LIFT_MODULE_REAR,
                       Smooth_Lift_Angle[CHARIOT_LIFT_MODULE_REAR]);
}

/**
 * @brief 将收到的一帧 CAN 报文路由到对应模块内匹配的电机。
 */
bool Class_Chariot_Lift::CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data, uint8_t CAN_Dlen)
{
    const uint32_t can_id_std = CAN_ID & 0x7FFU;
    Enum_Chariot_Lift_Module module;

    if (CAN_Rx_ToF(CAN_Channel, can_id_std, CAN_Data, CAN_Dlen))
        return true;

    /* 新车头为原车尾：CAN0=车头抬升模块，CAN1=车尾抬升模块 */
    if (CAN_Channel == 0U)
    {
        module = CHARIOT_LIFT_MODULE_FRONT;
    }
    else if (CAN_Channel == 1U)
    {
        module = CHARIOT_LIFT_MODULE_REAR;
    }
    else
    {
        return false;
    }

    if (can_id_std == Motor_Drive_Left[module].DM_CAN_Rx_ID)
    {
        Motor_Drive_Left[module].CAN_RxCpltCallback(CAN_Data);
        return true;
    }
    if (can_id_std == Motor_Drive_Right[module].DM_CAN_Rx_ID)
    {
        Motor_Drive_Right[module].CAN_RxCpltCallback(CAN_Data);
        return true;
    }
    if (can_id_std == Motor_Lift[module].DM_CAN_Rx_ID)
    {
        Motor_Lift[module].CAN_RxCpltCallback(CAN_Data);
        return true;
    }

    return false;
}

bool Class_Chariot_Lift::CAN_Rx_ToF(uint8_t CAN_Channel,
                                    uint32_t CAN_ID,
                                    const uint8_t *CAN_Data,
                                    uint8_t CAN_Dlen)
{
    if (CAN_Channel != kLiftToFCanChannel || CAN_Data == nullptr || CAN_Dlen == 0)
        return false;

    Enum_Chariot_Lift_ToF_Sensor sensor = CHARIOT_LIFT_TOF_UP_FRONT;
    if (!Match_Lift_ToF_Sensor(CAN_ID, sensor))
        return false;

    const uint8_t n = (CAN_Dlen > 64U) ? 64U : CAN_Dlen;
    for (uint8_t i = 0; i < n; ++i)
        Parse_ToF_Byte(sensor, CAN_Data[i]);

    return true;
}

void Class_Chariot_Lift::Parse_ToF_Byte(Enum_Chariot_Lift_ToF_Sensor sensor, uint8_t byte)
{
    const int idx = static_cast<int>(sensor);
    uint8_t &rx_index = ToF_Rx_Index[idx];
    uint8_t *rx_buffer = ToF_Rx_Buffer[idx];

    if (rx_index == 0U)
    {
        if (byte == 0x59U)
        {
            rx_buffer[0] = byte;
            rx_index = 1U;
        }
        return;
    }

    if (rx_index == 1U && byte != 0x59U)
    {
        rx_index = 0U;
        return;
    }

    rx_buffer[rx_index++] = byte;
    if (rx_index < 9U)
        return;

    rx_index = 0U;

    uint16_t checksum = 0;
    for (int i = 0; i < 8; ++i)
        checksum = static_cast<uint16_t>(checksum + rx_buffer[i]);

    if (static_cast<uint8_t>(checksum & 0xFFU) != rx_buffer[8])
        return;

    ChariotLiftToFData &tof = ToF_Data[idx];
    tof.distance_cm = static_cast<uint16_t>((rx_buffer[3] << 8) | rx_buffer[2]);
    tof.strength = static_cast<uint16_t>((rx_buffer[5] << 8) | rx_buffer[4]);
    tof.temperature_raw = static_cast<uint16_t>((rx_buffer[7] << 8) | rx_buffer[6]);
    tof.valid = (tof.strength >= 100U && tof.strength != kLiftToFInvalidStrength);
    tof.frame_count++;
    tof.online = true;
    ToF_Offline_Ticks[idx] = 0;

    if (tof.distance_cm == 0U)
    {
        tof.range_m = -std::numeric_limits<float>::infinity();
    }
    else if (tof.distance_cm == kLiftToFTooFarCm)
    {
        tof.range_m = std::numeric_limits<float>::infinity();
    }
    else if (!tof.valid)
    {
        tof.range_m = std::numeric_limits<float>::quiet_NaN();
    }
    else
    {
        tof.range_m = static_cast<float>(tof.distance_cm) * 0.01f;
    }
}

/**
 * @brief 刷新电机在线状态，并在抬升控制启用时保持电机使能。
 */
void Class_Chariot_Lift::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Motor_Drive_Left[i].TIM_Alive_PeriodElapsedCallback();
        Motor_Drive_Right[i].TIM_Alive_PeriodElapsedCallback();
        Motor_Lift[i].TIM_Alive_PeriodElapsedCallback();
    }

    for (int i = 0; i < CHARIOT_LIFT_TOF_NUM; ++i)
    {
        if (ToF_Data[i].frame_count != ToF_Pre_Frame_Count[i])
        {
            ToF_Data[i].online = true;
            ToF_Offline_Ticks[i] = 0;
        }
        else if (ToF_Offline_Ticks[i] < kLiftToFOfflineTicks)
        {
            ++ToF_Offline_Ticks[i];
            if (ToF_Offline_Ticks[i] >= kLiftToFOfflineTicks)
                ToF_Data[i].online = false;
        }
        else
        {
            ToF_Data[i].online = false;
        }

        ToF_Pre_Frame_Count[i] = ToF_Data[i].frame_count;
    }

    if (Control_Type == CHARIOT_LIFT_CONTROL_ENABLE)
    {
        Ensure_All_Motors_Enabled();
    }
}

/**
 * @brief 2 ms 抬升主循环：START 后保持电机使能，机构按钮按下后才进入动作控制。
 */
void Class_Chariot_Lift::TIM_2ms_Control_PeriodElapsedCallback()
{
    switch (Control_Type)
    {
        case CHARIOT_LIFT_CONTROL_DISABLE:
        {
            Stop_Stair_Auto();
            Disable_All_Motors(control_disable_exit_burst_ticks_ > 0);
            if (control_disable_exit_burst_ticks_ > 0)
                --control_disable_exit_burst_ticks_;
            Reset_Drive_State();
            return;
        }

        case CHARIOT_LIFT_CONTROL_ENABLE:
        {
            Ensure_All_Motors_Enabled();
            Update_Stair_Auto();

            for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
            {
                if (Lift_Module_Enable[i])
                    Output_Lift_Motor(static_cast<Enum_Chariot_Lift_Module>(i));
            }

            Kinematics_Diff_Resolution();
            Apply_Drive_Trapezoid_Profile();

            if (Diff_Drive_Enable || drive_disable_exit_burst_ticks_ > 0)
            {
                for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
                    Output_Drive_Motor(static_cast<Enum_Chariot_Lift_Module>(i));
            }
            else
            {
                Reset_Drive_State();
            }

            if (drive_disable_exit_burst_ticks_ > 0)
                --drive_disable_exit_burst_ticks_;
            break;
        }
    }
}

/**
 * @brief 让单台 DM 保持 Enter/使能态；不下发 MIT 控制量。
 */
void Class_Chariot_Lift::Ensure_Motor_Enabled(Class_Motor_DM_Normal &motor, bool clear_disable_state)
{
    const auto control_status = motor.Get_Now_Control_Status();
    if (control_status == Motor_DM_Control_Status_ENABLE)
        return;

    if (control_status == Motor_DM_Control_Status_DISABLE)
    {
        if (clear_disable_state)
            motor.CAN_Send_Clear_Error();
        else
            motor.CAN_Send_Enter();
    }
    else
    {
        motor.CAN_Send_Clear_Error();
    }
}

/**
 * @brief START 后整车使能：六台抬升相关电机都先进入电机使能态。
 */
void Class_Chariot_Lift::Ensure_All_Motors_Enabled()
{
    const int motor_slot = enable_service_tick_ % 3;
    const bool clear_disable_state = ((enable_service_tick_ / 3) % 2) == 0;

    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        switch (motor_slot)
        {
            case 0:
                Ensure_Motor_Enabled(Motor_Drive_Left[i], clear_disable_state);
                break;
            case 1:
                Ensure_Motor_Enabled(Motor_Drive_Right[i], clear_disable_state);
                break;
            default:
                Ensure_Motor_Enabled(Motor_Lift[i], clear_disable_state);
                break;
        }
    }

    ++enable_service_tick_;
}

/**
 * @brief START 上升沿立即发送一次六电机 Enter，不进入任何动作控制。
 */
void Class_Chariot_Lift::Send_Enable_Burst()
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Motor_Drive_Left[i].CAN_Send_Enter();
        Motor_Drive_Right[i].CAN_Send_Enter();
        Motor_Lift[i].CAN_Send_Enter();
    }
}

void Class_Chariot_Lift::Start_Stair_Up(float raise_angle)
{
    Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    Lift_Params[CHARIOT_LIFT_MODULE_FRONT].raise_angle = raise_angle;
    Lift_Params[CHARIOT_LIFT_MODULE_REAR].raise_angle = raise_angle;
    Stair_Raise_Angle = raise_angle;
    Reset_Stair_Attitude_Correction();
    Enter_Stair_State(CHARIOT_LIFT_STAIR_UP_RAISE_ALL);
}

void Class_Chariot_Lift::Start_Stair_Down(float raise_angle)
{
    Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    Lift_Params[CHARIOT_LIFT_MODULE_FRONT].raise_angle = raise_angle;
    Lift_Params[CHARIOT_LIFT_MODULE_REAR].raise_angle = raise_angle;
    Stair_Raise_Angle = raise_angle;
    Capture_Stair_Attitude_Target();
    Enter_Stair_State(CHARIOT_LIFT_STAIR_DOWN_CHASSIS_WAIT_UP_BACK);
}

void Class_Chariot_Lift::Stop_Stair_Auto()
{
    Stair_State = CHARIOT_LIFT_STAIR_IDLE;
    Stair_State_Ticks = 0;
    Stair_Chassis_Forward = 0.0f;
    Stair_Chassis_Omega = 0.0f;
    Set_Stair_Drive_Command(false, false, 0.0f);
    Reset_Stair_Attitude_Correction();
}

void Class_Chariot_Lift::Set_Stair_Attitude_Yaw(float yaw_rad, bool valid)
{
    if (valid && std::isfinite(yaw_rad))
    {
        Stair_Attitude_Yaw = Math_Modulus_Normalization(yaw_rad, 2.0f * PI);
        Stair_Attitude_Yaw_Valid = true;
    }
    else
    {
        Stair_Attitude_Yaw_Valid = false;
    }
}

void Class_Chariot_Lift::Set_Both_Lift_Raise(float raise_angle)
{
    Stop_Stair_Auto();
    Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    Set_Raise_Angle(raise_angle);
    Set_Stair_Lift_Command(true, CHARIOT_LIFT_POSITION_RAISE,
                           true, CHARIOT_LIFT_POSITION_RAISE);
}

void Class_Chariot_Lift::Set_Both_Lift_Raise_By_Motor_Angle(float lift_motor_angle)
{
    Set_Both_Lift_Raise(Motor_To_Lift_Rod_Angle(lift_motor_angle));
}

void Class_Chariot_Lift::Set_Both_Lift_Retract()
{
    Stop_Stair_Auto();
    Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    Set_Stair_Lift_Command(true, CHARIOT_LIFT_POSITION_RETRACT,
                           true, CHARIOT_LIFT_POSITION_RETRACT);
}

void Class_Chariot_Lift::Set_Both_Lift_Retract_To(float retract_angle)
{
    Lift_Params[CHARIOT_LIFT_MODULE_FRONT].retract_angle = retract_angle;
    Lift_Params[CHARIOT_LIFT_MODULE_REAR].retract_angle = retract_angle;
    Set_Both_Lift_Retract();
}

bool Class_Chariot_Lift::Are_Both_Lifts_Reached(Enum_Chariot_Lift_Position_State state)
{
    return Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_FRONT, state) &&
           Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_REAR, state) &&
           Is_Lift_Feedback_Reached(CHARIOT_LIFT_MODULE_FRONT, state) &&
           Is_Lift_Feedback_Reached(CHARIOT_LIFT_MODULE_REAR, state);
}

void Class_Chariot_Lift::Update_Stair_Auto()
{
    if (Stair_State == CHARIOT_LIFT_STAIR_IDLE)
        return;

    if (Stair_State == CHARIOT_LIFT_STAIR_ABORT)
    {
        Stair_Chassis_Forward = 0.0f;
        Stair_Chassis_Omega = 0.0f;
        Set_Stair_Drive_Command(false, false, 0.0f);
        return;
    }

    ++Stair_State_Ticks;

    const StairStateConfig *config = Find_Stair_State_Config(Stair_State);
    if (config == nullptr)
    {
        Enter_Stair_State(CHARIOT_LIFT_STAIR_ABORT);
        return;
    }

    Set_Stair_Lift_Command(config->front_lift_enable,
                           config->front_lift_state,
                           config->rear_lift_enable,
                           config->rear_lift_state);
    Set_Stair_Drive_Command(config->front_drive_enable,
                            config->rear_drive_enable,
                            config->lift_drive_forward_m_s);
    Stair_Chassis_Forward = config->chassis_forward_m_s;
    Stair_Chassis_Omega = 0.0f;

    bool transition_ready = false;
    switch (config->transition)
    {
        case STAIR_TRANSITION_FRONT_RAISED:
            transition_ready = Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_FRONT,
                                                       CHARIOT_LIFT_POSITION_RAISE);
            break;
        case STAIR_TRANSITION_REAR_RAISED:
            transition_ready = Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_REAR,
                                                       CHARIOT_LIFT_POSITION_RAISE);
            break;
        case STAIR_TRANSITION_BOTH_RAISED:
            transition_ready =
                Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_FRONT, CHARIOT_LIFT_POSITION_RAISE) &&
                Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_REAR, CHARIOT_LIFT_POSITION_RAISE);
            break;
        case STAIR_TRANSITION_FRONT_RETRACTED:
            transition_ready = Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_FRONT,
                                                       CHARIOT_LIFT_POSITION_RETRACT);
            break;
        case STAIR_TRANSITION_REAR_RETRACTED:
            transition_ready = Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_REAR,
                                                       CHARIOT_LIFT_POSITION_RETRACT);
            break;
        case STAIR_TRANSITION_BOTH_RETRACTED:
            transition_ready =
                Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_FRONT, CHARIOT_LIFT_POSITION_RETRACT) &&
                Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_REAR, CHARIOT_LIFT_POSITION_RETRACT);
            break;
        case STAIR_TRANSITION_TOF_NEAR_OR_JUMP:
            transition_ready = Is_Stair_ToF_Near(config->sensor) ||
                               Is_Stair_ToF_Jumped(config->sensor);
            break;
        case STAIR_TRANSITION_TOF_JUMP:
            transition_ready = Is_Stair_ToF_Jumped(config->sensor);
            break;
        case STAIR_TRANSITION_ATTITUDE_CORRECTED:
            transition_ready = Is_Stair_Attitude_Corrected(config->timeout_ticks);
            break;
        case STAIR_TRANSITION_TIMER:
            transition_ready = Is_Stair_State_Timed_Out(config->timeout_ticks);
            break;
        case STAIR_TRANSITION_NONE:
        default:
            transition_ready = false;
            break;
    }

    if (transition_ready)
    {
        if (config->finish_on_transition)
            Finish_Stair_Auto(true);
        else
            Enter_Stair_State(config->next_state);
        return;
    }

    if (config->transition != STAIR_TRANSITION_TIMER &&
        config->timeout_ticks > 0U &&
        Is_Stair_State_Timed_Out(config->timeout_ticks))
    {
        Enter_Stair_State(CHARIOT_LIFT_STAIR_ABORT);
    }
}

void Class_Chariot_Lift::Enter_Stair_State(Enum_Chariot_Lift_Stair_State state)
{
    Stair_State = state;
    Stair_State_Ticks = 0;
    Stair_Chassis_Forward = 0.0f;
    Stair_Chassis_Omega = 0.0f;
    Set_Stair_Drive_Command(false, false, 0.0f);

    for (int i = 0; i < CHARIOT_LIFT_TOF_NUM; ++i)
        Stair_ToF_Reference_Valid[i] = false;

    const StairStateConfig *config = Find_Stair_State_Config(state);
    if (config != nullptr && config->capture_sensor_reference)
        Capture_Stair_ToF_Reference(config->sensor);
    if (state == CHARIOT_LIFT_STAIR_DOWN_ATTITUDE_CORRECT)
        Stair_Attitude_Stable_Ticks = 0;
    if (state == CHARIOT_LIFT_STAIR_ABORT)
        Set_Stair_Drive_Command(false, false, 0.0f);
}

const char *Class_Chariot_Lift::Get_Stair_State_Name()
{
    if (Stair_State == CHARIOT_LIFT_STAIR_IDLE)
        return "IDLE";

    const StairStateConfig *config = Find_Stair_State_Config(Stair_State);
    return (config != nullptr) ? config->name : "UNKNOWN";
}

void Class_Chariot_Lift::Finish_Stair_Auto(bool retract_lift)
{
    Stair_State = CHARIOT_LIFT_STAIR_IDLE;
    Stair_State_Ticks = 0;
    Stair_Chassis_Forward = 0.0f;
    Stair_Chassis_Omega = 0.0f;
    Set_Stair_Drive_Command(false, false, 0.0f);
    Reset_Stair_Attitude_Correction();
    if (retract_lift)
    {
        Set_Stair_Lift_Command(true, CHARIOT_LIFT_POSITION_RETRACT,
                               true, CHARIOT_LIFT_POSITION_RETRACT);
    }
}

void Class_Chariot_Lift::Set_Stair_Lift_Command(bool front_enable,
                                                Enum_Chariot_Lift_Position_State front_state,
                                                bool rear_enable,
                                                Enum_Chariot_Lift_Position_State rear_state)
{
    Lift_Module_Enable[CHARIOT_LIFT_MODULE_FRONT] = front_enable;
    Lift_Module_Enable[CHARIOT_LIFT_MODULE_REAR] = rear_enable;
    Lift_State[CHARIOT_LIFT_MODULE_FRONT] = front_state;
    Lift_State[CHARIOT_LIFT_MODULE_REAR] = rear_state;
}

void Class_Chariot_Lift::Set_Stair_Drive_Command(bool front_enable,
                                                 bool rear_enable,
                                                 float forward_m_s)
{
    Stair_Drive_Module_Enable[CHARIOT_LIFT_MODULE_FRONT] = front_enable;
    Stair_Drive_Module_Enable[CHARIOT_LIFT_MODULE_REAR] = rear_enable;

    const bool enable = front_enable || rear_enable;
    if (Diff_Drive_Enable && !enable)
        drive_disable_exit_burst_ticks_ = 50;

    Diff_Drive_Enable = enable;
    Target_Diff_Forward = enable ? forward_m_s : 0.0f;
    Target_Diff_Yaw = 0.0f;
}

void Class_Chariot_Lift::Capture_Stair_ToF_Reference(Enum_Chariot_Lift_ToF_Sensor sensor)
{
    const int idx = static_cast<int>(sensor);
    Stair_ToF_Reference_Valid[idx] = false;
    if (Is_Stair_ToF_Usable(sensor))
    {
        Stair_ToF_Reference_Cm[idx] = ToF_Data[idx].distance_cm;
        Stair_ToF_Reference_Valid[idx] = true;
    }
}

bool Class_Chariot_Lift::Is_Stair_ToF_Usable(Enum_Chariot_Lift_ToF_Sensor sensor)
{
    const ChariotLiftToFData &tof = ToF_Data[static_cast<int>(sensor)];
    return tof.online &&
           tof.valid &&
           tof.distance_cm > 0U &&
           tof.distance_cm != kLiftToFTooFarCm &&
           std::isfinite(tof.range_m);
}

bool Class_Chariot_Lift::Is_Stair_ToF_Near(Enum_Chariot_Lift_ToF_Sensor sensor)
{
    const ChariotLiftToFData &tof = ToF_Data[static_cast<int>(sensor)];
    return tof.online &&
           tof.valid &&
           tof.distance_cm != kLiftToFTooFarCm &&
           tof.distance_cm <= kStairToFNearCm;
}

bool Class_Chariot_Lift::Is_Stair_ToF_Jumped(Enum_Chariot_Lift_ToF_Sensor sensor)
{
    if (!Is_Stair_ToF_Usable(sensor))
        return false;

    const int idx = static_cast<int>(sensor);
    const uint16_t distance_cm = ToF_Data[idx].distance_cm;
    if (!Stair_ToF_Reference_Valid[idx])
    {
        Stair_ToF_Reference_Cm[idx] = distance_cm;
        Stair_ToF_Reference_Valid[idx] = true;
        return false;
    }

    const int delta_cm = static_cast<int>(distance_cm) -
                         static_cast<int>(Stair_ToF_Reference_Cm[idx]);
    return std::abs(delta_cm) >= static_cast<int>(kStairToFJumpCm);
}

bool Class_Chariot_Lift::Is_Lift_Profile_Reached(Enum_Chariot_Lift_Module module,
                                                 Enum_Chariot_Lift_Position_State state)
{
    const float target = (state == CHARIOT_LIFT_POSITION_RAISE) ?
        Stair_Raise_Angle :
        Lift_Params[module].retract_angle;

    return !Lift_Profile_Active[module] &&
           fabsf(Smooth_Lift_Angle[module] - target) <= kStairLiftReachedTolerance;
}

bool Class_Chariot_Lift::Is_Lift_Feedback_Reached(Enum_Chariot_Lift_Module module,
                                                  Enum_Chariot_Lift_Position_State state)
{
    if (Motor_Lift[module].Get_Status() != Motor_DM_Status_ENABLE)
        return false;

    const float target = (state == CHARIOT_LIFT_POSITION_RAISE) ?
        Stair_Raise_Angle :
        Lift_Params[module].retract_angle;
    const float lift_rod_angle = Motor_To_Lift_Rod_Angle(Motor_Lift[module].Get_Now_Radian());
    return fabsf(lift_rod_angle - target) <= kStairLiftReachedTolerance;
}

bool Class_Chariot_Lift::Is_Stair_State_Timed_Out(uint32_t timeout_ticks)
{
    return Stair_State_Ticks >= timeout_ticks;
}

void Class_Chariot_Lift::Capture_Stair_Attitude_Target()
{
    Stair_Attitude_Stable_Ticks = 0;
    Stair_Attitude_Target_Valid = Stair_Attitude_Yaw_Valid;
    if (Stair_Attitude_Target_Valid)
        Stair_Attitude_Target_Yaw = Stair_Attitude_Yaw;
}

void Class_Chariot_Lift::Reset_Stair_Attitude_Correction()
{
    Stair_Attitude_Target_Valid = false;
    Stair_Attitude_Target_Yaw = 0.0f;
    Stair_Attitude_Stable_Ticks = 0;
    Stair_Chassis_Omega = 0.0f;
}

bool Class_Chariot_Lift::Is_Stair_Attitude_Corrected(uint32_t timeout_ticks)
{
    if (!Are_Both_Lifts_Reached(CHARIOT_LIFT_POSITION_RETRACT))
    {
        Stair_Chassis_Omega = 0.0f;
        Stair_Attitude_Stable_Ticks = 0;
        return false;
    }

    if (timeout_ticks > 0U && Is_Stair_State_Timed_Out(timeout_ticks))
    {
        Stair_Chassis_Omega = 0.0f;
        return true;
    }

    if (!Stair_Attitude_Target_Valid || !Stair_Attitude_Yaw_Valid)
    {
        Stair_Chassis_Omega = 0.0f;
        Stair_Attitude_Stable_Ticks = 0;
        return false;
    }

    const float yaw_error =
        Math_Modulus_Normalization(Stair_Attitude_Target_Yaw - Stair_Attitude_Yaw,
                                   2.0f * PI);

    if (fabsf(yaw_error) <= kStairDownAttitudeToleranceRad)
    {
        Stair_Chassis_Omega = 0.0f;
        if (Stair_Attitude_Stable_Ticks < kStairDownAttitudeStableTicks)
            ++Stair_Attitude_Stable_Ticks;
        return Stair_Attitude_Stable_Ticks >= kStairDownAttitudeStableTicks;
    }

    Stair_Attitude_Stable_Ticks = 0;
    float yaw_cmd = Clamp(yaw_error * kStairDownAttitudeKp,
                          -kStairDownAttitudeMaxYawRadS,
                           kStairDownAttitudeMaxYawRadS);
    if (fabsf(yaw_error) > kStairDownAttitudeMinYawErrorRad &&
        fabsf(yaw_cmd) < kStairDownAttitudeMinYawRadS)
        yaw_cmd = (yaw_cmd >= 0.0f) ?
            kStairDownAttitudeMinYawRadS :
            -kStairDownAttitudeMinYawRadS;

    Stair_Chassis_Omega = yaw_cmd;
    return false;
}

void Class_Chariot_Lift::Reset_Lift_Profile(Enum_Chariot_Lift_Module module,
                                            float lift_rod_angle)
{
    Smooth_Lift_Angle[module] = lift_rod_angle;
    Lift_Profile_Start_Angle[module] = lift_rod_angle;
    Lift_Profile_Target_Angle[module] = lift_rod_angle;
    Lift_Profile_Elapsed[module] = 0.0f;
    Lift_Profile_Duration[module] = 0.0f;
    Target_Lift_Rod_Omega[module] = 0.0f;
    Lift_Profile_Active[module] = false;
}

void Class_Chariot_Lift::Start_Lift_Profile(Enum_Chariot_Lift_Module module,
                                            float target_lift_rod_angle)
{
    const ChariotLiftPositionParams &params = Lift_Params[module];
    const float start_angle = Smooth_Lift_Angle[module];
    const float distance = target_lift_rod_angle - start_angle;
    const float safe_peak_speed = fmaxf(0.1f, params.max_speed);

    Lift_Profile_Start_Angle[module] = start_angle;
    Lift_Profile_Target_Angle[module] = target_lift_rod_angle;
    Lift_Profile_Elapsed[module] = 0.0f;
    Lift_Profile_Duration[module] =
        kLiftSCurvePeakVelocityScale * fabsf(distance) / safe_peak_speed;
    Target_Lift_Rod_Omega[module] = 0.0f;
    Lift_Profile_Active[module] = Lift_Profile_Duration[module] > 1e-4f;

    if (!Lift_Profile_Active[module])
        Reset_Lift_Profile(module, target_lift_rod_angle);
}

float Class_Chariot_Lift::Update_Lift_Profile(Enum_Chariot_Lift_Module module,
                                              float target_lift_rod_angle)
{
    if (fabsf(target_lift_rod_angle - Lift_Profile_Target_Angle[module]) > 1e-4f)
        Start_Lift_Profile(module, target_lift_rod_angle);

    if (!Lift_Profile_Active[module])
    {
        Smooth_Lift_Angle[module] = target_lift_rod_angle;
        Target_Lift_Rod_Omega[module] = 0.0f;
        return Target_Lift_Rod_Omega[module];
    }

    Lift_Profile_Elapsed[module] += kLiftControlDtS;
    const float u = (Lift_Profile_Duration[module] > 1e-5f) ?
        (Lift_Profile_Elapsed[module] / Lift_Profile_Duration[module]) :
        1.0f;

    if (u >= 1.0f)
    {
        Smooth_Lift_Angle[module] = Lift_Profile_Target_Angle[module];
        Target_Lift_Rod_Omega[module] = 0.0f;
        Lift_Profile_Active[module] = false;
        return Target_Lift_Rod_Omega[module];
    }

    const float distance =
        Lift_Profile_Target_Angle[module] - Lift_Profile_Start_Angle[module];
    Smooth_Lift_Angle[module] =
        Lift_Profile_Start_Angle[module] + distance * SCurve_Fraction(u);
    Target_Lift_Rod_Omega[module] =
        distance * SCurve_Fraction_Derivative(u) / Lift_Profile_Duration[module];
    return Target_Lift_Rod_Omega[module];
}

/**
 * @brief 更新单个抬升电机目标位置，包含抬升杆侧平滑和齿轮比换算。
 */
void Class_Chariot_Lift::Output_Lift_Motor(Enum_Chariot_Lift_Module module)
{
    const ChariotLiftPositionParams &params = Lift_Params[module];
    const float target_lift_rod_angle =
        !Lift_Module_Enable[module] ? Smooth_Lift_Angle[module] :
        (Lift_State[module] == CHARIOT_LIFT_POSITION_RAISE) ?
        params.raise_angle :
        params.retract_angle;

    const float target_lift_rod_omega =
        Update_Lift_Profile(module, target_lift_rod_angle);

    if (Motor_Lift[module].Get_Status() != Motor_DM_Status_ENABLE)
    {
        Motor_Lift[module].CAN_Send_Enter();
    }
    else
    {
        const float motor_target_angle = Lift_Rod_To_Motor_Angle(Smooth_Lift_Angle[module]);
        const float motor_target_omega = target_lift_rod_omega * kLiftMotorToRodRatio;
        const float hold_torque_ff = Interpolate_Lift_Hold_Torque(module, Smooth_Lift_Angle[module]);
        Motor_Lift[module].Set_Control_Maintain_Postion(motor_target_angle,
                                                        motor_target_omega,
                                                        hold_torque_ff,
                                                        params.kp,
                                                        params.kd);
    }
    Motor_Lift[module].TIM_Send_PeriodElapsedCallback();
}

/**
 * @brief 将临时底盘前进和偏航命令换算为左右抬升驱动轮角速度。
 */
void Class_Chariot_Lift::Kinematics_Diff_Resolution()
{
    /* 1. 清空所有模块的临时差速目标 */
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Raw_Target_Left_Omega[i] = 0.0f;
        Raw_Target_Right_Omega[i] = 0.0f;
    }

    if (!Diff_Drive_Enable)
        return;

    auto resolve_module = [this](int module) {
        const ChariotLiftDriveParams &params = Drive_Params[module];

        const float forward_cmd = Clamp(Target_Diff_Forward,
                                        -params.max_forward_speed,
                                         params.max_forward_speed);
        const float yaw = Clamp(Target_Diff_Yaw,
                                -params.max_yaw_omega,
                                 params.max_yaw_omega);

        // 抬升驱动轮的左右电机方向仍按原车体系标定；新车头为原车尾，所以前进分量取反。
        const float forward = -forward_cmd;

        const float left_linear = forward - yaw * params.track_width * 0.5f;
        const float right_linear = forward + yaw * params.track_width * 0.5f;

        Raw_Target_Left_Omega[module] =
            (left_linear / params.wheel_radius) *
            params.left_direction *
            params.left_speed_correction;
        Raw_Target_Right_Omega[module] =
            (right_linear / params.wheel_radius) *
            params.right_direction *
            params.right_speed_correction;

        const float max_abs = fmaxf(fabsf(Raw_Target_Left_Omega[module]),
                                    fabsf(Raw_Target_Right_Omega[module]));
        if (max_abs > params.max_wheel_omega && max_abs > 1e-4f)
        {
            const float scale = params.max_wheel_omega / max_abs;
            Raw_Target_Left_Omega[module] *= scale;
            Raw_Target_Right_Omega[module] *= scale;
        }

        if (fabsf(Raw_Target_Left_Omega[module]) < params.wheel_deadzone)
            Raw_Target_Left_Omega[module] = 0.0f;
        if (fabsf(Raw_Target_Right_Omega[module]) < params.wheel_deadzone)
            Raw_Target_Right_Omega[module] = 0.0f;
    };

    if (Is_Stair_Auto_Active())
    {
        for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
        {
            if (Stair_Drive_Module_Enable[i])
                resolve_module(i);
        }
        return;
    }

    bool any_module_enabled = false;
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        if (Lift_Module_Enable[i])
        {
            resolve_module(i);
            any_module_enabled = true;
        }
    }

    if (!any_module_enabled)
        resolve_module(Diff_Drive_Module);
}

/**
 * @brief 对抬升驱动轮目标角速度施加加减速限制。
 */
void Class_Chariot_Lift::Apply_Drive_Trapezoid_Profile()
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        const ChariotLiftDriveParams &params = Drive_Params[i];
        Target_Left_Omega[i] = Ramp_To(Target_Left_Omega[i],
                                       Raw_Target_Left_Omega[i],
                                       params.accel_limit,
                                       params.decel_limit,
                                       0.002f);
        Target_Right_Omega[i] = Ramp_To(Target_Right_Omega[i],
                                        Raw_Target_Right_Omega[i],
                                        params.accel_limit,
                                        params.decel_limit,
                                        0.002f);
    }
}

/**
 * @brief 向单个模块的左右驱动轮电机发送 MIT 速度命令。
 */
void Class_Chariot_Lift::Output_Drive_Motor(Enum_Chariot_Lift_Module module)
{
    const ChariotLiftDriveParams &params = Drive_Params[module];

    Motor_Drive_Left[module].Set_Control_Maintain_Postion(0.0f,
                                                          Target_Left_Omega[module],
                                                          0.0f,
                                                          params.wheel_kp,
                                                          params.wheel_kd);
    Motor_Drive_Left[module].TIM_Send_PeriodElapsedCallback();

    Motor_Drive_Right[module].Set_Control_Maintain_Postion(0.0f,
                                                           Target_Right_Omega[module],
                                                           0.0f,
                                                           params.wheel_kp,
                                                           params.wheel_kd);
    Motor_Drive_Right[module].TIM_Send_PeriodElapsedCallback();
}

/**
 * @brief 失能单个模块的两台抬升驱动轮电机，并重置轮速目标。
 */
void Class_Chariot_Lift::Disable_Drive_Motor(Enum_Chariot_Lift_Module module, bool force_exit)
{
    Target_Left_Omega[module] = 0.0f;
    Target_Right_Omega[module] = 0.0f;
    Raw_Target_Left_Omega[module] = 0.0f;
    Raw_Target_Right_Omega[module] = 0.0f;

    Motor_Drive_Left[module].Set_Control_Status(Motor_DM_Status_DISABLE);
    Motor_Drive_Left[module].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (force_exit ||
        Motor_Drive_Left[module].Get_Now_Control_Status() != Motor_DM_Control_Status_DISABLE)
    {
        Motor_Drive_Left[module].CAN_Send_Exit();
    }

    Motor_Drive_Right[module].Set_Control_Status(Motor_DM_Status_DISABLE);
    Motor_Drive_Right[module].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (force_exit ||
        Motor_Drive_Right[module].Get_Now_Control_Status() != Motor_DM_Control_Status_DISABLE)
    {
        Motor_Drive_Right[module].CAN_Send_Exit();
    }
}

/**
 * @brief 失能所有电机，并让抬升平滑状态对齐当前电机反馈。
 */
void Class_Chariot_Lift::Disable_All_Motors(bool force_exit)
{
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Disable_Drive_Motor(static_cast<Enum_Chariot_Lift_Module>(i), force_exit);

        Motor_Lift[i].Set_Control_Status(Motor_DM_Status_DISABLE);
        Motor_Lift[i].Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        if (force_exit ||
            Motor_Lift[i].Get_Now_Control_Status() != Motor_DM_Control_Status_DISABLE)
        {
            Motor_Lift[i].CAN_Send_Exit();
        }
        Smooth_Lift_Angle[i] = Motor_To_Lift_Rod_Angle(Motor_Lift[i].Get_Now_Radian());
        Reset_Lift_Profile(static_cast<Enum_Chariot_Lift_Module>(i), Smooth_Lift_Angle[i]);
        Lift_Module_Enable[i] = false;
    }
}

/**
 * @brief 清除临时差速驱动命令和轮速状态。
 */
void Class_Chariot_Lift::Reset_Drive_State()
{
    Target_Diff_Forward = 0.0f;
    Target_Diff_Yaw = 0.0f;
    Diff_Drive_Enable = false;
    for (int i = 0; i < CHARIOT_LIFT_MODULE_NUM; ++i)
    {
        Raw_Target_Left_Omega[i] = 0.0f;
        Raw_Target_Right_Omega[i] = 0.0f;
        Target_Left_Omega[i] = 0.0f;
        Target_Right_Omega[i] = 0.0f;
        Stair_Drive_Module_Enable[i] = false;
    }
}

/**
 * @brief 将数值限制在闭区间范围内。
 */
float Class_Chariot_Lift::Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/**
 * @brief 按加速度限制将当前值向目标值推进一个控制周期。
 */
float Class_Chariot_Lift::Ramp_To(float current,
                                  float target,
                                  float accel_limit,
                                  float decel_limit,
                                  float dt)
{
    const float delta = target - current;
    const float abs_delta = fabsf(delta);
    if (abs_delta < 1e-5f)
        return target;

    const bool changing_direction =
        (current * target < 0.0f) && (fabsf(current) > 1e-4f);
    const bool reducing_speed = fabsf(target) < fabsf(current);
    const float limit = (changing_direction || reducing_speed) ? decel_limit : accel_limit;
    const float max_step = limit * dt;

    if (abs_delta <= max_step)
        return target;

    return current + ((delta > 0.0f) ? max_step : -max_step);
}

/**
 * @brief 通过 1:3 齿轮比，将抬升杆侧角度换算为 DM3519 电机轴角度。
 */
float Class_Chariot_Lift::Lift_Rod_To_Motor_Angle(float lift_rod_angle)
{
    return lift_rod_angle * kLiftMotorToRodRatio;
}

/**
 * @brief 通过 1:3 齿轮比，将 DM3519 电机轴角度换算回抬升杆侧角度。
 */
float Class_Chariot_Lift::Motor_To_Lift_Rod_Angle(float motor_angle)
{
    return motor_angle / kLiftMotorToRodRatio;
}

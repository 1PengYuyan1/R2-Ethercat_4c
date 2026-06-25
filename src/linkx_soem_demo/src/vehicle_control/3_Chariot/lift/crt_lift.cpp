#include "crt_lift.h"

#include "math.h"

#include <cmath>
#include <limits>

namespace
{
constexpr int kLiftHoldPointCount = 5;
constexpr float kLiftControlDtS = 0.002f;
// 电机角常量（kLift*MotorAngle）现统一在 crt_lift.h 中定义，作为对外唯一抬升目标
// 位置来源；换算比与由其派生的内部杆角常量保留在此处。
constexpr float kLiftMotorToRodRatio = 3.0f;
constexpr float kLiftFrontRetractRodAngle = kLiftFrontRetractMotorAngle / kLiftMotorToRodRatio;
constexpr float kLiftRearRetractRodAngle = kLiftRearRetractMotorAngle / kLiftMotorToRodRatio;
constexpr float kLiftFrontRaiseRodAngle = kLiftFrontRaiseMotorAngle / kLiftMotorToRodRatio;
constexpr float kLiftRearRaiseRodAngle = kLiftRearRaiseMotorAngle / kLiftMotorToRodRatio;
constexpr float kLiftSCurvePeakVelocityScale = 1.875f;
constexpr uint32_t kLiftToFSlaveId = 2U;
constexpr uint8_t kLiftToFOfflineTicks = 5;  // 5 * 100ms
constexpr uint16_t kLiftToFInvalidStrength = 65535U;
constexpr uint16_t kLiftToFTooFarCm = 65532U;
constexpr uint16_t kStairToFNearCm = 10U;
constexpr uint16_t kStairToFJumpCm = 15U;
constexpr float kStairLiftReachedTolerance = 0.10f;
constexpr float kStairLiftDriveForwardMps = 0.6f;
constexpr float kStairChassisForwardMps = 0.6f;
constexpr float kStairDownChassisForwardMps = 0.6f;
constexpr uint32_t kStairLiftTimeoutTicks = 2500U;     // 5s at 2ms
constexpr uint32_t kStairSensorTimeoutTicks = 5000U;   // 10s at 2ms
constexpr uint32_t kStairExtraDriveTicks = 250U;       // 0.5s at 2ms
// 摆正（定点纠正）：吸附到最近正交航向后偏差最大可达 ±45°。
// 调快收敛 -> 提高 Kp 与角速度上限；放宽容差作为收敛死区，并缩短稳定确认 + 超时，
// 避免摆正后还要干等很久才交还控制（进入死区即判定完成；超时仅作兜底）。
constexpr uint32_t kStairDownAttitudeTimeoutTicks = 2000U;  // 4s at 2ms（兜底，正常用不到）
constexpr uint32_t kStairDownAttitudeStableTicks = 75U;      // 0.15s at 2ms
constexpr float kStairDownAttitudeToleranceRad = 0.035f;     // ~2° 收敛死区（原 1.15° 太严、难稳定）
constexpr float kStairDownAttitudeKp = 5.5f;
constexpr float kStairDownAttitudeMaxYawRadS = 1.50f;
constexpr float kStairDownAttitudeMinYawRadS = 0.16f;
constexpr float kStairDownAttitudeMinYawErrorRad = 0.06f;

// ===== 上/下台阶姿态纠正：纯 IMU，吸附最近的正交航向 =====
// 不直接锁固定角度（IMU 无磁、零点无意义，固定 -90° 不一定对应实际正前方）。
// 改为在启动纠正时把当前 IMU yaw 吸附到最近的正交航向（0 / ±90 / ±180），锁为目标。
// 这样目标始终在当前航向 ±45° 内，能纠掉小偏差又不会跑到错误象限。上/下台阶共用。
constexpr float kStairAttitudeSnapStepRad = PI / 2.0f;  // 吸附栅格步长：90°

// ===== 上/下台阶"过程中"的闭环偏航保持（第1步：仅纯触地阶段）=====
// 整车抬起 -> 4 个抬升轮触地，偏航走抬升差速(Target_Diff_Yaw)；
// 整车落下 -> 4 个全向轮触地，偏航走全向 omega(Stair_Chassis_Omega)。
// 混合阶段(一端全向+另一端抬升)需要侧移协同，留到第2步，这里不介入。
// 目标航向复用 Stair_Attitude_Target_Yaw（爬梯启动/前置纠正后锁定的航向）。
constexpr float kStairMotionYawKp = 2.0f;          // 过程偏航比例增益（比定点纠正温和，因同时在前进）
constexpr float kStairMotionYawMaxRadS = 0.50f;    // 过程偏航角速度上限
constexpr float kStairMotionYawDeadzoneRad = 0.03f;// 死区(~1.7°)，避免行进中来回小幅摆动
// 混合支撑阶段（一端抬升驱动、一端全向且逐步悬空）：只有一端在驱动，单位偏航指令的
// 实际纠正弱于纯触地阶段，且这段受力最不对称、最易偏，故单独给更强的增益与上限。
// 偏冲/摆动就调小，压不住就调大。死区沿用 kStairMotionYawDeadzoneRad。
constexpr float kStairMixedYawKp = 3.0f;
constexpr float kStairMixedYawMaxRadS = 0.70f;
constexpr float kStairMotionLiftYawSign = 1.0f;    // 抬升差速偏航方向符号，台架确认(+1/-1)；
                                                    // 全向 omega 沿用 attitude-correct 已验证的符号

// ===== 辅助 DM 电机（0x07，CAN0 Tx 0x07 / Rx 0x17）+ 抬升协同动作参数 =====
// Down：抬升伸出（前 -39/后 +39）到位后辅助臂外伸到 1.5；Up：辅助臂回 0 后抬升回收。
constexpr float kManualFrontLiftRaiseMotorAngle = -39.0f;
constexpr float kManualRearLiftRaiseMotorAngle = 39.0f;
constexpr float kManualFrontLiftRetractMotorAngle = -0.03f;
constexpr float kManualRearLiftRetractMotorAngle = 0.03f;
constexpr float kAuxiliaryMotorRaisedAngle = 2.4f;
constexpr float kAuxiliaryMotorHomeAngle = 0.95f;
constexpr float kAuxiliaryMotorReachedTolerance = 0.05f;
constexpr float kAuxiliaryMotorHomeReachedTolerance = 0.05f;
constexpr float kAuxiliaryMotorControlDtS = 0.002f;

// ---- 运动段：梯形速度规划（accel → cruise → decel），抬升/回零各一套限幅 ----
constexpr float kAuxiliaryMotorProfileMaxSpeed = 5.8f;
constexpr float kAuxiliaryMotorHomeProfileMaxSpeed = 4.0f;
constexpr float kAuxiliaryMotorProfileMaxAccel = 22.0f;
constexpr float kAuxiliaryMotorHomeProfileMaxAccel = 18.0f;
constexpr float kAuxiliaryMotorProfileMaxDecel = 28.0f;
constexpr float kAuxiliaryMotorHomeProfileMaxDecel = 28.0f;
constexpr float kAuxiliaryMotorHomeProfileSnapDistance = 0.06f;

// ---- 运动段 MIT 增益：跟随规划轨迹，偏软以保证平滑、不抖 ----
constexpr float kAuxiliaryMotorMoveTorqueNm = 0.0f;
constexpr float kAuxiliaryMotorMoveKp = 12.0f;
constexpr float kAuxiliaryMotorMoveKd = 2.0f;
constexpr float kAuxiliaryMotorHomeMoveKp = 14.0f;
constexpr float kAuxiliaryMotorHomeMoveKd = 2.0f;

// ---- 保持段：刚性 + 近临界阻尼 PD ----
constexpr float kAuxiliaryMotorHoldTorqueNm = 0.0f;
constexpr float kAuxiliaryMotorHoldKp = 20.0f;
constexpr float kAuxiliaryMotorHoldKd = 2.0f;
constexpr float kAuxiliaryMotorHoldDeadband = 0.015f;
constexpr float kAuxiliaryMotorHomeHoldKp = 20.0f;
constexpr float kAuxiliaryMotorHomeHoldKd = 2.0f;
constexpr float kAuxiliaryMotorHomeHoldDeadband = 0.015f;

// ---- 进入/退出保持的判据 ----
constexpr float kAuxiliaryMotorHoldEnterTolerance = 0.04f;
constexpr float kAuxiliaryMotorHomeHoldEnterTolerance = 0.12f;
constexpr float kAuxiliaryMotorHoldEnterOmega = 0.25f;
constexpr uint32_t kAuxiliaryMotorHoldEnterStableTicks = 25U;
constexpr float kAuxiliaryMotorHoldExitTolerance = 0.20f;
constexpr float kAuxiliaryMotorHoldGainRampTimeS = 0.25f;
constexpr float kAuxiliaryMotorHomeHoldGainRampTimeS = 0.20f;
constexpr uint8_t kAuxiliaryMotorCanChannel = 0U;
constexpr uint8_t kAuxiliaryMotorRxId = 0x17U;
constexpr uint8_t kAuxiliaryMotorTxId = 0x07U;

bool Auxiliary_Is_Home_Target(float target_angle)
{
    return std::fabs(target_angle - kAuxiliaryMotorHomeAngle) <= 1e-3f;
}


struct LiftHoldFeedforwardTable
{
    float rod_angle[kLiftHoldPointCount];
    float motor_torque_nm[kLiftHoldPointCount];
};

struct LiftToFCanMap
{
    uint32_t slave_id;
    uint8_t channel;
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
    // 车尾两路下视 ToF（DOWN_BACK + DOWN_BACK2）需同时突变才触发。
    STAIR_TRANSITION_TOF_JUMP_BOTH_BACK,
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

constexpr uint32_t kChassisToFSlaveId = 1U;  // 整车主控 slave1，承载底盘四面 8 路 TFmini-S
constexpr LiftToFCanMap kLiftToFCanMap[CHARIOT_LIFT_TOF_NUM] = {
    {kLiftToFSlaveId, 3U, 0x02U, CHARIOT_LIFT_TOF_UP_FRONT},
    {kLiftToFSlaveId, 2U, 0x01U, CHARIOT_LIFT_TOF_UP_BACK},
    {kLiftToFSlaveId, 3U, 0x01U, CHARIOT_LIFT_TOF_DOWN_FRONT},
    {kLiftToFSlaveId, 2U, 0x02U, CHARIOT_LIFT_TOF_DOWN_BACK},
    {kChassisToFSlaveId, 2U, 0x02U, CHARIOT_LIFT_TOF_DOWN_BACK2},
    {kChassisToFSlaveId, 1U, 0x01U, CHARIOT_LIFT_TOF_CHASSIS_FRONT_LEFT},
    {kChassisToFSlaveId, 1U, 0x02U, CHARIOT_LIFT_TOF_CHASSIS_FRONT_RIGHT},
};

constexpr StairStateConfig kStairStateConfigs[] = {
    {
        CHARIOT_LIFT_STAIR_UP_PRE_ATTITUDE_CORRECT,
        "UP_PRE_ATTITUDE_CORRECT",
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, CHARIOT_LIFT_POSITION_RETRACT,
        false, false, 0.0f, 0.0f,
        STAIR_TRANSITION_ATTITUDE_CORRECTED,
        CHARIOT_LIFT_TOF_DOWN_BACK,
        CHARIOT_LIFT_STAIR_UP_RAISE_ALL,
        kStairDownAttitudeTimeoutTicks,
        false,
        false,
    },
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
        CHARIOT_LIFT_TOF_UP_BACK,
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
        CHARIOT_LIFT_TOF_UP_BACK,
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
        STAIR_TRANSITION_TOF_JUMP_BOTH_BACK,
        CHARIOT_LIFT_TOF_DOWN_BACK,
        CHARIOT_LIFT_STAIR_UP_ATTITUDE_CORRECT,
        kStairSensorTimeoutTicks,
        true,
        false,
    },
    {
        CHARIOT_LIFT_STAIR_DOWN_CHASSIS_WAIT_UP_BACK,
        "DOWN_CHASSIS_WAIT_UP_BACK",
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, CHARIOT_LIFT_POSITION_RETRACT,
        false, false, 0.0f, kStairDownChassisForwardMps,
        STAIR_TRANSITION_TOF_JUMP,
        CHARIOT_LIFT_TOF_DOWN_FRONT,
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
        CHARIOT_LIFT_TOF_DOWN_FRONT,
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
        true, false, kStairLiftDriveForwardMps, kStairDownChassisForwardMps,
        STAIR_TRANSITION_TOF_JUMP_BOTH_BACK,
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
        CHARIOT_LIFT_STAIR_UP_ATTITUDE_CORRECT,
        "UP_ATTITUDE_CORRECT",
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
        CHARIOT_LIFT_STAIR_DOWN_PRE_ATTITUDE_CORRECT,
        "DOWN_PRE_ATTITUDE_CORRECT",
        true, CHARIOT_LIFT_POSITION_RETRACT,
        true, CHARIOT_LIFT_POSITION_RETRACT,
        false, false, 0.0f, 0.0f,
        STAIR_TRANSITION_ATTITUDE_CORRECTED,
        CHARIOT_LIFT_TOF_DOWN_BACK,
        CHARIOT_LIFT_STAIR_DOWN_CHASSIS_WAIT_UP_BACK,
        kStairDownAttitudeTimeoutTicks,
        false,
        false,
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

bool Match_Lift_ToF_Sensor(uint32_t slave_id,
                           uint8_t channel,
                           uint32_t can_id_std,
                           Enum_Chariot_Lift_ToF_Sensor &sensor)
{
    for (const auto &item : kLiftToFCanMap)
    {
        if (item.slave_id == slave_id &&
            item.channel == channel &&
            item.id == can_id_std)
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

// 把任意航向吸附到最近的正交航向（0 / ±90 / ±180），结果规整到 (-PI, PI]。
float Snap_Yaw_To_Cardinal(float yaw_rad)
{
    const float snapped =
        roundf(yaw_rad / kStairAttitudeSnapStepRad) * kStairAttitudeSnapStepRad;
    return Math_Modulus_Normalization(snapped, 2.0f * PI);
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

    Auxiliary_Motor.Init(__LinkX_Handler,
                         kAuxiliaryMotorCanChannel,
                         kAuxiliaryMotorRxId,
                         kAuxiliaryMotorTxId,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f,
                         395.0f,
                         7.8f,
                         9.2f);
    Auxiliary_Motor.Set_Use_FDCAN(true);
    Auxiliary_Motor.Set_Force_Output_Without_Feedback(false);

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
        .max_forward_speed      = 2.0f,
        .max_yaw_omega          = 2.0f,
        .max_wheel_omega        = 80.0f,
        .wheel_kp               = 0.0f,
        .wheel_kd               = 1.2f,
        .wheel_deadzone         = 0.05f,
        .left_direction         = 1.0f,
        .right_direction        = -1.0f,
        .left_speed_correction  = 1.0925f,
        .right_speed_correction = 0.9075f,
        .accel_limit            = 300.0f,
        .decel_limit            = 600.0f,
    };

    Drive_Params[CHARIOT_LIFT_MODULE_REAR] = {
        .wheel_radius           = 0.0761f,
        .track_width            = 0.420f,
        .max_forward_speed      = 2.0f,
        .max_yaw_omega          = 2.0f,
        .max_wheel_omega        = 80.0f,
        .wheel_kp               = 0.0f,
        .wheel_kd               = 1.2f,
        .wheel_deadzone         = 0.05f,
        .left_direction         = 1.0f,
        .right_direction        = -1.0f,
        .left_speed_correction  = 1.0925f,
        .right_speed_correction = 0.9075f,
        .accel_limit            = 300.0f,
        .decel_limit            = 600.0f,
    };

    Lift_Params[CHARIOT_LIFT_MODULE_FRONT] = {
        .retract_angle = kLiftFrontRetractRodAngle,
        .raise_angle   = kLiftFrontRaiseRodAngle,
        .max_speed     = 13.0f,
        .kp            = 20.0f,
        .kd            = 1.2f,
    };

    Lift_Params[CHARIOT_LIFT_MODULE_REAR] = {
        .retract_angle = kLiftRearRetractRodAngle,
        .raise_angle   = kLiftRearRaiseRodAngle,
        .max_speed     = 13.0f,
        .kp            = 20.0f,
        .kd            = 1.2f,
    };

    Stair_Raise_Angle[CHARIOT_LIFT_MODULE_FRONT] =
        Lift_Params[CHARIOT_LIFT_MODULE_FRONT].raise_angle;
    Stair_Raise_Angle[CHARIOT_LIFT_MODULE_REAR] =
        Lift_Params[CHARIOT_LIFT_MODULE_REAR].raise_angle;

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

    if (CAN_Rx_ToF_Frame(1U, CAN_Channel, can_id_std, CAN_Data, CAN_Dlen))
        return true;

    /* 辅助 DM 电机（0x07）反馈：CAN0，Rx ID 0x17 */
    if (CAN_Channel == kAuxiliaryMotorCanChannel && can_id_std == Auxiliary_Motor.DM_CAN_Rx_ID)
    {
        Auxiliary_Motor.CAN_RxCpltCallback(CAN_Data);
        return true;
    }

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

bool Class_Chariot_Lift::CAN_Rx_ToF_Frame(uint32_t slave_id,
                                          uint8_t CAN_Channel,
                                          uint32_t CAN_ID,
                                          const uint8_t *CAN_Data,
                                          uint8_t CAN_Dlen)
{
    if (CAN_Data == nullptr || CAN_Dlen == 0)
        return false;

    Enum_Chariot_Lift_ToF_Sensor sensor = CHARIOT_LIFT_TOF_UP_FRONT;
    if (!Match_Lift_ToF_Sensor(slave_id, CAN_Channel, CAN_ID & 0x7FFU, sensor))
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
    Auxiliary_Motor.TIM_Alive_PeriodElapsedCallback();

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

void Class_Chariot_Lift::Start_Stair_Up(float raise_motor_angle)
{
    Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    Set_Raise_Angle(raise_motor_angle);
    Capture_Stair_Attitude_Target();
    Enter_Stair_State(CHARIOT_LIFT_STAIR_UP_PRE_ATTITUDE_CORRECT);
}

void Class_Chariot_Lift::Start_Stair_Down(float raise_motor_angle)
{
    Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    Set_Raise_Angle(raise_motor_angle);
    Capture_Stair_Attitude_Target();
    Enter_Stair_State(CHARIOT_LIFT_STAIR_DOWN_PRE_ATTITUDE_CORRECT);
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

// 以下抬升 API 的角度参数统一为「电机角度」(rad)，入口处经
// Motor_To_Lift_Rod_Angle() 转换一次为内部杆角，内部一律使用杆角。

void Class_Chariot_Lift::Set_Both_Lift_Raise(float raise_motor_angle)
{
    // 单参（前后同一电机角度）：车尾电机反向安装，自动取反以维持前负/后正。
    Set_Both_Lift_Raise(raise_motor_angle, -raise_motor_angle);
}

void Class_Chariot_Lift::Set_Both_Lift_Raise(float front_raise_motor_angle, float rear_raise_motor_angle)
{
    Stop_Stair_Auto();
    Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    Set_Raise_Angle(front_raise_motor_angle, rear_raise_motor_angle);
    Set_Stair_Lift_Command(true, CHARIOT_LIFT_POSITION_RAISE,
                           true, CHARIOT_LIFT_POSITION_RAISE);
}

void Class_Chariot_Lift::Set_Raise_Angle(float raise_motor_angle)
{
    // 单参（前后同一电机角度）：车尾自动取反维持前负/后正。
    Set_Raise_Angle(raise_motor_angle, -raise_motor_angle);
}

void Class_Chariot_Lift::Set_Raise_Angle(float front_raise_motor_angle, float rear_raise_motor_angle)
{
    Set_Raise_Angle_Rod(Motor_To_Lift_Rod_Angle(front_raise_motor_angle),
                        Motor_To_Lift_Rod_Angle(rear_raise_motor_angle));
}

void Class_Chariot_Lift::Set_Raise_Angle_Rod(float front_raise_rod_angle, float rear_raise_rod_angle)
{
    Lift_Params[CHARIOT_LIFT_MODULE_FRONT].raise_angle = front_raise_rod_angle;
    Lift_Params[CHARIOT_LIFT_MODULE_REAR].raise_angle = rear_raise_rod_angle;
    Stair_Raise_Angle[CHARIOT_LIFT_MODULE_FRONT] =
        Lift_Params[CHARIOT_LIFT_MODULE_FRONT].raise_angle;
    Stair_Raise_Angle[CHARIOT_LIFT_MODULE_REAR] =
        Lift_Params[CHARIOT_LIFT_MODULE_REAR].raise_angle;
}

void Class_Chariot_Lift::Set_Both_Lift_Retract()
{
    Stop_Stair_Auto();
    Set_Control_Type(CHARIOT_LIFT_CONTROL_ENABLE);
    Set_Stair_Lift_Command(true, CHARIOT_LIFT_POSITION_RETRACT,
                           true, CHARIOT_LIFT_POSITION_RETRACT);
}

void Class_Chariot_Lift::Set_Both_Lift_Retract_To(float retract_motor_angle)
{
    // 单参（前后同一电机角度）：车尾自动取反维持前负/后正。
    Set_Both_Lift_Retract_To(retract_motor_angle, -retract_motor_angle);
}

void Class_Chariot_Lift::Set_Both_Lift_Retract_To(float front_retract_motor_angle, float rear_retract_motor_angle)
{
    Lift_Params[CHARIOT_LIFT_MODULE_FRONT].retract_angle =
        Motor_To_Lift_Rod_Angle(front_retract_motor_angle);
    Lift_Params[CHARIOT_LIFT_MODULE_REAR].retract_angle =
        Motor_To_Lift_Rod_Angle(rear_retract_motor_angle);
    Set_Both_Lift_Retract();
}

bool Class_Chariot_Lift::Are_Both_Lifts_Reached(Enum_Chariot_Lift_Position_State state)
{
    return Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_FRONT, state) &&
           Is_Lift_Profile_Reached(CHARIOT_LIFT_MODULE_REAR, state) &&
           Is_Lift_Feedback_Reached(CHARIOT_LIFT_MODULE_FRONT, state) &&
           Is_Lift_Feedback_Reached(CHARIOT_LIFT_MODULE_REAR, state);
}

// ============================================================================
//  辅助 DM 电机（0x07）+ 抬升协同动作（遥控 Up/Down、服务 lift_aux/raise|home）
//  Down：抬升伸出到位 → 辅助臂外伸 1.5；Up：辅助臂回 0 → 抬升回收。
//  1ms 节拍由 TIM_1ms_Aux_PeriodElapsedCallback() 驱动。
// ============================================================================

void Class_Chariot_Lift::Enable_Auxiliary_Motor()
{
    aux_vehicle_enabled_ = true;
    lift_aux_sequence_state_ = LiftAuxSequenceState::IDLE;
    auxiliary_motor_target_angle_ = kAuxiliaryMotorHomeAngle;
    auxiliary_motor_command_enable_ = true;
    auxiliary_motor_profile_initialized_ = false;
    auxiliary_motor_hold_active_ = false;
    auxiliary_motor_hold_blend_ = 0.0f;
    auxiliary_motor_hold_ready_ticks_ = 0U;
    Auxiliary_Motor.CAN_Send_Enter();
}

void Class_Chariot_Lift::Disable_Auxiliary_Motor()
{
    aux_vehicle_enabled_ = false;
    Cancel_Lift_Aux_Sequence();
    auxiliary_motor_profile_initialized_ = false;
    auxiliary_motor_profile_active_ = false;
    auxiliary_motor_hold_active_ = false;
    auxiliary_motor_hold_blend_ = 0.0f;
    auxiliary_motor_hold_ready_ticks_ = 0U;
    Auxiliary_Motor.Set_Control_Status(Motor_DM_Status_DISABLE);
    Auxiliary_Motor.Set_Control_Maintain_Postion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Auxiliary_Motor.CAN_Send_Exit();
}

void Class_Chariot_Lift::Cancel_Lift_Aux_Sequence()
{
    lift_aux_sequence_state_ = LiftAuxSequenceState::IDLE;
    if (aux_vehicle_enabled_)
    {
        auxiliary_motor_target_angle_ = kAuxiliaryMotorHomeAngle;
        auxiliary_motor_command_enable_ = true;
    }
    else
    {
        auxiliary_motor_command_enable_ = false;
    }
}

void Class_Chariot_Lift::Start_Lift_Aux_Raise_Sequence()
{
    Set_Both_Lift_Raise(kManualFrontLiftRaiseMotorAngle,
                        kManualRearLiftRaiseMotorAngle);
    auxiliary_motor_target_angle_ = kAuxiliaryMotorHomeAngle;
    auxiliary_motor_command_enable_ = true;
    lift_aux_sequence_state_ = LiftAuxSequenceState::RAISE_WAIT_LIFT_REACHED;
}

void Class_Chariot_Lift::Start_Lift_Aux_Home_Sequence()
{
    Stop_Stair_Auto();
    auxiliary_motor_target_angle_ = kAuxiliaryMotorHomeAngle;
    auxiliary_motor_command_enable_ = true;
    lift_aux_sequence_state_ = LiftAuxSequenceState::HOME_MOVE_AUXILIARY;
}

void Class_Chariot_Lift::TIM_1ms_Aux_PeriodElapsedCallback()
{
    Update_Lift_Aux_Sequence();
}

void Class_Chariot_Lift::Update_Lift_Aux_Sequence()
{
    switch (lift_aux_sequence_state_)
    {
        case LiftAuxSequenceState::RAISE_WAIT_LIFT_REACHED:
            if (Are_Both_Lifts_Reached(CHARIOT_LIFT_POSITION_RAISE))
            {
                auxiliary_motor_target_angle_ = kAuxiliaryMotorRaisedAngle;
                auxiliary_motor_command_enable_ = true;
                lift_aux_sequence_state_ = LiftAuxSequenceState::RAISE_MOVE_AUXILIARY;
            }
            break;

        case LiftAuxSequenceState::RAISE_MOVE_AUXILIARY:
            if (Is_Auxiliary_Motor_Reached(kAuxiliaryMotorRaisedAngle))
                lift_aux_sequence_state_ = LiftAuxSequenceState::RAISE_DONE;
            break;

        case LiftAuxSequenceState::HOME_WAIT_LIFT_REACHED:
            if (Are_Both_Lifts_Reached(CHARIOT_LIFT_POSITION_RETRACT))
                lift_aux_sequence_state_ = LiftAuxSequenceState::HOME_DONE;
            break;

        case LiftAuxSequenceState::HOME_MOVE_AUXILIARY:
            if (Is_Auxiliary_Motor_Reached(kAuxiliaryMotorHomeAngle))
            {
                Set_Both_Lift_Retract_To(kManualFrontLiftRetractMotorAngle,
                                         kManualRearLiftRetractMotorAngle);
                lift_aux_sequence_state_ = LiftAuxSequenceState::HOME_WAIT_LIFT_REACHED;
            }
            break;

        case LiftAuxSequenceState::RAISE_DONE:
        case LiftAuxSequenceState::HOME_DONE:
        case LiftAuxSequenceState::IDLE:
        default:
            break;
    }

    Output_Auxiliary_Motor();
}

void Class_Chariot_Lift::Reset_Auxiliary_Motor_Profile(float angle)
{
    auxiliary_motor_smooth_angle_ = angle;
    auxiliary_motor_profile_start_angle_ = angle;
    auxiliary_motor_profile_target_angle_ = angle;
    auxiliary_motor_profile_elapsed_ = 0.0f;
    auxiliary_motor_profile_duration_ = 0.0f;
    auxiliary_motor_profile_accel_time_ = 0.0f;
    auxiliary_motor_profile_decel_time_ = 0.0f;
    auxiliary_motor_profile_cruise_time_ = 0.0f;
    auxiliary_motor_profile_peak_speed_ = 0.0f;
    auxiliary_motor_profile_distance_ = 0.0f;
    auxiliary_motor_profile_direction_ = 1.0f;
    auxiliary_motor_target_omega_ = 0.0f;
    auxiliary_motor_profile_active_ = false;
    auxiliary_motor_profile_initialized_ = true;
    auxiliary_motor_hold_active_ = false;
    auxiliary_motor_hold_blend_ = 0.0f;
    auxiliary_motor_hold_ready_ticks_ = 0U;
}

void Class_Chariot_Lift::Start_Auxiliary_Motor_Profile(float target_angle)
{
    const float start_angle = auxiliary_motor_smooth_angle_;
    const float distance = target_angle - start_angle;
    const float abs_distance = std::fabs(distance);
    const bool home_target = Auxiliary_Is_Home_Target(target_angle);
    const float max_speed = home_target ?
        kAuxiliaryMotorHomeProfileMaxSpeed :
        kAuxiliaryMotorProfileMaxSpeed;
    const float safe_peak_speed =
        (max_speed > 0.05f) ? max_speed : 0.05f;
    const float max_accel = home_target ?
        kAuxiliaryMotorHomeProfileMaxAccel :
        kAuxiliaryMotorProfileMaxAccel;
    const float max_decel = home_target ?
        kAuxiliaryMotorHomeProfileMaxDecel :
        kAuxiliaryMotorProfileMaxDecel;
    const float safe_peak_accel =
        (max_accel > 0.5f) ? max_accel : 0.5f;
    const float safe_peak_decel =
        (max_decel > 0.5f) ? max_decel : 0.5f;

    auxiliary_motor_profile_start_angle_ = start_angle;
    auxiliary_motor_profile_target_angle_ = target_angle;
    auxiliary_motor_profile_elapsed_ = 0.0f;
    auxiliary_motor_profile_distance_ = abs_distance;
    auxiliary_motor_profile_direction_ = (distance >= 0.0f) ? 1.0f : -1.0f;

    const float accel_time_to_max = safe_peak_speed / safe_peak_accel;
    const float decel_time_to_max = safe_peak_speed / safe_peak_decel;
    const float accel_decel_distance_to_max =
        0.5f * safe_peak_speed * safe_peak_speed *
        ((1.0f / safe_peak_accel) + (1.0f / safe_peak_decel));
    if (abs_distance <= accel_decel_distance_to_max)
    {
        auxiliary_motor_profile_peak_speed_ =
            std::sqrt((2.0f * abs_distance * safe_peak_accel * safe_peak_decel) /
                      (safe_peak_accel + safe_peak_decel));
        auxiliary_motor_profile_accel_time_ =
            auxiliary_motor_profile_peak_speed_ / safe_peak_accel;
        auxiliary_motor_profile_decel_time_ =
            auxiliary_motor_profile_peak_speed_ / safe_peak_decel;
        auxiliary_motor_profile_cruise_time_ = 0.0f;
    }
    else
    {
        auxiliary_motor_profile_peak_speed_ = safe_peak_speed;
        auxiliary_motor_profile_accel_time_ = accel_time_to_max;
        auxiliary_motor_profile_decel_time_ = decel_time_to_max;
        auxiliary_motor_profile_cruise_time_ =
            (abs_distance - accel_decel_distance_to_max) / safe_peak_speed;
    }

    auxiliary_motor_profile_duration_ =
        auxiliary_motor_profile_accel_time_ +
        auxiliary_motor_profile_decel_time_ +
        auxiliary_motor_profile_cruise_time_;
    auxiliary_motor_target_omega_ = 0.0f;
    auxiliary_motor_profile_active_ = auxiliary_motor_profile_duration_ > 1e-4f;
    auxiliary_motor_hold_active_ = false;
    auxiliary_motor_hold_blend_ = 0.0f;
    auxiliary_motor_hold_ready_ticks_ = 0U;

    if (!auxiliary_motor_profile_active_)
        Reset_Auxiliary_Motor_Profile(target_angle);
}

float Class_Chariot_Lift::Update_Auxiliary_Motor_Profile(float target_angle)
{
    if (!auxiliary_motor_profile_initialized_)
        Reset_Auxiliary_Motor_Profile(Auxiliary_Motor.Get_Now_Radian());

    if (std::fabs(target_angle - auxiliary_motor_profile_target_angle_) > 1e-4f)
        Start_Auxiliary_Motor_Profile(target_angle);

    if (!auxiliary_motor_profile_active_)
    {
        auxiliary_motor_smooth_angle_ = target_angle;
        auxiliary_motor_target_omega_ = 0.0f;
        return auxiliary_motor_target_omega_;
    }

    auxiliary_motor_profile_elapsed_ += kAuxiliaryMotorControlDtS;

    if (auxiliary_motor_profile_elapsed_ >= auxiliary_motor_profile_duration_)
    {
        auxiliary_motor_smooth_angle_ = auxiliary_motor_profile_target_angle_;
        auxiliary_motor_target_omega_ = 0.0f;
        auxiliary_motor_profile_active_ = false;
        return auxiliary_motor_target_omega_;
    }

    const float accel_time = auxiliary_motor_profile_accel_time_;
    const float decel_time = auxiliary_motor_profile_decel_time_;
    const float cruise_time = auxiliary_motor_profile_cruise_time_;
    const float peak_speed = auxiliary_motor_profile_peak_speed_;
    const float accel = (accel_time > 1e-5f) ? (peak_speed / accel_time) : 0.0f;
    const float decel = (decel_time > 1e-5f) ? (peak_speed / decel_time) : 0.0f;
    const float accel_distance = 0.5f * accel * accel_time * accel_time;
    const float cruise_distance = peak_speed * cruise_time;
    float profile_distance = 0.0f;
    float profile_speed = 0.0f;

    if (auxiliary_motor_profile_elapsed_ < accel_time)
    {
        const float t = auxiliary_motor_profile_elapsed_;
        profile_distance = 0.5f * accel * t * t;
        profile_speed = accel * t;
    }
    else if (auxiliary_motor_profile_elapsed_ < accel_time + cruise_time)
    {
        const float t = auxiliary_motor_profile_elapsed_ - accel_time;
        profile_distance = accel_distance + peak_speed * t;
        profile_speed = peak_speed;
    }
    else
    {
        const float t = auxiliary_motor_profile_elapsed_ - accel_time - cruise_time;
        profile_distance =
            accel_distance +
            cruise_distance +
            peak_speed * t -
            0.5f * decel * t * t;
        profile_speed = peak_speed - decel * t;
        if (profile_speed < 0.0f)
            profile_speed = 0.0f;
    }

    if (profile_distance > auxiliary_motor_profile_distance_)
        profile_distance = auxiliary_motor_profile_distance_;

    const float remaining_distance =
        auxiliary_motor_profile_distance_ - profile_distance;
    if (Auxiliary_Is_Home_Target(auxiliary_motor_profile_target_angle_) &&
        remaining_distance <= kAuxiliaryMotorHomeProfileSnapDistance)
    {
        auxiliary_motor_smooth_angle_ = auxiliary_motor_profile_target_angle_;
        auxiliary_motor_target_omega_ = 0.0f;
        auxiliary_motor_profile_active_ = false;
        return auxiliary_motor_target_omega_;
    }

    auxiliary_motor_smooth_angle_ =
        auxiliary_motor_profile_start_angle_ +
        auxiliary_motor_profile_direction_ * profile_distance;
    auxiliary_motor_target_omega_ =
        auxiliary_motor_profile_direction_ * profile_speed;
    return auxiliary_motor_target_omega_;
}

bool Class_Chariot_Lift::Is_Auxiliary_Motor_Profile_At_Target() const
{
    return auxiliary_motor_profile_initialized_ &&
           !auxiliary_motor_profile_active_ &&
           std::fabs(auxiliary_motor_smooth_angle_ - auxiliary_motor_target_angle_) <= 1e-4f;
}

void Class_Chariot_Lift::Output_Auxiliary_Motor()
{
    if (!auxiliary_motor_command_enable_)
        return;

    if (Auxiliary_Motor.Get_Status() != Motor_DM_Status_ENABLE)
    {
        auxiliary_motor_profile_initialized_ = false;
        auxiliary_motor_hold_active_ = false;
        auxiliary_motor_hold_blend_ = 0.0f;
        auxiliary_motor_hold_ready_ticks_ = 0U;
        Auxiliary_Motor.CAN_Send_Enter();
        Auxiliary_Motor.TIM_Send_PeriodElapsedCallback();
        return;
    }

    // 1) 运动段：梯形速度规划生成平滑目标角 + 前馈角速度
    const float target_omega =
        Update_Auxiliary_Motor_Profile(auxiliary_motor_target_angle_);
    const bool home_target = Auxiliary_Is_Home_Target(auxiliary_motor_target_angle_);
    const float now_angle = Auxiliary_Motor.Get_Now_Radian();
    const float feedback_error = std::fabs(now_angle - auxiliary_motor_target_angle_);
    const float feedback_omega = std::fabs(Auxiliary_Motor.Get_Now_Omega());

    // 2) 保持段状态机：规划到位且稳定 → 进入保持；被外力推出退出阈值 → 退回规划平滑滑回。
    const float hold_enter_tolerance = home_target ?
        kAuxiliaryMotorHomeHoldEnterTolerance :
        kAuxiliaryMotorHoldEnterTolerance;

    if (auxiliary_motor_hold_active_ &&
        feedback_error > kAuxiliaryMotorHoldExitTolerance)
    {
        auxiliary_motor_hold_active_ = false;
        auxiliary_motor_hold_blend_ = 0.0f;
        auxiliary_motor_hold_ready_ticks_ = 0U;
    }

    if (!auxiliary_motor_hold_active_)
    {
        if (Is_Auxiliary_Motor_Profile_At_Target() &&
            feedback_error <= hold_enter_tolerance &&
            feedback_omega <= kAuxiliaryMotorHoldEnterOmega)
        {
            if (auxiliary_motor_hold_ready_ticks_ < kAuxiliaryMotorHoldEnterStableTicks)
                ++auxiliary_motor_hold_ready_ticks_;
        }
        else
        {
            auxiliary_motor_hold_ready_ticks_ = 0U;
        }

        if (auxiliary_motor_hold_ready_ticks_ >= kAuxiliaryMotorHoldEnterStableTicks)
        {
            auxiliary_motor_hold_active_ = true;
            auxiliary_motor_hold_blend_ = 0.0f;
        }
    }

    // 3) 增益选择
    const float move_kp = home_target ? kAuxiliaryMotorHomeMoveKp : kAuxiliaryMotorMoveKp;
    const float move_kd = home_target ? kAuxiliaryMotorHomeMoveKd : kAuxiliaryMotorMoveKd;

    float control_angle;
    float control_omega;
    float control_torque;
    float control_kp;
    float control_kd;

    if (auxiliary_motor_hold_active_)
    {
        // move → hold 增益渐变，避免切入瞬间的力矩台阶
        const float hold_kp = home_target ? kAuxiliaryMotorHomeHoldKp : kAuxiliaryMotorHoldKp;
        const float hold_kd = home_target ? kAuxiliaryMotorHomeHoldKd : kAuxiliaryMotorHoldKd;
        const float hold_deadband = home_target ?
            kAuxiliaryMotorHomeHoldDeadband : kAuxiliaryMotorHoldDeadband;
        const float hold_gain_ramp_time = home_target ?
            kAuxiliaryMotorHomeHoldGainRampTimeS : kAuxiliaryMotorHoldGainRampTimeS;
        const float blend_step = (hold_gain_ramp_time > 1e-4f) ?
            (kAuxiliaryMotorControlDtS / hold_gain_ramp_time) : 1.0f;
        auxiliary_motor_hold_blend_ =
            std::fmin(1.0f, auxiliary_motor_hold_blend_ + blend_step);

        const float blend = auxiliary_motor_hold_blend_;
        const float target_error = auxiliary_motor_target_angle_ - now_angle;

        control_angle = (std::fabs(target_error) <= hold_deadband) ?
            now_angle : auxiliary_motor_target_angle_;
        control_omega = 0.0f;
        control_torque = kAuxiliaryMotorHoldTorqueNm;
        control_kp = move_kp + blend * (hold_kp - move_kp);
        control_kd = move_kd + blend * (hold_kd - move_kd);
    }
    else
    {
        // 运动段：跟随规划轨迹（偏软增益 + 角速度前馈），又快又平滑地逼近目标
        auxiliary_motor_hold_blend_ = 0.0f;
        control_angle = auxiliary_motor_smooth_angle_;
        control_omega = target_omega;
        control_torque = kAuxiliaryMotorMoveTorqueNm;
        control_kp = move_kp;
        control_kd = move_kd;
    }

    Auxiliary_Motor.Set_Control_Maintain_Postion(
        control_angle,
        control_omega,
        control_torque,
        control_kp,
        control_kd);
    Auxiliary_Motor.TIM_Send_PeriodElapsedCallback();
}

bool Class_Chariot_Lift::Is_Auxiliary_Motor_Reached(float target_angle)
{
    const float reached_tolerance = Auxiliary_Is_Home_Target(target_angle) ?
        kAuxiliaryMotorHomeReachedTolerance :
        kAuxiliaryMotorReachedTolerance;
    return Auxiliary_Motor.Get_Status() == Motor_DM_Status_ENABLE &&
           Is_Auxiliary_Motor_Profile_At_Target() &&
           auxiliary_motor_hold_active_ &&
           std::fabs(Auxiliary_Motor.Get_Now_Radian() - target_angle) <=
               reached_tolerance;
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

    /* 过程偏航保持，按触地形态选执行器：
     *  - 纯抬升轮触地(4轮)        -> 抬升差速 Target_Diff_Yaw；
     *  - 纯全向轮触地(4轮)        -> 全向 omega Stair_Chassis_Omega；
     *  - 混合支撑(一端抬升、一端全向，且全向端逐步悬空) -> 两路同时给，同一误差、同向，
     *    全向随悬空逐渐失效、抬升差速持续有效，物理上自动接力。stair auto 下 Kinematics
     *    只解算正在驱动的那个模块，故全局 Target_Diff_Yaw 只落到该端抬升轮。 */
    switch (Stair_State)
    {
        case CHARIOT_LIFT_STAIR_UP_DRIVE_ALL_WAIT_UP_FRONT:
        case CHARIOT_LIFT_STAIR_DOWN_DRIVE_ALL_EXTRA:
            Target_Diff_Yaw = kStairMotionLiftYawSign * Compute_Stair_Yaw_Hold_Omega();
            break;
        case CHARIOT_LIFT_STAIR_UP_CHASSIS_WAIT_DOWN_BACK:
        case CHARIOT_LIFT_STAIR_DOWN_CHASSIS_WAIT_UP_BACK:
            Stair_Chassis_Omega = Compute_Stair_Yaw_Hold_Omega();
            break;
        case CHARIOT_LIFT_STAIR_DOWN_FRONT_DRIVE_WAIT_DOWN_BACK:  // 前抬升驱动 + 后全向(渐悬空)
        case CHARIOT_LIFT_STAIR_UP_REAR_DRIVE_WAIT_DOWN_FRONT:    // 后抬升驱动 + 前全向(渐悬空)
        {
            // 混合阶段权限弱、最易偏，用单独的更强增益/上限。
            const float omega = Compute_Stair_Yaw_Hold_Omega(kStairMixedYawKp,
                                                             kStairMixedYawMaxRadS);
            Target_Diff_Yaw = kStairMotionLiftYawSign * omega;  // 驱动端抬升轮差速（主力）
            Stair_Chassis_Omega = omega;                        // 全向端补充，悬空后自然失效
            break;
        }
        default:
            break;
    }

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
        case STAIR_TRANSITION_TOF_JUMP_BOTH_BACK:
            // 车尾两路下视 ToF 同时突变才触发：下台阶车尾抬升杆才升起 / 上台阶才进下一状态。
            transition_ready = Is_Stair_ToF_Jumped(CHARIOT_LIFT_TOF_DOWN_BACK) &&
                               Is_Stair_ToF_Jumped(CHARIOT_LIFT_TOF_DOWN_BACK2);
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
    {
        Capture_Stair_ToF_Reference(config->sensor);
        // 车尾双路触发：除 config->sensor(DOWN_BACK) 外，再锁定 DOWN_BACK2 的入状态基准。
        if (config->transition == STAIR_TRANSITION_TOF_JUMP_BOTH_BACK)
            Capture_Stair_ToF_Reference(CHARIOT_LIFT_TOF_DOWN_BACK2);
    }
    if (state == CHARIOT_LIFT_STAIR_DOWN_ATTITUDE_CORRECT ||
        state == CHARIOT_LIFT_STAIR_UP_PRE_ATTITUDE_CORRECT ||
        state == CHARIOT_LIFT_STAIR_UP_ATTITUDE_CORRECT ||
        state == CHARIOT_LIFT_STAIR_DOWN_PRE_ATTITUDE_CORRECT)
        Stair_Attitude_Stable_Ticks = 0;
    /* 前置纠正完成、即将开始上/下台阶动作时，把"已摆正"的当前航向重新锁为目标，
     * 作为过程保持与结束后 POST 纠正的基准（否则会退回未摆正的起始航向）。 */
    if (state == CHARIOT_LIFT_STAIR_UP_RAISE_ALL ||
        state == CHARIOT_LIFT_STAIR_DOWN_CHASSIS_WAIT_UP_BACK)
        Capture_Stair_Attitude_Target();
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
        Stair_Raise_Angle[module] :
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
        Stair_Raise_Angle[module] :
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
    // 采集当前 IMU yaw，吸附到最近的正交航向（0 / ±90 / ±180）锁为目标。
    // 仅在 IMU yaw 有效时锁定；无效则目标不可用，纠正暂不介入。
    Stair_Attitude_Stable_Ticks = 0;
    Stair_Attitude_Target_Valid = Stair_Attitude_Yaw_Valid;
    if (Stair_Attitude_Target_Valid)
        Stair_Attitude_Target_Yaw = Snap_Yaw_To_Cardinal(Stair_Attitude_Yaw);
}

void Class_Chariot_Lift::Reset_Stair_Attitude_Correction()
{
    Stair_Attitude_Target_Valid = false;
    Stair_Attitude_Target_Yaw = 0.0f;
    Stair_Attitude_Stable_Ticks = 0;
    Stair_Chassis_Omega = 0.0f;
}

float Class_Chariot_Lift::Get_Stair_Yaw_Error()
{
    if (!Stair_Attitude_Target_Valid || !Stair_Attitude_Yaw_Valid)
        return 0.0f;
    return Math_Modulus_Normalization(Stair_Attitude_Target_Yaw - Stair_Attitude_Yaw,
                                      2.0f * PI);
}

float Class_Chariot_Lift::Compute_Stair_Yaw_Hold_Omega()
{
    return Compute_Stair_Yaw_Hold_Omega(kStairMotionYawKp, kStairMotionYawMaxRadS);
}

float Class_Chariot_Lift::Compute_Stair_Yaw_Hold_Omega(float kp, float max_omega)
{
    if (!Stair_Attitude_Target_Valid || !Stair_Attitude_Yaw_Valid)
        return 0.0f;

    const float err = Math_Modulus_Normalization(
        Stair_Attitude_Target_Yaw - Stair_Attitude_Yaw, 2.0f * PI);

    /* 死区内不动，避免行进中来回小幅摆动 */
    if (fabsf(err) <= kStairMotionYawDeadzoneRad)
        return 0.0f;

    return Clamp(err * kp, -max_omega, max_omega);
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

    /* 纯 IMU 纠正：目标为启动时吸附到的最近正交航向（见 Capture_Stair_Attitude_Target），
     * 用当前 IMU yaw 与该锁定目标做闭环。IMU yaw 必须有效。 */
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

        const float target_forward = Module_Diff_Target_Enable[module] ?
            Module_Target_Diff_Forward[module] : Target_Diff_Forward;
        const float target_yaw = Module_Diff_Target_Enable[module] ?
            Module_Target_Diff_Yaw[module] : Target_Diff_Yaw;

        const float forward_cmd = Clamp(target_forward,
                                        -params.max_forward_speed,
                                         params.max_forward_speed);
        const float yaw = Clamp(target_yaw,
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
        Module_Diff_Target_Enable[i] = false;
        Module_Target_Diff_Forward[i] = 0.0f;
        Module_Target_Diff_Yaw[i] = 0.0f;
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

#ifndef STAIR_TRACE_LOGGER_H
#define STAIR_TRACE_LOGGER_H

#include <array>
#include <cstdint>

// 下台阶过程数据记录器：每按一次下台阶开一个独立 CSV，便于多次跑后逐次分析车身姿态偏移。
// 用法（在 robot.cpp 内）：
//   - 下台阶启动时调用 stair_trace::BeginRun()（开新文件）；
//   - 每个控制周期调用 stair_trace::Record(sample)（按周期抽样写一行）；
//   - 下台阶结束/退出时调用 stair_trace::EndRun()（刷盘关文件）。
// 环境变量：STAIR_TRACE_ENABLE(默认开) / STAIR_TRACE_PERIOD_MS(默认2) /
//          STAIR_TRACE_FLUSH_ROWS(默认50) / STAIR_TRACE_DIR(默认 var_data/stair_trace)。
namespace stair_trace
{
constexpr int kModuleCount = 2;  // 0=FRONT, 1=REAR

// 单个抬升模块的左右驱动轮 + 抬升电机反馈（用于判断该端是否触地/悬空/打滑）。
struct ModuleTrace
{
    float target_left_omega = 0.0f;   // 指令左轮角速度(rad/s)
    float target_right_omega = 0.0f;  // 指令右轮角速度(rad/s)
    float now_left_omega = 0.0f;      // 实测左轮角速度
    float now_right_omega = 0.0f;     // 实测右轮角速度
    float now_left_torque = 0.0f;     // 实测左轮力矩(Nm)：突降常意味着该轮悬空/失载
    float now_right_torque = 0.0f;    // 实测右轮力矩
    float lift_motor_radian = 0.0f;   // 抬升电机角(rad)，反映升/收进度（杆角=电机角/3）
    float lift_now_torque = 0.0f;     // 抬升电机力矩
    int lift_status = 0;              // 抬升电机状态枚举
};

struct Sample
{
    int64_t now_ns = 0;
    int stair_state = 0;               // Enum_Chariot_Lift_Stair_State
    char stair_state_name[40] = {0};   // 状态名（自解释）

    int attitude_yaw_valid = 0;        // IMU yaw 是否有效
    int attitude_target_valid = 0;     // 锁定目标是否有效
    float imu_yaw_rad = 0.0f;          // 当前 IMU 偏航
    float target_yaw_rad = 0.0f;       // 锁定的目标偏航（吸附后的正交航向）
    float yaw_error_rad = 0.0f;        // 归一化误差(target-now)
    float yaw_error_deg = 0.0f;        // 误差(度)，分析直观用

    float stair_chassis_forward = 0.0f;  // 全向底盘前进指令(m/s)
    float stair_chassis_omega = 0.0f;    // 全向底盘偏航指令(rad/s)
    int diff_drive_enable = 0;           // 抬升轮差速驱动是否使能
    float target_diff_forward = 0.0f;    // 抬升轮差速前进指令(m/s)
    float target_diff_yaw = 0.0f;        // 抬升轮差速偏航指令(rad/s)

    uint16_t tof_down_front_cm = 0;      // 下沿前 ToF 距离(cm)
    uint16_t tof_down_back_cm = 0;       // 下沿后 ToF 距离(cm)

    std::array<ModuleTrace, kModuleCount> modules {};
};

void BeginRun();                 // 开一个新的下台阶记录文件
void Record(const Sample &sample);
void EndRun();                   // 刷盘并关闭当前文件
void SetEnabled(bool enabled);

} // namespace stair_trace

#endif // STAIR_TRACE_LOGGER_H

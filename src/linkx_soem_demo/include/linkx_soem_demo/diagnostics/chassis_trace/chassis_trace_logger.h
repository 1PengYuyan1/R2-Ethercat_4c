#ifndef CHASSIS_TRACE_LOGGER_H
#define CHASSIS_TRACE_LOGGER_H

#include <array>
#include <cstdint>

namespace chassis_trace
{
constexpr int kWheelCount = 4;

struct WheelTrace
{
    float raw_target_omega = 0.0f;
    float target_omega = 0.0f;
    float control_omega = 0.0f;
    float now_omega = 0.0f;
    float now_torque = 0.0f;
    int motor_status = 0;
    int control_status = 0;
};

struct Sample
{
    int64_t now_ns = 0;
    int source = 0;
    int chassis_enabled = 0;
    int ros_known = 0;
    int remote_known = 0;
    int ros_recent = 0;
    int remote_recent = 0;
    int disable_watchdog_alive = 0;
    int64_t ros_age_ms = -1;
    int64_t remote_age_ms = -1;

    float ros_vx = 0.0f;
    float ros_vy = 0.0f;
    float ros_omega = 0.0f;
    float remote_vx = 0.0f;
    float remote_vy = 0.0f;
    float remote_omega = 0.0f;

    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_omega = 0.0f;
    float profiled_vx = 0.0f;
    float profiled_vy = 0.0f;
    float profiled_omega = 0.0f;
    float now_vx = 0.0f;
    float now_vy = 0.0f;
    float now_omega = 0.0f;

    std::array<WheelTrace, kWheelCount> wheels {};
};

void Record(const Sample &sample);
void SetEnabled(bool enabled);
void Flush();

} // namespace chassis_trace

#endif // CHASSIS_TRACE_LOGGER_H

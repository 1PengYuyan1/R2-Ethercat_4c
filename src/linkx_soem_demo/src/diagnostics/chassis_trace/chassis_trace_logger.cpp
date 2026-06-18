#include "chassis_trace_logger.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

namespace
{
class ChassisTraceLogger
{
public:
    void Record(const chassis_trace::Sample &sample);
    void SetEnabled(bool enabled);
    void Flush();

private:
    bool configured_ = false;
    bool enabled_ = true;
    int64_t sample_period_ns_ = 2LL * 1000LL * 1000LL;
    int flush_rows_ = 100;
    int rows_since_flush_ = 0;
    int64_t last_sample_ns_ = 0;
    std::ofstream stream_;
    std::string path_;

    void Configure();
    bool Open();
    void Write_Header();
};

bool Env_Enabled(const char *name, bool fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0')
        return fallback;

    if (std::strcmp(value, "0") == 0 ||
        std::strcmp(value, "false") == 0 ||
        std::strcmp(value, "FALSE") == 0 ||
        std::strcmp(value, "off") == 0 ||
        std::strcmp(value, "OFF") == 0)
        return false;

    return true;
}

int Env_Int(const char *name, int fallback, int min_value, int max_value)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0')
        return fallback;

    char *end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value)
        return fallback;
    if (parsed < min_value)
        return min_value;
    if (parsed > max_value)
        return max_value;
    return static_cast<int>(parsed);
}

bool Ensure_Directory(const char *path)
{
    if (::mkdir(path, 0775) == 0)
        return true;
    return errno == EEXIST;
}

std::string Make_Default_Path()
{
    std::time_t now = std::time(nullptr);
    std::tm tm_now {};
    localtime_r(&now, &tm_now);

    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm_now);

    std::ostringstream oss;
    oss << "var_data/chassis_trace/chassis_trace_" << timestamp << ".csv";
    return oss.str();
}

ChassisTraceLogger &Logger()
{
    static ChassisTraceLogger logger;
    return logger;
}
}

void ChassisTraceLogger::Configure()
{
    if (configured_)
        return;

    configured_ = true;
    enabled_ = Env_Enabled("CHASSIS_TRACE_ENABLE", true);
    const int period_ms = Env_Int("CHASSIS_TRACE_PERIOD_MS", 2, 1, 1000);
    sample_period_ns_ = static_cast<int64_t>(period_ms) * 1000LL * 1000LL;
    flush_rows_ = Env_Int("CHASSIS_TRACE_FLUSH_ROWS", 100, 1, 10000);

    const char *path_env = std::getenv("CHASSIS_TRACE_PATH");
    if (path_env != nullptr && path_env[0] != '\0')
        path_ = path_env;
    else
        path_ = Make_Default_Path();
}

bool ChassisTraceLogger::Open()
{
    if (!enabled_)
        return false;
    if (stream_.is_open())
        return true;

    Ensure_Directory("var_data");
    Ensure_Directory("var_data/chassis_trace");
    stream_.open(path_, std::ios::out | std::ios::trunc);
    if (!stream_.is_open())
    {
        enabled_ = false;
        return false;
    }

    Write_Header();
    return true;
}

void ChassisTraceLogger::Write_Header()
{
    stream_
        << "now_ns,source,chassis_enabled,"
        << "ros_known,remote_known,ros_recent,remote_recent,disable_watchdog_alive,"
        << "ros_age_ms,remote_age_ms,"
        << "ros_vx,ros_vy,ros_omega,remote_vx,remote_vy,remote_omega,"
        << "target_vx,target_vy,target_omega,"
        << "profiled_vx,profiled_vy,profiled_omega,"
        << "now_vx,now_vy,now_omega";

    for (int i = 0; i < chassis_trace::kWheelCount; ++i)
    {
        stream_
            << ",w" << i << "_raw_target_omega"
            << ",w" << i << "_target_omega"
            << ",w" << i << "_control_omega"
            << ",w" << i << "_now_omega"
            << ",w" << i << "_now_torque"
            << ",w" << i << "_motor_status"
            << ",w" << i << "_control_status";
    }
    stream_ << '\n';
}

void ChassisTraceLogger::Record(const chassis_trace::Sample &sample)
{
    Configure();
    if (!enabled_)
        return;
    if (last_sample_ns_ > 0 && (sample.now_ns - last_sample_ns_) < sample_period_ns_)
        return;
    if (!Open())
        return;

    last_sample_ns_ = sample.now_ns;

    stream_ << std::fixed << std::setprecision(6)
            << sample.now_ns << ','
            << sample.source << ','
            << sample.chassis_enabled << ','
            << sample.ros_known << ','
            << sample.remote_known << ','
            << sample.ros_recent << ','
            << sample.remote_recent << ','
            << sample.disable_watchdog_alive << ','
            << sample.ros_age_ms << ','
            << sample.remote_age_ms << ','
            << sample.ros_vx << ','
            << sample.ros_vy << ','
            << sample.ros_omega << ','
            << sample.remote_vx << ','
            << sample.remote_vy << ','
            << sample.remote_omega << ','
            << sample.target_vx << ','
            << sample.target_vy << ','
            << sample.target_omega << ','
            << sample.profiled_vx << ','
            << sample.profiled_vy << ','
            << sample.profiled_omega << ','
            << sample.now_vx << ','
            << sample.now_vy << ','
            << sample.now_omega;

    for (const auto &wheel : sample.wheels)
    {
        stream_ << ','
                << wheel.raw_target_omega << ','
                << wheel.target_omega << ','
                << wheel.control_omega << ','
                << wheel.now_omega << ','
                << wheel.now_torque << ','
                << wheel.motor_status << ','
                << wheel.control_status;
    }
    stream_ << '\n';

    if (++rows_since_flush_ >= flush_rows_)
    {
        rows_since_flush_ = 0;
        stream_.flush();
    }
}

void ChassisTraceLogger::SetEnabled(bool enabled)
{
    Configure();
    enabled_ = enabled;
    if (!enabled_ && stream_.is_open())
        stream_.flush();
}

void ChassisTraceLogger::Flush()
{
    if (stream_.is_open())
        stream_.flush();
}

void chassis_trace::Record(const Sample &sample)
{
    Logger().Record(sample);
}

void chassis_trace::SetEnabled(bool enabled)
{
    Logger().SetEnabled(enabled);
}

void chassis_trace::Flush()
{
    Logger().Flush();
}

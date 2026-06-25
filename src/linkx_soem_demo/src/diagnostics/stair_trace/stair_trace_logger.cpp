#include "stair_trace_logger.h"

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
class StairTraceLogger
{
public:
    void BeginRun();
    void Record(const stair_trace::Sample &sample);
    void EndRun();
    void SetEnabled(bool enabled);

private:
    bool configured_ = false;
    bool enabled_ = true;
    int64_t sample_period_ns_ = 2LL * 1000LL * 1000LL;
    int flush_rows_ = 50;
    int rows_since_flush_ = 0;
    int64_t last_sample_ns_ = 0;
    int run_index_ = 0;
    bool pending_new_file_ = false;
    std::ofstream stream_;
    std::string dir_;
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

StairTraceLogger &Logger()
{
    static StairTraceLogger logger;
    return logger;
}
}

void StairTraceLogger::Configure()
{
    if (configured_)
        return;

    configured_ = true;
    enabled_ = Env_Enabled("STAIR_TRACE_ENABLE", true);
    const int period_ms = Env_Int("STAIR_TRACE_PERIOD_MS", 2, 1, 1000);
    sample_period_ns_ = static_cast<int64_t>(period_ms) * 1000LL * 1000LL;
    flush_rows_ = Env_Int("STAIR_TRACE_FLUSH_ROWS", 50, 1, 10000);

    const char *dir_env = std::getenv("STAIR_TRACE_DIR");
    dir_ = (dir_env != nullptr && dir_env[0] != '\0') ? dir_env : "var_data/stair_trace";
}

void StairTraceLogger::BeginRun()
{
    Configure();
    if (stream_.is_open())
    {
        stream_.flush();
        stream_.close();
    }
    ++run_index_;
    pending_new_file_ = true;
    last_sample_ns_ = 0;
    rows_since_flush_ = 0;
}

bool StairTraceLogger::Open()
{
    if (!enabled_)
        return false;
    if (stream_.is_open() && !pending_new_file_)
        return true;

    if (stream_.is_open())
        stream_.close();

    Ensure_Directory("var_data");
    Ensure_Directory(dir_.c_str());

    std::time_t now = std::time(nullptr);
    std::tm tm_now {};
    localtime_r(&now, &tm_now);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm_now);

    std::ostringstream oss;
    oss << dir_ << "/stair_down_" << timestamp << "_" << run_index_ << ".csv";
    path_ = oss.str();

    stream_.open(path_, std::ios::out | std::ios::trunc);
    pending_new_file_ = false;
    if (!stream_.is_open())
        return false;

    Write_Header();
    return true;
}

void StairTraceLogger::Write_Header()
{
    stream_
        << "now_ns,stair_state,stair_state_name,"
        << "attitude_yaw_valid,attitude_target_valid,"
        << "imu_yaw_rad,target_yaw_rad,yaw_error_rad,yaw_error_deg,"
        << "stair_chassis_forward,stair_chassis_omega,"
        << "diff_drive_enable,target_diff_forward,target_diff_yaw,"
        << "tof_down_front_cm,tof_down_back_cm";

    const char *names[stair_trace::kModuleCount] = {"front", "rear"};
    for (int i = 0; i < stair_trace::kModuleCount; ++i)
    {
        stream_
            << ',' << names[i] << "_target_left_omega"
            << ',' << names[i] << "_target_right_omega"
            << ',' << names[i] << "_now_left_omega"
            << ',' << names[i] << "_now_right_omega"
            << ',' << names[i] << "_now_left_torque"
            << ',' << names[i] << "_now_right_torque"
            << ',' << names[i] << "_lift_motor_radian"
            << ',' << names[i] << "_lift_now_torque"
            << ',' << names[i] << "_lift_status";
    }
    stream_ << '\n';
}

void StairTraceLogger::Record(const stair_trace::Sample &sample)
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
            << sample.stair_state << ','
            << sample.stair_state_name << ','
            << sample.attitude_yaw_valid << ','
            << sample.attitude_target_valid << ','
            << sample.imu_yaw_rad << ','
            << sample.target_yaw_rad << ','
            << sample.yaw_error_rad << ','
            << sample.yaw_error_deg << ','
            << sample.stair_chassis_forward << ','
            << sample.stair_chassis_omega << ','
            << sample.diff_drive_enable << ','
            << sample.target_diff_forward << ','
            << sample.target_diff_yaw << ','
            << sample.tof_down_front_cm << ','
            << sample.tof_down_back_cm;

    for (const auto &m : sample.modules)
    {
        stream_ << ','
                << m.target_left_omega << ','
                << m.target_right_omega << ','
                << m.now_left_omega << ','
                << m.now_right_omega << ','
                << m.now_left_torque << ','
                << m.now_right_torque << ','
                << m.lift_motor_radian << ','
                << m.lift_now_torque << ','
                << m.lift_status;
    }
    stream_ << '\n';

    if (++rows_since_flush_ >= flush_rows_)
    {
        rows_since_flush_ = 0;
        stream_.flush();
    }
}

void StairTraceLogger::EndRun()
{
    if (stream_.is_open())
    {
        stream_.flush();
        stream_.close();
    }
}

void StairTraceLogger::SetEnabled(bool enabled)
{
    Configure();
    enabled_ = enabled;
    if (!enabled_ && stream_.is_open())
        stream_.flush();
}

void stair_trace::BeginRun()
{
    Logger().BeginRun();
}

void stair_trace::Record(const Sample &sample)
{
    Logger().Record(sample);
}

void stair_trace::EndRun()
{
    Logger().EndRun();
}

void stair_trace::SetEnabled(bool enabled)
{
    Logger().SetEnabled(enabled);
}

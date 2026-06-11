// omni_motion_record_main.cpp
//
// R2 omni chassis straight-line/yaw drift measurement tool.
//
// What it measures:
//   1) +X/-X/+Y/-Y straight commands with omega=0.
//   2) OPS displacement/yaw drift if OPS frames are present on CAN2 std id 0x01.
//   3) Per-wheel actual/target speed ratio and torque while moving.
//   4) Least-squares effective wheel scale and suggested wheel_speed_correction.
//
// Usage:
//   sudo IFNAME=enp86s0 ros2 run linkx_soem_demo omni_motion_record
//   sudo IFNAME=enp86s0 ./install/linkx_soem_demo/lib/linkx_soem_demo/omni_motion_record \
//        --speed 0.35 --duration 5 --settle 1.0 --analysis-delay 1.0
//
// Output:
//   var_data/omni_motion_record_<timestamp>.csv
//   var_data/omni_motion_summary_<timestamp>.txt

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include "crt_chassis_omni.h"
#include "dvc_motor_dm.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "math.h"

namespace
{
constexpr int kChannelCount = 4;
constexpr uint32_t kCtrlPeriodMs = 1;
constexpr uint32_t kChassisPeriodTicks = 2;
constexpr uint32_t kAlivePeriodTicks = 100;
constexpr float kThermalLimitC = 70.0f;
constexpr float kDegToRad = 0.017453292519943295769f;
constexpr float kPi = 3.14159265358979323846f;

constexpr std::array<float, OMNI_WHEEL_NUM> kWheelTheta = {
    5.0f * kPi / 4.0f,
    7.0f * kPi / 4.0f,
    kPi / 4.0f,
    3.0f * kPi / 4.0f,
};

constexpr std::array<const char *, OMNI_WHEEL_NUM> kWheelName = {
    "W0_RB_ch1_id2_old_LF",
    "W1_RF_ch0_id2_old_LB",
    "W2_LF_ch0_id1_old_RB",
    "W3_LB_ch1_id1_old_RF",
};

ecat_master_t st_master {};
linkx_t st_linkx {};
Class_Chassis_Omni st_chassis;
std::atomic<bool> st_running {true};
std::array<float, OMNI_WHEEL_NUM> st_measured_accel {};

struct Options
{
    std::string ifname = "enp86s0";
    float speed_mps = 0.35f;
    float duration_s = 5.0f;
    float settle_s = 1.0f;
    float analysis_delay_s = 1.0f;
    float ops_yaw_offset_deg = 90.0f;
    bool yaw_hold = false;
    float yaw_kp = 1.2f * kDegToRad;
    float yaw_kd = 0.08f * kDegToRad;
    float yaw_limit_rad_s = 0.35f;
    float startup_window_s = 0.60f;
    float reach_frac = 0.95f;
    float reach_abs_tol = 0.2f;
    float reach_hold_s = 0.05f;
    int sample_hz = 100;
    std::string directions = "all";
    std::string csv_path;
    std::string summary_path;
};

struct OpsSample
{
    bool valid = false;
    float yaw_deg = 0.0f;
    float pos_x_mm = 0.0f;
    float pos_y_mm = 0.0f;
    float omega_z_deg_s = 0.0f;
    uint32_t frame_count = 0;
    uint32_t invalid_count = 0;
};

class OpsCanParser
{
public:
    void push(const uint8_t *data, uint8_t dlen)
    {
        if (data == nullptr || dlen == 0)
            return;

        if (rx_len_ + dlen > rx_.size())
        {
            const size_t drop = (rx_len_ + dlen) - rx_.size();
            drop_front(drop);
        }

        std::copy(data, data + dlen, rx_.begin() + rx_len_);
        rx_len_ += dlen;
        last_frame_ns_ = now_ns();
        parse();
    }

    OpsSample sample() const
    {
        OpsSample s;
        s.valid = connected();
        s.yaw_deg = yaw_deg_;
        s.pos_x_mm = pos_x_mm_;
        s.pos_y_mm = pos_y_mm_;
        s.omega_z_deg_s = omega_z_deg_s_;
        s.frame_count = frame_count_;
        s.invalid_count = invalid_count_;
        return s;
    }

    bool connected() const
    {
        return last_update_ns_ > 0 && (now_ns() - last_update_ns_) < 500000000LL;
    }

private:
    static constexpr uint8_t kHeader0 = 0x0D;
    static constexpr uint8_t kHeader1 = 0x0A;
    static constexpr uint8_t kTail0 = 0x0A;
    static constexpr uint8_t kTail1 = 0x0D;
    static constexpr size_t kFrameLen = 28;
    static constexpr size_t kPayloadOffset = 2;

    std::array<uint8_t, 64> rx_ {};
    size_t rx_len_ = 0;
    int64_t last_frame_ns_ = 0;
    int64_t last_update_ns_ = 0;

    float yaw_deg_ = 0.0f;
    float pos_x_mm_ = 0.0f;
    float pos_y_mm_ = 0.0f;
    float omega_z_deg_s_ = 0.0f;
    uint32_t frame_count_ = 0;
    uint32_t invalid_count_ = 0;

    static int64_t now_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    static float read_float_le(const uint8_t *bytes)
    {
        const uint32_t word = static_cast<uint32_t>(bytes[0]) |
                              (static_cast<uint32_t>(bytes[1]) << 8) |
                              (static_cast<uint32_t>(bytes[2]) << 16) |
                              (static_cast<uint32_t>(bytes[3]) << 24);
        float value = 0.0f;
        std::memcpy(&value, &word, sizeof(value));
        return value;
    }

    void drop_front(size_t n)
    {
        if (n == 0)
            return;
        if (n >= rx_len_)
        {
            rx_len_ = 0;
            return;
        }

        const size_t remain = rx_len_ - n;
        std::move(rx_.begin() + n, rx_.begin() + rx_len_, rx_.begin());
        rx_len_ = remain;
    }

    void parse()
    {
        while (true)
        {
            size_t header = rx_len_;
            for (size_t i = 0; i + 1 < rx_len_; ++i)
            {
                if (rx_[i] == kHeader0 && rx_[i + 1] == kHeader1)
                {
                    header = i;
                    break;
                }
            }

            if (header == rx_len_)
            {
                if (rx_len_ > 0)
                {
                    rx_[0] = rx_[rx_len_ - 1];
                    rx_len_ = 1;
                }
                return;
            }

            if (header > 0)
                drop_front(header);

            if (rx_len_ < kFrameLen)
                return;

            if (rx_[kFrameLen - 2] != kTail0 || rx_[kFrameLen - 1] != kTail1)
            {
                drop_front(2);
                continue;
            }

            const uint8_t *payload = rx_.data() + kPayloadOffset;
            const float yaw = read_float_le(payload + 0);
            const float pos_x = read_float_le(payload + 12);
            const float pos_y = read_float_le(payload + 16);
            const float omega_z = read_float_le(payload + 20);
            frame_count_++;

            if (std::isfinite(yaw) && std::isfinite(pos_x) &&
                std::isfinite(pos_y) && std::isfinite(omega_z))
            {
                yaw_deg_ = yaw;
                pos_x_mm_ = pos_x;
                pos_y_mm_ = pos_y;
                omega_z_deg_s_ = omega_z;
                last_update_ns_ = now_ns();
            }
            else
            {
                invalid_count_++;
            }

            drop_front(kFrameLen);
        }
    }
};

OpsCanParser st_ops;

struct Segment
{
    std::string name;
    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;
};

struct Stats
{
    double sum = 0.0;
    double sum_sq = 0.0;
    double min = std::numeric_limits<double>::infinity();
    double max = -std::numeric_limits<double>::infinity();
    uint64_t n = 0;

    void add(double v)
    {
        if (!std::isfinite(v))
            return;
        sum += v;
        sum_sq += v * v;
        min = std::min(min, v);
        max = std::max(max, v);
        n++;
    }

    double mean() const
    {
        return n > 0 ? sum / static_cast<double>(n) : 0.0;
    }

    double stddev() const
    {
        if (n < 2)
            return 0.0;
        const double m = mean();
        const double var = (sum_sq / static_cast<double>(n)) - m * m;
        return std::sqrt(std::max(0.0, var));
    }
};

struct SegmentResult
{
    Segment segment;
    bool ops_valid = false;
    float measure_dt_s = 0.0f;
    float body_dx_m = 0.0f;
    float body_dy_m = 0.0f;
    float body_vx_mps = 0.0f;
    float body_vy_mps = 0.0f;
    float yaw_delta_deg = 0.0f;
    float yaw_rate_deg_s = 0.0f;
    float along_m = 0.0f;
    float lateral_m = 0.0f;
    float lateral_mm_per_m = 0.0f;
    float yaw_deg_per_m = 0.0f;
    OpsSample ops_start;
    OpsSample ops_end;

    Stats odom_vx;
    Stats odom_vy;
    Stats odom_omega;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_target;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_raw_target;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_profile_target;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_actual;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_ratio;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_torque;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_cmd_torque;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_cmd_accel;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_accel_filtered;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_measured_accel;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_raw_error;
    std::array<Stats, OMNI_WHEEL_NUM> wheel_profile_error;
    std::array<Stats, OMNI_WHEEL_NUM> startup_abs_raw_error;
    std::array<Stats, OMNI_WHEEL_NUM> startup_abs_profile_error;
    std::array<Stats, OMNI_WHEEL_NUM> startup_cmd_accel;
    std::array<Stats, OMNI_WHEEL_NUM> startup_measured_accel;
    std::array<bool, OMNI_WHEEL_NUM> wheel_reached {};
    std::array<float, OMNI_WHEEL_NUM> wheel_reach_time_s {};
};

struct LeastSquares4
{
    double ata[4][4] {};
    double aty[4] {};
    int rows = 0;

    void add_row(const std::array<double, 4> &a, double y)
    {
        for (int r = 0; r < 4; ++r)
        {
            aty[r] += a[r] * y;
            for (int c = 0; c < 4; ++c)
                ata[r][c] += a[r] * a[c];
        }
        rows++;
    }
};

void on_signal(int)
{
    st_running.store(false);
    st_master.is_running = false;
}

float normalize_angle_deg(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

float env_f(const char *name, float fallback)
{
    const char *v = std::getenv(name);
    if (v == nullptr || v[0] == '\0')
        return fallback;
    char *end = nullptr;
    const float parsed = std::strtof(v, &end);
    return (end == v) ? fallback : parsed;
}

int env_i(const char *name, int fallback)
{
    const char *v = std::getenv(name);
    if (v == nullptr || v[0] == '\0')
        return fallback;
    return std::atoi(v);
}

const char *cli_get(int argc, char **argv, const char *key, const char *fallback)
{
    const std::string k1 = std::string("--") + key;
    const std::string k2 = k1 + "=";
    for (int i = 1; i < argc; ++i)
    {
        if (k1 == argv[i] && (i + 1) < argc)
            return argv[i + 1];
        const std::string arg = argv[i];
        if (arg.compare(0, k2.size(), k2) == 0)
            return argv[i] + k2.size();
    }
    return fallback;
}

bool cli_has(int argc, char **argv, const char *key)
{
    const std::string k = std::string("--") + key;
    for (int i = 1; i < argc; ++i)
        if (k == argv[i])
            return true;
    return false;
}

std::string timestamp_string()
{
    const std::time_t now = std::time(nullptr);
    std::tm tm_now {};
    localtime_r(&now, &tm_now);
    char buf[32] {};
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_now);
    return std::string(buf);
}

void print_usage(const char *argv0)
{
    std::cerr
        << "Usage:\n"
        << "  sudo IFNAME=enp86s0 " << argv0 << " [options]\n\n"
        << "Options:\n"
        << "  --ifname IFACE           EtherCAT NIC, default IFNAME env or enp86s0\n"
        << "  --speed MPS             straight command speed, default 0.35\n"
        << "  --duration SEC          command duration per segment, default 5.0\n"
        << "  --settle SEC            zero-command settle time before each segment, default 1.0\n"
        << "  --analysis-delay SEC    ignored time after command start, default 1.0\n"
        << "  --ops-yaw-offset-deg D  OPS yaw to chassis body yaw offset, default -90\n"
        << "  --yaw-hold 0|1         close yaw with OPS during test, default 0\n"
        << "  --yaw-kp K             yaw hold proportional gain, rad/s per deg, default 0.02094\n"
        << "  --yaw-kd K             yaw hold damping gain, rad/s per deg/s, default 0.00140\n"
        << "  --yaw-limit W          yaw correction limit rad/s, default 0.35\n"
        << "  --startup-window SEC   startup diagnostic window, default 0.60\n"
        << "  --reach-frac F         wheel reach threshold fraction, default 0.95\n"
        << "  --reach-abs-tol RADS   wheel reach absolute tolerance, default 0.2\n"
        << "  --reach-hold SEC       wheel threshold hold time, default 0.05\n"
        << "  --sample-hz HZ          CSV sample rate, default 100\n"
        << "  --directions LIST       all or comma list: x+,x-,y+,y-\n"
        << "  --csv PATH              CSV output path\n"
        << "  --summary PATH          summary output path\n";
}

std::vector<std::string> split_csv(const std::string &text)
{
    std::vector<std::string> out;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
            out.push_back(item);
    }
    return out;
}

std::vector<Segment> make_segments(const Options &opt)
{
    const std::vector<std::string> names =
        (opt.directions == "all") ? std::vector<std::string>{"x+", "x-", "y+", "y-"}
                                  : split_csv(opt.directions);

    std::vector<Segment> segments;
    for (const auto &name : names)
    {
        if (name == "x+")
            segments.push_back({"x+", opt.speed_mps, 0.0f, 0.0f});
        else if (name == "x-")
            segments.push_back({"x-", -opt.speed_mps, 0.0f, 0.0f});
        else if (name == "y+")
            segments.push_back({"y+", 0.0f, opt.speed_mps, 0.0f});
        else if (name == "y-")
            segments.push_back({"y-", 0.0f, -opt.speed_mps, 0.0f});
        else
            std::cerr << "[OMNI-REC][WARN] unknown direction '" << name << "', skipped.\n";
    }
    return segments;
}

void dispatch_wheel_feedback(uint8_t ch, uint32_t can_id, uint8_t *data, uint8_t dlen)
{
    const uint32_t id_std = can_id & 0x7FFU;

    auto dispatch = [&](const int *indices, int n) -> bool {
        for (int k = 0; k < n; ++k)
        {
            const int i = indices[k];
            if (id_std == st_chassis.Motor_Wheel[i].DM_CAN_Rx_ID)
            {
                st_chassis.Motor_Wheel[i].CAN_RxCpltCallback(data);
                return true;
            }
        }
        return false;
    };

    if (ch == 0)
    {
        const int indices[] = {1, 2};
        dispatch(indices, 2);
    }
    else if (ch == 1)
    {
        const int indices[] = {0, 3};
        dispatch(indices, 2);
    }
    else if (ch == 2 && id_std == 0x01U)
    {
        st_ops.push(data, dlen);
    }
}

bool temperature_safe()
{
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const float mos_c = st_chassis.Motor_Wheel[i].Get_Now_MOS_Temperature() - CELSIUS_TO_KELVIN;
        const float rotor_c = st_chassis.Motor_Wheel[i].Get_Now_Rotor_Temperature() - CELSIUS_TO_KELVIN;
        const bool mos_valid = mos_c > -50.0f && mos_c < 200.0f;
        const bool rotor_valid = rotor_c > -50.0f && rotor_c < 200.0f;
        if ((mos_valid && mos_c >= kThermalLimitC) ||
            (rotor_valid && rotor_c >= kThermalLimitC))
        {
            std::cerr << "\n[OMNI-REC][THERMAL] " << kWheelName[i]
                      << " MOS=" << mos_c
                      << "C Rotor=" << rotor_c
                      << "C limit=" << kThermalLimitC << "C\n";
            return false;
        }
    }
    return true;
}

bool ec_step(uint32_t tick)
{
    if (!st_running.load() || !st_master.is_running)
        return false;

    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);

    can_msg_t msg {};
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
    {
        while (linkx_quick_recv(&st_linkx, ch, &msg))
            dispatch_wheel_feedback(ch, msg.id, msg.data, msg.dlen);
    }

    if ((tick % kAlivePeriodTicks) == 0U)
        st_chassis.TIM_100ms_Alive_PeriodElapsedCallback();

    if ((tick % kChassisPeriodTicks) == 0U)
    {
        st_chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
        st_chassis.TIM_2ms_Control_PeriodElapsedCallback();
    }

    linkx_send_pdos(&st_linkx);
    return true;
}

void set_chassis_command(float vx, float vy, float omega)
{
    st_chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_ENABLE);
    st_chassis.Set_Target_Velocity_X(vx);
    st_chassis.Set_Target_Velocity_Y(vy);
    st_chassis.Set_Target_Omega(omega);
}

void run_for_ms(uint32_t &tick, uint32_t ms)
{
    auto next_wakeup = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < ms && st_running.load(); ++i)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        if (!ec_step(tick++))
            return;
        std::this_thread::sleep_until(next_wakeup);
    }
}

float expected_wheel_omega(int i, float vx, float vy, float omega)
{
    const float wheel_linear = -vx * std::sin(kWheelTheta[i]) +
                                vy * std::cos(kWheelTheta[i]) +
                                omega * Omni_Wheel_To_Core_Distance_Define;
    return (wheel_linear / Omni_Wheel_Radius_Define) *
           st_chassis.wheel_params_[i].wheel_direction *
           st_chassis.wheel_params_[i].wheel_speed_correction;
}

float limited_expected_wheel_omega(int i, float vx, float vy, float omega)
{
    std::array<float, OMNI_WHEEL_NUM> wheel {};
    float max_abs = 0.0f;
    for (int k = 0; k < OMNI_WHEEL_NUM; ++k)
    {
        wheel[k] = expected_wheel_omega(k, vx, vy, omega);
        max_abs = std::max(max_abs, std::fabs(wheel[k]));
    }

    if (max_abs > OMNI_WHEEL_RELIABLE_OMEGA_LIMIT && max_abs > 1e-4f)
    {
        const float scale = OMNI_WHEEL_RELIABLE_OMEGA_LIMIT / max_abs;
        for (int k = 0; k < OMNI_WHEEL_NUM; ++k)
            wheel[k] *= scale;
    }
    return wheel[i];
}

float ideal_wheel_linear(int i, float vx, float vy, float omega)
{
    return -vx * std::sin(kWheelTheta[i]) +
            vy * std::cos(kWheelTheta[i]) +
            omega * Omni_Wheel_To_Core_Distance_Define;
}

void write_csv_header(std::ofstream &csv)
{
    csv << "segment,t_s,cmd_vx,cmd_vy,cmd_omega,profiled_vx,profiled_vy,profiled_omega,"
        << "ops_valid,ops_x_mm,ops_y_mm,ops_yaw_deg,"
        << "odom_vx,odom_vy,odom_omega";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        csv << "," << kWheelName[i] << "_ideal_omega"
            << "," << kWheelName[i] << "_raw_omega"
            << "," << kWheelName[i] << "_profile_omega"
            << "," << kWheelName[i] << "_actual_omega"
            << "," << kWheelName[i] << "_raw_error"
            << "," << kWheelName[i] << "_profile_error"
            << "," << kWheelName[i] << "_cmd_accel"
            << "," << kWheelName[i] << "_accel_filtered"
            << "," << kWheelName[i] << "_measured_accel"
            << "," << kWheelName[i] << "_cmd_torque"
            << "," << kWheelName[i] << "_actual_torque"
            << "," << kWheelName[i] << "_status"
            << "," << kWheelName[i] << "_ctrl_status";
    }
    csv << "\n";
}

void write_csv_sample(std::ofstream &csv, const Segment &seg, float t_s)
{
    const OpsSample ops = st_ops.sample();
    csv << seg.name << ","
        << std::fixed << std::setprecision(4) << t_s << ","
        << seg.vx << "," << seg.vy << "," << seg.omega << ","
        << st_chassis.Get_Profiled_Target_Velocity_X() << ","
        << st_chassis.Get_Profiled_Target_Velocity_Y() << ","
        << st_chassis.Get_Profiled_Target_Omega() << ","
        << (ops.valid ? 1 : 0) << ","
        << ops.pos_x_mm << "," << ops.pos_y_mm << "," << ops.yaw_deg << ","
        << st_chassis.Get_Now_Velocity_X() << ","
        << st_chassis.Get_Now_Velocity_Y() << ","
        << st_chassis.Get_Now_Omega();

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const float raw = st_chassis.Get_Raw_Target_Wheel_Omega(i);
        const float profile = st_chassis.Get_Target_Wheel_Omega(i);
        const float actual = st_chassis.Motor_Wheel[i].Get_Now_Omega();
        csv << "," << limited_expected_wheel_omega(i, seg.vx, seg.vy, seg.omega)
            << "," << raw
            << "," << profile
            << "," << actual
            << "," << (raw - actual)
            << "," << (profile - actual)
            << "," << st_chassis.Get_Wheel_Command_Accel(i)
            << "," << st_chassis.Get_Wheel_Accel_Filtered(i)
            << "," << st_measured_accel[i]
            << "," << st_chassis.Motor_Wheel[i].Get_Control_Torque()
            << "," << st_chassis.Motor_Wheel[i].Get_Now_Torque()
            << "," << static_cast<int>(st_chassis.Motor_Wheel[i].Get_Status())
            << "," << static_cast<int>(st_chassis.Motor_Wheel[i].Get_Now_Control_Status());
    }
    csv << "\n";
}

void accumulate_sample(SegmentResult &res, const Segment &cmd)
{
    res.odom_vx.add(st_chassis.Get_Now_Velocity_X());
    res.odom_vy.add(st_chassis.Get_Now_Velocity_Y());
    res.odom_omega.add(st_chassis.Get_Now_Omega());

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const float raw_target = st_chassis.Get_Raw_Target_Wheel_Omega(i);
        const float profile_target = st_chassis.Get_Target_Wheel_Omega(i);
        const float actual = st_chassis.Motor_Wheel[i].Get_Now_Omega();
        const float raw_error = raw_target - actual;
        const float profile_error = profile_target - actual;
        res.wheel_target[i].add(raw_target);
        res.wheel_raw_target[i].add(raw_target);
        res.wheel_profile_target[i].add(profile_target);
        res.wheel_actual[i].add(actual);
        res.wheel_torque[i].add(st_chassis.Motor_Wheel[i].Get_Now_Torque());
        res.wheel_cmd_torque[i].add(st_chassis.Motor_Wheel[i].Get_Control_Torque());
        res.wheel_cmd_accel[i].add(st_chassis.Get_Wheel_Command_Accel(i));
        res.wheel_accel_filtered[i].add(st_chassis.Get_Wheel_Accel_Filtered(i));
        res.wheel_measured_accel[i].add(st_measured_accel[i]);
        res.wheel_raw_error[i].add(raw_error);
        res.wheel_profile_error[i].add(profile_error);
        if (std::fabs(raw_target) > 0.2f)
            res.wheel_ratio[i].add(actual / raw_target);
    }
}

void finish_ops_metrics(SegmentResult &res, const Options &opt)
{
    if (!res.ops_start.valid || !res.ops_end.valid || res.measure_dt_s <= 0.1f)
    {
        res.ops_valid = false;
        return;
    }

    const float dx_m = (res.ops_end.pos_x_mm - res.ops_start.pos_x_mm) * 0.001f;
    const float dy_m = (res.ops_end.pos_y_mm - res.ops_start.pos_y_mm) * 0.001f;
    const float yaw0 = normalize_angle_deg(res.ops_start.yaw_deg +
                                           opt.ops_yaw_offset_deg) * kDegToRad;
    const float c = std::cos(yaw0);
    const float s = std::sin(yaw0);

    res.body_dx_m = c * dx_m + s * dy_m;
    res.body_dy_m = -s * dx_m + c * dy_m;
    res.body_vx_mps = res.body_dx_m / res.measure_dt_s;
    res.body_vy_mps = res.body_dy_m / res.measure_dt_s;
    res.yaw_delta_deg = normalize_angle_deg(res.ops_end.yaw_deg - res.ops_start.yaw_deg);
    res.yaw_rate_deg_s = res.yaw_delta_deg / res.measure_dt_s;

    const float cmd_norm = std::sqrt(res.segment.vx * res.segment.vx +
                                     res.segment.vy * res.segment.vy);
    if (cmd_norm > 1e-4f)
    {
        const float dir_x = res.segment.vx / cmd_norm;
        const float dir_y = res.segment.vy / cmd_norm;
        const float lat_x = -dir_y;
        const float lat_y = dir_x;
        res.along_m = res.body_dx_m * dir_x + res.body_dy_m * dir_y;
        res.lateral_m = res.body_dx_m * lat_x + res.body_dy_m * lat_y;
        if (std::fabs(res.along_m) > 0.02f)
        {
            res.lateral_mm_per_m = 1000.0f * res.lateral_m / std::fabs(res.along_m);
            res.yaw_deg_per_m = res.yaw_delta_deg / std::fabs(res.along_m);
        }
    }

    res.ops_valid = true;
}

SegmentResult run_segment(const Segment &seg, const Options &opt, uint32_t &tick, std::ofstream &csv)
{
    SegmentResult res;
    res.segment = seg;

    std::cout << "\n[OMNI-REC] segment " << seg.name
              << " cmd=(" << seg.vx << ", " << seg.vy << ", " << seg.omega
              << ") settle=" << opt.settle_s << "s duration=" << opt.duration_s << "s\n";

    set_chassis_command(0.0f, 0.0f, 0.0f);
    run_for_ms(tick, static_cast<uint32_t>(std::max(0.0f, opt.settle_s) * 1000.0f));

    const uint32_t total_ms = static_cast<uint32_t>(std::max(0.1f, opt.duration_s) * 1000.0f);
    const uint32_t analysis_start_ms =
        static_cast<uint32_t>(std::max(0.0f, opt.analysis_delay_s) * 1000.0f);
    const int sample_div = std::max(1, 1000 / std::max(1, opt.sample_hz));

    Segment current_cmd = seg;
    float yaw_target_deg = 0.0f;
    bool yaw_target_valid = false;
    if (opt.yaw_hold)
    {
        const OpsSample ops = st_ops.sample();
        if (ops.valid)
        {
            yaw_target_deg = ops.yaw_deg;
            yaw_target_valid = true;
        }
    }
    set_chassis_command(current_cmd.vx, current_cmd.vy, current_cmd.omega);

    bool measuring = false;
    uint32_t measured_ms = 0;
    std::array<float, OMNI_WHEEL_NUM> prev_omega {};
    std::array<int, OMNI_WHEEL_NUM> reach_hold_count {};
    std::array<float, OMNI_WHEEL_NUM> reach_candidate_time {};
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        prev_omega[i] = st_chassis.Motor_Wheel[i].Get_Now_Omega();
        st_measured_accel[i] = 0.0f;
        reach_hold_count[i] = 0;
        reach_candidate_time[i] = 0.0f;
    }
    const uint32_t startup_window_ms =
        static_cast<uint32_t>(std::max(0.05f, opt.startup_window_s) * 1000.0f);
    const int reach_hold_ticks =
        std::max(1, static_cast<int>(std::ceil(opt.reach_hold_s * 1000.0f)));
    auto next_wakeup = std::chrono::steady_clock::now();

    for (uint32_t ms = 0; ms < total_ms && st_running.load(); ++ms)
    {
        next_wakeup += std::chrono::milliseconds(kCtrlPeriodMs);
        current_cmd = seg;
        if (opt.yaw_hold && yaw_target_valid)
        {
            const OpsSample ops = st_ops.sample();
            if (ops.valid)
            {
                const float yaw_error_deg = normalize_angle_deg(yaw_target_deg - ops.yaw_deg);
                current_cmd.omega =
                    clamp_float(opt.yaw_kp * yaw_error_deg - opt.yaw_kd * ops.omega_z_deg_s,
                                -opt.yaw_limit_rad_s,
                                 opt.yaw_limit_rad_s);
            }
        }
        set_chassis_command(current_cmd.vx, current_cmd.vy, current_cmd.omega);

        if (!ec_step(tick++))
            break;

        const float t_s = static_cast<float>(ms + 1U) * 0.001f;
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const float actual = st_chassis.Motor_Wheel[i].Get_Now_Omega();
            st_measured_accel[i] = (actual - prev_omega[i]) * 1000.0f;
            prev_omega[i] = actual;

            const float raw = st_chassis.Get_Raw_Target_Wheel_Omega(i);
            const float profile = st_chassis.Get_Target_Wheel_Omega(i);
            const float final_target = limited_expected_wheel_omega(i, seg.vx, seg.vy, seg.omega);
            const float final_abs = std::fabs(final_target);
            if (ms < startup_window_ms)
            {
                res.startup_abs_raw_error[i].add(std::fabs(raw - actual));
                res.startup_abs_profile_error[i].add(std::fabs(profile - actual));
                res.startup_cmd_accel[i].add(st_chassis.Get_Wheel_Command_Accel(i));
                res.startup_measured_accel[i].add(st_measured_accel[i]);
            }

            if (!res.wheel_reached[i] && final_abs > 0.2f)
            {
                const float reach_threshold =
                    std::max(0.5f * final_abs,
                             std::min(opt.reach_frac * final_abs,
                                      final_abs - opt.reach_abs_tol));
                const float sign = (final_target >= 0.0f) ? 1.0f : -1.0f;
                const float signed_actual = sign * actual;

                if (signed_actual >= reach_threshold)
                {
                    if (reach_hold_count[i] == 0)
                        reach_candidate_time[i] = t_s;
                    reach_hold_count[i]++;
                    if (reach_hold_count[i] >= reach_hold_ticks)
                    {
                        res.wheel_reached[i] = true;
                        res.wheel_reach_time_s[i] = reach_candidate_time[i];
                    }
                }
                else
                {
                    reach_hold_count[i] = 0;
                }
            }
        }

        if (!measuring && ms >= analysis_start_ms)
        {
            measuring = true;
            measured_ms = 0;
            res.ops_start = st_ops.sample();
        }

        if (measuring)
        {
            accumulate_sample(res, current_cmd);
            measured_ms++;
        }

        if ((ms % static_cast<uint32_t>(sample_div)) == 0U)
            write_csv_sample(csv, current_cmd, static_cast<float>(ms) * 0.001f);

        if ((ms % 200U) == 0U)
        {
            std::cout << "  t=" << std::fixed << std::setprecision(2)
                      << (static_cast<float>(ms) * 0.001f)
                      << "s ops=" << (st_ops.sample().valid ? "OK" : "LOST")
                      << " odom=(" << st_chassis.Get_Now_Velocity_X()
                      << ", " << st_chassis.Get_Now_Velocity_Y()
                      << ", " << st_chassis.Get_Now_Omega() << ")\r"
                      << std::flush;
        }

        if (!temperature_safe())
        {
            st_running.store(false);
            break;
        }

        std::this_thread::sleep_until(next_wakeup);
    }

    res.measure_dt_s = static_cast<float>(measured_ms) * 0.001f;
    res.ops_end = st_ops.sample();
    finish_ops_metrics(res, opt);

    std::cout << "\n  done. ops_valid=" << (res.ops_valid ? "YES" : "NO")
              << " frames=" << res.ops_end.frame_count
              << " invalid=" << res.ops_end.invalid_count << "\n";

    set_chassis_command(0.0f, 0.0f, 0.0f);
    return res;
}

bool solve4(const LeastSquares4 &ls, std::array<double, 4> &x)
{
    double a[4][5] {};
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
            a[r][c] = ls.ata[r][c];
        a[r][4] = ls.aty[r];
    }

    for (int col = 0; col < 4; ++col)
    {
        int pivot = col;
        for (int r = col + 1; r < 4; ++r)
        {
            if (std::fabs(a[r][col]) > std::fabs(a[pivot][col]))
                pivot = r;
        }
        if (std::fabs(a[pivot][col]) < 1e-9)
            return false;
        if (pivot != col)
        {
            for (int c = col; c < 5; ++c)
                std::swap(a[col][c], a[pivot][c]);
        }

        const double div = a[col][col];
        for (int c = col; c < 5; ++c)
            a[col][c] /= div;

        for (int r = 0; r < 4; ++r)
        {
            if (r == col)
                continue;
            const double factor = a[r][col];
            for (int c = col; c < 5; ++c)
                a[r][c] -= factor * a[col][c];
        }
    }

    for (int i = 0; i < 4; ++i)
        x[i] = a[i][4];
    return true;
}

void add_segment_to_scale_fit(const SegmentResult &res, LeastSquares4 &ls)
{
    if (!res.ops_valid)
        return;

    const double cmd_norm = std::sqrt(res.segment.vx * res.segment.vx +
                                      res.segment.vy * res.segment.vy);
    if (cmd_norm < 0.05 || std::fabs(res.along_m) < 0.05)
        return;

    const std::array<double, 3> y = {
        res.body_vx_mps,
        res.body_vy_mps,
        res.yaw_rate_deg_s * kDegToRad,
    };

    for (int row_id = 0; row_id < 3; ++row_id)
    {
        std::array<double, 4> row {};
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const double s_i = ideal_wheel_linear(i, res.segment.vx, res.segment.vy, res.segment.omega);
            if (row_id == 0)
                row[i] = (-std::sin(kWheelTheta[i])) * 0.5 * s_i;
            else if (row_id == 1)
                row[i] = ( std::cos(kWheelTheta[i])) * 0.5 * s_i;
            else
                row[i] = (1.0 / (4.0 * Omni_Wheel_To_Core_Distance_Define)) * s_i;
        }
        ls.add_row(row, y[row_id]);
    }
}

void write_summary(const Options &opt, const std::vector<SegmentResult> &results)
{
    std::ofstream out(opt.summary_path);
    std::ostream &os = out.is_open() ? out : std::cout;

    os << "R2 omni chassis motion measurement summary\n"
       << "speed_mps=" << opt.speed_mps
       << " duration_s=" << opt.duration_s
       << " settle_s=" << opt.settle_s
       << " analysis_delay_s=" << opt.analysis_delay_s
       << " ops_yaw_offset_deg=" << opt.ops_yaw_offset_deg
       << " yaw_hold=" << (opt.yaw_hold ? 1 : 0)
       << " yaw_kp=" << opt.yaw_kp
       << " yaw_kd=" << opt.yaw_kd
       << " yaw_limit_rad_s=" << opt.yaw_limit_rad_s
       << " chassis_linear_accel_limit=" << OMNI_CHASSIS_LINEAR_ACCEL_LIMIT_M_S2
       << " chassis_linear_decel_limit=" << OMNI_CHASSIS_LINEAR_DECEL_LIMIT_M_S2
       << " chassis_ang_accel_limit=" << OMNI_CHASSIS_ANG_ACCEL_LIMIT_RAD_S2
       << " chassis_ang_decel_limit=" << OMNI_CHASSIS_ANG_DECEL_LIMIT_RAD_S2
       << " startup_window_s=" << opt.startup_window_s
       << " reach_frac=" << opt.reach_frac
       << " reach_abs_tol=" << opt.reach_abs_tol
       << " reach_hold_s=" << opt.reach_hold_s << "\n\n";

    os << "Current wheel params:\n";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const auto &p = st_chassis.wheel_params_[i];
        os << "  " << kWheelName[i]
           << " direction=" << p.wheel_direction
           << " correction=" << p.wheel_speed_correction
           << " kp=" << p.wheel_kp
           << " kd=" << p.wheel_kd
           << " stiction=" << p.wheel_stiction_torque
           << " dynamic_friction=" << p.wheel_dynamic_friction
           << " inertia=" << p.wheel_rotor_inertia
           << " ff_scale=" << p.wheel_feedforward_scale
           << " accel_limit=" << p.wheel_accel_limit
           << " decel_limit=" << p.wheel_decel_limit << "\n";
    }

    os << "\nSegments:\n";
    for (const auto &r : results)
    {
        os << "  [" << r.segment.name << "] cmd=(" << r.segment.vx << ", "
           << r.segment.vy << ", " << r.segment.omega << ")"
           << " odom_mean=(" << r.odom_vx.mean() << ", "
           << r.odom_vy.mean() << ", " << r.odom_omega.mean() << ")";
        if (r.ops_valid)
        {
            os << " ops_body_delta_m=(" << r.body_dx_m << ", " << r.body_dy_m << ")"
               << " yaw_delta_deg=" << r.yaw_delta_deg
               << " along_m=" << r.along_m
               << " lateral_m=" << r.lateral_m
               << " lateral_mm_per_m=" << r.lateral_mm_per_m
               << " yaw_deg_per_m=" << r.yaw_deg_per_m;
        }
        else
        {
            os << " ops=LOST";
        }
        os << "\n";

        os << "    wheel actual/target ratio:";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            os << " W" << i << "=" << r.wheel_ratio[i].mean()
               << "+/-" << r.wheel_ratio[i].stddev();
        os << "\n";

        os << "    startup reach_time_s:";
        double min_reach = std::numeric_limits<double>::infinity();
        double max_reach = -std::numeric_limits<double>::infinity();
        int fastest = -1;
        int slowest = -1;
        int reach_n = 0;
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            os << " W" << i << "=";
            if (r.wheel_reached[i])
            {
                const double reach = r.wheel_reach_time_s[i];
                os << reach;
                if (reach < min_reach)
                {
                    min_reach = reach;
                    fastest = i;
                }
                if (reach > max_reach)
                {
                    max_reach = reach;
                    slowest = i;
                }
                reach_n++;
            }
            else
            {
                os << "MISS";
            }
        }
        if (reach_n > 1)
            os << " spread_s=" << (max_reach - min_reach)
               << " fastest=W" << fastest
               << " slowest=W" << slowest;
        os << "\n";

        os << "    startup abs_raw_error_peak:";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            os << " W" << i << "="
               << (r.startup_abs_raw_error[i].n > 0 ? r.startup_abs_raw_error[i].max : 0.0);
        os << "\n";

        os << "    startup abs_profile_error_peak:";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            os << " W" << i << "="
               << (r.startup_abs_profile_error[i].n > 0 ? r.startup_abs_profile_error[i].max : 0.0);
        os << "\n";

        os << "    startup accel_mean cmd/measured:";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            os << " W" << i << "=" << r.startup_cmd_accel[i].mean()
               << "/" << r.startup_measured_accel[i].mean();
        os << "\n";
    }

    LeastSquares4 ls;
    for (const auto &r : results)
        add_segment_to_scale_fit(r, ls);

    std::array<double, 4> scale {};
    os << "\nWheel scale fit from OPS:\n";
    if (ls.rows >= 8 && solve4(ls, scale))
    {
        double mean_scale = 0.0;
        int positive_n = 0;
        for (double v : scale)
        {
            if (std::isfinite(v) && v > 0.05)
            {
                mean_scale += v;
                positive_n++;
            }
        }
        mean_scale = positive_n > 0 ? mean_scale / positive_n : 1.0;

        os << "  effective_scale means actual wheel contribution / ideal command.\n"
           << "  relative_correction keeps average speed; absolute_correction also corrects speed magnitude.\n";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const double current = st_chassis.wheel_params_[i].wheel_speed_correction;
            const bool usable = std::isfinite(scale[i]) && std::fabs(scale[i]) > 0.05;
            const double relative = usable ? current * mean_scale / scale[i] : current;
            const double absolute = usable ? current / scale[i] : current;

            os << "  W" << i
               << " effective_scale=" << scale[i]
               << " current_correction=" << current
               << " suggested_relative=" << relative
               << " suggested_absolute=" << absolute;
            if (scale[i] < -0.05)
                os << "  CHECK_DIRECTION_OR_INDEX";
            os << "\n";
        }
    }
    else
    {
        os << "  unavailable: OPS data missing or motion set is insufficient.\n";
    }

    os << "\nInterpretation:\n"
       << "  1) Large yaw_deg_per_m with omega command zero means wheel scale/index/direction asymmetry or yaw hold sign/connectivity issue.\n"
       << "  2) Large lateral_mm_per_m with small yaw drift means wheel scale/roller geometry/floor traction asymmetry.\n"
       << "  3) Wheel actual/target ratio far from 1 means MIT kd/friction feedforward is not tracking the commanded wheel speed.\n"
       << "  4) startup reach_time spread shows which wheel breaks synchronization during launch.\n"
       << "  5) abs_profile_error isolates motor tracking error after the trapezoid profile; abs_raw_error also includes the intentional ramp distance.\n"
       << "  6) Negative effective_scale usually means that wheel direction, physical index, or CAN mapping is wrong.\n";

    if (out.is_open())
        std::cout << "[OMNI-REC] summary written to " << opt.summary_path << "\n";
}

void disable_all(uint32_t &tick)
{
    set_chassis_command(0.0f, 0.0f, 0.0f);
    run_for_ms(tick, 300);

    st_chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);
    for (int cycle = 0; cycle < 50; ++cycle)
    {
        ecat_master_sync(&st_master);
        linkx_recv_pdos(&st_linkx);
        can_msg_t msg {};
        for (uint8_t ch = 0; ch < kChannelCount; ++ch)
        {
            while (linkx_quick_recv(&st_linkx, ch, &msg))
                dispatch_wheel_feedback(ch, msg.id, msg.data, msg.dlen);
        }
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            st_chassis.Motor_Wheel[i].CAN_Send_Exit();
        linkx_send_pdos(&st_linkx);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        tick++;
    }
}

bool init_ethercat_linkx(const std::string &ifname)
{
    if (!ecat_master_init(&st_master, ifname.c_str()))
    {
        std::cerr << "[OMNI-REC] ecat_master_init failed for " << ifname << "\n";
        return false;
    }

    linkx_init(&st_linkx, 1, &st_master.ctx);

    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);

    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        if (!linkx_set_can_baudrate(&st_linkx,
                                    static_cast<uint8_t>(ch),
                                    1, 2, 31, 8, 8, 1, 12, 3, 3))
        {
            std::cerr << "[OMNI-REC] CAN" << ch << " FDCAN 1M/5M config failed\n";
            return false;
        }
    }

    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_switch_can_channel(&st_linkx, static_cast<uint8_t>(ch), true);

    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[OMNI-REC] ecat_master_bring_online failed\n";
        return false;
    }
    return true;
}

Options parse_options(int argc, char **argv)
{
    Options opt;
    if (const char *env = std::getenv("IFNAME"))
        opt.ifname = env;

    opt.ifname = cli_get(argc, argv, "ifname", opt.ifname.c_str());
    opt.speed_mps = std::strtof(cli_get(argc, argv, "speed",
                                        std::to_string(env_f("OMNI_REC_SPEED", opt.speed_mps)).c_str()),
                                nullptr);
    opt.duration_s = std::strtof(cli_get(argc, argv, "duration",
                                         std::to_string(env_f("OMNI_REC_DURATION", opt.duration_s)).c_str()),
                                 nullptr);
    opt.settle_s = std::strtof(cli_get(argc, argv, "settle",
                                       std::to_string(env_f("OMNI_REC_SETTLE", opt.settle_s)).c_str()),
                               nullptr);
    opt.analysis_delay_s = std::strtof(
        cli_get(argc, argv, "analysis-delay",
                std::to_string(env_f("OMNI_REC_ANALYSIS_DELAY", opt.analysis_delay_s)).c_str()),
        nullptr);
    opt.ops_yaw_offset_deg = std::strtof(
        cli_get(argc, argv, "ops-yaw-offset-deg",
                std::to_string(env_f("OMNI_REC_OPS_YAW_OFFSET_DEG", opt.ops_yaw_offset_deg)).c_str()),
        nullptr);
    opt.yaw_hold = std::atoi(cli_get(argc, argv, "yaw-hold",
                                     std::to_string(env_i("OMNI_REC_YAW_HOLD", opt.yaw_hold ? 1 : 0)).c_str())) != 0;
    opt.yaw_kp = std::strtof(cli_get(argc, argv, "yaw-kp",
                                     std::to_string(env_f("OMNI_REC_YAW_KP", opt.yaw_kp)).c_str()),
                             nullptr);
    opt.yaw_kd = std::strtof(cli_get(argc, argv, "yaw-kd",
                                     std::to_string(env_f("OMNI_REC_YAW_KD", opt.yaw_kd)).c_str()),
                             nullptr);
    opt.yaw_limit_rad_s = std::strtof(
        cli_get(argc, argv, "yaw-limit",
                std::to_string(env_f("OMNI_REC_YAW_LIMIT", opt.yaw_limit_rad_s)).c_str()),
        nullptr);
    opt.startup_window_s = std::strtof(
        cli_get(argc, argv, "startup-window",
                std::to_string(env_f("OMNI_REC_STARTUP_WINDOW", opt.startup_window_s)).c_str()),
        nullptr);
    opt.reach_frac = std::strtof(cli_get(argc, argv, "reach-frac",
                                         std::to_string(env_f("OMNI_REC_REACH_FRAC", opt.reach_frac)).c_str()),
                                 nullptr);
    opt.reach_abs_tol = std::strtof(
        cli_get(argc, argv, "reach-abs-tol",
                std::to_string(env_f("OMNI_REC_REACH_ABS_TOL", opt.reach_abs_tol)).c_str()),
        nullptr);
    opt.reach_hold_s = std::strtof(cli_get(argc, argv, "reach-hold",
                                           std::to_string(env_f("OMNI_REC_REACH_HOLD", opt.reach_hold_s)).c_str()),
                                   nullptr);
    opt.sample_hz = std::atoi(cli_get(argc, argv, "sample-hz",
                                      std::to_string(env_i("OMNI_REC_SAMPLE_HZ", opt.sample_hz)).c_str()));
    opt.directions = cli_get(argc, argv, "directions",
                             std::getenv("OMNI_REC_DIRECTIONS") ? std::getenv("OMNI_REC_DIRECTIONS") : opt.directions.c_str());

    mkdir("var_data", 0755);
    const std::string ts = timestamp_string();
    opt.csv_path = cli_get(argc, argv, "csv", ("var_data/omni_motion_record_" + ts + ".csv").c_str());
    opt.summary_path = cli_get(argc, argv, "summary", ("var_data/omni_motion_summary_" + ts + ".txt").c_str());

    opt.startup_window_s = std::max(0.05f, opt.startup_window_s);
    opt.reach_frac = clamp_float(opt.reach_frac, 0.5f, 1.2f);
    opt.reach_abs_tol = std::max(0.0f, opt.reach_abs_tol);
    opt.reach_hold_s = std::max(0.0f, opt.reach_hold_s);
    opt.sample_hz = std::max(1, opt.sample_hz);
    return opt;
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    if (cli_has(argc, argv, "help"))
    {
        print_usage(argv[0]);
        return 0;
    }

    Options opt = parse_options(argc, argv);
    const auto segments = make_segments(opt);
    if (segments.empty())
    {
        std::cerr << "[OMNI-REC] no valid directions to run.\n";
        return 1;
    }

    std::cout << "===============================================\n"
              << "  R2 Omni Motion Record / Drift Measurement\n"
              << "  IFNAME   : " << opt.ifname << "\n"
              << "  speed    : " << opt.speed_mps << " m/s\n"
              << "  duration : " << opt.duration_s << " s per segment\n"
              << "  startup  : window=" << opt.startup_window_s
              << "s reach=" << opt.reach_frac
              << " or abs_tol=" << opt.reach_abs_tol
              << " hold=" << opt.reach_hold_s << "s\n"
              << "  yaw_hold : " << (opt.yaw_hold ? "ON" : "OFF") << "\n"
              << "  output   : " << opt.csv_path << "\n"
              << "===============================================\n"
              << "[SAFETY] Put the chassis on clear flat ground. Keep hands away from wheels.\n"
              << "[SAFETY] Press Ctrl+C to stop; the tool sends DM exit frames before returning.\n";

    if (!init_ethercat_linkx(opt.ifname))
        return 2;

    st_chassis.Init(&st_linkx);
    st_chassis.Init_Motor_Params();
    st_chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_ENABLE);
    set_chassis_command(0.0f, 0.0f, 0.0f);

    uint32_t tick = 0;
    std::cout << "[OMNI-REC] enabling wheel motors and waiting for feedback...\n";
    run_for_ms(tick, 1500);

    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        std::cout << "  " << kWheelName[i]
                  << " tx=0x" << std::hex << st_chassis.Motor_Wheel[i].DM_CAN_Tx_ID
                  << " rx=0x" << st_chassis.Motor_Wheel[i].DM_CAN_Rx_ID << std::dec
                  << " status=" << static_cast<int>(st_chassis.Motor_Wheel[i].Get_Status())
                  << " ctrl=" << static_cast<int>(st_chassis.Motor_Wheel[i].Get_Now_Control_Status())
                  << "\n";
    }

    std::ofstream csv(opt.csv_path);
    if (!csv.is_open())
    {
        std::cerr << "[OMNI-REC] failed to open CSV: " << opt.csv_path << "\n";
        disable_all(tick);
        return 3;
    }
    write_csv_header(csv);

    std::vector<SegmentResult> results;
    for (const auto &seg : segments)
    {
        if (!st_running.load())
            break;
        results.push_back(run_segment(seg, opt, tick, csv));
    }

    std::cout << "\n[OMNI-REC] stopping chassis...\n";
    disable_all(tick);
    csv.flush();
    csv.close();

    write_summary(opt, results);
    std::cout << "[OMNI-REC] csv written to " << opt.csv_path << "\n";
    std::cout << "[OMNI-REC] done.\n";
    return 0;
}

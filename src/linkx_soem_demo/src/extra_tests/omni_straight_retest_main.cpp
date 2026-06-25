// omni_straight_retest_main.cpp
//
// R2 omni chassis 前后直线全量重测工具 (大纲阶段 E/G + IMU 真值采集)
//
// 与 omni_motion_record 的区别 / 增量:
//   1) 单进程同时跑 EtherCAT 直驱底盘 + 后台 ROS 线程订阅 /IMU_data, 把 IMU 航向
//      (yaw) 作为整车真值, 和电机数据写入同一条 CSV、同一时间轴。
//   2) 按 "前后 5m、速度>=0.5、反复 N 次" 协议自动跑 (默认 x+ 去 / x- 回, 在同一条
//      ~5m 走廊里来回), 每个速度档重复 repeats 次。
//   3) 每一趟(run)在线计算可直接分析的整车指标:
//        - delta_psi0 : 起步前 1m 内 IMU yaw 最大偏离 (起步甩头, 大纲核心指标)
//        - yaw_pp     : 全程 yaw 峰峰
//        - yaw_final  : 终点 yaw 相对起点偏离
//        - lateral_m  : IMU 航向估算的横向漂移 = ∫ v·sin(Δyaw) dt  (e_lat 的真值估计,
//                       不依赖全向轮打滑后不可信的里程计)
//        - e_lat_pct  : 100*|lateral_m|/distance
//        - wheel_ratio: 四轮 actual/target 比 (稳态对称性)
//        - reach_time : 四轮到达目标轮速时间 + 极差/最快/最慢轮 (起步同步性)
//   4) 输出机器可读 JSON (var_data/omni/..._summary.json) + 逐样本 CSV + 人读 txt,
//      供 codex 直接解析、对照阈值、给出调参建议。见同目录 RUNBOOK。
//
// 用法:
//   sudo IFNAME=enp4s0 ./install/linkx_soem_demo/lib/linkx_soem_demo/omni_straight_retest \
//        --speeds 0.5,0.8,1.0 --distance 5.0 --repeats 10 --directions x+,x-
//
// 安全: 需 >=6m 净空直线走廊; 急停 Ctrl+C (会先发 DM exit)。架空联调用 --distance 小值。

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

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
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRad2Deg = 180.0f / kPi;

constexpr std::array<const char *, OMNI_WHEEL_NUM> kWheelName = {
    "W0_RB", "W1_RF", "W2_LF", "W3_LB",
};

ecat_master_t st_master {};
linkx_t st_linkx {};
Class_Chassis_Omni st_chassis;
std::atomic<bool> st_running {true};
std::array<float, OMNI_WHEEL_NUM> st_measured_accel {};

// ---- IMU (后台 ROS 线程写, 控制循环读) ----
std::atomic<bool> st_imu_valid {false};
std::atomic<float> st_imu_yaw {0.0f};       // rad, normalized [-pi,pi]
std::atomic<float> st_imu_omega_z {0.0f};
std::atomic<int64_t> st_imu_ns {0};

int64_t now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

float normalize_pi(float a)
{
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

struct Options
{
    std::string ifname = "enp86s0";
    int slave_id = 2;
    std::string imu_topic = "/IMU_data";
    std::vector<float> speeds = {0.5f, 0.8f, 1.0f};
    float distance_m = 5.0f;
    int repeats = 10;
    std::string directions = "x+,x-";
    float settle_s = 1.0f;
    float analysis_delay_s = 0.3f;
    float first_meter_m = 1.0f;          // delta_psi0 统计窗 (起步 N 米)
    float max_pass_factor = 2.5f;        // 单趟最大时长 = distance/speed*factor + 3s
    float imu_timeout_ms = 150.0f;
    int sample_hz = 100;
    float reach_frac = 0.95f;
    float reach_abs_tol = 0.2f;
    float reach_hold_s = 0.05f;
    // 参数覆盖 (扫参用, 与 omni_motion_record 一致)
    bool has_kd_override = false;        float kd_override = 0.0f;
    bool has_kd_list = false;            std::array<float, OMNI_WHEEL_NUM> kd_list {};
    bool has_corr = false;              std::array<float, OMNI_WHEEL_NUM> corr {};
    bool has_ff = false;                std::array<float, OMNI_WHEEL_NUM> ff {};
    bool has_dir = false;               std::array<float, OMNI_WHEEL_NUM> dir {};
    float breakaway_scale = 1.0f;
    float vel_kp = 0.0f;                 // 软件速度环增益 (默认 0 = 关闭)
    float vel_ki = 0.0f;
    float vel_i_limit = 1.5f;            // 积分力矩限幅 N·m (= OMNI_WHEEL_TORQUE_FF_LIMIT_NM)
    float torque_ff_limit = OMNI_WHEEL_TORQUE_FF_LIMIT_NM;
    std::string csv_path;
    std::string summary_path;
    std::string json_path;
};

struct Segment { std::string name; float vx; float vy; };

struct Stats
{
    double sum = 0.0, sum_sq = 0.0;
    double min = std::numeric_limits<double>::infinity();
    double max = -std::numeric_limits<double>::infinity();
    uint64_t n = 0;
    void add(double v)
    {
        if (!std::isfinite(v)) return;
        sum += v; sum_sq += v * v;
        min = std::min(min, v); max = std::max(max, v); n++;
    }
    double mean() const { return n > 0 ? sum / (double)n : 0.0; }
    double stddev() const
    {
        if (n < 2) return 0.0;
        const double m = mean();
        return std::sqrt(std::max(0.0, sum_sq / (double)n - m * m));
    }
};

struct RunMetrics
{
    float speed = 0.0f;
    std::string direction;
    int repeat = 0;
    bool imu_valid = false;
    float yaw_start_deg = 0.0f;
    float delta_psi0_deg = 0.0f;
    float yaw_pp_deg = 0.0f;
    float yaw_final_dev_deg = 0.0f;
    float lateral_m = 0.0f;
    float e_lat_pct = 0.0f;
    float distance_m = 0.0f;
    float pass_time_s = 0.0f;
    std::array<float, OMNI_WHEEL_NUM> wheel_ratio_mean {};
    std::array<float, OMNI_WHEEL_NUM> wheel_ratio_std {};
    std::array<float, OMNI_WHEEL_NUM> reach_time {};
    std::array<bool, OMNI_WHEEL_NUM> reached {};
    float reach_spread_s = 0.0f;
    int fastest = -1, slowest = -1;
};

// ---------- ROS IMU 节点 ----------
class ImuNode : public rclcpp::Node
{
public:
    explicit ImuNode(const std::string &topic) : Node("omni_straight_retest_imu")
    {
        sub_ = create_subscription<sensor_msgs::msg::Imu>(
            topic, rclcpp::SensorDataQoS(),
            [](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                const float w = (float)msg->orientation.w;
                const float x = (float)msg->orientation.x;
                const float y = (float)msg->orientation.y;
                const float z = (float)msg->orientation.z;
                const float yaw = std::atan2(2.0f * (w * z + x * y),
                                             1.0f - 2.0f * (y * y + z * z));
                st_imu_yaw.store(normalize_pi(yaw));
                st_imu_omega_z.store((float)msg->angular_velocity.z);
                st_imu_ns.store(now_ns());
                st_imu_valid.store(true);
            });
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

bool imu_fresh(float timeout_ms)
{
    if (!st_imu_valid.load()) return false;
    const int64_t age = now_ns() - st_imu_ns.load();
    return age <= (int64_t)(timeout_ms * 1.0e6);
}

// ---------- CLI ----------
void on_signal(int) { st_running.store(false); st_master.is_running = false; }

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float env_f(const char *n, float fb)
{
    const char *v = std::getenv(n);
    if (!v || !v[0]) return fb;
    char *e = nullptr; float p = std::strtof(v, &e); return e == v ? fb : p;
}
int env_i(const char *n, int fb) { const char *v = std::getenv(n); return (!v || !v[0]) ? fb : std::atoi(v); }

const char *cli_get(int argc, char **argv, const char *key, const char *fb)
{
    const std::string k1 = std::string("--") + key, k2 = k1 + "=";
    for (int i = 1; i < argc; ++i)
    {
        if (k1 == argv[i] && i + 1 < argc) return argv[i + 1];
        const std::string a = argv[i];
        if (a.compare(0, k2.size(), k2) == 0) return argv[i] + k2.size();
    }
    return fb;
}
bool cli_has(int argc, char **argv, const char *key)
{
    const std::string k = std::string("--") + key;
    for (int i = 1; i < argc; ++i) if (k == argv[i]) return true;
    return false;
}

std::vector<std::string> split_csv(const std::string &t)
{
    std::vector<std::string> o; std::stringstream ss(t); std::string it;
    while (std::getline(ss, it, ',')) if (!it.empty()) o.push_back(it);
    return o;
}
std::vector<float> parse_floats(const std::string &t)
{
    std::vector<float> o;
    for (const auto &s : split_csv(t)) { char *e = nullptr; float v = std::strtof(s.c_str(), &e); if (e != s.c_str()) o.push_back(v); }
    return o;
}
bool parse_f4(const std::string &t, std::array<float, OMNI_WHEEL_NUM> &out)
{
    const auto it = split_csv(t); if (it.size() != OMNI_WHEEL_NUM) return false;
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i) { char *e = nullptr; float v = std::strtof(it[i].c_str(), &e); if (e == it[i].c_str()) return false; out[i] = v; }
    return true;
}

std::string timestamp_string()
{
    const std::time_t now = std::time(nullptr); std::tm tm {}; localtime_r(&now, &tm);
    char buf[32] {}; std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm); return buf;
}

void print_usage(const char *a0)
{
    std::cerr
        << "Usage:\n  sudo IFNAME=enp4s0 " << a0 << " [options]\n\n"
        << "  --ifname IFACE        EtherCAT NIC (default IFNAME env or enp86s0)\n"
        << "  --slave-id N          motor slave (default R2_MOTOR_SLAVE or 2)\n"
        << "  --imu-topic T         IMU topic, default /IMU_data\n"
        << "  --speeds LIST         m/s list, default 0.5,0.8,1.0\n"
        << "  --distance M          per-pass distance, default 5.0\n"
        << "  --repeats N           repeats per speed, default 10\n"
        << "  --directions LIST     x+,x- (default) / y+,y-\n"
        << "  --settle SEC          zero-cmd settle between passes, default 1.0\n"
        << "  --analysis-delay SEC  ignored window before stats, default 0.3\n"
        << "  --first-meter M       delta_psi0 window distance, default 1.0\n"
        << "  --imu-timeout-ms MS   IMU freshness, default 150\n"
        << "  --sample-hz HZ        CSV rate, default 100\n"
        << "  --kd K / --kds a,b,c,d        override MIT Kd\n"
        << "  --corrections / --ff-scales a,b,c,d   override\n"
        << "  --wheel-directions a,b,c,d   override signs\n"
        << "  --breakaway-scale S   scale all breakaway torques\n"
        << "  --vel-kp K / --vel-ki K      software wheel velocity loop gains (0=off)\n"
        << "  --vel-i-limit NM             integral torque clamp, default 1.5\n"
        << "  --torque-ff-limit NM         final MIT torque feedforward clamp, default 1.5\n"
        << "  --csv / --summary / --json PATH\n";
}

// ---------- EtherCAT ----------
void dispatch_wheel_feedback(uint8_t ch, uint32_t can_id, uint8_t *data, uint8_t)
{
    const uint32_t id_std = can_id & 0x7FFU;
    auto disp = [&](const int *idx, int n) {
        for (int k = 0; k < n; ++k)
        {
            const int i = idx[k];
            if (id_std == st_chassis.Motor_Wheel[i].DM_CAN_Rx_ID)
            { st_chassis.Motor_Wheel[i].CAN_RxCpltCallback(data); return; }
        }
    };
    if (ch == 0) { const int idx[] = {1, 2}; disp(idx, 2); }
    else if (ch == 1) { const int idx[] = {0, 3}; disp(idx, 2); }
}

bool temperature_safe()
{
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const float mos = st_chassis.Motor_Wheel[i].Get_Now_MOS_Temperature() - CELSIUS_TO_KELVIN;
        const float rot = st_chassis.Motor_Wheel[i].Get_Now_Rotor_Temperature() - CELSIUS_TO_KELVIN;
        const bool mv = mos > -50.0f && mos < 200.0f, rv = rot > -50.0f && rot < 200.0f;
        if ((mv && mos >= kThermalLimitC) || (rv && rot >= kThermalLimitC))
        {
            std::cerr << "\n[RETEST][THERMAL] " << kWheelName[i] << " MOS=" << mos << " Rotor=" << rot << "\n";
            return false;
        }
    }
    return true;
}

bool ec_step(uint32_t tick)
{
    if (!st_running.load() || !st_master.is_running) return false;
    ecat_master_sync(&st_master);
    linkx_recv_pdos(&st_linkx);
    can_msg_t msg {};
    for (uint8_t ch = 0; ch < kChannelCount; ++ch)
        while (linkx_quick_recv(&st_linkx, ch, &msg))
            dispatch_wheel_feedback(ch, msg.id, msg.data, msg.dlen);
    if ((tick % kAlivePeriodTicks) == 0U) st_chassis.TIM_100ms_Alive_PeriodElapsedCallback();
    if ((tick % kChassisPeriodTicks) == 0U)
    {
        st_chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
        st_chassis.TIM_2ms_Control_PeriodElapsedCallback();
    }
    linkx_send_pdos(&st_linkx);
    return true;
}

void set_cmd(float vx, float vy, float omega)
{
    st_chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_ENABLE);
    st_chassis.Set_Target_Velocity_X(vx);
    st_chassis.Set_Target_Velocity_Y(vy);
    st_chassis.Set_Target_Omega(omega);
}

void run_for_ms(uint32_t &tick, uint32_t ms)
{
    auto next = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < ms && st_running.load(); ++i)
    {
        next += std::chrono::milliseconds(kCtrlPeriodMs);
        if (!ec_step(tick++)) return;
        std::this_thread::sleep_until(next);
    }
}

float expected_wheel_omega(int i, float vx, float vy)
{
    static const std::array<float, OMNI_WHEEL_NUM> th = {
        5.0f * kPi / 4.0f, 7.0f * kPi / 4.0f, kPi / 4.0f, 3.0f * kPi / 4.0f};
    const float lin = -vx * std::sin(th[i]) - vy * std::cos(th[i]);
    return (lin / Omni_Wheel_Radius_Define) *
           st_chassis.wheel_params_[i].wheel_direction *
           st_chassis.wheel_params_[i].wheel_speed_correction;
}
float limited_expected(int i, float vx, float vy)
{
    std::array<float, OMNI_WHEEL_NUM> w {}; float mx = 0.0f;
    for (int k = 0; k < OMNI_WHEEL_NUM; ++k) { w[k] = expected_wheel_omega(k, vx, vy); mx = std::max(mx, std::fabs(w[k])); }
    if (mx > OMNI_WHEEL_RELIABLE_OMEGA_LIMIT && mx > 1e-4f)
        for (int k = 0; k < OMNI_WHEEL_NUM; ++k) w[k] *= OMNI_WHEEL_RELIABLE_OMEGA_LIMIT / mx;
    return w[i];
}

void write_csv_header(std::ofstream &csv)
{
    csv << "speed,direction,repeat,t_s,cmd_vx,cmd_vy,"
        << "profiled_vx,profiled_vy,odom_vx,odom_vy,odom_omega,"
        << "imu_valid,imu_yaw_deg,imu_dyaw_deg,imu_omega_z,imu_age_ms,est_dist_m,est_lat_m";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        csv << "," << kWheelName[i] << "_ideal" << "," << kWheelName[i] << "_raw"
            << "," << kWheelName[i] << "_profile" << "," << kWheelName[i] << "_actual"
            << "," << kWheelName[i] << "_ratio" << "," << kWheelName[i] << "_cmd_accel"
            << "," << kWheelName[i] << "_cmd_torque" << "," << kWheelName[i] << "_actual_torque"
            << "," << kWheelName[i] << "_vel_i"
            << "," << kWheelName[i] << "_mos_c" << "," << kWheelName[i] << "_rotor_c";
    csv << "\n";
}

// 跑一趟 (一个方向, distance 米)
RunMetrics run_pass(const Segment &seg, float speed, int repeat,
                    const Options &opt, uint32_t &tick, std::ofstream &csv)
{
    RunMetrics m;
    m.speed = speed; m.direction = seg.name; m.repeat = repeat;

    std::cout << "\n[RETEST] speed=" << speed << " dir=" << seg.name
              << " rep=" << repeat << " dist=" << opt.distance_m << "m\n";

    // settle
    set_cmd(0.0f, 0.0f, 0.0f);
    run_for_ms(tick, (uint32_t)(std::max(0.0f, opt.settle_s) * 1000.0f));

    const float nx = seg.vx, ny = seg.vy;       // 命令方向
    const float nrm = std::sqrt(nx * nx + ny * ny);
    const float ux = nrm > 1e-6f ? nx / nrm : 0.0f;
    const float uy = nrm > 1e-6f ? ny / nrm : 0.0f;

    const uint32_t max_ms = (uint32_t)(opt.distance_m / std::max(0.1f, speed) * opt.max_pass_factor * 1000.0f + 3000.0f);
    const uint32_t analysis_ms = (uint32_t)(std::max(0.0f, opt.analysis_delay_s) * 1000.0f);
    const int sample_div = std::max(1, 1000 / std::max(1, opt.sample_hz));
    const int reach_hold_ticks = std::max(1, (int)std::ceil(opt.reach_hold_s * 1000.0f));

    bool yaw_started = false;
    float yaw_start = 0.0f, dyaw_min = 0.0f, dyaw_max = 0.0f, dyaw_last = 0.0f;
    float est_dist = 0.0f, est_lat = 0.0f;
    std::array<Stats, OMNI_WHEEL_NUM> ratio;
    std::array<int, OMNI_WHEEL_NUM> hold {}; std::array<float, OMNI_WHEEL_NUM> cand {};
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i) { m.reached[i] = false; m.reach_time[i] = std::numeric_limits<float>::quiet_NaN(); }

    auto next = std::chrono::steady_clock::now();
    uint32_t ms = 0;
    for (; ms < max_ms && st_running.load(); ++ms)
    {
        next += std::chrono::milliseconds(kCtrlPeriodMs);
        set_cmd(seg.vx, seg.vy, 0.0f);
        if (!ec_step(tick++)) break;

        const float dt = 0.001f;
        const float t_s = (float)(ms + 1) * dt;
        const float pvx = st_chassis.Get_Profiled_Target_Velocity_X();
        const float pvy = st_chassis.Get_Profiled_Target_Velocity_Y();
        const float v_fwd = pvx * ux + pvy * uy;        // 命令方向上的(指令)前进速度
        est_dist += v_fwd * dt;

        const bool fresh = imu_fresh(opt.imu_timeout_ms);
        float yaw_deg = 0.0f, dyaw_deg = 0.0f;
        if (fresh)
        {
            const float yaw = st_imu_yaw.load();
            if (!yaw_started) { yaw_start = yaw; yaw_started = true; m.imu_valid = true; m.yaw_start_deg = yaw * kRad2Deg; }
            const float dyaw = normalize_pi(yaw - yaw_start);
            yaw_deg = yaw * kRad2Deg; dyaw_deg = dyaw * kRad2Deg;
            dyaw_min = std::min(dyaw_min, dyaw); dyaw_max = std::max(dyaw_max, dyaw); dyaw_last = dyaw;
            est_lat += v_fwd * std::sin(dyaw) * dt;
            if (est_dist <= opt.first_meter_m)
                m.delta_psi0_deg = std::max(m.delta_psi0_deg, std::fabs(dyaw) * kRad2Deg);
        }

        // 轮速到达时间
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
        {
            const float actual = st_chassis.Motor_Wheel[i].Get_Now_Omega();
            const float fin = limited_expected(i, seg.vx, seg.vy);
            const float fa = std::fabs(fin);
            if (!m.reached[i] && fa > 0.2f)
            {
                const float thr = std::max(0.5f * fa, std::min(opt.reach_frac * fa, fa - opt.reach_abs_tol));
                const float sgn = fin >= 0.0f ? 1.0f : -1.0f;
                if (sgn * actual >= thr)
                {
                    if (hold[i] == 0) cand[i] = t_s;
                    if (++hold[i] >= reach_hold_ticks) { m.reached[i] = true; m.reach_time[i] = cand[i]; }
                }
                else hold[i] = 0;
            }
        }

        // 稳态比 (analysis 窗后)
        if (ms >= analysis_ms)
            for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            {
                const float raw = st_chassis.Get_Raw_Target_Wheel_Omega(i);
                const float actual = st_chassis.Motor_Wheel[i].Get_Now_Omega();
                if (std::fabs(raw) > 0.2f) ratio[i].add(actual / raw);
            }

        if ((ms % (uint32_t)sample_div) == 0U)
        {
            const int64_t age_ms = fresh ? (now_ns() - st_imu_ns.load()) / 1000000LL : -1;
            csv << speed << "," << seg.name << "," << repeat << ","
                << std::fixed << std::setprecision(4) << t_s << ","
                << seg.vx << "," << seg.vy << "," << pvx << "," << pvy << ","
                << st_chassis.Get_Now_Velocity_X() << "," << st_chassis.Get_Now_Velocity_Y() << ","
                << st_chassis.Get_Now_Omega() << ","
                << (fresh ? 1 : 0) << "," << yaw_deg << "," << dyaw_deg << ","
                << (fresh ? st_imu_omega_z.load() : 0.0f) << "," << age_ms << ","
                << est_dist << "," << est_lat;
            for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
            {
                const float raw = st_chassis.Get_Raw_Target_Wheel_Omega(i);
                const float actual = st_chassis.Motor_Wheel[i].Get_Now_Omega();
                csv << "," << limited_expected(i, seg.vx, seg.vy) << "," << raw << ","
                    << st_chassis.Get_Target_Wheel_Omega(i) << "," << actual << ","
                    << (std::fabs(raw) > 0.2f ? actual / raw : 0.0f) << ","
                    << st_chassis.Get_Wheel_Command_Accel(i) << ","
                    << st_chassis.Motor_Wheel[i].Get_Control_Torque() << ","
                    << st_chassis.Motor_Wheel[i].Get_Now_Torque() << ","
                    << st_chassis.Get_Wheel_Vel_Integral(i) << ","
                    << st_chassis.Motor_Wheel[i].Get_Now_MOS_Temperature() - CELSIUS_TO_KELVIN << ","
                    << st_chassis.Motor_Wheel[i].Get_Now_Rotor_Temperature() - CELSIUS_TO_KELVIN;
            }
            csv << "\n";
        }

        if ((ms % 200U) == 0U)
            std::cout << "  t=" << std::fixed << std::setprecision(2) << t_s
                      << " dist=" << est_dist << "m dyaw=" << dyaw_deg
                      << "deg lat=" << est_lat << "m\r" << std::flush;

        if (!temperature_safe()) { st_running.store(false); break; }
        if (est_dist >= opt.distance_m) { ++ms; break; }   // 到达 5m, 收尾

        std::this_thread::sleep_until(next);
    }

    set_cmd(0.0f, 0.0f, 0.0f);

    m.distance_m = est_dist;
    m.pass_time_s = (float)ms * 0.001f;
    m.lateral_m = est_lat;
    m.e_lat_pct = est_dist > 1e-3f ? 100.0f * std::fabs(est_lat) / est_dist : 0.0f;
    m.yaw_pp_deg = (dyaw_max - dyaw_min) * kRad2Deg;
    m.yaw_final_dev_deg = dyaw_last * kRad2Deg;
    double mn = std::numeric_limits<double>::infinity(), mx = -mn;
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        m.wheel_ratio_mean[i] = (float)ratio[i].mean();
        m.wheel_ratio_std[i] = (float)ratio[i].stddev();
        if (m.reached[i]) { if (m.reach_time[i] < mn) { mn = m.reach_time[i]; m.fastest = i; }
                            if (m.reach_time[i] > mx) { mx = m.reach_time[i]; m.slowest = i; } }
    }
    if (m.fastest >= 0 && m.slowest >= 0) m.reach_spread_s = (float)(mx - mn);

    std::cout << "\n  -> dist=" << m.distance_m << "m delta_psi0=" << m.delta_psi0_deg
              << "deg yaw_pp=" << m.yaw_pp_deg << "deg e_lat=" << m.e_lat_pct
              << "% reach_spread=" << m.reach_spread_s << "s"
              << (m.imu_valid ? "" : "  [IMU MISSING]") << "\n";
    return m;
}

// ---------- 输出 ----------
void write_json(const Options &opt, const std::vector<RunMetrics> &runs)
{
    std::ofstream js(opt.json_path);
    if (!js.is_open()) { std::cerr << "[RETEST] cannot open json " << opt.json_path << "\n"; return; }
    auto jf = [](float v) -> std::string {
        if (!std::isfinite(v)) return "null";
        std::ostringstream o; o << std::setprecision(6) << v; return o.str();
    };

    js << "{\n  \"meta\": {\n"
       << "    \"distance_m\": " << opt.distance_m << ",\n"
       << "    \"repeats\": " << opt.repeats << ",\n"
       << "    \"directions\": \"" << opt.directions << "\",\n"
       << "    \"imu_topic\": \"" << opt.imu_topic << "\",\n"
       << "    \"chassis_linear_accel_limit\": " << OMNI_CHASSIS_LINEAR_ACCEL_LIMIT_M_S2 << ",\n"
       << "    \"chassis_linear_decel_limit\": " << OMNI_CHASSIS_LINEAR_DECEL_LIMIT_M_S2 << ",\n"
       << "    \"vel_loop_kp\": " << jf(opt.vel_kp) << ",\n"
       << "    \"vel_loop_ki\": " << jf(opt.vel_ki) << ",\n"
       << "    \"vel_loop_i_limit\": " << jf(opt.vel_i_limit) << ",\n"
       << "    \"torque_ff_limit\": " << jf(opt.torque_ff_limit) << ",\n"
       << "    \"wheel_params\": [\n";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const auto &p = st_chassis.wheel_params_[i];
        js << "      {\"wheel\": \"" << kWheelName[i] << "\", \"kd\": " << jf(p.wheel_kd)
           << ", \"correction\": " << jf(p.wheel_speed_correction)
           << ", \"breakaway\": " << jf(p.wheel_breakaway_torque)
           << ", \"stiction\": " << jf(p.wheel_stiction_torque)
           << ", \"ff_scale\": " << jf(p.wheel_feedforward_scale) << "}"
           << (i + 1 < OMNI_WHEEL_NUM ? "," : "") << "\n";
    }
    js << "    ]\n  },\n  \"runs\": [\n";
    for (size_t r = 0; r < runs.size(); ++r)
    {
        const auto &m = runs[r];
        js << "    {\"speed\": " << jf(m.speed) << ", \"direction\": \"" << m.direction
           << "\", \"repeat\": " << m.repeat << ", \"imu_valid\": " << (m.imu_valid ? "true" : "false")
           << ", \"distance_m\": " << jf(m.distance_m)
           << ", \"delta_psi0_deg\": " << (m.imu_valid ? jf(m.delta_psi0_deg) : "null")
           << ", \"yaw_pp_deg\": " << (m.imu_valid ? jf(m.yaw_pp_deg) : "null")
           << ", \"yaw_final_dev_deg\": " << (m.imu_valid ? jf(m.yaw_final_dev_deg) : "null")
           << ", \"lateral_m\": " << (m.imu_valid ? jf(m.lateral_m) : "null")
           << ", \"e_lat_pct\": " << (m.imu_valid ? jf(m.e_lat_pct) : "null")
           << ", \"reach_spread_s\": " << jf(m.reach_spread_s)
           << ", \"fastest_wheel\": " << m.fastest << ", \"slowest_wheel\": " << m.slowest
           << ", \"wheel_ratio\": [";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i) js << jf(m.wheel_ratio_mean[i]) << (i + 1 < OMNI_WHEEL_NUM ? ", " : "");
        js << "], \"reach_time_s\": [";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i) js << jf(m.reach_time[i]) << (i + 1 < OMNI_WHEEL_NUM ? ", " : "");
        js << "]}" << (r + 1 < runs.size() ? "," : "") << "\n";
    }
    js << "  ],\n  \"aggregate\": [\n";

    // 按 (speed,direction) 聚合
    struct Key { float s; std::string d; };
    std::vector<std::pair<std::pair<float, std::string>, std::vector<const RunMetrics *>>> groups;
    for (const auto &m : runs)
    {
        auto k = std::make_pair(m.speed, m.direction);
        auto it = std::find_if(groups.begin(), groups.end(), [&](auto &g) { return g.first == k; });
        if (it == groups.end()) groups.push_back({k, {&m}}); else it->second.push_back(&m);
    }
    for (size_t g = 0; g < groups.size(); ++g)
    {
        Stats psi, pp, elat, spread;
        std::array<Stats, OMNI_WHEEL_NUM> ratio;
        int n_imu = 0;
        for (const auto *m : groups[g].second)
        {
            if (m->imu_valid) { psi.add(m->delta_psi0_deg); pp.add(m->yaw_pp_deg); elat.add(m->e_lat_pct); n_imu++; }
            spread.add(m->reach_spread_s);
            for (int i = 0; i < OMNI_WHEEL_NUM; ++i) ratio[i].add(m->wheel_ratio_mean[i]);
        }
        js << "    {\"speed\": " << jf(groups[g].first.first) << ", \"direction\": \"" << groups[g].first.second
           << "\", \"n\": " << groups[g].second.size() << ", \"n_imu\": " << n_imu
           << ", \"delta_psi0_deg\": {\"mean\": " << jf((float)psi.mean()) << ", \"max\": " << jf((float)psi.max) << ", \"std\": " << jf((float)psi.stddev()) << "}"
           << ", \"yaw_pp_deg\": {\"mean\": " << jf((float)pp.mean()) << ", \"max\": " << jf((float)pp.max) << "}"
           << ", \"e_lat_pct\": {\"mean\": " << jf((float)elat.mean()) << ", \"max\": " << jf((float)elat.max) << ", \"std\": " << jf((float)elat.stddev()) << "}"
           << ", \"reach_spread_s\": {\"mean\": " << jf((float)spread.mean()) << ", \"max\": " << jf((float)spread.max) << "}"
           << ", \"wheel_ratio_mean\": [";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i) js << jf((float)ratio[i].mean()) << (i + 1 < OMNI_WHEEL_NUM ? ", " : "");
        js << "]}" << (g + 1 < groups.size() ? "," : "") << "\n";
    }
    js << "  ]\n}\n";
    std::cout << "[RETEST] json written to " << opt.json_path << "\n";
}

void write_summary(const Options &opt, const std::vector<RunMetrics> &runs)
{
    std::ofstream out(opt.summary_path);
    std::ostream &os = out.is_open() ? out : std::cout;
    os << "R2 omni straight-line retest summary\n"
       << "distance_m=" << opt.distance_m << " repeats=" << opt.repeats
       << " directions=" << opt.directions << "\n\n";
    os << "Current wheel params:\n";
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        const auto &p = st_chassis.wheel_params_[i];
        os << "  " << kWheelName[i] << " kd=" << p.wheel_kd << " corr=" << p.wheel_speed_correction
           << " breakaway=" << p.wheel_breakaway_torque << " stiction=" << p.wheel_stiction_torque
           << " ff=" << p.wheel_feedforward_scale << "\n";
    }
    os << "\nPer-run:\n";
    for (const auto &m : runs)
    {
        os << "  spd=" << m.speed << " " << m.direction << " rep" << m.repeat
           << " | delta_psi0=" << m.delta_psi0_deg << "deg yaw_pp=" << m.yaw_pp_deg
           << "deg e_lat=" << m.e_lat_pct << "% reach_spread=" << m.reach_spread_s << "s";
        os << " ratio=";
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i) os << "W" << i << ":" << m.wheel_ratio_mean[i] << " ";
        if (!m.imu_valid) os << " [IMU MISSING]";
        os << "\n";
    }
    os << "\nPass thresholds (大纲): e_lat<=1%, delta_psi0 small&consistent, reach_spread<=10% of rise.\n";
    if (out.is_open()) std::cout << "[RETEST] summary written to " << opt.summary_path << "\n";
}

void disable_all(uint32_t &tick)
{
    set_cmd(0.0f, 0.0f, 0.0f);
    run_for_ms(tick, 300);
    st_chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_DISABLE);
    for (int c = 0; c < 50; ++c)
    {
        ecat_master_sync(&st_master);
        linkx_recv_pdos(&st_linkx);
        can_msg_t msg {};
        for (uint8_t ch = 0; ch < kChannelCount; ++ch)
            while (linkx_quick_recv(&st_linkx, ch, &msg)) dispatch_wheel_feedback(ch, msg.id, msg.data, msg.dlen);
        for (int i = 0; i < OMNI_WHEEL_NUM; ++i) st_chassis.Motor_Wheel[i].CAN_Send_Exit();
        linkx_send_pdos(&st_linkx);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        tick++;
    }
}

bool init_ethercat_linkx(const std::string &ifname, int slave_id)
{
    if (!ecat_master_init(&st_master, ifname.c_str())) { std::cerr << "[RETEST] ecat_master_init failed " << ifname << "\n"; return false; }
    if (slave_id > st_master.ctx.slavecount) { std::cerr << "[RETEST] slave-id " << slave_id << " > " << st_master.ctx.slavecount << "\n"; return false; }
    linkx_init(&st_linkx, (uint32_t)slave_id, &st_master.ctx);
    for (int ch = 0; ch < kChannelCount; ++ch) linkx_switch_can_channel(&st_linkx, (uint8_t)ch, true);
    for (int ch = 0; ch < kChannelCount; ++ch)
        if (!linkx_set_can_baudrate(&st_linkx, (uint8_t)ch, 1, 2, 31, 8, 8, 1, 12, 3, 3))
        { std::cerr << "[RETEST] CAN" << ch << " config failed\n"; return false; }
    for (int ch = 0; ch < kChannelCount; ++ch) linkx_switch_can_channel(&st_linkx, (uint8_t)ch, true);
    if (!ecat_master_bring_online(&st_master)) { std::cerr << "[RETEST] bring_online failed\n"; return false; }
    return true;
}

std::vector<Segment> make_dir_segments(const std::string &name, float speed)
{
    if (name == "x+") return {{"x+", speed, 0.0f}};
    if (name == "x-") return {{"x-", -speed, 0.0f}};
    if (name == "y+") return {{"y+", 0.0f, speed}};
    if (name == "y-") return {{"y-", 0.0f, -speed}};
    return {};
}

Options parse_options(int argc, char **argv)
{
    Options opt;
    if (const char *e = std::getenv("IFNAME")) opt.ifname = e;
    opt.ifname = cli_get(argc, argv, "ifname", opt.ifname.c_str());
    opt.slave_id = std::atoi(cli_get(argc, argv, "slave-id", std::to_string(env_i("R2_MOTOR_SLAVE", opt.slave_id)).c_str()));
    opt.imu_topic = cli_get(argc, argv, "imu-topic", opt.imu_topic.c_str());
    if (cli_has(argc, argv, "speeds")) { auto v = parse_floats(cli_get(argc, argv, "speeds", "")); if (!v.empty()) opt.speeds = v; }
    opt.distance_m = std::strtof(cli_get(argc, argv, "distance", std::to_string(opt.distance_m).c_str()), nullptr);
    opt.repeats = std::atoi(cli_get(argc, argv, "repeats", std::to_string(opt.repeats).c_str()));
    opt.directions = cli_get(argc, argv, "directions", opt.directions.c_str());
    opt.settle_s = std::strtof(cli_get(argc, argv, "settle", std::to_string(opt.settle_s).c_str()), nullptr);
    opt.analysis_delay_s = std::strtof(cli_get(argc, argv, "analysis-delay", std::to_string(opt.analysis_delay_s).c_str()), nullptr);
    opt.first_meter_m = std::strtof(cli_get(argc, argv, "first-meter", std::to_string(opt.first_meter_m).c_str()), nullptr);
    opt.imu_timeout_ms = std::strtof(cli_get(argc, argv, "imu-timeout-ms", std::to_string(opt.imu_timeout_ms).c_str()), nullptr);
    opt.sample_hz = std::atoi(cli_get(argc, argv, "sample-hz", std::to_string(opt.sample_hz).c_str()));
    if (cli_has(argc, argv, "kd")) { opt.has_kd_override = true; opt.kd_override = std::strtof(cli_get(argc, argv, "kd", "0"), nullptr); }
    if (cli_has(argc, argv, "kds")) opt.has_kd_list = parse_f4(cli_get(argc, argv, "kds", ""), opt.kd_list);
    if (cli_has(argc, argv, "corrections")) opt.has_corr = parse_f4(cli_get(argc, argv, "corrections", ""), opt.corr);
    if (cli_has(argc, argv, "ff-scales")) opt.has_ff = parse_f4(cli_get(argc, argv, "ff-scales", ""), opt.ff);
    if (cli_has(argc, argv, "wheel-directions")) opt.has_dir = parse_f4(cli_get(argc, argv, "wheel-directions", ""), opt.dir);
    opt.breakaway_scale = std::strtof(cli_get(argc, argv, "breakaway-scale", "1"), nullptr);
    opt.vel_kp = std::strtof(cli_get(argc, argv, "vel-kp", std::to_string(opt.vel_kp).c_str()), nullptr);
    opt.vel_ki = std::strtof(cli_get(argc, argv, "vel-ki", std::to_string(opt.vel_ki).c_str()), nullptr);
    opt.vel_i_limit = std::strtof(cli_get(argc, argv, "vel-i-limit", std::to_string(opt.vel_i_limit).c_str()), nullptr);
    opt.torque_ff_limit = std::strtof(cli_get(argc, argv, "torque-ff-limit", std::to_string(opt.torque_ff_limit).c_str()), nullptr);

    mkdir("var_data", 0755);
    mkdir("var_data/omni", 0755);
    const std::string ts = timestamp_string();
    opt.csv_path = cli_get(argc, argv, "csv", ("var_data/omni/omni_straight_retest_" + ts + ".csv").c_str());
    opt.summary_path = cli_get(argc, argv, "summary", ("var_data/omni/omni_straight_retest_" + ts + ".txt").c_str());
    opt.json_path = cli_get(argc, argv, "json", ("var_data/omni/omni_straight_retest_" + ts + ".json").c_str());

    opt.sample_hz = std::max(1, opt.sample_hz);
    opt.repeats = std::max(1, opt.repeats);
    if (!std::isfinite(opt.breakaway_scale) || opt.breakaway_scale < 0.0f) opt.breakaway_scale = 1.0f;
    if (!std::isfinite(opt.torque_ff_limit) || opt.torque_ff_limit <= 0.0f) opt.torque_ff_limit = OMNI_WHEEL_TORQUE_FF_LIMIT_NM;
    if (opt.slave_id < 1) opt.slave_id = 1;
    return opt;
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    if (cli_has(argc, argv, "help")) { print_usage(argv[0]); return 0; }

    Options opt = parse_options(argc, argv);

    std::cout << "===============================================\n"
              << "  R2 Omni Straight-Line Retest (+IMU)\n"
              << "  IFNAME=" << opt.ifname << " slave=" << opt.slave_id
              << " imu=" << opt.imu_topic << "\n"
              << "  speeds="; for (float s : opt.speeds) std::cout << s << " ";
    std::cout << " distance=" << opt.distance_m << "m repeats=" << opt.repeats
              << " directions=" << opt.directions << "\n"
              << "  csv=" << opt.csv_path << "\n  json=" << opt.json_path << "\n"
              << "===============================================\n"
              << "[SAFETY] Need a clear >=6m lane. Ctrl+C sends DM exit before返回.\n";

    // ROS IMU 后台线程
    rclcpp::init(0, nullptr);
    auto imu_node = std::make_shared<ImuNode>(opt.imu_topic);
    std::thread imu_thread([&]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(imu_node);
        while (rclcpp::ok() && st_running.load()) exec.spin_some(std::chrono::milliseconds(5));
    });

    if (!init_ethercat_linkx(opt.ifname, opt.slave_id))
    { st_running.store(false); if (imu_thread.joinable()) imu_thread.join(); rclcpp::shutdown(); return 2; }

    st_chassis.Init(&st_linkx);
    st_chassis.Init_Motor_Params();
    for (int i = 0; i < OMNI_WHEEL_NUM; ++i)
    {
        if (opt.has_kd_override) st_chassis.wheel_params_[i].wheel_kd = opt.kd_override;
        if (opt.has_kd_list)     st_chassis.wheel_params_[i].wheel_kd = opt.kd_list[i];
        if (opt.has_corr)        st_chassis.wheel_params_[i].wheel_speed_correction = opt.corr[i];
        if (opt.has_ff)          st_chassis.wheel_params_[i].wheel_feedforward_scale = opt.ff[i];
        if (opt.has_dir)         st_chassis.wheel_params_[i].wheel_direction = (opt.dir[i] < 0.0f ? -1.0f : 1.0f);
        st_chassis.wheel_params_[i].wheel_breakaway_torque *= opt.breakaway_scale;
    }
    st_chassis.Set_Velocity_Loop(opt.vel_kp, opt.vel_ki, opt.vel_i_limit);
    st_chassis.Set_Torque_FF_Limit(opt.torque_ff_limit);
    if (opt.vel_kp != 0.0f || opt.vel_ki != 0.0f)
        std::cout << "[RETEST] software velocity loop ON: kp=" << opt.vel_kp
                  << " ki=" << opt.vel_ki << " i_limit=" << opt.vel_i_limit << "\n";
    if (opt.torque_ff_limit != OMNI_WHEEL_TORQUE_FF_LIMIT_NM)
        std::cout << "[RETEST] torque_ff_limit=" << opt.torque_ff_limit << " Nm\n";
    st_chassis.Set_Chassis_Control_Type(Chassis_Omni_Control_Type_ENABLE);
    set_cmd(0.0f, 0.0f, 0.0f);

    uint32_t tick = 0;
    std::cout << "[RETEST] enabling motors, waiting feedback...\n";
    run_for_ms(tick, 1500);
    if (!imu_fresh(opt.imu_timeout_ms))
        std::cout << "[RETEST][WARN] no fresh IMU on " << opt.imu_topic
                  << " — motor data still logged, IMU metrics will be null.\n";

    std::ofstream csv(opt.csv_path);
    if (!csv.is_open())
    { std::cerr << "[RETEST] cannot open csv " << opt.csv_path << "\n"; disable_all(tick); st_running.store(false); if (imu_thread.joinable()) imu_thread.join(); rclcpp::shutdown(); return 3; }
    write_csv_header(csv);

    const auto dir_names = split_csv(opt.directions);
    std::vector<RunMetrics> runs;
    for (float speed : opt.speeds)
    {
        if (!st_running.load()) break;
        for (int rep = 1; rep <= opt.repeats && st_running.load(); ++rep)
            for (const auto &dn : dir_names)
            {
                if (!st_running.load()) break;
                auto segs = make_dir_segments(dn, speed);
                if (segs.empty()) { std::cerr << "[RETEST][WARN] bad direction " << dn << "\n"; continue; }
                runs.push_back(run_pass(segs[0], speed, rep, opt, tick, csv));
            }
    }

    std::cout << "\n[RETEST] stopping chassis...\n";
    disable_all(tick);
    csv.flush(); csv.close();

    write_summary(opt, runs);
    write_json(opt, runs);

    st_running.store(false);
    if (imu_thread.joinable()) imu_thread.join();
    rclcpp::shutdown();
    std::cout << "[RETEST] done. csv=" << opt.csv_path << " json=" << opt.json_path << "\n";
    return 0;
}

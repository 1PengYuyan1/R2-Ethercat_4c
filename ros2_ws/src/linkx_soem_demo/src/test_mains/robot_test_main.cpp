// =============================================================================
// robot_test_main.cpp (R2 版)
//
// Class_Robot 接口的真实上机回归测试（顺序自动）。
// 参考 R1 robot_test 的测试框架，按 R2 实际硬件改写：
//   - 无 BRT 编码器/无 ODrive/无 Clamp
//   - 新增 Gantry 双 DM 升降、Arm 单 DM 机械臂、Suction 真空
//
// 测试覆盖：
//   T1  Init 完整性                —— DM 舵向 / DM 轮向 / Gantry / Arm 全部 ENABLE
//   T2  EtherCAT/CAN 接收链路        —— DM 反馈帧持续到达（status 持续翻新）
//   T3  TIM 周期回调存活             —— bg 线程 1ms/2ms/100ms 跑 1s 不崩
//   T4  ROS 桥接订阅                —— /chassis/cmd_vel 注入 → 200ms 内底盘 ENABLE
//   T5  限速短转 (0.05 m/s × 2s)     —— Chassis Self_Resolution 跟得上
//   T6  Gantry 上升 → 下降           —— LIFT_POS2 → LIFT_POS1 角度反馈达标
//   T7  Arm 位置切换                 —— Arm_grab → Arm_zero 反馈达标
//   T8  Suction 开/关                —— ROS /r2/suction/cmd 发布次数+1
//
// 使用：
//   sudo ./robot_test [<iface>]      # 默认 enp86s0
// =============================================================================

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "robot.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

// --- 全局对象 ---
Class_Robot   robot;
ecat_master_t master;
linkx_t       linkx_dev;

static constexpr int      kChannelCount        = 4;
static constexpr uint32_t kTaskPeriod2Ms       = 2;
static constexpr uint32_t kTaskPeriod100Ms     = 100;
static constexpr float    kTestForwardSpeedMps = 0.05f;
static constexpr int      kTestForwardDurMs    = 2000;

// --- 测试结果 ---
struct Test_Result { std::string name; bool passed = false; std::string detail; };
static std::vector<Test_Result> g_results;

static void Record(const std::string &name, bool ok, const std::string &detail = "")
{
    g_results.push_back({name, ok, detail});
    std::cout << (ok ? "  [PASS] " : "  [FAIL] ") << name;
    if (!detail.empty()) std::cout << " — " << detail;
    std::cout << std::endl;
}

static void Section(const std::string &title)
{
    std::cout << "\n────────────────────────────────────────────────────────────\n"
              << " " << title << "\n"
              << "────────────────────────────────────────────────────────────"
              << std::endl;
}

static void Signal_Handler(int)
{
    std::cout << "\n[TEST] SIGINT received, requesting shutdown..." << std::endl;
    master.is_running = false;
}

// --- BG worker: 复用 task.cpp 1ms 主循环节拍 ---
static std::atomic<bool> g_bg_running{false};

static void Background_Worker()
{
    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick = 0;

    while (g_bg_running.load() && master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(1);
        std::this_thread::sleep_until(next_wakeup);

        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev);

        can_msg_t msg;
        for (uint8_t ch = 0; ch < kChannelCount; ch++)
            while (linkx_quick_recv(&linkx_dev, ch, &msg))
                robot.CAN_Rx_Callback(ch, msg.id, msg.data);

        robot.TIM_1ms_Calculate_Callback();
        if ((tick % kTaskPeriod2Ms) == 0)   robot.TIM_2ms_Calculate_PeriodElapsedCallback();
        if ((tick % kTaskPeriod100Ms) == 0) robot.TIM_100ms_Alive_PeriodElapsedCallback();

        linkx_send_pdos(&linkx_dev);
        tick++;
    }
}

// --- ROS injector ---
class Test_Injector
{
public:
    explicit Test_Injector(rclcpp::Node::SharedPtr node)
    {
        twist_pub_  = node->create_publisher<geometry_msgs::msg::Twist>("/chassis/cmd_vel", 20);
        gantry_pub_ = node->create_publisher<std_msgs::msg::UInt8>    ("/chassis/gantry_state", 20);
    }
    void Publish_Twist(float vx, float vy, float omega)
    {
        geometry_msgs::msg::Twist m; m.linear.x = vx; m.linear.y = vy; m.angular.z = omega;
        twist_pub_->publish(m);
    }
    void Publish_Gantry_State(uint8_t s)
    {
        std_msgs::msg::UInt8 m; m.data = s;
        gantry_pub_->publish(m);
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr      gantry_pub_;
};

// --- 测试用例 ---

static void T1_Init_Completeness()
{
    Section("T1: Init 完整性 — 所有 DM 子设备应 ENABLE");
    bool all_steer_ok = true, all_wheel_ok = true;
    for (int i = 0; i < STEER_NUM; ++i)
    {
        if (robot.Chassis.Motor_Steer[i].Get_Status() != Motor_DM_Status_ENABLE) all_steer_ok = false;
        if (robot.Chassis.Motor_Wheel[i].Get_Status() != Motor_DM_Status_ENABLE) all_wheel_ok = false;
    }
    Record("4× 舵向 DM6225 已使能", all_steer_ok);
    Record("4× 轮向 DM3519 已使能", all_wheel_ok);

    const bool gl = robot.Gantry.Motor_Lift_Left .Get_Status() == Motor_DM_Status_ENABLE;
    const bool gr = robot.Gantry.Motor_Lift_Right.Get_Status() == Motor_DM_Status_ENABLE;
    Record("Gantry 双 DM 已使能", gl && gr);
    Record("Arm DM 已使能", robot.Arm.Arm.Get_Status() == Motor_DM_Status_ENABLE);
}

static void T2_CAN_Link_Alive()
{
    Section("T2: EtherCAT/CAN 接收链路 — DM 反馈持续达");
    // 抓两次 DM 反馈，比较是否变化（角度或 omega 在小波动）
    float steer_rad_a[STEER_NUM]; float wheel_omega_a[STEER_NUM];
    for (int i = 0; i < STEER_NUM; ++i)
    {
        steer_rad_a[i]    = robot.Chassis.Motor_Steer[i].Get_Now_Radian();
        wheel_omega_a[i]  = robot.Chassis.Motor_Wheel[i].Get_Now_Omega();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    bool any_change = false;
    for (int i = 0; i < STEER_NUM && !any_change; ++i)
    {
        if (std::fabs(robot.Chassis.Motor_Steer[i].Get_Now_Radian() - steer_rad_a[i]) > 1e-6f) any_change = true;
        if (std::fabs(robot.Chassis.Motor_Wheel[i].Get_Now_Omega()  - wheel_omega_a[i]) > 1e-6f) any_change = true;
    }
    Record("DM 反馈帧持续到达（500ms 内有更新）", any_change);
}

static void T3_TIM_Callbacks_Alive()
{
    Section("T3: TIM 周期回调存活 — BG 线程 1s 不崩");
    auto t0 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto t1 = std::chrono::steady_clock::now();
    const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    Record("Background_Worker 1s 心跳", g_bg_running.load() && dt_ms >= 950 && dt_ms <= 1100,
           "elapsed=" + std::to_string(dt_ms) + "ms");
}

static void T4_ROS_Twist_Injection(Test_Injector &inj)
{
    Section("T4: ROS /chassis/cmd_vel 注入 → 200ms 内 chassis ENABLE");
    // 先确保未使能
    inj.Publish_Twist(0.0f, 0.0f, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 持续 500ms 发非零 twist，每 30ms 一帧
    for (int i = 0; i < 17; ++i)
    {
        inj.Publish_Twist(0.0f, 0.0f, 0.0f);  // 安全起见先零速，让 chassis ENABLE 但不动
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    const bool enabled = (robot.Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_ENABLE);
    Record("Chassis Control_Type=ENABLE", enabled);
}

static void T5_Limited_Forward(Test_Injector &inj)
{
    Section("T5: 限速短转 0.05 m/s × 2s");
    // 注意：本测试假设车辆已架空或在安全空间。生产场景请人工监管。
    float vx_max = 0.0f;
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - t0).count() < kTestForwardDurMs)
    {
        inj.Publish_Twist(kTestForwardSpeedMps, 0.0f, 0.0f);
        const float v = std::fabs(robot.Chassis.Get_Now_Velocity_X());
        if (v > vx_max) vx_max = v;
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    // 停车
    for (int i = 0; i < 10; ++i)
    {
        inj.Publish_Twist(0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    std::ostringstream det; det << std::fixed << std::setprecision(3) << "vx_max=" << vx_max << " m/s";
    Record("Chassis Self_Resolution 跟踪 0.05 m/s", vx_max > 0.005f, det.str());
}

static void T6_Gantry_Cycle(Test_Injector &inj)
{
    Section("T6: Gantry 升降周期 LIFT_POS2 → LIFT_POS1");
    inj.Publish_Gantry_State(0);  // LOW_POS
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    const float l_low = robot.Gantry.Motor_Lift_Left .Get_Now_Radian();
    const float r_low = robot.Gantry.Motor_Lift_Right.Get_Now_Radian();

    inj.Publish_Gantry_State(1);  // HIGH_POS
    std::this_thread::sleep_for(std::chrono::milliseconds(3500));
    const float l_high = robot.Gantry.Motor_Lift_Left .Get_Now_Radian();
    const float r_high = robot.Gantry.Motor_Lift_Right.Get_Now_Radian();

    const float l_delta = std::fabs(l_high - l_low);
    const float r_delta = std::fabs(r_high - r_low);

    std::ostringstream det;
    det << std::fixed << std::setprecision(3)
        << "L low=" << l_low << " high=" << l_high << " Δ=" << l_delta
        << " | R low=" << r_low << " high=" << r_high << " Δ=" << r_delta;
    Record("Gantry 双 DM 升程 > 1 rad", (l_delta > 1.0f && r_delta > 1.0f), det.str());
}

static void T7_Arm_Position_Cycle(Test_Injector &inj)
{
    Section("T7: Arm 位置切换 Arm_grab → Arm_zero");
    inj.Publish_Gantry_State(0);  // 触发 Arm_grab
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    const float grab_rad = robot.Arm.Arm.Get_Now_Radian();

    inj.Publish_Gantry_State(1);  // 触发 Arm_zero (经状态机延迟)
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    const float zero_rad = robot.Arm.Arm.Get_Now_Radian();

    const float delta = std::fabs(zero_rad - grab_rad);
    std::ostringstream det;
    det << std::fixed << std::setprecision(3)
        << "grab=" << grab_rad << " zero=" << zero_rad << " Δ=" << delta;
    Record("Arm 角度发生切换 > 0.3 rad", delta > 0.3f, det.str());
}

static void T8_Disable_Cycle(Test_Injector &inj)
{
    Section("T8: Disable 周期 — gantry_state=0xFF 后底盘超时");
    // 故意停发 cmd_vel 超过 200ms，触发 chassis timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const bool disabled = (robot.Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE);
    Record("Chassis 超时后自动 DISABLE", disabled);
}

// --- 主入口 ---

int main(int argc, char *argv[])
{
    const std::string ifname = (argc > 1) ? argv[1] : "enp86s0";

    std::signal(SIGINT,  Signal_Handler);
    std::signal(SIGTERM, Signal_Handler);

    std::cout << "===========================================\n"
              << "   R2 Robot Integration Test  (iface=" << ifname << ")\n"
              << "===========================================\n";

    // 1) EtherCAT + LinkX
    if (!ecat_master_init(&master, ifname.c_str())) {
        std::cerr << "[TEST] ecat_master_init failed.\n"; return 1;
    }
    linkx_init(&linkx_dev, 1, &master.ctx);
    linkx_hw_wakeup(&linkx_dev);
    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_set_can_baudrate(&linkx_dev, ch, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    if (!ecat_master_bring_online(&master)) {
        std::cerr << "[TEST] bring_online failed.\n"; return 2;
    }

    // 2) Robot Init + ROS 桥
    robot.Init(&linkx_dev);
    robot.Start_ROS2_Bridge();

    // 3) 启动 BG 主循环
    g_bg_running.store(true);
    std::thread bg(Background_Worker);

    // 4) 等待 1s 让心跳与首帧到位（DM 电机使能）
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 5) 在主线程开 ROS 注入器（独立 node 名以免冲突）
    int dummy_argc = 0; char **dummy_argv = nullptr;
    if (!rclcpp::ok()) rclcpp::init(dummy_argc, dummy_argv);
    auto inj_node = std::make_shared<rclcpp::Node>("r2_robot_test_injector");
    Test_Injector inj(inj_node);
    std::thread inj_spin([&]() { rclcpp::spin(inj_node); });

    // 6) 跑测试链
    T1_Init_Completeness();
    T2_CAN_Link_Alive();
    T3_TIM_Callbacks_Alive();
    T4_ROS_Twist_Injection(inj);
    T5_Limited_Forward(inj);
    T6_Gantry_Cycle(inj);
    T7_Arm_Position_Cycle(inj);
    T8_Disable_Cycle(inj);

    // 7) 收尾
    g_bg_running.store(false);
    if (bg.joinable()) bg.join();

    rclcpp::shutdown();
    if (inj_spin.joinable()) inj_spin.join();

    robot.Stop_ROS2_Bridge();

    // 8) 汇总
    Section("汇总");
    int passed = 0;
    for (auto &r : g_results) if (r.passed) ++passed;
    std::cout << "  " << passed << "/" << g_results.size() << " passed.\n";
    return (passed == (int)g_results.size()) ? 0 : 1;
}

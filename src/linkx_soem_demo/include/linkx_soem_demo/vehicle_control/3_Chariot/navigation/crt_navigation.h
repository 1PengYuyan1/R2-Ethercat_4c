//
// Created by Pzx on 26-3-24.
//

#ifndef CRT_NAVIGATION_H
#define CRT_NAVIGATION_H
#include "dvc_ops.h"
#include "alg_pid.h"
#include "math.h"
#include <cmath>

/**
 * @brief 自动导航状态机
 */
enum Enum_Nav_Status {
    Nav_Status_IDLE = 0,     // 待机模式（跟随遥控器）
    Nav_Status_NAVIGATING,   // 正在前往目标点
    Nav_Status_ARRIVED       // 已到达目标点
};

/**
 * @brief 航点结构体
 */
struct Struct_Waypoint {
    float x;           // 目标点 X 坐标 (mm)
    float y;           // 目标点 Y 坐标 (mm)
    float yaw;         // 目标点航向角 (deg)
    float max_speed;   // 到达该点允许的最大线速度 (m/s)
    float pass_radius; // 切换判定半径 (mm)，距离小于此值即认为到达该点并切换下一个点
};

/**
 * @brief 导航与路径规划控制类
 */
class Class_Navigation {
public:
    // 位置环PID
    Class_PID PID_X;
    Class_PID PID_Y;
    Class_PID PID_Yaw;

    void Init();

    void Set_Route(const Struct_Waypoint* route, uint8_t count);
    // 设定目标坐标 (单位: mm, mm, deg)
    void Set_Target_Position(float target_x, float target_y, float target_yaw);
    // 启动与停止导航
    void Start_Navigation(float current_x, float current_y);
    void Stop_Navigation();
    // 核心结算函数：传入当前OPS数据，输出底盘目标速度
    void Calculate(const Struct_OPS_Rx_Data& current_ops);

    // 获取结算后输出给底盘的速度 (Robot坐标系下)
    inline float Get_Target_Vx() const { return Target_Vx; }
    inline float Get_Target_Vy() const { return Target_Vy; }
    inline float Get_Target_Omega() const { return Target_Omega; }
    inline Enum_Nav_Status Get_Status() const { return Nav_Status; }

private:
    bool Is_First_Step = true;               // 用于导航开始时初始化虚拟点
    float Virtual_Target_X, Virtual_Target_Y; // 虚拟目标点（影子点）位置

    // 加速度限制相关
    float Last_Robot_Vx = 0.0f;
    float Last_Robot_Vy = 0.0f;
    const float dt = 0.001f;                 // 2ms 周期
    const float MAX_ACCEL = 1.5f;            // 加速度限制 (m/s^2)，根据底盘动力调

    // 路径数据
    const Struct_Waypoint* Current_Route = nullptr;
    uint8_t Route_Length = 0;
    uint8_t Current_Waypoint_Index = 0;

    // 输出给底盘的期望速度
    float Target_Vx = 0.0f;
    float Target_Vy = 0.0f;
    float Target_Omega = 0.0f;

    const float GLOBAL_MAX_V = 0.3;     // 全局最大线速度限制 (m/s)
    const float GLOBAL_MAX_OMEGA = 0.3; // 全局最大角速度限制 (rad/s)

    Enum_Nav_Status Nav_Status = Nav_Status_IDLE;

    // 最终目标点判定容差
    float Final_Distance_Threshold = 10.0f; // mm (3厘米)
    float Final_Yaw_Threshold = 2.0f;       // deg

    // float Last_Robot_Vx = 0.0f;
    // float Last_Robot_Vy = 0.0f;
    // const float MAX_ACCEL = 0.001f; // 每周期(2ms)允许的最大速度变化量(m/s)
    // // 换算：0.001 / 0.002s = 0.5m/s^2 (根据你的底盘动力性能调整)



};
#endif //CRT_NAVIGATION_H

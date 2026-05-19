#include "crt_navigation.h"

// 速度0.05
// PID_X.Init(0.05f, 0.05f, 0.01f, 0.0f, 0.0f, 3.0f, 0.002f); // 假设2ms控制周期
// PID_Y.Init(0.05f, 0.05f, 0.01f, 0.0f, 0.0f, 3.0f, 0.002f);
// PID_Yaw.Init(0.005f, 0.0001f, 0.00f, 0.0f, 5.0f, 1.0f, 0.002f);

void Class_Navigation::Init() {
    // 初始化位置环 PID 参数
    // X,Y 坐标单位是 mm，输出是 m/s，Kp设小一点
    PID_X.Init(0.0005f, 0.000005f, 0.00005f, 0.0f, 0.0f, 3.0f, 0.002f); // 假设2ms控制周期
    PID_Y.Init(0.0005f, 0.000005f, 0.00005f, 0.0f, 0.0f, 3.0f, 0.002f);
    PID_Yaw.Init(0.001f, 0.001f, 0.0001f, 0.0f, 5.0f, 1.0f, 0.002f);

    Nav_Status = Nav_Status_IDLE;
}

void Class_Navigation::Set_Route(const Struct_Waypoint* route, uint8_t count) {
    Current_Route = route;
    Route_Length = count;
    Current_Waypoint_Index = 0;
}

void Class_Navigation::Start_Navigation(float current_x, float current_y) {    if (Current_Route != nullptr && Route_Length > 0) {
        Nav_Status = Nav_Status_NAVIGATING;
        Is_First_Step = true; // 标记需要初始化虚拟点
        PID_X.Set_Integral_Error(0.0f);
        PID_Y.Set_Integral_Error(0.0f);
        PID_Yaw.Set_Integral_Error(0.0f);
    }
}

void Class_Navigation::Stop_Navigation() {
    Nav_Status = Nav_Status_IDLE;
    Target_Vx = 0.0f;
    Target_Vy = 0.0f;
    Target_Omega = 0.0f;
}

void Class_Navigation::Calculate(const Struct_OPS_Rx_Data& current_ops) {
    if (Nav_Status != Nav_Status_NAVIGATING || Current_Route == nullptr) return;

    Struct_Waypoint current_wp = Current_Route[Current_Waypoint_Index];

    if (Is_First_Step) {
        Virtual_Target_X = current_ops.Pos_X;
        Virtual_Target_Y = current_ops.Pos_Y;
        Is_First_Step = false;
    }

    float vec_x = current_wp.x - Virtual_Target_X;
    float vec_y = current_wp.y - Virtual_Target_Y;
    float dist_to_wp = sqrtf(vec_x * vec_x + vec_y * vec_y);

    if (dist_to_wp > 1.0f) {
        float step_size = current_wp.max_speed * dt * 1000.0f;
        if (step_size > dist_to_wp) step_size = dist_to_wp;

        Virtual_Target_X += (vec_x / dist_to_wp) * step_size;
        Virtual_Target_Y += (vec_y / dist_to_wp) * step_size;
    }

    PID_X.Set_Target(Virtual_Target_X);
    PID_X.Set_Now(current_ops.Pos_X);
    PID_X.TIM_Calculate_PeriodElapsedCallback();

    PID_Y.Set_Target(Virtual_Target_Y);
    PID_Y.Set_Now(current_ops.Pos_Y);
    PID_Y.TIM_Calculate_PeriodElapsedCallback();

    float angle_error = current_wp.yaw - current_ops.Yaw;
    if (angle_error > 180.0f) angle_error -= 360.0f;
    else if (angle_error < -180.0f) angle_error += 360.0f;
    PID_Yaw.Set_Target(angle_error);
    PID_Yaw.Set_Now(0);
    PID_Yaw.TIM_Calculate_PeriodElapsedCallback();

    float world_vx = PID_Y.Get_Out();
    float world_vy = PID_X.Get_Out();
    float out_omega = PID_Yaw.Get_Out();

    float theta = current_ops.Yaw * DEG_TO_RAD;
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    float robot_vx = world_vx * cos_theta + world_vy * sin_theta;
    float robot_vy = world_vx * sin_theta - world_vy * cos_theta;

    float max_dv = MAX_ACCEL * dt;
    float dv_x = robot_vx - Last_Robot_Vx;
    if (dv_x > max_dv) robot_vx = Last_Robot_Vx + max_dv;
    else if (dv_x < -max_dv) robot_vx = Last_Robot_Vx - max_dv;

    float dv_y = robot_vy - Last_Robot_Vy;
    if (dv_y > max_dv) robot_vy = Last_Robot_Vy + max_dv;
    else if (dv_y < -max_dv) robot_vy = Last_Robot_Vy - max_dv;

    Last_Robot_Vx = robot_vx;
    Last_Robot_Vy = robot_vy;

    Target_Vx = robot_vx;
    Target_Vy = robot_vy;
    Target_Omega = out_omega;

    float real_dist = sqrtf(powf(current_wp.x - current_ops.Pos_X, 2) + powf(current_wp.y - current_ops.Pos_Y, 2));
    if (real_dist < current_wp.pass_radius) {
        if (Current_Waypoint_Index < Route_Length - 1) {
            Current_Waypoint_Index++;
            // 切换时不重置虚拟点，让它自然地流向下一个目标
        } else if (real_dist < Final_Distance_Threshold) {
            Nav_Status = Nav_Status_ARRIVED;
        }
    }
}

// void Class_Navigation::Calculate(const Struct_OPS_Rx_Data& current_ops) {
//     // 1. 状态检查
//     if (Nav_Status != Nav_Status_NAVIGATING || Current_Route == nullptr) {
//         Target_Vx = 0.0f; Target_Vy = 0.0f; Target_Omega = 0.0f;
//         return;
//     }
//
//     // 2. 获取当前航点与终点判定
//     Struct_Waypoint current_wp = Current_Route[Current_Waypoint_Index];
//     bool is_final_waypoint = (Current_Waypoint_Index == Route_Length - 1);
//
//     // 3. 误差计算
//     float dx = current_wp.x - current_ops.Pos_X;
//     float dy = current_wp.y - current_ops.Pos_Y;
//     float distance = sqrtf(dx * dx + dy * dy);
//     float dyaw = Math_Modulus_Normalization(current_wp.yaw - current_ops.Yaw, 360.0f);
//
//     // 4. 航点切换逻辑
//     if (is_final_waypoint) {
//         if (distance < Final_Distance_Threshold && fabsf(dyaw) < Final_Yaw_Threshold) {
//             Nav_Status = Nav_Status_ARRIVED;
//             Target_Vx = 0.0f; Target_Vy = 0.0f; Target_Omega = 0.0f;
//             return;
//         }
//     } else {
//         if (distance < current_wp.pass_radius) {
//             Current_Waypoint_Index++;
//             PID_X.Set_Integral_Error(0.0f);
//             PID_Y.Set_Integral_Error(0.0f);
//             return;
//         }
//     }
//
//     // 5. PID 解算 (世界坐标系)
//     PID_X.Set_Target(current_wp.x);
//     PID_X.Set_Now(current_ops.Pos_X);
//     PID_X.TIM_Calculate_PeriodElapsedCallback();
//
//     PID_Y.Set_Target(current_wp.y);
//     PID_Y.Set_Now(current_ops.Pos_Y);
//     PID_Y.TIM_Calculate_PeriodElapsedCallback();
//
//     PID_Yaw.Set_Target(dyaw);
//     PID_Yaw.Set_Now(0.0f);
//     PID_Yaw.TIM_Calculate_PeriodElapsedCallback();
//
//     float world_vx = PID_Y.Get_Out();
//     float world_vy = PID_X.Get_Out();
//     float out_omega = PID_Yaw.Get_Out();
//
//     // 6. 第一层限幅：基于航点配置的 max_speed (等比例缩放，不改变运动方向)
//     float current_world_speed = sqrtf(world_vx * world_vx + world_vy * world_vy);
//     float dynamic_limit = current_wp.max_speed;
//
//     if (is_final_waypoint && distance < 300.0f) {
//         dynamic_limit = (distance / 300.0f) * current_wp.max_speed;
//         if (dynamic_limit < 0.15f) dynamic_limit = 0.15f;
//     }
//
//     if (current_world_speed > dynamic_limit && current_world_speed > 0.001f) {
//         float scale = dynamic_limit / current_world_speed;
//         world_vx *= scale;
//         world_vy *= scale;
//     }
//
//     // 7. 坐标转换：世界坐标系 -> 机器人本体坐标系
//     float theta = current_ops.Yaw * DEG_TO_RAD;
//     float cos_theta = cosf(theta);
//     float sin_theta = sinf(theta);
//
//     float robot_vx = world_vx * cos_theta + world_vy * sin_theta;
//     float robot_vy = world_vx * sin_theta - world_vy * cos_theta;
//
//     // 8. 第二层限幅：最终全局物理安全限幅 (新增)
//     // --- 线速度全局硬限幅 (防止极端情况导致的速度溢出) ---
//     float final_robot_speed = sqrtf(robot_vx * robot_vx + robot_vy * robot_vy);
//     if (final_robot_speed > GLOBAL_MAX_V) {
//         float scale = GLOBAL_MAX_V / final_robot_speed;
//         robot_vx *= scale;
//         robot_vy *= scale;
//     }
//
//     // --- 角速度硬限幅 (防止转弯过猛导致侧翻或舵轮堵转) ---
//     if (out_omega > GLOBAL_MAX_OMEGA) out_omega = GLOBAL_MAX_OMEGA;
//     else if (out_omega < -GLOBAL_MAX_OMEGA) out_omega = -GLOBAL_MAX_OMEGA;
//
//     // 9. 最终输出赋值
//     Target_Vx = Target_Vx * 0.5f + robot_vx * 0.5f;
//     Target_Vy = Target_Vy * 0.5f + robot_vy * 0.5f;
//     Target_Omega = out_omega;
//
// }
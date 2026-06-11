#ifndef ROBOT_TYPES_HPP
#define ROBOT_TYPES_HPP

// 定义底盘控制指令
struct ChassisCommand {
    float vx;       // 前后速度 m/s
    float vy;       // 左右速度 m/s
    float omega;    // 旋转速度 rad/s
    float right_y;  // 右摇杆前后，差速抬升底盘使用
};


#endif // ROBOT_TYPES_HPP

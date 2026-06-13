#ifndef JOYSTICK_MAPPER_HPP
#define JOYSTICK_MAPPER_HPP

#include <vector>
#include "linkx_soem_demo/remote/device/dvc_logF710.h"
#include "linkx_soem_demo/remote/ros2/robot_types.hpp"

class JoystickMapper {
public:
    // 构造函数声明
    JoystickMapper();

    // 析构函数（养成好习惯，虽然这里默认为空）
    ~JoystickMapper();

    // 设置参数
    void setParams(float max_v, float max_w, float deadzone);

    // 计算底盘指令
    ChassisCommand processChassis(const std::vector<float>& axes, const std::vector<int>& buttons);

    // 计算按键状态（键值码/组合键码）
    uint16_t processButtons(const std::vector<float>& axes, const std::vector<int>& buttons);

    private:
    Class_LogF710 logf710_;
};

#endif 

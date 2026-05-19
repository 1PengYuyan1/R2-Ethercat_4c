#include "linkx_soem_demo/remote/ros2/joystick_mapper.hpp"

// 构造函数实现：初始化默认参数
JoystickMapper::JoystickMapper() {
    logf710_.Set_Control_Params(1.0f, 1.0f, 0.05f);
}

JoystickMapper::~JoystickMapper() {
}

void JoystickMapper::setParams(float max_v, float max_w, float deadzone) {
    logf710_.Set_Control_Params(max_v, max_w, deadzone);
}

// 底盘逻辑实现
ChassisCommand JoystickMapper::processChassis(const std::vector<float>& axes, const std::vector<int>& buttons) {
    (void)buttons;
    const Struct_LogF710_Command remote_cmd = logf710_.Resolve_Chassis(axes);
    ChassisCommand cmd = {remote_cmd.vx, remote_cmd.vy, remote_cmd.omega};
    return cmd;
}

// 按键逻辑实现
uint16_t JoystickMapper::processButtons(const std::vector<float>& axes, const std::vector<int>& buttons) {
    return logf710_.Resolve_Button_Code(axes, buttons);
}

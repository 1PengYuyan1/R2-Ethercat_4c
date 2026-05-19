#include "dvc_buzzer.h"
#include <rclcpp/rclcpp.hpp>

extern "C" {

void Buzzer_Init(void)
{
    RCLCPP_INFO(rclcpp::get_logger("buzzer"), "init (stub)");
}

void Buzzer_Stop(void)
{
    RCLCPP_INFO(rclcpp::get_logger("buzzer"), "stop (stub)");
}

void Buzzer_Set_Mode(Buzzer_Mode_t mode)
{
    const char *names[] = { "OFF", "STARTUP", "DEVICE_LOST", "EMERGENCY" };
    const int  idx = static_cast<int>(mode);
    const char *name = (idx >= 0 && idx < 4) ? names[idx] : "?";
    RCLCPP_INFO(rclcpp::get_logger("buzzer"), "mode=%s (stub)", name);
}

}

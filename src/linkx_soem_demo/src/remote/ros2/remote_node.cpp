#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <cstdint>

#include "linkx_soem_demo/remote/ros2/robot_types.hpp"
#include "linkx_soem_demo/remote/ros2/joystick_mapper.hpp"
#include "linkx_soem_demo/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis_omni.h"

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("remote_node_cpp")
    {
        mapper_ = std::make_unique<JoystickMapper>();

        mapper_->setParams(MAX_OMNI_CHASSIS_SPEED, MAX_OMNI_CHASSIS_OMEGA, 0.05f);

        this->declare_parameter("cmd_topic", "/chassis/remote_cmd_vel");
        this->declare_parameter("buttons_topic", "/robot_buttons");

        const auto cmd_topic = this->get_parameter("cmd_topic").as_string();
        const auto buttons_topic = this->get_parameter("buttons_topic").as_string();

        sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            rclcpp::SensorDataQoS(),
            std::bind(&TeleopNode::joyCallback, this, std::placeholders::_1));

        pub_chassis_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
        pub_buttons_ = this->create_publisher<std_msgs::msg::UInt16>(buttons_topic, 10);

    }

private:
    std::unique_ptr<JoystickMapper> mapper_;
    uint16_t last_logged_button_ = 0xFFFFU;
    uint64_t last_logged_button_mask_ = ~0ULL;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_chassis_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_buttons_;

    // 摇杆回调函数
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // === 步骤 1: 处理底盘速度 ===
        ChassisCommand chassis_data = mapper_->processChassis(msg->axes, msg->buttons);
        publishChassisMsg(chassis_data);

        // 2. 解算功能机构按钮 (新增!)
        uint16_t button_data = mapper_->processButtons(msg->axes, msg->buttons);
        publishButtonMsg(button_data);

        uint64_t button_mask = 0ULL;
        for (size_t i = 0; i < msg->buttons.size() && i < 64U; ++i)
        {
            if (msg->buttons[i] != 0)
                button_mask |= (1ULL << i);
        }

        if (button_data != last_logged_button_ || button_mask != last_logged_button_mask_)
        {
            last_logged_button_ = button_data;
            last_logged_button_mask_ = button_mask;
            RCLCPP_WARN(this->get_logger(),
                        "F710 button decoded: code=0x%04X raw_mask=0x%016llX axes=%zu buttons=%zu",
                        static_cast<unsigned>(button_data),
                        static_cast<unsigned long long>(button_mask),
                        msg->axes.size(),
                        msg->buttons.size());
        }
    }

    void publishChassisMsg(const ChassisCommand &data)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = data.vx;
        msg.linear.y = data.vy;
        msg.angular.x = data.right_y;
        msg.angular.z = data.omega;
        pub_chassis_->publish(msg);
    }

    void publishButtonMsg(const uint16_t &data)
    {
        auto msg = std_msgs::msg::UInt16();
        msg.data = data;
        pub_buttons_->publish(msg);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}

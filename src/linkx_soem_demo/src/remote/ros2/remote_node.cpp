#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <cmath>

#include "linkx_soem_demo/remote/ros2/robot_types.hpp"
#include "linkx_soem_demo/remote/ros2/joystick_mapper.hpp"

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("remote_node_cpp")
    {
        mapper_ = std::make_unique<JoystickMapper>();

        this->declare_parameter("max_speed", 2.0);
        float max_speed = this->get_parameter("max_speed").as_double();
        
        mapper_->setParams(max_speed, 1.5f, 0.05f);

        sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&TeleopNode::joyCallback, this, std::placeholders::_1));

                    // 创建按键发布者

        pub_chassis_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 创建按键发布者
        pub_buttons_ = this->create_publisher<std_msgs::msg::UInt16>("/robot_buttons", 10);

        RCLCPP_INFO(this->get_logger(), "分离架构遥控节点已启动！");
    }

private:
    static constexpr bool kEnablePublishDebugLog = false;
    std::unique_ptr<JoystickMapper> mapper_;

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
        // 3. 调试输出：仅当有明显输入时打印
        if (kEnablePublishDebugLog &&
            (std::abs(chassis_data.vx) > 0.01 || std::abs(chassis_data.vy) > 0.01 ||
             std::abs(chassis_data.omega) > 0.01 || button_data != 0))
        {
            RCLCPP_INFO(this->get_logger(),
                        "发布 -> 速度: [vx=%.2f, vy=%.2f , omega=%.2f] | 按键掩码: 0x%04X",
                        chassis_data.vx, chassis_data.vy, chassis_data.omega, button_data);
        }
    }

    void publishChassisMsg(const ChassisCommand &data)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = data.vx;
        msg.linear.y = data.vy;
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

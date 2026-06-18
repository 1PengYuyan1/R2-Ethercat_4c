#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int16.hpp>

class TopicRelayNode : public rclcpp::Node {
public:
    TopicRelayNode() : Node("topic_relay_cpp") {
        this->declare_parameter("input_cmd_topic", "/cmd_vel");
        this->declare_parameter("input_buttons_topic", "/robot_buttons");
        this->declare_parameter("output_cmd_topic", "/chassis/cmd_vel");
        this->declare_parameter("output_buttons_topic", "/chassis/buttons");

        const auto input_cmd_topic = this->get_parameter("input_cmd_topic").as_string();
        const auto input_buttons_topic = this->get_parameter("input_buttons_topic").as_string();
        const auto output_cmd_topic = this->get_parameter("output_cmd_topic").as_string();
        const auto output_buttons_topic = this->get_parameter("output_buttons_topic").as_string();

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic, 10);
        pub_buttons_ = this->create_publisher<std_msgs::msg::UInt16>(output_buttons_topic, 10);

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            input_cmd_topic, 10, std::bind(&TopicRelayNode::cmdCallback, this, std::placeholders::_1));

        sub_buttons_ = this->create_subscription<std_msgs::msg::UInt16>(
            input_buttons_topic, 10, std::bind(&TopicRelayNode::buttonCallback, this, std::placeholders::_1));

    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_buttons_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_buttons_;

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        pub_cmd_->publish(*msg);
    }

    void buttonCallback(const std_msgs::msg::UInt16::SharedPtr msg) {
        pub_buttons_->publish(*msg);
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicRelayNode>());
    rclcpp::shutdown();
    return 0;
}

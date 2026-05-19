#include "dvc_suction.h"

rclcpp::Node *dvc_suction::node_ = nullptr;
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr dvc_suction::pub_;

void dvc_suction::Bind_Node(rclcpp::Node *node)
{
    node_ = node;
    if (node_ != nullptr)
    {
        pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/r2/suction/cmd", 10);
    }
}

void dvc_suction::Publish_Cmd(uint8_t code, const char *name)
{
    if (!pub_)
    {
        return;
    }
    std_msgs::msg::UInt8 msg;
    msg.data = code;
    pub_->publish(msg);
    if (node_)
    {
        RCLCPP_DEBUG(node_->get_logger(), "suction %s (code=%u)", name, code);
    }
}

void dvc_suction::stop2(void)     { Publish_Cmd(0, "stop2"); }
void dvc_suction::start2(void)    { Publish_Cmd(1, "start2"); }
void dvc_suction::stop1(void)     { Publish_Cmd(2, "stop1"); }
void dvc_suction::start1(void)    { Publish_Cmd(3, "start1"); }
void dvc_suction::stopa(void)     { Publish_Cmd(4, "stopa"); }
void dvc_suction::starta(void)    { Publish_Cmd(5, "starta"); }
void dvc_suction::stop_give(void) { Publish_Cmd(6, "stop_give"); }
void dvc_suction::give(void)      { Publish_Cmd(7, "give"); }

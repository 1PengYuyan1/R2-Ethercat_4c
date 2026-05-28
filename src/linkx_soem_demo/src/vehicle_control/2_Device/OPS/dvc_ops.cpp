#include "dvc_ops.h"

void Class_OPS::init(rclcpp::Node *node)
{
    node_ = node;
    if (node_ == nullptr)
    {
        return;
    }

    sub_pose_ = node_->create_subscription<geometry_msgs::msg::Pose2D>(
        "/r2/ops/pose2d", 20,
        [this](const geometry_msgs::msg::Pose2D::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            data_.Pos_X = static_cast<float>(msg->x);
            data_.Pos_Y = static_cast<float>(msg->y);
            data_.Yaw   = static_cast<float>(msg->theta);
            last_update_ns_.store(now_ns());
        });

    sub_extra_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/r2/ops/extra", 20,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            if (msg->data.size() < 3) return;
            std::lock_guard<std::mutex> lock(mtx_);
            data_.Pitch   = msg->data[0];
            data_.Roll    = msg->data[1];
            data_.Omega_Z = msg->data[2];
            last_update_ns_.store(now_ns());
        });

    RCLCPP_INFO(node_->get_logger(), "OPS subscriber bound: /r2/ops/pose2d + /r2/ops/extra");
}

Struct_OPS_Rx_Data Class_OPS::getData()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
}

bool Class_OPS::isConnected()
{
    const int64_t last = last_update_ns_.load();
    if (last == 0) return false;
    return (now_ns() - last) < 500'000'000LL;  // 500 ms
}

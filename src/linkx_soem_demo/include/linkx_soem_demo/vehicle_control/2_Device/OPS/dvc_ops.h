#ifndef DVC_OPS_H
#define DVC_OPS_H

// Linux/ROS 上位机替身：原 STM32 UART OPS（光学定位）传感器改为订阅 ROS 话题。
// 外部驱动节点把 OPS 串口数据 (Yaw/Pitch/Roll/X/Y/Omega_Z) 解析后发布到：
//   /r2/ops  (geometry_msgs/Pose2D + std_msgs/Float32MultiArray)
//
// 默认订阅:
//   /r2/ops/pose2d    Pose2D    x=Pos_X(mm), y=Pos_Y(mm), theta=Yaw(rad)
//   /r2/ops/extra     Float32MultiArray[3]  pitch / roll / omega_z (rad, rad/s)
//
// 上层 (Navigation) 通过 getData() 拿到 Struct_OPS_Rx_Data 副本。

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

struct Struct_OPS_Rx_Data
{
    float Yaw;
    float Pitch;
    float Roll;
    float Pos_X;
    float Pos_Y;
    float Omega_Z;
};

class Class_OPS
{
public:
    void init(rclcpp::Node *node);

    Struct_OPS_Rx_Data getData();
    Struct_OPS_Rx_Data *getData_Ptr() { return &data_; }

    bool isConnected();

private:
    rclcpp::Node *node_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr            sub_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr      sub_extra_;

    std::mutex                    mtx_;
    Struct_OPS_Rx_Data            data_{};
    std::atomic<int64_t>          last_update_ns_{0};

    static int64_t now_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }
};

#endif

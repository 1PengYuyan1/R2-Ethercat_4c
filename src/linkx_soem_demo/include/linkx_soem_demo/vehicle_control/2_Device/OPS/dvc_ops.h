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
// 上层通过 getData() 拿到的 Struct_OPS_Rx_Data 统一为 deg / deg/s。

#include <atomic>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

struct Struct_OPS_Rx_Data
{
    float Yaw;      // deg
    float Pitch;    // deg
    float Roll;     // deg
    float Pos_X;
    float Pos_Y;
    float Omega_Z;  // deg/s
};

struct Struct_OPS_Raw_CAN_Frame
{
    uint8_t dlen = 0;
    std::array<uint8_t, 8> data{};
};

struct Struct_OPS_Decoded_CAN_Frame
{
    uint32_t count = 0;
    uint32_t invalid_count = 0;
    bool valid = false;
    std::array<uint8_t, 28> data{};
};

class Class_OPS
{
public:
    void init(rclcpp::Node *node);
    void shutdown();

    void CAN_RxCpltCallback(const uint8_t *rx_data, uint8_t dlen);

    Struct_OPS_Rx_Data getData();
    Struct_OPS_Rx_Data *getData_Ptr() { return &data_; }
    Struct_OPS_Raw_CAN_Frame getRawCANFrame();
    Struct_OPS_Decoded_CAN_Frame getLastDecodedFrame();

    bool hasRecentFrame();
    bool isConnected();

private:
    rclcpp::Node *node_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr            sub_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr      sub_extra_;

    std::mutex                    mtx_;
    Struct_OPS_Rx_Data            data_{};
    Struct_OPS_Raw_CAN_Frame      raw_can_{};
    Struct_OPS_Decoded_CAN_Frame  last_decoded_frame_{};
    std::array<uint8_t, 64>       rx_buffer_{};
    size_t                        rx_buffer_len_ = 0;
    std::atomic<int64_t>          last_frame_ns_{0};
    std::atomic<int64_t>          last_update_ns_{0};

    void Try_Parse_Frame_();
    void Drop_Front_(size_t n);
    static float readFloatLE(const uint8_t *bytes);
    static bool isFiniteFrame(const Struct_OPS_Rx_Data &data);

    static int64_t now_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }
};

#endif

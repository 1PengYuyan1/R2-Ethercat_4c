#include "dvc_ops.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
constexpr float kRadToDeg = 57.29577951308232f;
constexpr uint8_t kOpsHeader0 = 0x0D;
constexpr uint8_t kOpsHeader1 = 0x0A;
constexpr uint8_t kOpsTail0 = 0x0A;
constexpr uint8_t kOpsTail1 = 0x0D;
constexpr size_t kOpsFrameLen = 28;      // 0D0A + yaw/pitch/roll/x/y/omega_z float + 0A0D
constexpr size_t kOpsPayloadOffset = 2;
}

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
            data_.Yaw   = static_cast<float>(msg->theta) * kRadToDeg;
            const int64_t now = now_ns();
            last_frame_ns_.store(now);
            if (isFiniteFrame(data_))
                last_update_ns_.store(now);
        });

    sub_extra_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/r2/ops/extra", 20,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            if (msg->data.size() < 3) return;
            std::lock_guard<std::mutex> lock(mtx_);
            data_.Pitch   = msg->data[0] * kRadToDeg;
            data_.Roll    = msg->data[1] * kRadToDeg;
            data_.Omega_Z = msg->data[2] * kRadToDeg;
            const int64_t now = now_ns();
            last_frame_ns_.store(now);
            if (isFiniteFrame(data_))
                last_update_ns_.store(now);
        });

}

void Class_OPS::shutdown()
{
    sub_pose_.reset();
    sub_extra_.reset();
    node_ = nullptr;
}

void Class_OPS::CAN_RxCpltCallback(const uint8_t *rx_data, uint8_t dlen)
{
    if (rx_data == nullptr || dlen == 0)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    raw_can_.dlen = std::min<uint8_t>(dlen, static_cast<uint8_t>(raw_can_.data.size()));
    std::fill(raw_can_.data.begin(), raw_can_.data.end(), 0U);
    std::copy(rx_data, rx_data + raw_can_.dlen, raw_can_.data.begin());

    if (rx_buffer_len_ + raw_can_.dlen > rx_buffer_.size())
    {
        const size_t need_drop = (rx_buffer_len_ + raw_can_.dlen) - rx_buffer_.size();
        Drop_Front_(need_drop);
    }

    std::copy(raw_can_.data.begin(),
              raw_can_.data.begin() + raw_can_.dlen,
              rx_buffer_.begin() + rx_buffer_len_);
    rx_buffer_len_ += raw_can_.dlen;

    Try_Parse_Frame_();
}

Struct_OPS_Rx_Data Class_OPS::getData()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
}

Struct_OPS_Raw_CAN_Frame Class_OPS::getRawCANFrame()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return raw_can_;
}

Struct_OPS_Decoded_CAN_Frame Class_OPS::getLastDecodedFrame()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return last_decoded_frame_;
}

bool Class_OPS::isConnected()
{
    const int64_t last = last_update_ns_.load();
    if (last == 0) return false;
    return (now_ns() - last) < 500'000'000LL;  // 500 ms
}

bool Class_OPS::hasRecentFrame()
{
    const int64_t last = last_frame_ns_.load();
    if (last == 0) return false;
    return (now_ns() - last) < 500'000'000LL;  // 500 ms
}

void Class_OPS::Try_Parse_Frame_()
{
    while (true)
    {
        size_t header_idx = SIZE_MAX;
        if (rx_buffer_len_ >= 2)
        {
            for (size_t i = 0; i + 1 < rx_buffer_len_; ++i)
            {
                if (rx_buffer_[i] == kOpsHeader0 && rx_buffer_[i + 1] == kOpsHeader1)
                {
                    header_idx = i;
                    break;
                }
            }
        }

        if (header_idx == SIZE_MAX)
        {
            if (rx_buffer_len_ >= 1)
            {
                rx_buffer_[0] = rx_buffer_[rx_buffer_len_ - 1];
                rx_buffer_len_ = 1;
            }
            return;
        }

        if (header_idx > 0)
        {
            Drop_Front_(header_idx);
        }

        if (rx_buffer_len_ < kOpsFrameLen)
        {
            return;
        }

        if (rx_buffer_[kOpsFrameLen - 2] != kOpsTail0 ||
            rx_buffer_[kOpsFrameLen - 1] != kOpsTail1)
        {
            Drop_Front_(2);
            continue;
        }

        const uint8_t *payload = rx_buffer_.data() + kOpsPayloadOffset;
        Struct_OPS_Rx_Data decoded_data{};
        decoded_data.Yaw = readFloatLE(payload + 0);
        decoded_data.Pitch = readFloatLE(payload + 4);
        decoded_data.Roll = readFloatLE(payload + 8);
        decoded_data.Pos_X = readFloatLE(payload + 12);
        decoded_data.Pos_Y = readFloatLE(payload + 16);
        decoded_data.Omega_Z = readFloatLE(payload + 20);

        const int64_t now = now_ns();
        const bool valid = isFiniteFrame(decoded_data);

        last_decoded_frame_.count += 1;
        last_decoded_frame_.valid = valid;
        if (!valid)
            last_decoded_frame_.invalid_count += 1;
        std::copy(rx_buffer_.begin(),
                  rx_buffer_.begin() + kOpsFrameLen,
                  last_decoded_frame_.data.begin());
        last_frame_ns_.store(now);
        if (valid)
        {
            data_ = decoded_data;
            last_update_ns_.store(now);
        }

        Drop_Front_(kOpsFrameLen);
    }
}

void Class_OPS::Drop_Front_(size_t n)
{
    if (n == 0) return;
    if (n >= rx_buffer_len_)
    {
        rx_buffer_len_ = 0;
        return;
    }

    const size_t remain = rx_buffer_len_ - n;
    std::move(rx_buffer_.begin() + n,
              rx_buffer_.begin() + rx_buffer_len_,
              rx_buffer_.begin());
    rx_buffer_len_ = remain;
}

float Class_OPS::readFloatLE(const uint8_t *bytes)
{
    const uint32_t word = static_cast<uint32_t>(bytes[0]) |
                          (static_cast<uint32_t>(bytes[1]) << 8) |
                          (static_cast<uint32_t>(bytes[2]) << 16) |
                          (static_cast<uint32_t>(bytes[3]) << 24);
    float value = 0.0f;
    std::memcpy(&value, &word, sizeof(value));
    return value;
}

bool Class_OPS::isFiniteFrame(const Struct_OPS_Rx_Data &data)
{
    return std::isfinite(data.Yaw) &&
           std::isfinite(data.Pitch) &&
           std::isfinite(data.Roll) &&
           std::isfinite(data.Pos_X) &&
           std::isfinite(data.Pos_Y) &&
           std::isfinite(data.Omega_Z);
}

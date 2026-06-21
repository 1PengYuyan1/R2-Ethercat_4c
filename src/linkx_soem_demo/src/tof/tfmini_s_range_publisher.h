#ifndef LINKX_SOEM_DEMO_TFMINI_S_RANGE_PUBLISHER_H
#define LINKX_SOEM_DEMO_TFMINI_S_RANGE_PUBLISHER_H

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "linkx.h"

class TfminiSRangePublisher
{
public:
    void Start(const rclcpp::Node::SharedPtr &node);
    void Stop();
    void ConfigureReset(linkx_t *slave1_linkx,
                        linkx_t *slave2_linkx,
                        bool slave2_enabled);

    bool HandleCanFrame(uint32_t slave_id,
                        uint8_t channel,
                        uint32_t can_id,
                        const uint8_t *data,
                        uint8_t dlen,
                        uint64_t timestamp);

    void Tick100ms();

    // 以下两个方法须在 IO 线程上下文调用，且围绕 linkx_send_pdos 顺序执行：
    //   ServiceResetRequests()  —— send_pdos 之前：把控制线程请求的复位帧入队
    //   ClearOneShotResetPdos() —— send_pdos 之后：清掉刚发出的复位帧
    // 二者同线程顺序执行，保证每条复位帧只占一个 EtherCAT 周期、只发一次
    // （参考 tfmini_s_can_read_main.cpp 的 io_thread_func 做法）。
    void ServiceResetRequests();
    void ClearOneShotResetPdos();

private:
    struct Frame
    {
        uint16_t distance_cm = 0;
        uint16_t strength = 0;
        uint16_t temperature_raw = 0;
        float temperature_c = 0.0f;
        bool valid = false;
    };

    struct Sensor
    {
        uint32_t slave_id = 0;
        uint8_t channel = 0;
        uint32_t can_id = 0;
        const char *topic = "";
        const char *frame_id = "";

        uint8_t rx_index = 0;
        uint8_t rx_buffer[9] = {};
        uint8_t reset_reply_index = 0;
        uint8_t reset_reply_buffer[5] = {};
        uint8_t last_raw_dlen = 0;
        uint8_t last_raw_data[LINKX_CAN_MAX_DATA_BYTES] = {};
        Frame latest {};
        bool online = false;
        uint64_t raw_can_count = 0;
        uint64_t raw_byte_count = 0;
        uint64_t parsed_frame_count = 0;
        uint64_t valid_frame_count = 0;
        uint64_t invalid_frame_count = 0;
        uint64_t checksum_error_count = 0;
        uint64_t published_count = 0;
        uint32_t frame_count = 0;
        uint32_t prev_alive_frame_count = 0;
        uint8_t offline_ticks = 0;
        uint64_t last_can_timestamp = 0;
        std::chrono::steady_clock::time_point last_frame_time {};
        std::chrono::steady_clock::time_point last_reset_time {};
        uint32_t reset_count = 0;
        uint32_t reset_success_count = 0;
        uint32_t reset_fail_count = 0;

        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher;
    };

    static constexpr size_t kSensorCount = 12;

    Sensor *FindSensor(uint32_t slave_id, uint8_t channel, uint32_t can_id);
    bool PushByte(Sensor &sensor, uint8_t byte, uint64_t timestamp, Frame &out);
    bool PushResetReplyByte(Sensor &sensor, uint8_t byte);
    void PublishRange(Sensor &sensor, const Frame &frame);
    void RequestStaleResets();
    bool SendReset(Sensor &sensor);
    linkx_t *LinkxForSensor(const Sensor &sensor);
    bool ResetAllowedForSensor(const Sensor &sensor) const;
    void PrintDebug();

    rclcpp::Node::SharedPtr node_;
    std::array<Sensor, kSensorCount> sensors_ {};
    // 控制线程（RequestStaleResets）检测到传感器失联即置位；IO 线程
    // （ServiceResetRequests）取走并真正发送复位帧。与 sensors_ 同序对应。
    // 参考 can_read_main 的 st_sensor_reset.request 原子请求做法。
    std::array<std::atomic<bool>, kSensorCount> reset_request_ {};
    // 复位帧的一次性清理标志：仅在 IO 线程内被 SendReset 置位、
    // ClearOneShotResetPdos 读取并清零，二者同线程顺序执行。
    std::array<std::array<std::atomic<bool>, LINKX_CAN_CHANNEL_NUM>, 2> clear_reset_pdo_ {};
    linkx_t *slave1_linkx_ = nullptr;
    linkx_t *slave2_linkx_ = nullptr;
    bool slave2_enabled_ = false;
    bool allow_slave1_reset_ = false;
    bool reset_never_seen_ = true;
    bool publish_invalid_ = true;
    uint32_t reset_after_ms_ = 1000;
    uint32_t reset_cooldown_ms_ = 3000;
    uint32_t reset_cmd_id_offset_ = 0;
    std::chrono::steady_clock::time_point start_time_ {};
    bool debug_ = false;
    uint32_t debug_ticks_ = 0;
    bool started_ = false;
};

#endif

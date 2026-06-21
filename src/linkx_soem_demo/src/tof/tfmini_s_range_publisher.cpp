#include "tfmini_s_range_publisher.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "soem.h"

namespace
{
constexpr uint16_t kInvalidStrength = 65535U;
constexpr uint16_t kTooFarDistanceCm = 65532U;
constexpr uint8_t kOfflineTicks = 5U;  // 5 * 100ms
constexpr size_t kTfminiSensorCount = 12U;
constexpr uint8_t kResetCmd[4] = {0x5AU, 0x04U, 0x02U, 0x60U};

struct SensorMap
{
    uint32_t slave_id;
    uint8_t channel;
    uint32_t can_id;
    const char *topic;
    const char *frame_id;
};

constexpr SensorMap kSensorMap[kTfminiSensorCount] = {
    {1U, 1U, 0x01U, "/high/front_left/range", "front_left"},
    {1U, 1U, 0x02U, "/high/front_right/range", "front_right"},
    {1U, 0U, 0x01U, "/high/left_left/range", "left_left"},
    {1U, 0U, 0x02U, "/high/left_right/range", "left_right"},
    {1U, 3U, 0x01U, "/high/right_left/range", "right_left"},
    {1U, 3U, 0x02U, "/high/right_right/range", "right_right"},
    {1U, 2U, 0x01U, "/high/back_left/range", "back_left"},
    {1U, 2U, 0x02U, "/high/back_right/range", "back_right"},
    {2U, 3U, 0x02U, "/high/up_front/range", "up_front"},
    {2U, 3U, 0x01U, "/high/down_front/range", "down_front"},
    {2U, 2U, 0x01U, "/high/up_back/range", "up_back"},
    {2U, 2U, 0x02U, "/high/down_back/range", "down_back"},
};

uint32_t env_u32(const char *name, uint32_t fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || *value == '\0')
        return fallback;

    char *end = nullptr;
    const unsigned long parsed = std::strtoul(value, &end, 0);
    return (end == value) ? fallback : static_cast<uint32_t>(parsed);
}

bool env_bool(const char *name, bool fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || *value == '\0')
        return fallback;
    return std::strcmp(value, "0") != 0 &&
           std::strcmp(value, "false") != 0 &&
           std::strcmp(value, "FALSE") != 0 &&
           std::strcmp(value, "off") != 0 &&
           std::strcmp(value, "OFF") != 0;
}
}  // namespace

void TfminiSRangePublisher::Start(const rclcpp::Node::SharedPtr &node)
{
    node_ = node;
    started_ = static_cast<bool>(node_);
    start_time_ = std::chrono::steady_clock::now();

    for (size_t i = 0; i < sensors_.size(); ++i)
    {
        const auto &map = kSensorMap[i];
        Sensor &sensor = sensors_[i];
        sensor = Sensor{};
        sensor.slave_id = map.slave_id;
        sensor.channel = map.channel;
        sensor.can_id = map.can_id;
        sensor.topic = map.topic;
        sensor.frame_id = map.frame_id;
        sensor.offline_ticks = kOfflineTicks;

        if (started_)
        {
            sensor.publisher = node_->create_publisher<sensor_msgs::msg::Range>(
                sensor.topic, rclcpp::SensorDataQoS());
            RCLCPP_INFO(node_->get_logger(),
                        "[TFMINI-S] slave=%u CAN%u id=0x%02X -> %s",
                        sensor.slave_id,
                        sensor.channel,
                        sensor.can_id,
                        sensor.topic);
        }
    }
}

void TfminiSRangePublisher::ConfigureReset(linkx_t *slave1_linkx,
                                           linkx_t *slave2_linkx,
                                           bool slave2_enabled)
{
    slave1_linkx_ = slave1_linkx;
    slave2_linkx_ = slave2_linkx;
    slave2_enabled_ = slave2_enabled;
    reset_after_ms_ = env_u32("TOF_RESET_AFTER_MS", 1000U);
    reset_cooldown_ms_ = env_u32("TOF_RESET_COOLDOWN_MS", 3000U);
    reset_cmd_id_offset_ = env_u32("TOF_RESET_CMD_ID_OFFSET", 0U);
    reset_never_seen_ = env_bool("TOF_RESET_NEVER_SEEN", true);
    allow_slave1_reset_ = env_bool("TOF_RESET_ALLOW_SLAVE1", false);
    publish_invalid_ = env_bool("TOF_PUBLISH_INVALID", true);
    debug_ = env_bool("TOF_DEBUG", false);

    if (node_)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[TFMINI-S] reset cfg: after=%ums cooldown=%ums cmd_id_offset=0x%X "
            "never_seen=%d allow_slave1_can0_1=%d slave1_can2_3=1 publish_invalid=%d debug=%d; "
            "command=5A 04 02 60",
            reset_after_ms_,
            reset_cooldown_ms_,
            reset_cmd_id_offset_,
            reset_never_seen_ ? 1 : 0,
            allow_slave1_reset_ ? 1 : 0,
            publish_invalid_ ? 1 : 0,
            debug_ ? 1 : 0);
    }
}

void TfminiSRangePublisher::Stop()
{
    for (auto &sensor : sensors_)
        sensor.publisher.reset();
    node_.reset();
    started_ = false;
}

bool TfminiSRangePublisher::HandleCanFrame(uint32_t slave_id,
                                           uint8_t channel,
                                           uint32_t can_id,
                                           const uint8_t *data,
                                           uint8_t dlen,
                                           uint64_t timestamp)
{
    Sensor *sensor = FindSensor(slave_id, channel, can_id);
    if (sensor == nullptr || data == nullptr || dlen == 0U)
        return false;

    const uint8_t n = std::min<uint8_t>(dlen, 64U);
    ++sensor->raw_can_count;
    sensor->raw_byte_count += n;
    sensor->last_raw_dlen = n;
    std::memset(sensor->last_raw_data, 0, sizeof(sensor->last_raw_data));
    std::memcpy(sensor->last_raw_data, data, n);

    for (uint8_t i = 0; i < n; ++i)
    {
        PushResetReplyByte(*sensor, data[i]);

        Frame frame {};
        if (PushByte(*sensor, data[i], timestamp, frame) &&
            (frame.valid || publish_invalid_))
        {
            PublishRange(*sensor, frame);
        }
    }

    return true;
}

void TfminiSRangePublisher::Tick100ms()
{
    for (auto &sensor : sensors_)
    {
        if (sensor.frame_count != sensor.prev_alive_frame_count)
        {
            sensor.online = true;
            sensor.offline_ticks = 0;
        }
        else if (sensor.offline_ticks < kOfflineTicks)
        {
            ++sensor.offline_ticks;
            if (sensor.offline_ticks >= kOfflineTicks)
                sensor.online = false;
        }
        else
        {
            sensor.online = false;
        }

        sensor.prev_alive_frame_count = sensor.frame_count;
    }

    RequestStaleResets();

    if (debug_)
    {
        ++debug_ticks_;
        if (debug_ticks_ >= 10U)
        {
            debug_ticks_ = 0U;
            PrintDebug();
        }
    }
}

void TfminiSRangePublisher::ClearOneShotResetPdos()
{
    linkx_t *links[2] = {slave1_linkx_, slave2_linkx_};

    for (size_t li = 0; li < clear_reset_pdo_.size(); ++li)
    {
        linkx_t *link = links[li];
        if (link == nullptr)
            continue;

        for (uint8_t ch = 0; ch < LINKX_CAN_CHANNEL_NUM; ++ch)
        {
            if (!clear_reset_pdo_[li][ch])
                continue;

            std::memset(&link->rx_pdos[ch], 0, sizeof(link->rx_pdos[ch]));
            clear_reset_pdo_[li][ch] = false;
        }
    }
}

TfminiSRangePublisher::Sensor *TfminiSRangePublisher::FindSensor(uint32_t slave_id,
                                                                  uint8_t channel,
                                                                  uint32_t can_id)
{
    const uint32_t std_id = can_id & 0x7FFU;
    for (auto &sensor : sensors_)
    {
        if (sensor.slave_id == slave_id &&
            sensor.channel == channel &&
            sensor.can_id == std_id)
        {
            return &sensor;
        }
    }
    return nullptr;
}

bool TfminiSRangePublisher::PushByte(Sensor &sensor,
                                     uint8_t byte,
                                     uint64_t timestamp,
                                     Frame &out)
{
    if (sensor.rx_index == 0U)
    {
        if (byte == 0x59U)
        {
            sensor.rx_buffer[0] = byte;
            sensor.rx_index = 1U;
        }
        return false;
    }

    if (sensor.rx_index == 1U && byte != 0x59U)
    {
        sensor.rx_index = 0U;
        if (byte == 0x59U)
        {
            sensor.rx_buffer[0] = byte;
            sensor.rx_index = 1U;
        }
        return false;
    }

    sensor.rx_buffer[sensor.rx_index++] = byte;
    if (sensor.rx_index < 9U)
        return false;

    sensor.rx_index = 0U;

    uint16_t checksum = 0;
    for (int i = 0; i < 8; ++i)
        checksum = static_cast<uint16_t>(checksum + sensor.rx_buffer[i]);

    if (static_cast<uint8_t>(checksum & 0xFFU) != sensor.rx_buffer[8])
    {
        ++sensor.checksum_error_count;
        return false;
    }

    out.distance_cm = static_cast<uint16_t>((sensor.rx_buffer[3] << 8) | sensor.rx_buffer[2]);
    out.strength = static_cast<uint16_t>((sensor.rx_buffer[5] << 8) | sensor.rx_buffer[4]);
    out.temperature_raw = static_cast<uint16_t>((sensor.rx_buffer[7] << 8) | sensor.rx_buffer[6]);
    out.temperature_c = static_cast<float>(out.temperature_raw) / 8.0f - 256.0f;
    out.valid = (out.strength >= 100U &&
                 out.strength != kInvalidStrength &&
                 out.distance_cm != 0U &&
                 out.distance_cm != kTooFarDistanceCm);

    sensor.latest = out;
    sensor.online = true;
    sensor.offline_ticks = 0;
    ++sensor.parsed_frame_count;
    if (out.valid)
        ++sensor.valid_frame_count;
    else
        ++sensor.invalid_frame_count;
    sensor.frame_count++;
    sensor.last_can_timestamp = timestamp;
    sensor.last_frame_time = std::chrono::steady_clock::now();
    return true;
}

bool TfminiSRangePublisher::PushResetReplyByte(Sensor &sensor, uint8_t byte)
{
    uint8_t &index = sensor.reset_reply_index;
    uint8_t *buffer = sensor.reset_reply_buffer;

    if (index == 0U)
    {
        if (byte == 0x5AU)
        {
            buffer[0] = byte;
            index = 1U;
        }
        return false;
    }

    if ((index == 1U && byte != 0x05U) ||
        (index == 2U && byte != 0x02U))
    {
        index = 0U;
        if (byte == 0x5AU)
        {
            buffer[0] = byte;
            index = 1U;
        }
        return false;
    }

    buffer[index++] = byte;
    if (index < 5U)
        return false;

    index = 0U;
    const uint8_t checksum = static_cast<uint8_t>(
        (buffer[0] + buffer[1] + buffer[2] + buffer[3]) & 0xFFU);
    if (checksum != buffer[4])
        return false;

    if (buffer[3] == 0x00U)
    {
        ++sensor.reset_success_count;
        if (node_)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[TFMINI-S] reset ACK success slave=%u CAN%u id=0x%02X "
                        "success=%u fail=%u",
                        sensor.slave_id,
                        sensor.channel,
                        sensor.can_id,
                        sensor.reset_success_count,
                        sensor.reset_fail_count);
        }
    }
    else if (buffer[3] == 0x01U)
    {
        ++sensor.reset_fail_count;
        if (node_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[TFMINI-S] reset ACK failed slave=%u CAN%u id=0x%02X "
                         "success=%u fail=%u",
                         sensor.slave_id,
                         sensor.channel,
                         sensor.can_id,
                         sensor.reset_success_count,
                         sensor.reset_fail_count);
        }
    }

    return true;
}

void TfminiSRangePublisher::PublishRange(Sensor &sensor, const Frame &frame)
{
    if (!started_ || !node_ || !sensor.publisher)
        return;

    sensor_msgs::msg::Range msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = sensor.frame_id;
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = 0.0349f;
    msg.min_range = 0.1f;
    msg.max_range = 12.0f;
    msg.range = static_cast<float>(frame.distance_cm) * 0.01f;
    sensor.publisher->publish(msg);
    ++sensor.published_count;
}

// 控制线程：对每个失联（或从未上线）超过 reset_after_ms 的传感器，置一个原子
// 复位请求，由 IO 线程在 send_pdos 之前真正发送。带 per-sensor 冷却，避免对一个
// 死掉的传感器每个 tick 都重发。只更新控制线程私有的 last_reset_time/reset_count。
// 参考 can_read_main 的 check_and_request_resets。
void TfminiSRangePublisher::RequestStaleResets()
{
    if (reset_after_ms_ == 0U)
        return;

    const auto now = std::chrono::steady_clock::now();
    for (size_t i = 0; i < sensors_.size(); ++i)
    {
        Sensor &sensor = sensors_[i];

        if (!ResetAllowedForSensor(sensor))
            continue;

        if (sensor.frame_count == 0U && !reset_never_seen_)
            continue;

        const auto ref = (sensor.frame_count == 0U) ? start_time_ : sensor.last_frame_time;
        if (ref.time_since_epoch().count() == 0)
            continue;

        const auto age_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - ref).count();
        if (age_ms < static_cast<long long>(reset_after_ms_))
            continue;

        if (sensor.last_reset_time.time_since_epoch().count() != 0)
        {
            const auto since_reset_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - sensor.last_reset_time).count();
            if (since_reset_ms < static_cast<long long>(reset_cooldown_ms_))
                continue;
        }

        // 仅置位请求；真正发送在 IO 线程的 ServiceResetRequests 完成。
        reset_request_[i].store(true, std::memory_order_relaxed);
        sensor.last_reset_time = now;
        ++sensor.reset_count;
        if (node_)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[TFMINI-S] slave=%u CAN%u id=0x%02X no data >%ums, "
                        "requesting reset (#%u)",
                        sensor.slave_id,
                        sensor.channel,
                        sensor.can_id,
                        reset_after_ms_,
                        sensor.reset_count);
        }
    }
}

// IO 线程：取走控制线程置位的复位请求，把复位帧入队（linkx_send_can 线程安全），
// 紧接其后的 linkx_send_pdos 会把它刷到总线，再由 ClearOneShotResetPdos 清掉，
// 三步同在 IO 线程顺序执行 —— 保证复位帧只发一次，不会把传感器按死在复位里。
void TfminiSRangePublisher::ServiceResetRequests()
{
    for (size_t i = 0; i < sensors_.size(); ++i)
    {
        if (reset_request_[i].exchange(false, std::memory_order_relaxed))
            SendReset(sensors_[i]);
    }
}

bool TfminiSRangePublisher::SendReset(Sensor &sensor)
{
    linkx_t *link = LinkxForSensor(sensor);
    if (link == nullptr || sensor.channel >= LINKX_CAN_CHANNEL_NUM)
        return false;

    const uint32_t cmd_id = sensor.can_id + reset_cmd_id_offset_;
    const bool ok = linkx_send_can(link,
                                   sensor.channel,
                                   cmd_id,
                                   true,
                                   true,
                                   false,
                                   false,
                                   sizeof(kResetCmd),
                                   kResetCmd);
    if (ok)
    {
        const size_t link_index = (sensor.slave_id == 1U) ? 0U : 1U;
        clear_reset_pdo_[link_index][sensor.channel] = true;
    }
    return ok;
}

linkx_t *TfminiSRangePublisher::LinkxForSensor(const Sensor &sensor)
{
    if (sensor.slave_id == 1U)
        return slave1_linkx_;
    if (sensor.slave_id == 2U && slave2_enabled_)
        return slave2_linkx_;
    return nullptr;
}

bool TfminiSRangePublisher::ResetAllowedForSensor(const Sensor &sensor) const
{
    if (sensor.slave_id == 2U)
        return slave2_enabled_;

    if (sensor.slave_id != 1U)
        return false;

    // Slave1 CAN2/CAN3 are ToF-only on this vehicle, so they can use the same
    // automatic recovery path as slave2. CAN0/CAN1 stay opt-in because they may
    // share IDs with vehicle actuators on other wiring revisions.
    if (sensor.channel == 2U || sensor.channel == 3U)
        return true;

    return allow_slave1_reset_;
}

void TfminiSRangePublisher::PrintDebug()
{
    if (!node_)
        return;

    const auto now = std::chrono::steady_clock::now();
    for (const auto &sensor : sensors_)
    {
        long long age_ms = -1;
        if (sensor.last_frame_time.time_since_epoch().count() != 0)
        {
            age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         now - sensor.last_frame_time).count();
        }

        char raw_hex[(LINKX_CAN_MAX_DATA_BYTES * 3) + 1] = {};
        size_t offset = 0;
        const uint8_t n = std::min<uint8_t>(sensor.last_raw_dlen, LINKX_CAN_MAX_DATA_BYTES);
        for (uint8_t i = 0; i < n && offset < sizeof(raw_hex); ++i)
        {
            const int written = std::snprintf(raw_hex + offset,
                                              sizeof(raw_hex) - offset,
                                              "%s%02X",
                                              (i == 0U) ? "" : " ",
                                              sensor.last_raw_data[i]);
            if (written <= 0)
                break;
            offset += static_cast<size_t>(written);
        }

        RCLCPP_WARN(node_->get_logger(),
                    "[TFMINI-S][DBG] slave=%u CAN%u id=0x%02X topic=%s "
                    "raw=%llu bytes=%llu parsed=%llu valid=%llu invalid=%llu dist_cm=%u strength=%u "
                    "cksum=%llu pub=%llu age_ms=%lld reset=%u ack_ok=%u ack_fail=%u last=[%s]",
                    sensor.slave_id,
                    sensor.channel,
                    sensor.can_id,
                    sensor.topic,
                    static_cast<unsigned long long>(sensor.raw_can_count),
                    static_cast<unsigned long long>(sensor.raw_byte_count),
                    static_cast<unsigned long long>(sensor.parsed_frame_count),
                    static_cast<unsigned long long>(sensor.valid_frame_count),
                    static_cast<unsigned long long>(sensor.invalid_frame_count),
                    sensor.latest.distance_cm,
                    sensor.latest.strength,
                    static_cast<unsigned long long>(sensor.checksum_error_count),
                    static_cast<unsigned long long>(sensor.published_count),
                    age_ms,
                    sensor.reset_count,
                    sensor.reset_success_count,
                    sensor.reset_fail_count,
                    raw_hex);
    }
}

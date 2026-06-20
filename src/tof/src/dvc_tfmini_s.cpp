#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "linkx4c_handler.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#define TFMINI_S_NUM 12

namespace {

constexpr int MODULE_ALIAS_FRONT_REAR = 0;
constexpr int MODULE_ALIAS_UP_DOWN = 1;

struct RxSource {
    int module_alias;
    uint8_t channel;
};

}  // namespace

class Dvc_TFmini_S {
public:
    Dvc_TFmini_S();

    void Init(int module_alias,
              uint8_t channel,
              uint32_t data_id,
              uint32_t cmd_id,
              rclcpp::Node::SharedPtr node,
              const std::string& topic_name);

    void CAN_RxCpltCallback(int module_alias, uint8_t channel, const can_msg_t& rx_msg);
    void TIM_Alive_PeriodElapsedCallback();
    void Shutdown();

    uint16_t GetDistance()    const { return distance_cm; }
    uint16_t GetStrength()    const { return strength; }
    float    GetTemperature() const { return temperature / 8.0f - 256.0f; }
    bool     IsOnline()       const { return online; }
    bool     IsValid()        const { return strength >= 100 && strength != 65535; }
    int      GetModuleAlias() const { return module_alias_; }
    uint8_t  GetChannel()     const { return can_channel; }
    uint32_t GetCanId()       const { return can_id; }
    uint32_t GetRateHz()      const { return rate_hz; }

private:
    void ParseByte(uint8_t byte);
    void PublishData();

    int       module_alias_;
    uint8_t   can_channel;
    uint32_t  can_id;
    uint32_t  cmd_can_id;
    uint8_t   rx_index;
    uint8_t   rx_buffer[9];

    uint16_t  distance_cm;
    uint16_t  strength;
    uint16_t  temperature;
    uint32_t  frame_cnt;
    uint32_t  pre_frame_cnt;
    uint32_t  rate_hz;
    bool      online;

    std::string frame_id_;
    std::string topic_name_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
};

class Dvc_TFmini_S_Array {
public:
    void Init(rclcpp::Node::SharedPtr node);
    void CAN_RxCpltCallback(int module_alias, uint8_t channel, const can_msg_t& rx_msg);
    void TIM_Alive_PeriodElapsedCallback();
    void Shutdown();

private:
    Dvc_TFmini_S sensors_[TFMINI_S_NUM];
    rclcpp::Node::SharedPtr node_;
};

Dvc_TFmini_S::Dvc_TFmini_S()
    : module_alias_(-1), can_channel(0), can_id(0), cmd_can_id(0),
      rx_index(0), distance_cm(0), strength(0), temperature(0),
      frame_cnt(0), pre_frame_cnt(0), rate_hz(0), online(false)
{
    rx_buffer[0] = 0;
}

void Dvc_TFmini_S::Init(int module_alias,
                        uint8_t channel,
                        uint32_t data_id,
                        uint32_t cmd_id,
                        rclcpp::Node::SharedPtr node,
                        const std::string& topic_name)
{
    module_alias_ = module_alias;
    can_channel = channel;
    can_id = data_id;
    cmd_can_id = cmd_id;
    node_ = node;
    topic_name_ = topic_name;
    frame_id_ = "radar_m" + std::to_string(module_alias_) +
                "_ch" + std::to_string(channel) +
                "_id_" + std::to_string(data_id);
    pub_ = node->create_publisher<sensor_msgs::msg::Range>(
        topic_name, rclcpp::SensorDataQoS());

    RCLCPP_INFO(node->get_logger(),
                "[TFmini-S] module=%d ID=0x%03X CMD_ID=0x%03X CH%d -> %s",
                module_alias_, can_id, cmd_can_id, can_channel, topic_name.c_str());
}

void Dvc_TFmini_S::CAN_RxCpltCallback(int module_alias,
                                      uint8_t channel,
                                      const can_msg_t& rx_msg)
{
    if (module_alias != module_alias_ || channel != can_channel || rx_msg.id != can_id) return;

    frame_cnt++;
    uint8_t n = (rx_msg.dlen > sizeof(rx_msg.data))
                ? static_cast<uint8_t>(sizeof(rx_msg.data))
                : rx_msg.dlen;
    for (uint8_t i = 0; i < n; i++) {
        ParseByte(rx_msg.data[i]);
    }
}

void Dvc_TFmini_S::TIM_Alive_PeriodElapsedCallback()
{
    rate_hz = frame_cnt - pre_frame_cnt;
    bool now_online = (rate_hz != 0);

    if (now_online != online) {
        if (now_online) {
            RCLCPP_INFO(node_->get_logger(),
                        "[TFmini-S] M%d CH%u/0x%02X ONLINE (%u fps) %s",
                        module_alias_, can_channel, can_id, rate_hz, topic_name_.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(),
                        "[TFmini-S] M%d CH%u/0x%02X OFFLINE %s",
                        module_alias_, can_channel, can_id, topic_name_.c_str());
        }
    }

    online = now_online;
    pre_frame_cnt = frame_cnt;
}

void Dvc_TFmini_S::ParseByte(uint8_t byte)
{
    if (rx_index == 0 && byte != 0x59) return;
    if (rx_index == 1 && byte != 0x59) {
        rx_index = 0;
        return;
    }

    rx_buffer[rx_index++] = byte;

    if (rx_index == 9) {
        rx_index = 0;

        uint16_t checksum = 0;
        for (int i = 0; i < 8; i++) checksum += rx_buffer[i];

        if (static_cast<uint8_t>(checksum & 0xFF) == rx_buffer[8]) {
            distance_cm = static_cast<uint16_t>((rx_buffer[3] << 8) | rx_buffer[2]);
            strength = static_cast<uint16_t>((rx_buffer[5] << 8) | rx_buffer[4]);
            temperature = static_cast<uint16_t>((rx_buffer[7] << 8) | rx_buffer[6]);
            PublishData();
        }
    }
}

void Dvc_TFmini_S::PublishData()
{
    if (!pub_) return;

    float range_m = distance_cm / 100.0f;
    if (distance_cm == 0) {
        range_m = -std::numeric_limits<float>::infinity();
    } else if (distance_cm == 65532) {
        range_m = std::numeric_limits<float>::infinity();
    } else if (!IsValid()) {
        range_m = std::numeric_limits<float>::quiet_NaN();
    }

    sensor_msgs::msg::Range msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = frame_id_;
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = 0.0349f;
    msg.min_range = 0.1f;
    msg.max_range = 12.0f;
    msg.range = range_m;

    pub_->publish(msg);
}

void Dvc_TFmini_S::Shutdown()
{
    pub_.reset();
    node_.reset();
}

void Dvc_TFmini_S_Array::Init(rclcpp::Node::SharedPtr node)
{
    node_ = node;

    struct SensorConfig {
        int module_alias;
        uint8_t channel;
        uint32_t id;
        const char* topic;
    };

    constexpr SensorConfig defaults[TFMINI_S_NUM] = {
        {MODULE_ALIAS_FRONT_REAR, 0, 0x01, "/high/left/range"},
        {MODULE_ALIAS_FRONT_REAR, 0, 0x02, "/high/left_2/range"},
        {MODULE_ALIAS_FRONT_REAR, 1, 0x01, "/high/front/range"},
        {MODULE_ALIAS_FRONT_REAR, 1, 0x02, "/high/front_2/range"},
        {MODULE_ALIAS_FRONT_REAR, 2, 0x01, "/high/back/range"},
        {MODULE_ALIAS_FRONT_REAR, 2, 0x02, "/high/back_2/range"},
        {MODULE_ALIAS_FRONT_REAR, 3, 0x01, "/high/right/range"},
        {MODULE_ALIAS_FRONT_REAR, 3, 0x02, "/high/right_2/range"},
        {MODULE_ALIAS_UP_DOWN, 2, 0x01, "/high/up_front/range"},
        {MODULE_ALIAS_UP_DOWN, 2, 0x02, "/high/up_back/range"},
        {MODULE_ALIAS_UP_DOWN, 3, 0x01, "/high/down_front/range"},
        {MODULE_ALIAS_UP_DOWN, 3, 0x02, "/high/down_back/range"},
    };

    for (int i = 0; i < TFMINI_S_NUM; ++i) {
        sensors_[i].Init(defaults[i].module_alias,
                         defaults[i].channel,
                         defaults[i].id,
                         defaults[i].id | 0x100,
                         node,
                         defaults[i].topic);
    }
}

void Dvc_TFmini_S_Array::CAN_RxCpltCallback(int module_alias,
                                            uint8_t channel,
                                            const can_msg_t& rx_msg)
{
    for (int i = 0; i < TFMINI_S_NUM; ++i) {
        sensors_[i].CAN_RxCpltCallback(module_alias, channel, rx_msg);
    }
}

void Dvc_TFmini_S_Array::TIM_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < TFMINI_S_NUM; ++i) {
        sensors_[i].TIM_Alive_PeriodElapsedCallback();
    }

    char line[1024];
    int off = 0;
    int n_online = 0;
    off += snprintf(line + off, sizeof(line) - off, "[TFmini-S ALIVE]");
    for (int i = 0; i < TFMINI_S_NUM; ++i) {
        const Dvc_TFmini_S& s = sensors_[i];
        if (s.IsOnline()) {
            n_online++;
            off += snprintf(line + off, sizeof(line) - off,
                            " M%d/CH%u/0x%02X=%ufps/%ucm",
                            s.GetModuleAlias(), s.GetChannel(), s.GetCanId(),
                            s.GetRateHz(), s.GetDistance());
        } else {
            off += snprintf(line + off, sizeof(line) - off,
                            " M%d/CH%u/0x%02X=----",
                            s.GetModuleAlias(), s.GetChannel(), s.GetCanId());
        }
    }
    snprintf(line + off, sizeof(line) - off, " [%d/%d online]", n_online, TFMINI_S_NUM);

    if (n_online == TFMINI_S_NUM) {
        RCLCPP_INFO(node_->get_logger(), "%s", line);
    } else {
        RCLCPP_WARN(node_->get_logger(), "%s", line);
    }
}

void Dvc_TFmini_S_Array::Shutdown()
{
    for (int i = 0; i < TFMINI_S_NUM; ++i) {
        sensors_[i].Shutdown();
    }
    node_.reset();
}

static Dvc_TFmini_S_Array tfmini_array;

void task_init(rclcpp::Node::SharedPtr node) {
    tfmini_array.Init(node);
}

void task_shutdown() {
    tfmini_array.Shutdown();
}

void Pump_CAN_Receive(int module_alias, uint8_t channel, const can_msg_t& rx_msg) {
    tfmini_array.CAN_RxCpltCallback(module_alias, channel, rx_msg);
}

void Tick_Alive_Check() {
    tfmini_array.TIM_Alive_PeriodElapsedCallback();
}

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "linkx4c_handler.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#define TFMINI_S_NUM 8

// ============================================================
//  类声明
// ============================================================

class Dvc_TFmini_S {
public:
    Dvc_TFmini_S();

    void Init(linkx_t* linkx_master,
              uint8_t channel,
              uint32_t data_id,
              uint32_t cmd_id,       // 传感器接收命令的 CAN ID，查传感器手册确认
              rclcpp::Node::SharedPtr node,
              const std::string& topic_name);

    void CAN_RxCpltCallback(uint8_t channel, const can_msg_t& rx_msg);
    void TIM_Alive_PeriodElapsedCallback();
    void Shutdown();

    uint16_t GetDistance()    const { return distance_cm; }
    uint16_t GetStrength()    const { return strength; }
    float    GetTemperature() const { return temperature / 8.0f - 256.0f; }
    bool     IsOnline()       const { return online; }
    bool     IsValid()        const { return strength >= 100 && strength != 65535; }
    uint8_t  GetChannel()     const { return can_channel; }
    uint32_t GetCanId()       const { return can_id; }
    uint32_t GetRateHz()      const { return rate_hz; }   // 上一周期(~1s)收到的 CAN 帧数

private:
    void ParseByte(uint8_t byte);
    void PublishData();

    linkx_t*  p_linkx;
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
    uint32_t  rate_hz;      // 上一存活周期(~1s)内收到的 CAN 帧数
    bool      online;

    std::string frame_id_;    // Init() 时预构建，PublishData() 直接使用
    std::string topic_name_;  // 仅用于诊断日志，便于定位物理位置

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
};

class Dvc_TFmini_S_Array {
public:
    void Init(linkx_t* linkx_master, rclcpp::Node::SharedPtr node);
    void CAN_RxCpltCallback(uint8_t channel, const can_msg_t& rx_msg);
    void TIM_Alive_PeriodElapsedCallback();
    void Shutdown();

private:
    Dvc_TFmini_S sensors_[TFMINI_S_NUM];
    rclcpp::Node::SharedPtr node_;
};

// ============================================================
//  单传感器实现
// ============================================================

Dvc_TFmini_S::Dvc_TFmini_S()
    : p_linkx(nullptr), can_channel(0), can_id(0), cmd_can_id(0),
      rx_index(0), distance_cm(0), strength(0), temperature(0),
      frame_cnt(0), pre_frame_cnt(0), rate_hz(0), online(false)
{
    rx_buffer[0] = 0;
}

void Dvc_TFmini_S::Init(linkx_t* linkx_master,
                         uint8_t channel,
                         uint32_t data_id,
                         uint32_t cmd_id,
                         rclcpp::Node::SharedPtr node,
                         const std::string& topic_name)
{
    p_linkx     = linkx_master;
    can_channel = channel;
    can_id      = data_id;
    cmd_can_id  = cmd_id;
    node_       = node;
    topic_name_ = topic_name;
    frame_id_   = "radar_ch" + std::to_string(channel) + "_id_" + std::to_string(data_id);
    pub_        = node->create_publisher<sensor_msgs::msg::Range>(
                      topic_name, rclcpp::SensorDataQoS());

    RCLCPP_INFO(node->get_logger(),
                "[TFmini-S] ID=0x%03X CMD_ID=0x%03X CH%d -> %s",
                can_id, cmd_can_id, can_channel, topic_name.c_str());
}

void Dvc_TFmini_S::CAN_RxCpltCallback(uint8_t channel,
                                        const can_msg_t& rx_msg)
{
    if (channel != can_channel || rx_msg.id != can_id) return;

    frame_cnt++;
    uint8_t n = (rx_msg.dlen > sizeof(rx_msg.data))
                ? static_cast<uint8_t>(sizeof(rx_msg.data))
                : rx_msg.dlen;
    for (uint8_t i = 0; i < n; i++)
        ParseByte(rx_msg.data[i]);
}

void Dvc_TFmini_S::TIM_Alive_PeriodElapsedCallback() {
    rate_hz = frame_cnt - pre_frame_cnt;       // 本周期(~1s)收到的 CAN 帧数
    bool now_online = (rate_hz != 0);

    // 仅在状态跳变时打印，抓间歇性掉线，避免刷屏
    if (now_online != online) {
        if (now_online)
            RCLCPP_INFO(node_->get_logger(),
                        "[TFmini-S] CH%u/0x%02X ONLINE  (%u fps) %s",
                        can_channel, can_id, rate_hz, topic_name_.c_str());
        else
            RCLCPP_WARN(node_->get_logger(),
                        "[TFmini-S] CH%u/0x%02X OFFLINE  %s",
                        can_channel, can_id, topic_name_.c_str());
    }

    online        = now_online;
    pre_frame_cnt = frame_cnt;
}

void Dvc_TFmini_S::ParseByte(uint8_t byte) {
    if (rx_index == 0 && byte != 0x59) return;
    if (rx_index == 1 && byte != 0x59) { rx_index = 0; return; }

    rx_buffer[rx_index++] = byte;

    if (rx_index == 9) {
        rx_index = 0;

        uint16_t checksum = 0;
        for (int i = 0; i < 8; i++) checksum += rx_buffer[i];

        if (static_cast<uint8_t>(checksum & 0xFF) == rx_buffer[8]) {
            distance_cm = static_cast<uint16_t>((rx_buffer[3] << 8) | rx_buffer[2]);
            strength    = static_cast<uint16_t>((rx_buffer[5] << 8) | rx_buffer[4]);
            temperature = static_cast<uint16_t>((rx_buffer[7] << 8) | rx_buffer[6]);
            PublishData();
        }
    }
}

void Dvc_TFmini_S::PublishData() {
    if (!pub_) return;

    float range_m = distance_cm / 100.0f;
    if (distance_cm == 0) {
        range_m = -std::numeric_limits<float>::infinity();   // too close
    } else if (distance_cm == 65532) {
        range_m = std::numeric_limits<float>::infinity();    // too far / no target
    } else if (!IsValid()) {
        range_m = std::numeric_limits<float>::quiet_NaN();   // signal invalid
    }

    sensor_msgs::msg::Range msg;
    msg.header.stamp    = node_->now();
    msg.header.frame_id = frame_id_;
    msg.radiation_type  = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view   = 0.0349f;
    msg.min_range       = 0.1f;
    msg.max_range       = 12.0f;
    msg.range           = range_m;

    pub_->publish(msg);
}

void Dvc_TFmini_S::Shutdown() {
    pub_.reset();
    node_.reset();
}

// ============================================================
//  传感器阵列实现
// ============================================================

void Dvc_TFmini_S_Array::Init(linkx_t* linkx_master,
                                rclcpp::Node::SharedPtr node)
{
    node_ = node;

    // ── 传感器映射（可经 ROS 参数覆盖，免重新编译即可改接线分布）──────
    //   tfmini.channels : 每颗传感器所在 CAN 通道 (0~3)，长度必须 = 8
    //   tfmini.ids      : 每颗传感器数据输出 CAN ID (0x01~0x08)，长度必须 = 8
    //   tfmini.topics   : 每颗传感器发布话题，长度必须 = 8
    //   cmd_id 一律取 (data_id | 0x100)，本驱动只读、并不使用它发送
    //   默认值 = 现有接线：CH3 四颗 + CH2 四颗
    const std::vector<int64_t> def_ch =
        { 3, 3, 3, 3,  2, 2, 2, 2 };
    const std::vector<int64_t> def_id =
        { 0x04, 0x03, 0x02, 0x01,  0x01, 0x02, 0x03, 0x04 };
    const std::vector<std::string> def_topic = {
        "/high/front/range", "/high/left/range",
        "/high/right/range", "/high/back/range",
        "/high/up_front/range", "/high/up_back/range",
        "/high/down_front/range", "/high/down_back/range" };

    auto chans  = node->declare_parameter<std::vector<int64_t>>("tfmini.channels", def_ch);
    auto ids    = node->declare_parameter<std::vector<int64_t>>("tfmini.ids",      def_id);
    auto topics = node->declare_parameter<std::vector<std::string>>("tfmini.topics", def_topic);

    if (chans.size()  != TFMINI_S_NUM ||
        ids.size()    != TFMINI_S_NUM ||
        topics.size() != TFMINI_S_NUM) {
        RCLCPP_WARN(node->get_logger(),
            "[TFmini-S] 参数数组长度 != %d，回退到默认映射", TFMINI_S_NUM);
        chans  = def_ch;
        ids    = def_id;
        topics = def_topic;
    }

    for (int i = 0; i < TFMINI_S_NUM; ++i) {
        uint8_t  ch = static_cast<uint8_t>(chans[i]);
        uint32_t id = static_cast<uint32_t>(ids[i]);
        sensors_[i].Init(linkx_master, ch, id, id | 0x100, node, topics[i]);
    }
}

void Dvc_TFmini_S_Array::CAN_RxCpltCallback(uint8_t channel,
                                              const can_msg_t& rx_msg)
{
    for (int i = 0; i < TFMINI_S_NUM; ++i)
        sensors_[i].CAN_RxCpltCallback(channel, rx_msg);
}

void Dvc_TFmini_S_Array::TIM_Alive_PeriodElapsedCallback() {
    for (int i = 0; i < TFMINI_S_NUM; ++i)
        sensors_[i].TIM_Alive_PeriodElapsedCallback();

    // ── 逐传感器在线状态一行汇总（每 ~1s 一次）──────────────
    //   ONLINE 显示帧率(fps) 与距离(cm)，OFFLINE 显示 "----"
    char line[640];
    int off = 0;
    int n_online = 0;
    off += snprintf(line + off, sizeof(line) - off, "[TFmini-S ALIVE]");
    for (int i = 0; i < TFMINI_S_NUM; ++i) {
        const Dvc_TFmini_S& s = sensors_[i];
        if (s.IsOnline()) {
            n_online++;
            off += snprintf(line + off, sizeof(line) - off,
                            "  CH%u/0x%02X=%ufps/%ucm",
                            s.GetChannel(), s.GetCanId(),
                            s.GetRateHz(), s.GetDistance());
        } else {
            off += snprintf(line + off, sizeof(line) - off,
                            "  CH%u/0x%02X=----",
                            s.GetChannel(), s.GetCanId());
        }
    }
    snprintf(line + off, sizeof(line) - off, "   [%d/%d online]",
             n_online, TFMINI_S_NUM);

    if (n_online == TFMINI_S_NUM)
        RCLCPP_INFO(node_->get_logger(), "%s", line);
    else
        RCLCPP_WARN(node_->get_logger(), "%s", line);   // 有掉线 → WARN 级别更醒目
}

void Dvc_TFmini_S_Array::Shutdown() {
    for (int i = 0; i < TFMINI_S_NUM; ++i)
        sensors_[i].Shutdown();
    node_.reset();
}

// ============================================================
//  任务接口（供 can_link_test.cpp 通过 extern 调用）
// ============================================================

static Dvc_TFmini_S_Array tfmini_array;

void task_init(rclcpp::Node::SharedPtr node, linkx_t* linkx_master) {
    tfmini_array.Init(linkx_master, node);
}

void task_shutdown() {
    tfmini_array.Shutdown();
}

void Pump_CAN_Receive(uint8_t channel, const can_msg_t& rx_msg) {
    tfmini_array.CAN_RxCpltCallback(channel, rx_msg);
}

void Tick_Alive_Check() {
    tfmini_array.TIM_Alive_PeriodElapsedCallback();
}

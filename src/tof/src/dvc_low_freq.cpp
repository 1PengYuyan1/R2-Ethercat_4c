// dvc_low_freq.cpp
// 低频测距传感器驱动（CAN 总线，经由 DM-LinKX-4c EtherCAT 网关）
//
// 与高频 TFmini-S 的关键区别：
//   高频: 每个传感器有独立 CAN ID，靠 msg.id 区分
//   低频: 所有传感器共用同一接收 CAN ID（LOW_DATA_CAN_ID），
//         靠数据帧 Byte0 中的设备地址 (0x01-0x04) 区分不同传感器
//
// 接收帧格式（按实际传感器手册调整）：
//   Byte 0   : 设备地址  0x01=后  0x02=右  0x03=左  0x04=前
//   Byte 1   : 距离低字节（mm，小端）
//   Byte 2   : 距离高字节（mm，小端）
//   Byte 3   : 状态 / 保留（可选）
//
// 命令帧格式（按手册调整，当前为占位符）：
//   Byte 0   : 设备地址
//   Byte 1   : 命令字
//   Byte 2-N : 参数
//
// 话题（CH3，对应图片配置）：
//   /low/front/range  /low/left/range  /low/right/range  /low/back/range

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>

#include "linkx4c_handler.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

// ── 按传感器手册填写这两个 CAN ID ─────────────────────────
static constexpr uint32_t LOW_DATA_CAN_ID = 0x05;  // 所有传感器发出数据使用的 CAN ID
static constexpr uint8_t  LOW_FREQ_CH     = 3;      // CAN 通道号（图片 ch3）

static constexpr uint16_t LOW_MIN_MM      = 50;
static constexpr uint16_t LOW_MAX_MM      = 3000;
static constexpr uint16_t LOW_NO_TGT_MM   = 3001;

// ============================================================
//  单传感器
// ============================================================

class Dvc_LowFreq {
public:
    Dvc_LowFreq();

    void Init(rclcpp::Node::SharedPtr node,
              uint8_t device_addr,       // 数据帧地址字节 (0x01-0x04)
              const std::string& topic_name);

    // 由阵列在地址匹配后调用
    void OnFrame(const uint8_t* data, uint8_t dlen);
    void TIM_Alive_PeriodElapsedCallback();
    void Shutdown();

    uint8_t GetAddr()   const { return addr_; }
    bool    IsOnline()  const { return online_; }

private:
    void PublishRange(float range_m);

    uint8_t  addr_;
    uint32_t frame_cnt_;
    uint32_t pre_frame_cnt_;
    bool     online_;

    std::string frame_id_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
};

Dvc_LowFreq::Dvc_LowFreq()
    : addr_(0), frame_cnt_(0), pre_frame_cnt_(0), online_(false)
{}

void Dvc_LowFreq::Init(rclcpp::Node::SharedPtr node,
                        uint8_t device_addr,
                        const std::string& topic_name)
{
    addr_     = device_addr;
    node_     = node;
    frame_id_ = "low_addr_" + std::to_string(device_addr);
    pub_ = node->create_publisher<sensor_msgs::msg::Range>(
               topic_name, rclcpp::SensorDataQoS());

    RCLCPP_INFO(node->get_logger(),
                "[LowFreq] addr=0x%02X  CAN_ID=0x%03X  CH%d -> %s",
                addr_, LOW_DATA_CAN_ID, LOW_FREQ_CH, topic_name.c_str());
}

void Dvc_LowFreq::OnFrame(const uint8_t* data, uint8_t dlen)
{
    frame_cnt_++;

    if (dlen < 3) {
        RCLCPP_WARN_ONCE(node_->get_logger(),
                         "[LowFreq addr=0x%02X] 帧长度不足 (%d bytes)", addr_, dlen);
        return;
    }

    // Bytes 1-2：距离 mm，小端 uint16（按手册调整）
    uint16_t raw_mm = static_cast<uint16_t>(data[1]) |
                      (static_cast<uint16_t>(data[2]) << 8);

    if (raw_mm >= LOW_NO_TGT_MM) {
        PublishRange(std::numeric_limits<float>::infinity());
        return;
    }
    if (raw_mm < LOW_MIN_MM) {
        PublishRange(std::numeric_limits<float>::quiet_NaN());
        return;
    }

    PublishRange(raw_mm / 1000.0f);
}

void Dvc_LowFreq::TIM_Alive_PeriodElapsedCallback()
{
    online_        = (frame_cnt_ != pre_frame_cnt_);
    pre_frame_cnt_ = frame_cnt_;
}

void Dvc_LowFreq::PublishRange(float range_m)
{
    if (!pub_) return;

    sensor_msgs::msg::Range msg;
    msg.header.stamp    = node_->now();
    msg.header.frame_id = frame_id_;
    msg.radiation_type  = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view   = 0.0f;
    msg.min_range       = LOW_MIN_MM / 1000.0f;
    msg.max_range       = LOW_MAX_MM / 1000.0f;
    msg.range           = range_m;
    pub_->publish(msg);
}

void Dvc_LowFreq::Shutdown()
{
    pub_.reset();
    node_.reset();
}

// ============================================================
//  传感器阵列
// ============================================================

class Dvc_LowFreq_Array {
public:
    void Init(linkx_t* linkx_master, rclcpp::Node::SharedPtr node);
    void CAN_RxCpltCallback(uint8_t channel, const can_msg_t& rx_msg);
    void TIM_Alive_PeriodElapsedCallback();
    void Shutdown();

private:
    linkx_t* p_linkx_;   // 仅保存以满足接口；本节点对传感器只读，不发送任何命令

    static constexpr int LOW_FREQ_NUM = 4;
    Dvc_LowFreq sensors_[LOW_FREQ_NUM];

    // 地址 → 传感器指针，O(1) 分发
    std::unordered_map<uint8_t, Dvc_LowFreq*> addr_map_;
};

void Dvc_LowFreq_Array::Init(linkx_t* linkx_master,
                               rclcpp::Node::SharedPtr node)
{
    p_linkx_ = linkx_master;

    // 按图片配置：地址 → 话题（Byte0 地址字节 & 话题名）
    struct Entry { uint8_t addr; const char* topic; };
    constexpr Entry table[] = {
        {0x04, "/low/front/range"},
        {0x03, "/low/left/range" },
        {0x02, "/low/right/range"},
        {0x01, "/low/back/range" },
    };

    for (int i = 0; i < LOW_FREQ_NUM; ++i) {
        sensors_[i].Init(node, table[i].addr, table[i].topic);
        addr_map_[table[i].addr] = &sensors_[i];
    }
}

void Dvc_LowFreq_Array::CAN_RxCpltCallback(uint8_t channel,
                                             const can_msg_t& rx_msg)
{
    // 只处理低频传感器数据帧
    if (channel != LOW_FREQ_CH || rx_msg.id != LOW_DATA_CAN_ID) return;
    if (rx_msg.dlen < 1) return;

    // Byte0 = 设备地址
    uint8_t addr = rx_msg.data[0];
    auto it = addr_map_.find(addr);
    if (it == addr_map_.end()) return;  // 未知地址，忽略

    it->second->OnFrame(rx_msg.data, rx_msg.dlen);
}

void Dvc_LowFreq_Array::TIM_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < LOW_FREQ_NUM; ++i)
        sensors_[i].TIM_Alive_PeriodElapsedCallback();
}

void Dvc_LowFreq_Array::Shutdown()
{
    addr_map_.clear();
    for (int i = 0; i < LOW_FREQ_NUM; ++i)
        sensors_[i].Shutdown();
    p_linkx_ = nullptr;
}

// ============================================================
//  任务接口（供 can_link_test.cpp 通过 extern 调用）
// ============================================================

static Dvc_LowFreq_Array low_freq_array;

void task_init_low(rclcpp::Node::SharedPtr node, linkx_t* linkx_master) {
    low_freq_array.Init(linkx_master, node);
}

void task_shutdown_low() {
    low_freq_array.Shutdown();
}

void Pump_CAN_Receive_Low(uint8_t channel, const can_msg_t& rx_msg) {
    low_freq_array.CAN_RxCpltCallback(channel, rx_msg);
}

void Tick_Alive_Check_Low() {
    low_freq_array.TIM_Alive_PeriodElapsedCallback();
}

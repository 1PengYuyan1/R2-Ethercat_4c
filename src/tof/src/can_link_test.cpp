// can_link_test.cpp
// 上位机统一传感器节点
//   - 通过 EtherCAT + DM-LinKX-4c 读取 CAN 报文
//   - 高频传感器 (TFmini-S, 12路): /high/...  (2 块 EtherCAT 模块)
//   - 低频传感器 (4路):             /low/...   (可选第 3 块模块)
//
// 用法:
//   ./can_link_test [ifname] [can_baud] [slave_id_high_front] [slave_id_high_up] [slave_id_low]
//   默认:  enp86s0  1M  2  1  0

#include "ecat_manager.h"
#include "linkx.h"
#include "linkx4c_handler.h"
#include "rclcpp/rclcpp.hpp"

#include <csignal>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <memory>
#include <thread>
#include <map>
#include <queue>
#include <mutex>
#include <condition_variable>

// ── 高频传感器接口（dvc_tfmini_s.cpp）──────────────────────
extern void task_init              (rclcpp::Node::SharedPtr node);
extern void task_shutdown          ();
extern void Pump_CAN_Receive       (int module_alias, uint8_t channel, const can_msg_t& rx_msg);
extern void Tick_Alive_Check       ();

// ── 低频传感器接口（dvc_low_freq.cpp）───────────────────────
extern void task_init_low          (rclcpp::Node::SharedPtr node, linkx_t* linkx_master);
extern void task_shutdown_low      ();
extern void Pump_CAN_Receive_Low   (uint8_t channel, const can_msg_t& rx_msg);
extern void Tick_Alive_Check_Low   ();

static volatile bool g_running = true;
static void signal_handler(int) { g_running = false; }

static bool recover_ecat_master(ecat_master_t* master,
                                const rclcpp::Logger& logger)
{
    RCLCPP_WARN(logger,
                "Local Ethercat-R2 ecat_manager has no dedicated recovery API; retrying OP transition");
    return ecat_master_bring_online(master);
}

// ============================================================
//  线程安全 CAN 帧队列
//  生产者：EtherCAT 主循环（全速轮询，push 不阻塞）
//  消费者：独立线程（调用 Pump_CAN_Receive / Pump_CAN_Receive_Low）
//
//  dispatch 位掩码:
//    bit 0 = 分发给高频 handler (Pump_CAN_Receive)
//    bit 1 = 分发给低频 handler (Pump_CAN_Receive_Low)
// ============================================================
class CanFrameQueue {
public:
    struct Entry {
        int       module_alias;
        uint8_t   channel;
        can_msg_t msg;
        uint8_t   dispatch;
    };

    static constexpr size_t MAX_SIZE = 4096;

    void push(int module_alias, uint8_t ch, const can_msg_t& msg, uint8_t dispatch) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (q_.size() >= MAX_SIZE) {
                q_.pop();
                drop_cnt_++;
            }
            q_.push({module_alias, ch, msg, dispatch});
        }
        cv_.notify_one();
    }

    // 阻塞等待，直到有帧或 stop
    bool pop(Entry& out) {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this] { return !q_.empty() || stop_; });
        if (q_.empty()) return false;
        out = q_.front();
        q_.pop();
        return true;
    }

    void request_stop() {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            stop_ = true;
        }
        cv_.notify_all();
    }

    size_t   size()       const { std::lock_guard<std::mutex> lk(mtx_); return q_.size(); }
    uint64_t drop_count() const { std::lock_guard<std::mutex> lk(mtx_); return drop_cnt_; }

private:
    mutable std::mutex      mtx_;
    std::condition_variable cv_;
    std::queue<Entry>       q_;
    uint64_t                drop_cnt_ = 0;
    bool                    stop_     = false;
};

// CAN 控制器时钟 80MHz，支持经典 CAN 1M / 500k / 250k，以及 CAN-FD 1m-5m
static bool configure_can_baudrate(linkx_t* linkx, uint8_t ch,
                                    const std::string& baud_str,
                                    const rclcpp::Logger& logger)
{
    uint8_t fd_en = 0;
    uint8_t n_pre, n_seg1, n_seg2, n_sjw;
    uint8_t d_pre, d_seg1, d_seg2, d_sjw;
    const char* mode = "classic CAN";

    if (baud_str == "1M") {
        n_pre = 2; n_seg1 = 29; n_seg2 = 10; n_sjw = 1;
        d_pre = 2; d_seg1 = 7; d_seg2 = 2; d_sjw = 1;
    } else if (baud_str == "500k") {
        n_pre = 4; n_seg1 = 29; n_seg2 = 10; n_sjw = 1;
        d_pre = 2; d_seg1 = 7; d_seg2 = 2; d_sjw = 1;
    } else if (baud_str == "250k") {
        n_pre = 8; n_seg1 = 29; n_seg2 = 10; n_sjw = 1;
        d_pre = 2; d_seg1 = 7; d_seg2 = 2; d_sjw = 1;
    } else if (baud_str == "1m-5m") {
        fd_en = 1;
        mode = "CAN-FD";
        n_pre = 2; n_seg1 = 31; n_seg2 = 8; n_sjw = 8;
        d_pre = 1; d_seg1 = 12; d_seg2 = 3; d_sjw = 3;
    } else {
        RCLCPP_WARN(logger, "Unknown baudrate '%s', falling back to classic CAN 1M", baud_str.c_str());
        n_pre = 2; n_seg1 = 29; n_seg2 = 10; n_sjw = 1;
        d_pre = 2; d_seg1 = 7; d_seg2 = 2; d_sjw = 1;
    }

    RCLCPP_INFO(logger,
                "CAN CH%d mode=%s baudrate=%s nominal(pre=%d seg1=%d seg2=%d sjw=%d) data(pre=%d seg1=%d seg2=%d sjw=%d)",
                ch, mode, baud_str.c_str(),
                n_pre, n_seg1, n_seg2, n_sjw,
                d_pre, d_seg1, d_seg2, d_sjw);

    return linkx_set_can_baudrate(linkx, ch,
                                   fd_en,
                                   n_pre, n_seg1, n_seg2, n_sjw,
                                   d_pre, d_seg1, d_seg2, d_sjw);
}

// 在 SAFE_OP 阶段完成从站 SDO 配置（wakeup + 波特率），必须在 ecat_master_bring_online 之前调用
static bool init_linkx_slave_safeop(linkx_t* linkx, uint32_t slave_id,
                                     ecat_master_t* master,
                                     const std::string& baud_str,
                                     const rclcpp::Logger& logger)
{
    if (slave_id < 1 || slave_id > static_cast<uint32_t>(master->slave_count)) {
        RCLCPP_ERROR(logger, "[slave %u] Not found on bus (found %d slave(s))",
                     slave_id, master->slave_count);
        return false;
    }

    std::memset(linkx, 0, sizeof(*linkx));
    linkx_init(linkx, slave_id, &master->ctx);

    if (!linkx_hw_wakeup(linkx))
        RCLCPP_WARN(logger, "[slave %u] linkx_hw_wakeup() failed, continuing", slave_id);

    for (int ch = 0; ch < LINKX_CAN_CHANNEL_NUM; ch++) {
        auto uch = static_cast<uint8_t>(ch);
        if (!linkx_switch_can_channel(linkx, uch, true))
            RCLCPP_WARN(logger, "[slave %u] Failed to enable CAN CH%d", slave_id, ch);
        else
            RCLCPP_INFO(logger, "[slave %u] CAN CH%d enabled", slave_id, ch);

        if (!configure_can_baudrate(linkx, uch, baud_str, logger))
            RCLCPP_WARN(logger, "[slave %u] Baudrate config failed for CAN CH%d", slave_id, ch);
    }
    return true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("upper_computer_radar_node");
    const auto& logger = node->get_logger();

    // --- 1. 解析启动参数 ---
    std::string ifname           = (argc > 1) ? argv[1] : "enp86s0";
    std::string can_baud         = (argc > 2) ? argv[2] : "1M";
    uint32_t slave_high_front    = (argc > 3) ? static_cast<uint32_t>(std::stoul(argv[3])) : 2;
    uint32_t slave_high_up_down  = (argc > 4) ? static_cast<uint32_t>(std::stoul(argv[4])) : 1;
    uint32_t slave_low           = (argc > 5) ? static_cast<uint32_t>(std::stoul(argv[5])) : 0;

    RCLCPP_INFO(logger,
                "Interface: %s  CAN: %s  slave_high_front=%u  slave_high_up_down=%u  slave_low=%u",
                ifname.c_str(), can_baud.c_str(),
                slave_high_front, slave_high_up_down, slave_low);

    // --- 2. EtherCAT 主站初始化 ---
    ecat_master_t master;
    std::memset(&master, 0, sizeof(master));

    if (!ecat_master_init(&master, ifname.c_str())) {
        RCLCPP_FATAL(logger, "ecat_master_init() failed on %s", ifname.c_str());
        rclcpp::shutdown();
        return 1;
    }

    // --- 3. SAFE_OP 下完成从站 SDO 配置 ---
    linkx_t linkx_high_front, linkx_high_up_down, linkx_low;

    bool high_front_ok = false;
    if (slave_high_front == 0) {
        RCLCPP_INFO(logger, "[slave_high_front=0] Front/left/right/back high-frequency slave disabled");
    } else {
        high_front_ok = init_linkx_slave_safeop(&linkx_high_front, slave_high_front, &master, can_baud, logger);
    }

    bool high_up_down_ok = false;
    if (slave_high_up_down == 0) {
        RCLCPP_INFO(logger, "[slave_high_up_down=0] Up/down high-frequency slave disabled");
    } else if (slave_high_up_down == slave_high_front && high_front_ok) {
        RCLCPP_INFO(logger, "[slave_high_up_down=%u] Sharing front/rear high-frequency slave",
                    slave_high_up_down);
        high_up_down_ok = true;
    } else {
        high_up_down_ok = init_linkx_slave_safeop(&linkx_high_up_down, slave_high_up_down, &master, can_baud, logger);
    }

    bool low_ok  = false;
    if (slave_low == 0) {
        RCLCPP_INFO(logger, "[slave_low=0] Low-frequency slave disabled");
    } else {
        low_ok = init_linkx_slave_safeop(&linkx_low, slave_low, &master, can_baud, logger);
    }

    if (!high_front_ok && !high_up_down_ok && !low_ok) {
        RCLCPP_FATAL(logger, "All LinkX slaves failed to initialize");
        rclcpp::shutdown();
        return 1;
    }

    // --- 4. 切换所有从站到 OP 状态 ---
    if (!ecat_master_bring_online(&master)) {
        RCLCPP_FATAL(logger, "ecat_master_bring_online() failed");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(logger, "EtherCAT online: %d slave(s), expected WKC=%d",
                master.slave_count, master.expected_wkc);

    // --- 5. 传感器阵列初始化 ---
    if (high_front_ok || high_up_down_ok) task_init(node);
    if (low_ok)  task_init_low(node, &linkx_low);

    // --- 6. 信号处理 ---
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // --- 7. ROS 2 回调线程 ---
    std::thread spin_thread([&node]() { rclcpp::spin(node); });

    // --- 8. CAN 帧队列 + 消费者线程 ---
    //   主循环只做 push（微秒级），消费者线程负责解析和 ROS 发布
    //   彻底解耦 EtherCAT 轮询速率与传感器处理耗时
    CanFrameQueue can_queue;

    std::thread consumer_thread([&]() {
        CanFrameQueue::Entry e;
        while (can_queue.pop(e)) {
            if (e.dispatch & 0x01) Pump_CAN_Receive    (e.module_alias, e.channel, e.msg);
            if (e.dispatch & 0x02) Pump_CAN_Receive_Low(e.channel, e.msg);
        }
    });

    // --- 9. 主循环：全速轮询 EtherCAT，帧入队即返回 ---
    RCLCPP_INFO(logger, "Entering main loop (max rate, queue dispatch)");

    using Clock = std::chrono::steady_clock;
    auto t_last_alive = Clock::now();
    auto t_last_stats = Clock::now();
    uint64_t loop_cnt = 0;
    uint64_t cnt_high_front[LINKX_CAN_CHANNEL_NUM] = {0};
    uint64_t cnt_high_up_down[LINKX_CAN_CHANNEL_NUM] = {0};
    uint64_t cnt_low [LINKX_CAN_CHANNEL_NUM] = {0};
    int sync_err = 0;
    std::map<uint32_t, uint64_t> id_dlen_cnt;

    // 诊断探针：绕过去重，直接记录每通道原始 PDO 曾出现过的 CAN ID
    static bool raw_id_seen_high_front[LINKX_CAN_CHANNEL_NUM][2048] = {{false}};
    static bool raw_id_seen_high_up_down[LINKX_CAN_CHANNEL_NUM][2048] = {{false}};
    static bool raw_id_seen_low [LINKX_CAN_CHANNEL_NUM][2048] = {{false}};

    while (g_running && rclcpp::ok()) {
        loop_cnt++;

        // EtherCAT 过程数据交换
        int wkc = ecat_master_sync(&master);
        if (wkc < master.expected_wkc) {
            if (++sync_err == 100) {
                RCLCPP_WARN(logger, "Persistent WKC error (got %d, expected %d), attempting recovery...",
                            wkc, master.expected_wkc);
                if (recover_ecat_master(&master, logger)) {
                    RCLCPP_INFO(logger, "EtherCAT recovery successful");
                    sync_err = 0;
                } else {
                    RCLCPP_ERROR(logger, "EtherCAT recovery failed, will retry...");
                    sync_err = 0;
                }
            }
        } else {
            if (sync_err >= 10)
                RCLCPP_INFO(logger, "EtherCAT link restored after %d errors", sync_err);
            sync_err = 0;
        }

        // ── 高频从站 ────────────────────────────────────────────
        if (high_front_ok) {
            linkx_send_pdos(&linkx_high_front);
            linkx_recv_pdos(&linkx_high_front);

            for (int ch = 0; ch < LINKX_CAN_CHANNEL_NUM; ch++) {
                can_tx_pdo_t* raw = linkx_recv_can(&linkx_high_front, static_cast<uint8_t>(ch));
                if (raw) raw_id_seen_high_front[ch][raw->can_id & 0x7FF] = true;

                can_msg_t msg;
                while (linkx_quick_recv(&linkx_high_front, static_cast<uint8_t>(ch), &msg)) {
                    if (msg.id == 0 && msg.dlen == 0) continue;
                    cnt_high_front[ch]++;
                    id_dlen_cnt[((uint32_t)ch<<24)|((uint32_t)msg.dlen<<16)|(msg.id&0xFFFF)]++;
                    can_queue.push(0, static_cast<uint8_t>(ch), msg, 0x01);
                }
            }
        }

        if (high_up_down_ok && slave_high_up_down != slave_high_front) {
            linkx_send_pdos(&linkx_high_up_down);
            linkx_recv_pdos(&linkx_high_up_down);

            for (int ch = 0; ch < LINKX_CAN_CHANNEL_NUM; ch++) {
                can_tx_pdo_t* raw = linkx_recv_can(&linkx_high_up_down, static_cast<uint8_t>(ch));
                if (raw) raw_id_seen_high_up_down[ch][raw->can_id & 0x7FF] = true;

                can_msg_t msg;
                while (linkx_quick_recv(&linkx_high_up_down, static_cast<uint8_t>(ch), &msg)) {
                    if (msg.id == 0 && msg.dlen == 0) continue;
                    cnt_high_up_down[ch]++;
                    can_queue.push(1, static_cast<uint8_t>(ch), msg, 0x01);
                }
            }
        }

        // ── 低频从站（独立物理从站时才单独处理）────────────────
        if (low_ok &&
            (!high_front_ok || slave_high_front != slave_low) &&
            (!high_up_down_ok || slave_high_up_down != slave_low)) {
            linkx_send_pdos(&linkx_low);
            linkx_recv_pdos(&linkx_low);

            for (int ch = 0; ch < LINKX_CAN_CHANNEL_NUM; ch++) {
                // 原始探针
                can_tx_pdo_t* raw = linkx_recv_can(&linkx_low, static_cast<uint8_t>(ch));
                if (raw) raw_id_seen_low[ch][raw->can_id & 0x7FF] = true;

                can_msg_t msg;
                while (linkx_quick_recv(&linkx_low, static_cast<uint8_t>(ch), &msg)) {
                    if (msg.id == 0 && msg.dlen == 0) continue;
                    cnt_low[ch]++;
                    id_dlen_cnt[((uint32_t)ch<<24)|((uint32_t)msg.dlen<<16)|(msg.id&0xFFFF)]++;
                    can_queue.push(-1, static_cast<uint8_t>(ch), msg, 0x02);
                }
            }
        }

        auto now = Clock::now();

        // 每 1 秒检测传感器在线状态
        if (now - t_last_alive >= std::chrono::seconds(1)) {
            if (high_front_ok || high_up_down_ok) Tick_Alive_Check();
            if (low_ok)  Tick_Alive_Check_Low();
            t_last_alive = now;
        }

        // 每 5 秒打印统计
        if (now - t_last_stats >= std::chrono::seconds(5)) {
            RCLCPP_INFO(logger, "STATS loop=%lu  queue_size=%zu  queue_drops=%lu",
                        loop_cnt, can_queue.size(), can_queue.drop_count());
            if (high_front_ok)
                RCLCPP_INFO(logger,
                    "  HIGH_FRONT slave=%u  CH0=%lu CH1=%lu CH2=%lu CH3=%lu",
                    slave_high_front,
                    cnt_high_front[0], cnt_high_front[1], cnt_high_front[2], cnt_high_front[3]);
            if (high_up_down_ok && slave_high_up_down != slave_high_front)
                RCLCPP_INFO(logger,
                    "  HIGH_UP_DOWN slave=%u  CH0=%lu CH1=%lu CH2=%lu CH3=%lu",
                    slave_high_up_down,
                    cnt_high_up_down[0], cnt_high_up_down[1], cnt_high_up_down[2], cnt_high_up_down[3]);
            if (low_ok)
                RCLCPP_INFO(logger,
                    "  LOW  slave=%u  CH0=%lu CH1=%lu CH2=%lu CH3=%lu",
                    slave_low, cnt_low[0], cnt_low[1], cnt_low[2], cnt_low[3]);

            for (const auto& kv : id_dlen_cnt) {
                RCLCPP_INFO(logger, "    CH%u id=0x%03X dlen=%u : %lu",
                            (unsigned)((kv.first >> 24) & 0xFF),
                            (unsigned)(kv.first & 0xFFFF),
                            (unsigned)((kv.first >> 16) & 0xFF),
                            kv.second);
            }

            auto print_raw_ids = [&logger](const char* tag, bool seen[][2048]) {
                for (int ch = 0; ch < LINKX_CAN_CHANNEL_NUM; ch++) {
                    char ids[512]; int o = 0;
                    for (int id = 0; id < 2048; id++)
                        if (seen[ch][id])
                            o += snprintf(ids + o, sizeof(ids) - o, " 0x%03X", id);
                    RCLCPP_INFO(logger, "    RAW %s CH%d ever-seen IDs:%s",
                                tag, ch, o ? ids : " (none)");
                }
            };
            if (high_front_ok) print_raw_ids("HIGH_FRONT", raw_id_seen_high_front);
            if (high_up_down_ok && slave_high_up_down != slave_high_front)
                print_raw_ids("HIGH_UP_DOWN", raw_id_seen_high_up_down);
            if (low_ok)  print_raw_ids("LOW ", raw_id_seen_low);

            t_last_stats = now;
        }
    }

    // --- 10. 清理 ---
    can_queue.request_stop();
    consumer_thread.join();

    if (high_front_ok || high_up_down_ok) task_shutdown();
    if (low_ok)  task_shutdown_low();
    if (high_front_ok) linkx_stop(&linkx_high_front);
    if (high_up_down_ok && slave_high_up_down != slave_high_front) linkx_stop(&linkx_high_up_down);
    if (low_ok)  linkx_stop(&linkx_low);

    RCLCPP_INFO(logger, "Shutdown complete (total loops=%lu)", loop_cnt);
    rclcpp::shutdown();
    spin_thread.join();
    node.reset();

    return 0;
}

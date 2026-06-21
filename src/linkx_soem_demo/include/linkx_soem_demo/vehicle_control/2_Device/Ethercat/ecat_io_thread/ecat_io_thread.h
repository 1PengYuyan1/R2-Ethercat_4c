#ifndef LINKX_SOEM_DEMO_ECAT_IO_THREAD_H
#define LINKX_SOEM_DEMO_ECAT_IO_THREAD_H

// ecat_io_thread —— EtherCAT 全速 IO 线程（生产者）。
//
// 背景：SOEM 的 ecat_master_sync 是收发一体的周期交换，且整车 slave1 与 ToF
// slave2 共用同一个 master/网卡，无法按从站拆分。本模块把整条 EtherCAT 周期
// 交换搬进一个 RT 优先级线程，全速轮询：
//   ecat_master_sync → linkx_recv_pdos → 去重读取所有 CAN 帧并入队
//   → linkx_send_pdos（把控制线程经 linkx_send_can 入队的 TX 帧刷到网线）
//
// 控制线程作为消费者：DrainRx() 取走原始帧自行分发，并照常调用
// linkx_send_can 入队电机/复位帧（由 linkx 内部 tx_lock 保证线程安全）。
//
// 参考 src/extra_tests/tfmini_s_can_read_main.cpp 的 io_thread_func /
// quick_recv_isolated / RxSnapshot 去重设计。

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "ecat_manager.h"
#include "linkx.h"
#include "linkx4c_handler.h"

// 一条由 IO 线程捕获的原始 CAN 帧，连同其归属从站/通道与接收时刻。
struct EcatRawCanMsg
{
    uint32_t slave_id = 0;
    uint8_t channel = 0;
    can_msg_t msg {};
    std::chrono::steady_clock::time_point recv_time {};
};

class EcatIoThread
{
public:
    static constexpr size_t kMaxLinkx = 2;

    // 在每轮 send_pdos 之前/之后于 IO 线程上下文调用。
    //   PreSendHook  —— send_pdos 之前：入队需在本周期刷出的帧（如 ToF 复位帧）
    //   PostSendHook —— send_pdos 之后：清理只应占一个周期的帧（如 ToF 复位帧）
    using PreSendHook = std::function<void()>;
    using PostSendHook = std::function<void()>;

    // 绑定 EtherCAT 主站。须在 Start() 之前、且 master 已 bring_online 后调用。
    void Configure(ecat_master_t *master);

    // 登记一片需要轮询的 LinkX 设备。enabled=false 的设备会被跳过（如 ToF
    // slave2 不存在时）。必须在 Start() 之前调用，最多 kMaxLinkx 片。
    void AddLinkx(linkx_t *linkx, uint32_t slave_id, bool enabled);

    // 设置 send_pdos 之前/之后的 IO 线程钩子（线程安全要求由钩子自身负责）。
    void SetPreSendHook(PreSendHook hook);
    void SetPostSendHook(PostSendHook hook);

    void Start();  // 拉起 RT 优先级 IO 线程
    void Stop();   // 通知退出并 join（幂等）

    // 同步模式用：由控制线程直接调用，不另起线程。
    //   PollReceiveOnce() —— 一次 sync + recv_pdos + 去重读取入队（收一拍）
    //   FlushSendOnce()   —— pre 钩子 + send_pdos + post 钩子（发一拍）
    // 与 IO 线程 Run() 内部完全相同的逻辑，便于在同步/异步两种模式间切换。
    void PollReceiveOnce();
    void FlushSendOnce();

    // 消费者（控制线程）调用：取走 IO 线程已捕获的全部原始帧，追加到 out。
    void DrainRx(std::vector<EcatRawCanMsg> &out);

    // 已完成的 EtherCAT 轮询周期数，用于实测轮询频率。
    uint64_t PollCount() const { return io_loops_.load(std::memory_order_relaxed); }

private:
    struct RxSnapshot
    {
        bool valid = false;
        uint32_t can_id = 0;
        uint16_t params_raw = 0;
        uint64_t timestamp = 0;
        uint8_t dlen = 0;
        uint8_t data[LINKX_CAN_MAX_DATA_BYTES] = {};
    };

    struct LinkxBinding
    {
        linkx_t *linkx = nullptr;
        uint32_t slave_id = 0;
        bool enabled = false;
    };

    void Run();
    bool QuickRecvIsolated(linkx_t *linkx, size_t linkx_index, uint8_t ch, can_msg_t *out);

    ecat_master_t *master_ = nullptr;
    std::array<LinkxBinding, kMaxLinkx> linkx_ {};
    size_t linkx_count_ = 0;
    std::array<std::array<RxSnapshot, LINKX_CAN_CHANNEL_NUM>, kMaxLinkx> snapshots_ {};

    PreSendHook pre_send_hook_;
    PostSendHook post_send_hook_;

    std::queue<EcatRawCanMsg> rx_queue_;
    std::mutex rx_mutex_;
    std::atomic<uint64_t> io_loops_ {0};
    std::atomic<bool> running_ {false};
    std::thread thread_;
};

#endif  // LINKX_SOEM_DEMO_ECAT_IO_THREAD_H

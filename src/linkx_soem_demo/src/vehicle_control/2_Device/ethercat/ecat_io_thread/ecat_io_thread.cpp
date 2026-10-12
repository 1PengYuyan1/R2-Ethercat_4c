#include "ecat_io_thread.h"

#include <cstdio>
#include <cstring>
#include <pthread.h>
#include <sched.h>

namespace
{
// 与 task.cpp 原 Pack_Can_Params 一致：把 PDO 参数压成可比较的 16 位标识。
uint16_t PackCanParams(const can_pdo_param_t &params)
{
    uint16_t raw = 0;
    raw |= static_cast<uint16_t>(params.ext & 0x01U);
    raw |= static_cast<uint16_t>((params.rtr & 0x01U) << 1);
    raw |= static_cast<uint16_t>((params.canfd & 0x01U) << 2);
    raw |= static_cast<uint16_t>((params.brs & 0x01U) << 3);
    raw |= static_cast<uint16_t>(params.dlen << 8);
    return raw;
}
}  // namespace

void EcatIoThread::Configure(ecat_master_t *master)
{
    master_ = master;
}

void EcatIoThread::AddLinkx(linkx_t *linkx, uint32_t slave_id, bool enabled)
{
    if (linkx_count_ >= kMaxLinkx)
        return;
    linkx_[linkx_count_] = LinkxBinding{linkx, slave_id, enabled};
    ++linkx_count_;
}

void EcatIoThread::SetPreSendHook(PreSendHook hook)
{
    pre_send_hook_ = std::move(hook);
}

void EcatIoThread::SetPostSendHook(PostSendHook hook)
{
    post_send_hook_ = std::move(hook);
}

// 去重读取单个通道的一帧。LinkX 的 RX PDO 每通道只有一个槽，必须按
// (can_id, params, timestamp, payload) 与上次快照比对，避免重复读到同一帧。
bool EcatIoThread::QuickRecvIsolated(linkx_t *linkx, size_t linkx_index, uint8_t ch, can_msg_t *out)
{
    if (linkx == nullptr || out == nullptr ||
        ch >= LINKX_CAN_CHANNEL_NUM || linkx_index >= kMaxLinkx)
    {
        return false;
    }

    can_tx_pdo_t *rx_pdo = linkx_recv_can(linkx, ch);
    if (rx_pdo == nullptr)
        return false;

    uint8_t pdo_dlen = rx_pdo->params.dlen;
    if (pdo_dlen > LINKX_CAN_MAX_DATA_BYTES)
        pdo_dlen = LINKX_CAN_MAX_DATA_BYTES;

    RxSnapshot &snap = snapshots_[linkx_index][ch];
    if (snap.valid &&
        snap.can_id == rx_pdo->can_id &&
        snap.params_raw == PackCanParams(rx_pdo->params) &&
        snap.timestamp == rx_pdo->timestamp &&
        snap.dlen == pdo_dlen &&
        (pdo_dlen == 0U || std::memcmp(snap.data, rx_pdo->data_u32, pdo_dlen) == 0))
    {
        return false;  // 与上次完全相同：不是新帧
    }

    out->id = rx_pdo->can_id;
    out->timestamp = rx_pdo->timestamp;
    out->canfd = rx_pdo->params.canfd != 0;
    out->brs = rx_pdo->params.brs != 0;
    out->ext = rx_pdo->params.ext != 0;
    out->rtr = rx_pdo->params.rtr != 0;
    out->dlen = (pdo_dlen > sizeof(out->data)) ? static_cast<uint8_t>(sizeof(out->data)) : pdo_dlen;
    std::memset(out->data, 0, sizeof(out->data));
    if (out->dlen > 0U)
        std::memcpy(out->data, rx_pdo->data_u32, out->dlen);

    snap.valid = true;
    snap.can_id = rx_pdo->can_id;
    snap.params_raw = PackCanParams(rx_pdo->params);
    snap.timestamp = rx_pdo->timestamp;
    snap.dlen = pdo_dlen;
    std::memset(snap.data, 0, sizeof(snap.data));
    if (pdo_dlen > 0U)
        std::memcpy(snap.data, rx_pdo->data_u32, pdo_dlen);

    linkx->can_stats[ch].rx_frames++;
    linkx->can_stats[ch].rx_bytes += pdo_dlen;
    return true;
}

void EcatIoThread::Run()
{
    // RT 优先级：把 IO 线程提到 SCHED_FIFO 最高级。ecat_master_sync 阻塞在
    // 网络往返上（receive_processdata 带超时），线程在该 syscall 内让出 CPU，
    // 因此不会忙等饿死控制线程；自定速在网络速率（数 kHz）。
    sched_param sp {};
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        std::fprintf(stderr, "[ECAT-IO] WARNING: failed to set RT priority (run as root?)\n");

    // 仅以 running_ 为退出条件，不绑定 master->is_running：收到停机请求后控制
    // 线程会退出主循环并进入失能阶段，此时 IO 线程仍需继续 sync + send_pdos
    // 把失能帧刷到总线，直到外部显式 Stop()。
    while (running_.load(std::memory_order_relaxed) && master_ != nullptr)
    {
        PollReceiveOnce();
        FlushSendOnce();
    }
}

// 收一拍：sync + recv_pdos + 去重读取所有通道入队。
void EcatIoThread::PollReceiveOnce()
{
    if (master_ == nullptr)
        return;

    // 进站：周期交换 + 取回输入 PDO。
    ecat_master_sync(master_);
    io_loops_.fetch_add(1, std::memory_order_relaxed);

    for (size_t mi = 0; mi < linkx_count_; ++mi)
    {
        if (linkx_[mi].enabled)
            linkx_recv_pdos(linkx_[mi].linkx);
    }

    // 抽干所有通道的去重接收，攒成一批后一次性入队。
    std::vector<EcatRawCanMsg> batch;
    can_msg_t msg {};
    for (size_t mi = 0; mi < linkx_count_; ++mi)
    {
        if (!linkx_[mi].enabled)
            continue;
        linkx_t *linkx = linkx_[mi].linkx;
        for (uint8_t ch = 0; ch < LINKX_CAN_CHANNEL_NUM; ++ch)
        {
            const auto recv_time = std::chrono::steady_clock::now();
            while (QuickRecvIsolated(linkx, mi, ch, &msg))
            {
                EcatRawCanMsg raw;
                raw.slave_id = linkx_[mi].slave_id;
                raw.channel = ch;
                raw.msg = msg;
                raw.recv_time = recv_time;
                batch.push_back(raw);
            }
        }
    }

    if (!batch.empty())
    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        for (const auto &raw : batch)
            rx_queue_.push(raw);
    }
}

// 发一拍：pre 钩子 + send_pdos + post 钩子。
void EcatIoThread::FlushSendOnce()
{
    // send_pdos 之前的钩子：把本周期需要刷出的帧入队（如 ToF 复位帧）。
    if (pre_send_hook_)
        pre_send_hook_();

    // 出站：把控制线程经 linkx_send_can 入队的 TX 帧刷到从站输出。
    for (size_t mi = 0; mi < linkx_count_; ++mi)
    {
        if (linkx_[mi].enabled)
            linkx_send_pdos(linkx_[mi].linkx);
    }

    // send_pdos 之后的钩子：清理一次性复位帧，保证只发一个 EtherCAT 周期。
    if (post_send_hook_)
        post_send_hook_();
}

void EcatIoThread::Start()
{
    if (running_.exchange(true))
        return;  // 已在运行
    thread_ = std::thread(&EcatIoThread::Run, this);
}

void EcatIoThread::Stop()
{
    if (!running_.exchange(false))
    {
        if (thread_.joinable())
            thread_.join();
        return;
    }
    if (thread_.joinable())
        thread_.join();
}

void EcatIoThread::DrainRx(std::vector<EcatRawCanMsg> &out)
{
    std::queue<EcatRawCanMsg> local;
    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        std::swap(local, rx_queue_);
    }
    while (!local.empty())
    {
        out.push_back(local.front());
        local.pop();
    }
}

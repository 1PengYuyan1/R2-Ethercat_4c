#include "can_terminal_printer.h"

#include <cstdio>
#include <cstdint>

namespace
{
constexpr int kChannelCount = 4;
}

void can_terminal::PrintStats(linkx_t *linkx)
{
    if (linkx == nullptr)
        return;

    std::printf("\n[CAN-TOTAL]\n");

    uint64_t sum_tx_req_f = 0;
    uint64_t sum_tx_req_b = 0;
    uint64_t sum_tx_sent_f = 0;
    uint64_t sum_tx_sent_b = 0;
    uint64_t sum_tx_drop_f = 0;
    uint64_t sum_rx_f = 0;
    uint64_t sum_rx_b = 0;

    for (int ch = 0; ch < kChannelCount; ++ch)
    {
        const auto &s = linkx->can_stats[ch];
        const uint16_t q_fill = linkx->tx_queues[ch].size;
        const uint64_t loss_f = (s.tx_frames > s.rx_frames) ? (s.tx_frames - s.rx_frames) : 0;
        const double loss_rt =
            (s.tx_frames > 0) ? (100.0 * static_cast<double>(loss_f) /
                                 static_cast<double>(s.tx_frames)) : 0.0;

        std::printf("  CH%d TX_REQ=%llu (%llu B) TX_SENT=%llu (%llu B) TX_DROP=%llu Q=%u"
                    " RX=%llu (%llu B) LOSS=%llu (%.2f%%)\n",
                    ch,
                    static_cast<unsigned long long>(s.tx_enqueued_frames),
                    static_cast<unsigned long long>(s.tx_enqueued_bytes),
                    static_cast<unsigned long long>(s.tx_frames),
                    static_cast<unsigned long long>(s.tx_bytes),
                    static_cast<unsigned long long>(s.tx_dropped_frames),
                    static_cast<unsigned>(q_fill),
                    static_cast<unsigned long long>(s.rx_frames),
                    static_cast<unsigned long long>(s.rx_bytes),
                    static_cast<unsigned long long>(loss_f),
                    loss_rt);

        sum_tx_req_f += s.tx_enqueued_frames;
        sum_tx_req_b += s.tx_enqueued_bytes;
        sum_tx_sent_f += s.tx_frames;
        sum_tx_sent_b += s.tx_bytes;
        sum_tx_drop_f += s.tx_dropped_frames;
        sum_rx_f += s.rx_frames;
        sum_rx_b += s.rx_bytes;
    }

    const uint64_t sum_loss_f =
        (sum_tx_sent_f > sum_rx_f) ? (sum_tx_sent_f - sum_rx_f) : 0;
    const double sum_loss_rt =
        (sum_tx_sent_f > 0) ? (100.0 * static_cast<double>(sum_loss_f) /
                               static_cast<double>(sum_tx_sent_f)) : 0.0;

    std::printf("  ALL TX_REQ=%llu (%llu B) TX_SENT=%llu (%llu B) TX_DROP=%llu"
                " RX=%llu (%llu B) LOSS=%llu (%.2f%%)\n",
                static_cast<unsigned long long>(sum_tx_req_f),
                static_cast<unsigned long long>(sum_tx_req_b),
                static_cast<unsigned long long>(sum_tx_sent_f),
                static_cast<unsigned long long>(sum_tx_sent_b),
                static_cast<unsigned long long>(sum_tx_drop_f),
                static_cast<unsigned long long>(sum_rx_f),
                static_cast<unsigned long long>(sum_rx_b),
                static_cast<unsigned long long>(sum_loss_f),
                sum_loss_rt);
}

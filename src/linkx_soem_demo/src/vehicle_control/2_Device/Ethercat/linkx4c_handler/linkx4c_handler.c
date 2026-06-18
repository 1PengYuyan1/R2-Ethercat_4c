
#include "linkx4c_handler.h"

#include "stdio.h"
#include "string.h"

#include "soem/osal.h"    // 提供基础操作系统抽象类型
#include "soem/ec_type.h" // 提供 uint16, uint8, ecx_contextt 等类型定义
#include "soem.h"         // 提供 SOEM 核心功能
#include "soem/ec_coe.h"  // 提供 SDO 读写函数 (ecx_SDOwrite)

#include "unistd.h"

typedef struct
{
    bool valid;
    uint32_t can_id;
    uint16_t params_raw;
    uint64_t timestamp;
    uint8_t dlen;
    uint8_t data[LINKX_CAN_MAX_DATA_BYTES];
} linkx_rx_snapshot_t;

static uint16_t linkx_pack_params(const can_pdo_param_t *params)
{
    uint16_t raw = 0;
    raw |= (uint16_t)(params->ext & 0x01u);
    raw |= (uint16_t)((params->rtr & 0x01u) << 1);
    raw |= (uint16_t)((params->canfd & 0x01u) << 2);
    raw |= (uint16_t)((params->brs & 0x01u) << 3);
    raw |= (uint16_t)(params->dlen << 8);
    return raw;
}

static bool linkx_snapshot_equals_pdo(const linkx_rx_snapshot_t *snap, const can_tx_pdo_t *pdo, uint8_t dlen)
{
    if (!snap->valid)
        return false;

    if (snap->can_id != pdo->can_id)
        return false;

    if (snap->params_raw != linkx_pack_params(&pdo->params))
        return false;

    if (snap->timestamp != pdo->timestamp)
        return false;

    if (snap->dlen != dlen)
        return false;

    if (dlen == 0)
        return true;

    return (memcmp(snap->data, pdo->data_u32, dlen) == 0);
}

static void linkx_snapshot_update(linkx_rx_snapshot_t *snap, const can_tx_pdo_t *pdo, uint8_t dlen)
{
    snap->valid = true;
    snap->can_id = pdo->can_id;
    snap->params_raw = linkx_pack_params(&pdo->params);
    snap->timestamp = pdo->timestamp;
    snap->dlen = dlen;

    if (dlen > 0)
        memcpy(snap->data, pdo->data_u32, dlen);
}

// 封装 SDO 唤醒逻辑，屏蔽 0x8001 寄存器细节
bool linkx_hw_wakeup(linkx_t *linkx)
{
    if (!linkx)
        return false;

    bool all_ok = true;
    uint8_t enable = 1;
    bool ch_ok[LINKX_CAN_CHANNEL_NUM] = {0};

    for (int ch = 0; ch < LINKX_CAN_CHANNEL_NUM; ch++)
    {
        bool ok = linkx_switch_can_channel(linkx, ch, enable != 0);
        ch_ok[ch] = ok;
        all_ok = all_ok && ok;
    }
    return all_ok;
}

// 封装发送：FDCAN自动处理指针强转和 linkx 参数顺序
void linkx_quick_FDcan_send(linkx_t *linkx, uint8_t ch, uint32_t id, const uint8_t *data)
{
    linkx_send_fdcan_frame(linkx, ch, id, true, false, false, 8, data);
}

// 封装发送：经典CAN自动处理指针强转和 linkx 参数顺序
void linkx_quick_can_send(linkx_t *linkx, uint8_t ch, uint32_t id, const uint8_t *data)
{
    linkx_send_classic_can_frame(linkx, ch, id, false, false, 8, data);
}

// 封装发送：支持可变长度 CAN-FD 负载
bool linkx_send_fdcan_frame(linkx_t *linkx, uint8_t ch, uint32_t id, bool brs, bool ext, bool rtr,
                            uint8_t dlen, const uint8_t *data)
{
    return linkx_send_can(linkx, ch, id, true, brs, ext, rtr, dlen, data);
}

// 封装发送：支持可变长度经典 CAN 负载
bool linkx_send_classic_can_frame(linkx_t *linkx, uint8_t ch, uint32_t id, bool ext, bool rtr,
                                  uint8_t dlen, const uint8_t *data)
{
    if (dlen > 8)
        dlen = 8;
    return linkx_send_can(linkx, ch, id, false, false, ext, rtr, dlen, data);
}

// 封装接收：内置硬件时间戳去重逻辑
bool linkx_quick_recv(linkx_t *linkx, uint8_t ch, can_msg_t *out_msg)
{
    static linkx_rx_snapshot_t last_snapshot[LINKX_CAN_CHANNEL_NUM] = {0};
    if (!linkx || !out_msg || ch >= LINKX_CAN_CHANNEL_NUM)
        return false;

    can_tx_pdo_t *rx_pdo = linkx_recv_can(linkx, ch); // 获取指向接收 PDO 的指针
    if (rx_pdo == NULL)
        return false;

    uint8_t pdo_dlen = rx_pdo->params.dlen;
    if (pdo_dlen > LINKX_CAN_MAX_DATA_BYTES)
        pdo_dlen = LINKX_CAN_MAX_DATA_BYTES;

    if (linkx_snapshot_equals_pdo(&last_snapshot[ch], rx_pdo, pdo_dlen))
        return false;

    out_msg->id = rx_pdo->can_id;           // CAN ID
    out_msg->timestamp = rx_pdo->timestamp; // 硬件时间戳
    out_msg->canfd = rx_pdo->params.canfd != 0;
    out_msg->brs = rx_pdo->params.brs != 0;
    out_msg->ext = rx_pdo->params.ext != 0;
    out_msg->rtr = rx_pdo->params.rtr != 0;

    uint8_t out_dlen = pdo_dlen;
    if (out_dlen > sizeof(out_msg->data))
        out_dlen = sizeof(out_msg->data);
    out_msg->dlen = out_dlen;

    memset(out_msg->data, 0, sizeof(out_msg->data));
    if (out_dlen > 0)
        memcpy(out_msg->data, rx_pdo->data_u32, out_dlen);

    linkx_snapshot_update(&last_snapshot[ch], rx_pdo, pdo_dlen);
    linkx->can_stats[ch].rx_frames++;
    linkx->can_stats[ch].rx_bytes += pdo_dlen;
    return true;
}

// 配置指定 CAN 通道的波特率 (仲裁段和数据段)
bool linkx_set_can_baudrate(linkx_t *linkx, uint8_t ch, uint8_t fd_en,
                          uint8_t n_pre, uint8_t n_seg1, uint8_t n_seg2, uint8_t n_sjw,
                          uint8_t d_pre, uint8_t d_seg1, uint8_t d_seg2, uint8_t d_sjw)
{
    if (!linkx || !linkx->master || ch >= LINKX_CAN_CHANNEL_NUM) return false;

    uint16_t index = 0x8002;  // 统一的时序配置索引
    int wkc_count = 0;        // 记录成功的 SDO 写入次数
    uint8_t val;

    // 1. 指定要配置的通道号 (SubIndex 0x01)
    val = ch;
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x01, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);

    // 2. 使能 CANFD (SubIndex 0x02)
    val = fd_en;
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x02, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);

    // 3-6. 写入仲裁段 (Nominal) 经典 CAN 波特率参数
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x03, FALSE, sizeof(n_pre), &n_pre, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x04, FALSE, sizeof(n_seg1), &n_seg1, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x05, FALSE, sizeof(n_seg2), &n_seg2, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x06, FALSE, sizeof(n_sjw), &n_sjw, EC_TIMEOUTRXM);

    // 7-10. 写入数据段 (Data) CAN-FD 高速波特率参数
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x07, FALSE, sizeof(d_pre), &d_pre, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x08, FALSE, sizeof(d_seg1), &d_seg1, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x09, FALSE, sizeof(d_seg2), &d_seg2, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x0A, FALSE, sizeof(d_sjw), &d_sjw, EC_TIMEOUTRXM);

    // 11. 配置生效触发 (SubIndex 0x0B)
    val = 1;
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x0B, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);

    // 一共进行了 11 次 SDO 写入，全部成功才算配置成功
    if (wkc_count == 11) {
        return true;
    } else {
        return false;
    }
}

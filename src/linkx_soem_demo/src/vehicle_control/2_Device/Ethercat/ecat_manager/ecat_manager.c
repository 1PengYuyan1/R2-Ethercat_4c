#include "ecat_manager.h"
#include <stdio.h>
#include "soem/soem.h"

#define ECAT_WKC_ERROR_LOG_PERIOD 100

static void ecat_dump_slave_states(ecat_master_t *master)
{
    if (!master)
        return;

    ecx_readstate(&master->ctx);
    printf("[ECAT] slave_count=%d expected_wkc=%d\n",
           master->ctx.slavecount,
           master->expected_wkc);

    for (int i = 1; i <= master->ctx.slavecount; ++i)
    {
        printf("[ECAT] slave %d name='%s' state=0x%02x AL=0x%04x (%s) obits=%d ibits=%d\n",
               i,
               master->ctx.slavelist[i].name,
               master->ctx.slavelist[i].state,
               master->ctx.slavelist[i].ALstatuscode,
               ec_ALstatuscode2string(master->ctx.slavelist[i].ALstatuscode),
               master->ctx.slavelist[i].Obits,
               master->ctx.slavelist[i].Ibits);
    }
}

// 初始化网卡并扫描从站
bool ecat_master_init(ecat_master_t *master, const char *ifname)
{
    if (!master || !ifname)
        return false;

    master->slave_count = 0;
    master->expected_wkc = 0;
    master->consecutive_wkc_errors = 0;
    master->is_running = false;

    if (!ecx_init(&master->ctx, ifname))
    {
        printf("[ECAT] ecx_init failed for interface '%s'. Check interface name, root/CAP_NET_RAW permission, and NIC availability.\n",
               ifname);
        return false;
    }

    if (ecx_config_init(&master->ctx) <= 0)
    {
        printf("[ECAT] ecx_config_init found no EtherCAT slaves on '%s'. Check cable, LinkX power, and that the EtherCAT cable is on the correct NIC.\n",
               ifname);
        return false;
    }

    printf("[ECAT] found %d slave(s) on '%s'\n", master->ctx.slavecount, ifname);

    // 1. 映射 IO 数据 (告诉系统报文有多长)
    ecx_config_map_group(&master->ctx, master->iomap, 0);
    master->expected_wkc = (master->ctx.grouplist[0].outputsWKC * 2) +
                           master->ctx.grouplist[0].inputsWKC;
    // DC 核心功能 ：对表与计算延迟
    // 必须放在 IO 映射之后，状态机检查之前！
    ecx_configdc(&master->ctx);
    
    // // DC 核心功能 ：激活从站的 Sync0 硬件心跳
    // // 假设你的控制周期是 1ms (1,000,000 纳秒)
    // // 偏移量 (Shift) 通常设置为周期的一半 (500,000 纳秒)，确保报文到达后，再触发动作
    // uint32_t cycle_time_ns = 1000000; 
    // uint32_t shift_time_ns = cycle_time_ns / 2;
    
    // for (int i = 1; i <= master->ctx.slavecount; i++) {
    //     // 激活第 i 个从站的 Sync0 中断
    //     ecx_dcsync0(&master->ctx, i, TRUE, cycle_time_ns, shift_time_ns);
    // }
    // 检查从站状态
    int state = ecx_statecheck(&master->ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 2);
    if (state != EC_STATE_SAFE_OP)
    {
        printf("[ECAT] SAFE_OP statecheck failed: requested=0x%02x got=0x%02x\n",
               EC_STATE_SAFE_OP,
               state);
        ecat_dump_slave_states(master);
        return false;
    }

    master->slave_count = master->ctx.slavecount;
    master->is_running = true;
    return true;
}


// 核心同步：发送并接收过程数据
int ecat_master_sync(ecat_master_t *master)
{
    if (!master)
        return 0;

    ecx_send_processdata(&master->ctx);
    int wkc = ecx_receive_processdata(&master->ctx, EC_TIMEOUTRET);

    if (master->expected_wkc > 0 && wkc < master->expected_wkc)
    {
        master->consecutive_wkc_errors++;
        if (master->consecutive_wkc_errors == 1 ||
            (master->consecutive_wkc_errors % ECAT_WKC_ERROR_LOG_PERIOD) == 0)
        {
            ecat_dump_slave_states(master);
        }
    }
    else
    {
        master->consecutive_wkc_errors = 0;
    }

    return wkc;
}

// 安全切换到 OP 状态（含看门狗预热逻辑）
bool ecat_master_bring_online(ecat_master_t *master)
{
    // 按 SOEM 推荐流程：先交换若干帧过程数据，避免 SAFE_OP->OP 时输出无效导致拒绝切换
    for (int i = 0; i < 5; i++)
    {
        ecx_send_processdata(&master->ctx);
        ecx_receive_processdata(&master->ctx, EC_TIMEOUTRET);
        osal_usleep(1000);
    }

    master->ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&master->ctx, 0);

    int chk = 40;
    do
    {
        ecx_send_processdata(&master->ctx);
        ecx_receive_processdata(&master->ctx, EC_TIMEOUTRET);
        osal_usleep(10000);
        ecx_statecheck(&master->ctx, 0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (master->ctx.slavelist[0].state != EC_STATE_OPERATIONAL));

    if (master->ctx.slavelist[0].state == EC_STATE_OPERATIONAL)
    {
        return true;
    }
    else
    {
        ecat_dump_slave_states(master);

        return false;
    }
}

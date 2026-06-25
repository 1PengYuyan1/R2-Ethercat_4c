#include <cstdio>
#include <signal.h>
#include <string>
#include "task.h"
#include "ecat_manager.h"

// 引用在 task.cpp 中定义的全局变量
extern ecat_master_t master;

/**
 * @brief 信号处理函数，用于优雅地关闭 EtherCAT 主站
 */
void signal_handler(int /*sig*/) {
    master.is_running = false;
}

int main(int argc, char *argv[]) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);

    // 设置默认网卡名称 (根据你的实际情况修改，或通过参数传入)
    std::string ifname = "enp86s0"; 

    // 如果运行程序时传入了参数，则使用参数作为网口名
    // 例如: sudo ./linkx_soem_demo eth0
    if (argc > 1)  ifname = argv[1];

    // 注册信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 启动机器人的核心控制任务
    // 该函数在 task.cpp 中实现，是阻塞运行的
    try {
        Robot_Control_Loop(ifname.c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[R2] Robot_Control_Loop 异常退出: %s\n", e.what());
        return -1;
    } catch (...) {
        std::fprintf(stderr, "[R2] Robot_Control_Loop 抛出未知异常退出\n");
        return -1;
    }
    
    return 0;
    
}

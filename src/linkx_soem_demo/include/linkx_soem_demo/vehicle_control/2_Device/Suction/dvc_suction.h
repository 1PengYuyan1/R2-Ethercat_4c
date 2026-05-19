#ifndef DVC_SUCTION_H
#define DVC_SUCTION_H

// Linux/ROS 上位机替身：原 STM32 GPIO 真空/电磁阀控制改为发布 std_msgs/UInt8 到 /r2/suction/cmd。
// 上位机其它节点（譬如 EtherCAT 数字输出桥）订阅这条话题，把命令落到真实硬件。
// 8 个静态方法的语义保留，便于上层代码无感切换。
//
// 命令编码：
//   0  stop2
//   1  start2
//   2  stop1
//   3  start1
//   4  stopa
//   5  starta
//   6  stop_give
//   7  give

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

class dvc_suction
{
public:
    static void Bind_Node(rclcpp::Node *node);

    static void start2(void);
    static void stop2(void);
    static void start1(void);
    static void stop1(void);
    static void starta(void);
    static void stopa(void);
    static void give(void);
    static void stop_give(void);

private:
    static rclcpp::Node *node_;
    static rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_;
    static void Publish_Cmd(uint8_t code, const char *name);
};

#endif

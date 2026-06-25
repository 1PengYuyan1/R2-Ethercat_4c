#include "crt_gripper.h"

#include "linkx4c_handler.h"

void Class_Chariot_Gripper::Init(linkx_t *__LinkX_Handler,
                                 uint8_t __CAN_Channel,
                                 uint32_t __CAN_ID)
{
    LinkX_Handler = __LinkX_Handler;
    CAN_Channel = __CAN_Channel;
    CAN_ID = __CAN_ID;
}

bool Class_Chariot_Gripper::Send_Command(uint8_t command)
{
    if (LinkX_Handler == nullptr)
    {
        return false;
    }

    // 下位机需要 3 字节负载：命令码重复 3 次（GRAB=01,01,01；RELEASE=02,02,02）。
    const uint8_t data[3] = {command, command, command};
    return linkx_send_classic_can_frame(LinkX_Handler,
                                        CAN_Channel,
                                        CAN_ID,
                                        false,
                                        false,
                                        sizeof(data),
                                        data);
}

bool Class_Chariot_Gripper::Grab()
{
    return Send_Command(CHARIOT_GRIPPER_CMD_GRAB);
}

bool Class_Chariot_Gripper::Release()
{
    return Send_Command(CHARIOT_GRIPPER_CMD_RELEASE);
}

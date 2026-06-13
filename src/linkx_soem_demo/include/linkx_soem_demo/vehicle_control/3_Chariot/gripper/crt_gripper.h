#ifndef CRT_GRIPPER_H
#define CRT_GRIPPER_H

#include "linkx.h"

#include <cstdint>

#define CHARIOT_GRIPPER_CAN_CHANNEL 0U
#define CHARIOT_GRIPPER_CAN_ID      0x06U

enum Enum_Chariot_Gripper_Command : uint8_t
{
    CHARIOT_GRIPPER_CMD_GRAB = 0x01U,
    CHARIOT_GRIPPER_CMD_RELEASE = 0x02U,
};

class Class_Chariot_Gripper
{
public:
    void Init(linkx_t *__LinkX_Handler,
              uint8_t __CAN_Channel = CHARIOT_GRIPPER_CAN_CHANNEL,
              uint32_t __CAN_ID = CHARIOT_GRIPPER_CAN_ID);

    bool Send_Command(uint8_t command);
    bool Grab();
    bool Release();

    uint8_t Get_CAN_Channel() const { return CAN_Channel; }
    uint32_t Get_CAN_ID() const { return CAN_ID; }

private:
    linkx_t *LinkX_Handler = nullptr;
    uint8_t CAN_Channel = CHARIOT_GRIPPER_CAN_CHANNEL;
    uint32_t CAN_ID = CHARIOT_GRIPPER_CAN_ID;
};

#endif

#ifndef DVC_BUZZER_H
#define DVC_BUZZER_H

// Linux/ROS 上位机替身：原 STM32 PWM 蜂鸣器只剩下日志打印。
// 接口保持与 r2 STM32 版兼容，便于上层代码无感切换。

#include <cstdint>

typedef enum {
    BUZZER_OFF = 0,
    BUZZER_STARTUP,
    BUZZER_DEVICE_LOST,
    BUZZER_EMERGENCY
} Buzzer_Mode_t;

#ifdef __cplusplus
extern "C" {
#endif

void Buzzer_Init(void);
void Buzzer_Stop(void);
void Buzzer_Set_Mode(Buzzer_Mode_t mode);

#ifdef __cplusplus
}
#endif

#endif

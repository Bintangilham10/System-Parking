#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ===================== SERVO SETTING ===================== */
#define SERVO_CLOSE_PULSE     500U
#define SERVO_OPEN_PULSE      1500U

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_Open(void);
void Servo_Close(void);
uint8_t Servo_IsOpen(void);

#ifdef __cplusplus
}
#endif

#endif

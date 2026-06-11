#include "servo_control.h"
#include "main.h"

static TIM_HandleTypeDef *servo_htim = 0;
static uint32_t servo_channel = TIM_CHANNEL_1;
static uint8_t servo_open_state = 0;

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
  servo_htim = htim;
  servo_channel = channel;
  servo_open_state = 0;

  if (HAL_TIM_PWM_Start(servo_htim, servo_channel) != HAL_OK)
  {
    Error_Handler();
  }

  Servo_Close();
}

void Servo_Open(void)
{
  if (servo_htim == 0)
  {
    return;
  }

  __HAL_TIM_SET_COMPARE(servo_htim, servo_channel, SERVO_OPEN_PULSE);
  servo_open_state = 1;
}

void Servo_Close(void)
{
  if (servo_htim == 0)
  {
    return;
  }

  __HAL_TIM_SET_COMPARE(servo_htim, servo_channel, SERVO_CLOSE_PULSE);
  servo_open_state = 0;
}

uint8_t Servo_IsOpen(void)
{
  return servo_open_state;
}

#ifndef LCD_I2C_H
#define LCD_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "parking_logic.h"
#include <stdint.h>

/* LCD I2C 16x2 PCF8574 */
#define LCD_ADDR_27       (0x27 << 1)
#define LCD_ADDR_3F       (0x3F << 1)

#define LCD_BACKLIGHT     0x08
#define LCD_ENABLE        0x04
#define LCD_COMMAND       0x00
#define LCD_DATA          0x01

void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_SendCmd(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SendString(char *str);
void LCD_Print16(const char *str);

void LCD_ShowParking(const ParkingSensorData *sensor,
                     const ParkingDecision *decision,
                     uint8_t gateOpen);

void LCD_ShowFault(SystemFaultCode faultCode);

#ifdef __cplusplus
}
#endif

#endif

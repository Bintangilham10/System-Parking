#include "lcd_i2c.h"
#include <stdio.h>

static I2C_HandleTypeDef *lcd_hi2c = 0;
static uint16_t lcd_addr = LCD_ADDR_27;
static uint8_t lcd_ready = 0;

static void LCD_Write(uint8_t data)
{
  if (!lcd_ready || lcd_hi2c == 0)
  {
    return;
  }

  HAL_I2C_Master_Transmit(lcd_hi2c, lcd_addr, &data, 1, 100);
}

static void LCD_PulseEnable(uint8_t data)
{
  LCD_Write(data | LCD_ENABLE | LCD_BACKLIGHT);
  HAL_Delay(1);

  LCD_Write((data & ~LCD_ENABLE) | LCD_BACKLIGHT);
  HAL_Delay(1);
}

static void LCD_Send4Bits(uint8_t data, uint8_t mode)
{
  uint8_t lcdData = (data & 0xF0) | LCD_BACKLIGHT | mode;
  LCD_PulseEnable(lcdData);
}

void LCD_SendCmd(uint8_t cmd)
{
  if (!lcd_ready)
  {
    return;
  }

  LCD_Send4Bits(cmd & 0xF0, LCD_COMMAND);
  LCD_Send4Bits((cmd << 4) & 0xF0, LCD_COMMAND);
}

void LCD_SendData(uint8_t data)
{
  if (!lcd_ready)
  {
    return;
  }

  LCD_Send4Bits(data & 0xF0, LCD_DATA);
  LCD_Send4Bits((data << 4) & 0xF0, LCD_DATA);
}

void LCD_Clear(void)
{
  if (!lcd_ready)
  {
    return;
  }

  LCD_SendCmd(0x01);
  HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
  if (!lcd_ready)
  {
    return;
  }

  if (row == 0)
  {
    LCD_SendCmd(0x80 + col);
  }
  else
  {
    LCD_SendCmd(0xC0 + col);
  }
}

void LCD_SendString(char *str)
{
  if (!lcd_ready)
  {
    return;
  }

  while (*str)
  {
    LCD_SendData((uint8_t)*str);
    str++;
  }
}

void LCD_Print16(const char *str)
{
  if (!lcd_ready)
  {
    return;
  }

  uint8_t i = 0;

  while (i < 16 && str[i] != '\0')
  {
    LCD_SendData((uint8_t)str[i]);
    i++;
  }

  while (i < 16)
  {
    LCD_SendData(' ');
    i++;
  }
}

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
  lcd_hi2c = hi2c;
  lcd_ready = 0;

  HAL_Delay(100);

  /*
    Cek alamat LCD otomatis.
    Umumnya 0x27 atau 0x3F.
  */
  if (HAL_I2C_IsDeviceReady(lcd_hi2c, LCD_ADDR_27, 3, 100) == HAL_OK)
  {
    lcd_addr = LCD_ADDR_27;
    lcd_ready = 1;
  }
  else if (HAL_I2C_IsDeviceReady(lcd_hi2c, LCD_ADDR_3F, 3, 100) == HAL_OK)
  {
    lcd_addr = LCD_ADDR_3F;
    lcd_ready = 1;
  }
  else
  {
    lcd_ready = 0;
    return;
  }

  HAL_Delay(50);

  /* Init LCD mode 4-bit */
  LCD_Send4Bits(0x30, LCD_COMMAND);
  HAL_Delay(5);

  LCD_Send4Bits(0x30, LCD_COMMAND);
  HAL_Delay(1);

  LCD_Send4Bits(0x30, LCD_COMMAND);
  HAL_Delay(10);

  LCD_Send4Bits(0x20, LCD_COMMAND);
  HAL_Delay(10);

  LCD_SendCmd(0x28);  /* 4-bit, 2 line, 5x8 font */
  LCD_SendCmd(0x0C);  /* display ON, cursor OFF */
  LCD_SendCmd(0x06);  /* entry mode */
  LCD_Clear();

  LCD_SetCursor(0, 0);
  LCD_Print16("RTOS PARKING");

  LCD_SetCursor(1, 0);
  LCD_Print16("LCD OK");
}

void LCD_ShowParking(const ParkingSensorData *sensor,
                     const ParkingDecision *decision,
                     uint8_t gateOpen)
{
  if (!lcd_ready || sensor == 0 || decision == 0)
  {
    return;
  }

  char line1[17];
  char line2[17];

  if (decision->gateBlocked)
  {
    snprintf(line1, sizeof(line1), "FULL - BLOCKED");
  }
  else if (decision->parkingFull)
  {
    snprintf(line1, sizeof(line1), "Parking FULL");
  }
  else if (gateOpen)
  {
    snprintf(line1, sizeof(line1), "Gate OPEN");
  }
  else
  {
    snprintf(line1, sizeof(line1), "Gate CLOSED");
  }

  snprintf(line2, sizeof(line2), "S1:%c S2:%c Safe:%c",
           sensor->slot1Occupied ? 'F' : 'E',
           sensor->slot2Occupied ? 'F' : 'E',
           sensor->safetyDetected ? 'Y' : 'N');

  LCD_SetCursor(0, 0);
  LCD_Print16(line1);

  LCD_SetCursor(1, 0);
  LCD_Print16(line2);
}

/* ===================== LCD FAULT DISPLAY ===================== */

void LCD_ShowFault(SystemFaultCode faultCode)
{
  if (!lcd_ready)
  {
    return;
  }

  LCD_SetCursor(0, 0);
  LCD_Print16("SYSTEM FAULT");

  LCD_SetCursor(1, 0);

  switch (faultCode)
  {
    case FAULT_SENSOR_TASK_TIMEOUT:
      LCD_Print16("Sensor Timeout");
      break;

    case FAULT_SAFETY_TASK_TIMEOUT:
      LCD_Print16("Safety Timeout");
      break;

    case FAULT_GATE_TASK_TIMEOUT:
      LCD_Print16("Gate Timeout");
      break;

    case FAULT_LED_TASK_TIMEOUT:
      LCD_Print16("LED Timeout");
      break;

    case FAULT_LCD_TASK_TIMEOUT:
      LCD_Print16("LCD Timeout");
      break;

    case FAULT_STACK_LOW:
      LCD_Print16("Stack Low");
      break;

    case FAULT_SIMULATION:
      LCD_Print16("Sim Fault");
      break;

    case FAULT_NONE:
      LCD_Print16("No Fault");
      break;

    default:
      LCD_Print16("Unknown Fault");
      break;
  }
}

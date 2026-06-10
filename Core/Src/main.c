/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - RTOS Smart Parking
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

/* ===================== BUZZER MANUAL ===================== */
#ifndef BUZZER_Pin
#define BUZZER_Pin GPIO_PIN_10
#endif

#ifndef BUZZER_GPIO_Port
#define BUZZER_GPIO_Port GPIOB
#endif

/* ===================== IR SAFETY PB2 ===================== */
#ifndef IR_SAFETY_Pin
  #ifdef sensor_safety_Pin
    #define IR_SAFETY_Pin sensor_safety_Pin
  #else
    #define IR_SAFETY_Pin GPIO_PIN_2
  #endif
#endif

#ifndef IR_SAFETY_GPIO_Port
  #ifdef sensor_safety_GPIO_Port
    #define IR_SAFETY_GPIO_Port sensor_safety_GPIO_Port
  #else
    #define IR_SAFETY_GPIO_Port GPIOB
  #endif
#endif

/* ===================== SERVO SETTING ===================== */
#define SERVO_CLOSE             500
#define SERVO_OPEN              1500

/* ===================== PARKING SETTING ===================== */
#define GATE_HOLD_MS            2000

/* IR aktif LOW */
#define IR_ACTIVE_LOW           1

/* ===================== LCD I2C SETTING ===================== */
/*
   LCD I2C 16x2 PCF8574
   SDA = PB7
   SCL = PB6
   Address umum: 0x27 atau 0x3F
*/
#define LCD_ADDR_27             (0x27 << 1)
#define LCD_ADDR_3F             (0x3F << 1)

#define LCD_BACKLIGHT           0x08
#define LCD_ENABLE              0x04
#define LCD_COMMAND             0x00
#define LCD_DATA                0x01

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN PV */

osThreadId_t sensorTaskHandle;
osThreadId_t gateTaskHandle;
osThreadId_t indicatorTaskHandle;

const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t gateTask_attributes = {
  .name = "gateTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t indicatorTask_attributes = {
  .name = "indicatorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

volatile uint8_t slot1Occupied = 0;
volatile uint8_t slot2Occupied = 0;
volatile uint8_t entryDetected = 0;
volatile uint8_t exitDetected = 0;
volatile uint8_t safetyDetected = 0;
volatile uint8_t parkingFull = 0;
volatile uint8_t gateOpen = 0;

volatile uint32_t lastGateOpenTick = 0;

/* LCD variable */
uint16_t lcd_addr = LCD_ADDR_27;
uint8_t lcd_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

void StartSensorTask(void *argument);
void StartGateTask(void *argument);
void StartIndicatorTask(void *argument);

void Servo_Close(void);
void Servo_Open(void);

uint8_t IR_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t IR_Safety_Read(void);

uint8_t Gate_CanOpenFromClosed(void);
uint8_t Gate_ShouldHoldOpen(void);

/* LCD function */
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_SendCmd(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SendString(char *str);
void LCD_Print16(const char *str);
void LCD_ShowParking(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ===================== SERVO ===================== */

void Servo_Close(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CLOSE);
  gateOpen = 0;
}

void Servo_Open(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_OPEN);
  gateOpen = 1;
}

/* ===================== SENSOR IR ===================== */

uint8_t IR_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
#if IR_ACTIVE_LOW
  return (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) ? 1 : 0;
#else
  return (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) ? 1 : 0;
#endif
}

uint8_t IR_Safety_Read(void)
{
#if IR_ACTIVE_LOW
  return (HAL_GPIO_ReadPin(IR_SAFETY_GPIO_Port, IR_SAFETY_Pin) == GPIO_PIN_RESET) ? 1 : 0;
#else
  return (HAL_GPIO_ReadPin(IR_SAFETY_GPIO_Port, IR_SAFETY_Pin) == GPIO_PIN_SET) ? 1 : 0;
#endif
}

/* ===================== LCD I2C ===================== */

static void LCD_Write(uint8_t data)
{
  if (!lcd_ready) return;

  HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, &data, 1, 100);
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
  if (!lcd_ready) return;

  LCD_Send4Bits(cmd & 0xF0, LCD_COMMAND);
  LCD_Send4Bits((cmd << 4) & 0xF0, LCD_COMMAND);
}

void LCD_SendData(uint8_t data)
{
  if (!lcd_ready) return;

  LCD_Send4Bits(data & 0xF0, LCD_DATA);
  LCD_Send4Bits((data << 4) & 0xF0, LCD_DATA);
}

void LCD_Clear(void)
{
  if (!lcd_ready) return;

  LCD_SendCmd(0x01);
  HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
  if (!lcd_ready) return;

  uint8_t address;

  if (row == 0)
  {
    address = 0x80 + col;
  }
  else
  {
    address = 0xC0 + col;
  }

  LCD_SendCmd(address);
}

void LCD_SendString(char *str)
{
  if (!lcd_ready) return;

  while (*str)
  {
    LCD_SendData((uint8_t)*str);
    str++;
  }
}

void LCD_Print16(const char *str)
{
  if (!lcd_ready) return;

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

void LCD_Init(void)
{
  HAL_Delay(100);

  /*
    Cek alamat LCD otomatis.
    Biasanya 0x27 atau 0x3F.
  */
  if (HAL_I2C_IsDeviceReady(&hi2c1, LCD_ADDR_27, 3, 100) == HAL_OK)
  {
    lcd_addr = LCD_ADDR_27;
    lcd_ready = 1;
  }
  else if (HAL_I2C_IsDeviceReady(&hi2c1, LCD_ADDR_3F, 3, 100) == HAL_OK)
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

  LCD_SendCmd(0x28);  // 4-bit, 2 line, 5x8 font
  LCD_SendCmd(0x0C);  // display ON, cursor OFF
  LCD_SendCmd(0x06);  // entry mode
  LCD_Clear();

  LCD_SetCursor(0, 0);
  LCD_Print16("RTOS PARKING");

  LCD_SetCursor(1, 0);
  LCD_Print16("LCD OK");
}

void LCD_ShowParking(void)
{
  if (!lcd_ready) return;

  char line1[17];
  char line2[17];

  if (parkingFull && (entryDetected || exitDetected))
  {
    snprintf(line1, sizeof(line1), "FULL - BLOCKED");
  }
  else if (parkingFull)
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
           slot1Occupied ? 'F' : 'E',
           slot2Occupied ? 'F' : 'E',
           safetyDetected ? 'Y' : 'N');

  LCD_SetCursor(0, 0);
  LCD_Print16(line1);

  LCD_SetCursor(1, 0);
  LCD_Print16(line2);
}

/* ===================== LOGIC GATE ===================== */

uint8_t Gate_CanOpenFromClosed(void)
{
  if (!parkingFull)
  {
    if (entryDetected || exitDetected || safetyDetected)
    {
      return 1;
    }
  }

  return 0;
}

uint8_t Gate_ShouldHoldOpen(void)
{
  if (!parkingFull)
  {
    if (entryDetected || exitDetected)
    {
      return 1;
    }
  }

  if (gateOpen && safetyDetected)
  {
    return 1;
  }

  return 0;
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  Servo_Close();

  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  LCD_Init();
  HAL_Delay(1000);
  LCD_ShowParking();

  /* USER CODE END 2 */

  osKernelInitialize();

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);
  gateTaskHandle = osThreadNew(StartGateTask, NULL, &gateTask_attributes);
  indicatorTaskHandle = osThreadNew(StartIndicatorTask, NULL, &indicatorTask_attributes);

  /* USER CODE END RTOS_THREADS */

  osKernelStart();

  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  __HAL_RCC_I2C1_CLK_ENABLE();

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_CLOSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin | LED_RED_Pin | BUZZER_Pin, GPIO_PIN_RESET);

  /*
    I2C1 LCD:
    PB6 = SCL
    PB7 = SDA
  */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*
    IR INPUT GPIOA:
    PA0 = Slot 1
    PA1 = Slot 2
    PA2 = Entry
    PA3 = Exit
  */
  GPIO_InitStruct.Pin = IR_SLOT1_Pin |
                        IR_SLOT2_Pin |
                        IR_ENTRY_Pin |
                        IR_EXIT_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*
    IR Safety:
    PB2 = sensor_safety
  */
  GPIO_InitStruct.Pin = IR_SAFETY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_SAFETY_GPIO_Port, &GPIO_InitStruct);

  /*
    LED + BUZZER:
    PB12 = LED Merah
    PB14 = LED Hijau
    PB10 = Buzzer
  */
  GPIO_InitStruct.Pin = LED_GREEN_Pin | LED_RED_Pin | BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void StartSensorTask(void *argument)
{
  for (;;)
  {
    slot1Occupied = IR_Read(IR_SLOT1_GPIO_Port, IR_SLOT1_Pin);
    slot2Occupied = IR_Read(IR_SLOT2_GPIO_Port, IR_SLOT2_Pin);

    entryDetected = IR_Read(IR_ENTRY_GPIO_Port, IR_ENTRY_Pin);
    exitDetected = IR_Read(IR_EXIT_GPIO_Port, IR_EXIT_Pin);
    safetyDetected = IR_Safety_Read();

    parkingFull = (slot1Occupied && slot2Occupied) ? 1 : 0;

    osDelay(20);
  }
}

void StartGateTask(void *argument)
{
  for (;;)
  {
    slot1Occupied = IR_Read(IR_SLOT1_GPIO_Port, IR_SLOT1_Pin);
    slot2Occupied = IR_Read(IR_SLOT2_GPIO_Port, IR_SLOT2_Pin);

    entryDetected = IR_Read(IR_ENTRY_GPIO_Port, IR_ENTRY_Pin);
    exitDetected = IR_Read(IR_EXIT_GPIO_Port, IR_EXIT_Pin);
    safetyDetected = IR_Safety_Read();

    parkingFull = (slot1Occupied && slot2Occupied) ? 1 : 0;

    if (!gateOpen)
    {
      if (Gate_CanOpenFromClosed())
      {
        Servo_Open();
        lastGateOpenTick = HAL_GetTick();
      }
      else
      {
        Servo_Close();
      }
    }
    else
    {
      if (Gate_ShouldHoldOpen())
      {
        Servo_Open();
        lastGateOpenTick = HAL_GetTick();
      }
      else
      {
        if ((HAL_GetTick() - lastGateOpenTick) >= GATE_HOLD_MS)
        {
          Servo_Close();
        }
      }
    }

    osDelay(10);
  }
}

void StartIndicatorTask(void *argument)
{
  uint32_t lastLcdUpdate = 0;

  for (;;)
  {
    slot1Occupied = IR_Read(IR_SLOT1_GPIO_Port, IR_SLOT1_Pin);
    slot2Occupied = IR_Read(IR_SLOT2_GPIO_Port, IR_SLOT2_Pin);

    entryDetected = IR_Read(IR_ENTRY_GPIO_Port, IR_ENTRY_Pin);
    exitDetected = IR_Read(IR_EXIT_GPIO_Port, IR_EXIT_Pin);
    safetyDetected = IR_Safety_Read();

    parkingFull = (slot1Occupied && slot2Occupied) ? 1 : 0;

    if (gateOpen)
    {
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }
    else
    {
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

      if (parkingFull && (entryDetected || exitDetected))
      {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      }
      else
      {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
      }
    }

    /*
      LCD jangan di-update terlalu cepat.
      Update tiap 500 ms supaya tidak flicker.
    */
    if ((HAL_GetTick() - lastLcdUpdate) >= 500)
    {
      LCD_ShowParking();
      lastLcdUpdate = HAL_GetTick();
    }

    osDelay(20);
  }
}

/* USER CODE END 4 */

void StartDefaultTask(void *argument)
{
  for (;;)
  {
    osDelay(1000);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

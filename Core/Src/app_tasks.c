#include "app_tasks.h"
#include "main.h"
#include "servo_control.h"
#include "lcd_i2c.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ===================== SOFTWARE WATCHDOG SETTING ===================== */
#define SENSOR_TASK_TIMEOUT_MS   100U
#define SAFETY_TASK_TIMEOUT_MS   100U
#define GATE_TASK_TIMEOUT_MS     100U
#define LED_TASK_TIMEOUT_MS      250U
#define LCD_TASK_TIMEOUT_MS      1500U

/* uxTaskGetStackHighWaterMark menghasilkan satuan word, bukan byte */
#define MIN_STACK_FREE_WORDS     40U

/*
  Debounce ISR Exit:
  100 ms berarti interrupt yang muncul terlalu cepat setelah interrupt valid
  akan dianggap noise/bounce dan tidak diteruskan ke SensorTask.
*/
#define EXIT_IRQ_DEBOUNCE_MS     100U

/*
  Untuk demo fault:
  0 = normal
  1 = aktifkan simulasi fault
*/
#define ENABLE_FAULT_SIMULATION  0

/* ===================== TASK HANDLE ===================== */
/*
  defaultTaskHandle TIDAK dibuat di sini karena sudah dibuat di main.c CubeMX.
  Di file ini hanya dibuat enam task aplikasi.
*/
osThreadId_t sensorTaskHandle;
osThreadId_t safetyTaskHandle;
osThreadId_t gateTaskHandle;
osThreadId_t ledTaskHandle;
osThreadId_t lcdTaskHandle;
osThreadId_t monitorTaskHandle;

/* ===================== RTOS OBJECT ===================== */
static osMutexId_t stateMutexHandle;
static osMutexId_t lcdMutexHandle;
static osMessageQueueId_t sensorQueueHandle;
static osSemaphoreId_t exitIrqSemaphoreHandle;

static const osMutexAttr_t stateMutex_attributes = {
  .name = "stateMutex"
};

static const osMutexAttr_t lcdMutex_attributes = {
  .name = "lcdMutex"
};

static const osSemaphoreAttr_t exitIrqSemaphore_attributes = {
  .name = "exitIrqSemaphore"
};

/* ===================== TASK ATTRIBUTE ===================== */
static const osThreadAttr_t sensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

static const osThreadAttr_t safetyTask_attributes = {
  .name = "SafetyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

static const osThreadAttr_t gateTask_attributes = {
  .name = "GateTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

static const osThreadAttr_t ledTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t lcdTask_attributes = {
  .name = "LcdTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

static const osThreadAttr_t monitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* ===================== SHARED STATE ===================== */
static ParkingSensorData g_sensorData;
static ParkingDecision g_decision;
static uint8_t g_gateOpen = 0;
static uint32_t g_lastGateOpenTick = 0;

/* ===================== SOFTWARE WATCHDOG STATE ===================== */
static volatile uint32_t sensorHeartbeatTick = 0;
static volatile uint32_t safetyHeartbeatTick = 0;
static volatile uint32_t gateHeartbeatTick = 0;
static volatile uint32_t ledHeartbeatTick = 0;
static volatile uint32_t lcdHeartbeatTick = 0;

static volatile uint8_t systemFault = 0;
static volatile SystemFaultCode systemFaultCode = FAULT_NONE;

#if ENABLE_FAULT_SIMULATION
static volatile uint8_t simulateFault = 1;
#else
static volatile uint8_t simulateFault = 0;
#endif

/* ===================== ISR DEBUG COUNTER ===================== */
/*
  exitIrqRawCount:
  Menghitung semua interrupt mentah dari PA3 sebelum debounce.

  exitIrqBounceCount:
  Menghitung interrupt yang ditolak karena dianggap noise/bounce.

  exitIrqCount:
  Menghitung interrupt valid setelah debounce.

  exitDeferredCount:
  Menghitung event yang berhasil diterima SensorTask dari semaphore.
*/
volatile uint32_t exitIrqRawCount = 0;
volatile uint32_t exitIrqBounceCount = 0;
volatile uint32_t exitIrqCount = 0;
volatile uint32_t exitDeferredCount = 0;
volatile uint32_t lastExitIrqTick = 0;

static volatile uint32_t lastExitIrqAcceptedTick = 0;

/* ===================== STACK MONITORING VARIABLE ===================== */
volatile uint32_t monitorSensorStackFree = 0;
volatile uint32_t monitorSafetyStackFree = 0;
volatile uint32_t monitorGateStackFree = 0;
volatile uint32_t monitorLedStackFree = 0;
volatile uint32_t monitorLcdStackFree = 0;
volatile uint32_t monitorTaskStackFree = 0;

/* ===================== TASK PROTOTYPE ===================== */
static void StartSensorTask(void *argument);
static void StartSafetyTask(void *argument);
static void StartGateTask(void *argument);
static void StartLedTask(void *argument);
static void StartLcdTask(void *argument);
static void StartMonitorTask(void *argument);

/* ===================== HELPER PROTOTYPE ===================== */
static uint8_t App_IR_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static ParkingSensorData App_ReadAllSensors(void);
static void App_SetSensorData(ParkingSensorData data);
static void App_SetDecision(ParkingDecision decision);
static void App_SetGateOpen(uint8_t gateOpen);
static void App_EnterSafeState(SystemFaultCode code);

/* ===================== ISR DEFERRED CALLBACK + DEBOUNCE ===================== */
void AppTasks_ExitIrqCallbackFromISR(void)
{
  uint32_t nowTick;

  nowTick = HAL_GetTick();

  /*
    Hitung semua interrupt mentah.
    Nilai ini bisa besar karena sensor IR dapat menghasilkan noise/bounce.
  */
  exitIrqRawCount++;

  /*
    Debounce:
    Jika interrupt baru muncul terlalu dekat dari interrupt valid sebelumnya,
    maka interrupt dianggap noise dan tidak diteruskan ke task.
  */
  if (lastExitIrqAcceptedTick != 0)
  {
    if ((nowTick - lastExitIrqAcceptedTick) < EXIT_IRQ_DEBOUNCE_MS)
    {
      exitIrqBounceCount++;
      return;
    }
  }

  /*
    Interrupt valid setelah debounce.
  */
  lastExitIrqAcceptedTick = nowTick;
  exitIrqCount++;

  /*
    Deferred processing:
    ISR hanya melepas semaphore.
    Pemrosesan event dilakukan di SensorTask.
  */
  if (exitIrqSemaphoreHandle != 0)
  {
    osSemaphoreRelease(exitIrqSemaphoreHandle);
  }
}

/* ===================== HELPER ===================== */
static uint8_t App_IR_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
#if IR_ACTIVE_LOW
  return (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) ? 1 : 0;
#else
  return (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) ? 1 : 0;
#endif
}

static ParkingSensorData App_ReadAllSensors(void)
{
  ParkingSensorData data;

  data.slot1Occupied = App_IR_Read(IR_SLOT1_GPIO_Port, IR_SLOT1_Pin);
  data.slot2Occupied = App_IR_Read(IR_SLOT2_GPIO_Port, IR_SLOT2_Pin);
  data.entryDetected = App_IR_Read(IR_ENTRY_GPIO_Port, IR_ENTRY_Pin);
  data.exitDetected = App_IR_Read(IR_EXIT_GPIO_Port, IR_EXIT_Pin);
  data.safetyDetected = App_IR_Read(sensor_safety_GPIO_Port, sensor_safety_Pin);

  return data;
}

ParkingSensorData AppTasks_GetSensorSnapshot(void)
{
  ParkingSensorData copy;

  memset(&copy, 0, sizeof(copy));

  if (stateMutexHandle != 0)
  {
    osMutexAcquire(stateMutexHandle, osWaitForever);
    copy = g_sensorData;
    osMutexRelease(stateMutexHandle);
  }

  return copy;
}

ParkingDecision AppTasks_GetDecisionSnapshot(void)
{
  ParkingDecision copy;

  memset(&copy, 0, sizeof(copy));

  if (stateMutexHandle != 0)
  {
    osMutexAcquire(stateMutexHandle, osWaitForever);
    copy = g_decision;
    osMutexRelease(stateMutexHandle);
  }

  return copy;
}

uint8_t AppTasks_GetGateOpen(void)
{
  uint8_t copy = 0;

  if (stateMutexHandle != 0)
  {
    osMutexAcquire(stateMutexHandle, osWaitForever);
    copy = g_gateOpen;
    osMutexRelease(stateMutexHandle);
  }

  return copy;
}

static void App_SetSensorData(ParkingSensorData data)
{
  if (stateMutexHandle != 0)
  {
    osMutexAcquire(stateMutexHandle, osWaitForever);
    g_sensorData = data;
    osMutexRelease(stateMutexHandle);
  }
}

static void App_SetDecision(ParkingDecision decision)
{
  if (stateMutexHandle != 0)
  {
    osMutexAcquire(stateMutexHandle, osWaitForever);
    g_decision = decision;
    osMutexRelease(stateMutexHandle);
  }
}

static void App_SetGateOpen(uint8_t gateOpen)
{
  if (stateMutexHandle != 0)
  {
    osMutexAcquire(stateMutexHandle, osWaitForever);
    g_gateOpen = gateOpen;
    osMutexRelease(stateMutexHandle);
  }
}

/* ===================== SAFE STATE ===================== */
static void App_EnterSafeState(SystemFaultCode code)
{
  ParkingSensorData sensor;

  systemFault = 1;
  systemFaultCode = code;

  sensor = AppTasks_GetSensorSnapshot();

  /*
    Safe-state:
    1. Jika safety sensor aktif, gate ditahan OPEN agar tidak menjepit objek.
    2. Jika safety sensor tidak aktif, gate CLOSE sebagai posisi aman default.
  */
  if (sensor.safetyDetected)
  {
    Servo_Open();
    App_SetGateOpen(1);

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  }
  else
  {
    Servo_Close();
    App_SetGateOpen(0);

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  }
}

/* ===================== INIT TASK ===================== */
void AppTasks_Init(void)
{
  uint32_t now;

  memset(&g_sensorData, 0, sizeof(g_sensorData));
  memset(&g_decision, 0, sizeof(g_decision));

  g_gateOpen = 0;
  g_lastGateOpenTick = 0;

  systemFault = 0;
  systemFaultCode = FAULT_NONE;

  now = HAL_GetTick();

  sensorHeartbeatTick = now;
  safetyHeartbeatTick = now;
  gateHeartbeatTick = now;
  ledHeartbeatTick = now;
  lcdHeartbeatTick = now;

  lastExitIrqAcceptedTick = 0;

  stateMutexHandle = osMutexNew(&stateMutex_attributes);
  lcdMutexHandle = osMutexNew(&lcdMutex_attributes);

  sensorQueueHandle = osMessageQueueNew(4,
                                        sizeof(ParkingSensorData),
                                        NULL);

  /*
    Binary semaphore untuk deferred processing dari ISR EXTI3.
    max count = 1, initial count = 0.
  */
  exitIrqSemaphoreHandle = osSemaphoreNew(1U,
                                          0U,
                                          &exitIrqSemaphore_attributes);

  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);
  safetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &safetyTask_attributes);
  gateTaskHandle = osThreadNew(StartGateTask, NULL, &gateTask_attributes);
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);
  lcdTaskHandle = osThreadNew(StartLcdTask, NULL, &lcdTask_attributes);
  monitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &monitorTask_attributes);
}

/* ===================== SENSOR TASK ===================== */
static void StartSensorTask(void *argument)
{
  (void) argument;

  ParkingSensorData data;
  uint8_t exitIrqEvent;

  for (;;)
  {
    sensorHeartbeatTick = HAL_GetTick();

    exitIrqEvent = 0;

    /*
      Deferred processing dari ISR:
      Jika EXTI3 terjadi, SensorTask mengambil semaphore
      lalu membaca semua sensor di level task, bukan di ISR.
    */
    if (exitIrqSemaphoreHandle != 0)
    {
      if (osSemaphoreAcquire(exitIrqSemaphoreHandle, 0) == osOK)
      {
        exitIrqEvent = 1;
        exitDeferredCount++;
        lastExitIrqTick = HAL_GetTick();
      }
    }

    data = App_ReadAllSensors();

    /*
      Jika event keluar datang dari ISR, exitDetected dilatch satu siklus.
      Ini membantu agar event tidak hilang walaupun pulsa sensor terlalu cepat.
    */
    if (exitIrqEvent)
    {
      data.exitDetected = 1;
    }

    App_SetSensorData(data);

    if (sensorQueueHandle != 0)
    {
      osMessageQueuePut(sensorQueueHandle, &data, 0, 0);
    }

    osDelay(20);
  }
}

/* ===================== SAFETY TASK ===================== */
static void StartSafetyTask(void *argument)
{
  (void) argument;

  ParkingSensorData data;
  ParkingDecision decision;
  uint8_t gateOpen;

  memset(&data, 0, sizeof(data));
  memset(&decision, 0, sizeof(decision));

  for (;;)
  {
    safetyHeartbeatTick = HAL_GetTick();

    if (sensorQueueHandle != 0)
    {
      if (osMessageQueueGet(sensorQueueHandle, &data, NULL, 20) != osOK)
      {
        data = AppTasks_GetSensorSnapshot();
      }
    }
    else
    {
      data = AppTasks_GetSensorSnapshot();
    }

    gateOpen = AppTasks_GetGateOpen();

    ParkingLogic_Evaluate(&data, gateOpen, &decision);

    App_SetDecision(decision);

    osDelay(10);
  }
}

/* ===================== GATE TASK ===================== */
static void StartGateTask(void *argument)
{
  (void) argument;

  ParkingDecision decision;
  uint8_t gateOpen;
  uint32_t nowTick;

  for (;;)
  {
    gateHeartbeatTick = HAL_GetTick();

    if (systemFault)
    {
      App_EnterSafeState(systemFaultCode);
      osDelay(10);
      continue;
    }

    decision = AppTasks_GetDecisionSnapshot();
    gateOpen = AppTasks_GetGateOpen();
    nowTick = HAL_GetTick();

    if (!gateOpen)
    {
      if (decision.allowGateOpen && !decision.gateBlocked)
      {
        Servo_Open();
        App_SetGateOpen(1);
        g_lastGateOpenTick = nowTick;
      }
      else
      {
        Servo_Close();
        App_SetGateOpen(0);
      }
    }
    else
    {
      if (decision.holdGateOpen && !decision.gateBlocked)
      {
        Servo_Open();
        App_SetGateOpen(1);
        g_lastGateOpenTick = nowTick;
      }
      else
      {
        if ((nowTick - g_lastGateOpenTick) >= GATE_HOLD_MS)
        {
          Servo_Close();
          App_SetGateOpen(0);
        }
      }
    }

    osDelay(10);
  }
}

/* ===================== LED TASK ===================== */
static void StartLedTask(void *argument)
{
  (void) argument;

  uint8_t gateOpen;
  ParkingDecision decision;

  for (;;)
  {
    ledHeartbeatTick = HAL_GetTick();

    if (systemFault)
    {
      gateOpen = AppTasks_GetGateOpen();

      if (gateOpen)
      {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      }
      else
      {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      }

      osDelay(50);
      continue;
    }

    gateOpen = AppTasks_GetGateOpen();
    decision = AppTasks_GetDecisionSnapshot();

    if (gateOpen && !decision.gateBlocked)
    {
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    }
    else
    {
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    }

    osDelay(50);
  }
}

/* ===================== LCD TASK ===================== */
static void StartLcdTask(void *argument)
{
  (void) argument;

  ParkingSensorData sensor;
  ParkingDecision decision;
  uint8_t gateOpen;

  for (;;)
  {
    lcdHeartbeatTick = HAL_GetTick();

    sensor = AppTasks_GetSensorSnapshot();
    decision = AppTasks_GetDecisionSnapshot();
    gateOpen = AppTasks_GetGateOpen();

    if (lcdMutexHandle != 0)
    {
      osMutexAcquire(lcdMutexHandle, osWaitForever);

      if (systemFault)
      {
        LCD_ShowFault(systemFaultCode);
      }
      else
      {
        LCD_ShowParking(&sensor, &decision, gateOpen);
      }

      osMutexRelease(lcdMutexHandle);
    }

    osDelay(500);
  }
}

/* ===================== MONITOR TASK / SOFTWARE WATCHDOG ===================== */
static void StartMonitorTask(void *argument)
{
  (void) argument;

  uint32_t nowTick;

  for (;;)
  {
    nowTick = HAL_GetTick();

    monitorSensorStackFree = uxTaskGetStackHighWaterMark((TaskHandle_t)sensorTaskHandle);
    monitorSafetyStackFree = uxTaskGetStackHighWaterMark((TaskHandle_t)safetyTaskHandle);
    monitorGateStackFree = uxTaskGetStackHighWaterMark((TaskHandle_t)gateTaskHandle);
    monitorLedStackFree = uxTaskGetStackHighWaterMark((TaskHandle_t)ledTaskHandle);
    monitorLcdStackFree = uxTaskGetStackHighWaterMark((TaskHandle_t)lcdTaskHandle);
    monitorTaskStackFree = uxTaskGetStackHighWaterMark((TaskHandle_t)monitorTaskHandle);

    if (simulateFault)
    {
      App_EnterSafeState(FAULT_SIMULATION);
    }
    else if ((nowTick - sensorHeartbeatTick) > SENSOR_TASK_TIMEOUT_MS)
    {
      App_EnterSafeState(FAULT_SENSOR_TASK_TIMEOUT);
    }
    else if ((nowTick - safetyHeartbeatTick) > SAFETY_TASK_TIMEOUT_MS)
    {
      App_EnterSafeState(FAULT_SAFETY_TASK_TIMEOUT);
    }
    else if ((nowTick - gateHeartbeatTick) > GATE_TASK_TIMEOUT_MS)
    {
      App_EnterSafeState(FAULT_GATE_TASK_TIMEOUT);
    }
    else if ((nowTick - ledHeartbeatTick) > LED_TASK_TIMEOUT_MS)
    {
      App_EnterSafeState(FAULT_LED_TASK_TIMEOUT);
    }
    else if ((nowTick - lcdHeartbeatTick) > LCD_TASK_TIMEOUT_MS)
    {
      App_EnterSafeState(FAULT_LCD_TASK_TIMEOUT);
    }
    else if ((monitorSensorStackFree < MIN_STACK_FREE_WORDS) ||
             (monitorSafetyStackFree < MIN_STACK_FREE_WORDS) ||
             (monitorGateStackFree < MIN_STACK_FREE_WORDS) ||
             (monitorLedStackFree < MIN_STACK_FREE_WORDS) ||
             (monitorLcdStackFree < MIN_STACK_FREE_WORDS))
    {
      App_EnterSafeState(FAULT_STACK_LOW);
    }

    osDelay(100);
  }
}

#ifndef APP_TASKS_H
#define APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "parking_logic.h"
#include <stdint.h>

/* ===================== TASK HANDLE ===================== */
/*
  defaultTaskHandle tetap dari main.c bawaan CubeMX.
  Enam task aplikasi dibuat di app_tasks.c:
  SensorTask, SafetyTask, GateTask, LedTask, LcdTask, MonitorTask.
*/
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t sensorTaskHandle;
extern osThreadId_t safetyTaskHandle;
extern osThreadId_t gateTaskHandle;
extern osThreadId_t ledTaskHandle;
extern osThreadId_t lcdTaskHandle;
extern osThreadId_t monitorTaskHandle;

/* ===================== APP INIT ===================== */
void AppTasks_Init(void);

/* ===================== SHARED STATE GETTER ===================== */
ParkingSensorData AppTasks_GetSensorSnapshot(void);
ParkingDecision AppTasks_GetDecisionSnapshot(void);
uint8_t AppTasks_GetGateOpen(void);

/* ===================== ISR DEFERRED PROCESSING ===================== */
/*
  Dipanggil dari HAL_GPIO_EXTI_Callback() ketika IR_EXIT PA3 memicu EXTI3.
  ISR hanya memberi sinyal ke SensorTask melalui semaphore.
*/
void AppTasks_ExitIrqCallbackFromISR(void);

#ifdef __cplusplus
}
#endif

#endif

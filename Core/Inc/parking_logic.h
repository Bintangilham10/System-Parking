#ifndef PARKING_LOGIC_H
#define PARKING_LOGIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ===================== PARKING SETTING ===================== */
#define GATE_HOLD_MS      2000U

/* IR aktif LOW: LOW = objek terdeteksi */
#define IR_ACTIVE_LOW     1

/* ===================== SOFTWARE WATCHDOG / FAULT CODE ===================== */
typedef enum
{
  FAULT_NONE = 0,
  FAULT_SENSOR_TASK_TIMEOUT,
  FAULT_SAFETY_TASK_TIMEOUT,
  FAULT_GATE_TASK_TIMEOUT,
  FAULT_LED_TASK_TIMEOUT,
  FAULT_LCD_TASK_TIMEOUT,
  FAULT_STACK_LOW,
  FAULT_SIMULATION
} SystemFaultCode;

/* ===================== SENSOR DATA ===================== */
typedef struct
{
  uint8_t slot1Occupied;
  uint8_t slot2Occupied;
  uint8_t entryDetected;
  uint8_t exitDetected;
  uint8_t safetyDetected;
} ParkingSensorData;

/* ===================== SAFETY DECISION ===================== */
typedef struct
{
  uint8_t parkingFull;
  uint8_t gateBlocked;
  uint8_t allowGateOpen;
  uint8_t holdGateOpen;
  uint8_t systemSafe;
} ParkingDecision;

void ParkingLogic_Evaluate(const ParkingSensorData *sensor,
                           uint8_t gateOpen,
                           ParkingDecision *decision);

#ifdef __cplusplus
}
#endif

#endif

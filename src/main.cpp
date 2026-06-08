#include <Arduino.h>
#include <Wire.h>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
// ========================= PIN MAP =========================

static constexpr uint8_t PIN_SLOT1 = 32;
static constexpr uint8_t PIN_SLOT2 = 33;
static constexpr uint8_t PIN_ENTRY = 25;
static constexpr uint8_t PIN_EXIT = 26;
static constexpr uint8_t PIN_FAULT = 27;
static constexpr uint8_t PIN_RESET = 14;

static constexpr uint8_t PIN_ULTRASONIC_TRIG = 5;
static constexpr uint8_t PIN_ULTRASONIC_ECHO = 18;
static constexpr uint8_t PIN_SERVO = 19;
static constexpr uint8_t PIN_BUZZER = 23;
static constexpr uint8_t PIN_LED_GREEN = 4;
static constexpr uint8_t PIN_LED_RED = 2;

static constexpr uint8_t PIN_I2C_SDA = 21;
static constexpr uint8_t PIN_I2C_SCL = 22;

// ========================= LCD I2C =========================

static constexpr uint8_t LCD_ADDR = 0x27;
static constexpr uint8_t LCD_RS = 0x01;
static constexpr uint8_t LCD_EN = 0x04;
static constexpr uint8_t LCD_BACKLIGHT = 0x08;

// ========================= SERVO =========================

static constexpr uint8_t SERVO_CHANNEL = 0;
static constexpr uint16_t SERVO_FREQ_HZ = 50;
static constexpr uint8_t SERVO_RESOLUTION_BITS = 16;
static constexpr uint16_t SERVO_CLOSED_US = 1000;
static constexpr uint16_t SERVO_OPEN_US = 2000;

// ========================= SAFETY CONFIG =========================

static constexpr uint8_t TOTAL_SLOTS = 2;
static constexpr uint16_t GATE_BLOCK_DISTANCE_CM = 50;
static constexpr uint16_t ULTRASONIC_MAX_CM = 400;
static constexpr uint32_t ULTRASONIC_TIMEOUT_US = 30000;
static constexpr uint32_t SENSOR_STALE_TIMEOUT_MS = 800;
static constexpr uint32_t GATE_HOLD_OPEN_MS = 1500;

static constexpr EventBits_t EVT_ENTRY = BIT0;
static constexpr EventBits_t EVT_EXIT = BIT1;
static constexpr EventBits_t EVT_FULL = BIT2;
static constexpr EventBits_t EVT_BLOCKED = BIT3;
static constexpr EventBits_t EVT_FAULT = BIT4;

enum SafetyStatus : uint8_t {
  SAFETY_SAFE,
  SAFETY_BLOCKED,
  SAFETY_FAULT
};

enum GateState : uint8_t {
  GATE_CLOSED,
  GATE_OPEN
};

struct SensorSnapshot {
  bool slot1Occupied;
  bool slot2Occupied;
  bool carAtEntry;
  bool carAtExit;
  bool forceFault;
  bool resetFault;
  bool ultrasonicOk;
  uint16_t distanceCm;
  uint8_t freeSlots;
  bool parkingFull;
  uint32_t timestampMs;
};

struct SystemState {
  SensorSnapshot sensor;
  SafetyStatus safety;
  GateState gate;
  bool alarmOn;
  char faultReason[40];
  uint32_t lastSensorMs;
};

static QueueHandle_t sensorQueue;
static SemaphoreHandle_t stateMutex;
static EventGroupHandle_t systemEvents;
static SystemState state;

// ========================= LCD LOW LEVEL =========================

static void lcdExpanderWrite(uint8_t data) {
  Wire.beginTransmission(LCD_ADDR);
  Wire.write(data | LCD_BACKLIGHT);
  Wire.endTransmission();
}

static void lcdPulseEnable(uint8_t data) {
  lcdExpanderWrite(data | LCD_EN);
  delayMicroseconds(1);
  lcdExpanderWrite(data & ~LCD_EN);
  delayMicroseconds(50);
}

static void lcdWrite4Bits(uint8_t value) {
  lcdExpanderWrite(value);
  lcdPulseEnable(value);
}

static void lcdSend(uint8_t value, uint8_t mode) {
  lcdWrite4Bits((value & 0xF0) | mode);
  lcdWrite4Bits(((value << 4) & 0xF0) | mode);
}

static void lcdCommand(uint8_t value) {
  lcdSend(value, 0);
  if (value == 0x01 || value == 0x02) {
    delay(2);
  }
}

static void lcdWriteChar(char c) {
  lcdSend(static_cast<uint8_t>(c), LCD_RS);
}

static void lcdPrintLine(uint8_t row, const char *text) {
  lcdCommand(row == 0 ? 0x80 : 0xC0);

  uint8_t i = 0;
  for (; i < 16 && text[i] != '\0'; i++) {
    lcdWriteChar(text[i]);
  }
  for (; i < 16; i++) {
    lcdWriteChar(' ');
  }
}

static void lcdInit() {
  delay(80);
  lcdWrite4Bits(0x30);
  delay(5);
  lcdWrite4Bits(0x30);
  delayMicroseconds(150);
  lcdWrite4Bits(0x30);
  delay(5);
  lcdWrite4Bits(0x20);

  lcdCommand(0x28); // 4-bit, 2 lines, 5x8 font
  lcdCommand(0x0C); // display on, cursor off
  lcdCommand(0x01); // clear
  lcdCommand(0x06); // entry mode
}

// ========================= HELPERS =========================

static bool pressed(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

static const char *safetyText(SafetyStatus status) {
  switch (status) {
    case SAFETY_SAFE:
      return "SAFE";
    case SAFETY_BLOCKED:
      return "BLOCKED";
    case SAFETY_FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

static const char *gateText(GateState gate) {
  return gate == GATE_OPEN ? "OPEN" : "CLOSED";
}

static uint32_t servoDutyFromMicros(uint16_t pulseUs) {
  const uint32_t maxDuty = (1UL << SERVO_RESOLUTION_BITS) - 1;
  return static_cast<uint32_t>((static_cast<uint64_t>(pulseUs) * maxDuty) / 20000ULL);
}

static void setServoPulse(uint16_t pulseUs) {
  const uint32_t duty = servoDutyFromMicros(pulseUs);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO, duty);
#else
  ledcWrite(SERVO_CHANNEL, duty);
#endif
}

static void setupServoPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_SERVO, SERVO_FREQ_HZ, SERVO_RESOLUTION_BITS);
#else
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ_HZ, SERVO_RESOLUTION_BITS);
  ledcAttachPin(PIN_SERVO, SERVO_CHANNEL);
#endif
}

static uint16_t readUltrasonicCm(bool &ok) {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

  const uint32_t duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    ok = false;
    return 0;
  }

  const uint32_t distance = duration / 58;
  ok = distance > 0 && distance <= ULTRASONIC_MAX_CM;
  return static_cast<uint16_t>(distance > ULTRASONIC_MAX_CM ? ULTRASONIC_MAX_CM : distance);
}

static void ledSelfTest() {
  for (uint8_t i = 0; i < 2; i++) {
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_RED, LOW);
    delay(250);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, HIGH);
    delay(250);
  }

  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
}

static void copyState(SystemState &out) {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  out = state;
  xSemaphoreGive(stateMutex);
}

static void updateEventBits(const SensorSnapshot &snapshot, SafetyStatus safety) {
  EventBits_t bits = 0;

  if (snapshot.carAtEntry) {
    bits |= EVT_ENTRY;
  }
  if (snapshot.carAtExit) {
    bits |= EVT_EXIT;
  }
  if (snapshot.parkingFull) {
    bits |= EVT_FULL;
  }
  if (safety == SAFETY_BLOCKED) {
    bits |= EVT_BLOCKED;
  }
  if (safety == SAFETY_FAULT) {
    bits |= EVT_FAULT;
  }

  xEventGroupClearBits(systemEvents, EVT_ENTRY | EVT_EXIT | EVT_FULL | EVT_BLOCKED | EVT_FAULT);
  xEventGroupSetBits(systemEvents, bits);
}

static void announceTask(const char *name) {
  Serial.printf("[RTOS] %-14s core=%d priority=%u\n",
                name,
                xPortGetCoreID(),
                static_cast<unsigned>(uxTaskPriorityGet(nullptr)));
}

// ========================= TASKS =========================

static void taskSensorRead(void *param) {
  (void)param;
  announceTask("SensorRead");

  bool hasPrevious = false;
  SensorSnapshot previous{};

  for (;;) {
    SensorSnapshot snapshot{};

    snapshot.slot1Occupied = pressed(PIN_SLOT1);
    snapshot.slot2Occupied = pressed(PIN_SLOT2);
    snapshot.carAtEntry = pressed(PIN_ENTRY);
    snapshot.carAtExit = pressed(PIN_EXIT);
    snapshot.forceFault = pressed(PIN_FAULT);
    snapshot.resetFault = pressed(PIN_RESET);
    snapshot.distanceCm = readUltrasonicCm(snapshot.ultrasonicOk);
    snapshot.freeSlots = TOTAL_SLOTS;

    if (snapshot.slot1Occupied) {
      snapshot.freeSlots--;
    }
    if (snapshot.slot2Occupied) {
      snapshot.freeSlots--;
    }

    snapshot.parkingFull = snapshot.freeSlots == 0;
    snapshot.timestampMs = millis();

    if (!hasPrevious ||
        snapshot.slot1Occupied != previous.slot1Occupied ||
        snapshot.slot2Occupied != previous.slot2Occupied ||
        snapshot.carAtEntry != previous.carAtEntry ||
        snapshot.carAtExit != previous.carAtExit ||
        snapshot.forceFault != previous.forceFault ||
        snapshot.resetFault != previous.resetFault) {
      Serial.printf("[BUTTON] slot1=%d slot2=%d entry=%d exit=%d fault=%d reset=%d -> free=%u full=%d\n",
                    snapshot.slot1Occupied,
                    snapshot.slot2Occupied,
                    snapshot.carAtEntry,
                    snapshot.carAtExit,
                    snapshot.forceFault,
                    snapshot.resetFault,
                    static_cast<unsigned>(snapshot.freeSlots),
                    snapshot.parkingFull);
      previous = snapshot;
      hasPrevious = true;
    }

    xQueueOverwrite(sensorQueue, &snapshot);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void taskSafetyMonitor(void *param) {
  (void)param;
  announceTask("SafetyMonitor");

  bool faultLatched = false;
  char latchedReason[40] = "No fault";
  SensorSnapshot lastSnapshot{};
  uint32_t lastReceiveMs = millis();

  for (;;) {
    SensorSnapshot snapshot{};
    const BaseType_t gotData = xQueueReceive(sensorQueue, &snapshot, pdMS_TO_TICKS(100));

    if (gotData == pdTRUE) {
      lastSnapshot = snapshot;
      lastReceiveMs = millis();

      if (snapshot.resetFault && snapshot.ultrasonicOk && !snapshot.forceFault) {
        faultLatched = false;
        snprintf(latchedReason, sizeof(latchedReason), "No fault");
      }

      if (snapshot.forceFault) {
        faultLatched = true;
        snprintf(latchedReason, sizeof(latchedReason), "Manual fault button");
      } else if (!snapshot.ultrasonicOk) {
        faultLatched = true;
        snprintf(latchedReason, sizeof(latchedReason), "Ultrasonic timeout");
      }
    } else if (millis() - lastReceiveMs > SENSOR_STALE_TIMEOUT_MS) {
      faultLatched = true;
      snprintf(latchedReason, sizeof(latchedReason), "Sensor task stale");
    }

    SafetyStatus safety = SAFETY_SAFE;
    if (faultLatched) {
      safety = SAFETY_FAULT;
    } else if (lastSnapshot.ultrasonicOk && lastSnapshot.distanceCm < GATE_BLOCK_DISTANCE_CM) {
      safety = SAFETY_BLOCKED;
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state.sensor = lastSnapshot;
    state.safety = safety;
    state.lastSensorMs = lastReceiveMs;
    snprintf(state.faultReason, sizeof(state.faultReason), "%s", latchedReason);
    xSemaphoreGive(stateMutex);

    updateEventBits(lastSnapshot, safety);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

static void taskGateControl(void *param) {
  (void)param;
  announceTask("GateControl");

  GateState currentGate = GATE_CLOSED;
  uint32_t openedAtMs = 0;
  setServoPulse(SERVO_CLOSED_US);

  for (;;) {
    SystemState local{};
    copyState(local);

    const bool entryAllowed = local.sensor.carAtEntry && !local.sensor.parkingFull;
    const bool exitAllowed = local.sensor.carAtExit;
    const bool requestOpen = entryAllowed || exitAllowed;
    const bool gateAreaBlocked = local.safety == SAFETY_BLOCKED;
    const bool fault = local.safety == SAFETY_FAULT;
    const bool holdOpenTime = millis() - openedAtMs < GATE_HOLD_OPEN_MS;
    const bool allClear = !local.sensor.carAtEntry && !local.sensor.carAtExit && !gateAreaBlocked && !fault;

    GateState nextGate = currentGate;

    if (requestOpen || gateAreaBlocked || fault) {
      nextGate = GATE_OPEN;
    } else if (currentGate == GATE_OPEN && allClear && !holdOpenTime) {
      nextGate = GATE_CLOSED;
    }

    if (nextGate != currentGate) {
      currentGate = nextGate;
      if (currentGate == GATE_OPEN) {
        openedAtMs = millis();
        setServoPulse(SERVO_OPEN_US);
      } else {
        setServoPulse(SERVO_CLOSED_US);
      }
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state.gate = currentGate;
    xSemaphoreGive(stateMutex);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void taskAlarm(void *param) {
  (void)param;
  announceTask("Alarm");

  for (;;) {
    SystemState local{};
    copyState(local);

    const bool alarm = local.sensor.parkingFull || local.safety == SAFETY_FAULT;
    const bool greenLed = (local.gate == GATE_OPEN || local.sensor.carAtEntry || local.sensor.carAtExit) &&
                          local.safety != SAFETY_FAULT;
    const bool redLed = local.gate == GATE_CLOSED || local.sensor.parkingFull ||
                        local.safety == SAFETY_FAULT || local.safety == SAFETY_BLOCKED;

    digitalWrite(PIN_BUZZER, alarm ? HIGH : LOW);
    digitalWrite(PIN_LED_GREEN, greenLed ? HIGH : LOW);
    digitalWrite(PIN_LED_RED, redLed ? HIGH : LOW);

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state.alarmOn = alarm;
    xSemaphoreGive(stateMutex);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void taskDisplay(void *param) {
  (void)param;
  announceTask("Display");

  char line1[17];
  char line2[17];

  for (;;) {
    SystemState local{};
    copyState(local);

    snprintf(line1, sizeof(line1), "Slot:%u Gate:%s",
             static_cast<unsigned>(local.sensor.freeSlots),
             local.gate == GATE_OPEN ? "OPEN" : "CLS");

    if (local.safety == SAFETY_FAULT) {
      snprintf(line2, sizeof(line2), "FAULT RESET?");
    } else {
      snprintf(line2, sizeof(line2), "%s D:%ucm",
               safetyText(local.safety),
               static_cast<unsigned>(local.sensor.distanceCm));
    }

    lcdPrintLine(0, line1);
    lcdPrintLine(1, line2);

    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

static void taskLogger(void *param) {
  (void)param;
  announceTask("Logger");

  for (;;) {
    SystemState local{};
    copyState(local);
    const EventBits_t bits = xEventGroupGetBits(systemEvents);

    Serial.printf("[STATE] slots=%u full=%d entry=%d exit=%d dist=%ucm safety=%s gate=%s alarm=%d events=0x%02X fault=\"%s\"\n",
                  static_cast<unsigned>(local.sensor.freeSlots),
                  local.sensor.parkingFull,
                  local.sensor.carAtEntry,
                  local.sensor.carAtExit,
                  static_cast<unsigned>(local.sensor.distanceCm),
                  safetyText(local.safety),
                  gateText(local.gate),
                  local.alarmOn,
                  static_cast<unsigned>(bits),
                  local.faultReason);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ========================= ARDUINO ENTRYPOINT =========================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("ESP32 FreeRTOS Smart Parking Simulation");

  pinMode(PIN_SLOT1, INPUT_PULLUP);
  pinMode(PIN_SLOT2, INPUT_PULLUP);
  pinMode(PIN_ENTRY, INPUT_PULLUP);
  pinMode(PIN_EXIT, INPUT_PULLUP);
  pinMode(PIN_FAULT, INPUT_PULLUP);
  pinMode(PIN_RESET, INPUT_PULLUP);

  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, HIGH);
  ledSelfTest();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  lcdInit();
  lcdPrintLine(0, "SMART PARKING");
  lcdPrintLine(1, "FreeRTOS ESP32");

  setupServoPwm();
  setServoPulse(SERVO_CLOSED_US);

  sensorQueue = xQueueCreate(1, sizeof(SensorSnapshot));
  stateMutex = xSemaphoreCreateMutex();
  systemEvents = xEventGroupCreate();

  if (sensorQueue == nullptr || stateMutex == nullptr || systemEvents == nullptr) {
    Serial.println("[FAULT] Failed to create RTOS primitives");
    while (true) {
      digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
      delay(200);
    }
  }

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  state.safety = SAFETY_SAFE;
  state.gate = GATE_CLOSED;
  state.alarmOn = false;
  state.sensor.freeSlots = TOTAL_SLOTS;
  snprintf(state.faultReason, sizeof(state.faultReason), "No fault");
  xSemaphoreGive(stateMutex);

  xTaskCreatePinnedToCore(taskSensorRead, "SensorRead", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskSafetyMonitor, "SafetyMonitor", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskGateControl, "GateControl", 4096, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(taskAlarm, "Alarm", 2048, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(taskDisplay, "Display", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskLogger, "Logger", 4096, nullptr, 1, nullptr, 0);

  Serial.println("[BOOT] FreeRTOS tasks created");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * =============================================================================
 *  V O X B O T  —  SELF-BALANCING ROBOT  |  ENCODER-FREE BUILD
 *  Firmware v2.0  |  ESP32 Dual-Core FreeRTOS
 * =============================================================================
 *  Core 0 — BLE Server  (receives NLP string commands asynchronously)
 *  Core 1 — 200Hz Single-Loop Angle PID  (IMU → DRV8833 PWM)
 *
 *  Removed from v1.0:
 *    ✕ Encoder ISRs (GPIO 34, 35, 36, 39)
 *    ✕ Inner Speed PID loop
 *    ✕ Encoder tick snapshot / portDISABLE_INTERRUPTS
 *
 *  Architecture change:
 *    Angle PID output → directly drives motor PWM (no speed feedback wrapper)
 *    Locomotion achieved by shifting the balance setpoint (lean-to-move)
 *
 *  BLE commands accepted (plain UTF-8 strings, no newline needed):
 *    "DRIVE_FORWARD"   "DRIVE_BACKWARD"
 *    "TURN_LEFT"       "TURN_RIGHT"
 *    "STOP"            "SPEED:0.75"    (custom ±1.0 float)
 *
 *  Arduino IDE Board   : "ESP32 Dev Module"
 *  Required libraries  : ESP32 BLE Arduino (built-in with esp32 core)
 *                        Wire              (built-in)
 * =============================================================================
 */

// ── Includes ────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Wire.h>
#include <esp32-hal-ledc.h>
#include <math.h>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// ── Version detection ───────────────────────────────────────────────────────
// ESP32 Arduino core 3.x removed ledcSetup()/ledcAttachPin() and changed
// ledcWrite() to take a pin number instead of a channel number.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define USE_NEW_LEDC_API 1
#else
  #define USE_NEW_LEDC_API 0
#endif

// =============================================================================
//  SECTION 1 — PIN MAP
// =============================================================================

// ── I²C
// ───────────────────────────────────────────────────────────────────────
#define PIN_SDA 21
#define PIN_SCL 22

// ── DRV8833 Motor Driver
// ──────────────────────────────────────────────────────
#define PIN_ML_IN1 25 // Left  motor — forward
#define PIN_ML_IN2 26 // Left  motor — reverse
#define PIN_MR_IN3 27 // Right motor — forward
#define PIN_MR_IN4 14 // Right motor — reverse

// =============================================================================
//  SECTION 2 — LEDC PWM CONFIGURATION
// =============================================================================

#define LEDC_FREQ_HZ 20000 // 20kHz — inaudible, kills N20 motor whine
#define LEDC_RESOLUTION 8  // 8-bit: duty range 0–255

#define LEDC_CH_ML_FWD 0
#define LEDC_CH_ML_REV 1
#define LEDC_CH_MR_FWD 2
#define LEDC_CH_MR_REV 3

// =============================================================================
//  SECTION 3 — PID TUNING CONSTANTS
// =============================================================================
//
//  ┌─────────────────────────────────────────────────────────────────────────┐
//  │  600 RPM N20 TUNING GUIDE (no encoders)                                 │
//  │                                                                         │
//  │  Without speed feedback, the angle PID output IS the PWM directly.     │
//  │  This means Kd is your only brake — it must be higher than usual.      │
//  │                                                                         │
//  │  STEP 1 — Set Kp=20, Ki=0, Kd=0                                        │
//  │           Hold robot in hand. Verify motors react to tilt direction.    │
//  │           Increase Kp until robot starts to weakly balance (shaky OK). │
//  │                                                                         │
//  │  STEP 2 — Increase Kd until oscillation damps out.                     │
//  │           Target: robot corrects tilt and holds steady.                │
//  │           Typical Kd = Kp × 0.08 to 0.12 for these motors.            │
//  │                                                                         │
//  │  STEP 3 — Add Ki ONLY if robot drifts slowly in one direction at rest. │
//  │           Start at Ki=0.3. Never exceed Ki=2.0 without anti-windup.    │
//  │                                                                         │
//  │  STEP 4 — Trim BALANCE_ANGLE_DEG until robot stands without drifting.  │
//  │           Adjust in 0.2° steps. Positive = lean forward.              │
//  └─────────────────────────────────────────────────────────────────────────┘

#define ANGLE_KP 22.0f
#define ANGLE_KI 0.6f
#define ANGLE_KD 1.8f
#define ANGLE_I_CLAMP 60.0f // Anti-windup: clamps integral accumulation

// ── Balance geometry
// ──────────────────────────────────────────────────────────
#define BALANCE_ANGLE_DEG 0.0f // Trim ±0.2° at a time until no idle drift
#define MAX_LEAN_DEG 3.5f      // Max setpoint shift for full speed command
#define MAX_TURN_PWM 55        // Max differential PWM for turning
#define FALL_ANGLE_DEG 40.0f   // Cut motors — robot has fallen

// ── 600 RPM Motor PWM Compensation ───────────────────────────────────────────
#define PWM_DEADBAND_MIN 15 // Jump-start PWM to overcome gearbox friction
#define PWM_MAX 200         // Hard ceiling — prevents DRV8833 stall spikes

// =============================================================================
//  SECTION 4 — COMPLEMENTARY FILTER
// =============================================================================
//
//  With hardware DLPF at 21Hz (register 0x04), accel is pre-smoothed.
//  0.95 alpha = 95% gyro (fast, low-latency) + 5% accel (long-term drift fix).
//  Increase alpha toward 0.98 if robot feels "wobbly" on motor vibration.
//  Decrease toward 0.90 if pitch drifts over 10–15 seconds.

#define CF_ALPHA 0.95f

// =============================================================================
//  SECTION 5 — TIMING
// =============================================================================

#define BALANCE_PERIOD_US 5000 // 200Hz control loop = 5ms per tick

// =============================================================================
//  SECTION 6 — MPU6050 REGISTERS
// =============================================================================

#define MPU_ADDR 0x68
#define MPU_REG_SMPLRT_DIV 0x19
#define MPU_REG_CONFIG 0x1A // DLPF register
#define MPU_REG_GYRO_CFG 0x1B
#define MPU_REG_ACCEL_CFG 0x1C
#define MPU_REG_ACCEL_OUT 0x3B // Start of 14-byte burst
#define MPU_REG_PWR_MGMT 0x6B
#define MPU_REG_WHOAMI 0x75

#define ACCEL_SCALE 16384.0f // ±2g  full-scale
#define GYRO_SCALE 131.0f    // ±250°/s full-scale

// =============================================================================
//  SECTION 7 — BLE IDENTIFIERS
// =============================================================================
//
//  Copy these UUIDs into your NLP app / frontend exactly as written.
//  The Command Characteristic is WRITE + WRITE_NR (no response = lower
//  latency).

#define BLE_DEVICE_NAME "VoxBot"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CMD_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_STATUS_UUID "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e" // Notify

// =============================================================================
//  SECTION 8 — IPC: COMMAND STRUCTURE & QUEUE
// =============================================================================
//
//  DriveCommand flows ONE WAY: Core 0 (BLE write) → Core 1 (balance loop).
//  Queue depth 4: allows burst of rapid commands without blocking BLE stack.
//  xQueueSend timeout = 0: NEVER blocks Core 0. If full, drop oldest.

struct DriveCommand {
  float linearVelocity;  // -1.0 (full back) … +1.0 (full forward)
  float angularVelocity; // -1.0 (full left) … +1.0 (full right)
};

#define CMD_QUEUE_DEPTH 4
static QueueHandle_t xCmdQueue = nullptr;

// =============================================================================
//  SECTION 9 — GLOBAL TELEMETRY  (written by Core 1, read by Core 0)
// =============================================================================
//
//  Declared volatile — these cross the core boundary.
//  Single float reads/writes are atomic on Xtensa LX6; no mutex needed.

static volatile float g_telPitch = 0.0f;
static volatile float g_telPWM = 0.0f;
static volatile bool g_bleConnected = false;

// Notify characteristic pointer (set once in bleSetup, read in telemetry task)
static BLECharacteristic *g_pStatusChar = nullptr;

// =============================================================================
//  FORWARD DECLARATIONS
//  Arduino IDE's preprocessor auto-generates prototypes, but it can't handle
//  user-defined types in function signatures correctly unless the type is
//  already in scope. Providing explicit prototypes here prevents the error.
// =============================================================================
struct ImuData;                            // forward-declare the struct
static bool mpuRead(ImuData &out);         // explicit prototype with correct type
static bool mpuInit();

// =============================================================================
//  SECTION 10 — PID CONTROLLER
// =============================================================================

class PIDController {
public:
  PIDController(float kp, float ki, float kd, float iClamp)
      : _kp(kp), _ki(ki), _kd(kd), _iClamp(iClamp), _integral(0.f),
        _prevError(0.f) {}

  float compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // Integral with anti-windup clamp
    _integral += error * dt;
    _integral = constrain(_integral, -_iClamp, _iClamp);

    // Derivative (on error, not measurement — avoids setpoint-kick)
    float deriv = (error - _prevError) / dt;
    _prevError = error;

    return (_kp * error) + (_ki * _integral) + (_kd * deriv);
  }

  void reset() {
    _integral = 0.f;
    _prevError = 0.f;
  }

private:
  float _kp, _ki, _kd, _iClamp;
  float _integral, _prevError;
};

// =============================================================================
//  SECTION 11 — MOTOR DRIVER HELPERS
// =============================================================================

static void initMotorPWM() {
  const int pins[4] = {PIN_ML_IN1, PIN_ML_IN2, PIN_MR_IN3, PIN_MR_IN4};
  const int chs[4]  = {LEDC_CH_ML_FWD, LEDC_CH_ML_REV, LEDC_CH_MR_FWD,
                       LEDC_CH_MR_REV};
  for (int i = 0; i < 4; i++) {
#if USE_NEW_LEDC_API
    // Core 3.x: ledcAttachChannel(pin, freq, resolution, channel)
    ledcAttachChannel(pins[i], LEDC_FREQ_HZ, LEDC_RESOLUTION, chs[i]);
    ledcWrite(pins[i], 0);
#else
    // Core 2.x: ledcSetup(channel, freq, resolution) + ledcAttachPin(pin, channel)
    ledcSetup(chs[i], LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcAttachPin(pins[i], chs[i]);
    ledcWrite(chs[i], 0);
#endif
  }
}

//
//  applyDeadband — critical for encoder-free 600 RPM N20 motors.
//
//  Without speed feedback, the only tool against gearbox stiction is this:
//    • |raw| < 2       → true zero  (robot intentionally stopped)
//    • 2 ≤ |raw|       → floor to PWM_DEADBAND_MIN (overcome static friction)
//    • ceiling at PWM_MAX → prevents DRV8833 overcurrent on direction reversal
//
static int applyDeadband(float raw) {
  if (fabsf(raw) < 2.0f)
    return 0;
  int sign = (raw > 0.0f) ? 1 : -1;
  int pwm = (int)fabsf(raw);
  pwm = constrain(pwm, PWM_DEADBAND_MIN, PWM_MAX);
  return sign * pwm;
}

// Helper: write to the correct target (pin for 3.x, channel for 2.x)
static inline void pwmWrite(int ch, int pin, int duty) {
#if USE_NEW_LEDC_API
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

static void driveMotors(int leftPWM, int rightPWM) {
  // Left motor
  if (leftPWM >= 0) {
    pwmWrite(LEDC_CH_ML_FWD, PIN_ML_IN1, leftPWM);
    pwmWrite(LEDC_CH_ML_REV, PIN_ML_IN2, 0);
  } else {
    pwmWrite(LEDC_CH_ML_FWD, PIN_ML_IN1, 0);
    pwmWrite(LEDC_CH_ML_REV, PIN_ML_IN2, -leftPWM);
  }
  // Right motor
  if (rightPWM >= 0) {
    pwmWrite(LEDC_CH_MR_FWD, PIN_MR_IN3, rightPWM);
    pwmWrite(LEDC_CH_MR_REV, PIN_MR_IN4, 0);
  } else {
    pwmWrite(LEDC_CH_MR_FWD, PIN_MR_IN3, 0);
    pwmWrite(LEDC_CH_MR_REV, PIN_MR_IN4, -rightPWM);
  }
}

static void stopMotors() {
  pwmWrite(LEDC_CH_ML_FWD, PIN_ML_IN1, 0);
  pwmWrite(LEDC_CH_ML_REV, PIN_ML_IN2, 0);
  pwmWrite(LEDC_CH_MR_FWD, PIN_MR_IN3, 0);
  pwmWrite(LEDC_CH_MR_REV, PIN_MR_IN4, 0);
}

// =============================================================================
//  SECTION 12 — MPU6050 DRIVER
// =============================================================================

// Declare ImuData BEFORE mpuInit/mpuRead so Arduino's auto-prototype generator
// sees the type before it creates forward declarations for mpuRead.
struct ImuData {
  float ax, ay, az; // g
  float gx, gy, gz; // degrees/s
};

// Read one big-endian 16-bit word from Wire — explicit helper avoids lambda
// issues with the Arduino IDE preprocessor.
static inline int16_t wireRead16() {
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

static bool mpuInit() {
  // ── WHO_AM_I probe ────────────────────────────────────────────────────────
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_WHOAMI);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  if (!Wire.available())
    return false;
  uint8_t id = Wire.read();
  if (id != 0x68 && id != 0x98)
    return false;

  // ── Wake up ───────────────────────────────────────────────────────────────
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_PWR_MGMT);
  Wire.write(0x00); // Clear SLEEP bit
  Wire.endTransmission();
  delay(10);

  // ── Sample rate = 200Hz  (1kHz internal / (1 + 4)) ───────────────────────
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_SMPLRT_DIV);
  Wire.write(0x04);
  Wire.endTransmission();

  // ── DLPF = 0x04 -> Accel BW 21Hz | Gyro BW 20Hz ─────────────────────────
  //   Removes motor vibration noise BEFORE the ADC.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_CONFIG);
  Wire.write(0x04);
  Wire.endTransmission();

  // ── Gyro full-scale: ±250 deg/s (highest resolution) ─────────────────────
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_GYRO_CFG);
  Wire.write(0x00);
  Wire.endTransmission();

  // ── Accel full-scale: ±2g ────────────────────────────────────────────────
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_CFG);
  Wire.write(0x00);
  Wire.endTransmission();

  return true;
}

// Single 14-byte burst: ACCEL(6) + TEMP(2) + GYRO(6)
static bool mpuRead(ImuData &out) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_OUT);
  if (Wire.endTransmission(false) != 0)
    return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14)
    return false;

  // No lambda — use explicit helper wireRead16() for IDE compatibility

  out.ax = wireRead16() / ACCEL_SCALE;
  out.ay = wireRead16() / ACCEL_SCALE;
  out.az = wireRead16() / ACCEL_SCALE;
  wireRead16(); // temperature — discard
  out.gx = wireRead16() / GYRO_SCALE;
  out.gy = wireRead16() / GYRO_SCALE;
  out.gz = wireRead16() / GYRO_SCALE;
  return true;
}

// =============================================================================
//  SECTION 13 — BLE COMMAND PARSER
// =============================================================================
//
//  NLP app / frontend should send one of these UTF-8 strings to
//  the Command Characteristic (UUID: CHAR_CMD_UUID).
//
//  ┌──────────────────┬──────────────────────────────────────────┐
//  │ String payload   │ Effect                                   │
//  ├──────────────────┼──────────────────────────────────────────┤
//  │ DRIVE_FORWARD    │ lean forward at 50% speed                │
//  │ DRIVE_BACKWARD   │ lean backward at 50% speed               │
//  │ TURN_LEFT        │ spin counterclockwise at 50%             │
//  │ TURN_RIGHT       │ spin clockwise at 50%                    │
//  │ STOP             │ zero all velocities                      │
//  │ SPEED:0.75       │ set custom forward speed (float ±1.0)    │
//  │ TURN:−0.4        │ set custom turn rate (float ±1.0)        │
//  └──────────────────┴──────────────────────────────────────────┘

static DriveCommand parseCommand(const std::string &s) {
  DriveCommand cmd = {0.0f, 0.0f};

  if (s == "DRIVE_FORWARD") {
    cmd.linearVelocity = 0.5f;
  } else if (s == "DRIVE_BACKWARD") {
    cmd.linearVelocity = -0.5f;
  } else if (s == "TURN_LEFT") {
    cmd.angularVelocity = -0.5f;
  } else if (s == "TURN_RIGHT") {
    cmd.angularVelocity = 0.5f;
  } else if (s == "STOP") { /* zeros already set */
  } else if (s.rfind("SPEED:", 0) == 0) {
    try {
      float v = std::stof(s.substr(6));
      cmd.linearVelocity = constrain(v, -1.0f, 1.0f);
    } catch (...) { /* malformed — zero safe */
    }
  } else if (s.rfind("TURN:", 0) == 0) {
    try {
      float v = std::stof(s.substr(5));
      cmd.angularVelocity = constrain(v, -1.0f, 1.0f);
    } catch (...) { /* malformed — zero safe */
    }
  }

  return cmd;
}

// =============================================================================
//  SECTION 14 — BLE SERVER CALLBACKS
// =============================================================================

class VoxBotServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    g_bleConnected = true;
    Serial.println("[BLE] Client connected.");
  }

  void onDisconnect(BLEServer *pServer) override {
    g_bleConnected = false;
    Serial.println(
        "[BLE] Client disconnected. Issuing STOP + restarting advertising.");

    // Safety: push a STOP command before going back to advertising
    DriveCommand stop = {0.0f, 0.0f};
    xQueueSend(xCmdQueue, &stop, 0);

    // Restart advertising so NLP app can reconnect
    BLEDevice::startAdvertising();
  }
};

class CommandCharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    // Core 3.x returns Arduino String; core 2.x returns std::string.
    // Convert to std::string via .c_str() for uniform handling.
    std::string payload = std::string(pChar->getValue().c_str());
    if (payload.empty())
      return;

    DriveCommand cmd = parseCommand(payload);

    // Non-blocking send — if queue is full, evict oldest and push new
    if (xQueueSend(xCmdQueue, &cmd, 0) != pdTRUE) {
      DriveCommand discard;
      xQueueReceive(xCmdQueue, &discard, 0);
      xQueueSend(xCmdQueue, &cmd, 0);
    }

    Serial.printf("[BLE->CMD]  '%s'   lin=%+.2f  ang=%+.2f\n", payload.c_str(),
                  cmd.linearVelocity, cmd.angularVelocity);
  }
};

static void bleSetup() {
  BLEDevice::init(BLE_DEVICE_NAME);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new VoxBotServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // ── Command Characteristic (WRITE + WRITE_NR) ─────────────────────────────
  //   WRITE_NR = Write Without Response → lower round-trip latency for NLP app
  BLECharacteristic *pCmdChar = pService->createCharacteristic(
      CHAR_CMD_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCmdChar->setCallbacks(new CommandCharCallbacks());

  // ── Status / Telemetry Characteristic (NOTIFY) ───────────────────────────
  //   Frontend can subscribe here to receive pitch + PWM in real-time.
  //   Format: "P:+1.23,W:+045"  (pitch degrees, base PWM)
  g_pStatusChar = pService->createCharacteristic(
      CHAR_STATUS_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  g_pStatusChar->addDescriptor(new BLE2902());

  pService->start();

  // ── Advertising ───────────────────────────────────────────────────────────
  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06); // helps iPhone BLE stack discover faster
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Device name  : VoxBot");
  Serial.println("[BLE] Service UUID : " SERVICE_UUID);
  Serial.println("[BLE] Command UUID : " CHAR_CMD_UUID);
  Serial.println("[BLE] Status  UUID : " CHAR_STATUS_UUID);
  Serial.println("[BLE] Advertising...");
}

// =============================================================================
//  SECTION 15 — CORE 0 TASK: Telemetry Broadcaster
// =============================================================================
//
//  Runs at 5Hz. Sends pitch + PWM to any BLE subscriber (frontend dashboard).
//  Also prints to Serial for desktop debugging.
//
//  This task intentionally uses vTaskDelay — it is NOT time-critical.

static void Task_Core0_Telemetry(void *pvParam) {
  Serial.println("[Core0] Telemetry task running.");
  char notifyBuf[32];

  for (;;) {
    float pitch = (float)g_telPitch;
    float pwm = (float)g_telPWM;

    // Serial monitor
    Serial.printf("[TEL]  pitch=%+6.2f°  pwm=%+5.0f  ble=%s\n", pitch, pwm,
                  g_bleConnected ? "CONNECTED" : "ADVERTISING");

    // BLE Notify → frontend (only if a client is subscribed)
    if (g_bleConnected && g_pStatusChar != nullptr) {
      snprintf(notifyBuf, sizeof(notifyBuf), "P:%+.2f,W:%+.0f", pitch, pwm);
      g_pStatusChar->setValue(notifyBuf);
      g_pStatusChar->notify();
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // 5Hz
  }
}

// =============================================================================
//  SECTION 15B — FALLBACK: DIRECT DRIVE TASK (no MPU / no balancing)
// =============================================================================
//
//  When the MPU6050 is absent or failed, the balance PID cannot run.
//  This task provides a simple command → motor mapping so BLE commands
//  still produce motor output.  No balancing, no PID — just direct PWM.
//
//  linearVelocity  → both motors at DIRECT_DRIVE_PWM × velocity
//  angularVelocity → differential drive (left/right opposite)
//

#define DIRECT_DRIVE_PWM 120  // Fixed PWM for direct-drive mode (0–200)

static void Task_DirectDrive(void *pvParam) {
  Serial.println("[Core1] Direct-drive task running (no MPU, no PID).");

  DriveCommand activeCmd = {0.0f, 0.0f};

  for (;;) {
    // Drain command queue — keep freshest
    DriveCommand tmp;
    while (xQueueReceive(xCmdQueue, &tmp, 0) == pdTRUE) {
      activeCmd = tmp;
      Serial.printf("[DD] Cmd: lin=%+.2f ang=%+.2f\n",
                    activeCmd.linearVelocity, activeCmd.angularVelocity);
    }

    // Convert velocities to motor PWM
    int basePWM = (int)(activeCmd.linearVelocity * (float)DIRECT_DRIVE_PWM);
    int turnOff = (int)(activeCmd.angularVelocity * (float)MAX_TURN_PWM);

    int leftPWM  = constrain(basePWM - turnOff, -PWM_MAX, PWM_MAX);
    int rightPWM = constrain(basePWM + turnOff, -PWM_MAX, PWM_MAX);

    driveMotors(leftPWM, rightPWM);

    // Update telemetry globals so BLE notifications still work
    g_telPitch = 0.0f;  // No IMU — report flat
    g_telPWM   = (float)basePWM;

    vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz — plenty for direct drive
  }
}

// =============================================================================
//  SECTION 16 — CORE 1 TASK: 200Hz ANGLE PID BALANCE LOOP
// =============================================================================
//
//  Per-tick pipeline (every 5ms):
//
//   1. Timing gate  — busy-wait until exactly 5ms has elapsed
//   2. Queue drain  — consume all pending commands, keep freshest
//   3. IMU read     — 14-byte I²C burst (accel + gyro)
//   4. Comp filter  — fuse gyro integral with accel atan2 angle
//   5. Fall check   — cut motors if |pitch| > 40°
//   6. PID compute  — angle error → raw PWM output
//   7. Deadband     — jump-start compensation for gearbox stiction
//   8. Turn mixing  — differential left/right for angular velocity
//   9. Drive motors — ledc hardware PWM write

static void Task_Core1_Balance(void *pvParam) {
  Serial.println("[Core1] Balance task starting. Warm-up 500ms...");

  PIDController anglePID(ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_I_CLAMP);

  float cfAngle = 0.0f;
  DriveCommand activeCmd = {0.0f, 0.0f};

  // ── 500ms warm-up: pre-seed complementary filter ─────────────────────────
  //   Without this, the filter starts at 0° and spikes violently on first tick
  ImuData imu;
  for (int i = 0; i < 100; i++) {
    if (mpuRead(imu)) {
      float ap = atan2f(imu.ax, imu.az) * RAD_TO_DEG;
      cfAngle = CF_ALPHA * cfAngle + (1.0f - CF_ALPHA) * ap;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  Serial.printf("[Core1] Ready. Initial pitch: %+.2f°\n", cfAngle);

  int64_t lastUs = esp_timer_get_time();

  // ═════════════════════════════════════════════════════════════════════════
  for (;;) {

    // ── Step 1: Strict 200Hz timing gate ──────────────────────────────────
    int64_t now = esp_timer_get_time();
    int64_t dtUs = now - lastUs;

    if (dtUs < (int64_t)BALANCE_PERIOD_US) {
      vTaskDelay(1); // yield slice — feeds RTOS watchdog, never busy-loops
      continue;
    }
    lastUs = now;
    float dt = (float)dtUs * 1.0e-6f; // µs → seconds

    // ── Step 2: Drain command queue (always keep freshest) ────────────────
    DriveCommand tmp;
    while (xQueueReceive(xCmdQueue, &tmp, 0) == pdTRUE) {
      activeCmd = tmp;
    }

    // ── Step 3: IMU read ──────────────────────────────────────────────────
    if (!mpuRead(imu)) {
      // I²C hiccup — hold last output, retry next tick
      Serial.println("[Core1] WARN: IMU read failed. Holding.");
      continue;
    }

    // ── Step 4: Complementary filter ─────────────────────────────────────
    //
    //  AXIS CONVENTION (verify this first on your physical build):
    //    Robot leans FORWARD  → pitch goes POSITIVE
    //    Robot leans BACKWARD → pitch goes NEGATIVE
    //
    //  If your MPU6050 is mounted differently, you may need to swap
    //  imu.ax / imu.az, or negate imu.gy. Verify in Serial Monitor
    //  before attempting to balance.
    //
    float accelPitch = atan2f(imu.ax, imu.az) * RAD_TO_DEG;
    float gyroPitch = cfAngle + imu.gy * dt;
    cfAngle = CF_ALPHA * gyroPitch + (1.0f - CF_ALPHA) * accelPitch;
    g_telPitch = cfAngle;

    // ── Step 5: Fall detection ────────────────────────────────────────────
    if (fabsf(cfAngle) > FALL_ANGLE_DEG) {
      // Log when commands are being dropped due to fall
      if (activeCmd.linearVelocity != 0.0f || activeCmd.angularVelocity != 0.0f) {
        Serial.printf("[FALL] Cmd ignored (pitch=%+.1f°): lin=%+.2f ang=%+.2f\n",
                      cfAngle, activeCmd.linearVelocity, activeCmd.angularVelocity);
      }
      stopMotors();
      anglePID.reset();
      // Don't clear activeCmd — let the user's command persist
      // so the robot can attempt to act when picked back upright
      vTaskDelay(pdMS_TO_TICKS(200)); // brief pause avoids flooding I²C
      continue;
    }

    // ── Step 6: Angle PID ─────────────────────────────────────────────────
    //
    //  Lean-to-move: shifting the setpoint makes the robot fall slightly
    //  in the commanded direction, and the physics of the inverted pendulum
    //  converts that lean into forward/backward locomotion.
    //
    //  DRIVE_FORWARD (linearVelocity = +0.5)
    //    → angleSetpoint = 0° + (0.5 × 3.5°) = +1.75° (lean forward)
    //    → PID error = +1.75° − actual
    //    → positive PWM → wheels drive forward → robot chases its own lean
    //
    float angleSetpoint =
        BALANCE_ANGLE_DEG + (activeCmd.linearVelocity * MAX_LEAN_DEG);
    float rawPWM = anglePID.compute(angleSetpoint, cfAngle, dt);

    // ── Step 7: Deadband + clamp ──────────────────────────────────────────
    int basePWM = applyDeadband(rawPWM);
    g_telPWM = (float)basePWM;

    // ── Step 8: Turn mixing ───────────────────────────────────────────────
    int turnOff = (int)(activeCmd.angularVelocity * (float)MAX_TURN_PWM);
    int leftPWM = constrain(basePWM - turnOff, -PWM_MAX, PWM_MAX);
    int rightPWM = constrain(basePWM + turnOff, -PWM_MAX, PWM_MAX);

    // ── Step 9: Drive motors ──────────────────────────────────────────────
    driveMotors(leftPWM, rightPWM);
  }
}

// =============================================================================
//  SECTION 17 — SETUP & LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║   VoxBot  v2.0  |  Encoder-Free      ║");
  Serial.println("║   ESP32 Dual-Core FreeRTOS            ║");
  Serial.println("╚══════════════════════════════════════╝");

  // ── FreeRTOS Queue ─────────────────────────────────────────────────────────
  //   Created first — BLE callbacks write to it immediately on connect
  xCmdQueue = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(DriveCommand));
  if (xCmdQueue == NULL) {
    Serial.println("[FATAL] Failed to create command queue!");
    while (true) delay(1000);
  }
  Serial.println("[OK] Command Queue (depth=4, sizeof=8 bytes)");

  // ── BLE Server — started BEFORE MPU check so device is always discoverable ─
  bleSetup();

  // ── Motor PWM ──────────────────────────────────────────────────────────────
  initMotorPWM();
  Serial.println("[OK] DRV8833  20kHz PWM  deadband=15  clamp=200");

  // ── I²C @ 400kHz ──────────────────────────────────────────────────────────
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  bool mpuOk = mpuInit();
  if (!mpuOk) {
    Serial.println();
    Serial.println("[WARN] MPU6050 not detected on I2C bus.");
    Serial.println("       Balancing disabled — running in DIRECT DRIVE mode.");
    Serial.println("       Checklist:");
    Serial.println("         • SDA → GPIO 21    SCL → GPIO 22");
    Serial.println("         • 4.7kΩ pull-ups from SDA and SCL to 3.3V");
    Serial.println("         • VCC → 3.3V  (NOT 5V)");
    Serial.println("         • AD0 → GND   (sets address 0x68)");
    Serial.println("       Fix wiring and reboot for full PID balance mode.");
  }
  if (mpuOk) {
    Serial.println("[OK] MPU6050  DLPF=21Hz  ODR=200Hz  ±250°/s  ±2g");
  }

  // ── Core 0 Task: Telemetry (non-RT, low priority) ─────────────────────────
  //   Always created — even without MPU, it broadcasts BLE status to dashboard
  xTaskCreatePinnedToCore(Task_Core0_Telemetry, // function
                          "Telemetry", // name (visible in FreeRTOS debugger)
                          4096,        // stack bytes
                          nullptr,     // no params
                          1,           // priority — low, non-real-time
                          nullptr,     // handle not needed
                          0            // Core 0
  );

  if (mpuOk) {
    // ── Core 1 Task: 200Hz PID Balance (hard-RT, highest priority) ──────────
    xTaskCreatePinnedToCore(
        Task_Core1_Balance, "BalancePID",
        8192, // larger stack — float math + I²C buffers
        nullptr,
        configMAX_PRIORITIES - 1, // never preempted by any other task
        nullptr,
        1 // Core 1 — exclusively owned
    );
    Serial.println("[OK] Tasks spawned (PID Balance mode).");
    Serial.println("[>>] VoxBot is live. Connect BLE and balance.\n");
  } else {
    // ── Fallback: Direct-drive task (no IMU → no balancing) ─────────────────
    //   Simply reads BLE commands and drives motors at a fixed PWM.
    //   This lets the robot respond to commands even without MPU6050.
    xTaskCreatePinnedToCore(Task_DirectDrive, "DirectDrive",
        4096, nullptr, 2, nullptr, 1);
    Serial.println("[OK] Tasks spawned (DIRECT DRIVE mode — no balancing).");
    Serial.println("[>>] VoxBot is live. Connect BLE to drive.\n");
  }
}

// loop() is pinned to Core 1 by Arduino framework.
// We have given Core 1 entirely to Task_Core1_Balance at max priority.
// vTaskDelay(portMAX_DELAY) permanently suspends loop() — it will never run.
void loop() { vTaskDelay(portMAX_DELAY); }
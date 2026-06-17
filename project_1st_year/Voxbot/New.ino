/**
 * =============================================================================
 *  V O X B O T  —  ISOLATED BALANCE TEST BUILD
 *  Firmware vISO-1.0  |  ESP32 Single-Task FreeRTOS  |  NO WIRELESS
 * =============================================================================
 *
 *  PURPOSE
 *  -------
 *  Strip every non-essential system out of VoxBot so we can isolate and tune
 *  the core balancing mechanics in total silence. No BLE, no WiFi, no NLP
 *  command parsing. The robot does exactly one thing: try to stand up.
 *
 *  CONTROL ARCHITECTURE — SINGLE-LOOP ANGLE PID (direct angle → PWM)
 *  -------------------------------------------------------------------------
 *  One PID loop converts the MPU6050 pitch angle directly into a PWM duty
 *  cycle that is sent to both wheels symmetrically:
 *
 *    ANGLE PID (runs every tick @ 200Hz)
 *      Input:  MPU6050 pitch (complementary filter)
 *      Output: PWM duty cycle to the DRV8833
 *      Job:    "robot is tilting forward → spin wheels forward to catch it"
 *
 *  RTOS DECISION — see chat explanation. Short version:
 *    One dedicated FreeRTOS task, pinned to Core 1, max priority.
 *    Core 0 is left free for the Arduino/WiFi idle housekeeping that the
 *    ESP32 framework runs regardless — costs us nothing, buys deterministic
 *    timing, and means a stalled balance loop can never starve the system
 *    watchdog into a reboot mid-test.
 *    loop() is permanently parked; all real work happens in Task_Balance().
 *
 *  I2C (PIN MAP CONFIRMED — unchanged from prior schematic):
 *    SDA → GPIO 21      SCL → GPIO 22
 *
 *  DRV8833 (PIN MAP CONFIRMED — unchanged from prior schematic):
 *    Left  motor:  IN1 → GPIO 25   IN2 → GPIO 26
 *    Right motor:  IN3 → GPIO 27   IN4 → GPIO 14
 *
 *  Board       : "ESP32 Dev Module"
 *  Core        : esp32 Arduino core 3.3.x (ESP-IDF 5.x)
 *  Libraries   : NONE external — Wire + FreeRTOS only, both built-in
 * =============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ── Forward declarations (prevent Arduino auto-prototype mangling) ──────────
struct ImuData;
static void initMotorPWM();
static void driveMotors(int leftPWM, int rightPWM);
static void stopMotors();
static bool mpuInit();
static bool mpuRead(ImuData &out);
static void Task_Balance(void *pvParam);

// =============================================================================
//  ╔═══════════════════════════════════════════════════════════════════════╗
//  ║   SECTION 0 — PID TUNING BLOCK  (EDIT THESE DURING PHYSICAL TESTING)  ║
//  ╚═══════════════════════════════════════════════════════════════════════╝
// =============================================================================
//
//  Re-flash after every change here. Nothing else in this file needs to be
//  touched during the tuning phase — keep all adjustments confined to this
//  block so the team has one place to look.

// ── ANGLE PID  (tilt angle → direct PWM) ────────────────────────────────────
//
//  ANGLE_KP — How aggressively the robot reacts to being tilted.
//    Too LOW : robot leans over slowly and faceplants — it "feels" the tilt
//              but doesn't react hard enough to catch itself.
//    Too HIGH: robot oscillates violently front-to-back at high frequency,
//              like a metronome having a seizure. Will eventually fling
//              itself off the table.
//    Starting point for 600 RPM N20s: 18.0 – 25.0
#define ANGLE_KP 20.0f

//  ANGLE_KI — Corrects slow standing drift (robot slowly creeps in one
//             direction even when "balanced").
//    Too LOW : robot drifts forward/backward slowly at rest, never settles.
//    Too HIGH: robot develops a slow forward/backward "rocking" wobble with
//              a period of several seconds — a classic integral windup
//              symptom. If you see this, set Ki back toward 0 immediately.
//    Leave this at 0.0 for the FIRST test. Only introduce it once Kp/Kd
//    feel stable and you observe a consistent drift direction.
#define ANGLE_KI 0.0f

//  ANGLE_KD — The "brake" against oscillation. This is your damping term.
//    Too LOW : robot oscillates (same symptom as Kp too high, but lower
//              frequency / larger swings).
//    Too HIGH: robot feels "stiff" and sluggish to respond, and amplifies
//              any sensor noise into jittery, twitchy motor response —
//              you'll hear the N20 gearboxes chattering audibly.
//    Starting point for 600 RPM N20s: Kd = Kp × 0.05 to Kp × 0.10
#define ANGLE_KD 1.2f

#define ANGLE_I_CLAMP 30.0f // Anti-windup ceiling for the angle integral term

// ── Balance geometry & safety limits ────────────────────────────────────────
#define BALANCE_ANGLE_DEG                                                      \
  0.0f // Physical trim — adjust in 0.2° steps if
       // the robot leans even when "balanced"
#define FALL_ANGLE_DEG                                                         \
  35.0f // Cut motors immediately past this tilt —
        // keep tight for bench testing safety

// =============================================================================
//  SECTION 1 — PIN MAP
// =============================================================================

// ── I2C (CONFIRMED — matches prior schematic) ───────────────────────────────
#define PIN_SDA 21
#define PIN_SCL 22

// ── DRV8833 Motor Driver (CONFIRMED — matches prior schematic) ─────────────
#define PIN_ML_IN1 25 // Left  motor — forward PWM
#define PIN_ML_IN2 26 // Left  motor — reverse PWM
#define PIN_MR_IN3 27 // Right motor — forward PWM
#define PIN_MR_IN4 14 // Right motor — reverse PWM

// =============================================================================
//  SECTION 2 — LEDC PWM CONFIG  (core 3.x pin-centric API)
// =============================================================================
#define LEDC_FREQ_HZ 20000 // 20kHz — inaudible, kills N20 motor whine
#define LEDC_RESOLUTION 8  // 8-bit duty range 0–255
#define PWM_MAX 200        // Hard ceiling — avoids DRV8833 stall spikes

// =============================================================================
//  SECTION 3 — COMPLEMENTARY FILTER
// =============================================================================
#define CF_ALPHA 0.95f // 95% gyro (fast) + 5% accel (drift correction)

// =============================================================================
//  SECTION 4 — TIMING
// =============================================================================
#define BALANCE_PERIOD_US 5000 // 200Hz control loop = 5ms per tick

// =============================================================================
//  SECTION 5 — MPU6050 REGISTERS
// =============================================================================
#define MPU_ADDR 0x68
#define MPU_REG_SMPLRT_DIV 0x19
#define MPU_REG_CONFIG 0x1A
#define MPU_REG_GYRO_CFG 0x1B
#define MPU_REG_ACCEL_CFG 0x1C
#define MPU_REG_ACCEL_OUT 0x3B
#define MPU_REG_PWR_MGMT 0x6B
#define MPU_REG_WHOAMI 0x75

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f

// =============================================================================
//  SECTION 6 — PID CONTROLLER
// =============================================================================

class PIDController {
public:
  PIDController(float kp, float ki, float kd, float iClamp)
      : _kp(kp), _ki(ki), _kd(kd), _iClamp(iClamp), _integral(0.f),
        _prevError(0.f) {}

  float compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    _integral += error * dt;
    _integral = constrain(_integral, -_iClamp, _iClamp);
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
//  SECTION 7 — MOTOR DRIVER HELPERS  (core 3.x pin-based LEDC)
// =============================================================================

static void initMotorPWM() {
  ledcAttach(PIN_ML_IN1, LEDC_FREQ_HZ, LEDC_RESOLUTION);
  ledcAttach(PIN_ML_IN2, LEDC_FREQ_HZ, LEDC_RESOLUTION);
  ledcAttach(PIN_MR_IN3, LEDC_FREQ_HZ, LEDC_RESOLUTION);
  ledcAttach(PIN_MR_IN4, LEDC_FREQ_HZ, LEDC_RESOLUTION);
  ledcWrite(PIN_ML_IN1, 0);
  ledcWrite(PIN_ML_IN2, 0);
  ledcWrite(PIN_MR_IN3, 0);
  ledcWrite(PIN_MR_IN4, 0);
}

static void driveMotors(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, -PWM_MAX, PWM_MAX);
  rightPWM = constrain(rightPWM, -PWM_MAX, PWM_MAX);

  if (leftPWM >= 0) {
    ledcWrite(PIN_ML_IN1, leftPWM);
    ledcWrite(PIN_ML_IN2, 0);
  } else {
    ledcWrite(PIN_ML_IN1, 0);
    ledcWrite(PIN_ML_IN2, -leftPWM);
  }

  if (rightPWM >= 0) {
    ledcWrite(PIN_MR_IN3, rightPWM);
    ledcWrite(PIN_MR_IN4, 0);
  } else {
    ledcWrite(PIN_MR_IN3, 0);
    ledcWrite(PIN_MR_IN4, -rightPWM);
  }
}

static void stopMotors() {
  ledcWrite(PIN_ML_IN1, 0);
  ledcWrite(PIN_ML_IN2, 0);
  ledcWrite(PIN_MR_IN3, 0);
  ledcWrite(PIN_MR_IN4, 0);
}

// =============================================================================
//  SECTION 8 — MPU6050 DRIVER
// =============================================================================

struct ImuData {
  float ax, ay, az, gx, gy, gz;
};

static inline int16_t wireRead16() {
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

static bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_WHOAMI);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  if (!Wire.available())
    return false;
  uint8_t id = Wire.read();
  if (id != 0x68 && id != 0x98)
    return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_PWR_MGMT);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_SMPLRT_DIV);
  Wire.write(0x04);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_CONFIG);
  Wire.write(0x04);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_GYRO_CFG);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_CFG);
  Wire.write(0x00);
  Wire.endTransmission();

  return true;
}

static bool mpuRead(ImuData &out) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_OUT);
  if (Wire.endTransmission(false) != 0)
    return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14)
    return false;

  out.ax = wireRead16() / ACCEL_SCALE;
  out.ay = wireRead16() / ACCEL_SCALE;
  out.az = wireRead16() / ACCEL_SCALE;
  wireRead16(); // temperature, discard
  out.gx = wireRead16() / GYRO_SCALE;
  out.gy = wireRead16() / GYRO_SCALE;
  out.gz = wireRead16() / GYRO_SCALE;
  return true;
}

// =============================================================================
//  SECTION 9 — THE BALANCE TASK  (single dedicated RTOS task, Core 1)
// =============================================================================
//
//  Single-loop control, every 5ms tick:
//    1. Timing gate
//    2. IMU read + complementary filter → cfAngle
//    3. Fall check
//    4. ANGLE PID: cfAngle → PWM duty cycle
//    5. Drive motors

static void Task_Balance(void *pvParam) {
  Serial.println("[Balance] Task starting. Warm-up 500ms...");

  PIDController anglePID(ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_I_CLAMP);

  float cfAngle = 0.0f;

  // Pre-seed the complementary filter so it doesn't spike on tick 1
  ImuData imu;
  for (int i = 0; i < 100; i++) {
    if (mpuRead(imu)) {
      float ap = atan2f(imu.ax, imu.az) * RAD_TO_DEG;
      cfAngle = CF_ALPHA * cfAngle + (1.0f - CF_ALPHA) * ap;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  Serial.printf("[Balance] Ready. Initial pitch: %+.2f°\n", cfAngle);

  int64_t lastUs = esp_timer_get_time();

  for (;;) {
    // ── Step 1: Strict 200Hz timing gate ───────────────────────────────────
    int64_t now = esp_timer_get_time();
    int64_t dtUs = now - lastUs;
    if (dtUs < (int64_t)BALANCE_PERIOD_US) {
      vTaskDelay(1);
      continue;
    }
    lastUs = now;
    float dt = (float)dtUs * 1.0e-6f;

    // ── Step 2: IMU read + complementary filter ────────────────────────────
    if (!mpuRead(imu)) {
      Serial.println("[Balance] WARN: IMU read failed. Holding.");
      continue;
    }
    float accelPitch = atan2f(imu.ax, imu.az) * RAD_TO_DEG;
    float gyroPitch = cfAngle + imu.gy * dt;
    cfAngle = CF_ALPHA * gyroPitch + (1.0f - CF_ALPHA) * accelPitch;

    // ── Step 3: Fall detection ──────────────────────────────────────────────
    if (fabsf(cfAngle) > FALL_ANGLE_DEG) {
      stopMotors();
      anglePID.reset();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // ── Step 4: ANGLE PID — angle error → PWM ──────────────────────────────
    //    Pure balance test: setpoint is always "stand at BALANCE_ANGLE_DEG".
    //    There is no commanded lean here because there is no remote control
    //    in this build — the robot's only job is to stay upright at zero.
    float pwm = anglePID.compute(BALANCE_ANGLE_DEG, cfAngle, dt);

    // ── Step 5: Drive motors ─────────────────────────────────────────────────
    driveMotors((int)pwm, (int)pwm);
  }
}

// =============================================================================
//  SECTION 10 — SETUP & LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=========================================");
  Serial.println(" VoxBot ISOLATED BALANCE TEST  (no wireless)");
  Serial.println("=========================================");

  initMotorPWM();
  Serial.println("[OK] DRV8833 PWM initialized (20kHz, 8-bit)");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if (!mpuInit()) {
    Serial.println();
    Serial.println("[FATAL] MPU6050 not detected on I2C bus.");
    Serial.println("        This is a pure balance build — without the IMU");
    Serial.println("        there is nothing for this firmware to do.");
    Serial.println(
        "        Checklist: SDA=21 SCL=22, 4.7k pull-ups, VCC=3.3V, AD0=GND");
    while (true) {
      delay(1000);
    } // Halt — no fallback mode in this build
  }
  Serial.println(
      "[OK] MPU6050 initialized (DLPF=21Hz, ODR=200Hz, ±250°/s, ±2g)");

  // Single dedicated task, Core 1, max priority — see header comment for
  // the reasoning on why this beats running everything in loop() directly.
  xTaskCreatePinnedToCore(Task_Balance, "Balance", 8192, nullptr,
                          configMAX_PRIORITIES - 1, nullptr,
                          1 // Core 1
  );

  Serial.println("[>>] Balance task live. Prop the robot upright now.\n");
}

void loop() { vTaskDelay(portMAX_DELAY); }

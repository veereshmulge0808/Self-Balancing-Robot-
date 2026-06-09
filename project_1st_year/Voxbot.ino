/**
 * =============================================================================
 *  V O X B O T  —  SELF-BALANCING BIPEDAL ROBOT
 *  Master Firmware v1.0  |  ESP32 Dual-Core FreeRTOS
 * =============================================================================
 *  Core 0 — "Consciousness"   BLE Server  (async command receiver)
 *  Core 1 — "Reflexes"        200Hz Cascaded PID  (Angle + Speed loops)
 *
 *  IMU    : MPU6050  (DLPF register 0x04 = 21Hz hardware filter)
 *  Driver : DRV8833  (ledc 20kHz PWM, 8-bit)
 *  Motors : 2× N20 600RPM (No Encoder)
 *  IPC    : FreeRTOS Queue — ZERO mutexes, ZERO blocking on Core 1
 * =============================================================================
 *  PlatformIO lib_deps:
 *    - ArduinoJson @ ^7.0   (optional, for JSON BLE payloads)
 *    - ESP32 BLE Arduino    (bundled with arduino-esp32 core)
 * =============================================================================
 */

/* ── Includes ────────────────────────────────────────────────────────────── */
#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 1 — PIN MAP
   ═══════════════════════════════════════════════════════════════════════════ */
// I²C — MPU6050
#define PIN_SDA            21
#define PIN_SCL            22

// DRV8833 Motor Driver
#define PIN_ML_IN1         25   // Left  motor — forward
#define PIN_ML_IN2         26   // Left  motor — reverse
#define PIN_MR_IN3         27   // Right motor — forward
#define PIN_MR_IN4         14   // Right motor — reverse

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 2 — LEDC PWM CONFIGURATION
   ═══════════════════════════════════════════════════════════════════════════ */
#define LEDC_FREQ_HZ       20000    // 20kHz — above hearing range, kills motor whine
#define LEDC_RESOLUTION    8        // 8-bit: duty 0–255

#define LEDC_CH_ML_FWD     0
#define LEDC_CH_ML_REV     1
#define LEDC_CH_MR_FWD     2
#define LEDC_CH_MR_REV     3

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 3 — CASCADED PID TUNING CONSTANTS
   ═══════════════════════════════════════════════════════════════════════════
   600 RPM N20 TUNING PHILOSOPHY:
   These motors are FAST but WEAK. Violent Kp → violent shaking → falls.
   Use Kd as the primary "electronic brake" on the fast motors.

   START HERE (robot on a stand with wheels free):
     Angle PID  → Set Kp=15, Ki=0, Kd=0. Increase Kp until robot reacts.
                  Add Kd to stop oscillation. Add Ki last for drift.
     Speed PID  → Set Kp=5, Ki=0.5, Kd=0. Increase until speed tracks angle.
   ═══════════════════════════════════════════════════════════════════════════ */

// ── Outer Loop: Angle PID (pitch → target encoder speed) ─────────────────
#define ANGLE_KP           18.0f
#define ANGLE_KI           0.8f
#define ANGLE_KD           1.2f
#define ANGLE_I_CLAMP      50.0f    // anti-windup clamp (ticks/s)

// ── 600 RPM Motor-Specific PWM Limits ────────────────────────────────────
#define PWM_DEADBAND_MIN   15       // Minimum PWM to overcome gearbox static friction
#define PWM_MAX            200      // Max PWM — prevents DRV8833 stall-current spikes

// ── Balance geometry ──────────────────────────────────────────────────────
// Trim this ±2° at a time until robot stands without drifting
#define BALANCE_ANGLE_DEG  0.0f     // True vertical offset (degrees)
#define MAX_LEAN_DEG       4.0f     // Max lean angle from a full velocity command
#define MAX_TURN_PWM       60       // Max differential PWM for turning
#define FALL_ANGLE_DEG     40.0f    // Cut motors if beyond this — robot has fallen

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 4 — COMPLEMENTARY FILTER
   ═══════════════════════════════════════════════════════════════════════════
   With hardware DLPF at 21Hz, the accel data is pre-smoothed.
   0.95 alpha = 95% gyro (fast response) + 5% accel (drift correction).
   ═══════════════════════════════════════════════════════════════════════════ */
#define CF_ALPHA           0.95f

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 5 — TIMING
   ═══════════════════════════════════════════════════════════════════════════ */
#define BALANCE_PERIOD_US  5000     // 200Hz = 5ms

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 6 — MPU6050 REGISTER MAP
   ═══════════════════════════════════════════════════════════════════════════ */
#define MPU_ADDR           0x68
#define MPU_REG_SMPLRT_DIV 0x19
#define MPU_REG_CONFIG     0x1A    // DLPF config register
#define MPU_REG_GYRO_CFG   0x1B
#define MPU_REG_ACCEL_CFG  0x1C
#define MPU_REG_ACCEL_OUT  0x3B    // First byte of 14-byte burst
#define MPU_REG_PWR_MGMT   0x6B
#define MPU_REG_WHOAMI     0x75

#define ACCEL_SCALE        16384.0f // ±2g  → LSB/g
#define GYRO_SCALE         131.0f   // ±250°/s → LSB/(°/s)

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 7 — BLE UUIDs
   ═══════════════════════════════════════════════════════════════════════════ */
#define BLE_DEVICE_NAME    "VoxBot"
#define SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CMD_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 8 — IPC: COMMAND STRUCTURE & QUEUE
   ═══════════════════════════════════════════════════════════════════════════ */
struct DriveCommand {
    float linearVelocity;    // [-1.0 = full back  ... +1.0 = full forward]
    float angularVelocity;   // [-1.0 = full left  ... +1.0 = full right ]
};

#define CMD_QUEUE_DEPTH    4     // Depth 4: burst buffer, physics drains it at 200Hz
static QueueHandle_t xCmdQueue = nullptr;

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 10 — SHARED TELEMETRY
   ═══════════════════════════════════════════════════════════════════════════ */
static volatile float g_telPitch   = 0.0f;
static volatile float g_telPWM     = 0.0f;
static volatile bool  g_bleConnected = false;

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 11 — PID CONTROLLER CLASS
   ═══════════════════════════════════════════════════════════════════════════ */
class PID {
public:
    float kp, ki, kd, iClamp;

    PID(float p, float i, float d, float ic)
        : kp(p), ki(i), kd(d), iClamp(ic), _integ(0.f), _prev(0.f) {}

    float compute(float setpoint, float measured, float dt) {
        float err  = setpoint - measured;
        _integ    += err * dt;
        _integ     = constrain(_integ, -iClamp, iClamp);  // anti-windup
        float deriv = (err - _prev) / dt;
        _prev = err;
        return kp * err + ki * _integ + kd * deriv;
    }

    void reset() { _integ = 0.f; _prev = 0.f; }

private:
    float _integ, _prev;
};

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 12 — MOTOR DRIVER (DRV8833 via ledc)
   ═══════════════════════════════════════════════════════════════════════════ */
static void initMotorPWM() {
    const int pins[4] = { PIN_ML_IN1, PIN_ML_IN2, PIN_MR_IN3, PIN_MR_IN4 };
    const int chs[4]  = { LEDC_CH_ML_FWD, LEDC_CH_ML_REV,
                          LEDC_CH_MR_FWD, LEDC_CH_MR_REV };
    for (int i = 0; i < 4; i++) {
        ledcSetup(chs[i], LEDC_FREQ_HZ, LEDC_RESOLUTION);
        ledcAttachPin(pins[i], chs[i]);
        ledcWrite(chs[i], 0);
    }
}

/**
 * applyDeadband — 600 RPM N20 gearbox compensation.
 *
 * Without this, small PID outputs produce zero motion (gearbox friction wins),
 * the integrator winds up, and the robot suddenly lurches. Instead we:
 *   1. Zero zone : |raw| < 2 → output 0 (truly idle)
 *   2. Jump-start: any non-zero → floor at PWM_DEADBAND_MIN (overcome friction)
 *   3. Clamp     : hard ceiling at PWM_MAX (prevent stall-current spikes)
 */
static int applyDeadband(float raw) {
    if (fabsf(raw) < 2.0f) return 0;
    int sign = (raw > 0) ? 1 : -1;
    int pwm  = constrain((int)fabsf(raw), PWM_DEADBAND_MIN, PWM_MAX);
    return sign * pwm;
}

static void driveMotors(int leftPWM, int rightPWM) {
    // Left motor
    if (leftPWM >= 0) {
        ledcWrite(LEDC_CH_ML_FWD, leftPWM);
        ledcWrite(LEDC_CH_ML_REV, 0);
    } else {
        ledcWrite(LEDC_CH_ML_FWD, 0);
        ledcWrite(LEDC_CH_ML_REV, -leftPWM);
    }
    // Right motor
    if (rightPWM >= 0) {
        ledcWrite(LEDC_CH_MR_FWD, rightPWM);
        ledcWrite(LEDC_CH_MR_REV, 0);
    } else {
        ledcWrite(LEDC_CH_MR_FWD, 0);
        ledcWrite(LEDC_CH_MR_REV, -rightPWM);
    }
}

static void stopMotors() {
    ledcWrite(LEDC_CH_ML_FWD, 0); ledcWrite(LEDC_CH_ML_REV, 0);
    ledcWrite(LEDC_CH_MR_FWD, 0); ledcWrite(LEDC_CH_MR_REV, 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 13 — MPU6050 DRIVER
   ═══════════════════════════════════════════════════════════════════════════ */
static bool mpuInit() {
    // Probe WHO_AM_I
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_WHOAMI);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
    if (!Wire.available()) return false;
    uint8_t id = Wire.read();
    if (id != 0x68 && id != 0x98) return false;

    // Wake up — clear SLEEP bit
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_PWR_MGMT); Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // Sample Rate Divider = 4 → ODR = 1kHz / (1+4) = 200Hz
    // Aligns hardware output rate with our polling rate
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_SMPLRT_DIV); Wire.write(0x04);
    Wire.endTransmission();

    // ┌─────────────────────────────────────────────────────────────────┐
    // │  DLPF_CFG = 4  →  Accel BW: 21Hz | Gyro BW: 20Hz              │
    // │  This is the hardware low-pass filter specified in your BOM.    │
    // │  It removes motor vibration noise BEFORE the ADC.               │
    // └─────────────────────────────────────────────────────────────────┘
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_CONFIG); Wire.write(0x04);
    Wire.endTransmission();

    // Gyro: ±250°/s — highest resolution for slow balancing dynamics
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_GYRO_CFG); Wire.write(0x00);
    Wire.endTransmission();

    // Accel: ±2g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_ACCEL_CFG); Wire.write(0x00);
    Wire.endTransmission();

    return true;
}

struct ImuData {
    float ax, ay, az;   // g
    float gx, gy, gz;   // °/s
};

// 14-byte single burst read: AX AY AZ TEMP GX GY GZ (all 16-bit big-endian)
static bool mpuRead(ImuData& out) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_ACCEL_OUT);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;

    auto r16 = [&]() -> int16_t {
        return (int16_t)((Wire.read() << 8) | Wire.read());
    };

    out.ax = r16() / ACCEL_SCALE;
    out.ay = r16() / ACCEL_SCALE;
    out.az = r16() / ACCEL_SCALE;
    r16();   // temperature — discard
    out.gx = r16() / GYRO_SCALE;
    out.gy = r16() / GYRO_SCALE;
    out.gz = r16() / GYRO_SCALE;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 14 — BLE SERVER + CALLBACKS
   ═══════════════════════════════════════════════════════════════════════════
   Supported string commands from NLP app:
     "DRIVE_FORWARD"  → lean forward at 50% speed
     "DRIVE_BACKWARD" → lean backward at 50% speed
     "TURN_LEFT"      → spin counterclockwise
     "TURN_RIGHT"     → spin clockwise
     "STOP"           → all velocities to zero
     "SPEED:0.75"     → set custom forward speed (float, 0.0–1.0)
   ═══════════════════════════════════════════════════════════════════════════ */

static DriveCommand parseCommand(const std::string& s) {
    DriveCommand cmd = { 0.0f, 0.0f };
    if      (s == "DRIVE_FORWARD")   cmd.linearVelocity  =  0.5f;
    else if (s == "DRIVE_BACKWARD")  cmd.linearVelocity  = -0.5f;
    else if (s == "TURN_LEFT")       cmd.angularVelocity = -0.5f;
    else if (s == "TURN_RIGHT")      cmd.angularVelocity =  0.5f;
    else if (s == "STOP")            { /* zeros already set */ }
    else if (s.rfind("SPEED:", 0) == 0) {
        try {
            float v = std::stof(s.substr(6));
            cmd.linearVelocity = constrain(v, -1.0f, 1.0f);
        } catch (...) { /* malformed — leave zero */ }
    }
    return cmd;
}

class VoxBotServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        g_bleConnected = true;
        Serial.println("[BLE] Client connected.");
    }
    void onDisconnect(BLEServer*) override {
        g_bleConnected = false;
        Serial.println("[BLE] Disconnected. Restarting advertising...");
        // Safety: issue a STOP before resuming advertising
        DriveCommand stop = { 0.0f, 0.0f };
        xQueueSend(xCmdQueue, &stop, 0);
        BLEDevice::startAdvertising();
    }
};

class CommandCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar) override {
        String payload = pChar->getValue();
        if(payload.length()==0)
            return;

       DriveCommand cmd = parseCommand(std::string(payload.c_str()));

        // Non-blocking: if queue full, drop oldest (latest command wins)
        if (xQueueSend(xCmdQueue, &cmd, 0) != pdTRUE) {
            // Queue full — discard and replace with fresh command
            DriveCommand discard;
            xQueueReceive(xCmdQueue, &discard, 0);
            xQueueSend(xCmdQueue, &cmd, 0);
        }

        Serial.printf("[BLE→Queue] '%s'  lin=%.2f  ang=%.2f\n",
                      payload.c_str(), cmd.linearVelocity, cmd.angularVelocity);
    }
};

static void bleSetup() {
    BLEDevice::init(BLE_DEVICE_NAME);

    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new VoxBotServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    BLECharacteristic* pCmdChar = pService->createCharacteristic(
        CHAR_CMD_UUID,
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR  // Write Without Response = lower latency
    );
    pCmdChar->setCallbacks(new CommandCharCallbacks());
    pCmdChar->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising* pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    pAdv->setMinPreferred(0x06);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Advertising as 'VoxBot'");
    Serial.println("[BLE] Service UUID : " SERVICE_UUID);
    Serial.println("[BLE] Command UUID : " CHAR_CMD_UUID);
}

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 15 — CORE 0 TASK: BLE Watchdog + Serial Telemetry
   ═══════════════════════════════════════════════════════════════════════════
   The BLE stack itself runs internal tasks on Core 0 managed by Bluedroid.
   This task provides a heartbeat and forwards telemetry to Serial.
   ═══════════════════════════════════════════════════════════════════════════ */
static void Task_Core0_BLE(void* pvParam) {
    Serial.println("[Core0] BLE watchdog running.");
    for (;;) {
        Serial.printf("[TEL]  pitch=%+6.2f°  pwm=%+5.0f  ble=%s\n",
                      (float)g_telPitch,
                      (float)g_telPWM,
                      g_bleConnected ? "CONNECTED" : "ADVERTISING");
        vTaskDelay(pdMS_TO_TICKS(500));  // 2 Hz telemetry
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 16 — CORE 1 TASK: 200Hz Single PID BALANCE LOOP
   ═══════════════════════════════════════════════════════════════════════════
   Pipeline per 5ms tick:
     1. Drain command queue (non-blocking)
     2. Read IMU (14-byte I²C burst)
     3. Complementary filter → pitch angle
     4. Fall detection → cut motors if |pitch| > 40°
     5. Snapshot encoder counts (interrupt-safe)
     6. Angle PID → PWM
     7. Deadband compensation
     8. Turn mixing
     9. Drive motors
   ═══════════════════════════════════════════════════════════════════════════ */
static void Task_Core1_Balance(void* pvParam) {
    Serial.println("[Core1] Balance task started. IMU warm-up...");

    // Instantiate cascaded controllers
    PID anglePID(ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_I_CLAMP);

    // Complementary filter state
    float cfAngle = 0.0f;

    // Active command (updated from queue)
    DriveCommand activeCmd = { 0.0f, 0.0f };

    // ── IMU warm-up: pre-seed filter with accel data (500ms) ────────────
    ImuData imu;
    for (int i = 0; i < 100; i++) {
        if (mpuRead(imu)) {
            float ap = atan2f(imu.ax,sqrtf(imu.ay * imu.ay + imu.az * imu.az)) * RAD_TO_DEG;
            cfAngle  = CF_ALPHA * cfAngle + (1.0f - CF_ALPHA) * ap;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    Serial.printf("[Core1] Warm-up done. Initial pitch: %.2f°\n", cfAngle);

    int64_t lastUs = esp_timer_get_time();

    // ═══════════════════════════════════════════════════════════════════
    for (;;) {
        // ── Strict 200Hz timing gate ───────────────────────────────────
        int64_t now  = esp_timer_get_time();
        int64_t dtUs = now - lastUs;

        if (dtUs < (int64_t)BALANCE_PERIOD_US) {
            vTaskDelay(1);   // yield CPU slice, feeds RTOS watchdog
            continue;
        }
        lastUs    = now;
        float dt  = (float)dtUs * 1.0e-6f;   // µs → seconds

        // ── Step 1: Drain command queue ────────────────────────────────
        // Always consume ALL pending commands — keep only the freshest
        DriveCommand tmp;
        while (xQueueReceive(xCmdQueue, &tmp, 0) == pdTRUE) {
            activeCmd = tmp;
        }

        // ── Step 2: Read IMU ────────────────────────────────────────────
        if (!mpuRead(imu)) {
            // I²C hiccup — hold previous output, try next tick
            Serial.println("[Core1] WARN: IMU read fail.");
            continue;
        }

        // ── Step 3: Complementary Filter ───────────────────────────────
        //
        // MOUNTING NOTE: This assumes MPU6050 mounted with:
        //   • Z-axis pointing UP  when robot is balanced
        //   • X-axis pointing FORWARD (direction of travel)
        //   • Gyro Y-axis = pitch rotation axis
        //
        // If your robot pitches on a different axis, swap imu.gy and
        // the atan2 arguments accordingly. Verify in Serial Monitor:
        //   Tilt robot forward → g_telPitch should INCREASE (go positive).
        //
        float accelPitch=atan2f(imu.ax,sqrtf(imu.ay*imu.ay+imu.az*imu.az))*RAD_TO_DEG;
        float gyroPitch  = cfAngle + imu.gy * dt;   // integrate pitch rate
        cfAngle  = CF_ALPHA * gyroPitch + (1.0f - CF_ALPHA) * accelPitch;
        g_telPitch = cfAngle;

        // ── Step 4: Fall detection ──────────────────────────────────────
        if (fabsf(cfAngle) > FALL_ANGLE_DEG) {
            stopMotors();
            anglePID.reset();
            // Pause 100ms then try again — avoids I²C flooding while fallen
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // ── Step 6: Outer PID — Angle → Target Speed ────────────────────
        //
        // Linear velocity command shifts the balance setpoint (lean angle).
        //   DRIVE_FORWARD → lean forward → physics drives robot forward.
        // This is the correct inverted-pendulum locomotion strategy.
        //
        float angleSetpoint = BALANCE_ANGLE_DEG
                              + (activeCmd.linearVelocity * MAX_LEAN_DEG);
        float rawPWM = anglePID.compute(angleSetpoint, cfAngle,dt);

        int basePWM = applyDeadband(rawPWM);
        g_telPWM    = (float)basePWM;

        // ── Step 9: Turn mixing (differential drive) ─────────────────────
        int turnOff  = (int)(activeCmd.angularVelocity * (float)MAX_TURN_PWM);
        int leftPWM  = constrain(basePWM - turnOff, -PWM_MAX, PWM_MAX);
        int rightPWM = constrain(basePWM + turnOff, -PWM_MAX, PWM_MAX);

        // ── Step 10: Drive ───────────────────────────────────────────────
        driveMotors(leftPWM, rightPWM);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   SECTION 17 — SETUP & LOOP
   ═══════════════════════════════════════════════════════════════════════════ */
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n╔══════════════════════════════════╗");
    Serial.println("║   VoxBot Firmware v1.0 Booting   ║");
    Serial.println("╚══════════════════════════════════╝");

    // ── I²C @ 400kHz ─────────────────────────────────────────────────────
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    if (!mpuInit()) {
        Serial.println("[FATAL] MPU6050 not found on I2C bus.");
        Serial.println("        Check: SDA=21, SCL=22, pull-ups=4.7kΩ, VCC=3.3V");
        while (true) delay(1000);
    }
    Serial.println("[OK] MPU6050  (DLPF=21Hz, ODR=200Hz, ±250°/s, ±2g)");

    // ── Motor PWM @ 20kHz ─────────────────────────────────────────────────
    initMotorPWM();
    Serial.println("[OK] DRV8833  (ledc 20kHz 8-bit, deadband=15, clamp=200)");

    // ── FreeRTOS Queue (MUST be created before BLE setup) ─────────────────
    xCmdQueue = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(DriveCommand));
    configASSERT(xCmdQueue != nullptr);
    Serial.println("[OK] Command Queue (depth=4)");

    // ── BLE Server ────────────────────────────────────────────────────────
    bleSetup();

    // ── Core 0 Task: BLE Watchdog / Telemetry ────────────────────────────
    xTaskCreatePinnedToCore(
        Task_Core0_BLE,   // function
        "BLE_Watchdog",   // name
        4096,             // stack bytes
        nullptr,          // params
        1,                // priority (low — non-RT)
        nullptr,          // handle
        0                 // Core 0
    );

    // ── Core 1 Task: 200Hz Cascaded PID ─────────────────────────────────
    xTaskCreatePinnedToCore(
        Task_Core1_Balance,
        "BalancePID",
        8192,                         // larger stack — math + I²C
        nullptr,
        configMAX_PRIORITIES - 1,    // MAXIMUM priority — never preempted
        nullptr,
        1                             // Core 1
    );

    Serial.println("[OK] All tasks spawned. VoxBot is live.\n");
}

// Arduino loop() runs under the idle task — keep it dead.
// Every microsecond of Core 1 belongs to the 200Hz balance loop.
void loop() {
    vTaskDelay(portMAX_DELAY);
}
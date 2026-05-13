// ============================================================
//  GloveMouse — Movement Only (Top Tier)
//  ESP32 WROOM + MPU6050 + BLE HID
//
//  Pipeline:
//    MPU6050 raw → Madgwick 6DOF → Quaternion
//    → Pitch / Roll extraction (drift-free, gravity-anchored)
//    → Perspective projection (ray-plane)
//    → 2-axis Kalman smoother
//    → Delta → BLE HID relative mouse @ 125Hz
//
//  Required libraries (Arduino Library Manager):
//    - "ESP32 BLE Mouse"  by T-vK
//    - "MadgwickAHRS"     by x-io Technologies
//    - "MPU6050"          by Electronic Cats  (or Jeff Rowberg)
// ============================================================

#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include <BleMouse.h>

// ─────────────────────────────────────────
//  CONFIG  —  tune these per your setup
// ─────────────────────────────────────────

// Screen resolution
#define SCREEN_W        1920
#define SCREEN_H        1080

// Field of view: how many degrees of wrist tilt = full screen width
// 60° means tilting 30° left → cursor at left edge, 30° right → right edge
#define FOV_H_DEG       60.0f
#define FOV_V_DEG       (FOV_H_DEG * (float)SCREEN_H / (float)SCREEN_W)

// Madgwick beta (0.033–0.1)
// Higher = more stable, more latency
// Lower  = more responsive, more drift
#define MADGWICK_BETA   0.05f

// Sample rate
#define SAMPLE_HZ       125
#define SAMPLE_US       (1000000 / SAMPLE_HZ)   // microseconds per frame

// Warmup: frames before sending BLE (filter convergence)
#define WARMUP_FRAMES   (SAMPLE_HZ * 2)          // 2 seconds

// Kalman measurement noise (tune per feel)
// Higher = smoother cursor, more lag
// Lower  = snappier, more jitter
#define KALMAN_R_X      4.0f
#define KALMAN_R_Y      4.0f

// Minimum pixel delta to bother sending (reduces BLE spam on still hand)
#define MOVE_THRESHOLD  1

// BLE device name
#define BLE_DEVICE_NAME "GloveMouse"


// ─────────────────────────────────────────
//  KALMAN FILTER  (1D, per axis)
// ─────────────────────────────────────────
struct Kalman1D {
    float x;        // estimate
    float p;        // error covariance
    float r;        // measurement noise
    float q;        // process noise

    Kalman1D(float measureNoise, float processNoise = 0.1f)
        : x(0), p(1), r(measureNoise), q(processNoise) {}

    float update(float z) {
        // Predict
        p += q;
        // Update
        float k = p / (p + r);
        x += k * (z - x);
        p *= (1.0f - k);
        return x;
    }

    void reset(float value) {
        x = value;
        p = 1.0f;
    }
};


// ─────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────
MPU6050     mpu;
Madgwick    filter;
BleMouse    bleMouse(BLE_DEVICE_NAME, "GloveLab", 100);

Kalman1D    kalX(KALMAN_R_X);
Kalman1D    kalY(KALMAN_R_Y);

// Pre-computed FOV tangent values (set in setup)
float halfTanH = 0;
float halfTanV = 0;

// Origin offset (set on warmup complete)
float originPitch = 0;
float originRoll  = 0;

// Previous smooth screen position (for delta)
float prevSX = SCREEN_W  / 2.0f;
float prevSY = SCREEN_H  / 2.0f;

// Timing
int64_t lastFrameUs = 0;

// Warmup state
int  warmupCount  = 0;
bool isWarmedUp   = false;

// Gyro calibration offsets (computed at startup)
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;


// ─────────────────────────────────────────
//  GYRO BIAS CALIBRATION
//  Call once at startup with sensor still
// ─────────────────────────────────────────
void calibrateGyro(int samples = 500) {
    Serial.print("Calibrating gyro");
    double sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        if (i % 50 == 0) Serial.print(".");
        delay(2);
    }
    gyroOffX = sumX / samples;
    gyroOffY = sumY / samples;
    gyroOffZ = sumZ / samples;
    Serial.println(" done.");
    Serial.printf("  Gyro offsets: %.1f  %.1f  %.1f\n", gyroOffX, gyroOffY, gyroOffZ);
}


// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== GloveMouse BOOT ===");

    // I2C at 400kHz
    Wire.begin();
    Wire.setClock(400000);

    // Init MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("[ERROR] MPU6050 not found! Check wiring.");
        while (true) { delay(1000); }
    }
    Serial.println("[OK] MPU6050 connected.");

    // Sensor config
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // ±500 °/s — enough for fast wrist
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);    // ±2g
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);              // 42 Hz low-pass — removes vibration noise

    // Gyro bias calibration (keep hand still for ~1s)
    calibrateGyro(500);

    // Madgwick init
    filter.begin(SAMPLE_HZ);
    filter.beta = MADGWICK_BETA;

    // Pre-compute FOV tangents
    halfTanH = tanf(FOV_H_DEG * 0.5f * DEG_TO_RAD);
    halfTanV = tanf(FOV_V_DEG * 0.5f * DEG_TO_RAD);

    // BLE HID
    bleMouse.begin();
    Serial.println("[OK] BLE advertising as \"" BLE_DEVICE_NAME "\"");
    Serial.printf("     FOV: %.0f° x %.0f°  |  Screen: %dx%d\n",
                  FOV_H_DEG, FOV_V_DEG, SCREEN_W, SCREEN_H);
    Serial.println("     Warming up filter (2s) — keep hand still...");

    lastFrameUs = esp_timer_get_time();
}


// ─────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────
void loop() {
    // ── Timing: enforce exact sample rate ──
    int64_t now = esp_timer_get_time();
    if ((now - lastFrameUs) < SAMPLE_US) return;
    float dt = (now - lastFrameUs) / 1000000.0f;
    lastFrameUs = now;

    // Clamp dt — prevents huge jumps after BLE blocking calls
    if (dt > 0.05f) dt = 0.05f;

    // ── Read IMU ──
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert accel to g  (±2g range → 16384 LSB/g)
    float axf =  ax / 16384.0f;
    float ayf =  ay / 16384.0f;
    float azf =  az / 16384.0f;

    // Convert gyro to rad/s  (±500°/s → 65.5 LSB/°/s), subtract bias
    float gxf = (gx - gyroOffX) / 65.5f * DEG_TO_RAD;
    float gyf = (gy - gyroOffY) / 65.5f * DEG_TO_RAD;
    float gzf = (gz - gyroOffZ) / 65.5f * DEG_TO_RAD;

    // ── Madgwick 6DOF fusion ──
    // Uses dt-aware update for accurate integration
    filter.updateIMU(gxf, gyf, gzf, axf, ayf, azf);

    // ── Warmup phase ──
    if (!isWarmedUp) {
        warmupCount++;
        if (warmupCount >= WARMUP_FRAMES) {
            isWarmedUp = true;

            // Capture current orientation as origin
            // getRoll/getPitch return degrees
            originPitch = filter.getPitch();
            originRoll  = filter.getRoll();

            // Reset Kalman to center
            kalX.reset(SCREEN_W  / 2.0f);
            kalY.reset(SCREEN_H  / 2.0f);
            prevSX = SCREEN_W  / 2.0f;
            prevSY = SCREEN_H  / 2.0f;

            Serial.println("[OK] Filter warmed up. Pair your device and move!");
            Serial.printf("     Origin → Pitch: %.2f°  Roll: %.2f°\n",
                          originPitch, originRoll);
        }
        return;
    }

    // ── Not connected → skip ──
    if (!bleMouse.isConnected()) return;

    // ── Extract orientation (drift-free axes only) ──
    // Pitch = forward/back tilt  → cursor Y  (gravity-anchored, zero drift)
    // Roll  = left/right tilt    → cursor X  (gravity-anchored, zero drift)
    // Yaw   = wrist twist        → NOT used for movement (drifts without mag)
    float pitch = filter.getPitch() - originPitch;   // degrees, offset from origin
    float roll  = filter.getRoll()  - originRoll;    // degrees, offset from origin

    // ── Perspective projection (ray-plane intersection) ──
    // tan(angle) / tan(halfFOV) gives normalized [-1, 1] within the FOV cone
    float normX =  tanf(roll  * DEG_TO_RAD) / halfTanH;
    float normY = -tanf(pitch * DEG_TO_RAD) / halfTanV;
    // Note: Y inverted so tilting hand forward moves cursor UP

    // Map normalized [-1,1] → screen pixels
    float screenX = (normX + 1.0f) * 0.5f * SCREEN_W;
    float screenY = (normY + 1.0f) * 0.5f * SCREEN_H;

    // Hard clamp to screen bounds
    screenX = constrain(screenX, 0.0f, (float)(SCREEN_W  - 1));
    screenY = constrain(screenY, 0.0f, (float)(SCREEN_H  - 1));

    // ── Kalman smooth ──
    float smoothX = kalX.update(screenX);
    float smoothY = kalY.update(screenY);

    // ── Compute delta for BLE relative mouse ──
    // (BLE HID mouse reports deltas, not absolute positions)
    int deltaX = (int)roundf(smoothX - prevSX);
    int deltaY = (int)roundf(smoothY - prevSY);

    prevSX = smoothX;
    prevSY = smoothY;

    // ── Send BLE report ──
    // Only send if cursor actually moved (reduces noise floor)
    if (abs(deltaX) >= MOVE_THRESHOLD || abs(deltaY) >= MOVE_THRESHOLD) {
        // BLE Mouse move() takes int8_t: clamp to [-127, 127]
        // If delta > 127, split into multiple reports
        while (deltaX != 0 || deltaY != 0) {
            int8_t tx = (int8_t)constrain(deltaX, -127, 127);
            int8_t ty = (int8_t)constrain(deltaY, -127, 127);
            bleMouse.move(tx, ty, 0, 0);
            deltaX -= tx;
            deltaY -= ty;
        }
    }
}

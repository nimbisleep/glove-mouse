/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║          GLOVE MOUSE — ESP32 + MPU6050 DMP               ║
 * ║          v3 — Origin capture, Kalman, ballistics         ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * WIRING:
 *   MPU6050  VCC  →  ESP32  3.3V
 *   MPU6050  GND  →  ESP32  GND
 *   MPU6050  SDA  →  ESP32  GPIO 21
 *   MPU6050  SCL  →  ESP32  GPIO 22
 *   MPU6050  INT  →  ESP32  GPIO 19
 *   MPU6050  AD0  →  GND    (sets I2C address to 0x68)
 *
 * LIBRARIES NEEDED:
 *   1. "MPU6050" by ElectronicCats
 *   2. ESP32 BLE (built into ESP32 Arduino core)
 *
 * WHAT CHANGED FROM v2:
 *   - Origin capture:   cursor starts at screen center regardless of hand position
 *   - Perspective map:  tan()-based projection feels natural (not linear stretch)
 *   - Deadzone:         applied in angle space (radians), not coordinate space
 *   - Ballistics curve: center of screen = high precision, edges = fast sweep
 *   - Kalman smoother:  replaces EMA, better tracking during motion
 *   - Serial throttled: was printing every DMP packet (~100Hz), now every 100ms
 *   - MOTION_RANGE:     tightened — 0.45 rad (≈26°) each side feels natural
 */

#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>

// ─────────────────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────────────────────
#define SDA_PIN      21
#define SCL_PIN      22
#define MPU_INT_PIN  19

// ─────────────────────────────────────────────────────────────────────────────
//  TUNING
// ─────────────────────────────────────────────────────────────────────────────

// FOV in radians — half-angle each side from center.
// 0.45 rad ≈ 26° per side = 52° total sweep.
// Increase if you need big wrist range. Decrease for tight precision.
#define HALF_FOV       0.45f

// Deadzone in radians (applied before projection).
// 0.012 rad ≈ 0.7° — kills hand tremor without feeling sticky.
// Increase if your readings are noisier.
#define DEADZONE_RAD   0.012f

// Kalman measurement noise. Higher = smoother, more lag. Lower = snappier.
#define KALMAN_R       2.5f

// Ballistics curve power (applied to normalized angle before screen mapping).
// 1.0 = linear (no curve). 1.4 = center precision boost, fast edge sweep.
// Increase toward 1.7 if you want even more center precision.
#define CURVE_POWER    1.4f

// Absolute coordinate range (must match HID descriptor — keep at 4095)
#define COORD_MAX      4095

// Report interval
#define SEND_INTERVAL_MS  8    // ~125Hz

// Serial debug print interval (ms) — does NOT affect BLE rate
#define DEBUG_INTERVAL_MS 100

// ─────────────────────────────────────────────────────────────────────────────
//  HID DESCRIPTOR — Absolute Mouse (unchanged from v2, it was correct)
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t hidDescriptor[] = {
  0x05, 0x01,
  0x09, 0x02,
  0xA1, 0x01,
  0x09, 0x01,
  0xA1, 0x00,
  0x85, 0x01,
  0x05, 0x09,
  0x19, 0x01,
  0x29, 0x03,
  0x15, 0x00,
  0x25, 0x01,
  0x75, 0x01,
  0x95, 0x03,
  0x81, 0x02,
  0x75, 0x05,
  0x95, 0x01,
  0x81, 0x03,
  0x05, 0x01,
  0x09, 0x30,
  0x09, 0x31,
  0x15, 0x00,
  0x26, 0xFF, 0x0F,
  0x75, 0x10,
  0x95, 0x02,
  0x81, 0x02,
  0xC0,
  0xC0
};

// ─────────────────────────────────────────────────────────────────────────────
//  KALMAN FILTER (1D)
// ─────────────────────────────────────────────────────────────────────────────
struct Kalman1D {
  float x = COORD_MAX / 2.0f;
  float p = 1.0f;
  const float r;   // measurement noise
  const float q;   // process noise

  Kalman1D(float measureNoise, float processNoise = 0.05f)
    : r(measureNoise), q(processNoise) {}

  float update(float z) {
    p += q;
    float k = p / (p + r);
    x += k * (z - x);
    p *= (1.0f - k);
    return x;
  }

  void seed(float value) { x = value; p = 1.0f; }
};

// ─────────────────────────────────────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────────────────────────────────────
MPU6050 mpu;

bool     dmpReady   = false;
uint16_t packetSize = 0;
uint8_t  fifoBuffer[64];

Quaternion  q;
VectorFloat gravity;
float       ypr[3];

// Origin captured after calibration — hand's resting position = screen center
float originPitch = 0.0f;
float originRoll  = 0.0f;
bool  originSet   = false;
int   originSamples = 0;
float originSumP = 0, originSumR = 0;
#define ORIGIN_SAMPLES 80   // ~0.8s of stable readings to set origin

BLEHIDDevice*      hid   = nullptr;
BLECharacteristic* input = nullptr;
bool               bleConnected = false;

Kalman1D kalX(KALMAN_R);
Kalman1D kalY(KALMAN_R);

unsigned long lastSend  = 0;
unsigned long lastDebug = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  BLE CALLBACKS
// ─────────────────────────────────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleConnected = true;
    Serial.println("[BLE] Connected!");
  }
  void onDisconnect(BLEServer* server) override {
    bleConnected = false;
    Serial.println("[BLE] Disconnected — restarting advertising...");
    server->startAdvertising();
  }
};

// ─────────────────────────────────────────────────────────────────────────────
//  BALLISTICS CURVE
//  Applies a power curve to normalized [-1, 1] angle.
//  Preserves sign, compresses near center, stretches near edges.
//  Result: slow wrist = precise cursor; fast wrist = edge sweep.
// ─────────────────────────────────────────────────────────────────────────────
inline float applyCurve(float norm) {
  float s = (norm >= 0.0f) ? 1.0f : -1.0f;
  return s * powf(fabsf(norm), CURVE_POWER);
}

// ─────────────────────────────────────────────────────────────────────────────
//  ANGLE → SCREEN COORDS
//
//  Uses tan()-based perspective projection so the mapping matches how a real
//  ray-plane intersection works. Feels more natural than linear mapping,
//  especially near the edges.
//
//  Both pitch and roll are relative to captured origin, so cursor always
//  starts at center regardless of how the glove is worn.
// ─────────────────────────────────────────────────────────────────────────────
void getScreenCoords(float pitch, float roll, float &outX, float &outY) {
  // Subtract origin so neutral hand position = (0,0)
  float dp = pitch - originPitch;
  float dr = roll  - originRoll;

  // Deadzone in angle space — ignore micro-tremor
  if (fabsf(dp) < DEADZONE_RAD) dp = 0.0f;
  if (fabsf(dr) < DEADZONE_RAD) dr = 0.0f;

  // Perspective projection: normalize by FOV tangent
  float halfTan = tanf(HALF_FOV);
  float normX =  tanf(dp) / halfTan;   // pitch → X  (tilt left/right)
  float normY = -tanf(dr) / halfTan;   // roll  → Y  (tilt forward/back, Y inverted)

  // Clamp to [-1, 1] before curve (no projection outside the FOV cone)
  normX = constrain(normX, -1.0f, 1.0f);
  normY = constrain(normY, -1.0f, 1.0f);

  // Ballistics curve — center precision boost
  normX = applyCurve(normX);
  normY = applyCurve(normY);

  // Map [-1, 1] → [0, COORD_MAX]
  outX = (normX + 1.0f) * 0.5f * COORD_MAX;
  outY = (normY + 1.0f) * 0.5f * COORD_MAX;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SEND HID REPORT
// ─────────────────────────────────────────────────────────────────────────────
void sendReport(uint8_t buttons, uint16_t x, uint16_t y) {
  uint8_t val[5] = {
    buttons,
    (uint8_t)(x & 0xFF), (uint8_t)(x >> 8),
    (uint8_t)(y & 0xFF), (uint8_t)(y >> 8)
  };
  input->setValue(val, sizeof(val));
  input->notify();
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Glove Mouse v3 Boot ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.print("MPU6050 init... ");
  mpu.initialize();
  pinMode(MPU_INT_PIN, INPUT);

  if (!mpu.testConnection()) {
    Serial.println("FAILED. Check wiring!");
    while (true) delay(1000);
  }
  Serial.println("OK");

  Serial.print("DMP init... ");
  uint8_t status = mpu.dmpInitialize();
  if (status != 0) {
    Serial.printf("FAILED (code %d)\n", status);
    while (true) delay(1000);
  }

  // DMP calibration — keep COMPLETELY STILL for ~8 seconds
  Serial.println("OK\nCalibrating — keep glove STILL for 8 seconds...");
  mpu.CalibrateAccel(15);
  mpu.CalibrateGyro(15);

  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady   = true;
  Serial.printf("DMP ready. Packet: %d bytes\n", packetSize);

  // After calibration, capture the first ORIGIN_SAMPLES readings
  // as origin (averages out any residual jitter)
  Serial.printf("Setting origin — hold natural position for ~1 second...\n");

  // BLE
  Serial.print("BLE init... ");
  BLEDevice::init("GloveMouse");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  hid   = new BLEHIDDevice(server);
  input = hid->inputReport(1);

  hid->manufacturer()->setValue("Salah");
  hid->pnp(0x02, 0x045E, 0x0000, 0x0110);
  hid->hidInfo(0x00, 0x01);
  hid->reportMap((uint8_t*)hidDescriptor, sizeof(hidDescriptor));
  hid->startServices();

  BLEAdvertising* adv = server->getAdvertising();
  adv->setAppearance(HID_MOUSE);
  adv->addServiceUUID(hid->hidService()->getUUID());
  adv->start();

  Serial.println("OK");
  Serial.println("Advertising as 'GloveMouse'");
  Serial.println("==========================================");
}

// ─────────────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  if (!dmpReady) return;
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

  // Get orientation from DMP
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float pitch = ypr[1];
  float roll  = ypr[2];

  // ── Origin capture phase ──
  // Averages the first ORIGIN_SAMPLES frames to lock cursor to screen center.
  // This means you can put the glove on in any position and it just works.
  if (!originSet) {
    originSumP += pitch;
    originSumR += roll;
    originSamples++;

    if (originSamples >= ORIGIN_SAMPLES) {
      originPitch = originSumP / originSamples;
      originRoll  = originSumR / originSamples;
      originSet   = true;

      // Seed Kalman at screen center
      kalX.seed(COORD_MAX / 2.0f);
      kalY.seed(COORD_MAX / 2.0f);

      Serial.printf("[OK] Origin set → Pitch: %.3f rad  Roll: %.3f rad\n",
                    originPitch, originRoll);
      Serial.println("     Cursor locked to screen center. Move!");
    }
    return;   // Don't send BLE until origin is ready
  }

  // ── Projection ──
  float rawX, rawY;
  getScreenCoords(pitch, roll, rawX, rawY);

  // ── Kalman smooth ──
  float finalX = kalX.update(rawX);
  float finalY = kalY.update(rawY);

  uint16_t outX = (uint16_t)constrain(finalX, 0.0f, (float)COORD_MAX);
  uint16_t outY = (uint16_t)constrain(finalY, 0.0f, (float)COORD_MAX);

  // ── BLE send ──
  unsigned long now = millis();
  if (bleConnected && (now - lastSend >= SEND_INTERVAL_MS)) {
    sendReport(0x00, outX, outY);
    lastSend = now;
  }

  // ── Serial debug (throttled — doesn't affect BLE rate) ──
  if (now - lastDebug >= DEBUG_INTERVAL_MS) {
    Serial.printf("P:%6.3f  R:%6.3f  →  X:%4d  Y:%4d  [%s]\n",
      pitch - originPitch,
      roll  - originRoll,
      outX, outY,
      bleConnected ? "CONN" : "----"
    );
    lastDebug = now;
  }
}

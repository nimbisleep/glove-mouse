/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║            GLOVE MOUSE — ESP32 + MPU6050 DMP             ║
 * ║            v5 — Hand goes up, cursor goes up.            ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * WIRING:
 *   MPU6050  VCC  →  ESP32  3.3V
 *   MPU6050  GND  →  ESP32  GND
 *   MPU6050  SDA  →  ESP32  GPIO 21
 *   MPU6050  SCL  →  ESP32  GPIO 22
 *   MPU6050  INT  →  ESP32  GPIO 19
 *   MPU6050  AD0  →  GND
 *
 * LIBRARIES:
 *   1. "MPU6050" by ElectronicCats
 *   2. ESP32 BLE (built into ESP32 Arduino core)
 *
 * HOW IT WORKS:
 *   PITCH (tilt hand up/down)   → cursor UP / DOWN
 *   ROLL  (tilt hand left/right) → cursor LEFT / RIGHT
 *   YAW   (wrist twist)          → NOT USED (drifts, not needed)
 *
 *   Both pitch and roll are gravity-anchored → ZERO drift ever.
 *   Cursor always returns to same spot for same hand position.
 *
 * IF AN AXIS IS FLIPPED:
 *   Flip sign on line marked  ← FLIP THIS if X is wrong
 *                             ← FLIP THIS if Y is wrong
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
//  PINS
// ─────────────────────────────────────────────────────────────────────────────
#define SDA_PIN     21
#define SCL_PIN     22
#define MPU_INT_PIN 19

// ─────────────────────────────────────────────────────────────────────────────
//  TUNING
// ─────────────────────────────────────────────────────────────────────────────

// Degrees of tilt to reach the screen edge from center.
// 30° means: tilt 30° right → cursor at right edge.
// Decrease for more sensitive (less tilt needed).
// Increase for less sensitive (more tilt needed).
#define TILT_RANGE_X  30.0f    // degrees — left/right
#define TILT_RANGE_Y  20.0f    // degrees — up/down (less range = more precision)

// Kalman smoothing. Lower = snappier. Higher = smoother but laggy.
#define KALMAN_R      3.0f

// Deadzone hysteresis (in screen coords 0–4095)
#define DZ_ENTER      5        // freeze below this
#define DZ_EXIT       9        // unfreeze above this

// Number of frames to average for origin (keep still this long after boot)
#define ORIGIN_FRAMES 60       // ~0.6 seconds

#define COORD_MAX          4095
#define SEND_INTERVAL_MS   8      // ~125Hz
#define DEBUG_INTERVAL_MS  100

// ─────────────────────────────────────────────────────────────────────────────
//  HID DESCRIPTOR — Absolute Mouse
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t hidDescriptor[] = {
  0x05, 0x01,  0x09, 0x02,  0xA1, 0x01,  0x09, 0x01,
  0xA1, 0x00,  0x85, 0x01,
  0x05, 0x09,  0x19, 0x01,  0x29, 0x03,
  0x15, 0x00,  0x25, 0x01,  0x75, 0x01,  0x95, 0x03,
  0x81, 0x02,  0x75, 0x05,  0x95, 0x01,  0x81, 0x03,
  0x05, 0x01,  0x09, 0x30,  0x09, 0x31,
  0x15, 0x00,  0x26, 0xFF, 0x0F,
  0x75, 0x10,  0x95, 0x02,  0x81, 0x02,
  0xC0, 0xC0
};

// ─────────────────────────────────────────────────────────────────────────────
//  KALMAN 1D
// ─────────────────────────────────────────────────────────────────────────────
struct Kalman1D {
  float x, p = 1.0f, r, q = 0.05f;
  Kalman1D(float noise) : x(COORD_MAX / 2.0f), r(noise) {}
  float update(float z) {
    p += q;
    float k = p / (p + r);
    x += k * (z - x);
    p *= (1.0f - k);
    return x;
  }
  void seed(float v) { x = v; p = 1.0f; }
};

// ─────────────────────────────────────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────────────────────────────────────
MPU6050     mpu;
Quaternion  q;
VectorFloat gravity;
float       ypr[3];
bool        dmpReady   = false;
uint16_t    packetSize = 0;
uint8_t     fifoBuffer[64];

BLEHIDDevice*      hid   = nullptr;
BLECharacteristic* input = nullptr;
bool               bleConnected = false;

Kalman1D kalX(KALMAN_R);
Kalman1D kalY(KALMAN_R);

// Origin
float originPitch = 0, originRoll = 0;
float sumPitch    = 0, sumRoll    = 0;
int   originCount = 0;
bool  originReady = false;

// Deadzone state
bool     dzFrozenX = false, dzFrozenY = false;
uint16_t frozenX   = COORD_MAX / 2;
uint16_t frozenY   = COORD_MAX / 2;

unsigned long lastSend = 0, lastDebug = 0;

// Pre-computed FOV tangents (set in setup)
float halfTanX = 0, halfTanY = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  BLE
// ─────────────────────────────────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
    Serial.println("[BLE] Connected!");
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    Serial.println("[BLE] Disconnected — re-advertising...");
    s->startAdvertising();
  }
};

// ─────────────────────────────────────────────────────────────────────────────
//  TILT → SCREEN COORDS
//
//  pitch = hand tilts up/down  (ypr[1])
//  roll  = hand tilts left/right (ypr[2])
//
//  Uses tan() so the mapping matches real angular perspective —
//  feels natural at all positions, not stretched at the edges.
// ─────────────────────────────────────────────────────────────────────────────
void tiltToScreen(float pitch, float roll, float &outX, float &outY) {
  // Subtract origin so neutral hand = screen center
  float dp = pitch - originPitch;
  float dr = roll  - originRoll;

  // Normalize by FOV tangent → range [-1, 1]
  float normX =  tanf(dr) / halfTanX;   // ← FLIP THIS (change + to -) if X is wrong
  float normY = -tanf(dp) / halfTanY;   // ← FLIP THIS (change - to +) if Y is wrong

  // Clamp within screen
  normX = constrain(normX, -1.0f, 1.0f);
  normY = constrain(normY, -1.0f, 1.0f);

  // Map [-1, 1] → [0, COORD_MAX]
  outX = (normX + 1.0f) * 0.5f * COORD_MAX;
  outY = (normY + 1.0f) * 0.5f * COORD_MAX;
}

// ─────────────────────────────────────────────────────────────────────────────
//  DEADZONE WITH HYSTERESIS
// ─────────────────────────────────────────────────────────────────────────────
uint16_t deadzone(uint16_t current, uint16_t &frozen, bool &isFrozen) {
  int16_t delta = abs((int16_t)current - (int16_t)frozen);
  if (isFrozen) {
    if (delta > DZ_EXIT)  { isFrozen = false; }
    else                  { return frozen; }
  } else {
    if (delta < DZ_ENTER) { isFrozen = true; frozen = current; return frozen; }
  }
  frozen = current;
  return current;
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
  Serial.println("\n=== GloveMouse v5 ===");

  // Pre-compute FOV tangents once
  halfTanX = tanf(TILT_RANGE_X * DEG_TO_RAD);
  halfTanY = tanf(TILT_RANGE_Y * DEG_TO_RAD);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.print("MPU6050... ");
  mpu.initialize();
  pinMode(MPU_INT_PIN, INPUT);
  if (!mpu.testConnection()) {
    Serial.println("FAILED — check wiring");
    while (true) delay(1000);
  }
  Serial.println("OK");

  Serial.print("DMP... ");
  if (mpu.dmpInitialize() != 0) {
    Serial.println("FAILED");
    while (true) delay(1000);
  }

  Serial.println("OK\nCalibrating — keep STILL for ~10 seconds...");
  mpu.CalibrateAccel(10);
  mpu.CalibrateGyro(10);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady   = true;
  Serial.println("Done. Hold natural position for 0.6s to set origin...");

  Serial.print("BLE... ");
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
  Serial.println("OK — advertising as 'GloveMouse'");
  Serial.println("===================================");
}

// ─────────────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  if (!dmpReady) return;

  // FIFO overflow guard — prevents cursor jumps from stale data
  if (mpu.getFIFOCount() >= 1020) {
    mpu.resetFIFO();
    Serial.println("[WARN] FIFO overflow reset");
    return;
  }

  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float pitch = ypr[1];   // hand tilts up/down
  float roll  = ypr[2];   // hand tilts left/right
  // ypr[0] = yaw — ignored, drifts without magnetometer

  // ── Origin capture — average first ORIGIN_FRAMES readings ────────────────
  if (!originReady) {
    sumPitch += pitch;
    sumRoll  += roll;
    originCount++;
    if (originCount >= ORIGIN_FRAMES) {
      originPitch = sumPitch / ORIGIN_FRAMES;
      originRoll  = sumRoll  / ORIGIN_FRAMES;
      originReady = true;
      kalX.seed(COORD_MAX / 2.0f);
      kalY.seed(COORD_MAX / 2.0f);
      frozenX = frozenY = COORD_MAX / 2;
      Serial.printf("[OK] Origin set (P:%.2f° R:%.2f°) — cursor is live!\n",
                    degrees(originPitch), degrees(originRoll));
    }
    return;
  }

  // ── Project tilt to screen coords ────────────────────────────────────────
  float rawX, rawY;
  tiltToScreen(pitch, roll, rawX, rawY);

  // ── Kalman smooth ─────────────────────────────────────────────────────────
  float sX = kalX.update(rawX);
  float sY = kalY.update(rawY);

  uint16_t finalX = (uint16_t)constrain(sX, 0.0f, (float)COORD_MAX);
  uint16_t finalY = (uint16_t)constrain(sY, 0.0f, (float)COORD_MAX);

  // ── Deadzone ──────────────────────────────────────────────────────────────
  finalX = deadzone(finalX, frozenX, dzFrozenX);
  finalY = deadzone(finalY, frozenY, dzFrozenY);

  // ── BLE send ──────────────────────────────────────────────────────────────
  unsigned long now = millis();
  if (bleConnected && now - lastSend >= SEND_INTERVAL_MS) {
    sendReport(0x00, finalX, finalY);
    lastSend = now;
  }

  // ── Serial debug (throttled) ──────────────────────────────────────────────
  if (now - lastDebug >= DEBUG_INTERVAL_MS) {
    Serial.printf("P:%5.1f°  R:%5.1f°  →  X:%4d  Y:%4d  [%s]\n",
      degrees(pitch - originPitch),
      degrees(roll  - originRoll),
      finalX, finalY,
      bleConnected ? "CONN" : "----"
    );
    lastDebug = now;
  }
}

/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║          GLOVE MOUSE — ESP32 + MPU6050 DMP               ║
 * ║          v4 — Same algorithm, better everything else     ║
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
 * WHAT CHANGED FROM PREVIOUS VERSION:
 *   - Origin averaging:  yaw AND pitch zeroed over 60 samples (not just 1st packet)
 *   - Kalman smoother:   replaces EMA — tracks fast motion better, calmer at rest
 *   - Deadzone hysteresis: enter deadzone at 5, exit at 8 — no more sticky cursor
 *   - FIFO overflow guard: resets DMP if buffer fills up (prevents freeze/drift)
 *   - Serial throttled:  was printing ~100Hz (slows loop), now every 100ms
 *   - Calibration depth: 6 → 10 iterations — more accurate bias removal
 *   - CORE ALGORITHM:    unchanged — surface projection is still the same
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
#define SDA_PIN      21
#define SCL_PIN      22
#define MPU_INT_PIN  19

// ─────────────────────────────────────────────────────────────────────────────
//  TUNING — same knobs as before
// ─────────────────────────────────────────────────────────────────────────────
#define SENSITIVITY_X     1.5f   // left/right sweep range
#define SENSITIVITY_Y     0.8f   // up/down sweep range

// Kalman measurement noise (replaces SMOOTHING)
// Lower = snappier response. Higher = smoother, more lag.
// Sweet spot: 2.0–5.0. Start at 3.0 and tune.
#define KALMAN_R          3.0f

// Deadzone hysteresis — two thresholds prevent sticky cursor edge
#define DEADZONE_ENTER    5     // below this → freeze cursor
#define DEADZONE_EXIT     9     // above this → unfreeze and move
// (gap between them = hysteresis band = no flickering at boundary)

#define COORD_MAX         4095
#define SEND_INTERVAL_MS  8     // ~125Hz BLE report rate
#define DEBUG_INTERVAL_MS 100   // serial print rate (does not affect BLE)

// How many DMP frames to average for the yaw/pitch origin.
// ~60 frames ≈ 0.6 seconds. Keep glove still during this window.
#define ORIGIN_FRAMES     60

// ─────────────────────────────────────────────────────────────────────────────
//  HID DESCRIPTOR — Absolute Mouse (unchanged — it was correct)
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
//  KALMAN FILTER — 1D
//  Tracks fast motion better than EMA, calmer at rest.
// ─────────────────────────────────────────────────────────────────────────────
struct Kalman1D {
  float x;          // current estimate
  float p = 1.0f;   // estimate error covariance
  float r;          // measurement noise (tune via KALMAN_R)
  float q = 0.05f;  // process noise (how much the true value can change per step)

  Kalman1D(float measureNoise) : x(COORD_MAX / 2.0f), r(measureNoise) {}

  float update(float z) {
    p += q;                        // prediction: error grows between measurements
    float k = p / (p + r);        // Kalman gain
    x += k * (z - x);             // correction
    p *= (1.0f - k);              // update error covariance
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

BLEHIDDevice*      hid   = nullptr;
BLECharacteristic* input = nullptr;
bool               bleConnected = false;

Kalman1D kalX(KALMAN_R);
Kalman1D kalY(KALMAN_R);

// Origin — averaged over ORIGIN_FRAMES packets after calibration
float    yawOrigin   = 0, pitchOrigin = 0;
float    yawSum      = 0, pitchSum    = 0;
int      originCount = 0;
bool     originReady = false;

// Deadzone state (hysteresis)
bool     dzFrozenX = false;
bool     dzFrozenY = false;
uint16_t frozenX   = COORD_MAX / 2;
uint16_t frozenY   = COORD_MAX / 2;

unsigned long lastSend  = 0;
unsigned long lastDebug = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  BLE CALLBACKS
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
//  SURFACE PROJECTION — UNCHANGED FROM WORKING VERSION
//  Yaw + pitch → 3D direction vector → intersect with plane → screen coords
// ─────────────────────────────────────────────────────────────────────────────
void getScreenCoords(float yaw, float pitch, float &outX, float &outY) {
  float dx = cos(yaw) * cos(pitch);
  float dy = sin(yaw) * cos(pitch);
  float dz = sin(pitch);

  if (abs(dx) < 0.01f) dx = (dx >= 0) ? 0.01f : -0.01f;

  float t     = 1.0f / dx;
  float projY = dy * t;
  float projZ = dz * t;

  projY = constrain(projY, -SENSITIVITY_X, SENSITIVITY_X);
  projZ = constrain(projZ, -SENSITIVITY_Y, SENSITIVITY_Y);

  outX = (projY + SENSITIVITY_X) / (2.0f * SENSITIVITY_X) * COORD_MAX;
  outY = (SENSITIVITY_Y - projZ) / (2.0f * SENSITIVITY_Y) * COORD_MAX;
}

// ─────────────────────────────────────────────────────────────────────────────
//  DEADZONE WITH HYSTERESIS
//  Two thresholds: cursor freezes below ENTER, only unfreezes above EXIT.
//  Gap between them = no flicker at the boundary.
// ─────────────────────────────────────────────────────────────────────────────
uint16_t applyDeadzone(uint16_t current, uint16_t &frozen, bool &isFrozen) {
  int16_t delta = (int16_t)current - (int16_t)frozen;
  int16_t absDelta = abs(delta);

  if (isFrozen) {
    if (absDelta > DEADZONE_EXIT) {
      isFrozen = false;   // unfreeze — movement is intentional
    } else {
      return frozen;      // still frozen
    }
  } else {
    if (absDelta < DEADZONE_ENTER) {
      isFrozen = true;    // freeze — hand is still
      frozen   = current;
      return frozen;
    }
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
  Serial.println("\n=== Glove Mouse v4 ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.print("MPU6050... ");
  mpu.initialize();
  pinMode(MPU_INT_PIN, INPUT);

  if (!mpu.testConnection()) {
    Serial.println("FAILED — check wiring (VCC→3.3V, SDA→21, SCL→22, AD0→GND)");
    while (true) delay(1000);
  }
  Serial.println("OK");

  Serial.print("DMP... ");
  if (mpu.dmpInitialize() != 0) {
    Serial.println("FAILED");
    while (true) delay(1000);
  }

  // 10 iterations = more accurate bias correction than 6
  Serial.println("OK\nCalibrating — keep STILL for ~10 seconds...");
  mpu.CalibrateAccel(10);
  mpu.CalibrateGyro(10);

  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady   = true;
  Serial.printf("DMP ready (%d byte packets)\n", packetSize);
  Serial.printf("Setting origin — hold natural position for ~%d frames...\n", ORIGIN_FRAMES);

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
  Serial.println("=====================================");
}

// ─────────────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  if (!dmpReady) return;

  // ── FIFO overflow guard ───────────────────────────────────────────────────
  // If the buffer fills up (e.g. loop stalled), reset instead of using
  // corrupted data — prevents sudden cursor jumps and DMP freeze.
  uint16_t fifoCount = mpu.getFIFOCount();
  if (fifoCount >= 1020) {
    mpu.resetFIFO();
    Serial.println("[WARN] FIFO overflow — reset");
    return;
  }

  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

  // ── Get orientation ───────────────────────────────────────────────────────
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw   = ypr[0];
  float pitch = ypr[1];

  // ── Origin capture — average first ORIGIN_FRAMES packets ─────────────────
  // Averaging (not just first packet) eliminates DMP startup jitter.
  // Both yaw AND pitch are zeroed so cursor always starts at screen center.
  if (!originReady) {
    yawSum   += yaw;
    pitchSum += pitch;
    originCount++;
    if (originCount >= ORIGIN_FRAMES) {
      yawOrigin   = yawSum   / ORIGIN_FRAMES;
      pitchOrigin = pitchSum / ORIGIN_FRAMES;
      originReady = true;
      kalX.seed(COORD_MAX / 2.0f);
      kalY.seed(COORD_MAX / 2.0f);
      frozenX = frozenY = COORD_MAX / 2;
      Serial.printf("[OK] Origin → Yaw: %.2f°  Pitch: %.2f°\n",
                    degrees(yawOrigin), degrees(pitchOrigin));
      Serial.println("     Cursor centered. Move!");
    }
    return;
  }

  // Apply origin offsets
  yaw   -= yawOrigin;
  pitch -= pitchOrigin;

  // ── Surface projection (unchanged algorithm) ──────────────────────────────
  float rawX, rawY;
  getScreenCoords(yaw, pitch, rawX, rawY);

  // ── Kalman smooth ─────────────────────────────────────────────────────────
  float smoothX = kalX.update(rawX);
  float smoothY = kalY.update(rawY);

  uint16_t finalX = (uint16_t)constrain(smoothX, 0.0f, (float)COORD_MAX);
  uint16_t finalY = (uint16_t)constrain(smoothY, 0.0f, (float)COORD_MAX);

  // ── Deadzone with hysteresis ──────────────────────────────────────────────
  finalX = applyDeadzone(finalX, frozenX, dzFrozenX);
  finalY = applyDeadzone(finalY, frozenY, dzFrozenY);

  // ── Send BLE HID report ───────────────────────────────────────────────────
  unsigned long now = millis();
  if (bleConnected && (now - lastSend >= SEND_INTERVAL_MS)) {
    sendReport(0x00, finalX, finalY);
    lastSend = now;
  }

  // ── Serial debug — throttled so it doesn't affect BLE timing ─────────────
  if (now - lastDebug >= DEBUG_INTERVAL_MS) {
    Serial.printf("Yaw:%6.1f°  Pitch:%6.1f°  →  X:%4d  Y:%4d  [%s]\n",
      degrees(yaw), degrees(pitch),
      finalX, finalY,
      bleConnected ? "CONN" : "----"
    );
    lastDebug = now;
  }
}

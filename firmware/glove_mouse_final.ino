/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║          GLOVE MOUSE — ESP32 + MPU6050 DMP               ║
 * ║          Full build: motion + left/right click + reset   ║
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
 *   TTP223 #1 (LEFT CLICK)   OUT → GPIO 4  | VCC → 3.3V | GND → GND
 *   TTP223 #2 (RIGHT CLICK)  OUT → GPIO 5  | VCC → 3.3V | GND → GND
 *
 *   Both tapped simultaneously = reset cursor to center
 *
 * LIBRARIES NEEDED (install via Arduino Library Manager):
 *   1. "MPU6050" by ElectronicCats
 *   2. ESP32 BLE (built into ESP32 Arduino core)
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
#define TOUCH_LEFT    4   // TTP223 #1 — left click
#define TOUCH_RIGHT   5   // TTP223 #2 — right click

// ─────────────────────────────────────────────────────────────────────────────
//  TUNING
// ─────────────────────────────────────────────────────────────────────────────
#define SENSITIVITY      0.5f
#define SMOOTHING        0.1f
#define COORD_MAX        4095
#define SEND_INTERVAL_MS 8

// ─────────────────────────────────────────────────────────────────────────────
//  HID DESCRIPTOR — Absolute Mouse, Report ID 1
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t hidDescriptor[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x02,        // Usage (Mouse)
  0xA1, 0x01,        // Collection (Application)
  0x09, 0x01,        //   Usage (Pointer)
  0xA1, 0x00,        //   Collection (Physical)
  0x85, 0x01,        //     Report ID (1)
  0x05, 0x09,        //     Usage Page (Button)
  0x19, 0x01,        //     Usage Minimum (Button 1)
  0x29, 0x03,        //     Usage Maximum (Button 3)
  0x15, 0x00,        //     Logical Minimum (0)
  0x25, 0x01,        //     Logical Maximum (1)
  0x75, 0x01,        //     Report Size (1 bit)
  0x95, 0x03,        //     Report Count (3)
  0x81, 0x02,        //     Input (Data, Variable, Absolute)
  0x75, 0x05,        //     Report Size (5 bits) — padding
  0x95, 0x01,        //     Report Count (1)
  0x81, 0x03,        //     Input (Constant)
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x30,        //     Usage (X)
  0x09, 0x31,        //     Usage (Y)
  0x15, 0x00,        //     Logical Minimum (0)
  0x26, 0xFF, 0x0F,  //     Logical Maximum (4095)
  0x75, 0x10,        //     Report Size (16 bits)
  0x95, 0x02,        //     Report Count (2)
  0x81, 0x02,        //     Input (Data, Variable, Absolute)
  0xC0,              //   End Collection
  0xC0               // End Collection
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

float smoothX = COORD_MAX / 2.0f;
float smoothY = COORD_MAX / 2.0f;

float yawOffset   = 0.0f;
float pitchOffset = 0.0f;
bool  offsetSet   = false;

unsigned long lastSend = 0;

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
//  SURFACE PROJECTION
// ─────────────────────────────────────────────────────────────────────────────
void getScreenCoords(float yaw, float pitch, float &outX, float &outY) {
  float dx = cos(yaw) * cos(pitch);
  float dy = sin(yaw) * cos(pitch);
  float dz = sin(pitch);

  if (abs(dx) < 0.01f) dx = (dx >= 0) ? 0.01f : -0.01f;

  float t     = SENSITIVITY / dx;
  float projY = constrain(dy * t, -SENSITIVITY, SENSITIVITY);
  float projZ = constrain(dz * t, -SENSITIVITY, SENSITIVITY);

  outX = (projY + SENSITIVITY) / (2.0f * SENSITIVITY) * COORD_MAX;
  outY = (SENSITIVITY - projZ) / (2.0f * SENSITIVITY) * COORD_MAX;
}

// ─────────────────────────────────────────────────────────────────────────────
//  EMA
// ─────────────────────────────────────────────────────────────────────────────
inline float ema(float prev, float next, float alpha) {
  return prev + alpha * (next - prev);
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
  Serial.println("\n=== Glove Mouse Boot ===");

  pinMode(TOUCH_LEFT,  INPUT);
  pinMode(TOUCH_RIGHT, INPUT);

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

  Serial.println("OK\nCalibrating — keep glove still...");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady   = true;
  Serial.printf("DMP ready. Packet size: %d bytes\n", packetSize);

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
  Serial.println("==============================");
}

// ─────────────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  if (!dmpReady) return;
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

  // ── Orientation ───────────────────────────────────────────────────────────
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw   = ypr[0];
  float pitch = ypr[2];

  // ── Zero on first packet ──────────────────────────────────────────────────
  if (!offsetSet) {
    yawOffset   = yaw;
    pitchOffset = pitch;
    offsetSet   = true;
    Serial.println("Yaw + Pitch zeroed.");
  }
  yaw   -= yawOffset;
  pitch -= pitchOffset;

  // ── Surface projection ────────────────────────────────────────────────────
  float rawX, rawY;
  getScreenCoords(yaw, pitch, rawX, rawY);

  // ── Smooth ────────────────────────────────────────────────────────────────
  smoothX = ema(smoothX, rawX, SMOOTHING);
  smoothY = ema(smoothY, rawY, SMOOTHING);

  uint16_t finalX = (uint16_t)constrain(smoothX, 0, COORD_MAX);
  uint16_t finalY = (uint16_t)constrain(smoothY, 0, COORD_MAX);

  // ── Touch sensors ─────────────────────────────────────────────────────────
  bool leftPressed  = digitalRead(TOUCH_LEFT);
  bool rightPressed = digitalRead(TOUCH_RIGHT);

  // Both = snap to center
  if (leftPressed && rightPressed) {
    smoothX = COORD_MAX / 2.0f;
    smoothY = COORD_MAX / 2.0f;
    finalX  = COORD_MAX / 2;
    finalY  = COORD_MAX / 2;
    Serial.println("RESET to center");
  }

  uint8_t buttons = 0;
  if (leftPressed  && !rightPressed) buttons |= 0x01;  // left click
  if (rightPressed && !leftPressed)  buttons |= 0x02;  // right click

  // ── Send ──────────────────────────────────────────────────────────────────
  unsigned long now = millis();
  if (bleConnected && (now - lastSend >= SEND_INTERVAL_MS)) {
    sendReport(buttons, finalX, finalY);
    lastSend = now;
  }

  // ── Serial debug ──────────────────────────────────────────────────────────
  Serial.printf("Yaw:%6.1f  Pitch:%6.1f  X:%4d  Y:%4d  L:%d R:%d  [%s]\n",
    degrees(yaw), degrees(pitch),
    finalX, finalY,
    leftPressed, rightPressed,
    bleConnected ? "CONN" : "----"
  );
}

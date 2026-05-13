/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║          GLOVE MOUSE — ESP32 + MPU6050 DMP               ║
 * ║          Movement test (no touch sensor)                 ║
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
 * LIBRARIES NEEDED (install via Arduino Library Manager):
 *   1. "MPU6050" by ElectronicCats
 *   2. ESP32 BLE (built into ESP32 Arduino core)
 *
 * ALGORITHM (Glove++ surface projection):
 *   Quaternion → Yaw/Pitch → 3D direction vector
 *   → Intersect with imaginary screen plane → (X, Y) coords
 *   → Smooth with EMA → Send as absolute BLE HID mouse
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
#define SDA_PIN      21   // I2C data
#define SCL_PIN      22   // I2C clock
#define MPU_INT_PIN  19   // MPU6050 interrupt (DATA_READY)

// ─────────────────────────────────────────────────────────────────────────────
//  TUNING — adjust these while testing
// ─────────────────────────────────────────────────────────────────────────────

// SENSITIVITY: "distance" to the imaginary screen in front of you.
// Higher = larger hand movements needed to reach screen edges.
// Start at 1.5, increase if cursor feels too jumpy.
#define SENSITIVITY  1.5f

// SMOOTHING: Exponential Moving Average weight (0.0 – 1.0).
// Lower = heavier smoothing (laggy but stable).
// Higher = faster response (twitchy). Start at 0.2.
#define SMOOTHING    0.2f

// Absolute coordinate range sent to host (must match HID descriptor)
#define COORD_MAX    4095

// Report rate — 8ms = ~125Hz
#define SEND_INTERVAL_MS  8

// ─────────────────────────────────────────────────────────────────────────────
//  HID DESCRIPTOR — Absolute Mouse, Report ID 1
//  Tells the host: "I am a mouse that sends absolute X/Y from 0 to 4095"
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t hidDescriptor[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x02,        // Usage (Mouse)
  0xA1, 0x01,        // Collection (Application)
  0x09, 0x01,        //   Usage (Pointer)
  0xA1, 0x00,        //   Collection (Physical)
  0x85, 0x01,        //     Report ID (1)

  // ── Buttons: 3 bits + 5 padding ──
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

  // ── X, Y absolute position: 16-bit each, 0–4095 ──
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

// DMP state
bool     dmpReady   = false;
uint16_t packetSize = 0;
uint8_t  fifoBuffer[64];

// Motion data
Quaternion  q;
VectorFloat gravity;
float       ypr[3];   // [0]=yaw  [1]=pitch  [2]=roll

// BLE
BLEHIDDevice*      hid   = nullptr;
BLECharacteristic* input = nullptr;
bool               bleConnected = false;

// EMA state — start at screen center
float smoothX = COORD_MAX / 2.0f;
float smoothY = COORD_MAX / 2.0f;

// Yaw zero reference (set at first valid packet)
float yawOffset = 0.0f;
bool  offsetSet = false;

unsigned long lastSend = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  BLE SERVER CALLBACKS
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
//  Takes yaw + pitch, returns absolute screen coords (0–COORD_MAX).
//
//  Math:
//    Build a 3D unit direction vector from the hand's orientation.
//    Find where that vector hits a plane at distance SENSITIVITY.
//    The hit point's Y and Z become our screen X and Y.
// ─────────────────────────────────────────────────────────────────────────────
void getScreenCoords(float yaw, float pitch, float &outX, float &outY) {
  // 3D direction vector
  float dx = cos(yaw) * cos(pitch);
  float dy = sin(yaw) * cos(pitch);
  float dz = sin(pitch);

  // Avoid divide-by-zero if hand is pointing 90° sideways
  if (abs(dx) < 0.01f) dx = (dx >= 0) ? 0.01f : -0.01f;

  // Parameter t: how far along the ray until we hit x = SENSITIVITY
  float t = SENSITIVITY / dx;

  // Intersection point on the imaginary plane
  float projY = dy * t;
  float projZ = dz * t;

  // Clamp to ±SENSITIVITY (beyond that is off-screen)
  projY = constrain(projY, -SENSITIVITY, SENSITIVITY);
  projZ = constrain(projZ, -SENSITIVITY, SENSITIVITY);

  // Map to 0–COORD_MAX
  // Y axis: right = higher X  |  Z axis: up = lower Y (screen coords)
  outX = (projY + SENSITIVITY) / (2.0f * SENSITIVITY) * COORD_MAX;
  outY = (SENSITIVITY - projZ) / (2.0f * SENSITIVITY) * COORD_MAX;  // flipped
}

// ─────────────────────────────────────────────────────────────────────────────
//  EXPONENTIAL MOVING AVERAGE
// ─────────────────────────────────────────────────────────────────────────────
inline float ema(float prev, float next, float alpha) {
  return prev + alpha * (next - prev);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SEND HID REPORT
//  Report layout (5 bytes):
//    [0]   buttons (bit0=left, bit1=right, bit2=middle)
//    [1,2] X low byte, high byte
//    [3,4] Y low byte, high byte
// ─────────────────────────────────────────────────────────────────────────────
void sendReport(uint8_t buttons, uint16_t x, uint16_t y) {
  uint8_t val[5] = {
    buttons,
    (uint8_t)(x & 0xFF),  (uint8_t)(x >> 8),
    (uint8_t)(y & 0xFF),  (uint8_t)(y >> 8)
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

  // ── I2C ───────────────────────────────────────────────────────────────────
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);   // Fast mode: 400 kHz

  // ── MPU6050 ───────────────────────────────────────────────────────────────
  Serial.print("MPU6050 init... ");
  mpu.initialize();
  pinMode(MPU_INT_PIN, INPUT);

  if (!mpu.testConnection()) {
    Serial.println("FAILED. Check wiring!");
    Serial.println("  VCC→3.3V  GND→GND  SDA→21  SCL→22  AD0→GND");
    while (true) delay(1000);
  }
  Serial.println("OK");

  // ── DMP ───────────────────────────────────────────────────────────────────
  Serial.print("DMP init... ");
  uint8_t status = mpu.dmpInitialize();

  if (status != 0) {
    Serial.printf("FAILED (code %d)\n", status);
    Serial.println("Common fix: check I2C wiring, try 3.3V not 5V");
    while (true) delay(1000);
  }

  // Auto-calibrate (keeps the sensor still for ~3 seconds)
  Serial.println("OK\nCalibrating — keep glove still...");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady   = true;
  Serial.printf("DMP ready. Packet size: %d bytes\n", packetSize);

  // ── BLE ───────────────────────────────────────────────────────────────────
  Serial.print("BLE init... ");
  BLEDevice::init("GloveMouse");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  hid   = new BLEHIDDevice(server);
  input = hid->inputReport(1);   // matches Report ID 1 in descriptor

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
  Serial.println("Advertising as 'GloveMouse' — connect from your PC/phone");
  Serial.println("==============================");
}

// ─────────────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  if (!dmpReady) return;

  // Wait for a complete DMP packet in the FIFO
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

  // ── Get orientation ───────────────────────────────────────────────────────
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw   = ypr[0];
  float pitch = ypr[1];
  // float roll = ypr[2];  // reserved for scroll later

  // ── Zero the yaw on first packet ─────────────────────────────────────────
  // So wherever the hand is pointing at boot = screen center
  if (!offsetSet) {
    yawOffset = yaw;
    offsetSet = true;
    Serial.println("Yaw zeroed to current position.");
  }
  yaw -= yawOffset;

  // ── Project onto imaginary surface ───────────────────────────────────────
  float rawX, rawY;
  getScreenCoords(yaw, pitch, rawX, rawY);

  // ── Smooth ────────────────────────────────────────────────────────────────
  smoothX = ema(smoothX, rawX, SMOOTHING);
  smoothY = ema(smoothY, rawY, SMOOTHING);

  uint16_t finalX = (uint16_t)constrain(smoothX, 0, COORD_MAX);
  uint16_t finalY = (uint16_t)constrain(smoothY, 0, COORD_MAX);

  // ── Send HID report ───────────────────────────────────────────────────────
  unsigned long now = millis();
  if (bleConnected && (now - lastSend >= SEND_INTERVAL_MS)) {
    sendReport(0x00, finalX, finalY);   // 0x00 = no buttons pressed
    lastSend = now;
  }

  // ── Serial debug ─────────────────────────────────────────────────────────
  Serial.printf("Yaw:%6.1f°  Pitch:%6.1f°  →  X:%4d  Y:%4d  [BLE:%s]\n",
    degrees(yaw), degrees(pitch),
    finalX, finalY,
    bleConnected ? "CONN" : "----"
  );
}

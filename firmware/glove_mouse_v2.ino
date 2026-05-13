/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║          GLOVE MOUSE — ESP32 + MPU6050 DMP               ║
 * ║          v2 — No yaw, stable, deadzoned                  ║
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

// How wide your hand motion range is (radians).
// Increase if cursor hits edges too easily.
// Decrease if you need big movements to reach edges.
#define MOTION_RANGE   1.0f

// Smoothing 0.0–1.0. Lower = more stable but slightly laggy.
#define SMOOTHING      0.1f

// Deadzone — movements smaller than this are ignored (kills tremor).
// Increase if still shaky, decrease if cursor feels stuck.
#define DEADZONE       30

// Absolute coordinate range (must match HID descriptor)
#define COORD_MAX      4095

// Report rate
#define SEND_INTERVAL_MS  8

// ─────────────────────────────────────────────────────────────────────────────
//  HID DESCRIPTOR — Absolute Mouse
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t hidDescriptor[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x02,        // Usage (Mouse)
  0xA1, 0x01,        // Collection (Application)
  0x09, 0x01,        //   Usage (Pointer)
  0xA1, 0x00,        //   Collection (Physical)
  0x85, 0x01,        //     Report ID (1)

  // Buttons: 3 bits + 5 padding
  0x05, 0x09,        //     Usage Page (Button)
  0x19, 0x01,        //     Usage Minimum (Button 1)
  0x29, 0x03,        //     Usage Maximum (Button 3)
  0x15, 0x00,        //     Logical Minimum (0)
  0x25, 0x01,        //     Logical Maximum (1)
  0x75, 0x01,        //     Report Size (1 bit)
  0x95, 0x03,        //     Report Count (3)
  0x81, 0x02,        //     Input (Data, Variable, Absolute)
  0x75, 0x05,        //     Report Size (5 bits) padding
  0x95, 0x01,        //     Report Count (1)
  0x81, 0x03,        //     Input (Constant)

  // X, Y absolute: 16-bit each, 0–4095
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
float       ypr[3];   // [0]=yaw  [1]=pitch  [2]=roll

BLEHIDDevice*      hid   = nullptr;
BLECharacteristic* input = nullptr;
bool               bleConnected = false;

// Start at screen center
float smoothX = COORD_MAX / 2.0f;
float smoothY = COORD_MAX / 2.0f;

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
//  SCREEN COORDS — pitch and roll only, yaw completely ignored
//
//  pitch: left/right tilt of hand  → X axis
//  roll:  up/down tilt of hand     → Y axis
//  yaw:   wrist rotation           → IGNORED
// ─────────────────────────────────────────────────────────────────────────────
void getScreenCoords(float pitch, float roll, float &outX, float &outY) {
  // Map pitch and roll from [-MOTION_RANGE, +MOTION_RANGE] → [0, COORD_MAX]
  outX = (pitch / MOTION_RANGE + 1.0f) / 2.0f * COORD_MAX;
  outY = (roll  / MOTION_RANGE + 1.0f) / 2.0f * COORD_MAX;

  // If up/down feels inverted, swap the sign on roll:
  // outY = (-roll / MOTION_RANGE + 1.0f) / 2.0f * COORD_MAX;

  outX = constrain(outX, 0, COORD_MAX);
  outY = constrain(outY, 0, COORD_MAX);
}

// ─────────────────────────────────────────────────────────────────────────────
//  EXPONENTIAL MOVING AVERAGE
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
  Serial.println("\n=== Glove Mouse v2 Boot ===");

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // MPU6050
  Serial.print("MPU6050 init... ");
  mpu.initialize();
  pinMode(MPU_INT_PIN, INPUT);

  if (!mpu.testConnection()) {
    Serial.println("FAILED. Check wiring!");
    Serial.println("  VCC→3.3V  GND→GND  SDA→21  SCL→22  AD0→GND");
    while (true) delay(1000);
  }
  Serial.println("OK");

  // DMP
  Serial.print("DMP init... ");
  uint8_t status = mpu.dmpInitialize();

  if (status != 0) {
    Serial.printf("FAILED (code %d)\n", status);
    while (true) delay(1000);
  }

  // Calibration — keep glove completely still for ~8 seconds
  Serial.println("OK\nCalibrating — keep glove STILL for 8 seconds...");
  mpu.CalibrateAccel(15);
  mpu.CalibrateGyro(15);

  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady   = true;
  Serial.printf("DMP ready. Packet size: %d bytes\n", packetSize);

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
  Serial.println("Advertising as 'GloveMouse' — pair from Bluetooth settings");
  Serial.println("==========================================");
}

// ─────────────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  if (!dmpReady) return;

  // Wait for DMP packet
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

  // Get orientation
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // Yaw = ypr[0] — completely ignored
  float pitch = ypr[1];   // left/right hand tilt → X
  float roll  = ypr[2];   // up/down hand tilt   → Y

  // Project to screen coordinates
  float rawX, rawY;
  getScreenCoords(pitch, roll, rawX, rawY);

  // Deadzone — ignore micro-movements (tremor filter)
  if (abs(rawX - smoothX) < DEADZONE) rawX = smoothX;
  if (abs(rawY - smoothY) < DEADZONE) rawY = smoothY;

  // Smooth
  smoothX = ema(smoothX, rawX, SMOOTHING);
  smoothY = ema(smoothY, rawY, SMOOTHING);

  uint16_t finalX = (uint16_t)constrain(smoothX, 0, COORD_MAX);
  uint16_t finalY = (uint16_t)constrain(smoothY, 0, COORD_MAX);

  // Send HID report
  unsigned long now = millis();
  if (bleConnected && (now - lastSend >= SEND_INTERVAL_MS)) {
    sendReport(0x00, finalX, finalY);
    lastSend = now;
  }

  // Serial debug
  Serial.printf("Pitch:%6.1f°  Roll:%6.1f°  →  X:%4d  Y:%4d  [BLE:%s]\n",
    degrees(pitch), degrees(roll),
    finalX, finalY,
    bleConnected ? "CONN" : "----"
  );
}

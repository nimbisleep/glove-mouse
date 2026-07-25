#  Glove Mouse — Wireless Gesture-Controlled HID

**Graduation Project (Bitirme)** — Düzce University, EEE Department

A wireless glove mouse controlled by hand gestures. Uses an **ESP32** with **MPU6050 IMU (DMP)** for drift-free cursor tracking over **Bluetooth BLE**. No receiver dongle needed — pairs like any standard Bluetooth mouse.

---

##  Features

- **Drift-free cursor** — MPU6050 DMP (Digital Motion Processor) handles all orientation math on-chip. Pitch moves cursor up/down, roll moves left/right. No accumulated drift.
- **Bluetooth BLE** — ESP32 built-in BLE, pairs natively with Windows, macOS, Linux, Android
- **3 Modes:**
  - **Fix** — cursor locked in place (thumb capacitive touch)
  - **Air Tap** — capacitive trigger for click
  - **Inactive** — hand resting, no cursor movement
- **Adjustable sensitivity** — dynamic deadzone filtering
- **Custom PCB** — KiCad-designed, LTspice-validated
- **Battery-powered** — portable, wearable form factor

---

## 📁 Repository Structure

```
glove-mouse/
├── firmware/                  # ESP32 Arduino firmware
│   ├── glove_mouse_v5.ino     # Latest — DMP-based, gravity anchored
│   ├── glove_mouse_dmp.ino    # DMP firmware variant
│   ├── GloveMouse_Movement.ino
│   ├── glove_mouse_final.ino
│   ├── glove_mouse_v4.ino
│   ├── glove_mouse_v3.ino
│   └── glove_mouse_v2.ino
├── hardware/
│   └── schematics/            # KiCad PCB design files
│       ├── schematics.kicad_pcb    # PCB layout
│       ├── schematics.kicad_sch    # Schematic
│       ├── schematics.kicad_pro    # Project file
│       └── *.gbr                  # Gerber files for fabrication
└── README.md
```

---

##  Hardware

| Component | Purpose |
|-----------|---------|
| ESP32 WROOM | Main MCU + BLE |
| MPU6050 | 6-axis IMU with DMP |
| TTP223 | Capacitive touch for click/mode |
| Flex sensors (x5) | Finger bend detection (WIP) |
| Custom PCB | KiCad-designed 2-layer board |

---

##  Firmware

The firmware uses the MPU6050's built-in DMP (Digital Motion Processor) to get stable, drift-free orientation data. The ESP32 handles BLE HID mouse reporting.

**Key firmware files:**
- `glove_mouse_v5.ino` — Latest version. Uses gravity-anchored pitch/roll, no yaw (drifts). Includes sensitivity tuning.
- `glove_mouse_dmp.ino` — DMP implementation with raw quaternion output.

### Wiring
```
MPU6050 VCC  → ESP32 3.3V
MPU6050 GND  → ESP32 GND
MPU6050 SDA  → ESP32 GPIO 21
MPU6050 SCL  → ESP32 GPIO 22
MPU6050 INT  → ESP32 GPIO 19
MPU6050 AD0  → GND
```

### Required Libraries
- `MPU6050` by ElectronicCats
- ESP32 BLE (built into ESP32 Arduino core)

---

##  PCB (KiCad)

The custom PCB is designed in KiCad as a 2-layer board:
- ESP32 footprint with all required passives
- MPU6050 IMU with I2C pull-ups and decoupling
- TTP223 capacitive touch pad
- Battery management circuit
- Breakout headers for flex sensors

Gerber files (`*.gbr`) are included for direct fabrication.

---

##  Status

| Component | Status |
|-----------|--------|
| ✅ Movement tracking | Working via BLE |
| ✅ Click/gesture logic | Implemented |
| ✅ KiCad PCB | Designed |
| ✅ Flex sensors | Added |
| ✅ Final assembly | Glove mounting + battery |

---

##  Contact

**Salah Hussein Osman**  
saalahsega01@gmail.com

---

*Presented at Düzce University Bitirme Projeleri Sunum ve Poster Sergisi — 21 May 2026*

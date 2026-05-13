# 🧤 Glove Mouse — Bitirme Projesi

**Final Year Graduation Project** — Düzce University, EEE Department

A glove-based mouse controller using flex sensors, IMU, and Bluetooth. Control your cursor with hand gestures.

## Features

- **Hand Tracking** — IMU (MPU6050) for orientation and movement
- **Gesture Recognition** — Flex sensors detect finger bends for clicks and gestures
- **Bluetooth** — HC-05 module for wireless communication
- **Custom PCB** — KiCad-designed PCB (ongoing)
- **Real-time Control** — Smooth cursor movement mapped to hand motion

## Hardware

| Component | Purpose |
|-----------|---------|
| Flex Sensors (x5) | Finger bend detection |
| MPU6050 (IMU) | Hand orientation & motion |
| HC-05 | Bluetooth communication |
| Arduino/STM32 | Microcontroller |
| Custom PCB | KiCad design (in progress) |
| Battery | Power supply (to be integrated) |

## Software

- **Firmware:** C/C++ (Arduino/STM32)
- **Communication:** Serial over Bluetooth
- **Host:** Python receiver script → system cursor control

## Status

- ✅ Movement tracking working via Bluetooth
- ✅ Gesture detection logic complete
- ✅ Hand tracking algorithm working
- ⏳ Flex sensors — awaiting delivery from supplier
- ⏳ Custom PCB — design in KiCad
- ⏳ Final assembly — glove mounting + battery integration

## Poster & Presentation

Presenting at the university's **Bitirme Projeleri Sunum ve Poster Sergisi** on **21 May 2026**.

---

**Supervisor:** (insert advisor name)
**Student:** Salah Hussein Osman

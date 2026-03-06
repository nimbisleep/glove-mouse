# glove-mouse
final year project ,-bitirme projesi
# Glove Mouse 🖱️

A TÜBİTAK final year project — a wearable glove that replaces
the traditional mouse using hand gestures.

## How It Works
- MPU6050 IMU tracks wrist movement for cursor control
- Flex sensors detect finger positions for mode switching
- Arduino Leonardo emulates a standard USB-HID mouse
- No drivers, no software — plug and play on any computer

## Modes
| Mode | Gesture | Function |
|------|---------|----------|
| Fix Mode | Fist | Cursor movement & scrolling |
| Air Tap Mode | Index finger up | Left click / Right click |
| Inactive Mode | Open hand | No action — prevents accidents |

## Components
- Arduino Leonardo
- MPU6050 IMU (I2C)
- Flex sensors x5
- Glove

## Built With
- C (Arduino firmware)
- KiCad (schematic design)
- USB-HID protocol

## Status
🔄 In progress — TÜBİTAK application submitted, result expected April 2025

---

# Eldiven Mouse 🖱️

TÜBİTAK bitirme projesi — el hareketleriyle geleneksel fareyi
simüle eden giyilebilir bir eldiven.

## Nasıl Çalışır
- MPU6050 IMU, bilek hareketini takip ederek imleç kontrolü sağlar
- Flex sensörler parmak pozisyonlarını algılayarak mod geçişini kontrol eder
- Arduino Leonardo standart bir USB-HID fare olarak emüle edilir
- Sürücü gerekmez — her bilgisayarda tak-çalıştır

## Modlar
| Mod | Jest | Fonksiyon |
|-----|------|-----------|
| Fix Mod | Yumruk | İmleç hareketi & kaydırma |
| Air Tap Mod | İşaret parmağı yukarı | Sol tık / Sağ tık |
| Inactive Mod | Açık el | Eylem yok — yanlışlıkla aktivasyonu önler |

## Bileşenler
- Arduino Leonardo
- MPU6050 IMU (I2C)
- Flex sensör x5
- Eldiven

## Kullanılan Teknolojiler
- C (Arduino firmware)
- KiCad (devre şeması)
- USB-HID protokolü

## Durum
🔄 Devam ediyor — TÜBİTAK başvurusu yapıldı, Nisan 2025 sonucu bekleniyor

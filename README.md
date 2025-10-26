# Poti360 Library

**Poti360** is a C++ library for microcontroller projects (e.g. ESP32, STM32, Arduino) that simplifies the evaluation of ALPHA TAIWANs 360-degree potentiometers (RV112FF).  
It provides the current angle, the rotational speed (in rad/s) and, optionally, the acceleration (in rad/s²).

---

## 🔧 Features

- Support for RV112FF potentiometers (NOT encoders!)
- Calculation of:
- Absolute angle (0 – 360°)
  - Angular velocity (rad/s)
  - Acceleration (rad/s²)
- Automatic overflow correction (0 ↔ 360° transition)
- hysteresis for noise reduction (0.15°)
- Compatible with **Arduino Core**, **ESP32**, **STM32**

## ⚙️ Installation

### 1. Via GitHub directly in PlatformIO:

Add the following to your `platformio.ini`:
```ini
lib_deps =
    https://github.com/<yourName>/Poti360.git


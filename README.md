# SCUBA Intermediate Pressure Monitor

A real-time intermediate pressure (IP) analyzer for scuba diving equipment, 
built on the Waveshare ESP32-S3-Touch-LCD-7 development board. Displays live 
PSI readings, scrolling waveform history, flow rate, and calibration tools 
on an 800×480 capacitive touchscreen.

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU Board | Waveshare ESP32-S3-Touch-LCD-7 Rev 1.2 |
| SoC | ESP32-S3, dual-core LX7 @ 240MHz |
| Flash | 8MB QIO |
| PSRAM | 8MB OPI |
| Display | 800×480 RGB565 parallel panel (EK9716B/ST7262) |
| Touch | GT911 capacitive, 5-point, I2C @ 0x5D |
| IO Expander | CH422G via I2C @ 0x24 (backlight, LCD reset, touch reset) |
| Pressure Sensor | FUSCH 0–300 PSI, 1/8" NPT, 0.5–4.5V analog output |
| Signal Conditioning | 10kΩ/20kΩ resistor voltage divider on GPIO2 |

---

## Features

- **25 Hz live pressure readout** with color-coded zone indicators
  - Green: Normal (0–99 PSI)
  - Yellow: Elevated (100–179 PSI)
  - Orange: High Zone (180–249 PSI)
  - Red: Critical (250+ PSI)
- **800-sample scrolling waveform graph** with touch tooltip
- **PSI/minute flow rate bar** with directional indicator
- **IP Baseline tracking** with peak-drop measurement
- **MIN/MAX statistics** with session reset
- **Units toggle** — PSI / mbar
- **ATMO CAL** — atmospheric offset calibration saved to NVS flash
- **2-Point Calibration** — voltage-to-PSI linear mapping saved to NVS flash
- **CSV serial output** at 115200 baud for data logging

---

## Software Stack

| Component | Library/Version |
|-----------|----------------|
| Framework | Arduino (ESP32) |
| Display Driver | LovyanGFX |
| Touch Driver | TAMC_GT911 |
| IO Expander | ESP32_Display_Panel / ESP_IOExpander |
| Storage | ESP32 Preferences (NVS) |
| Build System | PlatformIO |

---

## Current Development Status

### ✅ Completed
- Full application code written and compiling
- Sensor reading, calibration, NVS save/load
- All UI drawing functions (header, metrics, graph, flow bar, buttons)
- Touch handling for all buttons and graph tooltip
- 2-point calibration screen
- CSV serial output confirmed working
- Successful firmware upload via PlatformIO

### 🔴 Blocking Issue — Display Not Rendering
The firmware uploads and runs correctly (confirmed via serial output — all 
lifecycle checkpoints print, CSV data streams at 25Hz). However, the 800×480 
RGB parallel display produces no output.

**Confirmed working:**
- ESP32-S3 boots and runs application code
- Serial output confirmed via CH343P UART port
- Pressure sensor ADC readings correct
- CH422G IO expander partially responds (backlight intermittent)

**Root cause under investigation:**
The display uses an RGB parallel interface driven by ESP32-S3 LCD peripheral 
with DMA framebuffer in OPI PSRAM. `lcd.init()` completes without crash and 
returns size 800×480, but no pixels appear on panel.

Suspected issues:
1. LovyanGFX `Panel_RGB` framebuffer DMA initialization with OPI PSRAM
2. CH422G I2C driver conflict between Arduino Wire and ESP-IDF driver_ng
3. RGB panel clock/sync timing parameters

**Help wanted** — see Issues tab.

---

## Build Environment

```ini
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.37/platform-espressif32.zip
board = esp32-s3-devkitc-1
framework = arduino
board_build.psram_type = opi
board_build.arduino.memory_type = qio_opi


Real-time scuba intermediate pressure monitor built on the Waveshare ESP32-S3-Touch-LCD-7. Features live PSI readout, scrolling waveform graph, flow rate tracking, 2-point calibration, and touch UI. Currently debugging RGB parallel display initialization with OPI PSRAM.

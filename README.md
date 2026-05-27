# CEC 24-pin Module (ESP-IDF port)

This is the ESP-IDF port of the CEC 24-pin module firmware, replacing the Arduino-ESP32 framework used in the v0.5.x lineage.

The original Arduino-ESP32 firmware at v0.5.9 remains the working backup until this port has been validated against captured-data behavior parity.

## Project layout

```
cec-24pin-idf/
├── CMakeLists.txt              Project-level CMake config
├── sdkconfig.defaults          ESP-IDF defaults (PSRAM, flash size, etc.)
├── partitions.csv              Custom partition table with cec_data namespace
├── main/
│   ├── CMakeLists.txt          Main component config
│   └── main.c                  Entry point (app_main)
└── components/
    ├── cec_sensors/            Hardware abstractions: INA226, ACS712, NTC
    ├── cec_detection/          State classifier, swing detectors, profiles
    └── cec_capture/            Burst capture engine, pre-trigger buffer
```

Component-based layout is intentional. Each component is hardware-agnostic where possible so it can be reused in the planned ESP32-P4 firmware for the 12VHPWR module.

## Build

```sh
# First-time setup: select target and apply defaults
idf.py set-target esp32s3
idf.py menuconfig    # optional, for tweaking the defaults

# Build
idf.py build

# Flash and monitor (replace PORT with your serial device)
idf.py -p /dev/ttyACM0 flash monitor
```

On macOS the port is typically `/dev/cu.usbmodem*`. On Windows it's `COMx`.

To exit the monitor: Ctrl-]

## Porting status

Tracking the port progress from Arduino-ESP32 v0.5.9 to ESP-IDF:

| Component | Status |
|---|---|
| Project skeleton | Done |
| Hello World boot | Pending hardware test |
| I2C master driver (for INA226) | Done |
| ADC oneshot reads — rail voltages (12V/5V/3V3) | Done |
| ADC oneshot reads — NTC temperature | Done |
| ADC oneshot reads — rail currents (i_12V/i_5V/i_3V3 via ACS712) | Pending |
| ADC continuous mode (for HS burst) | Pending |
| Sample loop at 50 Hz | Done |
| EMA / median filter primitives | Done |
| Teleplot output via USB CDC | Done |
| State classifier | Pending |
| Layer 1/2/3 detectors | Pending |
| Burst capture engine | Pending |
| NVS profile storage | Pending |
| Serial command interface | Pending |

## Behavior parity check

Once each component lands, run side-by-side captures against the v0.5.9 baseline on the same PSU and same workloads, and compare the resulting bursts. Any divergence beyond measurement noise is a porting bug to track down before that component is considered done.

## API migration reference

For each Arduino-ESP32 API the v0.5.9 firmware uses, the ESP-IDF equivalent:

| Arduino-ESP32 | ESP-IDF |
|---|---|
| `Serial.begin()`, `Serial.printf()` | `esp_log_*` macros (`ESP_LOGI`, etc.) or `uart_driver_install` + `uart_write_bytes` |
| `Wire.begin()`, `Wire.requestFrom()` | `i2c_master_*` driver (new in IDF 5.x) |
| `analogReadMilliVolts()` | `adc_oneshot_*` driver |
| `analogReadResolution()`, `analogSetAttenuation()` | `adc_oneshot_config_channel` |
| `millis()` | `esp_timer_get_time() / 1000` (returns int64_t microseconds, divide for ms) |
| `delay()` | `vTaskDelay(pdMS_TO_TICKS(ms))` |
| `Preferences` (NVS) | `nvs_*` API directly |
| `WiFi.mode(WIFI_OFF)` | Don't init WiFi component at all (excluded via sdkconfig) |

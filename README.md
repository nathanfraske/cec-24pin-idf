# CEC 24-pin Module (ESP-IDF port)

This is the ESP-IDF port of the CEC 24-pin module firmware, replacing the Arduino-ESP32 framework used in the v0.5.x lineage.

The original Arduino-ESP32 firmware at v0.5.9 remains the working backup until this port has been validated against captured-data behavior parity.

## Related projects

- [`cec-eps-idf`](https://github.com/nathanfraske/cec-eps-idf) — companion firmware for the CEC EPS module. Both repos share the same ESP-IDF component layout (`cec_sensors` / `cec_detection` / `cec_capture` / `cec_telemetry` / `cec_cli`), build flow, and TelePlot-over-USB-CDC output format, so any reproducible-firmware tooling that works against one works against the other.

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
    ├── cec_common/             Shared enums (cec_state_t, cec_severity_t)
    ├── cec_filters/            EMA + rolling-median primitives
    ├── cec_nvs/                Thin NVS save/load/clear blob wrapper
    ├── cec_sensors/            Hardware abstractions: INA226, ACS712, NTC, ADC1
    ├── cec_detection/          State classifier, Layer 1/2/3, swing detectors
    ├── cec_capture/            Burst capture engine, pre-trigger ring
    ├── cec_telemetry/          TelePlot output helpers
    └── cec_cli/                Line-based serial command interface
```

Component-based layout is intentional. Each component is hardware-agnostic where possible so it can be reused in the planned ESP32-P4 firmware for the 12VHPWR module, and the small headers-only components (`cec_common`, `cec_filters`) are explicitly shaped to be vendored unchanged into the sibling [`cec-eps-idf`](https://github.com/nathanfraske/cec-eps-idf) repo.

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
| Hello World boot | Done (validated on prototype hardware) |
| I2C master driver (for INA226) | Done |
| ADC oneshot reads — rail voltages (12V/5V/3V3) | Done |
| ADC oneshot reads — NTC temperature | Done |
| ADC oneshot reads — rail currents (i_12V/i_5V/i_3V3 via ACS712) | Done (zero-point cal pending) |
| ADC continuous mode (DMA) | Done |
| Sample loop at 50 Hz | Done |
| EMA / median filter primitives | Done |
| Teleplot output via USB CDC | Done |
| State classifier | Done |
| Layer 1 static thresholds | Done |
| Layer 2 (adaptive transient) | Done |
| Layer 3 (Z-score anomaly) | Done |
| Power-swing detector | Done |
| Current-swing detector | Done |
| Burst capture engine | Done (non-blocking, oneshot ADC; DMA path pending) |
| NVS profile storage | Done |
| Serial command interface | Done (burst / set / status; ACS712 cal deferred — moving to INA226s) |
| Shutdown detection + mute window | Done |

## Behavior parity check

Once each component lands, run side-by-side captures against the v0.5.9 baseline on the same PSU and same workloads, and compare the resulting bursts. Any divergence beyond measurement noise is a porting bug to track down before that component is considered done.

## Serial topology

Two USB-C ports carry firmware output on the 24-pin board, matching the layout used by `cec-eps-idf`:

| Port | Transport | Content | Baud |
|---|---|---|---|
| **JTAG USB-C** | Native ESP32-S3 USB Serial-JTAG | CLI input, `ESP_LOG` output, command responses, boot banner | host-side default (CDC) |
| **UART USB-C** | CH340K bridge to UART0 (GPIO 43 TX / 44 RX) | All TelePlot lines — 10 Hz steady-state telemetry plus `>BURST_BEGIN ... >BURST_END` dumps | 921600 |

Splitting the streams keeps the heavy traffic (~600 KB per burst at full fidelity) off the same wire that's carrying CLI input. `cec_telemetry` owns the UART side; `teleplot_emit*` / `teleplot_writef` calls route through `uart_write_bytes` once `cec_telemetry_init_uart()` has run, with an automatic stdio fallback if the UART driver can't come up (e.g. the secondary cable isn't plugged in during dev).

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

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for the
full text.


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
    ├── cec_sensors/            INA226 (active); cec_adc + thermistor (dormant, NTC)
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
| I2C master driver | Done |
| Detection stack (state classifier, Layers 1/2/3, swings, shutdown mute, saturation watchdog) | Done |
| EMA / median filter primitives | Done |
| Sample loop at 50 Hz | Done |
| Teleplot output (dual-stream UART) | Done |
| Burst capture engine | Done (non-blocking, 1 kHz HS; phase-split gating preserves detection during dump) |
| NVS profile storage | Done |
| Serial command interface | Done (burst / set / status) |

The v0.5.9 → ESP-IDF port is complete; the firmware tracks the prototype board revisions below.

## Hardware revisions

**v2 (current):** all four rails sensed by INA226 over I2C0 — `0x40` +12V (0.002 Ω), `0x41` +5V (0.002 Ω), `0x44` +3.3V (0.025 Ω), `0x45` +5VSB (0.025 Ω). The 5V rail was re-shunted from 0.010 Ω to 0.002 Ω after the test PC's ~20 A 5V draw saturated the 8.19 A range and overheated the smaller shunt; 0.002 Ω gives a 40.96 A range at ~0.8 W. Current is computed in software (shunt µV / R_shunt). The v1 ADC voltage-divider taps and ACS712 current sensors are removed. The 1 kHz HS burst reconfigures the three main-rail INA226s to fast mode (~140 µs conversions) and captures current at 1 kHz with voltage decimated to ~100 Hz. A per-rail saturation watchdog flags any rail whose current pins at the INA226 full-scale ceiling (±81.92 mV / R_shunt) and holds flat — the signature of a railed/faulty sense front end or a sustained over-current, rather than a real reading.

**Pending the shared daughterboard** (`[PENDING]` in the v2 spec): NTC thermistor on ADC1_CH6 (GPIO7) — `cec_adc`/`thermistor` stay in the build, ADC subsystem uninitialized until wired; and CAN/TWAI (TX GPIO4, RX moved GPIO5→GPIO15) — no `cec_comms` component in this repo yet.

Deferred work — EPS-parity items, a code-review cleanup list, and the daughterboard NTC/CAN bring-up — is tracked in [FOLLOWUPS.md](FOLLOWUPS.md).

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


# CLAUDE.md — agent orientation for cec-24pin-idf

Context for an agent picking up this project. Read this first, then
`README.md` (architecture / build / hardware revisions) and `FOLLOWUPS.md`
(deferred work + known-issue lint list). Substantial deferred features
that have already been designed live in `design/` (currently:
`F1_continuous_hs.md` — the continuous 1 kHz INA226 streamer); read
those at implementation time rather than re-deriving the API. The
Arduino-ESP32 v0.5.9 firmware is the behavioral reference for the
port; a v1→v2 hardware change spec was provided during development.

## What this is

Firmware for the **CEC 24-pin sensing module** — an ESP32-S3 (N16R8) board
that monitors a PC PSU's rails (12V / 5V / 3.3V / 5VSB), classifies the
system state, runs a layered fault/anomaly detector, and dumps high-speed
bursts around events for offline analysis (TelePlot). It's an ESP-IDF port
of the older Arduino-ESP32 v0.5.9 firmware, hardware now on prototype **v2**.

Toolchain: **ESP-IDF v6.0.1**, target esp32s3. The agent container has **no
IDF toolchain** — you cannot `idf.py build` here. Verify pure-C components
with host `gcc -std=c99 -Wall -Wextra -fsyntax-only` against small stubbed
headers (see how prior commits did it); real builds/flash happen on the
user's Windows machine, who pastes back logs/errors.

## Component map (`components/`)

| Component | Role |
|---|---|
| `cec_common` | header-only shared types: `cec_state_t`, `cec_severity_t` |
| `cec_filters` | pure-C EMA + rolling-median primitives |
| `cec_nvs` | thin NVS save/load/clear blob wrapper (magic-prefixed) |
| `cec_sensors` | `ina226` (active), `cec_adc` + `thermistor` (dormant, daughterboard NTC) |
| `cec_detection` | `cec_state` classifier, `cec_layer1/2/3`, `cec_swing` |
| `cec_capture` | non-blocking burst engine (pre-trigger ring + HS task on Core 1) |
| `cec_telemetry` | TelePlot output over the UART transport |
| `cec_cli` | line-based serial command interface |

`main/main.c` wires it all: a 50 Hz sample loop → EMAs → state classifier →
detection layers → burst triggers, with TelePlot at 10 Hz and an INFO log at
1 Hz. The detection stack's constants are tuned for 50 Hz — keep that rate.

## Hardware truth (v2 prototype, as physically built)

- **All four rails are INA226** on I2C0 (SDA=GPIO8, SCL=GPIO9, 400 kHz). v1's
  ADC voltage-divider taps and ACS712 current sensors are gone.
- **Address → rail map** (matches v2 spec; see `main.c`):
  - `0x40` → 12V (0.002 Ω)
  - `0x41` → 5V (0.002 Ω, re-shunted from 0.010 Ω — see below)
  - `0x44` → 3.3V (0.025 Ω)
  - `0x45` → 5VSB (0.025 Ω)
- **Every rail's current is sign-inverted** (`INA226_ITRIM_* = -1.0`) because
  all four shunt terminals were wired backwards. The flip happens inside
  `ina226_read_current()`, invisible to everything downstream. If a future
  board wires a rail correctly, set that one rail's `INA226_ITRIM_*` to +1.0.
- Voltage = INA226 bus register (no trim needed; trims start at 1.0).
- **Pending the shared daughterboard (not built):** NTC thermistor (ADC1_CH6/
  GPIO7) and CAN/TWAI (TX GPIO4, RX GPIO15). `cec_adc`/`thermistor` stay in
  the build but the ADC subsystem is never started; there is no `cec_comms`
  component yet. Temperature reads unavailable until then.

## Serial topology (two USB-C ports)

- **JTAG USB-C** (native USB Serial-JTAG): CLI input, `ESP_LOG`, boot banner.
- **UART USB-C** (CH340K → UART0, GPIO43 TX / GPIO44 RX, 921600): every
  TelePlot line — 10 Hz steady telemetry + `>BURST_BEGIN…>BURST_END` dumps.
  Falls back to stdio if the UART can't init.

CLI commands: `burst now [text]`, `set <layer> <on|off>`, `status [json]`,
`help`. Layer enables + Layer-3 profiles persist to NVS.

## Burst capture (the non-obvious part)

Non-blocking: a Core-1 task captures while the Core-0 main loop keeps running.
HS path reconfigures the 3 main-rail INA226s to fast mode (`INA226_CONFIG_HS`
= 0x4007, ~140 µs conversions) for the window, captures **current at 1 kHz**
and **voltage decimated to ~100 Hz** (fits the 400 kHz I2C budget), then
restores steady mode (0x4527).

The engine exposes **two phase flags** so the main loop blocks only what each
phase actually contends for:

- `cec_capture_is_capturing()` — true during the 4 s HS window. Main-loop
  sensor reads and the pre-trigger push gate on this (I²C bus owned by the
  HS task; EMAs are also held since reads are skipped).
- `cec_capture_is_dumping()` — true during the post-capture TelePlot stream
  (~5 s). Steady TelePlot and the pre-trigger push gate on this (UART owned
  by the dump; ring is being read).
- `cec_capture_is_busy()` — true for either phase. NVS saves gate on this
  (flash erase/write stalls both cores' cache).

Reads / EMAs / detectors **do** run during the dumping phase — the bus is
idle then, so detection coverage is preserved through ~half of the burst.
This was a deliberate split (was previously gated as one big "busy" phase
and lost ~9 s of monitoring per burst).

The HS sample callback returns a **per-rail success bitmask**
(`CEC_HS_OK_*`); the engine carries each absent bit forward independently
so one bad device can't void healthy rails. Fast-mode-Plus (1 MHz I²C) for
HS voltage at full 1 kHz is a deferred option gated on a pull-up fix — see
FOLLOWUPS.

## Working conventions

- **Branch:** develop on `claude/ecstatic-bardeen-gO0Yf`; PR into `main`.
  Never push straight to main. (A fresh session may be assigned a different
  branch — honor whatever the environment specifies.)
- **Rhythm:** small, focused PRs, one concern each. Open the PR, subscribe to
  its activity, the user reviews/merges, then rebase onto main and continue.
  Pile onto an open PR only when the user okays it.
- **Verification:** host `gcc -fsyntax-only` for pure-C; on-device build is the
  user's loop. When unsure of a hardware-coupled decision, ask — the v2
  bring-up surfaced several (loop rate, HS-over-I2C, ADC disposition).
- **GitHub:** MCP tools only (`mcp__github__*`), scoped to this repo. Be frugal
  with PR comments.

## Current state

- v1 → ESP-IDF port complete and field-validated; full detection stack done.
- **PR #8 open**: v2 sensor block (4× INA226), IDF-6.0 build fix, boot I²C
  scan, the v2-spec address mapping + current-sign inversion above. Bring-up debugging
  done over chat (addressing/jumpers/wiring); all four INA226 now enumerate.
  Latest commits on the branch address findings from a live-capture
  analysis: per-rail HS bitmask (one bad rail no longer voids the other
  two), 5VSB EMA re-prime on `OFF→STANDBY` (kills the spurious Layer 1
  CRITICAL at PSU-plug-in), and a `capturing`/`dumping` phase split that
  recovers ~5 s of monitoring coverage per burst.
- 0x41 bus reliability (was the open hardware issue): **fixed** by the
  user's pull-up/joint rework — validation run shows 100% reads across
  OFF/STANDBY/IDLE. Not yet stress-tested into ACTIVE/PEAK. The 100 kHz
  `INA226_SCL_HZ` escape hatch was unneeded; left in place.
- Two validation runs done (`teleplot_*` captures on PR #8): per-rail HS
  carry-forward, 5VSB re-prime, phase split, and current-sign inversion
  all confirmed; a real 12V collapse was captured end-to-end by a
  shutdown burst. Latest fix on the branch: shutdown mute no longer
  sticks in STANDBY (arm only from running states + level-based clear).
- **5V/0x41 INA226 — undersized shunt, now resolved.** The 5V rail on the
  test PC pulls ~20 A, but the v2 sensor block put a 0.010 Ω shunt there
  (8.19 A range, and 4 W of I²R at 20 A). That saturated the reading at
  full scale (a "solid 8.19 A," dead flat even in HS, while `v_5v` read
  fine because VBUS is a separate pin) and cooked the shunt/module. Initial
  bench diagnosis chased a bad `V+→IN+` jumper joint and over-called a dead
  die; the actual root cause was the undersized shunt (the original "5V on
  a 25 mΩ module" oopsie was the same fault, worse: 10 W). **Fixed:**
  re-shunted to **0.002 Ω (R002)** + replaced the module → 40.96 A range,
  0.8 W at 20 A (mirrors the 12V rail). Firmware updated
  (`INA226_SHUNT_5V` / `INA226_IMAX_5V` → `0.002f` / `40.96f`). On first
  power-up: confirm the boot I²C scan still shows 0x41 (strap A0→VS,
  A1→GND), and that `i_5v` reads positive under load — if the replacement
  module's terminals are wired correctly rather than backwards like the
  others, flip `INA226_ITRIM_5V` to `+1.0`.
- The episode prompted a **per-rail saturation watchdog** in `main.c`: any
  rail pinned at its IMAX ceiling (`SAT_PIN_FRACTION`·IMAX) for ~1 s logs a
  loud `SATURATION on <rail>` warning (sustained over-current, a sensor
  beyond its range, *or* a railed/faulty chip), re-warns every ~5 s, and
  logs recovery on clear. Fed only fresh reads so a capture-skip or NACK
  freezes the episode rather than skewing it. With the 5V rail now on a
  40 A range it stays quiet under the normal ~20 A load.
- Next, when a PSU run goes into ACTIVE/PEAK: fully close out 0x41 under
  heavy load; otherwise the firmware side is validated.
- Deferred: shutdown-detect *premature-trip* debounce (analyzer C5, distinct
  from the mute-stick fix); daughterboard NTC + CAN; lint items L2/L3/L4/L6;
  1 MHz HS voltage; F1 streamer (designed). All in `FOLLOWUPS.md` / `design/`.
- **F1 designed, not built** — continuous 1 kHz INA226 streamer for
  pre-trigger HS data. Implementation spec in
  [`design/F1_continuous_hs.md`](design/F1_continuous_hs.md); deferred
  until the 0x41 hardware is stable and a real fault capture demonstrates
  the need. Would also fold in FOLLOWUPS L2 (kills the HS task's
  watchdog-margin spin) and remove the burst-engine's setup/teardown
  hooks. F2 (deferred-burst streaming) shares infrastructure; design it
  alongside if both land.

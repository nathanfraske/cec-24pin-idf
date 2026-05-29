# Follow-ups

Tracked deferred work for `cec-24pin-idf`. Nothing here is a blocker — the
firmware runs live on prototype hardware. These are the "we know about it,
not now" items, kept out of the code so they don't get lost in PR
descriptions.

## Code-review cleanup (lint)

Findings from a review pass over the concurrency-heavy paths. Ranked by how
much they'd actually bite. None are active crashes; most are latent or
fragility issues.

| # | Severity | Where | Item |
|---|---|---|---|
| L1 | ✅ fixed (v2 sensor PR) | `main.c` main loop | The raw instantaneous reading now holds at the last-good EMA on a failed/skipped read, so Layer 2 can't see a full-scale deviation from an I²C glitch. Especially relevant now that all four rails are I²C. |
| L2 | medium (fragility) | `cec_capture.c` `spin_until` + `hs_capture_task` | The HS burst busy-spins one core for ~4 s via `taskYIELD`-polling, which starves the Core 1 idle task to ~80% of the default 5 s task-watchdog window — no margin for a larger `HS_BURST_BUF_SIZE` or more per-sample work. With the v2 INA226 HS path each sample is a blocking I²C read (not a cache hit), so `spin_until` could be replaced with a `vTaskDelay`-based pace that yields to the idle task between samples. Still worth doing. |
| L3 | low (smell) | `main.c` periodic save block | `cec_nvs_save_blob` (`nvs_set_blob` + `nvs_commit`) runs synchronously in the 50 Hz supervisor loop. Flash erase/write disables cache on both cores — a multi-hundred-ms loop stall every 5 min when profiles are dirty. Move to a low-priority one-shot task. |
| L4 | low (benign race) | `cec_capture.c` `cec_capture_trigger_with_text` | Check-then-set on `s_busy` isn't atomic; the main loop and CLI call the trigger from different tasks. Binary semaphore swallows the double-give, so worst case is a misleading "burst triggered" log with no second capture. A short critical section or atomic CAS closes it. |
| L5 | ✅ fixed (capture-phase PR) | `cec_capture.c` `cec_capture_push` vs `dump_pretrigger` | Push now gates on the new `!capturing && !dumping` check, so the dump task has exclusive read of the ring. No more torn rows. |
| L6 | nit | `main.c` shutdown detection | `v_12v_rate` is a raw ΔV over the 50-sample history but is compared against a `-0.5f` "V/s" threshold — correct only because 50 samples × 20 ms = exactly 1 s. Changing `V_12V_RATE_HISTORY_SIZE` or `SAMPLE_PERIOD_MS` silently breaks the units. Either divide by the real window seconds or add a static-assert tying the two constants together. |
| L7 | ✅ fixed (v2 sensor PR) | `cec_capture.c` HS loop | The stale SAR-ADC zero-artifact heuristic was replaced with an explicit carry-forward keyed on the HS sample callback returning failure. |

### Live-capture analysis findings (`teleplot_2026528_2325.csv`)

| # | Severity | Where | Item |
|---|---|---|---|
| C1 | ✅ fixed (capture-phase PR) | `cec_capture.c` HS callback API | One bad rail's NACK was voiding the two healthy rails' samples (`bool ok` AND-collapsed across all rails). The callback now returns a per-rail bitmask (`CEC_HS_OK_*`) and the engine carries forward each absent bit independently. |
| C2 | ✅ fixed (capture-phase PR) | `main.c` state transitions | 5VSB EMA was starting at 0 V when the rail powered up on `OFF→STANDBY` and crawling up at α=0.02 (~1 s τ), tripping Layer 1 CRITICAL halfway through (mask was only 500 ms). EMA now re-primes on any `OFF→{non-OFF}` transition. |
| C3 | ✅ fixed (capture-phase PR) | `main.c` main loop gating | Burst dumps caused ~9.4 s monitoring blackouts; the gate has been split into `capturing` vs `dumping` phases. Reads / EMAs / detectors now run during the dump (I²C bus is idle); only steady TelePlot and NVS save stay blocked for the full burst. Recovers ~5 s of coverage per burst. |
| C4 | hardware (open) | `cec_sensors/ina226.c` bus reliability | The 0x41 INA226 NACKs 0–87% of reads depending on load (vs. 0% on the other three). Likely the 4-parallel-pull-up issue from v2 spec §6 — affects only the rail wired to it. **Hardware fix in progress** (user is reworking pull-ups / joints); firmware C1 mitigates the symptom but does not resolve the cause. Falling back to 100 kHz (`#define INA226_SCL_HZ 100000` in `main.c`) is the firmware escape hatch if the hardware fix doesn't stick. |
| C5 | tuning (deferred) | `main.c` shutdown detection | Premature shutdown trip at peak load (~19 s early): a single-second window with rate < −0.5 V/s isn't enough to discriminate load-step droop from real collapse. Proposed fix when actioned: require both rate AND `v_12v_ema < 11.5 V` floor. User wants to wait until other items settle. |

## EPS-parity deferrals

From the cross-repo TODO list; carried over as the EPS side evolves.

- **B1** — Add `cec_load_state_t` and `CEC_LOAD_COUNT` to `cec_common/include/cec_state.h` (TODO marker already in place), mirroring the EPS-side `cec_classifier.c` enum. Land the actual values when next syncing EPS source so the two repos can include each other's headers without name clashes.
- **D1** — ~~ACS712 driver style~~ moot: ACS712 removed in the v2 sensor swap; all rails are INA226 now.
- **A4 / G** — `cec_comms` component for CAN/TWAI when CAN ships on the 24-pin. Use the `esp_twai_onchip` node-handle API directly (skip the deprecated `driver/twai.h`). EPS-side `cec_comms/cec_can.c` is the reference; adapt frame layout / IDs for the 24-pin payload.

## Daughterboard bring-up (pending hardware)

The shared daughterboard isn't built for this module yet. When the pigtail lands:

- **NTC thermistor** (ADC1_CH6 / GPIO7): re-enable the ADC path —
  `cec_adc_init` + `cec_adc_setup_channel(ADC_CHANNEL_6)` + `cec_adc_start`,
  and restore the `thermistor_read_celsius` call + `ok_temp` in the main loop.
  `thermistor.c` already targets GPIO7; `cec_adc`/`thermistor` are still in the
  `cec_sensors` build, just unused. β = 3950, 10 kΩ NTC, 10 kΩ series to 3.3 V.
- **CAN / TWAI** (TX GPIO4, RX GPIO15 — moved from GPIO5 in v2): stand up the
  `cec_comms` component (see A4/G). RX is on ADC2/GPIO15 deliberately so all of
  ADC1 stays free for sensors.
- **INA226 bus at 1 MHz (Fast-mode-Plus):** optional, unlocks HS *voltage* at
  the full 1 kHz alongside current. Gated on the §6 pull-up fix (lift 3 of the
  4 module pull-ups) holding up at speed — check for garbage on the inputs
  before trusting it. Today HS voltage is decimated to ~100 Hz at 400 kHz.

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
| C4 | ✅ fixed (hardware) | `cec_sensors/ina226.c` bus reliability | The 0x41 INA226 NACK'd 0–87% of reads under load (the 4-parallel-pull-up issue from v2 spec §6). User reworked the pull-ups/joints; validation run `teleplot_2026529_133.csv` shows **100% reads (1188/1188)** across OFF/STANDBY/IDLE. Firmware C1 (per-rail carry-forward) remains as defence-in-depth. The 100 kHz escape hatch (`INA226_SCL_HZ`) was not needed; left in place. Not yet re-exercised into ACTIVE/PEAK — a heavy run would fully close it out. |
| C5 | tuning (deferred) | `main.c` shutdown detection | Premature shutdown trip at peak load (~19 s early): a single-second window with rate < −0.5 V/s isn't enough to discriminate load-step droop from real collapse. Proposed fix when actioned: require both rate AND `v_12v_ema < 11.5 V` floor. User wants to wait until other items settle. |
| C6 | ✅ fixed (shutdown-arm PR) | `main.c` shutdown detection | Shutdown mute stuck ~24 s in STANDBY (validation run). The mute-clear fired only on the transition *edge* into STANDBY/OFF, but 12V decaying through the 5–10.5 V band re-asserted `s_shutting_down` while already in STANDBY, with no new edge to clear it. Fixed two ways: shutdown is now armed only from a running state (IDLE/ACTIVE/PEAK), killing the re-assert at the source; and the mute-clear is now a level check (`s_state ∈ {STANDBY,OFF}`) rather than an edge, so it can't get stuck. Distinct from C5 (premature *trip*); this was premature *un-clear*. |

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

## Future architectural ideas (not committed — write-ups for later decisions)

These are real options, sized and reasoned-through, that aren't being built
now because their cost outweighs their current value. Each entry captures
the shape, the math, the downsides, and the trigger that would justify
elevating it to actual work — so a future agent (or you) doesn't have to
re-derive the analysis when the trigger fires.

### F1 — Continuous 1 kHz rolling buffer (pre-trigger HS data)

> **Design spec:** [`design/F1_continuous_hs.md`](design/F1_continuous_hs.md)
> — full API surface, data shapes, task model, migration order. Read that
> when implementation lands; the bullets below are the elevator pitch.

**What we don't have today.** The HS buffer starts *at the trigger moment*.
Pre-trigger context is 50 Hz only (20 s of EMA-filtered samples). So a burst
shows us 4 s of 1 kHz aftermath but zero kHz history — for a fault whose
*cause* is a sub-50 Hz transient (microsecond load steps, switching
artifacts), the trigger lag means we capture the consequence and miss the
trigger event itself.

**Shape.** Replace the per-burst HS task with a *continuous* sampler task
that keeps the 3 main-rail INA226s permanently in fast mode (AVG=1, 140 µs
conversions, `INA226_CONFIG_HS`) and writes a **rolling ring of the last
~2–4 s of 1 kHz current samples** (and decimated voltage). The main loop
stops reading I²C itself — it reads the latest slot from the rolling buffer
each iteration. On a trigger:

1. Snapshot the rolling ring into a per-burst buffer (pre-trigger HS).
2. Snapshot the existing 50 Hz pre-trigger ring (unchanged — wider context).
3. Capture an additional 4 s of 1 kHz samples post-trigger (same as today).
4. Dump all three over TelePlot.

**Memory.** Rolling ring at 2 s = 56 KB, at 4 s = 112 KB. Existing buffers
total ~160 KB. With 8 MB PSRAM the absolute number is irrelevant; if the
ring lives in SRAM the total stays well under the N16R8's budget. Honest
cost: small.

**The AVG=1 noise tradeoff.** Today the INA226s run at AVG=16
(`INA226_CONFIG_STEADY`), which averages 16 raw samples per reported value
and reduces per-sample noise by √16 = 4×. Moving to permanent AVG=1 means
per-sample current noise rises 4×: ~1.25 mA RMS on the 12 V rail (0.002 Ω
shunt), ~100 µA RMS on the 3V3/5VSB rails. *Downstream* this is absorbed by
the EMA at α=0.02, which with 1 kHz feed averages ~1000 samples for its
~1 s τ — so the 50 Hz consumer sees *more* smoothing than today, not less.
HS analysis explicitly wants the per-sample variance. Probably a net win.

**Side benefits.** Several real ones, not just the pre-trigger data:
- **Fixes FOLLOWUPS L2** for free. The HS task's `taskYIELD`-spin
  (~4 s of starving Core 1's idle task, ~80 % of the watchdog window) goes
  away — a continuous sampler can `vTaskDelayUntil(pdMS_TO_TICKS(1))`
  because it's not racing a 4 s window, just keeping a ring topped up.
- **Removes the mode-switch settle gap.** Today the HS task does
  `setup_fn → settle 2 ms → 4 s capture → teardown_fn`. With permanent fast
  mode the setup/teardown hooks (and the 2 ms settle) disappear — the
  burst-engine API simplifies, the `cec_capture_config_t.setup_fn`/
  `teardown_fn` fields can be removed.
- **No more "did the main loop and the HS task collide on I²C?"** Only the
  sampler task touches the bus; everyone else reads the latest slot. The
  `cec_capture_is_capturing()` gate on main-loop reads (and the whole
  "phase split" we did in this PR) becomes unnecessary — `is_dumping` is
  the only flag that matters (UART contention).
- **Burst capture during a dump becomes possible.** Today
  `cec_capture_trigger` rejects re-triggers while `s_phase != CAP_IDLE`,
  because the single HS buffer is in use. With per-burst buffers (which
  this design needs anyway), a second trigger could allocate another
  buffer mid-dump. Pairs naturally with F2 below.

**Downsides.** Permanent ~45% I²C bus utilization (3 reads × 1 kHz at 400
kHz vs. today's bursty pattern). Continuous fast-mode is *slightly* more
power. The AVG=1 noise needs to be re-validated against the detection-stack
tuning (probably fine — the analysis findings to date show the layers
handle real noise well). Bigger code change than fits in any of the
current PRs.

**Trigger to build it.** First real fault capture where the missing
pre-trigger HS data prevents diagnosis (you see the aftermath at 1 kHz but
can't tell *what caused* the L1/L2 fire). Until that's a concrete blocker
the existing 50 Hz pre-trigger is doing the job for the slow-trend cases
that have come up so far.

### F2 — Deferred burst streaming with PSRAM queue and UART arbiter

**Motivation.** During a fault storm (back-to-back bursts), each burst's
~5 s dump silences live TelePlot output. The dump itself is fine — it's
streaming useful data — but the steady traces pause for the storm's
duration. Detection survives (post-phase-split), so this is a
telemetry-continuity concern, not a correctness one. For offline-CSV
workflows it's a non-issue (TelePlot reorders by embedded device timestamp
on import); for live bench debug it's mildly annoying.

**Shape.** A PSRAM-backed ring of captured-but-not-yet-dumped bursts plus
a low-priority streamer task that drains the queue *opportunistically*,
yielding to live telemetry. Steady output and queue-drain share the UART
via a mutex in `cec_telemetry` (already cleanly factorable — `teleplot_writef`
formats a whole line into a local buffer then does one `uart_write_bytes`;
wrap that one call in a mutex for atomic lines).

PSRAM math: a burst is ~160 KB (112 KB HS + a 48 KB pre-trigger snapshot —
note the snapshot, F1 makes this mandatory anyway). A queue of 8 bursts is
1.3 MB; 16 is 2.6 MB. The N16R8's 8 MB PSRAM is fine for either.

**The Achilles heel — what if the system never stabilizes.** A dying PSU
storms indefinitely, which is exactly when you most want the data and least
get calm bandwidth. So "wait for stable, then stream" can't be the whole
policy. The honest version is **queue-and-drain-opportunistically with a
depth cap that forces a flush** — the streamer just runs whenever there's
spare bandwidth and falls behind during storms / catches up during calm,
and when the queue hits the cap it drains regardless. That softer framing
*doesn't need a stability oracle at all* — sidesteps the "firmware has to
detect stable" problem cleanly. A real stable/unstable gate is a refinement
that can land later, not the foundation.

**Trigger to build it.** You run real captures, hit a genuine fault storm,
and find the repeated live-telemetry gaps actually impede analysis (vs.
"slightly annoying"). Until then, the offline-CSV workflow makes this
mostly invisible to you specifically.

**Note: F1 and F2 share infrastructure.** Both need per-burst buffers
(snapshot semantics), and the UART mutex is needed by F2 and would also
unblock interleaving steady-telemetry rows mid-dump if that ever became
desired. If both eventually land, build the per-burst-buffer + UART-mutex
foundation once and let F1 and F2 sit on it.


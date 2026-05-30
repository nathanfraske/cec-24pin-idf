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
| C7 | ✅ resolved (saturation PR + re-shunt) | `main.c` rail config + main loop | **5V/0x41 shunt undersized for the rail.** The test PC pulls ~20 A on 5V, but the v2 sensor block put a 0.010 Ω shunt there: 8.19 A measurement range (so 20 A pinned the reading at full scale, dead flat even in HS, while `v_5v` read fine via the separate VBUS pin) and ~4 W of I²R that cooked the shunt/module. Bench diagnosis first chased a bad `V+→IN+` jumper joint and over-called a dead die; root cause was the undersized shunt (the earlier "5V on a 25 mΩ module" was the same fault at 10 W). **Fixed:** re-shunted to 0.002 Ω (R002) + new module → 40.96 A range, 0.8 W at 20 A; firmware `INA226_SHUNT_5V`/`INA226_IMAX_5V` → `0.002f`/`40.96f`. The episode also prompted a per-rail **saturation watchdog**: a rail pinned at ≥`SAT_PIN_FRACTION`·IMAX for ~1 s logs `SATURATION on <rail>` (over-current, beyond-range sensor, *or* railed/faulty chip), re-warns ~5 s, logs recovery on clear, fed only fresh reads (freezes on capture-skip/NACK). On the 40 A range the 5V rail now stays quiet under normal load. **Factory-spec feedback:** size the 5V production shunt for the real max 5V draw (range *and* dissipation), not the v2 default. **Optional follow-up:** latch a persistent "rail N sense fault — replace module" flag into `status` so it survives the scrolling log. |
| C8 | deferred (deep-dive) | `cec_layer3.c` z-score | **Layer 3 zero-variance z-score blow-up.** The 2026-05-30 run (`teleplot_2026530_146.csv`) hit `z_max`=645 with no real event: the 5V Vbus sense was not jumpered yet, so `v_5v` read dead-flat ~0.064 V and that quantity's σ estimate collapsed, scoring tiny deviations as hundreds of σ → spurious `CEC_TRIG_ANOMALY` bursts. The existing `MIN_STD_FOR_Z` guard doesn't prevent it (hard cutoff leaves a danger band just above the floor; the constant is sub-LSB and shared across V/I). Self-resolves once the sense is jumpered; latent for any flatlined/stuck sensor. Full writeup + candidate fixes: see **Detection-layer hardening** below. |
| C9 | deferred (deep-dive) | `main.c` slow-loop reads + `cec_layer3.c` | **Single-sample read glitch fires fake "all-rail dip" anomaly.** From `teleplot_2026530_1537.csv` (firmware `102d33f`): ~2 slow samples where 12V/5V/3V3 *all* read near-zero on both V and I simultaneously while 5VSB (0x45) stayed nominal; system ran 209 s more at full nominal voltages immediately after. Provably a sensor artifact (a real PSU collapse can't have V *and* I both go to zero, and the system survived). The bad reads bypassed any plausibility check, fed straight into the EMAs, and drove Layer 3 z-score 5.9→120 in one sample → spurious `CEC_TRIG_ANOMALY` (burst #29). Affects only the three rails that get mode-switched for HS bursts; lands ~9.9 s post-prior-teardown (near the 10 s cooldown boundary), hinting at a steady↔HS re-arm disturbance. 1 of 36 burst-adjacent windows. Pairs with C8 under **Detection-layer hardening** below. |

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

## Detection-layer hardening (deep-dive queued)

False-positive sources in the detection stack, parked for a focused pass.
None is a missed-fault risk — they manufacture spurious anomaly bursts and
misleading transients, not blind spots.

### Layer 3 — zero-variance z-score blow-up

**Symptom.** Run `teleplot_2026530_146.csv` (firmware `102d33f`) hit
`z_max` = 645 with no real event. The 5V Vbus sense was not jumpered yet
(interim hardware), so `v_5v` read a dead-flat ~0.064 V for the whole run.
Layer 3's profile for that quantity collapsed to ~zero spread, and small
deviations then scored as hundreds of σ, firing spurious
`CEC_TRIG_ANOMALY` bursts.

**Mechanism** (`cec_detection/cec_layer3.c`). `std_dev` is an EMA of
`|x − mean|` (a mean-absolute-deviation proxy for σ), adapted at
`PROFILE_ADAPT_RATE` (0.0005) once warm. On a flat input, once `mean`
converges, `|delta|` → ~0, so `std_dev` decays toward zero; `z_score()`
returns `(x − mean) / std_dev`, which blows up as the denominator shrinks.

**Why the existing guard misses it.** `MIN_STD_FOR_Z = 0.001f` only zeroes
the z-score when `std_dev` is *below* the floor. But on a flat rail the std
doesn't sit at zero — it's seeded at `INITIAL_STD_GUESS` (0.01) and decays
*through* the band (0.001 … 0.01) as `|delta|` → 0, and it re-enters that
band every time a per-state profile (OFF/STANDBY/IDLE/…) re-warms on the
flat rail. Throughout that band the guard passes but the denominator is
tiny, so any deviation (a transient read, or the rail not being perfectly
constant) scores in the hundreds. The hard cutoff creates a *danger band
just above the floor* instead of bounding the ratio. Compounding it: the
floor is a single absolute constant — 0.001 is below one bus LSB
(1.25 mV), and it's shared across voltage and current quantities whose
scales differ (the current LSB now depends on the per-rail shunt: 1.25 mA
on the 2 mΩ 5V shunt).

**Impact.** Spurious bursts → wasted captures, UART dump traffic, brief
capture-phase blackouts. Bounded by the 10 s burst cooldown (noise, not a
tight loop). Self-resolves once the 5V sense is jumpered (real rail → real
noise → healthy std), so this is *latent*, not a PR blocker — but any
genuinely flatlined/stuck sensor, or a legitimately ultra-stable quantity,
reproduces it.

**Candidate fixes (decide in the dive).**
- **A. Clamp, don't cut.** `z = (x − mean) / max(std_dev, floor)` bounds
  amplification at `Δ/floor` instead of opening a danger band.
- **B. Per-quantity (ideally per-rail) floors.** Separate V and I floors,
  the current floor scaled to the rail's shunt LSB / expected noise —
  passed into the profile or stored on it.
- **C. Gate L3 on rail-present.** Skip z-scoring a voltage rail reading
  below the rail-off floor (0.1 V); kills both the unjumpered-sense case
  and genuine collapse (Layer 1 owns those). Doesn't cover a sensor stuck
  at a *nominal* constant.
- **D. Minimum-variance warm gate.** Don't mark a profile usable until it
  has seen some spread. Risk: blinds L3 to slow excursions on quiet rails.

**Lean:** **C + A together** — C removes the off/unsensed-rail case
cleanly; A bounds the worst case for any other flat quantity; pick floors
per-quantity.

### Layer 3 — benign load-step anomalies (related)

From the `teleplot_2026529_1327.csv` analysis: a normal IDLE→ACTIVE load
ramp scored `z_max` = 8.09 (i_5v 2.7 → 3.6 A) and would fire
`CEC_TRIG_ANOMALY` on any significant load step (swallowed there only by a
burst cooldown). Different mechanism from the blow-up above (real step,
healthy variance) but the same layer — worth deciding together whether
routine load ramps should count as anomalies (profile warm-up, threshold,
or an explicit rate/step guard).

### Single-sample read glitch → fake "all-rail dip" anomaly (C9)

**Symptom.** Run `teleplot_2026530_1537.csv` (firmware `102d33f`):
**12V/5V/3V3** simultaneously dropped to **7.90 / 1.77 / 1.25 V** for ~2
slow samples (~100 ms) at t ≈ 4186.7 s, while **5VSB (0x45) stayed flat at
5.05 V**, then snapped back to 12.11/5.01/3.21 V. System ran another
209 s at nominal. Frequency: 1 of 36 burst-adjacent windows in the run.

**Why it's provably a sensor artifact, not a PSU event.**
- **5VSB unaffected** — the only rail whose INA226 is never mode-switched.
  The dip is isolated to *exactly* the three reconfigurable devices.
- **Both V and I collapsed together** (12V: 1.11 A → 0.06 A). A real sag
  is *caused by* a current change; both going to ~0 means the sensor
  returned ~0 on both registers.
- The PC kept running 209 s. At 5V = 1.77 V it'd have died instantly.

**Mechanism (open).** The two bad reads were normal 10 Hz slow-loop rows
(`cec_capture_is_capturing()` was false), reached the EMAs unfiltered,
and drove the Layer 3 z-score 5.9 → 23.8 → 120 in one sample, crossing
the 4.0 threshold → `CEC_TRIG_ANOMALY` (burst #29). The glitch landed
~9.9 s after the prior burst teardown, near the 10 s cooldown boundary —
suggests a steady↔HS mode-switch / re-arm disturbance on the three
reconfigured INA226s, but I can't prove the internal mechanism from
telemetry alone. Needs an on-bench repro with shunt µV / config-register
inspection across teardown.

**Two-layer impact.** This finding has two distinct fixes that should be
considered separately:

1. **Sensor-layer plausibility gate** (`main.c` main loop). A single
   sample where a rail is classified up but its bus voltage reads
   implausibly low, *or* where V and I both collapse to ~0 together,
   should be discarded (hold last-good EMA) the way the HS per-rail
   carry-forward already discards NACKs. Stops one bad read from reaching
   any downstream layer.

2. **Layer-3 spike immunity** (`cec_layer3.c`). The fact that **one
   sample** can drive z from 5.9 → 120 is independently fragile. Same
   "single sample shouldn't be able to fire a burst" theme as C8 — pairs
   naturally with the C8 deep-dive. Options: a median/Hampel pre-filter
   ahead of the z-score, a per-sample delta clamp, or requiring N
   consecutive over-threshold samples (matches the rest of the stack's
   consecutive-N pattern). The consecutive-N route also incidentally
   closes the benign load-step issue above.

3. **Root-cause the mode-switch glitch.** Whatever's producing the
   near-zero pair on the three reconfigurable INA226s ~9.9 s after a
   burst teardown deserves a focused look at the steady↔HS sequence in
   `cec_capture.c` (timing, settle window, config-register write order,
   whether the cooldown re-arm path can race the main-loop reads).
   Plausibility gate + spike immunity treat the symptom; this is the
   actual cure.

**Lean:** plausibility gate is the cheapest, most defensive win
(self-contained, no detector-tuning risk) and is worth a dedicated PR
even before the deep-dive. Spike-immunity belongs in the C8 dive. The
mode-switch investigation is its own piece of work.

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


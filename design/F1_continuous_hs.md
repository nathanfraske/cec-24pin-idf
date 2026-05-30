# F1 — Continuous 1 kHz INA226 streamer (design)

Status: **design only — not implemented.** Motivation, sizing, tradeoffs,
and the trigger-to-build live in [`FOLLOWUPS.md`](../FOLLOWUPS.md) under
F1. This file is the implementation spec: API surface, data shapes,
ownership boundaries, and the concrete migration from today's
trigger-time-only HS capture to a continuous-sampled rolling buffer.
Deferred pending 0x41 hardware stabilization; revisit when the first
real fault capture proves missing pre-trigger HS data prevents
diagnosis.

## Architecture in three sentences

A new sub-module in `cec_sensors` (`ina226_streamer.{c,h}`) runs the three
main-rail INA226s in permanent fast mode and a dedicated task pulls 1 kHz
samples into a rolling ring, decimating bus voltage to ~100 Hz to fit the
I²C budget. Every other consumer — the main loop's 50 Hz reads and the
burst engine's HS capture — pulls from that ring instead of touching the
bus, which kills the cross-consumer contention that motivated FOLLOWUPS
L2 and the `capturing` phase gate in the same stroke. The burst engine
keeps its existing pre-trigger ring + dump path but no longer needs
setup/teardown hooks, a per-sample callback, or the `CAP_CAPTURING`
phase — it snapshots the streamer at trigger time, snapshots again after
the post-trigger window, then dumps.

## Component placement

New files in `cec_sensors`:

```
components/cec_sensors/
  ina226_streamer.c
  include/ina226_streamer.h
```

Justification for placing it inside `cec_sensors` (vs. a standalone
component): it's the *active reader* of the existing `ina226` driver —
one layer up from the raw register access, one layer below any
algorithmic consumer. Keeps the dependency graph clean (cec_capture and
main both depend on cec_sensors already; nothing new). If it grows past
this scope (e.g. ADC streamer for the NTC, or shared scheduling with
other I²C devices) split into its own component then.

## Data shapes

```c
/* One sample of the rolling buffer. ~28 bytes — matches today's
 * cec_capture_hs_sample_t so dump-format code can be kept verbatim. */
typedef struct {
    uint32_t ts_us;     /* (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF) */
    float    i_12v, i_5v, i_3v3;       /* fresh every sample (1 kHz) */
    float    v_12v, v_5v, v_3v3;       /* fresh on decimated samples (~100 Hz);
                                          carries the most recent value otherwise */
    uint32_t ok_flags;                 /* CEC_HS_OK_* — fields actually fresh
                                          this sample (i.e. successfully read,
                                          plus the V_* bits only when the slot
                                          was a decimated-voltage slot) */
} ina226_hs_sample_t;
```

Streamer ring is `RING_DEPTH_SAMPLES` of these, plus a `volatile uint32_t
write_idx` (next slot to fill, wraps modulo depth) and a saturating
`volatile uint32_t sample_count` (so consumers can detect "ring not full
yet"). Both indexes are 32-bit aligned so single-word atomic reads/writes
on Xtensa cover the latest-sample fast path without a lock.

`RING_DEPTH_SAMPLES` sized for `pre_trigger_seconds + slack`. With pre =
4 s and 1 s of slack against snapshot-during-fill races: **5000 samples,
~140 KB** (`5000 × 28`). Sits in PSRAM (`EXT_RAM_BSS_ATTR` if static, or
`heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` at init).

## Public API

```c
/* ina226_streamer.h */

#include "ina226.h"
#include "cec_capture.h"  /* for CEC_HS_OK_* bits */

/* One-shot init. Takes the three already-created INA226 handles; rewrites
 * each one's config to INA226_CONFIG_HS (permanent fast mode, AVG=1,
 * 140 us conversions) and spawns the sampler task. Idempotent. */
esp_err_t ina226_streamer_init(ina226_handle_t i12v,
                               ina226_handle_t i5v,
                               ina226_handle_t i3v3);

/* Read the most recent sample. Lock-free single-slot read; safe from any
 * task, any core. Returns ESP_ERR_INVALID_STATE if the streamer hasn't
 * produced its first sample yet (boot-time race). */
esp_err_t ina226_streamer_get_latest(ina226_hs_sample_t *out);

/* Snapshot the last N samples into a caller-provided buffer in arrival
 * order (oldest first, most-recent last). Returns the number of samples
 * actually written: min(sample_count, n_samples). Takes the streamer
 * mutex briefly (~1 ms for a 4 s snapshot), so the sampler skips one
 * sample worst-case. Used by the burst engine at trigger time. */
size_t ina226_streamer_snapshot(ina226_hs_sample_t *dst, size_t n_samples);

/* Health + diagnostics, surfaced via the `status` CLI command and an
 * optional 1 Hz log line. */
typedef struct {
    uint64_t total_samples;     /* monotonic; useful for rate sanity */
    uint64_t read_failures;     /* slots with any expected CEC_HS_OK_* bit missing */
    uint32_t late_samples;      /* slots whose ts_us drifted > 1.5 * INTERVAL */
    uint32_t max_lateness_us;   /* worst observed jitter */
} ina226_streamer_stats_t;
void ina226_streamer_get_stats(ina226_streamer_stats_t *out);
```

That's the whole external surface. Three functions plus a stats getter.

## Internal sampler task

Pinned to Core 1 (mirrors today's HS task placement, leaves Core 0 for
the main loop + CLI). Priority `configMAX_PRIORITIES - 2` (same as the
current HS task — high enough to keep timing, not high enough to starve
the IDF event loops).

```
loop:
  vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1))    // 1 kHz tick
  slot = &ring[write_idx]
  ts_us = esp_timer_get_time()
  ok = 0
  if (read_current(i_12v, &slot->i_12v) == OK) ok |= CEC_HS_OK_I_12V
  if (read_current(i_5v,  &slot->i_5v)  == OK) ok |= CEC_HS_OK_I_5V
  if (read_current(i_3v3, &slot->i_3v3) == OK) ok |= CEC_HS_OK_I_3V3
  if (write_idx % HS_VOLTAGE_DECIMATE == 0):
    /* read v_*, set ok |= CEC_HS_OK_V_* per success */
  else:
    /* carry forward v_* from prev slot; ok bits for V_* stay clear */
  slot->ts_us = ts_us
  slot->ok_flags = ok
  __atomic_thread_fence(release)              // make slot writes visible
  write_idx = (write_idx + 1) % RING_DEPTH    // single-word, atomic
  if (sample_count < UINT32_MAX) sample_count++
  /* update late-sample stats; if (esp_timer_get_time() - ts_us) > 1500 us
     count as late, update max_lateness_us */
```

Key call-outs:

- **`vTaskDelayUntil` (not `spin_until`).** This is the FOLLOWUPS L2 fix
  folded in — the sampler isn't racing a finite window, just keeping a
  ring topped up, so a yield-and-wait pace is correct. The idle task
  on Core 1 actually runs; the task watchdog has zero pressure.
- **`ts_us` is monotonic device time** (truncated to uint32 to keep the
  sample slim). Wraps every ~71 minutes; consumers reconstruct full
  timestamps using `esp_timer_get_time()` at dump time as the anchor.
- **Decimation tracked off `write_idx`**, not a separate counter — keeps
  state minimal and the every-Nth pattern deterministic across snapshots.
- **Release fence before publishing `write_idx`** ensures a concurrent
  `get_latest` reader sees a consistent slot. Acquire fence in the reader
  before reading the slot.

## Burst trigger flow (new)

```
main / CLI calls cec_capture_trigger(reason):
  - Same as today: cooldown check, set s_phase = CAP_CAPTURING,
    give semaphore

capture task wakes:
  1. snapshot_pre = ina226_streamer_snapshot(pre_buf, PRE_TRIGGER_HS_SAMPLES)
       /* copies up to 4 s of pre-trigger HS data, atomic w.r.t. sampler */
  2. vTaskDelay(POST_TRIGGER_SECONDS * 1000)
       /* sampler keeps filling the ring; we wait for the post window */
  3. snapshot_post = ina226_streamer_snapshot(post_buf, POST_TRIGGER_HS_SAMPLES)
       /* by now the ring's full of the post-trigger window */
  4. s_phase = CAP_DUMPING
  5. emit BURST_BEGIN + annotation
  6. dump_pretrigger()    // slow 50 Hz ring — unchanged
  7. dump_hs(pre_buf + post_buf)    // pre and post interleaved by ts_us
  8. emit BURST_END
  9. s_phase = CAP_IDLE
```

Two per-burst buffers, each `POST_TRIGGER_HS_SAMPLES × sizeof(sample)`,
in PSRAM. With 4 s pre + 4 s post that's 2 × 112 KB = 224 KB. Both can
be static `EXT_RAM_BSS_ATTR` (one-burst-at-a-time semantics unchanged
from today). F2 later changes this to a pool — that's why the spec is
"two buffers" not "one combined buffer," so the F2 migration is just
"allocate from a pool" rather than re-architecting.

`CAP_CAPTURING` phase semantically becomes "waiting for post-trigger
window to fill" rather than "actively sampling," and lasts ≈ the
post-trigger duration (4 s). It's no longer a bus-contention phase since
the sampler owns the bus continuously.

## Main loop integration (in `main.c`)

The main loop's per-iteration sensor block changes shape:

```c
/* Today: 4 direct ina226_read_* calls per iteration */

/* With F1: */
ina226_hs_sample_t hs;
bool ok_main = (ina226_streamer_get_latest(&hs) == ESP_OK);
if (ok_main) {
    /* Map streamer fields onto the existing ema/detector inputs */
    v_12v = hs.v_12v; i_12v = hs.i_12v;
    v_5v  = hs.v_5v;  i_5v  = hs.i_5v;
    v_3v3 = hs.v_3v3; i_3v3 = hs.i_3v3;
    /* Per-rail ok comes from hs.ok_flags for the I_* bits;
     * V_* bits are only fresh on decimated slots, but the streamer
     * already carries forward the last good v_*, so main treats them
     * as always-ok. */
    ok_i_12v = (hs.ok_flags & CEC_HS_OK_I_12V) != 0;
    ok_v_12v = ok_main;   /* always honoured-via-carry-forward */
    /* ... etc ... */
}

/* 5VSB stays on its own slow-loop ina226_read_* path — it's not part of
 * the streamer (no HS capture interest in 5VSB; the streamer is the 3
 * main rails only). One ina226_read_bus_voltage + one
 * ina226_read_current per 20 ms iteration. */
```

The `cec_capture_is_capturing()` gate on main-loop reads **goes away** —
no bus contention to gate. `cec_capture_is_dumping()` gate on steady
TelePlot stays (UART contention). NVS save still gates on `_is_busy()`.

The on-failed-read-hold-at-last-good-EMA logic stays — it now triggers
on `hs.ok_flags & CEC_HS_OK_I_*` rather than per-call return code, but
the downstream behavior is unchanged.

## What goes away (deletions, not just additions)

- `cec_capture_hs_sample_fn_t` callback type, and the `sample_fn` /
  `setup_fn` / `teardown_fn` fields of `cec_capture_config_t`. The
  capture engine reads from the streamer directly; no callback needed.
- The HS task's `setup_fn() → settle → spin loop → teardown_fn()`
  sequence in `hs_capture_task`. Replaced by snapshot + wait + snapshot.
- `INA226_CONFIG_STEADY` use at startup for the 3 main rails. They open
  in `INA226_CONFIG_HS` and stay there. (5VSB stays in steady mode —
  no benefit to fast mode there.)
- `cec_capture_is_capturing()` gate on main-loop sensor reads (3 sites in
  `main.c`). The accessor itself stays public for diagnostics but no
  consumer treats it as a hot-path gate.
- `hs_sample_fn` in `main.c` — the callback that bridged the engine to
  ina226_read_*. Gone entirely.

Each deletion is a real LOC reduction and a real reduction in concurrency
edges; the streamer concentrates all bus-contention concerns into one
task.

## TelePlot output

No format change. `>hs_i_12v:<ts>:<v>` rows still emit at 1 kHz cadence,
`>hs_v_12v:<ts>:<v>` at ~100 Hz. The only difference is the *time range*
covered:

| Field | Today | With F1 |
|---|---|---|
| `>hs_*` time span | trigger .. trigger+4s | trigger-4s .. trigger+4s |
| Pre-trigger ramp visibility | 50 Hz only | 50 Hz **plus** 1 kHz |
| Total HS samples per burst | 4000 | 8000 |
| Total dump payload | ~600 KB | ~1.2 MB |
| Dump duration @ 921600 baud | ~5 s | ~10 s |

Dump duration doubles. That's the real visible cost to TelePlot consumers
during a storm — `_is_dumping` blackout extends accordingly. F2 (deferred
streaming) is the natural pair to mitigate it. Acceptable for occasional
faults; combined with F2 if storm-frequency increases.

## Memory placement

- **Streamer rolling ring** (~140 KB): PSRAM. Sampler task writes at
  1 kHz, snapshot reads it once per burst. PSRAM bandwidth (>40 MB/s on
  octal PSRAM) is two orders of magnitude over the streamer's needs;
  latency doesn't matter for sequential bulk writes. Use
  `EXT_RAM_BSS_ATTR` for a static placement so allocation can't fail.
- **Per-burst snapshot buffers** (2 × 112 KB = 224 KB): PSRAM, same
  attribute. Written once at trigger time, read once at dump time.
- **Sampler task stack** (4 KB): SRAM (FreeRTOS default; task stacks
  must be in SRAM).

Total PSRAM commitment: ~370 KB out of 8 MB. Total SRAM commitment for
this feature: 4 KB stack + small static state. The current burst engine's
~160 KB in DRAM can also move to PSRAM as part of this migration, paying
back another ~160 KB of SRAM — net effect is *more* SRAM headroom after
F1 than before, despite the feature being substantially larger.

## Concurrency model

| State | Owner (write) | Readers (read) | Synchronization |
|---|---|---|---|
| Ring slot `[i]` payload | sampler task | `get_latest`, `snapshot` | publish via release-fence + atomic `write_idx` |
| `write_idx` | sampler task | `get_latest`, `snapshot` | naturally atomic (uint32 single-word, aligned) |
| `sample_count` | sampler task | `get_latest`, `snapshot` | naturally atomic; saturates at UINT32_MAX |
| Stats counters | sampler task | `get_stats` | naturally atomic per field; minor torn-read on multi-field is acceptable |
| Snapshot operation | snapshot caller | sampler task (paused) | mutex for the duration (`~1 ms`) |
| Per-burst buffers | capture task | capture task (dump) | single-task, no sync needed |

The mutex is the only blocking primitive on the hot path, and it's held
for ~1 ms only at trigger time + post-trigger snapshot — i.e. twice per
burst. Sampler skips one sample worst-case during a snapshot.

## Init order (in `app_main`)

```
init_i2c_bus();
init_ina226_all();             // creates handles in INA226_CONFIG_STEADY
ina226_streamer_init(s_ina226_12v, s_ina226_5v, s_ina226_3v3);
                               // rewrites the 3 main rails to INA226_CONFIG_HS
                               // spawns sampler task
ema_init(...);                 // ... rest of init unchanged ...
```

There's a brief window between `ina226_streamer_init` returning and the
sampler producing its first sample (~2 ms — one fast-mode conversion).
Main loop's `get_latest` calls during that window return
`ESP_ERR_INVALID_STATE` and the existing "hold at last-good EMA" path
treats it the same as a read failure. No special handling needed.

## Stats / observability

`status` (and `status json`) CLI surface the streamer stats:

```
streamer:  samples=12345678  fails=3  late=12  max_lateness=850us
```

Optionally a 1 Hz INFO log if `late_samples` rate climbs (e.g. 0x41
NACKs spiking under load) — surfaces hardware reliability issues early.
The streamer is uniquely well-placed to detect bus health since it's the
only continuous I²C consumer.

## Open questions deliberately left to implementation time

1. **Wall-clock alignment.** Today `ts_ms` in pre-trigger samples comes
   from `esp_timer_get_time() / 1000` (device boot time). Streamer
   samples use `ts_us` (also device boot time, 32-bit). Both pre and
   HS streams in the dump should agree on units. Recommend: emit
   `ts_ms = ts_us / 1000` at dump time, matching today's pre-trigger
   format. No spec lock-in here.

2. **Snapshot mutex implementation.** `portMUX_TYPE`/spinlock vs.
   `SemaphoreHandle_t` mutex. Spinlock is lower latency (~µs) and simpler
   for a held-for-1ms region; mutex is friendlier to FreeRTOS scheduling.
   Either works; spinlock is faster. Decide at implementation time based
   on whether the 1 ms hold pushes against any other hard real-time
   requirement.

3. **5VSB streamer inclusion.** Currently spec'd as **not** in the
   streamer (5VSB has no HS-capture interest, no detection cares about
   1 kHz 5VSB). Could be added with no extra bus cost (1 more device on
   the same fast-mode pace) for consistency. Decide if any future
   detector wants it.

4. **Ring depth tunable via CLI.** Could expose a `set hs_ring_seconds N`
   command for runtime experimentation. Probably overkill — recompile is
   fine for a one-time tuning exercise.

5. **Interaction with 1 MHz Fast-mode-Plus (FOLLOWUPS deferred item).**
   If the bus eventually moves to 1 MHz, the streamer can drop the
   voltage decimation (read v_* every sample at full 1 kHz). The
   `HS_VOLTAGE_DECIMATE` constant becomes 1; nothing else changes.

## Implementation order when this gets built

1. Land the `ina226_streamer` module standalone (init + sampler task +
   `get_latest` + `snapshot`). Verify with a unit test or a temporary
   debug log of stats.
2. Switch main.c's per-iteration reads to `get_latest`. Verify steady-
   state behavior unchanged. The capture engine still works exactly as
   today during this step.
3. Strip the burst engine's setup/teardown/sample_fn machinery.
   Replace HS loop with snapshot + delay + snapshot.
4. Remove `INA226_CONFIG_STEADY` for the 3 main rails (streamer init
   already rewrote them to HS; this just drops dead config).
5. Update CLAUDE.md "Burst capture" + "Hardware truth" sections;
   FOLLOWUPS L2 marked ✅ fixed; F1 entry marked ✅ implemented with a
   link to this design doc for posterity.

Each step is independently testable on hardware. The whole thing is
backward-compatible until step 3 — if step 1 or 2 misbehaves, easy
revert.

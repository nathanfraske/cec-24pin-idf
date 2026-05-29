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
| L1 | medium (latent) | `main.c` main loop, `cec_layer2_update` calls | Layer 2 is fed the **raw** instantaneous reading, which stays `0.0f` on a failed read while its EMA reference falls back to last-good. On the I²C-fed 5VSB rail a sustained read failure (3 consecutive at 50 Hz) yields a `~5 V` deviation and a spurious `TRANSIENT` burst. Fix: on a failed read, feed the EMA value as the instant too (or skip that channel's L2 update that iteration). Every other consumer (p_total, swings, classifier) already uses the held EMA. |
| L2 | medium (fragility) | `cec_capture.c` `spin_until` + `hs_capture_task` | The HS burst busy-spins one core for ~4 s via `taskYIELD`-polling, which starves the Core 1 idle task to ~80% of the default 5 s task-watchdog window — no margin for a larger `HS_BURST_BUF_SIZE` or more per-sample work. Also the ADC reader task is unpinned and lower-priority than the HS task, so it survives a burst only by migrating to Core 0; pinning it to Core 1 later would stale the HS cache mid-burst (flatline capture). Now that ADC is continuous/DMA, `spin_until` can `vTaskDelayUntil` instead of busy-spinning, removing both hazards. |
| L3 | low (smell) | `main.c` periodic save block | `cec_nvs_save_blob` (`nvs_set_blob` + `nvs_commit`) runs synchronously in the 50 Hz supervisor loop. Flash erase/write disables cache on both cores — a multi-hundred-ms loop stall every 5 min when profiles are dirty. Move to a low-priority one-shot task. |
| L4 | low (benign race) | `cec_capture.c` `cec_capture_trigger_with_text` | Check-then-set on `s_busy` isn't atomic; the main loop and CLI call the trigger from different tasks. Binary semaphore swallows the double-give, so worst case is a misleading "burst triggered" log with no second capture. A short critical section or atomic CAS closes it. |
| L5 | low (by design) | `cec_capture.c` `cec_capture_push` vs `dump_pretrigger` | Core 0 keeps writing the pre-trigger ring while Core 1 reads it during a dump. Memory-safe (modulo'd indices, fixed array) but can emit a few torn rows in the diagnostic dump. Inherent to the non-blocking design; documenting, not necessarily fixing. |
| L6 | nit | `main.c` shutdown detection | `v_12v_rate` is a raw ΔV over the 50-sample history but is compared against a `-0.5f` "V/s" threshold — correct only because 50 samples × 20 ms = exactly 1 s. Changing `V_12V_RATE_HISTORY_SIZE` or `SAMPLE_PERIOD_MS` silently breaks the units. Either divide by the real window seconds or add a static-assert tying the two constants together. |
| L7 | nit (stale) | `cec_capture.c` HS loop | The "~1% SAR-ADC zero-artifact" carry-forward is near-dead post-DMA (continuous mode holds the last value, never returns exactly 0), and its comment still describes the old oneshot behavior. Drop the mitigation or refresh the comment. |

## EPS-parity deferrals

From the cross-repo TODO list; carried over as the EPS side evolves.

- **B1** — Add `cec_load_state_t` and `CEC_LOAD_COUNT` to `cec_common/include/cec_state.h` (TODO marker already in place), mirroring the EPS-side `cec_classifier.c` enum. Land the actual values when next syncing EPS source so the two repos can include each other's headers without name clashes.
- **D1** — ACS712 driver style: the 24-pin uses a `const`-config struct + `_measure_zero_point` helper; the EPS-side `acs758` keeps a runtime-mutable ctx with in-place cal. Both work. If either is rewritten, the const-config + measure-helper pattern is the cleaner target. Low priority — and moot here once the rails move to INA226.
- **A4 / G** — `cec_comms` component for CAN/TWAI when CAN ships on the 24-pin. Use the `esp_twai_onchip` node-handle API directly (skip the deprecated `driver/twai.h`). EPS-side `cec_comms/cec_can.c` is the reference; adapt frame layout / IDs for the 24-pin payload.

## Hardware-driven

- ACS712 → INA226 swap on all rails (planned). Fixes the residual trim drift
  and the i_5v zero-load noise, and brings the per-rail current path onto the
  same I²C device family as 5VSB. Brings a runtime INA226 cal command with it.

/*
 * Burst capture engine.
 *
 * Maintains a pre-trigger ring buffer of low-rate samples (50 Hz) that's
 * written every iteration of the main loop. On a trigger, dumps the ring
 * to TelePlot and then captures a 1 kHz HS window on a dedicated task
 * pinned to Core 1, so the main loop on Core 0 keeps producing real-time
 * telemetry throughout the burst.
 *
 * The capture engine is decoupled from specific sensor configs: callers
 * provide a sample callback that reads all six main-rail channels at HS
 * rate. The engine handles timing, storage, dump, and cooldown.
 *
 * Output format matches v0.5.9 so existing capture-analysis tooling
 * works unchanged:
 *
 *   >BURST_BEGIN:<reason>:<n_pre>_normal+<n_hs>_hs:<state>
 *   >b_v_12v:<ts_ms>:<value>            (pre-trigger samples, 50 Hz)
 *   ... b_i_12v / b_v_5v / b_i_5v / etc.
 *   >hs_v_12v:<ts_ms>:<value>           (HS samples, 1 kHz)
 *   ... hs_i_12v / hs_v_5v / etc.
 *   >BURST_END
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Trigger sources. Names line up with v0.5.9. */
typedef enum {
    CEC_TRIG_NONE = 0,
    CEC_TRIG_MANUAL,
    CEC_TRIG_STATIC_WARN,
    CEC_TRIG_STATIC_CRIT,
    CEC_TRIG_TRANSIENT,
    CEC_TRIG_ANOMALY,
    CEC_TRIG_STATE_CHANGE,
    CEC_TRIG_SHUTDOWN,
    CEC_TRIG_POWER_SWING,
    CEC_TRIG_CURRENT_SWING,
    CEC_TRIG_COUNT,
} cec_trigger_t;

/* Pre-trigger sample (50 Hz, full sensor set). */
typedef struct {
    uint32_t ts_ms;
    uint8_t  state;     /* cec_state_t cast to byte */
    float    v_12v,  i_12v;
    float    v_5v,   i_5v;
    float    v_3v3,  i_3v3;
    float    v_5vsb, i_5vsb;
    float    temp_c;
} cec_capture_sample_t;

/* HS sample (1 kHz, main rails only). Slim by design so the buffer
 * fits comfortably and per-sample callback runtime stays small. */
typedef struct {
    uint32_t ts_us_offset;   /* microseconds since HS capture start */
    float    v_12v, i_12v;
    float    v_5v,  i_5v;
    float    v_3v3, i_3v3;
} cec_capture_hs_sample_t;

/* Callback the capture engine calls at 1 kHz during the HS window.
 * Must complete well within the 1 ms budget. Runs on the HS task
 * (Core 1 by default) so it can safely call cec_adc_read_mv et al. */
typedef void (*cec_capture_hs_sample_fn_t)(cec_capture_hs_sample_t *out);

/* Human-readable trigger name for log/teleplot output. */
const char *cec_trigger_name(cec_trigger_t t);

/*
 * Initialize the engine. Creates the HS capture task pinned to Core 1,
 * registers the sample callback, zeroes the pre-trigger ring. Idempotent.
 */
esp_err_t cec_capture_init(cec_capture_hs_sample_fn_t hs_sample_fn);

/*
 * Push a sample into the pre-trigger ring. Called from the main loop
 * on every iteration (50 Hz). Lock-free for the writer; reads happen
 * only at dump time from the HS task.
 */
void cec_capture_push(const cec_capture_sample_t *s);

/*
 * Request a burst capture.
 *
 *   ESP_OK                  - capture started
 *   ESP_ERR_NOT_FINISHED    - a capture is already running
 *   ESP_ERR_INVALID_STATE   - within cooldown window from the previous burst
 *
 * Returns immediately; the capture and dump happen on the HS task.
 */
esp_err_t cec_capture_trigger(cec_trigger_t reason);

/*
 * True between the moment a trigger is accepted and the moment the
 * dump completes.
 */
bool cec_capture_is_busy(void);

#ifdef __cplusplus
}
#endif

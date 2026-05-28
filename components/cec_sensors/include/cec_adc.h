/*
 * ADC1 continuous-mode (DMA) wrapper for the CEC 24-pin module.
 *
 * The driver runs ADC1 in continuous mode at a fixed per-channel rate
 * (1 kHz). A background reader task drains the DMA buffer and keeps a
 * per-channel "latest calibrated millivolts" table. Reads via this API
 * are constant-time table lookups — no oneshot conversions, no
 * cross-core ADC lock contention.
 *
 * All channels live on ADC1 with the same attenuation (ADC_ATTEN_DB_12,
 * 0..~3.3 V at the pin), sufficient for every sensor on this PCB.
 *
 * Setup flow:
 *   cec_adc_init();
 *   cec_adc_setup_channel(ADC_CH_V_12V);
 *   cec_adc_setup_channel(ADC_CH_V_5V);
 *   ... (one call per channel; thermistor / acs712 drivers register theirs)
 *   cec_adc_start();
 *
 * After cec_adc_start the pattern is locked; further setup_channel calls
 * return ESP_ERR_INVALID_STATE.
 */

#pragma once

#include "esp_err.h"
#include "esp_adc/adc_continuous.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Per-channel configuration carried by the caller. Held as a const,
 * passed to cec_adc_read on every read.
 */
typedef struct {
    adc_channel_t channel;   /* ADC1 channel (CH0..CH9 on ESP32-S3) */
    int   samples;           /* IGNORED in continuous mode; retained for API
                                compatibility — see cec_adc_read_mv */
    float scale;             /* Hardware divider, Vrail = Vpin * scale */
    float trim;              /* Per-rail calibration trim (1.0 = no trim) */
} cec_adc_rail_t;

/*
 * One-time calibration setup. Idempotent; safe to call before any
 * channel is configured. Does NOT start the ADC; that happens in
 * cec_adc_start once all channels have been registered.
 */
esp_err_t cec_adc_init(void);

/*
 * Register a channel for inclusion in the continuous-mode pattern.
 * Idempotent. Must be called before cec_adc_start. Returns
 * ESP_ERR_INVALID_STATE if cec_adc_start has already run.
 */
esp_err_t cec_adc_setup_channel(adc_channel_t channel);

/*
 * Apply the accumulated pattern, start continuous-mode sampling, and
 * spawn the reader task. Idempotent. Must be called once after all
 * cec_adc_setup_channel calls; subsequent reads only return data after
 * this returns.
 */
esp_err_t cec_adc_start(void);

/*
 * Read the latest calibrated pin voltage in millivolts for `channel`.
 *
 * `samples` is preserved in the signature for source compatibility with
 * the previous oneshot driver, but is IGNORED — the underlying sample
 * rate is fixed at start time and any caller-side averaging is the job
 * of EMAs / medians, not of this primitive.
 *
 * Returns ESP_ERR_NOT_FOUND on the brief startup window before the
 * reader task has seen any sample for `channel`.
 */
esp_err_t cec_adc_read_mv(adc_channel_t channel, int samples, int *out_mv);

/*
 * Read a rail. Applies the curve-fit calibration (already done in the
 * latest-mV table), the hardware scale, and per-rail trim. Returns the
 * final voltage in volts.
 */
esp_err_t cec_adc_read(const cec_adc_rail_t *rail, float *out_volts);

#ifdef __cplusplus
}
#endif

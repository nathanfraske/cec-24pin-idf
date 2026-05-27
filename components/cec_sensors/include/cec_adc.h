/*
 * Generic ADC oneshot wrapper for the CEC 24-pin module.
 *
 * Wraps the ESP-IDF 6.x adc_oneshot + adc_cali drivers to give callers a
 * single function that returns a calibrated, scaled, trimmed voltage in
 * volts. All channels live on ADC1 and use the same attenuation
 * (ADC_ATTEN_DB_12, 0-~3.3 V at the pin), which is sufficient for every
 * sensor on this PCB.
 *
 * Used by main.c for the 12V/5V/3V3 voltage rails; the future ACS712 and
 * NTC drivers in cec_sensors will consume this same API.
 */

#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Per-channel configuration carried by the caller. Held as a const,
 * passed to cec_adc_read on every read.
 */
typedef struct {
    adc_channel_t channel;   /* ADC1 channel (CH0..CH9 on ESP32-S3) */
    int samples;             /* Averaging count per read, >= 1 */
    float scale;             /* Hardware divider, Vrail = Vpin * scale */
    float trim;              /* Per-rail calibration trim (1.0 = no trim) */
} cec_adc_rail_t;

/*
 * One-time ADC1 + curve-fit calibration setup. Idempotent; safe to call
 * before any channel is configured.
 */
esp_err_t cec_adc_init(void);

/*
 * Configure a single ADC1 channel. Must be called once per channel before
 * cec_adc_read or cec_adc_read_mv.
 */
esp_err_t cec_adc_setup_channel(adc_channel_t channel);

/*
 * Read the calibrated pin voltage in millivolts, averaged over `samples`
 * raw conversions. Lower-level than cec_adc_read; used by sensor drivers
 * (thermistor, ACS712) whose post-processing isn't a simple linear
 * rail-divider scale.
 */
esp_err_t cec_adc_read_mv(adc_channel_t channel, int samples, int *out_mv);

/*
 * Read a rail. Averages `rail->samples` raw conversions, applies the
 * curve-fit calibration, then the hardware scale and per-rail trim.
 * Returns the final voltage in volts.
 */
esp_err_t cec_adc_read(const cec_adc_rail_t *rail, float *out_volts);

#ifdef __cplusplus
}
#endif

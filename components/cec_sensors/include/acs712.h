/*
 * ACS712 Hall-effect current sensor driver.
 *
 * The ACS712 outputs an analog voltage centered on ~Vcc/2 at no load,
 * deviating linearly with current. On this board its output passes through
 * a 20k/30k divider before reaching the ESP32 ADC pin, so the read path is:
 *
 *   v_pin = (Vcc_at_sensor / (R_top + R_bot)) * R_bot
 *   v_acs = v_pin * divider_scale            // recover sensor output
 *   I     = (v_acs - zero_point_v) / sensitivity_v_per_a
 *
 * Sensitivity is part-specific: 0.066 V/A for the 30A version (used on the
 * 12V rail), 0.100 V/A for the 20A version (5V and 3.3V rails).
 *
 * Zero point needs per-unit calibration: the part-to-part variation in
 * the no-load output is several tens of mV, which is hundreds of mA at
 * the sensitivities above. v0.5.9 calibrates at boot with the PSU
 * disconnected; that path lands with the serial-command and NVS work.
 * For now the field is a const initializer (2.20 V is the nominal Vcc/2
 * post-divider).
 */

#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    adc_channel_t channel;        /* ADC1 channel the sensor's divider sits on */
    int samples;                  /* Averaging count per read, >= 1 */
    float divider_scale;          /* Recovers sensor output from pin voltage */
    float sensitivity_v_per_a;    /* Part-specific (e.g. 0.066 for 30A, 0.100 for 20A) */
    float zero_point_v;           /* Sensor output at no load (post-divider scale) */
} acs712_t;

/*
 * Configure the ADC channel the sensor sits on. Must be called once
 * before acs712_read_amps.
 */
esp_err_t acs712_setup(const acs712_t *s);

/*
 * Read the current in amps. Sign reflects current direction.
 */
esp_err_t acs712_read_amps(const acs712_t *s, float *out_a);

#ifdef __cplusplus
}
#endif

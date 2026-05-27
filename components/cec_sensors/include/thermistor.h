/*
 * NTC thermistor driver using the Beta-coefficient equation.
 *
 * Assumes a standard divider: VCC --- pull-up resistor --- ADC pin --- NTC --- GND.
 * As the NTC heats, its resistance drops and the pin voltage drops with it.
 *
 * Hardware constants come from the per-unit schematic; carry them forward
 * from the v0.5.9 firmware unless the board is rewired.
 *
 * Reads bypass the rail-style scale/trim path in cec_adc and go through
 * cec_adc_read_mv to get the raw calibrated pin voltage, which then feeds
 * the Beta equation.
 */

#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    adc_channel_t channel;        /* ADC1 channel the divider sits on */
    int samples;                  /* Averaging count per read, >= 1 */
    float beta;                   /* Thermistor B coefficient (Kelvin) */
    float nominal_resistance;     /* R at nominal_temperature_k (ohms) */
    float nominal_temperature_k;  /* Reference temperature, typically 298.15 (25C) */
    float pull_up_resistance;     /* Series pull-up resistor value (ohms) */
    float vcc;                    /* Supply voltage at the top of the divider */
} thermistor_t;

/*
 * Configure the ADC channel the thermistor lives on. Must be called once
 * before thermistor_read_celsius.
 */
esp_err_t thermistor_setup(const thermistor_t *t);

/*
 * Read the thermistor temperature in degrees Celsius.
 *
 * Returns ESP_ERR_INVALID_RESPONSE if the pin voltage is at one of the
 * divider rails (NTC open-circuited or shorted), in which case *out_c is
 * not written.
 */
esp_err_t thermistor_read_celsius(const thermistor_t *t, float *out_c);

#ifdef __cplusplus
}
#endif

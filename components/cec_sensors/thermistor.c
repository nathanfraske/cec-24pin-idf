/*
 * NTC thermistor driver.
 */

#include <math.h>
#include "esp_check.h"
#include "cec_adc.h"
#include "thermistor.h"

static const char *TAG = "thermistor";

/* Pin voltages within this margin of 0 or VCC are treated as a wiring
 * fault (NTC open-circuited or shorted) rather than a real reading. */
#define THERMISTOR_RAIL_MARGIN_V 0.01f

esp_err_t thermistor_setup(const thermistor_t *t)
{
    if (t == NULL) return ESP_ERR_INVALID_ARG;
    return cec_adc_setup_channel(t->channel);
}

esp_err_t thermistor_read_celsius(const thermistor_t *t, float *out_c)
{
    if (t == NULL || out_c == NULL) return ESP_ERR_INVALID_ARG;
    if (t->beta <= 0.0f || t->nominal_resistance <= 0.0f ||
        t->pull_up_resistance <= 0.0f || t->vcc <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    int mv = 0;
    ESP_RETURN_ON_ERROR(cec_adc_read_mv(t->channel, t->samples, &mv),
                        TAG, "cec_adc_read_mv");
    float v = mv / 1000.0f;

    if (v <= THERMISTOR_RAIL_MARGIN_V || v >= t->vcc - THERMISTOR_RAIL_MARGIN_V) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Beta equation:
     *   R_ntc = R_pullup * V_pin / (Vcc - V_pin)
     *   1/T   = 1/T0 + (1/B) * ln(R_ntc / R0)
     * Solve for T in Kelvin, return Celsius.
     */
    float r_ntc = t->pull_up_resistance * v / (t->vcc - v);
    float inv_t = 1.0f / t->nominal_temperature_k
                  + logf(r_ntc / t->nominal_resistance) / t->beta;
    *out_c = (1.0f / inv_t) - 273.15f;
    return ESP_OK;
}

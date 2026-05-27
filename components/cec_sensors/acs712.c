/*
 * ACS712 Hall-effect current sensor driver.
 */

#include "esp_check.h"
#include "cec_adc.h"
#include "acs712.h"

static const char *TAG = "acs712";

esp_err_t acs712_setup(const acs712_t *s)
{
    if (s == NULL) return ESP_ERR_INVALID_ARG;
    return cec_adc_setup_channel(s->channel);
}

esp_err_t acs712_read_amps(const acs712_t *s, float *out_a)
{
    if (s == NULL || out_a == NULL) return ESP_ERR_INVALID_ARG;
    if (s->sensitivity_v_per_a <= 0.0f) return ESP_ERR_INVALID_ARG;

    int mv = 0;
    ESP_RETURN_ON_ERROR(cec_adc_read_mv(s->channel, s->samples, &mv),
                        TAG, "cec_adc_read_mv");

    float v_pin = mv / 1000.0f;
    float v_acs = v_pin * s->divider_scale;
    *out_a = (v_acs - s->zero_point_v) / s->sensitivity_v_per_a;
    return ESP_OK;
}

esp_err_t acs712_measure_zero_point(const acs712_t *s, int samples, float *out_v)
{
    if (s == NULL || out_v == NULL || samples < 1) return ESP_ERR_INVALID_ARG;

    int mv = 0;
    ESP_RETURN_ON_ERROR(cec_adc_read_mv(s->channel, samples, &mv),
                        TAG, "cec_adc_read_mv");

    *out_v = (mv / 1000.0f) * s->divider_scale;
    return ESP_OK;
}

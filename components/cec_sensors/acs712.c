/*
 * ACS712 Hall-effect current sensor driver.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

    /* cec_adc is in continuous mode at 1 kHz per channel; a single call
     * returns the latest cached sample. Sleep 1 ms between reads so each
     * iteration sees a distinct sample, then average. Gives a clean
     * no-load baseline for the boot diagnostic and any future runtime
     * calibration command. */
    int64_t sum_mv = 0;
    int taken = 0;
    for (int i = 0; i < samples; i++) {
        int mv = 0;
        esp_err_t err = cec_adc_read_mv(s->channel, 1, &mv);
        if (err == ESP_OK) {
            sum_mv += mv;
            taken++;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (taken == 0) return ESP_ERR_NOT_FOUND;

    float v_pin = (float)sum_mv / (float)taken / 1000.0f;
    *out_v = v_pin * s->divider_scale;
    return ESP_OK;
}

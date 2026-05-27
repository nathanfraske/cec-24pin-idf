/*
 * Generic ADC oneshot wrapper.
 */

#include <stdbool.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "cec_adc.h"

static const char *TAG = "cec_adc";

static adc_oneshot_unit_handle_t s_unit = NULL;
static adc_cali_handle_t s_cali = NULL;
static bool s_inited = false;

esp_err_t cec_adc_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_cfg, &s_unit),
                        TAG, "adc_oneshot_new_unit");

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali),
                        TAG, "adc_cali_create_scheme_curve_fitting");

    s_inited = true;
    ESP_LOGI(TAG, "ADC1 + curve-fit calibration initialized (atten=DB_12)");
    return ESP_OK;
}

esp_err_t cec_adc_setup_channel(adc_channel_t channel)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    adc_oneshot_chan_cfg_t cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    return adc_oneshot_config_channel(s_unit, channel, &cfg);
}

esp_err_t cec_adc_read_mv(adc_channel_t channel, int samples, int *out_mv)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out_mv == NULL || samples < 1) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t sum_mv = 0;
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_unit, channel, &raw),
                            TAG, "adc_oneshot_read ch=%d", (int)channel);
        int mv = 0;
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cali, raw, &mv),
                            TAG, "adc_cali_raw_to_voltage");
        sum_mv += mv;
    }
    *out_mv = (int)(sum_mv / samples);
    return ESP_OK;
}

esp_err_t cec_adc_read(const cec_adc_rail_t *rail, float *out_volts)
{
    if (rail == NULL || out_volts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    int mv = 0;
    ESP_RETURN_ON_ERROR(cec_adc_read_mv(rail->channel, rail->samples, &mv),
                        TAG, "cec_adc_read_mv");
    *out_volts = (mv / 1000.0f) * rail->scale * rail->trim;
    return ESP_OK;
}

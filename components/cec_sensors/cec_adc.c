/*
 * ADC1 continuous-mode (DMA) driver.
 *
 * Replaces the original oneshot implementation. The hardware now samples
 * every configured channel at a fixed cadence (1 kHz per channel) and a
 * dedicated reader task drains the DMA buffer into a per-channel "latest
 * calibrated mV" table. Callers (main loop, ACS712, thermistor, HS burst
 * capture) hit that table on every read — constant-time, lock-free,
 * jitter-free.
 *
 * Why: the oneshot path serialized every read through the ADC unit's
 * internal mutex. The HS capture task on Core 1 and the 50 Hz main loop
 * on Core 0 hit that mutex on overlapping schedules, which produced the
 * ~5% "2 ms gap" jitter we documented in the burst capture engine.
 * Continuous mode removes the lock entirely.
 *
 * The `samples` parameter on cec_adc_read_mv is preserved for source
 * compatibility but is now ignored — the underlying sample rate is
 * fixed at config time and caller-side filtering (EMA, median) handles
 * the smoothing it used to provide via per-call averaging.
 */

#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "cec_adc.h"

static const char *TAG = "cec_adc";

/* Per-channel sample rate. Total ADC sample rate is this times the
 * number of channels in the pattern. 1 kHz per channel matches the
 * HS capture cadence so each HS read sees a fresh sample. */
#define ADC_PER_CHAN_HZ        1000U

/* ESP32-S3 ADC1 has 10 channels (CH0..CH9). */
#define MAX_CHANNELS           10

/* DMA buffer sizing. Reader task drains the buffer aggressively so
 * overflow is the only failure mode worth worrying about: at 7 channels
 * × 1 kHz × 4 bytes/sample = 28 KB/s, a 4 KB buffer fills every ~140 ms.
 * The reader runs much faster than that. */
#define DMA_FRAME_BYTES        256
#define DMA_BUF_BYTES          4096

#define READER_TASK_STACK      4096
#define READER_TASK_PRIORITY   (configMAX_PRIORITIES - 3)
#define READ_TIMEOUT_MS        100

static adc_continuous_handle_t s_handle = NULL;
static adc_cali_handle_t       s_cali = NULL;

/* Per-channel latest calibrated mV. int writes are word-sized and
 * atomic on ESP32-S3, so readers can sample without locking. */
static volatile int  s_latest_mv[MAX_CHANNELS];
static volatile bool s_channel_seen[MAX_CHANNELS];

/* Channel list accumulated via cec_adc_setup_channel and applied as a
 * pattern when cec_adc_start runs. */
static adc_channel_t s_pattern_channels[MAX_CHANNELS];
static size_t        s_pattern_count = 0;

static bool s_inited  = false;
static bool s_started = false;

esp_err_t cec_adc_init(void)
{
    if (s_inited) return ESP_OK;

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali),
                        TAG, "adc_cali_create_scheme_curve_fitting");

    memset((void *)s_latest_mv, 0, sizeof(s_latest_mv));
    memset((void *)s_channel_seen, 0, sizeof(s_channel_seen));
    s_pattern_count = 0;
    s_inited = true;
    ESP_LOGI(TAG, "ADC1 + curve-fit calibration ready (atten=DB_12)");
    return ESP_OK;
}

esp_err_t cec_adc_setup_channel(adc_channel_t channel)
{
    if (!s_inited)  return ESP_ERR_INVALID_STATE;
    if (s_started)  return ESP_ERR_INVALID_STATE;   /* pattern locked once started */
    if ((int)channel < 0 || (int)channel >= MAX_CHANNELS) return ESP_ERR_INVALID_ARG;

    /* Idempotent: re-registering an already-known channel is a no-op. */
    for (size_t i = 0; i < s_pattern_count; i++) {
        if (s_pattern_channels[i] == channel) return ESP_OK;
    }
    if (s_pattern_count >= MAX_CHANNELS) return ESP_ERR_NO_MEM;
    s_pattern_channels[s_pattern_count++] = channel;
    return ESP_OK;
}

static void reader_task(void *arg)
{
    (void)arg;
    static uint8_t buf[DMA_FRAME_BYTES];

    while (1) {
        uint32_t bytes_read = 0;
        esp_err_t err = adc_continuous_read(s_handle, buf, sizeof(buf),
                                            &bytes_read, READ_TIMEOUT_MS);
        if (err == ESP_ERR_TIMEOUT) {
            /* No data this window — usually means the reader is keeping
             * up perfectly. Loop and try again. */
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "adc_continuous_read: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= bytes_read;
             i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&buf[i];
            uint32_t ch  = p->type2.channel;
            uint32_t raw = p->type2.data;
            if (ch >= MAX_CHANNELS) continue;
            int mv = 0;
            if (adc_cali_raw_to_voltage(s_cali, (int)raw, &mv) == ESP_OK) {
                s_latest_mv[ch] = mv;
                s_channel_seen[ch] = true;
            }
        }
    }
}

esp_err_t cec_adc_start(void)
{
    if (!s_inited)           return ESP_ERR_INVALID_STATE;
    if (s_started)           return ESP_OK;
    if (s_pattern_count == 0) return ESP_ERR_INVALID_STATE;

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = DMA_BUF_BYTES,
        .conv_frame_size    = DMA_FRAME_BYTES,
    };
    ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&handle_cfg, &s_handle),
                        TAG, "adc_continuous_new_handle");

    adc_digi_pattern_config_t pattern[MAX_CHANNELS];
    for (size_t i = 0; i < s_pattern_count; i++) {
        pattern[i].atten     = ADC_ATTEN_DB_12;
        pattern[i].channel   = s_pattern_channels[i];
        pattern[i].unit      = ADC_UNIT_1;
        pattern[i].bit_width = ADC_BITWIDTH_12;
    }

    adc_continuous_config_t dig_cfg = {
        .pattern_num    = s_pattern_count,
        .adc_pattern    = pattern,
        .sample_freq_hz = ADC_PER_CHAN_HZ * (uint32_t)s_pattern_count,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    ESP_RETURN_ON_ERROR(adc_continuous_config(s_handle, &dig_cfg),
                        TAG, "adc_continuous_config");
    ESP_RETURN_ON_ERROR(adc_continuous_start(s_handle),
                        TAG, "adc_continuous_start");

    if (xTaskCreate(reader_task, "cec_adc_rd", READER_TASK_STACK, NULL,
                    READER_TASK_PRIORITY, NULL) != pdPASS) {
        return ESP_FAIL;
    }

    s_started = true;
    ESP_LOGI(TAG, "continuous mode running: %u channels @ %u Hz each (%u Hz total)",
             (unsigned)s_pattern_count, (unsigned)ADC_PER_CHAN_HZ,
             (unsigned)(ADC_PER_CHAN_HZ * s_pattern_count));
    return ESP_OK;
}

esp_err_t cec_adc_read_mv(adc_channel_t channel, int samples, int *out_mv)
{
    (void)samples;  /* see file header */
    if (!s_started)                                        return ESP_ERR_INVALID_STATE;
    if (out_mv == NULL)                                    return ESP_ERR_INVALID_ARG;
    if ((int)channel < 0 || (int)channel >= MAX_CHANNELS)  return ESP_ERR_INVALID_ARG;
    if (!s_channel_seen[channel])                          return ESP_ERR_NOT_FOUND;

    *out_mv = s_latest_mv[channel];
    return ESP_OK;
}

esp_err_t cec_adc_read(const cec_adc_rail_t *rail, float *out_volts)
{
    if (rail == NULL || out_volts == NULL) return ESP_ERR_INVALID_ARG;
    int mv = 0;
    ESP_RETURN_ON_ERROR(cec_adc_read_mv(rail->channel, rail->samples, &mv),
                        TAG, "cec_adc_read_mv");
    *out_volts = (mv / 1000.0f) * rail->scale * rail->trim;
    return ESP_OK;
}

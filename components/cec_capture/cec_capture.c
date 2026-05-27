/*
 * Burst capture engine.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cec_capture.h"

static const char *TAG = "cec_capture";

/* Sizes from v0.5.9. Pre-trigger covers 20 s at 50 Hz, HS covers 4 s at 1 kHz. */
#define PRE_TRIGGER_BUF_SIZE   1000
#define HS_BURST_BUF_SIZE      4000
#define HS_SAMPLE_INTERVAL_US  1000

/* Cooldown between bursts. SHUTDOWN bypasses this so a real shutdown
 * sequence can still be captured even if it follows another trigger. */
#define BURST_COOLDOWN_US      (10LL * 1000 * 1000)

/* HS task settings. Pinning to Core 1 leaves Core 0 free for the main
 * loop and IDF background tasks. Stack is generous since the dump uses
 * printf in tight loops. */
#define HS_TASK_STACK          8192
#define HS_TASK_PRIORITY       (configMAX_PRIORITIES - 2)
#define HS_TASK_CORE_ID        1

static cec_capture_sample_t   s_pre_buf[PRE_TRIGGER_BUF_SIZE];
static cec_capture_hs_sample_t s_hs_buf[HS_BURST_BUF_SIZE];

static volatile int s_pre_write_idx = 0;
static volatile int s_pre_count = 0;

static cec_capture_hs_sample_fn_t s_hs_fn = NULL;
static SemaphoreHandle_t s_trigger_sem = NULL;
static TaskHandle_t s_hs_task = NULL;

static volatile bool s_busy = false;
static volatile cec_trigger_t s_pending_reason = CEC_TRIG_NONE;
static volatile int64_t s_last_complete_us = 0;
static volatile bool s_inited = false;

static const char *TRIGGER_NAMES[CEC_TRIG_COUNT] = {
    [CEC_TRIG_NONE]          = "none",
    [CEC_TRIG_MANUAL]        = "manual",
    [CEC_TRIG_STATIC_WARN]   = "static_warn",
    [CEC_TRIG_STATIC_CRIT]   = "static_crit",
    [CEC_TRIG_TRANSIENT]     = "transient",
    [CEC_TRIG_ANOMALY]       = "anomaly",
    [CEC_TRIG_STATE_CHANGE]  = "state_change",
    [CEC_TRIG_SHUTDOWN]      = "shutdown",
    [CEC_TRIG_POWER_SWING]   = "power_swing",
    [CEC_TRIG_CURRENT_SWING] = "current_swing",
};

const char *cec_trigger_name(cec_trigger_t t)
{
    if ((int)t < 0 || (int)t >= CEC_TRIG_COUNT) return "?";
    return TRIGGER_NAMES[t];
}

void cec_capture_push(const cec_capture_sample_t *s)
{
    if (!s_inited || s == NULL) return;
    int idx = s_pre_write_idx;
    s_pre_buf[idx] = *s;
    s_pre_write_idx = (idx + 1) % PRE_TRIGGER_BUF_SIZE;
    if (s_pre_count < PRE_TRIGGER_BUF_SIZE) s_pre_count++;
}

bool cec_capture_is_busy(void)
{
    return s_busy;
}

esp_err_t cec_capture_trigger(cec_trigger_t reason)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (s_busy)    return ESP_ERR_NOT_FINISHED;

    int64_t now = esp_timer_get_time();
    bool bypass_cooldown = (reason == CEC_TRIG_SHUTDOWN);
    if (!bypass_cooldown && s_last_complete_us != 0
        && (now - s_last_complete_us) < BURST_COOLDOWN_US) {
        return ESP_ERR_INVALID_STATE;
    }

    s_busy = true;
    s_pending_reason = reason;
    xSemaphoreGive(s_trigger_sem);
    return ESP_OK;
}

/* Spin until target_us, yielding to other Core 1 work between checks.
 * At our priority on a dedicated core there's almost nothing else to
 * run, but yielding keeps the watchdog happy and lets background tasks
 * (IDF event loops, etc.) breathe. */
static void spin_until(int64_t target_us)
{
    while ((int64_t)(esp_timer_get_time() - target_us) < 0) {
        taskYIELD();
    }
}

static void dump_pretrigger(uint8_t state_at_trigger)
{
    int start = (s_pre_write_idx - s_pre_count + PRE_TRIGGER_BUF_SIZE) % PRE_TRIGGER_BUF_SIZE;
    for (int i = 0; i < s_pre_count; i++) {
        int idx = (start + i) % PRE_TRIGGER_BUF_SIZE;
        const cec_capture_sample_t *p = &s_pre_buf[idx];
        unsigned ts = (unsigned)p->ts_ms;
        printf(">b_v_12v:%u:%.3f\n",  ts, p->v_12v);
        printf(">b_i_12v:%u:%.3f\n",  ts, p->i_12v);
        printf(">b_v_5v:%u:%.3f\n",   ts, p->v_5v);
        printf(">b_i_5v:%u:%.3f\n",   ts, p->i_5v);
        printf(">b_v_3v3:%u:%.3f\n",  ts, p->v_3v3);
        printf(">b_i_3v3:%u:%.3f\n",  ts, p->i_3v3);
        printf(">b_v_5vsb:%u:%.3f\n", ts, p->v_5vsb);
        printf(">b_i_5vsb:%u:%.4f\n", ts, p->i_5vsb);
        printf(">b_temp:%u:%.2f\n",   ts, p->temp_c);
        printf(">b_state:%u:%d\n",    ts, (int)p->state);
    }
    /* state_at_trigger is logged via BURST_BEGIN; keep parameter for
     * future use (e.g. annotating the trigger sample). */
    (void)state_at_trigger;
}

static void dump_hs(int64_t hs_start_us)
{
    uint32_t hs_start_ms = (uint32_t)(hs_start_us / 1000);
    for (int i = 0; i < HS_BURST_BUF_SIZE; i++) {
        const cec_capture_hs_sample_t *s = &s_hs_buf[i];
        unsigned ts = (unsigned)(hs_start_ms + (s->ts_us_offset / 1000));
        printf(">hs_v_12v:%u:%.3f\n", ts, s->v_12v);
        printf(">hs_i_12v:%u:%.3f\n", ts, s->i_12v);
        printf(">hs_v_5v:%u:%.3f\n",  ts, s->v_5v);
        printf(">hs_i_5v:%u:%.3f\n",  ts, s->i_5v);
        printf(">hs_v_3v3:%u:%.3f\n", ts, s->v_3v3);
        printf(">hs_i_3v3:%u:%.3f\n", ts, s->i_3v3);
    }
}

static void hs_capture_task(void *arg)
{
    (void)arg;
    while (1) {
        xSemaphoreTake(s_trigger_sem, portMAX_DELAY);

        cec_trigger_t reason = s_pending_reason;
        uint8_t state_at_trigger = s_pre_count > 0
            ? s_pre_buf[(s_pre_write_idx - 1 + PRE_TRIGGER_BUF_SIZE) % PRE_TRIGGER_BUF_SIZE].state
            : 0;

        ESP_LOGI(TAG, "burst trigger=%s, capturing %d HS samples at 1 kHz",
                 cec_trigger_name(reason), HS_BURST_BUF_SIZE);

        int64_t hs_start_us = esp_timer_get_time();
        int64_t target_us = hs_start_us;
        for (int i = 0; i < HS_BURST_BUF_SIZE; i++) {
            target_us += HS_SAMPLE_INTERVAL_US;
            spin_until(target_us);
            cec_capture_hs_sample_t *s = &s_hs_buf[i];
            s->ts_us_offset = (uint32_t)(esp_timer_get_time() - hs_start_us);
            s_hs_fn(s);
        }
        int64_t hs_end_us = esp_timer_get_time();
        ESP_LOGI(TAG, "HS capture done in %lld us, dumping pre+HS to TelePlot",
                 (long long)(hs_end_us - hs_start_us));

        printf(">BURST_BEGIN:%s:%d_normal+%d_hs:%d\n",
               cec_trigger_name(reason),
               s_pre_count, HS_BURST_BUF_SIZE,
               (int)state_at_trigger);
        dump_pretrigger(state_at_trigger);
        dump_hs(hs_start_us);
        printf(">BURST_END\n");

        s_last_complete_us = esp_timer_get_time();
        s_busy = false;
        ESP_LOGI(TAG, "burst complete");
    }
}

esp_err_t cec_capture_init(cec_capture_hs_sample_fn_t hs_sample_fn)
{
    if (s_inited) return ESP_OK;
    if (hs_sample_fn == NULL) return ESP_ERR_INVALID_ARG;

    s_hs_fn = hs_sample_fn;
    memset(s_pre_buf, 0, sizeof(s_pre_buf));
    s_pre_write_idx = 0;
    s_pre_count = 0;

    s_trigger_sem = xSemaphoreCreateBinary();
    if (s_trigger_sem == NULL) return ESP_ERR_NO_MEM;

    BaseType_t r = xTaskCreatePinnedToCore(hs_capture_task, "cec_hs_cap",
                                           HS_TASK_STACK, NULL,
                                           HS_TASK_PRIORITY, &s_hs_task,
                                           HS_TASK_CORE_ID);
    if (r != pdPASS) {
        vSemaphoreDelete(s_trigger_sem);
        s_trigger_sem = NULL;
        return ESP_FAIL;
    }

    s_inited = true;
    ESP_LOGI(TAG, "engine ready: pre-trigger=%d samples @ 50 Hz, "
                  "HS=%d samples @ 1 kHz, cooldown=%lld ms",
             PRE_TRIGGER_BUF_SIZE, HS_BURST_BUF_SIZE,
             (long long)(BURST_COOLDOWN_US / 1000));
    return ESP_OK;
}

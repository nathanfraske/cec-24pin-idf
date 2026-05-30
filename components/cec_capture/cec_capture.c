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
#include "cec_teleplot.h"
#include "cec_capture.h"

static const char *TAG = "cec_capture";

/* Sizes from v0.5.9. Pre-trigger covers 20 s at 50 Hz, HS covers 4 s at 1 kHz. */
#define PRE_TRIGGER_BUF_SIZE   1000
#define HS_BURST_BUF_SIZE      4000
#define HS_SAMPLE_INTERVAL_US  1000

/* Capture current on every HS sample (1 kHz); capture bus voltage on
 * every 10th (~100 Hz) — the I2C budget at 400 kHz can't carry all six
 * INA226 reads every 1 ms, and rail voltage is slow-moving anyway. */
#define HS_VOLTAGE_DECIMATE    10

/* After switching the sensors to fast mode, wait a couple of conversion
 * times so the first HS sample reflects the new mode, not a stale
 * steady-mode value. */
#define HS_MODE_SETTLE_MS      2

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
static cec_capture_hs_hook_fn_t   s_hs_setup_fn = NULL;
static cec_capture_hs_hook_fn_t   s_hs_teardown_fn = NULL;
static SemaphoreHandle_t s_trigger_sem = NULL;
static TaskHandle_t s_hs_task = NULL;

typedef enum {
    CAP_IDLE = 0,
    CAP_CAPTURING,   /* 4 s HS window, capture task hot-polling I2C */
    CAP_DUMPING,     /* post-capture TelePlot streaming over UART */
} cec_cap_phase_t;
static volatile cec_cap_phase_t s_phase = CAP_IDLE;
static volatile cec_trigger_t s_pending_reason = CEC_TRIG_NONE;
static volatile int64_t s_last_complete_us = 0;
static volatile bool s_inited = false;

#define ANNOTATION_MAX 96
static char s_pending_annotation[ANNOTATION_MAX] = {0};

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

bool cec_capture_is_busy(void)     { return s_phase != CAP_IDLE; }
bool cec_capture_is_capturing(void) { return s_phase == CAP_CAPTURING; }
bool cec_capture_is_dumping(void)   { return s_phase == CAP_DUMPING; }

esp_err_t cec_capture_trigger_with_text(cec_trigger_t reason, const char *text)
{
    if (!s_inited)           return ESP_ERR_INVALID_STATE;
    if (s_phase != CAP_IDLE) return ESP_ERR_NOT_FINISHED;

    int64_t now = esp_timer_get_time();
    bool bypass_cooldown = (reason == CEC_TRIG_SHUTDOWN);
    if (!bypass_cooldown && s_last_complete_us != 0
        && (now - s_last_complete_us) < BURST_COOLDOWN_US) {
        return ESP_ERR_INVALID_STATE;
    }

    s_phase = CAP_CAPTURING;
    s_pending_reason = reason;
    if (text != NULL && text[0] != '\0') {
        /* strncpy + explicit NUL — we own the buffer, so we don't care
         * about strncpy's "no-NUL on truncate" quirk so long as the
         * trailing byte is zeroed. */
        strncpy(s_pending_annotation, text, ANNOTATION_MAX - 1);
        s_pending_annotation[ANNOTATION_MAX - 1] = '\0';
    } else {
        s_pending_annotation[0] = '\0';
    }
    xSemaphoreGive(s_trigger_sem);
    return ESP_OK;
}

esp_err_t cec_capture_trigger(cec_trigger_t reason)
{
    return cec_capture_trigger_with_text(reason, NULL);
}

/* Spin until target_us, yielding to other Core 1 work between checks.
 * At our priority on a dedicated core there's almost nothing else to
 * run, but yielding keeps the watchdog happy and lets background tasks
 * (IDF event loops, etc.) breathe.
 *
 * Observed jitter: ~5% of HS samples land ~1 ms late under load (giving
 * 2 ms between rows instead of 1 ms). Most likely cause is cross-core
 * ADC oneshot lock contention with the main loop's reads on Core 0;
 * each missed window is roughly the length of one main-loop ADC read.
 * Acceptable for transient capture, problematic for contiguous waveform
 * analysis. The proper fix is ADC continuous mode (DMA), which is its
 * own README item. */
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
        teleplot_writef(">b_v_12v:%u:%.3f\n",  ts, p->v_12v);
        teleplot_writef(">b_i_12v:%u:%.3f\n",  ts, p->i_12v);
        teleplot_writef(">b_v_5v:%u:%.3f\n",   ts, p->v_5v);
        teleplot_writef(">b_i_5v:%u:%.3f\n",   ts, p->i_5v);
        teleplot_writef(">b_v_3v3:%u:%.3f\n",  ts, p->v_3v3);
        teleplot_writef(">b_i_3v3:%u:%.3f\n",  ts, p->i_3v3);
        teleplot_writef(">b_v_5vsb:%u:%.3f\n", ts, p->v_5vsb);
        teleplot_writef(">b_i_5vsb:%u:%.4f\n", ts, p->i_5vsb);
        teleplot_writef(">b_temp:%u:%.2f\n",   ts, p->temp_c);
        teleplot_writef(">b_state:%u:%d\n",    ts, (int)p->state);
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
        /* Current on every sample (1 kHz). */
        teleplot_writef(">hs_i_12v:%u:%.3f\n", ts, s->i_12v);
        teleplot_writef(">hs_i_5v:%u:%.3f\n",  ts, s->i_5v);
        teleplot_writef(">hs_i_3v3:%u:%.3f\n", ts, s->i_3v3);
        /* Voltage only on the decimated (~100 Hz) samples where it was
         * actually read; the rest carry no fresh v_* so we don't emit it. */
        if (i % HS_VOLTAGE_DECIMATE == 0) {
            teleplot_writef(">hs_v_12v:%u:%.3f\n", ts, s->v_12v);
            teleplot_writef(">hs_v_5v:%u:%.3f\n",  ts, s->v_5v);
            teleplot_writef(">hs_v_3v3:%u:%.3f\n", ts, s->v_3v3);
        }
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

        /* Switch sensors to fast conversion mode for the burst, then let
         * the first conversions settle before sampling. */
        if (s_hs_setup_fn != NULL) {
            s_hs_setup_fn();
            vTaskDelay(pdMS_TO_TICKS(HS_MODE_SETTLE_MS));
        }

        int64_t hs_start_us = esp_timer_get_time();
        int64_t target_us = hs_start_us;
        int slots_with_carry = 0;
        for (int i = 0; i < HS_BURST_BUF_SIZE; i++) {
            target_us += HS_SAMPLE_INTERVAL_US;
            spin_until(target_us);
            cec_capture_hs_sample_t *s = &s_hs_buf[i];
            s->ts_us_offset = (uint32_t)(esp_timer_get_time() - hs_start_us);
            bool want_voltage = (i % HS_VOLTAGE_DECIMATE == 0);
            uint32_t ok = s_hs_fn(s, want_voltage);

            /* Per-rail carry-forward. The fields the callback was expected
             * to fill this iteration are CEC_HS_OK_ALL_I always plus
             * CEC_HS_OK_ALL_V on decimated samples. Any expected field not
             * marked OK gets the previous sample's value; healthy rails
             * are untouched, so one device's NACK can no longer void the
             * other two. */
            if (i > 0) {
                uint32_t expected = CEC_HS_OK_ALL_I | (want_voltage ? CEC_HS_OK_ALL_V : 0u);
                uint32_t missing  = expected & ~ok;
                if (missing) {
                    const cec_capture_hs_sample_t *prev = &s_hs_buf[i - 1];
                    if (missing & CEC_HS_OK_I_12V) s->i_12v = prev->i_12v;
                    if (missing & CEC_HS_OK_V_12V) s->v_12v = prev->v_12v;
                    if (missing & CEC_HS_OK_I_5V)  s->i_5v  = prev->i_5v;
                    if (missing & CEC_HS_OK_V_5V)  s->v_5v  = prev->v_5v;
                    if (missing & CEC_HS_OK_I_3V3) s->i_3v3 = prev->i_3v3;
                    if (missing & CEC_HS_OK_V_3V3) s->v_3v3 = prev->v_3v3;
                    slots_with_carry++;
                }
            }
        }
        int64_t hs_end_us = esp_timer_get_time();

        /* Restore steady conversion mode. After this returns the I2C bus
         * is idle until the next iteration of this task — so we drop the
         * capturing flag and let the main loop resume its own reads even
         * though the dump is about to take ~5 s over the UART. */
        if (s_hs_teardown_fn != NULL) {
            s_hs_teardown_fn();
        }
        s_phase = CAP_DUMPING;

        ESP_LOGI(TAG, "HS capture done in %lld us (slots with carry-forward: %d/%d), "
                      "dumping pre+HS to TelePlot",
                 (long long)(hs_end_us - hs_start_us),
                 slots_with_carry, HS_BURST_BUF_SIZE);

        teleplot_writef(">BURST_BEGIN:%s:%d_normal+%d_hs:%d\n",
                        cec_trigger_name(reason),
                        s_pre_count, HS_BURST_BUF_SIZE,
                        (int)state_at_trigger);
        if (s_pending_annotation[0] != '\0') {
            teleplot_writef(">BURST_ANNOTATION:%s\n", s_pending_annotation);
            s_pending_annotation[0] = '\0';
        }
        dump_pretrigger(state_at_trigger);
        dump_hs(hs_start_us);
        teleplot_writef(">BURST_END\n");

        s_last_complete_us = esp_timer_get_time();
        s_phase = CAP_IDLE;
        ESP_LOGI(TAG, "burst complete");
    }
}

esp_err_t cec_capture_init(const cec_capture_config_t *cfg)
{
    if (s_inited) return ESP_OK;
    if (cfg == NULL || cfg->sample_fn == NULL) return ESP_ERR_INVALID_ARG;

    s_hs_fn = cfg->sample_fn;
    s_hs_setup_fn = cfg->setup_fn;
    s_hs_teardown_fn = cfg->teardown_fn;
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

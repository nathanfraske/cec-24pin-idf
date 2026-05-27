/*
 * CEC 24-pin Module Firmware - ESP-IDF port
 *
 * Brings up the I2C bus + INA226 (5VSB), the ADC1 voltage-rail divider
 * reads (12V/5V/3V3), the ACS712 current sensors on the same three rails,
 * and the NTC thermistor, all matching the v0.5.9 hardware configuration.
 * Samples everything at 50 Hz, runs each channel through a fast EMA, and
 * emits TelePlot series at 10 Hz with a 1 Hz INFO summary line.
 *
 * Compare against v0.5.9 captures on the same hardware. Numbers should
 * match within measurement noise once per-unit trim is dialed in.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

#include "ina226.h"
#include "cec_adc.h"
#include "thermistor.h"
#include "acs712.h"
#include "cec_filters.h"
#include "cec_state.h"
#include "cec_layer1.h"
#include "cec_capture.h"
#include "cec_teleplot.h"

static const char *TAG = "cec_main";

/* I2C bus pins, matching v0.5.9 wiring on the Lonely Binary ESP32-S3 N16R8 */
#define I2C_PIN_SDA       8
#define I2C_PIN_SCL       9
#define I2C_PORT_NUM      I2C_NUM_0

/* ADC1 channels (GPIO1..7 = ADC1_CH0..6), pin map from v0.5.9 */
#define ADC_CH_V_12V      ADC_CHANNEL_0   /* GPIO1 */
#define ADC_CH_V_5V       ADC_CHANNEL_1   /* GPIO2 */
#define ADC_CH_V_3V3      ADC_CHANNEL_2   /* GPIO3 */
#define ADC_CH_I_12V      ADC_CHANNEL_3   /* GPIO4 */
#define ADC_CH_I_5V       ADC_CHANNEL_4   /* GPIO5 */
#define ADC_CH_I_3V3      ADC_CHANNEL_5   /* GPIO6 */
#define ADC_CH_NTC        ADC_CHANNEL_6   /* GPIO7 */

/* Per-rail trim factors carried forward from v0.5.9 (hardware-specific) */
#define TRIM_12V          1.0000f
#define TRIM_5V           0.9962f
#define TRIM_3V3          0.9915f
#define TRIM_5VSB         0.9901f

/* Hardware voltage dividers (top + bottom of resistor stack), v0.5.9 */
#define SCALE_12V         ((47000.0f + 10000.0f) / 10000.0f)
#define SCALE_5V          ((15000.0f + 10000.0f) / 10000.0f)
#define SCALE_3V3         (( 4700.0f + 10000.0f) / 10000.0f)

/* ACS712 divider (between sensor output and ADC pin) and per-part
 * sensitivities, from v0.5.9. The 30A part sits on the 12V rail; both
 * 20A parts sit on the 5V/3V3 rails. */
#define ACS712_DIVIDER       ((20000.0f + 30000.0f) / 30000.0f)
#define ACS712_30A_SENS      0.066f
#define ACS712_20A_SENS      0.100f
/* Per-rail no-load output (post-divider voltage). Nominal Vcc/2 = 2.20 V
 * but the part-to-part variation is several tens of mV (hundreds of mA
 * at these sensitivities), so each rail gets its own constant for hand-
 * tuning. The boot-time diagnostic logs the measured no-load output —
 * with the PSU disconnected at first boot, copy those values here. Full
 * runtime calibration lands with the serial-command + NVS path. */
#define ACS712_ZERO_12V      2.20f
#define ACS712_ZERO_5V       2.20f
#define ACS712_ZERO_3V3      2.20f

/* Loop cadence. Sample at 50 Hz to match v0.5.9; emit TelePlot at 10 Hz
 * (every 5th iteration); log an INFO summary at 1 Hz (every 50th). */
#define SAMPLE_PERIOD_MS  20
#define TELEPLOT_DIVIDER  5
#define LOG_DIVIDER       50

/* EMA smoothing. At 50 Hz, alpha=0.02 gives a ~1 s time constant, matching
 * the v0.5.9 EMA_ALPHA_FAST. */
#define EMA_ALPHA_FAST    0.02f

/* I2C bus handle (shared across components later) */
static i2c_master_bus_handle_t s_i2c_bus = NULL;

/* INA226 instances. For now only 5VSB. Will add 12V/5V/3V3 after PCB rev. */
static ina226_handle_t s_ina226_5vsb = NULL;

/* ADC rail configs. At 50 Hz the per-iteration ADC time budget is tight,
 * so samples-per-read is dropped from 8 to 4 vs. the 5 Hz era. Calibration
 * curve-fitting handles the rest. */
static const cec_adc_rail_t s_rail_12v = {
    .channel = ADC_CH_V_12V, .samples = 4, .scale = SCALE_12V, .trim = TRIM_12V,
};
static const cec_adc_rail_t s_rail_5v = {
    .channel = ADC_CH_V_5V,  .samples = 4, .scale = SCALE_5V,  .trim = TRIM_5V,
};
static const cec_adc_rail_t s_rail_3v3 = {
    .channel = ADC_CH_V_3V3, .samples = 4, .scale = SCALE_3V3, .trim = TRIM_3V3,
};

/* NTC config carried forward from v0.5.9. Standard 10k @ 25C, B=3950. */
static const thermistor_t s_ntc = {
    .channel = ADC_CH_NTC,
    .samples = 4,
    .beta = 3950.0f,
    .nominal_resistance = 10000.0f,
    .nominal_temperature_k = 298.15f,
    .pull_up_resistance = 10000.0f,
    .vcc = 3.3f,
};

/* ACS712 sensor configs. zero_point_v is the no-load nominal until per-
 * unit calibration lands; expect currents to be a few hundred mA off
 * until that path is wired in. */
static const acs712_t s_acs_12v = {
    .channel = ADC_CH_I_12V, .samples = 4,
    .divider_scale = ACS712_DIVIDER,
    .sensitivity_v_per_a = ACS712_30A_SENS,
    .zero_point_v = ACS712_ZERO_12V,
};
static const acs712_t s_acs_5v = {
    .channel = ADC_CH_I_5V,  .samples = 4,
    .divider_scale = ACS712_DIVIDER,
    .sensitivity_v_per_a = ACS712_20A_SENS,
    .zero_point_v = ACS712_ZERO_5V,
};
static const acs712_t s_acs_3v3 = {
    .channel = ADC_CH_I_3V3, .samples = 4,
    .divider_scale = ACS712_DIVIDER,
    .sensitivity_v_per_a = ACS712_20A_SENS,
    .zero_point_v = ACS712_ZERO_3V3,
};

/* HS-rate configs (samples=1) used by the burst capture engine to keep
 * each 1 kHz iteration well under the 1 ms budget. Scale/trim/zero match
 * the main-loop configs above. */
static const cec_adc_rail_t s_hs_rail_12v = {
    .channel = ADC_CH_V_12V, .samples = 1, .scale = SCALE_12V, .trim = TRIM_12V,
};
static const cec_adc_rail_t s_hs_rail_5v = {
    .channel = ADC_CH_V_5V,  .samples = 1, .scale = SCALE_5V,  .trim = TRIM_5V,
};
static const cec_adc_rail_t s_hs_rail_3v3 = {
    .channel = ADC_CH_V_3V3, .samples = 1, .scale = SCALE_3V3, .trim = TRIM_3V3,
};
static const acs712_t s_hs_acs_12v = {
    .channel = ADC_CH_I_12V, .samples = 1,
    .divider_scale = ACS712_DIVIDER,
    .sensitivity_v_per_a = ACS712_30A_SENS,
    .zero_point_v = ACS712_ZERO_12V,
};
static const acs712_t s_hs_acs_5v = {
    .channel = ADC_CH_I_5V,  .samples = 1,
    .divider_scale = ACS712_DIVIDER,
    .sensitivity_v_per_a = ACS712_20A_SENS,
    .zero_point_v = ACS712_ZERO_5V,
};
static const acs712_t s_hs_acs_3v3 = {
    .channel = ADC_CH_I_3V3, .samples = 1,
    .divider_scale = ACS712_DIVIDER,
    .sensitivity_v_per_a = ACS712_20A_SENS,
    .zero_point_v = ACS712_ZERO_3V3,
};

/* Filter state */
static ema_t s_v_5vsb_ema, s_i_5vsb_ema;
static ema_t s_v_12v_ema, s_v_5v_ema, s_v_3v3_ema;
static ema_t s_i_12v_ema, s_i_5v_ema, s_i_3v3_ema;
static ema_t s_temp_ema;

/* State machine */
static cec_state_t s_state = CEC_STATE_OFF;
static int64_t s_state_entered_us = 0;

/* Layer 1 static-threshold detectors per rail. Bands carry forward from
 * v0.5.9: 5%/10% on main rails, 10%/20% on 5VSB. */
#define L1_CRIT_CONSECUTIVE 3
static cec_layer1_detector_t s_l1_12v, s_l1_5v, s_l1_3v3, s_l1_5vsb;
static cec_severity_t s_last_sev_12v  = CEC_SEV_NONE;
static cec_severity_t s_last_sev_5v   = CEC_SEV_NONE;
static cec_severity_t s_last_sev_3v3  = CEC_SEV_NONE;
static cec_severity_t s_last_sev_5vsb = CEC_SEV_NONE;

static void init_i2c_bus(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .scl_io_num = I2C_PIN_SCL,
        .sda_io_num = I2C_PIN_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
    ESP_LOGI(TAG, "I2C bus: SDA=GPIO%d, SCL=GPIO%d", I2C_PIN_SDA, I2C_PIN_SCL);
}

static esp_err_t init_adc_rails(void)
{
    ESP_RETURN_ON_ERROR(cec_adc_init(), TAG, "cec_adc_init");
    ESP_RETURN_ON_ERROR(cec_adc_setup_channel(ADC_CH_V_12V), TAG, "setup v_12V");
    ESP_RETURN_ON_ERROR(cec_adc_setup_channel(ADC_CH_V_5V),  TAG, "setup v_5V");
    ESP_RETURN_ON_ERROR(cec_adc_setup_channel(ADC_CH_V_3V3), TAG, "setup v_3V3");
    ESP_RETURN_ON_ERROR(acs712_setup(&s_acs_12v),            TAG, "setup i_12V");
    ESP_RETURN_ON_ERROR(acs712_setup(&s_acs_5v),             TAG, "setup i_5V");
    ESP_RETURN_ON_ERROR(acs712_setup(&s_acs_3v3),            TAG, "setup i_3V3");
    ESP_RETURN_ON_ERROR(thermistor_setup(&s_ntc),            TAG, "setup NTC");
    return ESP_OK;
}

static esp_err_t init_ina226_5vsb(void)
{
    ina226_config_t cfg = INA226_CONFIG_DEFAULT();
    cfg.bus_handle = s_i2c_bus;
    cfg.i2c_addr = 0x40;          /* INA226 default A0=A1=GND */
    cfg.shunt_ohms = 0.002f;      /* R002 marking on the physical module */
    cfg.max_current_a = 5.0f;     /* Matches v0.5.9, gives CAL=0x418A */
    cfg.voltage_trim = TRIM_5VSB; /* Match v0.5.9 calibration */
    cfg.current_trim = 1.0f;      /* No current trim on 5VSB */

    return ina226_create(&cfg, &s_ina226_5vsb);
}

static void init_layer1(void)
{
    static const cec_rail_spec_t SPEC_12V  = { 12.0f, 0.05f, 0.10f };
    static const cec_rail_spec_t SPEC_5V   = {  5.0f, 0.05f, 0.10f };
    static const cec_rail_spec_t SPEC_3V3  = {  3.3f, 0.05f, 0.10f };
    static const cec_rail_spec_t SPEC_5VSB = {  5.0f, 0.10f, 0.20f };
    cec_layer1_init(&s_l1_12v,  &SPEC_12V,  L1_CRIT_CONSECUTIVE);
    cec_layer1_init(&s_l1_5v,   &SPEC_5V,   L1_CRIT_CONSECUTIVE);
    cec_layer1_init(&s_l1_3v3,  &SPEC_3V3,  L1_CRIT_CONSECUTIVE);
    cec_layer1_init(&s_l1_5vsb, &SPEC_5VSB, L1_CRIT_CONSECUTIVE);
}

typedef struct {
    cec_severity_t sev;
    bool entered_critical;   /* true on the iteration where sev steps to CRITICAL */
} layer1_step_result_t;

/* Update one rail's Layer 1 detector, log on severity transitions, and
 * report whether this is the iteration on which the rail just stepped
 * into CRITICAL — the caller uses that to fire a burst trigger exactly
 * once per fault entry. */
static layer1_step_result_t layer1_step(const char *rail_name,
                                        cec_layer1_detector_t *d,
                                        cec_severity_t *last_sev,
                                        float v_rail)
{
    cec_severity_t prev = *last_sev;
    cec_severity_t sev = cec_layer1_update(d, v_rail);
    if (sev != prev) {
        if (sev != CEC_SEV_NONE) {
            ESP_LOGW(TAG, "L1: %s %s (v=%.3f, nominal=%.2f)",
                     rail_name, cec_severity_name(sev), v_rail, d->spec.nominal);
        } else {
            ESP_LOGI(TAG, "L1: %s recovered (v=%.3f)", rail_name, v_rail);
        }
        *last_sev = sev;
    }
    return (layer1_step_result_t){
        .sev = sev,
        .entered_critical = (prev != CEC_SEV_CRITICAL && sev == CEC_SEV_CRITICAL),
    };
}

/* Sample each ACS712's no-load output and log what it reads, along with
 * the equivalent offset current vs. the constants currently compiled in.
 * Run once at boot before the main loop starts feeding the sensors.
 * Lets you copy the measured no-load voltages directly into
 * ACS712_ZERO_{12V,5V,3V3} for a per-unit-tuned offset until the proper
 * serial-command / NVS calibration path lands. */
static void log_acs712_zero_measurements(void)
{
    struct {
        const char *name;
        const acs712_t *cfg;
        float compiled_zero;
        float sens;
    } rails[] = {
        { "12V", &s_acs_12v, ACS712_ZERO_12V, ACS712_30A_SENS },
        { "5V",  &s_acs_5v,  ACS712_ZERO_5V,  ACS712_20A_SENS },
        { "3V3", &s_acs_3v3, ACS712_ZERO_3V3, ACS712_20A_SENS },
    };
    ESP_LOGI(TAG, "ACS712 no-load diagnostic (200-sample average):");
    for (size_t i = 0; i < sizeof(rails) / sizeof(rails[0]); i++) {
        float measured = 0.0f;
        esp_err_t err = acs712_measure_zero_point(rails[i].cfg, 200, &measured);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "  %s: measure failed (%s)", rails[i].name, esp_err_to_name(err));
            continue;
        }
        float implied_a = (measured - rails[i].compiled_zero) / rails[i].sens;
        ESP_LOGI(TAG, "  %s: measured=%.4f V, compiled=%.4f V "
                      "=> implied no-load current = %+.3f A",
                 rails[i].name, measured, rails[i].compiled_zero, implied_a);
    }
    ESP_LOGI(TAG, "If \"implied no-load current\" is non-zero with PSU "
                  "disconnected, paste the measured V into ACS712_ZERO_<rail>.");
}

/* Burst capture HS sample callback. Runs at 1 kHz on the cec_hs_cap task
 * (Core 1) — must stay well under 1 ms. Six oneshot ADC reads + scaling
 * comfortably fit. */
static void hs_sample_fn(cec_capture_hs_sample_t *out)
{
    cec_adc_read(&s_hs_rail_12v, &out->v_12v);
    cec_adc_read(&s_hs_rail_5v,  &out->v_5v);
    cec_adc_read(&s_hs_rail_3v3, &out->v_3v3);
    acs712_read_amps(&s_hs_acs_12v, &out->i_12v);
    acs712_read_amps(&s_hs_acs_5v,  &out->i_5v);
    acs712_read_amps(&s_hs_acs_3v3, &out->i_3v3);
}

static void log_hardware_info(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: %s rev v%d.%d, %d core(s)",
             CONFIG_IDF_TARGET,
             chip_info.revision / 100, chip_info.revision % 100,
             chip_info.cores);

    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Flash: %" PRIu32 " MB", flash_size / (1024 * 1024));

    if (esp_psram_is_initialized()) {
        size_t psram_size = esp_psram_get_size();
        ESP_LOGI(TAG, "PSRAM: %u MB", psram_size / (1024 * 1024));
    } else {
        ESP_LOGW(TAG, "PSRAM: not initialized");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "CEC 24-pin Module Firmware (ESP-IDF port)");
    ESP_LOGI(TAG, "Version: 0.6.0-dev (INA226 bringup)");
    ESP_LOGI(TAG, "===========================================");

    log_hardware_info();

    init_i2c_bus();

    esp_err_t err = init_ina226_5vsb();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INA226 5VSB init failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Continuing without INA226 readings");
    }

    err = init_adc_rails();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC rail init failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Continuing without ADC rail readings");
    }

    ema_init(&s_v_5vsb_ema, EMA_ALPHA_FAST);
    ema_init(&s_i_5vsb_ema, EMA_ALPHA_FAST);
    ema_init(&s_v_12v_ema,  EMA_ALPHA_FAST);
    ema_init(&s_v_5v_ema,   EMA_ALPHA_FAST);
    ema_init(&s_v_3v3_ema,  EMA_ALPHA_FAST);
    ema_init(&s_i_12v_ema,  EMA_ALPHA_FAST);
    ema_init(&s_i_5v_ema,   EMA_ALPHA_FAST);
    ema_init(&s_i_3v3_ema,  EMA_ALPHA_FAST);
    ema_init(&s_temp_ema,   EMA_ALPHA_FAST);

    init_layer1();

    log_acs712_zero_measurements();

    err = cec_capture_init(hs_sample_fn);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cec_capture_init failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Continuing without burst capture");
    }

    /* Sample at 50 Hz, emit TelePlot at 10 Hz, log an INFO summary at 1 Hz.
     * Read failures don't get spammed per-iteration; the divided cadence
     * makes them visible as gaps in TelePlot and bad values in the summary. */
    ESP_LOGI(TAG, "Entering main loop (sample=50 Hz, teleplot=10 Hz, log=1 Hz)");
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t iter = 0;
    while (1) {
        float v_5vsb = 0.0f, i_5vsb = 0.0f;
        float v_12v = 0.0f, v_5v = 0.0f, v_3v3 = 0.0f;
        float i_12v = 0.0f, i_5v = 0.0f, i_3v3 = 0.0f;
        float temp_c = 0.0f;
        bool ok_5vsb = false, ok_temp = false;
        bool ok_v_12v = false, ok_v_5v = false, ok_v_3v3 = false;
        bool ok_i_12v = false, ok_i_5v = false, ok_i_3v3 = false;

        if (s_ina226_5vsb != NULL) {
            ok_5vsb = (ina226_read_bus_voltage(s_ina226_5vsb, &v_5vsb) == ESP_OK &&
                       ina226_read_current(s_ina226_5vsb, &i_5vsb) == ESP_OK);
        }
        ok_v_12v = (cec_adc_read(&s_rail_12v, &v_12v) == ESP_OK);
        ok_v_5v  = (cec_adc_read(&s_rail_5v,  &v_5v)  == ESP_OK);
        ok_v_3v3 = (cec_adc_read(&s_rail_3v3, &v_3v3) == ESP_OK);
        ok_i_12v = (acs712_read_amps(&s_acs_12v, &i_12v) == ESP_OK);
        ok_i_5v  = (acs712_read_amps(&s_acs_5v,  &i_5v)  == ESP_OK);
        ok_i_3v3 = (acs712_read_amps(&s_acs_3v3, &i_3v3) == ESP_OK);
        ok_temp  = (thermistor_read_celsius(&s_ntc, &temp_c) == ESP_OK);

        /* On a failed read, fall back to the last good filtered value so a
         * single bad sample doesn't ripple into the state classifier. Before
         * the first successful read an EMA returns 0.0 (its init value). */
        float v_5vsb_ema = ok_5vsb  ? ema_update(&s_v_5vsb_ema, v_5vsb) : ema_value(&s_v_5vsb_ema);
        float i_5vsb_ema = ok_5vsb  ? ema_update(&s_i_5vsb_ema, i_5vsb) : ema_value(&s_i_5vsb_ema);
        float v_12v_ema  = ok_v_12v ? ema_update(&s_v_12v_ema,  v_12v)  : ema_value(&s_v_12v_ema);
        float v_5v_ema   = ok_v_5v  ? ema_update(&s_v_5v_ema,   v_5v)   : ema_value(&s_v_5v_ema);
        float v_3v3_ema  = ok_v_3v3 ? ema_update(&s_v_3v3_ema,  v_3v3)  : ema_value(&s_v_3v3_ema);
        float i_12v_ema  = ok_i_12v ? ema_update(&s_i_12v_ema,  i_12v)  : ema_value(&s_i_12v_ema);
        float i_5v_ema   = ok_i_5v  ? ema_update(&s_i_5v_ema,   i_5v)   : ema_value(&s_i_5v_ema);
        float i_3v3_ema  = ok_i_3v3 ? ema_update(&s_i_3v3_ema,  i_3v3)  : ema_value(&s_i_3v3_ema);
        float temp_ema   = ok_temp  ? ema_update(&s_temp_ema,   temp_c) : ema_value(&s_temp_ema);

        /* Total main-rail power, EMA-smoothed. 5VSB is standby and not
         * included by design (matches v0.5.9). */
        float p_total = (v_12v_ema * i_12v_ema)
                      + (v_5v_ema  * i_5v_ema)
                      + (v_3v3_ema * i_3v3_ema);

        cec_state_t next_state = cec_state_classify(v_12v_ema, v_5vsb_ema, p_total, s_state);
        if (next_state != s_state) {
            int64_t now_us = esp_timer_get_time();
            int64_t dwell_ms = (now_us - s_state_entered_us) / 1000;
            ESP_LOGI(TAG, "state: %s -> %s (dwell=%lld ms, p_total=%.1f W)",
                     cec_state_name(s_state), cec_state_name(next_state),
                     (long long)dwell_ms, p_total);
            s_state = next_state;
            s_state_entered_us = now_us;
        }

        /* Layer 1 main rails: only check in IDLE/ACTIVE/PEAK. During
         * OFF/STANDBY the rails are 0 V or ramping, and the brief
         * mid-range values during power-up would otherwise sustain a
         * spurious CRITICAL for several iterations. 5VSB is on whenever
         * the PSU is plugged in, so its detector runs unconditionally
         * and relies on the RAIL_OFF check to handle the actually-off
         * state. */
        cec_severity_t sev_12v = CEC_SEV_NONE;
        cec_severity_t sev_5v  = CEC_SEV_NONE;
        cec_severity_t sev_3v3 = CEC_SEV_NONE;
        bool any_entered_crit = false;
        if (s_state == CEC_STATE_IDLE || s_state == CEC_STATE_ACTIVE || s_state == CEC_STATE_PEAK) {
            layer1_step_result_t r;
            r = layer1_step("12V", &s_l1_12v, &s_last_sev_12v, v_12v_ema);
            sev_12v = r.sev; any_entered_crit |= r.entered_critical;
            r = layer1_step("5V",  &s_l1_5v,  &s_last_sev_5v,  v_5v_ema);
            sev_5v  = r.sev; any_entered_crit |= r.entered_critical;
            r = layer1_step("3V3", &s_l1_3v3, &s_last_sev_3v3, v_3v3_ema);
            sev_3v3 = r.sev; any_entered_crit |= r.entered_critical;
        } else {
            cec_layer1_reset(&s_l1_12v); s_last_sev_12v = CEC_SEV_NONE;
            cec_layer1_reset(&s_l1_5v);  s_last_sev_5v  = CEC_SEV_NONE;
            cec_layer1_reset(&s_l1_3v3); s_last_sev_3v3 = CEC_SEV_NONE;
        }
        layer1_step_result_t r5sb = layer1_step("5VSB", &s_l1_5vsb, &s_last_sev_5vsb, v_5vsb_ema);
        cec_severity_t sev_5vsb = r5sb.sev;
        any_entered_crit |= r5sb.entered_critical;

        if (any_entered_crit) {
            esp_err_t terr = cec_capture_trigger(CEC_TRIG_STATIC_CRIT);
            if (terr == ESP_OK) {
                ESP_LOGW(TAG, "burst trigger: STATIC_CRIT");
            } else if (terr == ESP_ERR_NOT_FINISHED) {
                ESP_LOGW(TAG, "burst trigger: STATIC_CRIT skipped (capture busy)");
            } else if (terr == ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "burst trigger: STATIC_CRIT skipped (cooldown)");
            }
        }

        /* Push a pre-trigger sample every iteration so the ring buffer
         * always holds the last ~20 s of filtered telemetry. */
        cec_capture_sample_t pre = {
            .ts_ms  = (uint32_t)(esp_timer_get_time() / 1000),
            .state  = (uint8_t)s_state,
            .v_12v  = v_12v_ema,  .i_12v  = i_12v_ema,
            .v_5v   = v_5v_ema,   .i_5v   = i_5v_ema,
            .v_3v3  = v_3v3_ema,  .i_3v3  = i_3v3_ema,
            .v_5vsb = v_5vsb_ema, .i_5vsb = i_5vsb_ema,
            .temp_c = temp_ema,
        };
        cec_capture_push(&pre);

        if (iter % TELEPLOT_DIVIDER == 0) {
            /* Use the device-side millisecond clock for every TelePlot
             * row so the slow-loop output shares a time base with the
             * burst-capture b_*hs_* streams. Without this TelePlot
             * auto-stamps with the host wallclock and the two streams
             * can't be aligned. */
            int64_t now_ms = esp_timer_get_time() / 1000;
            if (ok_5vsb) {
                teleplot_emit_t("v_5vsb",     now_ms, v_5vsb);
                teleplot_emit_t("v_5vsb_ema", now_ms, v_5vsb_ema);
                teleplot_emit_t("i_5vsb_raw", now_ms, i_5vsb);
                teleplot_emit_t("i_5vsb_ema", now_ms, i_5vsb_ema);
            }
            if (ok_v_12v) { teleplot_emit_t("v_12v", now_ms, v_12v); teleplot_emit_t("v_12v_ema", now_ms, v_12v_ema); }
            if (ok_v_5v)  { teleplot_emit_t("v_5v",  now_ms, v_5v);  teleplot_emit_t("v_5v_ema",  now_ms, v_5v_ema);  }
            if (ok_v_3v3) { teleplot_emit_t("v_3v3", now_ms, v_3v3); teleplot_emit_t("v_3v3_ema", now_ms, v_3v3_ema); }
            if (ok_i_12v) { teleplot_emit_t("i_12v", now_ms, i_12v); teleplot_emit_t("i_12v_ema", now_ms, i_12v_ema); }
            if (ok_i_5v)  { teleplot_emit_t("i_5v",  now_ms, i_5v);  teleplot_emit_t("i_5v_ema",  now_ms, i_5v_ema);  }
            if (ok_i_3v3) { teleplot_emit_t("i_3v3", now_ms, i_3v3); teleplot_emit_t("i_3v3_ema", now_ms, i_3v3_ema); }
            if (ok_temp)  { teleplot_emit_t("temp_c", now_ms, temp_c); teleplot_emit_t("temp_c_ema", now_ms, temp_ema); }
            teleplot_emit_t("p_total", now_ms, p_total);
            teleplot_emit_t("state",   now_ms, (float)s_state);
            teleplot_emit_t("sev_12v",  now_ms, (float)sev_12v);
            teleplot_emit_t("sev_5v",   now_ms, (float)sev_5v);
            teleplot_emit_t("sev_3v3",  now_ms, (float)sev_3v3);
            teleplot_emit_t("sev_5vsb", now_ms, (float)sev_5vsb);
        }

        if (iter % LOG_DIVIDER == 0) {
            ESP_LOGI(TAG, "[%s] V: 12=%.3f 5=%.3f 3V3=%.3f 5SB=%.3f | "
                          "I: 12=%.2f 5=%.2f 3V3=%.2f 5SB=%.4f | "
                          "P=%.1fW T=%.1fC",
                     cec_state_name(s_state),
                     v_12v_ema, v_5v_ema, v_3v3_ema, v_5vsb_ema,
                     i_12v_ema, i_5v_ema, i_3v3_ema, i_5vsb_ema,
                     p_total, temp_ema);
        }

        iter++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

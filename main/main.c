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
/* No-load output at the post-divider voltage. Nominal Vcc/2 ~ 2.20 V;
 * needs per-unit calibration once the serial-command path lands. */
#define ACS712_ZERO_DEFAULT  2.20f

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
    .zero_point_v = ACS712_ZERO_DEFAULT,
};
static const acs712_t s_acs_5v = {
    .channel = ADC_CH_I_5V,  .samples = 4,
    .divider_scale = ACS712_DIVIDER,
    .sensitivity_v_per_a = ACS712_20A_SENS,
    .zero_point_v = ACS712_ZERO_DEFAULT,
};
static const acs712_t s_acs_3v3 = {
    .channel = ADC_CH_I_3V3, .samples = 4,
    .divider_scale = ACS712_DIVIDER,
    .sensitivity_v_per_a = ACS712_20A_SENS,
    .zero_point_v = ACS712_ZERO_DEFAULT,
};

/* Filter state */
static ema_t s_i_5vsb_ema;
static ema_t s_v_12v_ema, s_v_5v_ema, s_v_3v3_ema;
static ema_t s_i_12v_ema, s_i_5v_ema, s_i_3v3_ema;
static ema_t s_temp_ema;

/* State machine */
static cec_state_t s_state = CEC_STATE_OFF;
static int64_t s_state_entered_us = 0;

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

    ema_init(&s_i_5vsb_ema, EMA_ALPHA_FAST);
    ema_init(&s_v_12v_ema,  EMA_ALPHA_FAST);
    ema_init(&s_v_5v_ema,   EMA_ALPHA_FAST);
    ema_init(&s_v_3v3_ema,  EMA_ALPHA_FAST);
    ema_init(&s_i_12v_ema,  EMA_ALPHA_FAST);
    ema_init(&s_i_5v_ema,   EMA_ALPHA_FAST);
    ema_init(&s_i_3v3_ema,  EMA_ALPHA_FAST);
    ema_init(&s_temp_ema,   EMA_ALPHA_FAST);

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

        cec_state_t next_state = cec_state_classify(v_12v_ema, p_total, s_state);
        if (next_state != s_state) {
            int64_t now_us = esp_timer_get_time();
            int64_t dwell_ms = (now_us - s_state_entered_us) / 1000;
            ESP_LOGI(TAG, "state: %s -> %s (dwell=%lld ms, p_total=%.1f W)",
                     cec_state_name(s_state), cec_state_name(next_state),
                     (long long)dwell_ms, p_total);
            s_state = next_state;
            s_state_entered_us = now_us;
        }

        if (iter % TELEPLOT_DIVIDER == 0) {
            if (ok_5vsb) {
                teleplot_emit("v_5vsb",     v_5vsb);
                teleplot_emit("i_5vsb_raw", i_5vsb);
                teleplot_emit("i_5vsb_ema", i_5vsb_ema);
            }
            if (ok_v_12v) { teleplot_emit("v_12v", v_12v); teleplot_emit("v_12v_ema", v_12v_ema); }
            if (ok_v_5v)  { teleplot_emit("v_5v",  v_5v);  teleplot_emit("v_5v_ema",  v_5v_ema);  }
            if (ok_v_3v3) { teleplot_emit("v_3v3", v_3v3); teleplot_emit("v_3v3_ema", v_3v3_ema); }
            if (ok_i_12v) { teleplot_emit("i_12v", i_12v); teleplot_emit("i_12v_ema", i_12v_ema); }
            if (ok_i_5v)  { teleplot_emit("i_5v",  i_5v);  teleplot_emit("i_5v_ema",  i_5v_ema);  }
            if (ok_i_3v3) { teleplot_emit("i_3v3", i_3v3); teleplot_emit("i_3v3_ema", i_3v3_ema); }
            if (ok_temp)  { teleplot_emit("temp_c", temp_c); teleplot_emit("temp_c_ema", temp_ema); }
            teleplot_emit("p_total", p_total);
            teleplot_emit("state",   (float)s_state);
        }

        if (iter % LOG_DIVIDER == 0) {
            ESP_LOGI(TAG, "[%s] V: 12=%.3f 5=%.3f 3V3=%.3f 5SB=%.3f | "
                          "I: 12=%.2f 5=%.2f 3V3=%.2f 5SB=%.4f | "
                          "P=%.1fW T=%.1fC",
                     cec_state_name(s_state),
                     v_12v_ema, v_5v_ema, v_3v3_ema, v_5vsb,
                     i_12v_ema, i_5v_ema, i_3v3_ema, i_5vsb_ema,
                     p_total, temp_ema);
        }

        iter++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

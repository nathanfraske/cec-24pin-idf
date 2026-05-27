/*
 * CEC 24-pin Module Firmware - ESP-IDF port
 *
 * Status: Bootstrap + INA226 driver validation
 *
 * Brings up the I2C bus and the INA226 on the 5VSB rail, matching the
 * v0.5.9 configuration (0.002 ohm shunt, 5 A max current, 0.9901 voltage
 * trim). Reads bus voltage and current at 5 Hz and logs to serial.
 *
 * Compare against v0.5.9 captures on the same hardware. Numbers should
 * match within measurement noise.
 */

#include <stdio.h>
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
#include "cec_filters.h"
#include "cec_teleplot.h"

static const char *TAG = "cec_main";

/* I2C bus pins, matching v0.5.9 wiring on the Lonely Binary ESP32-S3 N16R8 */
#define I2C_PIN_SDA       8
#define I2C_PIN_SCL       9
#define I2C_PORT_NUM      I2C_NUM_0

/* ADC1 channels for the voltage rails (GPIO1/2/3 = ADC1_CH0/1/2) */
#define ADC_CH_V_12V      ADC_CHANNEL_0
#define ADC_CH_V_5V       ADC_CHANNEL_1
#define ADC_CH_V_3V3      ADC_CHANNEL_2

/* Per-rail trim factors carried forward from v0.5.9 (hardware-specific) */
#define TRIM_12V          1.0000f
#define TRIM_5V           0.9962f
#define TRIM_3V3          0.9915f
#define TRIM_5VSB         0.9901f

/* Hardware voltage dividers (top + bottom of resistor stack), v0.5.9 */
#define SCALE_12V         ((47000.0f + 10000.0f) / 10000.0f)
#define SCALE_5V          ((15000.0f + 10000.0f) / 10000.0f)
#define SCALE_3V3         (( 4700.0f + 10000.0f) / 10000.0f)

/* EMA smoothing on the sampled rails. At the 5 Hz sample rate below this
 * gives a time constant of roughly 2 seconds — slow enough to make the
 * raw-vs-filtered traces visibly different in TelePlot. */
#define EMA_ALPHA_I_5VSB  0.1f
#define EMA_ALPHA_VOLTAGE 0.1f

/* I2C bus handle (shared across components later) */
static i2c_master_bus_handle_t s_i2c_bus = NULL;

/* INA226 instances. For now only 5VSB. Will add 12V/5V/3V3 after PCB rev. */
static ina226_handle_t s_ina226_5vsb = NULL;

/* ADC rail configs */
static const cec_adc_rail_t s_rail_12v = {
    .channel = ADC_CH_V_12V, .samples = 8, .scale = SCALE_12V, .trim = TRIM_12V,
};
static const cec_adc_rail_t s_rail_5v = {
    .channel = ADC_CH_V_5V,  .samples = 8, .scale = SCALE_5V,  .trim = TRIM_5V,
};
static const cec_adc_rail_t s_rail_3v3 = {
    .channel = ADC_CH_V_3V3, .samples = 8, .scale = SCALE_3V3, .trim = TRIM_3V3,
};

/* Filter state */
static ema_t s_i_5vsb_ema;
static ema_t s_v_12v_ema, s_v_5v_ema, s_v_3v3_ema;

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
    ESP_RETURN_ON_ERROR(cec_adc_setup_channel(ADC_CH_V_12V), TAG, "setup 12V");
    ESP_RETURN_ON_ERROR(cec_adc_setup_channel(ADC_CH_V_5V),  TAG, "setup 5V");
    ESP_RETURN_ON_ERROR(cec_adc_setup_channel(ADC_CH_V_3V3), TAG, "setup 3V3");
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

    ema_init(&s_i_5vsb_ema, EMA_ALPHA_I_5VSB);
    ema_init(&s_v_12v_ema,  EMA_ALPHA_VOLTAGE);
    ema_init(&s_v_5v_ema,   EMA_ALPHA_VOLTAGE);
    ema_init(&s_v_3v3_ema,  EMA_ALPHA_VOLTAGE);

    /* Main loop: at 5 Hz, read INA226 + ADC rails, log a summary and
     * emit TelePlot series for every channel on every iteration. */
    ESP_LOGI(TAG, "Entering main loop");
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        if (s_ina226_5vsb != NULL) {
            float v_5vsb = 0.0f, i_5vsb = 0.0f;
            int32_t shunt_uv = 0;

            err = ina226_read_bus_voltage(s_ina226_5vsb, &v_5vsb);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "bus_v read failed: %s", esp_err_to_name(err));
            }
            err = ina226_read_current(s_ina226_5vsb, &i_5vsb);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "current read failed: %s", esp_err_to_name(err));
            }
            err = ina226_read_shunt_microvolts(s_ina226_5vsb, &shunt_uv);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "shunt_uv read failed: %s", esp_err_to_name(err));
            }

            float i_5vsb_ema = ema_update(&s_i_5vsb_ema, i_5vsb);

            teleplot_emit("v_5vsb", v_5vsb);
            teleplot_emit("i_5vsb_raw", i_5vsb);
            teleplot_emit("i_5vsb_ema", i_5vsb_ema);

            ESP_LOGI(TAG, "5VSB: V=%.3f V, I=%.4f A (ema=%.4f), shunt=%" PRId32 " uV (P=%.4f W)",
                     v_5vsb, i_5vsb, i_5vsb_ema, shunt_uv, v_5vsb * i_5vsb);
        }

        float v_12v = 0.0f, v_5v = 0.0f, v_3v3 = 0.0f;
        if (cec_adc_read(&s_rail_12v, &v_12v) == ESP_OK) {
            float ema = ema_update(&s_v_12v_ema, v_12v);
            teleplot_emit("v_12v", v_12v);
            teleplot_emit("v_12v_ema", ema);
        } else {
            ESP_LOGW(TAG, "v_12v read failed");
        }
        if (cec_adc_read(&s_rail_5v, &v_5v) == ESP_OK) {
            float ema = ema_update(&s_v_5v_ema, v_5v);
            teleplot_emit("v_5v", v_5v);
            teleplot_emit("v_5v_ema", ema);
        } else {
            ESP_LOGW(TAG, "v_5v read failed");
        }
        if (cec_adc_read(&s_rail_3v3, &v_3v3) == ESP_OK) {
            float ema = ema_update(&s_v_3v3_ema, v_3v3);
            teleplot_emit("v_3v3", v_3v3);
            teleplot_emit("v_3v3_ema", ema);
        } else {
            ESP_LOGW(TAG, "v_3v3 read failed");
        }

        ESP_LOGI(TAG, "rails: 12V=%.3f V, 5V=%.3f V, 3.3V=%.3f V",
                 v_12v, v_5v, v_3v3);

        /* 5 Hz */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
    }
}

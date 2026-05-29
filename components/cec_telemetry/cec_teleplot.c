/*
 * TelePlot output helpers — dual-stream variant.
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <inttypes.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "cec_teleplot.h"

static const char *TAG = "cec_telemetry";

#define TELEMETRY_UART_NUM       UART_NUM_0
#define TELEMETRY_UART_TXD       43
#define TELEMETRY_UART_RXD       44
#define TELEMETRY_UART_BAUD      921600
#define TELEMETRY_UART_TX_BUF    4096   /* Burst dumps need headroom */
#define TELEMETRY_UART_RX_BUF    256

static bool s_uart_ready = false;

esp_err_t cec_telemetry_init_uart(void)
{
    if (s_uart_ready) return ESP_OK;

    const uart_config_t cfg = {
        .baud_rate  = TELEMETRY_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(TELEMETRY_UART_NUM,
                                        TELEMETRY_UART_RX_BUF,
                                        TELEMETRY_UART_TX_BUF,
                                        0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_driver_install: %s — falling back to stdio",
                 esp_err_to_name(err));
        return err;
    }
    err = uart_param_config(TELEMETRY_UART_NUM, &cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_param_config: %s — falling back to stdio",
                 esp_err_to_name(err));
        uart_driver_delete(TELEMETRY_UART_NUM);
        return err;
    }
    err = uart_set_pin(TELEMETRY_UART_NUM,
                       TELEMETRY_UART_TXD, TELEMETRY_UART_RXD,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_pin: %s — falling back to stdio",
                 esp_err_to_name(err));
        uart_driver_delete(TELEMETRY_UART_NUM);
        return err;
    }

    s_uart_ready = true;
    ESP_LOGI(TAG, "telemetry UART ready: UART%d on GPIO %d/%d @ %d baud",
             (int)TELEMETRY_UART_NUM, TELEMETRY_UART_TXD, TELEMETRY_UART_RXD,
             TELEMETRY_UART_BAUD);
    return ESP_OK;
}

/* Common backend: write a formatted line to whichever transport is
 * currently the telemetry sink. If UART came up cleanly at boot,
 * everything goes there; otherwise stdout (JTAG USB-C) carries it. */
static void write_line(const char *buf, int len)
{
    if (len <= 0) return;
    if (s_uart_ready) {
        uart_write_bytes(TELEMETRY_UART_NUM, buf, (size_t)len);
    } else {
        fwrite(buf, 1, (size_t)len, stdout);
    }
}

/* Clamp snprintf's documented "I would have written N bytes" return so
 * we never read past the formatting buffer on truncation. */
static int clamp_len(int n, size_t cap)
{
    if (n < 0) return 0;
    return (n < (int)cap) ? n : (int)cap - 1;
}

void teleplot_emit(const char *name, float value)
{
    char buf[80];
    int n = snprintf(buf, sizeof(buf), ">%s:%.6f\n", name, value);
    write_line(buf, clamp_len(n, sizeof(buf)));
}

void teleplot_emit_t(const char *name, int64_t time_ms, float value)
{
    char buf[96];
    int n = snprintf(buf, sizeof(buf), ">%s:%" PRId64 ":%.6f\n",
                     name, time_ms, value);
    write_line(buf, clamp_len(n, sizeof(buf)));
}

void teleplot_writef(const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    write_line(buf, clamp_len(n, sizeof(buf)));
}

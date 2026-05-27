/*
 * INA226 driver for ESP-IDF 6.x
 *
 * Wraps the i2c_master driver to provide a clean handle-based API.
 * Supports multiple INA226 instances on the same bus, which is the
 * post-PCB-rev architecture (4 INA226s on the 24-pin module).
 *
 * Optional trim factors allow matching the v0.5.9 firmware's per-rail
 * calibration constants. Setting trim values to 1.0 returns raw readings.
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handle for an INA226 device. */
typedef struct ina226_dev_t* ina226_handle_t;

/* Configuration for an INA226 instance. */
typedef struct {
    i2c_master_bus_handle_t bus_handle;   /* I2C bus this device sits on */
    uint8_t i2c_addr;                     /* 7-bit I2C address (0x40-0x4F) */
    float shunt_ohms;                     /* Physical shunt resistor value */
    float max_current_a;                  /* Target max current; determines CURRENT_LSB */
    uint32_t scl_speed_hz;                /* Per-device SCL speed (typically 400000) */
    float voltage_trim;                   /* Multiplier applied to bus voltage (1.0 = raw) */
    float current_trim;                   /* Multiplier applied to current (1.0 = raw) */
} ina226_config_t;

/* Default configuration block. Caller overrides fields they want changed. */
#define INA226_CONFIG_DEFAULT() (ina226_config_t){     \
    .bus_handle = NULL,                                 \
    .i2c_addr = 0x40,                                   \
    .shunt_ohms = 0.002f,                               \
    .max_current_a = 5.0f,                              \
    .scl_speed_hz = 400000,                             \
    .voltage_trim = 1.0f,                               \
    .current_trim = 1.0f,                               \
}

/*
 * Initialize an INA226 device on the bus.
 *
 * Validates the manufacturer ID register, writes the configuration register
 * (averaging=1, conversion=1.1ms shunt, 1.1ms bus, continuous mode), and
 * programs the calibration register based on shunt_ohms and max_current_a.
 *
 * Returns ESP_OK on success, ESP_ERR_NOT_FOUND if the manufacturer ID doesn't
 * match the expected 0x5449, or the underlying i2c_master_* error otherwise.
 */
esp_err_t ina226_create(const ina226_config_t *config, ina226_handle_t *out_handle);

/*
 * Release an INA226 device and remove it from the bus.
 */
esp_err_t ina226_destroy(ina226_handle_t handle);

/*
 * Read the bus voltage in volts. Trim factor is applied.
 *
 * The INA226 bus voltage register has 1.25 mV LSB resolution and a maximum
 * value of ~36 V.
 */
esp_err_t ina226_read_bus_voltage(ina226_handle_t handle, float *out_volts);

/*
 * Read the current in amps. Trim factor is applied.
 *
 * Sign reflects current direction. Saturates at max_current_a configured at
 * create time.
 */
esp_err_t ina226_read_current(ina226_handle_t handle, float *out_amps);

/*
 * Read the shunt voltage directly in microvolts (untrimmed).
 *
 * Useful for diagnostics and for verifying the calibration register is set
 * correctly (current = shunt_uV / shunt_ohms).
 */
esp_err_t ina226_read_shunt_microvolts(ina226_handle_t handle, int32_t *out_microvolts);

/*
 * Get the actual programmed CURRENT_LSB in amps (informational).
 */
float ina226_get_current_lsb(ina226_handle_t handle);

/*
 * Get the programmed CAL register value (informational, useful for boot logging).
 */
uint16_t ina226_get_cal_value(ina226_handle_t handle);

#ifdef __cplusplus
}
#endif

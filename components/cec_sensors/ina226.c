/*
 * INA226 driver implementation for ESP-IDF 6.x
 */

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_check.h"
#include "ina226.h"

static const char *TAG = "ina226";

/* Register addresses */
#define INA226_REG_CONFIG    0x00
#define INA226_REG_SHUNT_V   0x01
#define INA226_REG_BUS_V     0x02
#define INA226_REG_POWER     0x03
#define INA226_REG_CURRENT   0x04
#define INA226_REG_CAL       0x05
#define INA226_REG_MFR_ID    0xFE
#define INA226_REG_DIE_ID    0xFF

/* Manufacturer ID we expect to read back */
#define INA226_MFR_ID_EXPECTED 0x5449

/* Config-register presets live in ina226.h (INA226_CONFIG_STEADY / _HS);
 * the value actually written at create time comes from config->config_value. */

/* I2C operation timeout */
#define INA226_I2C_TIMEOUT_MS 100

/* Bus voltage LSB in volts (fixed in INA226) */
#define INA226_BUS_LSB_V 0.00125f

/* Internal device struct */
struct ina226_dev_t {
    i2c_master_dev_handle_t dev_handle;
    float shunt_ohms;
    float current_lsb;        /* Amps per LSB of CURRENT register */
    float voltage_trim;       /* Multiplier applied to bus voltage */
    float current_trim;       /* Multiplier applied to current */
    uint16_t cal_value;       /* Programmed CAL register value */
};

/* --- I2C primitives --- */

static esp_err_t ina226_write_reg(struct ina226_dev_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF),
    };
    return i2c_master_transmit(dev->dev_handle, buf, sizeof(buf), INA226_I2C_TIMEOUT_MS);
}

static esp_err_t ina226_read_reg(struct ina226_dev_t *dev, uint8_t reg, uint16_t *value)
{
    uint8_t rx[2];
    esp_err_t err = i2c_master_transmit_receive(dev->dev_handle, &reg, 1,
                                                rx, sizeof(rx),
                                                INA226_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    *value = ((uint16_t)rx[0] << 8) | rx[1];
    return ESP_OK;
}

/* --- Public API --- */

esp_err_t ina226_create(const ina226_config_t *config, ina226_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(config != NULL && out_handle != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "null config or out_handle");
    ESP_RETURN_ON_FALSE(config->bus_handle != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "bus_handle is NULL");
    ESP_RETURN_ON_FALSE(config->shunt_ohms > 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "shunt_ohms must be positive");
    ESP_RETURN_ON_FALSE(config->max_current_a > 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "max_current_a must be positive");
    ESP_RETURN_ON_FALSE(config->voltage_trim > 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "voltage_trim must be positive");
    /* current_trim may be negative to invert the shunt sign on boards
     * where IN+/IN- are wired backwards; only zero is invalid. */
    ESP_RETURN_ON_FALSE(config->current_trim != 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "current_trim must be nonzero");

    /* Allocate internal struct */
    struct ina226_dev_t *dev = calloc(1, sizeof(struct ina226_dev_t));
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_NO_MEM, TAG, "alloc failed");

    /* Add device to bus */
    i2c_device_config_t i2c_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_addr,
        .scl_speed_hz = config->scl_speed_hz,
    };
    esp_err_t err = i2c_master_bus_add_device(config->bus_handle, &i2c_dev_cfg,
                                              &dev->dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    /* Verify manufacturer ID */
    uint16_t mfr_id = 0;
    err = ina226_read_reg(dev, INA226_REG_MFR_ID, &mfr_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MFR_ID read failed: %s", esp_err_to_name(err));
        goto cleanup;
    }
    if (mfr_id != INA226_MFR_ID_EXPECTED) {
        ESP_LOGE(TAG, "MFR_ID mismatch: got 0x%04X, expected 0x%04X",
                 mfr_id, INA226_MFR_ID_EXPECTED);
        err = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }

    /* Write configuration register (caller-supplied; defaults to
     * INA226_CONFIG_STEADY = 16-sample avg, shunt+bus continuous). */
    err = ina226_write_reg(dev, INA226_REG_CONFIG, config->config_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CONFIG write failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    /* Compute and program calibration register
     *
     * From INA226 datasheet:
     *   CAL = 0.00512 / (CURRENT_LSB * R_SHUNT)
     * where CURRENT_LSB = max_current / 32768
     *
     * CAL is a 16-bit unsigned register, so the result must fit in 0..65535.
     */
    dev->shunt_ohms = config->shunt_ohms;
    dev->current_lsb = config->max_current_a / 32768.0f;
    dev->voltage_trim = config->voltage_trim;
    dev->current_trim = config->current_trim;
    uint32_t cal32 = (uint32_t)(0.00512f / (dev->current_lsb * dev->shunt_ohms));
    if (cal32 > 0xFFFF) {
        ESP_LOGE(TAG, "Computed CAL=%" PRIu32 " exceeds 16-bit range. Increase max_current_a.",
                 cal32);
        err = ESP_ERR_INVALID_ARG;
        goto cleanup;
    }
    dev->cal_value = (uint16_t)cal32;

    err = ina226_write_reg(dev, INA226_REG_CAL, dev->cal_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAL write failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    /* Read back CAL to verify */
    uint16_t cal_verify = 0;
    err = ina226_read_reg(dev, INA226_REG_CAL, &cal_verify);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAL verify read failed: %s", esp_err_to_name(err));
        goto cleanup;
    }
    if (cal_verify != dev->cal_value) {
        ESP_LOGE(TAG, "CAL verify mismatch: wrote 0x%04X, read 0x%04X",
                 dev->cal_value, cal_verify);
        err = ESP_FAIL;
        goto cleanup;
    }

    ESP_LOGI(TAG, "INA226 @ 0x%02X: shunt=%.4f ohm, max=%.1f A, LSB=%.1f uA, CAL=0x%04X, trim V=%.4f I=%.4f",
             config->i2c_addr, dev->shunt_ohms, config->max_current_a,
             dev->current_lsb * 1e6f, dev->cal_value,
             dev->voltage_trim, dev->current_trim);

    *out_handle = dev;
    return ESP_OK;

cleanup:
    i2c_master_bus_rm_device(dev->dev_handle);
    free(dev);
    return err;
}

esp_err_t ina226_destroy(ina226_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "null handle");
    esp_err_t err = i2c_master_bus_rm_device(handle->dev_handle);
    free(handle);
    return err;
}

esp_err_t ina226_set_config(ina226_handle_t handle, uint16_t config_value)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "null handle");
    return ina226_write_reg(handle, INA226_REG_CONFIG, config_value);
}

esp_err_t ina226_read_bus_voltage(ina226_handle_t handle, float *out_volts)
{
    ESP_RETURN_ON_FALSE(handle != NULL && out_volts != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "null arg");
    uint16_t raw;
    esp_err_t err = ina226_read_reg(handle, INA226_REG_BUS_V, &raw);
    if (err != ESP_OK) return err;
    *out_volts = (float)raw * INA226_BUS_LSB_V * handle->voltage_trim;
    return ESP_OK;
}

esp_err_t ina226_read_current(ina226_handle_t handle, float *out_amps)
{
    ESP_RETURN_ON_FALSE(handle != NULL && out_amps != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "null arg");
    /* Software path: read the shunt-voltage register directly and divide
     * by the shunt resistance. Independent of the CAL register, so it
     * stays correct across config/mode changes (including the HS preset)
     * and the v2 board's non-standard shunt values. */
    uint16_t raw;
    esp_err_t err = ina226_read_reg(handle, INA226_REG_SHUNT_V, &raw);
    if (err != ESP_OK) return err;
    int16_t signed_raw = (int16_t)raw;          /* signed, LSB 2.5 uV */
    float v_shunt = (float)signed_raw * 2.5e-6f;
    *out_amps = (v_shunt / handle->shunt_ohms) * handle->current_trim;
    return ESP_OK;
}

esp_err_t ina226_read_shunt_microvolts(ina226_handle_t handle, int32_t *out_microvolts)
{
    ESP_RETURN_ON_FALSE(handle != NULL && out_microvolts != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "null arg");
    uint16_t raw;
    esp_err_t err = ina226_read_reg(handle, INA226_REG_SHUNT_V, &raw);
    if (err != ESP_OK) return err;
    int16_t signed_raw = (int16_t)raw;
    /* Shunt register LSB is 2.5 uV; integer math (* 25 / 10) avoids float */
    *out_microvolts = ((int32_t)signed_raw * 25) / 10;
    return ESP_OK;
}

float ina226_get_current_lsb(ina226_handle_t handle)
{
    return handle ? handle->current_lsb : 0.0f;
}

uint16_t ina226_get_cal_value(ina226_handle_t handle)
{
    return handle ? handle->cal_value : 0;
}

/*
 * NVS storage for CEC firmware state.
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cec_nvs.h"

static const char *TAG = "cec_nvs";
#define NS "cec"

static bool s_inited = false;
static nvs_handle_t s_handle;

esp_err_t cec_nvs_init(void)
{
    if (s_inited) return ESP_OK;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase (%s); erasing", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_flash_init");
    ESP_RETURN_ON_ERROR(nvs_open(NS, NVS_READWRITE, &s_handle),
                        TAG, "nvs_open(%s)", NS);

    s_inited = true;
    ESP_LOGI(TAG, "ready (namespace=%s)", NS);
    return ESP_OK;
}

esp_err_t cec_nvs_save_blob(const char *key, uint32_t magic,
                            const void *data, size_t size)
{
    if (!s_inited)                  return ESP_ERR_INVALID_STATE;
    if (key == NULL || data == NULL) return ESP_ERR_INVALID_ARG;

    size_t total = sizeof(magic) + size;
    uint8_t *buf = malloc(total);
    if (buf == NULL) return ESP_ERR_NO_MEM;

    memcpy(buf, &magic, sizeof(magic));
    memcpy(buf + sizeof(magic), data, size);

    esp_err_t err = nvs_set_blob(s_handle, key, buf, total);
    free(buf);
    if (err != ESP_OK) return err;
    return nvs_commit(s_handle);
}

esp_err_t cec_nvs_load_blob(const char *key, uint32_t magic,
                            void *data, size_t size)
{
    if (!s_inited)                  return ESP_ERR_INVALID_STATE;
    if (key == NULL || data == NULL) return ESP_ERR_INVALID_ARG;

    size_t expected = sizeof(magic) + size;
    size_t actual = 0;
    esp_err_t err = nvs_get_blob(s_handle, key, NULL, &actual);
    if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_ERR_NOT_FOUND;
    if (err != ESP_OK)                return err;
    if (actual != expected)           return ESP_ERR_INVALID_SIZE;

    uint8_t *buf = malloc(actual);
    if (buf == NULL) return ESP_ERR_NO_MEM;

    err = nvs_get_blob(s_handle, key, buf, &actual);
    if (err != ESP_OK) { free(buf); return err; }

    uint32_t stored_magic;
    memcpy(&stored_magic, buf, sizeof(stored_magic));
    if (stored_magic != magic) {
        free(buf);
        return ESP_ERR_INVALID_VERSION;
    }
    memcpy(data, buf + sizeof(magic), size);
    free(buf);
    return ESP_OK;
}

esp_err_t cec_nvs_clear_blob(const char *key)
{
    if (!s_inited)    return ESP_ERR_INVALID_STATE;
    if (key == NULL)  return ESP_ERR_INVALID_ARG;
    esp_err_t err = nvs_erase_key(s_handle, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (err != ESP_OK) return err;
    return nvs_commit(s_handle);
}

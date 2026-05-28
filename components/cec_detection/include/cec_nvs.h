/*
 * NVS storage for CEC firmware state.
 *
 * Thin wrapper around ESP-IDF NVS that handles the boring parts:
 * initialization + namespace, magic-number sanity check on load, and
 * commit-on-save. Used initially by Layer 3 profile persistence so
 * the learned baseline survives reboots; designed to be generic enough
 * that other persistent state (ACS712 calibration, layer enables, etc.)
 * can ride the same path.
 *
 * Stored layout per key:
 *   [4 bytes: magic][payload bytes]
 * The magic is checked on load so a firmware revision with an
 * incompatible payload schema rejects the stored blob cleanly instead
 * of silently loading garbage.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialize the NVS partition and open the "cec" namespace. Idempotent.
 * If the NVS partition is unreadable (e.g. version bump or full), it's
 * erased and reinitialized.
 */
esp_err_t cec_nvs_init(void);

/*
 * Write data under `key`, prefixed by `magic`. Commits before returning.
 */
esp_err_t cec_nvs_save_blob(const char *key, uint32_t magic,
                            const void *data, size_t size);

/*
 * Read data under `key`. Stored blob must be exactly 4 + size bytes
 * with leading 4 bytes matching `magic`. Returns:
 *   ESP_OK                  - loaded
 *   ESP_ERR_NOT_FOUND       - no blob for that key
 *   ESP_ERR_INVALID_VERSION - magic mismatch (schema changed)
 *   ESP_ERR_INVALID_SIZE    - blob present but wrong size
 *   other                   - propagated from nvs_get_blob
 */
esp_err_t cec_nvs_load_blob(const char *key, uint32_t magic,
                            void *data, size_t size);

/*
 * Erase the blob under `key`. ESP_OK even if the key didn't exist.
 */
esp_err_t cec_nvs_clear_blob(const char *key);

#ifdef __cplusplus
}
#endif

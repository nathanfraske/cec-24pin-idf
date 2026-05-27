/*
 * TelePlot output helpers.
 *
 * Emits lines in the TelePlot serial format on the active console
 * (USB Serial JTAG by default on this project). Each emitted line
 * starts with '>' and is ignored by any consumer that isn't TelePlot,
 * so it coexists cleanly with ESP_LOG output.
 *
 * Format reference: https://github.com/nesnes/teleplot
 *   >name:value\n                emits a sample with host-side timestamp
 *   >name:time_ms:value\n        emits a sample with explicit timestamp
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Emit a TelePlot sample. TelePlot timestamps it on receipt.
 */
void teleplot_emit(const char *name, float value);

/*
 * Emit a TelePlot sample with an explicit timestamp in milliseconds.
 * Use when sample timing must be precise (e.g. burst capture replay).
 */
void teleplot_emit_t(const char *name, int64_t time_ms, float value);

#ifdef __cplusplus
}
#endif

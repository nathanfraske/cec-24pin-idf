/*
 * TelePlot output helpers — dual-stream variant.
 *
 * Two physical USB-C ports carry firmware output on the 24-pin board:
 *
 *   - JTAG USB-C  (native ESP32-S3 USB Serial-JTAG)
 *       CLI input, ESP_LOG output, command responses, boot banner.
 *       Default `printf`/stdio routes here.
 *
 *   - UART USB-C  (CH340K bridge to UART0 on GPIO 43/44)
 *       Every TelePlot line — steady-state 10 Hz telemetry from the
 *       main loop AND burst dumps (`>BURST_BEGIN ... >BURST_END`)
 *       from cec_capture. Runs at 921600 baud.
 *
 * Splitting the streams keeps the heavy traffic (~600 KB per burst at
 * full fidelity) off the same wire that's carrying CLI input.
 *
 * cec_telemetry_init_uart() must be called once at boot before any
 * teleplot_* helper produces useful output on the UART side. If the
 * init fails (cable not plugged in during dev, etc.) the helpers
 * silently fall back to stdio so TelePlot keeps working over the JTAG
 * port at the slower rate. Idempotent.
 *
 * Format reference: https://github.com/nesnes/teleplot
 *   >name:value\n                 — sample with host-side timestamp
 *   >name:time_ms:value\n         — sample with explicit timestamp
 *   any other '>'-prefixed line   — envelope / annotation
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Configure UART0 at 921600 baud on GPIO 43 (TX) / 44 (RX) and route
 * subsequent teleplot_* output to it. Returns ESP_OK on success or the
 * underlying UART driver error otherwise (helpers continue to work via
 * stdio fallback).
 */
esp_err_t cec_telemetry_init_uart(void);

/*
 * Emit a TelePlot sample with no embedded timestamp (TelePlot stamps
 * it on receipt). Prefer the _t variant if a device-side timestamp
 * is meaningful.
 */
void teleplot_emit(const char *name, float value);

/*
 * Emit a TelePlot sample with an explicit timestamp in milliseconds.
 * Use when sample timing must be precise (e.g. burst capture replay
 * alignment against the slow loop).
 */
void teleplot_emit_t(const char *name, int64_t time_ms, float value);

/*
 * Generic printf-style line emit, for non-sample TelePlot output:
 *   teleplot_writef(">BURST_BEGIN:%s:%d_normal+%d_hs:%d\n", ...);
 * Caller provides the leading '>' and trailing newline. Routed to
 * the same backend as the typed helpers above.
 */
void teleplot_writef(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

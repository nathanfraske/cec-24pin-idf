/*
 * Line-based command interface over USB CDC.
 *
 * Reads one command per line on stdin, splits on whitespace, looks up
 * the first token in the caller-supplied command table, and dispatches.
 * The framework adds a "help" command on top that lists all registered
 * commands (or shows the description of a specific one).
 *
 * Output uses standard printf — the same stream the burst dumps and
 * TelePlot output use, so a single serial connection carries both
 * directions cleanly. TelePlot ignores any line that doesn't start
 * with '>', so command-style output coexists with telemetry.
 *
 * The reader task runs at a low priority on whichever core FreeRTOS
 * picks; commands themselves run on that task. Handlers must therefore
 * be quick or yield — long-running work belongs on a dedicated task.
 */

#pragma once

#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Command handler. argv[0] is the command name. Return 0 on success,
 * any non-zero value to signal an error (the framework prints
 * "error: <retval>" to indicate failure).
 */
typedef int (*cec_cli_cmd_fn_t)(int argc, char **argv);

typedef struct {
    const char       *name;     /* First token of the command */
    const char       *help;     /* One-line description shown by 'help' */
    cec_cli_cmd_fn_t  handler;
} cec_cli_command_t;

/*
 * Initialize the USB CDC driver for blocking stdin reads, register the
 * caller-supplied command table, and start the reader task. `commands`
 * must remain valid for the lifetime of the program (static is fine).
 *
 * Idempotent.
 */
esp_err_t cec_cli_init(const cec_cli_command_t *commands, size_t count);

#ifdef __cplusplus
}
#endif

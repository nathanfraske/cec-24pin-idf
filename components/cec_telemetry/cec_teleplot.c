/*
 * TelePlot output helpers.
 */

#include <stdio.h>
#include <inttypes.h>
#include "cec_teleplot.h"

void teleplot_emit(const char *name, float value)
{
    printf(">%s:%.6f\n", name, value);
}

void teleplot_emit_t(const char *name, int64_t time_ms, float value)
{
    printf(">%s:%" PRId64 ":%.6f\n", name, time_ms, value);
}

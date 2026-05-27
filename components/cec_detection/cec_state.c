/*
 * System-state classifier.
 */

#include <stdbool.h>
#include "cec_state.h"

/* Thresholds. OFF/STANDBY split uses 5VSB and 12V respectively so the
 * "unplugged" state is distinguishable from "plugged but main rails off".
 * IDLE/ACTIVE/PEAK transitions are power-based with hysteresis (v0.5.9). */
#define V_5VSB_PRESENT_ABOVE  1.0f
#define V_12V_UP_ABOVE        10.5f
#define P_IDLE_TO_ACTIVE_IN   40.0f
#define P_IDLE_TO_ACTIVE_OUT  32.0f
#define P_ACTIVE_TO_PEAK_IN   150.0f
#define P_ACTIVE_TO_PEAK_OUT  130.0f

static const char *NAMES[CEC_STATE_COUNT] = {
    [CEC_STATE_OFF]     = "OFF",
    [CEC_STATE_STANDBY] = "STANDBY",
    [CEC_STATE_IDLE]    = "IDLE",
    [CEC_STATE_ACTIVE]  = "ACTIVE",
    [CEC_STATE_PEAK]    = "PEAK",
};

const char *cec_state_name(cec_state_t s)
{
    if ((int)s < 0 || (int)s >= CEC_STATE_COUNT) return "?";
    return NAMES[s];
}

cec_state_t cec_state_classify(float v_12v, float v_5vsb, float p_total,
                               cec_state_t current)
{
    if (v_5vsb < V_5VSB_PRESENT_ABOVE) return CEC_STATE_OFF;
    if (v_12v  < V_12V_UP_ABOVE)       return CEC_STATE_STANDBY;

    bool was_active = (current == CEC_STATE_ACTIVE || current == CEC_STATE_PEAK);
    bool was_peak   = (current == CEC_STATE_PEAK);

    float idle_to_active = was_active ? P_IDLE_TO_ACTIVE_OUT : P_IDLE_TO_ACTIVE_IN;
    float active_to_peak = was_peak   ? P_ACTIVE_TO_PEAK_OUT : P_ACTIVE_TO_PEAK_IN;

    if (p_total < idle_to_active) return CEC_STATE_IDLE;
    if (p_total < active_to_peak) return CEC_STATE_ACTIVE;
    return CEC_STATE_PEAK;
}

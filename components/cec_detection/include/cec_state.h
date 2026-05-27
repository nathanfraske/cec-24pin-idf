/*
 * System-state classifier.
 *
 * Maps (v_12v, p_total) to a coarse operating state with hysteresis on the
 * power-defined transitions. Stateless: callers hold the current state and
 * pass it back in on each call. Constants and behavior are the v0.5.9
 * defaults.
 *
 *   OFF      - 12V rail below 1.0 V (PSU off)
 *   STANDBY  - 12V below 10.5 V (PSU 5VSB only)
 *   IDLE     - p_total < 40 W (entry) / < 32 W (exit hysteresis)
 *   ACTIVE   - p_total < 150 W (entry) / < 130 W (exit hysteresis)
 *   PEAK     - p_total >= 150 W (entry) / >= 130 W (exit hysteresis)
 *
 * Power is the sum of main-rail products: V_12V*I_12V + V_5V*I_5V +
 * V_3V3*I_3V3. The 5VSB rail is not included by design (matches v0.5.9).
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CEC_STATE_OFF = 0,
    CEC_STATE_STANDBY,
    CEC_STATE_IDLE,
    CEC_STATE_ACTIVE,
    CEC_STATE_PEAK,
    CEC_STATE_COUNT
} cec_state_t;

/*
 * Human-readable state name. Always returns a valid pointer.
 */
const char *cec_state_name(cec_state_t s);

/*
 * Classify the next state given the latest filtered readings and the
 * caller's current state. Hysteresis is applied to the IDLE<->ACTIVE
 * and ACTIVE<->PEAK transitions.
 */
cec_state_t cec_state_classify(float v_12v, float p_total, cec_state_t current);

#ifdef __cplusplus
}
#endif

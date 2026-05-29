/*
 * Shared CEC firmware types.
 *
 * This header is part of the cec_common component (vs. any specific
 * detection layer) so callers can use the enums without dragging in
 * the full detector machinery — Layer 1 needs cec_severity_t, the
 * burst-capture engine wants cec_state_t for the pre-trigger sample
 * shape, the serial CLI surfaces both in `status` output, etc.
 * Implementations of the declared helpers live in the component that
 * owns the corresponding algorithm.
 *
 * The 24-pin module's state classifier maps (v_12v, v_5vsb, p_total)
 * to a coarse PSU operating state with hysteresis on the power-defined
 * transitions:
 *
 *   OFF      - 5VSB below 1.0 V (PSU unplugged / completely off)
 *   STANDBY  - 5VSB up, 12V below 10.5 V (PSU plugged in, main rails off)
 *   IDLE     - main rails up, p_total < 40 W (entry) / < 32 W (exit hysteresis)
 *   ACTIVE   - p_total < 150 W (entry) / < 130 W (exit hysteresis)
 *   PEAK     - p_total >= 150 W (entry) / >= 130 W (exit hysteresis)
 *
 * Note this diverges slightly from v0.5.9 in the OFF/STANDBY split:
 * v0.5.9 used 12V alone, which conflated "PSU unplugged" with "PSU
 * plugged in, switched off". Using 5VSB as the OFF discriminant lets
 * the two be distinguished.
 *
 * Power is the sum of main-rail products: V_12V*I_12V + V_5V*I_5V +
 * V_3V3*I_3V3. The 5VSB rail is not included by design (matches v0.5.9).
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* ---- PSU state (24-pin module) ---- */

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
cec_state_t cec_state_classify(float v_12v, float v_5vsb, float p_total,
                               cec_state_t current);

/* ---- Per-cable load state (EPS module) ----
 *
 * TODO: cec_load_state_t and CEC_LOAD_COUNT belong here, mirroring the
 * EPS-side cec_classifier.c enum. Land the actual values when next
 * syncing the EPS source so cec-24pin-idf and cec-eps-idf can include
 * each other's headers without name clashes. Until then, EPS keeps its
 * own private definition in cec-eps-idf's tree.
 */

/* ---- Detection severity (shared by Layer 1 and the CLI) ---- */

typedef enum {
    CEC_SEV_NONE = 0,
    CEC_SEV_WARNING,
    CEC_SEV_CRITICAL,
} cec_severity_t;

/*
 * Human-readable severity name. Always returns a valid pointer.
 * Implementation lives in the detection component alongside Layer 1.
 */
const char *cec_severity_name(cec_severity_t s);

#ifdef __cplusplus
}
#endif

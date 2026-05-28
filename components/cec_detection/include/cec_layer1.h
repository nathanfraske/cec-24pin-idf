/*
 * Layer 1 static voltage threshold detector.
 *
 * Watches one rail's voltage against a nominal +- warn / crit deviation
 * band. Returns the "sustained" severity, where:
 *   - NONE and WARNING are reported immediately on a single sample.
 *   - CRITICAL is reported only after `crit_required` consecutive samples
 *     are outside the crit band. This debounces single-sample glitches.
 *
 * The detector is stateful (holds the consecutive counter) but cheap to
 * reset, so callers can gate updates by system state (e.g. ignore while
 * the PSU is OFF/STANDBY) without losing config.
 *
 * Severity bands carry forward from v0.5.9: 5%/10% deviation for the main
 * rails (12V/5V/3V3) and 10%/20% for the loose-spec 5VSB.
 */

#pragma once

#include "cec_state.h"  /* for cec_severity_t */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float nominal;     /* Nominal rail voltage (e.g. 12.0) */
    float warn_band;   /* Fractional deviation triggering WARNING (e.g. 0.05) */
    float crit_band;   /* Fractional deviation triggering CRITICAL (e.g. 0.10) */
} cec_rail_spec_t;

typedef struct {
    cec_rail_spec_t spec;
    int crit_required;       /* Consecutive bad samples before reporting CRITICAL */
    int crit_consecutive;    /* Running counter, internal */
} cec_layer1_detector_t;

/*
 * Configure the detector. `crit_required` >= 1; 3 is the v0.5.9 default.
 */
void cec_layer1_init(cec_layer1_detector_t *d,
                     const cec_rail_spec_t *spec,
                     int crit_required);

/*
 * Feed a sample and return the sustained severity. Below 0.1 V the rail
 * is considered off and NONE is returned (the consecutive counter is
 * reset).
 */
cec_severity_t cec_layer1_update(cec_layer1_detector_t *d, float v_rail);

/*
 * Clear the consecutive counter. Use when entering a state where the
 * rail isn't expected to be at nominal (e.g. STANDBY) so a transient
 * during a later state change can't immediately trip CRITICAL.
 */
void cec_layer1_reset(cec_layer1_detector_t *d);

#ifdef __cplusplus
}
#endif

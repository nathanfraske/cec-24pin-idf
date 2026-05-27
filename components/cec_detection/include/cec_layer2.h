/*
 * Layer 2 adaptive transient detector.
 *
 * Watches the deviation between an instantaneous (or lightly-filtered)
 * reading and its slow EMA. The detection threshold is adaptive: it
 * scales with a running variance estimate of the deviation, so a rail
 * that's normally noisy gets a wide threshold and a quiet rail gets a
 * tight one. A configurable floor keeps the threshold from dropping
 * below a per-rail minimum so a perfectly quiet rail can still detect
 * a real transient.
 *
 * The variance estimator only updates on samples that are NOT firing
 * (i.e. when the rail looks calm), so a transient doesn't widen its
 * own threshold.
 *
 * Fires when |instant - ema| > max(min_threshold, k_sigma * std)
 * for required_consecutive samples in a row.
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float min_threshold;        /* Floor for the adaptive threshold */
    float k_sigma;              /* Multiplier on running std (typ 5.0) */
    int   required_consecutive; /* Debounce, typ 3 */
    int   consecutive_count;    /* Internal: running fire counter */
    float variance_est;         /* Internal: running variance estimate */
    bool  initialized;          /* Internal: first-sample flag */
} cec_layer2_detector_t;

/*
 * Initialize a detector. `required_consecutive` is clamped to >= 1.
 */
void cec_layer2_init(cec_layer2_detector_t *d,
                     float min_threshold,
                     float k_sigma,
                     int   required_consecutive);

/*
 * Feed an (instant, ema) pair. Returns true the iteration on which the
 * detector fires (consecutive count reaches the limit). The counter
 * resets on fire so a single sustained transient produces exactly one
 * fire event.
 */
bool cec_layer2_update(cec_layer2_detector_t *d, float instant, float ema);

/*
 * Current effective threshold (max of min_threshold and k_sigma*std).
 * Useful for visualizing detector behavior on TelePlot.
 */
float cec_layer2_current_threshold(const cec_layer2_detector_t *d);

/*
 * Clear the consecutive counter without touching the variance estimate.
 * Use during state-change settle windows so a pending fire from the
 * previous state can't carry over.
 */
void cec_layer2_reset(cec_layer2_detector_t *d);

#ifdef __cplusplus
}
#endif

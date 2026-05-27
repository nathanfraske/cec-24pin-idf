/*
 * Layer 2 adaptive transient detector.
 */

#include <math.h>
#include "cec_layer2.h"

/* Tiny variance floor so the initial std is never exactly zero and the
 * EMA-update can't get stuck. */
#define VARIANCE_FLOOR 0.0001f

/* Variance estimator update weights (from v0.5.9). */
#define VAR_RETAIN     0.98f
#define VAR_NEW        0.02f

void cec_layer2_init(cec_layer2_detector_t *d,
                     float min_threshold,
                     float k_sigma,
                     int   required_consecutive)
{
    d->min_threshold = min_threshold;
    d->k_sigma = k_sigma;
    d->required_consecutive = (required_consecutive >= 1) ? required_consecutive : 1;
    d->consecutive_count = 0;
    d->variance_est = VARIANCE_FLOOR;
    d->initialized = false;
}

void cec_layer2_reset(cec_layer2_detector_t *d)
{
    d->consecutive_count = 0;
}

bool cec_layer2_update(cec_layer2_detector_t *d, float instant, float ema)
{
    float dev = instant - ema;
    float abs_dev = fabsf(dev);

    if (!d->initialized) {
        d->variance_est = dev * dev + VARIANCE_FLOOR;
        d->initialized = true;
    }

    float std = sqrtf(d->variance_est);
    float adaptive = d->k_sigma * std;
    float threshold = (adaptive > d->min_threshold) ? adaptive : d->min_threshold;

    if (abs_dev > threshold) {
        d->consecutive_count++;
        if (d->consecutive_count >= d->required_consecutive) {
            d->consecutive_count = 0;
            return true;
        }
    } else {
        d->consecutive_count = 0;
        d->variance_est = VAR_RETAIN * d->variance_est + VAR_NEW * dev * dev;
    }
    return false;
}

float cec_layer2_current_threshold(const cec_layer2_detector_t *d)
{
    float std = sqrtf(d->variance_est);
    float adaptive = d->k_sigma * std;
    return (adaptive > d->min_threshold) ? adaptive : d->min_threshold;
}

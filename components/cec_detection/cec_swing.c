/*
 * Windowed swing detector.
 */

#include <math.h>
#include <string.h>
#include "cec_swing.h"

void cec_swing_detector_init(cec_swing_detector_t *d, float *buf,
                             size_t capacity, int required_consecutive)
{
    d->buf = buf;
    d->capacity = capacity;
    d->idx = 0;
    d->count = 0;
    d->sum = 0.0f;
    d->consecutive_count = 0;
    d->required_consecutive = (required_consecutive >= 1) ? required_consecutive : 1;
    memset(buf, 0, capacity * sizeof(float));
}

bool cec_swing_detector_update(cec_swing_detector_t *d,
                               float value, float threshold)
{
    /* Ring buffer + running sum. */
    float oldest = d->buf[d->idx];
    d->buf[d->idx] = value;
    d->sum = d->sum - oldest + value;
    d->idx = (d->idx + 1) % d->capacity;
    if (d->count < d->capacity) {
        d->count++;
    }

    /* Don't fire until the window is full so the mean is trustworthy. */
    if (d->count < d->capacity) {
        return false;
    }

    float mean = d->sum / (float)d->count;
    float swing = fabsf(value - mean);
    if (swing > threshold) {
        d->consecutive_count++;
        if (d->consecutive_count >= d->required_consecutive) {
            d->consecutive_count = 0;
            return true;
        }
    } else {
        d->consecutive_count = 0;
    }
    return false;
}

float cec_swing_detector_mean(const cec_swing_detector_t *d)
{
    if (d->count == 0) return 0.0f;
    return d->sum / (float)d->count;
}

bool cec_swing_detector_is_full(const cec_swing_detector_t *d)
{
    return d->count == d->capacity;
}

size_t cec_swing_detector_count(const cec_swing_detector_t *d)
{
    return d->count;
}

void cec_swing_detector_reset_to(cec_swing_detector_t *d, float value)
{
    for (size_t i = 0; i < d->capacity; i++) {
        d->buf[i] = value;
    }
    d->sum = value * (float)d->capacity;
    d->count = d->capacity;
    d->idx = 0;
    d->consecutive_count = 0;
}

void cec_swing_detector_reset_empty(cec_swing_detector_t *d)
{
    memset(d->buf, 0, d->capacity * sizeof(float));
    d->sum = 0.0f;
    d->count = 0;
    d->idx = 0;
    d->consecutive_count = 0;
}

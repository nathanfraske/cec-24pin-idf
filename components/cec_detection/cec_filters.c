/*
 * EMA and rolling-median filter primitives.
 */

#include <assert.h>
#include <string.h>
#include "cec_filters.h"

/* ------------------------- EMA ------------------------- */

void ema_init(ema_t *e, float alpha)
{
    e->alpha = alpha;
    e->value = 0.0f;
    e->primed = false;
}

float ema_update(ema_t *e, float sample)
{
    if (!e->primed) {
        e->value = sample;
        e->primed = true;
    } else {
        e->value = e->alpha * sample + (1.0f - e->alpha) * e->value;
    }
    return e->value;
}

float ema_value(const ema_t *e)
{
    return e->value;
}

void ema_reset(ema_t *e)
{
    e->primed = false;
    e->value = 0.0f;
}

/* ----------------------- Median ----------------------- */

void median_init(median_t *m, float *buf, size_t cap)
{
    assert(cap >= 1 && cap <= CEC_MEDIAN_MAX_CAP);
    m->buf = buf;
    m->cap = cap;
    m->count = 0;
    m->head = 0;
}

float median_update(median_t *m, float sample)
{
    m->buf[m->head] = sample;
    m->head = (m->head + 1) % m->cap;
    if (m->count < m->cap) {
        m->count++;
    }

    /* Sort a scratch copy. Window size is small (<= CEC_MEDIAN_MAX_CAP)
     * so the O(N^2) insertion sort is fine and avoids the dependency on
     * qsort's function-pointer call overhead. */
    float scratch[CEC_MEDIAN_MAX_CAP];
    size_t n = m->count;
    memcpy(scratch, m->buf, n * sizeof(float));
    for (size_t i = 1; i < n; i++) {
        float v = scratch[i];
        size_t j = i;
        while (j > 0 && scratch[j - 1] > v) {
            scratch[j] = scratch[j - 1];
            j--;
        }
        scratch[j] = v;
    }
    return scratch[n / 2];
}

size_t median_count(const median_t *m)
{
    return m->count;
}

void median_reset(median_t *m)
{
    m->count = 0;
    m->head = 0;
}

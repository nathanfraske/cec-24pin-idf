/*
 * EMA and rolling-median filter primitives.
 *
 * Pure software, no hardware deps. Caller owns all storage so no heap
 * allocation happens here.
 *
 * EMA: standard first-order IIR. Primes on the first sample so there is
 * no warm-up bias.
 *
 * Median: rolling window of the last N samples, sized at init time with
 * a caller-provided buffer. Returns the median on every update. The
 * implementation sorts a scratch copy on each call, which is fine for the
 * small N (typically <= 9) used by the v0.5.9 detector layers. The window
 * size is capped at CEC_MEDIAN_MAX_CAP.
 */

#pragma once

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CEC_MEDIAN_MAX_CAP 32

/* ------------------------- EMA ------------------------- */

typedef struct {
    float alpha;    /* Smoothing factor, 0 < alpha <= 1. Higher = faster. */
    float value;    /* Current filter output. Undefined until primed. */
    bool  primed;   /* true once at least one sample has been fed in. */
} ema_t;

/*
 * Configure an EMA filter. `alpha` is the smoothing factor; values outside
 * (0, 1] are accepted but produce a non-stable filter. The filter starts
 * unprimed.
 */
void ema_init(ema_t *e, float alpha);

/*
 * Feed a sample and return the new output. The first sample primes the
 * filter (output = sample) so there is no warm-up transient.
 */
float ema_update(ema_t *e, float sample);

/*
 * Current filter output. Undefined if the filter has not yet been primed.
 */
float ema_value(const ema_t *e);

/*
 * Mark the filter unprimed so the next update treats its sample as the
 * first one. `alpha` is preserved.
 */
void ema_reset(ema_t *e);

/* ----------------------- Median ----------------------- */

typedef struct {
    float *buf;     /* Caller-owned ring of size `cap`. */
    size_t cap;     /* Window size. 1 <= cap <= CEC_MEDIAN_MAX_CAP. */
    size_t count;   /* Valid samples in buf, capped at cap. */
    size_t head;    /* Next write index. */
} median_t;

/*
 * Configure a median filter over a caller-provided buffer of `cap` floats.
 * `cap` must be in [1, CEC_MEDIAN_MAX_CAP]; asserts otherwise. Odd values
 * give a true median; even values return the upper of the two middle
 * samples. The window starts empty.
 */
void median_init(median_t *m, float *buf, size_t cap);

/*
 * Add a sample and return the median over the samples seen so far (up to
 * `cap`). Before the window is full the median is taken over the partial
 * window so callers always get a reasonable value.
 */
float median_update(median_t *m, float sample);

/*
 * Number of valid samples currently in the window (0 .. cap).
 */
size_t median_count(const median_t *m);

/*
 * Empty the window. `buf` and `cap` are preserved.
 */
void median_reset(median_t *m);

#ifdef __cplusplus
}
#endif

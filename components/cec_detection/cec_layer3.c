/*
 * Layer 3 rail profile primitive.
 */

#include <math.h>
#include "cec_layer3.h"

#define FAST_ADAPT_RATE    0.01f
#define INITIAL_STD_GUESS  0.01f
#define MIN_STD_FOR_Z      0.001f

void cec_rail_profile_init(cec_rail_profile_t *p)
{
    p->mean = 0.0f;
    p->std_dev = 0.0f;
    p->sample_count = 0;
}

void cec_rail_profile_update(cec_rail_profile_t *p, float x, float adapt_rate)
{
    if (p->sample_count < CEC_PROFILE_FAST_SAMPLES) {
        if (p->sample_count == 0) {
            p->mean = x;
            p->std_dev = INITIAL_STD_GUESS;
        } else {
            float delta = x - p->mean;
            p->mean    += delta * FAST_ADAPT_RATE;
            p->std_dev += (fabsf(delta) - p->std_dev) * FAST_ADAPT_RATE;
        }
    } else {
        float delta = x - p->mean;
        p->mean    += delta * adapt_rate;
        p->std_dev += (fabsf(delta) - p->std_dev) * adapt_rate;
    }
    if (p->sample_count < UINT32_MAX) {
        p->sample_count++;
    }
}

bool cec_rail_profile_is_warm(const cec_rail_profile_t *p)
{
    return p->sample_count > CEC_PROFILE_WARM_SAMPLES;
}

float cec_rail_profile_z_score(const cec_rail_profile_t *p, float x)
{
    if (!cec_rail_profile_is_warm(p)) return 0.0f;
    if (p->std_dev < MIN_STD_FOR_Z)   return 0.0f;
    return (x - p->mean) / p->std_dev;
}

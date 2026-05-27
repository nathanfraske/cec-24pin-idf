/*
 * Layer 1 static voltage threshold detector.
 */

#include <math.h>
#include "cec_layer1.h"

/* Rails below this are treated as "off, not abnormal". */
#define RAIL_OFF_VOLTS 0.1f

static const char *NAMES[] = {
    [CEC_SEV_NONE]     = "NONE",
    [CEC_SEV_WARNING]  = "WARNING",
    [CEC_SEV_CRITICAL] = "CRITICAL",
};

const char *cec_severity_name(cec_severity_t s)
{
    if ((int)s < 0 || (int)s > CEC_SEV_CRITICAL) return "?";
    return NAMES[s];
}

void cec_layer1_init(cec_layer1_detector_t *d,
                     const cec_rail_spec_t *spec,
                     int crit_required)
{
    d->spec = *spec;
    d->crit_required = (crit_required >= 1) ? crit_required : 1;
    d->crit_consecutive = 0;
}

void cec_layer1_reset(cec_layer1_detector_t *d)
{
    d->crit_consecutive = 0;
}

cec_severity_t cec_layer1_update(cec_layer1_detector_t *d, float v_rail)
{
    if (v_rail < RAIL_OFF_VOLTS) {
        d->crit_consecutive = 0;
        return CEC_SEV_NONE;
    }

    float dev = fabsf((v_rail - d->spec.nominal) / d->spec.nominal);

    if (dev > d->spec.crit_band) {
        d->crit_consecutive++;
        if (d->crit_consecutive >= d->crit_required) {
            return CEC_SEV_CRITICAL;
        }
        /* Below the consecutive threshold the rail is still anomalous;
         * report WARNING so the caller can see the transient. */
        return CEC_SEV_WARNING;
    }

    d->crit_consecutive = 0;
    if (dev > d->spec.warn_band) {
        return CEC_SEV_WARNING;
    }
    return CEC_SEV_NONE;
}

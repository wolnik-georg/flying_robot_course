/**
 * rpm_filter.h — per-motor RPM sanity filter (DShot path only for spike rules).
 *
 * Used by rpm_get_all() in traj_iface.c. No firmware dependencies (host-testable).
 */
#ifndef RPM_FILTER_H
#define RPM_FILTER_H

#include <stdbool.h>
#include <stdint.h>

#define RPM_FILTER_DSHOT_ABS_MAX   28000u
#define RPM_FILTER_DSHOT_SLEW_MAX  10000u
#define RPM_FILTER_DSHOT_SENTINEL  0xffffu

/* 0 (DEFAULT) = legacy behaviour, bit-identical to the firmware flown on 2026-10-02/05:
 * the 0xFFFF "no value" marker becomes 0.  1 = hold the last good value instead (prepared
 * 2026-10-08, docs/67; enable only after the NS2 sign test, before the Omar+Iz INDI block,
 * with -DRPM_FILTER_HOLD_SENTINEL=1). */
#ifndef RPM_FILTER_HOLD_SENTINEL
#define RPM_FILTER_HOLD_SENTINEL 0
#endif

/**
 * One filter step: raw logged RPM → value for INDI / thrust reconstruction.
 *
 * @param raw_rpm   Value from log (or 0 if channel invalid).
 * @param rpm_source 0 = optical deck, non-zero = DShot telemetry.
 * @param rpm_prev   Per-motor hold state (updated on accepted samples).
 * @return Filtered RPM for this motor.
 */
static inline uint16_t rpm_filter_step(uint16_t raw_rpm, uint8_t rpm_source,
                                       uint16_t *rpm_prev)
{
    uint16_t v = raw_rpm;

    if (v == RPM_FILTER_DSHOT_SENTINEL) {
#if RPM_FILTER_HOLD_SENTINEL
        if (rpm_source != 0 && *rpm_prev > 0u) {
            return *rpm_prev;
        }
#endif
        v = 0u;
    }

    if (rpm_source != 0 && v > 0u) {
        bool reject = false;
        if (v > RPM_FILTER_DSHOT_ABS_MAX) {
            reject = true;
        } else if (*rpm_prev > 500u) {
            uint16_t lo = v < *rpm_prev ? v : *rpm_prev;
            uint16_t hi = v < *rpm_prev ? *rpm_prev : v;
            if ((uint16_t)(hi - lo) > RPM_FILTER_DSHOT_SLEW_MAX) {
                reject = true;
            }
        }
        if (reject && *rpm_prev > 0u) {
            v = *rpm_prev;
        } else if (!reject) {
            *rpm_prev = v;
        }
    } else if (rpm_source == 0 && v > 0u) {
        *rpm_prev = v;
    }
    return v;
}

#endif /* RPM_FILTER_H */

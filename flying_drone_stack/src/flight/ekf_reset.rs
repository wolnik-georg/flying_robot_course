//! EKF-jump / Kalman-reset detection and yaw angle unwrapping.
//!
//! The Crazyflie EKF can teleport the position / yaw estimate when the
//! Lighthouse system initialises or re-initialises mid-flight.  We detect this
//! by checking the apparent velocity between consecutive 10 ms ticks:
//!
//! * Position step > 0.05 m in one tick ≈ 5 m/s apparent velocity — impossible
//!   for this drone — so it must be a Kalman reset.
//! * Yaw step > 10° in one tick ≈ 1000 deg/s yaw rate — also impossible.
//!
//! Normal flying produces at most ~0.13 m/s (1.3 mm/tick) linear and perhaps
//! 200 deg/s yaw, both well below the thresholds.

use crate::math::Vec3;

// ─────────────────────────────────────────────────────────────────────────────
// Public types
// ─────────────────────────────────────────────────────────────────────────────

/// Which axes experienced a detected EKF jump, plus the measured step sizes.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct EkfResetFlags {
    /// True when the XY position jumped more than `xy_thresh`.
    pub xy: bool,
    /// True when the Z position jumped more than `z_thresh`.
    pub z: bool,
    /// True when the yaw jumped more than `yaw_thresh` degrees.
    pub yaw: bool,
    /// Actual XY step magnitude in metres (useful for log messages).
    pub step_xy_m: f32,
    /// Actual Z step magnitude in metres (useful for log messages).
    pub step_z_m: f32,
    /// Actual yaw step magnitude in degrees, shortest-path (useful for log messages).
    pub step_yaw_deg: f32,
}

impl EkfResetFlags {
    /// `true` if any axis triggered a reset.
    pub fn any(self) -> bool {
        self.xy || self.z || self.yaw
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Jump detection
// ─────────────────────────────────────────────────────────────────────────────

/// Compares the current EKF state against the previous tick and returns flags
/// indicating which axes appear to have teleported.
///
/// # Arguments
/// * `prev_pos`       — EKF position on the previous tick (`None` for the first call).
/// * `cur_pos`        — EKF position on the current tick.
/// * `prev_yaw_deg`   — EKF yaw on the previous tick, in degrees.
/// * `cur_yaw_deg`    — EKF yaw on the current tick, in degrees.
/// * `xy_thresh`      — maximum plausible XY step in metres (default 0.05 m).
/// * `z_thresh`       — maximum plausible Z step in metres (default 0.05 m).
/// * `yaw_thresh_deg` — maximum plausible yaw step in degrees (default 10°).
///
/// Returns [`EkfResetFlags::default()`] (all `false`) when `prev_pos` is `None`.
pub fn detect_ekf_reset(
    prev_pos: Option<Vec3>,
    cur_pos: Vec3,
    prev_yaw_deg: f32,
    cur_yaw_deg: f32,
    xy_thresh: f32,
    z_thresh: f32,
    yaw_thresh_deg: f32,
) -> EkfResetFlags {
    let Some(prev) = prev_pos else {
        return EkfResetFlags::default();
    };

    let step = cur_pos - prev;
    let step_xy = (step.x * step.x + step.y * step.y).sqrt();
    let step_z  = step.z.abs();
    let dyaw    = yaw_wrap_delta(prev_yaw_deg, cur_yaw_deg).abs();

    EkfResetFlags {
        xy:  step_xy > xy_thresh,
        z:   step_z  > z_thresh,
        yaw: dyaw    > yaw_thresh_deg,
        step_xy_m:   step_xy,
        step_z_m:    step_z,
        step_yaw_deg: dyaw,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Yaw unwrapping
// ─────────────────────────────────────────────────────────────────────────────

/// Returns the **shortest-path** signed angular difference `(to − from)` wrapped
/// to `(−180, +180]` degrees.
///
/// ```text
///   yaw_wrap_delta(170.0, -170.0)  →  +20.0   (went CCW 20°, not CW 340°)
///   yaw_wrap_delta(-170.0, 170.0)  →  −20.0
///   yaw_wrap_delta(0.0,   359.0)   →  −1.0
///   yaw_wrap_delta(0.0,     1.0)   →  +1.0
/// ```
pub fn yaw_wrap_delta(from_deg: f32, to_deg: f32) -> f32 {
    let d = to_deg - from_deg;
    // Map to (−180, 180]
    ((d + 180.0).rem_euclid(360.0)) - 180.0
}

/// Converts degrees to radians.
///
/// Convenience wrapper so control-loop code never contains the literal
/// `* std::f32::consts::PI / 180.0` pattern.
#[inline]
pub fn deg_to_rad(deg: f32) -> f32 {
    deg * std::f32::consts::PI / 180.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32) -> bool { (a - b).abs() < 1e-4 }

    // ── detect_ekf_reset ────────────────────────────────────────────────────

    #[test]
    fn first_call_no_reset() {
        let flags = detect_ekf_reset(
            None, Vec3::new(1.0, 2.0, 0.5),
            0.0, 5.0, 0.05, 0.05, 10.0,
        );
        assert!(!flags.any());
    }

    #[test]
    fn small_step_no_reset() {
        let prev = Vec3::new(0.0, 0.0, 0.5);
        let cur  = Vec3::new(0.001, 0.001, 0.501); // 1.4 mm XY, 1 mm Z
        let flags = detect_ekf_reset(Some(prev), cur, 0.0, 0.5, 0.05, 0.05, 10.0);
        assert!(!flags.any(), "flags = {flags:?}");
    }

    #[test]
    fn large_xy_step_triggers_xy_reset() {
        let prev = Vec3::new(0.0, 0.0, 0.5);
        let cur  = Vec3::new(0.5, 0.0, 0.5); // 0.5 m XY step >> 0.05 m threshold
        let flags = detect_ekf_reset(Some(prev), cur, 0.0, 0.0, 0.05, 0.05, 10.0);
        assert!(flags.xy, "xy reset expected");
        assert!(!flags.z && !flags.yaw);
    }

    #[test]
    fn large_z_step_triggers_z_reset() {
        let prev = Vec3::new(0.0, 0.0, 0.5);
        let cur  = Vec3::new(0.0, 0.0, 1.0); // 0.5 m Z step
        let flags = detect_ekf_reset(Some(prev), cur, 0.0, 0.0, 0.05, 0.05, 10.0);
        assert!(flags.z, "z reset expected");
        assert!(!flags.xy && !flags.yaw);
    }

    #[test]
    fn large_yaw_step_triggers_yaw_reset() {
        let pos = Vec3::new(0.0, 0.0, 0.5);
        let flags = detect_ekf_reset(Some(pos), pos, 0.0, 90.0, 0.05, 0.05, 10.0);
        assert!(flags.yaw, "yaw reset expected");
        assert!(!flags.xy && !flags.z);
    }

    #[test]
    fn step_magnitudes_reported_correctly() {
        let prev = Vec3::new(0.0, 0.0, 0.5);
        let cur  = Vec3::new(0.03, 0.04, 0.5); // XY = 0.05 m exactly
        let flags = detect_ekf_reset(Some(prev), cur, 0.0, 0.0, 0.04, 0.05, 10.0);
        // 0.05 > 0.04 → triggers
        assert!(flags.xy);
        assert!(approx_eq(flags.step_xy_m, 0.05));
    }

    // ── yaw_wrap_delta ──────────────────────────────────────────────────────

    #[test]
    fn yaw_wrap_delta_normal_positive() {
        assert!(approx_eq(yaw_wrap_delta(0.0, 90.0), 90.0));
    }

    #[test]
    fn yaw_wrap_delta_normal_negative() {
        assert!(approx_eq(yaw_wrap_delta(90.0, 0.0), -90.0));
    }

    #[test]
    fn yaw_wrap_delta_crossing_180_deg() {
        // From 170° to -170° — shortest path is +20° (CCW), not CW 340°
        assert!(approx_eq(yaw_wrap_delta(170.0, -170.0), 20.0));
    }

    #[test]
    fn yaw_wrap_delta_crossing_minus_180() {
        assert!(approx_eq(yaw_wrap_delta(-170.0, 170.0), -20.0));
    }

    #[test]
    fn yaw_wrap_delta_zero_no_change() {
        assert!(approx_eq(yaw_wrap_delta(45.0, 45.0), 0.0));
    }

    // ── deg_to_rad ──────────────────────────────────────────────────────────

    #[test]
    fn deg_to_rad_180_is_pi() {
        assert!((deg_to_rad(180.0) - std::f32::consts::PI).abs() < 1e-5);
    }
}

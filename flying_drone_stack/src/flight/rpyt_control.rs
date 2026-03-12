//! RPYT control math: convert a geometric controller output into the four scalars
//! that `cf.commander.setpoint_rpyt(roll, pitch, yaw_rate, thrust_u16)` expects.
//!
//! # Coordinate-convention note (Crazyflie RPYT pitch sign)
//! The Crazyflie RPYT commander (firmware `controller_lee.c`) internally does:
//! ```text
//!   q_att = rpy2quat(roll, -pitch_setpoint, yaw)
//! ```
//! i.e. it **negates** the pitch setpoint before converting to a quaternion.
//!
//! In the Crazyflie body frame, **positive pitch = nose-up**, which tilts the
//! thrust vector toward −X (drone accelerates backward in world frame).  We
//! therefore send `pitch_d_raw` **un-negated** so the firmware's negation
//! produces the correct negative (nose-down) attitude for forward acceleration.
//! The full sign chain is:
//! ```text
//!   ep.x > 0  (drone behind target, need +X force)
//!   → f_vec.x > 0
//!   → pitch_d_raw = atan2(f_vec.x, f_vec.z) > 0
//!   → pitch_d_cmd = +pitch_d_raw > 0          ← this module (no negation)
//!   → firmware negates: actual_pitch = −pitch_d_raw < 0
//!   → negative pitch → nose-down → thrust tilts toward +world_x
//!   → drone accelerates in +X  ✓
//! ```

use crate::math::Vec3;
use crate::controller::{GeometricController, TrajectoryReference};
use crate::dynamics::MultirotorParams;

// ─────────────────────────────────────────────────────────────────────────────
// Public types
// ─────────────────────────────────────────────────────────────────────────────

/// The four values passed to `cf.commander.setpoint_rpyt`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RpytCmd {
    /// Roll command in degrees.  Positive = right side down.
    pub roll_deg: f32,
    /// Pitch command in degrees **after** the firmware legacy-sign correction.
    /// Already negated compared to `pitch_d_raw`; ready to send verbatim to the
    /// Crazyflie RPYT commander.
    pub pitch_deg: f32,
    /// Yaw-rate command in degrees per second.
    pub yaw_rate_deg_s: f32,
    /// Thrust mapped to PWM units `[0, 65535]`, clamped to `[10 000, 60 000]`.
    pub thrust_pwm: u16,
}

// ─────────────────────────────────────────────────────────────────────────────
// Force vector
// ─────────────────────────────────────────────────────────────────────────────

/// Computes the desired **world-frame force vector** `F = m * (a_ff + kp*ep + kv*ev + ki_pos*i + g*ez)`.
///
/// This re-derives the internal `thrust_force` from the geometric controller so
/// that the *direction* (needed for roll/pitch commands) is available alongside
/// the *magnitude* (returned by `compute_control` as `control.thrust`).
///
/// # Arguments
/// * `reference`   — trajectory reference (provides `acceleration`, `position`, `velocity`)
/// * `ep`          — position error = `reference.position - state.position`
/// * `ev`          — velocity error = `reference.velocity - state.velocity`
/// * `i_pos`       — current position integral from the controller
/// * `controller`  — read-only access to gains `kp`, `kv`, `ki_pos`
/// * `params`      — read-only access to `mass` and `gravity`
pub fn compute_force_vector(
    reference: &TrajectoryReference,
    ep: Vec3,
    ev: Vec3,
    i_pos: Vec3,
    controller: &GeometricController,
    params: &MultirotorParams,
) -> Vec3 {
    let desired_accel = reference.acceleration
        + Vec3::new(controller.kp.x * ep.x, controller.kp.y * ep.y, controller.kp.z * ep.z)
        + Vec3::new(controller.kv.x * ev.x, controller.kv.y * ev.y, controller.kv.z * ev.z)
        + Vec3::new(
            controller.ki_pos.x * i_pos.x,
            controller.ki_pos.y * i_pos.y,
            controller.ki_pos.z * i_pos.z,
        )
        + Vec3::new(0.0, 0.0, params.gravity);

    desired_accel * params.mass
}

// ─────────────────────────────────────────────────────────────────────────────
// Roll / pitch from force vector
// ─────────────────────────────────────────────────────────────────────────────

/// Decomposes a world-frame force vector into clamped RPYT roll/pitch commands.
///
/// The force direction tells us which way to tilt the drone:
/// ```text
///   pitch_d_raw = atan2(F_x, F_z)   — positive pitch tilts body_z toward +world_x
///   roll_d_raw  = atan2(−F_y, F_z)  — positive roll  tilts body_z toward +world_y (right-down)
/// ```
///
/// `pitch_d_cmd` is passed **un-negated** so the firmware's own negation in
/// `rpy2quat(roll, -pitch, yaw)` produces the correct nose-down attitude
/// (see module-level doc).
///
/// Returns `(roll_cmd_deg, pitch_cmd_deg, roll_d_raw_deg, pitch_d_raw_deg)`.
/// The raw values are returned so the caller can decide on anti-windup rollback.
pub fn force_vector_to_rpyt(
    f_vec: Vec3,
    max_tilt_deg: f32,
) -> (f32, f32, f32, f32) {
    let pitch_d_raw = f_vec.x.atan2(f_vec.z).to_degrees();
    let roll_d_raw  = (-f_vec.y).atan2(f_vec.z).to_degrees();

    // Pass pitch un-negated — firmware negates it in rpy2quat (see module doc).
    let pitch_d_cmd = pitch_d_raw.clamp(-max_tilt_deg, max_tilt_deg);
    let roll_d_cmd  = roll_d_raw.clamp(-max_tilt_deg, max_tilt_deg);

    (roll_d_cmd, pitch_d_cmd, roll_d_raw, pitch_d_raw)
}

/// Returns `true` if either raw tilt angle exceeded `max_tilt_deg`.
/// Used to decide whether to roll back the position integral (anti-windup).
pub fn tilt_saturated(roll_d_raw: f32, pitch_d_raw: f32, max_tilt_deg: f32) -> bool {
    roll_d_raw.abs() > max_tilt_deg || pitch_d_raw.abs() > max_tilt_deg
}

// ─────────────────────────────────────────────────────────────────────────────
// Thrust → PWM
// ─────────────────────────────────────────────────────────────────────────────

/// Maps a thrust in **Newtons** to a PWM value in `[0, 65535]`.
///
/// The mapping is linear through the hover point:
/// ```text
///   pwm = thrust_n / hover_thrust_n * hover_pwm
/// ```
/// where `hover_thrust_n = params.mass * params.gravity`.
///
/// The result is clamped to `[pwm_min, pwm_max]`.  The raw (unclamped) value is
/// returned alongside the clamped `u16` so the caller can detect saturation for
/// anti-windup.
///
/// Typical safe range: `[10 000, 60 000]`.
pub fn thrust_to_pwm(
    thrust_n: f32,
    hover_thrust_n: f32,
    hover_pwm: f32,
    pwm_min: f32,
    pwm_max: f32,
) -> (u16, f32) {
    let raw = thrust_n / hover_thrust_n * hover_pwm;
    let clamped = raw.clamp(pwm_min, pwm_max) as u16;
    (clamped, raw)
}

// ─────────────────────────────────────────────────────────────────────────────
// Yaw rate command
// ─────────────────────────────────────────────────────────────────────────────

/// Computes a yaw-rate command in degrees/s from a yaw **reference** (radians)
/// and the current EKF yaw (degrees).
///
/// The proportional gain `kp_yaw` converts angle error → rate command.
/// The result is clamped to `[-max_yaw_rate_deg_s, +max_yaw_rate_deg_s]`.
///
/// The error is *not* wrapped here because the reference yaw is typically near 0
/// and the current yaw is bounded to ±180°; the clamp handles any extreme case.
pub fn yaw_rate_cmd(
    ref_yaw_rad: f32,
    current_yaw_deg: f32,
    kp_yaw: f32,
    max_yaw_rate_deg_s: f32,
) -> f32 {
    (ref_yaw_rad.to_degrees() - current_yaw_deg)
        .clamp(-max_yaw_rate_deg_s, max_yaw_rate_deg_s)
        * kp_yaw
}

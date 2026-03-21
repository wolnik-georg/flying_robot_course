//! Converts raw EKF log scalars (degrees, m/s, …) into a [`MultirotorState`].
//!
//! All three control loops (`my_hover`, `my_circle`, `my_figure8`) contain the
//! same 12-line orientation-quaternion block.  Extracting it here:
//!
//! 1. makes it testable against known angles,
//! 2. removes six opportunities for a copy-paste sign error,
//! 3. keeps the control loops focused on control policy, not unit conversion.

use crate::math::{Vec3, Quat};
use crate::dynamics::MultirotorState;

/// Builds a [`MultirotorState`] from the scalar values that the Crazyflie EKF
/// log provides.
///
/// # Conventions
/// * `roll_deg`, `pitch_deg`, `yaw_deg` — EKF Euler angles in **degrees**,
///   ZYX Tait-Bryan convention (`stabilizer.roll/pitch/yaw`).
/// * `gyro_{x,y,z}_deg_s` — body-frame angular rates in **deg/s** (`gyro.x/y/z`).
/// * All position and velocity inputs are in **metres / m/s** (no conversion needed).
///
/// # Quaternion construction
/// Uses the ZYX decomposition:  `q = q_yaw * q_pitch * q_roll`  (yaw applied first
/// in the *world frame*, then pitch, then roll), matching the firmware EKF output
/// convention.
///
/// Confirmed convention: positive pitch (nose-up) rotates `body_z` toward
/// `+world_x`.  See `test_quaternion_reconstruction_from_rpy` for the full
/// verification and the pitch sign-chain comment.
pub fn build_state(
    pos_x: f32, pos_y: f32, pos_z: f32,
    vel_x: f32, vel_y: f32, vel_z: f32,
    roll_deg: f32, pitch_deg: f32, yaw_deg: f32,
    gyro_x_deg_s: f32, gyro_y_deg_s: f32, gyro_z_deg_s: f32,
) -> MultirotorState {
    let pi = std::f32::consts::PI;

    let roll_rad  = roll_deg  * pi / 180.0;
    let pitch_rad = pitch_deg * pi / 180.0;
    let yaw_rad   = yaw_deg   * pi / 180.0;

    // ZYX quaternion: q_yaw * q_pitch * q_roll
    let q_yaw   = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw_rad);
    let q_pitch = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), pitch_rad);
    let q_roll  = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll_rad);
    let orientation = (q_yaw * q_pitch * q_roll).normalize();

    MultirotorState {
        position: Vec3::new(pos_x, pos_y, pos_z),
        velocity: Vec3::new(vel_x, vel_y, vel_z),
        orientation,
        angular_velocity: Vec3::new(
            gyro_x_deg_s * pi / 180.0,
            gyro_y_deg_s * pi / 180.0,
            gyro_z_deg_s * pi / 180.0,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Quat;

    fn approx_eq(a: f32, b: f32) -> bool { (a - b).abs() < 1e-4 }

    #[test]
    fn zero_angles_gives_identity_orientation() {
        let state = build_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let q = state.orientation;
        assert!(approx_eq(q.w, 1.0) && approx_eq(q.x, 0.0)
            && approx_eq(q.y, 0.0) && approx_eq(q.z, 0.0),
            "expected identity, got ({},{},{},{})", q.w, q.x, q.y, q.z);
    }

    #[test]
    fn position_and_velocity_passed_through() {
        let state = build_state(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(approx_eq(state.position.x, 1.0));
        assert!(approx_eq(state.position.y, 2.0));
        assert!(approx_eq(state.position.z, 3.0));
        assert!(approx_eq(state.velocity.x, 4.0));
        assert!(approx_eq(state.velocity.y, 5.0));
        assert!(approx_eq(state.velocity.z, 6.0));
    }

    #[test]
    fn gyro_converted_from_deg_s_to_rad_s() {
        let state = build_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            180.0, 0.0, 0.0);
        let pi = std::f32::consts::PI;
        assert!(approx_eq(state.angular_velocity.x, pi),
            "expected pi rad/s, got {}", state.angular_velocity.x);
    }

    #[test]
    fn pure_yaw_orientation_rotates_correctly() {
        // 90° yaw: body_x should point in world_y direction (right-hand rule)
        let state = build_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 0.0);
        let body_x = state.orientation.rotate_vector(Vec3::new(1.0, 0.0, 0.0));
        assert!(approx_eq(body_x.x, 0.0) && approx_eq(body_x.y, 1.0) && approx_eq(body_x.z, 0.0),
            "body_x after 90° yaw = ({},{},{})", body_x.x, body_x.y, body_x.z);
    }

    #[test]
    fn orientation_is_unit_quaternion() {
        let state = build_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0, -10.0, 45.0, 0.0, 0.0, 0.0);
        let q = state.orientation;
        let norm_sq = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
        assert!(approx_eq(norm_sq, 1.0), "norm² = {norm_sq}");
    }
}

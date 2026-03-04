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

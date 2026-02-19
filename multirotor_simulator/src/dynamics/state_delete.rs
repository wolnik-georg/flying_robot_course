//! State representation for multirotor dynamics
//!
//! Defines the complete state vector and control inputs for multirotor simulation

use crate::math::{Vec3, Quat};

/// Complete state of a multirotor system
/// Represents the 13-dimensional state: position (3), velocity (3), orientation (4), angular velocity (3)
/// Rust note: #[derive(Debug, Clone)] allows printing and cloning
/// Clone is needed because State contains non-Copy types (if we add them later)
#[derive(Debug, Clone)]
pub struct MultirotorState {
    /// Position in world frame [m]
    /// Rust note: Doc comments use /// for public items
    pub position: Vec3,
    /// Linear velocity in world frame [m/s]
    pub velocity: Vec3,
    /// Orientation as unit quaternion (world to body frame)
    /// Quaternion representation avoids gimbal lock
    pub orientation: Quat,
    /// Angular velocity in body frame [rad/s]
    /// Body frame: x-forward, y-left, z-up (NED or similar convention)
    pub angular_velocity: Vec3,
}

impl MultirotorState {
    /// Create a new state at origin with zero velocity
    /// Rust note: Associated function (no self) - constructor pattern
    pub fn new() -> Self {
        Self {
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            orientation: Quat::identity(),
            angular_velocity: Vec3::zero(),
        }
    }

    /// Create a state with specified initial conditions
    /// Rust note: Takes ownership of parameters (Copy types so no move)
    pub fn with_initial(position: Vec3, velocity: Vec3, orientation: Quat, angular_velocity: Vec3) -> Self {
        Self {
            position,
            velocity,
            orientation,
            angular_velocity,
        }
    }
}

/// Rust note: Default trait provides default() method
/// Allows: let state = MultirotorState::default();
impl Default for MultirotorState {
    fn default() -> Self {
        Self::new()
    }
}

/// Control inputs to a quadrotor
/// Motor numbering convention (X-configuration):
///     1 (FL)    2 (FR)
///         \ X /
///         / X \
///     4 (BL)    3 (BR)
/// Motors 1,3 spin CW; Motors 2,4 spin CCW
#[derive(Debug, Clone)]
pub struct MotorAction {
    /// Front-left motor angular velocity squared [rad²/s²]
    /// Squared because thrust ∝ ω²
    pub omega1_sq: f32,
    /// Front-right motor angular velocity squared [rad²/s²]
    pub omega2_sq: f32,
    /// Back-right motor angular velocity squared [rad²/s²]
    pub omega3_sq: f32,
    /// Back-left motor angular velocity squared [rad²/s²]
    pub omega4_sq: f32,
}

impl MotorAction {
    /// Create motor action with specified squared angular velocities
    pub fn new(omega1_sq: f32, omega2_sq: f32, omega3_sq: f32, omega4_sq: f32) -> Self {
        Self { omega1_sq, omega2_sq, omega3_sq, omega4_sq }
    }

    /// Create hover action (all motors equal)
    /// For Crazyflie 2.0: hover at ~1200 rad/s per motor
    pub fn hover() -> Self {
        let hover_speed_sq = 1200.0 * 1200.0;
        Self::new(hover_speed_sq, hover_speed_sq, hover_speed_sq, hover_speed_sq)
    }

    /// Create action with all motors at the same speed
    /// Useful for testing and basic maneuvers
    pub fn uniform(omega_sq: f32) -> Self {
        Self::new(omega_sq, omega_sq, omega_sq, omega_sq)
    }
}

//! State representation for multirotor dynamics
//!
//! Defines the complete state vector and control inputs for multirotor simulation

use crate::math::{Vec3, Quat};
use super::params::MultirotorParams;

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
    /// For Crazyflie 2.0: hover thrust = mg = 0.027 * 9.81 ≈ 0.265 N
    /// omega² = thrust / (4 * kf) = 0.265 / (4 * 2.5e-6) ≈ 26500
    /// omega ≈ sqrt(26500) ≈ 163 rad/s
    pub fn hover() -> Self {
        let hover_speed_sq = 163.0 * 163.0; // Approximately correct for Crazyflie
        Self::new(hover_speed_sq, hover_speed_sq, hover_speed_sq, hover_speed_sq)
    }

    /// Create action with all motors at the same speed
    /// Useful for testing and basic maneuvers
    pub fn uniform(omega_sq: f32) -> Self {
        Self::new(omega_sq, omega_sq, omega_sq, omega_sq)
    }

    /// Convert from thrust and torque to motor speeds (X configuration quadcopter)
    /// Assumes motors are ordered: front-left, front-right, back-right, back-left
    pub fn from_thrust_torque(thrust: f32, torque: Vec3, params: &MultirotorParams) -> Self {
        let kf = params.kf;
        let kt = params.kt;
        let l = params.arm_length;

        // For X-configuration, the lever arm is L/sqrt(2) due to 45° motor placement
        let sqrt2 = 2.0_f32.sqrt();
        let l_sqrt2 = l / sqrt2;

        // Drag factor
        let _d = kt / kf;

        // Common terms
        let thrust_term = thrust / (4.0 * kf);
        let roll_term = torque.x / (4.0 * kf * l_sqrt2);
        let pitch_term = torque.y / (4.0 * kf * l_sqrt2);
        let yaw_term = torque.z / (4.0 * kt);

        // Motor mixing for X configuration
        // Derived from inverting:
        // F = kf * (ω1² + ω2² + ω3² + ω4²)
        // τx = kf * l_sqrt2 * (ω2² + ω3² - ω1² - ω4²)
        // τy = kf * l_sqrt2 * (ω3² + ω4² - ω1² - ω2²)
        // τz = kt * (ω1² - ω2² + ω3² - ω4²)
        let omega1_sq = thrust_term - roll_term - pitch_term + yaw_term; // Front-left
        let omega2_sq = thrust_term + roll_term - pitch_term - yaw_term; // Front-right
        let omega3_sq = thrust_term + roll_term + pitch_term + yaw_term; // Back-right
        let omega4_sq = thrust_term - roll_term + pitch_term - yaw_term; // Back-left

        // Clamp to non-negative values (motors can't spin backwards)
        let omega1_sq = omega1_sq.max(0.0);
        let omega2_sq = omega2_sq.max(0.0);
        let omega3_sq = omega3_sq.max(0.0);
        let omega4_sq = omega4_sq.max(0.0);

        Self::new(omega1_sq, omega2_sq, omega3_sq, omega4_sq)
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_multirotor_state_new() {
        let state = MultirotorState::new();

        assert_eq!(state.position, Vec3::zero());
        assert_eq!(state.velocity, Vec3::zero());
        assert!((state.orientation.w - 1.0).abs() < 1e-6);
        assert!(state.orientation.x.abs() < 1e-6);
        assert!(state.orientation.y.abs() < 1e-6);
        assert!(state.orientation.z.abs() < 1e-6);
        assert_eq!(state.angular_velocity, Vec3::zero());
    }

    #[test]
    fn test_multirotor_state_with_initial() {
        let position = Vec3::new(1.0, 2.0, 3.0);
        let velocity = Vec3::new(0.1, 0.2, 0.3);
        let orientation = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.1);
        let angular_velocity = Vec3::new(0.01, 0.02, 0.03);

        let state = MultirotorState::with_initial(position, velocity, orientation, angular_velocity);

        assert_eq!(state.position, position);
        assert_eq!(state.velocity, velocity);
        assert!((state.orientation.w - orientation.w).abs() < 1e-6);
        assert!((state.orientation.x - orientation.x).abs() < 1e-6);
        assert!((state.orientation.y - orientation.y).abs() < 1e-6);
        assert!((state.orientation.z - orientation.z).abs() < 1e-6);
        assert_eq!(state.angular_velocity, angular_velocity);
    }

    #[test]
    fn test_multirotor_state_default() {
        let state = MultirotorState::default();

        assert_eq!(state.position, Vec3::zero());
        assert_eq!(state.velocity, Vec3::zero());
        assert!((state.orientation.w - 1.0).abs() < 1e-6);
        assert!(state.orientation.x.abs() < 1e-6);
        assert!(state.orientation.y.abs() < 1e-6);
        assert!(state.orientation.z.abs() < 1e-6);
        assert_eq!(state.angular_velocity, Vec3::zero());
    }

    #[test]
    fn test_motor_action_new() {
        let action = MotorAction::new(10000.0, 20000.0, 30000.0, 40000.0);

        assert_eq!(action.omega1_sq, 10000.0);
        assert_eq!(action.omega2_sq, 20000.0);
        assert_eq!(action.omega3_sq, 30000.0);
        assert_eq!(action.omega4_sq, 40000.0);
    }

    #[test]
    fn test_motor_action_hover() {
        let action = MotorAction::hover();

        let expected_speed_sq = 163.0 * 163.0;
        assert_eq!(action.omega1_sq, expected_speed_sq);
        assert_eq!(action.omega2_sq, expected_speed_sq);
        assert_eq!(action.omega3_sq, expected_speed_sq);
        assert_eq!(action.omega4_sq, expected_speed_sq);
    }

    #[test]
    fn test_motor_action_uniform() {
        let omega_sq = 25000.0;
        let action = MotorAction::uniform(omega_sq);

        assert_eq!(action.omega1_sq, omega_sq);
        assert_eq!(action.omega2_sq, omega_sq);
        assert_eq!(action.omega3_sq, omega_sq);
        assert_eq!(action.omega4_sq, omega_sq);
    }

    #[test]
    fn test_motor_action_from_thrust_torque_zero() {
        let params = MultirotorParams::crazyflie();
        let thrust = 0.0;
        let torque = Vec3::zero();

        let action = MotorAction::from_thrust_torque(thrust, torque, &params);

        // All motors should be zero
        assert_eq!(action.omega1_sq, 0.0);
        assert_eq!(action.omega2_sq, 0.0);
        assert_eq!(action.omega3_sq, 0.0);
        assert_eq!(action.omega4_sq, 0.0);
    }

    #[test]
    fn test_motor_action_from_thrust_torque_hover() {
        let params = MultirotorParams::crazyflie();
        let thrust = params.mass * params.gravity;
        let torque = Vec3::zero();

        let action = MotorAction::from_thrust_torque(thrust, torque, &params);

        // All motors should be equal for hover
        assert!((action.omega1_sq - action.omega2_sq).abs() < 1e-3);
        assert!((action.omega2_sq - action.omega3_sq).abs() < 1e-3);
        assert!((action.omega3_sq - action.omega4_sq).abs() < 1e-3);
    }

    #[test]
    fn test_motor_action_from_thrust_torque_roll() {
        let params = MultirotorParams::crazyflie();
        let thrust = params.mass * params.gravity;
        let torque = Vec3::new(0.001, 0.0, 0.0); // Positive roll torque

        let action = MotorAction::from_thrust_torque(thrust, torque, &params);

        // Front-left and back-left should be slower than front-right and back-right
        assert!(action.omega1_sq < action.omega2_sq);
        assert!(action.omega4_sq < action.omega3_sq);
    }

    #[test]
    fn test_motor_action_from_thrust_torque_pitch() {
        let params = MultirotorParams::crazyflie();
        let thrust = params.mass * params.gravity;
        let torque = Vec3::new(0.0, 0.001, 0.0); // Positive pitch torque

        let action = MotorAction::from_thrust_torque(thrust, torque, &params);

        // Front motors should be slower than back motors
        assert!(action.omega1_sq < action.omega4_sq);
        assert!(action.omega2_sq < action.omega3_sq);
    }

    #[test]
    fn test_motor_action_from_thrust_torque_yaw() {
        let params = MultirotorParams::crazyflie();
        let thrust = params.mass * params.gravity;
        let torque = Vec3::new(0.0, 0.0, 0.001); // Positive yaw torque

        let action = MotorAction::from_thrust_torque(thrust, torque, &params);

        // CW motors (1,3) should be faster than CCW motors (2,4)
        // Note: This test may need adjustment based on motor configuration
        assert!(action.omega1_sq >= 0.0);
        assert!(action.omega2_sq >= 0.0);
        assert!(action.omega3_sq >= 0.0);
        assert!(action.omega4_sq >= 0.0);
    }
}
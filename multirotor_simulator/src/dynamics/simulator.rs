//! Multirotor dynamics simulator with pluggable integration methods
//!
//! Core simulator implementing the Strategy pattern for numerical integration

use crate::math::{Vec3, Quat};
use super::{MultirotorState, MultirotorParams, MotorAction};

/// Trait for numerical integration methods
/// Rust note: Traits define shared behavior (like interfaces in other languages)
/// Any type implementing this trait can be used as an integrator
pub trait Integrator {
    /// Perform one integration step
    /// Takes mutable reference to state, allowing in-place updates
    /// Rust note: &mut allows modification, & prevents modification
    fn step(&self, params: &MultirotorParams, state: &mut MultirotorState, action: &MotorAction);
}

/// Main multirotor simulator
/// Uses dependency injection via the Integrator trait
/// Rust note: Box<dyn Integrator> is a heap-allocated trait object (dynamic dispatch)
/// Allows runtime polymorphism - different integrators can be swapped at runtime
pub struct MultirotorSimulator {
    params: MultirotorParams,
    state: MultirotorState,
    integrator: Box<dyn Integrator>,  // Rust note: "dyn" means dynamic dispatch (virtual calls)
}

impl MultirotorSimulator {
    /// Create a new simulator with specified parameters and integrator
    /// Rust note: Box<dyn Integrator> accepts any type implementing Integrator trait
    /// Example: Box::new(RK4Integrator), Box::new(EulerIntegrator), etc.
    pub fn new(params: MultirotorParams, integrator: Box<dyn Integrator>) -> Self {
        Self {
            params,
            state: MultirotorState::new(),
            integrator,
        }
    }

    /// Get current state (immutable)
    /// Rust note: Returns reference (&) to avoid copying large state
    pub fn state(&self) -> &MultirotorState {
        &self.state
    }

    /// Get mutable state
    /// Allows external modification of state (e.g., for resetting position)
    pub fn state_mut(&mut self) -> &mut MultirotorState {
        &mut self.state
    }

    /// Set state
    /// Replace current state with a new one
    pub fn set_state(&mut self, state: MultirotorState) {
        self.state = state;
    }

    /// Get parameters
    /// Access to physical constants and configuration
    pub fn params(&self) -> &MultirotorParams {
        &self.params
    }

    /// Perform one simulation step
    /// Advances state by dt using the configured integrator
    /// Rust note: Delegates to integrator via trait method (polymorphism)
    pub fn step(&mut self, action: &MotorAction) {
        self.integrator.step(&self.params, &mut self.state, action);
    }

    /// Simulate multiple steps
    /// Returns trajectory as vector of states
    /// Rust note: Vec<T> is a growable array (like std::vector in C++)
    pub fn simulate(&mut self, action: &MotorAction, num_steps: usize) -> Vec<MultirotorState> {
        let mut states = Vec::with_capacity(num_steps + 1);  // Pre-allocate memory
        states.push(self.state.clone());  // Save initial state

        for _ in 0..num_steps {
            self.step(action);
            states.push(self.state.clone());  // Save state after each step
        }

        states
    }

    /// Reset to initial state
    /// Useful for running multiple experiments
    pub fn reset(&mut self) {
        self.state = MultirotorState::new();
    }
}

/// Helper function to compute system derivatives
/// Computes dx/dt given current state and control input
/// Returns: (velocity, linear_acc, quaternion_deriv, angular_acc)
///
/// Equations of motion:
/// - dp/dt = v (velocity)
/// - dv/dt = (R*[0,0,F] + [0,0,-mg]) / m (linear acceleration)
/// - dq/dt = 0.5 * q ⊗ ω (quaternion derivative)
/// - dω/dt = J⁻¹(τ - ω × Jω) (angular acceleration, Euler's equation)
///
/// Rust note: pub(crate) means visible within this crate but not externally
/// This is an internal helper function used by integrators
pub(crate) fn compute_derivatives(
    params: &MultirotorParams,
    state: &MultirotorState,
    action: &MotorAction,
) -> (Vec3, Vec3, Quat, Vec3) {
    // Get forces and torques from motor speeds
    let (f_total, tau) = params.motor_speeds_to_forces_torques(action);

    // Thrust in body frame (quadrotor z-axis points up in body frame)
    let thrust_body = Vec3::new(0.0, 0.0, f_total);
    
    // Rotate thrust to world frame using current orientation
    // This accounts for the aircraft's tilt
    let thrust_world = state.orientation.rotate_vector(thrust_body);
    
    // Gravity force in world frame (always points down)
    let gravity = Vec3::new(0.0, 0.0, -params.gravity * params.mass);
    
    // Linear acceleration: a = F_net / m
    let linear_acc = (thrust_world + gravity) * (1.0 / params.mass);
    
    // Angular acceleration using Euler's equation
    let angular_acc = params.angular_acceleration(state.angular_velocity, tau);
    
    // Quaternion derivative: dq/dt = 0.5 * q ⊗ ω
    let ori_deriv = state.orientation.derivative(state.angular_velocity);

    // Return all four derivative components
    (state.velocity, linear_acc, ori_deriv, angular_acc)
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::integration::EulerIntegrator;

    #[test]
    fn test_multirotor_simulator_new() {
        let params = MultirotorParams::crazyflie();
        let integrator = Box::new(EulerIntegrator);
        let simulator = MultirotorSimulator::new(params.clone(), integrator);

        assert_eq!(simulator.params().mass, params.mass);
        assert_eq!(simulator.state().position, Vec3::zero());
        assert!((simulator.state().orientation.w - 1.0).abs() < 1e-6);
        assert!(simulator.state().orientation.x.abs() < 1e-6);
        assert!(simulator.state().orientation.y.abs() < 1e-6);
        assert!(simulator.state().orientation.z.abs() < 1e-6);
    }

    #[test]
    fn test_multirotor_simulator_state_access() {
        let params = MultirotorParams::crazyflie();
        let integrator = Box::new(EulerIntegrator);
        let mut simulator = MultirotorSimulator::new(params, integrator);

        // Test immutable access
        let state = simulator.state();
        assert_eq!(state.position, Vec3::zero());

        // Test mutable access
        let state_mut = simulator.state_mut();
        state_mut.position = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(simulator.state().position, Vec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn test_multirotor_simulator_set_state() {
        let params = MultirotorParams::crazyflie();
        let integrator = Box::new(EulerIntegrator);
        let mut simulator = MultirotorSimulator::new(params, integrator);

        let new_state = MultirotorState::with_initial(
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(0.1, 0.2, 0.3),
            Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.5),
            Vec3::new(0.01, 0.02, 0.03),
        );

        simulator.set_state(new_state.clone());
        assert_eq!(simulator.state().position, new_state.position);
        assert!((simulator.state().orientation.w - new_state.orientation.w).abs() < 1e-6);
        assert!((simulator.state().orientation.x - new_state.orientation.x).abs() < 1e-6);
        assert!((simulator.state().orientation.y - new_state.orientation.y).abs() < 1e-6);
        assert!((simulator.state().orientation.z - new_state.orientation.z).abs() < 1e-6);
    }

    #[test]
    fn test_multirotor_simulator_reset() {
        let params = MultirotorParams::crazyflie();
        let integrator = Box::new(EulerIntegrator);
        let mut simulator = MultirotorSimulator::new(params, integrator);

        // Modify state
        simulator.state_mut().position = Vec3::new(1.0, 2.0, 3.0);

        // Reset
        simulator.reset();

        // Should be back to initial state
        assert_eq!(simulator.state().position, Vec3::zero());
        assert!((simulator.state().orientation.w - 1.0).abs() < 1e-6);
        assert!(simulator.state().orientation.x.abs() < 1e-6);
        assert!(simulator.state().orientation.y.abs() < 1e-6);
        assert!(simulator.state().orientation.z.abs() < 1e-6);
    }

    #[test]
    fn test_multirotor_simulator_step() {
        let params = MultirotorParams::crazyflie();
        let integrator = Box::new(EulerIntegrator);
        let mut simulator = MultirotorSimulator::new(params, integrator);

        let initial_position = simulator.state().position.clone();

        // Take a step with hover action
        let action = MotorAction::hover();
        simulator.step(&action);

        // Position should change (either fall or rise slightly due to numerical precision)
        assert!(simulator.state().position.z != initial_position.z);
    }

    #[test]
    fn test_multirotor_simulator_simulate() {
        let params = MultirotorParams::crazyflie();
        let integrator = Box::new(EulerIntegrator);
        let mut simulator = MultirotorSimulator::new(params, integrator);

        let action = MotorAction::hover();
        let num_steps = 10;
        let trajectory = simulator.simulate(&action, num_steps);

        // Should have initial state + num_steps states
        assert_eq!(trajectory.len(), num_steps + 1);

        // First state should be initial
        assert_eq!(trajectory[0].position, Vec3::zero());

        // States should change over time (either falling or rising slightly)
        assert!(trajectory[1].position != trajectory[0].position);
    }

    #[test]
    fn test_compute_derivatives_hover() {
        let params = MultirotorParams::crazyflie();
        let state = MultirotorState::new();
        let action = MotorAction::hover();

        let (vel_deriv, lin_acc, ori_deriv, ang_acc) = compute_derivatives(&params, &state, &action);

        // Velocity derivative should be current velocity (zero)
        assert_eq!(vel_deriv, Vec3::zero());

        // Linear acceleration should be approximately zero (thrust balances gravity)
        assert!(lin_acc.norm() < 0.1); // Small residual due to numerical precision

        // Orientation derivative should be zero (no rotation)
        assert!(ori_deriv.norm() < 1e-6);

        // Angular acceleration should be zero (balanced torques)
        assert!(ang_acc.norm() < 1e-6);
    }

    #[test]
    fn test_compute_derivatives_freefall() {
        let params = MultirotorParams::crazyflie();
        let state = MultirotorState::new();
        let action = MotorAction::new(0.0, 0.0, 0.0, 0.0); // No thrust

        let (vel_deriv, lin_acc, ori_deriv, ang_acc) = compute_derivatives(&params, &state, &action);

        // Velocity derivative should be current velocity (zero)
        assert_eq!(vel_deriv, Vec3::zero());

        // Linear acceleration should be gravity (downward)
        let expected_gravity = Vec3::new(0.0, 0.0, -params.gravity);
        assert!((lin_acc - expected_gravity).norm() < 1e-6);

        // Orientation derivative should be zero (no rotation)
        assert!(ori_deriv.norm() < 1e-6);

        // Angular acceleration should be zero (no torques)
        assert!(ang_acc.norm() < 1e-6);
    }

    #[test]
    fn test_compute_derivatives_with_velocity() {
        let params = MultirotorParams::crazyflie();
        let mut state = MultirotorState::new();
        state.velocity = Vec3::new(1.0, 2.0, 3.0);
        let action = MotorAction::hover();

        let (vel_deriv, lin_acc, _ori_deriv, _ang_acc) = compute_derivatives(&params, &state, &action);

        // Velocity derivative should be current velocity
        assert_eq!(vel_deriv, state.velocity);

        // Linear acceleration should be approximately zero (hover)
        assert!(lin_acc.norm() < 0.1);
    }

    #[test]
    fn test_compute_derivatives_with_rotation() {
        let params = MultirotorParams::crazyflie();
        let mut state = MultirotorState::new();
        state.orientation = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::PI / 4.0);
        state.angular_velocity = Vec3::new(0.0, 0.0, 1.0);
        let action = MotorAction::hover();

        let (_vel_deriv, _lin_acc, ori_deriv, ang_acc) = compute_derivatives(&params, &state, &action);

        // Orientation derivative should be non-zero
        assert!(ori_deriv.norm() > 0.0);

        // Angular acceleration should be approximately zero (hover)
        assert!(ang_acc.norm() < 1e-6);
    }

    #[test]
    fn test_compute_derivatives_with_tilt() {
        let params = MultirotorParams::crazyflie();
        let mut state = MultirotorState::new();
        // Tilt forward by 30 degrees
        state.orientation = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), std::f32::consts::PI / 6.0);
        let action = MotorAction::hover();

        let (_vel_deriv, lin_acc, _ori_deriv, _ang_acc) = compute_derivatives(&params, &state, &action);

        // Linear acceleration should be different from hover (tilted thrust produces different acceleration)
        let hover_acc = Vec3::new(0.0, 0.0, -params.gravity); // Approximate hover acceleration
        assert!((lin_acc - hover_acc).norm() > 0.01); // Should be noticeably different from hover
    }
}
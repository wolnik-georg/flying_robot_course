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
    pub fn state_mut(&mut self) -> &MultirotorState {
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

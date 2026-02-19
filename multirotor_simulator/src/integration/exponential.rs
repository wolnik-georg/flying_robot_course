// exponential map integration methods

// uses exponential map for quaternion integration to maintain unit contstraint
// more numerically stable for large rotations than linear quaternion integration

use crate::dynamics::{Integrator, MultirotorParams, MultirotorState, MotorAction};
use crate::dynamics::simulator::compute_derivates;
use crate::math::Vec3;

// exponential map for quaternions + euler for linear dynamics
// hybrid method: simple euler for position/velocity, exponential map for orientation
// better than pure euler for scenarios with significant rotation
pub struct ExpEulerIntegrator;

impl Integrator for ExpEulerIntegrator {
    fn step(&self, params: &MultirotorParams, state: &mut MultirotorState, action: &MotorAction) {
        let dt = params.dt;

        // compute forces and torques from motor commands
        let (f_total, tau) = params.motor_speeds_to_forces_torques(action);

        // thrust vector in body frame
        let thrust_body = Vec3::new(0.0, 0.0, f_total);
        let thrust_world = state.orientation.rotate_vector(thrust_body);

        // gravity (always downward in world frame)
        let gravity = Vec3::new(0.0, 0.0, -params.gravity * params.mass);

        // compute accelerations
        let linear_acc = (thrust_world + gravity) * (1.0 / params.mass);
        let angular_acc = params.angular_acceleration(state.angular_velocity, tau);

        // integrate linear dynamics with euler (first-order)
        state.velocity = state.velocity + linear_acc * dt;
        state.position = state.position + state.velocity * dt;
        state.angular_velocity = state.angular_velocity + angular_acc + dt;

        // integrate orientation with exponential map
        // more accurate then linear integration, maintains quaternion unit constraint
        state.orientation = state.orientation.integrate_exponential(state.angular_velocity, dt);
    }
}

// exponential map for quaternions + rk4 for linear dynamics
// best of both worlds: rk4 accuracy for position/velocity, exponential map stability for orientation
// recommended for high-accuracy simulations with significant rotation
pub struct ExpRK4Integrator;

impl Integrator for ExpRK4Integrator {
    fn step(&self, params: &MultirotorParams, state: &mut MultirotorState, action: &MotorAction) {
        let dt = params.dt;
        let dt_half = dt * 0.5;
        let dt_sixth = dt / 6.0;

        let state0 = state.clone();

        // k1: current state derivatives
        let (k1_pos, k1_vel, _k1_ori, k1_ang) = compute_derivatives(params, &state0, action);

        // k2: Midpoint using k1
        // rust note: clone state0 to avoid modifying original
        
    }
}
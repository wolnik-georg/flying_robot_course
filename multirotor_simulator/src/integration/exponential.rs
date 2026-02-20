// exponential map integration methods

// uses exponential map for quaternion integration to maintain unit contstraint
// more numerically stable for large rotations than linear quaternion integration

use crate::dynamics::{Integrator, MultirotorParams, MultirotorState, MotorAction};
use crate::dynamics::simulator::compute_derivatives;
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

        // Integrate linear dynamics with Euler (first-order)
        state.velocity = state.velocity + linear_acc * dt;
        state.position = state.position + state.velocity * dt;
        state.angular_velocity = state.angular_velocity + angular_acc * dt;

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
        // Rust note: Clone state0 to avoid modifying original
        let mut state_k2 = state0.clone();
        state_k2.position = state0.position + k1_pos * dt_half;
        state_k2.velocity = state0.velocity + k1_vel * dt_half;
        // Use exponential map for orientation update (not linear!)
        state_k2.orientation = state0.orientation.integrate_exponential(k1_ang, dt_half);
        state_k2.angular_velocity = state0.angular_velocity + k1_ang * dt_half;
        let (k2_pos, k2_vel, _k2_ori, k2_ang) = compute_derivatives(params, &state_k2, action);

        // k3: Midpoint using k2
        let mut state_k3 = state0.clone();
        state_k3.position = state0.position + k2_pos * dt_half;
        state_k3.velocity = state0.velocity + k2_vel * dt_half;
        state_k3.orientation = state0.orientation.integrate_exponential(k2_ang, dt_half);
        state_k3.angular_velocity = state0.angular_velocity + k2_ang * dt_half;
        let (k3_pos, k3_vel, _k3_ori, k3_ang) = compute_derivatives(params, &state_k3, action);

        // k4: Endpoint using k3
        let mut state_k4 = state0.clone();
        state_k4.position = state0.position + k3_pos * dt;
        state_k4.velocity = state0.velocity + k3_vel * dt;
        state_k4.orientation = state0.orientation.integrate_exponential(k3_ang, dt);
        state_k4.angular_velocity = state0.angular_velocity + k3_ang * dt;
        let (k4_pos, k4_vel, _k4_ori, k4_ang) = compute_derivatives(params, &state_k4, action);

        // rk4 weighted combination for linear states
        state.position = state0.position + (k1_pos + k2_pos * 2.0 + k3_pos * 2.0 + k4_pos) * dt_sixth;
        state.velocity = state0.velocity + (k1_vel + k2_vel * 2.0 + k3_vel * 2.0 + k4_vel) * dt_sixth;
        state.angular_velocity = state0.angular_velocity + (k1_ang + k2_ang * 2.0 + k3_ang * 2.0 + k4_ang) * dt_sixth;

        // use weighted average angular velocity for final orientation update
        // this combines rk4's accuracy with exponential map's stability
        let avg_angular_vel = (k1_ang + k2_ang * 2.0 + k3_ang * 2.0 + k4_ang) * dt_sixth;
        state.orientation = state0.orientation.integrate_exponential(avg_angular_vel, dt);
    }
}

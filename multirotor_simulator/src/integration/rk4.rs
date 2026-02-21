//! Runge-Kutta 4th order integration
//!
//! Classic RK4 method - excellent accuracy with moderate computational cost
//! Uses weighted average of 4 derivative evaluations per step

use crate::dynamics::{Integrator, MultirotorParams, MultirotorState, MotorAction};
use crate::dynamics::simulator::compute_derivatives;
use crate::math::Quat;

/// RK4 integrator (4th-order Runge-Kutta)
/// Fourth-order method with O(dt⁴) local error, O(dt⁵) global error
/// Industry standard for moderate-accuracy simulations
/// Rust note: Unit struct - no fields, just methods
pub struct RK4Integrator;

impl RK4Integrator {
    /// Helper to integrate quaternion by a derivative
    /// Quaternions need special handling due to unit constraint
    /// Rust note: Private associated function (not in trait, internal helper)
    fn integrate_quaternion(q: &Quat, dq_dt: Quat, dt: f32) -> Quat {
        // Integrate using: q_new = q_old + dq/dt * dt, then normalize
        // NOT quaternion multiplication, but quaternion addition!
        let delta_q = dq_dt * dt;
        (*q + delta_q).normalize()
    }
}

impl Integrator for RK4Integrator {
    /// Perform one RK4 integration step
    /// RK4 formula: x_new = x_old + (k1 + 2*k2 + 2*k3 + k4) * dt/6
    /// where:
    /// - k1 = f(t, x)
    /// - k2 = f(t + dt/2, x + k1*dt/2)
    /// - k3 = f(t + dt/2, x + k2*dt/2)
    /// - k4 = f(t + dt, x + k3*dt)
    fn step(&self, params: &MultirotorParams, state: &mut MultirotorState, action: &MotorAction) {
        let dt = params.dt;
        let dt_half = dt * 0.5;
        let dt_sixth = dt / 6.0;

        let state0 = state.clone();  // Save initial state

        // k1: Evaluate derivatives at current state
        let (k1_pos, k1_vel, k1_ori, k1_ang) = compute_derivatives(params, &state0, action);

        // k2: Evaluate at midpoint using k1
        let mut state_k2 = state0.clone();
        state_k2.position = state0.position + k1_pos * dt_half;
        state_k2.velocity = state0.velocity + k1_vel * dt_half;
        state_k2.orientation = Self::integrate_quaternion(&state0.orientation, k1_ori, dt_half);
        state_k2.angular_velocity = state0.angular_velocity + k1_ang * dt_half;
        let (k2_pos, k2_vel, k2_ori, k2_ang) = compute_derivatives(params, &state_k2, action);

        // k3: Evaluate at midpoint using k2 (better midpoint estimate)
        let mut state_k3 = state0.clone();
        state_k3.position = state0.position + k2_pos * dt_half;
        state_k3.velocity = state0.velocity + k2_vel * dt_half;
        state_k3.orientation = Self::integrate_quaternion(&state0.orientation, k2_ori, dt_half);
        state_k3.angular_velocity = state0.angular_velocity + k2_ang * dt_half;
        let (k3_pos, k3_vel, k3_ori, k3_ang) = compute_derivatives(params, &state_k3, action);

        // k4: Evaluate at endpoint using k3
        let mut state_k4 = state0.clone();
        state_k4.position = state0.position + k3_pos * dt;
        state_k4.velocity = state0.velocity + k3_vel * dt;
        state_k4.orientation = Self::integrate_quaternion(&state0.orientation, k3_ori, dt);
        state_k4.angular_velocity = state0.angular_velocity + k3_ang * dt;
        let (k4_pos, k4_vel, k4_ori, k4_ang) = compute_derivatives(params, &state_k4, action);

        // Weighted average: (k1 + 2*k2 + 2*k3 + k4) / 6
        // This gives Simpson's rule-like accuracy
        state.position = state0.position + (k1_pos + k2_pos * 2.0 + k3_pos * 2.0 + k4_pos) * dt_sixth;
        state.velocity = state0.velocity + (k1_vel + k2_vel * 2.0 + k3_vel * 2.0 + k4_vel) * dt_sixth;
        state.angular_velocity = state0.angular_velocity + (k1_ang + k2_ang * 2.0 + k3_ang * 2.0 + k4_ang) * dt_sixth;

        // Quaternion integration with weighted average derivative
        // avg_ori_deriv is already scaled by dt/6, so pass dt=1.0 to avoid double-scaling
        let avg_ori_deriv = (k1_ori + k2_ori * 2.0 + k3_ori * 2.0 + k4_ori) * dt_sixth;
        state.orientation = Self::integrate_quaternion(&state0.orientation, avg_ori_deriv, 1.0);
    }

} // end impl Integrator for RK4Integrator


#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::{MultirotorParams, MultirotorState, MotorAction};
    use crate::math::Vec3;

    #[test]
    fn test_rk4_no_motion() {
        let params = MultirotorParams::crazyflie();
        let mut state = MultirotorState::new();
        let action = MotorAction::hover();
        let integrator = RK4Integrator;

        // With exactly balanced hover action, state should remain near initial value
        integrator.step(&params, &mut state, &action);
        assert!(state.position.norm() < 1e-2);
        assert!(state.velocity.norm() < 1e-2);
    }

    #[test]
    fn test_rk4_orientation_integration() {
        let params = MultirotorParams::crazyflie();
        let mut state = MultirotorState::new();
        state.angular_velocity = Vec3::new(0.0, 0.0, 1.0);
        let action = MotorAction::hover();
        let integrator = RK4Integrator;

        integrator.step(&params, &mut state, &action);
        // orientation should have rotated slightly around z
        let yaw = state.orientation.z; // approximate
        assert!(yaw.abs() > 0.0);
    }
}

// euler integration method (first order)

// simplest numerical integration
// fast but least accurate - linear approximation of curved trajectories

use crate::dynamics::{Integrator, MultirotorParams, MultirotorState, MotorAction};
use crate::math::Vec3;

// forward euler integration
// first order method with local error and global error
// pros: simple, fast
// cons: poor accuracy, can be unstable for stiff systems
// rust note: empty struct - no data needed, just behavior
pub struct EulerIntegrator;

impl Integrator for EulerIntegrator {
    // perform one euler integration step 
    fn step(&self, params: &MultirotorParams, state: &mut MultirotorState, action: &MotorAction) {
        let dt = params.dt;

        // compute forces and torques from motor speeds
        let (f_total, tau) = params.motor_speeds_to_forces_torques(action);

        // thrust vector in body frame (z-axis points up)
        let thrust_body = Vec3::new(0.0, 0.0, f_total);
        // rotate thrust to world frame based on current orientation
        let thrust_world = state.orientation.rotate_vector(thrust_body);

        // gravity force (always points down in world frame)
        let gravity = Vec3::new(0.0, 0.0, -params.gravity * params.mass);

        // linear acceleration
        let linear_acc = (thrust_world + gravity) * (1.0 / params.mass);

        // angular acceleration
        let angular_acc = params.angular_acceleration(state.angular_velocity, tau);

        // euler step
        // rust note: operator overloading allows + and * on Vec3
        state.velocity = state.velocity + linear_acc * dt;
        state.angular_velocity = state.angular_velocity + angular_acc * dt;

        // euler step
        // note: using new velocity (semi-implicit euler) would be more stable
        state.position = state.position + state.velocity * dt;

        // integrate orientation using quaternion integration
        // more complex than simple addition due to quaternion constraints
        state.orientation = state.orientation.integrate(state.angular_velocity, dt);
    }

} // end impl Integrator for EulerIntegrator


#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::{MultirotorParams, MultirotorState, MotorAction};
    use crate::math::{Vec3, Quat};

    #[test]
    fn test_euler_step_no_acceleration() {
        let params = MultirotorParams::crazyflie();
        let mut state = MultirotorState::new();
        let action = MotorAction::hover();
        let integrator = EulerIntegrator;

        // take a single step; since hover thrust balances gravity,
        // the vertical acceleration should be near zero and position
        // should change only by a negligible amount (numerical drift).
        integrator.step(&params, &mut state, &action);
        assert!(state.position.z.abs() < 1e-2, "unexpected large climb/fall");
        assert!(state.velocity.norm() < 1.0, "unexpected velocity");
    }

    #[test]
    fn test_euler_step_linear_accel() {
        let params = MultirotorParams::crazyflie();
        let mut state = MultirotorState::new();
        // apply constant upward thrust greater than weight to provide known acceleration
        let thrust = params.mass * (params.gravity + 1.0);
        let torque = Vec3::zero();
        let action = MotorAction::from_thrust_torque(thrust, torque, &params);
        let integrator = EulerIntegrator;

        integrator.step(&params, &mut state, &action);
    // expected linear acceleration positive upward (at least some upward velocity)
    assert!(state.velocity.z > 0.0, "expected upward velocity, got {}", state.velocity.z);
    }
}
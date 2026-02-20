//! Test figure-8 with Euler integrator instead of RK4

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Figure-8 with Euler Integrator ===\n");

    let params = MultirotorParams::crazyflie();
    let integrator = Box::new(EulerIntegrator); // Use Euler instead of RK4
    let mut simulator = MultirotorSimulator::new(params.clone(), integrator);
    
    let controller = GeometricController::new(
        Vec3::new(0.1, 0.1, 0.1),
        Vec3::new(0.05, 0.05, 0.05),
        Vec3::new(0.05, 0.05, 0.05),
        Vec3::new(0.01, 0.01, 0.01),
    );
    
    let trajectory = Figure8Trajectory::with_params(8.0, 0.5, 0.5);
    
    let initial_ref = trajectory.get_reference(0.0);
    let initial_state = MultirotorState::with_initial(
        initial_ref.position,
        initial_ref.velocity,
        Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), initial_ref.yaw),
        Vec3::zero(),
    );
    simulator.set_state(initial_state);

    let dt = 0.01;
    for step in 0..80 {
        let t = step as f32 * dt;
        let reference = trajectory.get_reference(t);
        let state = simulator.state();

        let pos_error = (reference.position - state.position).norm();
        let angular_vel_mag = state.angular_velocity.norm();

        if step % 10 == 0 || pos_error > 1.0 || angular_vel_mag > 10.0 {
            println!("t={:.3}s: pos_err={:.4}m, |ω|={:.2}rad/s",
                t, pos_error, angular_vel_mag);
        }

        if pos_error > 10.0 || angular_vel_mag > 500.0 || pos_error.is_nan() {
            println!("\n❌ DIVERGED at t={:.3}s", t);
            return;
        }

        let control = controller.compute_control(state, &reference, &params);
        let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
        simulator.step(&motor_action);
    }

    println!("\n✓ Stable for 0.8 seconds with Euler integrator!");
}

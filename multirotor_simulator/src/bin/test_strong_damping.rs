//! Test with MUCH stronger angular damping

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Testing with Very Strong Angular Damping ===\n");

    let params = MultirotorParams::crazyflie();
    let integrator = Box::new(RK4Integrator);
    let mut simulator = MultirotorSimulator::new(params.clone(), integrator);
    
    // Try VERY strong angular velocity damping
    let controller = GeometricController::new(
        Vec3::new(0.1, 0.1, 0.1),      // Position gains - keep same
        Vec3::new(0.05, 0.05, 0.05),   // Velocity gains - keep same
        Vec3::new(0.05, 0.05, 0.05),   // Attitude gains - keep same
        Vec3::new(1.0, 1.0, 1.0),      // Angular velocity gains - 100x increase!
    );
    
    println!("Controller gains (100x damping):");
    println!("  Kp = {:.2}", controller.kp.x);
    println!("  Kv = {:.2}", controller.kv.x);
    println!("  KR = {:.2}", controller.kr.x);
    println!("  Kω = {:.2} (100x increase!)\n", controller.kw.x);
    
    let trajectory = Figure8Trajectory::with_params(8.0, 0.5, 0.5);
    
    let initial_ref = trajectory.get_reference(0.0);
    let initial_state = MultirotorState::with_initial(
        initial_ref.position,
        initial_ref.velocity,
        Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), initial_ref.yaw),
        Vec3::zero(),
    );
    simulator.set_state(initial_state);

    let mut max_pos_error: f32 = 0.0;
    let mut max_angular_vel: f32 = 0.0;
    let mut diverged = false;
    
    let dt = 0.01;
    for step in 0..800 {  // Run for 8 seconds
        let t = step as f32 * dt;
        let reference = trajectory.get_reference(t);
        let state = simulator.state();

        let pos_error = (reference.position - state.position).norm();
        let angular_vel_mag = state.angular_velocity.norm();

        max_pos_error = max_pos_error.max(pos_error);
        max_angular_vel = max_angular_vel.max(angular_vel_mag);

        if step % 100 == 0 {
            println!("t={:.2}s: pos_err={:.4}m, |ω|={:.2}rad/s",
                t, pos_error, angular_vel_mag);
        }

        if pos_error > 10.0 || angular_vel_mag > 500.0 || pos_error.is_nan() {
            println!("\n⚠️  DIVERGENCE at t={:.3}s", t);
            diverged = true;
            break;
        }

        let control = controller.compute_control(state, &reference, &params);
        let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
        simulator.step(&motor_action);
    }

    println!("\n=== Results ===");
    if diverged {
        println!("❌ DIVERGED");
    } else {
        println!("✓ STABLE for entire 8-second trajectory!");
    }
    println!("Max position error: {:.4} m", max_pos_error);
    println!("Max angular velocity: {:.2} rad/s", max_angular_vel);
}

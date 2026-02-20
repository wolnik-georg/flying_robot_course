//! Test figure-8 with increased controller gains

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Testing Figure-8 with Increased Gains ===\n");

    let params = MultirotorParams::crazyflie();
    let integrator = Box::new(RK4Integrator);
    let mut simulator = MultirotorSimulator::new(params.clone(), integrator);
    
    // Try MUCH stronger gains
    let controller = GeometricController::new(
        Vec3::new(1.0, 1.0, 1.0),      // Position gains (was 0.1) - 10x increase
        Vec3::new(0.5, 0.5, 0.5),      // Velocity gains (was 0.05) - 10x increase
        Vec3::new(0.5, 0.5, 0.5),      // Attitude gains (was 0.05) - 10x increase  
        Vec3::new(0.1, 0.1, 0.1),      // Angular velocity gains (was 0.01) - 10x increase
    );
    
    println!("Controller gains (10x increase from default):");
    println!("  Kp = {:.2}", controller.kp.x);
    println!("  Kv = {:.2}", controller.kv.x);
    println!("  KR = {:.2}", controller.kr.x);
    println!("  Kω = {:.2}\n", controller.kw.x);
    
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
    let mut max_vel_error: f32 = 0.0;
    let mut max_angular_vel: f32 = 0.0;
    let mut diverged = false;
    
    let dt = 0.01;
    for step in 0..200 {
        let t = step as f32 * dt;
        let reference = trajectory.get_reference(t);
        let state = simulator.state();

        let pos_error = (reference.position - state.position).norm();
        let vel_error = (reference.velocity - state.velocity).norm();
        let angular_vel_mag = state.angular_velocity.norm();

        max_pos_error = max_pos_error.max(pos_error);
        max_vel_error = max_vel_error.max(vel_error);
        max_angular_vel = max_angular_vel.max(angular_vel_mag);

        if step % 20 == 0 {
            println!("t={:.2}s: pos_err={:.4}m, vel_err={:.3}m/s, |ω|={:.1}rad/s",
                t, pos_error, vel_error, angular_vel_mag);
        }

        if pos_error > 10.0 || vel_error > 50.0 || angular_vel_mag > 500.0 ||
           pos_error.is_nan() || vel_error.is_nan() {
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
        println!("✓ Stable for 2 seconds!");
    }
    println!("Max position error: {:.4} m", max_pos_error);
    println!("Max velocity error: {:.3} m/s", max_vel_error);
    println!("Max angular velocity: {:.1} rad/s", max_angular_vel);
}

//! Detailed debugging of figure-8 trajectory tracking
//! Step-by-step analysis to find where instability begins

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Figure-8 Trajectory Debugging ===\n");

    let params = MultirotorParams::crazyflie();
    let integrator = Box::new(RK4Integrator);
    let mut simulator = MultirotorSimulator::new(params.clone(), integrator);
    
    let mut controller = GeometricController::new(
        Vec3::new(0.1, 0.1, 0.1),    // Position gains
        Vec3::new(0.05, 0.05, 0.05), // Velocity gains
        Vec3::new(0.05, 0.05, 0.05), // Attitude gains
        Vec3::new(0.01, 0.01, 0.01), // Angular velocity gains
        Vec3::new(0.01, 0.01, 0.01), // Integral gains
    );
    
    // Create figure-8 trajectory (same as assignment2)
    let trajectory = Figure8Trajectory::with_params(8.0, 0.5, 0.5);
    
    println!("Trajectory info:");
    println!("  Type: Figure-8");
    println!("  Duration: 8.0 s");
    println!("  Time step: 0.01 s\n");

    // Start at initial trajectory position
    let initial_ref = trajectory.get_reference(0.0);
    let initial_state = MultirotorState::with_initial(
        initial_ref.position,
        initial_ref.velocity,
        Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), initial_ref.yaw),
        Vec3::zero(),
    );
    simulator.set_state(initial_state);

    println!("Initial state:");
    println!("  Position: ({:.6}, {:.6}, {:.6})", 
        simulator.state().position.x, 
        simulator.state().position.y, 
        simulator.state().position.z);
    println!("  Velocity: ({:.6}, {:.6}, {:.6})", 
        simulator.state().velocity.x, 
        simulator.state().velocity.y, 
        simulator.state().velocity.z);
    println!("  Quaternion: ({:.6}, {:.6}, {:.6}, {:.6})",
        simulator.state().orientation.w,
        simulator.state().orientation.x,
        simulator.state().orientation.y,
        simulator.state().orientation.z);
    println!();

    let mut max_pos_error: f32 = 0.0;
    let mut max_vel_error: f32 = 0.0;
    let mut max_quat_norm_error: f32 = 0.0;
    let mut diverged = false;
    let mut diverge_time: f32 = 0.0;

    // Run for first 2 seconds or until divergence
    let dt = 0.01;
    for step in 0..200 {
        let t = step as f32 * dt;
        let reference = trajectory.get_reference(t);
        let state = simulator.state();

        // Compute errors
        let pos_error = (reference.position - state.position).norm();
        let vel_error = (reference.velocity - state.velocity).norm();
        let quat_norm = state.orientation.norm();
        let quat_norm_error = (1.0 - quat_norm).abs();

        max_pos_error = max_pos_error.max(pos_error);
        max_vel_error = max_vel_error.max(vel_error);
        max_quat_norm_error = max_quat_norm_error.max(quat_norm_error);

        // Print every step for first 10, then every 10 steps, or if errors grow
        let should_print = step < 10 || step % 10 == 0 || pos_error > 0.1 || vel_error > 1.0 || quat_norm_error > 0.01;

        if should_print {
            println!("t={:.3}s (step {}):", t, step);
            println!("  Desired pos: ({:.4}, {:.4}, {:.4})", 
                reference.position.x, reference.position.y, reference.position.z);
            println!("  Actual pos:  ({:.4}, {:.4}, {:.4})", 
                state.position.x, state.position.y, state.position.z);
            println!("  Position error: {:.6} m", pos_error);
            
            println!("  Desired vel: ({:.4}, {:.4}, {:.4})", 
                reference.velocity.x, reference.velocity.y, reference.velocity.z);
            println!("  Actual vel:  ({:.4}, {:.4}, {:.4})", 
                state.velocity.x, state.velocity.y, state.velocity.z);
            println!("  Velocity error: {:.6} m/s", vel_error);
            
            println!("  Desired acc: ({:.4}, {:.4}, {:.4})", 
                reference.acceleration.x, reference.acceleration.y, reference.acceleration.z);
            
            println!("  Quaternion: ({:.6}, {:.6}, {:.6}, {:.6})",
                state.orientation.w, state.orientation.x, state.orientation.y, state.orientation.z);
            println!("  Quat norm: {:.9} (error: {:.2e})", quat_norm, quat_norm_error);
            
            println!("  Angular vel: ({:.6}, {:.6}, {:.6})",
                state.angular_velocity.x, state.angular_velocity.y, state.angular_velocity.z);
        }

        // Check for divergence
        if pos_error > 10.0 || vel_error > 50.0 || quat_norm_error > 0.1 || 
           pos_error.is_nan() || vel_error.is_nan() || quat_norm.is_nan() {
            println!("\n⚠️  DIVERGENCE DETECTED at t={:.3}s!", t);
            println!("  Position error: {:.3} m", pos_error);
            println!("  Velocity error: {:.3} m/s", vel_error);
            println!("  Quaternion norm error: {:.6}", quat_norm_error);
            diverged = true;
            diverge_time = t;
            break;
        }

        // Get control and step
        let control = controller.compute_control(state, &reference, &params, 0.01);
        let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
        
        if should_print {
            println!("  Control thrust: {:.6} N", control.thrust);
            println!("  Control torque: ({:.6}, {:.6}, {:.6}) Nm",
                control.torque.x, control.torque.y, control.torque.z);
            println!();
        }

        simulator.step(&motor_action);
    }

    println!("\n=== Summary ===");
    if diverged {
        println!("❌ System DIVERGED at t={:.3}s", diverge_time);
    } else {
        println!("✓ System remained stable for 2 seconds");
    }
    println!("Max position error: {:.6} m", max_pos_error);
    println!("Max velocity error: {:.6} m/s", max_vel_error);
    println!("Max quaternion norm error: {:.2e}", max_quat_norm_error);
}

//! Debug Controller - Step-by-step logging to diagnose issues
//!
//! This binary runs a simple test scenario with detailed logging at each step

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Controller Debug Session ===\n");

    // Setup
    let params = MultirotorParams::crazyflie();
    let integrator = Box::new(RK4Integrator);
    let mut simulator = MultirotorSimulator::new(params.clone(), integrator);

    // Very simple controller gains
    let mut controller = GeometricController::new(
        Vec3::new(0.1, 0.1, 0.1),    // Position gains
        Vec3::new(0.05, 0.05, 0.05), // Velocity gains
        Vec3::new(0.05, 0.05, 0.05), // Attitude gains
        Vec3::new(0.01, 0.01, 0.01), // Angular velocity gains
        Vec3::new(0.01, 0.01, 0.01), // Integral gains
    );

    println!("Parameters:");
    println!("  Mass: {} kg", params.mass);
    println!("  Gravity: {} m/s²", params.gravity);
    println!("  kf: {:.2e} N/(rad/s)²", params.kf);
    println!("  kt: {:.2e} Nm/(rad/s)²", params.kt);
    println!("  Inertia: [{:.2e}, {:.2e}, {:.2e}] kg⋅m²", params.inertia[0][0], params.inertia[1][1], params.inertia[2][2]);
    println!();

    // Test 1: Hover at origin
    println!("=== TEST 1: Hover at origin ===");
    
    // Reference: hover at (0, 0, 0.5)
    let reference = TrajectoryReference {
        position: Vec3::new(0.0, 0.0, 0.5),
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
            jerk: Vec3::zero(),
        yaw: 0.0,
        yaw_rate: 0.0,
        yaw_acceleration: 0.0,
    };

    // Initial state: at the reference
    let initial_state = MultirotorState::with_initial(
        reference.position,
        reference.velocity,
        Quat::identity(),
        Vec3::zero(),
    );
    simulator.set_state(initial_state);

    println!("Initial state:");
    let state = simulator.state();
    println!("  Position: ({:.6}, {:.6}, {:.6})", state.position.x, state.position.y, state.position.z);
    println!("  Velocity: ({:.6}, {:.6}, {:.6})", state.velocity.x, state.velocity.y, state.velocity.z);
    println!("  Quaternion: ({:.6}, {:.6}, {:.6}, {:.6})", state.orientation.w, state.orientation.x, state.orientation.y, state.orientation.z);
    println!("  Angular vel: ({:.6}, {:.6}, {:.6})", state.angular_velocity.x, state.angular_velocity.y, state.angular_velocity.z);
    println!();

    // Step 1: Compute control
    println!("Step 1: Compute control for hover");
    let control = controller.compute_control(&simulator.state(), &reference, &params, 0.01);
    println!("  Thrust: {:.6} N (expected: {:.6} N = m*g)", control.thrust, params.mass * params.gravity);
    println!("  Torque: ({:.6}, {:.6}, {:.6}) Nm (expected: ~0)", control.torque.x, control.torque.y, control.torque.z);
    println!();

    // Step 2: Convert to motor commands
    println!("Step 2: Convert to motor commands");
    let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
    println!("  Motor 1 (FL): ω² = {:.2}", motor_action.omega1_sq);
    println!("  Motor 2 (FR): ω² = {:.2}", motor_action.omega2_sq);
    println!("  Motor 3 (BR): ω² = {:.2}", motor_action.omega3_sq);
    println!("  Motor 4 (BL): ω² = {:.2}", motor_action.omega4_sq);
    
    // Verify motor mixing
    let (thrust_check, torque_check) = params.motor_speeds_to_forces_torques(&motor_action);
    println!("  Verification (forward model):");
    println!("    Thrust: {:.6} N (error: {:.2e})", thrust_check, thrust_check - control.thrust);
    println!("    Torque: ({:.6}, {:.6}, {:.6}) Nm", torque_check.x, torque_check.y, torque_check.z);
    println!("    Torque error: ({:.2e}, {:.2e}, {:.2e})", 
        torque_check.x - control.torque.x,
        torque_check.y - control.torque.y,
        torque_check.z - control.torque.z);
    println!();

    // Step 3: Simulate 5 steps
    println!("Step 3: Simulate 5 timesteps");
    for step in 0..5 {
        let state_before = simulator.state().clone();
        
        // Recompute control each time
        let control = controller.compute_control(&simulator.state(), &reference, &params, 0.01);
        let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
        
        simulator.step(&motor_action);
        
        let state_after = simulator.state();
        
        println!("  Step {} (t = {:.3} s):", step + 1, (step + 1) as f32 * params.dt);
        println!("    Position: ({:.6}, {:.6}, {:.6})", state_after.position.x, state_after.position.y, state_after.position.z);
        println!("    Velocity: ({:.6}, {:.6}, {:.6})", state_after.velocity.x, state_after.velocity.y, state_after.velocity.z);
        println!("    Quat: ({:.6}, {:.6}, {:.6}, {:.6}) [norm: {:.6}]", 
            state_after.orientation.w, state_after.orientation.x, 
            state_after.orientation.y, state_after.orientation.z,
            state_after.orientation.norm());
        println!("    Angular vel: ({:.6}, {:.6}, {:.6})", 
            state_after.angular_velocity.x, state_after.angular_velocity.y, state_after.angular_velocity.z);
        println!("    Thrust: {:.6} N, Torque: ({:.6}, {:.6}, {:.6}) Nm", 
            control.thrust, control.torque.x, control.torque.y, control.torque.z);
        
        // Check for anomalies
        if state_after.position.norm() > 1.0 {
            println!("    ⚠️  WARNING: Position magnitude > 1.0 m");
        }
        if state_after.velocity.norm() > 0.1 {
            println!("    ⚠️  WARNING: Velocity magnitude > 0.1 m/s for hover");
        }
        if state_after.angular_velocity.norm() > 1.0 {
            println!("    ⚠️  WARNING: Angular velocity > 1.0 rad/s");
        }
        if (state_after.orientation.norm() - 1.0).abs() > 0.01 {
            println!("    ⚠️  WARNING: Quaternion not normalized!");
        }
        println!();
    }

    println!("\n=== TEST 2: Small position offset ===");
    
    // Reset and offset position slightly
    simulator.reset();
    let offset_state = MultirotorState::with_initial(
        Vec3::new(0.1, 0.0, 0.5), // 10cm offset in x
        Vec3::zero(),
        Quat::identity(),
        Vec3::zero(),
    );
    simulator.set_state(offset_state);

    println!("Initial state: 10cm offset in +x direction");
    println!("Reference: hover at (0, 0, 0.5)");
    println!();

    for step in 0..10 {
        let state = simulator.state();
        let control = controller.compute_control(&state, &reference, &params, 0.01);
        let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
        
        if step % 2 == 0 {
            let pos_error = state.position - reference.position;
            println!("  t={:.3}s: pos_err=({:.4}, {:.4}, {:.4}), vel=({:.4}, {:.4}, {:.4}), thrust={:.4}N",
                step as f32 * params.dt,
                pos_error.x, pos_error.y, pos_error.z,
                state.velocity.x, state.velocity.y, state.velocity.z,
                control.thrust);
        }
        
        simulator.step(&motor_action);
    }
    
    let final_state = simulator.state();
    let final_error = final_state.position - reference.position;
    println!("\nFinal state after 10 steps:");
    println!("  Position error: ({:.6}, {:.6}, {:.6})", final_error.x, final_error.y, final_error.z);
    println!("  Error magnitude: {:.6} m", final_error.norm());
    
    if final_error.norm() < 0.05 {
        println!("  ✓ Controller is working - error decreased!");
    } else if final_error.norm() > 0.1 {
        println!("  ✗ Controller may be unstable - error increased!");
    }
}

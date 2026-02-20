//! Assignment 2: Geometric Control Visualization
//!
//! This binary demonstrates geometric control performance on various trajectory tracking scenarios:
//! - Hover control (stationary position holding)
//! - Figure-8 trajectory tracking
//! - Circle trajectory tracking
//!
//! Generates CSV data for plotting control performance, tracking errors, and motor commands.

use multirotor_simulator::prelude::*;
use std::fs::File;
use std::io::Write;

fn main() {
    println!("Assignment 2: Geometric Control Visualization");
    println!("Demonstrating trajectory tracking with geometric controller\n");

    // Create aircraft parameters and simulator
    let params = MultirotorParams::crazyflie();
    let integrator = Box::new(RK4Integrator);
    let mut simulator = MultirotorSimulator::new(params.clone(), integrator);

    // Create geometric controller with moderate gains
    let controller = GeometricController::new(
        Vec3::new(0.1, 0.1, 0.1),    // Position gains
        Vec3::new(0.05, 0.05, 0.05), // Velocity gains
        Vec3::new(0.01, 0.01, 0.01), // Attitude gains
        Vec3::new(0.001, 0.001, 0.001), // Angular velocity gains
    );

    println!("Controller gains:");
    println!("  Position: Kp = {:.3}", 0.1);
    println!("  Velocity: Kv = {:.3}", 0.05);
    println!("  Attitude: KR = {:.3}", 0.01);
    println!("  Angular velocity: Kω = {:.3}", 0.001);
    println!();

    // Simulation parameters
    let dt = 0.01;
    let scenarios = vec![
        ("hover", "Stationary hover at (0,0,0.5m)"),
        ("figure8", "Figure-8 trajectory tracking"),
        ("circle", "Circular trajectory tracking"),
    ];

    for (scenario_name, description) in scenarios {
        println!("Running scenario: {} - {}", scenario_name, description);

        // Reset simulator to initial state
        simulator.reset();

        // Create trajectory based on scenario
        let trajectory: Box<dyn Trajectory> = match scenario_name {
            "hover" => Box::new(CircleTrajectory::new(0.0, 0.5, 0.0)), // Zero radius circle = hover
            "figure8" => Box::new(Figure8Trajectory::new()), // Default figure-8
            "circle" => Box::new(CircleTrajectory::new(0.3, 0.3, 0.3)), // radius, height, angular velocity
            _ => unreachable!(),
        };

        // Set initial state to match trajectory start
        let initial_ref = trajectory.get_reference(0.0);
        let initial_state = MultirotorState::with_initial(
            initial_ref.position,
            initial_ref.velocity,
            Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), initial_ref.yaw),
            Vec3::zero(),
        );
        simulator.set_state(initial_state);

        // Determine simulation duration based on scenario
        let end_time = match scenario_name {
            "hover" => 5.0,    // 5 seconds for hover
            "figure8" => 8.0,  // 8 seconds for figure-8
            "circle" => 10.0,  // 10 seconds for circle
            _ => unreachable!(),
        };
        let steps = (end_time / dt) as usize;

        // Create CSV file
        let filename = format!("results/data/assignment2_{}.csv", scenario_name);
        let mut file = File::create(&filename).expect("Failed to create CSV file");

        // Write CSV header
        writeln!(file, "time,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,x_ref,y_ref,z_ref,vx_ref,vy_ref,vz_ref,thrust,tx,ty,tz,omega1,omega2,omega3,omega4,pos_error_x,pos_error_y,pos_error_z,vel_error_x,vel_error_y,vel_error_z").expect("Failed to write header");

        // Simulation loop
        let mut time = 0.0;
        for step in 0..steps {
            // Get current state
            let state = simulator.state();

            // Get trajectory reference
            let reference = trajectory.get_reference(time);

            // Compute control
            let control = controller.compute_control(state, &reference, &params);

            // Convert control to motor action
            let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);

            // Calculate tracking errors
            let pos_error = state.position - reference.position;
            let vel_error = state.velocity - reference.velocity;

            // Write to CSV
            writeln!(file, "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                time,
                state.position.x, state.position.y, state.position.z,
                state.velocity.x, state.velocity.y, state.velocity.z,
                state.orientation.w, state.orientation.x, state.orientation.y, state.orientation.z,
                state.angular_velocity.x, state.angular_velocity.y, state.angular_velocity.z,
                reference.position.x, reference.position.y, reference.position.z,
                reference.velocity.x, reference.velocity.y, reference.velocity.z,
                control.thrust,
                control.torque.x, control.torque.y, control.torque.z,
                motor_action.omega1_sq, motor_action.omega2_sq, motor_action.omega3_sq, motor_action.omega4_sq,
                pos_error.x, pos_error.y, pos_error.z,
                vel_error.x, vel_error.y, vel_error.z
            ).expect("Failed to write data");

            // Apply control to simulator
            simulator.step(&motor_action);
            time += dt;

            // Progress indicator
            if (step + 1) % 100 == 0 {
                print!(".");
                std::io::stdout().flush().unwrap();
            }
        }
        println!(" Done! Saved to {}", filename);

        // Print final statistics
        let final_state = simulator.state();
        let final_ref = trajectory.get_reference(time);
        let final_pos_error = (final_state.position - final_ref.position).norm();
        let final_vel_error = (final_state.velocity - final_ref.velocity).norm();

        println!("  Final position error: {:.3} m", final_pos_error);
        println!("  Final velocity error: {:.3} m/s", final_vel_error);
        println!();
    }

    println!("All scenarios complete! CSV files generated for plotting.");
    println!("Run the Python plotting script to visualize geometric control performance.");
}
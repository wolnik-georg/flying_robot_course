//! Assignment 1: Simulation Quality Experiment
//!
//! This binary compares different integration methods (Euler, RK4, ExpEuler, ExpRK4)
//! on a simple vertical takeoff trajectory to demonstrate simulation quality differences.

use multirotor_simulator::prelude::*;
use std::fs::File;
use std::io::Write;

fn main() {
    println!("Assignment 1: Simulation Quality Experiment");
    println!("Comparing integration methods on vertical takeoff trajectory\n");

    // Create aircraft parameters
    let params = MultirotorParams::crazyflie();

    // Define integration methods to compare
    let integrators = vec![
        ("Euler", Box::new(EulerIntegrator) as Box<dyn Integrator>),
        ("RK4", Box::new(RK4Integrator) as Box<dyn Integrator>),
        ("ExpEuler", Box::new(ExpEulerIntegrator) as Box<dyn Integrator>),
        ("ExpRK4", Box::new(ExpRK4Integrator) as Box<dyn Integrator>),
    ];

    // Simulation parameters
    let dt = 0.01;
    let end_time = 2.0; // 2 seconds of simulation
    let steps = (end_time / dt) as usize;

    // Create constant thrust input (slightly more than hover for upward motion)
    let hover_thrust = params.mass * 9.81; // Mass * gravity
    let takeoff_thrust = hover_thrust * 1.1; // 10% more thrust for takeoff
    let control_input = ControlOutput {
        thrust: takeoff_thrust,
        torque: Vec3::zero(),
    };

    println!("Crazyflie mass: {:.3} kg", params.mass);
    println!("Hover thrust needed: {:.6} N", hover_thrust);
    println!("Takeoff thrust used: {:.6} N", takeoff_thrust);
    println!("Motor thrust coefficient kf: {:.2e}", params.kf);
    println!();

    // Convert control to motor action
    let motor_action = MotorAction::from_thrust_torque(control_input.thrust, control_input.torque, &params);
    println!("Motor action: ω1²={:.1}, ω2²={:.1}, ω3²={:.1}, ω4²={:.1}",
        motor_action.omega1_sq, motor_action.omega2_sq, motor_action.omega3_sq, motor_action.omega4_sq);

    // Calculate expected individual motor thrust
    let motor_thrust = params.kf * motor_action.omega1_sq;
    println!("Expected motor thrust: {:.6} N per motor", motor_thrust);
    println!("Total thrust: {:.6} N", 4.0 * motor_thrust);
    println!("Net acceleration: {:.6} m/s²", (4.0 * motor_thrust - hover_thrust) / params.mass);
    println!();

    // Run simulation for each integrator
    for (name, integrator) in integrators {
        println!("Running {} integrator...", name);

        // Create simulator
        let mut simulator = MultirotorSimulator::new(params.clone(), integrator);

        // Create CSV file
        // ensure output directory exists so that users can delete it between runs
        std::fs::create_dir_all("results/data").expect("Failed to create results/data directory");
        let filename = format!("results/data/trajectory_modular_{}.csv", name.replace(" ", "_"));
        let mut file = File::create(&filename).expect("Failed to create CSV file");

        // Write CSV header
        writeln!(file, "time,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz").expect("Failed to write header");

        // Initial state (write first point)
        let initial_state = simulator.state();
        writeln!(file, "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            0.0,
            initial_state.position.x, initial_state.position.y, initial_state.position.z,
            initial_state.velocity.x, initial_state.velocity.y, initial_state.velocity.z,
            initial_state.orientation.w, initial_state.orientation.x, initial_state.orientation.y, initial_state.orientation.z,
            initial_state.angular_velocity.x, initial_state.angular_velocity.y, initial_state.angular_velocity.z
        ).expect("Failed to write initial state");

        // Convert control to motor action
        let motor_action = MotorAction::from_thrust_torque(control_input.thrust, control_input.torque, &params);

        // Simulation loop
        let mut time = 0.0;
        for step in 0..steps {
            // Step simulation
            simulator.step(&motor_action);

            // Get current state
            let state = simulator.state();
            time += dt;

            // Write to CSV
            writeln!(file, "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                time,
                state.position.x, state.position.y, state.position.z,
                state.velocity.x, state.velocity.y, state.velocity.z,
                state.orientation.w, state.orientation.x, state.orientation.y, state.orientation.z,
                state.angular_velocity.x, state.angular_velocity.y, state.angular_velocity.z
            ).expect("Failed to write state");

            // Progress indicator
            if (step + 1) % 100 == 0 {
                print!(".");
                std::io::stdout().flush().unwrap();
            }
        }
        println!(" Done! Saved to {}", filename);

        // Print final state summary
        let final_state = simulator.state();
        println!("  Final position: ({:.3}, {:.3}, {:.3}) m", final_state.position.x, final_state.position.y, final_state.position.z);
        println!("  Final velocity: ({:.3}, {:.3}, {:.3}) m/s", final_state.velocity.x, final_state.velocity.y, final_state.velocity.z);
        println!();
    }

    println!("All simulations complete! CSV files generated for plotting.");
    println!("Run the Python plotting script to visualize the results.");
}

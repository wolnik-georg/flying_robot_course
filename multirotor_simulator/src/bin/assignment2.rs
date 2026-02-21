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

    // Official Crazyflie Lee controller gains
    let mut controller = GeometricController::new(
        Vec3::new(7.0, 7.0, 7.0),
        Vec3::new(4.0, 4.0, 4.0),
        Vec3::new(0.007, 0.007, 0.008),
        Vec3::new(0.00115, 0.00115, 0.002),
        Vec3::new(0.03, 0.03, 0.03), // Attitude integral gains
    );
    // print the gains once before parsing arguments (they don't depend on flags)
    println!("Controller gains:");
    println!("  Kp = {:.3}, Kv = {:.3}, KR = {:.6}, Kω = {:.6}",
             controller.kp.x,
             controller.kv.x,
             controller.kr.x,
             controller.kw.x,
    );
    println!();

    // simple command line argument handling so we can toggle various debug
    // features without editing the source every time.  We don't bother with a
    // full CLI parser because the options are few and this binary is only for
    // experimentation.
    let args: Vec<String> = std::env::args().collect();
    let debug = args.contains(&"--debug".to_string());
    let disable_jerk = args.contains(&"--no-jerk".to_string());
    let disable_integral = args.contains(&"--no-ki".to_string());
    // optional scenario selection: `--scenario=hover|figure8|circle`
    let scenario_filter = args
        .iter()
        .find(|s| s.starts_with("--scenario="))
        .and_then(|s| s.splitn(2, '=').nth(1))
        .map(|s| s.to_string());

    // apply any global flags that affect the controller
    if disable_integral {
        println!("WARNING: integral gains disabled (--no-ki)");
        // zero out the integral gain vector so the controller ignores accumulated error
        controller.ki = Vec3::zero();
    }

    if disable_jerk {
        println!("WARNING: jerk feedforward disabled (--no-jerk)");
    }

    if debug {
        println!("Debug mode enabled (--debug)");
    }

    if let Some(ref filter) = scenario_filter {
        println!("Filtering to scenario '{}'", filter);
    }

    // Simulation parameters
    let dt = 0.01;
    let scenarios = vec![
        ("hover", "Stationary hover at (0,0,0.5m)"),
        ("figure8", "Figure-8 trajectory tracking"),
        ("circle", "Circular trajectory tracking"),
    ];

    for (scenario_name, description) in scenarios {
        // if a scenario filter was specified and it does not match the current
        // scenario, skip this iteration entirely.  use starts_with so that
        // partial names like "fig" still work if desired.
        if let Some(ref filter) = scenario_filter {
            if !scenario_name.starts_with(filter) {
                continue;
            }
        }
        println!("Running scenario: {} - {}", scenario_name, description);

        // Reset simulator to initial state
        simulator.reset();

    // Make sure controller internal state (integral) is cleared before each new
    // experiment.  Previous versions reused the controller across scenarios which
    // caused the integral error to carry over and destabilize subsequent paths.
    controller.reset();

        // Create trajectory based on scenario
        let trajectory: Box<dyn Trajectory> = match scenario_name {
            "hover" => Box::new(CircleTrajectory::new(0.0, 0.5, 0.0)), // Zero radius circle = hover
            "figure8" => Box::new(Figure8Trajectory::with_params(8.0, 0.5, 0.5)), // Slower figure-8
            "circle" => Box::new(CircleTrajectory::new(0.3, 0.3, std::f32::consts::PI / 5.0)), // Full circle every 10s
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
            "circle" => 20.0,  // 20 seconds for two full circles
            _ => unreachable!(),
        };
        let steps = (end_time / dt) as usize;

        // Create CSV file
        // ensure output directory exists (may have been removed)
        std::fs::create_dir_all("results/data").expect("Failed to create results/data directory");
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
            let mut reference = trajectory.get_reference(time);
            if disable_jerk {
                // manually zero jerk when debugging
                reference.jerk = Vec3::zero();
            }

            // Compute control, optionally with debug information
            let control = if debug {
                let (out, dbg) = controller.compute_control_debug(state, &reference, &params, dt);
                // print pipeline diagnostics
                println!("step {} t={:.3}", step, time);
                println!("  ref pos={:?} vel={:?} acc={:?} jerk={:?}",
                         reference.position, reference.velocity, reference.acceleration, reference.jerk);
                println!("  ep={:?} ev={:?} feedf={:?} thrust={:.6}",
                         dbg.ep, dbg.ev, dbg.feedforward, dbg.thrust);
                println!("  er={:?} eomega={:?}", dbg.er, dbg.eomega);
                println!("  hw={:?} omega_des={:?} omega_r={:?}", dbg.hw, dbg.omega_des, dbg.omega_r);
                println!("  torques P={:?} D={:?} I={:?} G={:?} total={:?}\n",
                         dbg.torque_proportional, dbg.torque_derivative,
                         dbg.torque_integral, dbg.gyroscopic_compensation, dbg.torque);
                out
            } else {
                controller.compute_control(state, &reference, &params, dt)
            };

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
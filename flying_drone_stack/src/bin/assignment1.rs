//! Assignment 1: Simulation Quality Experiment
//!
//! This binary compares different integration methods (Euler, RK4, ExpEuler, ExpRK4)
//! on two scenarios:
//!
//! 1. Vertical takeoff — constant thrust, no rotation.  Shows positional divergence
//!    between methods (small but measurable).
//!
//! 2. Spinning hover — hover thrust + constant yaw torque.  The drone spins at
//!    increasing angular velocity.  Standard Euler integration violates the unit-
//!    quaternion constraint (|q| drifts from 1), while the exponential-map methods
//!    (ExpEuler, ExpRK4) preserve |q| = 1 exactly by construction.

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
        std::fs::create_dir_all("results/assignment1/data").expect("Failed to create results/assignment1/data directory");
        let filename = format!("results/assignment1/data/trajectory_modular_{}.csv", name.replace(" ", "_"));
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

    // ── Scenario 2: Spinning hover — reveals quaternion norm drift ────────────
    println!("─────────────────────────────────────────────────────────────────");
    println!("Scenario 2: Spinning hover (quaternion norm test)");
    println!("─────────────────────────────────────────────────────────────────");
    println!("Hover thrust + constant yaw torque → increasing ω_z.");
    println!("Euler: |q| drifts from 1.0.  ExpEuler/ExpRK4: |q| = 1 exactly.\n");

    // Jzz = 29.26e-6 kg·m²  →  α = τ/J ≈ 1.71 rad/s² per 5e-5 N·m torque.
    // After 5 s: ω_z ≈ 8.6 rad/s.  Fast enough to make Euler norm drift ~10 %.
    let spin_torque = 5e-5_f32; // N·m yaw torque
    let spin_end_time = 5.0_f32;
    let spin_steps = (spin_end_time / dt) as usize;

    let spin_integrators: Vec<(&str, Box<dyn Integrator>)> = vec![
        ("Euler",    Box::new(EulerIntegrator)),
        ("RK4",      Box::new(RK4Integrator)),
        ("ExpEuler", Box::new(ExpEulerIntegrator)),
        ("ExpRK4",   Box::new(ExpRK4Integrator)),
    ];

    for (name, integrator) in spin_integrators {
        println!("Running {} integrator (spin)...", name);

        let mut simulator = MultirotorSimulator::new(params.clone(), integrator);

        let filename = format!("results/assignment1/data/trajectory_modular_{}_spin.csv", name);
        let mut file = File::create(&filename).expect("Failed to create spin CSV file");
        writeln!(file, "time,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz").expect("Failed to write header");

        // Write initial state
        let s0 = simulator.state();
        writeln!(file,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            0.0,
            s0.position.x, s0.position.y, s0.position.z,
            s0.velocity.x, s0.velocity.y, s0.velocity.z,
            s0.orientation.w, s0.orientation.x, s0.orientation.y, s0.orientation.z,
            s0.angular_velocity.x, s0.angular_velocity.y, s0.angular_velocity.z
        ).expect("Failed to write initial state");

        let mut time = 0.0_f32;
        for step in 0..spin_steps {
            // Hover thrust (no net vertical acceleration) + yaw torque
            let action = MotorAction::from_thrust_torque(
                hover_thrust,
                Vec3::new(0.0, 0.0, spin_torque),
                &params,
            );
            simulator.step(&action);

            let s = simulator.state();
            time += dt;

            writeln!(file,
                "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                time,
                s.position.x, s.position.y, s.position.z,
                s.velocity.x, s.velocity.y, s.velocity.z,
                s.orientation.w, s.orientation.x, s.orientation.y, s.orientation.z,
                s.angular_velocity.x, s.angular_velocity.y, s.angular_velocity.z
            ).expect("Failed to write state");

            if (step + 1) % 100 == 0 { print!("."); std::io::stdout().flush().unwrap(); }
        }

        let sf = simulator.state();
        let qnorm = (sf.orientation.w * sf.orientation.w
            + sf.orientation.x * sf.orientation.x
            + sf.orientation.y * sf.orientation.y
            + sf.orientation.z * sf.orientation.z).sqrt();
        println!(" Done!  Final |q| = {:.6}  (ideal = 1.0)  ω_z = {:.2} rad/s",
            qnorm, sf.angular_velocity.z);
        println!("  Saved to {}", filename);
        println!();
    }

    println!("All spin simulations complete.");
    println!("Run the Python plotting script to visualize the results.");

    // ── Runtime benchmark ─────────────────────────────────────────────────────
    // Measures wall-clock time for N steps with each integrator.
    // Uses the same hover action so the workload is identical for all methods.
    println!("\n─────────────────────────────────────────────────────────────────");
    println!("Runtime benchmark (wall-clock time per 10 000 steps, hover action)");
    println!("─────────────────────────────────────────────────────────────────");
    let bench_steps: usize = 10_000;
    let bench_action = MotorAction::from_thrust_torque(
        hover_thrust,
        Vec3::zero(),
        &params,
    );
    let bench_integrators: Vec<(&str, Box<dyn Integrator>)> = vec![
        ("Euler",    Box::new(EulerIntegrator)),
        ("RK4",      Box::new(RK4Integrator)),
        ("ExpEuler", Box::new(ExpEulerIntegrator)),
        ("ExpRK4",   Box::new(ExpRK4Integrator)),
    ];
    println!("{:<10} {:>18} {:>18}", "Method", "Total (µs)", "Per step (ns)");
    println!("{}", "-".repeat(50));
    for (name, integrator) in bench_integrators {
        let mut sim = MultirotorSimulator::new(params.clone(), integrator);
        let t0 = std::time::Instant::now();
        for _ in 0..bench_steps {
            sim.step(&bench_action);
        }
        let elapsed = t0.elapsed();
        let total_us = elapsed.as_micros();
        let per_step_ns = elapsed.as_nanos() / bench_steps as u128;
        println!("{:<10} {:>15} µs  {:>14} ns", name, total_us, per_step_ns);
    }
    println!("\nNote: ExpRK4 is ~4× slower than Euler (4 derivative evaluations per step).");
    println!("      All times are O(1) per step — no growth with trajectory length.");
    println!("      Step size dt=0.01 s used throughout; accuracy vs dt tradeoff");
    println!("      visible in the position/attitude error plots.");

    // ── Convergence analysis: position error vs step size ─────────────────────
    // Sweeps dt over [0.04, 0.02, 0.01, 0.005, 0.0025, 0.00125] on the vertical-
    // takeoff scenario.
    //
    // Two subtleties handled here:
    //
    // 1. Motor dynamics: MultirotorSimulator applies a first-order motor lag
    //    (alpha = dt/(tau+dt), tau=0.03 s).  Changing dt changes the transient,
    //    which corrupts the convergence signal.  Fix: set motor_time_constant=0
    //    so alpha≈1 (instant motor response) for all dt in the sweep.
    //
    // 2. Reference solution: instead of an analytical formula we use RK4 at a
    //    much finer dt (0.0005 s) as the "exact" reference — the standard approach
    //    for convergence studies when a closed-form is unavailable.
    //
    // Expected slopes on log-log: Euler/ExpEuler = 1 (first-order O(dt));
    //                             RK4/ExpRK4     = 4 (fourth-order O(dt⁴)).
    println!("\n─────────────────────────────────────────────────────────────────");
    println!("Convergence analysis: position error vs step size");
    println!("─────────────────────────────────────────────────────────────────");

    let dt_values: &[f32] = &[0.04, 0.02, 0.01, 0.005, 0.0025, 0.00125];
    let conv_end_time = 2.0_f32;
    // Takeoff action: 10 % above hover; kf/kt unchanged so &params is correct.
    let conv_action = MotorAction::from_thrust_torque(
        params.mass * params.gravity * 1.1,
        Vec3::zero(),
        &params,
    );

    // Reference: RK4 at dt=0.0005 (≥2× finer than the finest sweep point).
    // Motor dynamics bypassed (motor_time_constant=0 → alpha≈1 for all dt).
    let ref_dt = 0.0005_f32;
    let z_ref = {
        let mut ref_params = params.clone();
        ref_params.dt = ref_dt;
        ref_params.motor_time_constant = 0.0;
        let mut sim = MultirotorSimulator::new(ref_params, Box::new(RK4Integrator));
        let n = (conv_end_time / ref_dt).round() as usize;
        for _ in 0..n { sim.step(&conv_action); }
        sim.state().position.z
    };
    println!("  Reference: RK4 at dt={:.4} s  →  z(2 s) = {:.6} m", ref_dt, z_ref);
    println!();

    // Header row
    print!("{:<10}", "Method");
    for &dt_val in dt_values {
        print!("  dt={:.5}", dt_val);
    }
    println!("  (|z_sim − z_ref|, m)");
    println!("{}", "-".repeat(96));

    let conv_method_names = ["Euler", "RK4", "ExpEuler", "ExpRK4"];
    for &method_name in &conv_method_names {
        let filename = format!("results/assignment1/data/convergence_{}.csv", method_name);
        let mut csv = File::create(&filename).expect("Failed to create convergence CSV");
        writeln!(csv, "dt,pos_error").expect("Failed to write header");

        print!("{:<10}", method_name);
        for &dt_val in dt_values {
            let mut conv_params = params.clone();
            conv_params.dt = dt_val;
            conv_params.motor_time_constant = 0.0; // bypass motor lag for clean convergence

            let integrator: Box<dyn Integrator> = match method_name {
                "Euler"    => Box::new(EulerIntegrator),
                "RK4"      => Box::new(RK4Integrator),
                "ExpEuler" => Box::new(ExpEulerIntegrator),
                "ExpRK4"   => Box::new(ExpRK4Integrator),
                _          => unreachable!(),
            };
            let mut sim = MultirotorSimulator::new(conv_params, integrator);
            let steps   = (conv_end_time / dt_val).round() as usize;
            for _ in 0..steps { sim.step(&conv_action); }

            let pos_error = (sim.state().position.z - z_ref).abs();
            writeln!(csv, "{:.6},{:.6e}", dt_val, pos_error)
                .expect("Failed to write row");
            print!("  {:.2e}", pos_error);
        }
        println!();
    }
    println!();
    println!("  → results/assignment1/data/convergence_{{Euler,RK4,ExpEuler,ExpRK4}}.csv");
    println!("  Expected log-log slope: Euler/ExpEuler = 1.0  (first-order)");
    println!("                          RK4/ExpRK4     = 4.0  (fourth-order)");
}

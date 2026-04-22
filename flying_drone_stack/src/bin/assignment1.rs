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

    // ── Scenario 3: Real Flight Validation (slide 34) ────────────────────────
    // k=1..10 step-ahead position prediction vs actual logged flight states.
    println!("\n─────────────────────────────────────────────────────────────────");
    println!("Scenario 3: Real-flight validation (slide 34)");
    println!("k=1..10 step-ahead prediction vs actual Crazyflie flight data");
    println!("─────────────────────────────────────────────────────────────────");

    // Allow overriding the CSV via a command-line argument --csv=PATH
    let csv_path = std::env::args()
        .find(|a| a.starts_with("--csv="))
        .map(|a| a[6..].to_string())
        .unwrap_or_else(|| {
            "runs/circle_2026-03-17_19-35-31.csv".to_string()
        });
    println!("Flight CSV: {}", csv_path);

    real_flight_validation(&csv_path, &params);
}

// ── Real-flight validation helpers ────────────────────────────────────────

struct FlightRow {
    time_s:    f32,
    pos:       Vec3,
    vel:       Vec3,
    roll_deg:  f32,
    pitch_deg: f32,
    yaw_deg:   f32,
    gyro_rad:  Vec3,   // body-frame angular velocity in rad/s
    acc_g:     Vec3,   // body-frame specific force in G (1 G = 9.81 m/s²)
}

fn parse_f32_col(s: &str) -> f32 {
    s.trim().parse::<f32>().unwrap_or(0.0)
}

fn load_flight_csv(path: &str) -> Vec<FlightRow> {
    let content = match std::fs::read_to_string(path) {
        Ok(c) => c,
        Err(e) => { eprintln!("  Cannot open '{}': {}", path, e); return vec![]; }
    };
    let deg2rad = std::f32::consts::PI / 180.0;
    content.lines()
        .skip(1)   // skip header
        .filter(|l| !l.trim().is_empty())
        .filter_map(|line| {
            let cols: Vec<&str> = line.split(',').collect();
            if cols.len() < 18 { return None; }
            Some(FlightRow {
                time_s:    parse_f32_col(cols[0]) / 1000.0,
                pos:       Vec3::new(parse_f32_col(cols[1]),
                                     parse_f32_col(cols[2]),
                                     parse_f32_col(cols[3])),
                vel:       Vec3::new(parse_f32_col(cols[4]),
                                     parse_f32_col(cols[5]),
                                     parse_f32_col(cols[6])),
                roll_deg:  parse_f32_col(cols[7]),
                pitch_deg: parse_f32_col(cols[8]),
                yaw_deg:   parse_f32_col(cols[9]),
                gyro_rad:  Vec3::new(parse_f32_col(cols[12]) * deg2rad,
                                     parse_f32_col(cols[13]) * deg2rad,
                                     parse_f32_col(cols[14]) * deg2rad),
                acc_g:     Vec3::new(parse_f32_col(cols[15]),
                                     parse_f32_col(cols[16]),
                                     parse_f32_col(cols[17])),
            })
        })
        .collect()
}

/// ZYX (aerospace) Euler angles → unit quaternion.
fn euler_deg_to_quat(roll_deg: f32, pitch_deg: f32, yaw_deg: f32) -> Quat {
    let d2r = std::f32::consts::PI / 180.0;
    let r = roll_deg  * d2r * 0.5;
    let p = pitch_deg * d2r * 0.5;
    let y = yaw_deg   * d2r * 0.5;
    let (sr, cr) = (r.sin(), r.cos());
    let (sp, cp) = (p.sin(), p.cos());
    let (sy, cy) = (y.sin(), y.cos());
    Quat::new(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
}

/// Reconstruct a MotorAction from logged IMU data at row k.
///
/// Thrust  = m · |acc_body| · g    (accelerometer magnitude = specific force)
/// Torque  = J · α + ω × (J·ω)    (Euler's equation; α from central-diff gyro)
fn reconstruct_motor_action(rows: &[FlightRow], k: usize, params: &MultirotorParams) -> MotorAction {
    // Total thrust from accelerometer magnitude (body-frame specific force)
    let a = rows[k].acc_g;
    let acc_mag = (a.x*a.x + a.y*a.y + a.z*a.z).sqrt().max(0.0);
    let thrust = params.mass * acc_mag * params.gravity;

    // Angular acceleration: central difference of gyro, forward/backward at ends
    let (dt_diff, omega_next, omega_prev) = if k > 0 && k + 1 < rows.len() {
        let dt = rows[k+1].time_s - rows[k-1].time_s;
        (dt, rows[k+1].gyro_rad, rows[k-1].gyro_rad)
    } else if k == 0 && rows.len() > 1 {
        let dt = (rows[1].time_s - rows[0].time_s) * 2.0;
        (dt, rows[1].gyro_rad, rows[0].gyro_rad)
    } else {
        let dt = (rows[k].time_s - rows[k-1].time_s) * 2.0;
        (dt, rows[k].gyro_rad, rows[k-1].gyro_rad)
    };
    let dt_safe = dt_diff.max(1e-4);
    let alpha = Vec3::new(
        (omega_next.x - omega_prev.x) / dt_safe,
        (omega_next.y - omega_prev.y) / dt_safe,
        (omega_next.z - omega_prev.z) / dt_safe,
    );

    let w = rows[k].gyro_rad;
    let jxx = params.inertia[0][0];
    let jyy = params.inertia[1][1];
    let jzz = params.inertia[2][2];
    // τ = J·α + ω × (J·ω)
    let tau = Vec3::new(
        jxx * alpha.x + (jzz - jyy) * w.y * w.z,
        jyy * alpha.y + (jxx - jzz) * w.x * w.z,
        jzz * alpha.z + (jyy - jxx) * w.x * w.y,
    );

    MotorAction::from_thrust_torque(thrust, tau, params)
}

fn real_flight_validation(csv_path: &str, params: &MultirotorParams) {
    let rows = load_flight_csv(csv_path);
    if rows.is_empty() {
        println!("  No data loaded — skipping real-flight validation.");
        return;
    }
    println!("  Loaded {} rows  ({:.1} s)", rows.len(),
             rows.last().map(|r| r.time_s).unwrap_or(0.0));

    // Only consider rows where the drone is airborne
    let max_horizon = 10usize;
    let flight_start = rows.iter().position(|r| r.pos.z > 0.15).unwrap_or(0);
    let flight_end   = rows.len().saturating_sub(max_horizon + 1);
    if flight_start >= flight_end {
        println!("  Not enough in-flight rows (need z > 0.15 m). Skipping.");
        return;
    }
    let avg_dt = if rows.len() > 1 {
        (rows.last().unwrap().time_s - rows[flight_start].time_s)
            / (rows.len() - flight_start - 1) as f32
    } else { 0.1 };
    println!("  In-flight rows: {} … {}  (avg dt = {:.1} ms)",
             flight_start, flight_end, avg_dt * 1000.0);

    const N_INT: usize = 4;
    let int_names = ["Euler", "RK4", "ExpEuler", "ExpRK4"];
    let mut sum_sq   = [[0.0f32; 10]; N_INT];
    let mut n_samp   = [0usize; 10];

    // Sub-step each ~100ms CSV interval at dt=10ms for accuracy
    let sub_dt    = 0.01_f32;
    let sub_steps = ((avg_dt / sub_dt).round() as usize).max(1);
    let mut sub_params = params.clone();
    sub_params.dt = sub_dt;
    sub_params.motor_time_constant = 0.0;  // bypass motor lag for clean comparison

    // Sample every 5 rows so prediction windows don't overlap
    for k0 in (flight_start..flight_end).step_by(5) {
        let r0 = &rows[k0];
        let q0 = euler_deg_to_quat(r0.roll_deg, r0.pitch_deg, r0.yaw_deg);
        let init = MultirotorState::with_initial(r0.pos, r0.vel, q0, r0.gyro_rad);

        for (ii, iname) in int_names.iter().enumerate() {
            let integrator: Box<dyn Integrator> = match *iname {
                "Euler"    => Box::new(EulerIntegrator),
                "RK4"      => Box::new(RK4Integrator),
                "ExpEuler" => Box::new(ExpEulerIntegrator),
                _          => Box::new(ExpRK4Integrator),
            };
            let mut sim = MultirotorSimulator::new(sub_params.clone(), integrator);
            sim.set_state(init.clone());

            for h in 0..max_horizon {
                if k0 + h + 1 >= rows.len() { break; }

                let action = reconstruct_motor_action(&rows, k0 + h, &sub_params);
                for _ in 0..sub_steps { sim.step(&action); }

                let rp = rows[k0 + h + 1].pos;
                let sp = sim.state().position;
                let err_sq = (rp.x - sp.x).powi(2)
                           + (rp.y - sp.y).powi(2)
                           + (rp.z - sp.z).powi(2);
                sum_sq[ii][h] += err_sq;
                if ii == 0 { n_samp[h] += 1; }
            }
        }
    }

    // Print RMS error table
    println!("\n  Position RMS error [mm] vs prediction horizon k (each step ≈ {:.0} ms):",
             avg_dt * 1000.0);
    print!("  {:10}", "k steps →");
    for h in 1..=max_horizon { print!("   k={:02}", h); }
    println!();
    println!("  {}", "-".repeat(10 + max_horizon * 7));
    for ii in 0..N_INT {
        print!("  {:10}", int_names[ii]);
        for h in 0..max_horizon {
            let n = n_samp[h] as f32;
            if n > 0.0 {
                let rms_mm = (sum_sq[ii][h] / n).sqrt() * 1000.0;
                print!("  {:5.1}", rms_mm);
            } else {
                print!("    N/A");
            }
        }
        println!("  mm");
    }
    println!("  (n = {} starting points)", n_samp[0]);

    // Write CSV
    std::fs::create_dir_all("results/assignment1/data").expect("create dir");
    let out_path = "results/assignment1/data/real_flight_validation.csv";
    let mut f = File::create(out_path).expect("create real_flight_validation.csv");
    let header: String = {
        let mut h = "horizon_steps,horizon_s".to_string();
        for name in &int_names { h.push_str(&format!(",{}_rms_mm", name)); }
        h
    };
    writeln!(f, "{}", header).unwrap();
    for h in 0..max_horizon {
        let n = n_samp[h] as f32;
        write!(f, "{},{:.3}", h + 1, (h + 1) as f32 * avg_dt).unwrap();
        for ii in 0..N_INT {
            if n > 0.0 {
                write!(f, ",{:.3}", (sum_sq[ii][h] / n).sqrt() * 1000.0).unwrap();
            } else {
                write!(f, ",NaN").unwrap();
            }
        }
        writeln!(f).unwrap();
    }
    println!("\n  Saved: {}", out_path);
}

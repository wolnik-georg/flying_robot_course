//! Assignment 5: Safe-space trajectories (hover / circle / figure-8)
//!
//! This binary produces trajectories that fit inside a small safety box
//! (1.0 m x 1.0 m footprint, up to 0.30 m height) and runs open-loop and
//! closed-loop simulations using the MEKF estimator for controller/safety input.
use multirotor_simulator::prelude::*;
use multirotor_simulator::safety::{SafetyLimits, check_safety};
use std::fs::File;
use std::io::Write;
use std::fs;
use serde::Deserialize;

fn main() {
    println!("Assignment 5: Safe-space hover / circle / figure-8");

    // Simulation timestep
    let dt = 0.005_f32; // 200 Hz sim for faster runs but good resolution
    let mut params = MultirotorParams::crazyflie();
    params.dt = dt;

    // Safety box (1.0 x 1.0 m footprint, 0.30 m max height), centered at start
    let half_xy = 0.5_f32;
    let max_h = 0.30_f32;
    let safety = SafetyLimits {
        min_altitude: 0.0,
        max_altitude: max_h,
        max_speed: 1.0, // conservative
        x_min: -half_xy,
        x_max: half_xy,
        y_min: -half_xy,
        y_max: half_xy,
    };

    println!("Using safety box: x/y in [{:.2}, {:.2}] m, z in [{:.2}, {:.2}] m",
        safety.x_min, safety.x_max, safety.min_altitude, safety.max_altitude);

    // Parse requested trajectory and optional config file from argv: hover / circle / figure8
    let args: Vec<String> = std::env::args().collect();
    // default values (may be overridden by config file)
    let mut mode = if args.len() > 1 { args[1].as_str() } else { "circle" }.to_string();
    let mut speed_scale: f32 = if args.len() > 2 { args[2].parse().unwrap_or(1.0) } else { 1.0 };
    let mut gain_scale: f32 = if args.len() > 3 { args[3].parse().unwrap_or(1.0) } else { 1.0 };
    let mut thrust_max: f32 = if args.len() > 4 { args[4].parse().unwrap_or(1e6) } else { 1e6 };
    let mut motor_time_constant: f32 = params.motor_time_constant;
    let mut thrust_rate_limit_arg: f32 = 20.0_f32;

    // Simple TOML config support: --config path/to/flight_defaults.toml
    #[derive(Deserialize, Debug)]
    struct FlightConfig {
        mode: Option<String>,
        speed_scale: Option<f32>,
        gain_scale: Option<f32>,
        thrust_max: Option<f32>,
        motor_time_constant: Option<f32>,
        thrust_rate_limit: Option<f32>,
    }

    if let Some(pos) = args.iter().position(|a| a == "--config") {
        if pos + 1 < args.len() {
            let cfg_path = &args[pos + 1];
            match fs::read_to_string(cfg_path) {
                Ok(s) => match toml::from_str::<FlightConfig>(&s) {
                    Ok(cfg) => {
                        if let Some(m) = cfg.mode { mode = m; }
                        if let Some(ss) = cfg.speed_scale { speed_scale = ss; }
                        if let Some(gs) = cfg.gain_scale { gain_scale = gs; }
                        if let Some(tm) = cfg.thrust_max { thrust_max = tm; }
                        if let Some(mt) = cfg.motor_time_constant { motor_time_constant = mt; }
                        if let Some(tr) = cfg.thrust_rate_limit { thrust_rate_limit_arg = tr; }
                        println!("Loaded config from {}", cfg_path);
                    }
                    Err(e) => println!("Failed to parse config {}: {}", cfg_path, e),
                },
                Err(e) => println!("Could not read config {}: {}", cfg_path, e),
            }
        }
    }

    // CLI positional overrides (old behavior): args[5]=motor_tau, args[6]=thrust_rate_limit
    if args.len() > 5 { motor_time_constant = args[5].parse().unwrap_or(motor_time_constant); }
    if args.len() > 6 { thrust_rate_limit_arg = args[6].parse().unwrap_or(thrust_rate_limit_arg); }

    // Build a SequencedTrajectory: pre-hover, main pattern, post-hover
    // Use a shared start_z so the planned initial hover matches the simulator initial altitude
    let start_z = 0.2_f32;
    let pre_hover = (1.0_f32, Box::new(CircleTrajectory::new(0.0, start_z, 0.0)) as Box<dyn Trajectory>); // hover placeholder

    let main_phase: (f32, Box<dyn Trajectory>) = match mode.as_str() {
        "hover" => (6.0_f32, Box::new(CircleTrajectory::new(0.0, 0.25, 0.0))),
        "figure8" => {
            // Use the new smooth figure-8 trajectory (Lissajous parametric) which
            // produces C-infinite position/velocity/acceleration/jerk. Keep the old
            // polynomial Figure8Trajectory in the repo to avoid losing it.
            let f8 = SmoothFigure8Trajectory::with_params(8.0 * speed_scale, 0.25, 0.25);
            // duration scaled to match previous main phase sizing
            (12.0_f32 * speed_scale, Box::new(f8))
        }
        _ => {
            // default circle: radius 0.4 m, height 0.25 m
            let mut c = CircleTrajectory::new(0.4, 0.25, std::f32::consts::PI / 6.0);
            c.center = (0.0, 0.0);
            (12.0_f32 * speed_scale, Box::new(c))
        }
    };

    let post_hover = (1.0_f32, Box::new(CircleTrajectory::new(0.0, start_z, 0.0)) as Box<dyn Trajectory>);

    let seq = SequencedTrajectory::new(vec![pre_hover, main_phase, post_hover]);

    // Sampling
    let total_time = seq.duration().unwrap_or(0.0);
    let n_steps = (total_time / dt) as usize;

    // --- Write planned trajectory CSV (sampled) ---
    std::fs::create_dir_all("results/assignment5/data").expect("Failed to create results/assignment5/data");
    let planned_path = format!("results/assignment5/data/assignment5_planned_{}.csv", mode);
    write_planned_csv_traj(&planned_path, &seq, dt, n_steps, &params);
    println!("Wrote planned trajectory: {}", planned_path);

    // --- Sample planned references into a vector and apply smoothing on accel/jerk ---
    let mut sampled_refs: Vec<TrajectoryReference> = Vec::with_capacity(n_steps + 1);
    for k in 0..n_steps {
        let t_k = k as f32 * dt;
        sampled_refs.push(seq.get_reference(t_k));
    }

    // Moving-average window (odd) on acceleration and jerk to reduce large planned jerk spikes
    let win: usize = 7; // ~7 samples -> 0.035 s smoothing
    let half = win / 2;
    // Helper to average Vec3 over indices [i-half .. i+half]
    for i in 0..sampled_refs.len() {
        let lo = if i >= half { i - half } else { 0 };
        let hi = if i + half < sampled_refs.len() { i + half } else { sampled_refs.len() - 1 };
        let mut ax = 0.0_f32; let mut ay = 0.0_f32; let mut az = 0.0_f32;
        let mut jx = 0.0_f32; let mut jy = 0.0_f32; let mut jz = 0.0_f32;
        let mut cnt = 0_f32;
        for k in lo..=hi {
            ax += sampled_refs[k].acceleration.x;
            ay += sampled_refs[k].acceleration.y;
            az += sampled_refs[k].acceleration.z;
            jx += sampled_refs[k].jerk.x;
            jy += sampled_refs[k].jerk.y;
            jz += sampled_refs[k].jerk.z;
            cnt += 1.0;
        }
        sampled_refs[i].acceleration = Vec3::new(ax / cnt, ay / cnt, az / cnt);
        sampled_refs[i].jerk = Vec3::new(jx / cnt, jy / cnt, jz / cnt);
    }

    // Closed-loop: simulator + controller + MEKF estimation
    // Apply optional motor time constant override (if provided)
    params.motor_time_constant = motor_time_constant;
    let mut sim_cl = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    // Initialise at hover start (center)
    let start_pos = Vec3::new(0.0, 0.0, 0.2);
    sim_cl.state_mut().position = start_pos;
    sim_cl.state_mut().velocity = Vec3::zero();
    sim_cl.state_mut().orientation = Quat::identity();
    sim_cl.state_mut().angular_velocity = Vec3::zero();

    let mut controller = GeometricController::default();

    // MEKF for estimation (use defaults)
    let mut mekf = Mekf::new(MekfParams::default());
    // seed with simulator initial quaternion
    mekf.seed_qref([sim_cl.state().orientation.w, sim_cl.state().orientation.x, sim_cl.state().orientation.y, sim_cl.state().orientation.z]);

    // Seed MEKF initial position and body-velocity from simulator to improve convergence
    // MEKF state x layout: [p0,p1,p2, b0,b1,b2, delta0,delta1,delta2]
    mekf.state.x[0] = sim_cl.state().position.x;
    mekf.state.x[1] = sim_cl.state().position.y;
    mekf.state.x[2] = sim_cl.state().position.z;
    // Convert simulator (world) velocity to body-frame velocity estimate
    // Use provided orientation method to rotate velocity into body frame
    let v_body_vec = sim_cl.state().orientation.conjugate().rotate_vector(sim_cl.state().velocity);
    mekf.state.x[3] = v_body_vec.x;
    mekf.state.x[4] = v_body_vec.y;
    mekf.state.x[5] = v_body_vec.z;

    // For finite-difference acceleration estimate
    let mut prev_vel = sim_cl.state().velocity;

    let mut cl_records: Vec<Record> = Vec::with_capacity(n_steps + 1);

    // Last commanded action (start with hover)
    let mut last_action = MotorAction::hover();
    // For actuator slew-rate limiting: keep last commanded thrust/torque
    let mut last_cmd_thrust: f32 = params.mass as f32 * params.gravity as f32; // start near hover thrust
    let mut last_cmd_torque: Vec3 = Vec3::zero();
    // Conservative rate limits (can be tuned): thrust in N/s, torque in Nm/s
    let thrust_rate_limit: f32 = thrust_rate_limit_arg; // N/s (overridable via CLI arg)
    let torque_rate_limit: f32 = 0.8; // Nm/s per axis

    let mut emergency_active = false;
    let mut emergency_hover_steps = 0usize;
    let emergency_hover_duration = (0.5_f32 / dt) as usize;

    for k in 0..n_steps {
        let t = k as f32 * dt;

    // --- Planned reference ---
    // Use the pre-sampled (and smoothed) references so the controller sees the
    // smoothed acceleration/jerk instead of the raw trajectory provider.
    let reference = sampled_refs[k];

        // --- Simulated sensors (from true simulator state) ---
        // Gyro: angular velocity [rad/s] -> degrees/s for MEKF
        let gyro_rads = sim_cl.state().angular_velocity;
        let gyro_deg = [gyro_rads.x.to_degrees(), gyro_rads.y.to_degrees(), gyro_rads.z.to_degrees()];

        // Approximate linear acceleration by finite difference of velocity
        let vel = sim_cl.state().velocity;
        let lin_acc = (vel - prev_vel) * (1.0 / dt);
        prev_vel = vel;

        // Specific force: a - g
        let gravity_vec = Vec3::new(0.0, 0.0, -params.gravity);
        let a_minus_g = lin_acc - gravity_vec;
        // Rotate to body frame
        let body_acc = sim_cl.state().orientation.conjugate().rotate_vector(a_minus_g);
        let accel_g = [body_acc.x / params.gravity, body_acc.y / params.gravity, body_acc.z / params.gravity];

        // Range sensor (mm), clamp to positive
        let range_mm = (sim_cl.state().position.z.max(0.0) * 1000.0) as f32;

        // --- Synthetic optical flow ---
        // MEKF expects flow deltas (dnx, dny). Use the same simple model as the MEKF
        // implementation: d = dt * NP / (pz * THETA_P) * [bx, by]
        // NP and THETA_P are hard-coded here to match `mekf.rs`.
        let (flow_dnx_opt, flow_dny_opt) = if sim_cl.state().position.z.abs() < 0.05 {
            (None, None)
        } else {
            let pz = sim_cl.state().position.z;
            // constants from mekf.rs
            let scale = dt * 350.0_f32 / (pz * 0.71674_f32);
            // body-frame velocity
            let bv = sim_cl.state().orientation.conjugate().rotate_vector(sim_cl.state().velocity);
            (Some(scale * bv.x), Some(scale * bv.y))
        };

        // Feed MEKF (gyro in deg/s, accel in g, range in mm, optional flow)
        let est = mekf.feed_row(t, Some(gyro_deg), Some(accel_g), Some(range_mm), flow_dnx_opt, flow_dny_opt);

        // Extract MEKF outputs if available for logging
        let (mekf_roll, mekf_pitch, mekf_yaw, mekf_px, mekf_py, mekf_pz) = if let Some(est_vec) = &est {
            (est_vec[0], est_vec[1], est_vec[2], est_vec[3], est_vec[4], est_vec[5])
        } else {
            (std::f32::NAN, std::f32::NAN, std::f32::NAN, std::f32::NAN, std::f32::NAN, std::f32::NAN)
        };
        // Construct estimated state for controller/safety
        let estimated_state = if let Some(est_vec) = est {
            // est_vec = [roll, pitch, yaw, px, py, pz]
            let roll = est_vec[0];
            let pitch = est_vec[1];
            let yaw = est_vec[2];
            let px = est_vec[3];
            let py = est_vec[4];
            let pz = est_vec[5];

            // Build rotation matrix from ZYX Euler angles (yaw, pitch, roll)
            let rot = euler_zyx_to_rot_matrix(roll, pitch, yaw);
            let q = rot_to_quat(&rot);

            MultirotorState::with_initial(
                Vec3::new(px, py, pz),
                Vec3::zero(), // velocity not provided directly here
                Quat::new(q[0], q[1], q[2], q[3]),
                sim_cl.state().angular_velocity,
            )
        } else {
            // fallback: use simulator true state
            sim_cl.state().clone()
        };

    // --- Safety check using simulator true state (match assignment4) ---
    let safety_status = check_safety(&safety, sim_cl.state().position, sim_cl.state().velocity);
        if !safety_status.altitude_ok || !safety_status.speed_ok || !safety_status.geofence_ok {
            if !emergency_active {
                println!("[EMERGENCY] Safety violation at t={:.3} s. Entering hover.", t);
                emergency_active = true;
                emergency_hover_steps = 0;
            }
        }

        let mut exec_ref = TrajectoryReference { ..reference };
        if emergency_active {
            if emergency_hover_steps < emergency_hover_duration {
                exec_ref.position = sim_cl.state().position;
                exec_ref.velocity = Vec3::zero();
                exec_ref.acceleration = Vec3::zero();
                exec_ref.jerk = Vec3::zero();
                exec_ref.yaw = to_euler(sim_cl.state().orientation).2;
                exec_ref.yaw_rate = 0.0;
                exec_ref.yaw_acceleration = 0.0;
                emergency_hover_steps += 1;
            } else {
                exec_ref.position = sim_cl.state().position;
                exec_ref.velocity = Vec3::new(0.0, 0.0, -0.05);
            }
        }

        // Controller uses simulator true state (same approach as assignment4 for parity)
        let true_state_for_ctrl = sim_cl.state().clone();
        let ctrl_out = controller.compute_control(&true_state_for_ctrl, &exec_ref, &params, dt);
        // Desired controller outputs (before any actuator limits)
        let mut desired_thrust = ctrl_out.thrust;
        let mut desired_torque = ctrl_out.torque * gain_scale; // apply experimental gain scaling on torque

        // Thrust finite check and clamp to absolute max
        if !desired_thrust.is_finite() {
            desired_thrust = params.mass as f32 * params.gravity as f32; // fallback to hover
        }
        desired_thrust = desired_thrust.clamp(0.0, thrust_max);

        // --- Actuator slew-rate limiting (per-step) ---
        let max_dt_thrust = thrust_rate_limit * dt;
        let delta_thrust = (desired_thrust - last_cmd_thrust).clamp(-max_dt_thrust, max_dt_thrust);
        let cmd_thrust = last_cmd_thrust + delta_thrust;

        let max_dt_torque = torque_rate_limit * dt;
        let delta_tx = (desired_torque.x - last_cmd_torque.x).clamp(-max_dt_torque, max_dt_torque);
        let delta_ty = (desired_torque.y - last_cmd_torque.y).clamp(-max_dt_torque, max_dt_torque);
        let delta_tz = (desired_torque.z - last_cmd_torque.z).clamp(-max_dt_torque, max_dt_torque);
        let cmd_torque = Vec3::new(last_cmd_torque.x + delta_tx, last_cmd_torque.y + delta_ty, last_cmd_torque.z + delta_tz);

        // Update last commanded values
        last_cmd_thrust = cmd_thrust;
        last_cmd_torque = cmd_torque;

        // Convert limited commands to motor action
        let action = MotorAction::from_thrust_torque(cmd_thrust, cmd_torque, &params);
        last_action = action.clone();

        // Record closed-loop data
        cl_records.push(Record { t,
            ref_x: reference.position.x, ref_y: reference.position.y, ref_z: reference.position.z,
            sim_x: sim_cl.state().position.x, sim_y: sim_cl.state().position.y, sim_z: sim_cl.state().position.z,
            ref_vx: reference.velocity.x, ref_vy: reference.velocity.y, ref_vz: reference.velocity.z,
            sim_vx: sim_cl.state().velocity.x, sim_vy: sim_cl.state().velocity.y, sim_vz: sim_cl.state().velocity.z,
            ref_thrust: desired_thrust,
            ref_tx: desired_torque.x, ref_ty: desired_torque.y, ref_tz: desired_torque.z,
            ref_yaw: reference.yaw,
            ref_ox: 0.0, ref_oy: 0.0, ref_oz: 0.0,
            cmd_thrust: cmd_thrust, cmd_tx: cmd_torque.x, cmd_ty: cmd_torque.y, cmd_tz: cmd_torque.z,
            mekf_roll, mekf_pitch, mekf_yaw, mekf_px, mekf_py, mekf_pz,
        });

        // Step simulator with computed action
        sim_cl.step(&action);
    }

    // Write closed-loop CSV
    let cl_path = format!("results/assignment5/data/assignment5_closedloop_{}.csv", mode);
    write_record_csv(&cl_path, &cl_records);
    println!("Wrote closed-loop CSV: {}", cl_path);

    // Simple stats
    println!("Closed-loop simulation finished for mode='{}'.", mode);
}

// ---------------------- Helper code & types ------------------------------
struct Record {
    t: f32,
    ref_x: f32, ref_y: f32, ref_z: f32,
    sim_x: f32, sim_y: f32, sim_z: f32,
    ref_vx: f32, ref_vy: f32, ref_vz: f32,
    sim_vx: f32, sim_vy: f32, sim_vz: f32,
    ref_thrust: f32,
    ref_tx: f32, ref_ty: f32, ref_tz: f32,
    ref_yaw: f32,
    ref_ox: f32, ref_oy: f32, ref_oz: f32,
    cmd_thrust: f32,
    cmd_tx: f32, cmd_ty: f32, cmd_tz: f32,
    // MEKF estimated pose (optional -- NaN if unavailable)
    mekf_roll: f32, mekf_pitch: f32, mekf_yaw: f32,
    mekf_px: f32, mekf_py: f32, mekf_pz: f32,
}

fn write_record_csv(path: &str, records: &[Record]) {
    let mut f = File::create(path).expect("Could not create CSV file");
    writeln!(f,
        "t,ref_x,ref_y,ref_z,sim_x,sim_y,sim_z,ref_vx,ref_vy,ref_vz,sim_vx,sim_vy,sim_vz,ref_thrust,ref_tx,ref_ty,ref_tz,ref_yaw,ref_ox,ref_oy,ref_oz,cmd_thrust,cmd_tx,cmd_ty,cmd_tz,mekf_roll,mekf_pitch,mekf_yaw,mekf_px,mekf_py,mekf_pz"
    ).unwrap();
    for r in records {
        writeln!(f,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            r.t,
            r.ref_x, r.ref_y, r.ref_z,
            r.sim_x, r.sim_y, r.sim_z,
            r.ref_vx, r.ref_vy, r.ref_vz,
            r.sim_vx, r.sim_vy, r.sim_vz,
            r.ref_thrust,
            r.ref_tx, r.ref_ty, r.ref_tz,
            r.ref_yaw,
            r.ref_ox, r.ref_oy, r.ref_oz,
            r.cmd_thrust,
            r.cmd_tx, r.cmd_ty, r.cmd_tz,
            r.mekf_roll, r.mekf_pitch, r.mekf_yaw,
            r.mekf_px, r.mekf_py, r.mekf_pz,
        ).unwrap();
    }
}

fn write_planned_csv_traj(path: &str, traj: &dyn Trajectory, dt: f32, n_steps: usize, params: &MultirotorParams) {
    let mut f = File::create(path).expect("Could not create planned CSV file");
    writeln!(f, "t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw,yaw_dot,yaw_ddot,thrust,tx,ty,tz,ox,oy,oz").unwrap();
    for k in 0..n_steps {
        let t = k as f32 * dt;
        let r = traj.get_reference(t);
        // approximate flatness outputs: compute thrust/torque from desired accel/yaw
        let flat = FlatOutput { pos: r.position, vel: r.velocity, acc: r.acceleration, jerk: r.jerk, snap: Vec3::zero(), yaw: r.yaw, yaw_dot: r.yaw_rate, yaw_ddot: r.yaw_acceleration };
        let fr = compute_flatness(&flat, params.mass);
        writeln!(f, "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            t,
            r.position.x, r.position.y, r.position.z,
            r.velocity.x, r.velocity.y, r.velocity.z,
            r.acceleration.x, r.acceleration.y, r.acceleration.z,
            r.jerk.x, r.jerk.y, r.jerk.z,
            r.yaw, r.yaw_rate, r.yaw_acceleration,
            fr.thrust, fr.torque.x, fr.torque.y, fr.torque.z,
            fr.omega.x, fr.omega.y, fr.omega.z,
        ).unwrap();
    }
}

// Convert ZYX Euler angles (roll, pitch, yaw) to rotation matrix [[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]]
fn euler_zyx_to_rot_matrix(roll: f32, pitch: f32, yaw: f32) -> [[f32;3];3] {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    let r00 = cy * cp;
    let r01 = cy * sp * sr - sy * cr;
    let r02 = cy * sp * cr + sy * sr;

    let r10 = sy * cp;
    let r11 = sy * sp * sr + cy * cr;
    let r12 = sy * sp * cr - cy * sr;

    let r20 = -sp;
    let r21 = cp * sr;
    let r22 = cp * cr;

    // rot_to_quat expects columns stored as [[xb],[yb],[zb]] where each is [x,y,z]
    // We must return in the same column-major style used elsewhere: rot[0] = xb column
    [
        [r00, r10, r20],
        [r01, r11, r21],
        [r02, r12, r22],
    ]
}

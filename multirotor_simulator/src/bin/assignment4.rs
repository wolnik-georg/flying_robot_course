//! Assignment 4: Motion Planning with Minimum-Snap Splines and Differential Flatness
//!
//! This binary:
//! 1. Plans an aggressive figure-8 trajectory using 8th-order polynomial splines
//!    that minimise the integral of snap² (via QP / Clarabel solver).
//! 2. Computes desired forces and torques via differential flatness (Faessler 2018).
//! 3. Open-loop validation: feeds flatness-computed actions directly to the simulator.
//! 4. Closed-loop validation: tracks the spline using the geometric controller.
//! 5. Writes three CSV files for analysis and plotting.

use multirotor_simulator::prelude::*;
use multirotor_simulator::safety::{SafetyLimits, check_safety};
use std::fs::File;
use std::io::Write;

fn main() {
    println!("Assignment 4: Minimum-Snap Spline Planning + Differential Flatness");
    println!("====================================================================\n");

    // ── Aircraft parameters ──────────────────────────────────────────────────
    // Use 1 kHz physics timestep so the integrator actually steps at 1 ms.
    // The default crazyflie() has dt=0.01 (100 Hz) which is correct for
    // assignment2; here we override to 0.001 for higher-resolution planning.
    let dt = 0.001_f32;
    let mut params = MultirotorParams::crazyflie();
    params.dt = dt;


    // ── Waypoints: Add pre-hover and post-hover ─────────────────────────────
    // Start with a hover at the initial position
    let hover_pos = Vec3::new(0.0, 0.0, 1.0);
    let hover_yaw = 0.0;
    let hover_duration = 1.5_f32; // seconds to hover before and after

    // Main figure-8 waypoints
    let main_waypoints = vec![
        Waypoint { pos: hover_pos, yaw: hover_yaw },
        Waypoint { pos: Vec3::new( 1.0,  0.5, 1.1), yaw: 0.3 },
        Waypoint { pos: Vec3::new( 0.0,  1.0, 1.0), yaw: std::f32::consts::FRAC_PI_2 },
        Waypoint { pos: Vec3::new(-1.0,  0.5, 0.9), yaw: std::f32::consts::PI },
        Waypoint { pos: hover_pos, yaw: std::f32::consts::PI },
        Waypoint { pos: Vec3::new( 1.0, -0.5, 1.1), yaw: -std::f32::consts::FRAC_PI_2 + std::f32::consts::TAU },
        Waypoint { pos: Vec3::new( 0.0, -1.0, 1.0), yaw: -std::f32::consts::FRAC_PI_2 },
        Waypoint { pos: Vec3::new(-1.0, -0.5, 0.9), yaw: -std::f32::consts::PI },
        Waypoint { pos: hover_pos, yaw: hover_yaw },
    ];

    // Build full waypoint list: pre-hover, main, post-hover
    let mut waypoints = Vec::new();
    waypoints.push(Waypoint { pos: hover_pos, yaw: hover_yaw }); // pre-hover start
    waypoints.extend(main_waypoints.iter().cloned());
    waypoints.push(Waypoint { pos: hover_pos, yaw: hover_yaw }); // post-hover end

    // Durations: pre-hover, main, post-hover
    let mut durations = Vec::new();
    durations.push(hover_duration); // pre-hover
    let seg_time = 1.0_f32;
    let n_main_seg = main_waypoints.len() - 1;
    durations.extend(vec![seg_time; n_main_seg]);
    durations.push(hover_duration); // post-hover

    let n_seg = waypoints.len() - 1;

    println!("Planning minimum-snap spline trajectory with pre/post hover...");
    println!("  {} waypoints, {} segments, {:.1} s total", waypoints.len(), n_seg, durations.iter().sum::<f32>());

    let traj = match SplineTrajectory::plan(&waypoints, &durations) {
        Ok(t) => { println!("  QP solved successfully.\n"); t }
        Err(e) => { eprintln!("Spline planning failed: {}", e); return; }
    };

    // ── Sampling parameters ──────────────────────────────────────────────────
    let total_time = traj.total_time;
    let n_steps = (total_time / dt) as usize;

    // ── Safety limits configuration ────────────────────────────────────────
    let safety = SafetyLimits {
        min_altitude: 0.0,
        max_altitude: 0.25, // 25 cm
        max_speed: 0.3,     // 0.3 m/s
        x_min: -1.0,
        x_max: 1.0,
        y_min: -1.0,
        y_max: 1.0,
    };

    // ── Open-loop simulation ────────────────────────────────────────────────
    // Feed flatness-computed thrust and torque directly to the simulator.
    println!("Running open-loop simulation (flatness → simulator)...");
    let mut sim_ol = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    // Initialise at the first waypoint (hovering)
    let first_flat = traj.eval(0.0);
    let fr0 = compute_flatness(&first_flat, params.mass);
    let q0 = rot_to_quat(&fr0.rot);
    sim_ol.state_mut().position = waypoints[0].pos;
    sim_ol.state_mut().velocity = Vec3::zero();
    sim_ol.state_mut().orientation = Quat::new(q0[0], q0[1], q0[2], q0[3]);
    sim_ol.state_mut().angular_velocity = Vec3::zero();

    let mut ol_records: Vec<Record> = Vec::with_capacity(n_steps + 1);

    for k in 0..n_steps {
        let t = k as f32 * dt;
        let flat = traj.eval(t);
        let fr = compute_flatness(&flat, params.mass);

        // Safety checks (open-loop)
        let state = sim_ol.state().clone();
        let safety_status = check_safety(&safety, state.position, state.velocity);
        let safe_pos = safety_status.clamped_pos;
        let safe_vel = safety_status.clamped_vel;

        if !safety_status.altitude_ok {
            println!("[OPEN-LOOP][WARN] Altitude limit violated at t={:.3} s: z={:.3} m", t, state.position.z);
        }
        if !safety_status.speed_ok {
            println!("[OPEN-LOOP][WARN] Speed limit violated at t={:.3} s: v={:.3} m/s", t, state.velocity.norm());
        }
        if !safety_status.geofence_ok {
            println!("[OPEN-LOOP][WARN] Geofence limit violated at t={:.3} s: x={:.3}, y={:.3}", t, state.position.x, state.position.y);
        }

        ol_records.push(Record {
            t,
            ref_x: flat.pos.x, ref_y: flat.pos.y, ref_z: flat.pos.z,
            sim_x: safe_pos.x, sim_y: safe_pos.y, sim_z: safe_pos.z,
            ref_vx: flat.vel.x, ref_vy: flat.vel.y, ref_vz: flat.vel.z,
            sim_vx: safe_vel.x, sim_vy: safe_vel.y, sim_vz: safe_vel.z,
            ref_thrust: fr.thrust,
            ref_tx: fr.torque.x, ref_ty: fr.torque.y, ref_tz: fr.torque.z,
            ref_yaw: flat.yaw,
            ref_ox: fr.omega.x, ref_oy: fr.omega.y, ref_oz: fr.omega.z,
            // Open-loop: commanded == reference (flatness output fed directly)
            cmd_thrust: fr.thrust,
            cmd_tx: fr.torque.x, cmd_ty: fr.torque.y, cmd_tz: fr.torque.z,
        });

        // Convert to motor action and step
        let action = MotorAction::from_thrust_torque(fr.thrust, fr.torque, &params);
        sim_ol.step(&action);
    }
    println!("  Open-loop simulation complete ({} steps).\n", n_steps);

    // ── Closed-loop simulation ──────────────────────────────────────────────
    println!("Running closed-loop simulation (spline → geometric controller)...");
    let mut sim_cl = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    // Same initial state
    sim_cl.state_mut().position = waypoints[0].pos;
    sim_cl.state_mut().velocity = Vec3::zero();
    sim_cl.state_mut().orientation = Quat::new(q0[0], q0[1], q0[2], q0[3]);
    sim_cl.state_mut().angular_velocity = Vec3::zero();

    let mut controller = GeometricController::default();

    // Emergency hover/landing state
    let mut emergency_active = false;
    let mut emergency_hover_steps = 0;
    let emergency_hover_duration = (0.5 / dt) as usize; // 0.5 s hover before landing

    let mut cl_records: Vec<Record> = Vec::with_capacity(n_steps + 1);

    for k in 0..n_steps {
        let t = k as f32 * dt;
        let flat = traj.eval(t);
        let fr = compute_flatness(&flat, params.mass);

        let state = sim_cl.state().clone();
        let reference = TrajectoryReference {
            position: flat.pos,
            velocity: flat.vel,
            acceleration: flat.acc,
            jerk: flat.jerk,
            yaw: flat.yaw,
            yaw_rate: flat.yaw_dot,
            yaw_acceleration: flat.yaw_ddot,
        };

        // Safety checks (closed-loop)
        let safety_status = check_safety(&safety, state.position, state.velocity);
        let safe_pos = safety_status.clamped_pos;
        let safe_vel = safety_status.clamped_vel;

        if !safety_status.altitude_ok || !safety_status.speed_ok || !safety_status.geofence_ok {
            if !emergency_active {
                println!("[EMERGENCY] Safety violation at t={:.3} s. Entering hover.", t);
                emergency_active = true;
                emergency_hover_steps = 0;
            }
        }

        let mut emergency_ref = reference.clone();
        if emergency_active {
            if emergency_hover_steps < emergency_hover_duration {
                // Stable hover: hold current position, zero velocity
                emergency_ref.position = state.position;
                emergency_ref.velocity = Vec3::zero();
                emergency_ref.acceleration = Vec3::zero();
                emergency_ref.jerk = Vec3::zero();
                emergency_ref.yaw = to_euler(state.orientation).2;
                emergency_ref.yaw_rate = 0.0;
                emergency_ref.yaw_acceleration = 0.0;
                emergency_hover_steps += 1;
            } else {
                // Controlled descent: decrease altitude gently
                let landing_speed = -0.05; // m/s descent
                emergency_ref.position = state.position;
                emergency_ref.velocity = Vec3::new(0.0, 0.0, landing_speed);
                emergency_ref.acceleration = Vec3::zero();
                emergency_ref.jerk = Vec3::zero();
                emergency_ref.yaw = to_euler(state.orientation).2;
                emergency_ref.yaw_rate = 0.0;
                emergency_ref.yaw_acceleration = 0.0;
            }
        }

        let ctrl_out = controller.compute_control(&state, &emergency_ref, &params, dt);
        let action = MotorAction::from_thrust_torque(ctrl_out.thrust, ctrl_out.torque, &params);

        cl_records.push(Record {
            t,
            ref_x: flat.pos.x, ref_y: flat.pos.y, ref_z: flat.pos.z,
            sim_x: safe_pos.x, sim_y: safe_pos.y, sim_z: safe_pos.z,
            ref_vx: flat.vel.x, ref_vy: flat.vel.y, ref_vz: flat.vel.z,
            sim_vx: safe_vel.x, sim_vy: safe_vel.y, sim_vz: safe_vel.z,
            ref_thrust: fr.thrust,
            ref_tx: fr.torque.x, ref_ty: fr.torque.y, ref_tz: fr.torque.z,
            ref_yaw: flat.yaw,
            ref_ox: fr.omega.x, ref_oy: fr.omega.y, ref_oz: fr.omega.z,
            // Closed-loop: record what the geometric controller actually commanded
            cmd_thrust: ctrl_out.thrust,
            cmd_tx: ctrl_out.torque.x, cmd_ty: ctrl_out.torque.y, cmd_tz: ctrl_out.torque.z,
        });

        sim_cl.step(&action);
    }
    println!("  Closed-loop simulation complete ({} steps).\n", n_steps);

    // ── Planned trajectory CSV ───────────────────────────────────────────────
    println!("Writing output files...");
    let planned_path = "results/data/assignment4_planned.csv";
    write_planned_csv(planned_path, &traj, dt, n_steps, &params);

    // ── Open-loop CSV ────────────────────────────────────────────────────────
    let ol_path = "results/data/assignment4_openloop.csv";
    write_record_csv(ol_path, &ol_records);

    // ── Closed-loop CSV ──────────────────────────────────────────────────────
    let cl_path = "results/data/assignment4_closedloop.csv";
    write_record_csv(cl_path, &cl_records);

    println!("  Written: {}", planned_path);
    println!("  Written: {}", ol_path);
    println!("  Written: {}", cl_path);

    // ── Statistics ───────────────────────────────────────────────────────────
    println!();
    println!("Open-loop tracking error statistics:");
    print_errors(&ol_records);
    println!();
    println!("Closed-loop tracking error statistics:");
    print_errors(&cl_records);

    // ── Flatness consistency check ───────────────────────────────────────────
    println!();
    println!("Hover flatness test:");
    let hover_flat = FlatOutput {
        pos: Vec3::new(0.0, 0.0, 1.0),
        vel: Vec3::zero(),
        acc: Vec3::zero(),
        jerk: Vec3::zero(),
        snap: Vec3::zero(),
        yaw: 0.0,
        yaw_dot: 0.0,
        yaw_ddot: 0.0,
    };
    let hover_fr = compute_flatness(&hover_flat, params.mass);
    println!("  thrust = {:.4} N  (expected {:.4} N)",
        hover_fr.thrust, params.mass * params.gravity);
    println!("  |omega| = {:.2e} rad/s  (expected ~0)", hover_fr.omega.norm());
    println!("  |torque| = {:.2e} Nm   (expected ~0)", hover_fr.torque.norm());
}

// ---------------------------------------------------------------------------
// Helper types and functions
// ---------------------------------------------------------------------------

struct Record {
    t: f32,
    ref_x: f32, ref_y: f32, ref_z: f32,
    sim_x: f32, sim_y: f32, sim_z: f32,
    ref_vx: f32, ref_vy: f32, ref_vz: f32,
    sim_vx: f32, sim_vy: f32, sim_vz: f32,
    /// Desired thrust from differential flatness [N]
    ref_thrust: f32,
    ref_tx: f32, ref_ty: f32, ref_tz: f32,
    ref_yaw: f32,
    ref_ox: f32, ref_oy: f32, ref_oz: f32,
    /// Actual commanded thrust (controller output, 0 for open-loop records) [N]
    cmd_thrust: f32,
    cmd_tx: f32, cmd_ty: f32, cmd_tz: f32,
}

fn write_record_csv(path: &str, records: &[Record]) {
    let mut f = File::create(path).expect("Could not create CSV file");
    writeln!(f,
        "t,ref_x,ref_y,ref_z,sim_x,sim_y,sim_z,\
         ref_vx,ref_vy,ref_vz,sim_vx,sim_vy,sim_vz,\
         ref_thrust,ref_tx,ref_ty,ref_tz,ref_yaw,\
         ref_ox,ref_oy,ref_oz,\
         cmd_thrust,cmd_tx,cmd_ty,cmd_tz"
    ).unwrap();
    for r in records {
        writeln!(f,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6}",
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
        ).unwrap();
    }
}

fn write_planned_csv(path: &str, traj: &SplineTrajectory, dt: f32, n_steps: usize, params: &MultirotorParams) {
    let mut f = File::create(path).expect("Could not create planned CSV file");
    writeln!(f,
        "t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,\
         yaw,yaw_dot,yaw_ddot,\
         thrust,tx,ty,tz,\
         ox,oy,oz,odx,ody,odz"
    ).unwrap();

    for k in 0..n_steps {
        let t = k as f32 * dt;
        let flat = traj.eval(t);
        let fr = compute_flatness(&flat, params.mass);
        writeln!(f,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            t,
            flat.pos.x, flat.pos.y, flat.pos.z,
            flat.vel.x, flat.vel.y, flat.vel.z,
            flat.acc.x, flat.acc.y, flat.acc.z,
            flat.jerk.x, flat.jerk.y, flat.jerk.z,
            flat.yaw, flat.yaw_dot, flat.yaw_ddot,
            fr.thrust,
            fr.torque.x, fr.torque.y, fr.torque.z,
            fr.omega.x, fr.omega.y, fr.omega.z,
            fr.omega_dot.x, fr.omega_dot.y, fr.omega_dot.z,
        ).unwrap();
    }
}

fn print_errors(records: &[Record]) {
    let n = records.len() as f32;
    let rms = |vals: Vec<f32>| -> f32 {
        (vals.iter().map(|v| v * v).sum::<f32>() / n).sqrt()
    };

    let ex: Vec<f32> = records.iter().map(|r| r.ref_x - r.sim_x).collect();
    let ey: Vec<f32> = records.iter().map(|r| r.ref_y - r.sim_y).collect();
    let ez: Vec<f32> = records.iter().map(|r| r.ref_z - r.sim_z).collect();
    let ep3d: Vec<f32> = ex.iter().zip(ey.iter()).zip(ez.iter())
        .map(|((x, y), z)| (x*x + y*y + z*z).sqrt())
        .collect();

    println!("  RMS position error:  x={:.4} m  y={:.4} m  z={:.4} m  3D={:.4} m",
        rms(ex), rms(ey), rms(ez), rms(ep3d));

    let evx: Vec<f32> = records.iter().map(|r| r.ref_vx - r.sim_vx).collect();
    let evy: Vec<f32> = records.iter().map(|r| r.ref_vy - r.sim_vy).collect();
    let evz: Vec<f32> = records.iter().map(|r| r.ref_vz - r.sim_vz).collect();
    println!("  RMS velocity error:  vx={:.4} m/s  vy={:.4} m/s  vz={:.4} m/s",
        rms(evx), rms(evy), rms(evz));
}

//! Closed-loop simulation: min-snap spline + differential flatness + geometric controller + dynamics.
//!
//! Simulates the full trajectory-tracking pipeline at 100 Hz:
//!   SplineTrajectory → FlatOutput → compute_flatness → TrajectoryReference
//!   → GeometricController → MotorAction → MultirotorSimulator
//!
//! Also wires the perception module through the simulation:
//!   - SimFlowSensor + SimRangeSensor produce synthetic measurements each step.
//!   - SimCamera renders a frame with 4 world landmarks each step.
//!   - detect_features() counts FAST-9 corners in each frame.
//!   - The feature count is logged to verify the end-to-end perception pipeline.
//!
//! Usage:
//!   cargo run --bin sim_closed_loop             # circle trajectory (default)
//!   cargo run --bin sim_closed_loop -- figure8  # figure-8 trajectory
//!
//! Prints a CSV to stdout (redirect to a file for plotting) and RMSE to stderr.

use multirotor_simulator::planning::{SplineTrajectory, Waypoint, compute_flatness, flatness_to_reference};
use multirotor_simulator::controller::{GeometricController, Controller};
use multirotor_simulator::dynamics::{MultirotorParams, MultirotorSimulator, MotorAction};
use multirotor_simulator::estimation::{Mekf, MekfParams};
use multirotor_simulator::integration::RK4Integrator;
use multirotor_simulator::math::Vec3;
use multirotor_simulator::perception::sensors::sim::{SimFlowSensor, SimRangeSensor, SimCamera};
use multirotor_simulator::perception::processing::features::detect_features;

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_else(|| "circle".into());

    let params = MultirotorParams::crazyflie();
    let (wps, durs) = match mode.as_str() {
        "figure8" => figure8_waypoints(8, 0.5, 0.3, 0.3),
        _         => circle_waypoints(8, 0.4, 0.3),
    };

    eprintln!("Planning {} trajectory ({} waypoints, {:.1}s total)...",
        mode, wps.len(), durs.iter().sum::<f32>());

    let traj = SplineTrajectory::plan(&wps, &durs)
        .unwrap_or_else(|e| { eprintln!("Planning failed: {}", e); std::process::exit(1); });

    eprintln!("Simulating {} at 100 Hz...", mode);

    let mut sim  = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    let mut ctrl = GeometricController::default();

    // ── Perception sensors ─────────────────────────────────────────────────
    let mut flow_sensor  = SimFlowSensor::new(params.dt, 0.05);  // 2 cm/s noise
    let mut range_sensor = SimRangeSensor::new(0.002);             // 2 mm noise
    let mut camera       = SimCamera::default_hm01b0();

    // World landmarks for the synthetic camera (4 points around the trajectory).
    let landmarks = [
        [0.5, 0.0, 0.0f32],
        [0.0, 0.5, 0.0f32],
        [-0.5, 0.0, 0.0f32],
        [0.0, -0.5, 0.0f32],
    ];

    // MEKF (shadow estimator — doesn't influence control, just logged).
    let mut mekf       = Mekf::new(MekfParams::default());
    let mut mekf_seeded = false;

    // ──────────────────────────────────────────────────────────────────────

    // Start the drone at the first waypoint position
    sim.state_mut().position = wps[0].pos;

    let dt     = params.dt; // 0.01 s
    let n_steps = (traj.total_time / dt).ceil() as usize;

    // Accumulators for RMSE
    let mut sum_sq = [0.0f64; 3]; // x, y, z
    let mut count  = 0usize;
    let mut max_err = 0.0f32;

    println!("time_s,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,ep_x,ep_y,ep_z,ep_norm,thrust_n,roll_deg,pitch_deg,flow_dx,flow_dy,range_m,feature_count");

    for step in 0..=n_steps {
        let t = (step as f32 * dt).min(traj.total_time);
        let flat = traj.eval(t);

        // Differential flatness: desired thrust + rotation from trajectory derivatives
        let fr        = compute_flatness(&flat, params.mass);
        let reference = flatness_to_reference(&fr, flat.acc, flat.jerk,
                                              flat.yaw, flat.yaw_dot, flat.yaw_ddot);

        let state    = sim.state().clone();
        let ctrl_out = ctrl.compute_control(&state, &reference, &params, dt);

        // Invert the motor mixer to get motor ω² commands
        let action = thrust_torque_to_motors(&params, ctrl_out.thrust, ctrl_out.torque);
        sim.step(&action);

        // ── Perception measurements ────────────────────────────────────────
        let flow_meas  = flow_sensor.measure(&state);
        let range_meas = range_sensor.measure(&state);
        let frame      = camera.render(&state, &landmarks, (dt * 1000.0) as u64);
        let features   = detect_features(&frame, 20);

        // ── MEKF shadow update ─────────────────────────────────────────────
        // Seed once the drone is airborne (range_z > 0.05 m).
        if !mekf_seeded && state.position.z > 0.05 {
            let (roll, pitch, yaw) = multirotor_simulator::math::to_euler(state.orientation);
            let q0 = euler_to_quat(roll, pitch, yaw);
            mekf.seed_qref(q0);
            mekf.state.x[0] = state.position.x;
            mekf.state.x[1] = state.position.y;
            mekf.state.x[2] = state.position.z;
            mekf_seeded = true;
        }

        if mekf_seeded {
            // Approximate IMU: thrust/mass on body-z, angular velocity from sim.
            let accel_body_z = ctrl_out.thrust / params.mass / 9.81;
            let gyro_dps = [
                state.angular_velocity.x.to_degrees(),
                state.angular_velocity.y.to_degrees(),
                state.angular_velocity.z.to_degrees(),
            ];
            let acc_g = [0.0f32, 0.0f32, accel_body_z];

            // PMW3901 axis convention: vel_x ↔ -flow_dy, vel_y ↔ -flow_dx
            let (dnx, dny) = (-flow_meas.dy_px, -flow_meas.dx_px);
            let range_mm   = if range_meas.range_m > 0.01 {
                Some(range_meas.range_m * 1000.0)
            } else { None };

            let _mekf_out = mekf.feed_row(
                t,
                Some(gyro_dps),
                Some(acc_g),
                range_mm,
                Some(dnx), Some(dny),
            );
        }
        // ──────────────────────────────────────────────────────────────────

        // Tracking error
        let pos    = sim.state().position;
        let ep     = reference.position - pos;
        let ep_norm = (ep.x * ep.x + ep.y * ep.y + ep.z * ep.z).sqrt();

        sum_sq[0] += (ep.x * ep.x) as f64;
        sum_sq[1] += (ep.y * ep.y) as f64;
        sum_sq[2] += (ep.z * ep.z) as f64;
        max_err    = max_err.max(ep_norm);
        count     += 1;

        // Attitude from quaternion
        let q     = state.orientation;
        let roll  = (2.0*(q.w*q.x + q.y*q.z)).atan2(1.0 - 2.0*(q.x*q.x + q.y*q.y));
        let pitch = (2.0*(q.w*q.y - q.z*q.x)).asin();

        println!("{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.2},{:.2},{:.3},{:.3},{:.4},{}",
            t,
            reference.position.x, reference.position.y, reference.position.z,
            pos.x, pos.y, pos.z,
            ep.x, ep.y, ep.z, ep_norm,
            ctrl_out.thrust,
            roll.to_degrees(), pitch.to_degrees(),
            flow_meas.dx_px, flow_meas.dy_px,
            range_meas.range_m,
            features.len(),
        );
    }

    let n = count as f64;
    eprintln!("\n=== Tracking RMSE ({} steps, dt={:.3}s) ===", count, dt);
    eprintln!("  x:    {:.4} m", (sum_sq[0]/n).sqrt());
    eprintln!("  y:    {:.4} m", (sum_sq[1]/n).sqrt());
    eprintln!("  z:    {:.4} m", (sum_sq[2]/n).sqrt());
    eprintln!("  3D:   {:.4} m", ((sum_sq[0]+sum_sq[1]+sum_sq[2])/(3.0*n)).sqrt());
    eprintln!("  max:  {:.4} m", max_err);
}

// ---------------------------------------------------------------------------
// Euler angles to quaternion [w, x, y, z] (ZYX convention, body-to-world).
// ---------------------------------------------------------------------------
fn euler_to_quat(roll: f32, pitch: f32, yaw: f32) -> [f32; 4] {
    let (cr, sr) = ((roll  * 0.5).cos(), (roll  * 0.5).sin());
    let (cp, sp) = ((pitch * 0.5).cos(), (pitch * 0.5).sin());
    let (cy, sy) = ((yaw   * 0.5).cos(), (yaw   * 0.5).sin());
    [
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ]
}

// ──────────────────────────────────────────────────────────────────────────────
// Motor mixer inversion
// ──────────────────────────────────────────────────────────────────────────────

/// Convert desired thrust [N] and body torque [N·m] to motor ω² commands.
///
/// Inverts the X-configuration mixer in `MultirotorParams::motor_speeds_to_forces_torques`:
///   F  = kf * (u1 + u2 + u3 + u4)
///   τx = kf·L/√2 * (-u1 + u2 + u3 - u4)
///   τy = kf·L/√2 * (-u1 - u2 + u3 + u4)
///   τz = kt       * ( u1 - u2 + u3 - u4)
///
/// Solution (by row reduction):
///   u1 = (F/kf - τx/lk - τy/lk + τz/kt) / 4
///   u2 = (F/kf + τx/lk - τy/lk - τz/kt) / 4
///   u3 = (F/kf + τx/lk + τy/lk + τz/kt) / 4
///   u4 = (F/kf - τx/lk + τy/lk - τz/kt) / 4
/// where lk = kf * arm_length / √2
fn thrust_torque_to_motors(params: &MultirotorParams, thrust: f32, torque: Vec3) -> MotorAction {
    let kf = params.kf;
    let kt = params.kt;
    let lk = kf * params.arm_length / 2.0_f32.sqrt();

    let f_over_kf  = thrust / kf;
    let tx_over_lk = torque.x / lk;
    let ty_over_lk = torque.y / lk;
    let tz_over_kt = torque.z / kt;

    let u1 = (f_over_kf - tx_over_lk - ty_over_lk + tz_over_kt) * 0.25;
    let u2 = (f_over_kf + tx_over_lk - ty_over_lk - tz_over_kt) * 0.25;
    let u3 = (f_over_kf + tx_over_lk + ty_over_lk + tz_over_kt) * 0.25;
    let u4 = (f_over_kf - tx_over_lk + ty_over_lk - tz_over_kt) * 0.25;

    // Clamp to physically valid range (ω² ≥ 0)
    MotorAction::new(
        u1.max(0.0),
        u2.max(0.0),
        u3.max(0.0),
        u4.max(0.0),
    )
}

// ──────────────────────────────────────────────────────────────────────────────
// Trajectory waypoint builders
// ──────────────────────────────────────────────────────────────────────────────

fn circle_waypoints(n: usize, radius: f32, height: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let wps: Vec<Waypoint> = (0..=n)
        .map(|i| {
            let theta = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
            Waypoint {
                pos: Vec3::new(radius * theta.cos(), radius * theta.sin(), height),
                yaw: 0.0,
            }
        })
        .collect();
    (wps, vec![1.0f32; n])
}

fn figure8_waypoints(n: usize, ax: f32, ay: f32, height: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let wps: Vec<Waypoint> = (0..=n)
        .map(|i| {
            let theta = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
            Waypoint {
                pos: Vec3::new(ax * theta.sin(), ay * (2.0 * theta).sin() * 0.5, height),
                yaw: 0.0,
            }
        })
        .collect();
    (wps, vec![1.0f32; n])
}

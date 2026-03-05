use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod, LogStream};
use crazyflie_link::LinkContext;
use std::time::{Duration, Instant};
use std::fs::{self, File};
use std::io::Write;
use std::collections::HashMap;
use tokio::time::sleep;
use chrono::Utc;

use multirotor_simulator::math::Vec3;
use multirotor_simulator::dynamics::MultirotorParams;
use multirotor_simulator::controller::{GeometricController, TrajectoryReference, Controller};
use multirotor_simulator::trajectory::{CircleTrajectory, SmoothFigure8Trajectory, Trajectory};
use multirotor_simulator::flight::{
    build_state,
    compute_force_vector,
    force_vector_to_rpyt,
    thrust_to_pwm,
    yaw_rate_cmd,
    detect_ekf_reset,
    deg_to_rad,
};

// CHANGE THIS TO SWITCH MANEUVER
// Valid options: "hover", "circle", "figure8", "my_hover", "my_circle", "my_figure8"
const MANEUVER: &str = "my_hover";

/// PWM value (0–65535) that produces exactly hover thrust at the current battery charge.
/// Procedure: run MANEUVER="my_hover" once, read "thr_pwm" in the terminal during steady
/// hover (after ep.z settles), take the average and set it here.
/// Typical CF2.1 range: 35000–50000. Last flight showed thr_pwm settling around 42000.
/// If the drone rises (ep.z goes negative), HOVER_PWM is too high — lower it.
/// If the drone falls  (ep.z goes positive), HOVER_PWM is too low  — raise it.
const HOVER_PWM: f32 = 42000.0;

#[derive(Debug, Default, Clone)]
struct LogEntry {
    time_ms: u64,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
    thrust: u32,
    vbat: f32,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
    rate_roll: f32,
    rate_pitch: f32,
    rate_yaw: f32,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let link_context = LinkContext::new();
    let uri = "radio://0/80/2M/E7E7E7E7E7";

    println!("Connecting to {} ...", uri);
    let cf = Crazyflie::connect_from_uri(&link_context, uri, NoTocCache).await?;
    println!("Connected!");

    // Initialize your controller and params
    let mut controller = GeometricController::default();
    let params = MultirotorParams::crazyflie();

    // Fixed hover reference for "my_controller" mode
    let _hover_ref = TrajectoryReference {
        position: Vec3::new(0.0, 0.0, 0.3),
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
        jerk: Vec3::zero(),
        yaw: 0.0,
        yaw_rate: 0.0,
        yaw_acceleration: 0.0,
    };

    // Block 1: Position + velocity — 6 floats, 24 bytes (at CF2 CRTP limit)
    let mut block1 = cf.log.create_block().await?;
    for v in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz"] {
        add_var(&mut block1, v).await;
    }
    let stream1 = block1.start(LogPeriod::from_millis(10)?).await?;

    // Block 2: Attitude + body rates — 6 floats, 24 bytes
    // Merges old blocks 2+4: everything needed for the geometric controller state.
    let mut block2 = cf.log.create_block().await?;
    for v in ["stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw",
              "gyro.x", "gyro.y", "gyro.z"] {
        add_var(&mut block2, v).await;
    }
    let stream2 = block2.start(LogPeriod::from_millis(10)?).await?;

    // Block 3: Thrust, battery voltage, accelerometer — 5 floats + 1 uint16, ~22 bytes.
    // stabilizer.thrust is the uint16 motor PWM command (0–65535).
    // pm.vbat is the battery voltage in volts — useful for post-flight analysis.
    // acc.x/y/z are the IMU accelerometer readings in g (body frame).
    let mut block3 = cf.log.create_block().await?;
    for v in ["stabilizer.thrust", "pm.vbat", "acc.x", "acc.y", "acc.z"] {
        add_var(&mut block3, v).await;
    }
    let stream3 = block3.start(LogPeriod::from_millis(10)?).await?;

    let mut log_data: Vec<LogEntry> = Vec::new();
    let mut last_print = Instant::now();

    // Open CSV immediately so every row is flushed to disk as it is collected.
    // This means the log is preserved even on crashes, panics, or radio drops.
    fs::create_dir_all("runs")?;
    let timestamp = Utc::now().format("%Y-%m-%d_%H-%M-%S");
    let filename = format!("runs/{}_{}.csv", MANEUVER, timestamp);
    let mut log_file = File::create(&filename)?;
    writeln!(log_file,
        "time_ms,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll,pitch,yaw,thrust,vbat,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,rate_roll,rate_pitch,rate_yaw"
    )?;
    println!("Log file opened: {}", filename);

    println!("Logging started (100 Hz). Starting maneuver '{}' in 3 seconds...", MANEUVER);
    sleep(Duration::from_secs(3)).await;

    // Reset Kalman filter BEFORE takeoff so position estimates start near zero.
    // Doing this mid-flight zeros z-estimate and causes the drone to re-land.
    match cf.param.set("kalman.resetEstimation", 1u8).await {
        Ok(_) => println!("Kalman estimator reset (pre-takeoff)"),
        Err(e) => println!("Failed to reset Kalman: {}", e),
    }
    sleep(Duration::from_millis(500)).await;

    // my_hover uses setpoint_hover to lift and stabilize (so EKF z is correct at ~0.3 m),
    // then switches to RPYT for closed-loop control.  my_circle does its own position-hold.
    let is_rpyt_maneuver = matches!(MANEUVER, "my_circle");
    if !is_rpyt_maneuver {
        println!("Ramping up...");
        for y in 0..15 {
            let zdistance = y as f32 / 50.0;
            cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
            sleep(Duration::from_millis(150)).await;
        }

        println!("Stabilizing hover for 3 seconds...");
        let stab_start = Instant::now();
        for _ in 0..30 {
            run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stab_start, &mut log_file).await;
            cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.3).await?;
            sleep(Duration::from_millis(100)).await;
        }
    }

    // Maneuver-specific logic
    match MANEUVER {
        "hover" => {
            println!("Pure hover at 0.3 m for 12 seconds...");
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(12) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start, &mut log_file).await;
                cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.3).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }

        "circle" => {
            let radius = 0.5;
            let height = 0.3;
            let omega = 0.6;

            println!(
                "Circle: radius {:.2} m (diameter ~{:.2} m), height {:.2} m, ω = {:.2} rad/s (~{:.1} s per lap)",
                radius, 2.0 * radius, height, omega, 2.0 * std::f32::consts::PI / omega
            );

            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(35) {
                let t = start.elapsed().as_secs_f32();
                let vx = -radius * omega * (omega * t).sin();
                let vy = radius * omega * (omega * t).cos();

                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start, &mut log_file).await;
                cf.commander.setpoint_hover(vx, vy, 0.0, height).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }

        "figure8" => {
            let a = 0.5;
            let b = 0.3;
            let omega = 0.5;

            println!(
                "Figure-8: x amplitude = {:.2} m (width ~{:.2} m), y amplitude = {:.2} m (height ~{:.2} m), ω = {:.2}",
                a, 2.0 * a, b, 2.0 * b, omega
            );

            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(45) {
                let t = start.elapsed().as_secs_f32();
                let vx = -a * omega * (omega * t).sin();
                let vy = b * omega * (2.0 * omega * t).cos();

                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start, &mut log_file).await;
                cf.commander.setpoint_hover(vx, vy, 0.0, 0.3).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }

        "my_hover" => {
            // Geometric controller closed-loop hover via setpoint_rpyt (Level 1).
            //
            // HOVER_PWM calibration — why thr_pwm converges to the real hover PWM:
            //   At equilibrium the drone is stationary, so real_thrust = m*g.
            //   real_thrust is linear in PWM → thr_pwm = real_hover_PWM, always,
            //   regardless of what HOVER_PWM is set to.  The drone will hover below
            //   0.3 m if HOVER_PWM is too low (ep_z will be positive), but thr_pwm
            //   in the terminal is still the correct value to copy into HOVER_PWM.
            //
            // Procedure:
            //   1. Run this flight. Drone hovers somewhere (possibly below 0.3 m).
            //   2. Wait ~10 s for ep_z to settle. Read the stable thr_pwm.
            //   3. Set HOVER_PWM = that value, recompile.  Drone now hovers at 0.3 m.
            //   4. During the same or next flight, nudge drone +x (forward) by hand.
            //      pitch_d must go NEGATIVE (nose down, pushes drone back to origin).
            //      If pitch_d goes POSITIVE instead, flip pitch_d and roll_d signs in
            //      my_circle before flying it.

            // ── Step 1: drain the log buffer ────────────────────────────────────
            // setpoint_hover ran for ~4 s above; drain accumulated packets so the
            // first RPYT iteration uses fresh EKF state (z should now be ~0.3 m).
            println!("my_hover: draining log buffer (getting fresh position)...");
            let drain_start = Instant::now();
            for _ in 0..60 {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &drain_start, &mut log_file).await;
            }

            // ── Step 2: anchor hover reference ───────────────────────────────────
            // Anchor XY to the current EKF estimate so there is no lateral step
            // error at RPYT hand-off.  Z is intentionally set to TARGET_HEIGHT
            // (not the current EKF z), because:
            //   • If Lighthouse hasn't initialised yet, EKF z ≈ 0 (floor) → anchoring
            //     there would make ep.z go negative as the drone rises, winding the
            //     Z integral negative and collapsing thrust.
            //   • setpoint_hover has been running for 3 s; the drone IS at ~0.3 m
            //     physically, so targeting 0.3 m is correct.
            //   • After the first Lighthouse fix the EKF XY/yaw reset fires and we
            //     re-anchor XY then; Z is left at TARGET_HEIGHT which remains valid.
            const TARGET_HEIGHT: f32 = 0.3;
            let anchor = log_data.last().cloned().unwrap_or_default();
            let mut hover_ref_rpyt = TrajectoryReference {
                position: Vec3::new(anchor.pos_x, anchor.pos_y, TARGET_HEIGHT),
                velocity: Vec3::zero(),
                acceleration: Vec3::zero(),
                jerk: Vec3::zero(),
                // Anchor yaw to the drone's current heading so there is no yaw-rate
                // kick at RPYT hand-off.  The yaw field is in radians everywhere in
                // the controller; latest_entry.yaw from the EKF is in degrees.
                yaw: deg_to_rad(anchor.yaw),
                yaw_rate: 0.0,
                yaw_acceleration: 0.0,
            };
            println!(
                "my_hover: RPYT hand-off. XY anchor ({:.3}, {:.3}), Z target {:.3} m. HOVER_PWM={:.0}",
                anchor.pos_x, anchor.pos_y, TARGET_HEIGHT, HOVER_PWM
            );

            // ── Step 3: one-shot RPYT thrust=0 unlock ───────────────────────────
            // CF RPYT commander requires thrust=0 before accepting non-zero thrust.
            // One command is enough; the drone drops ~1 cm in 50 ms — acceptable.
            cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
            sleep(Duration::from_millis(50)).await;

            let hover_thrust_n = params.mass * params.gravity;

            // Track previous EKF position and yaw to detect sudden jumps (Kalman resets).
            // Position step > 0.05 m/tick = 5 m/s apparent velocity — impossible.
            // Yaw step > 10°/tick = 1000 deg/s — impossible.  Both indicate EKF resets.
            let mut prev_ekf_pos: Option<Vec3> = None;
            let mut prev_ekf_yaw: f32 = 0.0;

            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(5) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start, &mut log_file).await;

                let latest_entry = log_data.last().cloned().unwrap_or_default();
                let state = build_state(
                    latest_entry.pos_x,  latest_entry.pos_y,  latest_entry.pos_z,
                    latest_entry.vel_x,  latest_entry.vel_y,  latest_entry.vel_z,
                    latest_entry.roll,   latest_entry.pitch,  latest_entry.yaw,
                    latest_entry.gyro_x, latest_entry.gyro_y, latest_entry.gyro_z,
                );

                // Safety: stop on flip
                if latest_entry.roll.abs() > 90.0 {
                    println!("FLIP detected (roll={:.1}°) — stopping motors", latest_entry.roll);
                    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
                    break;
                }

                // Safety: abort if drone drifts >0.4 m from anchor in XY
                let drift_xy = {
                    let dx = state.position.x - hover_ref_rpyt.position.x;
                    let dy = state.position.y - hover_ref_rpyt.position.y;
                    (dx * dx + dy * dy).sqrt()
                };
                if drift_xy > 0.4 {
                    println!("DRIFT ABORT (XY drift={:.2}m > 0.4m) — stopping motors", drift_xy);
                    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
                    break;
                }

                let mut ep = hover_ref_rpyt.position - state.position;
                let mut ev = hover_ref_rpyt.velocity - state.velocity;

                // Safety: re-anchor ONLY on genuine EKF Kalman resets, not on
                // normal XY drift. We detect a reset by comparing the EKF position
                // change between consecutive 10 ms ticks. A step > 0.05 m in one
                // tick = 5 m/s apparent velocity — physically impossible for this
                // drone — so it must be a Kalman reset. Normal 130 mm/s drift is
                // only 1.3 mm/tick, far below this threshold.
                //
                // Also detect yaw resets: a yaw jump > 10° in one 10 ms tick =
                // 1000 deg/s yaw rate — physically impossible — so it must be a
                // Lighthouse/EKF yaw re-initialisation.  Re-anchor hover_ref yaw
                // so the yaw rate command doesn't blast at ±30 deg/s afterward.
                let cur_pos = state.position;
                let cur_yaw = latest_entry.yaw; // degrees
                let reset = detect_ekf_reset(
                    prev_ekf_pos, cur_pos,
                    prev_ekf_yaw, cur_yaw,
                    0.05, 0.05, 10.0,
                );
                if reset.xy {
                    let step_xy = {
                        let s = cur_pos - prev_ekf_pos.unwrap_or(cur_pos);
                        (s.x * s.x + s.y * s.y).sqrt()
                    };
                    println!(
                        "EKF XY reset detected (XY={:.2}m) — re-anchoring XY+Z + resetting position integral",
                        step_xy
                    );
                    hover_ref_rpyt.position.x = state.position.x;
                    hover_ref_rpyt.position.y = state.position.y;
                    hover_ref_rpyt.position.z = state.position.z;
                    controller.reset_position_integral();
                }
                if reset.z {
                    let step_z = (cur_pos - prev_ekf_pos.unwrap_or(cur_pos)).z.abs();
                    println!(
                        "EKF Z reset detected (Z={:.2}m) — re-anchoring Z + resetting Z integral",
                        step_z
                    );
                    hover_ref_rpyt.position.z = state.position.z;
                    let mut i = controller.i_error_pos();
                    i.z = 0.0;
                    controller.set_i_error_pos(i);
                }
                if reset.yaw {
                    println!(
                        "EKF yaw reset detected (Δyaw={:.1}°) — re-anchoring yaw",
                        reset.step_yaw_deg
                    );
                    hover_ref_rpyt.yaw = deg_to_rad(cur_yaw);
                }
                if reset.any() {
                    ep = hover_ref_rpyt.position - state.position;
                    ev = hover_ref_rpyt.velocity - state.velocity;
                }
                prev_ekf_pos = Some(cur_pos);
                prev_ekf_yaw = cur_yaw;

                // Save integral before compute_control updates it (for anti-windup rollback)
                let i_pos_prev = controller.i_error_pos();

                let control = controller.compute_control(&state, &hover_ref_rpyt, &params, 0.01);
                let i_pos = controller.i_error_pos();
                let _i_att = controller.i_error_att();

                let f_vec = compute_force_vector(
                    &hover_ref_rpyt, ep, ev, i_pos, &controller, &params,
                );
                let (roll_d_cmd, pitch_d_cmd, roll_d_raw, pitch_d_raw) =
                    force_vector_to_rpyt(f_vec, 25.0);

                // Anti-windup: if roll or pitch saturated, revert position integral
                // to its pre-step value so it doesn't accumulate during saturation.
                if multirotor_simulator::flight::rpyt_control::tilt_saturated(roll_d_raw, pitch_d_raw, 25.0) {
                    controller.set_i_error_pos(i_pos_prev);
                }

                let yaw_rate_d = yaw_rate_cmd(hover_ref_rpyt.yaw, latest_entry.yaw, 1.0, 30.0);
                let (thrust_pwm, thrust_pwm_raw) = thrust_to_pwm(
                    control.thrust, hover_thrust_n, HOVER_PWM, 10_000.0, 60_000.0,
                );

                // Anti-windup: also revert Z integral if thrust is saturated.
                if thrust_pwm_raw > 60_000.0 || thrust_pwm_raw < 10_000.0 {
                    controller.set_i_error_pos(i_pos_prev);
                }

                println!(
                    "  ep=({:+.3},{:+.3},{:+.3})  i_pos=({:+.3},{:+.3},{:+.3})  r={:+.1}°  p_cmd={:+.1}°  yaw={:+.1}°→{:+.1}°/s  thr_pwm={}",
                    ep.x, ep.y, ep.z,
                    i_pos.x, i_pos.y, i_pos.z,
                    roll_d_cmd, pitch_d_cmd,
                    latest_entry.yaw, yaw_rate_d,
                    thrust_pwm,
                );

                cf.commander.setpoint_rpyt(roll_d_cmd, pitch_d_cmd, yaw_rate_d, thrust_pwm).await?;
            }
        }

        "my_circle" => {
            println!("Circle trajectory using GeometricController (position setpoint mode)...");

            let radius = 0.3;
            let height = 0.3;
            let omega  = 0.15; // ~42 s per lap, max speed 0.045 m/s — gentle for first flight

            // Hold at origin for 8 s to let the Kalman EKF fully converge.
            // Then read the converged position and offset the circle center so
            // t=0 starts exactly where the drone is (no step error at launch).
            println!("Stabilising at (0, 0, {:.2}) for 8 s — waiting for EKF convergence...", height);
            let stab_start = Instant::now();
            while stab_start.elapsed() < Duration::from_secs(8) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stab_start, &mut log_file).await;
                cf.commander.setpoint_position(0.0, 0.0, height, 0.0).await?;
                sleep(Duration::from_millis(50)).await;
            }

            // Read converged position as the circle start point.
            let base = log_data.last().cloned().unwrap_or_default();
            println!("EKF converged at ({:.3}, {:.3}) — using as circle start.", base.pos_x, base.pos_y);

            // Center at (base.pos_x - radius, base.pos_y) so t=0 is at (base.pos_x, base.pos_y).
            let circle = CircleTrajectory::with_center(radius, height, omega, (base.pos_x - radius, base.pos_y));

            // Level 1 geometric control: geometric controller drives the drone via setpoint_rpyt.
            // The firmware only handles attitude rate PIDs + motor mixing + battery compensation.
            let hover_thrust_n = params.mass * params.gravity;

            // EKF reset detection — same logic as my_hover.
            let mut prev_ekf_pos_c: Option<Vec3> = None;
            let mut prev_ekf_yaw_c: f32 = 0.0;

            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(60) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start, &mut log_file).await;

                let t = start.elapsed().as_secs_f32();
                let reference = circle.get_reference(t);

                let latest_entry = log_data.last().cloned().unwrap_or_default();
                let state = build_state(
                    latest_entry.pos_x,  latest_entry.pos_y,  latest_entry.pos_z,
                    latest_entry.vel_x,  latest_entry.vel_y,  latest_entry.vel_z,
                    latest_entry.roll,   latest_entry.pitch,  latest_entry.yaw,
                    latest_entry.gyro_x, latest_entry.gyro_y, latest_entry.gyro_z,
                );

                // ── Axis-aware EKF reset detector ──────────────────────────────
                // For circle/trajectory mode we can't re-anchor the XY reference to the
                // drone's current position (that would teleport it off the trajectory).
                // Instead we only reset the position integral on EKF jumps, so the
                // controller doesn't fight a stale integral from before the jump.
                // Z is handled identically to my_hover: re-anchor ref.z + reset Z integral.
                let cur_pos_c = state.position;
                let cur_yaw_c = latest_entry.yaw;
                let reset_c = detect_ekf_reset(
                    prev_ekf_pos_c, cur_pos_c,
                    prev_ekf_yaw_c, cur_yaw_c,
                    0.05, 0.05, 10.0,
                );
                if reset_c.xy {
                    let step_xy = {
                        let s = cur_pos_c - prev_ekf_pos_c.unwrap_or(cur_pos_c);
                        (s.x * s.x + s.y * s.y).sqrt()
                    };
                    println!(
                        "EKF XY reset (XY={:.2}m) — resetting XY integral (trajectory continues)",
                        step_xy
                    );
                    let i_z = controller.i_error_pos().z;
                    controller.reset_position_integral();
                    let mut i_restored = controller.i_error_pos();
                    i_restored.z = i_z;
                    controller.set_i_error_pos(i_restored);
                }
                if reset_c.z {
                    let step_z = (cur_pos_c - prev_ekf_pos_c.unwrap_or(cur_pos_c)).z.abs();
                    println!(
                        "EKF Z reset (Z={:.2}m) — resetting Z integral",
                        step_z
                    );
                    let mut i = controller.i_error_pos();
                    i.z = 0.0;
                    controller.set_i_error_pos(i);
                }
                if reset_c.yaw {
                    println!("EKF yaw reset (Δyaw={:.1}°) — noted", reset_c.step_yaw_deg);
                    // No re-anchor needed: reference.yaw is set by the trajectory,
                    // and the yaw_rate_d term already compensates for any offset.
                }
                prev_ekf_pos_c = Some(cur_pos_c);
                prev_ekf_yaw_c = cur_yaw_c;

                let dt = 0.05;
                let i_pos_prev = controller.i_error_pos();
                let control = controller.compute_control(&state, &reference, &params, dt);

                let ep = reference.position - state.position;
                let ev = reference.velocity - state.velocity;
                let i_pos = controller.i_error_pos();

                let f_vec = compute_force_vector(
                    &reference, ep, ev, i_pos, &controller, &params,
                );
                let (roll_d_cmd, pitch_d_cmd, roll_d_raw, pitch_d_raw) =
                    force_vector_to_rpyt(f_vec, 20.0);

                // Anti-windup: if roll or pitch saturated, revert position integral
                if multirotor_simulator::flight::rpyt_control::tilt_saturated(roll_d_raw, pitch_d_raw, 20.0) {
                    controller.set_i_error_pos(i_pos_prev);
                }
                let yaw_rate_d = yaw_rate_cmd(reference.yaw, latest_entry.yaw, 2.0, 30.0);
                let (thrust_pwm, _) = thrust_to_pwm(
                    control.thrust, hover_thrust_n, HOVER_PWM, 10_000.0, 60_000.0,
                );

                println!(
                    "t={:.1}s  ref=({:+.2},{:+.2})  pos=({:+.2},{:+.2})  ep=({:+.3},{:+.3})  r={:+.1}° p_cmd={:+.1}° thr={}",
                    t,
                    reference.position.x, reference.position.y,
                    state.position.x, state.position.y,
                    ep.x, ep.y,
                    roll_d_cmd, pitch_d_cmd, thrust_pwm,
                );

                cf.commander.setpoint_rpyt(roll_d_cmd, pitch_d_cmd, yaw_rate_d, thrust_pwm).await?;
                // No sleep — tokio::join! on both streams drives the loop at 100 Hz.
            }
        }

    "my_figure8" => {
        println!("Figure-8 trajectory using GeometricController (position setpoint mode)...");

        let figure8 = SmoothFigure8Trajectory::new();

        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(60) {
            run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start, &mut log_file).await;

            let t = start.elapsed().as_secs_f32();
            let reference = figure8.get_reference(t);

            let latest_entry = log_data.last().cloned().unwrap_or_default();
            let state = build_state(
                latest_entry.pos_x,  latest_entry.pos_y,  latest_entry.pos_z,
                latest_entry.vel_x,  latest_entry.vel_y,  latest_entry.vel_z,
                latest_entry.roll,   latest_entry.pitch,  latest_entry.yaw,
                latest_entry.gyro_x, latest_entry.gyro_y, latest_entry.gyro_z,
            );

            let dt = 0.05;
            let control = controller.compute_control(&state, &reference, &params, dt);

            let ep = reference.position - state.position;

            println!(
                "t={:.1}s  ref=({:+.2},{:+.2})  pos=({:+.2},{:+.2})  ep=({:+.3},{:+.3})  thrust_N={:.3}",
                t,
                reference.position.x, reference.position.y,
                state.position.x, state.position.y,
                ep.x, ep.y,
                control.thrust,
            );

            cf.commander.setpoint_position(
                reference.position.x,
                reference.position.y,
                reference.position.z,
                reference.yaw,
            ).await?;

            sleep(Duration::from_millis(50)).await;
        }
    }

        _ => {
            println!("Unknown maneuver '{}'. Falling back to hover.", MANEUVER);
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(12) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start, &mut log_file).await;
                cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.3).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }
    }

    // Gentle ramp down
    println!("Ramping down gently...");
    for y in (0..40).rev() {
        let zdistance = y as f32 / 100.0;
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(120)).await;
    }

    cf.commander.setpoint_stop().await?;
    cf.commander.notify_setpoint_stop(500).await?;

    println!("Maneuver complete. Motors stopped.");
    println!("Log saved to: {}", filename);

    drop(stream1);
    drop(stream2);
    drop(stream3);
    cf.disconnect().await;

    println!("Disconnected cleanly.");
    Ok(())
}

// ────────────────────────────────────────────────
// Helper functions (unchanged)
// ────────────────────────────────────────────────

async fn run_logging_step(
    log_data: &mut Vec<LogEntry>,
    last_print: &mut Instant,
    stream1: &LogStream,  // pos + vel
    stream2: &LogStream,  // attitude + gyro
    stream3: &LogStream,  // thrust + vbat + acc
    start: &Instant,
    log_file: &mut File,
) {
    let mut entry = LogEntry {
        time_ms: start.elapsed().as_millis() as u64,
        ..Default::default()
    };

    // Read all three blocks in parallel — total wait = max(b1, b2, b3) not sum.
    let (r1, r2, r3) = tokio::join!(stream1.next(), stream2.next(), stream3.next());

    if let Ok(p) = r1 {
        let d = &p.data;
        entry.pos_x = get_f32(d, "stateEstimate.x");
        entry.pos_y = get_f32(d, "stateEstimate.y");
        entry.pos_z = get_f32(d, "stateEstimate.z");
        entry.vel_x = get_f32(d, "stateEstimate.vx");
        entry.vel_y = get_f32(d, "stateEstimate.vy");
        entry.vel_z = get_f32(d, "stateEstimate.vz");
    }

    if let Ok(p) = r2 {
        let d = &p.data;
        entry.roll   = get_f32(d, "stabilizer.roll");
        entry.pitch  = get_f32(d, "stabilizer.pitch");
        entry.yaw    = get_f32(d, "stabilizer.yaw");
        entry.gyro_x = get_f32(d, "gyro.x");
        entry.gyro_y = get_f32(d, "gyro.y");
        entry.gyro_z = get_f32(d, "gyro.z");
    }

    if let Ok(p) = r3 {
        let d = &p.data;
        entry.thrust = d.get("stabilizer.thrust")
            .and_then(|v| u32::try_from(*v).ok())
            .unwrap_or(0);
        entry.vbat  = get_f32(d, "pm.vbat");
        entry.acc_x = get_f32(d, "acc.x");
        entry.acc_y = get_f32(d, "acc.y");
        entry.acc_z = get_f32(d, "acc.z");
    }

    if last_print.elapsed() >= Duration::from_secs(1) {
        println!(
            "[t={:3}s] z={:+5.3} vx={:+5.3} vy={:+5.3} roll={:+5.1} gyro_z={:+6.1} thr={} vbat={:.2}V",
            start.elapsed().as_secs(),
            entry.pos_z, entry.vel_x, entry.vel_y, entry.roll, entry.gyro_z,
            entry.thrust, entry.vbat,
        );
        *last_print = Instant::now();
    }

    // Write row immediately — log is safe even on crash or radio drop
    let _ = writeln!(log_file,
        "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}",
        entry.time_ms,
        entry.pos_x, entry.pos_y, entry.pos_z,
        entry.vel_x, entry.vel_y, entry.vel_z,
        entry.roll, entry.pitch, entry.yaw,
        entry.thrust,
        entry.vbat,
        entry.gyro_x, entry.gyro_y, entry.gyro_z,
        entry.acc_x, entry.acc_y, entry.acc_z,
        entry.rate_roll, entry.rate_pitch, entry.rate_yaw
    );

    log_data.push(entry);
}

async fn add_var(block: &mut LogBlock, name: &str) {
    match block.add_variable(name).await {
        Ok(_) => println!("Added: {}", name),
        Err(e) => eprintln!("Failed to add {}: {}", name, e),
    }
}

fn get_f32(map: &HashMap<String, Value>, key: &str) -> f32 {
    map.get(key).and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0)
}


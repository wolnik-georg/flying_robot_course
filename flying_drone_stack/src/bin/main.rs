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
    deg_to_rad,
};

// CHANGE THIS TO SWITCH MANEUVER
// Valid options: "hover", "circle", "figure8", "my_hover", "my_circle", "my_figure8"
const MANEUVER: &str = "my_circle";

/// PWM value (0–65535) that produces exactly hover thrust at the current battery charge.
/// Procedure: run MANEUVER="my_hover" once, read "thr_pwm" in the terminal during steady
/// hover (after ep.z settles), take the average and set it here.
/// Typical CF2.1 range: 35000–50000.
/// Flight 2026-03-04: setpoint_hover worked, RPYT descended immediately → HOVER_PWM was too low.
/// Back-calculation from descent rate at RPYT handoff gives real hover ≈ 50 000.
/// If the drone rises (ep.z goes negative), HOVER_PWM is too high — lower it.
/// If the drone falls  (ep.z goes positive), HOVER_PWM is too low  — raise it.
const HOVER_PWM: f32 = 50000.0;

#[derive(Debug, Default, Clone)]
struct LogEntry {
    time_ms: u64,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
    range_z: f32,   // range.zrange — ToF height above floor (m), reliable without Lighthouse
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
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

    // Block 1: Velocity + attitude — 6 floats, 24 bytes.
    // All control-critical lateral variables in one packet at 200 Hz.
    let mut block1 = cf.log.create_block().await?;
    for v in ["stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz",
              "stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw"] {
        add_var(&mut block1, v).await;
    }
    let stream1 = block1.start(LogPeriod::from_millis(10)?).await?;

    // Block 2: Z-ranger + gyro — uint16 + 3 floats, 14 bytes.
    // range.zrange: ToF height in mm (uint16) → divided by 1000 to get metres.
    // gyro.x/y/z: body-frame angular rates needed by the geometric attitude controller.
    let mut block2 = cf.log.create_block().await?;
    for v in ["range.zrange", "gyro.x", "gyro.y", "gyro.z"] {
        add_var(&mut block2, v).await;
    }
    let stream2 = block2.start(LogPeriod::from_millis(10)?).await?;

    let mut log_data: Vec<LogEntry> = Vec::new();
    let mut last_print = Instant::now();

    // Open CSV immediately so every row is flushed to disk as it is collected.
    // This means the log is preserved even on crashes, panics, or radio drops.
    fs::create_dir_all("runs")?;
    let timestamp = Utc::now().format("%Y-%m-%d_%H-%M-%S");
    let filename = format!("runs/{}_{}.csv", MANEUVER, timestamp);
    let mut log_file = File::create(&filename)?;
    writeln!(log_file, "time_ms,vel_x,vel_y,vel_z,roll,pitch,yaw,range_z,gyro_x,gyro_y,gyro_z")?;
    println!("Log file opened: {}", filename);

    println!("Logging started (100 Hz, 2 blocks). Starting maneuver '{}' in 3 seconds...", MANEUVER);
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
    let skip_hover_bootstrap = matches!(MANEUVER, "my_circle" | "my_hover");
    if !skip_hover_bootstrap {
        println!("Ramping up...");
        for y in 0..15 {
            let zdistance = y as f32 / 50.0;
            cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
            sleep(Duration::from_millis(150)).await;
        }

        println!("Stabilizing hover for 3 seconds...");
        let stab_start = Instant::now();
        for _ in 0..30 {
            run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2,&stab_start, &mut log_file).await;
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
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2,&start, &mut log_file).await;
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

                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2,&start, &mut log_file).await;
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

                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2,&start, &mut log_file).await;
                cf.commander.setpoint_hover(vx, vy, 0.0, 0.3).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }

        "my_hover" => {
            // Geometric controller hover using range.zrange for height, EKF x/y for lateral.
            // No Lighthouse dependency.  Takes off directly from the ground via RPYT.
            //
            // HOVER_PWM calibration: run once, read stable thr_pwm when ep.z ≈ 0, copy here.

            // ── Read initial sensors on ground ───────────────────────────────────
            // 30 iterations ≈ 300 ms to fill log buffer with valid sensor readings.
            // Drain ALL buffered log packets before starting RPYT.
            // Log streams run from connect time (~3.5 s before this point) → ~350 stale
            // packets queued per stream.  30 iterations was not enough; the RPYT loop then
            // drained the remaining ~320 stale packets at 2000 Hz, blasting max-thrust
            // commands before the drone was ready (flight 2026-03-10 bug).
            // 500 iterations covers ~5 s of buffered data at 100 Hz and guarantees the
            // control loop starts with real-time packets.
            println!("my_hover: draining stale log buffer...");
            let drain_start = Instant::now();
            for _ in 0..500 {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &drain_start, &mut log_file).await;
            }
            let anchor = log_data.last().cloned().unwrap_or_default();
            println!(
                "my_hover: range_z={:.3}m  yaw={:.1}°  → taking off to 0.3 m",
                anchor.range_z, anchor.yaw,
            );

            // ── RPYT unlock ──────────────────────────────────────────────────────
            // CF RPYT commander ignores non-zero thrust until it sees thrust=0 first.
            cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;

            let hover_thrust_n = params.mass * params.gravity;

            // Reference position: XY = EKF starting position (= 0 after Kalman reset,
            // relative to where the drone sits), Z = 0.3 m target height in range_z frame.
            // On the ground: range_z ≈ 0 → ep.z = +0.3 → max thrust → natural takeoff.
            // At target:     range_z ≈ 0.3 → ep.z ≈ 0 → hover thrust → stable hold.
            let hover_ref_rpyt = TrajectoryReference {
                position: Vec3::new(0.0, 0.0, 0.3),  // XY unused (velocity-only); Z = target height
                velocity: Vec3::zero(),
                acceleration: Vec3::zero(),
                jerk: Vec3::zero(),
                yaw: deg_to_rad(anchor.yaw),
                yaw_rate: 0.0,
                yaw_acceleration: 0.0,
            };

            // Self-integrated XY position estimate — gentle rubber-band to start position.
            let mut est_pos_x = 0.0f32;
            let mut est_pos_y = 0.0f32;
            // Once the drone has been airborne, keep lateral damping active even if
            // range_z momentarily dips below 0.1 m (e.g. during a crash descent).
            // The takeoff gate only applies during the initial liftoff from the ground.
            let mut ever_airborne = false;

            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(15) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2,&start, &mut log_file).await;

                let latest_entry = log_data.last().cloned().unwrap_or_default();

                // XY position passed as 0 — velocity-only lateral control, no position hold.
                // Z from range.zrange (reliable ToF, no Lighthouse needed).
                let state = build_state(
                    0.0, 0.0, latest_entry.range_z,
                    latest_entry.vel_x,  latest_entry.vel_y,  latest_entry.vel_z,
                    latest_entry.roll,   latest_entry.pitch,  latest_entry.yaw,
                    latest_entry.gyro_x, latest_entry.gyro_y, latest_entry.gyro_z,
                );

                let vel_xy = (latest_entry.vel_x.powi(2) + latest_entry.vel_y.powi(2)).sqrt();

                // Safety: stop on flip
                if latest_entry.roll.abs() > 90.0 || latest_entry.pitch.abs() > 90.0 {
                    println!(
                        "FLIP detected (roll={:.1}°, pitch={:.1}°) — stopping motors",
                        latest_entry.roll, latest_entry.pitch
                    );
                    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
                    break;
                }

                // Safety: abort on excessive lateral velocity.
                if vel_xy > 1.5 {
                    println!("VELOCITY ABORT (vel_xy={:.2} m/s) — stopping motors", vel_xy);
                    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
                    break;
                }

                // XY: gentle position correction + velocity damping.
                // Z: full position control via range_z (absolute ToF, no drift).
                if latest_entry.range_z > 0.1 {
                    ever_airborne = true;
                }

                // Only integrate position when velocity is small — prevents a crash
                // spiral from inflating est_pos to 0.7+ m and causing a massive
                // destabilizing correction when the gate later reopens.
                if ever_airborne && vel_xy < 0.2 {
                    est_pos_x += latest_entry.vel_x * 0.01;
                    est_pos_y += latest_entry.vel_y * 0.01;
                    // Clamp so any residual bias can't build past ±0.5 m.
                    est_pos_x = est_pos_x.clamp(-0.5, 0.5);
                    est_pos_y = est_pos_y.clamp(-0.5, 0.5);
                }

                let mut ep = hover_ref_rpyt.position - state.position;
                // Effective XY kp = 0.5 — gentle rubber-band to starting position.
                ep.x = if ever_airborne { -est_pos_x * (0.5 / 7.0) } else { 0.0 };
                ep.y = if ever_airborne { -est_pos_y * (0.5 / 7.0) } else { 0.0 };

                let mut ev = hover_ref_rpyt.velocity - state.velocity;
                // Takeoff gate: suppress lateral damping only during initial liftoff.
                // Once ever_airborne is set, damping stays active even if range_z dips
                // (e.g. during a crash descent) — prevents gate-toggle destabilisation.
                if !ever_airborne {
                    ev.x = 0.0;
                    ev.y = 0.0;
                }

                let i_pos_prev = controller.i_error_pos();
                let control = controller.compute_control(&state, &hover_ref_rpyt, &params, 0.01);
                // Zero XY integral: with ep.x/y=0 the controller's internal integral
                // still accumulates XY position error — reset it each tick so only the
                // velocity term (ev) drives lateral corrections.
                let mut i_pos = controller.i_error_pos();
                i_pos.x = 0.0;
                i_pos.y = 0.0;
                controller.set_i_error_pos(i_pos);
                let i_pos = controller.i_error_pos();

                let f_vec = compute_force_vector(&hover_ref_rpyt, ep, ev, i_pos, &controller, &params);
                let (roll_d_cmd, pitch_d_cmd, roll_d_raw, pitch_d_raw) =
                    force_vector_to_rpyt(f_vec, 25.0);

                if multirotor_simulator::flight::rpyt_control::tilt_saturated(roll_d_raw, pitch_d_raw, 25.0) {
                    controller.set_i_error_pos(i_pos_prev);
                }

                // Crazyflie RPYT sign: positive yaw_rate = CW from above = decreasing EKF yaw.
                // yaw_rate_cmd returns (ref − current): negate to match hardware convention.
                let yaw_rate_d = -yaw_rate_cmd(hover_ref_rpyt.yaw, latest_entry.yaw, 1.0, 30.0);
                let (thrust_pwm, thrust_pwm_raw) = thrust_to_pwm(
                    control.thrust, hover_thrust_n, HOVER_PWM, 10_000.0, 60_000.0,
                );

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
            // Circle trajectory using the same approach as my_hover:
            // - range.zrange for height (no Lighthouse needed)
            // - velocity integration for XY position estimate
            // - RPYT control throughout, takeoff from ground
            // - Phase 1: hover at (0,0,height) until stable
            // - Phase 2: track circle trajectory once at height

            let radius = 0.3f32;
            let height = 0.3f32;
            let omega  = 0.15f32; // ~42 s per lap, max speed r*ω = 0.045 m/s — gentle

            // ── Drain stale buffer (same as my_hover) ────────────────────────
            println!("my_circle: draining stale log buffer...");
            let drain_start = Instant::now();
            for _ in 0..500 {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &drain_start, &mut log_file).await;
            }
            let anchor = log_data.last().cloned().unwrap_or_default();
            println!(
                "my_circle: range_z={:.3}m  yaw={:.1}°  → taking off to {:.2}m then circling",
                anchor.range_z, anchor.yaw, height,
            );

            // ── RPYT unlock ──────────────────────────────────────────────────
            cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;

            let hover_thrust_n = params.mass * params.gravity;

            // Circle centered at (-radius, 0) so t=0 starts at (0, 0) with velocity (0, r*ω).
            let circle = CircleTrajectory::with_center(radius, height, omega, (-radius, 0.0));

            // Hover reference used during Phase 1 (takeoff + height stabilisation).
            let hover_ref_circle = TrajectoryReference {
                position: Vec3::new(0.0, 0.0, height),
                velocity: Vec3::zero(),
                acceleration: Vec3::zero(),
                jerk: Vec3::zero(),
                yaw: deg_to_rad(anchor.yaw),
                yaw_rate: 0.0,
                yaw_acceleration: 0.0,
            };

            // Integrated XY position — same dead-reckoning as my_hover.
            let mut est_pos_x = 0.0f32;
            let mut est_pos_y = 0.0f32;
            let mut ever_airborne = false;

            // circle_t advances once the drone is stably at height.
            let mut at_height = false;
            let mut circle_t = 0.0f32;

            let start = Instant::now();
            // 70 s total: up to ~10 s to reach height + 60 s circle
            while start.elapsed() < Duration::from_secs(70) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &start, &mut log_file).await;

                let latest_entry = log_data.last().cloned().unwrap_or_default();

                // Safety: stop on flip
                if latest_entry.roll.abs() > 90.0 || latest_entry.pitch.abs() > 90.0 {
                    println!(
                        "FLIP detected (roll={:.1}°, pitch={:.1}°) — stopping motors",
                        latest_entry.roll, latest_entry.pitch
                    );
                    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
                    break;
                }

                let vel_xy = (latest_entry.vel_x.powi(2) + latest_entry.vel_y.powi(2)).sqrt();
                if vel_xy > 2.0 {
                    println!("VELOCITY ABORT (vel_xy={:.2} m/s) — stopping motors", vel_xy);
                    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
                    break;
                }

                if latest_entry.range_z > 0.1 {
                    ever_airborne = true;
                }

                // Integrate XY position from velocity (dt = 10 ms tick).
                // Gate: freeze during fast drift (same logic as my_hover).
                if ever_airborne && vel_xy < 0.3 {
                    est_pos_x += latest_entry.vel_x * 0.01;
                    est_pos_y += latest_entry.vel_y * 0.01;
                }

                // Transition to circle phase: drone must be airborne and within 3 cm of target height.
                // Reset est_pos to (0, 0) at transition so circle starts with zero position error.
                if !at_height && ever_airborne && (latest_entry.range_z - height).abs() < 0.03 {
                    est_pos_x = 0.0;
                    est_pos_y = 0.0;
                    at_height = true;
                    println!(
                        "my_circle: at height {:.3}m — starting circle (ω={:.2} rad/s, r={:.2}m)",
                        latest_entry.range_z, omega, radius,
                    );
                }

                let reference = if at_height {
                    circle_t += 0.01;
                    circle.get_reference(circle_t)
                } else {
                    hover_ref_circle.clone()
                };

                let state = build_state(
                    est_pos_x, est_pos_y, latest_entry.range_z,
                    latest_entry.vel_x,  latest_entry.vel_y,  latest_entry.vel_z,
                    latest_entry.roll,   latest_entry.pitch,  latest_entry.yaw,
                    latest_entry.gyro_x, latest_entry.gyro_y, latest_entry.gyro_z,
                );

                let ep = reference.position - state.position;
                let ev = reference.velocity - state.velocity;

                let i_pos_prev = controller.i_error_pos();
                let control = controller.compute_control(&state, &reference, &params, 0.01);
                let i_pos = controller.i_error_pos();

                let f_vec = compute_force_vector(&reference, ep, ev, i_pos, &controller, &params);
                let (roll_d_cmd, pitch_d_cmd, roll_d_raw, pitch_d_raw) =
                    force_vector_to_rpyt(f_vec, 20.0);

                if multirotor_simulator::flight::rpyt_control::tilt_saturated(roll_d_raw, pitch_d_raw, 20.0) {
                    controller.set_i_error_pos(i_pos_prev);
                }

                let yaw_rate_d = -yaw_rate_cmd(reference.yaw, latest_entry.yaw, 1.0, 30.0);
                let (thrust_pwm, thrust_pwm_raw) = thrust_to_pwm(
                    control.thrust, hover_thrust_n, HOVER_PWM, 10_000.0, 60_000.0,
                );

                if thrust_pwm_raw > 60_000.0 || thrust_pwm_raw < 10_000.0 {
                    controller.set_i_error_pos(i_pos_prev);
                }

                println!(
                    "{}  ref=({:+.2},{:+.2},{:+.2})  est=({:+.2},{:+.2})  ep=({:+.3},{:+.3},{:+.3})  r={:+.1}° p={:+.1}° thr={}",
                    if at_height { format!("c={:.1}s", circle_t) } else { "hover ".to_string() },
                    reference.position.x, reference.position.y, reference.position.z,
                    est_pos_x, est_pos_y,
                    ep.x, ep.y, ep.z,
                    roll_d_cmd, pitch_d_cmd, thrust_pwm,
                );

                cf.commander.setpoint_rpyt(roll_d_cmd, pitch_d_cmd, yaw_rate_d, thrust_pwm).await?;
            }
        }

    "my_figure8" => {
        println!("Figure-8 trajectory using GeometricController (position setpoint mode)...");

        let figure8 = SmoothFigure8Trajectory::new();

        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(60) {
            run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2,&start, &mut log_file).await;

            let t = start.elapsed().as_secs_f32();
            let reference = figure8.get_reference(t);

            let latest_entry = log_data.last().cloned().unwrap_or_default();
            let state = build_state(
                0.0, 0.0, latest_entry.range_z,
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
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2,&start, &mut log_file).await;
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
    stream1: &LogStream,  // vel + attitude (200 Hz)
    stream2: &LogStream,  // range_z + gyro (200 Hz)
    start: &Instant,
    log_file: &mut File,
) {
    let mut entry = LogEntry {
        time_ms: start.elapsed().as_millis() as u64,
        ..Default::default()
    };

    // Read both blocks in parallel — total wait = max(b1, b2), not sum.
    let (r1, r2) = tokio::join!(stream1.next(), stream2.next());

    if let Ok(p) = r1 {
        let d = &p.data;
        entry.vel_x = get_f32(d, "stateEstimate.vx");
        entry.vel_y = get_f32(d, "stateEstimate.vy");
        entry.vel_z = get_f32(d, "stateEstimate.vz");
        entry.roll  = get_f32(d, "stabilizer.roll");
        entry.pitch = get_f32(d, "stabilizer.pitch");
        entry.yaw   = get_f32(d, "stabilizer.yaw");
    }

    if let Ok(p) = r2 {
        let d = &p.data;
        // range.zrange: firmware logs as uint16 in mm → divide by 1000 to get metres.
        entry.range_z = get_f32(d, "range.zrange") / 1000.0;
        entry.gyro_x  = get_f32(d, "gyro.x");
        entry.gyro_y  = get_f32(d, "gyro.y");
        entry.gyro_z  = get_f32(d, "gyro.z");
    }

    if last_print.elapsed() >= Duration::from_secs(1) {
        println!(
            "[t={:3}s] rng={:+5.3}  vx={:+5.3} vy={:+5.3}  roll={:+5.1} pitch={:+5.1}",
            start.elapsed().as_secs(),
            entry.range_z, entry.vel_x, entry.vel_y, entry.roll, entry.pitch,
        );
        *last_print = Instant::now();
    }

    // Write row immediately — log is safe even on crash or radio drop
    let _ = writeln!(log_file,
        "{},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3}",
        entry.time_ms,
        entry.vel_x, entry.vel_y, entry.vel_z,
        entry.roll, entry.pitch, entry.yaw,
        entry.range_z,
        entry.gyro_x, entry.gyro_y, entry.gyro_z,
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
    // to_f64_lossy() handles every Value variant (U8/U16/U32/F32/...).
    // The old f32::try_from() only matched Value::F32 and silently returned 0
    // for integer types such as range.zrange (U16) and stabilizer.thrust (U16).
    map.get(key).map(|v| v.to_f64_lossy() as f32).unwrap_or(0.0)
}


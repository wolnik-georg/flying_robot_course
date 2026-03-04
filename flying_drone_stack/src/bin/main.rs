use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod, LogStream};
use crazyflie_link::LinkContext;
use std::time::{Duration, Instant};
use std::fs::{self, File};
use std::io::Write;
use std::collections::HashMap;
use tokio::time::sleep;
use chrono::Utc;

use multirotor_simulator::math::{Vec3, Quat};
use multirotor_simulator::dynamics::{MultirotorState, MultirotorParams};
use multirotor_simulator::controller::{GeometricController, TrajectoryReference, Controller};
use multirotor_simulator::trajectory::{CircleTrajectory, SmoothFigure8Trajectory, Trajectory};

// CHANGE THIS TO SWITCH MANEUVER
// Valid options: "hover", "circle", "figure8", "my_hover", "my_circle", "my_figure8"
const MANEUVER: &str = "my_circle";

/// PWM value (0–65535) that produces exactly hover thrust at the current battery charge.
/// Procedure: run MANEUVER="my_hover" once, read "thr_pwm" in the terminal during steady
/// hover, take the average and set it here before flying my_circle.
/// Typical CF2.1 range: 35000–45000. Start low and increase if drone won't climb.
const HOVER_PWM: f32 = 38000.0;

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
    let hover_ref = TrajectoryReference {
        position: Vec3::new(0.0, 0.0, 0.3),
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
        jerk: Vec3::zero(),
        yaw: 0.0,
        yaw_rate: 0.0,
        yaw_acceleration: 0.0,
    };

    // Block 1: Position + velocity (6 floats = 24 bytes)
    let mut block1 = cf.log.create_block().await?;
    let vars1 = vec![
        "stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
        "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz",
    ];
    for v in vars1 { add_var(&mut block1, v).await; }
    let stream1 = block1.start(LogPeriod::from_millis(50)?).await?;

    // Block 2: Attitude + thrust (3 floats + 1 uint16 = 14 bytes)
    let mut block2 = cf.log.create_block().await?;
    let vars2 = vec![
        "stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw",
        "stabilizer.thrust",
    ];
    for v in vars2 { add_var(&mut block2, v).await; }
    let stream2 = block2.start(LogPeriod::from_millis(50)?).await?;

    // Block 3: Body rates + battery (4 floats = 16 bytes)
    let mut block3 = cf.log.create_block().await?;
    let vars3 = vec![
        "rateRoll", "ratePitch", "rateYaw",
        "pm.vbat",
    ];
    for v in vars3 { add_var(&mut block3, v).await; }
    let stream3 = block3.start(LogPeriod::from_millis(50)?).await?;

    // Block 4: Raw IMU sensors (6 floats = 24 bytes)
    let mut block4 = cf.log.create_block().await?;
    let vars4 = vec![
        "gyro.x", "gyro.y", "gyro.z",
        "acc.x", "acc.y", "acc.z",
    ];
    for v in vars4 { add_var(&mut block4, v).await; }
    let stream4 = block4.start(LogPeriod::from_millis(50)?).await?;

    let mut log_data: Vec<LogEntry> = Vec::new();
    let mut last_print = Instant::now();

    println!("Logging started (20 Hz). Starting maneuver '{}' in 3 seconds...", MANEUVER);
    sleep(Duration::from_secs(3)).await;

    // Reset Kalman filter BEFORE takeoff so position estimates start near zero.
    // Doing this mid-flight zeros z-estimate and causes the drone to re-land.
    match cf.param.set("kalman.resetEstimation", 1u8).await {
        Ok(_) => println!("Kalman estimator reset (pre-takeoff)"),
        Err(e) => println!("Failed to reset Kalman: {}", e),
    }
    sleep(Duration::from_millis(500)).await;

    println!("Ramping up...");
    for y in 0..15 {
        let zdistance = y as f32 / 50.0;
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(150)).await;
    }

    println!("Stabilizing hover for 3 seconds...");
    for _ in 0..30 {
        run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &Instant::now()).await;
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.3).await?;
        sleep(Duration::from_millis(100)).await;
    }

    // Maneuver-specific logic
    match MANEUVER {
        "hover" => {
            println!("Pure hover at 0.3 m for 12 seconds...");
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(12) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &start).await;
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

                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &start).await;
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

                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &start).await;
                cf.commander.setpoint_hover(vx, vy, 0.0, 0.3).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }

        "my_hover" => {
            // Level 1 geometric control via setpoint_rpyt:
            //   position + velocity error → desired force vector → roll/pitch angles + thrust PWM
            //   firmware handles only: attitude rate PIDs, motor mixing, battery compensation.
            //
            // USE THIS MODE TO CALIBRATE HOVER_PWM:
            //   watch "thr_pwm" in the output during steady hover, average it, set HOVER_PWM above.
            println!("Hovering with GeometricController → setpoint_rpyt (Level 1 closed loop)");
            println!("HOVER_PWM={:.0} — if drone climbs/sinks slowly, adjust by ±2000 and re-run.", HOVER_PWM);

            let hover_thrust_n = params.mass * params.gravity; // 0.265 N for CF2.1

            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(12) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &start).await;

                let latest_entry = log_data.last().cloned().unwrap_or_default();
                let state = MultirotorState {
                    position: Vec3::new(latest_entry.pos_x, latest_entry.pos_y, latest_entry.pos_z),
                    velocity: Vec3::new(latest_entry.vel_x, latest_entry.vel_y, latest_entry.vel_z),
                    orientation: {
                        let roll_rad  = latest_entry.roll  * std::f32::consts::PI / 180.0;
                        let pitch_rad = latest_entry.pitch * std::f32::consts::PI / 180.0;
                        let yaw_rad   = latest_entry.yaw   * std::f32::consts::PI / 180.0;
                        let q_yaw   = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw_rad);
                        let q_pitch = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), pitch_rad);
                        let q_roll  = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll_rad);
                        (q_yaw * q_pitch * q_roll).normalize()
                    },
                    angular_velocity: Vec3::new(
                        latest_entry.gyro_x * std::f32::consts::PI / 180.0,
                        latest_entry.gyro_y * std::f32::consts::PI / 180.0,
                        latest_entry.gyro_z * std::f32::consts::PI / 180.0,
                    ),
                };

                let dt = 0.05;
                let control = controller.compute_control(&state, &hover_ref, &params, dt);

                let ep = hover_ref.position - state.position;
                let ev = hover_ref.velocity - state.velocity;

                // Desired force vector in world frame: F = m * (a_ff + kp*ep + kv*ev + g*ez)
                // This is the same thrust_force computed inside compute_control — we reproduce
                // it here so we have the direction, not just the magnitude (control.thrust).
                let desired_accel = hover_ref.acceleration
                    + Vec3::new(controller.kp.x * ep.x, controller.kp.y * ep.y, controller.kp.z * ep.z)
                    + Vec3::new(controller.kv.x * ev.x, controller.kv.y * ev.y, controller.kv.z * ev.z)
                    + Vec3::new(0.0, 0.0, params.gravity);
                let f_vec = desired_accel * params.mass;

                // Extract roll/pitch from force direction.
                // pitch_d = atan2(fx, fz): positive fx (forward force) → positive pitch (nose up in CF)
                // roll_d  = atan2(-fy, fz): positive fy (left force)   → negative roll (bank left)
                let pitch_d    = f_vec.x.atan2(f_vec.z).to_degrees().clamp(-15.0, 15.0);
                let roll_d     = (-f_vec.y).atan2(f_vec.z).to_degrees().clamp(-15.0, 15.0);

                // Yaw: simple P controller — drives yaw error to zero at up to 30 °/s
                let yaw_rate_d = (hover_ref.yaw.to_degrees() - latest_entry.yaw).clamp(-30.0, 30.0) * 2.0;

                // Thrust: scale control.thrust (N) proportionally to HOVER_PWM
                let thrust_pwm = (control.thrust / hover_thrust_n * HOVER_PWM)
                    .clamp(10_000.0, 60_000.0) as u16;

                println!(
                    "  ep=({:+.3},{:+.3},{:+.3})  r={:+.1}°  p={:+.1}°  thr_pwm={}  ctrl_N={:.3}",
                    ep.x, ep.y, ep.z, roll_d, pitch_d, thrust_pwm, control.thrust
                );

                cf.commander.setpoint_rpyt(roll_d, pitch_d, yaw_rate_d, thrust_pwm).await?;
                sleep(Duration::from_millis(50)).await;
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
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stab_start).await;
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

            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(60) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &start).await;

                let t = start.elapsed().as_secs_f32();
                let reference = circle.get_reference(t);

                let latest_entry = log_data.last().cloned().unwrap_or_default();
                let state = MultirotorState {
                    position: Vec3::new(latest_entry.pos_x, latest_entry.pos_y, latest_entry.pos_z),
                    velocity: Vec3::new(latest_entry.vel_x, latest_entry.vel_y, latest_entry.vel_z),
                    orientation: {
                        let roll_rad  = latest_entry.roll  * std::f32::consts::PI / 180.0;
                        let pitch_rad = latest_entry.pitch * std::f32::consts::PI / 180.0;
                        let yaw_rad   = latest_entry.yaw   * std::f32::consts::PI / 180.0;
                        let q_yaw   = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw_rad);
                        let q_pitch = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), pitch_rad);
                        let q_roll  = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll_rad);
                        (q_yaw * q_pitch * q_roll).normalize()
                    },
                    angular_velocity: Vec3::new(
                        latest_entry.gyro_x * std::f32::consts::PI / 180.0,
                        latest_entry.gyro_y * std::f32::consts::PI / 180.0,
                        latest_entry.gyro_z * std::f32::consts::PI / 180.0,
                    ),
                };

                let dt = 0.05;
                let control = controller.compute_control(&state, &reference, &params, dt);

                let ep = reference.position - state.position;
                let ev = reference.velocity - state.velocity;

                // Desired force vector in world frame: F = m*(a_ff + kp*ep + kv*ev + g*ez)
                // Reproduces the internal thrust_force from compute_control so we have
                // direction (for roll/pitch) in addition to magnitude (control.thrust).
                let desired_accel = reference.acceleration
                    + Vec3::new(controller.kp.x * ep.x, controller.kp.y * ep.y, controller.kp.z * ep.z)
                    + Vec3::new(controller.kv.x * ev.x, controller.kv.y * ev.y, controller.kv.z * ev.z)
                    + Vec3::new(0.0, 0.0, params.gravity);
                let f_vec = desired_accel * params.mass;

                // Level 1: roll/pitch from force direction, thrust magnitude → PWM.
                let pitch_d    = f_vec.x.atan2(f_vec.z).to_degrees().clamp(-20.0, 20.0);
                let roll_d     = (-f_vec.y).atan2(f_vec.z).to_degrees().clamp(-20.0, 20.0);
                let yaw_rate_d = (reference.yaw.to_degrees() - latest_entry.yaw).clamp(-30.0, 30.0) * 2.0;
                let thrust_pwm = (control.thrust / hover_thrust_n * HOVER_PWM)
                    .clamp(10_000.0, 60_000.0) as u16;

                println!(
                    "t={:.1}s  ref=({:+.2},{:+.2})  pos=({:+.2},{:+.2})  ep=({:+.3},{:+.3})  r={:+.1}° p={:+.1}° thr={}",
                    t,
                    reference.position.x, reference.position.y,
                    state.position.x, state.position.y,
                    ep.x, ep.y,
                    roll_d, pitch_d, thrust_pwm,
                );

                cf.commander.setpoint_rpyt(roll_d, pitch_d, yaw_rate_d, thrust_pwm).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }

    "my_figure8" => {
        println!("Figure-8 trajectory using GeometricController (position setpoint mode)...");

        let figure8 = SmoothFigure8Trajectory::new();

        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(60) {
            run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &start).await;

            let t = start.elapsed().as_secs_f32();
            let reference = figure8.get_reference(t);

            let latest_entry = log_data.last().cloned().unwrap_or_default();
            let state = MultirotorState {
                position: Vec3::new(latest_entry.pos_x, latest_entry.pos_y, latest_entry.pos_z),
                velocity: Vec3::new(latest_entry.vel_x, latest_entry.vel_y, latest_entry.vel_z),
                orientation: {
                    let roll_rad  = latest_entry.roll  * std::f32::consts::PI / 180.0;
                    let pitch_rad = latest_entry.pitch * std::f32::consts::PI / 180.0;
                    let yaw_rad   = latest_entry.yaw   * std::f32::consts::PI / 180.0;
                    let q_yaw   = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw_rad);
                    let q_pitch = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), pitch_rad);
                    let q_roll  = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll_rad);
                    (q_yaw * q_pitch * q_roll).normalize()
                },
                angular_velocity: Vec3::new(
                    latest_entry.gyro_x * std::f32::consts::PI / 180.0,
                    latest_entry.gyro_y * std::f32::consts::PI / 180.0,
                    latest_entry.gyro_z * std::f32::consts::PI / 180.0,
                ),
            };

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
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &start).await;
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

    let timestamp = Utc::now().format("%Y-%m-%d_%H-%M-%S");
    let filename = format!("runs/{}_{}.csv", MANEUVER, timestamp);

    fs::create_dir_all("runs")?;
    let mut file = File::create(&filename)?;

    writeln!(file,
        "time_ms,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll,pitch,yaw,thrust,vbat,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,rate_roll,rate_pitch,rate_yaw"
    )?;

    for e in log_data {
        writeln!(file,
            "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}",
            e.time_ms,
            e.pos_x, e.pos_y, e.pos_z,
            e.vel_x, e.vel_y, e.vel_z,
            e.roll, e.pitch, e.yaw,
            e.thrust,
            e.vbat,
            e.gyro_x, e.gyro_y, e.gyro_z,
            e.acc_x, e.acc_y, e.acc_z,
            e.rate_roll, e.rate_pitch, e.rate_yaw
        )?;
    }

    println!("Log saved to: {}", filename);

    drop(stream1);
    drop(stream2);
    drop(stream3);
    drop(stream4);
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
    stream1: &LogStream,
    stream2: &LogStream,
    stream3: &LogStream,
    stream4: &LogStream,
    start: &Instant,
) {
    let mut entry = LogEntry {
        time_ms: start.elapsed().as_millis() as u64,
        ..Default::default()
    };

    if let Ok(p) = stream1.next().await {
        let d = &p.data;
        entry.pos_x = get_f32(d, "stateEstimate.x");
        entry.pos_y = get_f32(d, "stateEstimate.y");
        entry.pos_z = get_f32(d, "stateEstimate.z");
        entry.vel_x = get_f32(d, "stateEstimate.vx");
        entry.vel_y = get_f32(d, "stateEstimate.vy");
        entry.vel_z = get_f32(d, "stateEstimate.vz");
    }

    if let Ok(p) = stream2.next().await {
        let d = &p.data;
        entry.roll = get_f32(d, "stabilizer.roll");
        entry.pitch = get_f32(d, "stabilizer.pitch");
        entry.yaw = get_f32(d, "stabilizer.yaw");
        entry.thrust = get_f32(d, "stabilizer.thrust") as u32;
    }

    if let Ok(p) = stream3.next().await {
        let d = &p.data;
        entry.rate_roll = get_f32(d, "rateRoll");
        entry.rate_pitch = get_f32(d, "ratePitch");
        entry.rate_yaw = get_f32(d, "rateYaw");
        entry.vbat = get_f32(d, "pm.vbat");
    }

    if let Ok(p) = stream4.next().await {
        let d = &p.data;
        entry.gyro_x = get_f32(d, "gyro.x");
        entry.gyro_y = get_f32(d, "gyro.y");
        entry.gyro_z = get_f32(d, "gyro.z");
        entry.acc_x = get_f32(d, "acc.x");
        entry.acc_y = get_f32(d, "acc.y");
        entry.acc_z = get_f32(d, "acc.z");
    }

    if last_print.elapsed() >= Duration::from_secs(1) {
        println!(
            "[t={:3}s] z={:+5.3} vx={:+5.3} vy={:+5.3} thrust={:5} roll={:+5.1} vbat={:.2}",
            start.elapsed().as_secs(),
            entry.pos_z, entry.vel_x, entry.vel_y, entry.thrust, entry.roll, entry.vbat
        );
        *last_print = Instant::now();
    }

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


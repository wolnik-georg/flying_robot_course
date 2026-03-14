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
use multirotor_simulator::estimation::{Mekf, MekfParams};
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
const MANEUVER: &str = "figure8";

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
    ekf_x: f32,     // stateEstimate.x — onboard EKF XY [m], reference for mekf_eval
    ekf_y: f32,
    acc_x: f32,     // acc.x — body-frame accelerometer [g], MEKF prediction input
    acc_y: f32,
    acc_z: f32,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let link_context = LinkContext::new();
    let uri = "radio://0/80/2M/E7E7E7E7E7";

    println!("Connecting to {} ...", uri);
    let cf = Crazyflie::connect_from_uri(&link_context, uri, NoTocCache).await?;
    println!("Connected!");

    // Firmware-controlled modes use the original working code path (20 Hz, no Kalman reset).
    if matches!(MANEUVER, "hover" | "circle" | "figure8") {
        return run_firmware_mode(&cf).await;
    }

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

    // Block 2: Z-ranger + gyro + EKF XY position — uint16 + 5 floats, 22 bytes.
    // range.zrange:      ToF height in mm (uint16) → divided by 1000 to get metres.
    // gyro.x/y/z:        body-frame angular rates [deg/s].
    // stateEstimate.x/y: onboard EKF XY position [m] — used as reference in mekf_eval.
    let mut block2 = cf.log.create_block().await?;
    for v in ["range.zrange", "gyro.x", "gyro.y", "gyro.z",
              "stateEstimate.x", "stateEstimate.y"] {
        add_var(&mut block2, v).await;
    }
    let stream2 = block2.start(LogPeriod::from_millis(10)?).await?;

    // Block 3: Accelerometer — 3 floats, 12 bytes.
    // acc.x/y/z: body-frame accelerometer [g] — primary MEKF prediction input.
    let mut block3 = cf.log.create_block().await?;
    for v in ["acc.x", "acc.y", "acc.z"] {
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
    writeln!(log_file, "time_ms,vel_x,vel_y,vel_z,roll,pitch,yaw,range_z,gyro_x,gyro_y,gyro_z,ekf_x,ekf_y,acc_x,acc_y,acc_z")?;
    println!("Log file opened: {}", filename);

    println!("Logging started (100 Hz, 3 blocks). Starting maneuver '{}' in 3 seconds...", MANEUVER);
    sleep(Duration::from_secs(3)).await;

    // Reset Kalman filter BEFORE takeoff so position estimates start near zero.
    // Doing this mid-flight zeros z-estimate and causes the drone to re-land.
    match cf.param.set("kalman.resetEstimation", 1u8).await {
        Ok(_) => println!("Kalman estimator reset (pre-takeoff)"),
        Err(e) => println!("Failed to reset Kalman: {}", e),
    }
    // Brief pause for EKF to process the reset before the ramp-up starts.
    // The ramp-up + stabilise phases (≈5 s of setpoint_hover commands) give the EKF
    // plenty of time to converge.  A long wait here is counterproductive: the drone
    // sits on the floor while the flow sensor reads garbage (floor at 3mm height),
    // causing EKF position to drift by >1 m before takeoff even begins.
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
            let radius = 0.3;
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
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &drain_start, &mut log_file).await;
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
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start, &mut log_file).await;

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

            let radius = 0.2f32;
            let height = 0.3f32;
            let omega  = 0.50f32; // ~12.6 s per lap, max speed r*ω = 0.100 m/s, SNR≈2

            // ── Drain stale buffer (same as my_hover) ────────────────────────
            println!("my_circle: draining stale log buffer...");
            let drain_start = Instant::now();
            for _ in 0..500 {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &drain_start, &mut log_file).await;
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

            // XY position via dead-reckoning: integrate optical-flow velocity at 100 Hz.
            // stateEstimate.x/y (EKF position) is UNRELIABLE without Lighthouse — the EKF
            // integrates accelerometer bias uncorrected, causing 2–6× position overestimation
            // (confirmed flight 2026-03-13_19-36-27: EKF y-range=0.826m vs expected 0.400m).
            // Vel gate at 1.5 m/s: freeze integration only during extreme maneuvers, not during
            // normal circle tracking (old 0.3 m/s gate caused runaway crash in flight 18-39-40).
            let mut est_pos_x = 0.0f32;
            let mut est_pos_y = 0.0f32;
            let mut ever_airborne = false;

            // Firmware-equivalent velocity PID state (body frame).
            // Filter: 20 Hz LPF on measured velocity (firmware: PID_VEL_XY_FILT_CUTOFF=20 Hz).
            //   alpha = dt/(dt + 1/(2π·fc)) = 0.01/(0.01 + 0.00796) = 0.557
            // Integral: ki=1 deg/(m/s·s) (firmware: PID_VEL_X_KI = PID_VEL_Y_KI = 1.0).
            // Both reset to zero when circle starts so hover-phase state doesn't carry over.
            let mut vx_b_filt = 0.0f32;
            let mut vy_b_filt = 0.0f32;
            let mut i_vel_x   = 0.0f32;
            let mut i_vel_y   = 0.0f32;

            // Phase logic:
            //   at_height      — set once drone reaches target height zone
            //   settle_ticks   — counts up after at_height; circle only starts after 5 s
            //   circle_t       — advances only after settle is complete
            // The settle period lets residual takeoff velocity damp to near-zero and the
            // Z integral converge, so the circle starts from a stable, near-stationary drone.
            let mut at_height = false;
            let mut settle_ticks: u32 = 0;
            let mut circle_t = 0.0f32;
            const SETTLE_TICKS: u32 = 500; // 5 s at 100 Hz before circle begins

            let start = Instant::now();
            // 70 s total: up to ~10 s to reach height + 60 s circle
            while start.elapsed() < Duration::from_secs(70) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start, &mut log_file).await;

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

                // Dead-reckoning XY: integrate velocity only when vel_xy is plausible.
                // Gate at 1.5 m/s prevents runaway from a crash spiral inflating est_pos,
                // while still allowing integration during normal circle tracking (peak ≈ 0.1 m/s).
                if ever_airborne && vel_xy < 1.5 {
                    est_pos_x += latest_entry.vel_x * 0.01;
                    est_pos_y += latest_entry.vel_y * 0.01;
                }
                let pos_x = est_pos_x;
                let pos_y = est_pos_y;

                // Phase 1 → Phase 2 transition: drone airborne and above 0.20 m.
                // Using a simple floor (not a narrow band) so the settle phase starts
                // immediately after liftoff rather than waiting for the drone to hit a
                // tight ±3 cm window around the 0.30 m target (which took 34 s in flight 3).
                // During the 5 s settle phase the controller still targets height = 0.30 m,
                // so the drone climbs to its final altitude naturally while velocity damps out.
                if !at_height && ever_airborne && latest_entry.range_z > 0.20 {
                    at_height = true;
                    println!(
                        "my_circle: at height {:.3}m — settling for {:.0}s before circle",
                        latest_entry.range_z, SETTLE_TICKS as f32 * 0.01,
                    );
                }

                // Count settle ticks; reset dead-reckoning position to (0,0) when settle
                // completes so circle_t=0 starts from a clean origin.
                if at_height && settle_ticks < SETTLE_TICKS {
                    settle_ticks += 1;
                    if settle_ticks == SETTLE_TICKS {
                        est_pos_x = 0.0;
                        est_pos_y = 0.0;
                        vx_b_filt = 0.0;  // reset filter — don't carry settle-phase velocity
                        vy_b_filt = 0.0;
                        i_vel_x   = 0.0;  // reset integral — clean start for circle
                        i_vel_y   = 0.0;
                        println!(
                            "my_circle: settled — starting circle (ω={:.2} rad/s, r={:.2}m, vel_xy={:.3}m/s)",
                            omega, radius, vel_xy,
                        );
                    }
                }

                let circle_started = at_height && settle_ticks >= SETTLE_TICKS;
                let reference = if circle_started {
                    circle_t += 0.01;
                    circle.get_reference(circle_t)
                } else {
                    hover_ref_circle.clone()
                };

                // For thrust: set state XY to reference XY so ep.xy=0 inside compute_control.
                // This ensures thrust is driven only by Z error.  If state_x=0 but reference
                // is at circle_x≠0, compute_control would see a large ep.xy and add spurious
                // XY force to the thrust magnitude — causing over-thrust as the drone follows
                // the circle.  With ep.xy=0 the thrust calculation is Z-only.
                // During hover the reference is fixed at (0,0) so est_pos is correct.
                let (state_x, state_y) = if circle_started {
                    (reference.position.x, reference.position.y)  // ep.xy = 0 → Z-only thrust
                } else {
                    (pos_x, pos_y)
                };
                let state = build_state(
                    state_x, state_y, latest_entry.range_z,
                    latest_entry.vel_x,  latest_entry.vel_y,  latest_entry.vel_z,
                    latest_entry.roll,   latest_entry.pitch,  latest_entry.yaw,
                    latest_entry.gyro_x, latest_entry.gyro_y, latest_entry.gyro_z,
                );

                let ep = reference.position - state.position;
                let ev = reference.velocity - state.velocity;

                let i_pos_prev = controller.i_error_pos();
                let control = controller.compute_control(&state, &reference, &params, 0.01);

                // Zero XY position integral every tick.
                let mut i_pos = controller.i_error_pos();
                i_pos.x = 0.0;
                i_pos.y = 0.0;
                controller.set_i_error_pos(i_pos);
                let i_pos = controller.i_error_pos();

                // ── XY roll/pitch: replicate firmware's velocityController exactly ─────────
                //
                // Firmware path for setpoint_hover(vx, vy, yaw_rate, height):
                //   position_controller_pid.c : velocityController()
                //     state_body_vx  = vx_world*cos(yaw) + vy_world*sin(yaw)  // world→body
                //     state_body_vy  = -vx_world*sin(yaw) + vy_world*cos(yaw)
                //     attitude->pitch = -(PID_VEL_X_KP=25) * (desired_vx_body - state_body_vx)
                //     attitude->roll  = -(PID_VEL_Y_KP=25) * (desired_vy_body - state_body_vy)
                //     clamped to ±rLimit/pLimit = ±20°
                //
                // RPYT convention (confirmed challenges.md):
                //   firmware negates pitch in RPYT mode: actual_pitch = -pitch_d_cmd
                //   firmware does NOT negate roll:       actual_roll  = roll_d_cmd
                //
                // To produce actual_pitch = -(25 * ev_bx):
                //   -pitch_d_cmd = -25 * ev_bx  →  pitch_d_cmd = +25 * ev_bx
                // To produce actual_roll  = -(25 * ev_by):
                //   roll_d_cmd = -25 * ev_by
                //
                // HOVER phase: geometric controller (stationary ref → slow drift → acceptable).
                let (roll_d_cmd, pitch_d_cmd) = if circle_started {
                    // Firmware velocity PI with 20 Hz input filter.
                    // Matches position_controller_pid.c : velocityController()
                    // Gains from platform_defaults_cf2.h:
                    //   PID_VEL_X_KP = 25.0, PID_VEL_X_KI = 1.0
                    //   PID_VEL_Y_KP = 25.0, PID_VEL_Y_KI = 1.0
                    //   PID_VEL_XY_FILT_CUTOFF = 20 Hz
                    const KP_VEL: f32 = 25.0;
                    const KI_VEL: f32 = 1.0;
                    // alpha = dt / (dt + 1/(2π·fc)) = 0.01 / (0.01 + 0.00796) = 0.557
                    const VEL_FILT_ALPHA: f32 = 0.557;

                    // World→body frame rotation (same as firmware's state_body_vx/vy).
                    let yaw_rad = deg_to_rad(latest_entry.yaw);
                    let cos_y   = yaw_rad.cos();
                    let sin_y   = yaw_rad.sin();

                    let vx_ref_b = reference.velocity.x * cos_y + reference.velocity.y * sin_y;
                    let vy_ref_b = -reference.velocity.x * sin_y + reference.velocity.y * cos_y;
                    let vx_b_raw = latest_entry.vel_x * cos_y + latest_entry.vel_y * sin_y;
                    let vy_b_raw = -latest_entry.vel_x * sin_y + latest_entry.vel_y * cos_y;

                    // 20 Hz low-pass filter on measured velocity (firmware: velFiltEnable=true).
                    vx_b_filt = (1.0 - VEL_FILT_ALPHA) * vx_b_filt + VEL_FILT_ALPHA * vx_b_raw;
                    vy_b_filt = (1.0 - VEL_FILT_ALPHA) * vy_b_filt + VEL_FILT_ALPHA * vy_b_raw;

                    let ev_bx = vx_ref_b - vx_b_filt;
                    let ev_by = vy_ref_b - vy_b_filt;

                    // Integral update.
                    i_vel_x += ev_bx * 0.01;
                    i_vel_y += ev_by * 0.01;

                    // PI output (before clamp).
                    let pitch_raw = KP_VEL * ev_bx + KI_VEL * i_vel_x;
                    let roll_raw  = -(KP_VEL * ev_by + KI_VEL * i_vel_y);

                    // Clamp to firmware's rLimit/pLimit = 20°.
                    let pitch = pitch_raw.clamp(-20.0, 20.0);
                    let roll  = roll_raw.clamp(-20.0, 20.0);

                    // Anti-windup: revert integral step if output was saturated.
                    if pitch_raw.abs() > 20.0 { i_vel_x -= ev_bx * 0.01; }
                    if roll_raw.abs()  > 20.0 { i_vel_y -= ev_by * 0.01; }

                    (roll, pitch)
                } else {
                    // Hover: geometric controller with attenuated position gain
                    let ep_f = Vec3::new(ep.x * (2.0 / 7.0), ep.y * (2.0 / 7.0), ep.z);
                    let ev_f = Vec3::new(ev.x * 0.2, ev.y * 0.2, ev.z);
                    let f_vec = compute_force_vector(&reference, ep_f, ev_f, i_pos, &controller, &params);
                    let (roll_d_cmd, pitch_d_cmd, roll_d_raw, pitch_d_raw) =
                        force_vector_to_rpyt(f_vec, 20.0);
                    if multirotor_simulator::flight::rpyt_control::tilt_saturated(roll_d_raw, pitch_d_raw, 20.0) {
                        controller.set_i_error_pos(i_pos_prev);
                    }
                    (roll_d_cmd, pitch_d_cmd)
                };

                // Hold anchor yaw throughout — do NOT use reference.yaw from the circle trajectory.
                // CircleTrajectory sets yaw = theta + π/2 (tangential heading).
                // Switching from hover yaw (0°) to circle yaw (90° at t=0) commands max yaw rate,
                // spinning the drone and misaligning roll/pitch with the world frame → spiral drift.
                let yaw_rate_d = -yaw_rate_cmd(deg_to_rad(anchor.yaw), latest_entry.yaw, 1.0, 30.0);
                let (thrust_pwm, thrust_pwm_raw) = thrust_to_pwm(
                    control.thrust, hover_thrust_n, HOVER_PWM, 10_000.0, 60_000.0,
                );

                if thrust_pwm_raw > 60_000.0 || thrust_pwm_raw < 10_000.0 {
                    controller.set_i_error_pos(i_pos_prev);
                }

                // During circle: show velocity reference vs actual (primary feedback).
                // During hover: show position estimate and error (primary feedback).
                if circle_started {
                    println!(
                        "c={:.1}s  ref_v=({:+.3},{:+.3})  meas_v=({:+.3},{:+.3})  ev=({:+.3},{:+.3})  ep_z={:+.3}  r={:+.1}° p={:+.1}° thr={}",
                        circle_t,
                        reference.velocity.x, reference.velocity.y,
                        latest_entry.vel_x, latest_entry.vel_y,
                        ev.x, ev.y, ep.z,
                        roll_d_cmd, pitch_d_cmd, thrust_pwm,
                    );
                } else {
                    println!(
                        "{}  ref=({:+.2},{:+.2})  est=({:+.2},{:+.2})  ep=({:+.3},{:+.3},{:+.3})  r={:+.1}° p={:+.1}° thr={}",
                        if at_height { format!("settle {}/{}", settle_ticks, SETTLE_TICKS) }
                        else { "takeoff".to_string() },
                        reference.position.x, reference.position.y,
                        pos_x, pos_y,
                        ep.x, ep.y, ep.z,
                        roll_d_cmd, pitch_d_cmd, thrust_pwm,
                    );
                }

                cf.commander.setpoint_rpyt(roll_d_cmd, pitch_d_cmd, yaw_rate_d, thrust_pwm).await?;
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
    stream1: &LogStream,  // vel + attitude (100 Hz)
    stream2: &LogStream,  // range_z + gyro + ekf_x/y (100 Hz)
    stream3: &LogStream,  // acc.xyz (100 Hz)
    start: &Instant,
    log_file: &mut File,
) {
    let mut entry = LogEntry {
        time_ms: start.elapsed().as_millis() as u64,
        ..Default::default()
    };

    // Read all three blocks in parallel — total wait = max(b1, b2, b3).
    let (r1, r2, r3) = tokio::join!(stream1.next(), stream2.next(), stream3.next());

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
        entry.ekf_x   = get_f32(d, "stateEstimate.x");
        entry.ekf_y   = get_f32(d, "stateEstimate.y");
    }

    if let Ok(p) = r3 {
        let d = &p.data;
        entry.acc_x = get_f32(d, "acc.x");
        entry.acc_y = get_f32(d, "acc.y");
        entry.acc_z = get_f32(d, "acc.z");
    }

    if last_print.elapsed() >= Duration::from_secs(1) {
        println!(
            "[t={:3}s] rng={:+5.3}  vx={:+5.3} vy={:+5.3}  roll={:+5.1} pitch={:+5.1}  acc_z={:+5.3}g",
            start.elapsed().as_secs(),
            entry.range_z, entry.vel_x, entry.vel_y, entry.roll, entry.pitch, entry.acc_z,
        );
        *last_print = Instant::now();
    }

    // Write row immediately — log is safe even on crash or radio drop
    let _ = writeln!(log_file,
        "{},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4}",
        entry.time_ms,
        entry.vel_x, entry.vel_y, entry.vel_z,
        entry.roll, entry.pitch, entry.yaw,
        entry.range_z,
        entry.gyro_x, entry.gyro_y, entry.gyro_z,
        entry.ekf_x, entry.ekf_y,
        entry.acc_x, entry.acc_y, entry.acc_z,
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

fn get_u32(map: &HashMap<String, Value>, key: &str) -> u32 {
    map.get(key).map(|v| v.to_f64_lossy() as u32).unwrap_or(0)
}

// ────────────────────────────────────────────────────────────────────────────
// Original working firmware-mode path (hover / circle / figure8)
//
// This is a faithful copy of the code that had hover, circle and figure8
// flying reliably.  Key differences from the my_* path:
//   • 20 Hz logging (50 ms period) — 60 CRTP packets/s vs 300 at 100 Hz
//   • No Kalman reset — EKF is left in whatever state the firmware settled it
//   • Sequential stream reads — matches the original timing behaviour
//   • CSV written at end (all data buffered in memory)
// ────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Default)]
struct FwLogEntry {
    time_ms: u64,
    pos_x: f32, pos_y: f32, pos_z: f32,
    vel_x: f32, vel_y: f32, vel_z: f32,
    roll: f32, pitch: f32, yaw: f32,
    thrust: u32,
    vbat: f32,
    gyro_x: f32, gyro_y: f32, gyro_z: f32,
    acc_x: f32, acc_y: f32, acc_z: f32,
    rate_roll: f32, rate_pitch: f32, rate_yaw: f32,
    range_z: f32,   // range.zrange — ToF height above floor [m] (uint16 mm → /1000)
    flow_dx: f32,   // motion.deltaX — raw flow pixel count (int16)
    flow_dy: f32,   // motion.deltaY — raw flow pixel count (int16)
    // Our MEKF running in parallel (passenger — no effect on flight)
    mekf_roll: f32, mekf_pitch: f32, mekf_yaw: f32,    // [deg]
    mekf_x: f32,    mekf_y: f32,    mekf_z: f32,        // [m]
}

async fn fw_logging_step(
    log_data: &mut Vec<FwLogEntry>,
    last_print: &mut Instant,
    stream1: &LogStream,
    stream2: &LogStream,
    stream3: &LogStream,
    stream4: &LogStream,
    start: &Instant,
    mekf: &mut Mekf,
    mekf_seeded: &mut bool,
) {
    let mut entry = FwLogEntry {
        time_ms: start.elapsed().as_millis() as u64,
        ..Default::default()
    };

    if let Ok(p) = stream1.next().await {
        let d = &p.data;
        entry.pos_x  = get_f32(d, "stateEstimate.x");
        entry.pos_y  = get_f32(d, "stateEstimate.y");
        entry.pos_z  = get_f32(d, "stateEstimate.z");
        entry.vel_x  = get_f32(d, "stateEstimate.vx");
        entry.vel_y  = get_f32(d, "stateEstimate.vy");
        entry.vel_z  = get_f32(d, "stateEstimate.vz");
        entry.thrust = get_u32(d, "stabilizer.thrust");
    }

    if let Ok(p) = stream2.next().await {
        let d = &p.data;
        entry.roll       = get_f32(d, "stabilizer.roll");
        entry.pitch      = get_f32(d, "stabilizer.pitch");
        entry.yaw        = get_f32(d, "stabilizer.yaw");
        entry.rate_roll  = get_f32(d, "rateRoll");
        entry.rate_pitch = get_f32(d, "ratePitch");
        entry.rate_yaw   = get_f32(d, "rateYaw");
    }

    if let Ok(p) = stream3.next().await {
        let d = &p.data;
        entry.vbat  = get_f32(d, "pm.vbat");
        entry.gyro_x = get_f32(d, "gyro.x");
        entry.gyro_y = get_f32(d, "gyro.y");
        entry.gyro_z = get_f32(d, "gyro.z");
        entry.acc_x  = get_f32(d, "acc.x");
        entry.acc_y  = get_f32(d, "acc.y");
        entry.acc_z  = get_f32(d, "acc.z");
    }

    if let Ok(p) = stream4.next().await {
        let d = &p.data;
        // range.zrange is uint16 in mm — convert to metres
        entry.range_z = get_f32(d, "range.zrange") / 1000.0;
        // motion.deltaX/Y are int16 raw pixel counts (accumulated since last poll)
        entry.flow_dx = get_f32(d, "motion.deltaX");
        entry.flow_dy = get_f32(d, "motion.deltaY");
    }

    // ── MEKF (passenger — purely for logging, no effect on flight control) ──
    //
    // Seed once at first genuine liftoff (range_z > 0.1 m).  Before that the
    // motors are spinning up with large gyro transients that would corrupt yaw,
    // so we don't start the filter until the drone is actually airborne.
    if !*mekf_seeded && entry.range_z > 0.1 {
        let q0 = euler_deg_to_quat(entry.roll, entry.pitch, entry.yaw);
        mekf.seed_qref(q0);
        mekf.state.x[0] = entry.pos_x;
        mekf.state.x[1] = entry.pos_y;
        mekf.state.x[2] = entry.range_z;
        *mekf_seeded = true;
        println!("[MEKF] seeded at t={:.2}s: x={:.3} y={:.3} z={:.3} yaw={:.1}°",
            entry.time_ms as f32 / 1000.0, entry.pos_x, entry.pos_y, entry.range_z, entry.yaw);
    }

    if *mekf_seeded {
        let t_s = entry.time_ms as f32 / 1000.0;
        let range_mm = if entry.range_z > 0.01 { Some(entry.range_z * 1000.0) } else { None };
        // PMW3901 axis convention (confirmed by correlation analysis on flight logs):
        //   vel_x ↔ -flow_dy  (r ≈ +0.74)
        //   vel_y ↔ -flow_dx  (r ≈ +0.30)
        let (dnx, dny) = (-entry.flow_dy, -entry.flow_dx);
        let flow = if dnx != 0.0 || dny != 0.0 { (Some(dnx), Some(dny)) } else { (None, None) };

        if let Some([roll_r, pitch_r, yaw_r, px, py, pz]) = mekf.feed_row(
            t_s,
            Some([entry.gyro_x, entry.gyro_y, entry.gyro_z]),
            Some([entry.acc_x,  entry.acc_y,  entry.acc_z]),
            range_mm,
            flow.0, flow.1,
        ) {
            entry.mekf_roll  = roll_r.to_degrees();
            entry.mekf_pitch = pitch_r.to_degrees();
            entry.mekf_yaw   = yaw_r.to_degrees();
            entry.mekf_x     = px;
            entry.mekf_y     = py;
            entry.mekf_z     = pz;
        }
    }

    if last_print.elapsed() >= Duration::from_secs(1) {
        println!(
            "[t={:3}s] z={:+5.3} vx={:+5.3} vy={:+5.3} thrust={:5} roll={:+5.1} vbat={:.2} | mekf x={:+5.3} y={:+5.3} z={:+5.3}",
            start.elapsed().as_secs(),
            entry.pos_z, entry.vel_x, entry.vel_y, entry.thrust, entry.roll, entry.vbat,
            entry.mekf_x, entry.mekf_y, entry.mekf_z,
        );
        *last_print = Instant::now();
    }

    log_data.push(entry);
}

async fn run_firmware_mode(cf: &Crazyflie) -> Result<(), Box<dyn std::error::Error>> {
    // Block 1: Position, velocity, thrust — 7 variables, 28 bytes @ 20 Hz
    let mut block1 = cf.log.create_block().await?;
    for v in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz",
              "stabilizer.thrust"] {
        add_var(&mut block1, v).await;
    }
    let stream1 = block1.start(LogPeriod::from_millis(50)?).await?;

    // Block 2: Attitude + body rates — 6 floats, 24 bytes @ 20 Hz
    let mut block2 = cf.log.create_block().await?;
    for v in ["stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw",
              "rateRoll", "ratePitch", "rateYaw"] {
        add_var(&mut block2, v).await;
    }
    let stream2 = block2.start(LogPeriod::from_millis(50)?).await?;

    // Block 3: Battery + raw sensors — 7 variables, 28 bytes @ 20 Hz
    let mut block3 = cf.log.create_block().await?;
    for v in ["pm.vbat", "gyro.x", "gyro.y", "gyro.z",
              "acc.x", "acc.y", "acc.z"] {
        add_var(&mut block3, v).await;
    }
    let stream3 = block3.start(LogPeriod::from_millis(50)?).await?;

    // Block 4: ToF range + optical flow — 3 variables (uint16/int16, 6 bytes total) @ 20 Hz
    // Kept separate so block 3 stays within the 26-byte payload limit.
    // range.zrange : uint16 [mm]   → converted to [m] on read
    // motion.deltaX: int16  [px]   → raw pixel accumulation since last read
    // motion.deltaY: int16  [px]   → raw pixel accumulation since last read
    let mut block4 = cf.log.create_block().await?;
    for v in ["range.zrange", "motion.deltaX", "motion.deltaY"] {
        add_var(&mut block4, v).await;
    }
    let stream4 = block4.start(LogPeriod::from_millis(50)?).await?;

    let mut log_data: Vec<FwLogEntry> = Vec::new();
    let mut last_print = Instant::now();
    let mut mekf = Mekf::new(MekfParams::default());
    let mut mekf_seeded = false;

    println!("Logging started (20 Hz). Starting maneuver '{}' in 3 seconds...", MANEUVER);
    sleep(Duration::from_secs(3)).await;

    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    sleep(Duration::from_millis(200)).await;

    println!("Ramping up...");
    for y in 0..15 {
        let zdistance = y as f32 / 50.0;
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(150)).await;
    }

    match MANEUVER {
        "hover" => {
            println!("Hovering at 0.3 m for 12 seconds...");
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(12) {
                fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &start, &mut mekf, &mut mekf_seeded).await;
                cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.3).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }
        "circle" => {
            let radius = 0.25f32;
            let height = 0.3f32;
            let omega = 0.6f32;
            println!("Circle: radius {:.2} m, height {:.2} m, ω = {:.2} rad/s", radius, height, omega);
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(30) {
                let t = start.elapsed().as_secs_f32();
                let vx = -radius * omega * (omega * t).sin();
                let vy =  radius * omega * (omega * t).cos();
                fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &start, &mut mekf, &mut mekf_seeded).await;
                cf.commander.setpoint_hover(vx, vy, 0.0, height).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }
        "figure8" => {
            let a = 0.25f32;
            let b = 0.15f32;
            let omega = 0.5f32;
            println!("Figure-8: a={:.2}, b={:.2}, ω={:.2}", a, b, omega);
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(40) {
                let t = start.elapsed().as_secs_f32();
                let vx = -a * omega * (omega * t).sin();
                let vy =  b * omega * (2.0 * omega * t).cos();
                fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &start, &mut mekf, &mut mekf_seeded).await;
                cf.commander.setpoint_hover(vx, vy, 0.0, 0.3).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }
        _ => {}
    }

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
    writeln!(file, "time_ms,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll,pitch,yaw,thrust,vbat,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,rate_roll,rate_pitch,rate_yaw,range_z,flow_dx,flow_dy,mekf_roll,mekf_pitch,mekf_yaw,mekf_x,mekf_y,mekf_z")?;
    for e in &log_data {
        writeln!(file,
            "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.0},{:.0},{:.2},{:.2},{:.2},{:.4},{:.4},{:.4}",
            e.time_ms,
            e.pos_x, e.pos_y, e.pos_z,
            e.vel_x, e.vel_y, e.vel_z,
            e.roll, e.pitch, e.yaw,
            e.thrust, e.vbat,
            e.gyro_x, e.gyro_y, e.gyro_z,
            e.acc_x, e.acc_y, e.acc_z,
            e.rate_roll, e.rate_pitch, e.rate_yaw,
            e.range_z,
            e.flow_dx, e.flow_dy,
            e.mekf_roll, e.mekf_pitch, e.mekf_yaw,
            e.mekf_x, e.mekf_y, e.mekf_z,
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

/// Convert Euler angles (degrees, ZYX / aerospace convention) to a unit quaternion [w, x, y, z].
/// Matches the convention used in mekf_eval.rs.
fn euler_deg_to_quat(roll_deg: f32, pitch_deg: f32, yaw_deg: f32) -> [f32; 4] {
    let r = roll_deg.to_radians()  * 0.5;
    let p = pitch_deg.to_radians() * 0.5;
    let y = yaw_deg.to_radians()   * 0.5;
    let (sr, cr) = r.sin_cos();
    let (sp, cp) = p.sin_cos();
    let (sy, cy) = y.sin_cos();
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod, LogStream};
use crazyflie_link::LinkContext;
use std::time::{Duration, Instant};
use std::fs::{self, File};
use std::io::Write;
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
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
use multirotor_simulator::perception::sensors::crtp::CrtpMultiRangeAdapter;
use multirotor_simulator::perception::processing::features::detect_features;
use multirotor_simulator::mapping::{OccupancyMap, KeyframeStore, KeyframeResult, VoTrajectory,
                                    LoopConstraint, PoseGraph};
use multirotor_simulator::estimation::mekf::mekf_update_vo;
use multirotor_simulator::mapping::vo_trajectory::VO_MAX_SIGMA;
use multirotor_simulator::planning::exploration::{ExplorationPlanner, ExplorationCommand};

// ---------------------------------------------------------------------------
// Phase 5 perception step macro
// ---------------------------------------------------------------------------
// Called once per control cycle after fw_logging_step to:
//   1. Update the shared OccupancyMap from the latest multi-ranger readings.
//   2. Drain any pending KeyframeResult from the AI Deck background task,
//      integrate it into the VoTrajectory, and fuse with the MEKF if the
//      chi-squared innovation gate passes.
//   3. Update `pending_vo` so the next fw_logging_step row carries current VO data.
macro_rules! step_perception {
    ($log_data:expr, $omap:expr, $mekf:expr, $mekf_seeded:expr,
     $ai_kf_result:expr, $vo_traj:expr, $vo_seeded:expr, $pending_vo:expr,
     $ai_loop_result:expr, $pose_graph:expr, $pending_pg:expr) => {{
        // ── Update occupancy map ─────────────────────────────────────────────
        if let Some((pos, roll, pitch, yaw, f, b, l, r, u, d)) = $log_data.last().map(|e| {
            let to_opt = |v: f32| if v > 0.02 { Some(v) } else { None };
            (Vec3::new(e.pos_x, e.pos_y, e.pos_z),
             e.roll, e.pitch, e.yaw,
             to_opt(e.multi_front), to_opt(e.multi_back), to_opt(e.multi_left),
             to_opt(e.multi_right), to_opt(e.multi_up), to_opt(e.range_z))
        }) {
            $omap.update(pos, roll, pitch, yaw, f, b, l, r, u, d);
        }
        // ── Visual odometry: consume latest keyframe result ──────────────────
        if let Ok(mut guard) = $ai_kf_result.lock() {
            if let Some(kf) = guard.take() {
                // Seed VO on first valid keyframe after MEKF converges.
                if !$vo_seeded && $mekf_seeded {
                    $vo_traj.seed(Vec3::new(
                        $mekf.state.x[0], $mekf.state.x[1], $mekf.state.x[2]));
                    $vo_seeded = true;
                }
                if $vo_seeded {
                    // Register this keyframe in the pose graph at the post-fusion MEKF position.
                    $pose_graph.add_node(kf.kf_index,
                        [$mekf.state.x[0], $mekf.state.x[1]]);

                    if let Some(vo_pos) = $vo_traj.integrate(&kf) {
                        // Gate: only fuse if sigma is below 90 % of max and the
                        // XY innovation is within 3-sigma of the MEKF position covariance.
                        if $vo_traj.sigma_xy < VO_MAX_SIGMA * 0.9 {
                            let r_vo = $vo_traj.sigma_xy.powi(2).max(R_VO_MIN);
                            let inno_x = vo_pos.x - $mekf.state.x[0];
                            let inno_y = vo_pos.y - $mekf.state.x[1];
                            let gate = (r_vo + $mekf.state.sigma.data[0][0]).max(0.01) * 9.0;
                            if inno_x * inno_x + inno_y * inno_y < gate {
                                mekf_update_vo(&mut $mekf.state, [vo_pos.x, vo_pos.y], r_vo);
                                $vo_traj.reset_to(Vec3::new(
                                    $mekf.state.x[0], $mekf.state.x[1], $mekf.state.x[2]));
                            }
                        }
                    }
                }
            }
        }
        // ── Loop closure: apply pose-graph correction ────────────────────────
        if let Ok(mut guard) = $ai_loop_result.lock() {
            if let Some(lc) = guard.take() {
                $pose_graph.add_loop(&lc);
                if let Some(pg_pos) = $pose_graph.optimize() {
                    mekf_update_vo(&mut $mekf.state, pg_pos, R_LOOP);
                    $vo_traj.reset_to(Vec3::new(
                        $mekf.state.x[0], $mekf.state.x[1], $mekf.state.x[2]));
                    eprintln!("[PG] correction pos=({:.3},{:.3}) lc#={}",
                        pg_pos[0], pg_pos[1], $pose_graph.lc_count);
                }
            }
        }
        // ── Update pending VO for next CSV row ───────────────────────────────
        $pending_vo = ($vo_traj.position.x, $vo_traj.position.y, $vo_traj.sigma_xy);
        // ── Update pending pose-graph for next CSV row ───────────────────────
        $pending_pg = (
            $pose_graph.latest_pos().map(|p| p[0]).unwrap_or(0.0),
            $pose_graph.latest_pos().map(|p| p[1]).unwrap_or(0.0),
            $pose_graph.lc_count as u32,
        );
    }}
}

// Default maneuver — overridden by --maneuver <name> on the command line.
// Valid options: "hover", "circle", "figure8", "my_hover", "my_circle", "my_figure8"
const DEFAULT_MANEUVER: &str = "figure8";

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
    let args: Vec<String> = std::env::args().collect();
    let maneuver: String = args.iter()
        .position(|a| a == "--maneuver")
        .and_then(|i| args.get(i + 1))
        .cloned()
        .unwrap_or_else(|| DEFAULT_MANEUVER.to_string());

    let link_context = LinkContext::new();
    let uri = "radio://0/80/2M/E7E7E7E7E7";

    println!("Connecting to {} ...", uri);
    let cf = Crazyflie::connect_from_uri(&link_context, uri, NoTocCache).await?;
    println!("Connected! Maneuver: {}", maneuver);

    // "hover", "circle", "figure8" use the firmware position PID path.
    // run_firmware_mode handles everything (Kalman reset, ramp, maneuver, landing).
    if matches!(maneuver.as_str(), "hover" | "circle" | "figure8" | "explore") {
        return run_firmware_mode(&cf, &maneuver).await;
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
    let filename = format!("runs/{}_{}.csv", maneuver, timestamp);
    let mut log_file = File::create(&filename)?;
    if maneuver == "my_circle" {
        writeln!(log_file, "time_ms,vel_x,vel_y,vel_z,roll,pitch,yaw,range_z,gyro_x,gyro_y,gyro_z,ekf_x,ekf_y,acc_x,acc_y,acc_z,ref_x,ref_y,ref_z,est_x,est_y")?;
    } else {
        writeln!(log_file, "time_ms,vel_x,vel_y,vel_z,roll,pitch,yaw,range_z,gyro_x,gyro_y,gyro_z,ekf_x,ekf_y,acc_x,acc_y,acc_z")?;
    }
    println!("Log file opened: {}", filename);

    println!("Logging started (100 Hz, 3 blocks). Starting maneuver '{}' in 3 seconds...", maneuver);
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
    let skip_hover_bootstrap = matches!(maneuver.as_str(), "my_circle" | "my_hover");
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
            let entry = run_logging_step(&mut last_print, &stream1, &stream2, &stream3, &stab_start).await;
            log_data.push(entry);
            cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.3).await?;
            sleep(Duration::from_millis(100)).await;
        }
    }

    // Maneuver-specific logic
    match maneuver.as_str() {
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
                let entry = run_logging_step(&mut last_print, &stream1, &stream2, &stream3, &drain_start).await;
                log_data.push(entry);
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
                let entry = run_logging_step(&mut last_print, &stream1, &stream2, &stream3, &start).await;
                log_data.push(entry.clone());
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
            let height = 0.22f32;
            let omega  = 0.50f32; // ~12.6 s per lap, max speed r*ω = 0.100 m/s, SNR≈2

            // ── Drain stale buffer (same as my_hover) ────────────────────────
            println!("my_circle: draining stale log buffer...");
            let drain_start = Instant::now();
            for _ in 0..500 {
                let entry = run_logging_step(&mut last_print, &stream1, &stream2, &stream3, &drain_start).await;
                log_data.push(entry);
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
                let entry = run_logging_step(&mut last_print, &stream1, &stream2, &stream3, &start).await;
                log_data.push(entry.clone());

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
                if !at_height && ever_airborne && latest_entry.range_z > 0.15 {
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

                let _ = writeln!(log_file,
                    "{},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
                    entry.time_ms,
                    entry.vel_x, entry.vel_y, entry.vel_z,
                    entry.roll, entry.pitch, entry.yaw,
                    entry.range_z,
                    entry.gyro_x, entry.gyro_y, entry.gyro_z,
                    entry.ekf_x, entry.ekf_y,
                    entry.acc_x, entry.acc_y, entry.acc_z,
                    reference.position.x, reference.position.y, reference.position.z,
                    pos_x, pos_y,
                );
                cf.commander.setpoint_rpyt(roll_d_cmd, pitch_d_cmd, yaw_rate_d, thrust_pwm).await?;
            }
        }

    "my_figure8" => {
        println!("Figure-8 trajectory using GeometricController (position setpoint mode)...");

        let figure8 = SmoothFigure8Trajectory::new();

        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(60) {
            let entry = run_logging_step(&mut last_print, &stream1, &stream2, &stream3, &start).await;
            log_data.push(entry.clone());
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
            println!("Unknown maneuver '{}'. Falling back to hover.", maneuver);
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(12) {
                let entry = run_logging_step(&mut last_print, &stream1, &stream2, &stream3, &start).await;
                log_data.push(entry.clone());
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
    last_print: &mut Instant,
    stream1: &LogStream,  // vel + attitude (100 Hz)
    stream2: &LogStream,  // range_z + gyro + ekf_x/y (100 Hz)
    stream3: &LogStream,  // acc.xyz (100 Hz)
    start: &Instant,
) -> LogEntry {
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

    entry
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
// Firmware-mode path  (hover / circle / figure8)
//
// Uses the Crazyflie's onboard position PID for all lateral control.
// Key properties:
//   • 20 Hz logging (50 ms period) — lower CRTP bandwidth than my_* path
//   • Kalman reset + convergence wait before takeoff and after stabilisation
//   • Absolute position setpoints — firmware PID corrects drift back to track
//   • CSV written row-by-row — log is preserved even on crash / radio drop
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
    // Shadow geometric controller (passenger — computed but NEVER sent to drone)
    our_ref_x: f32, our_ref_y: f32, our_ref_z: f32,    // spline reference position [m]
    our_thrust: f32,                                     // desired thrust [N]
    our_roll_cmd: f32, our_pitch_cmd: f32,               // desired roll/pitch [deg]
    our_yaw_rate_cmd: f32,                               // desired yaw rate [deg/s]
    // Multi-ranger Deck (perception module — Block 5)
    multi_front: f32, multi_back: f32, multi_left: f32,
    multi_right: f32, multi_up: f32,                     // [m], 0 = not available
    // AI Deck feature count (perception module — CPX camera, optional)
    ai_feat_count: u32,
    // Visual odometry (from AI Deck keyframes, one-cycle lag vs sensor data)
    vo_x: f32, vo_y: f32,   // VO trajectory position [m]; 0 before first keyframe
    vo_sigma: f32,           // VO accumulated uncertainty [m]
    // Pose-graph SLAM (Phase 6 — loop closure corrected position)
    pg_x: f32, pg_y: f32,   // Latest pose-graph node position [m]; 0 before first keyframe
    lc_count: u32,           // Total loop closures detected so far
}

/// Describes the trajectory our shadow controller is tracking.
/// Maneuver-specific parameters needed to evaluate the reference at each timestep.
/// None of this information is ever sent to the drone — it is purely for logging.
#[derive(Clone)]
enum ShadowManeuver {
    Hover  { cx: f32, cy: f32, height: f32 },
    Circle { cx: f32, cy: f32, height: f32, radius: f32, omega: f32 },
    Figure8 { cx: f32, cy: f32, height: f32, a: f32, b: f32, omega: f32 },
}

/// State carried into every `fw_logging_step` call for shadow controller evaluation.
struct ShadowCtx {
    /// Which trajectory to evaluate.
    maneuver: ShadowManeuver,
    /// Instant the trajectory phase began (used to compute elapsed t).
    /// Set to `None` during ramp-up / stabilise / before the maneuver starts.
    traj_start: Option<Instant>,
    /// Shadow geometric controller (maintains integral state across steps).
    controller: GeometricController,
    /// Drone parameters (mass, gravity, inertia).
    params: MultirotorParams,
}

impl ShadowCtx {
    /// Compute the shadow controller outputs for the given entry.
    /// Returns `(ref_x, ref_y, ref_z, thrust_N, roll_deg, pitch_deg, yaw_rate_deg_s)`
    /// or `None` if there is not enough state to run (MEKF not yet seeded, no traj_start).
    fn compute(&mut self, entry: &FwLogEntry, mekf_seeded: bool) -> Option<(f32,f32,f32,f32,f32,f32,f32)> {
        if !mekf_seeded { return None; }

        // Build MultirotorState from FIRMWARE EKF outputs (pos_x/y/z, roll/pitch/yaw).
        // This is the same state the firmware's position controller acts on, so our
        // shadow errors and the firmware's errors are computed in the same frame.
        // Using MEKF position here caused a coordinate-frame mismatch: setpoint_position
        // commands use the firmware EKF frame, so the shadow reference and the drone's
        // actual position were in different frames → large position error → saturated cmds.
        let state = build_state(
            entry.pos_x,  entry.pos_y,  entry.pos_z,
            entry.vel_x,  entry.vel_y,  entry.vel_z,
            entry.roll,   entry.pitch,  entry.yaw,
            entry.gyro_x, entry.gyro_y, entry.gyro_z,
        );

        // Evaluate reference trajectory at current elapsed time.
        // Before the maneuver phase begins (traj_start == None) we use a hover reference
        // at the current MEKF position so the shadow integrators don't wind up.
        let dt = 0.05_f32; // 20 Hz log rate
        let reference: TrajectoryReference = match &self.maneuver {
            ShadowManeuver::Hover { cx, cy, height } => {
                let (cx, cy, height) = (*cx, *cy, *height);
                TrajectoryReference {
                    position: Vec3::new(cx, cy, height),
                    velocity: Vec3::new(0.0, 0.0, 0.0),
                    acceleration: Vec3::new(0.0, 0.0, 0.0),
                    jerk: Vec3::new(0.0, 0.0, 0.0),
                    yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
                }
            }
            ShadowManeuver::Circle { cx, cy, height, radius, omega } => {
                let (cx, cy, height, radius, omega) = (*cx, *cy, *height, *radius, *omega);
                let t = self.traj_start.map(|s| s.elapsed().as_secs_f32()).unwrap_or(0.0);
                // Use CircleTrajectory so the reference math is shared with the live path.
                // Yaw is overridden to 0 below — shadow holds fixed heading like the firmware.
                let mut r = CircleTrajectory::with_center(radius, height, omega, (cx, cy))
                    .get_reference(t);
                r.yaw = 0.0; r.yaw_rate = 0.0; r.yaw_acceleration = 0.0;
                r
            }
            ShadowManeuver::Figure8 { cx, cy, height, a, b, omega } => {
                let (cx, cy, height, a, b, omega) = (*cx, *cy, *height, *a, *b, *omega);
                let t = self.traj_start.map(|s| s.elapsed().as_secs_f32()).unwrap_or(0.0);
                // Derive the loop duration from omega so SmoothFigure8Trajectory wraps correctly.
                let duration = 2.0 * std::f32::consts::PI / omega;
                let f8 = SmoothFigure8Trajectory { duration, height, a, b, time_scale: 1.0 };
                let mut r = f8.get_reference(t);
                // SmoothFigure8Trajectory centres at origin — apply the centre offset.
                r.position.x += cx;
                r.position.y += cy;
                // Yaw fixed at 0 throughout (firmware holds anchor heading).
                r.yaw = 0.0; r.yaw_rate = 0.0; r.yaw_acceleration = 0.0;
                r
            }
        };

        let ref_pos = reference.position;

        // Position & velocity errors
        let ep = reference.position - state.position;
        let ev = reference.velocity - state.velocity;
        let i_pos = self.controller.i_error_pos();

        // Compute desired force vector and roll/pitch commands
        let f_vec = compute_force_vector(&reference, ep, ev, i_pos, &self.controller, &self.params);
        let (roll_cmd_deg, pitch_cmd_deg, roll_raw, pitch_raw) = force_vector_to_rpyt(f_vec, 25.0);
        let thrust_n = f_vec.z; // world-frame z component ≈ thrust [N]

        // Update position integral (clamped inside controller, mimic firmware anti-windup)
        if !multirotor_simulator::flight::rpyt_control::tilt_saturated(roll_raw, pitch_raw, 25.0) {
            self.controller.set_i_error_pos(i_pos + ep * dt);
        }

        // Yaw-rate command: hold yaw = 0, using firmware EKF yaw (same frame as state).
        let yaw_rate_deg_s = yaw_rate_cmd(0.0_f32, entry.yaw, 30.0, 120.0);

        Some((ref_pos.x, ref_pos.y, ref_pos.z, thrust_n, roll_cmd_deg, pitch_cmd_deg, yaw_rate_deg_s))
    }
}

async fn fw_logging_step(
    log_data: &mut Vec<FwLogEntry>,
    last_print: &mut Instant,
    stream1: &LogStream,
    stream2: &LogStream,
    stream3: &LogStream,
    stream4: &LogStream,
    stream5: &LogStream,
    ai_feat: u32,
    start: &Instant,
    mekf: &mut Mekf,
    mekf_seeded: &mut bool,
    log_file: &mut File,
    shadow: &mut ShadowCtx,
    pending_vo: (f32, f32, f32),
    pending_pg: (f32, f32, u32),
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
        // motion.deltaX/Y are int16 raw pixel counts (accumulated since last read)
        entry.flow_dx = get_f32(d, "motion.deltaX");
        entry.flow_dy = get_f32(d, "motion.deltaY");
        entry.vbat    = get_f32(d, "pm.vbat");
    }

    if let Ok(p) = stream5.next().await {
        let d = &p.data;
        // Multi-ranger: uint16 mm → metres; 0 or u16::MAX → 0.0 (not available).
        let mr = CrtpMultiRangeAdapter::from_log_row(
            get_f32(d, "range.front") as u16,
            get_f32(d, "range.back")  as u16,
            get_f32(d, "range.left")  as u16,
            get_f32(d, "range.right") as u16,
            get_f32(d, "range.up")    as u16,
            0u16, // down channel comes from Flow Deck (already in range_z above)
        );
        entry.multi_front = mr.front_m.unwrap_or(0.0);
        entry.multi_back  = mr.back_m.unwrap_or(0.0);
        entry.multi_left  = mr.left_m.unwrap_or(0.0);
        entry.multi_right = mr.right_m.unwrap_or(0.0);
        entry.multi_up    = mr.up_m.unwrap_or(0.0);
    }

    // AI Deck feature count (from background CPX task via atomic).
    entry.ai_feat_count = ai_feat;

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
            entry.mekf_pitch = -pitch_r.to_degrees(); // negate: CF firmware pitch positive = nose-down; ZYX convention positive = nose-up
            entry.mekf_yaw   = yaw_r.to_degrees();
            entry.mekf_x     = px;
            entry.mekf_y     = py;
            entry.mekf_z     = pz;
        }
    }

    // ── Shadow geometric controller (passenger — logged only, NEVER sent to drone) ──
    if let Some((rx, ry, rz, thr, roll_c, pitch_c, yaw_rate_c)) = shadow.compute(&entry, *mekf_seeded) {
        entry.our_ref_x      = rx;
        entry.our_ref_y      = ry;
        entry.our_ref_z      = rz;
        entry.our_thrust     = thr;
        entry.our_roll_cmd   = roll_c;
        entry.our_pitch_cmd  = -pitch_c; // negate to match CF firmware pitch convention
        entry.our_yaw_rate_cmd = yaw_rate_c;
    }

    if last_print.elapsed() >= Duration::from_secs(1) {
        println!(
            "[t={:3}s] z={:+5.3} vx={:+5.3} vy={:+5.3} thrust={:5} roll={:+5.1} vbat={:.2} | mekf x={:+5.3} y={:+5.3} z={:+5.3} | shadow roll={:+5.1} pitch={:+5.1} thr={:.3}",
            start.elapsed().as_secs(),
            entry.pos_z, entry.vel_x, entry.vel_y, entry.thrust, entry.roll, entry.vbat,
            entry.mekf_x, entry.mekf_y, entry.mekf_z,
            entry.our_roll_cmd, entry.our_pitch_cmd, entry.our_thrust,
        );
        *last_print = Instant::now();
    }

    // Fill in VO values from the previous control cycle (one-step lag).
    entry.vo_x     = pending_vo.0;
    entry.vo_y     = pending_vo.1;
    entry.vo_sigma = pending_vo.2;

    // Fill in pose-graph values from the previous control cycle (one-step lag).
    entry.pg_x     = pending_pg.0;
    entry.pg_y     = pending_pg.1;
    entry.lc_count = pending_pg.2;

    // Write row immediately to disk so data survives a crash/panic.
    let _ = writeln!(log_file,
        "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.0},{:.0},{:.2},{:.2},{:.2},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{:.4},{:.4},{:.4},{:.4},{:.4},{},{:.4},{:.4},{:.4},{:.4},{:.4},{}",
        entry.time_ms,
        entry.pos_x, entry.pos_y, entry.pos_z,
        entry.vel_x, entry.vel_y, entry.vel_z,
        entry.roll, entry.pitch, entry.yaw,
        entry.thrust, entry.vbat,
        entry.gyro_x, entry.gyro_y, entry.gyro_z,
        entry.acc_x, entry.acc_y, entry.acc_z,
        entry.rate_roll, entry.rate_pitch, entry.rate_yaw,
        entry.range_z,
        entry.flow_dx, entry.flow_dy,
        entry.mekf_roll, entry.mekf_pitch, entry.mekf_yaw,
        entry.mekf_x, entry.mekf_y, entry.mekf_z,
        entry.our_ref_x, entry.our_ref_y, entry.our_ref_z,
        entry.our_thrust,
        entry.our_roll_cmd, entry.our_pitch_cmd, entry.our_yaw_rate_cmd,
        entry.multi_front, entry.multi_back, entry.multi_left,
        entry.multi_right, entry.multi_up,
        entry.ai_feat_count,
        entry.vo_x, entry.vo_y, entry.vo_sigma,
        entry.pg_x, entry.pg_y, entry.lc_count,
    );
    log_data.push(entry);
}

// ── Safety layer ─────────────────────────────────────────────────────────────
//
// Reactive collision avoidance via multi-ranger position-setpoint correction.
//
// When any multi-ranger reading falls below the safety threshold the planned
// position setpoint is nudged away from the obstacle.  The correction is
// proportional to proximity and capped at SAFETY_MAX_OFFSET_M, so normal
// flight is unaffected when the drone is well clear of walls.
//
// Applied only during active maneuvers (hover/circle/figure8 setpoint_position
// calls).  Ramp-up and landing use setpoint_hover with zero velocity, so the
// drone is already being commanded to stay put — no correction needed there.
//
// Coordinate convention:
//   Body  +X = forward  (multi_front measures distance to front wall)
//   Body  +Y = left     (multi_left  measures distance to left  wall)
//   Body  +Z = up       (multi_up    measures distance to ceiling)
//   World frame = body frame rotated by EKF yaw (roll/pitch neglected for
//   the purposes of this horizontal correction).

/// Horizontal range at which repulsion starts (metres).
const SAFETY_DIST_HORIZ_M: f32 = 0.40;
/// Ceiling range at which repulsion starts (metres).
const SAFETY_DIST_UP_M: f32 = 0.35;
/// Maximum correction per axis (metres).  Large enough to matter; small enough
/// not to cause abrupt position jumps the firmware PID cannot track.
const SAFETY_MAX_OFFSET_M: f32 = 0.12;

/// Compute a safety-adjusted position setpoint.
///
/// Returns `(safe_x, safe_y, safe_z)` — the planned setpoint shifted away from
/// any obstacle detected by the multi-ranger.  When all readings are above the
/// threshold the output equals the input.
///
/// # Arguments
/// - `x, y, z`       — Planned position in EKF world frame (metres).
/// - `yaw_deg`       — EKF heading used to rotate body→world (degrees).
/// - `front/back/left/right/up` — Multi-ranger distances (metres; 0.0 = not available).
fn safety_position(
    x: f32, y: f32, z: f32,
    yaw_deg:  f32,
    front_m:  f32, back_m:  f32,
    left_m:   f32, right_m: f32,
    up_m:     f32,
) -> (f32, f32, f32) {
    // Returns a negative offset (push away) proportional to proximity.
    let repel = |dist_m: f32, threshold: f32| -> f32 {
        if dist_m > 0.02 && dist_m < threshold {
            -SAFETY_MAX_OFFSET_M * (1.0 - dist_m / threshold)
        } else {
            0.0
        }
    };

    // Body-frame corrections: front/back share the X axis, left/right share Y.
    // repel() is negative (push away), so:
    //   front close → body_x negative (push backward)
    //   back  close → -repel() → body_x positive (push forward)
    let body_x = repel(front_m, SAFETY_DIST_HORIZ_M) - repel(back_m,  SAFETY_DIST_HORIZ_M);
    let body_y = repel(left_m,  SAFETY_DIST_HORIZ_M) - repel(right_m, SAFETY_DIST_HORIZ_M);
    let body_z = repel(up_m,    SAFETY_DIST_UP_M);

    // Rotate horizontal correction from body → world frame via yaw only.
    let (sin_y, cos_y) = yaw_deg.to_radians().sin_cos();
    let dx = body_x * cos_y - body_y * sin_y;
    let dy = body_x * sin_y + body_y * cos_y;

    (x + dx, y + dy, z + body_z)
}

/// Extract the latest multi-ranger readings from the log and return a
/// safety-adjusted position.  Falls back to the raw setpoint if no log
/// data is available yet.
fn safe_setpoint(
    log_data: &[FwLogEntry],
    x: f32, y: f32, z: f32,
) -> (f32, f32, f32) {
    match log_data.last() {
        Some(e) => safety_position(
            x, y, z, e.yaw,
            e.multi_front, e.multi_back,
            e.multi_left,  e.multi_right,
            e.multi_up,
        ),
        None => (x, y, z),
    }
}

/// Safety-adjusted setpoint that combines the reactive multi-ranger layer
/// with the persistent occupancy map.
///
/// First applies `safe_setpoint` (multi-ranger proximity repulsion), then
/// probes 8 world-frame directions around the resulting setpoint in the
/// occupancy map.  Any occupied voxel within `OMAP_PROBE_DIST` nudges the
/// setpoint away by up to `OMAP_MAX_REPEL`.
///
/// The map layer complements the reactive layer: it catches obstacles that
/// currently return out-of-range readings (e.g. specular walls) but were
/// previously confirmed occupied by the multi-ranger.
fn safe_setpoint_omap(
    log_data: &[FwLogEntry],
    omap: &OccupancyMap,
    x: f32, y: f32, z: f32,
) -> (f32, f32, f32) {
    // Step 1: reactive multi-ranger layer.
    let (x1, y1, z1) = safe_setpoint(log_data, x, y, z);

    // Step 2: occupancy-map repulsion (world-frame horizontal ring).
    const PROBE_DIST: f32 = 0.25; // m — ring radius around setpoint
    const OMAP_MAX_REPEL: f32 = 0.08; // m — max per-obstacle correction

    let mut dx = 0.0f32;
    let mut dy = 0.0f32;
    for i in 0..8u32 {
        let angle = i as f32 * std::f32::consts::PI / 4.0;
        let (ca, sa) = angle.sin_cos();
        let px = x1 + PROBE_DIST * ca;
        let py = y1 + PROBE_DIST * sa;
        if omap.is_occupied(Vec3::new(px, py, z1)) {
            // Repel away from this obstacle direction.
            dx -= OMAP_MAX_REPEL * ca;
            dy -= OMAP_MAX_REPEL * sa;
        }
    }
    // Clamp total map correction to OMAP_MAX_REPEL.
    let mag = (dx * dx + dy * dy).sqrt();
    let (dx, dy) = if mag > OMAP_MAX_REPEL {
        (dx / mag * OMAP_MAX_REPEL, dy / mag * OMAP_MAX_REPEL)
    } else {
        (dx, dy)
    };

    (x1 + dx, y1 + dy, z1)
}

async fn run_firmware_mode(cf: &Crazyflie, maneuver: &str) -> Result<(), Box<dyn std::error::Error>> {
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

    // Block 3: Raw sensors — 6 floats = 24 bytes @ 20 Hz (within 26-byte limit).
    // pm.vbat moved to block 4 to stay under the limit (7 floats = 28 bytes > 26).
    let mut block3 = cf.log.create_block().await?;
    for v in ["gyro.x", "gyro.y", "gyro.z", "acc.x", "acc.y", "acc.z"] {
        add_var(&mut block3, v).await;
    }
    let stream3 = block3.start(LogPeriod::from_millis(50)?).await?;

    // Block 4: ToF range + optical flow + battery — 10 bytes @ 20 Hz.
    // range.zrange : uint16 [mm]  → converted to [m] on read
    // motion.deltaX: int16  [px]  → raw pixel accumulation since last read
    // motion.deltaY: int16  [px]  → raw pixel accumulation since last read
    // pm.vbat      : float  [V]
    let mut block4 = cf.log.create_block().await?;
    for v in ["range.zrange", "motion.deltaX", "motion.deltaY", "pm.vbat"] {
        add_var(&mut block4, v).await;
    }
    let stream4 = block4.start(LogPeriod::from_millis(50)?).await?;

    // Block 5: Multi-ranger Deck — 5 × uint16 = 10 bytes @ 20 Hz.
    // range.front/back/left/right/up: VL53L1x sensors [mm] → converted to [m].
    // Out-of-range sentinel: 65535 (u16::MAX) → reported as 0.0 in CSV.
    // If the Multi-ranger Deck is not attached, add_var prints an error but
    // does not abort; the block returns zeros for all variables.
    let mut block5 = cf.log.create_block().await?;
    for v in ["range.front", "range.back", "range.left", "range.right", "range.up"] {
        add_var(&mut block5, v).await;
    }
    let stream5 = block5.start(LogPeriod::from_millis(50)?).await?;

    // Optional AI Deck CPX camera — only if --ai-deck flag is present.
    //
    // Two shared values updated by the background task:
    //   ai_feat_count  — latest FAST-9 feature count (logged per cycle).
    //   ai_kf_result   — latest keyframe registration result (logged on each new keyframe).
    //
    // The background task also runs a KeyframeStore; it reads the current drone
    // pose from ai_pose (written by the main loop each fw_logging_step).
    let ai_feat_count  = Arc::new(AtomicU32::new(0));
    let ai_kf_result: Arc<std::sync::Mutex<Option<KeyframeResult>>> =
        Arc::new(std::sync::Mutex::new(None));
    let ai_loop_result: Arc<std::sync::Mutex<Option<LoopConstraint>>> =
        Arc::new(std::sync::Mutex::new(None));
    // (pos_x, pos_y, pos_z, yaw_deg, range_z) — written by main loop, read by AI task.
    let ai_pose: Arc<std::sync::Mutex<(f32,f32,f32,f32,f32)>> =
        Arc::new(std::sync::Mutex::new((0.0, 0.0, 0.3, 0.0, 0.3)));

    let ai_deck_mode  = std::env::args().any(|a| a == "--ai-deck");
    if ai_deck_mode {
        use multirotor_simulator::perception::sensors::cpx::CpxCamera;
        let feat_clone  = Arc::clone(&ai_feat_count);
        let kf_clone    = Arc::clone(&ai_kf_result);
        let loop_clone  = Arc::clone(&ai_loop_result);
        let pose_clone  = Arc::clone(&ai_pose);
        tokio::spawn(async move {
            // kf_store lives outside the reconnect loop so the full SLAM history
            // (keyframes, pose graph, spatial grid) is preserved across Nina reboots.
            let mut kf_store = KeyframeStore::new();
            loop {
                // ── reconnect loop ────────────────────────────────────────────
                // Nina (ESP32) reboots every 13–70 s due to WiFi TX-queue overflow.
                // When it comes back up (~5 s later) we open a fresh TcpStream and
                // resume streaming without resetting the SLAM state.
                println!("[AI Deck] connecting to 192.168.4.1:5000 ...");
                let mut cam = match CpxCamera::connect("192.168.4.1:5000").await {
                    Ok(c)  => { println!("[AI Deck] connected"); c }
                    Err(e) => {
                        eprintln!("[AI Deck] connect failed: {e} — retrying in 10 s");
                        sleep(Duration::from_secs(10)).await;
                        continue;
                    }
                };

                // ── frame loop ────────────────────────────────────────────────
                loop {
                    match cam.recv_frame().await {
                        Ok(frame) => {
                            let n = detect_features(&frame, 20).len() as u32;
                            feat_clone.store(n, Ordering::Relaxed);

                            // Feed frame into keyframe store with latest pose.
                            let (px, py, pz, yaw, rz) = {
                                *pose_clone.lock().unwrap()
                            };
                            let pos = multirotor_simulator::math::Vec3::new(px, py, pz);
                            if let Some(result) = kf_store.push(frame, pos, yaw, rz) {
                                println!("[KF] kf#{} matches={} inliers={} t=({:.3},{:.3},{:.3})m",
                                    result.kf_index, result.match_count, result.inlier_count,
                                    result.translation_m.x, result.translation_m.y, result.translation_m.z);

                                // Run loop closure detection on this new keyframe.
                                let new_idx = result.kf_index;
                                if let Some(lc) = kf_store.detect_loop(new_idx) {
                                    eprintln!("[LC] loop kf{}→kf{} inliers={} t=({:.3},{:.3})m",
                                        lc.from_idx, lc.to_idx, lc.inlier_count,
                                        lc.translation_world[0], lc.translation_world[1]);
                                    *loop_clone.lock().unwrap() = Some(lc);
                                }
                                *kf_clone.lock().unwrap() = Some(result);
                            }
                        }
                        Err(e) => {
                            eprintln!("[AI Deck] lost connection: {e} — reconnecting in 5 s");
                            feat_clone.store(0, Ordering::Relaxed);
                            sleep(Duration::from_secs(5)).await;
                            break; // break frame loop → outer reconnect loop
                        }
                    }
                }
                // cam is dropped here; the dead TcpStream is released.
            }
        });
    }

    let mut log_data: Vec<FwLogEntry> = Vec::new();
    let mut last_print = Instant::now();
    let mut mekf = Mekf::new(MekfParams::default());
    let mut mekf_seeded = false;

    // ── Phase 5: Visual odometry + occupancy map (shared across all maneuvers) ──
    let mut vo_traj   = VoTrajectory::new();
    let mut vo_seeded = false;
    /// Minimum VO position noise variance [m²] = (0.2 m sigma)².
    /// Prevents over-trusting a single keyframe step.
    const R_VO_MIN: f32 = 0.04;
    // Pending VO values written to each log row (one-cycle lag vs sensor data).
    let mut pending_vo: (f32, f32, f32) = (0.0, 0.0, 0.0);
    // Occupancy map — hoisted to outer scope so all maneuver arms can update and
    // query it, and so safe_setpoint_omap is available everywhere.
    let mut omap = OccupancyMap::new();

    // ── Phase 6: Pose-graph loop closure SLAM ────────────────────────────────
    let mut pose_graph = PoseGraph::new();
    // Pending pose-graph values written to each log row (one-cycle lag).
    let mut pending_pg: (f32, f32, u32) = (0.0, 0.0, 0);
    /// MEKF position noise for pose-graph correction [m²] = (0.1 m σ)².
    /// Tighter than VO noise — a loop closure is a hard geometric constraint.
    const R_LOOP: f32 = 0.01;

    // Shadow controller — starts with a hover placeholder; maneuver params and
    // traj_start are filled in once we know the maneuver origin (cx, cy).
    let mut shadow = ShadowCtx {
        maneuver: ShadowManeuver::Hover { cx: 0.0, cy: 0.0, height: 0.3 },
        traj_start: None,
        controller: GeometricController::default(),
        params: MultirotorParams::crazyflie(),
    };

    // Open CSV immediately — every row is written to disk as it arrives.
    // This means the log is preserved even if the drone crashes mid-flight.
    fs::create_dir_all("runs")?;
    let timestamp = Utc::now().format("%Y-%m-%d_%H-%M-%S");
    let filename = format!("runs/{}_{}.csv", maneuver, timestamp);
    let mut log_file = File::create(&filename)?;
    writeln!(log_file, "time_ms,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll,pitch,yaw,thrust,vbat,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,rate_roll,rate_pitch,rate_yaw,range_z,flow_dx,flow_dy,mekf_roll,mekf_pitch,mekf_yaw,mekf_x,mekf_y,mekf_z,our_ref_x,our_ref_y,our_ref_z,our_thrust,our_roll_cmd,our_pitch_cmd,our_yaw_rate_cmd,multi_front,multi_back,multi_left,multi_right,multi_up,ai_feat_count,vo_x,vo_y,vo_sigma,pg_x,pg_y,lc_count")?;
    println!("Log file opened: {}", filename);

    println!("Starting maneuver '{}' in 3 seconds...", maneuver);
    sleep(Duration::from_secs(3)).await;

    // ── Pre-takeoff: Kalman reset ─────────────────────────────────────────────
    // Pulse resetEstimation 1→0 then wait 1 s for the EKF to settle.
    // We do NOT poll variance here — on the ground the flow sensor reads garbage
    // (3 mm height) so variance never converges and the wait blocks forever.
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    let _ = cf.param.set("kalman.resetEstimation", 1u8).await;
    sleep(Duration::from_millis(200)).await;
    let _ = cf.param.set("kalman.resetEstimation", 0u8).await;
    println!("[Kalman] reset pulse sent, waiting 1 s...");
    sleep(Duration::from_secs(1)).await;

    // Single flight timer — every fw_logging_step call uses this same Instant
    // so time_ms is continuous and monotonic across all phases in the CSV.
    let flight_start = Instant::now();

    // Helper: after each fw_logging_step, publish the latest pose to the AI
    // Deck background task so its keyframe store captures it with the frame.
    let sync_ai_pose = |log_data: &[FwLogEntry]| {
        if let Some(e) = log_data.last() {
            if let Ok(mut g) = ai_pose.lock() {
                *g = (e.pos_x, e.pos_y, e.pos_z, e.yaw, e.range_z);
            }
        }
    };

    // ── Ramp up ──────────────────────────────────────────────────────────────
    println!("Ramping up...");
    for y in 0..15 {
        let zdistance = y as f32 / 50.0;
        fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stream5, ai_feat_count.load(Ordering::Relaxed), &flight_start, &mut mekf, &mut mekf_seeded, &mut log_file, &mut shadow, pending_vo, pending_pg).await;
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(100)).await;
    }

    // ── Stabilize for 8 s at 0.3 m ───────────────────────────────────────────
    println!("Stabilizing hover for 8 seconds...");
    {
        let deadline = Instant::now() + Duration::from_secs(8);
        while Instant::now() < deadline {
            fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stream5, ai_feat_count.load(Ordering::Relaxed), &flight_start, &mut mekf, &mut mekf_seeded, &mut log_file, &mut shadow, pending_vo, pending_pg).await;
            sync_ai_pose(&log_data);
            step_perception!(log_data, omap, mekf, mekf_seeded,
                             ai_kf_result, vo_traj, vo_seeded, pending_vo,
                             ai_loop_result, pose_graph, pending_pg);
            cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.3).await?;
            sleep(Duration::from_millis(50)).await;
        }
    }

    // ── Sample EKF XY to get maneuver origin ─────────────────────────────────
    // Average pos_x/y over 2 s while holding hover — this is the (cx, cy) centre
    // used as the reference point for circle and figure-8 trajectories.
    println!("Sampling EKF position for maneuver origin (2 s)...");
    let (cx, cy) = {
        let mut sum_x = 0.0f32;
        let mut sum_y = 0.0f32;
        let mut count = 0u32;
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stream5, ai_feat_count.load(Ordering::Relaxed), &flight_start, &mut mekf, &mut mekf_seeded, &mut log_file, &mut shadow, pending_vo, pending_pg).await;
            cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.3).await?;
            if let Some(last) = log_data.last() {
                sum_x += last.pos_x;
                sum_y += last.pos_y;
                count += 1;
            }
            sleep(Duration::from_millis(50)).await;
        }
        let mx = if count > 0 { sum_x / count as f32 } else { 0.0 };
        let my = if count > 0 { sum_y / count as f32 } else { 0.0 };
        println!("Maneuver origin: cx={:.3} m, cy={:.3} m (mean over {} samples)", mx, my, count);
        (mx, my)
    };

    // Update shadow with real maneuver origin — hover placeholder until traj phase.
    shadow.maneuver = ShadowManeuver::Hover { cx, cy, height: 0.3 };

    match maneuver {
        "hover" => {
            // Shadow tracks the same hover setpoint as the firmware.
            shadow.maneuver = ShadowManeuver::Hover { cx, cy, height: 0.3 };
            shadow.traj_start = Some(Instant::now());
            println!("Holding position ({:.3}, {:.3}) at 0.3 m for 12 seconds...", cx, cy);
            let deadline = Instant::now() + Duration::from_secs(12);
            while Instant::now() < deadline {
                fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stream5, ai_feat_count.load(Ordering::Relaxed), &flight_start, &mut mekf, &mut mekf_seeded, &mut log_file, &mut shadow, pending_vo, pending_pg).await;
                sync_ai_pose(&log_data);
                step_perception!(log_data, omap, mekf, mekf_seeded,
                                 ai_kf_result, vo_traj, vo_seeded, pending_vo,
                                 ai_loop_result, pose_graph, pending_pg);
                let (sx, sy, sz) = safe_setpoint_omap(&log_data, &omap, cx, cy, 0.3);
                cf.commander.setpoint_position(sx, sy, sz, 0.0).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }
        "circle" => {
            let radius = 0.25f32;
            let height = 0.3f32;
            let omega = 0.3f32;  // reduced from 0.6 — gentler circle, less drift into walls
            println!("Circle: radius {:.2} m, height {:.2} m, ω = {:.2} rad/s, centre ({:.3}, {:.3})", radius, height, omega, cx, cy);
            // Shadow tracks the same circle the firmware will execute.
            shadow.maneuver = ShadowManeuver::Circle { cx, cy, height, radius, omega };

            let x_start = cx + radius;
            let y_start = cy;
            println!("Moving to circle start point ({:.3}, {:.3})...", x_start, y_start);
            let move_deadline = Instant::now() + Duration::from_secs(3);
            while Instant::now() < move_deadline {
                fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stream5, ai_feat_count.load(Ordering::Relaxed), &flight_start, &mut mekf, &mut mekf_seeded, &mut log_file, &mut shadow, pending_vo, pending_pg).await;
                sync_ai_pose(&log_data);
                step_perception!(log_data, omap, mekf, mekf_seeded,
                                 ai_kf_result, vo_traj, vo_seeded, pending_vo,
                                 ai_loop_result, pose_graph, pending_pg);
                let (sx, sy, sz) = safe_setpoint_omap(&log_data, &omap, x_start, y_start, height);
                cf.commander.setpoint_position(sx, sy, sz, 0.0).await?;
                sleep(Duration::from_millis(50)).await;
            }

            println!("Starting circle trajectory...");
            let traj_start = Instant::now();
            shadow.traj_start = Some(traj_start);
            let deadline = traj_start + Duration::from_secs(50); // 50 s ≈ 2.4 laps at ω=0.3
            while Instant::now() < deadline {
                let t = traj_start.elapsed().as_secs_f32();
                let x = cx + radius * (omega * t).cos();
                let y = cy + radius * (omega * t).sin();
                fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stream5, ai_feat_count.load(Ordering::Relaxed), &flight_start, &mut mekf, &mut mekf_seeded, &mut log_file, &mut shadow, pending_vo, pending_pg).await;
                sync_ai_pose(&log_data);
                step_perception!(log_data, omap, mekf, mekf_seeded,
                                 ai_kf_result, vo_traj, vo_seeded, pending_vo,
                                 ai_loop_result, pose_graph, pending_pg);
                let (sx, sy, sz) = safe_setpoint_omap(&log_data, &omap, x, y, height);
                cf.commander.setpoint_position(sx, sy, sz, 0.0).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }
        "figure8" => {
            let a = 0.25f32;
            let b = 0.15f32;
            let omega = 0.5f32;
            let height = 0.3f32;
            println!("Figure-8: a={:.2}, b={:.2}, ω={:.2}, centre ({:.3}, {:.3})", a, b, omega, cx, cy);
            // Shadow tracks the same figure-8 the firmware will execute.
            shadow.maneuver = ShadowManeuver::Figure8 { cx, cy, height, a, b, omega };

            // Phase offset π/2: x = a·cos(ωt), y = −b·sin(2ωt)
            // Start point (t=0): (cx+a, cy) — drone is moved there first so
            // there is no position jump on the first trajectory command.
            let x_start = cx + a;
            let y_start = cy;
            println!("Moving to figure-8 start point ({:.3}, {:.3})...", x_start, y_start);
            let move_deadline = Instant::now() + Duration::from_secs(3);
            while Instant::now() < move_deadline {
                fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stream5, ai_feat_count.load(Ordering::Relaxed), &flight_start, &mut mekf, &mut mekf_seeded, &mut log_file, &mut shadow, pending_vo, pending_pg).await;
                sync_ai_pose(&log_data);
                step_perception!(log_data, omap, mekf, mekf_seeded,
                                 ai_kf_result, vo_traj, vo_seeded, pending_vo,
                                 ai_loop_result, pose_graph, pending_pg);
                let (sx, sy, sz) = safe_setpoint_omap(&log_data, &omap, x_start, y_start, height);
                cf.commander.setpoint_position(sx, sy, sz, 0.0).await?;
                sleep(Duration::from_millis(50)).await;
            }

            println!("Starting figure-8 trajectory...");
            let traj_start = Instant::now();
            shadow.traj_start = Some(traj_start);
            let deadline = traj_start + Duration::from_secs(40);
            while Instant::now() < deadline {
                let t = traj_start.elapsed().as_secs_f32();
                let x = cx + a * (omega * t).cos();
                let y = cy - b * (2.0 * omega * t).sin();
                fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stream5, ai_feat_count.load(Ordering::Relaxed), &flight_start, &mut mekf, &mut mekf_seeded, &mut log_file, &mut shadow, pending_vo, pending_pg).await;
                sync_ai_pose(&log_data);
                step_perception!(log_data, omap, mekf, mekf_seeded,
                                 ai_kf_result, vo_traj, vo_seeded, pending_vo,
                                 ai_loop_result, pose_graph, pending_pg);
                let (sx, sy, sz) = safe_setpoint_omap(&log_data, &omap, x, y, height);
                cf.commander.setpoint_position(sx, sy, sz, 0.0).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }
        "explore" => {
            // Frontier-based autonomous exploration.
            // Planner runs a SCAN→NAVIGATE→LAND state machine using the
            // live OccupancyMap (hoisted to outer scope, shared with all arms).
            let hover_z = 0.3f32;
            let mut planner = ExplorationPlanner::new(hover_z);
            let explore_start = Instant::now();

            println!("Starting exploration (max {} s)...", 120);
            loop {
                fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stream5, ai_feat_count.load(Ordering::Relaxed), &flight_start, &mut mekf, &mut mekf_seeded, &mut log_file, &mut shadow, pending_vo, pending_pg).await;
                sync_ai_pose(&log_data);
                // Map update + VO integration (same as all other maneuver arms).
                step_perception!(log_data, omap, mekf, mekf_seeded,
                                 ai_kf_result, vo_traj, vo_seeded, pending_vo,
                                 ai_loop_result, pose_graph, pending_pg);

                let pos = log_data.last()
                    .map(|e| Vec3::new(e.pos_x, e.pos_y, e.pos_z))
                    .unwrap_or(Vec3::new(cx, cy, hover_z));
                let yaw_now = log_data.last().map(|e| e.yaw).unwrap_or(0.0);
                let vbat    = log_data.last().map(|e| e.vbat).unwrap_or(4.0);
                let elapsed = explore_start.elapsed().as_secs_f32();

                let cmd = planner.step(pos, yaw_now, &omap, vbat, elapsed);
                match cmd {
                    ExplorationCommand::Hold { x, y, z, yaw_deg } => {
                        let (sx, sy, sz) = safe_setpoint_omap(&log_data, &omap, x, y, z);
                        cf.commander.setpoint_position(sx, sy, sz, yaw_deg).await?;
                    }
                    ExplorationCommand::GoTo { x, y, z, yaw_deg } => {
                        let (sx, sy, sz) = safe_setpoint_omap(&log_data, &omap, x, y, z);
                        cf.commander.setpoint_position(sx, sy, sz, yaw_deg).await?;
                    }
                    ExplorationCommand::Land { reason } => {
                        println!("[explore] Landing: {}", reason);
                        // Save map before breaking into the landing ramp.
                        let stats = omap.stats();
                        println!("[explore] Map: {} occupied, {} free voxels",
                                 stats.n_occupied, stats.n_free);
                        let ply = omap.to_ply();
                        let map_path = format!("results/data/explore_map_{}.ply", timestamp);
                        if fs::create_dir_all("results/data").is_ok() {
                            let _ = fs::write(&map_path, &ply);
                            println!("[explore] Map saved: {}", map_path);
                        }
                        break;
                    }
                }
                sleep(Duration::from_millis(50)).await;
            }
        }
        _ => {}
    }

    println!("Ramping down gently...");
    for step in (0..40).rev() {
        let zdistance = step as f32 / 100.0;
        fw_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &stream4, &stream5, ai_feat_count.load(Ordering::Relaxed), &flight_start, &mut mekf, &mut mekf_seeded, &mut log_file, &mut shadow, pending_vo, pending_pg).await;
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
    drop(stream4);
    drop(stream5);
    cf.disconnect().await;
    println!("Disconnected cleanly.");
    Ok(())
}

/// Reset the Crazyflie Kalman estimator and wait until its position variance has
/// converged (10-sample rolling window, max−min < 0.001 for X, Y and Z).
///
/// This mirrors the Python `reset_estimator()` + `wait_for_position_estimator()`
/// pattern used in the original `autonomous_sequence_high_level.py` script.
///
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

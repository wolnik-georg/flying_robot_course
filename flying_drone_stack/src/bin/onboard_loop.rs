//! Mode D Vertical Loop — full 360° aerobatic loop in the X-Z plane.
//!
//! The loop is planned as a true 3D circle: position polynomial encodes both X
//! (cx) and Z (cz) motion; firmware evaluates them with z_mode=1.  Y is held
//! at oy throughout.  Planning Mode 2 adds an attitude polynomial that prescribes
//! the correct quaternion through the inverted phase without relying on flatness.
//!
//! Inversion condition: R·ω² > g  →  ω_min = sqrt(g/R)
//!   R=0.5m → ω_min=4.43 rad/s → speed_min ≈ 2.1× (brushless + mocap required)
//!
//! Usage:
//!   cargo run --release --bin onboard_loop
//!   cargo run --release --bin onboard_loop -- --speed 2.5 --radius 0.5 --reps 3
//!   cargo run --release --bin onboard_loop -- --mode 1 --kt 0.5
//!   cargo run --release --bin onboard_loop -- --mode 2 --kt 0.5
//!   cargo run --release --bin onboard_loop -- --mode 3 --kt 0.5
//!
//! --speed  : trajectory speed multiplier for Mode 0 only (default 1.0)
//!            Ignored for Mode 1/2 — use --kt to control speed there.
//! --radius : loop radius in metres (default 0.5)
//! --reps   : number of complete loops (default 1)
//! --mode   : planning mode 0=Spline 1=Richter 2=Se3 3=Paper (default 0)
//! --kt     : aggressiveness for timing (Mode 1/2, default 0.5)
//!
//! Planning mode comparison:
//!   Mode 0/1: position polynomial in 3D; firmware derives attitude via flatness.
//!             Works at sub-inversion speeds. Flatness is well-defined because thrust
//!             vector never reaches zero (only reduces at the top).
//!   Mode 2:   same 3D position polynomial + attitude polynomial (explicit quaternion
//!             profile, monotonically increasing pitch through 360°). Required for
//!             above-inversion speeds where flatness is undefined at zero-thrust.
//!             Firmware uses traj.att_mode=1 to evaluate the uploaded attitude poly.
//!
//! Mode 0 baseline: r=0.5m, t_lap=3.0s, v_avg ~1.05 m/s (sub-inversion).
//! Mode 1/2 --kt reference table (loop r=0.5m, circumference ~3.14m):
//!   --kt 0.1    →  v_avg ~1.0  m/s  ~3.1s/lap   (sub-inversion, comparable to Mode 0)
//!   --kt 0.3    →  v_avg ~1.32 m/s  ~2.4s/lap   (moderate, sub-inversion)
//!   --kt 0.5    →  v_avg ~1.56 m/s  ~2.0s/lap   ← default (sub-inversion)
//!   --kt 1.5    →  v_avg ~2.34 m/s  ~1.3s/lap   ← above inversion threshold (ω ~4.8 rad/s)

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{TrajectoryPlanner, Se3Waypoint, Waypoint, Vec3, Quat};

const GRAVITY:        f32 = 9.81;
const RADIUS_DEFAULT: f32 = 0.5;
const T_LAP_DEFAULT:  f32 = 3.0;

/// 3D loop waypoints in the X-Z plane.
///
/// Position: x = R·sin(θ),  y = 0,  z = R·(1 − cos(θ)).
/// Starts and ends at (0, 0, 0).  Periodic — first == last in position.
fn loop_waypoints_3d(radius: f32, t_lap: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let n = 8usize;
    let seg_dur = t_lap / n as f32;
    let waypoints: Vec<Waypoint> = (0..=n).map(|i| {
        let theta = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
        Waypoint {
            pos: Vec3::new(
                radius * theta.sin(),
                0.0,
                radius * (1.0 - theta.cos()),
            ),
            yaw: 0.0,
        }
    }).collect();
    (waypoints, vec![seg_dur; n])
}

/// Se3 loop waypoints with explicit quaternion attitude for each point on the circle.
///
/// Attitude = rotation around world-Y by -θ (Ry(-θ)) so body-z tracks the
/// centripetal direction.  At θ=0 (bottom): upright.  At θ=π (top): inverted.
///
/// q(θ) = [cos(θ/2), 0, −sin(θ/2), 0]  in [w,x,y,z] convention.
fn loop_se3_waypoints(radius: f32, t_lap: f32) -> (Vec<Se3Waypoint>, Vec<f32>) {
    let n = 8usize;
    let seg_dur = t_lap / n as f32;
    let waypoints: Vec<Se3Waypoint> = (0..=n).map(|i| {
        let theta = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
        let pos = Vec3::new(
            radius * theta.sin(),
            0.0,
            radius * (1.0 - theta.cos()),
        );
        let att = Quat::new(
            (theta * 0.5).cos(),
            0.0,
            -(theta * 0.5).sin(),
            0.0,
        );
        Se3Waypoint::new(pos, att)
    }).collect();
    (waypoints, vec![seg_dur; n])
}

fn build_planner(mode: u8, radius: f32, t_lap: f32, k_t: f32) -> TrajectoryPlanner {
    match mode {
        1 => {
            let (wps, _) = loop_waypoints_3d(radius, t_lap);
            TrajectoryPlanner::richter(&wps, k_t, true)
                .expect("Loop Mode 1 QP failed")
        }
        2 => {
            // Reuse Richter timing from Mode 1 for consistent speed comparison.
            let (flat_wps, _) = loop_waypoints_3d(radius, t_lap);
            let m1 = TrajectoryPlanner::richter(&flat_wps, k_t, true)
                .expect("Loop Mode 2 timing (Richter) failed");
            let kt_durs = m1.segment_durations();
            let (se3_wps, _) = loop_se3_waypoints(radius, t_lap);
            TrajectoryPlanner::se3(&se3_wps, &kt_durs, 0.031, true)
                .expect("Loop Mode 2 QP failed")
        }
        3 => {
            let (wps, _) = loop_waypoints_3d(radius, t_lap);
            TrajectoryPlanner::paper(&wps, k_t, true)
                .expect("Loop Mode 3 paper QP failed")
        }
        _ => {
            let (wps, durs) = loop_waypoints_3d(radius, t_lap);
            TrajectoryPlanner::spline(&wps, &durs, true)
                .expect("Loop Mode 0 QP failed")
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let speed: f32 = args.iter().position(|a| a == "--speed")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let radius: f32 = args.iter().position(|a| a == "--radius")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(RADIUS_DEFAULT);
    let n_reps: u32 = args.iter().position(|a| a == "--reps")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1);
    let mode: u8 = args.iter().position(|a| a == "--mode")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(0);
    let k_t: f32 = args.iter().position(|a| a == "--kt")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(0.5);

    if n_reps == 0 || n_reps > 20 { eprintln!("--reps must be 1-20"); std::process::exit(1); }
    if mode > 3                   { eprintln!("--mode must be 0, 1, 2, or 3"); std::process::exit(1); }

    let t_lap  = T_LAP_DEFAULT / speed;
    let omega  = 2.0 * std::f32::consts::PI / t_lap;
    let omega_min = (GRAVITY / radius).sqrt();
    let speed_for_inversion = omega_min / (2.0 * std::f32::consts::PI / T_LAP_DEFAULT);
    let top_height = HOVER_HEIGHT + 2.0 * radius;

    println!("Vertical Loop | Mode D | planning_mode={}", mode);
    println!("  R={:.2}m  T={:.2}s/lap  omega={:.2}rad/s  speed={:.1}x  reps={}",
             radius, t_lap, omega, speed, n_reps);
    println!("  Bottom: z={:.1}m  Top: z={:.1}m", HOVER_HEIGHT, top_height);
    println!("  Inversion threshold: speed>={:.1}x (omega>={:.2}rad/s)",
             speed_for_inversion, omega_min);
    if omega >= omega_min {
        println!("  INVERTED FLIGHT — brushless + mocap required");
    } else {
        println!("  Sub-inversion (upright throughout). To invert: --speed {:.1}", speed_for_inversion);
    }

    let planner = build_planner(mode, radius, t_lap, k_t);
    let spline = planner.as_spline();
    let traj_total = spline.total_time;
    let coefs = serialise_coefs(spline);
    let z_coefs = serialise_z_coefs(spline);
    let n_segs = spline.segments.len();
    let att_coefs_raw = planner.attitude_poly_coefs();
    let att_coefs = serialise_att_coefs(&att_coefs_raw);
    println!("Planned: {} segs  {} pos coefs  {} z coefs  lap={:.2}s",
             n_segs, coefs.len(), z_coefs.len(), traj_total);
    if mode >= 2 && mode != 3 { println!("  Mode {}: uploading {} att coefs", mode, att_coefs.len()); }

    let link_ctx = LinkContext::new();
    let cf = connect_drone(&link_ctx).await?;
    verify_firmware(&cf).await?;

    let (sa, sb, sc) = setup_log_blocks(&cf).await?;
    kalman_reset(&cf).await?;
    let o = sample_origin(&cf, &sa, &sb, &sc).await?;

    upload_trajectory(&cf, &coefs).await?;
    upload_z_trajectory(&cf, &z_coefs).await?;
    if mode >= 2 && mode != 3 { upload_att_trajectory(&cf, &att_coefs).await?; }
    cf.param.set("traj.nseg",     n_segs as u8).await?;
    cf.param.set("traj.ox",       o.ox).await?;
    cf.param.set("traj.oy",       o.oy).await?;
    cf.param.set("traj.hz",       HOVER_HEIGHT).await?;
    cf.param.set("traj.dz",       0.0f32).await?;
    cf.param.set("traj.z_mode",   1u8).await?;
    cf.param.set("traj.att_mode", ((mode >= 2) && (mode != 3)) as u8).await?;

    ramp_to_hover(&cf, &o, &sa, &sb, &sc).await?;
    hover_settle(&cf, &o, &sa, &sb, &sc).await?;

    cf.param.set("traj.mode",  1u8).await?;   // Mode D (all onboard trajectories)
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== LOOP! R={:.2}m  {:.1}s/lap  reps={}  top={:.1}m ===",
             radius, traj_total, n_reps, top_height);
    if omega >= omega_min { println!("    (inverted flight at top)"); }

    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0 = Instant::now();

    while Instant::now() < t0 + traj_end {
        let _t = (Instant::now() - t0).as_secs_f32();
        // Keepalive: firmware uses the polynomial for position — send hover as watchdog.
        cf.commander.setpoint_full_state(
            o.ox, o.oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            o.qw0, 0.0, 0.0, o.qz0, 0.0, 0.0, 0.0,
        ).await?;
        if let Ok(pa) = sa.next().await {
            if let Ok(Ok(pb)) = timeout(Duration::from_millis(15), sb.next()).await { last_b = pb.data; }
            if let Ok(Ok(pc)) = timeout(Duration::from_millis(15), sc.next()).await { last_c = pc.data; }
            let ts = (Instant::now() - t0).as_secs_f32();
            rows.push(collect_row(&pa.data, &last_b, &last_c, ts));
            if last_print.elapsed() >= Duration::from_millis(1000) {
                let r = rows.last().unwrap();
                let theta_deg = (omega * ts).to_degrees() % 360.0;
                println!("  [t={:.1}s theta={:.0}deg] z={:.2}m pitch={:.1}deg {:.2}V",
                         ts, theta_deg, r.z, r.pitch, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    cf.param.set("traj.mode",     0u8).await?;
    cf.param.set("traj.z_mode",   0u8).await?;
    cf.param.set("traj.att_mode", 0u8).await?;
    println!("Loop done — returning to hover {:.1}m...", HOVER_HEIGHT);
    hold_and_land(&cf, &o, HOVER_HEIGHT, 3, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let speed_tag = format!("{:.1}", speed).replace('.', "-");
    let csv_path = format!("../Controls/logs/loop_onboard_m{}_s{}x_r{}_{}.csv",
                           mode, speed_tag, n_reps, ts_str);
    let metadata = vec![
        ("run_trajectory".to_string(), "loop".to_string()),
        ("run_mode".to_string(), mode.to_string()),
        ("run_kt".to_string(), format!("{k_t:.6}")),
        ("run_speed".to_string(), format!("{speed:.6}")),
        ("run_reps".to_string(), n_reps.to_string()),
        ("run_periodic".to_string(), "true".to_string()),
        ("run_att_poly".to_string(), (((mode >= 2) && (mode != 3)) as u8).to_string()),
        ("run_lap_time_s".to_string(), format!("{traj_total:.6}")),
        ("run_radius_m".to_string(), format!("{radius:.6}")),
    ];
    write_csv_with_metadata(&rows, &csv_path, &metadata)?;
    println!("Saved {} rows -> {}", rows.len(), csv_path);
    println!("Analyse: ~/.pyenv/versions/flying_robots/bin/python \
              Controls/analyze_flight.py --csv {} --type loop", csv_path);
    Ok(())
}

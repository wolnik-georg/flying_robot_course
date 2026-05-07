//! Mode D Immelmann Turn — ascending half-loop + roll recovery to level heading-reversed.
//!
//! Phase 1: Ascending half-loop in the X-Z plane (θ: 0 → π).
//!   Position: (R·sin θ, 0, h + R·(1−cos θ))  q = Ry(-θ)  [body-z tracks centripetal]
//!   θ=0: upright at bottom (h).  θ=π: inverted at apex (h + 2R).
//! Phase 2: Stationary half-roll at apex  →  upright with heading reversed.
//!   Position fixed at (0, 0, h + 2R).  q: Ry(-π) → Rz(π).
//!
//! Requires:
//!   traj.z_mode = 1  (full 3D z-polynomial; z rises from 0 to 2R through the half-loop)
//!   traj.att_mode = 1  (explicit attitude polynomial — Mode 2 only)
//!
//! Usage:
//!   cargo run --release --bin onboard_immelmann
//!   cargo run --release --bin onboard_immelmann -- --radius 0.4 --tloop 1.2 --troll 0.4
//!
//! --radius : loop radius in metres (default 0.5)
//! --tloop  : half-loop duration in seconds (default 1.5)
//! --troll  : apex roll phase duration in seconds (default 0.35)
//! --reps   : number of repetitions (default 1)
//!
//! The drone hovers at HOVER_HEIGHT, then the trajectory raises it to HOVER_HEIGHT + 2·radius
//! during the half-loop.  After completion it holds at apex height before landing.

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;
use std::f32::consts::PI;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{TrajectoryPlanner, Se3Waypoint, Vec3, Quat};

const RADIUS_DEFAULT: f32 = 0.5;
const T_HALF_DEFAULT: f32 = 1.5;  // half-loop duration [s]
const T_ROLL_DEFAULT: f32 = 0.35; // apex roll phase duration [s]

/// Build Immelmann Se3 waypoints.
///
/// Ascending half-loop (n=4 segments, 45°/step) followed by one roll-recovery segment.
/// All z-values are relative (0 at start = drone hover height).
fn immelmann_se3_waypoints(r: f32, t_half: f32, t_roll: f32) -> (Vec<Se3Waypoint>, Vec<f32>) {
    let n = 4usize; // 4 segments = 5 waypoints, 45° each
    let seg_t = t_half / n as f32;

    let mut wps = Vec::new();
    let mut durs = Vec::new();

    // Phase 1: ascending half-loop (θ: 0 → π)
    for i in 0..=n {
        let theta = PI * i as f32 / n as f32;
        let pos = Vec3::new(r * theta.sin(), 0.0, r * (1.0 - theta.cos()));
        // Ry(-θ): body-z = (-sin θ, 0, cos θ) = centripetal toward loop center
        let q = Quat::new((theta * 0.5).cos(), 0.0, -(theta * 0.5).sin(), 0.0);
        wps.push(Se3Waypoint::new(pos, q));
        if i < n { durs.push(seg_t); }
    }

    // Phase 2: roll recovery at apex (Ry(-π) → Rz(π) = upright, heading reversed)
    let apex = Vec3::new(0.0, 0.0, 2.0 * r);
    wps.push(Se3Waypoint::new(apex, Quat::new(0.0, 0.0, 0.0, 1.0))); // Rz(π)
    durs.push(t_roll);

    (wps, durs)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let radius: f32 = args.iter().position(|a| a == "--radius")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(RADIUS_DEFAULT);
    let t_half: f32 = args.iter().position(|a| a == "--tloop")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(T_HALF_DEFAULT);
    let t_roll: f32 = args.iter().position(|a| a == "--troll")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(T_ROLL_DEFAULT);
    let n_reps: u32 = args.iter().position(|a| a == "--reps")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1);

    if !(0.2..=1.0).contains(&radius) { eprintln!("--radius must be 0.2-1.0"); std::process::exit(1); }
    if !(0.5..=5.0).contains(&t_half) { eprintln!("--tloop must be 0.5-5.0"); std::process::exit(1); }
    if !(0.2..=2.0).contains(&t_roll) { eprintln!("--troll must be 0.2-2.0"); std::process::exit(1); }
    if n_reps == 0 || n_reps > 5      { eprintln!("--reps must be 1-5"); std::process::exit(1); }

    let apex_height = HOVER_HEIGHT + 2.0 * radius;

    let (se3_wps, se3_durs) = immelmann_se3_waypoints(radius, t_half, t_roll);
    let planner = TrajectoryPlanner::se3(&se3_wps, &se3_durs, 0.031, false)
        .expect("Immelmann Se3 QP failed");
    let spline = planner.as_spline();
    let traj_total = spline.total_time;
    let coefs     = serialise_coefs(spline);
    let z_coefs   = serialise_z_coefs(spline);
    let n_segs    = spline.segments.len();
    let att_coefs = serialise_att_coefs(&planner.attitude_poly_coefs());

    println!("Mode D Immelmann Turn");
    println!("  R={:.2}m  T_half={:.2}s  T_roll={:.2}s  reps={}",
             radius, t_half, t_roll, n_reps);
    println!("  Total per rep: {:.2}s  Apex height: {:.2}m", traj_total, apex_height);
    println!("  Uploading {} pos coefs, {} z coefs, {} att coefs",
             coefs.len(), z_coefs.len(), att_coefs.len());

    let link_ctx = LinkContext::new();
    let cf = connect_drone(&link_ctx).await?;
    verify_firmware(&cf).await?;

    let (sa, sb, sc) = setup_log_blocks(&cf).await?;
    kalman_reset(&cf).await?;
    let o = sample_origin(&cf, &sa, &sb, &sc).await?;

    upload_trajectory(&cf, &coefs).await?;
    upload_z_trajectory(&cf, &z_coefs).await?;
    upload_att_trajectory(&cf, &att_coefs).await?;
    cf.param.set("traj.nseg",     n_segs as u8).await?;
    cf.param.set("traj.ox",       o.ox).await?;
    cf.param.set("traj.oy",       o.oy).await?;
    cf.param.set("traj.hz",       HOVER_HEIGHT).await?;
    cf.param.set("traj.dz",       0.0f32).await?;
    cf.param.set("traj.z_mode",   1u8).await?;
    cf.param.set("traj.att_mode", 1u8).await?;

    ramp_to_hover(&cf, &o, &sa, &sb, &sc).await?;
    hover_settle(&cf, &o, &sa, &sb, &sc).await?;

    cf.param.set("traj.mode",  1u8).await?;
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== IMMELMANN x{} ({:.2}s total) R={:.2}m apex={:.2}m ===",
             n_reps, traj_total * n_reps as f32, radius, apex_height);

    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0 = Instant::now();

    while Instant::now() < t0 + traj_end {
        let t = (Instant::now() - t0).as_secs_f32();
        cf.commander.setpoint_full_state(
            o.ox, o.oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            o.qw0, 0.0, 0.0, o.qz0, 0.0, 0.0, 0.0,
        ).await?;
        if let Ok(pa) = sa.next().await {
            if let Ok(Ok(pb)) = timeout(Duration::from_millis(15), sb.next()).await { last_b = pb.data; }
            if let Ok(Ok(pc)) = timeout(Duration::from_millis(15), sc.next()).await { last_c = pc.data; }
            let ts = (Instant::now() - t0).as_secs_f32();
            rows.push(collect_row(&pa.data, &last_b, &last_c, ts));
            if last_print.elapsed() >= Duration::from_secs(1) {
                let r = rows.last().unwrap();
                let phase = if t < t_half { "half-loop" } else { "roll" };
                println!("  [t={:.1}s {}] z={:.2}m pitch={:.1}deg {:.2}V",
                         t, phase, r.z, r.pitch, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    cf.param.set("traj.mode",     0u8).await?;
    cf.param.set("traj.z_mode",   0u8).await?;
    cf.param.set("traj.att_mode", 0u8).await?;
    hold_and_land(&cf, &o, apex_height, 3, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let csv_path = format!("../Controls/logs/immelmann_onboard_r{}_t{}_n{}_{}.csv",
                           (radius * 100.0) as u32,
                           format!("{:.2}", t_half).replace('.', "-"),
                           n_reps, ts_str);
    let metadata = vec![
        ("run_trajectory".to_string(), "immelmann".to_string()),
        ("run_mode".to_string(), "2".to_string()),
        ("run_radius_m".to_string(), format!("{radius:.6}")),
        ("run_t_half_s".to_string(), format!("{t_half:.6}")),
        ("run_t_roll_s".to_string(), format!("{t_roll:.6}")),
        ("run_reps".to_string(), n_reps.to_string()),
        ("run_att_poly".to_string(), "1".to_string()),
        ("run_lap_time_s".to_string(), format!("{traj_total:.6}")),
    ];
    write_csv_with_metadata(&rows, &csv_path, &metadata)?;
    println!("Saved {} rows -> {}", rows.len(), csv_path);
    Ok(())
}

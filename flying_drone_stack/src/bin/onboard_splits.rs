//! Mode D Split-S — stationary half-roll to inverted + descending half-loop.
//!
//! Phase 1: Stationary half-roll at start position (identity → Ry(π) = inverted).
//!   Position fixed at (0, 0, 0) relative to traj.hz = SPLITS_HEIGHT.
//! Phase 2: Descending half-loop in the X-Z plane (ψ: 0 → π).
//!   Position: (R·sin ψ, 0, −R + R·cos ψ)  q = Ry(π + ψ)  [body-z tracks centripetal]
//!   ψ=0: inverted at top.  ψ=π: upright at bottom (z = −2R relative to start).
//!
//! Requires:
//!   traj.z_mode = 1  (full 3D z-polynomial; z falls from 0 to −2R through the descent)
//!   traj.att_mode = 1  (explicit attitude polynomial — Mode 2 only)
//!   traj.hz = SPLITS_HEIGHT = HOVER_HEIGHT + 2·radius
//!
//! The drone hovers at HOVER_HEIGHT, then climbs to SPLITS_HEIGHT before the maneuver.
//! After completion it lands from HOVER_HEIGHT (the trajectory ends at traj.hz − 2R).
//!
//! Usage:
//!   cargo run --release --bin onboard_splits
//!   cargo run --release --bin onboard_splits -- --radius 0.4 --tloop 1.2 --troll 0.4
//!
//! --radius : half-loop radius in metres (default 0.5); SPLITS_HEIGHT = HOVER_HEIGHT + 2·radius
//! --tloop  : descending half-loop duration in seconds (default 1.5)
//! --troll  : initial roll-to-inverted phase duration (default 0.35)
//! --reps   : number of repetitions (default 1; drone returns to SPLITS_HEIGHT between reps)
//!
//! CAUTION: Requires clearance of 2·radius below the starting height.

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;
use std::f32::consts::PI;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{TrajectoryPlanner, Se3Waypoint, Vec3, Quat};

const RADIUS_DEFAULT: f32 = 0.5;
const T_HALF_DEFAULT: f32 = 1.5;  // descending half-loop duration [s]
const T_ROLL_DEFAULT: f32 = 0.35; // initial roll-to-inverted phase duration [s]

/// Build Split-S Se3 waypoints (all z relative to traj.hz = SPLITS_HEIGHT).
///
/// Phase 1: stationary roll (1 segment), z = 0 throughout.
/// Phase 2: descending half-loop (4 segments), z: 0 → −2R.
fn splits_se3_waypoints(r: f32, t_half: f32, t_roll: f32) -> (Vec<Se3Waypoint>, Vec<f32>) {
    let n = 4usize; // 4 segments for half-loop, 45° each
    let seg_t = t_half / n as f32;

    let start = Vec3::new(0.0, 0.0, 0.0);

    // Phase 1: stationary roll to inverted (identity → Ry(π))
    // Ry(π) = [cos(π/2), 0, sin(π/2), 0] = [0, 0, 1, 0]
    let mut wps = vec![
        Se3Waypoint::levelled(start),
        Se3Waypoint::new(start, Quat::new(0.0, 0.0, 1.0, 0.0)), // Ry(π) = inverted
    ];
    let mut durs = vec![t_roll];

    // Phase 2: descending half-loop (ψ: 0 → π)
    // Loop center at (0, 0, −R). Position: (R·sin ψ, 0, −R + R·cos ψ).
    // Quaternion Ry(π + ψ) = [cos((π+ψ)/2), 0, sin((π+ψ)/2), 0]
    //   ψ=0: Ry(π) = [0,0,1,0] = inverted  ✓ (matches Phase 1 end)
    //   ψ=π: Ry(2π) = [-1,0,0,0] ≡ identity = upright  ✓
    // ψ=0 waypoint already in wps as Phase 1 endpoint — add ψ = π·i/n, i=1..=n
    for i in 1..=n {
        let psi = PI * i as f32 / n as f32;
        let pos = Vec3::new(r * psi.sin(), 0.0, -r + r * psi.cos());
        let half_angle = (PI + psi) * 0.5;
        let q = Quat::new(half_angle.cos(), 0.0, half_angle.sin(), 0.0);
        wps.push(Se3Waypoint::new(pos, q));
        durs.push(seg_t);
    }

    (wps, durs)
}

/// Climb to a target z height using full-state setpoints, then pause briefly to settle.
async fn climb_to(
    cf: &crazyflie_lib::Crazyflie,
    o: &Origin,
    target_z: f32,
    sa: &multirotor_simulator::flight_common::LogStream,
    sb: &multirotor_simulator::flight_common::LogStream,
    sc: &multirotor_simulator::flight_common::LogStream,
) -> Result<(), Box<dyn std::error::Error>> {
    let t0 = Instant::now();
    let climb_s = 3.0_f32; // ramp up over 3 seconds
    let settle_s = 1.5_f32;
    let start_z = HOVER_HEIGHT;

    while t0.elapsed().as_secs_f32() < climb_s + settle_s {
        let t = t0.elapsed().as_secs_f32();
        let z = if t < climb_s {
            start_z + (target_z - start_z) * (t / climb_s)
        } else {
            target_z
        };
        cf.commander.setpoint_full_state(
            o.ox, o.oy, z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            o.qw0, 0.0, 0.0, o.qz0, 0.0, 0.0, 0.0,
        ).await?;
        // consume log blocks to avoid buffer overflow
        let _ = timeout(Duration::from_millis(50), sa.next()).await;
        let _ = timeout(Duration::from_millis(15), sb.next()).await;
        let _ = timeout(Duration::from_millis(15), sc.next()).await;
        sleep(Duration::from_millis(20)).await;
    }
    Ok(())
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

    if !(0.2..=0.8).contains(&radius) { eprintln!("--radius must be 0.2-0.8 (height clearance needed)"); std::process::exit(1); }
    if !(0.5..=5.0).contains(&t_half) { eprintln!("--tloop must be 0.5-5.0"); std::process::exit(1); }
    if !(0.2..=2.0).contains(&t_roll) { eprintln!("--troll must be 0.2-2.0"); std::process::exit(1); }
    if n_reps == 0 || n_reps > 5      { eprintln!("--reps must be 1-5"); std::process::exit(1); }

    let splits_height = HOVER_HEIGHT + 2.0 * radius;
    let end_height    = HOVER_HEIGHT; // trajectory ends 2R below splits_height

    let (se3_wps, se3_durs) = splits_se3_waypoints(radius, t_half, t_roll);
    let planner = TrajectoryPlanner::se3(&se3_wps, &se3_durs, 0.031, false)
        .expect("Split-S Se3 QP failed");
    let spline = planner.as_spline();
    let traj_total = spline.total_time;
    let coefs     = serialise_coefs(spline);
    let z_coefs   = serialise_z_coefs(spline);
    let n_segs    = spline.segments.len();
    let att_coefs = serialise_att_coefs(&planner.attitude_poly_coefs());

    println!("Mode D Split-S");
    println!("  R={:.2}m  T_half={:.2}s  T_roll={:.2}s  reps={}",
             radius, t_half, t_roll, n_reps);
    println!("  Total per rep: {:.2}s  Start: {:.2}m  End: {:.2}m",
             traj_total, splits_height, end_height);
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
    cf.param.set("traj.hz",       splits_height).await?;
    cf.param.set("traj.dz",       0.0f32).await?;
    cf.param.set("traj.z_mode",   1u8).await?;
    cf.param.set("traj.att_mode", 1u8).await?;

    ramp_to_hover(&cf, &o, &sa, &sb, &sc).await?;
    hover_settle(&cf, &o, &sa, &sb, &sc).await?;

    println!("Climbing to Split-S start height ({:.2}m)...", splits_height);
    climb_to(&cf, &o, splits_height, &sa, &sb, &sc).await?;

    cf.param.set("traj.mode",  1u8).await?;
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== SPLIT-S x{} ({:.2}s total) R={:.2}m start={:.2}m end={:.2}m ===",
             n_reps, traj_total * n_reps as f32, radius, splits_height, end_height);

    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0 = Instant::now();

    while Instant::now() < t0 + traj_end {
        let t = (Instant::now() - t0).as_secs_f32();
        cf.commander.setpoint_full_state(
            o.ox, o.oy, splits_height, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            o.qw0, 0.0, 0.0, o.qz0, 0.0, 0.0, 0.0,
        ).await?;
        if let Ok(pa) = sa.next().await {
            if let Ok(Ok(pb)) = timeout(Duration::from_millis(15), sb.next()).await { last_b = pb.data; }
            if let Ok(Ok(pc)) = timeout(Duration::from_millis(15), sc.next()).await { last_c = pc.data; }
            let ts = (Instant::now() - t0).as_secs_f32();
            rows.push(collect_row(&pa.data, &last_b, &last_c, ts));
            if last_print.elapsed() >= Duration::from_secs(1) {
                let r = rows.last().unwrap();
                let phase = if t < t_roll { "roll" } else { "descent" };
                println!("  [t={:.1}s {}] z={:.2}m pitch={:.1}deg {:.2}V",
                         t, phase, r.z, r.pitch, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    cf.param.set("traj.mode",     0u8).await?;
    cf.param.set("traj.z_mode",   0u8).await?;
    cf.param.set("traj.att_mode", 0u8).await?;
    hold_and_land(&cf, &o, end_height, 3, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let csv_path = format!("../Controls/logs/splits_onboard_r{}_t{}_n{}_{}.csv",
                           (radius * 100.0) as u32,
                           format!("{:.2}", t_half).replace('.', "-"),
                           n_reps, ts_str);
    let metadata = vec![
        ("run_trajectory".to_string(), "splits".to_string()),
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

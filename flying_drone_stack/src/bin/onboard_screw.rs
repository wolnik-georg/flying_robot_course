//! Screw ascent — vertical climb while spinning in yaw (Mode 2 only).
//!
//! The drone ascends DZ metres while rotating 360° around the world z-axis (yaw spin).
//! Body-z always points straight up — no inversion, no motor cut.
//! Distinct from corkscrew (circular XY helix + body roll) and flip/roll (inversion maneuvers).
//!
//!   Segment 0 → π/2:   yaw 0° → 90°, rising to dz/4.
//!   Segment π/2 → π:   yaw 90° → 180°, rising to dz/2.
//!   Segment π → 3π/2:  yaw 180° → 270°, rising to 3dz/4.
//!   Segment 3π/2 → 2π: yaw 270° → 360°, rising to dz.
//!
//! Quaternion Rz(θ) = [cos(θ/2), 0, 0, sin(θ/2)] — body-z always (0,0,1) in world frame.
//! Thrust ≈ mg throughout — fully flyable on Crazyflie without brushless upgrade.
//!
//! Firmware protocol note: att_mode=1 only uploads roll/pitch polynomials (no yaw channel).
//! The drone will ascend level (correct Z trajectory) but yaw spin is not commanded in hardware.
//! Full yaw spin would require a firmware protocol extension (traj.yaw_poly channel).
//!
//! Requires:
//!   traj.z_mode = 0  (linear Z ramp via traj.dz per rep)
//!   traj.att_mode = 0  (flatness-derived attitude — level flight, yaw=0)
//!
//! Usage:
//!   cargo run --release --bin onboard_screw
//!   cargo run --release --bin onboard_screw -- --dz 0.4 --tscrew 2.5 --reps 2
//!
//! --dz    : altitude gain per rep in metres (default 0.5; drone ends at HOVER_HEIGHT + dz * reps)
//! --tscrew: total duration per rep in seconds (default 2.0)
//! --reps  : number of repetitions 1-5 (default 1)

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;
use std::f32::consts::PI;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{TrajectoryPlanner, Se3Waypoint, Vec3, Quat};

const DZ_DEFAULT: f32 = 0.5;      // altitude gain per rep [m]
const T_SCREW_DEFAULT: f32 = 2.0; // duration per rep [s]

/// Build screw Se3 waypoints: 4 segments × 90° yaw, z: 0 → dz.
///
/// Rz(θ) quaternion: [cos(θ/2), 0, 0, sin(θ/2)]. Body-z = (0,0,1) at all angles.
/// w goes 1→0→-1 monotonically — no sign-flip. Thrust ≈ mg throughout (no motor cut).
fn screw_se3_waypoints(dz: f32, t_screw: f32) -> (Vec<Se3Waypoint>, Vec<f32>) {
    let n = 4usize;
    let seg_t = t_screw / n as f32;
    let wps: Vec<Se3Waypoint> = (0..=n).map(|i| {
        let frac = i as f32 / n as f32;
        let theta = 2.0 * PI * frac;
        let q = Quat::new((theta * 0.5).cos(), 0.0, 0.0, (theta * 0.5).sin()); // Rz(θ)
        Se3Waypoint::new(Vec3::new(0.0, 0.0, dz * frac), q)
    }).collect();
    (wps, vec![seg_t; n])
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let dz: f32 = args.iter().position(|a| a == "--dz")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(DZ_DEFAULT);
    let t_screw: f32 = args.iter().position(|a| a == "--tscrew")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(T_SCREW_DEFAULT);
    let n_reps: u32 = args.iter().position(|a| a == "--reps")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1);

    if !(0.2..=1.0).contains(&dz)      { eprintln!("--dz must be 0.2-1.0"); std::process::exit(1); }
    if !(0.5..=5.0).contains(&t_screw) { eprintln!("--tscrew must be 0.5-5.0"); std::process::exit(1); }
    if n_reps == 0 || n_reps > 5       { eprintln!("--reps must be 1-5"); std::process::exit(1); }

    let total_dz  = dz * n_reps as f32;
    let end_height = HOVER_HEIGHT + total_dz;

    let (se3_wps, se3_durs) = screw_se3_waypoints(dz, t_screw);
    let planner = TrajectoryPlanner::se3(&se3_wps, &se3_durs, 0.031, false)
        .expect("Screw Se3 QP failed");
    let spline    = planner.as_spline();
    let traj_total = spline.total_time;
    let coefs     = serialise_coefs(spline);
    let n_segs    = spline.segments.len();

    println!("Mode D Screw (vertical ascent + yaw spin)");
    println!("  dz={:.2}m/rep  T={:.2}s/rep  reps={}", dz, traj_total, n_reps);
    println!("  z: {:.2}m → {:.2}m  (total {:.2}m ascent)",
             HOVER_HEIGHT, end_height, total_dz);
    println!("  Uploading {} pos coefs", coefs.len());
    println!("  Note: yaw spin planned but not commanded in hardware (no yaw poly channel in firmware).");
    println!("  Drone ascends level; extend firmware protocol for full yaw spin.");

    let link_ctx = LinkContext::new();
    let cf = connect_drone(&link_ctx).await?;
    let mut mocap = start_mocap_udp_task();
    verify_firmware(&cf).await?;

    let (sa, sb, sc) = setup_log_blocks(&cf).await?;
    kalman_reset(&cf).await?;
    let o = sample_origin(&cf, &sa, &sb, &sc).await?;

    upload_trajectory(&cf, &coefs).await?;
    cf.param.set("traj.nseg",     n_segs as u8).await?;
    cf.param.set("traj.ox",       o.ox).await?;
    cf.param.set("traj.oy",       o.oy).await?;
    cf.param.set("traj.hz",       HOVER_HEIGHT).await?;
    cf.param.set("traj.dz",       dz).await?;
    cf.param.set("traj.att_mode", 0u8).await?;

    ramp_to_hover(&cf, &o, &sa, &sb, &sc).await?;
    hover_settle(&cf, &o, &sa, &sb, &sc).await?;

    cf.param.set("traj.mode",  1u8).await?;
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== SCREW x{} ({:.2}s total) dz={:.2}m/rep  start={:.2}m  end={:.2}m ===",
             n_reps, traj_total * n_reps as f32, dz, HOVER_HEIGHT, end_height);

    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0 = Instant::now();

    while Instant::now() < t0 + traj_end {
        let t = (Instant::now() - t0).as_secs_f32();
        forward_mocap(&mut mocap, &cf).await?;
        let frac = t / (traj_total * n_reps as f32);
        let expected_z = HOVER_HEIGHT + frac * total_dz;
        cf.commander.setpoint_full_state(
            o.ox, o.oy, expected_z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            o.qw0, 0.0, 0.0, o.qz0, 0.0, 0.0, 0.0,
        ).await?;
        if let Ok(pa) = sa.next().await {
            if let Ok(Ok(pb)) = timeout(Duration::from_millis(15), sb.next()).await { last_b = pb.data; }
            if let Ok(Ok(pc)) = timeout(Duration::from_millis(15), sc.next()).await { last_c = pc.data; }
            let ts = (Instant::now() - t0).as_secs_f32();
            rows.push(collect_row(&pa.data, &last_b, &last_c, ts));
            if last_print.elapsed() >= Duration::from_secs(1) {
                let r = rows.last().unwrap();
                let rep_frac = (t / traj_total) % 1.0;
                let yaw_deg = rep_frac * 360.0;
                println!("  [t={:.1}s yaw≈{:.0}°] z={:.2}m (target={:.2}m) {:.2}V",
                         t, yaw_deg, r.z, expected_z, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    cf.param.set("traj.mode",     0u8).await?;
    cf.param.set("traj.dz",       0.0f32).await?;
    hold_and_land(&cf, &o, end_height, 3, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let csv_path = format!("../Controls/logs/screw_onboard_dz{}_t{}_n{}_{}.csv",
                           (dz * 100.0) as u32,
                           format!("{:.2}", t_screw).replace('.', "-"),
                           n_reps, ts_str);
    let metadata = vec![
        ("run_trajectory".to_string(), "screw".to_string()),
        ("run_mode".to_string(), "2".to_string()),
        ("run_dz_m".to_string(), format!("{dz:.6}")),
        ("run_t_screw_s".to_string(), format!("{t_screw:.6}")),
        ("run_reps".to_string(), n_reps.to_string()),
        ("run_att_poly".to_string(), "1".to_string()),
        ("run_lap_time_s".to_string(), format!("{traj_total:.6}")),
    ];
    write_csv_with_metadata(&rows, &csv_path, &metadata)?;
    println!("Saved {} rows -> {}", rows.len(), csv_path);
    Ok(())
}

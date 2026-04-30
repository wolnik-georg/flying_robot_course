//! Mode D Onboard Helix — ascending spiral, onboard circle eval + linear Z ramp.
//!
//! XY: circle r=0.30m evaluated onboard at 500 Hz.
//! Z:  linear ramp via traj.dz — firmware uses t_global so multi-rep helices
//!     stack continuously (0 -> n_reps * HELIX_DZ).
//!
//! Usage:
//!   cargo run --release --bin onboard_helix
//!   cargo run --release --bin onboard_helix -- --speed 1.2 --reps 2
//!
//! --speed : trajectory speed multiplier (0.5-3.0, default 1.0)
//! --reps  : number of ascending laps, each adds HELIX_DZ (default 1)

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{SplineTrajectory, Waypoint, Vec3};

const HELIX_RADIUS: f32 = 0.30;
const HELIX_DZ:     f32 = 0.40;
const OMEGA:        f32 = 0.6;

fn helix_trajectory(speed: f32) -> SplineTrajectory {
    let n = 8usize;
    let seg_dur = 2.0 * std::f32::consts::PI / (n as f32 * OMEGA) / speed;
    let waypoints: Vec<Waypoint> = (0..=n).map(|i| {
        let theta = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
        Waypoint { pos: Vec3::new(HELIX_RADIUS * theta.cos(), HELIX_RADIUS * theta.sin(), 0.0), yaw: 0.0 }
    }).collect();
    let durations = vec![seg_dur; n];
    SplineTrajectory::plan(&waypoints, &durations, true)
        .expect("Helix QP planning failed")
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let speed: f32 = args.iter().position(|a| a == "--speed")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let n_reps: u32 = args.iter().position(|a| a == "--reps")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1);

    if speed < 0.5 || speed > 3.0 { eprintln!("--speed must be 0.5-3.0"); std::process::exit(1); }
    if n_reps == 0 || n_reps > 10  { eprintln!("--reps must be 1-10");   std::process::exit(1); }

    let traj = helix_trajectory(speed);
    let traj_total = traj.total_time;
    let coefs = serialise_coefs(&traj);
    let n_segs = traj.segments.len();
    let total_dz = HELIX_DZ * n_reps as f32;
    let top_z    = HOVER_HEIGHT + total_dz;
    println!("Mode D Helix | speed={:.1}x | lap={:.2}s | reps={} | r={}m | dz={}m/lap",
             speed, traj_total, n_reps, HELIX_RADIUS, HELIX_DZ);
    println!("  z: {:.1}m -> {:.1}m over {:.1}s  ({}x{:.1}s)",
             HOVER_HEIGHT, top_z, traj_total * n_reps as f32, n_reps, traj_total);

    let link_ctx = LinkContext::new();
    let cf = connect_drone(&link_ctx).await?;
    verify_firmware(&cf).await?;

    let (sa, sb, sc) = setup_log_blocks(&cf).await?;
    kalman_reset(&cf).await?;
    let o = sample_origin(&cf, &sa, &sb, &sc).await?;

    upload_trajectory(&cf, &coefs).await?;
    cf.param.set("traj.nseg", n_segs as u8).await?;
    cf.param.set("traj.ox",   o.ox - HELIX_RADIUS).await?;
    cf.param.set("traj.oy",   o.oy).await?;
    cf.param.set("traj.hz",   HOVER_HEIGHT).await?;
    cf.param.set("traj.dz",   HELIX_DZ).await?;
    println!("Metadata: z {:.1} -> {:.1}m  ({} laps)", HOVER_HEIGHT, top_z, n_reps);

    ramp_to_hover(&cf, &o, &sa, &sb, &sc).await?;
    hover_settle(&cf, &o, &sa, &sb, &sc).await?;

    cf.param.set("traj.mode",  1u8).await?;
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== Helix ascending: {:.1} -> {:.1}m over {:.1}s ({} laps) ===",
             HOVER_HEIGHT, top_z, traj_total * n_reps as f32, n_reps);

    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0 = Instant::now();

    while Instant::now() < t0 + traj_end {
        let t = (Instant::now() - t0).as_secs_f32();
        let expected_z = HOVER_HEIGHT + (t / (traj_total * n_reps as f32)) * total_dz;
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
                println!("  [t={:.1}s] z={:.2}m (target={:.2}m) {:.2}V",
                         t, r.z, expected_z, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    cf.param.set("traj.mode", 0u8).await?;
    cf.param.set("traj.dz",   0.0f32).await?;
    println!("Helix done -- holding at top ({:.1}m) for 2s...", top_z);
    hold_and_land(&cf, &o, top_z, 2, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let speed_tag = format!("{:.1}", speed).replace('.', "-");
    let csv_path = format!("../Controls/logs/helix_onboard_s{}x_r{}_{}.csv",
                           speed_tag, n_reps, ts_str);
    write_csv(&rows, &csv_path)?;
    println!("Saved {} rows -> {}", rows.len(), csv_path);
    Ok(())
}

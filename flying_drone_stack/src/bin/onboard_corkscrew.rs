//! Mode D Onboard Corkscrew — ascending helix with explicit roll schedule (Mode 2).
//!
//! XY: circle r=0.30m evaluated onboard at 500 Hz.
//! Z:  linear ramp via traj.dz — firmware uses t_global so multi-rep helices
//!     stack continuously (0 -> n_reps * HELIX_DZ).
//!
//! Mode 2 difference from helix: each waypoint has an explicit roll quaternion
//! that increases linearly 0→2π over the lap (one full x-axis roll per lap).
//! At the halfway point (180°) the drone is inverted → thrust crosses zero
//! → motor cut expected on a Crazyflie (insufficient thrust reserve).
//!
//! Usage:
//!   cargo run --release --bin onboard_corkscrew
//!   cargo run --release --bin onboard_corkscrew -- --speed 1.2 --reps 2
//!   cargo run --release --bin onboard_corkscrew -- --mode 1 --kt 0.1
//!   cargo run --release --bin onboard_corkscrew -- --mode 2 --kt 0.3
//!   cargo run --release --bin onboard_corkscrew -- --mode 3 --kt 0.3
//!
//! --speed : trajectory speed multiplier for Mode 0 only (0.5-3.0, default 1.0)
//!           Ignored for Mode 1/2/3 — use --kt to control speed there.
//! --reps  : number of ascending laps, each adds HELIX_DZ (default 1)
//! --mode  : planning mode 0=Spline 1=Richter 2=Se3 3=Paper (default 0)
//! --kt    : aggressiveness for timing (Mode 1/2/3, default 0.1)
//!
//! --mode 2: attitude polynomial uploaded and evaluated onboard at 500 Hz.
//!   Roll increases linearly from 0→2π over one lap (x-axis rotation).
//!   Laptop fits degree-8 roll/pitch polynomials via least-squares,
//!   uploads via traj.aci/acv/acw, firmware evaluates at 500 Hz.
//!
//! Mode 0 baseline: r=0.30m, omega=0.6 rad/s, lap ~10.5s, v_avg ~0.18 m/s (conservative).
//! Mode 1/2 is always faster — Richter v_avg starts at 0.5 m/s minimum.
//! Mode 1/2 --kt reference table (circle r=0.30m, circumference ~1.88m):
//!   --kt 0.05   →  v_avg ~0.84 m/s  ~2.2s/lap   ← gentle start for Mode 1/2
//!   --kt 0.1    →  v_avg ~1.0  m/s  ~1.9s/lap   ← default
//!   --kt 0.3    →  v_avg ~1.32 m/s  ~1.4s/lap   (moderate)
//!   --kt 1.0    →  v_avg ~2.0  m/s  ~0.9s/lap   (fast)

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{TrajectoryPlanner, Se3Waypoint, Waypoint, Vec3, Quat};

const HELIX_RADIUS: f32 = 0.30;
const HELIX_DZ:     f32 = 0.40;
const OMEGA:        f32 = 0.6;
const N_WAYPOINTS:  usize = 8;

fn corkscrew_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let n = N_WAYPOINTS;
    let seg_dur = 2.0 * std::f32::consts::PI / (n as f32 * OMEGA) / speed;
    let waypoints: Vec<Waypoint> = (0..=n).map(|i| {
        let theta = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
        Waypoint {
            pos: Vec3::new(HELIX_RADIUS * theta.cos(), HELIX_RADIUS * theta.sin(), 0.0),
            yaw: 0.0,
        }
    }).collect();
    let durations = vec![seg_dur; n];
    (waypoints, durations)
}

fn build_planner(mode: u8, speed: f32, k_t: f32) -> TrajectoryPlanner {
    let (wps, durs) = corkscrew_waypoints(speed);
    match mode {
        1 => TrajectoryPlanner::richter(&wps, k_t, true)
            .expect("Corkscrew Mode 1 QP failed"),
        2 => {
            // Use Richter timing (k_t) for duration allocation, same as Mode 1.
            let m1 = TrajectoryPlanner::richter(&wps, k_t, true)
                .expect("Corkscrew Mode 2 timing (Richter) failed");
            let kt_durs = m1.segment_durations();
            // Mode 2 policy: explicit roll quaternion at each waypoint.
            // Roll increases linearly from 0 → 2π over the lap (x-axis rotation).
            // At i=n/2 (halfway) the drone is inverted → motor cut on Crazyflie.
            let n = wps.len();
            let se3_wps: Vec<Se3Waypoint> = wps.iter().enumerate()
                .map(|(i, w)| {
                    let frac = i as f32 / (n - 1) as f32;
                    let roll_angle = 2.0 * std::f32::consts::PI * frac;
                    Se3Waypoint::new(
                        w.pos,
                        Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll_angle),
                    )
                })
                .collect();
            TrajectoryPlanner::se3(&se3_wps, &kt_durs, 0.031, false)
                .expect("Corkscrew Mode 2 QP failed")
        }
        3 => TrajectoryPlanner::paper(&wps, k_t, true)
            .expect("Corkscrew Mode 3 paper QP failed"),
        _ => TrajectoryPlanner::spline(&wps, &durs, true)
            .expect("Corkscrew Mode 0 QP failed"),
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let speed: f32 = args.iter().position(|a| a == "--speed")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let n_reps: u32 = args.iter().position(|a| a == "--reps")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1);
    let mode: u8 = args.iter().position(|a| a == "--mode")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(0);
    let k_t: f32 = args.iter().position(|a| a == "--kt")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(0.1);

    if speed < 0.5 || speed > 3.0 { eprintln!("--speed must be 0.5-3.0"); std::process::exit(1); }
    if n_reps == 0 || n_reps > 10  { eprintln!("--reps must be 1-10");   std::process::exit(1); }
    if mode > 3                    { eprintln!("--mode must be 0, 1, 2, or 3"); std::process::exit(1); }

    let planner = build_planner(mode, speed, k_t);
    let spline = planner.as_spline();
    let traj_total = spline.total_time;
    let coefs = serialise_coefs(spline);
    let n_segs = spline.segments.len();
    let att_coefs_raw = planner.attitude_poly_coefs();
    let att_coefs = serialise_att_coefs(&att_coefs_raw);
    let total_dz = HELIX_DZ * n_reps as f32;
    let top_z    = HOVER_HEIGHT + total_dz;
    println!("Mode D Corkscrew | planning_mode={} | speed={:.1}x | lap={:.2}s | reps={} | r={}m | dz={}m/lap",
             mode, speed, traj_total, n_reps, HELIX_RADIUS, HELIX_DZ);
    println!("  z: {:.1}m -> {:.1}m over {:.1}s  ({}x{:.1}s)",
             HOVER_HEIGHT, top_z, traj_total * n_reps as f32, n_reps, traj_total);
    if mode >= 2 && mode != 3 {
        println!("  Mode {}: uploading {} att coefs (x-axis roll schedule 0->2pi per lap)", mode, att_coefs.len());
        println!("  WARNING: at 180deg roll the drone is inverted — motor cut expected on Crazyflie!");
    }

    let link_ctx = LinkContext::new();
    let cf = connect_drone(&link_ctx).await?;
    let mut mocap = start_mocap_udp_task();
    verify_firmware(&cf).await?;

    let (sa, sb, sc) = setup_log_blocks(&cf).await?;
    kalman_reset(&cf).await?;
    let o = sample_origin(&cf, &sa, &sb, &sc).await?;

    upload_trajectory(&cf, &coefs).await?;
    if mode >= 2 && mode != 3 { upload_att_trajectory(&cf, &att_coefs).await?; }
    cf.param.set("traj.nseg",          n_segs as u8).await?;
    cf.param.set("traj.ox",            o.ox - HELIX_RADIUS).await?;
    cf.param.set("traj.oy",            o.oy).await?;
    cf.param.set("traj.hz",            HOVER_HEIGHT).await?;
    cf.param.set("traj.dz",            HELIX_DZ).await?;
    cf.param.set("traj.att_mode",      ((mode >= 2) && (mode != 3)) as u8).await?;
    cf.param.set("traj.att_ctrl_mode", 1u8).await?;
    println!("Metadata: z {:.1} -> {:.1}m  ({} laps)", HOVER_HEIGHT, top_z, n_reps);

    ramp_to_hover(&cf, &o, &sa, &sb, &sc).await?;
    hover_settle(&cf, &o, &sa, &sb, &sc).await?;

    cf.param.set("traj.mode",  1u8).await?;
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== Corkscrew ascending: {:.1} -> {:.1}m over {:.1}s ({} laps) ===",
             HOVER_HEIGHT, top_z, traj_total * n_reps as f32, n_reps);

    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0 = Instant::now();

    while Instant::now() < t0 + traj_end {
        let t = (Instant::now() - t0).as_secs_f32();
        forward_mocap(&mut mocap, &cf).await?;
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

    cf.param.set("traj.mode",          0u8).await?;
    cf.param.set("traj.dz",            0.0f32).await?;
    cf.param.set("traj.att_mode",      0u8).await?;
    cf.param.set("traj.att_ctrl_mode", 1u8).await?;
    println!("Corkscrew done -- holding at top ({:.1}m) for 2s...", top_z);
    hold_and_land(&cf, &o, top_z, 2, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let speed_tag = format!("{:.1}", speed).replace('.', "-");
    let csv_path = format!("../Controls/logs/corkscrew_onboard_m{}_s{}x_r{}_{}.csv",
                           mode, speed_tag, n_reps, ts_str);
    let metadata = vec![
        ("run_trajectory".to_string(), "corkscrew".to_string()),
        ("run_mode".to_string(), mode.to_string()),
        ("run_kt".to_string(), format!("{k_t:.6}")),
        ("run_speed".to_string(), format!("{speed:.6}")),
        ("run_reps".to_string(), n_reps.to_string()),
        ("run_periodic".to_string(), "true".to_string()),
        ("run_att_poly".to_string(), (((mode >= 2) && (mode != 3)) as u8).to_string()),
        ("run_lap_time_s".to_string(), format!("{traj_total:.6}")),
    ];
    write_csv_with_metadata(&rows, &csv_path, &metadata)?;
    println!("Saved {} rows -> {}", rows.len(), csv_path);
    Ok(())
}

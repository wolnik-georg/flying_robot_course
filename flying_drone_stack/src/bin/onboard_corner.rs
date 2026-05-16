//! Mode D Onboard Racing Corner — explicit bank profile in Mode 2.
//!
//! Usage:
//!   cargo run --release --bin onboard_corner
//!   cargo run --release --bin onboard_corner -- --speed 1.2 --reps 3
//!   cargo run --release --bin onboard_corner -- --mode 1 --kt 0.2
//!   cargo run --release --bin onboard_corner -- --mode 2 --kt 0.2
//!   cargo run --release --bin onboard_corner -- --mode 3 --kt 0.2
//!
//! --speed : trajectory speed multiplier for Mode 0 only (0.5-3.0, default 1.0)
//!           Ignored for Mode 1/2 — use --kt to control speed there.
//! --reps  : number of corner repetitions (default 2)
//! --mode  : planning mode 0=Spline 1=Richter 2=Se3 3=Paper (default 0)
//! --kt    : aggressiveness for timing (Mode 1/2, default 0.2)
//!
//! Mode 2 uses explicit attitude waypoints with pre-bank -> apex bank -> unload.

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{TrajectoryPlanner, Se3Waypoint, Waypoint, Vec3, Quat};

fn corner_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let wps = vec![
        Waypoint { pos: Vec3::new(-0.9, -0.2, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.5, -0.2, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.15, -0.10, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.05,  0.10, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.18,  0.42, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.22,  0.78, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.22,  1.10, 0.0), yaw: 0.0 },
    ];
    let durs = [0.55_f32, 0.40, 0.34, 0.34, 0.42, 0.55];
    let durations: Vec<f32> = durs.iter().map(|d| d / speed).collect();
    (wps, durations)
}

fn build_planner(mode: u8, speed: f32, k_t: f32) -> TrajectoryPlanner {
    let (wps, durs) = corner_waypoints(speed);
    match mode {
        1 => TrajectoryPlanner::richter(&wps, k_t, false)
            .expect("Corner Mode 1 QP failed"),
        2 => {
            let m1 = TrajectoryPlanner::richter(&wps, k_t, false)
                .expect("Corner Mode 2 timing (Richter) failed");
            let kt_durs = m1.segment_durations();
            // Explicit bank schedule around corner apex.
            let bank = [0.0_f32, 0.18, 0.38, 0.52, 0.36, 0.16, 0.0];
            let se3_wps: Vec<Se3Waypoint> = wps.iter().zip(bank.iter())
                .map(|(w, &phi)| {
                    let q = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), phi);
                    Se3Waypoint::new(w.pos, q)
                })
                .collect();
            TrajectoryPlanner::se3(&se3_wps, &kt_durs, 0.031, false)
                .expect("Corner Mode 2 QP failed")
        }
        3 => TrajectoryPlanner::paper(&wps, k_t, false)
            .expect("Corner Mode 3 paper QP failed"),
        _ => TrajectoryPlanner::spline(&wps, &durs, false)
            .expect("Corner Mode 0 QP failed"),
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let speed: f32 = args.iter().position(|a| a == "--speed")
        .and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let n_reps: u32 = args.iter().position(|a| a == "--reps")
        .and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(2);
    let mode: u8 = args.iter().position(|a| a == "--mode")
        .and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(0);
    let k_t: f32 = args.iter().position(|a| a == "--kt")
        .and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(0.2);

    if speed < 0.5 || speed > 3.0 { eprintln!("--speed must be 0.5-3.0"); std::process::exit(1); }
    if n_reps == 0 || n_reps > 20  { eprintln!("--reps must be 1-20");   std::process::exit(1); }
    if mode > 3                    { eprintln!("--mode must be 0, 1, 2, or 3"); std::process::exit(1); }

    let planner = build_planner(mode, speed, k_t);
    let spline = planner.as_spline();
    let traj_total = spline.total_time;
    let coefs = serialise_coefs(spline);
    let n_segs = spline.segments.len();
    let att_coefs_raw = planner.attitude_poly_coefs();
    let att_coefs = serialise_att_coefs(&att_coefs_raw);
    println!("Mode D Corner | planning_mode={} | speed={:.1}x | run={:.2}s | reps={}",
             mode, speed, traj_total, n_reps);
    if mode >= 2 && mode != 3 { println!("  Mode {}: uploading {} att coefs", mode, att_coefs.len()); }

    let link_ctx = LinkContext::new();
    let cf = connect_drone(&link_ctx).await?;
    let mut mocap = start_mocap_udp_task();
    verify_firmware(&cf).await?;

    let (sa, sb, sc) = setup_log_blocks(&cf).await?;
    kalman_reset(&cf).await?;
    let o = sample_origin(&cf, &sa, &sb, &sc).await?;

    upload_trajectory(&cf, &coefs).await?;
    if mode >= 2 && mode != 3 { upload_att_trajectory(&cf, &att_coefs).await?; }
    cf.param.set("traj.nseg",     n_segs as u8).await?;
    cf.param.set("traj.ox",       o.ox).await?;
    cf.param.set("traj.oy",       o.oy).await?;
    cf.param.set("traj.hz",       HOVER_HEIGHT).await?;
    cf.param.set("traj.dz",       0.0f32).await?;
    cf.param.set("traj.att_mode", ((mode >= 2) && (mode != 3)) as u8).await?;

    ramp_to_hover(&cf, &o, &sa, &sb, &sc).await?;
    hover_settle(&cf, &o, &sa, &sb, &sc).await?;

    cf.param.set("traj.mode",  1u8).await?;
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== Corner {:.1}x x{} reps ({:.1}s total) ===",
             speed, n_reps, traj_total * n_reps as f32);

    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0 = Instant::now();

    while Instant::now() < t0 + traj_end {
        let t = (Instant::now() - t0).as_secs_f32();
        forward_mocap(&mut mocap, &cf).await?;
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
                println!("  [t={:.1}s rep={:.1}/{}] z={:.2}m roll={:.1}deg {:.2}V",
                         t, t / traj_total, n_reps, r.z, r.roll, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    cf.param.set("traj.mode",     0u8).await?;
    cf.param.set("traj.att_mode", 0u8).await?;
    hold_and_land(&cf, &o, HOVER_HEIGHT, 2, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let speed_tag = format!("{:.1}", speed).replace('.', "-");
    let csv_path = format!("../Controls/logs/corner_onboard_m{}_s{}x_r{}_{}.csv",
                           mode, speed_tag, n_reps, ts_str);
    let metadata = vec![
        ("run_trajectory".to_string(), "corner".to_string()),
        ("run_mode".to_string(), mode.to_string()),
        ("run_kt".to_string(), format!("{k_t:.6}")),
        ("run_speed".to_string(), format!("{speed:.6}")),
        ("run_reps".to_string(), n_reps.to_string()),
        ("run_periodic".to_string(), "false".to_string()),
        ("run_att_poly".to_string(), (((mode >= 2) && (mode != 3)) as u8).to_string()),
        ("run_lap_time_s".to_string(), format!("{traj_total:.6}")),
    ];
    write_csv_with_metadata(&rows, &csv_path, &metadata)?;
    println!("Saved {} rows -> {}", rows.len(), csv_path);
    println!("Analyse: ~/.pyenv/versions/flying_robots/bin/python \
              Controls/analyze_flight.py --csv {} --type corner", csv_path);
    Ok(())
}

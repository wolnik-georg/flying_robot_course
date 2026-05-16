//! Mode D Flip — onboard SE(3) attitude execution with explicit quaternions.
//!
//! This script enforces the Mode 2 policy:
//!   attitude is authored explicitly by waypoint quaternions (SLERP path),
//!   not derived from flatness.
//!
//! Flip profile: level -> inverted (pitch pi) -> level, with fixed position.
//! Position and attitude polynomials are uploaded once; firmware evaluates at 500 Hz.
//!
//! Usage:
//!   cargo run --release --bin onboard_flip
//!   cargo run --release --bin onboard_flip -- --tflip 0.70 --reps 2
//!
//! --mode  : planning mode 2=Se3 (default 2)

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{TrajectoryPlanner, Se3Waypoint, Vec3, Quat};

const FLIP_HEIGHT: f32 = 1.2; // m
const T_FLIP_DEFAULT: f32 = 0.70; // s

fn build_flip_planner(mode: u8, t_flip: f32) -> TrajectoryPlanner {
    let pos = Vec3::new(0.0, 0.0, 0.0);
    let waypoints = vec![
        Se3Waypoint::new(pos, Quat::identity()),
        Se3Waypoint::new(pos, Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), std::f32::consts::PI)),
        Se3Waypoint::new(pos, Quat::identity()),
    ];
    let durations = vec![t_flip * 0.5, t_flip * 0.5];
    match mode {
        2 => TrajectoryPlanner::se3(&waypoints, &durations, 0.031, false)
            .expect("Flip Mode 2 QP failed"),
        _ => unreachable!("onboard_flip only supports mode 2"),
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let t_flip: f32 = args.iter().position(|a| a == "--tflip")
        .and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(T_FLIP_DEFAULT);
    let n_reps: u32 = args.iter().position(|a| a == "--reps")
        .and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(1);
    let mode: u8 = args.iter().position(|a| a == "--mode")
        .and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(2);

    if !(0.30..=2.0).contains(&t_flip) { eprintln!("--tflip must be 0.30-2.0"); std::process::exit(1); }
    if n_reps == 0 || n_reps > 20      { eprintln!("--reps must be 1-20"); std::process::exit(1); }
    if mode != 2 { eprintln!("--mode must be 2"); std::process::exit(1); }

    let planner = build_flip_planner(mode, t_flip);
    let spline = planner.as_spline();
    let traj_total = spline.total_time;
    let coefs = serialise_coefs(spline);
    let n_segs = spline.segments.len();
    let att_coefs_raw = planner.attitude_poly_coefs();
    let att_coefs = serialise_att_coefs(&att_coefs_raw);

    println!("Mode D Flip | planning_mode={} (explicit quaternion attitude)", mode);
    println!("  t_flip={:.2}s  reps={}  segs={}", traj_total, n_reps, n_segs);
    println!("  uploading {} pos coefs, {} att coefs", coefs.len(), att_coefs.len());

    let link_ctx = LinkContext::new();
    let cf = connect_drone(&link_ctx).await?;
    let mut mocap = start_mocap_udp_task();
    verify_firmware(&cf).await?;

    let (sa, sb, sc) = setup_log_blocks(&cf).await?;
    kalman_reset(&cf).await?;
    let o = sample_origin(&cf, &sa, &sb, &sc).await?;

    upload_trajectory(&cf, &coefs).await?;
    upload_att_trajectory(&cf, &att_coefs).await?;
    cf.param.set("traj.nseg", n_segs as u8).await?;
    cf.param.set("traj.ox", o.ox).await?;
    cf.param.set("traj.oy", o.oy).await?;
    cf.param.set("traj.hz", FLIP_HEIGHT).await?;
    cf.param.set("traj.dz", 0.0f32).await?;
    cf.param.set("traj.att_mode", 1u8).await?;

    ramp_to_hover(&cf, &o, &sa, &sb, &sc).await?;
    hover_settle(&cf, &o, &sa, &sb, &sc).await?;

    cf.param.set("traj.mode", 1u8).await?;
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== FLIP x{} ({:.2}s total) at z={:.2}m ===", n_reps, traj_total * n_reps as f32, FLIP_HEIGHT);

    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0 = Instant::now();

    while Instant::now() < t0 + traj_end {
        let t = (Instant::now() - t0).as_secs_f32();
        forward_mocap(&mut mocap, &cf).await?;
        // Keepalive + arming condition while firmware tracks uploaded trajectory.
        cf.commander.setpoint_full_state(
            o.ox, o.oy, FLIP_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            o.qw0, 0.0, 0.0, o.qz0, 0.0, 0.0, 0.0,
        ).await?;
        if let Ok(pa) = sa.next().await {
            if let Ok(Ok(pb)) = timeout(Duration::from_millis(15), sb.next()).await { last_b = pb.data; }
            if let Ok(Ok(pc)) = timeout(Duration::from_millis(15), sc.next()).await { last_c = pc.data; }
            let ts = (Instant::now() - t0).as_secs_f32();
            rows.push(collect_row(&pa.data, &last_b, &last_c, ts));
            if last_print.elapsed() >= Duration::from_secs(1) {
                let r = rows.last().unwrap();
                println!("  [t={:.1}s rep={:.1}/{}] z={:.2}m pitch={:.1}deg {:.2}V",
                         t, t / traj_total, n_reps, r.z, r.pitch, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    cf.param.set("traj.mode", 0u8).await?;
    cf.param.set("traj.att_mode", 0u8).await?;
    hold_and_land(&cf, &o, HOVER_HEIGHT, 2, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let tflip_tag = format!("{:.2}", t_flip).replace('.', "-");
    let csv_path = format!("../Controls/logs/flip_onboard_m{}_t{}_r{}_{}.csv", mode, tflip_tag, n_reps, ts_str);
    let metadata = vec![
        ("run_trajectory".to_string(), "flip".to_string()),
        ("run_mode".to_string(), mode.to_string()),
        ("run_tflip_s".to_string(), format!("{t_flip:.6}")),
        ("run_reps".to_string(), n_reps.to_string()),
        ("run_periodic".to_string(), "false".to_string()),
        ("run_att_poly".to_string(), "1".to_string()),
        ("run_lap_time_s".to_string(), format!("{traj_total:.6}")),
    ];
    write_csv_with_metadata(&rows, &csv_path, &metadata)?;
    println!("Saved {} rows -> {}", rows.len(), csv_path);
    println!("Analyse: ~/.pyenv/versions/flying_robots/bin/python \
              Controls/analyze_flight.py --csv {} --type flip", csv_path);
    Ok(())
}

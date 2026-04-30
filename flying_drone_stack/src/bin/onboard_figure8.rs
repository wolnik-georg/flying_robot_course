//! Mode D Figure-8 — onboard spline evaluation at 500 Hz.
//!
//! The drone hovers at the trajectory origin, then the firmware evaluates
//! the spline locally at 500 Hz with full ω_d feedforward — no radio lag.
//!
//! Usage:
//!   cargo run --release --bin onboard_figure8
//!   cargo run --release --bin onboard_figure8 -- --speed 1.4 --reps 2
//!
//! --speed : trajectory speed multiplier (0.5–3.0, default 1.0)
//! --reps  : number of full figure-8 repetitions (default 1)

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{SplineTrajectory, Waypoint, Vec3};

fn figure8_trajectory(speed: f32) -> SplineTrajectory {
    let waypoints = vec![
        Waypoint { pos: Vec3::new( 0.000000,  0.000000, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.396058, -0.445604, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.922409, -0.291165, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.923174,  0.289869, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.405364,  0.450742, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.000000,  0.000000, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.402804, -0.449354, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.921641, -0.292459, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.923935,  0.288570, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.398611,  0.447039, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.000000,  0.000000, 0.0), yaw: 0.0 },
    ];
    let base_durations = [1.050000_f32, 0.710000, 0.620000, 0.700000, 0.560000,
                          0.560000,     0.700000, 0.620000, 0.710000, 1.053185];
    let durations: Vec<f32> = base_durations.iter().map(|d| d / speed).collect();
    SplineTrajectory::plan(&waypoints, &durations, false)
        .expect("Figure-8 QP planning failed")
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let speed: f32 = args.iter().position(|a| a == "--speed")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let n_reps: u32 = args.iter().position(|a| a == "--reps")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1);

    if speed < 0.5 || speed > 3.0 { eprintln!("--speed must be 0.5-3.0"); std::process::exit(1); }
    if n_reps == 0 || n_reps > 20  { eprintln!("--reps must be 1-20");   std::process::exit(1); }

    let traj = figure8_trajectory(speed);
    let traj_total = traj.total_time;
    let coefs = serialise_coefs(&traj);
    let n_segs = traj.segments.len();
    let peak_ms = 1.34 * speed;
    println!("Mode D Figure-8 | speed={:.1}x | lap={:.2}s | peak~{:.2}m/s | reps={}",
             speed, traj_total, peak_ms, n_reps);
    println!("  accel ~{:.2}x  jerk ~{:.2}x", speed * speed, speed * speed * speed);

    let link_ctx = LinkContext::new();
    let cf = connect_drone(&link_ctx).await?;
    verify_firmware(&cf).await?;

    let (sa, sb, sc) = setup_log_blocks(&cf).await?;
    kalman_reset(&cf).await?;
    let o = sample_origin(&cf, &sa, &sb, &sc).await?;

    upload_trajectory(&cf, &coefs).await?;
    cf.param.set("traj.nseg", n_segs as u8).await?;
    cf.param.set("traj.ox",   o.ox).await?;
    cf.param.set("traj.oy",   o.oy).await?;
    cf.param.set("traj.hz",   HOVER_HEIGHT).await?;
    cf.param.set("traj.dz",   0.0f32).await?;

    ramp_to_hover(&cf, &o, &sa, &sb, &sc).await?;
    hover_settle(&cf, &o, &sa, &sb, &sc).await?;

    cf.param.set("traj.mode",  1u8).await?;
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== Figure-8 {:.1}x x{} ({:.1}s total) ===",
             speed, n_reps, traj_total * n_reps as f32);

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
                println!("  [t={:.1}s rep={:.1}/{}] z={:.2}m {:.2}V",
                         t, t / traj_total, n_reps, r.z, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    cf.param.set("traj.mode", 0u8).await?;
    hold_and_land(&cf, &o, HOVER_HEIGHT, 2, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let speed_tag = format!("{:.1}", speed).replace('.', "-");
    let csv_path = format!("../Controls/logs/figure8_onboard_s{}x_r{}_{}.csv",
                           speed_tag, n_reps, ts_str);
    write_csv(&rows, &csv_path)?;
    println!("Saved {} rows -> {}", rows.len(), csv_path);
    println!("Analyse: ~/.pyenv/versions/flying_robots/bin/python \
              Controls/analyze_flight.py --csv {} --type figure8", csv_path);
    Ok(())
}

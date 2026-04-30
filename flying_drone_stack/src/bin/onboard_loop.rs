//! Mode E -- Vertical Loop: full 360 deg aerobatic loop in the X-Z plane.
//!
//! Inversion condition: R*omega^2 > g  ->  omega > sqrt(g/R)
//! R=0.5m -> omega_min=4.43 rad/s -> speed_min ~2.1x (brushless + mocap required)
//!
//! traj.mode=2 (Mode E): cx -> X,  cy -> Z offset above hover_z,  Y fixed.
//!
//! Usage:
//!   cargo run --release --bin onboard_loop
//!   cargo run --release --bin onboard_loop -- --speed 2.5 --radius 0.5 --reps 3
//!
//! --speed  : trajectory speed multiplier (default 1.0)
//! --radius : loop radius in metres (default 0.5)
//! --reps   : number of complete loops (default 1)

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{SplineTrajectory, Waypoint, Vec3};

const GRAVITY:        f32 = 9.81;
const RADIUS_DEFAULT: f32 = 0.5;
const T_LAP_DEFAULT:  f32 = 3.0;

fn loop_trajectory(radius: f32, t_lap: f32) -> SplineTrajectory {
    let n = 8usize;
    let seg_dur = t_lap / n as f32;
    let waypoints: Vec<Waypoint> = (0..=n).map(|i| {
        let theta = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
        Waypoint {
            pos: Vec3::new(radius * theta.sin(), radius * (1.0 - theta.cos()), 0.0),
            yaw: 0.0,
        }
    }).collect();
    let durations = vec![seg_dur; n];
    SplineTrajectory::plan(&waypoints, &durations, true)
        .expect("Loop QP planning failed")
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

    if n_reps == 0 || n_reps > 20 { eprintln!("--reps must be 1-20"); std::process::exit(1); }

    let t_lap  = T_LAP_DEFAULT / speed;
    let omega  = 2.0 * std::f32::consts::PI / t_lap;
    let omega_min = (GRAVITY / radius).sqrt();
    let speed_for_inversion = omega_min / (2.0 * std::f32::consts::PI / T_LAP_DEFAULT);
    let top_height = HOVER_HEIGHT + 2.0 * radius;

    println!("Vertical Loop | Mode E");
    println!("  R={:.2}m  T={:.2}s/lap  omega={:.2}rad/s  speed={:.1}x  reps={}",
             radius, t_lap, omega, speed, n_reps);
    println!("  Bottom: z={:.1}m  Top: z={:.1}m", HOVER_HEIGHT, top_height);
    println!("  Inversion threshold: speed>={:.1}x (omega>={:.2}rad/s)",
             speed_for_inversion, omega_min);
    if omega >= omega_min {
        println!("  INVERTED FLIGHT -- drone inverts at top (brushless + mocap required)");
    } else {
        println!("  Sub-inversion (upright throughout). To invert: --speed {:.1}", speed_for_inversion);
    }

    let traj = loop_trajectory(radius, t_lap);
    let coefs = serialise_coefs(&traj);
    let n_segs = traj.segments.len();
    println!("Planned: {} segs  {} coefs  lap={:.2}s", n_segs, coefs.len(), traj.total_time);

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

    cf.param.set("traj.mode",  2u8).await?;
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== LOOP! R={:.2}m  {:.1}s/lap  reps={}  top={:.1}m ===",
             radius, traj.total_time, n_reps, top_height);
    if omega >= omega_min { println!("    (inverted flight at top)"); }

    let traj_end = Duration::from_secs_f32(traj.total_time * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0 = Instant::now();

    while Instant::now() < t0 + traj_end {
        let t = (Instant::now() - t0).as_secs_f32();
        let expected_z = HOVER_HEIGHT + radius * (1.0 - (omega * t).cos());
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
                let theta_deg = omega * ts * 180.0 / std::f32::consts::PI;
                println!("  [t={:.1}s theta={:.0}deg] z={:.2}m pitch={:.1}deg {:.2}V",
                         ts, theta_deg, r.z, r.pitch, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    cf.param.set("traj.mode", 0u8).await?;
    println!("Loop done -- returning to hover {:.1}m...", HOVER_HEIGHT);
    hold_and_land(&cf, &o, HOVER_HEIGHT, 3, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let speed_tag = format!("{:.1}", speed).replace('.', "-");
    let csv_path = format!("../Controls/logs/loop_onboard_s{}x_r{}_{}.csv",
                           speed_tag, n_reps, ts_str);
    write_csv(&rows, &csv_path)?;
    println!("Saved {} rows -> {}", rows.len(), csv_path);
    println!("Analyse: ~/.pyenv/versions/flying_robots/bin/python \
              Controls/analyze_flight.py --csv {} --type loop", csv_path);
    Ok(())
}

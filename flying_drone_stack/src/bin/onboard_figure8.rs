//! Mode D Figure-8 — onboard spline evaluation at 500 Hz.
//!
//! The drone hovers at the trajectory origin, then the firmware evaluates
//! the spline locally at 500 Hz with full ω_d feedforward — no radio lag.
//!
//! Usage:
//!   cargo run --release --bin onboard_figure8
//!   cargo run --release --bin onboard_figure8 -- --speed 1.4 --reps 2
//!   cargo run --release --bin onboard_figure8 -- --mode 1 --kt 0.008
//!   cargo run --release --bin onboard_figure8 -- --mode 2 --kt 0.05
//!
//! --speed : trajectory speed multiplier for Mode 0 only (0.5–3.0, default 1.0)
//!           Ignored for Mode 1/2 — use --kt to control speed there.
//! --reps  : number of full figure-8 repetitions (default 1)
//! --mode  : planning mode 0=Spline 1=Richter 2=Se3 (default 0)
//! --kt    : Richter aggressiveness for timing (Mode 1 and 2, default 0.008)
//!
//! --mode 2: attitude polynomial uploaded and evaluated onboard at 500 Hz. Laptop fits degree-8
//! roll/pitch polynomials via least-squares, uploads via traj.aci/acv/acw, firmware evaluates

//!
//! Mode 0 baseline: professor's hand-tuned durations, lap ~7.28s, v_avg ~0.69 m/s.
//! Mode 1/2 --kt reference table (figure-8 path ~5 m):
//!   --kt 0.008  →  v_avg ~0.63 m/s  ~7.9s/lap   ← closest to Mode 0 baseline
//!   --kt 0.05   →  v_avg ~0.84 m/s  ~5.9s/lap   (moderate)
//!   --kt 0.2    →  v_avg ~1.17 m/s  ~4.3s/lap   (fast)
//!   --kt 1.0    →  v_avg ~2.0  m/s  ~2.5s/lap   (aggressive)

use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;

use multirotor_simulator::flight_common::*;
use multirotor_simulator::prelude::{TrajectoryPlanner, Se3Waypoint, Waypoint, Vec3};

const BASE_DURATIONS: [f32; 10] = [
    1.050000, 0.710000, 0.620000, 0.700000, 0.560000,
    0.560000, 0.700000, 0.620000, 0.710000, 1.050000,
];

fn figure8_waypoints() -> Vec<Waypoint> {
    vec![
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
    ]
}

fn build_planner(mode: u8, speed: f32, k_t: f32) -> TrajectoryPlanner {
    let wps = figure8_waypoints();
    let durs: Vec<f32> = BASE_DURATIONS.iter().map(|d| d / speed).collect();
    match mode {
        1 => TrajectoryPlanner::richter(&wps, k_t, false)
            .expect("Figure-8 Mode 1 QP failed"),
        2 => {
            // Use Richter timing (k_t) for duration allocation, same as Mode 1.
            let m1 = TrajectoryPlanner::richter(&wps, k_t, false)
                .expect("Figure-8 Mode 2 timing (Richter) failed");
            let kt_durs = m1.segment_durations();
            let se3_wps: Vec<Se3Waypoint> = wps.iter()
                .map(|w| Se3Waypoint::flat(w.pos, w.yaw))
                .collect();
            TrajectoryPlanner::se3(&se3_wps, &kt_durs, 0.031, false)
                .expect("Figure-8 Mode 2 QP failed")
        }
        _ => TrajectoryPlanner::spline(&wps, &durs, false)
            .expect("Figure-8 Mode 0 QP failed"),
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
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(0.008);

    if speed < 0.5 || speed > 3.0 { eprintln!("--speed must be 0.5-3.0"); std::process::exit(1); }
    if n_reps == 0 || n_reps > 20  { eprintln!("--reps must be 1-20");   std::process::exit(1); }
    if mode > 2                    { eprintln!("--mode must be 0, 1, or 2"); std::process::exit(1); }

    let planner = build_planner(mode, speed, k_t);
    let spline = planner.as_spline();
    let traj_total = spline.total_time;
    let coefs = serialise_coefs(spline);
    let n_segs = spline.segments.len();
    let att_coefs_raw = planner.attitude_poly_coefs();
    let att_coefs = serialise_att_coefs(&att_coefs_raw);
    let peak_ms = 1.34 * speed;
    println!("Mode D Figure-8 | planning_mode={} | speed={:.1}x | lap={:.2}s | peak~{:.2}m/s | reps={}",
             mode, speed, traj_total, peak_ms, n_reps);
    println!("  accel ~{:.2}x  jerk ~{:.2}x", speed * speed, speed * speed * speed);
    if mode == 2 { println!("  Mode 2: uploading {} att coefs", att_coefs.len()); }

    let link_ctx = LinkContext::new();
    let cf = connect_drone(&link_ctx).await?;
    verify_firmware(&cf).await?;

    let (sa, sb, sc) = setup_log_blocks(&cf).await?;
    kalman_reset(&cf).await?;
    let o = sample_origin(&cf, &sa, &sb, &sc).await?;

    upload_trajectory(&cf, &coefs).await?;
    if mode == 2 { upload_att_trajectory(&cf, &att_coefs).await?; }
    cf.param.set("traj.nseg",     n_segs as u8).await?;
    cf.param.set("traj.ox",       o.ox).await?;
    cf.param.set("traj.oy",       o.oy).await?;
    cf.param.set("traj.hz",       HOVER_HEIGHT).await?;
    cf.param.set("traj.dz",       0.0f32).await?;
    cf.param.set("traj.att_mode", (mode == 2) as u8).await?;

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

    cf.param.set("traj.mode",     0u8).await?;
    cf.param.set("traj.att_mode", 0u8).await?;
    hold_and_land(&cf, &o, HOVER_HEIGHT, 2, &sa, &sb, &sc).await?;

    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let speed_tag = format!("{:.1}", speed).replace('.', "-");
    let csv_path = format!("../Controls/logs/figure8_onboard_m{}_s{}x_r{}_{}.csv",
                           mode, speed_tag, n_reps, ts_str);
    write_csv(&rows, &csv_path)?;
    println!("Saved {} rows -> {}", rows.len(), csv_path);
    println!("Analyse: ~/.pyenv/versions/flying_robots/bin/python \
              Controls/analyze_flight.py --csv {} --type figure8", csv_path);
    Ok(())
}

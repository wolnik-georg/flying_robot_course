//! Mode B Figure-8 — our min-snap QP spline (degree-8) + full-state feedforward at ~20 Hz
//!
//! Why this beats Mode C (Python HLC Poly4D):
//!   1. Trajectory evaluated at full degree-8 precision — Mode C exports a truncated
//!      degree-7 Poly4D, losing the highest-frequency term.
//!   2. Full-state setpoint: pos+vel+acc+quat+omega_d → firmware uses omega_d feedforward
//!      so e_omega = omega - omega_d instead of e_omega = omega. This directly cancels the
//!      attitude lag that was the bottleneck in Block I analysis.
//!
//! Trajectory: professor's figure-8 waypoints re-solved as our QP min-snap spline.
//! Same as export_figure8_match: 10 segments, 7.28s rest-to-rest, ±0.92m × ±0.45m.
//!
//! Logs to ../Controls/logs/figure8_<YYYYMMDD_HHMMSS>.csv — same 18-column format as
//! Python FlightLogger so analyze_flight.py works unchanged on both Mode B and Mode C data.
//!
//! Usage:
//!   cargo run --release --bin spline_figure8_test
//!   cargo run --release --bin spline_figure8_test -- --reps 3
//!
//! Requires OOT firmware: cd firmware_app && make cload

use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod};
use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::fs;
use std::io::{BufWriter, Write};
use std::time::{Duration, Instant};
use tokio::time::{sleep, timeout};
use chrono::Local;

use multirotor_simulator::prelude::{SplineTrajectory, Waypoint, Vec3};
use multirotor_simulator::planning::{compute_flatness, rot_to_quat, FlatOutput};

const CF_URI:       &str = "radio://0/80/2M/E7E7E7E7E7";
const HOVER_HEIGHT: f32  = 1.0;   // m — matches Python run_figure8.py
const MASS:         f32  = 0.031; // kg — CF2.1 + Flow Deck
const LOG_MS:       u64  = 50;    // 20 Hz — matches Python FlightLogger period

/// Our QP min-snap spline through the professor's figure-8 waypoints.
/// Same spatial path as the Mode C Poly4D but evaluated at full degree-8 in Rust.
fn figure8_trajectory() -> SplineTrajectory {
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
    let durations = vec![1.050000_f32, 0.710000, 0.620000, 0.700000, 0.560000,
                         0.560000, 0.700000, 0.620000, 0.710000, 1.053185];
    SplineTrajectory::plan(&waypoints, &durations, true)
        .expect("Figure-8 QP planning failed")
}

struct Row {
    time_s: f32, x: f32, y: f32, z: f32,
    vx: f32, vy: f32, vz: f32,
    roll: f32, pitch: f32, yaw: f32,
    thrust: f32, vbat: f32,
    gyro_x: f32, gyro_y: f32, gyro_z: f32,
    acc_x: f32, acc_y: f32, acc_z: f32,
}

fn gf(map: &HashMap<String, Value>, k: &str) -> f32 {
    map.get(k).map(|v| v.to_f64_lossy() as f32).unwrap_or(f32::NAN)
}

async fn add_var(block: &mut LogBlock, name: &str) {
    if let Err(e) = block.add_variable(name).await {
        eprintln!("  [log] MISSING: {name} ({e})");
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let n_reps: u32 = args.iter()
        .position(|a| a == "--reps")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(2);

    let traj = figure8_trajectory();
    let traj_total = traj.total_time;
    println!("Mode B Figure-8 | reps={n_reps} | traj={traj_total:.2}s/rep | hover={HOVER_HEIGHT}m");
    println!("Requires OOT firmware: cd firmware_app && make cload");

    let link_context = LinkContext::new();
    println!("Connecting to {CF_URI} ...");
    let cf = Crazyflie::connect_from_uri(&link_context, CF_URI, NoTocCache).await?;
    println!("Connected.");

    let _ = cf.param.set("stabilizer.controller", 6u8).await;
    sleep(Duration::from_millis(100)).await;

    // 3 log blocks — 18 variables matching Python FlightLogger columns exactly
    let mut block_a = cf.log.create_block().await?;
    for v in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw"] {
        add_var(&mut block_a, v).await;
    }
    let stream_a = block_a.start(LogPeriod::from_millis(LOG_MS)?).await?;

    let mut block_b = cf.log.create_block().await?;
    for v in ["stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz",
              "stabilizer.thrust", "pm.vbat"] {
        add_var(&mut block_b, v).await;
    }
    let stream_b = block_b.start(LogPeriod::from_millis(LOG_MS)?).await?;

    let mut block_c = cf.log.create_block().await?;
    for v in ["gyro.x", "gyro.y", "gyro.z", "acc.x", "acc.y", "acc.z"] {
        add_var(&mut block_c, v).await;
    }
    let stream_c = block_c.start(LogPeriod::from_millis(LOG_MS)?).await?;

    // Kalman reset
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    let _ = cf.param.set("kalman.resetEstimation", 1u8).await;
    sleep(Duration::from_millis(200)).await;
    let _ = cf.param.set("kalman.resetEstimation", 0u8).await;
    println!("Kalman reset — place drone flat on pad. Waiting 4s...");
    for _ in 0..40 {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        sleep(Duration::from_millis(100)).await;
    }

    // Sample EKF origin (~2s)
    let (mut xs, mut ys, mut yaws) = (Vec::new(), Vec::new(), Vec::new());
    println!("Sampling EKF origin...");
    for _ in 0..40 {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        if let Ok(p) = stream_a.next().await {
            xs.push(gf(&p.data, "stateEstimate.x"));
            ys.push(gf(&p.data, "stateEstimate.y"));
            yaws.push(gf(&p.data, "stabilizer.yaw"));
        }
        let _ = stream_b.next().await;
        let _ = stream_c.next().await;
        sleep(Duration::from_millis(50)).await;
    }
    let n = xs.len() as f32;
    let ox = xs.iter().sum::<f32>() / n;
    let oy = ys.iter().sum::<f32>() / n;
    let oyaw_deg = yaws.iter().sum::<f32>() / n;
    let oyaw_rad = oyaw_deg * std::f32::consts::PI / 180.0;
    let half_yaw = oyaw_rad * 0.5;
    let (qw0, qz0) = (half_yaw.cos(), half_yaw.sin());
    println!("Origin: x={ox:+.3} y={oy:+.3} yaw={oyaw_deg:+.1}°");

    // Ramp to HOVER_HEIGHT over ~5s (50 steps × ~100ms each)
    println!("Ramp to {HOVER_HEIGHT:.1}m...");
    for step in 0..50usize {
        let h = 0.05 + step as f32 * (HOVER_HEIGHT - 0.05) / 49.0;
        cf.commander.setpoint_full_state(
            ox, oy, h, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw0, 0.0, 0.0, qz0, 0.0, 0.0, 0.0,
        ).await?;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
        let _ = stream_c.next().await;
        sleep(Duration::from_millis(50)).await;
    }

    // Hover settle 5s
    println!("Hover settle 5s...");
    let settle = Instant::now() + Duration::from_secs(5);
    while Instant::now() < settle {
        cf.commander.setpoint_full_state(
            ox, oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw0, 0.0, 0.0, qz0, 0.0, 0.0, 0.0,
        ).await?;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
        let _ = stream_c.next().await;
    }

    // ── Figure-8 trajectory ───────────────────────────────────────────────────
    println!("=== Figure-8 x{n_reps} ({:.1}s total) ===", traj_total * n_reps as f32);
    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let traj_start = Instant::now();

    while Instant::now() < traj_start + traj_end {
        let t = (Instant::now() - traj_start).as_secs_f32();
        let flat = traj.eval(t % traj_total);

        // Offset trajectory to world frame; z constant at hover height
        let flat_out = FlatOutput {
            pos:     Vec3::new(ox + flat.pos.x, oy + flat.pos.y, HOVER_HEIGHT),
            vel:     Vec3::new(flat.vel.x, flat.vel.y, 0.0),
            acc:     Vec3::new(flat.acc.x, flat.acc.y, 0.0),
            jerk:    Vec3::new(flat.jerk.x, flat.jerk.y, 0.0),
            snap:    Vec3::new(flat.snap.x, flat.snap.y, 0.0),
            yaw: 0.0, yaw_dot: 0.0, yaw_ddot: 0.0,
        };
        let res = compute_flatness(&flat_out, MASS);
        let q = rot_to_quat(&res.rot);

        // Full-state setpoint: pos+vel+acc+quat+omega_d (the key Mode B advantage)
        cf.commander.setpoint_full_state(
            flat_out.pos.x, flat_out.pos.y, flat_out.pos.z,
            res.vel.x,      res.vel.y,      res.vel.z,
            flat_out.acc.x, flat_out.acc.y, flat_out.acc.z,
            q[0], q[1], q[2], q[3],
            res.omega.x, res.omega.y, res.omega.z,
        ).await?;

        // stream_a drives the row rate (~20 Hz); B and C polled with short timeout
        // so the setpoint loop is never stalled by a late log packet
        if let Ok(pa) = stream_a.next().await {
            if let Ok(Ok(pb)) = timeout(Duration::from_millis(15), stream_b.next()).await {
                last_b = pb.data;
            }
            if let Ok(Ok(pc)) = timeout(Duration::from_millis(15), stream_c.next()).await {
                last_c = pc.data;
            }
            let ts = (Instant::now() - traj_start).as_secs_f32();
            rows.push(Row {
                time_s: ts,
                x:      gf(&pa.data, "stateEstimate.x"),
                y:      gf(&pa.data, "stateEstimate.y"),
                z:      gf(&pa.data, "stateEstimate.z"),
                vx:     gf(&last_b,  "stateEstimate.vx"),
                vy:     gf(&last_b,  "stateEstimate.vy"),
                vz:     gf(&last_b,  "stateEstimate.vz"),
                roll:   gf(&pa.data, "stabilizer.roll"),
                pitch:  gf(&pa.data, "stabilizer.pitch"),
                yaw:    gf(&pa.data, "stabilizer.yaw"),
                thrust: gf(&last_b,  "stabilizer.thrust"),
                vbat:   gf(&last_b,  "pm.vbat"),
                gyro_x: gf(&last_c,  "gyro.x"),
                gyro_y: gf(&last_c,  "gyro.y"),
                gyro_z: gf(&last_c,  "gyro.z"),
                acc_x:  gf(&last_c,  "acc.x"),
                acc_y:  gf(&last_c,  "acc.y"),
                acc_z:  gf(&last_c,  "acc.z"),
            });

            if last_print.elapsed() >= Duration::from_secs(1) {
                let r = rows.last().unwrap();
                let ref_x = ox + flat.pos.x;
                let ref_y = oy + flat.pos.y;
                let err_xy = ((r.x - ref_x).powi(2) + (r.y - ref_y).powi(2)).sqrt();
                println!("  [t={t:.1}s rep={:.1}/{n_reps}] z={:.2}m xy_err={:.3}m {:.2}V",
                    t / traj_total, r.z, err_xy, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    // Hold at center before landing
    println!("Hold 2s...");
    let hold = Instant::now() + Duration::from_secs(2);
    while Instant::now() < hold {
        cf.commander.setpoint_full_state(
            ox, oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw0, 0.0, 0.0, qz0, 0.0, 0.0, 0.0,
        ).await?;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
        let _ = stream_c.next().await;
    }

    // Land
    println!("Landing...");
    for step in (0..=40usize).rev() {
        let h = (step as f32 * HOVER_HEIGHT / 40.0).max(0.05);
        cf.commander.setpoint_full_state(
            ox, oy, h, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw0, 0.0, 0.0, qz0, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(100)).await;
    }
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    sleep(Duration::from_millis(200)).await;

    // ── Write CSV ─────────────────────────────────────────────────────────────
    fs::create_dir_all("../Controls/logs")?;
    let ts = Local::now().format("%Y%m%d_%H%M%S");
    let csv_path = format!("../Controls/logs/figure8_{ts}.csv");
    let file = fs::File::create(&csv_path)?;
    let mut w = BufWriter::new(file);
    writeln!(w, "time_s,x,y,z,vx,vy,vz,roll_deg,pitch_deg,yaw_deg,thrust,vbat,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z")?;
    for r in &rows {
        writeln!(w, "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            r.time_s, r.x, r.y, r.z, r.vx, r.vy, r.vz,
            r.roll, r.pitch, r.yaw, r.thrust, r.vbat,
            r.gyro_x, r.gyro_y, r.gyro_z, r.acc_x, r.acc_y, r.acc_z)?;
    }
    println!("Saved {} rows → {csv_path}", rows.len());
    println!("Done.");
    Ok(())
}

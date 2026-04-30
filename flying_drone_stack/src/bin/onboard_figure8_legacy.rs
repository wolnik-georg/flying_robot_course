//! Mode D Figure-8 — onboard spline evaluation with ω_d feedforward at 500 Hz
//!
//! Architecture:
//!   Laptop  → uploads 190 spline coefficients once before flight (≈3 s)
//!           → sets traj.mode=1, traj.start=1
//!           → sends hover keepalive setpoints during manoeuvre (safety watchdog)
//!   Firmware → evaluates degree-8 spline locally at 500 Hz (every 2 ms tick)
//!           → computes ω_d from jerk via flatness, uses e_ω = ω − ω_d
//!           → NO radio setpoint lag during trajectory
//!
//! Why this beats Mode B (spline_figure8_test.rs):
//!   Mode B: trajectory evaluated on laptop at 20 Hz, sent over radio → 25 ms stale
//!           reference + jitter.  ω_d was computed but arrived too late to help.
//!   Mode D: reference fresh every 2 ms; ω_d computed in the same controller tick
//!           that uses it → zero stale lag, proper angular velocity feedforward.
//!
//! Expected improvement: XY RMSE 5–7 cm (vs ~10 cm for Mode B/C), pitch error 5–7°
//! (vs ~11°), tighter corners on the direction reversals.
//!
//! Backwards compatibility: traj.mode defaults to 0, so this binary must be run
//! explicitly; spline_figure8_test.rs (Mode B) is completely unchanged.
//!
//! Coefficient upload protocol:
//!   for each coef (0..189):
//!     set traj.ci = index
//!     set traj.cv = value
//!     set traj.cw = 1        (commit)
//!     poll traj.cw == 0      (firmware clears within 2 ms)
//!
//! After flight the CSV is saved to ../Controls/logs/figure8_onboard_<timestamp>.csv
//! in the same 18-column format as Mode B/C so analyze_flight.py works unchanged.
//!
//! Usage:
//!   cargo run --release --bin onboard_figure8
//!   cargo run --release --bin onboard_figure8 -- --reps 2
//!
//! Requires firmware with traj_iface.c: cd firmware_app && make cload

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

const CF_URI:       &str = "radio://0/80/2M/E7E7E7E7E7";
const HOVER_HEIGHT: f32  = 1.0;   // m — must match traj.hz uploaded to firmware
const MASS:         f32  = 0.031; // kg — CF2.1 + Flow Deck
const LOG_MS:       u64  = 50;    // 20 Hz logging (same as Mode B/C for fair comparison)

/// Same figure-8 trajectory as Mode B — professor's waypoints re-solved as QP min-snap.
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
    SplineTrajectory::plan(&waypoints, &durations, false)
        .expect("Figure-8 QP planning failed")
}

/// Serialise trajectory segments into the firmware coefficient buffer layout.
///
/// Each segment → [duration, cx0..cx8, cy0..cy8] (19 f32 values).
fn serialise_coefs(traj: &SplineTrajectory) -> Vec<f32> {
    let mut out = Vec::with_capacity(traj.segments.len() * 19);
    for seg in &traj.segments {
        out.push(seg.duration);
        out.extend_from_slice(&seg.cx);
        out.extend_from_slice(&seg.cy);
    }
    out
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

/// Upload a single f32 coefficient at `index` using the ci/cv/cw protocol.
///
/// No polling needed: radio RTT (~5 ms) >> firmware processing time (2 ms at 500 Hz).
/// By the time the next param write reaches the drone, the previous cw has been cleared.
async fn upload_coef(cf: &Crazyflie, index: u8, value: f32) -> Result<(), Box<dyn std::error::Error>> {
    cf.param.set("traj.ci", index).await?;
    cf.param.set("traj.cv", value).await?;
    cf.param.set("traj.cw", 1u8).await?;
    // Small gap so radio doesn't saturate; firmware clears cw within 2 ms.
    sleep(Duration::from_millis(8)).await;
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let n_reps: u32 = args.iter()
        .position(|a| a == "--reps")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(2);

    // ── Plan trajectory ───────────────────────────────────────────────────
    let traj = figure8_trajectory();
    let traj_total = traj.total_time;
    let coefs = serialise_coefs(&traj);
    let n_segs = traj.segments.len();
    println!("Mode D Onboard Figure-8 | reps={n_reps} | traj={traj_total:.2}s/rep | \
              {n_segs} segs | {} coefs", coefs.len());
    println!("Requires updated OOT firmware: cd firmware_app && make cload");

    // ── Connect ───────────────────────────────────────────────────────────
    let link_context = LinkContext::new();
    println!("Connecting to {CF_URI} ...");
    let cf = Crazyflie::connect_from_uri(&link_context, CF_URI, NoTocCache).await?;
    println!("Connected.");

    // Select geometric controller (mode 6)
    let _ = cf.param.set("stabilizer.controller", 6u8).await;
    sleep(Duration::from_millis(100)).await;

    // ── Firmware check: verify traj params exist ──────────────────────────
    // If traj.mode is missing the firmware hasn't been updated. Fail fast here
    // rather than silently uploading to /dev/null and not flying.
    cf.param.set("traj.mode", 0u8).await
        .map_err(|e| format!(
            "traj.mode param not found ({e}). Flash updated firmware first:\n  \
             cd firmware_app && make cload"
        ))?;
    println!("Firmware OK — traj params present.");

    // ── Log blocks (identical to Mode B for fair comparison) ──────────────
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

    // ── Kalman reset ──────────────────────────────────────────────────────
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    let _ = cf.param.set("kalman.resetEstimation", 1u8).await;
    sleep(Duration::from_millis(200)).await;
    let _ = cf.param.set("kalman.resetEstimation", 0u8).await;
    println!("Kalman reset — place drone flat on pad. Waiting 4 s...");
    for _ in 0..40 {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        sleep(Duration::from_millis(100)).await;
    }

    // ── Sample EKF origin (~2 s) ──────────────────────────────────────────
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

    // ── Upload trajectory coefficients ────────────────────────────────────
    // 190 coefficients (10 segs × 19 floats) at ~15 ms per coef ≈ 3 s total.
    println!("Uploading {} coefficients ({} segments)...", coefs.len(), n_segs);
    let upload_start = Instant::now();
    for (idx, &val) in coefs.iter().enumerate() {
        upload_coef(&cf, idx as u8, val).await?;
        if idx % 20 == 19 {
            println!("  [{}/{}]", idx + 1, coefs.len());
        }
    }
    println!("Upload complete in {:.1} s", upload_start.elapsed().as_secs_f32());

    // Upload metadata: number of segments, EKF origin, hover height
    cf.param.set("traj.nseg",  n_segs as u8).await?;
    cf.param.set("traj.ox",    ox).await?;
    cf.param.set("traj.oy",    oy).await?;
    cf.param.set("traj.hz",    HOVER_HEIGHT).await?;
    println!("Metadata uploaded: nseg={n_segs} ox={ox:.3} oy={oy:.3} hz={HOVER_HEIGHT}");

    // ── Ramp to hover (~5 s) ──────────────────────────────────────────────
    println!("Ramp to {HOVER_HEIGHT:.1} m ...");
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

    // ── Hover settle (5 s) ────────────────────────────────────────────────
    println!("Hover settle 5 s...");
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

    // ── Switch to Mode D and trigger onboard trajectory ───────────────────
    // Set mode=1 FIRST so the firmware is ready, then set start=1 to latch T0.
    cf.param.set("traj.mode",  1u8).await?;
    sleep(Duration::from_millis(50)).await; // one controller tick safety margin
    cf.param.set("traj.start", 1u8).await?;
    println!("=== Mode D onboard Figure-8 ×{n_reps} ({:.1} s total) ===",
             traj_total * n_reps as f32);

    // ── Trajectory logging ────────────────────────────────────────────────
    // The firmware drives the drone autonomously.  We send a hover keepalive
    // setpoint every ~50 ms so the CF setpoint watchdog doesn't trigger a landing.
    // We also log telemetry at 20 Hz for post-flight analysis.
    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String, Value> = HashMap::new();
    let mut last_c: HashMap<String, Value> = HashMap::new();
    let mut last_print = Instant::now();
    let traj_t0_wall = Instant::now();

    while Instant::now() < traj_t0_wall + traj_end {
        let t = (Instant::now() - traj_t0_wall).as_secs_f32();

        // Keepalive: hover setpoint so the CF watchdog stays satisfied.
        // The firmware ignores this when traj.mode=1 but needs it for arming
        // (sp.position.z > 0.05 check) and the CRTP watchdog.
        cf.commander.setpoint_full_state(
            ox, oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw0, 0.0, 0.0, qz0, 0.0, 0.0, 0.0,
        ).await?;

        if let Ok(pa) = stream_a.next().await {
            if let Ok(Ok(pb)) = timeout(Duration::from_millis(15), stream_b.next()).await {
                last_b = pb.data;
            }
            if let Ok(Ok(pc)) = timeout(Duration::from_millis(15), stream_c.next()).await {
                last_c = pc.data;
            }
            let ts = (Instant::now() - traj_t0_wall).as_secs_f32();
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
                println!("  [t={t:.1}s rep={:.1}/{n_reps}] z={:.2}m {:.2}V",
                    t / traj_total, r.z, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    // ── Hand back control: switch to Mode B passthrough ───────────────────
    cf.param.set("traj.mode", 0u8).await?;
    println!("Mode D complete — reverting to Mode B passthrough.");

    // ── Hold at hover before landing ──────────────────────────────────────
    println!("Hold 2 s...");
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

    // ── Land ──────────────────────────────────────────────────────────────
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

    // ── Write CSV ─────────────────────────────────────────────────────────
    fs::create_dir_all("../Controls/logs")?;
    let ts = Local::now().format("%Y%m%d_%H%M%S");
    let csv_path = format!("../Controls/logs/figure8_onboard_{ts}.csv");
    let file = fs::File::create(&csv_path)?;
    let mut w = BufWriter::new(file);
    writeln!(w, "time_s,x,y,z,vx,vy,vz,roll_deg,pitch_deg,yaw_deg,thrust,vbat,\
                 gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z")?;
    for r in &rows {
        writeln!(w, "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            r.time_s, r.x, r.y, r.z, r.vx, r.vy, r.vz,
            r.roll, r.pitch, r.yaw, r.thrust, r.vbat,
            r.gyro_x, r.gyro_y, r.gyro_z, r.acc_x, r.acc_y, r.acc_z)?;
    }
    println!("Saved {} rows → {csv_path}", rows.len());
    println!("Analyse: ~/.pyenv/versions/flying_robots/bin/python Controls/analyze_flight.py \
              --csv {csv_path} --type figure8");
    println!("Done.");
    Ok(())
}

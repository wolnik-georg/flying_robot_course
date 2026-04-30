//! Mode E — Vertical Loop: full 360° aerobatic loop in the X-Z plane.
//!
//! ## Geometry
//! The loop is a circle of radius R in the X-Z plane, centred at
//! (ox, oy, hover_z + R).  Starting from the bottom (hover position):
//!
//!   theta=0   (bottom) : x=0,   z=hover_z        ← drone starts here
//!   theta=π/2 (front)  : x=+R,  z=hover_z+R
//!   theta=π   (top)    : x=0,   z=hover_z+2R     ← INVERTED if fast enough
//!   theta=3π/2(rear)   : x=-R,  z=hover_z+R
//!   theta=2π  (bottom) : x=0,   z=hover_z        ← back to start
//!
//! ## Physics / inversion condition
//! At the top of the loop the centripetal acceleration points downward:
//!   f_d_z = m*(az + g) = m*(g − R·ω²)
//! The drone inverts when R·ω² > g, i.e. ω > √(g/R).
//!
//!   R=0.5 m → ω_min = 4.43 rad/s → T_max = 1.42 s → speed_min ≈ 2.1×
//!
//! ## Firmware mode
//! traj.mode = 2 (Mode E — vertical loop):
//!   cx polynomial → X axis
//!   cy polynomial → Z offset above hover_z   (same buffer, reinterpreted)
//!   Y held fixed at oy
//!
//! ## Iterative test plan
//!   1. CF2.1 + optical flow  : speed=0.5 (no inversion — tests firmware mode)
//!   2. CF2.1 + motion capture: speed=1.0 (near-inversion boundary)
//!   3. Brushless + mocap     : speed=2.5 (full inversion, ω≈5.5 rad/s >> ω_min)
//!
//! ## Usage
//!   cargo run --release --bin onboard_loop
//!   cargo run --release --bin onboard_loop -- --speed 2.5 --radius 0.5

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
const HOVER_HEIGHT: f32  = 1.0;   // m — must have clearance above + below for loop
const GRAVITY:      f32  = 9.81;
const LOG_MS:       u64  = 50;    // 20 Hz logging

/// Default loop parameters — safe for CF2.1 optical flow at speed=1.0 (sub-inversion).
const RADIUS_DEFAULT: f32  = 0.5;   // m
const T_LAP_DEFAULT:  f32  = 3.0;   // s at speed=1.0  →  ω=2.09 rad/s (sub-inversion)

/// Plan the vertical loop as a spline in (X, Z_offset) space.
///
/// Waypoints: circle parameterised as
///   x         = R * sin(theta)
///   z_offset  = R * (1 − cos(theta))    (0 at bottom, 2R at top)
///
/// The planner treats z_offset as the "Y" dimension; the firmware's Mode E
/// reinterprets cy as Z.  Y motion stays at zero throughout.
fn loop_trajectory(radius: f32, t_lap: f32) -> SplineTrajectory {
    let n = 8usize;
    let seg_dur = t_lap / n as f32;
    let waypoints: Vec<Waypoint> = (0..=n)
        .map(|i| {
            let theta = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
            let x        =  radius * theta.sin();
            let z_offset =  radius * (1.0 - theta.cos());
            // Pass (x, z_offset) — planner uses .x → cx, .y → cy (= cz in Mode E)
            Waypoint { pos: Vec3::new(x, z_offset, 0.0), yaw: 0.0 }
        })
        .collect();
    let durations = vec![seg_dur; n];
    SplineTrajectory::plan(&waypoints, &durations, true)
        .expect("Loop QP planning failed")
}

fn serialise_coefs(traj: &SplineTrajectory) -> Vec<f32> {
    let mut out = Vec::with_capacity(traj.segments.len() * 19);
    for seg in &traj.segments {
        out.push(seg.duration);
        out.extend_from_slice(&seg.cx);
        out.extend_from_slice(&seg.cy); // cy encodes Z offset for Mode E
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

async fn upload_coef(cf: &Crazyflie, index: u8, value: f32)
    -> Result<(), Box<dyn std::error::Error>>
{
    cf.param.set("traj.ci", index).await?;
    cf.param.set("traj.cv", value).await?;
    cf.param.set("traj.cw", 1u8).await?;
    sleep(Duration::from_millis(8)).await;
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // ── Parse args ────────────────────────────────────────────────────────
    let args: Vec<String> = std::env::args().collect();
    let speed: f32 = args.iter().position(|a| a == "--speed")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let radius: f32 = args.iter().position(|a| a == "--radius")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(RADIUS_DEFAULT);
    let n_reps: u32 = args.iter().position(|a| a == "--reps")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1);

    if n_reps == 0 || n_reps > 20 { eprintln!("--reps must be 1–20"); std::process::exit(1); }

    let t_lap = T_LAP_DEFAULT / speed;
    let omega = 2.0 * std::f32::consts::PI / t_lap;
    let omega_min_inversion = (GRAVITY / radius).sqrt();
    let speed_for_inversion = omega_min_inversion / (2.0 * std::f32::consts::PI / T_LAP_DEFAULT);
    let top_height = HOVER_HEIGHT + 2.0 * radius;

    println!("╔═══════════════════════════════════════════════════════╗");
    println!("║  Vertical Loop — Mode E (onboard XZ spline eval)      ║");
    println!("╚═══════════════════════════════════════════════════════╝");
    println!("  R={radius:.2}m  T={t_lap:.2}s/lap  ω={omega:.2}rad/s  speed={speed:.1}×  reps={n_reps}");
    println!("  Bottom: z={HOVER_HEIGHT:.1}m  Top: z={top_height:.1}m");
    println!("  Inversion threshold: speed≥{speed_for_inversion:.1}× (ω≥{omega_min_inversion:.2}rad/s)");
    if omega >= omega_min_inversion {
        println!("  ✅ INVERTED FLIGHT — drone inverts at top of loop");
        println!("     ⚠️  Requires: brushless hardware + motion capture");
    } else {
        println!("  ℹ️  Sub-inversion (upright throughout) — safe for CF2.1 optical flow");
        println!("     To invert: --speed {speed_for_inversion:.1}  (brushless + mocap required)");
    }

    // ── Plan trajectory ───────────────────────────────────────────────────
    let traj = loop_trajectory(radius, t_lap);
    let coefs = serialise_coefs(&traj);
    let n_segs = traj.segments.len();
    println!("\nPlanned: {n_segs} segs  {} coefs  traj={:.2}s", coefs.len(), traj.total_time);

    // ── Connect ───────────────────────────────────────────────────────────
    let link_context = LinkContext::new();
    println!("\nConnecting to {CF_URI} ...");
    let cf = Crazyflie::connect_from_uri(&link_context, CF_URI, NoTocCache).await?;
    println!("Connected.");

    let _ = cf.param.set("stabilizer.controller", 6u8).await;
    sleep(Duration::from_millis(100)).await;

    // Verify firmware has traj params (and check mode=2 is accepted — it's just a u8)
    cf.param.set("traj.mode", 0u8).await
        .map_err(|e| format!("traj params missing — flash updated firmware first: {e}"))?;
    println!("Firmware OK — traj params present.");

    // ── Log blocks ────────────────────────────────────────────────────────
    let mut block_a = cf.log.create_block().await?;
    for v in ["stateEstimate.x","stateEstimate.y","stateEstimate.z",
              "stabilizer.roll","stabilizer.pitch","stabilizer.yaw"] {
        add_var(&mut block_a, v).await;
    }
    let stream_a = block_a.start(LogPeriod::from_millis(LOG_MS)?).await?;

    let mut block_b = cf.log.create_block().await?;
    for v in ["stateEstimate.vx","stateEstimate.vy","stateEstimate.vz",
              "stabilizer.thrust","pm.vbat"] {
        add_var(&mut block_b, v).await;
    }
    let stream_b = block_b.start(LogPeriod::from_millis(LOG_MS)?).await?;

    let mut block_c = cf.log.create_block().await?;
    for v in ["gyro.x","gyro.y","gyro.z","acc.x","acc.y","acc.z"] {
        add_var(&mut block_c, v).await;
    }
    let stream_c = block_c.start(LogPeriod::from_millis(LOG_MS)?).await?;

    // ── Kalman reset ──────────────────────────────────────────────────────
    cf.commander.setpoint_rpyt(0.0,0.0,0.0,0u16).await?;
    let _ = cf.param.set("kalman.resetEstimation", 1u8).await;
    sleep(Duration::from_millis(200)).await;
    let _ = cf.param.set("kalman.resetEstimation", 0u8).await;
    println!("\nKalman reset — place flat on pad. Waiting 4s...");
    for _ in 0..40 {
        cf.commander.setpoint_rpyt(0.0,0.0,0.0,0u16).await?;
        sleep(Duration::from_millis(100)).await;
    }

    // ── Sample EKF origin ─────────────────────────────────────────────────
    let (mut xs, mut ys, mut yaws) = (Vec::new(), Vec::new(), Vec::new());
    println!("Sampling EKF origin...");
    for _ in 0..40 {
        cf.commander.setpoint_rpyt(0.0,0.0,0.0,0u16).await?;
        if let Ok(p) = stream_a.next().await {
            xs.push(gf(&p.data,"stateEstimate.x"));
            ys.push(gf(&p.data,"stateEstimate.y"));
            yaws.push(gf(&p.data,"stabilizer.yaw"));
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
    let (qw0, qz0) = ((oyaw_rad*0.5).cos(), (oyaw_rad*0.5).sin());
    println!("Origin: x={ox:+.3} y={oy:+.3} yaw={oyaw_deg:+.1}°");

    // ── Upload coefficients ───────────────────────────────────────────────
    println!("Uploading {} coefs ({n_segs} segs)...", coefs.len());
    let t0 = Instant::now();
    for (i, &v) in coefs.iter().enumerate() {
        upload_coef(&cf, i as u8, v).await?;
        if i % 20 == 19 { println!("  [{}/{}]", i+1, coefs.len()); }
    }
    println!("Upload done in {:.1}s", t0.elapsed().as_secs_f32());

    // traj.ox = ox  (loop starts at x=0 relative to origin, no horizontal offset)
    // traj.oy = oy  (Y fixed at origin throughout)
    // traj.hz = HOVER_HEIGHT  (Z offset added on top; loop bottom = hover_z + 0)
    // traj.dz = 0   (Z comes from polynomial, not linear ramp)
    // traj.mode = 2 (Mode E: cy → Z offset, Y fixed)
    cf.param.set("traj.nseg", n_segs as u8).await?;
    cf.param.set("traj.ox",   ox).await?;
    cf.param.set("traj.oy",   oy).await?;
    cf.param.set("traj.hz",   HOVER_HEIGHT).await?;
    cf.param.set("traj.dz",   0.0f32).await?;
    println!("Metadata: ox={ox:.3} oy={oy:.3} hz={HOVER_HEIGHT}");

    // ── Ramp to hover ─────────────────────────────────────────────────────
    println!("Ramp to {HOVER_HEIGHT:.1}m...");
    for step in 0..50usize {
        let h = 0.05 + step as f32 * (HOVER_HEIGHT - 0.05) / 49.0;
        cf.commander.setpoint_full_state(ox,oy,h,0.0,0.0,0.0,0.0,0.0,0.0,
                                         qw0,0.0,0.0,qz0,0.0,0.0,0.0).await?;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
        let _ = stream_c.next().await;
        sleep(Duration::from_millis(50)).await;
    }

    println!("Hover settle 5s...");
    let settle = Instant::now() + Duration::from_secs(5);
    while Instant::now() < settle {
        cf.commander.setpoint_full_state(ox,oy,HOVER_HEIGHT,0.0,0.0,0.0,0.0,0.0,0.0,
                                         qw0,0.0,0.0,qz0,0.0,0.0,0.0).await?;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
        let _ = stream_c.next().await;
    }

    // ── Trigger loop ──────────────────────────────────────────────────────
    cf.param.set("traj.mode",  2u8).await?;  // Mode E: vertical loop
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("\n=== LOOP! R={radius:.2}m  {:.1}s/lap  reps={n_reps}  top={top_height:.1}m ===", traj.total_time);
    if omega >= omega_min_inversion {
        println!("    (inverted flight at top)");
    }

    // ── Fly loop with logging ─────────────────────────────────────────────
    // Keepalive z tracks the expected loop height so the arming watchdog stays happy.
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String,Value> = HashMap::new();
    let mut last_c: HashMap<String,Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0_wall = Instant::now();
    let traj_end = Duration::from_secs_f32(traj.total_time * n_reps as f32);

    while Instant::now() < t0_wall + traj_end {
        let t = (Instant::now() - t0_wall).as_secs_f32();
        let theta = omega * t;
        // Keepalive follows expected Z of the loop (periodic — naturally handles multi-rep)
        let expected_z = HOVER_HEIGHT + radius * (1.0 - theta.cos());

        cf.commander.setpoint_full_state(ox,oy,expected_z,0.0,0.0,0.0,0.0,0.0,0.0,
                                         qw0,0.0,0.0,qz0,0.0,0.0,0.0).await?;
        if let Ok(pa) = stream_a.next().await {
            if let Ok(Ok(pb)) = timeout(Duration::from_millis(15), stream_b.next()).await {
                last_b = pb.data;
            }
            if let Ok(Ok(pc)) = timeout(Duration::from_millis(15), stream_c.next()).await {
                last_c = pc.data;
            }
            let ts = (Instant::now() - t0_wall).as_secs_f32();
            rows.push(Row {
                time_s: ts,
                x:      gf(&pa.data,"stateEstimate.x"),
                y:      gf(&pa.data,"stateEstimate.y"),
                z:      gf(&pa.data,"stateEstimate.z"),
                vx:     gf(&last_b, "stateEstimate.vx"),
                vy:     gf(&last_b, "stateEstimate.vy"),
                vz:     gf(&last_b, "stateEstimate.vz"),
                roll:   gf(&pa.data,"stabilizer.roll"),
                pitch:  gf(&pa.data,"stabilizer.pitch"),
                yaw:    gf(&pa.data,"stabilizer.yaw"),
                thrust: gf(&last_b, "stabilizer.thrust"),
                vbat:   gf(&last_b, "pm.vbat"),
                gyro_x: gf(&last_c, "gyro.x"),
                gyro_y: gf(&last_c, "gyro.y"),
                gyro_z: gf(&last_c, "gyro.z"),
                acc_x:  gf(&last_c, "acc.x"),
                acc_y:  gf(&last_c, "acc.y"),
                acc_z:  gf(&last_c, "acc.z"),
            });
            if last_print.elapsed() >= Duration::from_secs(1) {
                let r = rows.last().unwrap();
                let theta_deg = omega * ts * 180.0 / std::f32::consts::PI;
                println!("  [t={ts:.1}s θ={theta_deg:.0}°] z={:.2}m pitch={:.1}° {:.2}V",
                         r.z, r.pitch, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    // ── Revert to passthrough and hold at hover_z ────────────────────────
    cf.param.set("traj.mode", 0u8).await?;
    println!("\nLoop done — returning to hover {HOVER_HEIGHT:.1}m, 3s...");
    let hold = Instant::now() + Duration::from_secs(3);
    while Instant::now() < hold {
        cf.commander.setpoint_full_state(ox,oy,HOVER_HEIGHT,0.0,0.0,0.0,0.0,0.0,0.0,
                                         qw0,0.0,0.0,qz0,0.0,0.0,0.0).await?;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
        let _ = stream_c.next().await;
    }

    // ── Land ──────────────────────────────────────────────────────────────
    println!("Landing...");
    for step in (0..=40usize).rev() {
        let h = (step as f32 * HOVER_HEIGHT / 40.0).max(0.05);
        cf.commander.setpoint_full_state(ox,oy,h,0.0,0.0,0.0,0.0,0.0,0.0,
                                         qw0,0.0,0.0,qz0,0.0,0.0,0.0).await?;
        sleep(Duration::from_millis(100)).await;
    }
    cf.commander.setpoint_rpyt(0.0,0.0,0.0,0u16).await?;
    sleep(Duration::from_millis(200)).await;

    // ── Write CSV ─────────────────────────────────────────────────────────
    fs::create_dir_all("../Controls/logs")?;
    let ts_str = Local::now().format("%Y%m%d_%H%M%S");
    let speed_tag = format!("{speed:.1}").replace('.', "-");
    let csv_path = format!("../Controls/logs/loop_onboard_s{speed_tag}x_r{n_reps}_{ts_str}.csv");
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
    println!("Analyse: ~/.pyenv/versions/flying_robots/bin/python \
              Controls/analyze_flight.py --csv {csv_path} --type loop");
    Ok(())
}

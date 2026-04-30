//! Mode D Onboard Helix — ascending spiral using same architecture as onboard_circle.
//!
//! XY: circle r=0.30m evaluated onboard at 500 Hz (same as circle).
//! Z:  linear ramp via traj.dz param — firmware computes z = hover_z + lap_frac * dz.
//!     No Z polynomial needed: a helix is exactly a circle + linear Z ramp.
//!
//! One ascending lap: z goes from HOVER_HEIGHT → HOVER_HEIGHT + HELIX_DZ.
//! After the trajectory, the drone holds at the final height then lands.
//!
//! Usage:
//!   cargo run --release --bin onboard_helix
//!   cargo run --release --bin onboard_helix -- --speed 1.2
//!
//! Requires updated firmware (traj.dz param): cd firmware_app && make && cfloader flash ...

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

const CF_URI:        &str = "radio://0/80/2M/E7E7E7E7E7";
const HOVER_HEIGHT:  f32  = 1.0;
const HELIX_RADIUS:  f32  = 0.30;  // m — circle radius
const HELIX_DZ:      f32  = 0.40;  // m — height gained over one ascending lap
const OMEGA:         f32  = 0.6;   // rad/s → 1 lap ≈ 10.47 s
const LOG_MS:        u64  = 50;

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

fn serialise_coefs(traj: &SplineTrajectory) -> Vec<f32> {
    let mut out = Vec::with_capacity(traj.segments.len() * 19);
    for seg in &traj.segments { out.push(seg.duration); out.extend_from_slice(&seg.cx); out.extend_from_slice(&seg.cy); }
    out
}

struct Row { time_s: f32, x: f32, y: f32, z: f32, vx: f32, vy: f32, vz: f32, roll: f32, pitch: f32, yaw: f32, thrust: f32, vbat: f32, gyro_x: f32, gyro_y: f32, gyro_z: f32, acc_x: f32, acc_y: f32, acc_z: f32 }

fn gf(map: &HashMap<String, Value>, k: &str) -> f32 { map.get(k).map(|v| v.to_f64_lossy() as f32).unwrap_or(f32::NAN) }

async fn add_var(block: &mut LogBlock, name: &str) {
    if let Err(e) = block.add_variable(name).await { eprintln!("  [log] MISSING: {name} ({e})"); }
}

async fn upload_coef(cf: &Crazyflie, index: u8, value: f32) -> Result<(), Box<dyn std::error::Error>> {
    cf.param.set("traj.ci", index).await?;
    cf.param.set("traj.cv", value).await?;
    cf.param.set("traj.cw", 1u8).await?;
    sleep(Duration::from_millis(8)).await;
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let speed: f32 = args.iter().position(|a| a == "--speed")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let n_reps: u32 = args.iter().position(|a| a == "--reps")
        .and_then(|i| args.get(i+1)).and_then(|s| s.parse().ok()).unwrap_or(1);

    if speed < 0.5 || speed > 3.0 { eprintln!("--speed must be 0.5–3.0"); std::process::exit(1); }
    if n_reps == 0 || n_reps > 10 { eprintln!("--reps must be 1–10"); std::process::exit(1); }

    let traj = helix_trajectory(speed);
    let traj_total = traj.total_time;
    let coefs = serialise_coefs(&traj);
    let n_segs = traj.segments.len();
    let total_dz = HELIX_DZ * n_reps as f32;
    let top_z = HOVER_HEIGHT + total_dz;
    println!("Mode D Helix | speed={speed:.1}× | lap={traj_total:.2}s | reps={n_reps} | r={HELIX_RADIUS}m | dz={HELIX_DZ}m/lap");
    println!("  z: {HOVER_HEIGHT:.1}m → {top_z:.1}m over {:.1}s  ({n_reps}×{traj_total:.1}s)", traj_total * n_reps as f32);

    let link_context = LinkContext::new();
    println!("Connecting to {CF_URI} ...");
    let cf = Crazyflie::connect_from_uri(&link_context, CF_URI, NoTocCache).await?;
    println!("Connected.");

    let _ = cf.param.set("stabilizer.controller", 6u8).await;
    sleep(Duration::from_millis(100)).await;

    cf.param.set("traj.mode", 0u8).await
        .map_err(|e| format!("traj params missing — flash firmware first: {e}"))?;
    // Check traj.dz exists (added in helix firmware update)
    cf.param.set("traj.dz", 0.0f32).await
        .map_err(|e| format!("traj.dz missing — rebuild firmware with helix support: {e}"))?;
    println!("Firmware OK.");

    let mut block_a = cf.log.create_block().await?;
    for v in ["stateEstimate.x","stateEstimate.y","stateEstimate.z","stabilizer.roll","stabilizer.pitch","stabilizer.yaw"] { add_var(&mut block_a, v).await; }
    let stream_a = block_a.start(LogPeriod::from_millis(LOG_MS)?).await?;
    let mut block_b = cf.log.create_block().await?;
    for v in ["stateEstimate.vx","stateEstimate.vy","stateEstimate.vz","stabilizer.thrust","pm.vbat"] { add_var(&mut block_b, v).await; }
    let stream_b = block_b.start(LogPeriod::from_millis(LOG_MS)?).await?;
    let mut block_c = cf.log.create_block().await?;
    for v in ["gyro.x","gyro.y","gyro.z","acc.x","acc.y","acc.z"] { add_var(&mut block_c, v).await; }
    let stream_c = block_c.start(LogPeriod::from_millis(LOG_MS)?).await?;

    cf.commander.setpoint_rpyt(0.0,0.0,0.0,0u16).await?;
    let _ = cf.param.set("kalman.resetEstimation", 1u8).await;
    sleep(Duration::from_millis(200)).await;
    let _ = cf.param.set("kalman.resetEstimation", 0u8).await;
    println!("Kalman reset — place flat on pad. Waiting 4s...");
    for _ in 0..40 { cf.commander.setpoint_rpyt(0.0,0.0,0.0,0u16).await?; sleep(Duration::from_millis(100)).await; }

    let (mut xs, mut ys, mut yaws) = (Vec::new(), Vec::new(), Vec::new());
    println!("Sampling EKF origin...");
    for _ in 0..40 {
        cf.commander.setpoint_rpyt(0.0,0.0,0.0,0u16).await?;
        if let Ok(p) = stream_a.next().await {
            xs.push(gf(&p.data,"stateEstimate.x")); ys.push(gf(&p.data,"stateEstimate.y")); yaws.push(gf(&p.data,"stabilizer.yaw"));
        }
        let _ = stream_b.next().await; let _ = stream_c.next().await;
        sleep(Duration::from_millis(50)).await;
    }
    let n = xs.len() as f32;
    let ox = xs.iter().sum::<f32>() / n;
    let oy = ys.iter().sum::<f32>() / n;
    let oyaw_deg = yaws.iter().sum::<f32>() / n;
    let oyaw_rad = oyaw_deg * std::f32::consts::PI / 180.0;
    let (qw0, qz0) = ((oyaw_rad*0.5).cos(), (oyaw_rad*0.5).sin());
    println!("Origin: x={ox:+.3} y={oy:+.3} yaw={oyaw_deg:+.1}°");

    println!("Uploading {} coefs...", coefs.len());
    let t0 = Instant::now();
    for (i, &v) in coefs.iter().enumerate() {
        upload_coef(&cf, i as u8, v).await?;
        if i % 20 == 19 { println!("  [{}/{}]", i+1, coefs.len()); }
    }
    println!("Upload done in {:.1}s", t0.elapsed().as_secs_f32());

    cf.param.set("traj.nseg", n_segs as u8).await?;
    cf.param.set("traj.ox",   ox - HELIX_RADIUS).await?;  // align first waypoint to hover
    cf.param.set("traj.oy",   oy).await?;
    cf.param.set("traj.hz",   HOVER_HEIGHT).await?;
    cf.param.set("traj.dz",   HELIX_DZ).await?;  // ascending: firmware uses t_global so stacks over n_reps laps
    println!("Metadata: z {HOVER_HEIGHT:.1} → {top_z:.1}m  ({n_reps} laps)");

    println!("Ramp to {HOVER_HEIGHT:.1}m...");
    for step in 0..50usize {
        let h = 0.05 + step as f32 * (HOVER_HEIGHT-0.05) / 49.0;
        cf.commander.setpoint_full_state(ox,oy,h,0.0,0.0,0.0,0.0,0.0,0.0,qw0,0.0,0.0,qz0,0.0,0.0,0.0).await?;
        let _ = stream_a.next().await; let _ = stream_b.next().await; let _ = stream_c.next().await;
        sleep(Duration::from_millis(50)).await;
    }
    println!("Hover settle 5s...");
    let settle = Instant::now() + Duration::from_secs(5);
    while Instant::now() < settle {
        cf.commander.setpoint_full_state(ox,oy,HOVER_HEIGHT,0.0,0.0,0.0,0.0,0.0,0.0,qw0,0.0,0.0,qz0,0.0,0.0,0.0).await?;
        let _ = stream_a.next().await; let _ = stream_b.next().await; let _ = stream_c.next().await;
    }

    cf.param.set("traj.mode",  1u8).await?;
    sleep(Duration::from_millis(50)).await;
    cf.param.set("traj.start", 1u8).await?;
    println!("=== Helix ascending: {HOVER_HEIGHT:.1} → {top_z:.1}m over {:.1}s ({n_reps} laps) ===",
             traj_total * n_reps as f32);

    let traj_end = Duration::from_secs_f32(traj_total * n_reps as f32);
    let mut rows: Vec<Row> = Vec::new();
    let mut last_b: HashMap<String,Value> = HashMap::new();
    let mut last_c: HashMap<String,Value> = HashMap::new();
    let mut last_print = Instant::now();
    let t0_wall = Instant::now();

    while Instant::now() < t0_wall + traj_end {
        let t = (Instant::now()-t0_wall).as_secs_f32();
        // Keepalive z tracks the expected helix height (continuous ramp over all reps)
        let expected_z = HOVER_HEIGHT + (t / (traj_total * n_reps as f32)) * total_dz;
        cf.commander.setpoint_full_state(ox,oy,expected_z,0.0,0.0,0.0,0.0,0.0,0.0,qw0,0.0,0.0,qz0,0.0,0.0,0.0).await?;
        if let Ok(pa) = stream_a.next().await {
            if let Ok(Ok(pb)) = timeout(Duration::from_millis(15), stream_b.next()).await { last_b = pb.data; }
            if let Ok(Ok(pc)) = timeout(Duration::from_millis(15), stream_c.next()).await { last_c = pc.data; }
            let ts = (Instant::now()-t0_wall).as_secs_f32();
            rows.push(Row {
                time_s: ts,
                x: gf(&pa.data,"stateEstimate.x"), y: gf(&pa.data,"stateEstimate.y"), z: gf(&pa.data,"stateEstimate.z"),
                vx: gf(&last_b,"stateEstimate.vx"), vy: gf(&last_b,"stateEstimate.vy"), vz: gf(&last_b,"stateEstimate.vz"),
                roll: gf(&pa.data,"stabilizer.roll"), pitch: gf(&pa.data,"stabilizer.pitch"), yaw: gf(&pa.data,"stabilizer.yaw"),
                thrust: gf(&last_b,"stabilizer.thrust"), vbat: gf(&last_b,"pm.vbat"),
                gyro_x: gf(&last_c,"gyro.x"), gyro_y: gf(&last_c,"gyro.y"), gyro_z: gf(&last_c,"gyro.z"),
                acc_x: gf(&last_c,"acc.x"), acc_y: gf(&last_c,"acc.y"), acc_z: gf(&last_c,"acc.z"),
            });
            if last_print.elapsed() >= Duration::from_secs(1) {
                let r = rows.last().unwrap();
                println!("  [t={t:.1}s] z={:.2}m (target={expected_z:.2}m) {:.2}V", r.z, r.vbat);
                last_print = Instant::now();
            }
        }
    }

    cf.param.set("traj.mode", 0u8).await?;
    cf.param.set("traj.dz",   0.0f32).await?;  // reset for next flight
    println!("Helix done — holding at top ({top_z:.1}m) for 2s...");
    let hold = Instant::now() + Duration::from_secs(2);
    while Instant::now() < hold {
        cf.commander.setpoint_full_state(ox,oy,top_z,0.0,0.0,0.0,0.0,0.0,0.0,qw0,0.0,0.0,qz0,0.0,0.0,0.0).await?;
        let _ = stream_a.next().await; let _ = stream_b.next().await; let _ = stream_c.next().await;
    }

    println!("Landing from {top_z:.1}m...");
    for step in (0..=40usize).rev() {
        let h = (step as f32 * top_z / 40.0).max(0.05);
        cf.commander.setpoint_full_state(ox,oy,h,0.0,0.0,0.0,0.0,0.0,0.0,qw0,0.0,0.0,qz0,0.0,0.0,0.0).await?;
        sleep(Duration::from_millis(120)).await;
    }
    cf.commander.setpoint_rpyt(0.0,0.0,0.0,0u16).await?;
    sleep(Duration::from_millis(200)).await;

    fs::create_dir_all("../Controls/logs")?;
    let ts = Local::now().format("%Y%m%d_%H%M%S");
    let speed_tag = format!("{:.1}",speed).replace('.',"-");
    let csv_path = format!("../Controls/logs/helix_onboard_s{speed_tag}x_r{n_reps}_{ts}.csv");
    let file = fs::File::create(&csv_path)?;
    let mut w = BufWriter::new(file);
    writeln!(w,"time_s,x,y,z,vx,vy,vz,roll_deg,pitch_deg,yaw_deg,thrust,vbat,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z")?;
    for r in &rows {
        writeln!(w,"{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            r.time_s,r.x,r.y,r.z,r.vx,r.vy,r.vz,r.roll,r.pitch,r.yaw,r.thrust,r.vbat,
            r.gyro_x,r.gyro_y,r.gyro_z,r.acc_x,r.acc_y,r.acc_z)?;
    }
    println!("Saved {} rows → {csv_path}", rows.len());
    Ok(())
}

//! Shared helpers used by all onboard flight binaries (figure8, circle, helix, loop).
//!
//! Each binary brings everything in with:
//!   use multirotor_simulator::flight_common::*;
//!
//! What is shared here:
//!   - CF_URI, HOVER_HEIGHT, LOG_MS constants
//!   - Row struct and collect_row()
//!   - Origin struct
//!   - Pure helpers: gf(), serialise_coefs()
//!   - Async sequences: connect_drone(), verify_firmware(), setup_log_blocks(),
//!     kalman_reset(), sample_origin(), upload_trajectory(),
//!     ramp_to_hover(), hover_settle(), hold_and_land()
//!   - write_csv(), write_csv_with_metadata()
//!
//! What stays in each binary:
//!   - Trajectory-specific constants (radius, omega, etc.)
//!   - The trajectory planning function
//!   - main(): arg parsing, traj.{nseg,ox,oy,hz,dz,mode} params,
//!             the main flight loop (keepalive z + progress print)

pub use crazyflie_lib::{Crazyflie, NoTocCache, Value};
pub use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod, LogStream};
pub use crazyflie_link::LinkContext;

use std::collections::HashMap;
use std::fs;
use std::io::{BufWriter, Write};
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use tokio::time::sleep;

use crate::prelude::SplineTrajectory;

// ── Constants ──────────────────────────────────────────────────────────────

pub const CF_URI:       &str = "radio://0/80/2M/E7E7E7E7E7";
pub const HOVER_HEIGHT: f32  = 1.0;   // m
pub const LOG_MS:       u64  = 50;    // 20 Hz logging

// ── Data types ─────────────────────────────────────────────────────────────

/// One logged sample row.
pub struct Row {
    pub time_s: f32,
    pub x: f32, pub y: f32, pub z: f32,
    pub vx: f32, pub vy: f32, pub vz: f32,
    pub roll: f32, pub pitch: f32, pub yaw: f32,
    pub thrust: f32, pub vbat: f32,
    pub gyro_x: f32, pub gyro_y: f32, pub gyro_z: f32,
    pub acc_x: f32, pub acc_y: f32, pub acc_z: f32,
}

/// EKF origin sampled on the pad, with pre-computed hover quaternion.
pub struct Origin {
    pub ox:      f32,
    pub oy:      f32,
    pub yaw_deg: f32,
    pub qw0:     f32,
    pub qz0:     f32,
}

// ── Pure helpers ───────────────────────────────────────────────────────────

/// Extract a f32 from a log data map, returning NaN on missing keys.
pub fn gf(map: &HashMap<String, Value>, k: &str) -> f32 {
    map.get(k).map(|v| v.to_f64_lossy() as f32).unwrap_or(f32::NAN)
}

/// Serialise a SplineTrajectory to the flat firmware layout:
/// [duration, cx0..cx8, cy0..cy8] × n_segs  (19 floats/segment).
pub fn serialise_coefs(traj: &SplineTrajectory) -> Vec<f32> {
    let mut out = Vec::with_capacity(traj.segments.len() * 19);
    for seg in &traj.segments {
        out.push(seg.duration);
        out.extend_from_slice(&seg.cx);
        out.extend_from_slice(&seg.cy);
    }
    out
}

/// Serialise the Z-axis polynomials for the 3D trajectory firmware layout:
/// [cz0..cz8] × n_segs  (9 floats/segment, no duration — shared with position buffer).
pub fn serialise_z_coefs(traj: &SplineTrajectory) -> Vec<f32> {
    let mut out = Vec::with_capacity(traj.segments.len() * 9);
    for seg in &traj.segments {
        out.extend_from_slice(&seg.cz);
    }
    out
}

/// Build a Row from already-collected log maps.
pub fn collect_row(
    pa: &HashMap<String, Value>,
    lb: &HashMap<String, Value>,
    lc: &HashMap<String, Value>,
    ts: f32,
) -> Row {
    Row {
        time_s: ts,
        x:      gf(pa, "stateEstimate.x"),
        y:      gf(pa, "stateEstimate.y"),
        z:      gf(pa, "stateEstimate.z"),
        vx:     gf(lb, "stateEstimate.vx"),
        vy:     gf(lb, "stateEstimate.vy"),
        vz:     gf(lb, "stateEstimate.vz"),
        roll:   gf(pa, "stabilizer.roll"),
        pitch:  gf(pa, "stabilizer.pitch"),
        yaw:    gf(pa, "stabilizer.yaw"),
        thrust: gf(lb, "stabilizer.thrust"),
        vbat:   gf(lb, "pm.vbat"),
        gyro_x: gf(lc, "gyro.x"),
        gyro_y: gf(lc, "gyro.y"),
        gyro_z: gf(lc, "gyro.z"),
        acc_x:  gf(lc, "acc.x"),
        acc_y:  gf(lc, "acc.y"),
        acc_z:  gf(lc, "acc.z"),
    }
}

/// Write all rows to a CSV file at `path`.
pub fn write_csv(rows: &[Row], path: &str) -> std::io::Result<()> {
    write_csv_with_metadata(rows, path, &[])
}

/// Write rows to CSV with optional run-signature metadata lines.
///
/// Metadata is emitted as comment lines before the header:
///   # meta:key=value
pub fn write_csv_with_metadata(
    rows: &[Row],
    path: &str,
    metadata: &[(String, String)],
) -> std::io::Result<()> {
    fs::create_dir_all("../Controls/logs")?;
    let file = fs::File::create(path)?;
    let mut w = BufWriter::new(file);
    for (k, v) in metadata {
        writeln!(w, "# meta:{}={}", k, v)?;
    }
    writeln!(w, "time_s,x,y,z,vx,vy,vz,roll_deg,pitch_deg,yaw_deg,thrust,vbat,\
                 gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z")?;
    for r in rows {
        writeln!(w, "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            r.time_s, r.x, r.y, r.z, r.vx, r.vy, r.vz,
            r.roll, r.pitch, r.yaw, r.thrust, r.vbat,
            r.gyro_x, r.gyro_y, r.gyro_z, r.acc_x, r.acc_y, r.acc_z)?;
    }
    Ok(())
}

// ── Mocap UDP bridge ───────────────────────────────────────────────────────

pub const MOCAP_UDP_PORT: u16 = 9872;

/// Receives decoded mocap poses from the background UDP task.
pub struct MocapRx(mpsc::Receiver<([f32; 3], [f32; 4])>);

/// Spawn a background task that reads UDP pose packets from `mocap_bridge.py`
/// and queues them. Call `forward_mocap` in your flight loop to drain the queue
/// and send each pose to the CF EKF.
pub fn start_mocap_udp_task() -> MocapRx {
    let (tx, rx) = mpsc::channel(64);
    tokio::spawn(async move {
        let sock = UdpSocket::bind(format!("127.0.0.1:{MOCAP_UDP_PORT}"))
            .await
            .expect("mocap UDP bind failed — is port 9872 free?");
        println!("[mocap] Listening on UDP port {MOCAP_UDP_PORT}");
        let mut buf = [0u8; 28]; // 7 × f32
        loop {
            if sock.recv(&mut buf).await.is_ok() {
                let f = |i: usize| f32::from_le_bytes(buf[i*4..i*4+4].try_into().unwrap());
                let pos  = [f(0), f(1), f(2)];
                let quat = [f(3), f(4), f(5), f(6)];
                let _ = tx.try_send((pos, quat)); // drop if queue full (never blocks)
            }
        }
    });
    MocapRx(rx)
}

/// Drain all queued mocap poses and send each one to the CF EKF.
/// Call this once per iteration of your flight loop.
pub async fn forward_mocap(
    rx: &mut MocapRx,
    cf: &Crazyflie,
) -> Result<(), Box<dyn std::error::Error>> {
    while let Ok((pos, quat)) = rx.0.try_recv() {
        cf.localization.external_pose.send_external_pose(pos, quat).await?;
    }
    Ok(())
}

// ── Async helpers ──────────────────────────────────────────────────────────

/// Register a log variable, printing a warning if it is absent.
pub async fn add_var(block: &mut LogBlock, name: &str) {
    if let Err(e) = block.add_variable(name).await {
        eprintln!("  [log] MISSING: {name} ({e})");
    }
}

/// Write one coefficient into the firmware's trajectory coefficient store.
pub async fn upload_coef(
    cf: &Crazyflie,
    index: u8,
    value: f32,
) -> Result<(), Box<dyn std::error::Error>> {
    cf.param.set("traj.ci", index).await?;
    cf.param.set("traj.cv", value).await?;
    cf.param.set("traj.cw", 1u8).await?;
    sleep(Duration::from_millis(8)).await;
    Ok(())
}

/// Connect to the Crazyflie at CF_URI and activate the SE(3) geometric controller.
pub async fn connect_drone(
    link_ctx: &LinkContext,
) -> Result<Crazyflie, Box<dyn std::error::Error>> {
    println!("Connecting to {CF_URI} ...");
    let cf = Crazyflie::connect_from_uri(link_ctx, CF_URI, NoTocCache).await?;
    println!("Connected.");
    let _ = cf.param.set("stabilizer.controller", 6u8).await;
    sleep(Duration::from_millis(100)).await;
    Ok(cf)
}

/// Check that the trajectory firmware params exist (traj.mode + traj.dz + traj.z_mode).
/// Returns a clear error message if the firmware needs to be flashed.
pub async fn verify_firmware(cf: &Crazyflie) -> Result<(), Box<dyn std::error::Error>> {
    cf.param.set("traj.mode", 0u8).await
        .map_err(|e| format!("traj params missing — flash firmware first: {e}"))?;
    cf.param.set("traj.dz", 0.0f32).await
        .map_err(|e| format!("traj.dz missing — rebuild firmware: {e}"))?;
    cf.param.set("traj.z_mode", 0u8).await
        .map_err(|e| format!("traj.z_mode missing — rebuild firmware (3D trajectory update): {e}"))?;
    cf.param.set("traj.att_ctrl_mode", 1u8).await
        .map_err(|e| format!("traj.att_ctrl_mode missing — rebuild firmware (att_ctrl_mode update): {e}"))?;
    println!("Firmware OK.");
    Ok(())
}

/// Create the three standard log blocks (pose, velocity/power, IMU) and start them.
pub async fn setup_log_blocks(
    cf: &Crazyflie,
) -> Result<(LogStream, LogStream, LogStream), Box<dyn std::error::Error>> {
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

    Ok((stream_a, stream_b, stream_c))
}

/// Upload all trajectory coefficients with progress reporting.
pub async fn upload_trajectory(
    cf: &Crazyflie,
    coefs: &[f32],
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Uploading {} coefs...", coefs.len());
    let t0 = Instant::now();
    for (i, &v) in coefs.iter().enumerate() {
        upload_coef(cf, i as u8, v).await?;
        if i % 20 == 19 { println!("  [{}/{}]", i + 1, coefs.len()); }
    }
    println!("Upload done in {:.1}s", t0.elapsed().as_secs_f32());
    Ok(())
}

/// Upload Z-axis polynomial coefficients via the zci/zcv/zcw protocol.
pub async fn upload_z_trajectory(
    cf: &Crazyflie,
    coefs: &[f32],
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Uploading {} z coefs...", coefs.len());
    let t0 = Instant::now();
    for (i, &v) in coefs.iter().enumerate() {
        cf.param.set("traj.zci", i as u8).await?;
        cf.param.set("traj.zcv", v).await?;
        cf.param.set("traj.zcw", 1u8).await?;
        sleep(Duration::from_millis(8)).await;
        if i % 20 == 19 { println!("  [{}/{}]", i + 1, coefs.len()); }
    }
    println!("Z upload done in {:.1}s", t0.elapsed().as_secs_f32());
    Ok(())
}

/// Serialise attitude polynomials to the flat firmware layout:
/// [croll0..croll8, cpitch0..cpitch8] × n_segs  (18 floats/segment, no duration).
pub fn serialise_att_coefs(coefs: &[([f32; 9], [f32; 9])]) -> Vec<f32> {
    let mut out = Vec::with_capacity(coefs.len() * 18);
    for (cr, cp) in coefs {
        out.extend_from_slice(cr);
        out.extend_from_slice(cp);
    }
    out
}

/// Upload attitude polynomial coefficients via the aci/acv/acw protocol.
pub async fn upload_att_trajectory(
    cf: &Crazyflie,
    coefs: &[f32],
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Uploading {} att coefs...", coefs.len());
    let t0 = Instant::now();
    for (i, &v) in coefs.iter().enumerate() {
        cf.param.set("traj.aci", i as u8).await?;
        cf.param.set("traj.acv", v).await?;
        cf.param.set("traj.acw", 1u8).await?;
        sleep(Duration::from_millis(8)).await;
        if i % 20 == 19 { println!("  [{}/{}]", i + 1, coefs.len()); }
    }
    println!("Att upload done in {:.1}s", t0.elapsed().as_secs_f32());
    Ok(())
}

/// Kalman reset + 4 s settle (drone must be placed flat on the pad).
pub async fn kalman_reset(cf: &Crazyflie) -> Result<(), Box<dyn std::error::Error>> {
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    let _ = cf.param.set("kalman.resetEstimation", 1u8).await;
    sleep(Duration::from_millis(200)).await;
    let _ = cf.param.set("kalman.resetEstimation", 0u8).await;
    println!("Kalman reset — place flat on pad. Waiting 4s...");
    for _ in 0..40 {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        sleep(Duration::from_millis(100)).await;
    }
    Ok(())
}

/// Sample EKF position + yaw over 40 packets to get a stable origin estimate.
pub async fn sample_origin(
    cf: &Crazyflie,
    sa: &LogStream,
    sb: &LogStream,
    sc: &LogStream,
) -> Result<Origin, Box<dyn std::error::Error>> {
    let (mut xs, mut ys, mut yaws) = (Vec::new(), Vec::new(), Vec::new());
    println!("Sampling EKF origin...");
    for _ in 0..40 {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        if let Ok(p) = sa.next().await {
            xs.push(gf(&p.data, "stateEstimate.x"));
            ys.push(gf(&p.data, "stateEstimate.y"));
            yaws.push(gf(&p.data, "stabilizer.yaw"));
        }
        let _ = sb.next().await;
        let _ = sc.next().await;
        sleep(Duration::from_millis(50)).await;
    }
    let n = xs.len() as f32;
    let ox      = xs.iter().sum::<f32>() / n;
    let oy      = ys.iter().sum::<f32>() / n;
    let yaw_deg = yaws.iter().sum::<f32>() / n;
    let yaw_rad = yaw_deg * std::f32::consts::PI / 180.0;
    let (qw0, qz0) = ((yaw_rad * 0.5).cos(), (yaw_rad * 0.5).sin());
    println!("Origin: x={ox:+.3} y={oy:+.3} yaw={yaw_deg:+.1}°");
    Ok(Origin { ox, oy, yaw_deg, qw0, qz0 })
}

/// Ramp from 5 cm to HOVER_HEIGHT over ~2.5 s (50 steps × 50 ms).
pub async fn ramp_to_hover(
    cf: &Crazyflie,
    o: &Origin,
    sa: &LogStream,
    sb: &LogStream,
    sc: &LogStream,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Ramp to {HOVER_HEIGHT:.1}m...");
    for step in 0..50usize {
        let h = 0.05 + step as f32 * (HOVER_HEIGHT - 0.05) / 49.0;
        cf.commander.setpoint_full_state(
            o.ox, o.oy, h, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            o.qw0, 0.0, 0.0, o.qz0, 0.0, 0.0, 0.0,
        ).await?;
        let _ = sa.next().await;
        let _ = sb.next().await;
        let _ = sc.next().await;
        sleep(Duration::from_millis(50)).await;
    }
    Ok(())
}

/// Hover at HOVER_HEIGHT for 5 s to let the EKF converge.
pub async fn hover_settle(
    cf: &Crazyflie,
    o: &Origin,
    sa: &LogStream,
    sb: &LogStream,
    sc: &LogStream,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Hover settle 5s...");
    let settle = Instant::now() + Duration::from_secs(5);
    while Instant::now() < settle {
        cf.commander.setpoint_full_state(
            o.ox, o.oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            o.qw0, 0.0, 0.0, o.qz0, 0.0, 0.0, 0.0,
        ).await?;
        let _ = sa.next().await;
        let _ = sb.next().await;
        let _ = sc.next().await;
    }
    Ok(())
}

/// Hold at `hold_z` for `hold_secs`, then descend linearly to the ground and cut motors.
pub async fn hold_and_land(
    cf: &Crazyflie,
    o: &Origin,
    hold_z: f32,
    hold_secs: u64,
    sa: &LogStream,
    sb: &LogStream,
    sc: &LogStream,
) -> Result<(), Box<dyn std::error::Error>> {
    let hold = Instant::now() + Duration::from_secs(hold_secs);
    while Instant::now() < hold {
        cf.commander.setpoint_full_state(
            o.ox, o.oy, hold_z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            o.qw0, 0.0, 0.0, o.qz0, 0.0, 0.0, 0.0,
        ).await?;
        let _ = sa.next().await;
        let _ = sb.next().await;
        let _ = sc.next().await;
    }
    println!("Landing from {hold_z:.1}m...");
    for step in (0..=40usize).rev() {
        let h = (step as f32 * hold_z / 40.0).max(0.05);
        cf.commander.setpoint_full_state(
            o.ox, o.oy, h, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            o.qw0, 0.0, 0.0, o.qz0, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(100)).await;
    }
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    sleep(Duration::from_millis(200)).await;
    Ok(())
}

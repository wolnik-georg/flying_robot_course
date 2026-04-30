//! INDI Hover — first hardware validation of INDI firmware (Mode B, 25 Hz)
//!
//! Assumes the firmware is flashed with CONTROLLER_MODE = 1 (attitude INDI).
//! Sends full-state CRTP setpoints (pos + vel + acc + quat + ω) identical to
//! hover_test.rs so the firmware's outer SE(3) position loop and inner INDI
//! torque loop both receive proper feedforward.
//!
//! Logs detailed state to runs/indi_hover_<timestamp>.csv including:
//!   - EKF position, velocity, attitude
//!   - Raw gyro and reconstructed angular acceleration (J·α, [µNm])
//!   - Firmware thrust
//!   - Desired quaternion sent as setpoint (for attitude error audit)
//!
//! Post-flight checks:
//!   1. τ_meas = J·α (reconstructed torque) should oscillate symmetrically around
//!      zero at hover — confirms INDI incremental law is balanced.
//!   2. gyro std < 30 °/s RMS — INDI should be at least as smooth as geometric.
//!   3. Position drift < 10 cm after 10 s — same criterion as hover_test.
//!
//! Usage:
//!   cargo run --release --bin indi_hover               # 30 s hover
//!   cargo run --release --bin indi_hover -- --secs 10  # 10 s hover
//!
//! Requirements:
//!   cd firmware_app && make cload  (CONTROLLER_MODE = 1 must be set)

use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod};
use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::fs::File;
use std::io::Write;
use std::time::{Duration, Instant};
use tokio::time::sleep;
use chrono::Utc;

const CF_URI:       &str = "radio://0/80/2M/E7E7E7E7E7";
const CONTROLLER:   u8   = 6;   // OOT Rust controller (must be INDI-flashed)
const HOVER_HEIGHT: f32  = 0.5; // m

// Crazyflie 2.1 inertia [kg·m²] — for τ_meas = J·α reconstruction.
const JXX: f32 = 16.571710e-6;
const JYY: f32 = 16.655602e-6;
const JZZ: f32 = 29.261652e-6;

// ── Helpers ────────────────────────────────────────────────────────────────

async fn add_var(block: &mut LogBlock, name: &str) {
    match block.add_variable(name).await {
        Ok(_)  => println!("  [log] added: {name}"),
        Err(e) => println!("  [log] MISSING: {name} ({e})"),
    }
}

async fn add_var_first(block: &mut LogBlock, candidates: &[&'static str]) -> Option<&'static str> {
    for &name in candidates {
        if block.add_variable(name).await.is_ok() {
            println!("  [log] added: {name}");
            return Some(name);
        }
    }
    None
}

fn get_f32(map: &HashMap<String, Value>, key: &str) -> f32 {
    map.get(key).map(|v| v.to_f64_lossy() as f32).unwrap_or(0.0)
}

// ── Log row ────────────────────────────────────────────────────────────────

#[derive(Default, Clone)]
struct Row {
    time_ms:   u64,
    pos_x:     f32, pos_y: f32, pos_z: f32,
    vel_x:     f32, vel_y: f32, vel_z: f32,
    roll:      f32, pitch: f32, yaw: f32,
    thrust_fw: f32, vbat:  f32,
    gyro_x:    f32, gyro_y: f32, gyro_z: f32,
    // desired setpoint (what we send)
    sp_qw:     f32, sp_qx: f32, sp_qy: f32, sp_qz: f32,
    sp_z:      f32,
    // reconstructed torque τ_meas = J·α  [µNm]  (finite-diff of gyro × inertia)
    tau_meas_x: f32, tau_meas_y: f32, tau_meas_z: f32,
}

// ── Main ───────────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let hover_secs: u64 = args.iter()
        .position(|a| a == "--secs")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(30);

    println!("=== INDI Hover Test ===");
    println!("  HOVER_HEIGHT={HOVER_HEIGHT:.2}m  duration={hover_secs}s");
    println!("  Controller: OOT INDI (CONTROLLER_MODE=1 must be flashed)");

    let link_context = LinkContext::new();
    println!("Connecting to {CF_URI}...");
    let cf = Crazyflie::connect_from_uri(&link_context, CF_URI, NoTocCache).await?;
    println!("Connected.");

    // ── Log blocks ────────────────────────────────────────────────────────
    let mut block_a = cf.log.create_block().await?;
    for v in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz",
              "stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw",
              "stabilizer.thrust"] {
        add_var(&mut block_a, v).await;
    }
    let stream_a = block_a.start(LogPeriod::from_millis(40)?).await?;

    let mut block_b = cf.log.create_block().await?;
    let m1_var = add_var_first(&mut block_b,
        &["motor.m1req", "motor.m1pwm", "motor.m1"]).await
        .unwrap_or("motor.m1req");
    add_var(&mut block_b, "pm.vbat").await;
    // Gyro — 25 Hz update from the IMU
    for v in ["gyro.x", "gyro.y", "gyro.z"] {
        add_var(&mut block_b, v).await;
    }
    let stream_b = block_b.start(LogPeriod::from_millis(40)?).await?;

    // ── Set OOT controller ────────────────────────────────────────────────
    let _ = cf.param.set("stabilizer.controller", CONTROLLER).await;
    sleep(Duration::from_millis(100)).await;

    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    let _ = cf.param.set("kalman.resetEstimation", 1u8).await;
    sleep(Duration::from_millis(200)).await;
    let _ = cf.param.set("kalman.resetEstimation", 0u8).await;
    println!("Kalman reset — place drone flat, do NOT move...");

    for _ in 0..20usize {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        sleep(Duration::from_millis(200)).await;
    }

    // ── Sample EKF origin ─────────────────────────────────────────────────
    let mut xs = Vec::new(); let mut ys = Vec::new(); let mut yaws = Vec::new();
    let n = 40usize;
    println!("Sampling EKF origin ({n} packets ≈ 2 s)...");
    for _ in 0..n {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        if let Ok(p) = stream_a.next().await {
            xs.push(get_f32(&p.data, "stateEstimate.x"));
            ys.push(get_f32(&p.data, "stateEstimate.y"));
            yaws.push(get_f32(&p.data, "stabilizer.yaw"));
        }
        let _ = stream_b.next().await;
    }
    let ox = xs.iter().sum::<f32>() / n as f32;
    let oy = ys.iter().sum::<f32>() / n as f32;
    let oyaw_rad = (yaws.iter().sum::<f32>() / n as f32) * std::f32::consts::PI / 180.0;
    let (qw_h, qz_h) = ((oyaw_rad / 2.0).cos(), (oyaw_rad / 2.0).sin());
    println!("Origin: x={ox:+.3} y={oy:+.3} yaw={:.1}°", oyaw_rad.to_degrees());

    println!("Takeoff in 3 s...");
    for _ in 0..15usize {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        sleep(Duration::from_millis(200)).await;
    }

    let mut rows: Vec<Row> = Vec::new();
    let flight_start = Instant::now();
    let mut last_print = Instant::now();
    let mut prev_gyro_x = 0.0_f32;
    let mut prev_gyro_y = 0.0_f32;
    let mut prev_gyro_z = 0.0_f32;
    let mut prev_t_ms = 0u64;

    // ── Ramp up ───────────────────────────────────────────────────────────
    println!("=== RAMP UP ===");
    for step in 0..25usize {
        let h = 0.02 + step as f32 * ((HOVER_HEIGHT - 0.02) / 24.0);
        cf.commander.setpoint_full_state(
            ox, oy, h, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw_h, 0.0, 0.0, qz_h, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(120)).await;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
    }

    // ── Hover ─────────────────────────────────────────────────────────────
    println!("=== HOVER at {HOVER_HEIGHT:.2} m for {hover_secs} s ===");
    let hover_end = Instant::now() + Duration::from_secs(hover_secs);
    while Instant::now() < hover_end {
        cf.commander.setpoint_full_state(
            ox, oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw_h, 0.0, 0.0, qz_h, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(40)).await; // 25 Hz

        let pa = stream_a.next().await?;
        let pb = stream_b.next().await?;

        let t_ms = flight_start.elapsed().as_millis() as u64;
        let dt_s = if prev_t_ms == 0 { 0.04 } else { (t_ms - prev_t_ms) as f32 / 1000.0 };

        let gx = get_f32(&pb.data, "gyro.x");
        let gy = get_f32(&pb.data, "gyro.y");
        let gz = get_f32(&pb.data, "gyro.z");
        let deg2rad = std::f32::consts::PI / 180.0;

        // Reconstruct torque: τ_meas = J · dω/dt  [µNm]
        let alpha_x = (gx - prev_gyro_x) * deg2rad / dt_s;
        let alpha_y = (gy - prev_gyro_y) * deg2rad / dt_s;
        let alpha_z = (gz - prev_gyro_z) * deg2rad / dt_s;
        let tau_x_unm = JXX * alpha_x * 1e6;  // Nm → µNm
        let tau_y_unm = JYY * alpha_y * 1e6;
        let tau_z_unm = JZZ * alpha_z * 1e6;

        let row = Row {
            time_ms:   t_ms,
            pos_x:     get_f32(&pa.data, "stateEstimate.x"),
            pos_y:     get_f32(&pa.data, "stateEstimate.y"),
            pos_z:     get_f32(&pa.data, "stateEstimate.z"),
            vel_x:     get_f32(&pa.data, "stateEstimate.vx"),
            vel_y:     get_f32(&pa.data, "stateEstimate.vy"),
            vel_z:     get_f32(&pa.data, "stateEstimate.vz"),
            roll:      get_f32(&pa.data, "stabilizer.roll"),
            pitch:     get_f32(&pa.data, "stabilizer.pitch"),
            yaw:       get_f32(&pa.data, "stabilizer.yaw"),
            thrust_fw: get_f32(&pa.data, "stabilizer.thrust"),
            vbat:      get_f32(&pb.data, "pm.vbat"),
            gyro_x:    gx, gyro_y: gy, gyro_z: gz,
            sp_qw: qw_h, sp_qx: 0.0, sp_qy: 0.0, sp_qz: qz_h,
            sp_z: HOVER_HEIGHT,
            tau_meas_x: tau_x_unm,
            tau_meas_y: tau_y_unm,
            tau_meas_z: tau_z_unm,
        };

        if last_print.elapsed() > Duration::from_secs(1) {
            let drift = ((row.pos_x - ox).powi(2) + (row.pos_y - oy).powi(2)).sqrt();
            println!("  [{:.0}s] z={:.3}m roll={:.1}° pitch={:.1}° drift={:.3}m \
                      τx={:.0}µNm τy={:.0}µNm m1={} {:.2}V",
                flight_start.elapsed().as_secs_f32(),
                row.pos_z, row.roll, row.pitch, drift,
                row.tau_meas_x, row.tau_meas_y,
                get_f32(&pb.data, m1_var) as i32,
                row.vbat);
            last_print = Instant::now();
        }

        prev_gyro_x = gx; prev_gyro_y = gy; prev_gyro_z = gz;
        prev_t_ms = t_ms;
        rows.push(row);
    }

    // ── Land ──────────────────────────────────────────────────────────────
    println!("=== LAND ===");
    for step in (0..20usize).rev() {
        let h = 0.04 + step as f32 * ((HOVER_HEIGHT - 0.04) / 19.0);
        cf.commander.setpoint_full_state(
            ox, oy, h, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw_h, 0.0, 0.0, qz_h, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(150)).await;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
    }
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    sleep(Duration::from_millis(500)).await;

    // ── Write CSV ─────────────────────────────────────────────────────────
    let ts = Utc::now().format("%Y-%m-%d_%H-%M-%S");
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/bin/runs");
    std::fs::create_dir_all(&dir)?;
    let path = dir.join(format!("indi_hover_{ts}.csv"));
    let mut f = File::create(&path)?;
    writeln!(f, "time_ms,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,\
                 roll,pitch,yaw,thrust_fw,vbat,\
                 gyro_x,gyro_y,gyro_z,\
                 sp_qw,sp_qx,sp_qy,sp_qz,sp_z,\
                 tau_meas_x_unm,tau_meas_y_unm,tau_meas_z_unm")?;
    for r in &rows {
        writeln!(f,
            "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},\
             {:.3},{:.3},{:.3},{:.1},{:.3},\
             {:.3},{:.3},{:.3},\
             {:.6},{:.6},{:.6},{:.6},{:.3},\
             {:.2},{:.2},{:.2}",
            r.time_ms,
            r.pos_x, r.pos_y, r.pos_z, r.vel_x, r.vel_y, r.vel_z,
            r.roll, r.pitch, r.yaw, r.thrust_fw, r.vbat,
            r.gyro_x, r.gyro_y, r.gyro_z,
            r.sp_qw, r.sp_qx, r.sp_qy, r.sp_qz, r.sp_z,
            r.tau_meas_x, r.tau_meas_y, r.tau_meas_z,
        )?;
    }
    println!("CSV written: {}", path.display());

    // ── Post-flight summary ───────────────────────────────────────────────
    if rows.len() > 5 {
        let pos_err: Vec<f32> = rows.iter()
            .map(|r| ((r.pos_x - ox).powi(2) + (r.pos_y - oy).powi(2) + (r.pos_z - HOVER_HEIGHT).powi(2)).sqrt())
            .collect();
        let rmse = (pos_err.iter().map(|e| e*e).sum::<f32>() / pos_err.len() as f32).sqrt();
        let gyro_rms = (rows.iter().map(|r| r.gyro_x*r.gyro_x + r.gyro_y*r.gyro_y).sum::<f32>()
                        / rows.len() as f32).sqrt();
        let tau_rms = (rows.iter().map(|r| r.tau_meas_x*r.tau_meas_x + r.tau_meas_y*r.tau_meas_y).sum::<f32>()
                       / rows.len() as f32).sqrt();
        println!("\nPost-flight summary ({} rows):", rows.len());
        println!("  Position RMSE:   {rmse:.4} m");
        println!("  Gyro XY RMS:     {gyro_rms:.2} °/s  (< 30 °/s target)");
        println!("  τ_meas XY RMS:   {tau_rms:.1} µNm");
        if rmse < 0.10 { println!("  ✓ Position hold OK (< 10 cm)"); }
        else            { println!("  ✗ Position drift: {rmse:.3} m (> 10 cm limit)"); }
        if gyro_rms < 30.0 { println!("  ✓ Gyro noise OK"); }
        else                { println!("  ✗ Gyro noisy: {gyro_rms:.1} °/s — lower FC_GYRO_HZ"); }
    }

    println!("\nINDI hover test finished.");
    Ok(())
}

//! XY Steps Test — starts exactly like your working fw_test (0.5 m hover), then does XY steps
//!
//! Usage: cargo run --release --bin xy_steps -- --controller 6

use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod};
use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::sleep;

const CF_URI: &str = "radio://0/80/2M/E7E7E7E7E7";
const DEFAULT_CONTROLLER: u8 = 6;

async fn add_var(block: &mut LogBlock, name: &str) -> bool {
    match block.add_variable(name).await {
        Ok(_)  => { println!("  [log] added: {name}"); true }
        Err(e) => { println!("  [log] MISSING: {name} ({e})"); false }
    }
}

async fn add_var_first(block: &mut LogBlock, candidates: &[&'static str]) -> Option<&'static str> {
    for &name in candidates {
        match block.add_variable(name).await {
            Ok(_)  => { println!("  [log] added: {name} (from candidates)"); return Some(name); }
            Err(_) => {}
        }
    }
    println!("  [log] NONE of {:?} found in firmware TOC", candidates);
    None
}

fn get_f32(map: &HashMap<String, Value>, key: &str) -> f32 {
    map.get(key).map(|v| v.to_f64_lossy() as f32).unwrap_or(0.0)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let controller: u8 = args.iter()
        .position(|a| a == "--controller")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(DEFAULT_CONTROLLER);

    let ctrl_name = match controller {
        1 => "PID", 2 => "Mellinger", 3 => "INDI",
        4 => "Brescianini", 5 => "Lee (firmware)", 6 => "OOT Rust geometric",
        n => { eprintln!("Unknown controller type {n}"); return Ok(()); }
    };
    println!("Controller: {ctrl_name} (type {controller}) | Mode: XY Steps");

    let link_context = LinkContext::new();
    println!("Connecting to {CF_URI} ...");
    let cf = Crazyflie::connect_from_uri(&link_context, CF_URI, NoTocCache).await?;
    println!("Connected.");

    // ── Log block A ───────────────────────────────────────────────────────────
    println!("Setting up log block A (position/attitude/thrust)...");
    let mut block_a = cf.log.create_block().await?;
    for v in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stabilizer.yaw", "stabilizer.thrust"] {
        add_var(&mut block_a, v).await;
    }
    let stream_a = block_a.start(LogPeriod::from_millis(50)?).await?;

    // ── Log block B ───────────────────────────────────────────────────────────
    println!("Setting up log block B (ctrltarget/motor/supervisor)...");
    let mut block_b = cf.log.create_block().await?;
    add_var(&mut block_b, "ctrltarget.z").await;
    let m1_var = add_var_first(&mut block_b,
        &["motor.m1req", "motor.m1pwm", "motor.m1"]).await
        .unwrap_or("motor.m1req");
    add_var(&mut block_b, "sys.canfly").await;
    add_var(&mut block_b, "pm.vbat").await;
    let stream_b = block_b.start(LogPeriod::from_millis(50)?).await?;

    // ── Select controller ─────────────────────────────────────────────────────
    println!("Setting stabilizer.controller = {controller} ({ctrl_name})...");
    let _ = cf.param.set("stabilizer.controller", controller).await;
    sleep(Duration::from_millis(100)).await;

    // ── Kalman reset ──────────────────────────────────────────────────────────
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    let _ = cf.param.set("kalman.resetEstimation", 1u8).await;
    sleep(Duration::from_millis(200)).await;
    let _ = cf.param.set("kalman.resetEstimation", 0u8).await;
    println!("Kalman reset sent — place drone flat on pad, do NOT move it...");

    for _ in 0..20usize {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        sleep(Duration::from_millis(200)).await;
    }

    // ── Sample EKF origin ─────────────────────────────────────────────────────
    let mut xs: Vec<f32> = Vec::new();
    let mut ys: Vec<f32> = Vec::new();
    let mut yaws: Vec<f32> = Vec::new();
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

    let mean_x = xs.iter().sum::<f32>() / n as f32;
    let mean_y = ys.iter().sum::<f32>() / n as f32;
    let mean_yaw = yaws.iter().sum::<f32>() / n as f32;
    let spread = xs.iter().zip(ys.iter())
        .map(|(x, y)| ((x - mean_x).powi(2) + (y - mean_y).powi(2)).sqrt())
        .fold(0.0f32, f32::max);

    let (ox, oy, oyaw_deg) = if (mean_x * mean_x + mean_y * mean_y).sqrt() > 0.15 || spread > 0.05 {
        println!("WARNING: EKF origin unreliable, falling back to (0, 0)");
        (0.0f32, 0.0f32, mean_yaw)
    } else {
        (mean_x, mean_y, mean_yaw)
    };
    let oyaw_rad = oyaw_deg * std::f32::consts::PI / 180.0;
    println!("Using origin: x={ox:+.3} y={oy:+.3} yaw={oyaw_deg:+.1} deg");

    println!("Takeoff in 3 s...");
    for _ in 0..15usize {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        sleep(Duration::from_millis(200)).await;
    }

    let start = Instant::now();
    let mut last_print = Instant::now();

    // ── Ramp up to 0.50 m (identical) ───────────────────────────────────────
    println!("=== RAMP UP (0.02 -> 0.50 m) ===");
    for step in 0..25usize {
        let height = 0.02 + step as f32 * (0.48 / 24.0);
        cf.commander.setpoint_position(ox, oy, height, oyaw_rad).await?;
        sleep(Duration::from_millis(120)).await;

        if let (Ok(pa), Ok(pb)) = (stream_a.next().await, stream_b.next().await) {
            let ekf_z  = get_f32(&pa.data, "stateEstimate.z");
            let ekf_x  = get_f32(&pa.data, "stateEstimate.x");
            let ekf_y  = get_f32(&pa.data, "stateEstimate.y");
            let fw_thr = get_f32(&pa.data, "stabilizer.thrust") as u32;
            let cz     = get_f32(&pb.data, "ctrltarget.z");
            let m1     = get_f32(&pb.data, m1_var) as i32;
            let canfly = get_f32(&pb.data, "sys.canfly") as u8;
            let vbat   = get_f32(&pb.data, "pm.vbat");

            if last_print.elapsed() >= Duration::from_millis(400) {
                println!("  [ramp {:.1}s] sp_z={:.2} z={:+.3} xy=({:+.3},{:+.3}) cz={:+.3} m1={:6} fw_thr={} cf={} {:.2}V",
                    start.elapsed().as_secs_f32(), height, ekf_z, ekf_x, ekf_y, cz, m1, fw_thr, canfly, vbat);
                last_print = Instant::now();
            }
        }
    }

    // ── Hover at 0.50 m for 15 s (identical) ─────────────────────────────────
    println!("=== HOVER at 0.50 m for 15 s ===");
    let hover_end = Instant::now() + Duration::from_secs(15);
    while Instant::now() < hover_end {
        cf.commander.setpoint_position(ox, oy, 0.50, oyaw_rad).await?;
        sleep(Duration::from_millis(50)).await;

        if let (Ok(pa), Ok(pb)) = (stream_a.next().await, stream_b.next().await) {
            let ekf_z  = get_f32(&pa.data, "stateEstimate.z");
            let ekf_x  = get_f32(&pa.data, "stateEstimate.x");
            let ekf_y  = get_f32(&pa.data, "stateEstimate.y");
            let fw_thr = get_f32(&pa.data, "stabilizer.thrust") as u32;
            let cz     = get_f32(&pb.data, "ctrltarget.z");
            let m1     = get_f32(&pb.data, m1_var) as i32;
            let canfly = get_f32(&pb.data, "sys.canfly") as u8;
            let vbat   = get_f32(&pb.data, "pm.vbat");

            if last_print.elapsed() >= Duration::from_secs(1) {
                let drift = ((ekf_x - ox).powi(2) + (ekf_y - oy).powi(2)).sqrt();
                println!("  [hover {:3}s] z={:+.3} drift={:.3}m cz={:+.3} m1={:6} fw_thr={} cf={} {:.2}V",
                    start.elapsed().as_secs(), ekf_z, drift, cz, m1, fw_thr, canfly, vbat);
                last_print = Instant::now();
            }
        }
    }

    // ── XY Steps (new part) ──────────────────────────────────────────────────
    println!("=== XY Steps Test (20 cm steps) ===");
    let step = 0.20_f32;

    for axis in ["X", "Y"] {
        for &delta in &[-step, step, -step] {
            let tx = if axis == "X" { ox + delta } else { ox };
            let ty = if axis == "Y" { oy + delta } else { oy };
            println!("Moving {} by {:.2} m", axis, delta);

            let end = Instant::now() + Duration::from_secs(6);
            while Instant::now() < end {
                cf.commander.setpoint_position(tx, ty, 0.50, oyaw_rad).await?;
                sleep(Duration::from_millis(100)).await;
            }
        }
    }

    // ── Return to center and land (identical) ────────────────────────────────
    println!("Returning to center...");
    for _ in 0..40 {
        cf.commander.setpoint_position(ox, oy, 0.50, oyaw_rad).await?;
        sleep(Duration::from_millis(100)).await;
    }

    println!("=== LAND ===");
    for step in (0..20usize).rev() {
        let height = 0.04 + step as f32 * (0.46 / 19.0);
        cf.commander.setpoint_position(ox, oy, height, oyaw_rad).await?;
        sleep(Duration::from_millis(150)).await;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
    }

    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    sleep(Duration::from_millis(500)).await;

    println!("XY Steps Test finished.");

    Ok(())
}
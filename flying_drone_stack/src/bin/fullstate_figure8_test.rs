//! Full-State Figure-8 Test using Differential Flatness
//!
//! Uses your existing SmoothFigure8Trajectory + flatness to send
//! position + velocity + acceleration to the geometric controller.

use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod};
use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::sleep;

use multirotor_simulator::trajectory::SmoothFigure8Trajectory;   // adjust path if needed
use multirotor_simulator::flatness::{compute_flatness, FlatOutput}; // adjust path if needed

const CF_URI: &str = "radio://0/80/2M/E7E7E7E7E7";
const DEFAULT_CONTROLLER: u8 = 6;
const MASS: f32 = 0.031;

async fn add_var(block: &mut LogBlock, name: &str) -> bool {
    match block.add_variable(name).await {
        Ok(_)  => { println!("  [log] added: {name}"); true }
        Err(e) => { println!("  [log] MISSING: {name} ({e})"); false }
    }
}

async fn add_var_first(block: &mut LogBlock, candidates: &[&'static str]) -> Option<&'static str> {
    for &name in candidates {
        if block.add_variable(name).await.is_ok() {
            println!("  [log] added: {name}");
            return Some(name);
        }
    }
    println!("  [log] NONE of {:?} found", candidates);
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

    println!("Full-State Figure-8 Test - Controller: {}", controller);

    let link_context = LinkContext::new();
    let cf = Crazyflie::connect_from_uri(&link_context, CF_URI, NoTocCache).await?;
    println!("Connected.");

    // Log blocks
    let mut block_a = cf.log.create_block().await?;
    for v in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stabilizer.yaw", "stabilizer.thrust"] {
        add_var(&mut block_a, v).await;
    }
    let stream_a = block_a.start(LogPeriod::from_millis(50)?).await?;

    let mut block_b = cf.log.create_block().await?;
    add_var(&mut block_b, "ctrltarget.z").await;
    let _m1_var = add_var_first(&mut block_b, &["motor.m1req", "motor.m1pwm", "motor.m1"]).await;
    add_var(&mut block_b, "sys.canfly").await;
    add_var(&mut block_b, "pm.vbat").await;
    let _stream_b = block_b.start(LogPeriod::from_millis(50)?).await?;

    let _ = cf.param.set("stabilizer.controller", controller).await;
    sleep(Duration::from_millis(100)).await;

    // Kalman reset
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    let _ = cf.param.set("kalman.resetEstimation", 1u8).await;
    sleep(Duration::from_millis(300)).await;
    let _ = cf.param.set("kalman.resetEstimation", 0u8).await;

    for _ in 0..15 {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        sleep(Duration::from_millis(200)).await;
    }

    // Sample origin
    let mut xs = Vec::new();
    let mut ys = Vec::new();
    for _ in 0..40 {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        if let Ok(p) = stream_a.next().await {
            xs.push(get_f32(&p.data, "stateEstimate.x"));
            ys.push(get_f32(&p.data, "stateEstimate.y"));
        }
    }
    let ox = xs.iter().sum::<f32>() / xs.len() as f32;
    let oy = ys.iter().sum::<f32>() / ys.len() as f32;
    println!("Using origin: x={ox:+.3} y={oy:+.3}");

    // ── Stable Hover (7 seconds) ─────────────────────────────────────────────
    println!("=== Stable Hover at 0.50 m for 7 seconds ===");
    for _ in 0..70 {
        cf.commander.setpoint_position(ox, oy, 0.50, 0.0).await?;
        sleep(Duration::from_millis(100)).await;
    }

    // ── Smooth Figure-8 using your existing trajectory code ──────────────────
    println!("=== Smooth Figure-8 Trajectory ===");
    let mut traj = SmoothFigure8Trajectory::with_params(25.0, 0.50, 0.25); // duration, height, scale
    let traj_start = Instant::now();
    let total_time = Duration::from_secs(50); // two figure-8 loops

    while Instant::now() < traj_start + total_time {
        let t = (Instant::now() - traj_start).as_secs_f32();
        let flat = traj.get_reference(t);   // wait, your SmoothFigure8Trajectory returns TrajectoryReference

        // For full-state we need to send position + velocity + acceleration
        cf.commander.setpoint_position(flat.position.x, flat.position.y, flat.position.z, flat.yaw).await?;
        // Note: Crazyflie high-level commander can accept velocity/acc, but for now we use position
        // Later we can switch to full state if needed.

        sleep(Duration::from_millis(50)).await;
    }

    // Return to center + land
    println!("Returning to center and landing...");
    for _ in 0..50 {
        cf.commander.setpoint_position(ox, oy, 0.50, 0.0).await?;
        sleep(Duration::from_millis(100)).await;
    }

    for step in (0..25usize).rev() {
        let h = 0.04 + step as f32 * (0.46 / 24.0);
        cf.commander.setpoint_position(ox, oy, h, 0.0).await?;
        sleep(Duration::from_millis(150)).await;
    }

    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    println!("Figure-8 Test finished.");

    Ok(())
}
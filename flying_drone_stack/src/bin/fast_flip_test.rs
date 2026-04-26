//! Fast Flip Test — 360° pitch flip in T=0.45 s via full-state setpoints at 25 Hz
//!
//! Same structure as flip_test.rs but with T_FLIP=0.45 s (peak ω ≈ 1876°/s vs 1205°/s).
//! At this speed motor lag is ~2.5% of the maneuver vs 1.4% at T=0.7 s, so geometric
//! feedforward becomes noticeably inaccurate — illustrates where INDI helps.
//!
//! Requires OOT firmware: cd firmware_app && make cload
//!
//! Usage: cargo run --release --bin fast_flip_test

use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod};
use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::sleep;

use multirotor_simulator::prelude::{SplineTrajectory, Waypoint, Vec3};

const CF_URI:        &str = "radio://0/80/2M/E7E7E7E7E7";
const CONTROLLER:    u8   = 6;
const HOVER_HEIGHT:  f32  = 1.0;   // m
const FLIP_HEIGHT:   f32  = 1.3;   // m — extra buffer for faster motion
const T_FLIP:        f32  = 0.45;  // s — total flip duration (2× faster than flip_test)

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

// ── Fast flip trajectory planning ──────────────────────────────────────────

/// Plan smooth 0→2π angle trajectory via QP (2 segments, z-axis = θ).
fn plan_flip() -> SplineTrajectory {
    use std::f32::consts::TAU;
    let t_seg = T_FLIP / 2.0;
    let waypoints = vec![
        Waypoint { pos: Vec3::new(0.0, 0.0, 0.0),       yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.0, 0.0, TAU / 2.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.0, 0.0, TAU),        yaw: 0.0 },
    ];
    SplineTrajectory::plan(&waypoints, &[t_seg, t_seg], false)
        .expect("Fast flip planning failed")
}

// ── Main ───────────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Fast Flip Test ===");
    println!("  HOVER_HEIGHT={HOVER_HEIGHT:.2}m  FLIP_HEIGHT={FLIP_HEIGHT:.2}m  T_FLIP={T_FLIP:.2}s");
    println!("  (2× faster than flip_test — demonstrates geometric feedforward accuracy limit)");
    println!("  Controller: OOT Rust geometric (type {CONTROLLER})");

    println!("Planning fast flip trajectory...");
    let traj = plan_flip();
    let peak_omega = (0..=1000)
        .map(|i| traj.eval(T_FLIP * i as f32 / 1000.0).vel.z.abs())
        .fold(0.0f32, f32::max);
    println!("  Peak omega: {:.0} deg/s ({:.2} rad/s)",
        peak_omega.to_degrees(), peak_omega);

    let link_context = LinkContext::new();
    println!("Connecting to {CF_URI}...");
    let cf = Crazyflie::connect_from_uri(&link_context, CF_URI, NoTocCache).await?;
    println!("Connected.");

    let mut block_a = cf.log.create_block().await?;
    for v in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw",
              "stabilizer.thrust"] {
        add_var(&mut block_a, v).await;
    }
    let stream_a = block_a.start(LogPeriod::from_millis(50)?).await?;

    let mut block_b = cf.log.create_block().await?;
    let m1_var = add_var_first(&mut block_b,
        &["motor.m1req", "motor.m1pwm", "motor.m1"]).await
        .unwrap_or("motor.m1req");
    add_var(&mut block_b, "sys.canfly").await;
    add_var(&mut block_b, "pm.vbat").await;
    let stream_b = block_b.start(LogPeriod::from_millis(50)?).await?;

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

    // Sample EKF origin
    let mut xs = Vec::new();
    let mut ys = Vec::new();
    let mut yaws = Vec::new();
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

    // ── Ramp up to HOVER_HEIGHT ───────────────────────────────────────────
    println!("=== RAMP UP (0.02 → {HOVER_HEIGHT:.2} m) ===");
    let start = Instant::now();
    let mut last_print = Instant::now();
    for step in 0..25usize {
        let h = 0.02 + step as f32 * ((HOVER_HEIGHT - 0.02) / 24.0);
        cf.commander.setpoint_full_state(
            ox, oy, h, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw_h, 0.0, 0.0, qz_h, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(120)).await;
        if let (Ok(pa), Ok(pb)) = (stream_a.next().await, stream_b.next().await) {
            if last_print.elapsed() > Duration::from_millis(400) {
                println!("  [ramp {:.1}s] sp={:.2} z={:.3} m1={} {:.2}V",
                    start.elapsed().as_secs_f32(), h,
                    get_f32(&pa.data, "stateEstimate.z"),
                    get_f32(&pb.data, m1_var) as i32,
                    get_f32(&pb.data, "pm.vbat"));
                last_print = Instant::now();
            }
        }
    }

    // ── Hover at HOVER_HEIGHT (stabilise) ────────────────────────────────
    println!("=== HOVER at {HOVER_HEIGHT:.2} m for 5 s ===");
    let hover_end = Instant::now() + Duration::from_secs(5);
    while Instant::now() < hover_end {
        cf.commander.setpoint_full_state(
            ox, oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw_h, 0.0, 0.0, qz_h, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(40)).await;
        if let (Ok(pa), Ok(_pb)) = (stream_a.next().await, stream_b.next().await) {
            if last_print.elapsed() > Duration::from_secs(1) {
                let drift = ((get_f32(&pa.data, "stateEstimate.x") - ox).powi(2)
                           + (get_f32(&pa.data, "stateEstimate.y") - oy).powi(2)).sqrt();
                println!("  [hover {:.0}s] z={:.3} drift={:.3}m",
                    start.elapsed().as_secs_f32(),
                    get_f32(&pa.data, "stateEstimate.z"), drift);
                last_print = Instant::now();
            }
        }
    }

    // ── Climb to FLIP_HEIGHT ──────────────────────────────────────────────
    println!("=== CLIMB to {FLIP_HEIGHT:.2} m ===");
    let climb_end = Instant::now() + Duration::from_secs(2);
    while Instant::now() < climb_end {
        cf.commander.setpoint_full_state(
            ox, oy, FLIP_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw_h, 0.0, 0.0, qz_h, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(40)).await;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
    }

    // ── Fast Flip ─────────────────────────────────────────────────────────
    println!("=== FAST FLIP: 0→360° in {T_FLIP:.2} s  (peak ω={:.0}°/s) ===",
        peak_omega.to_degrees());
    let flip_start = Instant::now();
    let flip_end_t = flip_start + Duration::from_secs_f32(T_FLIP);

    while Instant::now() < flip_end_t {
        let t = (Instant::now() - flip_start).as_secs_f32().min(T_FLIP);
        let flat = traj.eval(t);
        let theta = flat.pos.z;
        let omega = flat.vel.z;

        let qw = (theta / 2.0).cos();
        let qy = (theta / 2.0).sin();

        cf.commander.setpoint_full_state(
            ox, oy, FLIP_HEIGHT,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            qw, 0.0, -qy, 0.0,
            0.0, -omega, 0.0,
        ).await?;

        sleep(Duration::from_millis(40)).await;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
    }

    // ── Recovery hold ─────────────────────────────────────────────────────
    println!("Fast flip complete — recovery hold 1 s...");
    let rec_end = Instant::now() + Duration::from_secs(1);
    while Instant::now() < rec_end {
        cf.commander.setpoint_full_state(
            ox, oy, FLIP_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw_h, 0.0, 0.0, qz_h, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(40)).await;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
    }

    // ── Return to hover height ────────────────────────────────────────────
    println!("Returning to {HOVER_HEIGHT:.2} m...");
    let ret_end = Instant::now() + Duration::from_secs(2);
    while Instant::now() < ret_end {
        cf.commander.setpoint_full_state(
            ox, oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw_h, 0.0, 0.0, qz_h, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(40)).await;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
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

    println!("Fast flip test finished.");
    Ok(())
}

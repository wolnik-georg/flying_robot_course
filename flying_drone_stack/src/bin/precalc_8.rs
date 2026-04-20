//! Pre-calculated Polynomial Figure-8 Test — starts exactly like your working fw_test
//!
//! Usage: cargo run --release --bin precalc_figure8_test -- --controller 6

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

// Pre-calculated figure-8 polynomial coefficients (10 segments)
const FIGURE8_COEFFS: [[f32; 33]; 10] = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
];

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let controller: u8 = args.iter()
        .position(|a| a == "--controller")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(DEFAULT_CONTROLLER);

    let ctrl_name = match controller {
        5 => "Lee (firmware)", 6 => "OOT Rust geometric",
        n => { eprintln!("Unknown controller type {n}"); return Ok(()); }
    };
    println!("Controller: {ctrl_name} (type {controller}) | Mode: Precalculated Figure-8");

    let link_context = LinkContext::new();
    println!("Connecting to {CF_URI} ...");
    let cf = Crazyflie::connect_from_uri(&link_context, CF_URI, NoTocCache).await?;
    println!("Connected.");

    // Log blocks - identical to working version
    let mut block_a = cf.log.create_block().await?;
    for v in ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stabilizer.yaw", "stabilizer.thrust"] {
        add_var(&mut block_a, v).await;
    }
    let stream_a = block_a.start(LogPeriod::from_millis(50)?).await?;

    let mut block_b = cf.log.create_block().await?;
    add_var(&mut block_b, "ctrltarget.z").await;
    let m1_var = add_var_first(&mut block_b,
        &["motor.m1req", "motor.m1pwm", "motor.m1"]).await
        .unwrap_or("motor.m1req");
    add_var(&mut block_b, "sys.canfly").await;
    add_var(&mut block_b, "pm.vbat").await;
    let stream_b = block_b.start(LogPeriod::from_millis(50)?).await?;

    let _ = cf.param.set("stabilizer.controller", controller).await;
    sleep(Duration::from_millis(100)).await;

    // Kalman reset - identical
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
    let _ = cf.param.set("kalman.resetEstimation", 1u8).await;
    sleep(Duration::from_millis(200)).await;
    let _ = cf.param.set("kalman.resetEstimation", 0u8).await;
    println!("Kalman reset sent — place drone flat on pad, do NOT move it...");

    for _ in 0..20usize {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        sleep(Duration::from_millis(200)).await;
    }

    // Sample EKF origin - identical
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

    // ── Ramp + Hover — EXACTLY the same as your working fw_test ───────────────
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

    println!("=== HOVER at 0.50 m for 7 seconds ===");
    let hover_end = Instant::now() + Duration::from_secs(7);
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

    // ── Pre-calculated Polynomial Figure-8 ───────────────────────────────────
    println!("=== Pre-calculated Polynomial Figure-8 ===");

    let figure_start = Instant::now();
    let total_time = Duration::from_secs(35);   // one full figure-8

    while Instant::now() < figure_start + total_time {
        let t = (Instant::now() - figure_start).as_secs_f32();

        // Find current segment
        let mut seg_idx = 0;
        let mut seg_time = t;
        for (i, coeff) in FIGURE8_COEFFS.iter().enumerate() {
            let seg_duration = coeff[0];
            if seg_time <= seg_duration {
                seg_idx = i;
                break;
            }
            seg_time -= seg_duration;
        }

        let coeff = &FIGURE8_COEFFS[seg_idx];
        let seg_duration = coeff[0];
        let s = if seg_duration > 0.0 { seg_time / seg_duration } else { 0.0 };

        // Evaluate polynomials for x, y, z, yaw
        let mut x = 0.0f32;
        let mut y = 0.0f32;
        let mut z = 0.0f32;
        let mut yaw = 0.0f32;

        for i in 0..8 {
            let p = s.powi(i as i32);
            x += coeff[1 + i] * p;
            y += coeff[9 + i] * p;
            z += coeff[17 + i] * p;
            yaw += coeff[25 + i] * p;
        }

        cf.commander.setpoint_position(ox + x, oy + y, 0.50 + z, oyaw_rad + yaw).await?;
        sleep(Duration::from_millis(60)).await;
    }

    // Return to center + land
    println!("Returning to center...");
    for _ in 0..50 {
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

    println!("Pre-calculated Figure-8 Test finished.");

    Ok(())
}
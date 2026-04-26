//! Fast Helix Test — 2× speed ascending then descending helix via full-state setpoints at 25 Hz
//!
//! Analytic circle in XY + linear Z ramp → differential flatness → setpoint_full_state.
//! Two laps: first lap ascending (HOVER_HEIGHT → HOVER_HEIGHT + HELIX_DZ),
//!           second lap descending back to HOVER_HEIGHT.
//!
//! Circle is centred at (ox + HELIX_RADIUS, oy) so the drone starts on the
//! circle at its hover position (ox, oy) — no position jump at t=0.
//!
//! Requires OOT firmware: cd firmware_app && make cload
//!
//! Usage: cargo run --release --bin fast_helix_test

use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod};
use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::f32::consts::PI;
use std::time::{Duration, Instant};
use tokio::time::sleep;

use multirotor_simulator::planning::{compute_flatness, rot_to_quat, FlatOutput};
use multirotor_simulator::math::Vec3;

const CF_URI:        &str = "radio://0/80/2M/E7E7E7E7E7";
const CONTROLLER:    u8   = 6;
const HOVER_HEIGHT:  f32  = 0.8;   // m — base hover height
const HELIX_RADIUS:  f32  = 0.30;  // m — circle radius
const HELIX_DZ:      f32  = 0.40;  // m — height gained over one ascending lap
const LAP_TIME:      f32  =  5.25;  // s — one full circle (2× faster than helix_test)
const MASS:          f32  = 0.031; // kg — CF 2.1 + decks

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

// ── Main ───────────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let omega_c  = 2.0 * PI / LAP_TIME;   // circle angular velocity [rad/s]
    let vz_up    = HELIX_DZ / LAP_TIME;   // vertical speed [m/s] per lap

    println!("=== Fast Helix Test (2×) ===");
    println!("  r={HELIX_RADIUS:.2}m  dz={HELIX_DZ:.2}m  lap={LAP_TIME:.2}s  2 laps  (2× faster)");
    println!("  z: {HOVER_HEIGHT:.2}m → {:.2}m → {HOVER_HEIGHT:.2}m",
             HOVER_HEIGHT + HELIX_DZ);
    println!("  Max tilt ≈ {:.1}°",
             (HELIX_RADIUS * omega_c * omega_c / 9.81_f32).atan().to_degrees());

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
    let oyaw_rad = (yaws.iter().sum::<f32>() / n as f32) * PI / 180.0;
    let (qw_h, qz_h) = ((oyaw_rad / 2.0).cos(), (oyaw_rad / 2.0).sin());
    println!("Origin: x={ox:+.3} y={oy:+.3} yaw={:.1}°", oyaw_rad.to_degrees());

    // Circle centre: one radius ahead so drone starts on the circle at (ox, oy)
    let cx = ox + HELIX_RADIUS;
    let cy = oy;
    let phase0 = PI; // at phase=PI → cos(PI)=-1 → x = cx - r = ox ✓

    println!("Takeoff in 3 s...");
    for _ in 0..15usize {
        cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
        sleep(Duration::from_millis(200)).await;
    }

    // ── Ramp up ───────────────────────────────────────────────────────────
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

    // ── Hover (stabilise) ─────────────────────────────────────────────────
    println!("=== HOVER at {HOVER_HEIGHT:.2} m for 5 s ===");
    let hover_end = Instant::now() + Duration::from_secs(5);
    while Instant::now() < hover_end {
        cf.commander.setpoint_full_state(
            ox, oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw_h, 0.0, 0.0, qz_h, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(40)).await;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
    }

    // ── Helix (2 laps) ────────────────────────────────────────────────────
    println!("=== FAST HELIX: 2 laps × {LAP_TIME:.1} s  (ascending + descending) ===");
    let helix_start = Instant::now();
    let helix_total = 2.0 * LAP_TIME;

    while (Instant::now() - helix_start).as_secs_f32() < helix_total {
        let t = (Instant::now() - helix_start).as_secs_f32().min(helix_total);
        let phase = phase0 + omega_c * t;

        // Position
        let px = cx + HELIX_RADIUS * phase.cos();
        let py = cy + HELIX_RADIUS * phase.sin();
        let vz = if t < LAP_TIME { vz_up } else { -vz_up };
        let pz = if t <= LAP_TIME {
            HOVER_HEIGHT + vz_up * t
        } else {
            HOVER_HEIGHT + HELIX_DZ - vz_up * (t - LAP_TIME)
        };

        // Velocity (circle is analytic — exact derivatives)
        let vx = -HELIX_RADIUS * omega_c * phase.sin();
        let vy =  HELIX_RADIUS * omega_c * phase.cos();

        // Acceleration (centripetal only — no tangential for const ω)
        let ax = -HELIX_RADIUS * omega_c * omega_c * phase.cos();
        let ay = -HELIX_RADIUS * omega_c * omega_c * phase.sin();

        // Jerk
        let jx =  HELIX_RADIUS * omega_c.powi(3) * phase.sin();
        let jy = -HELIX_RADIUS * omega_c.powi(3) * phase.cos();

        let flat = FlatOutput {
            pos:     Vec3::new(px, py, pz),
            vel:     Vec3::new(vx, vy, vz),
            acc:     Vec3::new(ax, ay, 0.0),
            jerk:    Vec3::new(jx, jy, 0.0),
            snap:    Vec3::new(0.0, 0.0, 0.0),
            yaw:     oyaw_rad,
            yaw_dot: 0.0,
            yaw_ddot: 0.0,
        };

        let res = compute_flatness(&flat, MASS);
        let q   = rot_to_quat(&res.rot);

        cf.commander.setpoint_full_state(
            res.pos.x, res.pos.y, res.pos.z,
            res.vel.x, res.vel.y, res.vel.z,
            flat.acc.x, flat.acc.y, flat.acc.z,
            q[0], q[1], q[2], q[3],
            res.omega.x, res.omega.y, res.omega.z,
        ).await?;

        sleep(Duration::from_millis(40)).await; // 25 Hz

        if let (Ok(pa), Ok(_pb)) = (stream_a.next().await, stream_b.next().await) {
            if last_print.elapsed() > Duration::from_secs(2) {
                let drift = ((get_f32(&pa.data, "stateEstimate.x") - px).powi(2)
                           + (get_f32(&pa.data, "stateEstimate.y") - py).powi(2)).sqrt();
                println!("  [helix {:.0}s] z={:.3}/{:.3} roll={:.1}° pitch={:.1}° drift={:.3}m",
                    t,
                    get_f32(&pa.data, "stateEstimate.z"), pz,
                    get_f32(&pa.data, "stabilizer.roll"),
                    get_f32(&pa.data, "stabilizer.pitch"),
                    drift);
                last_print = Instant::now();
            }
        }
    }

    // ── Hold + Land ───────────────────────────────────────────────────────
    println!("Helix complete — hold 3 s at {HOVER_HEIGHT:.2} m...");
    let hold_end = Instant::now() + Duration::from_secs(3);
    while Instant::now() < hold_end {
        cf.commander.setpoint_full_state(
            ox, oy, HOVER_HEIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            qw_h, 0.0, 0.0, qz_h, 0.0, 0.0, 0.0,
        ).await?;
        sleep(Duration::from_millis(40)).await;
        let _ = stream_a.next().await;
        let _ = stream_b.next().await;
    }

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

    println!("Helix test finished.");
    Ok(())
}

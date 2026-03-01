use crazyflie_lib::{Crazyflie, NoTocCache};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod, LogStream};
use crazyflie_link::LinkContext;
use std::time::{Duration, Instant};
use std::fs::{self, File};
use std::io::Write;
use tokio::time::sleep;
use chrono::Utc;

#[derive(Debug)]
struct LogEntry {
    time_ms: u64,
    x: f32,
    y: f32,
    z: f32,
    vx: f32,
    vy: f32,
    vz: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
    thrust: u32,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let link_context = LinkContext::new();
    let uri = "radio://0/80/2M/E7E7E7E7E7";

    println!("Connecting to {} ...", uri);
    let cf = Crazyflie::connect_from_uri(&link_context, uri, NoTocCache).await?;
    println!("Connected!");

    // ────────────────────────────────────────────────
    // Extended logging (limited to 9 variables to avoid error 7)
    // ────────────────────────────────────────────────
    let mut block: LogBlock = cf.log.create_block().await?;

    let variables = vec![
        "stateEstimate.x",
        "stateEstimate.y",
        "stateEstimate.z",
        "stateEstimate.vx",
        "stateEstimate.vy",
        "stateEstimate.vz",
        "stabilizer.roll",
        "stabilizer.pitch",
        "stabilizer.yaw",
        // "stabilizer.thrust", // ← commented out – add back if needed and block still fits
    ];

    for var in variables {
        match block.add_variable(var).await {
            Ok(_) => println!("Added log variable: {}", var),
            Err(e) => eprintln!("Failed to add {}: {}", var, e),
        }
    }

    let period = LogPeriod::from_millis(50)?; // 20 Hz
    let stream: LogStream = block.start(period).await?;

    // Prepare data collection
    let mut log_data: Vec<LogEntry> = Vec::new();
    let mut last_print = Instant::now();

    println!("Logging started (20 Hz). Starting hover maneuver in 3 seconds...");
    sleep(Duration::from_secs(3)).await;

    // ────────────────────────────────────────────────
    // Hover maneuver
    // ────────────────────────────────────────────────
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0).await?;
    sleep(Duration::from_millis(200)).await;

    println!("Ramping up...");
    for y in 0..15 {
        let zdistance = y as f32 / 50.0; // max 0.3 m
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(150)).await;
    }

    println!("Hovering at 0.3 m for 12 seconds...");
    let hover_start = Instant::now();
    while hover_start.elapsed() < Duration::from_secs(12) {
        if let Ok(data_packet) = stream.next().await {
            let data = &data_packet.data;

            let x     = data.get("stateEstimate.x")    .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let y     = data.get("stateEstimate.y")    .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let z     = data.get("stateEstimate.z")    .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let vx    = data.get("stateEstimate.vx")   .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let vy    = data.get("stateEstimate.vy")   .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let vz    = data.get("stateEstimate.vz")   .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let roll  = data.get("stabilizer.roll")    .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let pitch = data.get("stabilizer.pitch")   .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let yaw   = data.get("stabilizer.yaw")     .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let thrust = data.get("stabilizer.thrust") .and_then(|v| u32::try_from(*v).ok()).unwrap_or(0);

            log_data.push(LogEntry {
                time_ms: hover_start.elapsed().as_millis() as u64,
                x, y, z, vx, vy, vz, roll, pitch, yaw, thrust,
            });

            // Print summary every ~1 second
            if last_print.elapsed() >= Duration::from_secs(1) {
                println!(
                    "[t={:3}s] z={:+5.3} m  vx={:+5.3} vy={:+5.3}  thrust={:5}",
                    hover_start.elapsed().as_secs(),
                    z, vx, vy, thrust
                );
                last_print = Instant::now();
            }
        }

        cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.3).await?;
        sleep(Duration::from_millis(50)).await;
    }

    // Gentle ramp down
    println!("Ramping down gently...");
    for y in (0..40).rev() {
        let zdistance = y as f32 / 100.0;
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(120)).await;
    }

    cf.commander.setpoint_stop().await?;
    cf.commander.notify_setpoint_stop(500).await?;

    println!("Maneuver complete. Motors stopped.");

    // ────────────────────────────────────────────────
    // Save to CSV
    // ────────────────────────────────────────────────
    let maneuver_type = "hover";

    let timestamp = Utc::now().format("%Y-%m-%d_%H-%M-%S");
    let filename = format!("runs/{}_{}.csv", maneuver_type, timestamp);

    fs::create_dir_all("runs")?;
    let mut file = File::create(&filename)?;

    writeln!(file, "time_ms,x,y,z,vx,vy,vz,roll,pitch,yaw,thrust")?;

    for entry in log_data {
        writeln!(file,
            "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{}",
            entry.time_ms,
            entry.x, entry.y, entry.z,
            entry.vx, entry.vy, entry.vz,
            entry.roll, entry.pitch, entry.yaw,
            entry.thrust
        )?;
    }

    println!("Log saved to: {}", filename);

    drop(stream);
    cf.disconnect().await;

    println!("Disconnected cleanly.");
    Ok(())
}
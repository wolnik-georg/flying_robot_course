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
    x: f32, y: f32, z: f32,
    vx: f32, vy: f32, vz: f32,
    roll: f32, pitch: f32, yaw: f32,
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
    // Log block 1: Core pose + velocity + thrust
    // ────────────────────────────────────────────────
    let mut block1: LogBlock = cf.log.create_block().await?;
    let block1_vars = vec![
        "stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
        "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz",
        "stabilizer.thrust",
    ];
    for var in block1_vars {
        if let Err(e) = block1.add_variable(var).await {
            eprintln!("Failed to add to block1 {}: {}", var, e);
        } else {
            println!("Added to block1: {}", var);
        }
    }
    let period = LogPeriod::from_millis(50)?;
    let stream1: LogStream = block1.start(period).await?;

    // ────────────────────────────────────────────────
    // Log block 2: Attitude + extras (recreate period since moved)
    // ────────────────────────────────────────────────
    let mut block2: LogBlock = cf.log.create_block().await?;
    let block2_vars = vec![
        "stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw",
        // Add more here as needed – try one by one
        // "pm.vbat",
        // "gyro.x", "gyro.y", "gyro.z",
    ];
    for var in block2_vars {
        if let Err(e) = block2.add_variable(var).await {
            eprintln!("Failed to add to block2 {}: {}", var, e);
        } else {
            println!("Added to block2: {}", var);
        }
    }
    let period2 = LogPeriod::from_millis(50)?; // recreate
    let stream2: LogStream = block2.start(period2).await?;

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
        let zdistance = y as f32 / 50.0;
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(150)).await;
    }

    println!("Hovering at 0.3 m for 12 seconds...");
    let hover_start = Instant::now();
    while hover_start.elapsed() < Duration::from_secs(12) {
        // Read block 1
        if let Ok(data1) = stream1.next().await {
            let d = &data1.data;

            let x     = d.get("stateEstimate.x")    .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let y     = d.get("stateEstimate.y")    .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let z     = d.get("stateEstimate.z")    .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let vx    = d.get("stateEstimate.vx")   .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let vy    = d.get("stateEstimate.vy")   .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let vz    = d.get("stateEstimate.vz")   .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0);
            let thrust = d.get("stabilizer.thrust") .and_then(|v| u32::try_from(*v).ok()).unwrap_or(0);

            // Read block 2 (attitude)
            let (roll, pitch, yaw) = if let Ok(data2) = stream2.next().await {
                let d2 = &data2.data;
                (
                    d2.get("stabilizer.roll") .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0),
                    d2.get("stabilizer.pitch").and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0),
                    d2.get("stabilizer.yaw")  .and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0),
                )
            } else {
                (0.0, 0.0, 0.0)
            };

            log_data.push(LogEntry {
                time_ms: hover_start.elapsed().as_millis() as u64,
                x, y, z, vx, vy, vz, roll, pitch, yaw, thrust,
            });

            // Print summary every second
            if last_print.elapsed() >= Duration::from_secs(1) {
                println!(
                    "[t={:3}s] z={:+5.3} vx={:+5.3} vy={:+5.3} thrust={:5} roll={:+5.1} pitch={:+5.1}",
                    hover_start.elapsed().as_secs(),
                    z, vx, vy, thrust, roll, pitch
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

    drop(stream1);
    drop(stream2);
    cf.disconnect().await;

    println!("Disconnected cleanly.");
    Ok(())
}
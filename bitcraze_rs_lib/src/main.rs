use crazyflie_lib::{Crazyflie, NoTocCache};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod, LogStream};
use crazyflie_link::LinkContext;
use std::time::{Duration, Instant};
use std::fs::{self, File};
use std::io::Write;
use tokio::time::sleep;
use chrono::Utc;

// CHANGE THIS TO SWITCH MANEUVER
// "hover" | "circle" | "figure8"
const MANEUVER: &str = "figure8";

#[derive(Debug, Default)]
struct LogEntry {
    time_ms: u64,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
    thrust: u32,
    vbat: f32,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
    rate_roll: f32,
    rate_pitch: f32,
    rate_yaw: f32,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let link_context = LinkContext::new();
    let uri = "radio://0/80/2M/E7E7E7E7E7";

    println!("Connecting to {} ...", uri);
    let cf = Crazyflie::connect_from_uri(&link_context, uri, NoTocCache).await?;
    println!("Connected!");

    // Block 1: Position, velocity, thrust
    let mut block1 = cf.log.create_block().await?;
    let vars1 = vec![
        "stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
        "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz",
        "stabilizer.thrust",
    ];
    for v in vars1 { add_var(&mut block1, v).await; }
    let stream1 = block1.start(LogPeriod::from_millis(50)?).await?;

    // Block 2: Attitude + body rates
    let mut block2 = cf.log.create_block().await?;
    let vars2 = vec![
        "stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw",
        "rateRoll", "ratePitch", "rateYaw",
    ];
    for v in vars2 { add_var(&mut block2, v).await; }
    let stream2 = block2.start(LogPeriod::from_millis(50)?).await?;

    // Block 3: Battery + raw sensors
    let mut block3 = cf.log.create_block().await?;
    let vars3 = vec![
        "pm.vbat",
        "gyro.x", "gyro.y", "gyro.z",
        "acc.x", "acc.y", "acc.z",
    ];
    for v in vars3 { add_var(&mut block3, v).await; }
    let stream3 = block3.start(LogPeriod::from_millis(50)?).await?;

    let mut log_data: Vec<LogEntry> = Vec::new();
    let mut last_print = Instant::now();

    println!("Logging started (20 Hz). Starting maneuver '{}' in 3 seconds...", MANEUVER);
    sleep(Duration::from_secs(3)).await;

    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0).await?;
    sleep(Duration::from_millis(200)).await;

    println!("Ramping up...");
    for y in 0..15 {
        let zdistance = y as f32 / 50.0;
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(150)).await;
    }

    match MANEUVER {
        "hover" => {
            println!("Hovering at 0.3 m for 12 seconds...");
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(12) {
                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start).await;
                cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.3).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }
        "circle" => {
            let radius = 0.25;
            let height = 0.3;
            let omega = 0.6;

            println!("Circle: radius {:.2} m, height {:.2} m, ω = {:.2} rad/s", radius, height, omega);

            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(30) {
                let t = start.elapsed().as_secs_f32();
                let vx = -radius * omega * (omega * t).sin();
                let vy = radius * omega * (omega * t).cos();

                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start).await;
                cf.commander.setpoint_hover(vx, vy, 0.0, height).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }
        "figure8" => {
            let a = 0.25;
            let b = 0.15;
            let omega = 0.5;

            println!("Figure-8: a={:.2}, b={:.2}, ω={:.2}", a, b, omega);

            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(40) {
                let t = start.elapsed().as_secs_f32();
                let vx = -a * omega * (omega * t).sin();
                let vy = b * omega * (2.0 * omega * t).cos();

                run_logging_step(&mut log_data, &mut last_print, &stream1, &stream2, &stream3, &start).await;
                cf.commander.setpoint_hover(vx, vy, 0.0, 0.3).await?;
                sleep(Duration::from_millis(50)).await;
            }
        }
        _ => println!("Unknown maneuver. Running hover."),
    }

    println!("Ramping down gently...");
    for y in (0..40).rev() {
        let zdistance = y as f32 / 100.0;
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(120)).await;
    }

    cf.commander.setpoint_stop().await?;
    cf.commander.notify_setpoint_stop(500).await?;

    println!("Maneuver complete. Motors stopped.");

    let timestamp = Utc::now().format("%Y-%m-%d_%H-%M-%S");
    let filename = format!("runs/{}_{}.csv", MANEUVER, timestamp);

    fs::create_dir_all("runs")?;
    let mut file = File::create(&filename)?;

    writeln!(file,
        "time_ms,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll,pitch,yaw,thrust,vbat,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,rate_roll,rate_pitch,rate_yaw"
    )?;

    for e in log_data {
        writeln!(file,
            "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}",
            e.time_ms,
            e.pos_x, e.pos_y, e.pos_z,
            e.vel_x, e.vel_y, e.vel_z,
            e.roll, e.pitch, e.yaw,
            e.thrust,
            e.vbat,
            e.gyro_x, e.gyro_y, e.gyro_z,
            e.acc_x, e.acc_y, e.acc_z,
            e.rate_roll, e.rate_pitch, e.rate_yaw
        )?;
    }

    println!("Log saved to: {}", filename);

    drop(stream1);
    drop(stream2);
    drop(stream3);
    cf.disconnect().await;

    println!("Disconnected cleanly.");
    Ok(())
}

async fn run_logging_step(
    log_data: &mut Vec<LogEntry>,
    last_print: &mut Instant,
    stream1: &LogStream,
    stream2: &LogStream,
    stream3: &LogStream,
    start: &Instant,
) {
    let mut entry = LogEntry {
        time_ms: start.elapsed().as_millis() as u64,
        ..Default::default()
    };

    if let Ok(p) = stream1.next().await {
        let d = &p.data;
        entry.pos_x = get_f32(d, "stateEstimate.x");
        entry.pos_y = get_f32(d, "stateEstimate.y");
        entry.pos_z = get_f32(d, "stateEstimate.z");
        entry.vel_x = get_f32(d, "stateEstimate.vx");
        entry.vel_y = get_f32(d, "stateEstimate.vy");
        entry.vel_z = get_f32(d, "stateEstimate.vz");
        entry.thrust = get_u32(d, "stabilizer.thrust");
    }

    if let Ok(p) = stream2.next().await {
        let d = &p.data;
        entry.roll = get_f32(d, "stabilizer.roll");
        entry.pitch = get_f32(d, "stabilizer.pitch");
        entry.yaw = get_f32(d, "stabilizer.yaw");
        entry.rate_roll = get_f32(d, "rateRoll");
        entry.rate_pitch = get_f32(d, "ratePitch");
        entry.rate_yaw = get_f32(d, "rateYaw");
    }

    if let Ok(p) = stream3.next().await {
        let d = &p.data;
        entry.vbat = get_f32(d, "pm.vbat");
        entry.gyro_x = get_f32(d, "gyro.x");
        entry.gyro_y = get_f32(d, "gyro.y");
        entry.gyro_z = get_f32(d, "gyro.z");
        entry.acc_x = get_f32(d, "acc.x");
        entry.acc_y = get_f32(d, "acc.y");
        entry.acc_z = get_f32(d, "acc.z");
    }

    if last_print.elapsed() >= Duration::from_secs(1) {
        println!(
            "[t={:3}s] z={:+5.3} vx={:+5.3} vy={:+5.3} thrust={:5} roll={:+5.1} vbat={:.2}",
            start.elapsed().as_secs(),
            entry.pos_z, entry.vel_x, entry.vel_y, entry.thrust, entry.roll, entry.vbat
        );
        *last_print = Instant::now();
    }

    log_data.push(entry);
}

async fn add_var(block: &mut LogBlock, name: &str) {
    match block.add_variable(name).await {
        Ok(_) => println!("Added: {}", name),
        Err(e) => eprintln!("Failed to add {}: {}", name, e),
    }
}

fn get_f32(map: &std::collections::HashMap<String, crazyflie_lib::Value>, key: &str) -> f32 {
    map.get(key).and_then(|v| f32::try_from(*v).ok()).unwrap_or(0.0)
}

fn get_u32(map: &std::collections::HashMap<String, crazyflie_lib::Value>, key: &str) -> u32 {
    map.get(key).and_then(|v| u32::try_from(*v).ok()).unwrap_or(0)
}
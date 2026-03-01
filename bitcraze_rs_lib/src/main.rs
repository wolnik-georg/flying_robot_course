use crazyflie_lib::{Crazyflie, NoTocCache};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod, LogStream};
use crazyflie_link::LinkContext;
use std::time::Duration;
use tokio::time::{sleep};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let link_context = LinkContext::new();
    let uri = "radio://0/80/2M/E7E7E7E7E7";

    println!("Connecting to {} ...", uri);
    let cf = Crazyflie::connect_from_uri(&link_context, uri, NoTocCache).await?;
    println!("Connected!");

    // ────────────────────────────────────────────────
    // Optional: start logging during the whole maneuver
    // ────────────────────────────────────────────────
    let mut block: LogBlock = cf.log.create_block().await?;
    block.add_variable("stateEstimate.x").await?;
    block.add_variable("stateEstimate.y").await?;
    block.add_variable("stateEstimate.z").await?;
    block.add_variable("stabilizer.thrust").await?;

    let period = LogPeriod::from_millis(100)?;
    let stream: LogStream = block.start(period).await?;

    println!("Logging started. Starting hover maneuver in 3 seconds...");
    sleep(Duration::from_secs(3)).await;

    // ────────────────────────────────────────────────
    // From the repo example — low-level hover control
    // ────────────────────────────────────────────────

    // Unlock / prepare thrust
    cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0).await?;
    sleep(Duration::from_millis(100)).await;

    println!("Ramping up to hover...");

    // Ramp up height slowly (0 → 0.4 m)
    for y in 0..10 {
        let zdistance = y as f32 / 25.0; // 0.0 → 0.4 m
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(100)).await;
    }

    // Hover at 0.4 m for ~2 seconds
    println!("Hovering at 0.4 m...");
    for _ in 0..100 {
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.4).await?;
        sleep(Duration::from_millis(100)).await;
    }

    // Optional: small motion example (comment out if you want pure hover)
    /*
    println!("Moving forward + yaw left...");
    for _ in 0..50 {
        cf.commander.setpoint_hover(0.2, 0.0, 72.0, 0.4).await?;
        sleep(Duration::from_millis(100)).await;
    }
    */

    // Ramp down slowly
    // Ramp down to 0.05 m first
    for y in (5..40).rev() {   // from 0.39 down to 0.05
        let zdistance = y as f32 / 100.0;
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, zdistance).await?;
        sleep(Duration::from_millis(120)).await;
    }

    // Very slow final descent from 5 cm to 0
    println!("Final soft touchdown...");
    for _ in 0..10 {
        cf.commander.setpoint_hover(0.0, 0.0, 0.0, 0.05).await?;  // keep telling 5 cm
        sleep(Duration::from_millis(100)).await;
    }

    // Then stop
    cf.commander.setpoint_stop().await?;
    cf.commander.notify_setpoint_stop(500).await?;  // longer grace period

    println!("Hover maneuver complete. Motors stopped.");

    // ────────────────────────────────────────────────
    // Optional: stop logging & print final stats
    // ────────────────────────────────────────────────
    drop(stream); // stops logging when dropped
    cf.disconnect().await;

    println!("Disconnected cleanly.");
    Ok(())
}
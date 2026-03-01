use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod, LogStream};
use crazyflie_link::LinkContext;
use std::time::Duration;
use tokio::time::{sleep, Instant};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let link_context = LinkContext::new();

    let uri = "radio://0/80/2M/E7E7E7E7E7";

    println!("Connecting to {} ...", uri);

    let cf = Crazyflie::connect_from_uri(
        &link_context,
        uri,
        NoTocCache,
    )
    .await?;

    println!("Connected!");

    let mut block: LogBlock = cf.log.create_block().await?;

    block.add_variable("stateEstimate.x").await?;
    block.add_variable("stateEstimate.y").await?;
    block.add_variable("stateEstimate.z").await?;
    block.add_variable("stabilizer.thrust").await?;

    let period = LogPeriod::from_millis(100)?;

    let stream: LogStream = block.start(period).await?;

    println!("Logging live state for ~10 seconds... (try gently lifting the drone by hand)");

    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(10) {
        match stream.next().await {
            Ok(data) => {
                let x: f32 = data.data.get("stateEstimate.x")
                    .and_then(|v| f32::try_from(*v).ok())
                    .unwrap_or(0.0);
                let y: f32 = data.data.get("stateEstimate.y")
                    .and_then(|v| f32::try_from(*v).ok())
                    .unwrap_or(0.0);
                let z: f32 = data.data.get("stateEstimate.z")
                    .and_then(|v| f32::try_from(*v).ok())
                    .unwrap_or(0.0);
                let thrust: u32 = data.data.get("stabilizer.thrust")
                    .and_then(|v| u32::try_from(*v).ok())
                    .unwrap_or(0);

                println!(
                    "t={:4} ms | x={:+6.3} m  y={:+6.3} m  z={:+6.3} m | thrust={:6}",
                    start.elapsed().as_millis(),
                    x, y, z,
                    thrust
                );
            }
            Err(e) => {
                eprintln!("Log read error: {}", e);
                break;
            }
        }

        sleep(Duration::from_millis(90)).await;
    }

    cf.disconnect().await;

    println!("Disconnected cleanly.");

    Ok(())
}
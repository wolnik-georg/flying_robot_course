use crazyflie_lib::{Crazyflie, NoTocCache};
use crazyflie_link::LinkContext;  // ← correct crate & import

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let link_context = LinkContext::new();

    let uri = "radio://0/80/2M/E7E7E7E7E7";  // your real one

    println!("Connecting to {} ...", uri);

    let cf = Crazyflie::connect_from_uri(
        &link_context,
        uri,
        NoTocCache,
    )
    .await?;

    println!("Connected!");

    let firmware_version = cf.platform.firmware_version().await?;
    let protocol_version = cf.platform.protocol_version().await?;
    println!(
        "Firmware version:     {} (protocol {})",
        firmware_version, protocol_version
    );

    let device_type = cf.platform.device_type_name().await?;
    println!("Device type:          {}", device_type);

    println!("Number of params var: {}", cf.param.names().len());
    println!("Number of log var:    {}", cf.log.names().len());

    cf.disconnect().await;  // no ? needed

    println!("Disconnected cleanly.");

    Ok(())
}
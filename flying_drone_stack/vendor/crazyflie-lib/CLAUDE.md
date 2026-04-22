# vendor/crazyflie-lib/ — Patched cflib-rs 0.4.0

This is a vendored fork of `crazyflie-lib` crate version 0.4.0. It is patched to add
`setpoint_full_state` (CRTP type 6) which is not in the upstream release.

**Do not upgrade this vendored copy** without re-applying the patch (adding `setpoint_full_state`).

Linked via `Cargo.toml`:
```toml
[patch.crates-io]
crazyflie-lib = { path = "vendor/crazyflie-lib" }
```

## Key patch: setpoint_full_state

**File**: `src/subsystems/commander.rs`

Added constants and functions:
```rust
const TYPE_FULL_STATE: u8 = 6;

fn compress_quaternion(w: f32, x: f32, y: f32, z: f32) -> u32
// Smallest-3 quaternion compression: drops largest component, encodes remaining 3
// as 10-bit values in a u32. Matches Crazyflie firmware decoder exactly.

pub async fn setpoint_full_state(
    &self,
    px, py, pz: f32,      // position [m] → sent as i16 mm
    vx, vy, vz: f32,      // velocity [m/s] → sent as i16 mm/s
    ax, ay, az: f32,      // acceleration [m/s²] → sent as i16 mm/s²
    qw, qx, qy, qz: f32,  // quaternion → compressed to u32 (smallest-3)
    omegax, omegay, omegaz: f32,  // angular velocity [rad/s] → sent as i16 mrad/s
) -> Result<()>
```

**Packet layout** (29 bytes total, fits in CRTP max 31):
```
byte 0:     TYPE_FULL_STATE = 6
bytes 1-6:  px, py, pz as i16 (mm)
bytes 7-12: vx, vy, vz as i16 (mm/s)
bytes 13-18: ax, ay, az as i16 (mm/s²)
bytes 19-22: compressed quaternion (u32 LE)
bytes 23-28: omegax, omegay, omegaz as i16 (mrad/s)
```

## Other key APIs (unchanged from upstream)

```rust
// Position setpoint (type 7) — used by main.rs Mode A
cf.commander.setpoint_position(x, y, z, yaw_deg).await?

// RPYT setpoint — used by my_* maneuvers
cf.commander.setpoint_rpyt(roll_deg, pitch_deg, yaw_rate_dps, thrust_u16).await?

// High-level commander — used by Controls/ Python scripts (via Python cflib, not here)
cf.high_level_commander.takeoff(height, duration).await?
cf.high_level_commander.start_trajectory(id, speed_scale, relative).await?
```

## Logging

```rust
let mut block = cf.log.create_block().await?;
block.add_variable("stateEstimate.x").await?;
let stream = block.start(LogPeriod::from_millis(50)?).await?;

// In loop:
if let Ok(packet) = stream.next().await {
    let x: f64 = packet.data["stateEstimate.x"].to_f64_lossy();
}
```

## Parameters

```rust
cf.param.set("stabilizer.controller", 6u8).await?;  // OOT geometric controller
cf.param.set("kalman.resetEstimation", 1u8).await?; // reset EKF
```

## Connection pattern (used in all test binaries)

```rust
let link_context = LinkContext::new();
let cf = Crazyflie::connect_from_uri(&link_context, CF_URI, NoTocCache).await?;
```

`NoTocCache` skips caching the parameter/log table of contents — faster connect, no stale cache issues.

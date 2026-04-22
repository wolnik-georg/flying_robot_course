Scaffold a new Mode B test binary (spline → flatness → full-state CRTP).

The argument `$ARGUMENTS` should be the binary name (e.g. `spline_lemniscate_test`).

Create `src/bin/$ARGUMENTS.rs` using this exact template structure:

```rust
//! <describe what this binary does>
//!
//! Usage: cargo run --release --bin $ARGUMENTS -- [--controller 6]

use crazyflie_lib::{Crazyflie, NoTocCache, Value};
use crazyflie_lib::subsystems::log::{LogBlock, LogPeriod};
use crazyflie_link::LinkContext;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::time::sleep;

use multirotor_simulator::prelude::{SplineTrajectory, Waypoint, Vec3};
use multirotor_simulator::planning::{compute_flatness, rot_to_quat, FlatOutput};

const CF_URI: &str = "radio://0/80/2M/E7E7E7E7E7";
const DEFAULT_CONTROLLER: u8 = 6;
const MASS_KG: f32 = 0.031;   // CF 2.1 + Flow Deck + Multi-ranger

// [helper functions: add_var, add_var_first, get_f32 — copy from spline_circle_test.rs]

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Parse --controller arg
    // 2. Connect, create log blocks (stateEstimate.x/y/z, stabilizer.yaw/thrust, ctrltarget.z, motor, sys.canfly, pm.vbat)
    // 3. Set controller param
    // 4. Kalman reset + RPYT zeros
    // 5. Sample EKF origin (40 packets)
    // 6. RAMP UP (setpoint_full_state, 25 steps, 120ms each)
    // 7. HOVER 7s
    // 8. --- TRAJECTORY LOOP ---
    //    let flat = traj.eval(t % traj.total_time);
    //    let res = compute_flatness(&flat_out, MASS_KG);
    //    let q = rot_to_quat(&res.rot);
    //    cf.commander.setpoint_full_state(
    //        res.pos.x, res.pos.y, res.pos.z,
    //        res.vel.x, res.vel.y, res.vel.z,
    //        flat_out.acc.x, flat_out.acc.y, flat_out.acc.z,
    //        q[0], q[1], q[2], q[3],
    //        res.omega.x, res.omega.y, res.omega.z,
    //    ).await?;
    //    sleep(Duration::from_millis(40)).await;  // 25 Hz
    // 9. Return to center (50 × setpoint_full_state hover, 100ms each)
    // 10. LAND (20 steps, 150ms each, descend 0.50→0.04m)
    // 11. setpoint_rpyt(0,0,0,0) + 500ms
    Ok(())
}
```

Fill in:
- The waypoints and trajectory planning for the specific maneuver
- The doc comment at the top
- Any maneuver-specific parameters as constants

After creating the file, add it to the project structure in `src/bin/CLAUDE.md`.
Verify it compiles: `cargo build --release --bin $ARGUMENTS`

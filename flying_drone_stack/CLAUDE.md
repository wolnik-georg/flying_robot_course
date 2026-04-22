# Flying Drone Stack — Claude Context

## What this project is

A complete Rust autonomous flight stack for Crazyflie 2.1. Single crate `multirotor_simulator`
(library + binaries) plus a separate `firmware_app/` crate (no_std onboard controller).

## Quick commands

```bash
cargo build --release          # build everything
cargo test                     # 249 tests, 0 failures expected
cargo test <name>              # run one test by name substring

# Assignment simulations (no hardware)
cargo run --release --bin assignment1
cargo run --release --bin assignment2
cargo run --release --bin assignment4

# Real hardware — Mode A (position setpoints, SLAM, CSV logging)
cargo run --release --bin main -- --maneuver hover
cargo run --release --bin main -- --maneuver circle
cargo run --release --bin main -- --maneuver circle --ai-deck

# Real hardware — Mode B (full-state, requires OOT firmware)
cargo run --release --bin spline_circle_test
cargo run --release --bin spline_figure8_test

# Firmware build + flash
cd firmware_app && make        # build cf2.bin
cd firmware_app && make cload  # flash via Crazyradio

# Post-flight analysis
cargo run --release --bin mekf_eval -- runs/<file>.csv
cargo run --release --bin slam_eval -- runs/<file>.csv
~/.pyenv/versions/flying_robots/bin/python scripts/plot_flight_diagnostic.py
```

## Module map

```
src/
├── math/          Vec3, Quat, Mat9 — all Copy, all f32
├── dynamics/      MultirotorState, MultirotorParams, MotorAction, MultirotorSimulator
├── integration/   Euler, RK4, ExpEuler, ExpRK4 — all impl Integrator trait
├── controller/    Controller trait, GeometricController (Lee SE(3)), TrajectoryReference, ControlOutput
├── trajectory/    Trajectory trait, Circle/Figure8/Csv/Sequenced/Takeoff trajectories
├── planning/      SplineTrajectory, Waypoint, FlatOutput, FlatnessResult, compute_flatness, rot_to_quat
├── estimation/    MekfState, MekfParams, mekf_predict, mekf_update_{height,flow,vo}
├── flight/        build_state, force_vector_to_rpyt, thrust_to_pwm, yaw_rate_cmd
├── perception/    ImageFrame, FAST-9, BRIEF, CPX camera (AI Deck)
├── mapping/       OccupancyMap, KeyframeStore, VoTrajectory, PoseGraph, LoopConstraint
├── safety.rs      safe_setpoint, safe_setpoint_omap (multi-ranger repulsion + omap probe)
└── bin/
    ├── main.rs                   real hardware flight (20 Hz, Mode A)
    ├── spline_circle_test.rs     Mode B: spline→flatness→full-state 25 Hz
    ├── spline_figure8_test.rs    Mode B
    ├── fullstate_circle_test.rs  Mode B: analytic circle
    ├── fullstate_figure8_test.rs Mode B: analytic figure-8
    ├── export_poly4d.rs          export min-snap spline as Poly4D for HLC Python
    ├── mekf_eval.rs              offline MEKF replay, prints RMSE
    ├── slam_eval.rs              offline VO/loop closure/pose graph analysis
    ├── build_map.rs              replay CSV → PLY occupancy map
    ├── ai_deck_test.rs           AI Deck bench test (no flight)
    ├── assignment1..5.rs         course assignment simulations
    └── sim_closed_loop.rs
firmware_app/
    src/lib.rs   no_std SE(3) controller at 500 Hz (separate crate, thumbv7em)
```

## Key types — cheat sheet

```rust
// Math
Vec3::new(x, y, z)    // Copy — pass by value
Quat::new(w, x, y, z) // Copy — [w, x, y, z] convention

// Dynamics
MultirotorState { position, velocity, orientation, angular_velocity }
MultirotorParams::crazyflie()   // 27g defaults
MotorAction::from_thrust_torque(thrust_n, torque_vec3, &params)

// Control
TrajectoryReference { position, velocity, acceleration, jerk, yaw, yaw_rate, yaw_acceleration }
ControlOutput { thrust: f32, torque: Vec3 }
GeometricController::default()  // tuned Crazyflie gains

// Planning — spline
let traj = SplineTrajectory::plan(&waypoints, &durations, periodic)?;
let flat: FlatOutput = traj.eval(t);   // note: eval, not evaluate
let res: FlatnessResult = compute_flatness(&flat, mass);
let q: [f32; 4] = rot_to_quat(&res.rot).into(); // [w,x,y,z]

// MEKF
mekf_predict(&mut state, gyro_rads, accel_ms2, dt, &proc_noise);
mekf_update_height(&mut state, pz_m, r_height);
mekf_update_flow(&mut state, Some([fx, fy]), Some(pz), theta_p, r_flow, zero_thresh);
mekf_update_vo(&mut state, [vo_x, vo_y], r_vo_xy);

// Mapping
omap.update(pos, roll, pitch, yaw, front, back, left, right, up, down);
kf_store.push(frame, pos, yaw_deg, range_z)  // → Option<KeyframeResult>
vo_traj.push(translation, rotation);         // chain relative poses
pose_graph.optimize(100);                    // Gauss-Seidel 100 sweeps
```

## Three flight modes

| Mode | Binary | Setpoint | Hz | Notes |
|------|--------|----------|----|-------|
| A | `main.rs` | `setpoint_position` (x,y,z,yaw) | 20 | Full SLAM + CSV; firmware position PID |
| B | `spline_*_test` | `setpoint_full_state` (type 6) | 25 | Flatness feedforward; needs OOT firmware |
| C | Python HLC | Poly4D upload + `start_trajectory` | ~100 | Drone evaluates onboard; radio optional after start |

Mode B packet: `commander.setpoint_full_state(px,py,pz, vx,vy,vz, ax,ay,az, qw,qx,qy,qz, wx,wy,wz)`
Implemented in `vendor/crazyflie-lib/src/subsystems/commander.rs`.

## CSV format (49 columns)

```
time_ms, pos_x/y/z [m], vel_x/y/z [m/s],
roll/pitch/yaw [deg], thrust, vbat,
gyro_x/y/z [deg/s], acc_x/y/z [g],
rate_roll/pitch/yaw, range_z [m], flow_dx/dy [px],
mekf_roll/pitch/yaw [deg], mekf_x/y/z [m],
our_ref_x/y/z, our_thrust, our_roll/pitch_cmd, our_yaw_rate_cmd,
multi_front/back/left/right/up [m],
ai_feat_count, vo_x, vo_y, vo_sigma,
pg_x, pg_y, lc_count
```

**Pitch sign**: `stabilizer.pitch` (firmware) and `mekf_pitch` have opposite signs — negate
`mekf_pitch` in plot scripts for visual comparison.

## Coding conventions

- All arithmetic in `f32`; no `f64` anywhere in flight code.
- `Vec3` and `Quat` are `Copy` — prefer value semantics, no needless `.clone()`.
- Rotation matrices are `[[f32; 3]; 3]` row-major. Column `[:][2]` is the body z-axis.
- Quaternion convention: `[w, x, y, z]` with w scalar first (math module);
  firmware uses scalar-last (`q3 = qw`) — see `firmware_app/CLAUDE.md`.
- `SplineTrajectory::plan` third arg `periodic: bool` — use `true` for closed loops.
- `mekf_update_flow` takes `Option<[f32;2]>` — pass `None` to skip update without changing code.
- Never call `f32::sqrt()` in `firmware_app/` — use `libm::sqrtf()`.
- Add new maneuvers by adding a `match` arm in `src/bin/main.rs`; everything else unchanged.

## Key gotchas

- **MEKF seed row**: `mekf_eval` must start from the first row where `range_z > 0.1`, NOT row 0.
- **Flow axis swap**: PMW3901 convention — `vel_x ↔ -flow_dy`, `vel_y ↔ -flow_dx`.
- **theta_p calibration**: 3.50 for March 2026 flights; 0.717 for course dataset `fr00.csv`.
- **Arming threshold** (firmware): controller outputs zero until `setpoint.position.z > 0.05 m`.
- **Gyro in degrees**: `sensors.gyro` in firmware is deg/s — convert with `* π/180`.
- **Setpoint yaw**: `setpoint.attitude.yaw` in firmware is degrees.
- **OOT firmware required** for Modes B and C: `cd firmware_app && make cload`.
- **AI Deck**: Nina reboots after 13–70s (hardware limitation); stack reconnects automatically.

## Hardware

| Component | Status | Notes |
|-----------|--------|-------|
| Crazyflie 2.1 | ✅ | URI: `radio://0/80/2M/E7E7E7E7E7` |
| Flow Deck v2 | ✅ | Needs textured floor for XY |
| Multi-ranger Deck | ✅ | Required for safety + omap |
| AI Deck (GAP8+Nina) | ⚠️ | Reboots after 13–70s; SLAM survives |

## Tests

```bash
cargo test                    # run all 249
cargo test mekf               # filter by name
cargo test -- --nocapture     # show println! output
```

Test files: `tests/test_mekf.rs`, `test_flatness.rs`, `test_spline.rs`,
`test_geometric_controller.rs`, `test_safety.rs`, `test_math.rs`, `test_flight_math.rs`,
`test_hover_control_loop.rs`, `test_controller_detailed.rs`.

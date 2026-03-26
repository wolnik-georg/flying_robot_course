# Flying Drone Stack

A complete Rust-based stack for Crazyflie 2.1 real-hardware flight, covering dynamics
simulation, SE(3) geometric control, MEKF state estimation, minimum-snap motion planning,
3D occupancy mapping, visual odometry (VO), loop-closure SLAM, and autonomous frontier
exploration — all validated against real flight logs.

> **Course assignments with results and plots → [ASSIGNMENTS.md](ASSIGNMENTS.md)**

## Quick Start

```bash
cd flying_drone_stack

# Build everything
cargo build --release

# Run all tests (249 tests, 0 failures)
cargo test
```

## Real Hardware Flight

```bash
# Hover at 0.30 m for 30 s
cargo run --release --bin main -- --maneuver hover

# Circle r=0.25 m, ω=0.30 rad/s, 50 s (~2.4 laps)
cargo run --release --bin main -- --maneuver circle

# Figure-8 a=0.25 m b=0.15 m, ω=0.5 rad/s, 40 s
cargo run --release --bin main -- --maneuver figure8

# Autonomous exploration (frontier-based, SCAN→NAVIGATE→LAND)
cargo run --release --bin main -- --maneuver explore

# Any maneuver with AI Deck camera (VO + loop closure + SLAM enabled)
cargo run --release --bin main -- --maneuver circle --ai-deck
cargo run --release --bin main -- --maneuver explore --ai-deck
```

**Hardware required**: Crazyflie 2.1 + Flow Deck v2 + Multi-ranger Deck.
Optional: AI Deck (enables full visual-SLAM path).

Radio URI: `radio://0/80/2M/E7E7E7E7E7`

## Offline Analysis Binaries

```bash
# Offline MEKF vs firmware EKF (prints RMSE)
cargo run --release --bin mekf_eval -- runs/<file>.csv

# Analyse VO, loop closure, and pose graph from a flight CSV
cargo run --release --bin slam_eval -- runs/<file>.csv

# Build a 3D occupancy map from one or more flight CSVs → PLY file
cargo run --release --bin build_map -- runs/<file>.csv

# Test AI Deck camera connection (bench test, no flight)
cargo run --release --bin ai_deck_test -- --frames 50 --save-all
```

## Post-Flight Scripts

```bash
# Diagnostic overview: EKF vs MEKF, shadow controller, multi-ranger
python3 scripts/plot_flight_diagnostic.py          # auto-picks latest CSV

# Shadow controller evaluation: roll/pitch cmd vs actual
python3 scripts/plot_shadow_eval.py runs/<file>.csv

# View a saved map in MeshLab or CloudCompare
meshlab results/data/map.ply
```

## Assignment Binaries (simulation only)

```bash
cargo run --release --bin assignment1   # Integrator comparison (Euler/RK4/Exp)
cargo run --release --bin assignment2   # SE(3) geometric control, figure-8 tracking
cargo run --release --bin assignment3 -- --csv "../State Estimation/logging_ekf/logging/fr00.csv"
cargo run --release --bin assignment4   # Minimum-snap spline + differential flatness
cargo run --release --bin assignment5 -- circle   # Safe-space sim (hover/circle/fig8) + MEKF
```

## Flight Procedure

1. **Charge battery** — start ≥ 3.7 V; hover pulls to ~3.5 V.
2. **Place drone on textured surface** — PMW3901 flow sensor needs texture for XY estimation.
3. **Plug in Crazyradio PA** — URI `radio://0/80/2M/E7E7E7E7E7`.
4. **Run**: `cargo run --release --bin main -- --maneuver <name>` (with `--ai-deck` if using camera).
5. **Evaluate**: `python3 scripts/plot_flight_diagnostic.py`

The binary auto-resets the Kalman filter, ramps up, hovers to stabilise, executes the maneuver, and lands.  CSV written to `runs/<maneuver>_<timestamp>.csv`.

### Pre-flight checklist
- [ ] Battery ≥ 3.7 V per cell
- [ ] Textured paper/cardboard under drone
- [ ] Crazyradio PA plugged in
- [ ] Clear 1 m × 1 m flight area, ceiling > 0.5 m
- [ ] Multi-ranger Deck attached (required for safety repulsion + occupancy map)
- [ ] If `--ai-deck`: laptop connected to "WiFi streaming example" AP, drone powered ≥ 15 s

## Stack Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│  Sensors (hardware)                                                   │
│  IMU: gyro [deg/s] + accel [g]                                       │
│  Flow Deck v2: PMW3901 optical flow [px] + VL53L1x ToF range [m]    │
│  Multi-ranger Deck: 5× VL53L1x (front/back/left/right/up)           │
│  AI Deck: HiMax HM01B0 camera 324×244, JPEG over WiFi TCP (CPX)     │
└─────────────────────────────────┬────────────────────────────────────┘
                                  │
             ┌────────────────────▼────────────────────┐
             │  Estimation — MEKF  (estimation/mekf.rs) │
             │  State: position, body vel, quaternion   │
             │  Fuses: IMU predict + ToF height +       │
             │         flow XY vel + VO position        │
             └────────────────────┬────────────────────┘
                                  │
     ┌────────────────────────────▼────────────────────────────────┐
     │  Mapping  (mapping/)                                         │
     │  OccupancyMap: sparse log-odds voxel grid (5 cm),           │
     │                ray-cast from multi-ranger                    │
     │  KeyframeStore: FAST-9 + BRIEF, normalized 8-pt E matrix,   │
     │                 metric VO, spatial-grid loop closure         │
     │  VoTrajectory: chains relative poses → global XY            │
     │  PoseGraph: Gauss-Seidel 100× → corrects VO drift           │
     └──────────────────┬─────────────────┬───────────────────────┘
                        │                 │
         ┌──────────────▼──┐   ┌──────────▼──────────────┐
         │  Safety         │   │  Planning                │
         │  Multi-ranger   │   │  ExplorationPlanner:     │
         │  repulsion +    │   │  SCAN→NAVIGATE→LAND      │
         │  occupancy      │   │  frontier selection      │
         │  probe          │   │  (planning/exploration)  │
         └──────────────┬──┘   └──────────┬───────────────┘
                        │                 │
             ┌──────────▼─────────────────▼──────────────┐
             │  Firmware track (main.rs)                  │
             │  setpoint_position → Crazyflie PID         │
             │  Shadow track (main.rs)                    │
             │  SE(3) geometric controller → CSV only     │
             └────────────────────────────────────────────┘
```

## CSV Output Format (49 columns)

```
time_ms, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
roll, pitch, yaw, thrust, vbat,
gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z,
rate_roll, rate_pitch, rate_yaw,
range_z, flow_dx, flow_dy,
mekf_roll, mekf_pitch, mekf_yaw, mekf_x, mekf_y, mekf_z,
our_ref_x, our_ref_y, our_ref_z,
our_thrust, our_roll_cmd, our_pitch_cmd, our_yaw_rate_cmd,
multi_front, multi_back, multi_left, multi_right, multi_up,
ai_feat_count,
vo_x, vo_y, vo_sigma,
pg_x, pg_y, lc_count
```

Units: time [ms], pos/vel [m, m/s], roll/pitch/yaw [deg], gyro [deg/s], acc [g],
range [m], flow [px], mekf_roll/pitch/yaw [deg], mekf_x/y/z [m].

## Project Structure

```
flying_drone_stack/
├── Cargo.toml
├── README.md                         # This file
├── ROADMAP.md                        # What is built and what to do next
├── VALIDATION_PLAN.md                # Tiered flight validation plan (Tier 0–7)
├── docs/
│   ├── ARCHITECTURE.md               # Full architecture documentation
│   └── DRONE_STACK_ASPECTS.md
├── scripts/
│   ├── plot_flight_diagnostic.py     # Post-flight overview (picks latest CSV)
│   ├── plot_shadow_eval.py           # Shadow controller evaluation
│   ├── plot_assignment1..5.py        # Assignment plot scripts
│   └── plot_mekf_eval.py
├── src/
│   ├── lib.rs                        # Library entry point
│   ├── safety.rs                     # Multi-ranger repulsion, safety checks
│   ├── math/                         # Vec3, Quat, Mat9 (README.md inside)
│   ├── dynamics/                     # MultirotorState, MultirotorParams, simulator
│   ├── integration/                  # Euler, RK4, ExpEuler, ExpRK4
│   ├── controller/                   # GeometricController (SE(3) Lee et al. 2010)
│   ├── trajectory/                   # CircleTrajectory, Figure8, Sequenced, …
│   ├── planning/                     # SplineTrajectory, flatness, ExplorationPlanner (README.md)
│   ├── estimation/                   # MEKF: predict + height/flow/VO updates (README.md)
│   ├── flight/                       # build_state, force_vector_to_rpyt, yaw_rate_cmd
│   ├── mapping/                      # OccupancyMap, KeyframeStore, VoTrajectory, PoseGraph (README.md)
│   ├── perception/                   # Sensor traits, CPX camera, FAST-9/BRIEF features (README.md)
│   └── bin/
│       ├── main.rs                   # Real hardware flight (CLI maneuver arg + --ai-deck)
│       ├── mekf_eval.rs              # Offline MEKF vs firmware EKF RMSE
│       ├── slam_eval.rs              # Offline VO/loop closure/pose graph analysis
│       ├── build_map.rs              # Replay CSV → PLY occupancy map
│       ├── ai_deck_test.rs           # AI Deck camera bench test
│       ├── assignment1..5.rs         # Course assignment simulations
│       └── sim_closed_loop.rs
├── tests/
│   ├── test_mekf.rs
│   ├── test_flight_math.rs
│   ├── test_geometric_controller.rs
│   ├── test_controller_detailed.rs
│   ├── test_flatness.rs
│   ├── test_hover_control_loop.rs
│   ├── test_math.rs
│   ├── test_safety.rs
│   └── test_spline.rs
└── runs/                             # Real flight CSVs (49 columns, timestamped)
```

## Testing

```bash
cargo test
# Expected: 249 tests, 0 failures
```

Coverage: MEKF predict/update/flow gate/VO update, flight math, geometric controller,
differential flatness, hover control loop, safety limits, spline planner, occupancy map
ray casting, FAST-9/BRIEF features, CPX frame reassembly, VO trajectory chain,
loop closure age/spatial gates, pose graph Gauss-Seidel, spatial grid index, exploration FSM.

## Key Design Decisions

### Firmware track vs shadow track

The `main` binary runs two parallel tracks every loop iteration:

1. **Firmware track** — `setpoint_position(x, y, z, yaw)` commands the Crazyflie's onboard
   position PID. This is what actually flies the drone.
2. **Shadow track** — our SE(3) geometric controller evaluates what *it would command* given
   the same state and reference trajectory, logs the result, and **never touches the motors**.

### SLAM pipeline (with `--ai-deck`)

Each JPEG frame from the AI Deck is processed by `KeyframeStore::push()`. When the drone
has moved ≥ 0.15 m or ≥ 30° since the last keyframe, FAST-9 features are detected, BRIEF
descriptors computed, features matched to the previous keyframe, and the relative pose
estimated via the normalized 8-point essential matrix algorithm (metric scale from `range_z`).

`VoTrajectory` accumulates these relative poses into a world-frame position estimate and
feeds corrections into the MEKF via `mekf_update_vo`. `PoseGraph` applies loop-closure
corrections when `detect_loop` finds a spatially and visually consistent revisit.

### Safety layer

`safe_setpoint_omap()` applies two layers of repulsion before every setpoint:
1. **Multi-ranger repulsion** — proportional push away from walls (threshold 0.40 m,
   max correction 0.12 m per axis).
2. **Occupancy-map probe** — 8-direction horizontal ring at 0.25 m radius around the
   setpoint; any occupied voxel cancels motion in that direction.

### MEKF sensor calibration

- `THETA_P = 3.50` rad — empirically calibrated from March 2026 flights.
- Zero-motion gate `0.3 px` — skips PMW3901 zero-padded entries (~46% of airborne samples).
- Flow axis convention: `vel_x ↔ -flow_dy`, `vel_y ↔ -flow_dx`.
- Validated RMSE (Mar 17 flights): hover roll 0.84°, pitch 2.39°, yaw 1.03°; circle roll 0.59°.

## Dependencies

| Crate | Purpose |
|-------|---------|
| `crazyflie-lib 0.4` | CRTP protocol — logging, parameters, commander |
| `crazyflie-link 0.3` | Radio/USB link layer |
| `tokio` | Async runtime for concurrent log stream reading |
| `chrono` | UTC timestamps for CSV filenames |
| `simple_qp` / Clarabel | QP solver for minimum-snap spline planner |
| `jpeg-decoder` | JPEG decoding for AI Deck frames |
| `serde` | Config deserialisation (toml) |

## License

Educational / research use.

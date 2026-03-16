# Flying Drone Stack

A complete Rust-based stack for Crazyflie 2.1 real-hardware flight, covering dynamics simulation, SE(3) geometric control, MEKF state estimation, minimum-snap motion planning, and a shadow-controller evaluation framework — all validated against real flight logs.

## Quick Start

```bash
cd flying_drone_stack

# Build everything
cargo build --release

# Run all tests
cargo test

# Verify full pipeline (tests + all binaries + all plots)
bash verify_all.sh
```

## Available Binaries

### 📊 Assignment 1 — Integrator comparison
```bash
cargo run --release --bin assignment1
PYENV_VERSION=flying_robots python3 scripts/plot_assignment1.py
```
Compares Euler, RK4, ExpEuler, ExpRK4 on a free-flight trajectory. Writes CSVs to `results/data/`.

### 🚁 Assignment 2 — Geometric control & trajectory tracking
```bash
cargo run --release --bin assignment2
cargo run --release --bin assignment2 -- --realistic-start
PYENV_VERSION=flying_robots python3 scripts/plot_assignment2.py
```
Tracks a figure-8 with the SE(3) geometric controller. `--realistic-start` prepends takeoff + hover.

### 🎯 Assignment 3 — MEKF offline validation
```bash
cargo run --release --bin assignment3 -- --csv "../State Estimation/logging_ekf/logging/fr00.csv"
PYENV_VERSION=flying_robots python3 scripts/plot_assignment3.py
```
Runs the Multiplicative EKF offline against a real Crazyflie flight log and compares to the on-board EKF.

### 🏁 Assignment 4 — Minimum-snap spline planning & differential flatness
```bash
cargo run --release --bin assignment4
PYENV_VERSION=flying_robots python3 scripts/plot_assignment4.py
```
Generates minimum-snap polynomial trajectories. Writes planned/open-loop/closed-loop CSVs to `results/data/`.

### 🛡️ Assignment 5 — Safe-space simulation (hover / circle / figure-8 + MEKF)
```bash
cargo run --release --bin assignment5 -- hover
cargo run --release --bin assignment5 -- circle
cargo run --release --bin assignment5 -- figure8
PYENV_VERSION=flying_robots python3 scripts/plot_assignment5.py
```
Full MEKF + geometric controller simulation inside a 1.0 × 1.0 m safety box (max 0.30 m height).

### 🔬 MEKF evaluation — offline MEKF vs flight log
```bash
cargo run --release --bin mekf_eval -- runs/circle_2026-03-15_11-46-16.csv
# With parameter overrides for tuning:
cargo run --release --bin mekf_eval -- runs/circle_2026-03-15_11-46-16.csv --r_flow 4.0
```
Runs the Rust MEKF against any recorded flight CSV and prints RMSE for orientation and position.

### 🚀 Main — Real hardware flight (Crazyflie 2.1 + Flow/ToF decks)
```bash
# 1. Edit MANEUVER at the top of src/bin/main.rs:
#      "hover"   — hold position at 0.30 m for 12 s
#      "circle"  — radius 0.25 m, omega = 0.6 rad/s, 30 s
#      "figure8" — a=0.25 m, b=0.15 m, omega = 0.5 rad/s, 40 s
# 2. Build and run:
cargo build --release --bin main
./target/release/main

# 3. After the flight, evaluate the shadow controller:
PYENV_VERSION=flying_robots python3 scripts/plot_shadow_eval.py
```

Connects to the drone over `radio://0/80/2M/E7E7E7E7E7`, executes the chosen maneuver using the firmware's onboard position PID, and simultaneously logs:
- Firmware EKF state (pos, vel, attitude, body rates, thrust)
- Our MEKF running as a passenger (logged, never sent to the drone)
- Shadow geometric controller outputs (what we would have commanded — logged only)

Each flight produces a timestamped 37-column CSV in `runs/`.

## Flight Procedure

1. **Charge battery fully** — hover load pulls the cell to ~3.5–3.6 V; starting below 3.7 V risks mid-flight voltage collapse.
2. **Place drone on textured surface** — the PMW3901 flow sensor requires texture for XY estimation.
3. **Set `MANEUVER`** constant at the top of `src/bin/main.rs`.
4. **Build**: `cargo build --release --bin main`
5. **Run**: `./target/release/main`  
   The binary waits 3 s, pulses Kalman reset, ramps up over ~1.5 s, stabilises at 0.30 m for 8 s, samples the EKF XY origin, then executes the maneuver and lands.
6. **Evaluate**: `PYENV_VERSION=flying_robots python3 scripts/plot_shadow_eval.py`

### Pre-flight checklist
- [ ] Battery ≥ 3.7 V per cell (fully charged)
- [ ] Textured paper/cardboard under drone (flow sensor needs texture)
- [ ] Crazyradio PA plugged in, URI = `radio://0/80/2M/E7E7E7E7E7`
- [ ] `MANEUVER` set correctly in `src/bin/main.rs`
- [ ] Clear 1 m × 1 m flight area, ceiling > 0.5 m

## Scripts

All scripts live in `scripts/` and require the `flying_robots` pyenv environment.

| Script | Purpose |
|--------|---------|
| `plot_assignment1.py` | Integrator comparison plots |
| `plot_assignment2.py` | Geometric control tracking plots |
| `plot_assignment3.py` | MEKF vs on-board EKF orientation comparison |
| `plot_assignment4.py` | Minimum-snap spline planning plots |
| `plot_assignment5.py` | Safe-space simulation plots |
| `plot_mekf_eval.py` | MEKF evaluation plots from `mekf_eval` output |
| `plot_shadow_eval.py` | Shadow controller evaluation: roll/pitch/thrust/XY vs firmware |

`plot_shadow_eval.py` auto-detects the newest CSV in `runs/`. It handles both the 30-column MEKF-only format (older `bitcraze_rs_lib` flights) and the 37-column full format (main binary with shadow columns).

## Project Structure

```
flying_drone_stack/
├── Cargo.toml                   # Workspace config & dependencies
├── README.md                    # This file
├── verify_all.sh                # Full pipeline verification script
├── configs/
│   └── flight_defaults.toml     # Default flight parameters
├── docs/
│   ├── ARCHITECTURE.md          # Detailed architecture documentation
│   └── DRONE_STACK_ASPECTS.md   # Feature roadmap
├── scripts/
│   ├── plot_assignment1.py      # Assignment 1 plots
│   ├── plot_assignment2.py      # Assignment 2 plots
│   ├── plot_assignment3.py      # Assignment 3 plots
│   ├── plot_assignment4.py      # Assignment 4 plots
│   ├── plot_assignment5.py      # Assignment 5 plots
│   ├── plot_mekf_eval.py        # MEKF evaluation plots
│   └── plot_shadow_eval.py      # Shadow controller evaluation
├── src/
│   ├── lib.rs                   # Library entry point & prelude
│   ├── math/                    # Vec3, Quat, Mat9
│   ├── dynamics/                # MultirotorState, MultirotorParams, simulator
│   ├── integration/             # Euler, RK4, ExpEuler, ExpRK4
│   ├── controller/              # GeometricController (SE(3) Lee et al. 2010)
│   ├── trajectory/              # Figure8, Circle, SmoothFigure8, Sequenced, …
│   ├── planning/                # Minimum-snap QP spline + differential flatness
│   ├── estimation/              # Mekf: IMU predict + height/flow updates
│   ├── flight/                  # build_state, force_vector_to_rpyt, yaw_rate_cmd, …
│   └── bin/
│       ├── assignment1.rs       # Assignment 1
│       ├── assignment2.rs       # Assignment 2
│       ├── assignment3.rs       # Assignment 3
│       ├── assignment4.rs       # Assignment 4
│       ├── assignment5.rs       # Assignment 5
│       ├── demo.rs              # Quick demo
│       ├── mekf_eval.rs         # Offline MEKF evaluation vs flight log
│       ├── sim_closed_loop.rs   # Closed-loop simulation
│       └── main.rs              # Real hardware flight binary
├── tests/
│   ├── test_mekf.rs             # MEKF unit + integration tests
│   ├── test_flight_math.rs      # Flight math helpers
│   ├── test_geometric_controller.rs
│   ├── test_controller_detailed.rs
│   ├── test_flatness.rs
│   ├── test_hover_control_loop.rs
│   ├── test_math.rs
│   ├── test_safety.rs
│   └── test_spline.rs
└── runs/                        # Real flight CSVs (timestamped)
    ├── hover_<timestamp>.csv
    ├── circle_<timestamp>.csv
    └── figure8_<timestamp>.csv
```

## Testing

```bash
cargo test
# Expected: 248 tests, 0 failures
```

Test coverage: MEKF predict/update/flow gate, flight math, geometric controller gains, differential flatness chain, hover control loop convergence, safety limits, spline planner continuity.

## Verification

```bash
bash verify_all.sh
# Runs: cargo test → all 5 assignment binaries → all 5 plot scripts
# All steps must pass.
```

## Key Design Decisions

### Shadow controller
The `main` binary runs two parallel tracks during every flight:

1. **Firmware track** — `setpoint_position(x, y, z, yaw)` commands the Crazyflie's onboard position PID. This is what actually flies the drone.
2. **Shadow track** — our geometric controller evaluates what it *would* command given the same state and reference, logs it, and **never touches the motors**.

Both tracks use the same coordinate frame (firmware EKF: `pos_x/y/z`, `roll/pitch/yaw`). Target: shadow roll/pitch RMSE < 5° airborne before the shadow is trusted to fly.

### MEKF sensor calibration
Empirically calibrated from March 2026 flights (circle + figure-8 logs):
- `THETA_P = 3.50` rad effective constant (old value 0.717 gave 4.9× too little XY motion)
- **Zero-motion gate**: skips flow updates when both axes < 0.3 px, filtering PMW3901 zero-padding artefacts (~46% of airborne 100 Hz log entries are padded zeros)
- Flow scale uses ToF-measured height rather than estimated state height (removes height-wobble → velocity-drift feedback loop)

### Coordinate frame rule
Shadow controller state **must** use firmware EKF fields (`pos_x/y/z`, `roll/pitch/yaw`), not MEKF fields. The firmware's `setpoint_position` commands live in the firmware EKF frame; mixing frames causes up to 50 cm position error and full command saturation.

## Dependencies

| Crate | Purpose |
|-------|---------|
| `crazyflie-lib 0.4` | CRTP protocol — logging, parameters, commander |
| `crazyflie-link 0.3` | Radio/USB link layer |
| `tokio` | Async runtime for concurrent log stream reading |
| `chrono` | UTC timestamps for CSV filenames |
| `simple_qp` | QP solver for minimum-snap spline planner |
| `serde` | Config deserialisation (`toml`) |

## License

Educational / research use.

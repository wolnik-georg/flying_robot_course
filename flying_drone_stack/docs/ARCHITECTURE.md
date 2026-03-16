# Architecture

## Overview

A modular Rust library for Crazyflie 2.1 real-hardware flight, covering the full perception-to-control stack: IMU-driven MEKF state estimation, SE(3) geometric control, minimum-snap motion planning, and a shadow-controller evaluation framework. Everything is in `f32`, no-std compatible, and validated against real flight logs.

---

## Full Pipeline Data Flow

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  Sensors (hardware)                                                          │
│  IMU: gyro [deg/s] + accel [g]  │  ToF range [mm]  │  Optical flow [px]    │
└──────────────────┬──────────────────────────────────────────────────────────┘
                   │ raw sensor rows (100 Hz)
                   ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  Estimation — MEKF  (src/estimation/mekf.rs)                                │
│  State: position [m], body velocity [m/s], attitude (quaternion)            │
│  Fuses IMU (predict) + ToF (height update) + flow (XY velocity update)      │
└──────────────────┬──────────────────────────────────────────────────────────┘
                   │ MultirotorState: p, v, q, ω
                   ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  Trajectory / Planning                                                       │
│  SplineTrajectory::eval(t) → FlatOutput → compute_flatness → FlatnessResult │
│  or: Figure8Trajectory / CircleTrajectory / CsvTrajectory                   │
│  → TrajectoryReference: pos, vel, acc, jerk, yaw, yaw_rate                  │
└──────────────────┬──────────────────────────────────────────────────────────┘
                   │ TrajectoryReference
                   ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  Controller — SE(3) Geometric (src/controller/mod.rs)                       │
│  Position PD + feedforward → desired force vector                           │
│  Rotation error on SO(3) → torque                                           │
│  Output: thrust [N] + torque [N·m]                                          │
└──────────────────┬──────────────────────────────────────────────────────────┘
                   │ ControlOutput: thrust, torque
                   ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  Motor Mixer (MotorAction::from_thrust_torque)                              │
│  Invert mixer: (f, τ) → [ω₁², ω₂², ω₃², ω₄²]                             │
└──────────────────┬──────────────────────────────────────────────────────────┘
                   │ MotorAction (simulation) or RPYT setpoint (hardware)
                   ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  Actuators                                                                  │
│  Simulation: MultirotorSimulator::step → next MultirotorState               │
│  Hardware:   Crazyflie firmware setpoint_position / setpoint_rpyt           │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Module Data-Flow Table

| Module | Key Inputs | Key Outputs | Consumed By |
|--------|-----------|-------------|-------------|
| `estimation/` | gyro, accel, range, flow | `MultirotorState` (p, v, q, ω) | `controller/`, `flight/`, `bin/main.rs` |
| `planning/` | waypoints, time allocations | `TrajectoryReference` (pos, vel, acc, jerk, yaw) | `controller/` |
| `trajectory/` | time `t` | `TrajectoryReference` | `controller/` |
| `controller/` | `MultirotorState` + `TrajectoryReference` | `ControlOutput` (f, τ) | `dynamics/` (sim), `flight/` (hardware) |
| `dynamics/` | `MotorAction`, `MultirotorParams` | next `MultirotorState` | simulation loop |
| `integration/` | `MultirotorState`, `MotorAction` | updated `MultirotorState` | `dynamics/simulator.rs` |
| `flight/` | firmware log row + `ControlOutput` | CRTP RPYT setpoint | `bin/main.rs` |

### Per-Module Educational Docs

- [Dynamics — Multirotor Rigid-Body Physics](../src/dynamics/README.md)
- [Integration — Numerical ODE Solvers](../src/integration/README.md)
- [Controller — SE(3) Geometric Controller](../src/controller/README.md)
- [Trajectory — Reference Signal Generators](../src/trajectory/README.md)
- [Estimation — Multiplicative Extended Kalman Filter](../src/estimation/README.md)
- [Planning — Minimum-Snap Motion Planning & Differential Flatness](../src/planning/README.md)
- [Flight — Hardware Bridge](../src/flight/README.md)

---

## Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────────┐
│                             Application Layer                             │
│                                                                           │
│  assignment1 · assignment2 · assignment3 · assignment4 · assignment5      │
│  mekf_eval   · main (real hardware)      · demo · sim_closed_loop        │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────────────────┐
│                       Flying Drone Stack Library                          │
├───────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐                │
│  │  math/   │  │ dynamics/│  │ integr./ │  │controller│                │
│  │  Vec3    │  │  State   │  │  Euler   │  │Geometric │                │
│  │  Quat    │  │  Params  │  │  RK4     │  │SE(3) Lee │                │
│  │  Mat9    │  │  Simul.  │  │  Exp     │  │          │                │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘                │
│                                                                           │
│  ┌───────────────┐  ┌──────────────────────┐  ┌──────────────────────┐  │
│  │  planning/    │  │    trajectory/        │  │    estimation/       │  │
│  │  Spline ·     │  │  Figure8 · Circle ·   │  │  MEKF                │  │
│  │  Flatness ·   │  │  CsvTraj · Takeoff ·  │  │  f32, Joseph form    │  │
│  │  Minimum-snap │  │  Sequenced            │  │  zero-flow gate      │  │
│  └───────────────┘  └──────────────────────┘  └──────────────────────┘  │
│                                                                           │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  flight/                                                           │  │
│  │  build_state · force_vector_to_rpyt · yaw_rate_cmd · rpm_to_pwm   │  │
│  └────────────────────────────────────────────────────────────────────┘  │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## Module Responsibilities

### `math/` — Mathematical Primitives

Pure mathematical operations, zero external dependencies.

- `vec3.rs`: 3D vectors for position, velocity, force, torque
- `quaternion.rs`: Unit quaternion for 3D rotation without gimbal lock
- `matrix.rs`: `Mat9` — 9×9 f32 matrix for MEKF covariance arithmetic:
  - `mat_mul`, `transpose`, `scale`, `add` — standard matrix algebra
  - `joseph_update(Σ, K, H, R)` — full Joseph form `(I−KH)Σ(I−KH)ᵀ + K·R·Kᵀ`, preserves positive-definiteness
  - `symmetrise()` — forces `Σ ← (Σ+Σᵀ)/2` after every predict/update
  - `clamp_diagonal(max_var)` — caps each diagonal, scales off-diagonal proportionally

---

### `dynamics/` — Physics Core

- `state.rs`: `MultirotorState` (position, velocity, quaternion, angular velocity) and `MotorAction`
- `params.rs`: `MultirotorParams` — Crazyflie 2.1 physical parameters (mass, inertia, motor constants, arm length)
- `simulator.rs`: `MultirotorSimulator` — wires state + injected integrator + rigid-body derivatives

---

### `integration/` — Numerical Integration

Pluggable via `Integrator` trait:

- `euler.rs`: First-order forward Euler
- `rk4.rs`: Fourth-order Runge-Kutta (quaternion integration bug-fixed)
- `exponential.rs`: ExpEuler, ExpRK4 — exponential-map methods for stiff quaternion dynamics

---

### `controller/` — SE(3) Geometric Controller

`GeometricController` based on Lee et al. (2010). Full position PD + feedforward and attitude PD control.

**Key types**: `GeometricController`, `TrajectoryReference`, `ControlOutput`

---

### `trajectory/` — Reference Signal Generators

All trajectory types implement the `Trajectory` trait (`get_reference(t: f32) → TrajectoryReference`):

| Type | Description |
|------|-------------|
| `Figure8Trajectory` | Polynomial figure-8, configurable amplitude and `time_scale` |
| `CircleTrajectory` | Horizontal circle, configurable radius, height, angular velocity |
| `CsvTrajectory` | Interpolates waypoints from a CSV file |
| `TakeoffTrajectory` | Minimum-jerk polynomial from ground to target height |
| `SequencedTrajectory` | Chains multiple trajectories with per-phase durations |

---

### `planning/` — Minimum-Snap Motion Planning

- `spline.rs`: 8th-order polynomial spline planner (QP, minimum-snap, continuity up to snap). Backend: `simple_qp` / Clarabel.
- `flatness.rs`: Differential flatness chain — position/yaw + derivatives → rotation, thrust, angular velocity, torque.

**Key types**: `SplineTrajectory`, `Waypoint`, `FlatOutput`, `compute_flatness`

---

### `estimation/` — MEKF State Estimator

`mekf.rs` implements a Multiplicative Extended Kalman Filter.

#### State vector `x ∈ ℝ⁹`

| Component | Symbol | Units |
|-----------|--------|-------|
| Position (world frame) | `p` | m |
| Velocity (body frame) | `b` | m/s |
| Attitude error angles | `δ` | rad |

`q_ref` is maintained outside the filter and reset to zero attitude error after each update.

#### Inputs vs. measurements

- **Control input** (not measured): gyroscope [deg/s] + accelerometer [G]
- **Measurements**: range sensor height (mm → m) and PMW3901 optical flow (px)

#### Numerical stability (all three required for f32)

1. Full Joseph form covariance update
2. Symmetrisation `Σ ← (Σ+Σᵀ)/2` after every step
3. Diagonal cap `MAX_COVARIANCE = 100.0` (mirrors Crazyflie `kalman_core.c`)

#### Tuned noise parameters

Empirically calibrated from March 2026 Crazyflie flights (circle + figure-8 logs):

| Parameter | Value | Notes |
|-----------|-------|-------|
| `q_pos` | 1e-7 | Process noise, position |
| `q_vel` | 1e-3 | Process noise, velocity |
| `q_att` | 1e-6 | Process noise, attitude |
| `r_height` | 1e-3 | Range measurement noise |
| `r_flow` | 8.0 | Flow measurement noise |
| `NP` | 350.0 px | Sensor pixel count |
| `THETA_P` | **3.50** rad | Effective flow scale constant (see calibration note) |
| `zero_flow_threshold` | **0.3 px** | Zero-motion gate (see below) |

#### `THETA_P` calibration

The original firmware value of 0.71674 rad produced 4.9× too little XY motion in the Rust MEKF. An offline sweep against real circle flights (March 15, 2026) found **THETA_P = 3.50** minimises RMSE vs the firmware EKF. The discrepancy likely arises from the firmware linearising the flow model differently.

#### Zero-motion gate (`zero_flow_threshold = 0.3 px`)

The PMW3901 logs zero-padded entries at 100 Hz when the drone is hovering or moving slowly (~46% of airborne log entries). Incorporating those zeros as genuine "no motion" measurements drives the MEKF into a velocity bias. Any flow reading where both axes are < 0.3 px is skipped entirely.

#### MEKF validation results

From offline replay of fr00.csv (Crazyflie figure-8, 7.2 s, ~1 kHz) with calibrated params:

| Metric | Value |
|--------|-------|
| Orientation RMS vs on-board EKF — roll | 0.87° |
| Orientation RMS vs on-board EKF — pitch | 1.02° |
| Orientation RMS vs on-board EKF — yaw | 0.21° |
| Position RMS vs on-board EKF — x | 0.088 m |
| Position RMS vs on-board EKF — y | 0.127 m |
| Position RMS vs on-board EKF — z | 0.006 m |

---

### `flight/` — Hardware Flight Helpers

Thin utilities used by `main.rs` to bridge Rust types and Crazyflie commands:

| Function | Purpose |
|----------|---------|
| `build_state` | Constructs `MultirotorState` from a firmware log row |
| `force_vector_to_rpyt` | Converts SE(3) output force vector to roll/pitch/yaw-rate/thrust |
| `yaw_rate_cmd` | Computes yaw-rate setpoint from attitude error |
| `rpm_to_pwm` / `thrust_to_pwm` | Motor scaling helpers |

**Frame rule**: `build_state` must use firmware EKF fields (`pos_x/y/z`, `roll/pitch/yaw`) — not MEKF fields. The firmware's `setpoint_position` commands live in the firmware EKF frame; mixing frames causes a ~50 cm offset and full command saturation.

---

## Binaries

### Assignment binaries (`assignment1`–`assignment5`)
Simulation-only. No hardware required. Each writes CSVs to `results/data/` and reads them back from the corresponding `scripts/plot_assignment*.py`.

### `mekf_eval` — Offline MEKF vs flight log
```bash
cargo run --release --bin mekf_eval -- runs/circle_2026-03-15_11-46-16.csv
cargo run --release --bin mekf_eval -- runs/circle_2026-03-15_11-46-16.csv --r_flow 4.0
```
Replays any recorded flight CSV through the Rust MEKF and prints RMSE vs firmware EKF. Supports per-run parameter overrides for tuning sweeps.

### `main` — Real hardware flight binary

```bash
# Set MANEUVER in src/bin/main.rs line 27, then:
cargo build --release --bin main && ./target/release/main
```

**Flight sequence**:
1. Connect to `radio://0/80/2M/E7E7E7E7E7`
2. Wait 3 s, pulse Kalman reset
3. Ramp thrust over ~1.5 s, climb to 0.30 m
4. Stabilise hover for 8 s; record EKF XY origin
5. Execute maneuver (hover / circle / figure-8)
6. Ramp down and land

**Shadow controller**: Every loop iteration the geometric controller evaluates what it *would* command (roll/pitch/thrust) given the same firmware state and trajectory reference. These shadow commands are **logged to the CSV but never sent to the drone**. The firmware `setpoint_position` track always controls the drone.

**CSV output**: 37-column timestamped CSV in `runs/`:

```
t, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
roll, pitch, yaw, wx, wy, wz, thrust,
mekf_x, mekf_y, mekf_z, mekf_vx, mekf_vy, mekf_vz,
mekf_roll, mekf_pitch, mekf_yaw,
ref_x, ref_y, ref_z, ref_yaw,
shadow_roll, shadow_pitch, shadow_yaw_rate, shadow_thrust,
flow_x, flow_y, range_mm, vbat, maneuver, ts
```

---

## Shadow Controller Evaluation

After each flight, run:
```bash
PYENV_VERSION=flying_robots python3 scripts/plot_shadow_eval.py
```

`plot_shadow_eval.py` loads the newest CSV from `runs/` (auto-detected), filters to the airborne window, and plots:
- Shadow roll/pitch vs firmware roll/pitch
- Shadow thrust vs firmware thrust
- Shadow XY trajectory vs reference vs firmware XY
- Per-axis RMSE summary

**Target**: roll + pitch RMSE < 5° airborne before switching the shadow to active control.

---

## Safety Module (`src/safety.rs`)

`SafetyLimits` struct: altitude, speed, and geofence limits. Clamping and violation checks. Used by all simulation binaries (assignments 4, 5). Emergency hover + landing triggered on violation. All maneuvers start and end with a stable hover.

---

## File Structure

```
flying_drone_stack/
├── Cargo.toml
├── README.md
├── verify_all.sh
├── configs/
│   └── flight_defaults.toml
├── docs/
│   ├── ARCHITECTURE.md          # This file
│   └── DRONE_STACK_ASPECTS.md
├── scripts/
│   ├── plot_assignment1.py
│   ├── plot_assignment2.py
│   ├── plot_assignment3.py
│   ├── plot_assignment4.py
│   ├── plot_assignment5.py
│   ├── plot_mekf_eval.py
│   └── plot_shadow_eval.py
├── src/
│   ├── lib.rs
│   ├── safety.rs
│   ├── math/          vec3 · quaternion · matrix (Mat9)
│   ├── dynamics/      state · params · simulator
│   ├── integration/   euler · rk4 · exponential (ExpEuler, ExpRK4)
│   ├── controller/    GeometricController (SE(3) Lee et al. 2010)
│   ├── trajectory/    Figure8 · Circle · CsvTraj · Takeoff · Sequenced
│   ├── planning/      SplineTrajectory (QP) · flatness chain
│   ├── estimation/    Mekf (predict · height · flow · zero-gate)
│   ├── flight/        build_state · force_vector_to_rpyt · yaw_rate_cmd
│   └── bin/
│       ├── assignment1.rs
│       ├── assignment2.rs
│       ├── assignment3.rs
│       ├── assignment4.rs
│       ├── assignment5.rs
│       ├── demo.rs
│       ├── mekf_eval.rs
│       ├── sim_closed_loop.rs
│       └── main.rs              ← real hardware binary
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
└── runs/
    ├── hover_<timestamp>.csv
    ├── circle_<timestamp>.csv
    └── figure8_<timestamp>.csv
```

---

## Testing

```bash
cargo test
# Expected: 248 tests, 0 failures
```

Coverage: MEKF predict/update/flow gate, flight math, geometric controller gains, differential flatness chain, hover control loop convergence, safety limits, spline planner continuity.

---

## Build & Run

```bash
# Library + all binaries
cargo build --release

# Tests
cargo test

# Full pipeline verification
bash verify_all.sh

# Assignment simulations (example)
cargo run --release --bin assignment5 -- circle
PYENV_VERSION=flying_robots python3 scripts/plot_assignment5.py

# Offline MEKF evaluation
cargo run --release --bin mekf_eval -- runs/circle_2026-03-15_11-46-16.csv

# Real hardware flight
cargo build --release --bin main
./target/release/main
PYENV_VERSION=flying_robots python3 scripts/plot_shadow_eval.py
```

---

## Design Principles

1. **Separation of concerns** — each module has one well-defined responsibility; math doesn't know about physics, dynamics doesn't know about integration methods.
2. **Dependency inversion** — `Simulator` depends on the `Integrator` *trait*, not on concrete implementations.
3. **Open/closed** — add a new integrator by implementing one trait; add a new aircraft by adding a `MultirotorParams` constructor.
4. **Testability** — every component is independently testable; no global state.
5. **Frame discipline** — hardware helpers always use firmware EKF coordinates; MEKF output is clearly labelled and never mixed into firmware-frame commands.

---

## Component Interaction Example — Simulation loop (assignment2)

```rust
let params = MultirotorParams::crazyflie();
let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
let controller = GeometricController::default();
let trajectory = CircleTrajectory::new(0.5, 0.5, 0.3);

let mut t = 0.0_f32;
while t < 10.0 {
    let reference = trajectory.get_reference(t);
    let control = controller.compute_control(sim.state(), &reference, &params);
    let action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
    sim.step(&action);
    t += 0.01;
}
```

## Component Interaction Example — Real hardware loop (main)

```
every 10 ms:
  read firmware log row → build_state (firmware EKF frame)
  trajectory.get_reference(t) → ref_pos, ref_yaw
  firmware_track: setpoint_position(ref_x, ref_y, ref_z, ref_yaw)  ← sent to drone
  shadow_track:   controller.compute_control(state, ref)            ← logged only
  mekf.predict(imu) + mekf.update_height(range) + mekf.update_flow(flow, pz)
  write CSV row
```

---

## Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| `crazyflie-lib` | 0.4 | CRTP protocol — logging, parameters, commander |
| `crazyflie-link` | 0.3 | Radio/USB link layer |
| `tokio` | 1 | Async runtime for concurrent log stream reading |
| `chrono` | 0.4 | UTC timestamps for CSV filenames |
| `simple_qp` | — | QP solver for minimum-snap spline planner |
| `serde` | 1 | Config deserialisation (toml) |

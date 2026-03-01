# Project Architecture

## Overview

This project implements a clean, modular multirotor dynamics simulator and state estimator following software engineering best practices. It covers the full perception-to-control stack: IMU-driven MEKF state estimation, SE(3) geometric control, and pluggable trajectory generation — all in f32, no-std compatible, and validated against Crazyflie hardware logs.

## Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                        Application Layer                          │
│  assignment1 · assignment2 · assignment3 · assignment4 ·          │
│  assignment5 · demo · debug/test bins                             │
└────────────────────────────┬─────────────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────────────┐
│                   Multirotor Simulator Library                    │
├───────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │  Math    │  │ Dynamics │  │Integr.   │  │Controller│        │
│  │  Vec3    │  │  State   │  │  Euler   │  │Geometric │        │
│  │  Quat    │  │  Params  │  │  RK4     │  │SE(3) Lee │        │
│  │  Mat9    │  │  Simul.  │  │  Exp     │  │          │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
│                                                                   │
│  ┌───────────────┐  ┌──────────────────────────┐  ┌────────────┐ │
│  │   Planning    │  │       Trajectory         │  │ Estimation │ │
│  │  Spline ·     │  │  Figure8 · Circle · CSV  │  │  MEKF      │ │
│  │  Flatness     │  │  Takeoff · Sequenced     │  │  f32 ·     │ │
│  │  Minimum-snap │  │                          │  │  Joseph    │ │
│  │  QP           │  │                          │  │  capped Σ  │ │
│  └───────────────┘  └──────────────────────────┘  └────────────┘ │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

## Module Responsibilities
### `planning/` — Motion Planning & Trajectory Optimization
**Purpose**: Advanced trajectory generation using minimum-snap splines and differential flatness.

- `spline.rs`: 8th-order polynomial spline planner (QP, minimum-snap, continuity up to snap)
- `flatness.rs`: Differential flatness chain for multirotor (position/yaw + derivatives → rotation, thrust, angular velocity, torque)
- `mod.rs`: Module entry point, re-exports planner and flatness types

**Key types**: `SplineTrajectory`, `Waypoint`, `FlatOutput`, `compute_flatness`

**Features**:
  - Minimum-snap spline optimization (Clarabel/simple_qp backend)
  - Dynamically feasible, smooth trajectories for aggressive maneuvers
  - Full flatness chain for feedforward control and reference generation
  - Validated against closed-loop geometric controller tracking
  - **Safety integration:** All maneuvers start and end with a stable hover. Altitude, speed, and geofence limits are enforced in simulation. Emergency hover and landing logic is triggered on violation.
### `safety` (`src/safety.rs`) — Safety Limits and Emergency Handling
**Purpose**: Enforce safety constraints and provide emergency override logic for simulation and real flight.

- `safety.rs` lives directly in `src/` (not a subdirectory).
- Defines `SafetyLimits` (altitude, speed, geofence), clamping, and violation checks.
- Integrated in all simulation binaries (e.g., `assignment4.rs`, `assignment5.rs`).

**Features**:
  - Altitude, speed, and geofence limits
  - Clamping of position/velocity if limits are exceeded
  - Logging and warnings for violations
  - Emergency override: stable hover and controlled landing if a violation occurs
  - All flight programs start and end with a stable hover for safe takeoff/landing

### `math/` — Mathematical Primitives
**Purpose**: Pure mathematical operations, zero external dependencies.

- `vec3.rs`: 3D vectors for position, velocity, force, torque
- `quaternion.rs`: Unit quaternion for 3D rotations without gimbal lock
- `matrix.rs`: `Mat9` — 9×9 f32 matrix for MEKF covariance arithmetic. Key methods:
  - `mat_mul`, `transpose`, `scale`, `add` — standard matrix algebra
  - `joseph_update(Σ, k, h, r)` — full Joseph form `(I−KH)Σ(I−KH)ᵀ + K·r·Kᵀ`, keeps Σ positive-definite
  - `symmetrise()` — forces `Σ ← (Σ+Σᵀ)/2` after every predict/update
  - `clamp_diagonal(max_var)` — caps each diagonal entry, scaling off-diagonal terms proportionally

**Key Traits**: `Copy`, `Clone`, `Add`, `Mul`

### `dynamics/` — Physics Core
**Purpose**: Physical system representation and dynamics equations.

- `state.rs`: `MultirotorState` (position, velocity, quaternion, angular velocity) and `MotorAction` (motor speeds + control allocation)
- `params.rs`: `MultirotorParams` — Crazyflie 2.1 physical parameters (mass, inertia, motor constants, arm length)
- `simulator.rs`: `MultirotorSimulator` — main simulation engine, wires state + integrator + derivatives

**Design Pattern**: Strategy pattern — integrator is injected via `Box<dyn Integrator>`.

### `integration/` — Numerical Integration
**Purpose**: Pluggable time-stepping algorithms.

- `euler.rs`: First-order forward Euler
- `rk4.rs`: Fourth-order Runge-Kutta (quaternion integration bug-fixed)
- `exponential.rs`: Exponential map methods (ExpEuler, ExpRK4) for stiff quaternion dynamics

**Design Pattern**: Trait-based polymorphism via `Integrator` trait.

### `controller/` — Control Algorithms
**Purpose**: Compute thrust and torques from state error and trajectory reference.

- `mod.rs`: `GeometricController` — SE(3) geometric controller based on Lee et al. (2010). Uses Crazyflie firmware gains. Implements full position PD + feedforward and attitude PD control.

**Key types**: `GeometricController`, `TrajectoryReference`, `ControlOutput`

### `trajectory/` — Trajectory Generators
**Purpose**: Provide position/velocity/acceleration reference signals as a function of time.

- `mod.rs`: All trajectory types:
  - `Figure8Trajectory` — polynomial figure-8 with configurable amplitude and `time_scale`
  - `CircleTrajectory` — horizontal circle with configurable radius, height, angular velocity
  - `CsvTrajectory` — interpolates waypoints from a CSV file
  - `TakeoffTrajectory` — minimum-jerk polynomial takeoff from ground to target height
  - `SequencedTrajectory` — chains multiple trajectories with per-phase durations

**Design Pattern**: Trait-based polymorphism via `Trajectory` trait.

### `estimation/` — State Estimation
**Purpose**: Sensor-fusion algorithms that recover full vehicle state from noisy IMU, range, and flow measurements.

- `mekf.rs`: `Mekf` — Multiplicative Extended Kalman Filter.
  - **State**: `x = [p(3), b(3), δ(3)]` ∈ ℝ⁹ with `q_ref` maintained outside the filter
    - `p`: position in world frame [m]
    - `b`: velocity in body frame [m/s]
    - `δ`: attitude error angles [rad] — reset to zero after each `q_ref` update
  - **Inputs**: gyroscope [deg/s] + accelerometer [G] as control input (not measurements)
  - **Measurements**: height from range sensor (mm→m) and optical flow (pixels)
  - **Numerical stability** (all three required for f32 on STM32-class hardware):
    1. Full Joseph form covariance update — prevents loss of positive-definiteness
    2. Symmetrisation `Σ ← (Σ+Σᵀ)/2` — suppresses f32 rounding asymmetry
    3. Diagonal cap `MAX_COVARIANCE = 100.0` — mirrors Crazyflie `kalman_core.c`
  - **Tuned noise params**: `q_pos=1e-7, q_vel=1e-3, q_att=1e-6, r_height=1e-3, r_flow=8.0`
  - **Sensor constants**: `NP=350.0` pixels, `THETA_P=0.71674` rad/pixel
  - **Validated against**: Crazyflie fr00.csv figure-8 log (7.2 s, ~1 kHz)
    - Orientation RMS vs on-board EKF: roll 0.87°, pitch 1.02°, yaw 0.21°
    - Position RMS vs on-board EKF: x 0.088 m, y 0.127 m, z 0.006 m

**Key types**: `Mekf`, `MekfState`, `MekfParams`, `quat_to_euler`

## Design Principles

### 1. Separation of Concerns
Each module has a single, well-defined responsibility:
- Math doesn't know about physics
- Dynamics doesn't know about integration methods
- Integration methods don't know about specific aircraft

### 2. Dependency Inversion
High-level modules don't depend on low-level modules:
```
Simulator depends on Integrator trait (abstraction)
   └─> Not on specific integrators (concrete implementations)
```

### 3. Open/Closed Principle
Easy to extend without modifying existing code:
- Add new integrator: implement `Integrator` trait
- Add new aircraft: create new `MultirotorParams`
- Add new validation: create new module

### 4. Testability
Each component is independently testable:
- Math functions have unit tests
- Integrators can be tested in isolation
- Simulator can use mock integrators

## File Organization

```
multirotor_simulator/
├── Cargo.toml                    # Project metadata & dependencies
├── README.md                     # User documentation
├── ARCHITECTURE.md               # This file
├── QUICKSTART.sh                 # Shell quick-start guide
├── plot_assignment1.py           # Assignment 1 plotting
├── plot_assignment2.py           # Assignment 2 plotting (Normal + Realistic)
├── plot_assignment4.py           # Assignment 4 plotting
├── plot_assignment5.py           # Assignment 5 plotting
│
├── src/
│   ├── lib.rs                    # Library entry point & prelude
│   ├── safety.rs                 # SafetyLimits: altitude/speed/geofence, emergency land
│   │
│   ├── math/
│   │   ├── mod.rs
│   │   ├── vec3.rs
│   │   ├── quaternion.rs
│   │   └── matrix.rs             # Mat9: 9×9 f32 matrix (Joseph form, symmetrise, clamp)
│   │
│   ├── dynamics/
│   │   ├── mod.rs
│   │   ├── state.rs
│   │   ├── params.rs
│   │   └── simulator.rs
│   │
│   ├── integration/
│   │   ├── mod.rs
│   │   ├── euler.rs
│   │   ├── rk4.rs
│   │   └── exponential.rs
│   │
│   ├── controller/
│   │   └── mod.rs               # GeometricController (SE(3) Lee)
│   │
│   ├── trajectory/
│   │   └── mod.rs               # All trajectory types
│   │
│   ├── planning/
│   │   ├── mod.rs               # Motion planning module (Assignment 4)
│   │   ├── spline.rs            # Minimum-snap spline planner (QP)
│   │   └── flatness.rs          # Differential flatness chain
│   │
│   ├── estimation/
│   │   ├── mod.rs               # Re-exports Mekf, MekfState, MekfParams
│   │   └── mekf.rs              # Full MEKF: predict, height update, flow update
│   │
│   └── bin/
│       ├── assignment1.rs       # Integrator comparison
│       ├── assignment2.rs       # Geometric control (--realistic-start)
│       ├── assignment3.rs       # MEKF offline validation vs on-board EKF
│       ├── assignment4.rs       # Minimum-snap spline planning & differential flatness
│       ├── assignment5.rs       # Safe-space simulation (hover/circle/figure-8 + MEKF)
│       ├── demo.rs              # Quick demo
│       ├── check_saturation.rs
│       ├── check_trajectory.rs
│       ├── debug_*.rs           # Debug / diagnostic binaries
│       └── test_*.rs            # Standalone test binaries
│
├── tests/
│   └── test_geometric_controller.rs   # Integration tests
│
├── Controls/                          # Real hardware flight (Python / cflib)
│   └── run_assignment5_onboard.py     # Onboard flight: hover, circle, figure-8
│
└── results/
    ├── data/                    # CSV outputs
    │   ├── assignment2_<scenario>.csv
    │   ├── assignment3_mekf.csv # MEKF: time, roll_rad, pitch_rad, yaw_rad, x, y, z
    │   ├── assignment3_ekf.csv  # On-board EKF reference (same columns)
    │   ├── assignment4_planned.csv      # Planned minimum-snap spline trajectory
    │   ├── assignment4_openloop.csv     # Open-loop simulation (flatness feedforward)
    │   └── assignment4_closedloop.csv   # Closed-loop simulation (geometric controller)
    └── images/                  # PNG plots
        ├── assignment4_3d.png
        ├── assignment4_position_cl.png
        ├── assignment4_velocity_cl.png
        ├── assignment4_actions.png
        ├── assignment4_omega.png
        ├── assignment4_errors.png
        ├── assignment4_actions_comparison.png
        └── assignment4_derivatives.png
```

## Component Interactions

### Simulation loop (assignment2)

```rust
let params = MultirotorParams::crazyflie();
let integrator = Box::new(RK4Integrator);
let mut sim = MultirotorSimulator::new(params.clone(), integrator);

let controller = GeometricController::default();
let trajectory = CircleTrajectory::new(0.5, 0.5, 0.3);

let mut time = 0.0_f32;
while time < 10.0 {
    let reference = trajectory.get_reference(time);
    let control = controller.compute_control(sim.state(), &reference, &params);
    let action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
    sim.step(&action);
    time += 0.01;
}
```

### Sequenced / realistic-start

```rust
let takeoff = TakeoffTrajectory::new(0.0, 0.0, 0.5, 3.0, 0.0);
let hover   = HoverTrajectory::new(Vec3::new(0.0, 0.0, 0.5));
let circle  = CircleTrajectory::new(0.5, 0.5, 0.3);

let seq = SequencedTrajectory::new(vec![
    (3.0, Box::new(takeoff)),
    (2.0, Box::new(hover)),
    (10.0, Box::new(circle)),
]);
// seq.get_reference(t) delegates to the active phase
```

## Extensibility Points

### Adding a new integration method
1. Create `src/integration/mymethod.rs`, implement `Integrator` trait
2. Export in `src/integration/mod.rs`
3. Use: `Box::new(MyMethodIntegrator)`

### Adding a new trajectory
1. Add a struct to `src/trajectory/mod.rs`, implement `Trajectory` trait
2. Export in `src/lib.rs`

### Adding a new aircraft model
1. Add a constructor to `MultirotorParams` in `src/dynamics/params.rs`

## Testing

```bash
cargo test                         # all tests
cargo test --test test_geometric_controller  # integration test only
```

Current status: **69 tests pass**, 1 known pre-existing failing test (`test_geometric_controller_creation`).

## Build & Run

```bash
cargo build --release

# Assignment 1 — integrator comparison
cargo run --release --bin assignment1
python plot_assignment1.py

# Assignment 2 — geometric control (both modes)
cargo run --release --bin assignment2
cargo run --release --bin assignment2 -- --realistic-start
python plot_assignment2.py

# Assignment 3 — MEKF offline validation
cargo run --release --bin assignment3 -- --csv "../State Estimation/logging_ekf/logging/fr00.csv"
# then from the State Estimation/ directory:
python plot_assignment3.py      # MEKF orientation + position vs on-board EKF
python plot_comparison.py       # Three-way: Python MEKF · Rust MEKF · on-board EKF

# Assignment 4 — Minimum-snap spline planning & differential flatness
cargo run --release --bin assignment4
python plot_assignment4.py

# Assignment 5 — Safe-space simulation (hover / circle / figure-8 + MEKF)
cargo run --release --bin assignment5
python plot_assignment5.py

# Real hardware flight (Crazyflie + Flow deck)
python Controls/run_assignment5_onboard.py hover
python Controls/run_assignment5_onboard.py circle
python Controls/run_assignment5_onboard.py figure8
```

## Benefits of This Architecture

1. **Clarity**: Each file has ~100-200 lines, easy to understand
2. **Maintainability**: Changes localized to specific modules
3. **Reusability**: Library can be used in multiple projects
4. **Testability**: Each component independently testable
5. **Scalability**: Easy to add features without breaking existing code
6. **Documentation**: Clear module boundaries and responsibilities
7. **Safety**: Built-in safety logic for simulation and real flight, including emergency handling and safe maneuver transitions


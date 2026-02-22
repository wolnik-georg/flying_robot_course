# Project Architecture

## Overview

This project implements a clean, modular multirotor dynamics simulator following software engineering best practices.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
│  (Binaries: assignment1, assignment2, demo, debug/test bins) │
└───────────────────────┬─────────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────────┐
│                  Multirotor Simulator Library                │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │  Math    │  │ Dynamics │  │Integr.   │  │Controller│  │
│  │  Vec3    │  │  State   │  │  Euler   │  │Geometric │  │
│  │  Quat    │  │  Params  │  │  RK4     │  │SE(3) Lee │  │
│  │          │  │  Simul.  │  │  Exp     │  │          │  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                   Trajectory                         │  │
│  │  Figure8 · Circle · CSV · Takeoff · Sequenced        │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

## Module Responsibilities

### `math/` — Mathematical Primitives
**Purpose**: Pure mathematical operations, zero external dependencies.

- `vec3.rs`: 3D vectors for position, velocity, force, torque
- `quaternion.rs`: Unit quaternion for 3D rotations without gimbal lock

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
│
├── src/
│   ├── lib.rs                    # Library entry point & prelude
│   │
│   ├── math/
│   │   ├── mod.rs
│   │   ├── vec3.rs
│   │   └── quaternion.rs
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
│   └── bin/
│       ├── assignment1.rs       # Assignment 1: integrator comparison
│       ├── assignment2.rs       # Assignment 2: geometric control (--realistic-start)
│       ├── demo.rs              # Quick demo
│       ├── check_saturation.rs
│       ├── check_trajectory.rs
│       ├── debug_control.rs
│       ├── debug_controller.rs
│       ├── debug_figure8.rs
│       ├── debug_geometric.rs
│       ├── debug_jerk.rs
│       ├── debug_pipeline.rs
│       ├── debug_rotation.rs
│       ├── debug_rotation_error.rs
│       ├── test_coordinates.rs
│       ├── test_equivalence.rs
│       ├── test_euler.rs
│       ├── test_gains.rs
│       ├── test_motor_mixing.rs
│       ├── test_oscillation.rs
│       └── test_strong_damping.rs
│
├── tests/
│   └── test_geometric_controller.rs   # Integration tests
│
└── results/
    ├── data/                    # CSV outputs (assignment2_<scenario>{_realistic}.csv)
    └── images/                  # PNG plots (*_paths.png, *_errors.png)
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

# Assignment 1
cargo run --release --bin assignment1
python plot_assignment1.py

# Assignment 2 (both modes)
cargo run --release --bin assignment2
cargo run --release --bin assignment2 -- --realistic-start
python plot_assignment2.py
```

## Benefits of This Architecture

1. **Clarity**: Each file has ~100-200 lines, easy to understand
2. **Maintainability**: Changes localized to specific modules
3. **Reusability**: Library can be used in multiple projects
4. **Testability**: Each component independently testable
5. **Scalability**: Easy to add features without breaking existing code
6. **Documentation**: Clear module boundaries and responsibilities


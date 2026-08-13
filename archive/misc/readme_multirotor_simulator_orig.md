# Multirotor Dynamics Simulator

A modular, high-performance Rust library for simulating multirotor dynamics with geometric control and MEKF state estimation, designed for the Flying Robot Course assignments.

## Overview

This project implements a complete multirotor perception-to-control stack: IMU-driven Multiplicative Extended Kalman Filter (MEKF) for state estimation, SE(3) geometric control, and pluggable trajectory generation. It targets the Crazyflie nano-quadcopter and is written entirely in f32 with no heap allocation in any loop — structurally identical to firmware code to make C porting mechanical.

### Key Features

- **Geometric Control**: SE(3) geometric controller implementation based on Lee et al. (2010)
- **MEKF State Estimation**: Attitude (quaternion) + position/velocity from IMU, range and optical-flow fusion
- **Modular Architecture**: Pluggable integrators, controllers, and trajectory generators
- **Embedded Ready**: f32 precision and no_std compatibility for hardware deployment
- **Comprehensive Testing**: Unit tests and integration tests for validation
- **Trajectory Support**: Figure-8 patterns, circular trajectories, and CSV waypoint tracking

## Architecture

The simulator follows a clean, modular architecture with clear separation of concerns:

```
multirotor_simulator/
├── math/           # Vector, quaternion and 9×9 matrix mathematics
├── dynamics/       # Physical models and simulation
│   ├── state.rs    # State representation and motor actions
│   ├── params.rs   # Aircraft parameters and dynamics
│   └── simulator.rs # Main simulation engine
├── integration/    # Numerical integration methods
├── controller/     # Control algorithms
├── trajectory/     # Trajectory generation
└── estimation/     # State estimation (MEKF)
    └── mekf.rs     # Attitude + position Kalman filter (f32, numerically stable)
```

### Core Components

#### 1. Mathematics (`math/`)
- **Vec3**: 3D vector with operator overloading
- **Quat**: Unit quaternion for 3D rotations
- **Mat9**: 9×9 f32 matrix for MEKF covariance — full Joseph form update, symmetrisation, diagonal clamping

#### 2. Dynamics (`dynamics/`)
- **MultirotorState**: Complete 13-DOF state (position, velocity, orientation, angular velocity)
- **MultirotorParams**: Physical parameters (mass, inertia, motor constants)
- **MultirotorSimulator**: Main simulation engine with pluggable integrators
- **MotorAction**: Control allocation for X-configuration quadcopters

#### 3. Integration (`integration/`)
- **Integrator** trait for numerical integration methods
- **EulerIntegrator**: First-order explicit Euler
- **RK4Integrator**: Fourth-order Runge-Kutta
- **ExpEulerIntegrator**: Exponential Euler for stiff systems
- **ExpRK4Integrator**: Exponential Runge-Kutta

#### 4. Controller (`controller/`)
- **Controller** trait for control algorithms
- **GeometricController**: SE(3) geometric controller implementation
- **TrajectoryReference**: Reference signals for tracking control

#### 5. Trajectory (`trajectory/`)
- **Trajectory** trait for trajectory generators
- **Figure8Trajectory**: Polynomial-based figure-8 patterns
- **CircleTrajectory**: Circular trajectories with configurable parameters
- **CsvTrajectory**: Waypoint interpolation from CSV files

#### 6. Estimation (`estimation/`)
- **Mekf**: Multiplicative Extended Kalman Filter — fuses IMU, height (range sensor) and optical flow
  - State `x = [p(3), b(3), δ(3)]` ∈ ℝ⁹, reference quaternion `q_ref` maintained outside filter
  - Three numerical stability measures for f32: full Joseph-form update, Σ symmetrisation, diagonal cap `MAX_COVARIANCE=100.0` (mirrors Crazyflie `kalman_core.c`)
  - Validated on real Crazyflie figure-8 flight log: orientation RMS ≤ 1.02°, position RMS ≤ 0.13 m vs on-board EKF

## Usage

### Basic Simulation

```rust
use multirotor_simulator::prelude::*;

// Create aircraft parameters
let params = MultirotorParams::crazyflie();

// Create simulator with RK4 integration
let integrator = Box::new(RK4Integrator);
let mut simulator = MultirotorSimulator::new(params.clone(), integrator);

// Create geometric controller
let controller = GeometricController::default();

// Define hover reference
let hover_ref = TrajectoryReference {
    position: Vec3::new(0.0, 0.0, 0.5),
    velocity: Vec3::zero(),
    acceleration: Vec3::zero(),
    yaw: 0.0,
    yaw_rate: 0.0,
    yaw_acceleration: 0.0,
};

// Simulation loop
let dt = 0.01;
for _ in 0..1000 {
    let state = simulator.state();
    let control = controller.compute_control(state, &hover_ref, &params);
    let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
    simulator.step(&motor_action);
}
```

### Trajectory Tracking

```rust
// Create circular trajectory
let trajectory = CircleTrajectory::new(0.5, 0.3, 0.5); // radius, height, angular velocity

// Simulation with trajectory tracking
let mut time = 0.0;
let dt = 0.01;
while time < 10.0 {
    let state = simulator.state();
    let reference = trajectory.get_reference(time);
    let control = controller.compute_control(state, &reference, &params);
    let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
    simulator.step(&motor_action);
    time += dt;
}
```

## Building and Testing

### Prerequisites
- Rust 1.70+ with 2021 edition
- Cargo package manager

### Build
```bash
cd multirotor_simulator
cargo build --release
```

### Run Tests
```bash
# Run all tests
cargo test

# Run specific test suites
cargo test controller::tests  # Controller unit tests
cargo test test_geometric_controller  # Integration tests
```

### Run Examples
```bash
# Run demo simulation
cargo run --bin demo

# Run assignment 1 validation
cargo run --bin assignment1

# Run assignment 2 (geometric control)
cargo run --release --bin assignment2
cargo run --release --bin assignment2 -- --realistic-start

# Run assignment 3 (MEKF offline validation)
cargo run --release --bin assignment3 -- --csv "../State Estimation/logging_ekf/logging/fr00.csv"
```

## Assignment Context

This project implements assignments from the Flying Robot Course:

### Assignment 1
- Basic multirotor dynamics simulation
- Integration method comparison (Euler, RK4, ExpEuler, ExpRK4)
- Hover control validation

### Assignment 2
- Geometric control on SE(3) manifold
- Trajectory tracking (figure-8, circle, CSV waypoints)
- Motor control allocation for X-configuration quadcopters
- Hardware-ready implementation for Crazyflie deployment

### Assignment 3
- MEKF state estimation: attitude + position/velocity from IMU + range + optical flow
- Offline validation against real Crazyflie figure-8 flight log (`fr00.csv`)
- Three-way comparison: Rust MEKF · Python MEKF · on-board EKF
- f32 numerical stability: Joseph form + symmetrisation + covariance cap
- Results: orientation RMS ≤ 1.02°, position RMS ≤ 0.13 m vs on-board EKF

## API Reference

### Key Types

- **MultirotorSimulator**: Main simulation interface
- **GeometricController**: SE(3) trajectory tracking controller
- **TrajectoryReference**: Reference signals for control
- **MotorAction**: Motor speed commands
- **MultirotorState**: Complete aircraft state

### Control Laws

The geometric controller implements:

**Position Control:**
```
Fd = m(p̈d + Kp ep + Kv ev + gez)
```

**Attitude Control:**
```
τ = -KR er - Kω eω + ω×Jω - J(R^T Rd ω̇d - R^T Rd ωd_dot)
```

Where:
- `ep, ev`: Position and velocity errors
- `er, eω`: Rotation and angular velocity errors
- `Kp, Kv, KR, Kω`: Controller gain matrices
- `gez = [0,0,-g]`: Gravity vector

### Motor Mixing

X-configuration quadcopter control allocation:

```
ω₁² = F/(4kf) - τ_x/(4kf·l) - τ_y/(4kf·l) - τ_z/(4kd)
ω₂² = F/(4kf) + τ_x/(4kf·l) - τ_y/(4kf·l) + τ_z/(4kd)
ω₃² = F/(4kf) + τ_x/(4kf·l) + τ_y/(4kf·l) - τ_z/(4kd)
ω₄² = F/(4kf) - τ_x/(4kf·l) + τ_y/(4kf·l) + τ_z/(4kd)
```

## Performance

- **Real-time capable**: Sub-millisecond simulation steps
- **Numerically stable**: Proper handling of quaternion singularities
- **Memory efficient**: No dynamic allocation in control loops
- **Embedded ready**: f32 precision, deterministic execution

## Contributing

This is an educational project for the Flying Robot Course. For questions or improvements, please refer to the course materials or contact the course staff.

## License

Educational use only - part of the Flying Robot Course curriculum. 
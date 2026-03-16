# Multirotor Simulator

A modular, well-architected multirotor dynamics simulator for flying robots research and education.

## Quick Start

```bash
# Navigate to the project
cd /home/georg/Desktop/flying_robot_course/multirotor_simulator

# Run the demo (fastest way to see it work!)
cargo run --bin demo

# Run tests
cargo test

# Build optimized version
cargo build --release
```

## Available Programs

### 📊 Assignment 1 — Integrator comparison
```bash
cargo run --release --bin assignment1
python plot_assignment1.py
```
Compares Euler, RK4, ExpEuler, ExpRK4 integrators on a free-flight trajectory. Writes CSVs to `results/data/`.

### 🚁 Assignment 2 — Geometric control & trajectory tracking
```bash
cargo run --release --bin assignment2
cargo run --release --bin assignment2 -- --realistic-start
python plot_assignment2.py
```
Tracks a figure-8 trajectory with the SE(3) geometric controller. `--realistic-start` prepends takeoff + hover. Writes CSVs to `results/data/assignment2_<scenario>.csv`.

### 🎯 Assignment 3 — MEKF offline validation
```bash
cargo run --release --bin assignment3 -- --csv "../State Estimation/logging_ekf/logging/fr00.csv"
# then from State Estimation/:
python plot_assignment3.py
python plot_comparison.py
```
Runs the Multiplicative EKF offline against a real Crazyflie flight log. Compares Rust MEKF, Python MEKF, and on-board EKF.

### 🏁 Assignment 4 — Minimum-snap spline planning & differential flatness
```bash
cargo build --release --bin assignment4
cargo run --release --bin assignment4
python plot_assignment4.py
```
Generates planned, open-loop, and closed-loop figure-8 CSVs in `results/data/`. Produces 3D trajectory, tracking, action, and error plots in `results/images/`.

### � Assignment 5 — Safe-space simulation (hover / circle / figure-8)
```bash
cargo run --release --bin assignment5
python plot_assignment5.py
```
Simulates hover, circle, and figure-8 trajectories inside a 1.0 × 1.0 m safety box (max 0.30 m height) using the full MEKF estimator + geometric controller. For **real hardware flight**, use `Controls/run_assignment5_onboard.py`.

### 🎯 Demo — Quick demonstration
```bash
cargo run --bin demo
```
Shows all 4 integration methods in action. Output:
```
Euler        | z =    2.879 m | vz =   52.352 m/s
RK4          | z =    2.618 m | vz =   52.352 m/s
Exp+Euler    | z =    2.879 m | vz =   52.352 m/s
Exp+RK4      | z =    2.618 m | vz =   52.352 m/s
```

### 📊 Test Equivalence — Verification
```bash
cargo run --bin test_equivalence
```
Runs all methods and exports CSV files for comparison with original implementation.

## Using as a Library

```rust
use multirotor_simulator::prelude::*;

fn main() {
    let params = MultirotorParams::crazyflie();
    let integrator = Box::new(RK4Integrator);
    let mut sim = MultirotorSimulator::new(params.clone(), integrator);

    let controller = GeometricController::default();
    let trajectory = CircleTrajectory::new(0.5, 0.5, 0.3);

    let dt = 0.01;
    let mut time = 0.0_f32;
    while time < 10.0 {
        let reference = trajectory.get_reference(time);
        let control = controller.compute_control(sim.state(), &reference, &params);
        let action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
        sim.step(&action);
        time += dt;
    }
}
```

## Project Structure

```
multirotor_simulator/
├── Cargo.toml              # Project configuration
├── README.md               # This file
├── ARCHITECTURE.md         # Detailed architecture documentation
├── QUICKSTART.sh           # Quick-start shell script
├── plot_assignment1.py     # Plotting: Assignment 1 integrator comparison
├── plot_assignment2.py     # Plotting: Assignment 2 geometric control
├── plot_assignment4.py     # Plotting: Assignment 4 spline planning
├── plot_assignment5.py     # Plotting: Assignment 5 safe-space simulation
├── src/
│   ├── lib.rs              # Library entry point & prelude
│   ├── safety.rs           # SafetyLimits: altitude/speed/geofence, emergency land
│   ├── math/               # Mathematical primitives
│   │   ├── vec3.rs         # 3D vector (position, velocity, force, torque)
│   │   ├── quaternion.rs   # Unit quaternion (no gimbal lock)
│   │   └── matrix.rs       # Mat9: 9×9 f32 matrix (Joseph form, symmetrise, clamp)
│   ├── dynamics/           # Physics and dynamics
│   │   ├── state.rs        # MultirotorState & MotorAction
│   │   ├── params.rs       # MultirotorParams — Crazyflie 2.1 physical parameters
│   │   └── simulator.rs    # MultirotorSimulator — main simulation engine
│   ├── integration/        # Numerical integration methods
│   │   ├── euler.rs        # First-order forward Euler
│   │   ├── rk4.rs          # Runge-Kutta 4th order
│   │   └── exponential.rs  # Exponential map integrators (ExpEuler, ExpRK4)
│   ├── controller/
│   │   └── mod.rs          # GeometricController — SE(3) Lee et al. 2010
│   ├── trajectory/
│   │   └── mod.rs          # Figure8, Circle, CSV, Takeoff, Sequenced trajectories
│   ├── planning/           # Motion planning (Assignment 4)
│   │   ├── spline.rs       # Minimum-snap 8th-order polynomial QP planner
│   │   └── flatness.rs     # Differential flatness chain (position/yaw → full state)
│   ├── estimation/
│   │   └── mekf.rs         # MEKF: IMU predict + height/flow update (f32, Joseph form)
│   └── bin/                # Runnable binaries
│       ├── assignment1.rs  # Integrator comparison
│       ├── assignment2.rs  # Geometric control & trajectory tracking
│       ├── assignment3.rs  # MEKF offline validation vs on-board EKF
│       ├── assignment4.rs  # Minimum-snap planning & differential flatness
│       ├── assignment5.rs  # Safe-space simulation (hover/circle/figure-8 + MEKF)
│       ├── demo.rs         # Quick demo
│       ├── debug_*.rs      # Debug / diagnostic binaries
│       └── test_*.rs       # Standalone test binaries
├── tests/
│   └── test_geometric_controller.rs  # Integration tests
├── Controls/               # Real hardware flight scripts (Python / cflib)
│   └── run_assignment5_onboard.py    # Onboard flight: hover, circle, figure-8
└── results/
    ├── data/               # CSV outputs from simulation runs
    └── images/             # PNG plots generated by plot scripts
```

## Architecture

### Core Modules

1. **math/** - Mathematical primitives (`Vec3`, `Quat`, `Mat9`)
2. **dynamics/** - Physics core (state, Crazyflie 2.1 parameters, simulator)
3. **integration/** - Pluggable integrators (Euler, RK4, ExpEuler, ExpRK4)
4. **controller/** - Geometric SE(3) controller (Lee et al. 2010)
5. **trajectory/** - Trajectory generators: `Figure8Trajectory`, `CircleTrajectory`, `CsvTrajectory`, `TakeoffTrajectory`, `SequencedTrajectory`
6. **planning/** - Motion planning: minimum-snap spline planner, differential flatness chain
7. **estimation/** - `Mekf`: IMU-driven state estimator (predict + height/flow updates, f32, Joseph form)
8. **safety** (`safety.rs`) - `SafetyLimits`: altitude/speed/geofence enforcement, emergency hover/land

See `ARCHITECTURE.md` for detailed design documentation with data-flow diagrams and per-module API descriptions.

## Testing

```bash
cargo test
```

Current status: **69 tests pass**, 1 known pre-existing failing test (`test_geometric_controller_creation` — checks legacy hardcoded gain values, left as-is).

## Features

- ✅ Multiple integration methods (Euler, RK4, Exponential)
- ✅ Quaternion-based orientation (no gimbal lock)
- ✅ Geometric SE(3) controller (Lee et al. 2010) — Crazyflie-tuned gains
- ✅ Trajectory generators: figure-8, circle, CSV waypoints, takeoff, sequenced
- ✅ Realistic mission sequencing (`--realistic-start`)
- ✅ CSV output with phase metadata for plotting
- ✅ Side-by-side Normal vs Realistic comparison plots
- ✅ Crazyflie 2.1 parameters
- ✅ Clean, modular architecture
- ✅ **Safety features:** Altitude, speed, and geofence limits; clamping; logging; emergency hover/landing; all maneuvers start and end with a stable hover

## Design Principles

1. **Modularity**: Each component has a single, well-defined responsibility
2. **Testability**: Pure functions and clear interfaces
3. **Extensibility**: Easy to add new integration methods or aircraft models
4. **Performance**: Efficient implementations without sacrificing clarity
5. **Documentation**: Every module and function is documented

## License

Educational/Research use


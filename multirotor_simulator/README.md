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
### 🏁 Assignment 4 — Minimum-snap spline planning & differential flatness
```bash
cargo build --release --bin assignment4
cargo run --release --bin assignment4
```
Generates CSVs for planned, open-loop, and closed-loop figure-8 trajectory in `results/data/`. Plot with:
```bash
python plot_assignment4.py
```
Produces 3D trajectory, tracking, action, and error plots in `results/images/`.
### 📊 Assignment 1 — Integrator comparison
```bash
cargo run --release --bin assignment1
```
Runs all 4 integrators and writes CSVs to `results/data/`. Plot with:
```bash
python plot_assignment1.py
```

### 🚁 Assignment 2 — Geometric control & trajectory tracking
```bash
# Normal start (drone teleported to trajectory start)
cargo run --release --bin assignment2

# Realistic start (takeoff + hover prepended)
cargo run --release --bin assignment2 -- --realistic-start
```
Writes CSVs to `results/data/assignment2_<scenario>{_realistic}.csv`. Plot with:
```bash
python plot_assignment2.py
```
Produces side-by-side Normal vs Realistic path and error images in `results/images/`.

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
│   ├── planning/           # Motion planning module (assignment4)
│   │   ├── mod.rs          # Module entry point
│   │   ├── spline.rs       # Minimum-snap spline planner (QP)
│   │   └── flatness.rs     # Differential flatness chain
```
multirotor_simulator/
├── Cargo.toml              # Project configuration
├── README.md               # This file
├── ARCHITECTURE.md         # Architecture documentation
├── QUICKSTART.sh           # Quick-start shell script
├── plot_assignment1.py     # Plotting script for Assignment 1 results
├── plot_assignment2.py     # Plotting script for Assignment 2 results
├── src/
│   ├── lib.rs             # Library entry point
│   ├── math/              # Mathematical primitives
│   │   ├── mod.rs
│   │   ├── vec3.rs        # 3D vector
│   │   └── quaternion.rs  # Unit quaternion
│   ├── dynamics/          # Physics and dynamics
│   │   ├── mod.rs
│   │   ├── state.rs       # State representation & motor actions
│   │   ├── params.rs      # Physical parameters (Crazyflie)
│   │   └── simulator.rs   # Main simulation engine
│   ├── integration/       # Numerical integration methods
│   │   ├── mod.rs
│   │   ├── euler.rs       # First-order Euler
│   │   ├── rk4.rs         # Runge-Kutta 4th order
│   │   └── exponential.rs # Exponential map methods
│   ├── controller/        # Control algorithms
│   │   └── mod.rs         # Geometric SE(3) controller (Lee et al.)
│   ├── trajectory/        # Trajectory generators
│   │   └── mod.rs         # Figure-8, Circle, CSV, Takeoff, Sequenced
│   └── bin/               # Runnable binaries
│       ├── assignment1.rs  # Assignment 1: integrator comparison
│       ├── assignment2.rs  # Assignment 2: geometric control & trajectories
│       ├── demo.rs         # Quick demo
│       ├── debug_*.rs      # Debug / diagnostic binaries
│       └── test_*.rs       # Standalone test binaries
├── tests/
│   └── test_geometric_controller.rs  # Integration tests
└── results/
    ├── data/               # CSV output from simulation runs
    └── images/             # PNG plots generated by plot scripts
```

## Architecture
6. **planning/** - Motion planning: minimum-snap spline planner, differential flatness chain
### Core Modules

1. **math/** - Mathematical primitives (Vec3, Quat)
2. **dynamics/** - Physics core (state, parameters, simulator)
3. **integration/** - Pluggable integrators (Euler, RK4, Exponential)
4. **controller/** - Geometric SE(3) controller (Lee et al. 2010)
5. **trajectory/** - Trajectory generators: `Figure8Trajectory`, `CircleTrajectory`, `CsvTrajectory`, `TakeoffTrajectory`, `SequencedTrajectory`

See `ARCHITECTURE.md` for detailed design documentation.

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


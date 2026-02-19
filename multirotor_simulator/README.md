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

### 🎯 Demo - Quick demonstration
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

### 📊 Test Equivalence - Verification
```bash
cargo run --bin test_equivalence
```
Runs all methods and exports CSV files for comparison with original implementation.

### 📝 Assignment 1 - Full implementation (WIP)
```bash
cargo run --bin assignment1
```

## Using as a Library

```rust
use multirotor_simulator::prelude::*;

fn main() {
    // 1. Configure aircraft
    let params = MultirotorParams::crazyflie();
    
    // 2. Choose integration method
    let integrator = Box::new(RK4Integrator);  // or EulerIntegrator, etc.
    
    // 3. Create simulator
    let mut sim = MultirotorSimulator::new(params, integrator);
    
    // 4. Define control input
    let action = MotorAction::hover();
    
    // 5. Run simulation
    for step in 0..100 {
        sim.step(&action);
        println!("z = {:.3} m", sim.state().position.z);
    }
}
```

## Project Structure

```
multirotor_simulator/
├── Cargo.toml              # Project configuration
├── README.md               # This file
├── src/
│   ├── lib.rs             # Library entry point
│   ├── math/              # Mathematical primitives
│   │   ├── mod.rs         # Module definition
│   │   ├── vec3.rs        # 3D vector implementation
│   │   └── quaternion.rs  # Quaternion implementation
│   ├── dynamics/          # Physics and dynamics
│   │   ├── mod.rs         # Module definition
│   │   ├── state.rs       # State representation
│   │   ├── params.rs      # Physical parameters
│   │   └── simulator.rs   # Main simulator
│   ├── integration/       # Numerical integration methods
│   │   ├── mod.rs         # Module definition
│   │   ├── euler.rs       # Euler integration
│   │   ├── rk4.rs         # Runge-Kutta 4th order
│   │   └── exponential.rs # Exponential map methods
│   ├── validation/        # Validation and testing
│   │   ├── mod.rs         # Module definition
│   │   ├── synthetic.rs   # Synthetic validation
│   │   └── flight_data.rs # Real flight data validation
│   ├── io/                # Input/Output operations
│   │   ├── mod.rs         # Module definition
│   │   ├── csv_export.rs  # CSV data export
│   │   └── flight_log.rs  # Flight log parsing
│   └── bin/               # Binary executables
│       ├── assignment1.rs # Assignment 1 experiment
│       └── demo.rs        # Interactive demo
└── data/                  # Data files (CSV, logs, etc.)
```

## Architecture

### Core Modules

1. **math/** - Mathematical primitives (vectors, quaternions)
   - Zero dependencies, pure mathematical operations
   - Operator overloading for intuitive usage
   - Optimized for 3D robotics applications

2. **dynamics/** - Physics simulation core
   - State representation (position, velocity, orientation)
   - Physical parameters (mass, inertia, motor constants)
   - Dynamics equations (forces, torques, accelerations)

3. **integration/** - Numerical integration methods
   - Modular integrator trait for easy extension
   - Multiple implementations (Euler, RK4, Exponential)
   - Configurable time step and accuracy

4. **validation/** - Testing and validation
   - Synthetic trajectory generation
   - Real flight data comparison
   - K-step prediction accuracy

5. **io/** - Data import/export
   - CSV trajectory export for visualization
   - Flight log parsing
   - Python-compatible formats

## Usage

### As a Library

```rust
use multirotor_simulator::prelude::*;

let params = MultirotorParams::crazyflie();
let mut sim = MultirotorSimulator::new(params, IntegrationMethod::RK4);

let action = MotorAction::hover();
sim.step(&action);

println!("Position: {:?}", sim.state().position);
```

### Run Assignment 1

```bash
cargo run --bin assignment1
```

### Run Demo

```bash
cargo run --bin demo
```

## Features

- ✅ Multiple integration methods (Euler, RK4, Exponential map)
- ✅ Quaternion-based orientation (no gimbal lock)
- ✅ Crazyflie 2.1 parameters
- ✅ CSV export for visualization
- ✅ Real flight data validation
- ✅ Comprehensive documentation
- ✅ Clean, modular architecture

## Design Principles

1. **Modularity**: Each component has a single, well-defined responsibility
2. **Testability**: Pure functions and clear interfaces
3. **Extensibility**: Easy to add new integration methods or aircraft models
4. **Performance**: Efficient implementations without sacrificing clarity
5. **Documentation**: Every module and function is documented

## Testing

```bash
cargo test
cargo test --lib        # Library tests only
cargo test --bin        # Binary tests only
```

## Documentation

```bash
cargo doc --open
```

## License

Educational/Research use

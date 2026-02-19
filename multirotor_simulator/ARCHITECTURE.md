# Project Architecture

## Overview

This project implements a clean, modular multirotor dynamics simulator following software engineering best practices.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
│  (Binaries: assignment1, demo, experiments)                  │
└───────────────────────┬─────────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────────┐
│                  Multirotor Simulator Library                │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │    Math      │  │  Dynamics    │  │ Integration  │     │
│  │              │  │              │  │              │     │
│  │  - Vec3      │  │  - State     │  │  - Euler     │     │
│  │  - Quat      │  │  - Params    │  │  - RK4       │     │
│  │              │  │  - Simulator │  │  - Exp Map   │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

## Module Responsibilities

### `math/` - Mathematical Primitives
**Purpose**: Pure mathematical operations with zero dependencies

- `vec3.rs`: 3D vectors for position, velocity, force
- `quaternion.rs`: 3D rotations without gimbal lock

**Key Traits**: Copy, Clone, Add, Mul

### `dynamics/` - Physics Core
**Purpose**: Physical system representation and dynamics equations

- `state.rs`: System state (position, velocity, orientation, angular velocity)
- `params.rs`: Physical parameters (mass, inertia, motor constants)
- `simulator.rs`: Main simulator with pluggable integrators

**Design Pattern**: Strategy pattern for integrators

### `integration/` - Numerical Methods
**Purpose**: Pluggable time-stepping algorithms

- `euler.rs`: First-order forward Euler
- `rk4.rs`: Fourth-order Runge-Kutta
- `exponential.rs`: Exponential map for quaternions

**Design Pattern**: Trait-based polymorphism via `Integrator` trait

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
│
├── src/
│   ├── lib.rs                    # Library entry point
│   │
│   ├── math/                     # Mathematical primitives
│   │   ├── mod.rs               # Module exports
│   │   ├── vec3.rs              # 3D vector implementation
│   │   └── quaternion.rs        # Quaternion implementation
│   │
│   ├── dynamics/                 # Physics simulation
│   │   ├── mod.rs               # Module exports
│   │   ├── state.rs             # State representation
│   │   ├── params.rs            # Physical parameters
│   │   └── simulator.rs         # Simulator core
│   │
│   ├── integration/              # Numerical integration
│   │   ├── mod.rs               # Module exports
│   │   ├── euler.rs             # Euler method
│   │   ├── rk4.rs               # RK4 method
│   │   └── exponential.rs       # Exponential map methods
│   │
│   └── bin/                      # Binary executables
│       ├── assignment1.rs        # Assignment 1 experiment
│       └── demo.rs               # Quick demonstration
│
└── data/                         # Data files (to be added)
    ├── trajectories/            # CSV trajectory exports
    └── flight_logs/             # Real flight data
```

## Component Interactions

### Creating a Simulator

```rust
// 1. Choose physical parameters
let params = MultirotorParams::crazyflie();

// 2. Select integration method
let integrator = Box::new(RK4Integrator);

// 3. Create simulator
let mut sim = MultirotorSimulator::new(params, integrator);

// 4. Define control input
let action = MotorAction::hover();

// 5. Simulate
sim.step(&action);
```

### Integration Method Flow

```
MotorAction → Simulator.step()
                   ↓
           Integrator.step()
                   ↓
         compute_derivatives()
                   ↓
      params.motor_speeds_to_forces_torques()
      params.angular_acceleration()
                   ↓
         Update State
```

## Extensibility Points

### Adding a New Integration Method

1. Create file: `src/integration/mymethod.rs`
2. Implement `Integrator` trait
3. Export in `src/integration/mod.rs`
4. Use: `Box::new(MyMethodIntegrator)`

### Adding a New Aircraft

1. Create method in `MultirotorParams`
2. Define physical constants
3. Use existing dynamics equations

### Adding Validation Tools

1. Create module: `src/validation/`
2. Import necessary types
3. Implement validation functions
4. Export in `src/lib.rs`

## Testing Strategy

### Unit Tests
- Math operations (Vec3, Quat)
- Parameter calculations
- Individual integrator steps

### Integration Tests
- Full simulation runs
- Method comparison
- Accuracy validation

### Documentation Tests
- Examples in doc comments
- Ensure API examples stay valid

## Future Extensions

Planned additions to the architecture:

1. **validation/** module
   - Synthetic trajectory generation
   - Real flight data comparison
   - K-step prediction

2. **io/** module
   - CSV export/import
   - Flight log parsing
   - Visualization data generation

3. **control/** module
   - PID controllers
   - Model predictive control
   - Trajectory tracking

4. **sensors/** module
   - IMU simulation
   - Sensor noise models
   - State estimation

## Build & Run

```bash
# Build library and binaries
cargo build

# Run demo
cargo run --bin demo

# Run assignment
cargo run --bin assignment1

# Run tests
cargo test

# Generate documentation
cargo doc --open
```

## Benefits of This Architecture

1. **Clarity**: Each file has ~100-200 lines, easy to understand
2. **Maintainability**: Changes localized to specific modules
3. **Reusability**: Library can be used in multiple projects
4. **Testability**: Each component independently testable
5. **Scalability**: Easy to add features without breaking existing code
6. **Documentation**: Clear module boundaries and responsibilities

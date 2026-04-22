# dynamics/ — Multirotor Physics Simulator

## What this module does

SE(3) rigid-body dynamics for a quadrotor. Provides:
- `MultirotorState` — 13D state (position, velocity, orientation, angular velocity)
- `MultirotorParams` — physical constants (mass, inertia, motor coefficients)
- `MotorAction` — control input (4× motor ω²)
- `MultirotorSimulator` — wraps state + params + integrator

## Key types

```rust
pub struct MultirotorState {
    pub position: Vec3,          // [m] world frame
    pub velocity: Vec3,          // [m/s] world frame
    pub orientation: Quat,       // world→body rotation, [w,x,y,z]
    pub angular_velocity: Vec3,  // [rad/s] body frame
}

pub struct MultirotorParams {
    pub mass: f32,               // [kg]
    pub arm_length: f32,         // [m] center to motor
    pub inertia: [[f32; 3]; 3], // [kg·m²] — diagonal, off-diagonal ≈ 0
    pub gravity: f32,            // [m/s²] ≈ 9.81
    pub kf: f32,                 // [N/(rad/s)²] thrust coefficient per motor
    pub kt: f32,                 // [Nm/(rad/s)²] reaction torque coefficient
    pub dt: f32,                 // [s] default simulation timestep
    pub motor_time_constant: f32, // [s] first-order motor dynamics τ
}

pub struct MotorAction {
    pub omega1_sq: f32,   // front-left  ω² [rad²/s²]
    pub omega2_sq: f32,   // front-right
    pub omega3_sq: f32,   // back-right
    pub omega4_sq: f32,   // back-left
}
```

## Crazyflie 2.1 defaults

```rust
MultirotorParams::crazyflie()
// mass = 0.027 kg
// arm_length = 0.046 m
// inertia = diag([16.571710e-6, 16.655602e-6, 29.261652e-6])
// gravity = 9.81
// kf = 2.5e-6 N/(rad/s)²
// kt = 1.0e-7 Nm/(rad/s)²
// dt = 0.001 s (1 ms)
// motor_time_constant = 0.03 s
```

## MotorAction helpers

```rust
MotorAction::hover()    // all motors at hover equilibrium
MotorAction::uniform(omega_sq)   // all motors equal
MotorAction::from_thrust_torque(thrust_n: f32, torque: Vec3, params: &MultirotorParams) -> Self
// → inverse motor mixing matrix: solves for ω²_i from desired thrust + torques
```

## MultirotorSimulator

```rust
pub struct MultirotorSimulator { /* params, state, integrator: Box<dyn Integrator> */ }

impl MultirotorSimulator {
    pub fn new(params: MultirotorParams, integrator: Box<dyn Integrator>) -> Self
    pub fn state(&self) -> &MultirotorState
    pub fn state_mut(&mut self) -> &mut MultirotorState
    pub fn set_state(&mut self, state: MultirotorState)
    pub fn params(&self) -> &MultirotorParams
    pub fn step(&mut self, action: &MotorAction)
    pub fn simulate(&mut self, action: &MotorAction, num_steps: usize) -> Vec<MultirotorState>
    pub fn reset(&mut self)   // restore to initial state
}
```

## Typical usage pattern

```rust
let params = MultirotorParams::crazyflie();
let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));

// Set initial state
sim.state_mut().position = Vec3::new(0.0, 0.0, 0.5);

// Step
let action = MotorAction::from_thrust_torque(thrust, torque, &params);
sim.step(&action);
let state = sim.state();
```

## Dynamics equations

```
p̈ = (1/m) R f_thrust e3 - g e3
q̇ = 0.5 q ⊗ ω_quat
ω̇ = J⁻¹ (τ - ω × Jω)   (Euler's rotation equation)
```

Where `f_thrust = kf * Σ ωi²` and `τ = motor_mixing(kf, kt, arm) * [ωi²]`.

## Motor mixing (X-configuration)

```
Motor positions (top view):
  1 (FL +CW)    2 (FR -CCW)
  4 (BL -CCW)   3 (BR +CW)

Thrust:  T = kf * (ω1² + ω2² + ω3² + ω4²)
Roll:    τx = kf * arm * (-ω1² + ω2² + ω3² - ω4²)  [+right]
Pitch:   τy = kf * arm * ( ω1² - ω2² + ω3² - ω4²)  [+nose up]
Yaw:     τz = kt * (-ω1² + ω2² - ω3² + ω4²)         [+CCW from above]
```

## Gotchas

- `MultirotorState` is NOT `Copy` (contains `Vec3` and `Quat` which are Copy, but the struct itself is a value type). Use `.clone()` to duplicate.
- `MotorAction` fields are `ω²` not `ω` — motor speed squared. Already squared for efficiency.
- `inertia` is a full 3×3 matrix but is diagonal in practice; off-diagonal elements are negligible and kept zero.
- `dt` in `MultirotorParams` is the default step size for `simulate()`. The `step()` method uses it too — check params.dt if timing seems wrong.

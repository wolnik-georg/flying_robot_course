# Dynamics — Multirotor Rigid-Body Physics

## 1. Overview

The dynamics module models a quadrotor as a rigid body in 3D space. It defines the physical state, the equations of motion, and the mapping from motor commands to forces and torques. Everything else in the stack — the controller, the estimator, the planner — either consumes or produces states and actions defined here.

Why model dynamics at all? In simulation (assignments 1–5) the dynamics module *is* the drone: it advances the state forward in time given motor commands. On real hardware it provides the physical understanding needed to design the controller and understand what the estimator is tracking.

---

## 2. Physical / Mathematical Foundation

### 2.1 State Vector

The complete rigid-body state is 13-dimensional:

```
x = (p, v, q, ω)
    p ∈ ℝ³   position in world frame [m]
    v ∈ ℝ³   velocity in world frame [m/s]
    q ∈ ℍ₁   unit quaternion (body→world rotation)
    ω ∈ ℝ³   angular velocity in body frame [rad/s]
```

The quaternion has 4 components `(w, x, y, z)` but is constrained to unit norm, so the system has 12 true degrees of freedom.

### 2.2 Body Frame Convention

```
        x (forward)
        ↑
        |
  y ←───┼    z (up, out of page)
   (left)|
```

Motors are in X-configuration at 45° angles:

```
  M1(FL,CW)  M2(FR,CCW)
       \   /
        \ /
        / \
       /   \
  M4(BL,CCW) M3(BR,CW)
```

### 2.3 Translational Dynamics (Newton)

In the world frame:

```
m·v̇ = R·[0, 0, F_total]ᵀ − m·g·ẑ
```

- `R` is the rotation matrix (body→world) derived from `q`
- `F_total = kf·(ω₁² + ω₂² + ω₃² + ω₄²)` is the total thrust along the body z-axis
- Gravity `g = 9.81 m/s²` acts downward in world frame

The thrust is always along the body z-axis; the drone steers by tilting the body.

### 2.4 Rotational Dynamics (Euler)

In the body frame:

```
I·ω̇ = τ − ω × (I·ω)
```

- `I` is the diagonal inertia tensor `diag(Jxx, Jyy, Jzz)` [kg·m²]
- `τ` is the net torque from motor forces [N·m]
- `ω × (I·ω)` is the gyroscopic term — it couples angular velocity components and is why a spinning drone resists changes to its axis

### 2.5 Motor Model

Each motor produces thrust and reactive torque proportional to the square of its angular speed:

```
Thrust:          f_i = kf · ωᵢ²     [N]
Reactive torque: τ_i = kt · ωᵢ²     [N·m]
```

The code stores `ωᵢ²` directly (not `ωᵢ`) because `f_i` is linear in `ωᵢ²`, which simplifies the mixer arithmetic.

### 2.6 Mixer (X-Configuration)

For the X-configuration with arm length `L` (motor-to-center), the effective lever arm is `L/√2`:

```
F     = kf · (ω₁² + ω₂² + ω₃² + ω₄²)
τ_roll  = kf·(L/√2) · (ω₂² + ω₃² − ω₁² − ω₄²)
τ_pitch = kf·(L/√2) · (ω₃² + ω₄² − ω₁² − ω₂²)
τ_yaw   = kt         · (ω₁² − ω₂² + ω₃² − ω₄²)
```

CW motors (1, 3) produce a positive yaw reaction; CCW motors (2, 4) negative. To yaw clockwise, spin CW motors faster.

### 2.7 Quaternion Kinematics

The orientation quaternion evolves as:

```
q̇ = 0.5 · q ⊗ [0, ω]
```

where `⊗` is the Hamilton product and `[0, ω]` is the pure quaternion formed from angular velocity.

---

## 3. Architecture Diagram

```
┌─────────────────────────────────────────────────────┐
│                    Inputs                           │
│  MotorAction: [ω₁², ω₂², ω₃², ω₄²]                │
└─────────────────────┬───────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────┐
│           MultirotorParams::motor_speeds_to_forces  │
│                                                     │
│   ω² → F_total (thrust)                             │
│   ω² → τ_roll, τ_pitch, τ_yaw (torques)            │
└─────────────────────┬───────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────┐
│           compute_derivatives (simulator.rs)        │
│                                                     │
│   ṗ = v                                             │
│   v̇ = (R·[0,0,F] − m·g·ẑ) / m                     │
│   q̇ = 0.5 · q ⊗ [0, ω]                            │
│   ω̇ = J⁻¹·(τ − ω × J·ω)                           │
└─────────────────────┬───────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────┐
│           Integrator (Euler / RK4 / ExpRK4)         │
│                                                     │
│   state_{k+1} = state_k + dt · derivatives         │
└─────────────────────┬───────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────┐
│           MultirotorState (updated)                 │
│   position, velocity, orientation, angular_velocity │
└─────────────────────────────────────────────────────┘
```

First-order motor dynamics (low-pass filter, `α = dt/(τ_motor + dt)`) is applied inside `MultirotorSimulator::step` before the integrator runs, so commanded motor speeds converge to actual with time constant `τ_motor = 30 ms`.

---

## 4. Key Data Types

### `MultirotorState` — `src/dynamics/state.rs:13`

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `position` | `Vec3` | m | World-frame position |
| `velocity` | `Vec3` | m/s | World-frame velocity |
| `orientation` | `Quat` | — | Unit quaternion (body→world) |
| `angular_velocity` | `Vec3` | rad/s | Body-frame angular velocity |

### `MotorAction` — `src/dynamics/state.rs:67`

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `omega1_sq` | `f32` | rad²/s² | Front-left motor ω² (CW) |
| `omega2_sq` | `f32` | rad²/s² | Front-right motor ω² (CCW) |
| `omega3_sq` | `f32` | rad²/s² | Back-right motor ω² (CW) |
| `omega4_sq` | `f32` | rad²/s² | Back-left motor ω² (CCW) |

`MotorAction::from_thrust_torque` inverts the mixer equations to find the required motor speeds given a desired thrust and torque vector (`src/dynamics/state.rs:102`).

### `MultirotorParams` — `src/dynamics/params.rs:12`

Crazyflie 2.1 values:

| Parameter | Value | Units | Description |
|-----------|-------|-------|-------------|
| `mass` | 0.027 | kg | Total mass |
| `arm_length` | 0.046 | m | Center-to-motor distance |
| `inertia[0][0]` (Jxx) | 16.57×10⁻⁶ | kg·m² | Roll inertia |
| `inertia[1][1]` (Jyy) | 16.66×10⁻⁶ | kg·m² | Pitch inertia |
| `inertia[2][2]` (Jzz) | 29.26×10⁻⁶ | kg·m² | Yaw inertia |
| `kf` | 2.5×10⁻⁶ | N/(rad/s)² | Thrust coefficient |
| `kt` | 1.0×10⁻⁷ | N·m/(rad/s)² | Torque coefficient |
| `dt` | 0.01 | s | Simulation timestep (100 Hz) |
| `motor_time_constant` | 0.03 | s | Motor low-pass time constant |

Source: Julian Foerster's ETHZ BA thesis + Crazyflie firmware `controller_lee.c`.

### `MultirotorSimulator` — `src/dynamics/simulator.rs:22`

Owns a `MultirotorParams`, a `MultirotorState`, and a `Box<dyn Integrator>`. Calling `sim.step(action)` applies motor dynamics then advances the state by one `dt`.

---

## 5. Algorithm Walkthrough

**One simulation step** (`src/dynamics/simulator.rs:70`):

1. **Motor dynamics** (line 73–81): Low-pass filter each `ωᵢ²` toward the commanded value.
   ```rust
   alpha = dt / (tau_motor + dt)  // ≈ 0.25 at 100 Hz
   omega_i_sq += alpha * (commanded - omega_i_sq)
   ```

2. **Compute derivatives** (`src/dynamics/simulator.rs:121`):
   - `motor_speeds_to_forces_torques` → `(F_total, τ)` (`src/dynamics/params.rs:69`)
   - Rotate thrust to world frame: `thrust_world = R · [0, 0, F_total]`
   - Linear acceleration: `a = (thrust_world + gravity) / mass`
   - Angular acceleration via Euler: `α = J⁻¹·(τ − ω × Jω)` (`src/dynamics/params.rs:98`)
   - Quaternion derivative: `q̇ = 0.5 · q ⊗ [0, ω]`

3. **Integrate** (delegated to `Integrator` trait): advance state by `dt`. RK4 uses four evaluations of `compute_derivatives` with intermediate states.

---

## 6. Parameters & Tuning

**`kf` and `kt`**: The thrust and torque coefficients map motor speed to physical forces. They are identified empirically (propeller tests on a thrust stand). Changing propellers requires re-identification.

**`arm_length`**: Determines how much torque a given motor speed differential produces. Longer arms = more torque authority = faster roll/pitch response.

**`inertia`**: The Jxx ≈ Jyy values reflect the Crazyflie's near-symmetric body. Jzz is roughly 1.8× larger because the motors are far from the yaw axis (contributing `kf·L²` each), and yaw inertia accumulates all four contributions.

**`motor_time_constant`**: At 30 ms the motor low-pass has bandwidth ~5 Hz. This limits how fast the actual thrust can track the commanded thrust, which is important for high-frequency manoeuvres. Reduce only if you have measured data suggesting the real motors are faster.

---

## 7. Connections to Other Modules

| Direction | Module | What is exchanged |
|-----------|--------|------------------|
| Consumes | — | Nothing external (physics is self-contained) |
| Produces | `controller/` | `MultirotorState` (current position, velocity, orientation, angular velocity) |
| Produces | `estimation/` | The same state structure is what the MEKF estimates |
| Consumed by | `integration/` | `compute_derivatives` is called by every integrator |
| Consumed by | `flight/` | `MultirotorParams` provides mass/inertia to the hardware bridge |

---

## 8. Common Pitfalls

**Quaternions vs Euler angles**: Using Euler angles (roll/pitch/yaw) for orientation causes *gimbal lock* — at 90° pitch the roll and yaw axes align, making the parameterisation singular. Quaternions avoid this at the cost of one extra state component and a unit-norm constraint. Never integrate Euler angles directly as if they were a vector.

**Frame confusion for velocity**: `velocity` in `MultirotorState` is in the *world frame*. The MEKF's body-frame velocity `b` is different. The relationship is `v_world = R · b`.

**Integrating in the right frame**: The angular velocity `ω` is in the body frame, but the rotation update `q̇ = 0.5·q⊗[0,ω]` correctly maps it into the quaternion's reference frame. Don't pre-rotate `ω` to world frame before applying this formula.

**Motor speed saturation**: `MotorAction::from_thrust_torque` clamps `ωᵢ²` to zero, but has no upper limit. In simulation this can lead to unrealistically high thrusts. Real hardware clamps at the firmware level.

---

## 9. Related Tests

| Test file | What it covers |
|-----------|---------------|
| `src/dynamics/params.rs` (inline) | Crazyflie params values, hover thrust = weight, roll/pitch/yaw torque signs |
| `src/dynamics/state.rs` (inline) | `MotorAction` construction, `from_thrust_torque` round-trip |
| `src/dynamics/simulator.rs` (inline) | `compute_derivatives` in hover (near-zero acc), free-fall (gravity only), tilt |
| `tests/test_hover_control_loop.rs` | Full simulation convergence: controller + dynamics hold hover at target height |

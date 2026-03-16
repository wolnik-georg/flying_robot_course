# Controller — SE(3) Geometric Controller

## 1. Overview

The controller computes the thrust and torque commands needed to make the drone follow a reference trajectory. It is a *geometric controller on SE(3)* based on Lee, Leok, and McClamroch (2010). "Geometric" here means the errors are computed directly on the rotation manifold — no Euler angles, no linearisation — so it remains valid at large attitude angles.

The controller is a two-loop structure: an outer *position loop* generates a desired force vector; an inner *attitude loop* tracks the required orientation. Both loops run at the same rate (100 Hz in simulation, 100 Hz via the CRTP shadow path on hardware).

**Reference**: Lee, T., Leok, M., McClamroch, N.H. (2010). Geometric tracking control of a quadrotor UAV on SE(3). *49th IEEE CDC*, pp. 5420–5425.

---

## 2. Physical / Mathematical Foundation

### 2.1 Two-Loop Structure

```
position error → desired force vector F_des
                      ↓
                rotation extractor → desired rotation R_d
                      ↓
               attitude error (SO(3)) → torque τ
```

The position and attitude loops are coupled: the position controller outputs a desired force *direction*, and the attitude controller tracks that direction.

### 2.2 Position Control Law

The desired force vector in the world frame:

```
F_des = m · (a_ref + Kp·ep + Kv·ev + Ki_pos·∫ep + g·ẑ)
```

where:
- `ep = p_ref − p` is the position error
- `ev = v_ref − v` is the velocity error
- `a_ref` is the reference acceleration (feedforward)
- `g·ẑ` compensates gravity
- `Kp, Kv, Ki_pos` are gain matrices (diagonal for the Crazyflie)

The scalar thrust is **not** `‖F_des‖`; it is the projection onto the current body z-axis:

```
f = F_des · (R · ẑ_body)
```

This correctly accounts for the fact that when the drone is tilted, only the component of the desired force along the *actual* thrust axis is achievable from the motors.

### 2.3 Desired Rotation

Given `F_des` and the desired yaw `ψ_ref`, the desired rotation `R_d` is constructed geometrically:

```
ẑ_d = F_des / ‖F_des‖          (desired body z = thrust direction)
ŷ_d = normalize(ẑ_d × x̂_c)    where x̂_c = [cos ψ, sin ψ, 0]ᵀ
x̂_d = ŷ_d × ẑ_d
R_d = [x̂_d | ŷ_d | ẑ_d]       (column matrix)
```

This construction always gives an orthonormal frame with the correct yaw and thrust direction.

### 2.4 Attitude Error on SO(3)

Unlike Euler angle differences, the attitude error is computed directly on SO(3):

```
e_R = 0.5 · (R_d^T·R − R^T·R_d)∨
```

where `(·)∨` is the *vee map* — it extracts the 3-vector from a skew-symmetric matrix:
```
[[0, −c, b], [c, 0, −a], [−b, a, 0]]  →  [a, b, c]
```

`(R_d^T·R − R^T·R_d)` is skew-symmetric by construction, so this always gives a well-defined 3-vector. When `R = R_d`, both terms cancel and `e_R = 0`.

### 2.5 Angular Velocity Error

The reference angular velocity is computed from jerk feedforward:

```
h_ω = (m / f) · (p⃛_ref − (ẑ_d · p⃛_ref) · ẑ_d)   [jerk perpendicular to thrust]
ω_des = [−h_ω · ŷ_d,  h_ω · x̂_d,  ψ̇_ref · (ẑ_d · ẑ_world)]
```

The angular velocity error:
```
e_ω = ω − R^T · R_d · ω_des
```

### 2.6 Attitude Control Law

```
τ = −K_R · e_R − K_ω · e_ω + ω × (I · ω)
```

The last term `ω × Iω` is *gyroscopic compensation* — it cancels the gyroscopic coupling in Euler's equation, making the torque loop act like three independent PD controllers on the rotation axes.

---

## 3. Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────┐
│  Inputs                                                              │
│  TrajectoryReference: pos, vel, acc, jerk, yaw, yaw_rate            │
│  MultirotorState:     position, velocity, orientation, ang_vel       │
└────────────────────────────┬─────────────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────────────┐
│  POSITION LOOP                                                     │
│                                                                    │
│  ep = p_ref − p,   ev = v_ref − v                                 │
│  F_d = m·(a_ref + Kp·ep + Kv·ev + Ki·∫ep + g·ẑ)                  │
│  f   = F_d · (R·ẑ_body)    ← project onto current body z          │
└────────────────────────────┬───────────────────────────────────────┘
                             │  F_d, f
                             ▼
┌────────────────────────────────────────────────────────────────────┐
│  ROTATION EXTRACTOR                                                │
│                                                                    │
│  ẑ_d = F_d / ‖F_d‖                                               │
│  ŷ_d = normalize(ẑ_d × x̂_c(ψ_ref))                              │
│  x̂_d = ŷ_d × ẑ_d                                                │
│  R_d = [x̂_d | ŷ_d | ẑ_d]                                        │
└────────────────────────────┬───────────────────────────────────────┘
                             │  R_d
                             ▼
┌────────────────────────────────────────────────────────────────────┐
│  ATTITUDE LOOP                                                     │
│                                                                    │
│  e_R   = 0.5·(R_d^T·R − R^T·R_d)∨                                │
│  ω_des = jerk feedforward + ψ̇_ref component                      │
│  e_ω   = ω − R^T·R_d·ω_des                                       │
│  τ     = −K_R·e_R − K_ω·e_ω + ω×(I·ω)                           │
└────────────────────────────┬───────────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────────────┐
│  ControlOutput: thrust f [N], torque τ [N·m]                      │
│                                                                    │
│  → MotorAction::from_thrust_torque → [ω₁², ω₂², ω₃², ω₄²]       │
└────────────────────────────────────────────────────────────────────┘
```

---

## 4. Key Data Types

### `TrajectoryReference` — `src/controller/mod.rs:13`

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `position` | `Vec3` | m | Reference position |
| `velocity` | `Vec3` | m/s | Reference velocity (feedforward) |
| `acceleration` | `Vec3` | m/s² | Reference acceleration (feedforward) |
| `jerk` | `Vec3` | m/s³ | Reference jerk (for angular velocity feedforward) |
| `yaw` | `f32` | rad | Reference yaw angle |
| `yaw_rate` | `f32` | rad/s | Reference yaw rate |
| `yaw_acceleration` | `f32` | rad/s² | Reference yaw acceleration |

All trajectory generators (`Figure8Trajectory`, `CircleTrajectory`, `SplineTrajectory`, etc.) produce this type via the `Trajectory` trait.

### `ControlOutput` — `src/controller/mod.rs:32`

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `thrust` | `f32` | N | Scalar thrust to produce |
| `torque` | `Vec3` | N·m | Body-frame torque vector |

### `GeometricController` — `src/controller/mod.rs:54`

| Field | Type | Description |
|-------|------|-------------|
| `kp` | `Vec3` | Position proportional gains |
| `kv` | `Vec3` | Position derivative gains |
| `ki_pos` | `Vec3` | Position integral gains |
| `kr` | `Vec3` | Attitude proportional gains (K_R) |
| `kw` | `Vec3` | Attitude derivative gains (K_ω) |
| `ki` | `Vec3` | Attitude integral gains (set to zero on hardware) |

---

## 5. Algorithm Walkthrough

The main computation is `compute_control` at `src/controller/mod.rs:205`:

**Step 1 — Position errors and integral** (lines 214–225):
```rust
ep = reference.position - state.position
ev = reference.velocity - state.velocity
i_error_pos += ep * dt          // accumulate integral
i_error_pos.clamp(-0.5, 0.5)   // anti-windup
```

**Step 2 — Desired force** (lines 229–247):
```rust
F_d = acc_ref + Kp*ep + Kv*ev + Ki_pos*i_err_pos + [0,0,g]
f   = (F_d * mass).dot(body_z)   // project onto actual body z
```

**Step 3 — Integral reset** (lines 259–262): When `f < 0.01 N` (drone on ground), reset integrals to prevent windup during idle.

**Step 4 — Desired rotation** (`compute_desired_rotation`, lines 150–176):
Build `R_d` from `F_d` direction and `ψ_ref`.

**Step 5 — Rotation error** (`compute_rotation_error`, lines 183–201):
```rust
e_R = 0.5 * vee(R_d^T * R - R^T * R_d)
```

**Step 6 — Jerk feedforward and desired angular velocity** (lines 280–304):
Project jerk perpendicular to `ẑ_d`, compute `ω_des`, transform to body frame.

**Step 7 — Attitude control torque** (lines 309–335):
```rust
τ = -K_R * e_R - K_ω * e_ω + ω × (J*ω)
```

---

## 6. Parameters & Tuning

Default gains (`GeometricController::default`, `src/controller/mod.rs:91`):

| Gain | Value (x,y,z) | Physical Meaning | Tuning Effect |
|------|--------------|------------------|---------------|
| `kp` | (12, 12, 7) | Position stiffness [m/s²/m] | Higher = faster but oscillatory response |
| `kv` | (8, 8, 4) | Position damping [m/s²/(m/s)] | Higher = more damping, less overshoot |
| `ki_pos` | (0.05, 0.05, 0.05) | Steady-state drift elimination | Higher = faster wind-up correction, risk of slow oscillation |
| `kr` | (0.007, 0.007, 0.008) | Attitude stiffness [N·m/rad] | Firmware defaults from `controller_lee.c` |
| `kw` | (0.00115, 0.00115, 0.002) | Attitude damping [N·m/(rad/s)] | Higher kw/kr ratio = more damping |
| `ki` | (0, 0, 0) | Attitude integral | Zero on hardware — see pitfalls |

**Separation of XY vs Z gains**: XY position gains are higher (12/8) than Z (7/4). The optical flow / Kalman Z estimate is more accurate than XY, so less gain is needed on Z. The firmware default is 7/4 for all axes.

**Crazyflie context**: At 27g, the inertia values (Jxx ≈ 16.6 µN·m²) are tiny. The attitude gains `kr` and `kw` are correspondingly small. Scaling to a heavier vehicle requires proportionally larger attitude gains.

---

## 7. Connections to Other Modules

| Direction | Module | What is exchanged |
|-----------|--------|------------------|
| Consumes | `dynamics/` | `MultirotorState` (current state) and `MultirotorParams` (mass, inertia) |
| Consumes | `trajectory/` or `planning/` | `TrajectoryReference` (desired state at current time) |
| Produces | `dynamics/` | `ControlOutput` → `MotorAction` via `from_thrust_torque` |
| Produces | `flight/` | `ControlOutput` is converted to RPYT commands for hardware |

---

## 8. Common Pitfalls

**Attitude integral windup on hardware**: In the RPYT flight mode only `f`, `roll_d`, and `pitch_d` are sent to the drone (not the full torque vector). An attitude integral `ki != 0` accumulates indefinitely without visible effect — and then causes a torque spike if the mode ever changes. The default sets `ki = [0, 0, 0]` and handles drift with `ki_pos` instead.

**Rotation error singularity at 180°**: The SO(3) error formula `e_R = 0.5·(R_d^T R − R^T R_d)∨` gives `‖e_R‖ = 0` when the rotation between `R` and `R_d` is exactly 180°. This is a genuine singularity of the error function, not of the physical problem. In practice the drone is never flipped 180° during normal flight, so this never triggers.

**Thrust projection matters**: Using `‖F_des‖` instead of `F_des · (R·ẑ)` over-estimates thrust when the drone is tilted. During aggressive manoeuvres this causes height oscillation. The code uses the projection (line 256).

**Feedforward quality**: The acceleration/jerk feedforward terms from `TrajectoryReference` must match the actual planned trajectory derivatives. If jerk is zero (e.g., a simple hovering reference) the angular velocity feedforward term `hw` is also zero — the attitude loop acts purely on error. This is fine for slow manoeuvres but causes lag on fast ones.

---

## 9. Related Tests

| Test file | What it covers |
|-----------|---------------|
| `src/controller/mod.rs` (inline) | Hover at origin → thrust = mg, torque = 0; identity rotation error = 0 |
| `tests/test_geometric_controller.rs` | Gain sanity, hover, position tracking convergence direction |
| `tests/test_controller_detailed.rs` | Detailed step-by-step: error directions, torque signs, jerk feedforward |
| `tests/test_hover_control_loop.rs` | Closed-loop: controller + dynamics converges to 1m hover in <5 s |

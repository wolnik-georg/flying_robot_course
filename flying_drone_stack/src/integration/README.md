# Integration — Numerical ODE Solvers

## 1. Overview

The integration module advances the drone's state forward in time by numerically solving the ordinary differential equation (ODE):

```
dx/dt = f(x, u)
```

where `x` is the 13-dimensional state `(p, v, q, ω)` and `u` is the motor command. Since `f` has no closed-form solution under time-varying control inputs, we approximate it step by step.

Four integrators are provided, with increasing accuracy and cost:

| Integrator | Method | Order | Cost | Use case |
|------------|--------|-------|------|----------|
| `EulerIntegrator` | Forward Euler | 1st | 1× deriv | Quick experiments, debugging |
| `RK4Integrator` | Runge-Kutta 4 | 4th | 4× derivs | Standard simulation accuracy |
| `ExpEulerIntegrator` | Euler + exp-map orientation | 1st+exp | 1× deriv | Better orientation at low cost |
| `ExpRK4Integrator` | RK4 + exp-map orientation | 4th+exp | 4× derivs | Best overall; used in assignments |

All four implement the `Integrator` trait and are injected into `MultirotorSimulator` via `Box<dyn Integrator>`, so they are swappable at runtime without changing any other code.

---

## 2. Physical / Mathematical Foundation

### 2.1 The Integration Problem

The equations of motion produce *derivatives*, not the next state. To advance time we must integrate:

```
x(t+dt) = x(t) + ∫ₜᵗ⁺ᵈᵗ f(x(τ), u) dτ
```

The different integrators approximate this integral with varying accuracy.

### 2.2 Forward Euler (1st Order)

The simplest approximation — use the derivative at the current point:

```
x(t+dt) ≈ x(t) + f(x(t), u) · dt
```

**Local error**: O(dt²) — the error per step grows quadratically with step size.
**Global error**: O(dt) — after `N = T/dt` steps the accumulated error grows linearly with `dt`.

For physical simulation at 100 Hz (`dt = 0.01 s`), Euler is often unstable for the angular dynamics — a small over-shoot in angular velocity causes the quaternion to drift off the unit sphere, which compounds each step.

### 2.3 Runge-Kutta 4 (4th Order)

RK4 evaluates `f` four times per step at strategically chosen intermediate points, then takes a weighted average:

```
k₁ = f(xₙ)
k₂ = f(xₙ + k₁·dt/2)     ← midpoint estimate 1
k₃ = f(xₙ + k₂·dt/2)     ← midpoint estimate 2 (better)
k₄ = f(xₙ + k₃·dt)       ← endpoint estimate

x(t+dt) = xₙ + (k₁ + 2k₂ + 2k₃ + k₄) · dt/6
```

The weighting `1:2:2:1` mirrors Simpson's rule for numerical integration. **Local error** is O(dt⁵), **global error** O(dt⁴) — four times as accurate per halving of `dt` compared to Euler.

### 2.4 The Quaternion Problem: Why Simple Addition Breaks

The quaternion `q = (w, x, y, z)` must satisfy `‖q‖ = 1`. Standard vector addition does not preserve this constraint:

```
q_new = q + q̇ · dt
‖q_new‖ ≠ 1 in general
```

Even with renormalisation after each step, the *direction* of `q_new` is wrong — it drifts off the unit sphere's geodesic. Over many steps this produces incorrect attitude estimates.

### 2.5 The Exponential Map Solution

For a pure quaternion `p = [0, ωdt/2]`, the quaternion exponential is:

```
exp(p) = [cos(‖ωdt/2‖),  sin(‖ωdt/2‖) · ω̂]
```

This is exactly a unit quaternion representing rotation by angle `‖ω‖·dt` around axis `ω̂`. Composing with the current orientation:

```
q(t+dt) = q(t) ⊗ exp([0, ω·dt/2])
```

This stays exactly on the unit sphere by construction — no renormalisation error, no drift from the manifold. The exponential map is the geometrically correct way to integrate rotations.

In code (`src/math/quaternion.rs:191`):
```rust
pub fn integrate_exponential(&self, omega: Vec3, dt: f32) -> Quat {
    let omega_quat = Quat::new(0.0, omega.x*dt*0.5, omega.y*dt*0.5, omega.z*dt*0.5);
    let delta_q = omega_quat.exp();
    (*self * delta_q).normalize()
}
```

The `.normalize()` is a safety guard against f32 accumulation — mathematically `exp(p)` is already unit norm.

### 2.6 Axis-Angle vs Exponential Map

The `integrate` method (used by `RK4Integrator`) uses the axis-angle approach:

```rust
let delta_q = Quat::from_axis_angle(omega.normalize(), omega_norm * dt);
(*self * delta_q).normalize()
```

For small `dt` (≤ 0.01 s, ‖ω‖ ≤ ~10 rad/s, angle ≤ 0.1 rad) the two approaches give nearly identical results. The exponential map (`integrate_exponential`) is slightly more principled for large rotations but the practical difference at 100 Hz is negligible. Both are dramatically better than additive integration.

---

## 3. Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│  MultirotorSimulator::step(action)                               │
│                                                                  │
│  1. Motor low-pass filter  (α = dt/(τ+dt))                      │
│  2. integrator.step(params, state, action)      ← trait call    │
└──────────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼──────────────────────┐
        │                     │                      │
        ▼                     ▼                      ▼
┌──────────────┐   ┌──────────────────┐   ┌─────────────────────┐
│ EulerIntegr. │   │  RK4Integrator   │   │  ExpRK4Integrator   │
│              │   │                  │   │                     │
│ 1× compute_  │   │ 4× compute_      │   │ 4× compute_         │
│ derivatives  │   │ derivatives      │   │ derivatives         │
│              │   │                  │   │                     │
│ q: .integrate│   │ q: .integrate    │   │ q: .integrate_      │
│ (axis-angle) │   │ (axis-angle)     │   │ exponential         │
└──────────────┘   └──────────────────┘   └─────────────────────┘
        │                     │                      │
        └─────────────────────┴──────────────────────┘
                              │
                    updated MultirotorState
```

---

## 4. Key Data Types

### `Integrator` trait — `src/dynamics/simulator.rs:11`

```rust
pub trait Integrator {
    fn step(&self, params: &MultirotorParams,
                   state: &mut MultirotorState,
                   action: &MotorAction);
}
```

One method. Any type implementing this can be used as an integrator. `MultirotorSimulator` holds a `Box<dyn Integrator>`, allowing runtime selection.

### The four concrete types

| Type | File | Key method used for `q` |
|------|------|------------------------|
| `EulerIntegrator` | `src/integration/euler.rs:14` | `Quat::integrate` (axis-angle) |
| `RK4Integrator` | `src/integration/rk4.rs:14` | `RK4Integrator::integrate_quaternion` (axis-angle + normalise) |
| `ExpEulerIntegrator` | `src/integration/exponential.rs:13` | `Quat::integrate_exponential` |
| `ExpRK4Integrator` | `src/integration/exponential.rs:47` | `Quat::integrate_exponential` at each of 4 stages |

---

## 5. Algorithm Walkthrough

### Euler (`src/integration/euler.rs:18`)

1. Compute `(F_total, τ)` from motor speeds
2. Rotate thrust to world frame: `thrust_world = R · [0, 0, F]`
3. `linear_acc = (thrust_world + gravity) / m`
4. `angular_acc = J⁻¹·(τ − ω×Jω)`
5. Update: `v += linear_acc·dt`, `ω += angular_acc·dt`
6. Update: `p += v·dt` (uses *updated* velocity — this is semi-implicit Euler)
7. Update: `q = q.integrate(ω, dt)`

Note: Euler uses the *new* velocity for position (line 45: `state.position + state.velocity * dt` after velocity was updated). This is "semi-implicit" Euler — slightly more stable than pure explicit.

### RK4 (`src/integration/rk4.rs:36`)

1. `k1 = compute_derivatives(state₀)` — at current state
2. Build `state_k2 = state₀ + k1·dt/2`, compute `k2 = compute_derivatives(state_k2)`
3. Build `state_k3 = state₀ + k2·dt/2`, compute `k3`
4. Build `state_k4 = state₀ + k3·dt`, compute `k4`
5. Weighted combine: `x_new = x₀ + (k1 + 2k2 + 2k3 + k4)·dt/6`
6. For orientation: compute `avg_ori_deriv = (k1_ori + 2k2_ori + 2k3_ori + k4_ori)·dt/6` then `q_new = integrate_quaternion(q₀, avg_ori_deriv, 1.0)` — note `dt=1.0` because the derivative is already pre-scaled by `dt/6`

### ExpRK4 (`src/integration/exponential.rs:50`)

Identical to RK4 for linear states (`p, v, ω`), but at each intermediate stage the orientation is updated with the exponential map:

```
state_k2.orientation = q₀.integrate_exponential(k1_ang, dt/2)
```

The final orientation update also uses the exponential map with the weighted-average angular velocity:

```
avg_omega = (k1_ang + 2k2_ang + 2k3_ang + k4_ang) * dt/6
q_new = q₀.integrate_exponential(avg_omega, dt)   // note: scale by dt, not 1.0
```

The ExpRK4 vs plain RK4 difference for orientation: RK4 adds quaternion *derivatives* (which are `Quat`-valued), while ExpRK4 always composes with a geometrically correct `exp(ω·dt/2)` rotation. For `dt = 0.01 s` and typical drone angular velocities (<10 rad/s, angle <0.1 rad per step), the difference is negligible in practice — both are verified to agree within `1e-2` in the tests.

---

## 6. Parameters & Tuning

**Which integrator to use:**

- **Assignment simulations**: `ExpRK4Integrator` (used by default). 4th-order accuracy + geometrically correct orientation — best choice when simulation fidelity matters.
- **Fast prototyping / debugging**: `EulerIntegrator`. One derivative evaluation per step, quick to reason about.
- **High-spin scenarios** (simulating aggressive flips, wind perturbations): `ExpRK4Integrator`. The exponential map prevents orientation drift at large angles.

**Step size `dt`**: Set in `MultirotorParams.dt` (default 0.01 s = 100 Hz). Halving `dt` reduces global error by ~16× for RK4 but doubles simulation time. For the Crazyflie at normal flight speeds and 100 Hz, `dt = 0.01 s` gives well-converged results with RK4.

**Motor time constant `τ_motor`**: Affects how quickly the motor state in the simulator tracks the commanded `MotorAction`. At `τ_motor = 30 ms` and `dt = 10 ms`, `α ≈ 0.25`, so the motor needs ~4 steps (~40 ms) to reach 63% of a step command. This limits effective bandwidth to ~5 Hz.

---

## 7. Connections to Other Modules

| Direction | Module | What is exchanged |
|-----------|--------|------------------|
| Consumes | `dynamics/simulator.rs` | Called as `Box<dyn Integrator>` inside `MultirotorSimulator::step` |
| Consumes | `dynamics/simulator.rs` | Calls `compute_derivatives` to evaluate `f(x, u)` |
| Consumes | `math/quaternion.rs` | Uses `Quat::integrate`, `Quat::integrate_exponential`, `Quat::exp` |
| Consumed by | `bin/assignment*` | Assignment binaries choose the integrator at startup |

---

## 8. Common Pitfalls

**Euler diverges with large `dt` or high angular velocity**: At `dt = 0.1 s` and ω = 10 rad/s, the Euler quaternion step angle is 1 radian — well into territory where linear approximation breaks down. Use RK4 or ExpRK4 if you increase `dt` beyond 0.02 s.

**RK4 quaternion subtlety (dt=1.0 in final step)**: In `RK4Integrator`, the weighted-average orientation derivative `avg_ori_deriv` is already scaled by `dt/6` (line 78). The final integration call uses `dt=1.0` to avoid double-scaling (line 79). This is a common point of confusion — the `1.0` is intentional.

**Semi-implicit Euler position update**: The Euler integrator updates `v` before using it to update `p` (lines 40, 45). This is intentional — it gives semi-implicit Euler which is more stable than fully explicit. Be aware if you copy the Euler code and change the order.

**`integrate` vs `integrate_exponential`**: Both `Quat::integrate` and `Quat::integrate_exponential` are geometrically correct for small angles (they produce the same `from_axis_angle` result). The exponential method becomes noticeably better only for step angles > ~0.3 rad. At 100 Hz with a drone, this threshold corresponds to ω > 30 rad/s — well beyond normal flight.

---

## 9. Related Tests

| Test file | What it covers |
|-----------|---------------|
| `src/integration/euler.rs` (inline) | Hover → near-zero motion; extra thrust → upward velocity |
| `src/integration/rk4.rs` (inline) | Hover → near-zero motion; rotation around Z produces nonzero `q.z` |
| `src/integration/exponential.rs` (inline) | ExpEuler hover → orientation near identity; ExpRK4 vs RK4 hover agree within `1e-2` |
| `src/math/quaternion.rs` (inline) | `Quat::integrate` at ω=1 rad/s → correct `sin(angle/2)` in quaternion component |
| `tests/test_math.rs` | Full quaternion algebra including `exp`, `integrate`, normalisation |

# Planning — Minimum-Snap Motion Planning & Differential Flatness

## 1. Overview

The planning module solves two related problems:

1. **Trajectory generation** (`spline.rs`): Given a set of waypoints and time allocations, compute a smooth, dynamically feasible polynomial trajectory that passes through all waypoints.

2. **Differential flatness** (`flatness.rs`): Given the trajectory's position derivatives (up to 4th order) and yaw derivatives (up to 2nd order), compute the exact rotation, thrust, angular velocity, and torque that the drone must produce — without integration.

Together they form a complete planning pipeline: waypoints in → motor commands out (given current state).

---

## 2. Physical / Mathematical Foundation

### 2.1 Polynomial Trajectory Representation

Each trajectory segment (between two consecutive waypoints) is an 8th-order polynomial in normalised time `t ∈ [0, 1]`:

```
p(t) = c₀ + c₁t + c₂t² + c₃t³ + c₄t⁴ + c₅t⁵ + c₆t⁶ + c₇t⁷ + c₈t⁸
```

There are 9 coefficients per axis (x, y, z, yaw) per segment. The polynomial is planned on `[0,1]` internally; to recover physical-time derivatives, divide by powers of `T` (the segment duration):

```
dp/dτ   = (1/T) · dp/dt    at t = τ/T
d²p/dτ² = (1/T²) · d²p/dt²
etc.
```

### 2.2 Minimum-Snap Objective

The objective minimises the **snap** (4th derivative of position) integrated over the trajectory:

```
min ∫₀ᵀ ‖p⁽⁴⁾(τ)‖² dτ
```

Why snap? Differential flatness (Section 2.4) shows that the drone's thrust and torque are directly driven by position acceleration and jerk. Minimising snap (the derivative of jerk) keeps these quantities smooth, which:
- Avoids sharp torque spikes that saturate the motors
- Produces smooth flight that is comfortable to track
- Minimises structural vibrations

Minimising jerk (3rd derivative) produces "minimum-jerk" trajectories used in robotics; snap is one order higher and produces even smoother commands.

### 2.3 QP Formulation

The snap cost for one segment is a quadratic function of the coefficients:

```
∫₀¹ (p''''(t))² dt = cᵀ·Q·c
```

where `Q` is a 9×9 Gram matrix computed from the 4th-derivative basis. This makes the whole problem a **Quadratic Program (QP)**:

```
minimize   Σ_segs  (1/T_seg⁷) · cᵀ·Q·c
subject to:
  - Position continuity:   p_seg(0) = waypoint,  p_seg(1) = next_waypoint
  - Derivative continuity: dp/dτ, d²p/dτ², d³p/dτ³, d⁴p/dτ⁴ match at each junction
  - Boundary conditions:   velocity = acc = jerk = snap = 0 at start and end
```

The `1/T_seg⁷` weight normalises the cost so that faster segments (shorter `T`) are not penalised more heavily just because they happen faster.

The QP is solved using the **Clarabel** interior-point solver via the `simple_qp` crate.

### 2.4 Differential Flatness

A quadrotor is *differentially flat* in the outputs `(x, y, z, ψ)` — meaning the complete state and input trajectory can be computed algebraically from the flat outputs and their derivatives, with no integration.

The flatness chain (Faessler, Franchi, Scaramuzza, IEEE RA-L 2018):

**Step 1 — Body z-axis from acceleration:**
```
acc_g = p̈ + g·ẑ                    (virtual acceleration including gravity)
ẑ_b = acc_g / ‖acc_g‖              (body z must point along this)
```

**Step 2 — Full rotation from yaw and thrust direction:**
```
x̂_c = [cos ψ, sin ψ, 0]ᵀ          (yaw reference direction)
ŷ_c = [−sin ψ, cos ψ, 0]ᵀ
x̂_b = normalize(ŷ_c × ẑ_b)
ŷ_b = normalize(ẑ_b × x̂_b)
R = [x̂_b | ŷ_b | ẑ_b]
```

**Step 3 — Thrust scalar:**
```
f = m · ẑ_b · (p̈ + g·ẑ) = m · ‖p̈ + g·ẑ‖
```

**Step 4 — Angular velocity from jerk:**
```
ω_x = −ŷ_b · p⃛ / f
ω_y =  x̂_b · p⃛ / f
ω_z = (f · ψ̇·(x̂_c·x̂_b) − b₃·(x̂_b·p⃛)) / (f · c₃)
       where c₃ = ‖ŷ_c × ẑ_b‖,  b₃ = −ŷ_c·ẑ_b
```

**Step 5 — Angular acceleration from snap:**
Similar but one derivative higher (involves `p⁽⁴⁾` and `ψ̈`).

**Step 6 — Torque:**
```
τ = J·ω̇ − Jω × ω     (Euler's equation, body frame)
```

No ODEs are solved — every quantity is computed algebraically at each time step.

---

## 3. Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│  Inputs                                                         │
│  waypoints: [(pos₀, yaw₀), ..., (posₙ, yawₙ)]                 │
│  durations: [T₀, T₁, ..., Tₙ₋₁]                               │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│  SplineTrajectory::plan  (spline.rs:156)                        │
│                                                                 │
│  For each axis (x, y, z, yaw):                                  │
│    solve_axis → QP → [c₀, c₁, ..., c₈] per segment            │
└────────────────────────┬────────────────────────────────────────┘
                         │  SplineTrajectory (segments + coefficients)
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│  SplineTrajectory::eval(t)  (spline.rs:183)                     │
│                                                                 │
│  Given time t:                                                  │
│    locate segment → evaluate polynomial and 4 derivatives       │
│    returns FlatOutput: pos, vel, acc, jerk, snap, yaw, ẏaw, ÿaw │
└────────────────────────┬────────────────────────────────────────┘
                         │  FlatOutput
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│  compute_flatness(flat, mass)  (flatness.rs:106)                │
│                                                                 │
│  acc_g → ẑ_b → R (rotation)                                    │
│  f = m·‖acc_g‖   (thrust)                                      │
│  jerk → ω       (angular velocity)                             │
│  snap → ω̇       (angular acceleration)                        │
│  τ = J·ω̇ − Jω×ω  (torque)                                     │
└────────────────────────┬────────────────────────────────────────┘
                         │  FlatnessResult
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│  flatness_to_reference → TrajectoryReference                    │
│  → GeometricController                                          │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. Key Data Types

### `Waypoint` — `src/planning/spline.rs:147`

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `pos` | `Vec3` | m | 3D position |
| `yaw` | `f32` | rad | Desired heading at this waypoint |

### `SplineTrajectory` — `src/planning/spline.rs:140`

| Field | Type | Description |
|-------|------|-------------|
| `segments` | `Vec<SplineSegment>` | One segment per waypoint interval |
| `total_time` | `f32` | Sum of all segment durations [s] |

Each `SplineSegment` holds four 9-coefficient arrays (`cx, cy, cz, cyaw`) and a `duration`.

### `FlatOutput` — `src/planning/flatness.rs:28`

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `pos` | `Vec3` | m | Position |
| `vel` | `Vec3` | m/s | Velocity |
| `acc` | `Vec3` | m/s² | Acceleration |
| `jerk` | `Vec3` | m/s³ | Jerk |
| `snap` | `Vec3` | m/s⁴ | Snap |
| `yaw` | `f32` | rad | Yaw angle |
| `yaw_dot` | `f32` | rad/s | Yaw rate |
| `yaw_ddot` | `f32` | rad/s² | Yaw acceleration |

### `FlatnessResult` — `src/planning/flatness.rs:41`

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `pos` | `Vec3` | m | Desired position (pass-through) |
| `vel` | `Vec3` | m/s | Desired velocity (pass-through) |
| `rot` | `[[f32;3];3]` | — | Desired rotation matrix `[x̂_b, ŷ_b, ẑ_b]` (column-major) |
| `thrust` | `f32` | N | Desired thrust |
| `omega` | `Vec3` | rad/s | Desired angular velocity |
| `omega_dot` | `Vec3` | rad/s² | Desired angular acceleration |
| `torque` | `Vec3` | N·m | Desired torque (`Jω̇ − Jω×ω`) |

---

## 5. Algorithm Walkthrough

### Planning Phase (`src/planning/spline.rs`)

**`SplineTrajectory::plan`** (line 156) calls `solve_axis` four times (x, y, z, yaw):

**`solve_axis`** (line 238):
1. Create `n_seg × 9` QP variables (coefficients)
2. Build snap cost: sum of `snap_cost(&vars[seg]) / T_seg⁷` (lines 251–259)
3. Add boundary constraints: `p(0) = wp_0`, `v(0)=a(0)=j(0)=s(0)=0`, and same at end (lines 264–280)
4. Add interior constraints: position continuity + derivative continuity up to 4th order (lines 283–315)
5. Solve with `ClarabelSolver::default()` (line 319)
6. Extract coefficients from solution (lines 324–332)

### Evaluation Phase (`src/planning/spline.rs:183`)

**`SplineTrajectory::eval(t)`**:
1. Find the correct segment by subtracting durations until `trem ≤ seg.duration`
2. Normalise: `t_norm = trem / seg.duration ∈ [0,1]`
3. Evaluate polynomial and 4 derivatives using precomputed factorial coefficients, dividing by appropriate powers of `duration` to recover physical-time derivatives

### Flatness Phase (`src/planning/flatness.rs:106`)

**`compute_flatness(flat, mass)`** follows Faessler et al. 2018 directly:
- Lines 118–136: Build rotation matrix from `acc_g` and yaw
- Lines 140–142: Compute thrust `f = m · ẑ_b · acc_g`
- Lines 148–165: Compute angular velocity from jerk and yaw rate
- Lines 169–192: Compute angular acceleration from snap and yaw acceleration
- Lines 197–205: Compute torque `τ = J·ω̇ − (Jω)×ω`

---

## 6. Parameters & Tuning

**Segment durations**: The most important design choice. Too short = QP is infeasible (the drone cannot physically cover the distance). Too long = slower manoeuvres. A good heuristic is `T_seg = distance / max_speed`.

**Polynomial order (8th = 9 coefficients)**: 8th order with zero-snap boundary conditions provides 4 equality constraints at each end + 5 per interior junction = enough degrees of freedom for any reasonable waypoint count. Lower order (e.g., 5th = "minimum-jerk") provides fewer free variables and may produce infeasible QPs for closely-spaced waypoints.

**Time normalisation (t ∈ [0,1])**: The QP is solved on the unit interval for numerical conditioning. If you were to solve it in physical time `τ ∈ [0, T]` with `T = 10 s`, the coefficient for `τ⁸` would be numerically tiny and would cause conditioning issues. The `1/T⁷` scaling of the objective accounts for this correctly.

**Snap cost vs jerk cost**: The code minimises snap (4th derivative). For short trajectories at low speed, minimising jerk (3rd derivative, polynomial order 6) is sufficient. Snap optimisation is necessary when jerk needs to be continuous (required by the angular velocity feedforward in the Lee controller).

---

## 7. Connections to Other Modules

| Direction | Module | What is exchanged |
|-----------|--------|------------------|
| Consumes | User / `bin/` | Waypoints and time allocations |
| Produces | `controller/` | `FlatOutput → FlatnessResult → TrajectoryReference` |
| Depends on | `math/` | `Vec3` for waypoints and flat output fields |
| Alternative to | `trajectory/` | `SplineTrajectory` replaces `CircleTrajectory`/`Figure8Trajectory` for custom paths |

---

## 8. Common Pitfalls

**Time normalisation is per-segment**: The coefficients `c₀..c₈` are in normalised time `t ∈ [0,1]`, not physical time. When evaluating derivatives, you *must* divide by `T^n` for the `n`-th derivative (see `eval` lines 203–217). Forgetting this produces velocities that are `T` times too large.

**QP infeasibility from close waypoints**: If two waypoints are very close in space but must be reached in a short time, the boundary conditions (zero velocity at start and end) conflict with needing high velocity in between. The solver will return an error. Solution: increase the segment duration or remove boundary conditions on intermediate waypoints.

**Flatness singularity when `f → 0`**: The angular velocity formula divides by `f = m·‖acc_g‖`. Near hover this is fine (`f ≈ mg`). If the planned trajectory requires near-zero thrust (e.g., a ballistic arc), the angular velocity becomes undefined. `compute_flatness` guards against `f = 0` by normalising `acc_g` only when nonzero (line 69 in `flatness.rs`), but this is worth watching.

**`c₃ = ‖ŷ_c × ẑ_b‖` near zero**: The yaw rate term in `ω_z` divides by `c₃`. When `ẑ_b` is nearly parallel to `ŷ_c` (drone tilted almost sideways toward the yaw reference), `c₃ → 0`. This is a genuine geometric singularity of the yaw parameterisation and does not occur during normal flight (the Crazyflie's maximum useful tilt is ~35°).

**Axis convention for rotation matrix**: `rot` in `FlatnessResult` is column-major: `rot[0] = x̂_b`, `rot[1] = ŷ_b`, `rot[2] = ẑ_b`. To recover the 3×3 rotation matrix `R[row][col]`, use `R[row][col] = rot[col][row]`.

---

## 9. Related Tests

| Test file | What it covers |
|-----------|---------------|
| `src/planning/spline.rs` (inline) | `plan_simple_trajectory`: 3 waypoints, checks position continuity and zero velocity at boundaries |
| `tests/test_spline.rs` | Multi-segment continuity (position, velocity, acceleration match at junctions), total time, boundary conditions |
| `src/planning/flatness.rs` (inline) | `hover_flatness`: zero acc/jerk/snap → thrust = mg, ω = 0, τ = 0 |
| `tests/test_flatness.rs` | Circular trajectory flatness consistency: expected angular velocity from known geometry |
| `src/bin/assignment4.rs` | Planned figure-8 via minimum-snap spline, closed-loop simulation |

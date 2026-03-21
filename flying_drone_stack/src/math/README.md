# Math — 3D Primitives (Vec3, Quat, Mat9)

## 1. Overview

The `math` module provides the core numerical building blocks used throughout the
stack.  Everything is `f32` to match the Crazyflie firmware and the MEKF Python
reference implementation.

| Type | File | Purpose |
|------|------|---------|
| **Vec3** | `vec3.rs` | 3D vector with arithmetic, dot/cross product, norm |
| **Quat** | `quaternion.rs` | Unit quaternion for 3D rotation (Hamilton product, axis-angle, ZYX Euler) |
| **Mat9** | `matrix.rs` | Dense 9×9 matrix for MEKF covariance; scalar EKF update operations |

All three types are `Copy + Clone + Debug`.

---

## 2. Mathematical Foundation

### 2.1 Vec3

Standard Euclidean 3-vector.  Operators implemented: `+`, `-`, `*` (scalar), `/`
(scalar), `+=`, `-=`, `*=`, negation.

Key functions:

| Function | Formula |
|----------|---------|
| `dot(a, b)` | `a.x·b.x + a.y·b.y + a.z·b.z` |
| `cross(a, b)` | `(a.y·b.z − a.z·b.y,  a.z·b.x − a.x·b.z,  a.x·b.y − a.y·b.x)` |
| `norm()` | `√(x²+y²+z²)` |
| `normalize()` | `v / ‖v‖`  (returns `Vec3::zero()` for zero vector) |

### 2.2 Quaternion

Unit quaternion `q = (w, x, y, z)` represents a rotation by angle `θ` around unit
axis `n`:

```
q = ( cos(θ/2),  n · sin(θ/2) )
```

Satisfies `‖q‖ = 1`.  Avoids gimbal lock and provides smooth composition via the
**Hamilton product**.

**Hamilton product** (`Mul` operator, `p * q`):

```
(p*q).w = p.w·q.w − p.x·q.x − p.y·q.y − p.z·q.z
(p*q).x = p.w·q.x + p.x·q.w + p.y·q.z − p.z·q.y
(p*q).y = p.w·q.y − p.x·q.z + p.y·q.w + p.z·q.x
(p*q).z = p.w·q.z + p.x·q.y − p.y·q.x + p.z·q.w
```

Composition is non-commutative: `p*q ≠ q*p` in general.

**Rotation of a vector** (`rotate_vector`):

```
v' = q ⊗ (0, v) ⊗ q*     where q* = conjugate(q) = (w, -x, -y, -z)
```

**ZYX Euler → Quaternion** (used in `build_state` and throughout the stack):

```
q = q_yaw(ψ) * q_pitch(θ) * q_roll(φ)
```

where each factor is `from_axis_angle(axis, half_angle)`.  The ZYX order means yaw
is applied first in the world frame, then pitch, then roll — matching the Crazyflie
firmware (`stabilizer.roll/pitch/yaw`) and the MEKF convention.

**Quaternion → ZYX Euler** (`to_euler`):

```
roll  = atan2( 2(wy + xz),  1 − 2(x²+y²) )
pitch = asin(  2(wy − zx) )                   [clamped at ±π/2 for gimbal lock]
yaw   = atan2( 2(wz + xy),  1 − 2(y²+z²) )
```

Returns `(roll, pitch, yaw)` in radians.

**Quaternion derivative** (used in MEKF and simulator):

```
q̇ = ½ · q ⊗ (0, ω_body)
```

**Exponential map** (for rotation integration):

```
exp(q) = cos(‖v‖) + v/‖v‖ · sin(‖v‖)    where v = (x, y, z)
```

Used by `integrate_exponential(ω, dt)` — more accurate than Euler integration for
large time steps.

**Rotation matrix** (`to_rotation_matrix`):

```
R = [ 1−2(y²+z²)    2(xy−wz)     2(xz+wy)  ]
    [ 2(xy+wz)      1−2(x²+z²)   2(yz−wx)  ]
    [ 2(xz−wy)      2(yz+wx)     1−2(x²+y²) ]
```

### 2.3 Mat9 — 9×9 MEKF Covariance Matrix

The MEKF state `x ∈ R⁹` = `[p(3), b(3), δ(3)]` has a 9×9 covariance `Σ`.
`Mat9` provides all the linear-algebra primitives the EKF needs.

**Key operations and their EKF roles**:

| Method | Math | EKF use |
|--------|------|---------|
| `mat_mul(B)` | `A·B` | Covariance propagation `F·Σ·Fᵀ` |
| `transpose()` | `Aᵀ` | Used in `F·Σ·Fᵀ` |
| `mat_vec(v)` | `A·v` | Apply Jacobian to state vector |
| `outer(u, v)` | `uᵀ·v` | Build rank-1 update `K·H` |
| `h_sigma_ht(H, Σ)` | `H·Σ·Hᵀ` | Innovation covariance `S = H·Σ·Hᵀ + R` |
| `sigma_ht(Σ, H)` | `Σ·Hᵀ` | Kalman gain numerator |
| `joseph_update(Σ, K, H, r)` | `(I-KH)·Σ·(I-KH)ᵀ + K·r·Kᵀ` | Numerically stable covariance update |
| `symmetrise()` | `Σ ← (Σ+Σᵀ)/2` | Prevents f32 rounding from breaking symmetry |
| `clamp_diagonal(max)` | `Σ[i][i] ← min(Σ[i][i], max)` | Mirrors firmware MAX_COVARIANCE guard |

**Joseph form** vs standard form:

The standard update `Σ' = (I-KH)·Σ` can accumulate numerical error and make `Σ`
non-positive-definite over time.  The Joseph form
`(I-KH)·Σ·(I-KH)ᵀ + K·r·Kᵀ` is provably positive-definite for any K, H, r > 0.
This is critical for embedded f32 EKFs running thousands of updates per flight.

---

## 3. Rotation Conventions

The stack uses a single consistent convention throughout:

| Convention | Value |
|------------|-------|
| Euler order | ZYX Tait-Bryan (yaw → pitch → roll) |
| Angle sense | Right-hand (positive = counter-clockwise viewed from +axis) |
| Quaternion composition | `q_total = q_yaw * q_pitch * q_roll` |
| Body frame | +X forward, +Y left, +Z up |
| Angular velocity | Body frame, rad/s |

**Pitch sign note**: the Crazyflie firmware RPYT commander applies an internal
negation `rpy2quat(roll, -pitch, yaw)`.  The `rpyt_control.rs` module accounts
for this by passing pitch un-negated (see that module's doc for the full sign chain).

---

## 4. Key Types

### `Vec3` — `src/math/vec3.rs`

```rust
pub struct Vec3 { pub x: f32, pub y: f32, pub z: f32 }
```

| Constructor | Description |
|-------------|-------------|
| `Vec3::new(x, y, z)` | Direct construction |
| `Vec3::zero()` | (0, 0, 0) |

### `Quat` — `src/math/quaternion.rs`

```rust
pub struct Quat { pub w: f32, pub x: f32, pub y: f32, pub z: f32 }
```

| Constructor | Description |
|-------------|-------------|
| `Quat::new(w, x, y, z)` | Direct construction |
| `Quat::identity()` | (1, 0, 0, 0) |
| `Quat::from_axis_angle(axis, angle_rad)` | Rotation around unit axis |

Free function: `to_euler(q) → (roll, pitch, yaw)` in radians (ZYX).

### `Mat9` — `src/math/matrix.rs`

```rust
pub struct Mat9 { pub data: [[f32; 9]; 9] }  // row-major
```

| Constructor | Description |
|-------------|-------------|
| `Mat9::zeros()` | All entries 0 |
| `Mat9::identity()` | 1 on diagonal |
| `Mat9::diag([f32; 9])` | Custom diagonal |

---

## 5. Usage Examples

```rust,no_run
use multirotor_simulator::math::{Vec3, Quat, Mat9};

// Vec3 arithmetic
let a = Vec3::new(1.0, 0.0, 0.0);
let b = Vec3::new(0.0, 1.0, 0.0);
let c = a.cross(b);               // (0, 0, 1)
assert!((c.z - 1.0).abs() < 1e-6);

// Quaternion rotation
let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0),
                               std::f32::consts::FRAC_PI_2); // 90° yaw
let rotated = q.rotate_vector(Vec3::new(1.0, 0.0, 0.0));
// body_x now points in world_y: (0, 1, 0)

// ZYX Euler → Quat → back
use multirotor_simulator::math::to_euler;
let (roll, pitch, yaw) = to_euler(q);

// 9×9 EKF covariance update
let sigma = Mat9::identity();
let h = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // measure state[0]
let s = Mat9::h_sigma_ht(&h, &sigma); // innovation covariance = 1.0
let k = Mat9::sigma_ht(&sigma, &h);   // Kalman gain numerator
let r = 0.1;                          // measurement noise
let sigma_new = Mat9::joseph_update(&sigma, &k, &h, r);
```

---

## 6. Common Pitfalls

**Quaternion normalisation drift**: every multiplication introduces a tiny
`‖q‖ = 1 + ε` error. Call `.normalize()` after each integration step. The MEKF
does this at the end of `mekf_predict`.

**`to_euler` gimbal lock**: when `|sinp| ≥ 1` (pitch = ±90°), roll and yaw share
a degree of freedom. `to_euler` clamps and returns `pitch = ±π/2`, but roll and
yaw values become unreliable. The drone never reaches 90° pitch in normal flight.

**`from_axis_angle` requires a unit axis**: the axis vector is NOT normalised
internally. Passing a non-unit axis scales the rotation angle silently.

**Mat9 is row-major**: `data[i][j]` is row `i`, column `j`. This matches C
convention but differs from Fortran/MATLAB column-major. All internal operations
are consistent, but be careful when indexing manually.

**`symmetrise` vs `joseph_update`**: call `symmetrise` after covariance propagation
(`F·Σ·Fᵀ + Q`). Call `joseph_update` instead of the standard form for measurement
updates.  Do not mix: using `symmetrise` alone after a standard update does not
restore positive-definiteness.

---

## 7. Related Tests

| Test location | What it covers |
|---------------|---------------|
| `src/math/vec3.rs` (inline, 12 tests) | Arithmetic operators, dot, cross, norm, normalize edge cases |
| `src/math/quaternion.rs` (inline, 13 tests) | Hamilton product, axis-angle, rotate_vector, to_euler round-trip, integration |
| `src/math/matrix.rs` (inline, 15 tests) | zeros/identity/diag, transpose, mat_mul, add, scale, mat_vec, outer, h_sigma_ht, sigma_ht, joseph_update symmetry, symmetrise, clamp_diagonal |

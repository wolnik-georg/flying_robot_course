# math/ — Mathematical Primitives

## Vec3

```rust
pub struct Vec3 { pub x: f32, pub y: f32, pub z: f32 }
// COPY — always pass by value, never &Vec3 unless needed for struct field

impl Vec3 {
    fn new(x, y, z)     fn zero()
    fn norm(&self)       fn normalize(&self)    // safe: returns e3 if near-zero
    fn dot(&self, b)     fn cross(&self, b)
    fn component_mul(&self, b)                  // Hadamard product
}
// Operators: + - * (by f32 or Vec3) neg
```

## Quat

```rust
pub struct Quat { pub w: f32, pub x: f32, pub y: f32, pub z: f32 }
// COPY — [w, x, y, z] with scalar FIRST

impl Quat {
    fn new(w, x, y, z)   fn identity()
    fn from_axis_angle(axis: Vec3, angle: f32)
    fn conjugate(&self)                   // = inverse for unit quaternion
    fn norm(&self)        fn normalize(&self)
    fn rotate_vector(&self, v: Vec3) -> Vec3
    fn derivative(&self, omega: Vec3) -> Quat   // dq/dt = 0.5 * q ⊗ ω_quat
    fn integrate(&self, omega: Vec3, dt: f32) -> Quat        // axis-angle
    fn integrate_exponential(&self, omega: Vec3, dt: f32) -> Quat  // exp map
    fn to_rotation_matrix(&self) -> [[f32; 3]; 3]   // row-major
}
// Operators: * (Hamilton product)  + (quaternion addition)

pub fn to_euler(q: Quat) -> (f32, f32, f32)  // (roll, pitch, yaw) [rad], ZYX convention
```

**Convention**: `[w, x, y, z]` with w scalar first.
**Firmware exception**: firmware uses `q3=qw` (scalar last) — see `firmware_app/CLAUDE.md`.

## Mat9

```rust
pub struct Mat9 { pub data: [[f32; 9]; 9] }
// Row-major 9×9 matrix for MEKF covariance

impl Mat9 {
    fn zeros()    fn identity()    fn diag(d: [f32; 9])
    fn transpose(&self)
    fn mat_mul(&self, rhs: &Mat9) -> Mat9
    fn mat_vec(&self, v: &[f32; 9]) -> [f32; 9]
    fn add(&self, rhs: &Mat9) -> Mat9
    fn scale(&self, s: f32) -> Mat9
    fn outer(u: &[f32; 9], v: &[f32; 9]) -> Mat9    // u ⊗ vᵀ
    fn joseph_update(sigma, k, h, r) -> Mat9         // numerically stable P update
    fn h_sigma_ht(h, sigma) -> f32                   // scalar H·Σ·Hᵀ
    fn sigma_ht(sigma, h) -> [f32; 9]                // Σ·Hᵀ vector
    fn symmetrise(&mut self)                         // force symmetric (prevent f32 drift)
    fn clamp_diagonal(&mut self, max_val: f32)       // cap diagonal entries
}
```

## Rotation matrices

Rotation matrices throughout the codebase use `[[f32; 3]; 3]` row-major:
- `mat[row][col]`
- Column 2 (`mat[0][2], mat[1][2], mat[2][2]`) = body z-axis in world frame
- Transpose = inverse (SO(3))

```rust
// From quaternion:
let r: [[f32; 3]; 3] = q.to_rotation_matrix();  // Quat method
let r = quat_to_rot(q_arr);                       // mekf.rs utility, takes [f32;4]

// From flatness:
let res = compute_flatness(&flat, mass);
let r: [[f32; 3]; 3] = res.rot;                  // body→world

// To quaternion:
let q: [f32; 4] = rot_to_quat(&r);              // planning module, returns [w,x,y,z]
```

## Common operations

```rust
// Attitude from CSV (deg → rad → quaternion)
let (roll, pitch, yaw) = (roll_deg.to_radians(), pitch_deg.to_radians(), yaw_deg.to_radians());
// Use build_state() from flight/ instead — handles this conversion

// Quaternion derivative for ODE
let q_dot = q.derivative(omega_body);   // omega in rad/s body frame
let q_new = q + q_dot * dt;
let q_new = q_new.normalize();          // always renormalize after integration

// Cross product for gyroscopic term
let gyro_comp = omega.cross(j_omega);   // ω × (J·ω)
```

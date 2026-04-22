# planning/ — Spline Planner + Differential Flatness

## What this module does

Two independent sub-systems:
1. **Min-snap spline planner** (`spline.rs`): solves a QP (Clarabel) for degree-8 (9-coefficient)
   piecewise polynomials in normalized time τ∈[0,1] that minimise ∫snap².
2. **Differential flatness** (`flatness.rs`): converts flat outputs (pos + derivatives up to snap,
   yaw + derivatives) into full rigid-body state: thrust, rotation matrix, ω, ω̇, torque.
3. **Exploration FSM** (`exploration.rs`): SCAN→NAVIGATE→LAND frontier planner.

## Key types

```rust
pub struct Waypoint {
    pub pos: Vec3,   // position [m]
    pub yaw: f32,    // yaw [rad]
}

pub struct FlatOutput {
    pub pos: Vec3,       // position
    pub vel: Vec3,       // 1st derivative
    pub acc: Vec3,       // 2nd derivative
    pub jerk: Vec3,      // 3rd derivative
    pub snap: Vec3,      // 4th derivative
    pub yaw: f32,
    pub yaw_dot: f32,
    pub yaw_ddot: f32,
}

pub struct FlatnessResult {
    pub pos: Vec3,
    pub vel: Vec3,
    pub rot: [[f32; 3]; 3],  // rotation matrix (body→world, col-major body axes)
    pub thrust: f32,          // [N]
    pub omega: Vec3,          // angular velocity [rad/s] body frame
    pub omega_dot: Vec3,      // angular acceleration [rad/s²] body frame
    pub torque: Vec3,         // [Nm]
}
```

## Key functions

```rust
// Plan a min-snap trajectory
let traj = SplineTrajectory::plan(
    waypoints: &[Waypoint],
    durations: &[f32],       // one per segment (NOT one per waypoint)
    periodic: bool,          // true = smooth loop (derivatives wrap at junction)
) -> Result<SplineTrajectory, String>;

// Evaluate at time t (physical seconds, wraps if looped in caller)
let flat: FlatOutput = traj.eval(t);  // method is eval(), NOT evaluate()

// Differential flatness
let res: FlatnessResult = compute_flatness(&flat, mass_kg);

// Rotation matrix → quaternion [w, x, y, z]
let q: [f32; 4] = rot_to_quat(&res.rot);

// Convert FlatOutput to TrajectoryReference (for GeometricController)
let ref_: TrajectoryReference = flatness_to_reference(&flat);
```

## Typical usage pattern (spline_circle_test.rs style)

```rust
// 1. Plan
let waypoints: Vec<Waypoint> = (0..=n).map(|i| { ... }).collect();
let durations = vec![seg_dur; n];
let traj = SplineTrajectory::plan(&waypoints, &durations, true)?;

// 2. Loop at 25 Hz
let flat = traj.eval(t % traj.total_time);   // wrap for looping
let res = compute_flatness(&flat, 0.031);
let q = rot_to_quat(&res.rot);

// 3. Send full-state setpoint
cf.commander.setpoint_full_state(
    res.pos.x, res.pos.y, res.pos.z,
    res.vel.x, res.vel.y, res.vel.z,
    flat.acc.x, flat.acc.y, flat.acc.z,
    q[0], q[1], q[2], q[3],
    res.omega.x, res.omega.y, res.omega.z,
).await?;
```

## Poly4D export (for HLC Python scripts)

`src/bin/export_poly4d.rs` converts the degree-8 normalized coefficients to degree-7
physical-time coefficients via Hermite interpolation (`to_hermite_phys7`).

```bash
cargo run --release --bin export_poly4d           # circle
cargo run --release --bin export_poly4d figure8   # figure-8
# stdout: Python list; stderr: waypoint validation errors
```

**Why degree reduction?** Poly4D format (Crazyflie HLC) supports max degree 7.
The Hermite conversion extracts 8 boundary conditions (pos/vel/acc/jerk at both ends)
and solves analytically for a C3-continuous degree-7 polynomial — no velocity jumps.

## Gotchas

- `SplineTrajectory::plan` third arg is `periodic: bool`. For a closed circle: `true`.
  With `periodic=false` on a closed loop, the planner bows the path ~17% outward.
- `durations` has length `n_segments` (= `waypoints.len() - 1` for open, = `waypoints.len() - 1`
  for periodic since first==last).
- `traj.total_time` is the sum of all durations. For looping, caller wraps: `t % traj.total_time`.
- `compute_flatness` uses `mass` in kg for thrust scaling. Use `0.031` for CF 2.1 + decks.
- `rot_to_quat` takes `&[[f32; 3]; 3]` (reference). Returns `[f32; 4]` = [w, x, y, z].
- The planner works in normalized time τ∈[0,1] internally; `eval()` takes physical time in seconds.

## SplineSegment internals

```rust
pub struct SplineSegment {
    pub cx:   [f32; 9],  // 9 coefficients for x, normalized time
    pub cy:   [f32; 9],
    pub cz:   [f32; 9],
    pub cyaw: [f32; 9],
    pub duration: f32,   // segment duration [s]
}
pub struct SplineTrajectory {
    pub segments: Vec<SplineSegment>,
    pub total_time: f32,
}
```

## Exploration FSM (exploration.rs)

```rust
pub struct ExplorationPlanner { ... }
pub enum ExplorationCommand {
    Scan { yaw_deg: f32 },
    GoTo { x: f32, y: f32, z: f32, yaw_deg: f32 },
    Hold { x: f32, y: f32, z: f32, yaw_deg: f32 },
    Land,
}

impl ExplorationPlanner {
    pub fn new(omap: Arc<Mutex<OccupancyMap>>) -> Self
    pub fn step(&mut self, pos: Vec3, yaw_deg: f32) -> ExplorationCommand
}
```

State machine: **SCAN** (360° rotation, 3°/step accumulating) → **NAVIGATE** (frontier GoTo) → **LAND**.
Bug fix April 7 2026: SCAN now accumulates yaw correctly (`start_yaw + accumulated`, not incremental).

# Next Steps: State Estimation & Motion Planning

**Current status:** Control is parked on onboard firmware (`MANEUVER = "circle"`).
The firmware handles XY velocity control via its PID cascade (100 Hz position + 500 Hz
attitude), sending only a `setpoint_hover` (velocity + absolute height) from our Rust
code.  This gives reliable, stable flight without Lighthouse and frees us to focus on
estimation and planning.

---

## Phase 1 — State Estimation: MEKF

### What already exists
- `src/estimation/mekf.rs` — complete MEKF implementation
  - State: `[p(3), b(3), δ(3)]` + reference quaternion `q_ref` (9-dim + SO3)
  - IMU (gyro + accel) as control input → `mekf_predict`
  - Height measurement (ToF range sensor) → `mekf_update_height`
  - Optical flow measurement → `mekf_update_flow`
  - Convenience wrapper `Mekf::feed_row` for CSV-driven offline evaluation
- `tests/test_mekf.rs` — unit tests: reset, predict, height update, flow update, feed_row

### Step 1.1 — Run existing tests, confirm green
```
cargo test --test test_mekf
```
All 11 tests should pass.  If any fail, fix before proceeding.

### Step 1.2 — Offline validation against real flight data
Load a real flight CSV (`src/bin/runs/*.csv`) into the MEKF and compare its
position/attitude estimates against the onboard EKF (`stateEstimate.*`).

Write a binary or test:
```rust
// src/bin/mekf_eval.rs  (or a #[test] in tests/)
// 1. Parse CSV rows (time, gyro, accel, range_mm, flow_dnx, flow_dny)
// 2. Feed each row into Mekf::feed_row
// 3. Compare MEKF position to EKF columns, print RMSE
```
Target: MEKF height RMSE vs EKF Z < 5 cm.
XY: MEKF should track dead-reckoning velocity at least as well as the onboard EKF.

### Step 1.3 — Tune noise parameters if needed
`MekfParams::default()` was grid-searched in Python (`State Estimation/mekf_offline.py`).
If the offline RMSE shows systematic bias, adjust:
- `q_vel` (velocity process noise) — increase if MEKF drifts on straight segments
- `r_flow` (flow measurement noise) — decrease if MEKF ignores flow too much
- `r_height` (height noise) — very low (1e-3) is correct for the ToF sensor

### Step 1.4 — Real-time integration into the flight loop
Add MEKF to `main.rs`:
```rust
use multirotor_simulator::estimation::{Mekf, MekfParams};
let mut mekf = Mekf::new(MekfParams::default());
// seed q_ref from the first EKF attitude at takeoff (stabilizer roll/pitch/yaw → quat)

// Inside the 100 Hz loop:
let estimate = mekf.feed_row(
    t_s,
    Some([entry.gyro_x, entry.gyro_y, entry.gyro_z]),
    Some([acc_x_g, acc_y_g, acc_z_g]),     // log acc.x / acc.y / acc.z
    Some(entry.range_z * 1000.0),           // mm
    Some(flow_dpx), Some(flow_dpy),         // log motion.deltaX / deltaY
);
```
Add `acc.x`, `acc.y`, `acc.z`, `motion.deltaX`, `motion.deltaY` to the log block.

### Step 1.5 — Fly with MEKF estimates instead of onboard EKF
Switch the circle mode to use MEKF position/velocity output instead of
`stateEstimate.*`.  Log both and compare in-flight — this is the success criterion.

---

## Phase 2 — Motion Planning: Minimum-Snap Spline

### What already exists
- `src/planning/spline.rs` — minimum-snap 8th-order polynomial QP (Clarabel backend)
  - `SplineTrajectory::plan(waypoints, durations)` → solves QP
  - `SplineTrajectory::eval(t)` → returns `FlatOutput` (pos, vel, acc, jerk, snap, yaw)
  - Inline test: `plan_simple_trajectory` (3 waypoints, checks boundary conditions)
- `src/planning/flatness.rs` — differential flatness (Faessler et al. 2018)
  - `compute_flatness(flat, mass)` → desired rotation, thrust, ω, ω̇, torque
  - `flatness_to_reference(fr, ...)` → `TrajectoryReference` for geometric controller
  - Inline test: `hover_flatness`

### Step 2.1 — Run existing tests, confirm green
```
cargo test -p multirotor_simulator
```
`hover_flatness` and `plan_simple_trajectory` should both pass.

### Step 2.2 — Generate a circle trajectory with the spline planner
A circle of radius R at height h can be approximated with N waypoints evenly spaced
on the circle; equal segment durations = `2πR / (N * v_max)`.

Add a test `plan_circle_trajectory`:
```rust
// 8 waypoints on a circle, radius 0.5 m, height 0.3 m
// Check: eval(0) at (R,0,h), eval(T/4) near (0,R,h), eval(T) back at (R,0,h)
// Check: eval velocity at t=0 is approximately tangential (vy ≈ v, vx ≈ 0)
```

### Step 2.3 — Generate a figure-8 trajectory
Figure-8 = two circles, connected at the origin with direction reversal.
Use N waypoints (e.g. 16), alternating CW and CCW arcs.

### Step 2.4 — Integrate spline + flatness + geometric controller in `my_circle`
Replace the current circular parametric trajectory with the spline planner output:
```rust
// Offline: plan the circle once before the control loop
let traj = SplineTrajectory::plan(&circle_waypoints, &durations)?;

// In the loop (t_circle = elapsed time since circle_started):
let flat = traj.eval(t_circle);
let reference = TrajectoryReference {
    position: flat.pos,
    velocity: flat.vel,
    acceleration: flat.acc,
    jerk: flat.jerk,
    yaw: flat.yaw,
    yaw_rate: flat.yaw_dot,
    yaw_acceleration: flat.yaw_ddot,
};
// Then use compute_force_vector + force_vector_to_rpyt (geometric controller)
```

### Step 2.5 — Fly: spline circle with onboard EKF, then with MEKF
- First flight: spline reference + RPYT + EKF position (if Lighthouse is present)
- Second flight: spline reference + RPYT + MEKF position (no Lighthouse)
- Success metric: mean position error < 5 cm on a 0.5 m radius circle at ω=0.5 rad/s

---

## Summary table

| Step | What | Success criterion |
|------|------|-------------------|
| 1.1 | `cargo test --test test_mekf` | All 11 tests green |
| 1.2 | Offline CSV validation | Height RMSE < 5 cm vs EKF |
| 1.3 | Tune noise params | No systematic bias |
| 1.4 | MEKF in real-time loop | Finite output every tick |
| 1.5 | Fly with MEKF | Comparable tracking to EKF |
| 2.1 | `cargo test -p multirotor_simulator` | All inline tests green |
| 2.2 | Circle spline test | Waypoints satisfied < 1 cm |
| 2.3 | Figure-8 spline test | Direction reversal smooth |
| 2.4 | Spline + flatness in `my_circle` | Builds and compiles |
| 2.5 | Flight with spline | Error < 5 cm, no crash |

---

## Dependency order

```
1.1 → 1.2 → 1.3 → 1.4 → 1.5
2.1 → 2.2 → 2.3 → 2.4 → 2.5
                         ↑
                      (1.5 feeds here — MEKF replaces EKF in 2.5)
```

Steps 1.x and 2.x up to 2.4 are independent and can be worked in parallel.

---

## Files to add (not yet existing)

| File | Purpose |
|------|---------|
| `src/bin/mekf_eval.rs` | Offline CSV → MEKF → RMSE printout |
| `tests/test_spline_circle.rs` | Circle and figure-8 spline tests |
| Log variables to add in `main.rs` | `acc.x/y/z`, `motion.deltaX/deltaY` |

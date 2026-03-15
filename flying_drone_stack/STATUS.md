# Flying Drone Stack — Current State & Next Steps

> Last updated March 15, 2026.

---

## 0. Course Deliverables — ✅ ALL COMPLETE

All 5 assignments compile, run, and produce plots with zero errors.
Run `bash verify_all.sh` from the project root to reproduce everything end-to-end.

| Assignment | Binary | Plot script | Status |
|---|---|---|---|
| 1 — Integration methods (Euler/RK4/Exp) | `assignment1` | `plot_assignment1.py` | ✅ |
| 2 — Geometric control (hover/circle/fig8) | `assignment2` | `plot_assignment2.py` | ✅ |
| 3 — MEKF offline vs on-board EKF | `assignment3` | `plot_assignment3.py` | ✅ |
| 4 — Minimum-snap spline + flatness | `assignment4` | `plot_assignment4.py` | ✅ |
| 5 — Safe-space trajectories + MEKF | `assignment5` | `plot_assignment5.py` | ✅ |

**Test suite:** 246 tests, 0 failures.

**Python env for plots:** `PYENV_VERSION=flying_robots` (numpy 2.4, matplotlib 3.10, pandas 3.0).
Do **not** use system Python — its matplotlib is compiled against numpy 1.x and crashes with numpy 2.x.

**Assignment 3 RMSE (fr00 figure-8 flight):**
- Orientation: roll 4.1°, pitch 3.6°, yaw 7.1° vs on-board EKF
- Position: x=246 cm, y=154 cm, z=0.9 cm — large XY because `fr00` was logged with Lighthouse
  (absolute positioning) so EKF has a fixed world anchor while MEKF dead-reckons from zero.
  The MEKF Z is excellent (0.9 cm) which is the only axis with an absolute sensor (ToF range).

---

## 1. Repository Layout

```
flying_drone_stack/
├── src/
│   ├── lib.rs                     # crate root — all modules exposed
│   ├── safety.rs                  # SafetyLimits, check_safety
│   ├── math/                      # Vec3, Quat, Mat9 (Joseph-form, symmetrise, clamp)
│   ├── dynamics/                  # MultirotorState, MultirotorParams, MultirotorSimulator
│   ├── integration/               # Euler, RK4, ExpEuler, ExpRK4 (Integrator trait)
│   ├── controller/                # GeometricController (SE(3) Lee 2010), TrajectoryReference
│   ├── trajectory/                # CircleTrajectory, Figure8Trajectory, CsvTrajectory
│   ├── planning/
│   │   ├── spline.rs              # SplineTrajectory::plan / ::eval, Waypoint, FlatOutput
│   │   └── flatness.rs            # compute_flatness, flatness_to_reference, FlatnessResult
│   ├── estimation/
│   │   └── mekf.rs                # Mekf, MekfState, MekfParams — full MEKF f32 impl
│   ├── flight/                    # (flight helpers)
│   └── bin/
│       ├── main.rs                # real-flight binary (~1340 lines) — shadow controller live
│       ├── mekf_eval.rs           # offline CSV replay → MEKF vs firmware EKF RMSE
│       ├── assignment1..5.rs      # course assignment binaries
│       └── sim_closed_loop.rs     # closed-loop simulation
├── tests/
│   ├── test_mekf.rs
│   ├── test_spline.rs
│   ├── test_flatness.rs
│   ├── test_geometric_controller.rs
│   ├── test_controller_detailed.rs
│   ├── test_hover_control_loop.rs
│   ├── test_flight_math.rs
│   ├── test_math.rs
│   └── test_safety.rs
├── scripts/
│   ├── plot_mekf_eval.py          # fixed and working (ref_* columns)
│   ├── plot_assignment1..5.py
│   └── plot_assignment4_standalone.py
├── runs/                          # real flight CSVs (see §3)
├── results/images/                # generated plots
├── challenges.md                  # documented real-flight bugs + fixes
├── NEXT_STEPS.md                  # original planning doc (superseded by this file)
└── STATUS.md                      # ← this file
```

---

## 2. Module API Quick-Reference

### `estimation/mekf.rs`
```rust
pub struct MekfParams {
    q_pos: f32,    // 1e-7  process noise position
    q_vel: f32,    // 1e-3  process noise body velocity
    q_att: f32,    // 1e-6  process noise attitude error
    r_height: f32, // 1e-3  height measurement noise
    r_flow: f32,   // 8.0   flow measurement noise [px²]
}

pub struct MekfState { q_ref: [f32;4], x: [f32;9], sigma: Mat9 }
// x = [p(3), b(3), δ(3)]  — position, body-vel, attitude error

pub fn mekf_predict(state, gyro_rads, accel_ms2, dt, q_noise)
pub fn mekf_update_height(state, z_m, r_height)
pub fn mekf_update_flow(state, dnx, dny, dt_flow, r_flow, pz_meas: Option<f32>)

pub struct Mekf { state: MekfState, params: MekfParams, ... }
impl Mekf {
    pub fn new(params) -> Self
    pub fn seed_from_quat(&mut self, q: [f32;4])
    pub fn feed_row(&mut self, t_s, gyro, accel, range_mm, flow_dx, flow_dy) -> MekfOutput
}

// Key calibration constants (in mekf.rs):
const NP: f32 = 350.0;
const THETA_P: f32 = 3.50;   // NP/THETA_P = 100 — empirical March 15 2026
                              // DO NOT re-calibrate using fw EKF amplitude ratios (causes drift contamination)
```

### `planning/spline.rs`
```rust
pub struct Waypoint { pub pos: Vec3, pub yaw: f32 }
pub struct FlatOutput { pos, vel, acc, jerk, snap: Vec3, yaw, yaw_dot, yaw_ddot: f32 }

pub struct SplineTrajectory { segments: Vec<SplineSegment>, durations: Vec<f32> }
impl SplineTrajectory {
    pub fn plan(waypoints: &[Waypoint], durations: &[f32]) -> Result<Self, String>
    pub fn eval(&self, t: f32) -> FlatOutput   // clamps t to [0, total_duration]
    pub fn total_duration(&self) -> f32
}
```

### `planning/flatness.rs`
```rust
pub fn compute_flatness(flat: &FlatOutput, mass: f32) -> FlatnessResult
// FlatnessResult { pos, vel, rot: [[f32;3];3], thrust: f32, omega: Vec3, omega_dot: Vec3, torque: Vec3 }

pub fn flatness_to_reference(fr: &FlatnessResult, acc, jerk, psi, psi_dot, psi_ddot) -> TrajectoryReference
pub fn rot_to_quat(rot: &[[f32;3];3]) -> [f32;4]
```

### `controller/` (GeometricController)
```rust
// Produces roll_cmd [deg], pitch_cmd [deg], yaw_rate_cmd [deg/s] from:
//   current state (pos, vel, quat, omega) + TrajectoryReference
// Sign conventions (documented in challenges.md):
//   pitch:    send un-negated — firmware negates internally (negation documented, bug #3)
//   yaw_rate: negate before sending — firmware sign is CW from above (bug #4)
```

---

## 3. Real Flight Data

Located in `flying_drone_stack/runs/` and `bitcraze_rs_lib/runs/`.

| File | Maneuver | Key use |
|------|----------|---------|
| `hover_2026-03-01_*.csv` | Hover | MEKF Z validation |
| `circle_2026-03-01_*.csv` | Circle R=0.5m | MEKF XY validation |
| `figure8_2026-03-01_*.csv` | Figure-8 | MEKF XY validation |

**CSV columns (37 total, March 15 schema):**
```
time_ms, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
roll, pitch, yaw, thrust, vbat,
gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z,
rate_roll, rate_pitch, rate_yaw,
range_z, flow_dx, flow_dy,
mekf_roll, mekf_pitch, mekf_yaw, mekf_x, mekf_y, mekf_z,
our_ref_x, our_ref_y, our_ref_z,
our_thrust, our_roll_cmd, our_pitch_cmd, our_yaw_rate_cmd
```

**Note:** The `our_*` columns (shadow controller outputs) are **all zeros** in the March 1 logs because the shadow infrastructure was added later. These columns will be populated in the next real flight.

**MEKF eval RMSE (THETA_P=3.50, March 15 re-run):**
| Maneuver | RMSE X | RMSE Y | RMSE Z |
|----------|--------|--------|--------|
| Hover    | 4.5 cm | 3.7 cm | 1.2 cm |
| Circle   | 12.7 cm| 17.8 cm| 2.1 cm |
| Figure-8 | 9.3 cm | 9.6 cm | 1.8 cm |

**Important finding:** Firmware EKF drifts ~30 cm during hover (46% zero flow samples). MEKF stays near origin — MEKF is actually more accurate during hover. The circle XY RMSE drops from 24.9 cm → 10.1 cm when the firmware EKF DC offset is subtracted, confirming the MEKF shape tracking is good.

---

## 4. Shadow Controller (`src/bin/main.rs`)

The shadow controller runs our full stack **in parallel** with the firmware — it never sends commands to the drone, only logs them.

```rust
enum ShadowManeuver {
    Hover   { cx: f32, cy: f32, height: f32 },
    Circle  { cx: f32, cy: f32, height: f32, radius: f32, omega: f32 },
    Figure8 { cx: f32, cy: f32, height: f32, a: f32, b: f32, omega: f32 },
}

struct ShadowCtx {
    maneuver: ShadowManeuver,
    traj_start: Option<Instant>,   // None = ramp phase, Some = trajectory active
    controller: GeometricController,
    params: MultirotorParams,
}
// ShadowCtx::compute(entry, mekf_seeded) -> Option<(ref_x,ref_y,ref_z,thrust,roll,pitch,yaw_rate)>
```

`fw_logging_step` signature (11 args, including `shadow: &mut ShadowCtx`).
Shadow fields are written to the last 7 CSV columns every tick once MEKF is seeded.

**Status:** Compiles clean (`cargo build --bin main`). All 246 tests passing.

---

## 5. Test Coverage

```
248 tests total, 0 failing (as of March 15 2026)

test_math.rs              — Vec3, Quat, Mat9 ops
test_mekf.rs              — reset, predict, height/flow update, feed_row, zero-flow gate (2 new)
test_spline.rs            — plan_simple_trajectory (3 waypoints, boundary conditions),
                            test_spline_circle_waypoints, test_spline_figure8_waypoints
test_flatness.rs          — hover_flatness, flatness chain
test_geometric_controller.rs  — creation, rotation error, hover control
test_controller_detailed.rs   — detailed SE(3) controller tests
test_hover_control_loop.rs    — closed-loop hover sim
test_flight_math.rs       — flight math utilities
test_safety.rs            — SafetyLimits, check_safety
```

---

## 6. Known Issues / Do-Not-Touch

1. **THETA_P must stay at 3.50 (single constant).**
   Per-axis calibration (THETA_P_X=3.16, THETA_P_Y=5.70) was attempted March 15 and reverted — it made RMSE *worse* because firmware EKF drift contaminates amplitude ratios. Any future re-calibration needs motion-capture or UWB ground truth.

2. **Pitch sign / yaw rate sign are counter-intuitive but correct.**
   See `challenges.md` §3 and §4. Do not "fix" them.

3. **`our_*` shadow columns are zeros in all existing CSVs.**
   All runs so far were made with the `bitcraze_rs_lib` binary (30-column format).
   The `flying_drone_stack` main binary (37-column format) has been flown once and
   compiles clean (`cargo build --bin main`) but no new flight has been taken yet to populate `our_*`.

---

## 7. Post-Course Progress (Track A & Track B)

### Track B — MEKF & Shadow Code Improvements

| Item | Status | Notes |
|------|--------|-------|
| Flow outlier gate (`zero_flow_threshold = 0.3`) | ✅ Done | `MekfParams` + `Mekf::feed_row` gating; 2 new tests added |
| Shadow trajectory: use `CircleTrajectory` / `SmoothFigure8Trajectory` | ✅ Done | Replaces hand-coded sinusoid in `ShadowCtx::compute()` |
| All 248 tests passing | ✅ | `cargo test` 0 failures |
| `cargo build --bin main` clean | ✅ | All 9 `fw_logging_step` call sites fixed |

**MEKF flow gate detail:**
`zero_flow_threshold: f32 = 0.3` added to `MekfParams`. In `Mekf::feed_row`, any flow sample where `|dnx| < 0.3 && |dny| < 0.3` is silently skipped — these are PMW3901 zero-padding artefacts (46% of airborne samples). The gate is in `feed_row` (not inside `mekf_update_flow`) so the clean low-level function signature is preserved.

### Track A — Shadow Validation Flight

| Item | Status | Notes |
|------|--------|-------|
| `scripts/plot_shadow_eval.py` | ✅ Done | Works with both 30-col and 37-col CSVs |
| New flight with `flying_drone_stack main` binary | ⬜ TODO | Needed to populate `our_*` columns |
| Roll/pitch RMSE < 5° validation | ⬜ Blocked on flight | Script is ready; will run automatically |

**Using `plot_shadow_eval.py` now (30-column CSVs from today):**
```bash
PYENV_VERSION=flying_robots python3 scripts/plot_shadow_eval.py runs/circle_2026-03-15_11-46-16.csv
# → prints warning, saves shadow_overview_*.png flight overview plot
```

**After next main-binary flight (37-column CSV):**
```bash
PYENV_VERSION=flying_robots python3 scripts/plot_shadow_eval.py runs/circle_<date>.csv
# → full shadow comparison: roll/pitch cmd vs actual, thrust, XY reference vs actual
```

---

**File:** `src/estimation/mekf.rs`

Add `zero_flow_threshold: f32` field to `MekfParams` (default `0.3`).

In `mekf_update_flow`, before the scale computation, add:
```rust
if dnx.abs() < params.zero_flow_threshold && dny.abs() < params.zero_flow_threshold {
    return; // skip — zero-motion artefact (PMW3901 fires at <20 Hz)
}
```

`mekf_update_flow` currently doesn't receive `params` — pass `zero_flow_threshold: f32` as an extra arg OR add it to the `Mekf` wrapper's `feed_row` path. The `Mekf::feed_row` method already has `self.params` available; route the threshold from there.

Add a test in `test_mekf.rs`: call `feed_row` with `flow_dx=0.0, flow_dy=0.0` and assert the velocity state does not change.

---

### Step 1 — ✅ DONE: Flow outlier gate in MEKF

`zero_flow_threshold: f32 = 0.3` added to `MekfParams`. Gate in `Mekf::feed_row`.
Two tests added: `test_flow_zero_gate_skips_update`, `test_flow_above_threshold_updates_velocity`.
**248 tests, 0 failures.**

---

### Step 2 — ✅ DONE: Shadow trajectory uses `CircleTrajectory` / `SmoothFigure8Trajectory`

`ShadowCtx::compute()` circle arm now calls `CircleTrajectory::with_center(...).get_reference(t)`.
Figure-8 arm calls `SmoothFigure8Trajectory { duration, height, a, b, ... }.get_reference(t)` with centre offset applied.
`cargo build --bin main` clean. All 9 `fw_logging_step` call sites have `&mut shadow` argument.

---

### Step 3 — ✅ DONE: `scripts/plot_shadow_eval.py`

Works for both CSV formats. With 30-col CSVs (today's flights): prints a clear message and saves a 2×2 flight overview plot. With 37-col CSVs (after a main-binary flight): full shadow comparison (roll/pitch cmd vs actual, thrust, XY reference vs trajectory).

---

### Step 4 — ⬜ TODO: New real flight with `flying_drone_stack main` binary

**Prerequisite for Track A validation.**

```bash
# Build (already clean)
cargo build --release --bin main
# Set MANEUVER = "circle" in main.rs, connect Crazyflie, run:
./target/release/main
# CSV written to runs/circle_<date>.csv  (37 columns, our_* populated)
# Then evaluate:
PYENV_VERSION=flying_robots python3 scripts/plot_shadow_eval.py runs/circle_<date>.csv
```
Success criterion: **Roll/pitch RMSE < 5°** during steady-state circle.

---

### Step 5 — ⬜ TODO: Write `tests/test_spline_circle.rs` and `test_spline_figure8.rs`

Spline circle/figure-8 test stubs from the original plan. Low priority (library is tested via assignment5 and existing test_spline.rs).

---

### Step 6 — New real flight (hardware, ~1 hour)

1. `cargo build --release --bin main` (already clean)
2. Connect Crazyflie, run `main` with `MANEUVER = "circle"` then `"figure8"`
3. Collect CSVs with `our_*` columns populated
4. Run Step 4 analysis → quantify controller match
5. Run `mekf_eval` on new logs → re-check RMSE with outlier rejection from Step 1

---

### Step 7 — Perception: flow outlier gate → better XY RMSE (follows Step 1)

After Step 1 is implemented and a new flight is done, re-run `mekf_eval` on the new logs. Expected improvement: circle XY RMSE should drop from ~15 cm toward ~10 cm because the zero-reading contamination during the hover phase is removed.

---

### Step 8 — (Future / post-course) SLAM scaffold

Not required for course deliverables. Prerequisite: Multi-ranger Deck or AI Deck (camera).

With existing hardware (Flow Deck only):
- The PMW3901 + ToF already provides 2.5D odometry (the MEKF IS the odometry front-end).
- Add a simple particle-filter loop-closure over the MEKF position trace to detect drift — viable with pure dead-reckoning on short (< 30 s) flights.

With AI Deck:
- ORB/FAST feature extraction on-drone (nRF9161 NPU)
- Loop closure via bag-of-words descriptor matching
- Feed loop-closure pose corrections as absolute position updates into MEKF via a new `mekf_update_position(state, pos, r_pos)` measurement update (straightforward — H = [I3 | 0 | 0])

---

## 8. Dependency Graph

```
Step 1 (flow gate) ──────────────────────────────────────────► Step 7 (better RMSE)
                                                                     ↑
Step 2 (circle test) ─┐                                             │
Step 3 (fig8 test)  ──┼──► Step 5 (spline in shadow) ──► Step 6 (new flight) ──► Step 4 (shadow eval)
                       │
                       └──► (course deliverable: spline planner tests done)

Step 4, 6 require real hardware.
Steps 1, 2, 3, 5 are pure code — no hardware needed.
```

---

## 9. Build & Test Commands

```bash
# Full test suite (246 tests, ~3 s)
cargo test

# Build real-flight binary
cargo build --release --bin main

# Run MEKF offline evaluation (all 3 maneuvers)
cargo build --release --bin mekf_eval
./target/release/mekf_eval runs/hover_2026-03-01_13-20-21.csv    > results/hover_mekf.csv
./target/release/mekf_eval runs/circle_2026-03-01_13-59-43.csv   > results/circle_mekf.csv
./target/release/mekf_eval runs/figure8_2026-03-01_14-02-12.csv  > results/figure8_mekf.csv

# Plot MEKF eval results
python3 scripts/plot_mekf_eval.py

# Run only MEKF tests
cargo test --test test_mekf

# Run only spline tests
cargo test --test test_spline
```

---

## 10. Constraints & Non-Negotiables

- **Never replace the firmware's PID/attitude controller** — shadow/parallel only. The firmware handles all safety-critical loops.
- **Validate each layer in isolation** (unit tests + offline CSV replay) before moving to real flight.
- **All new code must pass `cargo test` with 0 failures** before committing.
- **THETA_P = 3.50 is fixed** until a non-firmware ground truth source is available (motion capture / UWB).
- **Pitch and yaw-rate sign conventions are intentional** — see `challenges.md` §3 and §4.

# Architecture

## Overview

A modular Rust library for Crazyflie 2.1 real-hardware autonomous flight, covering the full
perception-to-control stack: MEKF state estimation, SE(3) geometric control, minimum-snap
motion planning, 3D occupancy mapping, visual odometry (VO), loop-closure pose-graph SLAM,
and a frontier-based exploration planner — all in `f32`, validated against real flight logs.

For per-module educational write-ups see the `README.md` inside each source directory.

---

## Full Pipeline Data Flow

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  Sensors (hardware)                                                           │
│  Flow Deck v2: PMW3901 flow [px] + VL53L1x ToF range [m]                   │
│  Multi-ranger Deck: 5× VL53L1x (front/back/left/right/up) [m]               │
│  AI Deck: HiMax HM01B0 camera 324×244, JPEG/TCP over WiFi (CPX protocol)   │
│  IMU: gyro [deg/s] + accel [g]  (onboard, logged via CRTP)                  │
└──────────────────┬──────────────────────────┬──────────────────┬────────────┘
                   │ IMU + flow + range        │ multi-ranger     │ camera frames
                   ▼                           ▼                  ▼
┌──────────────────────────────┐  ┌──────────────────┐  ┌───────────────────┐
│  Estimation — MEKF           │  │  Mapping —       │  │  Perception —     │
│  (estimation/mekf.rs)        │  │  OccupancyMap    │  │  KeyframeStore    │
│  State: p, v_body, q  (9D)   │◄─┤  ray-cast voxels │  │  FAST-9 + BRIEF   │
│  Predict: gyro + accel       │  │  5 cm log-odds   │  │  8-pt E matrix    │
│  Update: height, flow,       │  │  grid            │  │  VO + loop detect │
│          VO position (XY)    │  └────────┬─────────┘  └────────┬──────────┘
└──────────────┬───────────────┘           │frontiers             │ KeyframeResult
               │ position, orientation     ▼                      ▼
               │                ┌──────────────────┐  ┌───────────────────────┐
               │                │  Planning —       │  │  Mapping —            │
               │                │  ExplorationPlan. │  │  VoTrajectory         │
               │                │  SCAN→NAV→LAND    │  │  PoseGraph            │
               │                │  frontier select  │  │  Gauss-Seidel 100×   │
               │                └────────┬──────────┘  └──────────┬────────────┘
               │                         │ ExplorationCommand      │ mekf_update_vo
               │                         │ (GoTo / Hold / Land)    │ (XY correction)
               └─────────────────────────▼─────────────────────────┘
                               ┌────────────────────────────────────┐
                               │  Safety  (safety.rs)               │
                               │  Multi-ranger repulsion            │
                               │  OccupancyMap 8-dir probe          │
                               │  → safe setpoint (x, y, z)        │
                               └─────────────────┬──────────────────┘
                                                 │
                               ┌─────────────────▼──────────────────┐
                               │  main.rs — Real hardware loop       │
                               │  Firmware track:                    │
                               │    setpoint_position → drone PID   │
                               │  Shadow track:                      │
                               │    GeometricController → CSV only  │
                               │  20 Hz logging → 49-column CSV      │
                               └────────────────────────────────────┘
```

---

## Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                             Application Layer                                 │
│                                                                               │
│  main · mekf_eval · slam_eval · build_map · ai_deck_test                     │
│  assignment1..5 · demo · sim_closed_loop                                     │
└──────────────────────────────┬────────────────────────────────────────────────┘
                               │
┌──────────────────────────────▼────────────────────────────────────────────────┐
│                       Flying Drone Stack Library                               │
├────────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐                    │
│  │  math/   │  │ dynamics/│  │ integr./ │  │controller│                    │
│  │  Vec3    │  │  State   │  │  Euler   │  │Geometric │                    │
│  │  Quat    │  │  Params  │  │  RK4     │  │SE(3) Lee │                    │
│  │  Mat9    │  │  Simul.  │  │  Exp     │  │          │                    │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘                    │
│                                                                               │
│  ┌───────────────┐  ┌──────────────────────┐  ┌──────────────────────────┐  │
│  │  planning/    │  │    trajectory/        │  │    estimation/           │  │
│  │  Spline ·     │  │  Figure8 · Circle ·   │  │  MEKF                    │  │
│  │  Flatness ·   │  │  CsvTraj · Takeoff ·  │  │  predict + height +      │  │
│  │  Minimum-snap │  │  Sequenced            │  │  flow + VO updates       │  │
│  │  Exploration  │  │                       │  │  f32, Joseph form        │  │
│  │  FSM          │  │                       │  │  zero-flow gate          │  │
│  └───────────────┘  └──────────────────────┘  └──────────────────────────┘  │
│                                                                               │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │  mapping/                                                              │  │
│  │  OccupancyMap · KeyframeStore · VoTrajectory · PoseGraph               │  │
│  │  SpatialGrid (O(1) loop lookup)                                        │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
│                                                                               │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │  perception/                                                           │  │
│  │  sensors: SimSensor · CrtpAdapter · CpxCamera (AI Deck WiFi)          │  │
│  │  processing: FAST-9 · BRIEF-256 · CameraIntrinsics · ImuCameraSync    │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
│                                                                               │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │  flight/                                                               │  │
│  │  build_state · force_vector_to_rpyt · yaw_rate_cmd · rpm_to_pwm       │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
│                                                                               │
│  safety.rs   SafetyLimits · multi-ranger repulsion · occupancy probe         │
│                                                                               │
└────────────────────────────────────────────────────────────────────────────────┘
```

---

## Module Responsibilities

### `math/` — Mathematical Primitives
→ [Detailed doc](../src/math/README.md)

Pure math, zero external dependencies.

- `vec3.rs`: `Vec3` — 3D position, velocity, force, torque arithmetic
- `quaternion.rs`: `Quat` — unit quaternion, Hamilton product, ZYX Euler, rotate_vector, exp map
- `matrix.rs`: `Mat9` — 9×9 f32 MEKF covariance; Joseph form, symmetrise, clamp_diagonal

---

### `dynamics/` — Physics Core

- `state.rs`: `MultirotorState` (position, velocity, quaternion, angular velocity), `MotorAction`
- `params.rs`: `MultirotorParams` — Crazyflie 2.1 physical parameters (mass, inertia, motor constants)
- `simulator.rs`: `MultirotorSimulator` — wires state + pluggable integrator + rigid-body derivatives

---

### `integration/` — Numerical Integration

Pluggable via `Integrator` trait: `Euler`, `RK4`, `ExpEuler`, `ExpRK4`.

---

### `controller/` — SE(3) Geometric Controller

`GeometricController` (Lee et al. 2010): position PD + feedforward → desired force vector;
rotation error on SO(3) → torque. Output: `ControlOutput { thrust, torque }`.

---

### `trajectory/` — Reference Signal Generators

All implement `Trajectory::get_reference(t) → TrajectoryReference`.

| Type | Description |
|------|-------------|
| `CircleTrajectory` | Horizontal circle, configurable radius / height / ω / centre |
| `Figure8Trajectory` / `SmoothFigure8Trajectory` | Polynomial figure-8 |
| `TakeoffTrajectory` | Minimum-jerk polynomial from ground to target height |
| `SequencedTrajectory` | Chains multiple trajectories with per-phase durations |
| `CsvTrajectory` | Interpolates from a CSV file |

---

### `planning/` — Minimum-Snap Spline + Flatness + Exploration
→ [Detailed doc](../src/planning/README.md)

- `spline.rs`: 8th-order polynomial QP (Clarabel), minimum-snap, continuity up to snap.
- `flatness.rs`: Differential flatness — position/yaw + derivatives → rotation, thrust, ω, τ.
- `exploration.rs`: `ExplorationPlanner` — SCAN→NAVIGATE→LAND frontier FSM. Reads
  `OccupancyMap::frontiers()`, selects nearest qualifying frontier, emits `ExplorationCommand`.

---

### `estimation/` — MEKF State Estimator
→ [Detailed doc](../src/estimation/README.md)

Multiplicative Extended Kalman Filter. State `x ∈ ℝ⁹ = [p(3), b(3), δ(3)]`:
position, body velocity, attitude error angles. `q_ref` quaternion maintained outside filter.

| Update | Sensor | When |
|--------|--------|------|
| `mekf_predict` | gyro + accel | every 50 ms loop |
| `mekf_update_height` | VL53L1x ToF | every loop when range > 0 |
| `mekf_update_flow` | PMW3901 optical flow | every loop, zero-motion gated (< 0.3 px skipped) |
| `mekf_update_vo` | VoTrajectory XY estimate | when new keyframe accepted and sigma < threshold |

Validated RMSE (Mar 17, 2026 flights, shadow in-flight):
- Hover: roll 0.84°, pitch 2.39°, yaw 1.03°
- Circle: roll 0.59°, pitch 1.53°, yaw 3.28°

Key calibration: `THETA_P = 3.50` (empirical; old firmware value 0.717 gave 4.9× too little XY).

---

### `mapping/` — 3D Occupancy Map + Visual Odometry + SLAM
→ [Detailed doc](../src/mapping/README.md)

| Component | File | Purpose |
|-----------|------|---------|
| `OccupancyMap` | `occupancy.rs` | Sparse log-odds voxel grid (5 cm), ray-cast from multi-ranger |
| `KeyframeStore` | `keyframe.rs` | FAST-9+BRIEF keyframes, 8-pt essential matrix VO, spatial-grid loop detection |
| `VoTrajectory` | `vo_trajectory.rs` | Chains `KeyframeResult` relative poses → global XY estimate + drift sigma |
| `PoseGraph` | `loop_closure.rs` | XY pose graph, Gauss-Seidel 100 sweeps, applies loop closure corrections |
| `SpatialGrid` | `keyframe.rs` | O(1) loop candidate lookup via `HashMap<(i32,i32), Vec<usize>>` (cell = 1.5 m) |

**SLAM pipeline** (with `--ai-deck`):
1. Camera frame → `KeyframeStore::push()` (if moved ≥ 0.15 m or ≥ 30° yaw)
2. FAST-9 features → BRIEF-256 descriptors → Hamming matching with ratio test
3. Normalized 8-point algorithm → essential matrix E → `{R, t}` (cheirality), scaled by avg `range_z`
4. `VoTrajectory::integrate()` → world-frame position; chi² gated fusion into MEKF via `mekf_update_vo`
5. `detect_loop()` via spatial grid → `LoopConstraint` → `PoseGraph::add_loop() + optimize()`
6. Corrected XY fused into MEKF with tight noise `R_LOOP = 0.01 m²`

---

### `perception/` — Sensor Abstraction + Feature Extraction
→ [Detailed doc](../src/perception/README.md)

Three layers:
1. **`traits/`**: `SensorSource<M>` (poll), `ImageSource` (next_frame) — hardware-agnostic
2. **`sensors/`**: `SimSensor` (driven from `MultirotorState`), `CrtpAdapter` (unit conversion from
   CRTP log variables), `CpxCamera` (AI Deck JPEG fragment reassembly via WiFi TCP)
3. **`processing/`**: `detect_features()` (FAST-9 + BRIEF-256), `CameraIntrinsics` (project/unproject,
   radial undistortion), `ImuCameraSync` (linear IMU interpolation at query timestamp)

---

### `flight/` — Hardware Bridge

Thin utilities for `main.rs`:

| Function | Purpose |
|----------|---------|
| `build_state` | `MultirotorState` from firmware log row (firmware EKF frame) |
| `force_vector_to_rpyt` | SE(3) force vector → roll/pitch/yaw-rate/thrust CRTP setpoint |
| `yaw_rate_cmd` | Yaw-rate setpoint from attitude error |
| `rpm_to_pwm` / `thrust_to_pwm` | Motor scaling helpers |

**Frame rule**: always use firmware EKF fields (`pos_x/y/z`, `roll/pitch/yaw`) in `build_state`.
Mixing MEKF fields causes ~50 cm offset and full command saturation.

---

### `safety.rs` — Safety Layer

`safety_position()` + `safe_setpoint_omap()` in `main.rs`:

1. **Multi-ranger repulsion**: proportional push away from walls when any ranger < 0.40 m;
   max correction 0.12 m per axis.
2. **Occupancy-map probe**: 8-direction horizontal ring at 0.25 m radius around desired setpoint;
   occupied voxel in any direction cancels motion in that direction.

Active during all maneuver setpoint calls; skipped during ramp-up and landing.

---

## Binaries

| Binary | What it does |
|--------|--------------|
| `main` | Real hardware flight. CLI: `--maneuver <name> [--ai-deck]`. Full SLAM stack. |
| `mekf_eval` | Offline MEKF replay vs firmware EKF. Prints RMSE per axis. |
| `slam_eval` | Analyse VO, loop closures, pose graph from 49-col CSV. |
| `build_map` | Replay CSV → `OccupancyMap` → PLY file. |
| `ai_deck_test` | Bench-test AI Deck camera: receive N frames, optionally save JPEGs. |
| `assignment1..5` | Simulation-only course assignments. |
| `mekf_eval` | Offline MEKF vs flight log with optional parameter overrides. |
| `sim_closed_loop` | Closed-loop simulation (no hardware). |

---

## `main.rs` Flight Sequence

```
1. Parse --maneuver <name> [--ai-deck] from command line
2. Connect to radio://0/80/2M/E7E7E7E7E7
3. If --ai-deck: spawn background thread → CpxCamera TCP connect → KeyframeStore loop
4. Reset Kalman filter, wait for EKF convergence
5. Ramp thrust over ~1.5 s, climb to 0.30 m
6. Stabilise hover 8 s; sample EKF XY origin (cx, cy)
7. Execute maneuver (hover / circle / figure8 / explore):
   Every 50 ms:
     fw_logging_step → read 5 CRTP log blocks
     step_perception! → omap.update() + VO consume/integrate/fuse
     safe_setpoint_omap() → multi-ranger + omap repulsion
     setpoint_position(x, y, z, yaw) → firmware PID
     shadow controller → logged only
     write 49-column CSV row
8. Land; if explore: write PLY map
```

---

## CSV Output: 49 Columns

```
time_ms,
pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
roll, pitch, yaw, thrust, vbat,
gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z,
rate_roll, rate_pitch, rate_yaw,
range_z, flow_dx, flow_dy,
mekf_roll, mekf_pitch, mekf_yaw, mekf_x, mekf_y, mekf_z,
our_ref_x, our_ref_y, our_ref_z,
our_thrust, our_roll_cmd, our_pitch_cmd, our_yaw_rate_cmd,
multi_front, multi_back, multi_left, multi_right, multi_up,
ai_feat_count,
vo_x, vo_y, vo_sigma,
pg_x, pg_y, lc_count
```

- **pos_x/y/z**: Firmware EKF position [m]
- **mekf_x/y/z**: Our MEKF position [m]; **mekf_roll/pitch/yaw**: MEKF attitude [deg]
- **our_***: Shadow controller — reference + commands logged but never sent to drone
- **multi_front/back/left/right/up**: Multi-ranger distances [m]; `NaN` = out of range
- **vo_x, vo_y, vo_sigma**: VO world-frame position [m] and XY drift uncertainty [m]
- **pg_x, pg_y**: Pose-graph corrected XY position [m]; **lc_count**: cumulative loop closures

---

## File Structure

```
flying_drone_stack/
├── Cargo.toml
├── README.md
├── ROADMAP.md
├── VALIDATION_PLAN.md
├── docs/
│   ├── ARCHITECTURE.md          # This file
│   └── DRONE_STACK_ASPECTS.md
├── scripts/
│   ├── plot_flight_diagnostic.py
│   ├── plot_shadow_eval.py
│   └── plot_assignment1..5.py
├── src/
│   ├── lib.rs
│   ├── safety.rs
│   ├── math/          vec3 · quaternion · matrix (Mat9)
│   ├── dynamics/      state · params · simulator
│   ├── integration/   euler · rk4 · exponential
│   ├── controller/    GeometricController (SE(3) Lee 2010)
│   ├── trajectory/    Figure8 · Circle · CsvTraj · Takeoff · Sequenced
│   ├── planning/      SplineTrajectory (QP) · flatness chain · ExplorationPlanner
│   ├── estimation/    mekf.rs (predict · height · flow · VO update)
│   ├── flight/        build_state · force_vector_to_rpyt · yaw_rate_cmd
│   ├── mapping/       occupancy · keyframe · vo_trajectory · loop_closure
│   ├── perception/    sensors/{sim,crtp,cpx} · processing/{features,calibration,sync}
│   └── bin/
│       ├── main.rs              real hardware flight binary
│       ├── mekf_eval.rs
│       ├── slam_eval.rs
│       ├── build_map.rs
│       ├── ai_deck_test.rs
│       ├── assignment1..5.rs
│       └── sim_closed_loop.rs
├── tests/
│   ├── test_mekf.rs
│   ├── test_flight_math.rs
│   ├── test_geometric_controller.rs
│   ├── test_controller_detailed.rs
│   ├── test_flatness.rs
│   ├── test_hover_control_loop.rs
│   ├── test_math.rs
│   ├── test_safety.rs
│   └── test_spline.rs
└── runs/
    └── <maneuver>_<timestamp>.csv    (49 columns)
```

---

## Testing

```bash
cargo test
# Expected: 249 tests, 0 failures
```

Coverage: MEKF (predict/update/flow gate/VO update), flight math, SE(3) controller,
differential flatness, hover control loop, safety limits, spline planner, occupancy map
ray casting + frontiers + PLY, FAST-9/BRIEF features + sorting, CPX fragment reassembly,
camera calibration project/unproject, IMU sync, VO trajectory chain + sigma + reset,
loop closure age/spatial gates, pose graph Gauss-Seidel convergence, spatial grid index
O(1) lookup + boundary coverage, exploration FSM transitions.

---

## Design Principles

1. **Separation of concerns** — each module has one responsibility; math ≠ physics ≠ estimation.
2. **Dependency inversion** — `Simulator` depends on `Integrator` trait, not concrete types.
3. **Testability** — every component independently testable; no global state.
4. **Frame discipline** — firmware EKF frame for CRTP commands; MEKF output clearly labelled.
5. **Graceful degradation** — AI Deck on background thread; failure non-fatal to flight.

---

## Component Interaction Example — Main Loop

```
every 50 ms:
  fw_logging_step:
    read CRTP log blocks (EKF, IMU, flow, multi-ranger, vbat)
    mekf.predict(gyro, accel, dt)
    mekf.update_height(range_z)
    mekf.update_flow(flow_dx, flow_dy, range_z)
    shadow_controller.compute(state, reference) → logged to CSV

  step_perception! macro:
    omap.update(pos, attitude, multi_ranger_readings)
    if ai_pose_changed:
      vo_traj.integrate(kf_result) → vo_pos
      chi²-gated mekf_update_vo(vo_pos, r_vo)
      vo_traj.reset_to(mekf_pos)
    if loop_constraint:
      pose_graph.add_loop(lc)
      pose_graph.optimize() → corrected XY
      mekf_update_vo(pg_xy, R_LOOP)

  safe_setpoint_omap(desired_setpoint, omap, multi_ranger)
    → repulsion-adjusted setpoint

  setpoint_position(safe_x, safe_y, safe_z, yaw)  ← sent to drone
  write 49-column CSV row
```

---

## Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| `crazyflie-lib` | 0.4 | CRTP protocol — logging, parameters, commander |
| `crazyflie-link` | 0.3 | Radio/USB link layer |
| `tokio` | 1 | Async runtime for concurrent log stream reading |
| `chrono` | 0.4 | UTC timestamps for CSV filenames |
| `simple_qp` | — | QP solver (Clarabel) for minimum-snap spline planner |
| `jpeg-decoder` | — | JPEG decoding for AI Deck frames |
| `serde` | 1 | Config deserialisation (toml) |

# Flying Drone Stack — Roadmap

> Last updated: 2026-05-01
> Tests: 264 lib + 195 integration = **459 passing, 0 failures**
> Build: `cargo build --release` clean

---

## What is built (all phases complete)

### Core simulation and control
| Component | Module | What it does |
|-----------|--------|--------------|
| Dynamics & simulation | `dynamics/`, `integration/` | SE(3) rigid-body simulator; Euler, RK4, ExpEuler, ExpRK4 |
| SE(3) geometric controller | `controller/` | Lee et al. 2010; RPYT pipeline; validated in sim + real flights |
| Min-snap spline planner | `planning/spline.rs`, `flatness.rs` | Degree-8 QP (Clarabel), C3-continuous; differential flatness → thrust/torque/ω/ω̇ |
| Richter auto-time planner | `planning/richter.rs` | Mode 1 wrapper: same QP + automatic T_i from single `k_t` scalar; pre-flight feasibility check |
| Frontier exploration FSM | `planning/exploration.rs` | SCAN→NAVIGATE→LAND; bugs fixed Apr 7, awaiting re-flight |

### State estimation and SLAM
| Component | Module | What it does |
|-----------|--------|--------------|
| MEKF | `estimation/mekf.rs` | Mueller 2015, fusing IMU + ToF + flow + VO; RMSE hover roll 0.84°/pitch 2.39° |
| 3D occupancy map | `mapping/occupancy.rs` | Sparse log-odds 5 cm voxels, multi-ranger ray-cast |
| Visual odometry | `mapping/keyframe.rs`, `vo_trajectory.rs` | FAST-9 + BRIEF, 8-point essential matrix, metric scale from range_z |
| Loop closure + pose graph | `mapping/loop_closure.rs` | Gauss-Seidel 100 sweeps; validated: 12 closures, up to 0.24 m correction |
| Spatial grid index | `mapping/keyframe.rs` `SpatialGrid` | O(1) loop candidate lookup (3×3 neighbourhood, 1.5 m cells) |

### Onboard firmware controller
| Component | Path | What it does |
|-----------|------|--------------|
| Out-of-tree SE(3) controller | `firmware_app/` | `no_std` Rust, `bindgen` FFI, `controlModeForceTorque` at 500 Hz on STM32 |
| Full-state CRTP setpoint | `vendor/crazyflie-lib/` | `setpoint_full_state` (type 6): pos+vel+acc+quat+ω in 29-byte packet |
| INDI controller (3-mode) | `firmware_app/src/lib.rs`, `src/controller/indi.rs` | `CONTROLLER_MODE` 0/1/2 = Geometric / Attitude INDI / Full INDI; **all fully implemented**, mode 0 active |

### Trajectory flight modes
| Mode | What it does |
|------|-------------|
| `main.rs` maneuvers (position setpoints) | hover/circle/figure8/explore at 20 Hz, MEKF+SLAM+safety, CSV logging |
| Mode B Rust binaries (`*_test`) | `hover/helix/flip/roll/figure8/circle` + fast variants — spline→flatness→CRTP full-state at 25 Hz |
| Richter Mode 1 binary | `richter_figure8_test` — same figure-8 waypoints, duration from `k_t`; prints feasibility report before flight |
| HLC Python scripts (`Controls/`) | `run_*.py` — same 10 maneuver variants; Poly4D upload → HLC `start_trajectory` at 100 Hz |

### Course assignments ✅ all complete
| Assignment | Status |
|-----------|--------|
| A1 — Dynamics & integrators | ✅ 3 scenarios: synthetic, convergence, real-flight k-step (165 windows, 13→63 mm) |
| A2 — SE(3) controller | ✅ Simulation + PC-side real flight (70.8 mm RMS 3D) + firmware onboard controller |
| A3 — MEKF | ✅ Real flight validated (hover/circle/figure-8 vs firmware EKF) |
| A4 — Min-snap + flatness | ✅ Simulation: open-loop 321 mm, closed-loop 2.2 mm RMS |

**CSV format:** 49 columns — `time_ms … acc_x/y/z … mekf_* … our_* … vo_x/y/vo_sigma … pg_x/pg_y/lc_count`

---

## Stage 0 — Firmware + Full-State Flight Tests (immediate priority)

These two flight modes are implemented and ready but need first real-hardware validation:

### 0a. Onboard SE(3) controller (firmware_app)
```bash
cd firmware_app && make cload   # flash OOT firmware
cargo run --release --bin main -- --maneuver circle
```
**Check:** drone tracks circle with noticeably tighter attitude (controller runs at 500 Hz vs 20 Hz RPYT mode).

### 0b. Rust live full-state spline (CRTP type 6)
```bash
cargo run --release --bin spline_circle_test   # requires firmware flashed above
```
**Check:** terminal shows `[log]` lines with non-zero ctrltarget vel/acc; tracking smoother than position-only.

### 0c. HLC Python circle
```bash
~/.pyenv/versions/flying_robots/bin/python Controls/run_spline_circle.py
```
**Check:** drone executes 10.47 s circle lap; radio not needed after `start_trajectory`.

---

## Stage 1 — SLAM Validation (after Stage 0)

These are sequential; each informs the next.

### 1a. Basic flight with full stack
```bash
cargo run --release --bin main -- --maneuver circle --ai-deck
```
**Check in CSV:**
- `mekf_roll/pitch/yaw` track `roll/pitch/yaw` (firmware EKF) within ≈ 3–4°
- `our_roll_cmd / our_pitch_cmd` are finite, non-zero during circle
- `ai_feat_count` column > 0 (camera is streaming features)

### 1b. Verify VO is producing keyframes
Look for `[KF]` lines in terminal output during flight.
**Check in CSV:**
- `vo_x / vo_y` are non-zero after the first keyframe pair
- `vo_sigma` grows slowly (< 0.5 m after 30 s of circle)

### 1c. Verify loop closures fire
```bash
cargo run --release --bin slam_eval -- runs/circle_<date>.csv
```
**Check:**
- At least 1 loop closure event printed
- `lc_count` column increments in the CSV
- `pg_x / pg_y` shift after each correction (pose graph is running)

### 1d. Visual sanity check
```bash
python3 scripts/plot_flight_diagnostic.py   # picks latest CSV automatically
```
Overlay firmware EKF XY vs MEKF XY vs pose-graph XY on one plot.

**Go/no-go for Stage 2:** VO keyframes firing ✓, ≥ 1 loop closure on a circle ✓, pose graph corrects drift ✓

---

## Stage 2 — Tuning (after Stage 1 passes)

| Item | Why | What to adjust |
|------|-----|----------------|
| Loop closure sensitivity | Might be too tight or too loose on real texture | `LOOP_MIN_MATCHES`, `LOOP_MIN_INLIERS`, `LOOP_SEARCH_RADIUS_M` in `keyframe.rs` |
| VO drift rate | `vo_sigma` growth tells you how fast VO drifts | `r_vo_xy` in `mekf_update_vo` call in `main.rs` |
| Keyframe spacing | `[KF]` lines every 0.3 m — may need to loosen/tighten | `KF_MIN_DIST_M`, `KF_MIN_YAW_DEG` in `keyframe.rs` |
| Exploration safety margins | Reactive avoidance may be too conservative or too loose | `HORIZ_THRESH_M`, `MAX_CORRECTION_M` in `main.rs` |

---

## Stage 3 — If time allows (post-validation improvements)

Ordered roughly by value vs effort:

### 3a. Global path planning on occupancy map (medium effort)
Replace the reactive SCAN→NAVIGATE FSM with A* on the occupancy map.
**Files:** `src/planning/exploration.rs`, new `src/planning/astar.rs`
**Value:** drone revisits fewer dead ends, faster coverage.

### 3b. Map persistence across sessions (low effort)
Save pose graph + occupancy map PLY on landing; reload on next boot.
**Files:** `main.rs` land handler, new `load_map()` function.
**Value:** multi-session mapping without redoing the whole room.

### 3c. Denser occupancy from camera depth (high effort)
Use essential-matrix baseline + range_z to triangulate sparse 3D points per keyframe.
Insert into `OccupancyMap` as additional occupied cells.
**Value:** richer map than multi-ranger alone (5-beam vs N feature points).

### 3d. Appearance-based loop closure (high effort)
Replace spatial proximity gating with a bag-of-words descriptor index (e.g. DBoW-style).
Allows loop detection when revisiting places from a different direction.
**Value:** more robust SLAM; required if flights are longer than the current room.

---

## When to fly vs when to code

| Situation | Recommendation |
|-----------|----------------|
| Any doubt about a new algorithm | Offline CSV replay first (`slam_eval`, `mekf_eval`) |
| Stage 1 not yet done | **Fly now** — no amount of coding replaces this |
| Tuning constants | Fly, collect CSV, tune offline, fly again |
| Stage 3 features | Implement + unit test first; fly only to validate |

**Rule of thumb:** if the change touches `main.rs` control/safety paths → fly to validate within the same session.  If it touches only `mapping/` or `estimation/` → offline replay is enough to check correctness.

---

## Quick-reference commands

```bash
# Build & test
cargo test                                          # 452 tests (258 lib + 194 integration)
cargo build --release                               # clean build

# Flash onboard controller (required for spline test binaries)
# Mode 0 = Geometric (default), Mode 1 = Attitude INDI, Mode 2 = Full INDI (RPM deck)
# Change CONTROLLER_MODE in firmware_app/src/lib.rs, then:
cd firmware_app && make cload

# Fly — position setpoints (20 Hz, full SLAM stack)
cargo run --release --bin main -- --maneuver circle --ai-deck
cargo run --release --bin main -- --maneuver explore --ai-deck

# Fly — Rust live full-state Mode B (25 Hz, requires firmware flashed above)
cargo run --release --bin hover_test
cargo run --release --bin indi_hover               # INDI hover validation (30 s, logs to runs/)
cargo run --release --bin spline_circle_test
cargo run --release --bin spline_figure8_test      # Mode 0: manual durations (7.28 s/rep)
cargo run --release --bin richter_figure8_test     # Mode 1: auto durations from k_t=0.3 (default)
cargo run --release --bin richter_figure8_test -- --kt 0.5 --reps 3  # faster / more reps
cargo run --release --bin helix_test        # and fast_helix_test
cargo run --release --bin flip_test         # and fast_flip_test
cargo run --release --bin roll_test         # and fast_roll_test
cargo run --release --bin spline_figure8_test  # and fast_figure8_test

# Fly — HLC Python Mode C (10 maneuver variants)
~/.pyenv/versions/flying_robots/bin/python Controls/run_hover.py
~/.pyenv/versions/flying_robots/bin/python Controls/run_circle.py
~/.pyenv/versions/flying_robots/bin/python Controls/run_helix.py
~/.pyenv/versions/flying_robots/bin/python Controls/run_flip.py
# ... same pattern for fast_* variants
~/.pyenv/versions/flying_robots/bin/python Controls/run_spline_circle.py  # HLC Poly4D

# Assignment simulations
cargo run --release --bin assignment1    # A1: integrators + real-flight k-step
cargo run --release --bin assignment2    # A2: geometric control sim
cargo run --release --bin assignment4    # A4: min-snap + flatness sim

# Offline analysis
cargo run --release --bin slam_eval -- runs/<file>.csv
cargo run --release --bin mekf_eval -- runs/<file>.csv
~/.pyenv/versions/flying_robots/bin/python scripts/plot_flight_diagnostic.py

# View saved map
# PLY files written to runs/ on explore landing
meshlab runs/explore_map_<timestamp>.ply
```

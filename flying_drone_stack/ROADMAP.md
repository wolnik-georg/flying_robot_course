# Flying Drone Stack — Roadmap

> Last updated: 2026-03-21
> Tests: 249 passing, 0 failures
> Build: `cargo build --release` clean

---

## What is built (all phases complete)

| Phase | Module | What it does |
|-------|--------|--------------|
| Dynamics & control | `dynamics/`, `controller/` | SE(3) geometric controller, RPYT pipeline |
| Planning | `planning/spline.rs`, `flatness.rs`, `exploration.rs` | Min-snap spline, flatness, SCAN→NAVIGATE→LAND FSM |
| MEKF estimation | `estimation/mekf.rs` | Mueller 2015 MEKF fusing flow + rangefinder; validated offline (roll 0.84°, pitch 2.39°, yaw 1.03° RMSE on real flights) |
| 3-D occupancy map | `mapping/occupancy.rs` | Sparse log-odds voxel map (5 cm), ray-cast from multi-ranger |
| Safety | `safety.rs`, `main.rs` | Multi-ranger repulsion + occupancy-map probe |
| Visual odometry | `mapping/keyframe.rs`, `vo_trajectory.rs` | FAST-9 + BRIEF, 8-point essential matrix, metric scale from range_z; VoTrajectory chains relative poses |
| VO–MEKF fusion | `estimation/mekf.rs` `mekf_update_vo` | Two scalar EKF updates for XY; reset after loop correction |
| Loop closure / pose graph | `mapping/loop_closure.rs` | LoopConstraint, Gauss-Seidel pose graph (100 sweeps) |
| Spatial grid index | `mapping/keyframe.rs` `SpatialGrid` | O(1) loop candidate lookup (3×3 cell neighbourhood, 1.5 m cells) |

**CSV format:** 49 columns — `time_ms … acc_x/y/z … mekf_* … our_* … vo_x/y/vo_sigma … pg_x/pg_y/lc_count`

---

## Stage 1 — Validation (do this before anything else)

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
cargo test                                          # 249 tests
cargo build --release                               # clean build

# Fly
cargo run --release --bin main -- --maneuver circle --ai-deck
cargo run --release --bin main -- --maneuver explore --ai-deck

# Offline analysis
cargo run --release --bin slam_eval -- runs/<file>.csv
cargo run --release --bin mekf_eval -- runs/<file>.csv
python3 scripts/plot_flight_diagnostic.py           # latest CSV

# View saved map
# PLY files written to runs/ on explore landing
```

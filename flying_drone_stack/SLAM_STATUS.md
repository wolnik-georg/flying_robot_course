# SLAM & Perception — Status Document

> Last updated: 2026-04-07
> Purpose: Session-closing snapshot. Use this to resume or build on top of the SLAM work.

---

## 1. What Was Built — Component Checklist

| Phase | Component | Location | Status | Flight-validated |
|-------|-----------|----------|--------|-----------------|
| 1 | 3D occupancy map (sparse log-odds, 5 cm voxels, ray-cast) | `src/mapping/occupancy.rs` | ✅ Complete | ✅ Yes |
| 2 | Safety layer (multi-ranger repulsion + omap probe) | `src/safety.rs`, `src/bin/main.rs` | ✅ Complete | ✅ Yes |
| 3 | Frontier-based exploration planner (SCAN→NAVIGATE→LAND) | `src/planning/exploration.rs` | ✅ Complete (see §3) | ⚠️ Partial |
| 4 | Camera keyframes (FAST-9 + BRIEF + 8-pt essential matrix) | `src/mapping/keyframe.rs` | ✅ Complete | ✅ Yes |
| 5 | Visual odometry → MEKF fusion | `src/mapping/vo_trajectory.rs`, `src/estimation/mekf.rs` | ✅ Complete | ✅ Yes |
| 6 | Loop closure + pose graph SLAM (Gauss-Seidel) | `src/mapping/loop_closure.rs` | ✅ Complete | ✅ Yes |
| 8 | Spatial grid index for O(1) loop detection | `src/mapping/keyframe.rs` (`SpatialGrid`) | ✅ Complete | ✅ Yes (via unit tests + real flights) |
| — | `vo_sigma` fix (`reset_to` no longer clears sigma) | `src/mapping/vo_trajectory.rs` | ✅ Fixed | ✅ Yes |

**Summary:** All SLAM components are implemented, tested (249 tests, 0 failures), and validated on real hardware — except the end-to-end **explore** flight (Phase 3), which has two bugs that were fixed but not yet re-flown.

---

## 2. What Still Needs Work

### 2a. Explore flight — NOT yet re-flight-tested after bug fixes

Two bugs were found and fixed during the April 7 session:

**Bug 1 — `Hold` yaw was silently discarded** (`src/bin/main.rs`):
```rust
// Before (broken): ignored yaw_deg from planner
ExplorationCommand::Hold { x, y, z, .. } => {
    cf.commander.setpoint_position(sx, sy, sz, yaw_now).await?;
}
// After (fixed): uses planner's commanded yaw
ExplorationCommand::Hold { x, y, z, yaw_deg } => {
    cf.commander.setpoint_position(sx, sy, sz, yaw_deg).await?;
}
```

**Bug 2 — SCAN commanded incremental yaw (3°/step), not cumulative** (`src/planning/exploration.rs`):
```rust
// Before: target = current + 3° → tiny error → drone barely rotates
let target_yaw = yaw_deg + delta;

// After: target = start_yaw + accumulated → grows each step → drone actually rotates
let sy = *start_yaw.get_or_insert(yaw_deg);
let target_yaw = sy + *yaw_accumulated;
```

Both are fixed and tests pass. **The explore flight needs one clean run to confirm end-to-end.**

```bash
cargo run --release --bin main -- --maneuver explore --ai-deck
# expect: visible 360° SCAN rotation, then drone navigates to frontier(s), then lands cleanly
```

### 2b. Future enhancements (not started)

| Enhancement | Effort | Notes |
|-------------|--------|-------|
| Multi-height exploration (e.g., 0.3 m then 0.6 m) | Low–medium | Exploration FSM already stores 3D; add second altitude pass |
| Camera depth → occupancy map | High | Monocular only; would use VO translation as sparse depth proxy |
| A* path planning on omap | Medium | Replace straight-line GoTo with obstacle-aware path |
| Map persistence between sessions | Low | Serialize/deserialize omap HashMap to disk |

---

## 2c. Three Planned Extensions for Advanced Flying Robots Project

These three are independent of SLAM and can be pursued in any order.

### A. Lighthouse positioning (strongly recommended for home use)

Replaces optical flow XY estimation with sub-centimetre absolute positioning — no drift.
Essential for aggressive manoeuvre experiments (INDI etc.) where XY drift makes position
control unreliable.

**Hardware needed:**
- Lighthouse Deck (Bitcraze, ~€35)
- One SteamVR Base Station 2.0 (~€150 standalone) — covers ~4 m × 4 m room
- Two base stations for better coverage and redundancy

**Software impact (minimal):**
- Zero code changes needed. The Crazyflie firmware Kalman filter natively fuses Lighthouse.
- `pos_x/y/z` in the CSV become accurate absolute positions automatically.
- MEKF attitude estimation unchanged; SLAM/VO still useful for mapping but no longer
  needed for localisation.

---

### B. INDI controller for aggressive manoeuvres

Add an Incremental Nonlinear Dynamic Inversion (INDI) controller as a new `Controller`
implementation. INDI works primarily on angular rates + accelerations from the IMU (already
available at 500 Hz) rather than on a dynamics model — making it robust to model
uncertainty and ideal for aggressive flight.

**How to add to the stack:**
1. `src/controller/indi.rs` — implement the `Controller` trait
2. `src/controller/mod.rs` — add `pub mod indi;`
3. `src/lib.rs` — export `IndiController`
4. `src/bin/main.rs` — add `"indi_hover"` / `"indi_circle"` match arm

**Validate safely using the shadow track** (already built):
- Run INDI in shadow mode first: `our_roll_cmd` / `our_thrust` logged in CSV, motors untouched
- Compare against SE(3) geometric controller output column-by-column
- Only enable live control once shadow outputs look correct

**Lighthouse is a prerequisite** for INDI at home — flow-based XY drift corrupts the
position loop, making INDI experiments impossible to evaluate cleanly.

---

### C. Onboard computation (companion computer)

Currently the full stack (MEKF, SLAM, control loop) runs on the laptop and sends 20 Hz
setpoints to the Crazyflie via radio. Moving computation onboard would:
- Eliminate ~50 ms radio round-trip latency (crucial for INDI inner loop at 100–500 Hz)
- Enable truly autonomous flight without laptop tether
- Allow faster control loops (currently limited to 20 Hz by CRTP logging throughput)

**Recommended approach:**
- **Raspberry Pi Zero 2W** (€18) connected via USB to the Crazyflie (USB link replaces radio)
- Compile the Rust stack for `aarch64-unknown-linux-gnu` (cross-compile from laptop)
- Replace `crazyflie-link` radio URI with USB URI: `usb://0`
- Control loop can run at 100+ Hz over USB vs 20 Hz over radio

**What changes in the stack:**
- `Cargo.toml`: no dependency changes needed
- `main.rs`: change the link URI from `radio://...` to `usb://...`
- Everything else — Controller, MEKF, SLAM, CSV logging — unchanged
- GAP8 (AI Deck) continues to stream JPEG over WiFi to the RPi instead of the laptop

**Note:** The GAP8 already does JPEG encoding onboard. Feature extraction (FAST-9) could
also be moved to GAP8 in the future, further reducing WiFi bandwidth needs.

---

## 3. Documentation Checklist

| Item | Status |
|------|--------|
| `src/math/README.md` | ✅ |
| `src/dynamics/README.md` | ✅ |
| `src/integration/README.md` | ✅ |
| `src/controller/README.md` | ✅ |
| `src/trajectory/README.md` | ✅ |
| `src/planning/README.md` | ✅ |
| `src/estimation/README.md` | ✅ |
| `src/flight/README.md` | ✅ |
| `src/perception/README.md` | ✅ |
| `src/mapping/README.md` | ✅ (includes spatial grid, loop closure, VO, pose graph, correct `reset_to` semantics) |
| Top-level `README.md` | ✅ (49-col CSV, all binaries, test count, flight procedure) |
| `ROADMAP.md` | ✅ |
| `VALIDATION_PLAN.md` | ✅ (Tier 0–7) |
| `SLAM_STATUS.md` | ✅ (this file) |
| Inline source doc comments | ✅ (all modules have `//!` module-level comments) |
| 249 unit + integration tests | ✅ (0 failures) |

---

## 4. Validated Flight Results

| Flight CSV | What was confirmed |
|-----------|-------------------|
| `circle_2026-03-30_18-51-03.csv` | 9 KFs, 12 loop closures, pose-graph corrections up to 0.24 m, vo_sigma 0.033→0.273, circle std 0.054 m |
| `circle_2026-03-30_18-45-11.csv` | vo_sigma fix confirmed; 10 KFs; 0 LCs (low texture environment) |
| Various `circle_2026-03-*` | MEKF validated: hover roll 0.84°, pitch 2.39°, yaw 1.03° RMSE |
| `explore_2026-04-07_*` | Both explore attempts failed (bugs above); bugs now fixed |

**SLAM validation tiers passed:** 0, 1, 2, 3, 4, 5, 6 ✅
**Tier 7 (explore end-to-end):** ⚠️ Bugs fixed, awaiting one successful flight

---

## 5. Stack Architecture for Reuse / Extension

The flying drone stack is a **Rust library + binary** structure. All intelligence lives in
the library (`multirotor_simulator`); `main.rs` is just the orchestrator.

```
flying_drone_stack/
├── src/
│   ├── lib.rs                  ← exports all modules cleanly
│   ├── math/                   ← Vec3, Quat, Mat9
│   ├── dynamics/               ← MultirotorState, simulator
│   ├── integration/            ← Euler, RK4, Exp variants
│   ├── controller/             ← Controller trait + SE(3) geometric
│   ├── trajectory/             ← Trajectory trait + Circle, Figure8, …
│   ├── planning/               ← spline, flatness, ExplorationPlanner
│   ├── estimation/             ← MEKF
│   ├── flight/                 ← state builder, RPYT pipeline
│   ├── perception/             ← sensors, FAST-9, BRIEF, CPX
│   ├── mapping/                ← OccupancyMap, KeyframeStore, SLAM
│   └── safety.rs
└── src/bin/
    └── main.rs                 ← orchestrates everything for real flight
```

### Key traits for extension

**`Controller` trait** (`src/controller/mod.rs`):
```rust
pub trait Controller {
    fn compute(&mut self, state: &MultirotorState, reference: &TrajectoryReference)
        -> ControlOutput;
}
```
To add an INDI controller: implement this trait in `src/controller/indi.rs`, add
`pub mod indi;` to `src/controller/mod.rs`, export from `lib.rs`, use in `main.rs`.

**`Trajectory` trait** (`src/trajectory/mod.rs`):
```rust
pub trait Trajectory {
    fn reference(&self, t: f32) -> TrajectoryReference;
}
```
Any new trajectory (spline, aggressive maneuver, etc.) just implements this.

### How `main.rs` is structured

Four clearly separated phases every control cycle (20 Hz):

```
1. fw_logging_step()       ← read all sensor streams from Crazyflie, update MEKF, log CSV row
2. step_perception!()      ← update omap, drain AI keyframe, fuse VO into MEKF
3. planner.step()          ← compute next setpoint (maneuver-specific)
4. cf.commander.setpoint_position() ← send to firmware
```

To add a new maneuver (e.g., aggressive flip, INDI-controlled circle):
- Add an arm to the `match maneuver.as_str()` block in `main.rs`
- Call your new controller/planner in step 3
- Everything else (sensors, MEKF, SLAM, safety, logging) stays unchanged

### Adding INDI controller for Advanced Flying Robots

Concrete steps:

1. **Create** `src/controller/indi.rs` — implement `Controller` trait
2. **Export** from `src/controller/mod.rs` and `src/lib.rs`
3. **In `main.rs`** — add `--maneuver indi_circle` arm, instantiate `IndiController`, call in the loop
4. **Shadow track** — the existing shadow controller pattern already logs your controller output to CSV without touching motors; use this to validate INDI before going live

The CSV format (49 columns) already includes `our_thrust`, `our_roll_cmd`, `our_pitch_cmd`,
`our_yaw_rate_cmd` for shadow controller output — these will capture INDI commands for post-flight analysis.

---

## 6. Quick-Reference Commands

```bash
# Build & test
cd flying_drone_stack
cargo build --release
cargo test                          # 249 tests, 0 failures

# Validate explore (the remaining open item)
cargo run --release --bin main -- --maneuver explore --ai-deck

# SLAM analysis
cargo run --release --bin slam_eval -- runs/<file>.csv
cargo run --release --bin build_map -- runs/<file>.csv
meshlab results/data/explore_map_<timestamp>.ply

# Post-flight diagnostic
~/.pyenv/versions/flying_robots/bin/python scripts/plot_flight_diagnostic.py

# INDI development workflow (future)
# 1. Implement Controller trait in src/controller/indi.rs
# 2. Add --maneuver indi_hover arm to main.rs (shadow mode first)
# 3. cargo run --release --bin main -- --maneuver indi_hover
# 4. Check our_roll_cmd / our_thrust in CSV before enabling live control
```

---

## 7. Hardware Notes

| Hardware | Status | Notes |
|----------|--------|-------|
| Crazyflie 2.1 | ✅ Working | Radio URI: `radio://0/80/2M/E7E7E7E7E7` |
| Flow Deck v2 | ✅ Working | Needs textured floor; MEKF validated |
| Multi-ranger Deck | ✅ Working | Required for safety + omap |
| AI Deck (GAP8 + Nina) | ⚠️ Best-effort | Nina reboots after 13–70 s; SLAM survives gracefully via reconnect loop |
| AI Deck firmware | ✅ Flashed | wifi-img-streamer with all stability fixes + 600 ms throttle (~1.5 fps) |
| Nina firmware | ✅ Flashed | 2025.02 (65 KB TCP TX buffer — the root-cause fix for Nina reboots) |

**Pre-flight checklist:**
- [ ] Battery ≥ 3.7 V
- [ ] Textured surface under drone + textured walls for camera features
- [ ] Crazyradio PA plugged in
- [ ] Laptop WiFi → "WiFi streaming example" AP, `ping 192.168.4.1` OK
- [ ] Clear 1.5 m × 1.5 m area

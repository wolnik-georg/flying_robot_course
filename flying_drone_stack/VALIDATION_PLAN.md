# Component Validation Plan

> Last updated: 2026-03-21
> Purpose: Validate each architectural layer in isolation before end-to-end testing.
> Order is strictly bottom-up — each tier depends on the one before passing.

---

## Tier 0 — Pre-flight hardware check (no flight, desk only)

**Goal:** confirm all hardware is connected and all log streams are alive before risking a flight.

**Steps:**
```bash
cargo run --release --bin main -- --maneuver hover
# Let it sit on the desk (do NOT arm/fly). Ctrl-C after 5 s.
```

**Pass criteria:**

| Check | How to verify |
|-------|---------------|
| Radio connects | Terminal prints `Connected! Maneuver: hover` |
| All log blocks streaming | Terminal prints log lines every 50 ms (no `timeout` errors) |
| Battery OK | `vbat` column in CSV > 3.80 V |
| IMU alive | `gyro_x/y/z`, `acc_x/y/z` non-NaN and non-zero |
| Flow deck alive | `flow_dx`, `flow_dy` present (zeros on desk is normal) |
| Range sensor alive | `range_z` > 0 (reading desk surface, ~0.05–0.15 m) |
| Multi-ranger alive | At least one of `multi_front/back/left/right/up` non-zero |

**Stop if:** any column is NaN every row, or radio fails to connect.

---

## Tier 1 — MEKF + core sensors (hover, 30 s)

**Goal:** confirm MEKF attitude and height estimation are correct during stable hover.
No camera. Shortest possible flight.

```bash
cargo run --release --bin main -- --maneuver hover
```

**What to check in CSV** (`plot_flight_diagnostic.py` or manual):

| Column(s) | Expected | Pass threshold |
|-----------|----------|----------------|
| `range_z` | ≈ HOVER_HEIGHT (0.25 m) | 0.20–0.35 m during steady hover |
| `mekf_z` vs `range_z` | Track each other | RMSE < 3 cm |
| `mekf_roll` vs `roll` | Track firmware attitude | RMSE < 4° |
| `mekf_pitch` vs `pitch` | Track firmware attitude (note opposite sign convention) | RMSE < 4° |
| `mekf_yaw` vs `yaw` | Track firmware yaw | RMSE < 5° |
| `flow_dx`, `flow_dy` | Non-zero when drone drifts; zero when stationary | Any non-zero sample during flight |
| `acc_x`, `acc_y`, `acc_z` | `acc_z` ≈ −1.0 g in hover | `|acc_z + 1.0| < 0.1` |

**Offline replay to cross-check MEKF:**
```bash
cargo run --release --bin mekf_eval -- runs/hover_<date>.csv
```
Expected output: roll/pitch RMSE < 4°, Z RMSE < 3 cm.

**Stop if:** MEKF Z drifts > 10 cm from range_z, or attitude RMSE > 8°. Check `theta_p` and `r_height` params.

---

## Tier 2 — Multi-ranger + safety repulsion (hover near wall, 30 s)

**Goal:** confirm multi-ranger is reading distances correctly and safety layer pushes the drone away from obstacles.

**Setup before running:** physically place the drone on the floor ≈ 25–30 cm from a wall (facing it), then launch.
The hover maneuver sends a fixed position setpoint — it will not drift toward the wall on its own, so the wall must already be within range at takeoff.

```bash
cargo run --release --bin main -- --maneuver hover
```

**What to check in CSV:**

| Column(s) | Expected |
|-----------|----------|
| `multi_front` | Decreases smoothly as drone approaches wall; < 0.5 m when close |
| `multi_back/left/right` | Non-zero when corresponding wall is nearby |
| `multi_up` | Non-zero if ceiling is within range (< 2 m) |
| `our_ref_x/y/z` | Setpoint shifts away from wall when `multi_front` < 0.40 m (safety repulsion active) |

**Key check:** plot `multi_front` vs `our_ref_x` — when `multi_front` drops below 0.40 m the reference should step backward. If the setpoint does not move, safety repulsion is not triggering.

**Stop if:** any ranger reads 0.0 consistently while the drone is clearly within 1 m of a surface (sensor fault or deck not enabled in firmware params).

---

## Tier 3 — Shadow controller correctness (circle, 60 s)

**Goal:** confirm our geometric controller produces physically sensible commands that match what the firmware actually does.
No camera. Pure control validation.

```bash
cargo run --release --bin main -- --maneuver circle
```

**What to check in CSV:**

| Column(s) | Expected |
|-----------|----------|
| `our_roll_cmd`, `our_pitch_cmd` | Oscillate at circle frequency, amplitude ≈ 5–15° |
| `our_yaw_rate_cmd` | Near-zero for a fixed-heading circle |
| `our_thrust` | Finite, near hover thrust |
| Shadow vs firmware attitude | `our_roll_cmd` shape matches `roll` with small lag |
| `our_ref_x/y/z` | Traces a circle in XY plane |

**Offline analysis:**
```bash
python3 scripts/plot_shadow_eval.py runs/circle_<date>.csv
```
Expected: roll/pitch command vs actual RMSE < 8°. Shape match (correlation > 0.8) confirms the controller understands the maneuver geometry.

**Stop if:** `our_roll_cmd` / `our_pitch_cmd` are identically zero the entire flight — shadow controller not seeding, check `mekf_seeded` path in `main.rs`.

---

## Tier 4 — AI Deck camera streaming (hover, 30 s)

**Goal:** confirm camera frames are arriving, feature detection is running, and the background thread is stable.

```bash
cargo run --release --bin main -- --maneuver hover --ai-deck
```

**What to check — terminal:**
- `AI Deck connected` printed before flight starts
- No `AI Deck disconnected` or reconnect messages during flight
- `[FEAT]` or feature-count log lines appear every few seconds

**What to check in CSV:**

| Column | Expected |
|--------|----------|
| `ai_feat_count` | > 10 per frame on a textured floor; > 0 on any surface |

**Note:** keyframes will NOT trigger during hover (drone doesn't move > 0.3 m). That is expected — this tier only validates the camera pipeline is alive.

**Stop if:** `ai_feat_count` is 0 for the entire flight. Check WiFi connection to AI Deck AP ("WiFi streaming example"), wait 15 s after power-on, verify JPEG firmware is flashed.

---

## Tier 5 — Visual odometry (circle with camera, 2 laps)

**Goal:** confirm keyframes trigger at the right rate, feature matching works, and VO produces a plausible XY trajectory estimate.

```bash
cargo run --release --bin main -- --maneuver circle --ai-deck
```

**What to check — terminal:**
- `[KF]` lines appear (≥ 1 per 0.3 m of travel ≈ every 1–2 s at typical circle speed)
- No `estimate_essential: degenerate` or panic messages

**What to check in CSV:**

| Column(s) | Expected |
|-----------|----------|
| `vo_x`, `vo_y` | Non-zero after 2nd keyframe; traces rough circle shape |
| `vo_sigma` | Grows slowly (< 1.0 m after one full circle) |
| `ai_feat_count` | Consistently > 0 during circle |

**Offline check:**
```bash
cargo run --release --bin slam_eval -- runs/circle_<date>.csv
```
At this tier just check that keyframe events are printed — loop closures are not expected yet (only 1 lap).

**Quick CSV spot-check (vo columns not plotted by plot_flight_diagnostic.py):**
```bash
python3 -c "
import pandas as pd
df = pd.read_csv('runs/circle_<date>.csv')
print('vo non-zero rows:', (df['vo_x'].abs() > 0.001).sum())
print('vo_sigma max:', df['vo_sigma'].max())
print('ai_feat_count mean:', df['ai_feat_count'].mean())
"
```

**Stop if:** `vo_x/vo_y` remain 0.0 the entire flight. Check that `[KF]` lines appear in terminal — if no keyframes trigger, the drone may be flying too fast (increase circle period) or the floor has insufficient texture.

---

## Tier 6 — Loop closure + pose graph (circle, 3+ laps)

**Goal:** confirm that loop closures fire when the drone revisits the start of the circle, and that the pose graph corrects accumulated VO drift.

The default circle runs 30 s at ω = 0.6 rad/s → period ≈ 10.5 s → **≈ 2.85 laps**.
Loop closure can fire after the first return to start (~10.5 s in). No code change needed.

```bash
cargo run --release --bin main -- --maneuver circle --ai-deck
```

**What to check — terminal:**
- `[LC]` or loop closure lines printed after completing the first full lap
- Pose graph update messages

**What to check in CSV:**

| Column(s) | Expected |
|-----------|----------|
| `lc_count` | Increments (≥ 1 per lap after the first) |
| `pg_x`, `pg_y` | Shift at loop closure moments (discontinuous jumps) |
| `pg_x/pg_y` vs `mekf_x/mekf_y` | After correction, `pg_*` closer to true circle path |

**Quick CSV spot-check (pg/lc columns not plotted by plot_flight_diagnostic.py):**
```bash
python3 -c "
import pandas as pd
df = pd.read_csv('runs/circle_<date>.csv')
print('max lc_count:', df['lc_count'].max())
print('loop closure rows:', (df['lc_count'].diff() > 0).sum())
print('pg non-zero rows:', (df['pg_x'].abs() > 0.001).sum())
"
```

**Offline analysis:**
```bash
cargo run --release --bin slam_eval -- runs/circle_<date>.csv
```
Expected: ≥ 1 loop closure event logged per lap, with `from_idx` / `to_idx` ≥ `LOOP_MIN_AGE` apart.

**Stop if:** `lc_count` stays 0 after 3 laps. Likely causes:
1. Too few features (floor texture) → lower `LOOP_MIN_MATCHES` tentatively
2. Poses too far apart (drift) → check `vo_sigma` — if > 1.5 m, the spatial gate never fires
3. `LOOP_MIN_INLIERS` too tight for real images → lower from 8 to 6 and retest

---

## Tier 7 — Full end-to-end: autonomous exploration (5–10 min)

**Goal:** exercise the entire stack together — MEKF + multi-ranger + VO + loop closure + pose graph + exploration planner — in a single untethered flight.

```bash
cargo run --release --bin main -- --maneuver explore --ai-deck
```

**What to check — terminal:**
- FSM transitions: `[EXPLORE] SCAN` → `[EXPLORE] NAVIGATE` → `[EXPLORE] LAND`
- `[KF]` keyframes throughout flight
- `[LC]` loop closures on revisited areas
- `[LAND]` message and clean shutdown

**What to check post-flight:**
- CSV written to `runs/explore_<date>.csv`
- PLY map file written to `runs/`
- `lc_count` > 0
- Drone did not hit any walls (multi-ranger safety held)

**Offline:**
```bash
python3 scripts/plot_flight_diagnostic.py   # latest CSV
# Open PLY in MeshLab or CloudCompare — should show recognisable room outline
```

**Pass criteria:**

| Criterion | Pass |
|-----------|------|
| Safe flight | No wall collision, multi-ranger repulsion worked |
| Exploration coverage | At least 3 distinct NAVIGATE targets reached |
| SLAM running | `lc_count` > 0, `vo_sigma` < 2.0 m at landing |
| Map quality | PLY file non-empty; occupied voxels trace room perimeter |

---

## Stopping rules summary

| Tier | Hard stop condition |
|------|---------------------|
| 0 | Any sensor column all-NaN or radio fails to connect |
| 1 | MEKF Z RMSE > 10 cm or attitude RMSE > 8° |
| 2 | Multi-ranger reads 0.0 while within 1 m of wall |
| 3 | Shadow commands identically 0 the whole flight |
| 4 | `ai_feat_count` = 0 entire flight |
| 5 | `vo_x/vo_y` = 0 entire flight (no keyframes) |
| 6 | `lc_count` = 0 after 3 laps |
| 7 | Drone contacts wall / safety failure |

Do not advance to the next tier until the current tier passes. Each failure points to a specific, isolated subsystem.

---

## Quick command reference

```bash
# Tiers 1–3 (no camera)
cargo run --release --bin main -- --maneuver hover
cargo run --release --bin main -- --maneuver circle

# Tiers 4–7 (with camera)
cargo run --release --bin main -- --maneuver hover   --ai-deck
cargo run --release --bin main -- --maneuver circle  --ai-deck
cargo run --release --bin main -- --maneuver explore --ai-deck

# Offline analysis
cargo run --release --bin mekf_eval  -- runs/<file>.csv
cargo run --release --bin slam_eval  -- runs/<file>.csv
python3 scripts/plot_flight_diagnostic.py
python3 scripts/plot_shadow_eval.py runs/<file>.csv
```

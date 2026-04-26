# src/bin/ — Executable Binaries

## Real hardware flight binaries

### main.rs — Mode A (position setpoints, 20 Hz, full stack)

The primary flight binary. Structure:
```
1. Connect + reset Kalman, sample EKF origin
2. Ramp up to hover height
3. Hover (stabilize MEKF)
4. Execute maneuver (CLI arg)
5. Land
6. Write CSV to runs/<maneuver>_<timestamp>.csv
```

**CLI**: `cargo run --release --bin main -- --maneuver <name> [--ai-deck]`

Maneuvers: `hover`, `circle`, `figure8`, `explore`, `my_hover`, `my_circle`, `my_figure8`
- `hover/circle/figure8` → `run_firmware_mode()` — position setpoints, full SLAM + CSV
- `my_*` → RPYT setpoints (100 Hz, velocity PI), no SLAM
- `explore` → ExplorationPlanner FSM, requires `--ai-deck` for loop closure

**Adding a new maneuver**: add a `match` arm in `main.rs` around the maneuver dispatch block.
Everything else (sensors, MEKF, SLAM, CSV, safety) stays unchanged.

**Control loop structure** every 20 Hz tick:
```rust
fw_logging_step()        // read all sensor streams, update MEKF, log CSV row
step_perception!()       // update omap, drain AI keyframe, fuse VO
planner.step()           // compute next setpoint
cf.commander.setpoint_position(sx, sy, sz, yaw).await?   // send to firmware
```

---

### Mode B flight binaries (full-state setpoints, 25 Hz)

All require OOT firmware: `cd firmware_app && make cload`

Structure for all: connect → Kalman reset → sample EKF origin → ramp → hover → **trajectory** → hold → land.

| Binary | Maneuver | Duration | Notes |
|--------|----------|----------|-------|
| `spline_circle_test` | Circle r=0.30m | 10.47s/lap | Min-snap spline → flatness |
| `spline_figure8_test` | Figure-8 | 7.28s | Min-snap spline → flatness |
| `fullstate_circle_test` | Circle | analytic | `CircleTrajectory`, no spline |
| `fullstate_figure8_test` | Figure-8 | analytic | `Figure8Trajectory`, no spline |
| `hover_test` | Hover | indefinite | Full-state hover at HOVER_HEIGHT |
| `helix_test` | Ascending+descending helix | 2×10.5s | Analytic flatness, no spline |
| `fast_helix_test` | Helix 2× speed | 2×5.25s | LAP_TIME=5.25s |
| `flip_test` | 360° backflip | T=0.70s | QP angle polynomial, FLIP_HEIGHT=1.0m |
| `fast_flip_test` | Backflip 2× speed | T=0.45s | FLIP_HEIGHT=1.3m |
| `roll_test` | 360° roll | T=0.70s | QP angle polynomial |
| `fast_roll_test` | Roll 2× speed | T=0.45s | FLIP_HEIGHT=1.3m |
| `figure8_test` | Figure-8 | 7.28s | Spline variant |
| `fast_figure8_test` | Figure-8 2× speed | 3.64s | QP re-solved with halved durations |

Trajectory loop pattern (spline-based binaries):
```rust
let flat = traj.eval(t % traj.total_time);
let res = compute_flatness(&flat, 0.031);       // 0.031 kg = CF + decks
let q = rot_to_quat(&res.rot);                  // [w, x, y, z]
cf.commander.setpoint_full_state(
    res.pos.x, res.pos.y, res.pos.z,
    res.vel.x, res.vel.y, res.vel.z,
    flat.acc.x, flat.acc.y, flat.acc.z,
    q[0], q[1], q[2], q[3],
    res.omega.x, res.omega.y, res.omega.z,
).await?;
sleep(Duration::from_millis(40)).await;
```

---

### export_poly4d.rs — Mode C prep

Generates Poly4D coefficients for HLC Python scripts.
```bash
cargo run --release --bin export_poly4d                # circle
cargo run --release --bin export_poly4d figure8_match  # figure-8 (QP re-plan of prof's waypoints)
cargo run --release --bin export_poly4d fast_figure8   # figure-8 2× speed (separate QP solve)
cargo run --release --bin export_poly4d flip           # flip angle profile
cargo run --release --bin export_poly4d fast_flip      # flip at T=0.45s
# stdout: Python list; stderr: validation (endpoint gap, velocity)
```
Paste stdout into the corresponding `Controls/run_*.py` script.

---

## Offline analysis binaries

### mekf_eval.rs

```bash
cargo run --release --bin mekf_eval -- runs/<file>.csv
```
- Reads 49-column CSV
- Replays MEKF (predict + height + flow + zero-motion gate)
- Prints RMSE vs firmware EKF columns
- **CRITICAL**: loop starts from `seed_idx` (first row where `range_z > 0.1`), NOT row 0

### slam_eval.rs

```bash
cargo run --release --bin slam_eval -- runs/<file>.csv
```
- Reads 49-column CSV (needs all columns including `pg_x`, `pg_y`, `lc_count`)
- Replays VO trajectory, loop closures, pose graph
- Prints: total KFs, loop closure events, max pose graph correction, final vo_sigma

### build_map.rs

```bash
cargo run --release --bin build_map -- runs/<file>.csv [runs/file2.csv ...]
# writes results/data/map.ply
meshlab results/data/map.ply
```

### ai_deck_test.rs

```bash
cargo run --release --bin ai_deck_test -- --frames 50 --save-all
# saves results/data/frame_NNNN.jpg
ffplay -f image2 -framerate 3 results/data/frame_%04d.jpg
```
Requires: laptop WiFi connected to "WiFi streaming example" AP, drone powered ≥ 15s.

---

## Assignment binaries

### assignment1.rs

Three scenarios: (1) synthetic spin — integrator drift comparison, (2) convergence vs dt,
(3) real-flight k-step prediction (requires `runs/circle_2026-03-17_19-35-31.csv`).

```bash
cargo run --release --bin assignment1              # default CSV
cargo run --release --bin assignment1 -- --csv runs/other.csv   # custom CSV
```
Writes CSVs to `results/assignment1/data/`, plots via `scripts/plot_assignment1.py`.

### assignment2.rs

SE(3) geometric controller in simulation: step response, circle, figure-8. Tunes gains.
No real hardware needed.

### assignment3.rs

MEKF simulation + offline validation against fr00.csv.
```bash
cargo run --release --bin assignment3 -- --csv "../State Estimation/logging_ekf/logging/fr00.csv"
```

### assignment4.rs

Min-snap spline + flatness, open-loop and closed-loop simulation.
Three CSVs: reference, open-loop, closed-loop. Six plots.

### assignment5.rs

Safe-space sim with MEKF: hover, circle, figure-8 modes.
```bash
cargo run --release --bin assignment5 -- circle
```

---

## CSV written by main.rs (49 columns)

```
time_ms  pos_x pos_y pos_z  vel_x vel_y vel_z
roll pitch yaw  thrust  vbat
gyro_x gyro_y gyro_z  acc_x acc_y acc_z
rate_roll rate_pitch rate_yaw
range_z  flow_dx flow_dy
mekf_roll mekf_pitch mekf_yaw  mekf_x mekf_y mekf_z
our_ref_x our_ref_y our_ref_z
our_thrust our_roll_cmd our_pitch_cmd our_yaw_rate_cmd
multi_front multi_back multi_left multi_right multi_up
ai_feat_count  vo_x vo_y vo_sigma  pg_x pg_y lc_count
```

Units: time [ms], pos/vel [m, m/s], angles [deg], gyro [deg/s], acc [g], range [m], flow [px].
`mekf_roll/pitch/yaw` in deg. `mekf_pitch` sign is opposite to `stabilizer.pitch`.

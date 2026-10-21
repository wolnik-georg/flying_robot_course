# Flying Drone Stack — Claude Context

## ⚠️ BRANCH PROTECTION — READ FIRST

**NEVER commit to, merge into, or modify these branches — they are locked baselines:**
- `stable-crazyflie-standard` — standard CF2.1 working state
- `stable-jun30-working` — clean state before upgraded/INDI work

**Active working branches (all new work goes here):**
| Platform | Branch | Build command |
|----------|--------|---------------|
| Thrust-upgraded CF2.1 | **`main`** | `cd firmware_app && make DRONE=upgrade cload` |
| Brushless CF21BL | **`main`** | `cd firmware_app && make cload` (DRONE=bl is default) |
| Standard CF2.1 (reference only) | `stable-crazyflie-standard` | read-only — do not commit |

**As of 2026-08-22 everything is merged to `main` in both repos and 13 stale branches were
deleted.** Older notes pointing at `brushless-port` or `tune-indi-on-crazyflie-thrust-upgraded`
are wrong; those labels are gone. `brushless-port` survives only as a frozen snapshot.

The 3-drone Makefile switch (`DRONE=bl|upgrade|std`) is on `main`. Switching drone re-seeds the firmware `.config` automatically via stamp file.

## What this project is

A complete Rust autonomous flight stack for Crazyflie 2.1. Single crate `multirotor_simulator`
(library + binaries) plus a separate `firmware_app/` crate (no_std onboard controller).

## Quick commands

```bash
cargo build --release          # build everything
cargo test                     # 276 tests, 0 failures expected
cargo test <name>              # run one test by name substring

# Motion planning simulation (no hardware)
cargo run --release --bin planning_sim -- --export-reference-only   # reference plots only
cargo run --release --bin planning_sim                              # full closed-loop sim
scripts/plot_planning_all.sh

# Assignment simulations (no hardware)
cargo run --release --bin assignment1
cargo run --release --bin assignment2
cargo run --release --bin assignment4

# ── Mode E — CS2 + OptiTrack (CURRENT STANDARD) ──────────────────────────
# Step 1: export trajectory CSV (writes to crazyswarm2/crazyflie_examples/data/)
cargo run --release --bin export_poly4d -- --trajectory figure8 --mode 0
cargo run --release --bin export_poly4d -- --trajectory figure8 --mode 1 --kt 0.008
cargo run --release --bin export_poly4d -- --trajectory circle  --mode 1 --kt 0.1

# Step 2: fly via CS2 (in crazyswarm2 workspace, after colcon build)
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 0
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.008 --reps 3

# Post-flight analysis (CS2 logs save to Controls/logs/ — same as Rust)
~/.pyenv/versions/flying_robots/bin/python Controls/analyze_flight.py   # auto-picks latest

# ── Mode D — optical flow + onboard firmware scripts (predecessor) ────────
# (kept for reference; use Mode E with mocap for all new flights)

# Firmware build + flash (required for both Mode D and Mode E)
cd firmware_app && make        # build cf2.bin
cd firmware_app && make cload  # flash via Crazyradio (controller=6 geometric SE3)

# Offline analysis
cargo run --release --bin mekf_eval -- runs/<file>.csv
cargo run --release --bin slam_eval -- runs/<file>.csv
~/.pyenv/versions/flying_robots/bin/python scripts/plot_flight_diagnostic.py
```

## Module map

```
src/
├── math/          Vec3, Quat, Mat9 — all Copy, all f32
├── dynamics/      MultirotorState, MultirotorParams, MotorAction, MultirotorSimulator
├── integration/   Euler, RK4, ExpEuler, ExpRK4 — all impl Integrator trait
├── controller/    Controller trait, GeometricController (Lee SE(3)), IndiController (Tal & Karaman 2020),
│                  Butterworth2, IirFilter, TrajectoryReference, ControlOutput
├── trajectory/    Trajectory trait, Circle/Figure8/Csv/Sequenced/Takeoff trajectories
├── planning/      TrajectoryPlanner (Mode 0/1/2/3), SplineTrajectory, RichterTrajectory, Se3Trajectory,
│                  Waypoint, Se3Waypoint, FlatOutput, Se3Output, FlatnessResult,
│                  compute_flatness, rot_to_quat, flatness_to_reference
├── estimation/    MekfState, MekfParams, mekf_predict, mekf_update_{height,flow,vo}
├── flight/        build_state, force_vector_to_rpyt, thrust_to_pwm, yaw_rate_cmd
├── perception/    ImageFrame, FAST-9, BRIEF, CPX camera (AI Deck)  ← NOT ACTIVE, leave as-is
├── mapping/       OccupancyMap, KeyframeStore, VoTrajectory, PoseGraph, LoopConstraint  ← NOT ACTIVE, leave as-is
├── safety.rs      safe_setpoint, safe_setpoint_omap (multi-ranger repulsion + omap probe)
└── bin/
    ├── main.rs                   real hardware flight (20 Hz, Mode A)
    ├── spline_circle_test.rs     Mode B: spline→flatness→full-state 25 Hz
    ├── spline_figure8_test.rs    Mode B: Mode 0 figure-8 (manual durations)
    ├── richter_figure8_test.rs   Mode B: Mode 1 figure-8 (auto durations from k_t; --kt 0.3)
    ├── fullstate_circle_test.rs  Mode B: analytic circle
    ├── fullstate_figure8_test.rs Mode B: analytic figure-8
    ├── export_poly4d.rs          export min-snap spline as Poly4D for HLC Python
    ├── mekf_eval.rs              offline MEKF replay, prints RMSE
    ├── slam_eval.rs              offline VO/loop closure/pose graph analysis
    ├── build_map.rs              replay CSV → PLY occupancy map
    ├── ai_deck_test.rs           AI Deck bench test (no flight)
    ├── planning_sim.rs           planning simulation: all modes × trajectories × controllers
    ├── assignment1..5.rs         course assignment simulations
    └── sim_closed_loop.rs
firmware_app/
    src/lib.rs          no_std controller at 500 Hz (separate crate, thumbv7em)
                        ctrl_mode 0 = geometric SE(3), 1 = position INDI, 2 = attitude INDI,
                        3 = full INDI -- selected at runtime by `indi_gains.ctrl_mode`
    src/residual_nn.rs  learned residual model -- Neural-Swarm2 port (19297 weights), onboard
                        NEVER modify this file; RAM is handled by the `residual_nn` Cargo
                        feature (default OFF) gating the static, not by changing the network
    traj_iface.c        param/log groups + the C bridges (RPM, peers, log sinks)
    host/               host build for the simulator + the numerical tests
tools/
    residual/           residual training pipeline (PyTorch -> firmware weights)
    README_usd_thesis_logging.md  THE ordered uSD workflow -- read this before flying
    check_usd_deck.py   read-only deck/card readiness check (never touches usd.logging)
    copy_usd_log.py     safe, sha256-verified, wall-clock-named extraction from a card
    find_flight_window.py  locate the real flight inside a recording by correlating
                        against the scenario's own commanded trajectory
    merge_usd_logs.py   per-drone uSD logs -> one time-aligned CSV (use --meta/--roles)
    decode_usd_log.py   binary uSD log -> numpy dict
```

## Key types — cheat sheet

```rust
// Math
Vec3::new(x, y, z)    // Copy — pass by value
Quat::new(w, x, y, z) // Copy — [w, x, y, z] convention

// Dynamics
MultirotorState { position, velocity, orientation, angular_velocity }
MultirotorParams::crazyflie()   // 27g defaults
MotorAction::from_thrust_torque(thrust_n, torque_vec3, &params)

// Control
TrajectoryReference { position, velocity, acceleration, jerk, yaw, yaw_rate, yaw_acceleration }
ControlOutput { thrust: f32, torque: Vec3 }
GeometricController::default()            // tuned Crazyflie gains (simulation)
IndiController::default_crazyflie()       // Tal & Karaman INDI (simulation, τ_prev-based)
// INDI inner gains: kr [1/s²] (ωₙ=√kr), kw [1/s] (ζ=kw/(2ωₙ)); outer loop identical to geometric
// Firmware: the control law is chosen at RUNTIME by `indi_gains.ctrl_mode`
// (0 geometric, 1 pos INDI, 2 att INDI, 3 full INDI) from crazyflies.yaml -- no reflash.

// Planning — four modes, one interface (see src/planning/CLAUDE.md for full docs)
// Mode 0: SplineTrajectory   — manual durations, min-snap QP (Mellinger 2012)
// Mode 1: RichterTrajectory  — automatic timing from k_t scalar, 0 redistribution iters
// Mode 2: Se3Trajectory      — SE(3) explicit attitude on SO(3), supports flips/loops/inversion
//          SplineTrajectory::plan_with_accel_pins() — NEW: acceleration-pinned flatness for
//          trajectories where attitude should emerge automatically (e.g. teardrop vertical loop).
//          Pin a_z=-1.5g at apex → body_z=(0,0,-1) at top without authoring any quaternion.
//          Requires CF Brushless (max omega ~34 rad/s, thrust ~0.77 N; CF2.1 insufficient).
// Mode 3: RichterTrajectory  — same as Mode 1 + 8 redistribution iters (full Richter 2016 paper)

// One-constant mode switch:
const PLANNER_MODE: u8 = 1;
let planner = match PLANNER_MODE {
    1 => TrajectoryPlanner::richter(&wps, k_t, periodic).unwrap(),
    2 => TrajectoryPlanner::se3(&se3_wps, &durs, mass, periodic).unwrap(),
    3 => TrajectoryPlanner::paper(&wps, k_t, periodic).unwrap(),
    _ => TrajectoryPlanner::spline(&wps, &durs, periodic).unwrap(),
};
let flat  = planner.eval(t);           // FlatOutput — identical for all modes
let total = planner.total_time();
let rep   = planner.check_feasibility(mass, max_thrust, max_omega); // None for Mode 0; Some for 1/2/3

// Mode 0
let traj = SplineTrajectory::plan(&waypoints, &durations, periodic)?;
let flat: FlatOutput = traj.eval(t);

// Mode 1  (k_t: 0.01→0.65 m/s, 0.1→1.0 m/s, 1.0→2.0 m/s, 10→5.2 m/s)
let traj = RichterTrajectory::plan(&waypoints, k_t, periodic)?;
// Dense waypoints (arc < 0.12 m) → use plan_from_times to avoid T_MIN clamping:
let traj = RichterTrajectory::plan_from_times(&wps, &manual_durs, k_t, periodic, 0)?;

// Mode 2 — use Se3Waypoint::flat() for normal flight, explicit quat for flip/loop
let wps: Vec<Se3Waypoint> = pos_wps.iter().map(|w| Se3Waypoint::flat(w.pos, w.yaw)).collect();
let traj = Se3Trajectory::plan(&wps, &durs, mass_kg, periodic)?;
let out: Se3Output = traj.eval(t);  // rot, omega, alpha, thrust (can be negative)

// Differential flatness (all modes)
let res: FlatnessResult = compute_flatness(&flat, mass);
let q: [f32; 4] = rot_to_quat(&res.rot); // [w,x,y,z]
let ref_ = flatness_to_reference(&flat);  // → TrajectoryReference for GeometricController

// MEKF
mekf_predict(&mut state, gyro_rads, accel_ms2, dt, &proc_noise);
mekf_update_height(&mut state, pz_m, r_height);
mekf_update_flow(&mut state, Some([fx, fy]), Some(pz), theta_p, r_flow, zero_thresh);
mekf_update_vo(&mut state, [vo_x, vo_y], r_vo_xy);

// Mapping
omap.update(pos, roll, pitch, yaw, front, back, left, right, up, down);
kf_store.push(frame, pos, yaw_deg, range_z)  // → Option<KeyframeResult>
vo_traj.push(translation, rotation);         // chain relative poses
pose_graph.optimize(100);                    // Gauss-Seidel 100 sweeps
```

## Flight mode evolution — current standard is Mode E (CS2 + OptiTrack)

| Mode | Execution | Setpoint | Hz | Status | Notes |
|------|-----------|----------|----|--------|-------|
| A | `main.rs` | `setpoint_position` (x,y,z,yaw) | 20 | reference | Full SLAM + CSV; firmware position PID; optical flow |
| B | `spline_*_test`, `richter_figure8_test` | `setpoint_full_state` (type 6) | 25 | predecessor | Rust evaluates spline live, sends full-state over radio |
| C | Python HLC (`Controls/run_*.py`) | Poly4D upload + `start_trajectory` | ~100 | predecessor | Python uploads Poly4D, drone evaluates onboard; optical flow |
| D | `onboard_*.rs` | 9-coeff poly uploaded once via CRTP params | 500 | predecessor | Firmware evaluates at 500 Hz; optical flow; best optical-flow tracking |
| **E** | **CS2 `flight.py`** | **8-coeff Poly4D via HLC** | **1000** | **current standard** | **OptiTrack mocap → no drift; geometric SE3 controller=6; same motion planning** |

**Why Mode E supersedes Mode D**: OptiTrack gives globally consistent pose with no drift, enabling repeatable long flights. The HLC evaluates at 1000 Hz (vs 500 Hz for Mode D). The 9→8 coefficient conversion (Hermite boundary match) preserves C3 continuity — tracking quality is equivalent.

**Mode E architecture**:
```
Rust motion planning (unchanged) → export_poly4d → Poly4D CSV
OptiTrack → motion_capture_tracking → CS2 server → send_extpose → drone EKF
CS2 flight.py → upload_trajectory + startTrajectory → HLC evaluates at 1000 Hz
geometric SE(3) controller=6 → motors
Logs → Controls/logs/{trajectory}_mode{N}_{timestamp}.csv (same format as Mode D)
```

Mode B packet: `commander.setpoint_full_state(px,py,pz, vx,vy,vz, ax,ay,az, qw,qx,qy,qz, wx,wy,wz)`
Implemented in `vendor/crazyflie-lib/src/subsystems/commander.rs`.

### Mode D planning modes (0/1/2) and firmware upload protocol

All three planning modes share the same Mode D firmware path (`traj.mode=1`). The difference is which coefficient buffers are uploaded and how the firmware derives attitude.

**Position buffer** (all modes, 19 floats/segment):
```
[duration, cx0..cx8, cy0..cy8]   — uploaded via traj.ci/cv/cw
```

**Z-axis buffer** (3D trajectories — loop, helix; 9 floats/segment):
```
[cz0..cz8]   — uploaded via traj.zci/zcv/zcw
```
`traj.z_mode=0` (default): `z = traj.hz + traj.dz * lap_frac` (flat/helix linear ramp).
`traj.z_mode=1`: `z = traj.hz + poly_eval(cz)` — full 3D trajectory (loop, any 3D path).

**Attitude buffer** (planning Mode 2 only, 18 floats/segment):
```
[croll0..croll8, cpitch0..cpitch8]   — uploaded via traj.aci/acv/acw
```
`traj.att_mode=0` (default): firmware derives attitude via flatness — **Modes 0 and 1** always use this; the attitude coefficient buffer is not uploaded.
`traj.att_mode=1`: firmware evaluates uploaded roll/pitch polynomials at 500 Hz (`traj.aci` / `acv` / `acw`). **Required** where differential flatness breaks (zero thrust). **All onboard Mode 2 flights in this repo** (`onboard_circle`, `onboard_figure8`, `onboard_helix`, `onboard_loop`, …) set `att_mode=1` and call `upload_att_trajectory` whenever `--mode 2` — including level paths. Waypoints may be “flat” (`Se3Waypoint::flat`, thrust never crosses zero); attitude is still **pre-planned in Rust** as part of `Se3Trajectory`, fitted to roll/pitch segments, and **evaluated on the MCU from those polynomials** instead of re-running flatness there.

All three modes plan **3D** piecewise polynomials (`cx`, `cy`, `cz` on the Rust side). The split is **segment timing** (manual vs `k_t`) and **whether the laptop uploads attitude** (never for 0/1; always for 2 in our onboard scripts). **`z_mode`** only chooses how **Z** is read (`ramp` vs `cz` poly), not planning mode.

| Mode | Planning mode | Position | Z | Attitude (onboard scripts in this repo) |
|------|-----------------|----------|---|----------|
| **0** | Spline (manual segment times) | 3D poly — `cx`/`cy` always; `cz` for spatial paths — uploaded via `traj.ci` / `cv` / `cw` | Full `z` from `cz` when `traj.z_mode=1`; else `hz + dz·lap_frac` (`z_mode=0`) | `att_mode=0`: flatness from position + yaw |
| **1** | Richter (auto timing from `k_t`) | Same coefficient layout after solve | Same as Mode 0 | Same as Mode 0 |
| **2** | SE(3) (quaternion waypoints in the planner) | Same position buffer as 0/1 | Same as Mode 0 | **`att_mode=1` always** for `--mode 2`: `upload_att_trajectory` for **every** maneuver (circle, figure-8, helix, loop, …). Roll/pitch polys come from the Se3 solve (level quats or explicit flip/loop quats); the firmware does **not** switch back to flatness for “easy” trajectories. |

`verify_firmware()` in `flight_common.rs` checks for `traj.z_mode` parameter presence — old firmware without 3D support fails with a clear message.

## CSV format (49 columns)

```
time_ms, pos_x/y/z [m], vel_x/y/z [m/s],
roll/pitch/yaw [deg], thrust, vbat,
gyro_x/y/z [deg/s], acc_x/y/z [g],
rate_roll/pitch/yaw, range_z [m], flow_dx/dy [px],
mekf_roll/pitch/yaw [deg], mekf_x/y/z [m],
our_ref_x/y/z, our_thrust, our_roll/pitch_cmd, our_yaw_rate_cmd,
multi_front/back/left/right/up [m],
ai_feat_count, vo_x, vo_y, vo_sigma,
pg_x, pg_y, lc_count
```

**Pitch sign**: `stabilizer.pitch` (firmware) and `mekf_pitch` have opposite signs — negate
`mekf_pitch` in plot scripts for visual comparison.

## Coding conventions

- All arithmetic in `f32`; no `f64` anywhere in flight code.
- `Vec3` and `Quat` are `Copy` — prefer value semantics, no needless `.clone()`.
- Rotation matrices are `[[f32; 3]; 3]` row-major. Column `[:][2]` is the body z-axis.
- Quaternion convention: `[w, x, y, z]` with w scalar first (math module);
  firmware uses scalar-last (`q3 = qw`) — see `firmware_app/CLAUDE.md`.
- All planner constructors take `periodic: bool` as an explicit argument — never auto-detect.
- `mekf_update_flow` takes `Option<[f32;2]>` — pass `None` to skip update without changing code.
- Never call `f32::sqrt()` in `firmware_app/` — use `libm::sqrtf()`.
- Add new maneuvers by adding a `match` arm in `src/bin/main.rs`; everything else unchanged.

## Key gotchas

- **MEKF seed row**: `mekf_eval` must start from the first row where `range_z > 0.1`, NOT row 0.
- **Flow axis swap**: PMW3901 convention — `vel_x ↔ -flow_dy`, `vel_y ↔ -flow_dx`.
- **theta_p calibration**: 3.50 for March 2026 flights; 0.717 for course dataset `fr00.csv`.
- **Arming threshold** (firmware): controller outputs zero until `setpoint.position.z > 0.05 m`.
- **Gyro in degrees**: `sensors.gyro` in firmware is deg/s — convert with `* π/180`.
- **NEVER broadcast `usec.reset` mid-flight** — the high-level commander's whole time base is that
  clock (`crtp_commander_high_level.c`), with `t_begin` stored from it at takeoff. Zeroing it in the
  air makes `piecewise_eval` evaluate a degree-7 polynomial hundreds of seconds outside its domain
  and the vehicle drops or tumbles within one tick. It goes **on the ground, before takeoff** (fixed
  2026-09-15 in all three flight scripts).
- **uSD: a new `thesisNN` file per `usd.logging` 0→1**, and `f_close` only on 1→0. A **0-byte file
  is a session that never stopped cleanly** (crash/power loss/card pulled). The counter is neither a
  timeline nor a drone identity — cards get moved between vehicles.
- **Aligning two drones' uSD logs**: use each drone's own commanded trajectory
  (`merge_usd_logs.py --meta --roles`), never cross-drone z-correlation — on a flat-altitude
  scenario that was off by an order of magnitude (+262 ms at corr 0.39 vs a true ~20 ms).
- **Setpoint yaw**: `setpoint.attitude.yaw` in firmware is degrees.
- **OOT firmware required** for Modes B and C: `cd firmware_app && make cload`.
- **AI Deck**: Nina reboots after 13–70s (hardware limitation); stack reconnects automatically.
- **Clarabel sequential limit**: QP solver degrades after ~10–15 sequential solves per process. In `planning_sim.rs` Mode 1 is built first (reusing durations for Mode 2), then Mode 0 last. Never batch >10 QP calls.
- **Richter T_MIN = 0.15 s**: segments where `dist/v_avg < 0.15 s` clamp to T_MIN → `1/T^7 ≈ 2.2e7` → degenerate QP. Use `richter_from_times` / `plan_from_times` for dense waypoints (helix n=24).
- **Se3Trajectory periodic**: must be set explicitly — helix starts/ends at same XY but needs `false` (different z and velocity); auto-detect caused QP divergence in the old implementation.

## Hardware

| Component | Status | Notes |
|-----------|--------|-------|
| Crazyflie 2.1 | ✅ | URI: `radio://0/80/2M/E7E7E7E7E7` |
| Flow Deck v2 | ✅ | Needs textured floor for XY |
| Multi-ranger Deck | ✅ | Required for safety + omap |
| AI Deck (GAP8+Nina) | ⚠️ | Reboots after 13–70s; SLAM survives |

## Tests

```bash
cargo test                    # run all 276
cargo test mekf               # filter by name
cargo test -- --nocapture     # show println! output
```

Test files: `tests/test_mekf.rs`, `test_flatness.rs`, `test_spline.rs`,
`test_geometric_controller.rs`, `test_safety.rs`, `test_math.rs`, `test_flight_math.rs`,
`test_hover_control_loop.rs`, `test_controller_detailed.rs`, `test_indi_closed_loop.rs`.

---

## Where things stand (2026-09-15) — read `docs/07` for the authoritative version

The status of this project lives in
`~/Desktop/flying_robot_course/docs/07_Thesis_Progress_Checklist.md`, **not here**. This section
exists so you do not act on a stale assumption while working in this directory.

> **★ Core Thesis Workflow** (master plan in `docs/07`): **C.0** Hardware Gate (mostly cleared) →
> **C.1** Residual Data Collection ⬅️ **next** → **C.2** Train the Residual Model → **C.3**
> Integrate the Strategies → **C.4** Systematic Comparison.
>
> **2026-09-15: the uSD logging chain is validated end to end and the first usable two-drone
> dataset exists** (A8, both drones, 1.9/1.8 cm RMS vs commanded, ~20–30 ms cross-drone sync,
> `a_res` live). Logging is no longer the blocker for C.1. The one controller question still open
> is whether our own **full INDI** flies A8 cleanly with 2 drones. **Historical note (superseded
> below): 2026-09-16 found `controller=7`/`8` crashing a plain single-drone hover in CS2 sim.**
> **2026-09-18 UPDATE: that sim instability is fully resolved.** Root cause was a real
> `crazyflie_sil.py` bug (never populated `state.acc`, which both controllers read for their
> position-INDI residual term) — fixed and re-verified across single-drone (4 trajectory
> shapes: hover/figure8/circle/oval), 2-drone, and 3-drone scenarios, including the real
> downwash-modeling backend (`neuralswarm`) with a genuine multi-vehicle interaction force —
> all clean, no divergence, for both controllers. **`controller=7`/`8` are no longer blocked
> from hardware by anything sim-side** — a real flight (single-drone hover first, normal C.0
> gate discipline) is a reasonable next step, understood as testing what the sim supports, not
> a guarantee. Full account: `firmware_app/host/naindi_reference_build_notes.md`. If our own
> full INDI (`controller=6`, `ctrl_mode=3`) has trouble with 2 drones in the meantime, the
> fallback options remain geometric (`ctrl_mode=0`) or retuning our own INDI gains. See
> `docs/strategy_controller_map.html` and memory `project_naindi_faithful_port`/
> `project_controller8_naindi_hybrid_2026-09-16` for the full history. Workflow:
> `flying_drone_stack/tools/README_usd_thesis_logging.md`.
> **All lab-relevant software preparation is finished. The project is waiting on lab access, not
> on code.**

### The short version

INDI is commissioned and flying. The project has moved on to the thesis proper: comparing up to
seven control strategies for interaction-force-aware multirotor teams. All simulation and software
preparation is complete; everything remaining is hardware.

| Subsystem | State |
|---|---|
| Geometric SE(3) + INDI (modes 0–3) | Flying. Gains in `crazyswarm2/crazyflie/config/crazyflies.yaml`, pushed over CRTP at connect |
| Trajectory upload | **Mode E** (stock Crazyswarm2 `uploadTrajectory`) is the standard. Mode D (`--onboard`) is frozen byte-identical and still required for aggressive manoeuvres |
| Formation library | 16 scenarios, sim-validated 34/34. `crazyswarm2/crazyflie_examples/crazyflie_examples/formations/` |
| Simulation | The same Rust source that flies runs in the CS2 simulator via SIL bindings |
| Residual measurement | `indi.a_res_*` = `a_meas − a_model` = f_res/m — **the core thesis signal**, logged in every controller mode |
| Learned residual model | Built end to end: onboard network, weight upload, training pipeline, sim dry run. **Never flown** |

### ⚠️ Three flight-code changes exist that have NEVER FLOWN

All three change control behaviour, and none is detectable in single-drone flight. They must clear
the C.0 hardware-validation gate (`docs/11`) before any data collection counts.

1. **Residual sign fix** — position INDI was *adding* `a_res` instead of subtracting it, so it
   reinforced every unmodelled force at exactly 2.00×. Now `.sub(a_indi)`.
2. **`a_res` gating fix** — RPM was read only when `mode != 0`, so `a_res` read exactly zero under
   the geometric controller, silently blocking Geometric+NN training data.
3. **`rnn.en` feedforward** — the learned residual now feeds the position loop (`.sub(a_nn)`).
   Default off → byte-identical, but it is a control term.

### Do not

- Change attitude gains (KR/KW) on the brushless platform **outside C.0**. Scoped 2026-08-23: an
  attitude retune to remove the remaining oscillation is explicitly allowed *during* the Hardware
  Gate, and forbidden after the freeze. Never as desk work — it needs flight data.
- Start the RL policy networks (strategies 5 and 7) — deliberately deferred.
- Select `stabilizer.controller=7` or `=8` (`naindi.rs`/`naindi_hybrid.rs`) on hardware **without
  going through the normal C.0 hardware-validation gate first** — the 2026-09-16 sim crash that
  originally blocked them is resolved as of 2026-09-18 (see the workflow banner above: single-
  drone, 2-drone, and 3-drone SIL tests, including the real downwash backend, all clean), so a
  first real flight is now a legitimate next step, not something to keep deferring — but sim
  cleanliness is a hypothesis for the lab to confirm, same as any other new control law, not a
  substitute for C.0 (`docs/11`).
- Use `--rest-to-rest` — measured worse.
- Fork `crazyflie-firmware`. It tracks bitcraze upstream; changes stay local and are recorded in
  `firmware_app/host/LOCAL_MODIFICATIONS.md`.
- Commit or push unless explicitly asked, and never add a `Co-Authored-By:` trailer.

### Controllers 11 / 12 — Tal & Karaman INDI (2026-10-12; first flown 2026-10-13: runs, flips at takeoff, NOT validated)
`stabilizer.controller 11` = `src/tk_indi.rs` (Rust), `12` = `controller_tk_indi.c` in `crazyflie-firmware` (C). The INDI of the paper item for item (paper frames NED/FRD inside, Table III gains, 30 Hz filter, no limits, `ctrlTk.*` params, `ctrlTk.jrz` default 2.5e-8 = estimate). Needs the image `build_artifacts/cf21bl_study_usd96_rpmf_rnn_iz_sentinel_ON_tk.bin` (OOT6/OOT7 on); plain `make` in the tree still builds the 6–10 layout. Tests: `host/test_tk_indi_rust_vs_ref.py`, `experiments/analysis/test_tk_indi_reference.py`, `experiments/analysis/tk_indi_sil_run.py`. Details and open risks: `docs/74` §6. Do not select on hardware without the C.0 gate (`docs/11`); first fallback `ctrlTk.jrz 0`. **Hardware 2026-10-13:** the first attempts silently ran controller 6 (cf5 had the non-`_tk` image; the firmware ignores `stabilizer.controller` 11/12 without the OOT6/OOT7 rows, and the param readback is only a cache) — always run `flying_drone_stack/tools/check_active_controller.py <uri> 6 10 11 12 6` after a flash (console must say "Using OutOfTree6 (11)" / "OutOfTree7 (12)"). With the right image the C port flips within 1.5 s of motor start (torque command follows the RPM-based torque estimate from the first tick on the ground; hypothesis). Not tried: takeoff on the geometric ramp and switch at altitude (`simple_flight` default handover). `crazyswarm2/crazyflie/config/study/tk_variants/` (gain/filter patches) are parked. Details: `docs/74` §6, `docs/lab_sessions/2026-10-13.md` §7.

### Residual-learning quick reference

**2026-09-21:** Pipeline runs on real C.1 merges; E2E validation on **4 flights** (see
`docs/30` + `experiments/analysis/out/c2_e2e_2026-09-21/`). LOO: A3 folds beat predict-zero;
A1 held-out and cross-scenario folds fail badly. `test_real_data_pipeline.py` closes the
loader↔compiled-firmware gap on real rows (~1e-6 m/s²). C.2 is **data-limited**, not blocked on
software — see `tools/residual/README.md` and `docs/13` §6.

```bash
# The two test suites need the residual_nn feature (OFF by default -- the weight array
# overflows real firmware RAM, so normal flight builds exclude it)
cd firmware_app && DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" \
    cargo build --release --target x86_64-unknown-linux-gnu --features residual_nn
cd ~/Desktop/crazyflie-firmware && make bindings_python

python3 firmware_app/host/test_residual_nn.py     # 19 checks -- onboard evaluation
python3 tools/residual/test_pipeline.py           # 13 checks -- train->export->upload->compare
python3 tools/residual/test_real_data_pipeline.py # real merged CSV vs compiled firmware (Stage C)

python3 tools/residual/train.py merged_a1.csv merged_a3.csv -o weights/c1.npz

# Upload to a drone (does NOT enable it -- rnn.en stays 0 without --enable)
ros2 run crazyflie_examples upload_residual_weights -- --weights weights/c1.npz
```

Design and results: `~/Desktop/flying_robot_course/docs/13_Residual_Learning.md`.

## Behavioral guidelines

**Think before coding.** State assumptions explicitly. If multiple interpretations exist, present them — don't pick silently. If something is unclear, stop and ask.

**Simplicity first.** Minimum code that solves the problem. No features, abstractions, or flexibility beyond what was asked. If you write 200 lines and it could be 50, rewrite it.

**Surgical changes.** Touch only what you must. Don't improve adjacent code, comments, or formatting. Match existing style. If you notice unrelated dead code, mention it — don't delete it. Remove only imports/variables/functions that YOUR changes made unused.

**Goal-driven execution.** Define success criteria before starting. For multi-step tasks, state a brief plan with a verify step for each. Clarifying questions come before implementation, not after mistakes.

## ⚠️ STUDY STATE 2026-10-17
The comparative study runs on ONE Omar C controller tree (controller 9 on both drones; S0 geometric, S1 INDI, S2 + NS2, S3 INDI + NS2). NS2 is implemented in C (`crazyflie-firmware/src/modules/src/ns2_residual.c`, patch `firmware_patches/ns2_residual_module.patch`, weights `firmware_app/ns2_weights.c` — now the **ns2_v2** export, v1 kept as `ns2_weights_v1.c` — generated by `tools/residual/export_weights_c.py`; the C weights come from `ns2_weights.c`, NOT from `CF_RNN_WEIGHTS_NPZ`). **ns2_v2 is released** (trained on 73 flights, all data; `tools/residual/weights/ns2_v2.npz`, registry `docs/ns2_model_registry.md`, never overwrite a registered model) and the image `build_artifacts/study_tree_v2.bin` is flashed on both drones (2026-10-19). **Unified-tree comparison (A1 + A8 + A4 offsets, NS2 v2, 76 pilot flights, uSD only; closed 2026-10-20, `docs/lab_sessions/2026-10-20.md` §0): `docs/tree_comparison_2026-10-19.md` §9** — A8 dip −12.7 / −7.1 / −5.6 / −4.0 cm, A1 mean z error −17.3 / −0.9 / −8.7 / +1.9 cm for S0 / S1 / S2 / S3; it replaces the 2026-10-15 validation. **uSD rule: count the `thesisNN` files on both cards before every block and archive at ≥ 85 (100 files = drones reset in the air when logging stops).** Strategy per flight is verified with `experiments/analysis/ns2_applied_check.py` (full thrust model), not the old slope test. S3 = INDI + NS2 as two plain switches (decided 2026-10-18, `docs/s3_indi_ns2_decision_2026-10-18.md`; do not re-introduce a second S3 variant without new evidence). Do NOT touch controller 6 / `lib.rs`, Omar Rust, Tal & Karaman or Omar's law. State and next lab steps: `docs/state_2026-10-17.md`, `docs/lab_sessions/next_session_plan.md`. FBL (Strategy 3, part of the study since 2026-10-19; one-page state and resume guide `docs/fbl/PROGRESS_AND_RESUME.md`; integration patch `firmware_patches/fbl_integration.patch`, inert unless `ctrlFbl.en = 1`, ARM candidate image built, never flashed) desk track Stages 0–2 done 2026-10-18 (`docs/fbl/STATUS.md`, `experiments/fbl/`): own residual fit R² 0.41 vs NS2 v2 0.79, the network adds nothing over the fitted physics model, closed-loop ordering holds, FBL gains untested on hardware; onboard port (Stage 3) needs a go.


## FBL (Strategy 3) — state 2026-10-21 (paused)
Desk part, robustness studies (`docs/fbl/ROBUSTNESS_2026-10-21.md`) and lab preparation closed, tested lower-gain fallback in the reading guide (reading guide `docs/fbl/FIRST_FLIGHT_CHECKLIST.md`; `lab_preflight.py --study FBL`; crazyswarm2 `--strategy FBL`); read `docs/fbl/PROGRESS_AND_RESUME.md` §4b, then `docs/fbl/LAB_PLAN.md`. `ctrlFbl.preset` 0 = authors' gains (kept), 1 = ours (default); `ctrlFbl.en` 0 by default (byte-identical to Omar). Tools: `tools/fbl_bench_check.py`, `tools/usd_fbl_config.txt` (`restore_study_sd_config.py --fbl`), `experiments/analysis/fbl_log_check.py`. Patch order: `fbl_integration.patch` then `fbl_gain_presets.patch` (`firmware_patches/README.md`). Nothing flashed or flown.

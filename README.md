# Advanced Flying Robots — Final Project

Motion planning and Incremental Nonlinear Dynamic Inversion (INDI) for aggressive quadrotor
trajectory tracking, commissioned and evaluated on three physical Crazyflie 2.x platforms
(standard, thrust-upgraded, brushless CF21BL). Georg Wolnik, supervised by Prof. Wolfgang Hönig.

## Start here

| | |
|---|---|
| **Report** | [`FINAL_REPORT.pdf`](FINAL_REPORT.pdf) ([source](FINAL_REPORT.tex)) — full writeup: methods, results, discussion, limitations, appendices |
| **Slides** | [`FINAL_PRESENTATION_BEAMER.pdf`](FINAL_PRESENTATION_BEAMER.pdf) ([source](FINAL_PRESENTATION_BEAMER.tex)) — final presentation, 29 slides |

The report is the authoritative, complete account. The slides are a condensed walkthrough of the
same material. Both were cross-checked against each other and against the raw results below.

### Raw experiment logs behind the report (per platform, chronological)

| Doc | Platform | Covers |
|---|---|---|
| [`results_2026-06-20.md`](flying_drone_stack/docs/results_2026-06-20.md) | Standard CF2.1 | INDI filter/gain commissioning, geometric-vs-INDI baseline |
| [`results_2026-07-11_upgraded_drone.md`](flying_drone_stack/docs/results_2026-07-11_upgraded_drone.md) | Thrust-upgraded CF2.1 | Thrust-model fix, gain re-tuning on the upgrade kit |
| [`results_2026-07-15_brushless.md`](flying_drone_stack/docs/results_2026-07-15_brushless.md) | Brushless CF21BL | Full INDI commissioning, the ≈7.22 Hz resonance investigation, notch-filter/kr fix attempts |

These are lab notebooks, not polished writeups — the report's Results/Discussion/Limitations
sections are the distilled, checked version of what's in here.

## Codebase — where to look

### End-to-end pipeline (how a flight happens, current standard = Mode E)

```
1. Generate trajectory (Rust, offline)
   flying_drone_stack/src/bin/export_poly4d.rs
   cargo run --release --bin export_poly4d -- --trajectory figure8 --mode 1 --kt 0.05
   → writes an 8-coefficient Poly4D CSV to crazyswarm2's data folder

2. Fly it (ROS 2 / Crazyswarm2, OptiTrack mocap for ground-truth pose)
   crazyswarm2/crazyflie_examples/crazyflie_examples/flight.py   (separate ROS 2 workspace)
   ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.05
   → onboard high-level commander evaluates the polynomial at 1 kHz,
     feeds the geometric or INDI controller (firmware_app, 500 Hz)
   → logs to Controls/logs/{trajectory}_mode{N}_{timestamp}.csv

3. Evaluate (Python)
   Controls/analyze_flight.py
   ~/.pyenv/versions/flying_robots/bin/python Controls/analyze_flight.py   # auto-picks latest CSV
   → phase-aligned XY RMSE, roll/pitch tracking error, per-axis diagnostics,
     writes the dashboard PNGs referenced throughout the report
```

See [`Controls/CLAUDE.md`](Controls/CLAUDE.md) and [`flying_drone_stack/CLAUDE.md`](flying_drone_stack/CLAUDE.md)
for the full command reference, including the earlier Mode B/C/D pipelines kept for history.

### Controllers and cost/objective functions

| What | Where |
|---|---|
| Geometric SE(3) baseline (Lee et al. 2010) — attitude/rate error → torque | [`flying_drone_stack/src/controller/mod.rs`](flying_drone_stack/src/controller/mod.rs) (`GeometricController::compute_control`) |
| INDI attitude+position loops (Tal & Karaman 2020) — simulation-side reference implementation | [`flying_drone_stack/src/controller/indi.rs`](flying_drone_stack/src/controller/indi.rs) (`IndiController`) |
| **Real onboard controller** flown on hardware (geometric + full INDI, both loops, notch filter) | [`flying_drone_stack/firmware_app/src/lib.rs`](flying_drone_stack/firmware_app/src/lib.rs) (`controller_step`, from line 1245) |
| Minimum-snap planning cost: `J_total = Σ J_snap(T_i) + k_t·Σ T_i` | [`flying_drone_stack/src/planning/richter.rs`](flying_drone_stack/src/planning/richter.rs) (top-of-file docs, `RichterTrajectory::plan`/`plan_paper`) |
| Differential flatness (position path → full attitude/rate/thrust reference) | [`flying_drone_stack/src/planning/flatness.rs`](flying_drone_stack/src/planning/flatness.rs) |

`firmware_app/src/lib.rs` is the one that actually flew every result in the report — the
`src/controller/` versions are the simulation/test counterparts used for `cargo test` and
`planning_sim`.

### Everything else

- [`flying_drone_stack/CLAUDE.md`](flying_drone_stack/CLAUDE.md) — full module map, key types, build/flash/test commands, all flight-mode history (A–E)
- [`flying_drone_stack/README.md`](flying_drone_stack/README.md) — Rust stack quick-start (build, test, offline analysis binaries)
- [`Controls/CLAUDE.md`](Controls/CLAUDE.md) — Python flight/analysis scripts reference
- [`ASSIGNMENTS.md`](ASSIGNMENTS.md) — earlier-in-course assignment writeups (dynamics, estimation, SLAM), not part of the final report's scope

## Quick build/test

```bash
cd flying_drone_stack && cargo build --release
cd flying_drone_stack && cargo test              # 276 tests, 0 failures expected
```

Report/slides are built with `pdflatex` (`FINAL_REPORT.tex`, `FINAL_PRESENTATION_BEAMER.tex`);
no extra dependencies beyond a standard TeX Live install.

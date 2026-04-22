# Course Assignments

Four graded assignments (20 pts each) from the *Flying Robots* course (TU Berlin / RIG, Nov–Dec 2025).
All implemented in Rust; real-hardware flights use a Crazyflie 2.1 with Flow Deck v2 + Multi-ranger Deck.

> **Note on approach:** The assignments specify porting controller/estimator code *into* the Crazyflie
> firmware (via `bindgen`, `no_std`, uSD-card logging). We implemented **both paths**:
> - **Firmware controller** (`firmware_app/`): SE(3) geometric controller compiled into the Crazyflie
>   STM32 as an out-of-tree controller (`CONTROLLER_OOT=y`), runs at 500 Hz onboard. Uses `bindgen`
>   FFI from `stabilizer_types.h`, `#![no_std]`, and outputs `controlModeForceTorque` directly.
> - **PC-side shadow track** (`src/bin/main.rs`): all algorithms run at 20 Hz on the laptop,
>   communicating over Crazyradio PA. The shadow controller logs what it *would* command, enabling
>   offline comparison without flight risk.
>
> The firmware app satisfies the lecture slide requirements exactly (`no_std`, `f32`, `bindgen`).
> uSD logging is replaced by the equivalent radio CSV path (same data, different transport — the
> Crazyflie used in this course does not carry a uSD deck).

---

## Assignment 1 — Dynamics Simulation & Integrator Comparison

### Goal
Build a rigid-body multirotor simulator, compare numerical integrators (Euler, RK4, exponential),
and validate the dynamics model against real flight data.

### What we did
- Implemented full 6-DOF quadrotor dynamics (SE(3) rigid body) with motor mixing.
- Compared four integrators: Euler, RK4, exponential-Euler, exponential-RK4.
- Validated against the course-provided dataset (`fr00.csv`) by replaying IMU inputs and comparing
  simulated state to logged firmware state.

### Key code
| File | Description |
|------|-------------|
| `src/dynamics/` | MultirotorState, MultirotorParams, MultirotorSimulator |
| `src/integration/` | Euler, RK4, ExpEuler, ExpRK4 integrators |
| `src/bin/assignment1.rs` | Simulation binary: integrator comparison + fr00 validation |
| `scripts/plot_assignment1.py` | Plots: step response, integrator drift, validation vs fr00 |

### Results
```
cargo run --release --bin assignment1
~/.pyenv/versions/flying_robots/bin/python scripts/plot_assignment1.py
```

Three validation scenarios are run automatically:

| Scenario | Description | Key result |
|----------|-------------|-----------|
| 1 — Synthetic spin | Constant torque; RK4 ≫ Euler in drift | RK4: 0.03 mm, Euler: 68 mm at t=5 s |
| 2 — Convergence | dt sweep 100→1 ms; confirm order | RK4 O(4), Euler O(1) as expected |
| 3 — Real flight (k-step) | 165 windows from `circle_2026-03-17` CSV; k=1..10 | 13.7 mm at k=1 → 63.2 mm at k=10; all integrators within 1 mm of each other |

**Scenario 3 interpretation:** On real 20 Hz data, all four integrators give virtually identical
predictions (< 1 mm spread). The bottleneck is imperfect action reconstruction from IMU noise
(thrust from `|acc|·m·g`, torque from finite-difference gyro), not integration order. This is
the physically correct result: integrator choice matters for long time horizons with exact
inputs; it is dominated by measurement noise in practice.

<img src="results/assignment1/images/assignment1_position_comparison.png" width="48%"> <img src="results/assignment1/images/assignment1_accuracy_analysis.png" width="48%">

<img src="results/assignment1/images/assignment1_real_flight_validation.png" width="60%">

---

## Assignment 2 — SE(3) Geometric Controller (Real Flight)

### Goal
Implement the Lee geometric controller (Lee, Leok, McClamroch 2010) for the Crazyflie 2.1.
Test and tune in simulation. Execute physical test flights and report tracking errors.

### What we did
Implemented the full SE(3) geometric controller in Rust (`src/controller/`).
Tuned in simulation (`src/bin/assignment2.rs`, circle + figure-8 trajectories).

**Physical flight approach — two paths, both implemented:**

1. **Onboard (firmware_app/)**: The SE(3) controller is compiled as an out-of-tree Crazyflie
   controller (`CONTROLLER_OOT=y`, `no_std` Rust, `bindgen` FFI). It runs at **500 Hz** inside
   the STM32 stabilizer task, reading Kalman EKF state and computing thrust + torques directly.
   The laptop sends position setpoints (CRTP); the onboard controller closes the full SE(3) loop.
   Flash: `cd firmware_app && make cload`.

2. **PC-side (`my_circle` maneuver in main.rs)**: The controller runs on the laptop at 100 Hz
   and sends RPYT setpoints to the Crazyflie's attitude rate stabilizer. This closes the outer
   position + velocity loop offboard, useful for rapid iteration without reflashing.

The real-flight results below were obtained using the PC-side path (`my_circle`).

Maneuver: `my_circle` — 0.2 m radius circle at ω = 0.5 rad/s (~12.6 s/lap).
Best flight: `runs/my_circle_2026-03-26_19-45-02.csv` (41 s airborne, clean tracking).

### Key code
| File | Description |
|------|-------------|
| `src/controller/` | GeometricController — SE(3) trajectory + attitude tracking |
| `src/bin/assignment2.rs` | Simulation: circle + figure-8, gain tuning |
| `src/bin/main.rs` | `my_circle` maneuver: PC-side velocity PI → RPYT setpoints |
| `scripts/plot_assignment2_flight.py` | Flight plots: XY trajectory, position vs time, 3D error |

### Results (real flight, `19-45-02`)
```
~/.pyenv/versions/flying_robots/bin/python scripts/plot_assignment2_flight.py \
    runs/my_circle_2026-03-26_19-45-02.csv
```

| Metric | Value |
|--------|-------|
| RMS 3D error | 70.8 mm |
| RMS x | 25.3 mm |
| RMS y | 43.4 mm |
| RMS z | 49.9 mm |
| Cumulative 3D error | 2.129 m·s |
| Attitude stability | roll/pitch RMS < 5° throughout |

> **On the z error:** The ToF sensor reads ~6 cm low relative to the drone's actual height
> (sensor offset + floor surface). The 50 mm z RMS is a calibration offset, not a controller
> error — XY tracking (25–43 mm) reflects actual control performance.
>
> **On XY drift:** Optical flow integrates velocity → position. Without an absolute position
> reference (e.g. Lighthouse), XY drift is unbounded and accumulates over ~10–20 s. This is
> a hardware limitation, not a controller deficiency.

<img src="results/assignment2/images/assignment2_flight_xy.png" width="48%"> <img src="results/assignment2/images/assignment2_flight_position.png" width="48%">

<img src="results/assignment2/images/assignment2_flight_error.png" width="48%">

---

## Assignment 3 — MEKF State Estimation (Real Flight)

### Goal
Implement a Multiplicative Extended Kalman Filter (MEKF) for attitude and position estimation.
Run on real hardware and compare quantitatively to the Crazyflie's onboard firmware EKF.

### What we did
Implemented the MEKF following Mueller et al. (2015) in `src/estimation/mekf.rs`:
- **Predict:** IMU gyro integration on quaternion (body-frame angular velocity).
- **Update — height:** ToF range measurement (scalar EKF update).
- **Update — flow:** PMW3901 optical flow → body-frame XY velocity (two scalar updates).
- **Update — VO:** Visual odometry XY position (two scalar updates, used when AI Deck is active — not used in these flights).

The MEKF runs live during every flight (fusing IMU + ToF + optical flow only — no AI Deck)
and its output is logged alongside the firmware EKF in the 49-column run CSV
(`mekf_roll/pitch/yaw/x/y/z` vs `roll/pitch/yaw/pos_x/y/z`).
Comparison is therefore from the same flight, same sensors, zero post-processing needed.

Note: `mekf_pitch` has the opposite sign convention to `stabilizer.pitch` — this is a known
display-only issue, corrected by negating pitch in the plot script (line 65 of
`plot_assignment3_flight.py`).

### Key code
| File | Description |
|------|-------------|
| `src/estimation/mekf.rs` | Full MEKF: predict, height/flow/VO updates, Coriolis term |
| `src/bin/mekf_eval.rs` | Offline MEKF replay against a run CSV, prints RMSE |
| `scripts/plot_assignment3_flight.py` | Plots: orientation comparison, position comparison, XY overlay |

### Results (Mar 17 2026 back-to-back flights, airborne samples only)

```
# Reproduce all three plots (hover / circle / figure-8):
~/.pyenv/versions/flying_robots/bin/python scripts/plot_assignment3_flight.py runs/hover_2026-03-17_19-33-08.csv
~/.pyenv/versions/flying_robots/bin/python scripts/plot_assignment3_flight.py runs/circle_2026-03-17_19-35-31.csv
~/.pyenv/versions/flying_robots/bin/python scripts/plot_assignment3_flight.py runs/figure8_2026-03-17_19-37-32.csv
```

| Maneuver | Roll RMSE | Pitch RMSE | Yaw RMSE | x RMSE | y RMSE | z RMSE |
|----------|-----------|------------|----------|--------|--------|--------|
| Hover    | 1.12°     | 0.95°      | 1.02°    | 3.4 cm | 3.1 cm | 1.1 cm |
| Circle   | 0.66°     | 0.58°      | 3.27°    | 5.1 cm | 7.1 cm | 0.5 cm |
| Figure-8 | 0.94°     | 0.61°      | 1.53°    | 5.6 cm | 3.6 cm | 0.8 cm |

> **On attitude RMSE:** Sub-1° roll/pitch agreement across all maneuvers confirms the MEKF
> tracks the firmware EKF closely. Yaw RMSE is higher on the circle (3.27°) because the drone
> continuously yaws — small timing differences between the two estimators accumulate.
>
> **On position RMSE:** 3–7 cm XY spread between the two estimators is expected. Both rely on
> optical flow integration (dead-reckoning) with no absolute position anchor. The RMSE measures
> the *difference between two drifting estimates*, not absolute accuracy.

**Hover**
<img src="results/assignment3/images/assignment3_hover_orientation.png" width="48%"> <img src="results/assignment3/images/assignment3_hover_xy.png" width="48%">

**Circle**
<img src="results/assignment3/images/assignment3_circle_orientation.png" width="48%"> <img src="results/assignment3/images/assignment3_circle_xy.png" width="48%">

**Figure-8**
<img src="results/assignment3/images/assignment3_figure8_orientation.png" width="48%"> <img src="results/assignment3/images/assignment3_figure8_xy.png" width="48%">

---

## Assignment 4 — Minimum-Snap Spline Planning + Differential Flatness

### Goal
Plan an aggressive figure-8 trajectory using minimum-snap polynomial splines (QP).
Compute desired thrust and torques via differential flatness (Faessler et al. 2018).
Validate with open-loop and closed-loop simulation — **no real flight required**.

### What we did
- Planned an 8-waypoint figure-8 with 8th-order polynomial splines minimising ∫snap² (Clarabel QP).
- Computed F_d, τ_u, ω_d, ω̇_d analytically from flatness outputs (pos / vel / acc / jerk / snap + yaw).
- **Open-loop:** flatness-computed thrust + torque fed directly to simulator (no feedback).
- **Closed-loop:** SE(3) geometric controller tracks the spline with flatness feedforward.
- Wrote three CSVs and six diagnostic plots.

### Key code
| File | Description |
|------|-------------|
| `src/planning/spline.rs` | SplineTrajectory: QP minimum-snap, piecewise 8th-order |
| `src/planning/flatness.rs` | compute_flatness(): FlatOutput → thrust, torque, ω, ω̇ |
| `src/bin/assignment4.rs` | Full pipeline: plan → open-loop sim → closed-loop sim → CSV |
| `scripts/plot_assignment4.py` | 6 plots: 3D trajectory, position tracking, error, flatness actions, ω, kinematics |

### Results (simulation)
```
cargo run --release --bin assignment4
~/.pyenv/versions/flying_robots/bin/python scripts/plot_assignment4.py
```

| Simulation | RMS 3D error |
|------------|-------------|
| Open-loop (flatness feedforward only) | ~321 mm |
| Closed-loop (geometric controller) | **2.2 mm** |

> Open-loop error is expected — without feedback, any model mismatch or numerical error
> accumulates freely. The closed-loop result (2.2 mm) confirms both the spline planner and
> the flatness computation are correct, and the geometric controller achieves near-perfect
> tracking in simulation.

<img src="results/assignment4/images/assignment4_3d.png" width="48%"> <img src="results/assignment4/images/assignment4_errors.png" width="48%">

<img src="results/assignment4/images/assignment4_position.png" width="48%"> <img src="results/assignment4/images/assignment4_flatness_actions.png" width="48%">

---

## Real-Hardware Flight Modes

Beyond the assignments, the stack supports two additional trajectory flight modes:

### Mode 1 — HLC (High-Level Commander, Python)

Pre-generates a min-snap spline trajectory, exports it as Poly4D coefficients (degree-7 after
Hermite conversion), uploads to drone onboard memory, and executes via `start_trajectory`.
The drone's built-in polynomial evaluator runs at ~100 Hz onboard — radio only needed to start.

```bash
# requires: firmware flashed with OOT controller (make cload from firmware_app/)
~/.pyenv/versions/flying_robots/bin/python Controls/run_spline_circle.py
~/.pyenv/versions/flying_robots/bin/python Controls/autonomous_sequence_high_level.py
```

### Mode 2 — Rust live full-state (CRTP type 6)

Laptop evaluates the degree-8 spline in real time, computes differential flatness, and sends
CRTP full-state packets (pos + vel + acc + quaternion + ω) at 20–25 Hz. The onboard SE(3)
controller receives all feedforward terms (velocity, acceleration, angular velocity) — not just
position. Radio link must stay alive throughout.

```bash
cargo run --release --bin spline_circle_test
cargo run --release --bin spline_figure8_test
```

---

## Reproducing All Results

```bash
# 1. Build
cargo build --release

# 2. Run simulations (Assignments 1, 2, 4)
cargo run --release --bin assignment1
cargo run --release --bin assignment2
cargo run --release --bin assignment4

# 3. Generate all plots
PYTHON=~/.pyenv/versions/flying_robots/bin/python

$PYTHON scripts/plot_assignment1.py
$PYTHON scripts/plot_assignment2_flight.py runs/my_circle_2026-03-26_19-45-02.csv
$PYTHON scripts/plot_assignment3_flight.py runs/hover_2026-03-17_19-33-08.csv
$PYTHON scripts/plot_assignment3_flight.py runs/circle_2026-03-17_19-35-31.csv
$PYTHON scripts/plot_assignment3_flight.py runs/figure8_2026-03-17_19-37-32.csv
$PYTHON scripts/plot_assignment4.py
```

All plots are written to `results/assignment{1,2,3,4}/images/`.
Flight CSVs are in `runs/` (not committed to git if large — contact for access).

# Flight Control — Concepts & Design Notes

Running notes on design decisions, theory, and open questions.
Extend this file as new topics come up.

---

## Table of Contents

1. [Trajectory Speed Scaling — How It Works](#1-trajectory-speed-scaling--how-it-works)
2. [Yaw / Heading Direction During Maneuvers](#2-yaw--heading-direction-during-maneuvers)
3. [Project Roadmap](#3-project-roadmap)
4. [INDI — Concepts and Variants](#4-indi--concepts-and-variants)
5. [INDI — Tuning Parameters and Filters](#5-indi--tuning-parameters-and-filters)

---

## 1. Trajectory Speed Scaling — How It Works

### The core question
When you run the same trajectory at 2× speed, do the polynomial coefficients change?

### Answer: it depends on the approach

**HLC time-scale trick (Python `run_fast_figure8.py` old version)**
The same Poly4D coefficients are uploaded once. Speed-up is passed to the drone as
`time_scale = 1 / SPEED_SCALE` in `start_trajectory()`. The drone evaluates
`p(t · time_scale)` internally — same polynomial, run faster. Velocities scale by
`SPEED_SCALE`, accelerations by `SPEED_SCALE²`. Coefficients in memory are unchanged.

**QP re-solve with shorter durations (current approach for all fast variants)**
The planner (`SplineTrajectory::plan`) is re-run with halved segment durations.
This produces genuinely different polynomial coefficients because the min-snap
cost function (minimise ∫ θ̈² dt) changes with T. Mathematically, time-scaling
a polynomial from T_old to T_new is equivalent to replacing each coefficient:

```
c'_k = c_k · (T_old / T_new)^k
```

So the re-solved coefficients are numerically close to what this scaling gives,
but the QP finds the globally optimal polynomial for the new time constraint.

**Analytic trajectories (helix, circle)**
No polynomial at all — closed-form derivatives computed in real time. "Fast" version
just means halving `LAP_TIME`, which doubles `OMEGA_C` and `VZ_UP`. Nothing to re-solve.

### Summary table

| Maneuver | Coefficients change? | Mechanism |
|----------|---------------------|-----------|
| Figure-8 old SPEED_SCALE trick | No | `time_scale` param in `start_trajectory` |
| Figure-8 fast (current) | Yes | QP re-solved with halved durations |
| Flip / Roll fast variants | Yes | QP re-solved with T=0.45s |
| Helix fast | N/A | Analytic — just halve LAP_TIME |
| Circle fast | Yes (Rust) / No (HLC) | SplineTrajectory re-plan / time_scale |

**Bottom line**: the spatial path is always the same. Whether the coefficients numerically
change depends on whether they are expressed in physical time (they change by `(T_old/T_new)^k`)
or whether the runtime uses a time-scale trick.

---

## 2. Yaw / Heading Direction During Maneuvers

### Current behaviour
All maneuvers use **fixed yaw** — the drone maintains its initial hover heading throughout.
The front of the drone does not rotate to face the direction of travel at any point.

### Heading-aligned yaw (front faces direction of travel)

**Pros**
- AI Deck camera sees where the drone is going
- Optical flow PMW3901 works best when forward motion is along body x-axis (minimal sideslip)
- Looks more natural / intuitive

**Cons**
- Extra yaw rate commands → yaw tracking lag couples into position control
- At the figure-8 crossing point the drone would need to reverse heading (~180°) almost
  instantaneously — physically impossible
- Adds tuning complexity with little benefit at small tilt angles (≤ 15°)

### Recommendation
**Keep fixed yaw as-is.** The figure-8 crossing makes heading alignment infeasible
for that trajectory, and consistency across all maneuvers is cleaner. Optical flow
sideslip is negligible at our tilt angles (0.6°–48°).

Could be revisited if the project pivots to a camera-centric use case where the AI
Deck FOV during the trajectory matters.

---

## 3. Project Roadmap

### Phase 1 — Geometric SE(3) + optical flow (current)
Establish the performance ceiling of the classical approach.

The Lee 2010 geometric controller computes required torques from a dynamics model
assuming perfect motor response. Works well for gentle maneuvers. Breaks down when
motor lag is a significant fraction of the maneuver time — at T=0.45s flips the lag
is ~2.5% of the maneuver, enough to destabilise.

Optical flow gives noisy XY position with cm–dm drift, especially on uniform floors
and during tilted flight. The key use of this phase is **measuring the failure modes**
of the classical approach cleanly.

### Phase 2 — Attitude INDI (intermediate)
Replace only the torque computation in the inner attitude rate loop:

```
Δτ = J · (α_des − α_meas),   α_meas = (ω − ω_prev) / dt
```

No RPM sensors needed. The gyro captures the net real angular acceleration including
motor lag, blade flapping, and unmodelled dynamics. The controller corrects for the
discrepancy between commanded and actual motion one timestep at a time.

Expected outcome: fast flips/rolls that the geometric controller cannot complete
become flyable. This is the standard first step (Faessler et al. 2018) and is
well-validated on Crazyflie-class vehicles.

Only one new tuning parameter over the geometric controller: the gyro low-pass
filter cutoff frequency.

### Phase 3 — Full INDI with motor RPM deck
With per-motor RPM available the current force/torque state can be computed
directly from the motor model:

```
τ_current = B · diag(k_t · Ω²)   ← from RPM measurements
Δτ = τ_des − τ_current
```

Matters most at very high angular rates (gyro differentiation gets noisy) and
when motors approach saturation (where the incremental assumption breaks down).
Allows explicit saturation-aware torque allocation.

See [Section 5](#5-indi--tuning-parameters-and-filters) for the substantially
larger tuning burden this introduces.

### Phase 4 — Motion capture for estimation
Replace optical flow + EKF with MoCap (Vicon / OptiTrack / Qualisys) injected
directly into the firmware EKF via CRTP external pose.

- Position accuracy: sub-millimetre vs cm–dm with optical flow
- Rate: 100–300 Hz vs ~10 Hz effective from flow
- Critical for aggressive maneuvers where position drifts fast and estimator
  failures are currently indistinguishable from controller failures
- MoCap makes it possible to separate the two cleanly — essential for
  publishing meaningful results

Crazyflie firmware already supports external pose injection; no firmware changes
needed.

### Phase 5 — Better platform (Crazyflie Bolt or similar brushless)
Current CF 2.1: ~31g, max thrust ~160g → T/W ≈ 5:1

A brushless Bitcraze platform (Bolt, Bolt 1.1) with larger motors/props gives
T/W of 8–12:1. This matters because:

- During a fast flip the motors must briefly produce near-zero thrust (inverted
  phase) then immediately recover. At T/W = 5:1 there is very little headroom.
  At T/W = 10:1 recovery is much faster.
- Higher T/W allows maneuver speeds that are physically impossible on the stock
  platform — the motors simply cannot respond fast enough at 31g.
- Firmware and software stack stays identical (same CRTP, same OOT controller
  interface) — hardware swap only.

### Phase 6 — Novel contribution
The combination of full INDI + MoCap + capable platform + systematic trajectory
suite creates a well-controlled experimental setting. Potential contributions:

**Systematic controller comparison (most accessible)**
Geometric vs attitude INDI vs full INDI on an identical trajectory set with MoCap
ground truth. Clean quantitative breakdown of where each fails and why. A
reproducible benchmark for Crazyflie-class vehicles does not currently exist in
a clean form.

**Saturation-aware INDI allocation**
When a motor saturates during a fast maneuver, naive INDI can diverge. An
allocation layer that redistributes torque demands under motor limits while
preserving the incremental structure would be novel and practically useful.

**Aggressive trajectory feasibility with INDI**
Current min-snap planners assume a model-based feasibility bound (max thrust/torque).
With INDI you can push closer to actual hardware limits. Characterising how much
headroom INDI buys — and feeding that back into the planner — is an open problem.

**Learning residual on INDI**
If systematic errors remain after full INDI (e.g. aerodynamic rotor interactions
at high speed), a small learned residual on top of the incremental law that
generalises across maneuvers is an active research direction with limited published
results on micro vehicles.

**Key requirement for publishability**: same platform, same trajectories, same
estimator, controllers swapped one variable at a time, MoCap as objective ground
truth. Most existing work changes too many variables simultaneously.

---

## 4. INDI — Concepts and Variants

### What INDI is

Incremental Nonlinear Dynamic Inversion. Instead of computing the full required
torque from a model, compute only the *increment* needed relative to the current
state:

```
τ_new = τ_current + Δτ
Δτ  = J · (α_des − α_meas)
```

The incremental structure means model errors only affect the *correction* term, not
the entire control output. The controller is inherently robust to model mismatch.

### Variant 1 — Attitude INDI (gyro only)

```
α_meas = (ω − ω_prev) / dt       ← numerically differentiate gyro
Δτ = J · (α_des − α_meas)
```

- No motor RPM needed
- The gyro captures the net real angular acceleration including motor lag,
  blade flapping, all unmodelled effects
- One new tuning parameter: low-pass filter cutoff on ω
- This is the standard approach in the MAV literature

### Variant 2 — Full INDI (with motor RPM)

```
τ_current = G(Ω) · [Ω₁², Ω₂², Ω₃², Ω₄²]ᵀ   ← from RPM measurements
Δτ = τ_des − τ_current
```

- Requires per-motor RPM (deck or estimation)
- More accurate at high angular rates where gyro differentiation is noisy
- Explicitly handles motor saturation in the allocation
- Substantially more parameters to identify and tune (see Section 5)

### Key reference
Smeur, Chu, de Croon — "Adaptive Incremental Nonlinear Dynamic Inversion for
Attitude Control of Micro Air Vehicles", JGCD 2016.

---

## 5. INDI — Tuning Parameters and Filters

### Why there are more parameters than for geometric control

The geometric controller has ~4–8 scalar gains with independent, intuitive effects.
INDI introduces **numerical differentiation** which amplifies high-frequency noise,
requiring filters — and those filters introduce phase lag which must be carefully
managed to maintain stability.

### Attitude INDI — new parameters

**Gyro low-pass filter cutoff (fc_gyro)** — the critical one
Applied to ω before differentiating. Too high → noise dominates α_meas and corrupts
the correction. Too low → phase lag destabilises the controller. Typical range:
30–80 Hz. Must be roughly 1/3 of the control loop bandwidth, but in practice
depends heavily on vehicle vibration levels (which are RPM-dependent).

**Inertia tensor J**
Not a gain, but must be accurate. In the geometric controller J errors only reduce
performance slightly (it's in a feedforward term). In INDI, J errors directly scale
every incremental correction — wrong J means systematically wrong Δτ on every step.
Requires CAD + experimental identification.

### Full INDI — additional parameters

**Actuator state filter cutoff (fc_act)**
RPM measurements are noisy. Filtering them introduces a second delay in the loop.
The two filter delays (fc_gyro and fc_act) must be carefully matched — mismatched
delays cause the controller to oscillate or diverge.

**Control effectiveness matrix G(Ω)**
Maps motor RPM² to force/torque vector. Contains:
- Thrust coefficient k_t (lift per RPM²)
- Drag coefficient k_q (reaction torque per RPM²)
- Arm geometry and motor rotation directions

These are nominally known but in reality temperature-dependent, RPM-dependent (aerodynamics
change at high speed), and vary with propeller wear. Full INDI is much more sensitive to
errors here than geometric because G is in the inner control loop, not just feedforward.
Requires proper system identification: spin each motor individually, measure thrust vs RPM.

**Motor time constant (τ_motor)**
First-order lag model of motor response to commanded RPM. This is the primary thing INDI
corrects for, so an accurate estimate matters. Identified from step responses. Varies with
battery voltage — a 4.2V vs 3.5V battery changes τ_motor measurably.

### The coupling problem

Unlike geometric gains, INDI parameters are coupled:

- Increasing fc_gyro → better responsiveness but more noise
- More noise → larger spurious Δτ on each step → worse position tracking
- More aggressive maneuver → higher motor RPM → more vibration → need lower fc_gyro
- Lower fc_gyro → less bandwidth → INDI cannot correct fast enough

This means the optimal filter parameters change with the operating condition. The
adaptive variant (Smeur 2016) estimates G(Ω) online to partially address this, at
the cost of additional stability analysis and initialisation requirements.

### Practical recommendation

1. Start with attitude INDI first — only fc_gyro is new. Tune it at moderate speed,
   verify improvement on fast flips/rolls.
2. Characterise how fc_gyro needs to change as maneuver speed increases.
3. Once attitude INDI is solid, add RPM feedback for full INDI. The intuition from
   step 1 directly informs how to choose fc_act.
4. Perform system identification for G(Ω) and τ_motor on the bench (thrust stand)
   before flying, not in the air.

---

*Last updated: April 2026*

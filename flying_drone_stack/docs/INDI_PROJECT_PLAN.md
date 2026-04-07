# Advanced Flying Robots — Project Plan
# INDI Controller for Aggressive Manoeuvres on Crazyflie

**Student:** Georg
**Mentor:** Omar Elsayed (partially Khaled Wahba)
**Module:** Multi-Robot Systems Project (9 ECTS, ~270 h)
**Start:** April 2026 (immediately)

---

## 1. Goal

Leverage the **existing INDI controller in the Crazyflie firmware** (`controller_indi.c`,
selectable via `stabilizer.controller = 4`) to perform aggressive quadrotor manoeuvres on a
Bitcraze platform (Crazyflie Brushless / Crazyflie 2.1 with thrust upgrade kit).

The focus shifts from *implementing* INDI (already done in firmware) to:
- **Tuning and characterising** the firmware INDI for aggressive flight
- **Designing aggressive trajectories** that exploit INDI's capabilities (flip, fast circle)
- **Systematic comparison** against SE(3) geometric and firmware PID baselines
- **Validating in simulation first** using the existing Rust stack, then on hardware

Deliverables: INDI working on hardware performing at least one aggressive manoeuvre,
simulation + hardware comparison, written report (6–8 pages), and final presentation.

---

## 2. Background & Motivation

The Flying Robots course (prerequisite) used an SE(3) geometric controller — a full
model-based approach that requires accurate inertia parameters and is inherently limited in
aggressiveness because the control law assumes the drone is near the desired trajectory.

**INDI** is an *incremental* control law that:
- Works from the *current* measured state and actuator output, not a reference model
- Needs only angular rate measurements and actuator feedback — robust to model mismatch
- Has been proven in literature for flips, fast rolls, and aggressive trajectories on
  small quadrotors (Smeur et al. 2016, 2019)
- Is already deployed in production autopilots (PX4, Paparazzi UAV)
- **Is already implemented in the Crazyflie firmware** (`controller_indi.c`), activated by
  setting `stabilizer.controller = 4` — no firmware modification needed to start

This makes INDI the natural next step after the geometric controller, and the existing
firmware implementation means the project focuses on trajectory design, tuning, and
systematic evaluation rather than low-level controller derivation.

---

## 3. Existing Foundation (From Flying Robots Course)

The project starts with a complete, validated Rust drone stack:

| Component | Status | Relevance to INDI |
|-----------|--------|-------------------|
| MultirotorSimulator (Newton-Euler) | ✅ | INDI simulation testbed |
| SE(3) geometric controller | ✅ | Comparison baseline |
| `Controller` trait (clean interface) | ✅ | INDI slots in here directly |
| MEKF state estimator | ✅ | Provides angular rates + attitude |
| Shadow track (log-without-flying) | ✅ | Safe INDI validation before enabling live |
| 49-column CSV logging | ✅ | Full post-flight analysis pipeline |
| Hardware integration (Crazyflie 2.1) | ✅ | Radio, CRTP, sensor streams all working |

This means **Phase 1 (sim) starts immediately** without any hardware setup.

---

## 4. Technical Approach

### 4.1 INDI Architecture

The standard cascaded INDI structure for a quadrotor:

```
Position error
     │
     ▼
┌─────────────────────┐
│  Outer loop          │   Position → desired acceleration
│  (PD or INDI-outer) │   Can reuse existing geometric outer loop
└──────────┬──────────┘
           │ desired body acceleration / angular acceleration
           ▼
┌─────────────────────┐
│  Inner loop — INDI  │   Incremental: Δu = G⁻¹ (ν − f(x,u₀))
│  Angular rate loop  │   G = control effectiveness matrix
└──────────┬──────────┘   ν = desired angular acceleration
           │ motor commands
           ▼
        Motors
```

**INDI inner loop law (Smeur 2016):**
```
Δu = G⁻¹ · (ν_des − ω̇_measured + G · u₀)
u_new = u₀ + Δu
```
Where:
- `ν_des` = desired angular acceleration (from outer loop)
- `ω̇_measured` = measured angular acceleration (differentiated gyro, low-pass filtered)
- `G` = control effectiveness matrix (maps motor Δu to angular acceleration Δω̇)
- `u₀` = previous actuator state

**Key insight:** G is the only model parameter needed — and it can be identified online or
from a simple hover experiment, making INDI robust to model uncertainty.

### 4.2 Firmware INDI — what's already there

The Crazyflie firmware `controller_indi.c` implements a rate-loop INDI inner controller:
- Runs at the firmware attitude rate (~500 Hz) — no radio latency in the inner loop
- Activated at runtime: `stabilizer.controller = 4` (no reflash needed)
- Parameters tunable over radio: `imu_filt_cutoff`, `act_filt_cutoff`, G matrix entries
- Outer position loop remains the existing firmware position PID (same as current setup)

**What this means for the project:**
- Inner loop (angular rate): firmware INDI at 500 Hz
- Outer loop (position → setpoint): our Rust stack at 20 Hz via radio (unchanged)
- The Rust stack continues to handle trajectory generation, logging, SLAM, safety

**Possible extension (if scope allows):** implement a Cascaded INDI outer loop in Rust
(position → desired acceleration → handed to firmware INDI inner loop) — this is
consistent with the `Controller` trait architecture.

### 4.3 Control loop architecture

```
Rust stack (laptop, 20 Hz)          Crazyflie firmware (onboard, 500 Hz)
┌─────────────────────────┐         ┌──────────────────────────────────┐
│ Trajectory generator    │         │ Outer loop: position PID         │
│ Safety layer            │──pos──▶ │   → desired attitude/thrust      │
│ MEKF / SLAM             │ setpt   │                                  │
│ CSV logging             │         │ Inner loop: INDI (controller_indi)│
└─────────────────────────┘         │   → motor PWM commands           │
                                    └──────────────────────────────────┘
```

**Onboard computation question (to discuss with Omar):**
The Rust stack currently runs on the laptop. Two upgrade paths exist:
- **Companion computer (RPi Zero 2W, ~€18):** runs existing Rust stack over USB instead
  of radio. Eliminates 50 ms radio latency in the outer loop, enables 100+ Hz outer loop.
  Change one URI string (`radio://` → `usb://`), cross-compile for `aarch64`. The current
  Rust crate cannot run directly on the Crazyflie STM32 (uses `std`, tokio, HashMap etc.)
  but runs trivially on a Linux companion computer.
- **Firmware-only approach:** keep Rust stack on laptop, implement aggressive trajectory
  commands purely as position setpoints. Simpler, works with current setup.

---

## 5. Platform

### Option A — Crazyflie Brushless (Bolt)
- Thrust-to-weight ~4–5× → can perform flip, fast rolls
- Different hardware from current stack (new driver/link work needed)
- **Recommended** for truly aggressive manoeuvres

### Option B — Crazyflie 2.1 with thrust upgrade kit
- Current platform, minimal driver changes
- Thrust-to-weight ~2–2.5× with upgrade → marginal for flip, good for fast circle
- Faster to get running since all hardware integration already exists

*To be decided with Omar based on lab availability.*

---

## 6. Positioning / Estimation

**Optical flow (current):** drifts in XY, unusable for aggressive manoeuvres at home.

**Lighthouse positioning (strongly recommended):**
- Crazyflie Lighthouse Deck (~€35) + SteamVR Base Station 2.0 (~€150)
- Sub-centimetre accuracy, no drift, natively fused by Crazyflie Kalman filter
- Zero code changes to the existing stack — `pos_x/y/z` in CSV become accurate
- Essential for home work; lab likely has a Lighthouse setup already

---

## 7. Work Plan & Milestones

**Total:** ~15 weeks, 17 h/week = 255 h project work + 15 h meetings

### Phase 0 — Setup & Literature (weeks 1–2, ~30 h)
- [ ] Read Smeur 2016 (INDI for multirotors), Smeur 2019 (cascaded INDI), Faessler 2017
      (differential flatness for aggressive trajectories)
- [ ] Survey INDI implementations in PX4 and Paparazzi UAV source
- [ ] Confirm platform + manoeuvre targets with Omar
- [ ] Confirm Lighthouse access (lab or purchase)
- [ ] Confirm control loop rate approach (firmware vs companion)

### Phase 1 — Simulation baseline + trajectory design (weeks 3–6, ~65 h)
- [ ] Implement `IndiController` in `src/controller/indi.rs` matching firmware INDI law
      (validates understanding of the algorithm before touching hardware)
- [ ] Validate hover: INDI sim matches SE(3) geometric sim
- [ ] Design aggressive trajectory profiles: fast circle, flip thrust pulse, aggressive climb
- [ ] Simulate flip: open-loop thrust spike → INDI stabilisation after rotation
- [ ] Compare INDI vs SE(3) geometric in sim: tracking error, control effort, robustness
- [ ] Identify expected G matrix values from sim dynamics (cross-check with hover ID)

**Go/no-go for Phase 2:** INDI stable in hover sim, flip trajectory completes in sim.

### Phase 2 — Hardware Preparation (weeks 5–7, ~30 h)
*(overlaps with Phase 1 second half)*
- [ ] Obtain platform (Bolt or thrust upgrade kit)
- [ ] Set up Lighthouse positioning (lab or home)
- [ ] Characterise motor dynamics: measure G (control effectiveness) from hover experiment
- [ ] Measure motor time constant τ (step response test)
- [ ] Verify angular rate measurements at target loop rate

### Phase 3 — Conservative Hardware Validation (weeks 7–10, ~60 h)
- [ ] Run INDI in **shadow mode** first (log `our_roll_cmd`, `our_thrust` — motors untouched)
      Compare shadow INDI output vs live geometric controller output CSV column-by-column
- [ ] Enable INDI live: hover test, no aggressive input
- [ ] Slowly increasing aggressiveness: fast yaw rate, tilt recovery
- [ ] Tune G matrix and filter cutoffs from flight data

**Go/no-go for Phase 4:** INDI stable in hover and gentle manoeuvres, shadow outputs
match sim predictions.

### Phase 4 — Aggressive Manoeuvres (weeks 10–13, ~50 h)
- [ ] Fast circle (r = 0.5 m, ω = 1.5+ rad/s)
- [ ] Target manoeuvre: flip (or platform-specific aggressive trajectory)
- [ ] Systematic comparison: INDI vs SE(3) geometric — tracking error, disturbance rejection
- [ ] Data collection for report

### Phase 5 — Report + Presentation (weeks 13–15, ~30 h)
- [ ] Written report (6–8 pages, paper format)
- [ ] Presentation (poster or slides)
- [ ] Final code cleanup and documentation

---

## 8. Open Questions for First Meeting with Omar

1. **Platform:** Brushless (Bolt) or brushed + thrust upgrade? What is available in the lab?

2. **Target manoeuvre:** Flip specifically, or open to fast circle / aggressive descent?

3. **Firmware INDI scope:** The firmware `controller_indi.c` is already there and activated by
   `stabilizer.controller = 4`. Is it sufficient as-is for the project, or does it need
   modification / extension (e.g. adding a cascaded outer INDI loop, changing filter cutoffs)?

4. **Outer loop — onboard computation:**
   The Rust outer loop currently runs on the laptop at 20 Hz via radio.  Two options:
   - **Firmware-only:** keep laptop stack, send position setpoints as now (simpler).
   - **Companion computer (RPi Zero 2W):** move Rust stack onto drone via USB link — eliminates
     50 ms radio latency, enables 100+ Hz outer loop. Cross-compile for `aarch64`, change one URI
     string. Is there lab hardware available, or does this scope creep too much?

5. **INDI variant:** Rate-loop inner only (firmware already does this) or full cascaded INDI
   (outer loop also INDI, in Rust — more ambitious, better disturbance rejection)?

6. **Lighthouse:** Does the lab have a Lighthouse setup? Or should I purchase my own base
   station (~€150)? Lighthouse is a prerequisite for reliable INDI experiments at home.

7. **Comparison baseline:** Compare INDI vs SE(3) geometric (the existing Rust controller)?
   Also include the Crazyflie built-in PID as a third baseline?

8. **Reporting expectations:** Any specific metrics/experiments the lab expects for the report
   (tracking error at what speed, which manoeuvres, minimum hardware demo requirements)?

---

## 9. Key References

- Smeur, E.J.J. et al. (2016). *Adaptive Incremental Nonlinear Dynamic Inversion for
  Attitude Control of Micro Air Vehicles.* Journal of Guidance, Control, and Dynamics.
- Smeur, E.J.J. et al. (2019). *Cascaded Incremental Nonlinear Dynamic Inversion for
  MAV Disturbance Rejection.* Control Engineering Practice.
- Faessler, M. et al. (2017). *Differential Flatness of Quadrotor Dynamics Subject to
  Rotor Drag for Accurate Tracking of High-Speed Trajectories.* IEEE RA-L.
- Mueller, M.W. and D'Andrea, R. (2015). *Relaxed hover solutions for multicopters.*
  (Relevant for thrust upgrade characterisation.)
- PX4 INDI implementation source (for reference, not copied):
  `src/modules/mc_att_control/AttitudeControl/`
- Paparazzi UAV INDI source (for reference):
  `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_indi.c`

---

## 10. How INDI Fits Into the Existing Stack

```
src/
├── controller/
│   ├── mod.rs          ← Controller trait (unchanged)
│   ├── geometric.rs    ← SE(3) (unchanged, used as baseline)
│   └── indi.rs         ← NEW: INDI implementation
├── bin/
│   └── main.rs         ← add "indi_hover", "indi_circle", "indi_flip" match arms
```

Everything else — MEKF, SLAM, safety layer, occupancy map, CSV logging, Lighthouse-fused
position — carries over unchanged.  The shadow track (existing `our_roll_cmd` /
`our_thrust` CSV columns) provides free validation before enabling INDI live.

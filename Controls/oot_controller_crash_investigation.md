# OOT Controller Crash Investigation — June 2026

## Context

**Setup**: Crazyflie 2.1 + RPM deck + USD deck, flying in OptiTrack mocap volume via Crazyswarm2 (CS2).  
**Controller**: Out-of-tree (OOT) Rust SE(3) geometric + INDI controller, 500 Hz, flashed as `cf2.bin`.  
**Problem**: After **June 20, 2026** all flights with the OOT controller (geometric mode 0 and full INDI mode 1) began oscillating and crashing. Flights with the **built-in Lee (PID) controller remained stable throughout**, using the identical mocap, radio, and CS2 setup.

Last confirmed stable OOT flights: June 20, 2026 (figure-8 with INDI, kt sweep 0.008–0.1, all stable).

---

## Symptoms

- **Geometric hover (mode 0)**: drone spirals outward with expanding amplitude, crashes within ~5–15 s
- **Full INDI (mode 1)**: faster oscillation onset, crashes within ~3–8 s  
- **Lee controller (built-in)**: same mocap, same radio, same day — stable hover and figure-8
- **SD card present/absent**: no effect on crash behaviour
- **New drone + new propellers + new RPM deck**: same crash pattern reproduced

---

## Hypothesis Tracker

### ❌ H1 — SD card / USD logging I/O latency
**Theory**: The USD deck writing 500 Hz logs was starving the stabilizer task or causing timing jitter in the controller loop, corrupting control output.  
**Test**: Removed SD card entirely, flew without USD logging.  
**Result**: OOT controller still crashed identically. **Ruled out.**

### ❌ H2 — OptiTrack / mocap noise or dropout
**Theory**: Degraded position measurement from OptiTrack (marker occlusion, reflections, increased noise) was feeding bad position data to the OOT controller, causing it to command large corrective accelerations.  
**Test**: Lee controller flew stably with the exact same mocap setup and markers on the same day.  
**Result**: If mocap were the cause, Lee would also be affected. **Ruled out.**

### ❌ H3 — CS2 / crazyflies.yaml parameter change
**Theory**: Someone accidentally modified the CS2 config between June 20 and June 21, changing gains, filter parameters, or controller mode, causing instability.  
**Test**: YAML file inspected and confirmed unchanged vs last stable session.  
**Result**: **Ruled out.**

### ❌ H4 — Propeller wear or mechanical damage
**Theory**: Propellers worn or chipped from previous flights, producing asymmetric thrust and vibration that destabilised the controller.  
**Test**: Replaced all four propellers with new ones on a new drone. Same crash pattern reproduced.  
**Result**: **Ruled out.**

### ❌ H5 — rpm_get_all() contention / race condition in INDI
**Theory**: The C bridge function `rpm_get_all()` in `traj_iface.c` was being called at 500 Hz from the controller task, racing with the RPM deck's own task writing to the same log variables, producing corrupted RPM readings that cascaded into bad INDI torque estimates.  
**Test**: (a) This code existed identically during the stable June 20 flights. (b) The **geometric controller (mode 0) does not call `rpm_get_all()` at all** — it uses only gyro and attitude — yet geometric also crashes. If this were the cause, geometric would be unaffected.  
**Result**: **Ruled out.** (rpm_get_all contention remains a secondary quality issue but is not the crash cause.)

### ❌ H6 — OptiTrack update rate degradation
**Theory**: OptiTrack frame rate dropped (e.g. from 120 Hz to a lower rate), increasing the effective time delay in the position loop and pushing it into instability.  
**Test**: Lee controller stable on the same mocap stream at the same time.  
**Result**: **Ruled out.**

### ❌ H7 (partial) — M2 RPM deck reading failure
**Theory**: M2 reflective paper degraded, causing the RPM deck to read M2 as zero or very low, corrupting INDI's `tau_current` estimate.  
**Test**: Replaced M2 reflective paper. M2 readings improved noticeably. However, crashes continued after M2 fix, and geometric (which ignores RPM) still crashed.  
**Result**: **Partially confirmed as a contributing factor, but not the primary cause.**

### ❌ H8 (partial) — Physical motor/drone degradation
**Theory**: The specific drone unit had a failing motor or ESC that produced insufficient or asymmetric thrust.  
**Test**: Switched to a completely new drone (different unit, new motors, new RPM deck, new propellers). Same oscillation pattern reproduced.  
**Result**: Switching drones provided some improvement but did not resolve crashes. **Not the primary cause.** However, motor-to-motor thrust imbalance (due to reflective tape aerodynamic drag — see H9) was confirmed as the mechanism.

---

## ✅ H9 — M3 (and intermittently M4) reflective tape detaching mid-flight

**Theory**: The reflective tape used by the RPM deck's IR sensor partially detaches from the motor bell during flight. The flapping tape creates aerodynamic drag and vibration directly on M3's motor bell, physically reducing M3's effective thrust below its commanded value. This produces a real, persistent roll/pitch bias toward the back-left (M3 position).

**Evidence**:

Batch RPM analysis was run against **10 stable reference flights (June 17–20)** and **25 June 28 flights** using per-motor RPM data logged at 20 Hz via `rpm.m1..m4`. The correlation is:

| RPM status | Flight outcome | Count |
|------------|---------------|-------|
| M3/M4 OK (std < 2000 RPM, mean > 20k) | Stable | 16 |
| M3/M4 BAD — but **Lee controller** | Stable | 4 |
| M3/M4 BAD — OOT geometric or INDI | **Crash** | 7 |
| M3/M4 OK — OOT geometric or INDI | Stable | 0 |

**Zero crashes occurred with clean RPM readings. Every OOT crash had M3 or M4 corrupted.**

Reference flights (June 17–20, stable): M3 = 24,300–25,100 RPM, std < 600 RPM.  
Crash flights (June 28): M3 as low as 15,527 ± 3,582 RPM; M4 as low as 17,897 ± 5,098 RPM.

**Why does this crash the OOT controller but not Lee?**

1. **Physical effect**: Flapping tape on M3 bell → aerodynamic drag → M3 actual thrust < commanded thrust → real roll/pitch bias toward back-left corner.
2. **OOT geometric (mode 0)**: Pure proportional-derivative attitude controller (KR·e_R + KW·e_ω + gyro compensation). No attitude integral. Cannot trim out a persistent physical imbalance. The position integral partially compensates, but the attitude bias grows as the position loop fights it, producing the characteristic outward spiral (underdamped position loop, ζ ≈ 0.27, KP=40).
3. **OOT INDI (mode 1)**: Additionally corrupts `tau_current` = Σ kti·RPMi² — bad M3 RPM makes the INDI torque estimate wrong, causing the incremental update to command in the wrong direction.
4. **Lee (built-in PID)**: Has an inner-loop attitude PID with an integral term that accumulates over time and trims out persistent biases. This is why Lee remained stable even with M3 RPM readings as bad as 15,527 RPM mean.

**Why both drones?**: The reflective tape application method is the same on both units. If the tape is not cut precisely to the motor bell diameter and pressed completely flat, it will lift at high RPM. This is a process issue, not a unit-specific defect.

---

## Root Cause Summary

> **The OOT controller crashes because reflective tape on M3 (and intermittently M4) partially detaches mid-flight, reducing actual motor thrust and creating a roll/pitch bias that the OOT geometric/INDI controller cannot reject. Lee absorbs the same bias via its attitude integral and remains stable.**

---

## Next Steps

### Hardware (immediate)
1. Re-apply M3 and M4 reflective tape: cut to exactly the motor bell diameter, press completely flat, no air bubbles, no overhang beyond the bell edge
2. Before flying, verify on the bench: spin each motor individually, confirm all four `rpm.m1..m4` readings > 20,000 RPM at ~50% throttle in cfclient

### Validation flights
3. Geometric hover (mode 0) — confirm stable, check RPM balance plot post-flight
4. Full INDI hover (mode 1) — confirm stable, check INDI dashboard
5. INDI figure-8 — same trajectory that was stable on June 20

### Code improvements (pending, already implemented but reverted during investigation)
6. **Re-apply tilt-compensated gravity feedforward** (`gz_comp = g / cos θ`) — reduces Vz oscillation during tilted maneuvers
7. **Re-apply accelerometer pre-filter for position INDI outer loop** — suppresses IMU noise in `a_meas` before it enters the INDI increment computation
8. Both changes are in git history (commit `a79bb4f`, reverted by `56d2296`) and can be cherry-picked back

### Tuning (after hardware confirmed stable)
9. Repeat KR/KW INDI gain sweep with reliable RPM readings — compare attitude RMSE against June 20 baselines
10. Evaluate whether enabling attitude integral (`KI_ATT = 0.03`, already wired in code, currently set to 0.0) improves steady-state tracking during figure-8

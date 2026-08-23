# Advanced Flying Robots — Source of Truth

> ## ⚠️ HISTORICAL
> This document describes the *course* project and its assignments. Its claim to supersede other files applies to the
> course era only.
>
> **The current source of truth for project status is
> [`docs/07_Thesis_Progress_Checklist.md`](../docs/07_Thesis_Progress_Checklist.md);
> [`docs/00_README.md`](../docs/00_README.md) maps every other document.** Kept because it
> records what was decided and why -- do not act on it as if it were current.

> **THIS FILE SUPERSEDES ASSIGNMENTS.md AND ROADMAP.md.**
> Check this file FIRST before reasoning about scope, requirements, or priorities.
> ASSIGNMENTS.md / ROADMAP.md are legacy docs from the basic course — do not use them
> to infer what is or is not in scope for the advanced phase.

---

## Project Overview

Building on the completed basic Flying Robots course (Assignments 1–4).
Now in the **advanced controls + motion planning** phase directed by supervisor meetings
and assigned papers.

Platform: Crazyflie 2.1 → progressing toward brushless (Bolt / Bolt 1.1).
Stack: Rust (`flying_drone_stack/`), `no_std` firmware OOT controller, real hardware flights.

---

## Supervisor Meetings

### Meeting — [~Apr/May 2026]
**Agreed:**
- Mode 1 planner: Richter et al. (2016) automatic time allocation → **DONE**
- Mode 2 planner: SE(3) explicit orientation planning → **DONE**
- INDI controller (3 modes: Geometric / Attitude INDI / Full INDI) → **DONE** (Mode 2 pending RPM deck)
- MoCap integration for state estimation → **PLANNED** (Phase 3)
- RPM deck integration to enable Full INDI (Mode 2) → **PLANNED** (Phase 2)

> TODO: Fill in exact date, full attendee list, and any additional items not captured here.

---

## Phases

### Phase 1 — Motion Planning (DONE ✅)

Three trajectory planning modes, all sharing one `TrajectoryPlanner` enum interface.
Switch with `PLANNER_MODE` constant in each binary — parallel to `CONTROLLER_MODE` in firmware.

| Mode | Planner | Key idea | Status |
|------|---------|----------|--------|
| 0 | `SplineTrajectory` | Degree-8 min-snap QP, manual segment durations | ✅ Done (basic course) |
| 1 | `RichterTrajectory` | Same QP, automatic timing from single `k_t` scalar | ✅ Done |
| 2 | `Se3Trajectory` | Min-snap position + SLERP attitude on SO(3); handles flips/rolls | ✅ Done |

**Mode 0/1**: attitude is a *consequence* of the position trajectory, recovered via
differential flatness: z_b = (a + g·e₃) / |a + g·e₃|

**Mode 2**: attitude is a *co-equal input* — prescribed as explicit quaternion waypoints
interpolated on SO(3). Bypasses the flatness singularity when thrust passes through zero.
Enables: full flips, rolls, banked turns, inverted hover.

Key files:
- `src/planning/spline.rs` — Mode 0
- `src/planning/richter.rs` — Mode 1 (6 tests)
- `src/planning/se3_traj.rs` — Mode 2 (7 tests)
- `src/planning/mod.rs` — `TrajectoryPlanner` enum, `eval(t)`, `eval_se3(t)`
- `src/bin/richter_figure8_test.rs` — real-hardware binary with PLANNER_MODE switch

---

### Phase 2 — INDI Controller + RPM Deck (IN PROGRESS 🔧)

Three controller modes switched with `CONTROLLER_MODE` constant in firmware.

| Mode | Controller | Torque source | Hardware needed | Status |
|------|-----------|--------------|-----------------|--------|
| 0 | SE(3) Geometric | Model-based | None extra | ✅ Done, flying |
| 1 | Attitude INDI | Gyro incremental (modelled τ_prev) | None extra | ✅ Implemented, needs flight validation |
| 2 | Full INDI | RPM² measurements (actual τ_current) | **RPM deck required** | ✅ Code done, blocked on hardware |

Full INDI replaces the modelled previous torque with the measured one:
  τ_current = G(Ω) · [Ω₁², Ω₂², Ω₃², Ω₄²]

This eliminates the motor model assumption, enables saturation-aware allocation, and
is essential for aggressive manoeuvres at high angular rates.

**Blockers for Mode 2:**
1. RPM deck physically fitted to drone
2. `KT` identified on thrust stand (currently placeholder `3.16e-10 N/RPM²`)
3. 4-line RPM sensor read wired into call site in `firmware_app/src/lib.rs`

Key files:
- `src/controller/indi.rs` — PC-side INDI (all 3 modes)
- `firmware_app/src/lib.rs` — onboard INDI, `compute_tau_from_rpm`, `full_indi_torque`

---

### Phase 3 — Motion Capture Integration (PLANNED 📋)

Replace optical flow + onboard EKF with MoCap (Vicon / OptiTrack / Qualisys) injected
via CRTP external pose into the firmware EKF.

**Why it matters:**
- Position accuracy: sub-mm vs cm–dm with optical flow
- Rate: 100–300 Hz vs ~10 Hz effective from flow
- Separates estimator failures from controller failures — essential for meaningful results
- Crazyflie firmware already supports external pose injection — no firmware changes needed

**Work needed:** PC-side MoCap CRTP bridge.

**Blockers:**
- Access to MoCap lab / system
- Decision on system (Vicon / OptiTrack / Qualisys)

---

### Phase 4 — Better Platform (PLANNED 📋)

Current CF 2.1: ~31g, T/W ≈ 5:1 — insufficient headroom for full flips at speed.
Target: Crazyflie Bolt / Bolt 1.1, T/W ≈ 8–12:1.
Firmware and software stack stays identical — hardware swap only.

---

### Phase 5 — Novel Contribution (FUTURE 🔭)

Enabled by Full INDI + MoCap + capable platform + systematic trajectory suite.

Candidates (from supervisor):
1. **Systematic controller comparison** — Geometric vs Attitude INDI vs Full INDI on
   identical trajectories with MoCap ground truth. Reproducible Crazyflie benchmark.
2. **Saturation-aware INDI torque allocation** — redistribute under motor limits while
   preserving incremental structure.
3. **Aggressive trajectory feasibility with INDI** — characterise headroom INDI buys
   vs model-based bounds; feed back into planner.
4. **Learned residual on INDI** — small learned correction for aerodynamic rotor
   interactions at high speed.

> TODO: Agree with supervisor which of these is the primary contribution target.

---

## Current Test Count

**466 passing, 0 failures** (as of 2026-05-01)

---

## Key Design Decisions

| Decision | Detail |
|----------|--------|
| `PLANNER_MODE` / `CONTROLLER_MODE` | Single constant per binary; nothing downstream changes |
| Mode 2 thrust can be negative | Physically correct for inverted flight; caller clamps to [0, T_max] — no clamp inside planner |
| Clarabel one-call rule | `plan()` makes exactly ONE QP call; `T_MIN = 0.15 s` |
| Full INDI RPM call site | Currently passes `[0.0; 4]` — correct placeholder until RPM deck fitted |

---

## Papers in Scope

| Paper | Technique | Status |
|-------|-----------|--------|
| Richter et al. (2016) — "Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Environments" | Automatic time allocation | ✅ Mode 1 |
| Mellinger & Kumar (2011) / Faessler et al. (2018) | Min-snap QP + differential flatness | ✅ Mode 0 |
| SE(3) decoupled attitude planning (Müller et al. 2011 style) | SLERP on SO(3), independent of flatness | ✅ Mode 2 |
| Smeur et al. (2016) — "Adaptive Incremental Nonlinear Dynamic Inversion" | INDI for attitude + full state | ✅ Implemented |

> TODO: Add any additional papers assigned by supervisor — exact titles, specific
> algorithms/sections, implementation status.

---

## What NOT to Use for Scope Decisions

- `ASSIGNMENTS.md` — basic course, Assignments 1–4, frozen
- `ROADMAP.md` — basic course roadmap, outdated
- `CLAUDE.md` — implementation cheat-sheet for code details only, not scope

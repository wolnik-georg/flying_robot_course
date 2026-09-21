# Experimental Protocol – Two-Robot Comparison

> **Where this sits:** protocol for **C.4 — Systematic Comparison** ([`07`](07_Thesis_Progress_Checklist.md)).
> **C.1** uses the same scenarios but collects under **geometric on cf5** ([`25`](25_C1_Data_Collection_Plan.md),
> [`13`](13_Residual_Learning.md)). **Living table:** [`24`](24_Downwash_Compensation_Comparison.md).
> **Desk index:** [`31`](31_Desk_Parallel_Track.md).
>
> **2026-09-21:** C.0 is closed enough to fly C.1; cf5 + cf_second, uSD end-to-end. C.4 still
> needs repeat flights and [`27`](27_Analysis_and_Metrics_Plan.md) aggregation.
>
> The formations below were written before the scenario library existed. They are now **realised
> by the frozen 16-scenario library** in [`10_Formation_Library.md`](10_Formation_Library.md) —
> see the mapping table under *Formations*. Fly the scenario IDs, not the prose descriptions:
> the library versions are spec-checked, safety-gated and sim-validated.

## Goal
Fair comparison of:
- Pure INDI
- Geometric + NN
- FBL + NN
- Hybrid (Neural-Augmented INDI / NA-INDI)

on two Crazyflie brushless drones in tight formation. We aim for up to 7 controllers if time allows (see `docs/01_Thesis_Project_Snapshot.md` §2 for the full list and target levels: **Minimum = Methods 1–4** (Pure INDI, Geometric+NN, FBL+NN, Hybrid/NA-INDI), **Advanced = Methods 5–7** (the residual-RL and learning-based-MPC strategies)).

**Phasing:** **C.1** collects under **geometric** on the study drone ([`25`](25_C1_Data_Collection_Plan.md))
so `a_res` labels are not cancelled by full INDI. **C.4** compares reactive (S0, S1), predictive
(S2), and hybrid (S4) on matched scenarios. Later campaigns add
**Geometric + NN**, **Hybrid/NA-INDI** (`stabilizer.controller=8`, not `7` — see
`docs/strategy_controller_map.html` for the corrected mapping; numerically verified vs the
reference's own compiled C, but **2026-09-16: crashes in CS2 closed-loop sim** on a plain
single-drone hover, same as `controller=7` — not ready, not a near-term fallback, needs its own
investigation before it can clear a hardware gate), and **FBL + NN** once the respective residual
models / controller code are ready.

## Hardware
- 2× Crazyflie brushless (identical configuration)
- OptiTrack @ 100 Hz + Crazyswarm2
- High-rate logging of states, commands, and motor signals

## Formations (Phase A)

**Primary:**
- Vertical stack
- Horizontal offset stack
- Side-by-side (close)

**Secondary / Multi-robot:**
- 3-drone vertical / I-stack or V-stack
- Triangle / diamond
- Leader-follower line
- Dynamic height exchange / swapping
- Close docking / very tight vertical approach
- Dynamic formation change (e.g. stack → side-by-side)

**Separations to test:** 40 cm → 30 cm → 25 cm → 20 cm (and tighter if stable)

### Mapping onto the frozen scenario library

These prose formations are implemented as the parameterised scenarios in
[`10_Formation_Library.md`](10_Formation_Library.md). Use the IDs — they carry the spec check, the
safety gate and the sim validation.

| Protocol formation | Library scenario |
|---|---|
| Vertical stack | **A1** (static Δz), **A3** (vertical stack in motion) |
| Horizontal offset stack | **A4** (lateral offset) |
| Side-by-side (close) | **A2**, **A7** |
| 3-drone stack / line | **B1**, **B2**, **B3** |
| Coplanar / no-interaction controls | **C1–C3** — measured **0.0 mm** in sim under both controllers. The null result is what makes the positive ones meaningful; keep them in the campaign |
| Coverage cases | **C4**, **C5** |

⚠️ **C.1 must include both vertical and lateral motion** (A1, A3 *and* A4, A7). The simulation dry
run showed A3 alone never excites relative `y`, which leaves the residual model extrapolating
across half its input space the first time a formation moves sideways.

## Trajectories
- Hover
- Horizontal circle / figure-8
- Vertical motion while maintaining formation
- Minimum-snap trajectories

## Primary Metrics

The three C.4 headline axes are **tracking error**, **residual rejection** and **robustness**.

| Axis | Measured by |
|---|---|
| **Tracking error** | Position RMSE (total and per axis); maximum vertical deviation; attitude RMSE |
| **Residual rejection** | Realised separation error against the *commanded* separation — downwash shows up as a **bias** (the lower vehicle sags), so report the mean, not only the spread. Plus `indi.a_res_*` magnitude |
| **Robustness** | Control effort; behaviour as separation tightens; failure/abort rate |

For every learned strategy, also log `rnn.pred_*` against `indi.a_res_*`. That comparison is the
model's own evaluation and it is available on **every** flight, including ones where the
prediction is not being used.

## Procedure

Controller selection is a **runtime parameter** — `indi_gains.ctrl_mode` in `crazyflies.yaml`
(0 geometric, 1 position INDI, 2 attitude INDI, 3 full INDI). No reflash between strategies, which
is what makes a same-day A/B comparison possible.

For each controller and each separation:
1. Take off and form the formation at safe distance
2. Reduce to target separation
3. Execute trajectory
4. Log data
5. Repeat ≥ 5 times

Safety: Human pilot always ready. Start with larger separations.

**Gains are frozen before C.1 and must not change during the campaign** (Checklist C in
[`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md)). Re-tuning between
strategies would measure tuning effort rather than the methods.

**Verify `a_res` is non-zero on the first flight of every session.** It reads exactly 0.0 without
an RPM source — that is the absence of a measurement, not a measurement of no interaction, and a
session logged that way produces nothing usable.

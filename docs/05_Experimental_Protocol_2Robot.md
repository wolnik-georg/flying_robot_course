# Experimental Protocol – Two-Robot Comparison

> **Where this sits:** this is the protocol for **C.4 — Systematic Comparison**, the last step of
> the Core Thesis Workflow in [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md).
> Its *Procedure* and *Metrics* also govern the flights in **C.1 — Residual Data Collection**,
> which run the same formations under pure Geometric control.
>
> **Nothing here can start until C.0 — the Hardware Gate — passes.**
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

on two Crazyflie brushless drones in tight formation. We aim for up to 7 controllers if time allows (see `docs/01_Thesis_Project_Snapshot.md` §2 for the full list and target levels: Minimum = Methods 1–3, Ideal = Methods 1–5, Super perfect = all 7).

**Phasing:** The first data-collection campaign uses **Pure INDI only** — it needs no learned
residual model and no FBL code, so it can start immediately. Later campaigns add
**Geometric + NN** and **FBL + NN** once the respective residual models / controller code are
ready.

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

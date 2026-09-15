# Thesis Project Snapshot
**Last updated:** 14 September 2026

> **Status:** software preparation is **finished**. The project now runs **two parallel tracks** —
> a lab track and a writing track. The formal problem statement and research questions are in
> `15`; this document holds the *idea*, `15` the
> *questions*, and `07` the *plan*.
>
> Lab track: next is **C.0 — Hardware Gate**, step 1 of the five-step Core Thesis Workflow in
> [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md), which is the master plan
> and the single source of truth for status. This document holds the *idea*; `07` holds the *plan*.

## 1. Core Idea of the Thesis

Systematically compare **reactive**, **predictive**, and (optionally) **hybrid** control strategies for compensating inter-vehicle aerodynamic interaction forces (mainly downwash) during tight multirotor formation flight.

- Hardware: Crazyflie brushless + OptiTrack + Crazyswarm2
- Core evidence must come from **real flights**
- Start with 2 robots → then extend to ≥ 3 robots
- Formations start from those used in the core papers, then can be tightened

**Main contribution**  
First systematic multi-robot head-to-head comparison of the controller families on the same hardware, quantifying when each approach is preferable (tracking error, residual force, control effort, robustness, computational cost).

## 2. Control Strategies to Compare (7 methods)

**2026-09-14: revised from a three-tier split (Minimum 1-3 / Ideal 1-5 / Super perfect 1-7) to
two tiers.** Strategy 4 (Hybrid/NA-INDI) moved from "Ideal" into "Minimum". Neural-Swarm2 is not
its own strategy; it is the NN used inside Strategy 2 (Geometric + NN).

**2026-09-15: controller mapping corrected.** `stabilizer.controller=7` is a faithful,
numerically-verified Rust port of Cobo-Briesewitz's `controller_lee.c` (~1e-9 vs their compiled
C), but their file's `use_nn` flag is dead code in every config either project has flown — so
`controller=7` as it exists TODAY runs their **plain INDI**, not NA-INDI. Their own paper's
comparison table treats INDI as one of its four baseline methods (Lee / INDI / LINDI / NA-INDI),
so this is a real, citable alternative *implementation of Strategy 1*, not a stretch or a
mislabelling — Strategy 1 now has two controller options. Getting genuine NA-INDI behaviour
(the network predicting the bulk of the residual, INDI correcting only the remainder) needs
`use_nn` enabled and a trained residual model in their convention wired into that same module —
unbuilt, tracked as a future `controller=8`, not started.

**Full reference page** (kept in sync with this table): `docs/strategy_controller_map.html`.

| # | Method | Family | Tier | Key Papers | Controller(s) |
|---|--------|--------|------|------------|----------------|
| 0 | Geometric baseline | Reactive | **Minimum** | — | `controller=6`, `ctrl_mode=0` |
| 1 | Pure INDI | Reactive | **Minimum** | Tal & Karaman, Smeur — *or* Cobo-Briesewitz et al. (their INDI baseline) | `controller=6` (ours, `ctrl_mode=3`) **or** `controller=7` (Cobo-Briesewitz INDI, `use_nn=0`) |
| 2 | Geometric + NN (Neural-Swarm2) | Predictive | **Minimum** | Neural-Swarm2, SO(2)/Aggregate | `controller=6` + `rnn.en=1` |
| 3 | FBL + NN | Predictive | **Minimum** | Flatness-Preserving Residual (Hsieh et al.) | not assigned — blocked on FBL code |
| 4 | Hybrid (Neural-Augmented INDI / NA-INDI) | Hybrid | **Minimum** | Cobo-Briesewitz et al. (core paper [1]), `use_nn=1` | `controller=8` — **not built** (flag + trained NN inside the existing ported module) |
| 5 | Residual RL (ProxFly-style) | Residual learning | **Advanced** | ProxFly | **open — not yet defined** |
| 6 | Light Learning-based MPC (residual-MPC / simplified KNODE-style) | Predictive + Optimisation | **Advanced** | KNODE-DW MPC (Chee et al.), L1 KNODE-DW MPC (Hsieh et al.) | **open — not yet defined** |
| 7 | Geometric + Residual RL | Residual learning | **Advanced** | ProxFly + Geometric literature | **open — not yet defined** |

**Target levels (two-tier, current):**
- **Minimum:** Methods 1–4 — the thesis's baseline claim; every strategy here either flies today
  or has a numerically-verified, unflown implementation
- **Advanced:** Methods 5–7 — the residual-RL and learning-based-MPC strategies, pursued only once Minimum
  is complete and data collection (C.1–C.4) is underway

Strategy 3 (FBL + NN) is Minimum-tier but its controller code is still with the FBL authors
(requested via the professor) — not blocking the other three, just not implementable yet.

## 2a. Formations (keep all of them)

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

## 3. Thesis Document Structure

1. Introduction  
2. Background & Related Work  
3. System Modelling & Residual-Force Formulation  
4. Control Architectures  
5. Experimental Setup & Protocol  
6. Results – Two-Robot Teams  
7. Results – Three-or-More-Robot Teams & Scaling  
8. Discussion  
9. Conclusion & Future Work

## 4. High-Level Timeline (1 Sep 2026 – end of February 2027)

- **September**: Literature matrix + residual model + code cleanup + protocol
- **October – mid-November**: Implement & tune 2-robot controllers
- **mid-November – mid-December**: 2-robot real-flight comparison
- **January**: Extend to ≥ 3 robots
- **late January – February**: Optional methods + intensive writing

## 5. Immediate Next Steps

**All planning, literature, modelling, protocol, tooling and simulation work is complete.** So is
the entire residual-learning software foundation: the onboard network, the weight-upload path, the
training pipeline, and an end-to-end dry run in simulation ([`13`](13_Residual_Learning.md)).
Nothing in software blocks progress.

FBL controller code (Strategy 3) was requested via the professor and is still with the authors.
**This does not block anything** — the other strategies proceed without it.

The next steps are the Core Thesis Workflow — see
[`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md) for the full version:

1. **C.0 — Hardware Gate** ⬅️ *next.* Inventory, flight-volume measurement, single-robot ladder,
   geometric and INDI both flying cleanly, the three unflown flight-code changes cleared,
   gains frozen
2. **C.1 — Residual Data Collection** under **pure Geometric** control, vertical *and* lateral
   (A1, A3, A4, A7)
3. **C.2 — Train the Residual Model** on real flight data; export and upload weights
4. **C.3 — Integrate the Strategies**
5. **C.4 — Systematic Comparison** across the frozen formation library

> ⚠️ **Correction to earlier plans.** An earlier version of this document said data collection
> would start with **pure INDI**. It starts with **pure Geometric** instead: INDI actively
> compensates the disturbance, so it would partly cancel the very residual being measured.
> Geometric does not compensate, so the residual is *observed* rather than suppressed — which is
> what a training set needs. Pure INDI remains Strategy 1 in the comparison at C.4.

# Thesis Project Snapshot
**Last updated:** 23 August 2026

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

| # | Method | Family | Priority | Key Papers |
|---|--------|--------|----------|------------|
| 1 | Pure INDI | Reactive | Core | Tal & Karaman, Smeur |
| 2 | Geometric + NN | Predictive | Core | Neural-Swarm2, SO(2)/Aggregate |
| 3 | FBL + NN | Predictive | Core | Flatness-Preserving Residual (Hsieh et al.) |
| 4 | Hybrid (Neural-Augmented INDI) | Hybrid | Optional | Cobo-Briesewitz et al. (core paper [1]) |
| 5 | Residual RL (ProxFly-style) | Residual learning | Optional | ProxFly |
| 6 | Light Learning-based MPC (residual-MPC / simplified KNODE-style) | Predictive + Optimisation | Extra | KNODE-DW MPC (Chee et al.), L1 KNODE-DW MPC (Hsieh et al.) |
| 7 | Geometric + Residual RL | Residual learning | Extra | ProxFly + Geometric literature |

**Target levels:**
- Minimum: Methods 1–3
- Ideal: Methods 1–5
- Super perfect: All 7

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

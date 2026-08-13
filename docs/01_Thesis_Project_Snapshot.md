# Thesis Project Snapshot
**Last updated:** 13 August 2026

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

1. Save all documentation files
2. Send Gap & Contribution Statement to mentors
3. Clean geometric + INDI code
4. Request FBL code via professor
5. Check hardware status and set up high-rate logging
6. Start residual-force data collection with pure INDI

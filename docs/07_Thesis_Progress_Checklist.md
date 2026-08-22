# Thesis Progress Checklist
**Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams**

**Last updated:** 19 August 2026

This is a living document. Update it regularly so we always know what has been completed and what comes next.

---

## A. Planning & Documentation

- [x] Understand official thesis topic and milestones (from thesis PDF)
- [x] Read core papers + most important additional papers
- [x] Create Literature Matrix
- [x] Write Gap & Contribution Statement
- [x] Derive Unified Residual-Wrench Model
- [x] Define control strategies to compare (Core + Optional)
- [x] Write Experimental Protocol (2-robot)
- [x] Define overall thesis chapter structure
- [x] Create project documentation in `docs/`
- [x] Clean and restructure the repository

**Status:** Completed

---

## B. Practical Preparation

- [x] Request FBL controller code via professor (waiting for authors)
- [x] Align planner/Crazyswarm2 polynomial format and use official upload path (legacy path preserved) — see `flying_drone_stack/docs/TRAJECTORY_UPLOAD_PATHS.md`
- [ ] Hardware inventory
- [ ] High-rate logging check
- [ ] Sanity-check geometric and INDI on brushless
- [ ] Re-validate Mode E in flight: hover → figure8 → circle, geometric then INDI, vs Mode D baseline
- [ ] Enable second drone in `crazyflies.yaml` (URI, initial_position, platform) + switch `indi_gains` to the brushless block + resolve radio bandwidth for 2 drones
- [ ] First 2-drone formation flight (`formation_flight --dry-run` first, vertical, start wide)
- [ ] Prepare first INDI residual-force data collection

**Status:** In progress

---

## C. Core Experimental Work

### C.1 Data & Models
- [ ] Collect residual-force data with pure INDI
- [ ] Train first NN residual models
- [ ] Validate residual model quality

### C.2 Controller Integration (7 methods)
- [ ] 1. Pure INDI
- [ ] 2. Geometric + NN
- [ ] 3. FBL + NN (once code is available)
- [ ] 4. (Optional) Hybrid (Neural-Augmented INDI)
- [ ] 5. (Optional) Residual RL (ProxFly-style)
- [ ] 6. (Extra) Light Learning-based MPC (residual-MPC / simplified KNODE-style)
- [ ] 7. (Extra) Geometric + Residual RL

### C.3 Experiments
- [ ] Systematic 2-robot comparison (INDI vs Geometric+NN vs FBL+NN)
- [ ] Extend best methods to ≥ 3 robots
- [ ] Analyse results and compute metrics

**Status:** Not started

---

## D. Writing & Finalisation

- [ ] Chapter 1 – Introduction
- [ ] Chapter 2 – Background & Related Work
- [ ] Chapter 3 – System Modelling & Residual-Force Formulation
- [ ] Chapter 4 – Control Architectures
- [ ] Chapter 5 – Experimental Setup & Protocol
- [ ] Chapter 6 – Results (2-robot)
- [ ] Chapter 7 – Results (≥ 3 robots)
- [ ] Chapter 8 – Discussion
- [ ] Chapter 9 – Conclusion & Future Work
- [ ] Final polishing and defense preparation

**Status:** Not started

---

## Quick Current Focus

**Right now:** Finish remaining items in B, then start residual-force data collection with pure INDI. FBL code is not a blocker.  
**Next major milestone:** Start residual-force data collection with pure INDI

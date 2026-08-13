# Thesis Progress Checklist
**Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams**

**Last updated:** 13 August 2026

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

- [ ] Request FBL controller code via professor
- [ ] Hardware inventory (number of working Crazyflie brushless drones, decks, batteries, firmware)
- [ ] Verify / set up high-rate logging (states, motor commands, residual signals)
- [ ] Confirm geometric and INDI controllers still run correctly on the brushless platforms
- [ ] Prepare first residual-force data collection flights (using pure INDI)

**Status:** Next up

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

**Right now:** Finish Practical Preparation (Section B)  
**Next major milestone:** Start residual-force data collection with pure INDI

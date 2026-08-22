# Thesis Progress Checklist
**Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams**

**Last updated:** 22 August 2026

**This is the single source of truth for project status.** It is the one file to read to know what
is done, where we are, and what comes next. Keep it current — see *Keeping this current* at the
bottom.

---

## ▶ WHERE WE ARE

| | |
|---|---|
| **Phase** | B — Practical Preparation |
| **Done** | All software. Mode E migration, multi-robot script, repo/branch cleanup, docs. |
| **Next action** | **B.2 step 1 — flash the firmware, then fly the validation ladder** |
| **Real gate** | B.2 step 3 (figure8 on Mode E). Everything is offline-verified; nothing has flown. |
| **Hard blocker** | B.3 step 5 (hardware inventory) — gates everything multi-drone |
| **Not blocking** | FBL code (Method 3) still with the authors; the other methods proceed without it |

---

## A. Planning & Documentation — ✅ COMPLETE

- [x] Understand official thesis topic and milestones (from thesis PDF)
- [x] Read core papers + most important additional papers
- [x] Create Literature Matrix
- [x] Write Gap & Contribution Statement
- [x] Derive Unified Residual-Wrench Model
- [x] Define control strategies to compare (7 methods)
- [x] Write Experimental Protocol (2-robot)
- [x] Define overall thesis chapter structure
- [x] Create project documentation in `docs/`
- [x] Clean and restructure the repository

---

## B. Practical Preparation — 🔄 IN PROGRESS

### B.1 Software — ✅ COMPLETE

- [x] Request FBL controller code via professor *(waiting on authors — not a blocker)*
- [x] Mode D → Mode E migration: official Crazyswarm2 upload path, Mode D preserved byte-identical — [`08_Trajectory_Upload_Paths.md`](08_Trajectory_Upload_Paths.md)
- [x] Multi-robot flight script `formation_flight.py` *(offline-verified, never run)*
- [x] Continuous multi-lap support (`--laps`) and rest-to-rest on all closed loops incl. figure8
- [x] Single body-frame convention (Mellinger) on Mode E; Mode D pinned to its original
- [x] Repository/branch cleanup — merged to `main`, 13 stale branches removed, stable/finalized preserved
- [x] Documentation + memory updated
- [x] **Log the residual** `indi.a_res_{x,y,z}` = `a_meas - a_model` = f_res/m — the core thesis measurement. Recorded in ALL controller modes, so downwash is measurable while flying geometric too
- [x] Multi-drone analyser `experiments/analysis/analyze_formation.py` (separation + residual force + downwash report)
- [x] Shared clock across per-drone loggers, so residual and relative position are correlatable

### B.2 Flight-validate Mode E — single drone ⬅️ **NEXT**

| # | Task | Command | Pass criterion |
|---|---|---|---|
| 1 | Flash firmware | `cd flying_drone_stack/firmware_app && make cload` | Boots, hovers normally |
| 2 | figure8 **Mode D** | `flight -- --trajectory figure8 --mode 1 --kt 0.05 --onboard` | Matches the existing baseline |
| 3 | figure8 **Mode E** | same, **without** `--onboard` | **RMSE ≈ step 2** |
| 4 | circle Mode E | `flight -- --trajectory circle --mode 1 --kt 0.1` | No swing-out at start |

- [ ] 1. Flash firmware
- [ ] 2. figure8 Mode D — baseline
- [ ] 3. figure8 Mode E — **the migration test**
- [ ] 4. circle Mode E — rest-to-rest confirmed

> If 3 does not match 2, **stop and investigate** — the offline analysis would be incomplete.
> figure8 is the cleanest comparison because it isolates the mode switch.

### B.3 Hardware configuration for two drones

- [ ] 5. **Hardware inventory** — working brushless drones, decks, batteries *(blocks everything below)*
- [ ] 6. Switch `crazyflies.yaml` `indi_gains` to the **brushless** block *(currently upgraded CF2.1, `mass 0.0386`)*
- [ ] 7. Fill the three `cf_second` TODOs — real URI, real `initial_position`, platform
- [ ] 8. Resolve radio bandwidth — 6 topics × 100 Hz × 2 drones saturates one Crazyradio *(2nd dongle, or lower rates)*
- [ ] 9. Set `cf_second: enabled: true`
- [ ] 10. High-rate logging check — states, motor commands, residual signals. **Confirm `indi.a_res_*` is non-zero in flight** (it needs an RPM source: `rpm.m*` deck or DShot `motor.m*_rpm`); zero means no telemetry and no thesis data

### B.4 First multi-drone flights

- [ ] 11. Dry run — `formation_flight -- ... --dry-run` *(prints the plan, commands nothing)*
- [ ] 12. First 2-drone flight — `--formation vertical --separation 0.6` *(start wide)*
- [ ] 13. Walk separation down — 0.6 → 0.4 → 0.3 → 0.25 → 0.2 m

---

## C. Core Experimental Work — ⬜ NOT STARTED

### C.1 Data & Models
- [ ] Collect residual-force data with pure INDI
- [ ] Train first NN residual models
- [ ] Validate residual model quality

### C.2 Controller Integration (7 methods)
- [ ] 1. Pure INDI
- [ ] 2. Geometric + NN
- [ ] 3. FBL + NN *(once code is available)*
- [ ] 4. (Optional) Hybrid — Neural-Augmented INDI
- [ ] 5. (Optional) Residual RL — ProxFly-style
- [ ] 6. (Extra) Light Learning-based MPC — residual-MPC / simplified KNODE
- [ ] 7. (Extra) Geometric + Residual RL

### C.3 Experiments
- [ ] Systematic 2-robot comparison (INDI vs Geometric+NN vs FBL+NN)
- [ ] Extend best methods to ≥ 3 robots
- [ ] Analyse results and compute metrics

---

## D. Writing & Finalisation — ⬜ NOT STARTED

- [ ] Ch. 1 Introduction
- [ ] Ch. 2 Background & Related Work
- [ ] Ch. 3 System Modelling & Residual-Force Formulation
- [ ] Ch. 4 Control Architectures
- [ ] Ch. 5 Experimental Setup & Protocol
- [ ] Ch. 6 Results — 2-robot
- [ ] Ch. 7 Results — ≥ 3 robots
- [ ] Ch. 8 Discussion
- [ ] Ch. 9 Conclusion & Future Work
- [ ] Final polishing and defence preparation

---

## Keeping this current

Update this file **in the same change** that alters project state — not afterwards. Specifically:

| When | Do |
|---|---|
| A task is finished | Tick its box **and** move the ▶ *Next action* line |
| A flight happens | Record the outcome under the relevant step; if it failed, say so and why |
| Something is descoped | Strike it with a one-line reason — never delete silently |
| A blocker changes | Update ▶ *Hard blocker* |
| Anything is added | Add the task **and** a History line below |

Rules that keep it trustworthy:

- **Never tick a box for something that has not actually been verified.** Code that compiles is not code that flew — B.1 items are marked complete as *software*, and B.2 exists precisely because none of it has flown.
- Keep ▶ WHERE WE ARE to six lines. If it grows, the detail belongs in a section.
- One source of truth: put status here, engineering detail in [`08_Trajectory_Upload_Paths.md`](08_Trajectory_Upload_Paths.md) or the relevant doc, and link.

---

## History

| Date | Change |
|---|---|
| 2026-08-22 | Mode E migration complete and merged to `main` (both repos). `formation_flight.py` added. `--laps`, figure8 rest-to-rest, single frame convention. 13 stale branches deleted, stable/finalized preserved. Docs + memory updated. B.2–B.4 laid out as the operational ladder. |
| 2026-08-19 | Chee et al. (arXiv:2410.09727) read → Method 6 reference. FBL code requested, pending. Section A closed out. |
| 2026-08-13 | Control strategies expanded to 7 methods; formation list broadened. |
| 2026-08-12 | Thesis documentation created (`docs/00`–`07`); repository restructured, course material archived. |

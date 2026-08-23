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
| **Next action** | **Hardware readiness — see ◆ PREPARATION OVERVIEW below.** All simulation work is complete |
| **Done** | ✅ Sim formation validation complete — **34/34 cases, 33 pass, 1 expected, 0 defects** ([`12_Sim_Formation_Validation_Report.md`](12_Sim_Formation_Validation_Report.md)). All 16 scenarios fly under BOTH controllers. Found and fixed **two flight-code bugs** neither of which single-drone flight could catch |
| **New** | All 4 control modes run in the CS2 simulator with the downwash model — [`09_Simulation.md`](09_Simulation.md). Sim can disprove a controller, not validate one |
| **Flight volume** | x −1..1, y −2..2, z 0.30..1.70 — applied and **corroborated** by an independent statement from 2026-07-27 (z, x identical; y then ±2.1). Still not tape-measured. With `--auto-center --rotate 90` **all 19 configs fit at full default parameters** |
| **Open (sim), time-boxed** | Sim needs more position damping than hardware: wall now `kv_xy` 8 (was 10) vs hardware 5. Motor lag closed ~40%; KI_P, rotor drag, inertia, airframe all refuted. SIM-ONLY `kv_xy: 8.0`; hardware keeps 5.0. **Investigation paused — hardware readiness is the priority** |
| **Flag for the lab** | Hardware crashed 2/2 at `kv_xy=4` and flies at 5 — a thin margin, and formation flight adds downwash to that same loop |
| **Fragile** | The sim depends on ~110 uncommitted lines in `crazyflie-firmware/bindings/` (upstream tree, deliberately not forked). Patch preserved at `firmware_app/host/cffirmware_bindings.patch` — re-apply if the sim stops building |
| **Real gate** | B.2 step 3 (figure8 on Mode E). Everything is offline-verified; nothing has flown. |
| **Hard blocker** | B.3 step 5 (hardware inventory) — gates everything multi-drone |
| **Not blocking** | FBL code (Method 3) still with the authors; the other methods proceed without it |
| **Verify in the lab** | uSD sync is *predicted* at a few ms — `merge_usd_logs.py` prints the measured value; check it on the first 2-drone flight |

### Architecture decisions (fixed 2026-08-22)

| Decision | Consequence |
|---|---|
| **NN inference runs ONBOARD; training is offline** | Every controller — geometric, hybrid, learning-based — runs on the brushless Crazyflie. Needs an MLP in `no_std` Rust + a weight-upload path (thesis C.2). Peer position is **already available onboard** via `peerLocalizationGetPositionByID()`, so the NN's main input is free. |
| **uSD logging is the dataset; radio is for monitoring** | Radio cannot carry 2 drones at full rate. 500 Hz onboard per drone, independent of radio — `tools/README_usd_thesis_logging.md`. Requires a Micro SD deck per drone. |

---

---

## ◆ PREPARATION OVERVIEW — everything before the thesis core

One table for "what is actually ready". Nothing in the **Done** rows still needs work;
everything in **Left** stands between here and collecting the first real dataset.

### ✅ Done

| # | Area | What exists | Evidence |
|---|---|---|---|
| 1 | Planning & literature | 7 control strategies defined, literature matrix, gap statement, residual-wrench model, 2-robot protocol, chapter structure | [`01`](01_Thesis_Project_Snapshot.md)–[`06`](06_References_Overview.md) |
| 2 | Trajectory path | Mode D → Mode E migration; Mode D frozen byte-identical; `--laps`, rest-to-rest closed loops, one frame convention | [`08`](08_Trajectory_Upload_Paths.md) |
| 3 | **Residual measurement** | `indi.a_res_*` = f_res/m, logged in **every** controller mode | §5 of [`12`](12_Sim_Formation_Validation_Report.md) |
| 4 | Multi-drone execution | `formation_flight.py` (shared trajectory) + `run_formation.py` (per-robot trajectories), staged takeoff, safety gate | [`10`](10_Formation_Library.md) |
| 5 | Logging | uSD 500 Hz × 34 vars, broadcast start so per-drone logs share an origin, `merge_usd_logs.py` measures the real offset | — |
| 6 | Analysis tooling | `analyze_formation.py`, `verify_formation_sim.py`, `summarise_matrix.py` (completeness audit) | — |
| 7 | **Simulation** | Geometric + all 3 INDI modes run as the *same Rust source that flies*, with the Neural-Swarm2 downwash model | [`09`](09_Simulation.md) |
| 8 | **Formation library** | 16 scenarios (A1–A8, B1–B3, C1–C5), spec-checked, safety-gated | [`10`](10_Formation_Library.md) |
| 9 | **Formation validation in sim** | **34 cases = 16 scenarios × 2 controllers. 33 pass, 1 expected, 0 defects** | [`12`](12_Sim_Formation_Validation_Report.md) |
| 10 | Flight-code bugs found & fixed | Residual sign (INDI was *doubling* disturbances); `a_res` dead under geometric (blocked Geometric+NN data) | §6 of [`12`](12_Sim_Formation_Validation_Report.md) |
| 11 | Lab volume applied | x −1…1, y −2…2, z 0.30…1.70; all 19 configs fit via `--auto-center --rotate 90` | [`11`](11_Hardware_Readiness_Checklist.md) |
| 12 | Docs & memory | This file, 08–12, external review brief, memory current | — |
| 13 | **Firmware local modifications — RESOLVED** | Decision 2026-08-23: kept as local, intentional, **do not touch**. All four files annotated in place; durable manifest written | [`LOCAL_MODIFICATIONS.md`](../flying_drone_stack/firmware_app/host/LOCAL_MODIFICATIONS.md) |
| 14 | **Hardware inventory checklist — PREPARED** | Vehicles, decks, power, logging, environment — tick only after a power-on check | [`11` Checklist A](11_Hardware_Readiness_Checklist.md) |
| 15 | **C.0 acceptance criteria — PREPARED** | 6 required flights, log checks, explicit pass/fail for the two unflown fixes | [`11` Checklist B](11_Hardware_Readiness_Checklist.md) |
| 16 | **Freeze-the-gains criteria — PREPARED** | Required flights, acceptance metrics, what is frozen, the one legitimate exception | [`11` Checklist C](11_Hardware_Readiness_Checklist.md) |
| 17 | **Residual collection order — DECIDED** | A1 (0.75→0.50) → A3 (0.40→0.30) → A4 (0.60, offset 0.10), under geometric | [`11`](11_Hardware_Readiness_Checklist.md) |

### ⬜ Left before the core work — **all lab-only**

Every remaining item is a measurement, a decision made at the bench, or a flight. **Nothing here
is software.**

| # | Task | Type | Why it blocks |
|---|---|---|---|
| 1 | **Tape-measure the flight volume** | Lab measurement | Current numbers are an estimate; scenarios sit within ~10 cm of the walls |
| 2 | Hardware inventory | Lab check | Checklist A — tick only after a power-on check |
| 3 | Flash firmware, confirm gains read back | Bench | `kv_xy` must be **5.0**, not the simulator's 8.0 |
| 4 | **Single-robot ladder** | Flight | **Mode E has never flown.** figure8 Mode D → figure8 Mode E → circle |
| 5 | **C.0 — validate the two unflown fixes** | Flight | Checklist B. No data collected before this passes counts |
| 6 | Two-robot at large separation | Flight | A1 at Δz 0.75 → 0.50, then A3 |
| 7 | Confirm `a_res` non-zero and correctly signed | Flight | Zero means no thesis data at all |
| 8 | Check measured uSD sync | Flight | The few-ms figure is predicted, never measured on real logs |
| 9 | **Freeze the gains** | Decision | Checklist C. Re-tuning later would measure tuning effort, not the methods |

> **Ordering is a constraint, not a suggestion.** Items 5 onward must not be attempted until the
> preceding ones are clean — the single-robot rungs exist to separate "the migration broke
> something" from "the interaction broke something". If a two-robot flight is the first thing that
> fails, those two causes are indistinguishable.

### 🔎 Open, tracked, not blocking

| Item | State |
|---|---|
| Sim needs ~2× hardware's position damping | Time-boxed and paused. Motor lag closed ~40%; `KI_P`, rotor drag, inertia, airframe all refuted |
| Library coverage gaps | Yaw fixed at zero; no vertical relative motion except A7/C4; nothing above 3 robots |
| External review | Brief sent for independent check of scenario-set completeness |

**Then the core begins:** collect residual data → train the NN residual models → integrate the
7 methods → run the systematic 2-robot comparison → extend to ≥3 robots.


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
- [x] Switch `crazyflies.yaml` to the **brushless CF21BL** gains (upgraded block preserved, commented)
- [x] uSD logging config + guide for the thesis dataset (`tools/usd_thesis_config.txt`)
- [x] Documented 1 ↔ N drone switching in one place (`crazyflies.yaml` header + multi-drone log note)
- [x] uSD logging started by **broadcast** (`allcfs.setParam`) so per-drone logs share an origin
- [x] **Formation scenario library** — 16 scenarios (A1–A8, B1–B3, C1–C5) covering the downwash geometries from the interaction-force literature, each with parameterised separations, a specification check and a safety gate. Per-robot trajectories on the stock Crazyswarm2 upload path — [`10_Formation_Library.md`](10_Formation_Library.md)
- [x] `tools/merge_usd_logs.py` — merges per-drone uSD logs, measures the actual offset/drift, emits relative state + f_res. Self-tested against known shifts
- [x] **Sim formation validation matrix** — all 16 scenarios (A1–A8, B1–B3, C1–C5) × both controllers, automated commanded-vs-realised geometry check. 33/34 pass; B1 under geometric is recorded as EXPECTED — a correct measurement of downwash, not a defect. Coplanar control cases C1/C2/C3 read 0.0 mm, which is what makes the positives meaningful
- [x] **`a_res` sign fix** — position INDI was ADDING the residual, reinforcing every unmodelled force instead of rejecting it. Measured at exactly 2.00x on a known disturbance; now cancels it to 0.0 mm. Invisible in single-drone flight, where `a_res` is ~0
- [x] **`a_res` measurable under geometric** — RPM was gated on `mode != 0`, so the residual read exactly zero on every geometric flight. That silently blocked Geometric+NN training data
- [x] **Sim verification infrastructure** — `run_formation` writes a sidecar JSON with the exact sim-clock trajectory start (needed because a hover scenario gives no recoverable window), and `experiments/analysis/verify_formation_sim.py` checks the `record_states` output against the commanded geometry. Replaces the runner's own report, which cannot work in sim: the radio log topics do not exist there and `/pose` is published only by the hardware server
- [x] **3-robot sim roster** — `crazyflies_sim3.yaml`, three floor-start drones. `crazyflies_sim.yaml` untouched so the 2-drone downwash test still works
- [x] **Thesis controllers run in simulation** — the same Rust firmware source, via SIL, selectable with `sim.oot_ctrl_mode` (0/1/2/3). Two-drone downwash reproduced in ROS under both geometric and full INDI (geo −30.5/−2.9 mm, INDI −58.8/−6.2 mm; the ~10x lower/upper asymmetry is the downwash signature, but the geo-vs-INDI difference is not yet a result). Five simulator fidelity bugs fixed first (missing accelerometer, wrong compiled airframe, mismatched motor calibrations, controller called at 2 kHz with a ms tick, and `firmware_params` never applied so the sim flew gains nobody flies) — [`09_Simulation.md`](09_Simulation.md)

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

### B.2b What is DONE in software — nothing here blocks you

| Area | What exists |
|---|---|
| Trajectory | Mode E on the official Crazyswarm2 path; Mode D frozen byte-identical |
| | Rest-to-rest on all closed loops incl. figure8; `--laps` continuous multi-lap |
| | One frame convention (Mellinger) on Mode E |
| Measurement | `indi.a_res_*` — the residual force, logged in **every** controller mode |
| Multi-drone | `formation_flight.py` — N drones, formations, safety checks, dry-run |
| Logging | uSD config (500 Hz x 34 vars); broadcast start so logs share an origin |
| Analysis | `analyze_formation.py` (separation + downwash), `merge_usd_logs.py` (merge + measured sync) |
| Config | Brushless gains active; 1 <-> N drone switching documented |
| Formations | 16 scenarios, offline-verified (38 spec cases, 20 safety combos). ROS-sim execution matrix **in progress** — see `experiments/sim_validation/` |
| Sim verification | Automated commanded-vs-realised geometry check, no hardware `/pose` needed |
| Simulation | All 4 control modes + Neural-Swarm2 downwash, same source as the drone; airframe derived from firmware so plant and controller cannot diverge |
| Housekeeping | Merged to `main`, branches cleaned, frozen branches preserved, docs + memory current |

### B.3 Hardware configuration for two drones

- [ ] 5. **Hardware inventory** — working brushless drones, decks, batteries *(blocks everything below)*
- [ ] 6. ~~Switch `indi_gains` to brushless~~ — **done 2026-08-22** (mass 0.041, kr 2400, kw 170)
- [ ] 6b. **Decide `clamp_en`** — currently `11` = tilt clamp OFF, set for inverted-loop work. Nothing in hover/figure8/circle needs it off. Decide before the first 2-drone flight
- [ ] 7. Fill the three `cf_second` TODOs — real URI, real `initial_position`, platform
- [ ] 8. Fit a Micro SD deck per drone + install `tools/usd_thesis_config.txt` as `config.txt`; drop radio log rates to 20 Hz for 2-drone flights
- [ ] 9. Set `cf_second: enabled: true`
- [ ] 9b. **Re-check `clamp_en`** against what the sim now shows — the tilt clamp is off (`11`) from the inverted-loop work and nothing in the planned flights needs it off
- [ ] 10. High-rate logging check — states, motor commands, residual signals. **Confirm `indi.a_res_*` is non-zero in flight** (it needs an RPM source: `rpm.m*` deck or DShot `motor.m*_rpm`); zero means no telemetry and no thesis data

### B.4 First multi-drone flights

- [ ] 10a. **Work through [`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md)** — Stage 0 bench items, then the single-robot ladder, then 2 robots at large separation only
- [ ] 10b. **Confirm the flight volume with a tape measure** (currently an estimate: x −1..1, y −2..2, z 0.30..1.70) and set `FLIGHT_SPACE` in `formations/safety.py` (currently a deliberately-too-small placeholder, so scenarios are refused rather than flown into the netting)
- [ ] 11. Dry run — `formation_flight -- ... --dry-run` *(prints the plan, commands nothing)*
- [ ] 12. First 2-drone flight — `--formation vertical --separation 0.6` *(start wide)*
- [ ] 12b. Then walk the formation library: **A1 → A3 → A2 → A8 → A4 → A5**, wide separations first. B1–B3 need a third drone; A6/A7/C4 are `--allow-extreme` and come last
- [ ] 13. Walk separation down — 0.6 → 0.4 → 0.3 → 0.25 → 0.2 m
- [ ] 14. **Check the real uSD sync number** that `merge_usd_logs.py` prints — the few-ms figure is predicted from broadcast jitter and clock drift and has only been validated against synthetic data

### B.5 Verify, then FREEZE the gains

- [ ] 15. Confirm the locked brushless gains still track in formation
- [ ] 16. **Freeze them.** Re-tuning per controller or per separation would measure tuning effort rather than the compensation methods, and invalidate the comparison

---

## C. Core Experimental Work — ⬜ NOT STARTED

Gated behind the whole of §B. Theoretical writing (problem statement, related work) can run in
parallel and is not listed here.

### C.0 — Hardware validation that nothing is broken

The first core task is not an experiment, it is a check. Two things changed in flight code that
have never flown.

- [ ] Geometric SE(3) flies cleanly on hardware — single robot, then large-separation two robots
- [ ] Corrected full INDI flies cleanly — **the residual sign fix inverts a control term**
- [ ] Explicitly confirm the sign fix introduced no new instability
- [ ] Log and inspect `a_res` under **both** controllers; confirm magnitude, sign and noise

### C.1 — Residual data collection

- [ ] Fly the validated scenarios under **geometric** control while logging `a_res`, states and
      rotor speeds. Geometric is the right choice here: it does not compensate, so the residual is
      observed rather than partly cancelled
- [ ] Start with A1, A3, A4, A7 at safe separations
- [ ] Stage progressively smaller separations only once large-separation flights are clean
- [ ] This is the training set for **every** residual-learning method

### C.2 — Train the models and integrate the seven strategies

- [ ] Train the neural residual models on the collected data
- [ ] Validate residual model quality before integrating
- [ ] Integrate all seven strategies into the same codebase:
      1. Pure INDI · 2. Geometric + NN · 3. FBL + NN *(code pending from authors)* ·
      4. Hybrid neural-augmented INDI · 5. Residual RL (ProxFly-style) ·
      6. Light learning-based MPC · 7. Geometric + residual RL
- [ ] *(When adding any stateful controller: give it a per-vehicle state swap, or multi-drone sim
      silently shares its filters — see [`09_Simulation.md`](09_Simulation.md))*

### C.3 — The comparison campaign

- [ ] Systematic 2-robot comparison under the frozen formation library
- [ ] Extend the best methods to ≥ 3 robots
- [ ] Analyse and compute the protocol metrics

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
| 2026-08-23 (4) | Remote decisions closed. Firmware local modifications resolved — kept as local by decision, all four annotated in place, durable manifest at `host/LOCAL_MODIFICATIONS.md`; found that `usddeck.c`'s raised log-variable limit is load-bearing for the 34-variable thesis config and would silently truncate if lost. Three checklists prepared (hardware inventory, C.0 acceptance, freeze-the-gains) and the residual collection order decided: A1 → A3 → A4 under geometric. |
| 2026-08-23 (3) | External review returned. Confirmed the assessment and found no additional scenario gaps beyond the five already documented — a weak positive, not proof of completeness. Contributed the Core Task sequencing, now §C.0–C.3, with hardware validation of the two unflown flight-code fixes as an explicit gate before any data collection. |
| 2026-08-23 (2) | Validation extended to the full library: 34/34 cases, 33 pass, 1 expected, 0 defects. A6/A7/C1–C5 run for the first time (C5 needed a 1-drone roster). Flight volume applied and corroborated; scenario rotation added so all 19 configs fit the real room at full parameters. Shareable report artifact published. |
| 2026-08-23 | Sim formation validation completed: 20/20 cases, 19 pass. Two flight-code bugs found and fixed — the residual sign (position INDI reinforced disturbances at exactly 2.00x instead of rejecting them) and `a_res` being dead under geometric (blocking Geometric+NN training data). Neither is detectable in single-drone flight. INDI now reduces residual displacement by 2-3 orders of magnitude in sim; not a thesis result until hardware confirms. |
| 2026-08-22 (8) | Sim execution gaps addressed. Infrastructure: sidecar JSON + `verify_formation_sim.py` (automated commanded-vs-realised check that works without hardware `/pose`), `crazyflies_sim3.yaml` for the 3-robot scenarios, `run_sim_matrix.sh` harness. Full 20-run matrix (Priority A × 2 robots, Priority B × 3 robots, each under geometric and INDI) launched. First result: A1/geometric PASS at 27.6 mm, which is the downwash sag rather than tracking error. |
| 2026-08-22 (7) | Damping investigation time-boxed and paused. KI_P refuted (bit-identical with the integral off); motor lag added to the plant and closes ~40% of the gap (wall 10 -> 8); rotor drag and airframe refuted. Config policy frozen: sim `kv_xy` 8.0, hardware 5.0, cross-warnings in both files. `11_Hardware_Readiness_Checklist.md` added. |
| 2026-08-22 (6) | Found that the OOT controller cannot track HLC trajectories in simulation (stock controllers can), which blocks sim validation of moving formation scenarios. Pre-existing and unrelated to the formation work. CSV coefficient precision fixed along the way (`%.6f` -> `%.12g`); `export_poly4d.rs` has the same latent bug at 3.00 s pieces. |
| 2026-08-22 (5) | Formation scenario library added: 16 scenarios, one curve→Poly4D compiler, spec check (38 cases) and safety gate (31 combinations, extreme scenarios gated). Sim-validated; same command runs on hardware. `10_Formation_Library.md` added, including an explicit assessment of what the set still does not cover. |
| 2026-08-22 (4) | `crazyflie-firmware` deliberately NOT forked; its ~110 bindings lines kept as a patch in this repo with `host/README.md` covering re-apply, since that tree follows bitcraze upstream. |
| 2026-08-22 (3) | Geometric + all INDI modes wired into the CS2 simulator through the existing SIL bindings, running the same Rust source that flies. Five plant/config fidelity bugs found and fixed, each of which had looked like a controller fault — the last being that `crazyflies.yaml` firmware_params were never pushed to the simulated controller. Two-drone downwash reproduced under both geometric and full INDI with the correct 10x lower/upper asymmetry. `09_Simulation.md` added, including what the simulator still cannot show (no motor lag, no EKF, no noise). |
| 2026-08-22 (2) | Decisions fixed: NN inference onboard / training offline; uSD is the dataset. Brushless gains activated. uSD config + 1↔N drone switching documented. Found peer position is already available onboard, so the NN input needs no new comms. |
| 2026-08-22 | Mode E migration complete and merged to `main` (both repos). `formation_flight.py` added. `--laps`, figure8 rest-to-rest, single frame convention. 13 stale branches deleted, stable/finalized preserved. Docs + memory updated. B.2–B.4 laid out as the operational ladder. |
| 2026-08-19 | Chee et al. (arXiv:2410.09727) read → Method 6 reference. FBL code requested, pending. Section A closed out. |
| 2026-08-13 | Control strategies expanded to 7 methods; formation list broadened. |
| 2026-08-12 | Thesis documentation created (`docs/00`–`07`); repository restructured, course material archived. |

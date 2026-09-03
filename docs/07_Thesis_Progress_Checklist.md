# Thesis Progress Checklist
**Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams**

**Last updated:** 24 August 2026

**This is the single source of truth for project status.** It is the one file to read to know what
is done, where we are, and what comes next. Keep it current — see *Keeping this current* at the
bottom.

---

## ▶ WHERE WE ARE

> ### 🏁 **Software preparation is FINISHED. The next actionable phase is the Hardware Gate (C.0).**
>
> Everything that could be built, tested or verified away from the lab has been. Nothing on the
> critical path is a software task any more. The project is now waiting on a drone.

| | |
|---|---|
| **Phase** | **Transitioning: Preparation (complete) → Core Experimental Work.** Two parallel tracks — see ⇄ TWO PARALLEL TRACKS below |
| **Next action — lab** | **C.0 stage 1 — validate the controllers still behave.** [`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md) |
| **Next action — writing** | **Nothing blocking.** Chapters 1–5 drafted (38 pp, ~14,356 words) and revised 24 Aug — full-text reading of every held source, 3 misattributions + 18 accuracy corrections applied. Remaining writing items need flight data or the lab |
| **Blocking** | Lab track: lab access. Writing track: nothing |
| **⚠️ Must clear in C.0** | **Three flight-code changes have never flown**: (1) residual sign fix, (2) `a_res` gating fix, (3) `rnn.en` residual compensation. All three change control behaviour and **none is detectable in single-drone flight** |

### What is complete

| | |
|---|---|
| Formation library | **Complete and frozen** — 16 scenarios (A1–A8, B1–B3, C1–C5) — [`10`](10_Formation_Library.md) |
| Sim validation of the library | **34/34 cases, 33 pass, 1 EXPECTED, 0 defects** — [`12`](12_Sim_Formation_Validation_Report.md). Found two flight-code bugs single-drone flight could not catch |
| Residual measurement path | **Complete** — `indi.a_res_*` = f_res/m, logged in every controller mode |
| `no_std` residual MLP + weight upload | **Complete and verified** — 987 weights, 10/10 checks against an independent reference — [`13`](13_Residual_Learning.md) |
| Training pipeline | **Complete and verified** — 12/12 checks end to end against the *compiled* controller |
| End-to-end simulation dry run | **Complete** — collect → train → upload → enable → measure. The plumbing works |
| Simulation of all 4 control modes | **Complete** — same Rust source that flies — [`09`](09_Simulation.md) |
| Documentation & repo map | **Complete** — [`00`](00_README.md), [`14`](14_Repository_Map.md) |

### Carry into the lab

| | |
|---|---|
| **Flight volume** | x −1..1, y −2..2, z 0.30..1.70 — corroborated but **not tape-measured**. With `--auto-center --rotate 90` all 19 configs fit at full default parameters |
| **Thin damping margin** | Hardware crashed 2/2 at `kv_xy = 4` and flies at 5. Formation flight adds downwash to that same loop |
| **uSD sync** | Predicted at a few ms, never measured. `merge_usd_logs.py` prints the real value — check it on the first 2-drone flight |
| **`a_res` must be non-zero** | It reads exactly 0.0 without an RPM source. Zero means no thesis data at all |
| **Sim-only gain** | Simulator uses `kv_xy: 8.0`; **hardware keeps 5.0**. Confirm the readback before flying |
| **Fragile** | The simulator depends on uncommitted lines in `crazyflie-firmware/bindings/`. Patch: `firmware_app/host/cffirmware_bindings.patch` |
| **Not blocking** | FBL code (Strategy 3) still with the authors; the other strategies proceed without it |
| **Open, paused** | Sim needs ~2× hardware's position damping. Time-boxed and deliberately parked — `KI_P`, rotor drag, inertia and airframe all refuted; motor lag closed ~40% |

---

---

## ✍️ Writing-track files are LOCAL ONLY

The thesis prose (`docs/thesis/`), its supporting documents (`15`–`20`, the review brief), the
bibliography (`docs/references.bib`) and the paper library (`docs/papers/`) are **deliberately not
versioned in this repository**. They were removed from the pushed history on 2026-08-24 and are
gitignored so they cannot be re-added by accident.

They still exist in the working tree, and a copy is kept outside the repository at
`~/Desktop/flying_robot_course_THESIS_LOCAL`. **Back that up separately — git no longer protects
it.**

The written record of *what* was done stays here (§ Writing Track and the History table below);
only the manuscript itself is absent.

## ⇄ TWO PARALLEL TRACKS

From 23 August 2026 the project runs two tracks that do not block each other. Keep them separate:
work on one should never be reported as progress on the other.

| | **🔬 LAB TRACK** | **✍️ WRITING TRACK** |
|---|---|---|
| **What** | Hardware gate, data collection, the comparison campaign | The theoretical backbone of the thesis |
| **Plan** | ★ Core Thesis Workflow, C.0 → C.4 (below) | ✍️ Writing Track plan (below) |
| **Blocked by** | **Lab access.** No software work blocks it | Nothing |
| **Next** | **C.0 stage 1** — validate the controllers still behave | *(nothing blocking — awaiting flight data)* |
| **Rule** | Nothing here is a desk task | Nothing here touches flight code |

The writing track exists so that time without lab access is not idle time, and so that the
theoretical chapters are not written under deadline after the experiments finish. Chapters that
depend on results (6–8) wait; everything else does not.

---

## ★ CORE THESIS WORKFLOW — the master plan

This is the plan the rest of the project follows. Preparation (everything above) is **done**;
the five steps below are the thesis itself. They are strictly ordered.

| # | Phase | State |
|---|---|---|
| **1** | **[C.0 — Hardware Gate](#c0--hardware-gate)** | ⬅️ **NEXT** |
| **2** | [C.1 — Residual Data Collection](#c1--residual-data-collection) | ⬜ Blocked by C.0 |
| **3** | [C.2 — Train the Residual Model](#c2--train-the-residual-model) | ⬜ Blocked by C.1. *Pipeline ready; needs real data* |
| **4** | [C.3 — Integrate the Strategies](#c3--integrate-the-strategies) | ⬜ 1 of 7 wired |
| **5** | [C.4 — Systematic Comparison](#c4--systematic-comparison) | ⬜ The thesis result |

**Ordering is a constraint, not a suggestion.** Each step exists to make the next one
interpretable. If a two-robot flight is the first thing that fails, "the migration broke
something" and "the interaction broke something" are indistinguishable.

---

### Architecture decisions (fixed 2026-08-22)

| Decision | Consequence |
|---|---|
| **NN inference runs ONBOARD; training is offline** | Every controller — geometric, hybrid, learning-based — runs on the brushless Crazyflie. Needed an MLP in `no_std` Rust + a weight-upload path — **built and verified**, see [`13`](13_Residual_Learning.md). Peer position is **already available onboard** via `peerLocalizationGetPositionByID()`, so the NN's main input is free. |
| **uSD logging is the dataset; radio is for monitoring** | Radio cannot carry 2 drones at full rate. 500 Hz onboard per drone, independent of radio — `tools/README_usd_thesis_logging.md`. Requires a Micro SD deck per drone. |

---

### Speed & separation are now real experimental factors — CLOSED 2026-08-24

Separation was already parameterised. **Speed was not**: `--speed` never reached `circle` or
`lemniscate` paths — the defaults for A2, A4, B1, B2 — yet was still recorded, so those runs
logged 0.4 m/s and flew **0.63**, above the 0.2–0.5 m/s band this work targets. Three fixes:
speed converts to a period, the *realised* speed is recorded and verified, and an inapplicable
flag is now an error instead of a silent default.

Validated 16/16 in sim on both controllers, with a negative control proving the new check fires.
All 16 scenarios remain byte-identical at their defaults, and 40/40 existing sidecars replay
unchanged. **Geometry conclusions in [`12`](12_Sim_Formation_Validation_Report.md) stand** —
only the speed field was ever wrong.

Report: [`../experiments/sim_validation/SPEED_MATRIX.md`](../experiments/sim_validation/SPEED_MATRIX.md).
Official grids are in [`10_Formation_Library.md`](10_Formation_Library.md) § Experimental grids.

> **Do not expand this.** No further grid growth, no combinatorial sweeps. The mm figures are
> simulation against the Neural-Swarm2 model — a hypothesis for the lab, not thesis results.

**Known, deliberately unfixed:** `run_sim_matrix.sh` deletes `state_$CTRL` each run, so raw
states are not re-examinable after the fact. Non-blocking; fix only if re-inspection matters.

---

### Design decisions closed by external review (2026-08-24)

Six open design questions were put to independent review and are now **settled**. They are recorded
here so they are not re-opened without new evidence.

| # | Question | Decision | Reason |
|---|---|---|---|
| 1 | C.1 scenario order | **Changed — A4 is now mandatory**, A7 or a lateral translate strongly preferred | A3 never excites relative *y*; without A4 the model trains on a degenerate slice and RQ3 becomes unanswerable. See [`11`](11_Hardware_Readiness_Checklist.md) § Minimum viable dataset |
| 2 | Frozen shared gains vs per-method tuning | **Keep frozen and shared** | Strategies 0/1/2/4 share the whole loop, so freezing is the stronger scientific choice. Record the residual risk for 3/6 under threats to validity |
| 3 | Strategy 4 form | **Keep the reference form** — NN predicts the bulk, incremental term corrects the remainder | Independent subtraction double-counts. Masking of a poor prediction is mitigated by assessing model quality open-loop |
| 4 | 2 vs 3 robots | **2 robots suffice for the primary ranking (RQ1); 3 are required for the transfer-penalty claim (RQ3)** | The library already supports 3 — **do not descope the B scenarios** |
| 5 | Strategies 5 / 7 (RL) | **Explicitly deferred / stretch, not core** | The contribution does not depend on them. Do **not** invest in an RL training environment now |
| 6 | Mode E maturity | **The 3-rung ladder is necessary and sufficient**: figure-8 Mode D → figure-8 Mode E → circle Mode E, only then multi-robot | One trajectory flight is not enough trust for a whole campaign |

**Explicitly not to be done now:** wire Strategy 4 or 6 · per-method gain tuning · RL environment
work · further simulator fidelity archaeology · results chapters.

---

---

## ◆ PREPARATION OVERVIEW — ✅ FINISHED

**The preparation phase is complete.** Everything below is the detailed record of it. Nothing here
is on the critical path any more — the project's next action is C.0 in the Core Thesis Workflow
above. This section is kept as evidence of what was built and verified, not as a to-do list.

The **Left** rows below are lab-only items that have been folded into C.0; they are listed here
because they were identified during preparation, and they are tracked in
[`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md).

One table for "what is actually ready". Nothing in the **Done** rows still needs work;
everything in **Left** stands between here and collecting the first real dataset.

### ✅ Done

| # | Area | What exists | Evidence |
|---|---|---|---|
| 1 | Planning & literature | 7 control strategies defined, literature matrix, gap statement, residual-wrench model, 2-robot protocol, chapter structure | [`01`](01_Thesis_Project_Snapshot.md)–[`06`](06_References_Overview.md) |
| 2 | Trajectory path | Mode D → Mode E migration; Mode D frozen byte-identical; `--laps`, rest-to-rest closed loops, one frame convention | [`08`](08_Trajectory_Upload_Paths.md) |
| 3 | **Residual measurement** | `indi.a_res_*` = f_res/m, logged in **every** controller mode | §5 of [`12`](12_Sim_Formation_Validation_Report.md) |
| 4 | Multi-drone execution | `formation_flight.py` (shared trajectory) + `run_formation.py` (per-robot trajectories), staged takeoff, safety gate | [`10`](10_Formation_Library.md) |
| 5 | Logging | uSD 500 Hz × 35 vars (incl. `rnn.pred_*`), broadcast start so per-drone logs share an origin, `merge_usd_logs.py` measures the real offset | — |
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
| 18 | **Residual network onboard — BUILT & TESTED** | Deep-sets MLP (987 weights) in `no_std` Rust, `rnn.*` upload protocol, prediction logged every tick regardless of `rnn.en`. 10/10 numerical checks against an independent reference | [`13`](13_Residual_Learning.md) |
| 19 | **Residual training pipeline — BUILT & TESTED** | uSD loader, PyTorch model, training with provenance, normalisation folded at export, ROS uploader. 12/12 checks end-to-end against the *compiled* controller. Found two silent-data-loss bugs in the uSD log path | [`13` §6](13_Residual_Learning.md) |
| 20 | **Residual dry run in simulation — DONE** — *this closed the software preparation phase* | Collect → train → upload → enable → measure, through the lab's code paths. Simulator grew peer injection, a residual log in the merged-uSD schema, and commanded-position logging | [`13` §7](13_Residual_Learning.md), [report](../experiments/sim_validation/RESIDUAL_DRYRUN.md) |

### ⬜ Left before data collection — **all lab-only, all inside C.0**

Every remaining item is a measurement, a decision made at the bench, or a flight. **Nothing here
is software.** These are the same items as the C.0 checklist above, listed in the order they were
identified.

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

**Then the core begins** — see ★ CORE THESIS WORKFLOW above: C.0 Hardware Gate → C.1 Residual Data
Collection → C.2 Train the Residual Model → C.3 Integrate the Strategies → C.4 Systematic
Comparison.


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

## B. Practical Preparation — ✅ SOFTWARE COMPLETE; remaining items are lab work folded into C.0

### B.1 Software — ✅ COMPLETE

- [x] Request FBL controller code via professor *(waiting on authors — not a blocker)*
- [x] Mode D → Mode E migration: official Crazyswarm2 upload path, Mode D preserved byte-identical — [`08_Trajectory_Upload_Paths.md`](08_Trajectory_Upload_Paths.md)
- [x] Multi-robot flight script `formation_flight.py` — **sim-run for the first time 2026-09-03**: core flow (arm, staged climb, XY convergence, trajectory upload/start, land) completes with 0 exceptions, 2-drone vertical figure-8. Two gaps found, both sim-only, neither blocks hardware use — see 2026-09-03 History entry
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

### B.2 Flight-validate Mode E — single drone → *now the single-robot ladder inside C.0*

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
| Logging | uSD config (500 Hz x 35 vars, incl. `rnn.pred_*`); broadcast start so logs share an origin |
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

## ✍️ WRITING TRACK — the theoretical backbone

Runs in parallel with the lab track and is blocked by nothing. Items W.1–W.4 are the theoretical
foundation; W.5 onward are the chapters themselves, which are tracked in §D.

| # | Item | State |
|---|---|---|
| **W.1** | **Problem statement & research questions** | ✅ **Drafted** — `15` |
| **W.2** | **Related Work structure** | ✅ **Drafted** — `16` |
| **W.3** | **Strategy descriptions** — the 7 methods, with their treatment of the interaction force made explicit | ✅ **Drafted** — `18` |
| **W.4** | **Contribution statement** | ✅ **Drafted** — `19` |
| **W.0** | **Source ledger & citation discipline** — added after a misattribution was found | ✅ `17` |
| **W.2b** | **Collect the paper PDFs + a `.bib`** | ✅ **Done** — 15 papers, `references.bib` |
| **W.2c** | **Read the held papers at claim level** | ✅ **Done** — all 15, `20` |
| **W.5** | **Chapter 2 prose** | ✅ **Drafted** — `thesis/ch2_related_work.tex`, ~3200 words, builds clean |
| **W.6** | **Chapter 3** — system modelling & residual formulation | ✅ **Drafted** — `thesis/ch3_system_modelling.tex` |
| **W.8** | **Chapter 4** — control architectures | ✅ **Drafted** — `thesis/ch4_control_architectures.tex` |
| **W.7** | **Chapter 5** — experimental setup & protocol | ✅ **Drafted** — `thesis/ch5_experimental_setup.tex` |
| **W.9** | **Chapter 1** — introduction | ✅ **Drafted** — `thesis/ch1_introduction.tex` |
| **W.10** | **Close the open verification items** | ✅ **4 of 5 closed** — incl. the non-arXiv sweep. The last needs the lab |
| **W.5** | Chapter 2 prose, following the W.2 skeleton | ⬜ |
| **W.6** | Chapter 3 — system modelling, from [`04`](04_Unified_Residual_Wrench_Model.md) | ⬜ |
| **W.7** | Chapter 5 — experimental setup, from [`05`](05_Experimental_Protocol_2Robot.md), [`10`](10_Formation_Library.md), [`11`](11_Hardware_Readiness_Checklist.md) | ⬜ |

### W.1 — Problem statement & research questions ✅

Drafted in `15_Problem_Statement_and_Research_Questions.md`:
formal problem statement in the notation of [`04`](04_Unified_Residual_Wrench_Model.md), an explicit
scope table, **four research questions** with falsifiable hypotheses, a mapping from each question
to the experiment that answers it, and a threats-to-validity table.

| RQ | Question | Answered by |
|---|---|---|
| **RQ1** | How do reactive, predictive and hybrid compensation compare under identical conditions? | C.4 |
| **RQ2** | Does the advantage depend on the interaction regime (separation, relative velocity, transient vs quasi-static)? | C.4 + a deliberate train/test split across scenario classes |
| **RQ3** | Does a 2-robot residual model transfer to 3 robots — do interactions superpose? | C.1 logs + C.2. **Needs only logged data, not a working 3-robot controller** |
| **RQ4** | What does each strategy cost in sensing, compute and data — is the hybrid justified? | C.2, C.4 |

> **Three open questions for the supervisor** are listed at the end of `15`, including whether RQ3
> should be promoted to a primary question.

### W.2 — Related Work structure ✅

Drafted in `16_Related_Work_Structure.md`: nine sections ordered so
that reading them **produces** the gap statement rather than asserting it, each grounding a specific
strategy or research question. §2.8 — *how these methods are evaluated, and why the results do not
compose* — is the load-bearing section and the one that makes this thesis a contribution rather
than a re-implementation.

### W.3 — Strategy descriptions ✅

`18_Strategy_Descriptions.md`. All seven, each split into **what we
implement** (verifiable against our source) and **the reference method** (attributed, with a
verification status and stated deviations) — never merged into one sentence. The differences are
made explicit as three questions: what the controller knows about the residual, *when* it knows it,
and *where* that knowledge enters the control law.

Implementation reality, stated plainly: **S1 flying · S2 implemented, unflown · S3, S4, S6 not
wired · S5, S7 deliberately deferred.** The uncompensated geometric baseline (S0) is added as a
flown condition — without it, "how much did compensation help" has no denominator.

### W.4 — Contribution statement ✅

`19_Contribution_Statement.md`. Five claims (C1 controlled
comparison, C2 hybrid multi-robot, C3 superposition test, C4 cost account, C5 apparatus), an
explicit **what this thesis does not claim** table, and a one-paragraph abstract version. Two
honesty notes carried in the document: the "first systematic comparison" claim depends on a survey
our own source ledger rates as least-verified and must be re-audited before use; and a null result
(strategies do not separate) is designed to be distinguishable from a failed experiment.

### W.0 — Source ledger & citation discipline ✅ *(added, not originally planned)*

`17_Source_Ledger_and_Citation_Discipline.md`.
Added after drafting W.1/W.2 propagated a misattribution out of [`03`](03_Gap_and_Contribution_Statement.md):
the neural-augmented INDI hybrid was described as showing "limited benefit" when the source reports
it achieving the **lowest tracking error**. Corrected in three documents.

Holds seven citation rules, a per-paper **verification status**, and the known discrepancies.

> ⚠️ **The repository holds no full text of any core interaction-force paper and no `.bib`.**
> Most citable claims are currently 🔴 *matrix only* — supported by a one-line entry in our own
> literature matrix. **Chapter 2 can be structured but not finalised until the PDFs are collected.**

### W.2b — Collect the PDFs and build the bibliography ✅

**15 papers** collected in `papers/`, with `references.bib` generated
**from the arXiv API** rather than typed from memory. Filenames match the BibTeX keys. PDFs are
gitignored (48 MB of third-party work, not ours to redistribute) and rebuildable via
`papers/fetch_papers.sh`, since every entry carries its arXiv id.

Two papers not previously in our matrix were found and added: Yang, Welde & Matni (2025), the
flatness-preserving-residual theory the 2026 formation paper builds on; and Bauersfeld et al.
(RA-L 2024), a quadrotor-airflow characterisation relevant to §2.2.

**Five bibliographic errors in our own notes were corrected:**

| Our notes said | Actually |
|---|---|
| "Neural-Augmented INDI … *with Payload Adaptation*" | *Learned Incremental Nonlinear Dynamic Inversion for Quadrotors with and without Slung Payloads*, L4DC 2026 |
| method "IL-NDI" | method **LINDI** |
| "L1 KNODE-DW MPC" as a title | *Online Adaptation for Flying Quadrotors in Tight Formations* — "L1 KNODE-DW MPC" is our shorthand |
| "Hsieh et al." for two different papers | **Pei-An Hsieh** (first author, 2025 & 2026) vs **M. Ani Hsieh** (senior author, all three). Cite with year + short title |
| Neural-Swarm2 as 2022 only | arXiv 2020, T-RO **2022**. `references.bib` carries a `yearnote` for all three preprint/venue mismatches |

> ### ⚠️ A second misattribution was found and corrected during this step
>
> The W.3 draft had "corrected" the neural-augmented INDI result to *"achieved the lowest tracking
> error of the methods compared"* — taken from our own `paper_summaries.md`. **The full text shows
> that is also wrong.** The hybrid is best without a payload *"although with a small margin"*
> (4.14 vs 4.28 cm) and *"remains similar in performance"* to INDI with one; the paper's headline
> is **sensor elimination** (LINDI matching INDI without rotor-speed measurement).
>
> Corrected in `03`, `15`, `16`, `18`, `19`. `paper_summaries.md` now carries a **DO NOT CITE**
> banner. Full account: `17` §1.

### W.2c — Read the held papers at claim level ✅

All 15 read. Every quotable claim is recorded **in the paper's own words** in
`20_Verified_Claims.md`, with the depth of reading stated per paper (two
deep — abstract, results, conclusion; thirteen at abstract level, which is what the chapter needs
from them and is marked as such). **Cite from `20` or the PDF, never from `02`.**

**Two further errors in our own matrix, corrected:**

| Entry | We had | Verified |
|---|---|---|
| Neural-Swarm separation | "~30 cm vertical" | **25 cm** (*"Each quadrotor flies at a different fixed height (25 cm vertical separation)"*) |
| Aggregate Downwash compute | "Usable onboard" | Unsupported by anything read — flagged, not asserted |

Also corrected in the other direction: **"L1 KNODE-DW MPC" is the paper's own method name**, not
our shorthand as the W.2b note claimed. The *title* is *Online Adaptation for Flying Quadrotors in
Tight Formations*; both names are correct.

> ### ⚠️ RQ3 had to be reframed — the literature already answers it
>
> RQ3 asked *"do pairwise interactions superpose?"* Shi et al. (2020) state outright that with more
> than two vehicles *"the aerodynamic effect is **not** a simple superposition of each pair"*, and
> Gielis et al. (2023) exists precisely because single-vehicle models are *"unlikely to be
> sufficient"*. Asking it would have claimed to discover published work.
>
> **Narrowed to:** *how much* accuracy does our 2-robot-trained deep-sets model lose on 3-robot
> formations, and is the loss small enough that the simpler architecture stays the right
> engineering choice? A quantitative penalty on our platform — defensible, and still worth asking.
> Updated in `15` and C3 of
> `19`.

**Bars our work is measured against** (now known, and stronger than assumed): flatness-preserving
residual reports **31 %** error reduction, **NMPC-matching at ~10× less compute**, **< 30 s**
training data, **5 ms** loop. SO(2)-equivariant reports **36 %** 3D / **56 %** vertical on two
vehicles from **5 minutes** of data. §2.8 must not imply the field lacks strong results — it lacks
*comparable* ones.

### W.5 — Chapter 2 prose ✅

`thesis/ch2_related_work.tex` — ~3200 words, 9 pages with
references, **builds clean** (`latexmk -pdf ch2_standalone.tex`, zero undefined citations, all 15
sources cited).

Written strictly from `20_Verified_Claims.md`: every number in the chapter
is one recorded there in the source's own words. Follows the `16`
skeleton, with §2.8 arguing the gap methodologically — including
Table~`tab:rw-conditions`, which lists each work's platform, team size, separation and reported
quantity and shows that **no two rows share all four**.

The chapter's framing, carried over from the verified reading: the field is **not short of strong
results** — a factor of 2–4, a factor of 3, 36 %/56 %, 31 %, ~40 % are all real — it is short of
**comparable** ones. That is a narrower and far more defensible claim.

`preamble_draft.tex` is drafting-only: it stubs `siunitx` if absent and renders forward references
to unwritten chapters as `[?]`. The real thesis preamble replaces both.

**Four `\todo{}` markers are live open items in the text**, not prose placeholders:

| Where | What |
|---|---|
| §2.1 | The Lee et al. citation — we hold *Control of Complex Maneuvers*, but our implementation follows the CDC 2010 *geometric tracking* paper |
| §2.8 | Two separations and one team size still unread (abstract-level only) |
| §2.9 | The "no such comparison exists" claim rests on our own survey; re-audit before submission |

### W.6 — Chapter 3, system modelling ✅

`thesis/ch3_system_modelling.tex` — ~2250 words. Combined build
with Chapter 2 is **16 pages, no undefined references** (`latexmk -pdf thesis_draft.tex`).

Defines the residual wrench as the single quantity every strategy estimates, then gives both routes
to it: measurement from `a_meas − a_model` and prediction from the deep-sets form. Both are
expressed as accelerations in the world frame and enter the control law at the same point with the
same sign — which is what makes the comparison a comparison rather than a set of separately-tuned
controllers.

**A real inconsistency was found and fixed.** [`04`](04_Unified_Residual_Wrench_Model.md) and the
first Chapter 2 draft both wrote gravity as `+m g e₃`; the implementation uses **z-up (ENU)**, where
gravity acts along `−e₃`. Left alone, the thesis equations would have had the opposite sign to the
flight code. Corrected in Chapter 2, and `04` now carries a superseded-on-this-point banner.

Three things the chapter states that the experiments then depend on:

| | |
|---|---|
| **The residual is a catch-all, not a physical quantity** | It is non-zero in isolated flight. The interaction can only be isolated by differencing against a no-neighbour condition — which is what the coplanar null scenarios are for |
| **The sign derivation** | §3.3.3 derives why the residual is *subtracted*, and predicts that an inverted sign gives ≈2× the uncompensated displacement while looking perfectly stable. That is a falsifiable prediction for C.4, and it is the bug that was found in simulation |
| **Assumptions table with failure modes** | Seven assumptions, each with where it breaks and what it costs, so a negative result can be attributed to an assumption rather than to the model as a whole |

### W.8 — Chapter 4, control architectures ✅

`thesis/ch4_control_architectures.tex` — ~2850 words.
**Combined build is now 24 pages** across Chapters 2–4, no undefined citations, no dangling
references, all 15 sources cited.

Organised around one observation: strategies 0, 1, 2 and 4 are **terms in a single shared control
law** — they differ only in which of the two residual terms in the desired-acceleration vector is
active. That is the strongest fairness argument available, and it is verifiable from the source
rather than asserted.

§4.9 states what that does **not** guarantee, before results rather than after:

| Limitation | Why it is stated up front |
|---|---|
| Strategies 3 and 6 do **not** share the substrate | A linearising law and a receding-horizon optimiser are different controllers, not different terms. For those, "identical conditions" means trajectories and hardware, not structure — a weaker comparison, and a difference could be attributed to the controller |
| Frozen shared gains are fair but not neutral | Gains suiting a geometric loop may disadvantage a strategy that could take more aggressive ones. The alternative reintroduces the tuning confound. Neither is free of criticism; the choice is deliberate |
| Each implementation represents a family, not a paper | Where a strategy underperforms, the conclusion concerns *this implementation* on *this* platform |

**Implementation status is stated honestly**: 2 flying, 1 implemented and unflown, 1 as separable
components, 3 not implemented. The chapter does not describe unimplemented strategies as though
they exist.

Two design questions are recorded rather than left implicit: the Strategy 4 form (independent terms
vs. INDI correcting only the network's residual — the two measure different things), and whether
Strategy 6 is feasible onboard at all, which is itself an RQ4 result.

### W.7 — Chapter 5, experimental setup ✅

`thesis/ch5_experimental_setup.tex` — ~2460 words.
**Combined build is 31 pages** across Chapters 2–5, zero undefined citations or references.

Platform, flight volume, the 16 frozen scenarios, the measurement chain, the C.0 gate, the metrics,
and an explicit threats-to-validity table. The reasoning throughout is the same: the design exists
to remove alternative explanations for an observed difference.

Three things the chapter makes explicit that were previously only in planning docs:

| | |
|---|---|
| **Why the null scenarios are the most important in the library** | The residual is a catch-all and non-zero even in isolated flight (Ch. 3), so a residual in formation cannot be attributed to interaction without a reference. C1–C3 bound the measurement floor *and* detect mis-identified mass or `kt` |
| **Why collection runs under the uncompensated baseline** | A compensating controller records the residual that *survived* compensation — not the quantity a predictive model should be fitted to |
| **Why the split is by contiguous segment, not by sample** | At 500 Hz a random per-sample split puts a near-duplicate of almost every test point into training, and reports a validation error that means nothing |

**Simulation's role is stated and bounded**: used to disprove implementations, never to validate a
method — because the interaction force in simulation comes from a learned model of the same family
being fitted. No result in Ch. 6–7 rests on it.

⚠️ **The chapter is written in the future tense because nothing has flown.** A `\todo{}` marks the
requirement to revise it to past tense as the campaign proceeds and to record protocol deviations
there rather than only in the results.

### W.9 — Chapter 1, introduction ✅

`thesis/ch1_introduction.tex` — ~1810 words. At the time W.9 closed the combined build was
35 pages, ~12,600 words across Chapters 1–5, zero undefined citations or references.
**Current figure after the 24 Aug revision and accuracy passes: 38 pages, ~14,356 words.**

Written last of the unblocked chapters deliberately: an introduction promises what the body
delivers, and Chapters 2–5 now exist to be checked against. Its five contributions correspond
one-to-one to C1–C5 in `19`; its four research questions to
`15`. Those three documents must move together.

The motivation opens from the physics rather than from generic swarm applications: a
\SI{9}{\centi\metre} vehicle is commonly flown with a \SI{60}{\centi\metre} safety margin, that
margin sets operating density, and the applications that motivate teams are the ones with no space
to spare. It states plainly that data-driven methods **have worked** — the margin has already been
substantially reduced — so the gap is not that the field lacks results.

**No result is claimed anywhere in it.** Contributions are phrased as what the thesis sets out to
establish, and a paragraph states explicitly that a null result is also a contribution and is
designed to be distinguishable from a failed experiment.

> ### ✍️ This completes every chapter that the lab does not block.
> Chapters 6–9 need C.4 data. The writing track's remaining work is verification, not prose.

### W.10 — Close the open verification items ✅ *(3 of 5; the rest are not desk work)*

| # | Item | Outcome |
|---|---|---|
| 1 | **"First systematic comparison"** | ✅ **Audited twice, claim sharpened.** arXiv sweep, then a cross-database sweep (OpenAlex/Crossref/DBLP/S2) **plus a citation-graph scan of 157 works citing the Neural-Swarm line**. Two adjacent comparisons found and now cited — see below |
| 2 | **Lee et al. citation** | ✅ **Resolved — the citation was already correct.** arXiv:1003.2005 contains our exact attitude error (eq. 8, `eR = ½(Rdᵀ R − Rᵀ Rd)ᵛ`) and our exact torque law (eq. 11, `M = −kR eR − kΩ eΩ + Ω × JΩ`). The doubt was unfounded |
| 3 | **Three abstract-only details in Ch. 2's table** | ✅ **Filled from the experimental sections:** Hsieh 2026 flies **2** quadrotors at **0.2–0.5 m**; Smith 2023 closes to **≈0.5 m**; ProxFly uses **0.5 m** (heterogeneous pair; docking from 0.05 m) |
| 4 | Tape-measure the flight volume | ⬜ Needs the lab — first C.0 session |
| 5 | Revise Ch. 5 to past tense | ⬜ Ongoing as the campaign runs |

**Two papers were found that we did not have**, both now in the library, the bibliography and
Chapter 2 §2.5:

| Paper | Why it matters |
|---|---|
| Li et al., *NDP-NMPC* (2023) | Neural downwash predictor + NMPC, trained with spectral normalisation — a second instance of Strategy 6's injection point |
| Shankar, Woo & Prorok, *Docking with Learnt Downwash* (2023) | Learned downwash in an optimal feedback controller for **vertical docking** at 1–2 body-lengths. The closest published setting to our tightest scenarios, and the regime where compensation stops being an accuracy improvement and becomes a feasibility condition |

The library is now **17 papers**; all are cited and all citations resolve.

### The non-arXiv sweep found two adjacent comparisons — and they improve the claim

The strongest test was structural rather than lexical: **every work citing Neural-Swarm,
Neural-Swarm2 or Gielis et al. — 157 in total — filtered for comparison/survey-like titles.** Any
systematic comparison of these methods would almost certainly cite one of the three.

| Study | Compares | Why it is not this thesis |
|---|---|---|
| Gu, Zhao & Rizzo, **JIRS 2025** | Four *online-learning* uncertainty estimators (ELM, RBFNN, ESN, GP) — accuracy, tracking, compute cost | **Single vehicle**, generic uncertainty, **online** learning, and **simulation throughout** — its "hardware experiments" are embedded latency benchmarks in the loop with a simulator, not flight. **Read in full** |
| Abro, Alsayegh & Mustafa, **IEEE JESTIE 2025** | **FOPID, FOLQR and integral state feedback (ISFA)** on close-proximity, downwash-motivated flight | Multi-vehicle and downwash-aware, but all three are **feedback regulators** — only ISFA is classical — and none presents a **learned or first-principles predictor** of the interaction force. Full text consulted 2026-08-27; PDF not held, numbers not imported |

**The claim is now sharper, not weaker.** "Nobody compares controllers for close-proximity flight"
would be **false** — Abro et al. do. What holds is that the families defined by *how they obtain
information about the interaction force* have not been placed under common conditions. Chapter 2
gains §2.8.1 stating this against both studies rather than in a vacuum, which is more honest and
more useful to a reader who knows the field.

Gu et al.\ also gives a **methodological precedent**: a comparison whose conclusion is a set of
trade-offs rather than a winner. That is the shape this thesis should aim for.

> **Still not proof of absence.** Both audits are keyword- and citation-based; a study citing none
> of the three seeds, or using different terminology, would be missed. IEEE Xplore and the ACM DL
> could not be searched through their own interfaces, though their content is indexed by the
> databases used. The *"to the best of our knowledge"* phrasing stays.

---

## C. CORE EXPERIMENTAL WORK — ⬜ NOT STARTED

**This is the thesis.** Everything in §A and §B was preparation and is complete. The five steps
below are the Core Thesis Workflow summarised at the top of this file. Theoretical writing (§D)
can run in parallel and is not gated on any of them.

---

### C.0 — Hardware Gate

**⬅️ THE NEXT ACTIONABLE PHASE.** Not an experiment — the check that nothing is broken before any
measurement counts. Acceptance criteria: [`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md) Checklist B.

C.0 runs in **three stages, strictly ordered** (confirmed 2026-08-23). The ordering is the whole
point: tuning before validating means you tune against an unknown fault, and flying two robots
before freezing means the gains move underneath the dataset.

#### Stage 1 — Validate that the controllers still behave

- [ ] Hardware inventory and bench checks (Checklist A); tape-measure the flight volume
- [ ] Confirm gains read back correctly — **`kv_xy` must be 5.0, not the simulator's 8.0**
- [ ] Single-robot ladder: figure8 Mode D → figure8 Mode E → circle. **Mode E has never flown**
- [ ] **Geometric SE(3) flies cleanly**, single robot
- [ ] **Corrected full INDI flies cleanly**, single robot — the residual sign fix inverts a
      control term
- [ ] Confirm the three unflown changes introduce no new instability

> Single-robot flights **cannot** confirm the three changes are *correct* — only that they are not
> catastrophic. `a_res ≈ 0` with no neighbour, so all three are invisible here. Correctness is
> established at stage 3.

#### Stage 2 — Retune the brushless attitude loop *if needed*, then FREEZE

- [ ] Retune the attitude loop to remove the remaining oscillation — **allowed here and only here**
- [ ] **FREEZE THE GAINS** (Checklist C)

> ⚠️ **This is the last moment any gain may change.** After the freeze, re-tuning would mean the
> campaign measures tuning effort rather than the control methods. The earlier blanket "do not
> change brushless KR/KW" rule is scoped to this: changeable inside C.0, fixed afterwards.

#### Stage 3 — Multi-robot and logging, at large separation

- [ ] Two robots at **large** separation, geometric first, then INDI
- [ ] **Confirm `indi.a_res_*` is non-zero and correctly signed.** Zero means no thesis data at all
- [ ] Confirm the residual sign fix behaves — this is the first flight where it is observable
- [ ] Check the measured uSD sync offset (`merge_usd_logs.py` prints it; the few-ms figure is
      predicted, never measured)
- [ ] Verify the merged multi-drone log actually contains relative state and `f_res`

**The three unflown flight-code changes cleared here:**

| # | Change | Why it cannot be skipped |
|---|---|---|
| 1 | **Residual sign fix** | Position INDI was *adding* `a_res` instead of subtracting it, reinforcing every unmodelled force at exactly 2.00×. The fix inverts a control term |
| 2 | **`a_res` gating fix** | RPM was read only when `mode != 0`, so `a_res` was dead under geometric — silently blocking all Geometric+NN training data |
| 3 | **`rnn.en` compensation** | The learned residual now feeds the position loop. Default off → byte-identical, but it is a control term. **Leave it off for all of C.0** |

> **None of the three is detectable in single-drone flight** — `a_res ≈ 0` with no neighbour.
> They only appear once another vehicle's downwash is present, which is why stage 3 exists.

### C.1 — Residual Data Collection

Fly the validated formations under **pure Geometric control**, logging high-quality `a_res` and
states. Geometric is the right choice: it does not compensate, so the residual is *observed*
rather than partly cancelled.

- [ ] Confirm `a_res` is non-zero and correctly signed before collecting anything
- [ ] Collect at safe separations first, tightening only once the loose cases are clean
- [ ] **Must include both vertical and lateral motion** — A1, A3 (vertical), **A4, A7 (lateral)**
- [ ] Log `a_res`, full state and rotor speeds at 500 Hz to uSD (radio cannot carry two drones)
- [ ] Verify the merged multi-drone logs actually contain relative state and `f_res`

> ⚠️ **A4/A7 are not optional.** The simulation dry run showed A3 alone never excites relative
> `y` — `train.py` reported `sigma = 1.0` for those inputs. Collecting only vertical scenarios
> leaves the model extrapolating across half its input space the first time a formation moves
> sideways.

Order and separations: [`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md).
This is the training set for **every** residual-learning strategy.

---

### C.2 — Train the Residual Model

The pipeline is **built and verified**; this step is running it on real data for the first time.
Tooling: `flying_drone_stack/tools/residual/` — [`13_Residual_Learning.md`](13_Residual_Learning.md) §6.

- [ ] Merge the per-drone uSD logs onto a common clock (`merge_usd_logs.py`)
- [ ] Train the deep-sets residual network on the collected flights
- [ ] **Validate model quality before integrating** — RMSE against *predicting zero* is the number
      that matters; a model that cannot beat that has learned nothing
- [ ] Export (normalisation folded into layer 1) and upload the weights
- [ ] Fly once with weights loaded and **`rnn.en = 0`** — predicted vs measured residual, open
      loop. This is the only comparison that distinguishes a good model from a lucky feedback loop

---

### C.3 — Integrate the Strategies

All seven share one residual model, one weight format and one upload path, so they differ in
*how they use the prediction* rather than in how they obtain it. That is what makes the comparison
a comparison.

| # | Strategy | Uses the NN residual? | State |
|---|---|---|---|
| 1 | **Pure INDI** | No — reacts to the *measured* residual | ✅ Implemented, flying |
| 2 | **Geometric + NN residual** | Yes, as feedforward | ✅ Implemented (`rnn.en`), **unflown** |
| 3 | **FBL + NN residual** | Yes | ⬜ *FBL code still with the authors* |
| 4 | **Hybrid Neural-Augmented INDI** | Yes, alongside the INDI measurement | ⬜ Not wired |
| 5 | **Residual RL** | Separate policy network | ⬜ **Deliberately deferred** |
| 6 | **Learning-based MPC** | Yes, as the prediction model in the horizon | ⬜ Not wired |
| 7 | **Geometric + Residual RL** | Separate policy network | ⬜ **Deliberately deferred** |

- [ ] Wire strategies 4 and 6 to the existing prediction
- [ ] Strategy 3 when the FBL code arrives — it does not block the others
- [ ] Strategies 5 and 7 last
- [ ] *Any stateful controller needs a per-vehicle state swap, or the multi-drone simulator
      silently shares its filters — see [`09_Simulation.md`](09_Simulation.md)*

> Strategies 2 and 4 may run the measured and predicted residual **together**. Double-counting is
> a real risk and is exactly what Strategy 4 exists to measure; it is not prevented in code,
> because preventing it would remove the comparison.

---

### C.4 — Systematic Comparison

**The thesis result.** Fly the same frozen formation library under every strategy.

- [ ] Run the full library per strategy, 2 robots
- [ ] Measure **tracking error**, **residual rejection**, and **robustness**
- [ ] Extend the best strategies to the **3-robot** tight formations (B1–B3)
- [ ] Verify interactions superpose — B1/B2 exist for exactly this
- [ ] Analyse against the protocol metrics ([`05_Experimental_Protocol_2Robot.md`](05_Experimental_Protocol_2Robot.md))

The coplanar control cases C1–C3 measured **0.0 mm under both controllers** in simulation. That
null result is what makes the positive ones meaningful; keep them in the campaign.

## D. Writing & Finalisation — 🔄 STARTED (writing track)

Chapters. The theoretical ones are **not blocked** and are being drafted now via the ✍️ Writing
Track above; the results chapters wait on the lab track.

| Chapter | Blocked by | Feeds from |
|---|---|---|
| 1. Introduction | — | `15` |
| 2. Background & Related Work | — | **W.5**, skeleton in `16` |
| 3. System Modelling & Residual-Force Formulation | — | **W.6**, from [`04`](04_Unified_Residual_Wrench_Model.md) |
| 4. Control Architectures | — | **W.3**, then [`13`](13_Residual_Learning.md) |
| 5. Experimental Setup & Protocol | — | **W.7**, from [`05`](05_Experimental_Protocol_2Robot.md), [`10`](10_Formation_Library.md), [`11`](11_Hardware_Readiness_Checklist.md) |
| 6. Results — 2-robot | **C.4** | — |
| 7. Results — ≥3 robots | **C.4** | — |
| 8. Discussion | C.4 | Threats to validity, `15` §4 |
| 9. Conclusion & Future Work | C.4 | — |

- [ ] Ch. 1 Introduction
- [ ] Ch. 2 Background & Related Work
- [ ] Ch. 3 System Modelling & Residual-Force Formulation
- [ ] Ch. 4 Control Architectures
- [ ] Ch. 5 Experimental Setup & Protocol
- [ ] Ch. 6 Results — 2-robot *(blocked by C.4)*
- [ ] Ch. 7 Results — ≥ 3 robots *(blocked by C.4)*
- [ ] Ch. 8 Discussion *(blocked by C.4)*
- [ ] Ch. 9 Conclusion & Future Work *(blocked by C.4)*
- [ ] Final polishing and defence preparation

> **Five of the nine chapters are not blocked by the lab.** That is the point of running two tracks:
> if the theoretical chapters are still unwritten when the experiments finish, the writing track was
> not used.

## Keeping this current

Update this file **in the same change** that alters project state — not afterwards. Specifically:

| When | Do |
|---|---|
| A task is finished | Tick its box **and** move the ▶ *Next action* line |
| A flight happens | Record the outcome under the relevant step; if it failed, say so and why |
| Something is descoped | Strike it with a one-line reason — never delete silently |
| A blocker changes | Update ▶ *Hard blocker* |
| Anything is added | Add the task **and** a History line below |
| A new document is created | Index it in [`00_README.md`](00_README.md) **and** in the root `README.md` |
| Work happens on either track | Update **that track's** section only. Writing progress is not lab progress |

Rules that keep it trustworthy:

- **Never tick a box for something that has not actually been verified.** Code that compiles is not code that flew — B.1 items are marked complete as *software*, and B.2 exists precisely because none of it has flown.
- Keep ▶ WHERE WE ARE to six lines. If it grows, the detail belongs in a section.
- One source of truth: put status here, engineering detail in [`08_Trajectory_Upload_Paths.md`](08_Trajectory_Upload_Paths.md) or the relevant doc, and link.

---

## History

| Date | Change |
|---|---|
| 2026-09-03 (2) | Regression-checked the sim pipeline after today's `launch.py` churn (reverted then re-reverted for the hardware IndexError investigation, unrelated) using `run_sim_matrix.sh one geo 2 --scenario A3`: PASS, 3.6 mm mean error, matching the 2026-08-23 baseline — confirms nothing broke. Along the way found and cleared 3 stale `crazyflie_server` processes left running from earlier manual tests (self-inflicted, not a code bug — killing the `ros2 launch` wrapper PID doesn't kill its child server process) and a separate harness bug: `one_run()`'s `find -newer $MARKER` sidecar-pairing gave a false "NO SIDECAR" even though the meta.json and record_states CSVs existed with correct timestamps; verified manually against `verify_formation_sim.py` instead. Then extended `formation_flight.py` sim coverage: dry-run matrix of all 3 formations (`vertical`/`horizontal`/`side_by_side`) x both rosters (2-drone, 3-drone) — 6/6 structurally sound, sane offsets, all XY transits under the script's own 1.0 m warning threshold. One real 3-drone vertical flight — 0 exceptions, same two known sim-only gaps as the 2-drone case (zero-row telemetry, controller-switching no-op), no new bugs. `formation_flight.py`'s core flight logic is now sim-proven for 2 and 3 drones across all 3 formation types. |
| 2026-09-03 | Lab access lost until Monday (unrelated logistics issue). `formation_flight.py` sim-run for the first time — 2-drone vertical figure-8, 0 exceptions, staged climb/converge/upload/start/land all completed. Two gaps found, both sim-only: (1) `DroneLogger`'s achieved-separation readback and per-drone CSVs got 0 rows — same known limitation as `run_formation`'s own report (live `{name}/state` topics and `/pose` don't populate in sim; `verify_formation_sim.py`'s record_states + sidecar JSON is the only sim-trustworthy check). (2) Runtime per-phase controller switching (`setParam('stabilizer.controller', ...)` etc. for geometric-ramp / INDI-cruise) silently no-ops in sim with `keyError raised` warnings — confirmed via `crazyflie_sim/crazyflie_sim/crazyflie_server.py`: the sim backend applies `firmware_params` **once** at server startup (`_apply_oot_firmware_params`) and never declares the per-robot `{name}.params.*` runtime parameters the real hardware C++ server does, so `formation_flight.py`'s intended takeoff-geometric/cruise-INDI/land-geometric switching never actually happens in sim — the whole sim flight runs on whatever `ctrl_mode` the chosen `server_sim_*.yaml` baked in at launch. Tested and ruled out timing as the cause (15s server warmup vs 5s: same 48 keyErrors either way) before concluding this is architectural, not a race. **Not fixed** — adding real per-robot runtime param support to the sim backend is a deliberate feature addition to shared sim code, out of scope for a quick pass; flagged here instead of silently patched. Does not affect hardware, where the real server does support this switching. |
| 2026-08-27 (3) | **C.0 order corrected.** The tape-measure step sat *after* the two-robot sign flight on the flight card, so §2 could be flown inside an unverified geofence --- and A1 at Δz 0.75 needs `--height 0.85` to fit at all. Moved into §0, together with an explicit flash check that the **three unflown residual changes** (sign fix, `a_res` gating, `rnn.en` off) are actually on the bird, since §2 is meaningless if any of them is missing from the build being flown. Also corrected a stale figure in [`11`](11_Hardware_Readiness_Checklist.md): the simulator's `kv_xy` is **8.0**, not 10.0; hardware stays at 5.0. No procedure rewritten and no rung invented. |
| 2026-08-27 (2) | **Citation audit CLOSED.** Every work cited by the thesis at that date was checked against its source PDF and the corrections applied to the `.tex`. The baseline is [`thesis/CITATION_BASELINE.md`](thesis/CITATION_BASELINE.md) plus the annotated git tag **`citation-audit-2026-08-27` (07f2e92)**, which carries the manuscript sources and builds to 41 pages with zero undefined citations. **The next literature pass is a `git diff` against that tag, not a restart**: only a new cite key, an edited prior-work sentence, or a new number on an old key needs checking. **`docs/thesis/*.tex` is the source of truth for every scientific claim**; the working notes in `docs/_archive/citation_notes/` (matrix, ledger, structure note, review brief) are **not sources** and must not be cited — `02` was archived in `ae4f768`. Thesis sources are tracked **locally only**: `origin/main` remains thesis-free and neither the commits nor the tag have been pushed. Writing status unchanged — Chapters 1–5 in design tense, ~41 pp, **no flight results claimed anywhere**. Software and simulation preparation were already finished and the residual stack remains **unflown**; **C.0 is the critical path**. |
| 2026-08-27 | **Hsieh 2025 correction pass, and a consistency sweep across every file.** The 2025 L1 paper was being described as "two three-quadrotor formations" with no separations and no platform detail, close enough to Chee 2024 and Hsieh 2026 to be confused with either. It is now explicitly **three Crazyflie 2.1, hardware-only, V-stack and I-stack, pairwise 0.2 m, off-board on two machines**, with **only the L1 estimate adapting online** while KNODE-DW reuses Chee's training data. Its abstract sentence is now inside quotation marks with our gloss marked as our reading — the same slippage that produced misattribution #2. Swept every doc for the eight papers' claims: fixed the reversed Smith reading in **`MEMORY.md`** and **`project_writing_track.md`** (it was flagged "must not be un-learned", the worst possible place for a stale claim), in **`THESIS_REVIEW_BRIEF.md`**, and the "graph-based aggregation" description of Gielis in **`16`**. Added the missing **Chee 2024** row to `02` — the matrix described the extension while omitting the work it extends — and corrected `17`'s status block, which claimed "15 sources" (there are 20) and contradicted itself by saying every paper was read in full *and* that most were read at abstract level. Draft rebuilt: 40 pages, zero undefined. |
| 2026-08-26 | **External verification pass over seven papers, one earlier decision reversed.** An independent close reading of Neural-Swarm, Neural-Swarm2, Hsieh 2026, Cobo-Briesewitz, Smith, Chee and Gielis returned corrections for all seven. The recurring fault was **not misreading a number but attaching it to the wrong thing**: Neural-Swarm's 2.4 cm is the **four**-vehicle-trained net (two-trained reaches 2.7 cm); Neural-Swarm2's 16-robot flight and its 3× are **separate reports** (that flight gives ≈1.5×); Hsieh 2026's 31 % is **abstract-only** and its 20× is **simulation**; Cobo's 40 % is **no-payload only** (19–24 % with); Gielis's sufferer is **held on a load stand**, "graph" is abstract language for a Deep-Sets implementation, and K ≤ 3. **Smith's 36 % was reversed from lateral back to 3D** — the reading recorded on 24 Aug was defensible and still wrong. All applied to Ch. 1–4, recorded as ⚠️ Corrections blocks in `20`, and propagated to `02`. |
| 2026-08-24 (3) | **Accuracy and consistency pass over Chapters 1–5.** The main defect was presentational but pervasive: 51 cross-references were written with a drafting macro that rendered every one as `[?]`, including the 28 pointing at chapters that now exist. All 28 now resolve to real numbers (Chapters 1–5), and the 23 pointing at unwritten chapters are named in prose — "the two-robot results chapter", "the discussion chapter" — rather than left as broken references; the macro now fails loudly if reused. **One real internal inconsistency was found and fixed:** §4.1 said "which of them is present" distinguishes Strategies 0/1/2/4, which implied Strategy 4 simply enables both residual terms of eq. (4.1) — the arrangement §4.5 explicitly rejects. §4.1 now states that Strategy 4 uses the predicted residual plus the *difference*, per eq. (4.6), and says why. Everything else verified rather than changed: all numeric claims trace to the ledger with their qualifiers intact (Smith's abstract-vs-body footnote, Hsieh's off-board clarification and 20× figure, Gielis's three regimes, Neural-Swarm's separate networks, Neural-Swarm2's two figures kept apart); the fairness argument is scoped to Strategies 0/1/2/4 in all four required places; Chapter 3's residual sign still matches `lib.rs` (`.sub(a_indi).sub(a_nn)`); the three interpretive glosses remain marked as our reading; Abro et al. remains abstract-only with its footnote. No new papers, claims, methods or protocol changes; nothing claimed as flown. 38 pages, zero undefined citations or references. |
| 2026-08-24 (2) | **Every held paper read in full.** Only Abro et al. remains abstract-level, and only because it is paywalled with no open copy. Four corrections followed, three of them material. **(1) Smith et al.\'s 36 % figure is described differently in three places in their own paper** — the abstract calls it 3D trajectory tracking, the contributions list calls it *lateral* tracking error, and the results measure it on a lateral transect beneath the upper vehicle. We had quoted the abstract. Chapter 2 now uses the specific reading (56 % vertical, 36 % lateral) with the manoeuvre named and footnotes the discrepancy. **⚠️ REVERSED on 2026-08-26 — see that entry above: the headline is now 36 % 3D / 56 % vertical, and "lateral" is the dissenting reading.** **(2) Hsieh et al.\'s controller runs off-board**, on an i5 laptop radioing commands at 400 Hz — so the 5 ms loop we cited as evidence of cheapness is a workstation figure, not an onboard one. Also recovered: the NMPC they match is specifically Chee et al. 2024, the concrete factor is a 20× solve-time reduction, their platform is two Crazyflie 2.1 units with thrust-upgrade bundles (very nearly ours), and they compensate forces only while leaving torques to the onboard rate loop — the same scoping decision we took. **(3) Gielis et al. do not simply refute superposition**: they report three regimes — chaotic, *well estimated by linear summation*, and non-linear — depending on formation and state-space location, which makes the question one of how much accuracy is lost and where, and sharpens RQ3 rather than undermining it. **(4) Their "usable onboard" attribute in our matrix is refuted outright** — the paper contains no discussion of onboard deployment. Everything else checked out: ProxFly\'s no-communication claim, Tal\'s encoder dependence and model-free property, Smeur\'s wind-tunnel experiment, Kiran\'s PIV, Chee\'s physical experiments (also on Crazyflies), Hsieh 2025\'s two three-quadrotor formations, Yang\'s simulation-only validation, Bauersfeld\'s figures, and Li\'s and Shankar\'s abstract claims, all confirmed in their bodies. Neural-Swarm2\'s threefold reduction appears only in its abstract with no body experiment tying it to the 16-robot result, which is why Chapter 2 states the two figures separately. No code touched. |
| 2026-08-24 | **Accuracy-only review of Chapter 2 and every prior-work claim elsewhere.** Fourteen corrections, of which four were substantive. (1) **Neural-Swarm trained *separate* networks on 2-, 3- and 4-vehicle data**, not one model on the combined set — our text implied the latter. (2) **Bauersfeld's turbulent-jet result is for hover** and we stated it unconditionally in both Ch. 1 and Ch. 2. (3) **Neural-Swarm's baseline is a *nonlinear* tracking controller**, not "a geometric controller" as §2.5 said — its attitude part cites Lee, but the description was ours. (4) **Neural-Swarm2's 16-robot/24 cm result and its threefold error reduction were joined as one outcome**; both come from the abstract and we cannot show they describe the same experiment, so they are now stated separately with that caveat. Also: "reproduce INDI's estimate" softened to "generate smooth approximations of" (twice) to match the source; "on average" restored to the 36 % figure; the unsupported "against the larger networks of the reference works" removed, since we hold no verified claim about published network sizes; four interpretive glosses that read as the papers' claims re-marked as our reading. **Two cited papers had no ledger entry at all** — Li et al. and Shankar et al., both added abstract-level with quotations. Five further claims added to the ledger that were being asserted without one, including Cobo-Briesewitz's spline-fitted training data and unseen test trajectories, both verified in the source during this pass. The stale Lee "citation choice to resolve" warning was removed. 38 pages, builds clean. No code touched. |
| 2026-08-23 (23) | **Full revision round over Chapters 1–5 and documents 15–20.** Three decisions applied. **(a) Citations fixed:** Lee, Leok & McClamroch CDC 2010 is now the single primary geometric reference (metadata via Crossref), with the arXiv companion retained as secondary and its role stated in both bib entries — the equations were verified there, and no claim rests on a document we have not read. Tal & Karaman is primary for INDI, Smeur secondary. **(b) Chapter 3 re-verified against the current firmware:** residual subtracted, `a_res = a_meas − a_model`, z-up gravity, deep-sets sizes, attitude error and torque law all confirmed to match `lib.rs` and `residual_nn.rs`; the chapter now states explicitly that these *are* the code and must be kept in sync. **(c) Strategy 4 locked to the reference form** — the network predicts the bulk, INDI corrects only the remainder, `− â_res − (a_res − â_res)`. Independent subtraction is rejected in text with the reason: it double-counts, so a *better* prediction would over-compensate and the strategy would measure an artefact. Its cost is stated too — a poor prediction is partly masked, which is why model quality is assessed open-loop instead. **Fairness argument scoped in all four places** (Ch. 1, 4, 5, `19`): the strong "one control law, different residual terms" form covers Strategies 0/1/2/4 only, not 3 and 6, which are structurally different controllers. **RQ3 and C3 reworded** so they cannot read as claiming to discover non-superposition, which Shi et al. and Gielis et al. already report. **Citation hygiene audited mechanically:** every numeric claim near a citation now traces to a quotation in `20`; three separations recorded there that had only been in the changelog; no chapter cites `02` or `paper_summaries.md`. The Abro et al. description was softened — "evaluated in hardware" overstated an abstract that says only "practical settings" — and carries a footnote that only the abstract was available. Apparatus demoted out of the numbered contributions in Ch. 1 and `19`. Ch. 5's lateral-excitation requirement (A4/A5/A7) promoted to a displayed block and repeated in the collection protocol. 38 pages, ~14,100 words, builds clean, zero undefined citations or references. No code touched. |
| 2026-08-23 (22) | **Gu et al. (JIRS 2025) read in full** — the only paper we hold that is itself a systematic comparison, and the closest methodological precedent. Four findings changed our documents. **(1) A correction to our own text:** its abstract says "extensive simulation and hardware experiments", and our note from that abstract implied real flight. No flight took place — the hardware component is the learning modules running on a Jetson in the loop with gym-pybullet-drones, and the conclusion lists real-quadrotor validation as future work. Corrected in Ch. 2; logged as incident 3 in `17`, the third time reading a source has corrected an abstract-level claim, and the first where the abstract was not wrong but was *read into more than it said*. **(2) Their fairness choice is the opposite of ours** — Bayesian optimisation tuning every method to its own optimum, where we freeze shared gains. Ch. 4 §4.9 previously presented per-method tuning as the alternative we rejected; it can now cite a study that chose it, which makes the discussion a genuine fork rather than a straw man, and states why the choice differs here (strategies 1/2/4 share the whole loop; our tuning budget is a lab session, not an optimiser). **(3) It justifies our output clamp** — they report a learned residual crashing the vehicle through "excessively large prediction values" from random initialisation. Ch. 3 now cites that as the documented failure the 8 m/s² ceiling guards, replacing reasoning with evidence. **(4) Our comparison is on real hardware and theirs is not** — a concrete differentiator, now stated rather than implied. Also noted for our own discipline: they frame the work as demonstrating superiority rather than testing for it, where our H1 is written so a null result is informative. No code touched. |
| 2026-08-23 (21) | **Non-arXiv sweep done — the last desk-side verification item.** OpenAlex, Crossref, DBLP and Semantic Scholar (which index IEEE Xplore, the ACM DL, Springer and Elsevier), plus the decisive test: a **citation-graph scan of all 157 works citing Neural-Swarm, Neural-Swarm2 or Gielis et al.**, filtered for comparison/survey-like titles. Asking structurally rather than by guessing keywords. **Two genuine comparison studies were found, both adjacent, neither ours**, and both are now cited in a new §2.8.1: Gu, Zhao & Rizzo (JIRS 2025) benchmark four *online-learning* uncertainty estimators on a *single* vehicle — a methodological precedent whose conclusion is a set of trade-offs rather than a winner, which is the shape this thesis should aim for; and Abro, Alsayegh & Mustafa (IEEE JESTIE 2025) evaluate fractional-order regulators for close-proximity swarms, downwash-motivated and multi-vehicle, but comparing classical controllers that neither estimate nor predict the interaction force. **These sharpen the claim rather than threaten it.** "Nobody compares controllers for close-proximity flight" would now be false; what holds is that the families defined by *how they obtain information about the interaction force* have not been placed under common conditions. Stating the gap against two named studies is both more honest and more useful than stating it in a vacuum. Library now 19 papers, all cited. The "to the best of our knowledge" phrasing stays — neither audit is exhaustive. No code touched. |
| 2026-08-23 (20) | W.10: three of five verification items closed. **The Lee citation was already correct** — arXiv:1003.2005 contains our exact attitude error (eq. 8) and torque law (eq. 11); the `todo` recorded my own unfounded doubt rather than a real problem, and checking cost less than carrying it. **Chapter 2's evaluation table is now complete** from the experimental sections (Hsieh 2026: 2 quadrotors at 0.2–0.5 m; Smith 2023: to ≈0.5 m; ProxFly: 0.5 m, heterogeneous, docking from 0.05 m). **The "first systematic comparison" claim was audited and narrowed**: an arXiv keyword sweep found no comparison or survey study, but it covered arXiv only and was keyword-based, so the wording is now "to the best of our knowledge" rather than unqualified — absence of evidence in a keyword search is weak evidence of absence. The audit found two papers we did not have, both added and cited: Li et al. NDP-NMPC (2023), a second instance of Strategy 6's injection point, and Shankar/Woo/Prorok on docking with learnt downwash (2023), the closest published setting to our tightest scenarios and the regime where compensation becomes a feasibility condition rather than an accuracy improvement. Library now 17 papers, all cited, all resolving. The two remaining items are not desk work: tape-measuring the flight volume needs the lab, and revising Ch. 5 to past tense follows the campaign. A non-arXiv sweep remains the last verification item. No code touched. |
| 2026-08-23 (19) | W.9 done: **Chapter 1 drafted**, ~1810 words. **This completes every chapter the lab does not block** — Chapters 1–5, 35 pages, ~12,600 words, building clean with no undefined citations or references. The introduction was written last of the unblocked chapters on purpose, so its promises could be checked against a body that already exists: its five contributions correspond one-to-one to C1–C5 in `19` and its four research questions to `15`, and those three documents must now move together. The motivation opens from the physics rather than from generic swarm applications, and states plainly that data-driven methods have already worked — the gap is not that the field lacks results but that they do not compose. No result is claimed anywhere in the chapter, and a paragraph states that a null result is also a contribution and is designed to be distinguishable from a failed experiment. The writing track's remaining work is verification rather than prose: seven live `todo` markers, the highest-risk being the re-audit of the "first systematic comparison" claim, which rests on the same survey that has already produced seven errors. No code touched. |
| 2026-08-23 (18) | W.7 done: **Chapter 5 drafted**, ~2460 words; combined build 31 pages across Chapters 2–5, zero undefined citations or references. The chapter promotes three things from planning docs into the thesis argument: why the null scenarios C1–C3 are the most important in the library (the residual is a catch-all and non-zero in isolated flight, so formation measurements need a reference — and the same scenarios detect a mis-identified mass or `kt`); why collection runs under the *uncompensated* baseline (a compensating controller records only the residual that survived compensation); and why the train/test split is by contiguous segment rather than by sample (at 500 Hz a random split leaks a near-duplicate of every test point into training). Simulation's role is bounded explicitly — used to disprove implementations, never to validate a method, because the simulated interaction comes from a learned model of the same family being fitted. The chapter is deliberately in the future tense with a `todo` requiring revision to past tense as the campaign runs, since nothing has flown. Four of the nine chapters are now drafted; Chapter 1 is the last one unblocked by the lab. No code touched. |
| 2026-08-23 (17) | W.8 done: **Chapter 4 drafted**, ~2850 words; combined build now 24 pages across Chapters 2–4 with no undefined citations and no dangling references. The chapter is organised around the observation that strategies 0, 1, 2 and 4 are terms in one shared control law, differing only in which residual term is active — a fairness argument that is verifiable from the source rather than asserted. §4.9 states what that does *not* guarantee, before results rather than after: strategies 3 and 6 do not share the substrate, so their comparison is structurally weaker; frozen shared gains are fair but not neutral, and the alternative reintroduces the tuning confound; and each implementation represents a family rather than reproducing a paper, so underperformance is a statement about this implementation on this platform. Implementation status is stated honestly per strategy — 2 flying, 1 unflown, 1 as components, 3 not implemented — and unimplemented strategies are not described as existing. Two design questions recorded rather than left implicit: the Strategy 4 form (independent terms vs. INDI correcting only the network's residual, which measure different things) and whether Strategy 6 fits onboard at all. No code touched. |
| 2026-08-23 (16) | W.6 done: **Chapter 3 drafted**, ~2250 words; combined build with Chapter 2 is 16 pages with no undefined references. The chapter's job is to establish that every strategy estimates the *same* quantity — two controllers compensating differently-defined residuals are not comparable however similar their block diagrams look — so it gives both routes to the residual in one notation, entering the control law at one point with one sign. **A real inconsistency was found: `04` and the Chapter 2 draft wrote gravity as `+m g e₃`, but the implementation is z-up (ENU) with `−e₃`.** Left alone the thesis equations would have carried the opposite sign to the flight code; corrected in Chapter 2 and `04` banner-annotated. §3.3.3 derives the compensation sign and turns the simulation-era sign bug into a falsifiable prediction for C.4 (inverted sign ⇒ ≈2× the uncompensated displacement, while looking perfectly stable). §3.5 tabulates seven assumptions with their failure modes so a negative result can be attributed to a specific assumption. No code touched. |
| 2026-08-23 (15) | W.5 done: **Chapter 2 drafted** as `docs/thesis/ch2_related_work.tex`, ~3200 words, 9 pages with references, building clean with zero undefined citations. Written strictly from `20_Verified_Claims.md` — every number in it is one recorded there in the source's own words. §2.8 makes the gap argument methodologically rather than critically, anchored by a table of each work's platform, team size, separation and reported quantity showing no two share all four. The chapter's central framing came out of W.2c and is the opposite of what the first structural draft assumed: the field is not short of strong results, it is short of comparable ones. `docs/thesis/` added with a drafting-only preamble that stubs `siunitx` and renders forward references as `[?]`; build artefacts gitignored, sources tracked. `references.bib` entry types corrected from article to misc (arXiv preprints), which also removed the empty-journal warnings. Four `\todo{}` markers remain in the chapter as real open items. No code touched. |
| 2026-08-23 (14) | W.2c done: **all 15 papers read at claim level**, with every quotable claim recorded in the paper's own words in `20_Verified_Claims.md` and the depth of reading stated per paper. **RQ3 had to be reframed.** It asked whether pairwise interactions superpose; Shi et al. (2020) state outright that they do not, and Gielis et al. (2023) exists because of it — asking it would have claimed to discover published work. Narrowed to *how much* accuracy our 2-robot-trained model loses on 3 robots, which is a quantitative question about our platform and still worth asking. Two more matrix errors corrected (Neural-Swarm separation is 25 cm, not ~30; "usable onboard" for Gielis is unsupported), and one over-correction reversed — "L1 KNODE-DW MPC" is the paper's own method name. Verified figures now known: flatness-preserving residual reports 31 % error reduction, NMPC-matching at ~10× less compute, <30 s training data, 5 ms loop; SO(2)-equivariant 36 % 3D / 56 % vertical on two vehicles from 5 min of data. These are the bars our comparison sits beside, and §2.8 must not imply the field lacks strong results — it lacks comparable ones. Chapter 2 prose (W.5) is now unblocked. No code touched. |
| 2026-08-23 (13) | W.2b done: **15 papers collected**, `docs/references.bib` generated from the arXiv API rather than typed from memory, PDFs gitignored but rebuildable via `papers/fetch_papers.sh`. Two papers not previously in our matrix were found (Yang/Welde/Matni 2025 flatness theory; Bauersfeld et al. RA-L 2024 airflow characterisation). **Five bibliographic errors in our own notes were corrected**, including a wrong paper title, a wrong method name (LINDI, not "IL-NDI"), and the Hsieh ambiguity resolved (Pei-An Hsieh first author vs M. Ani Hsieh senior author on three different papers). **A second misattribution was found**: the W.3 draft had corrected the NA-INDI result to "lowest tracking error", taken from our own `paper_summaries.md`; the full text shows the hybrid's margin is small and condition-dependent, and the paper's headline is sensor elimination. Corrected in five documents; `paper_summaries.md` now carries a DO NOT CITE banner. The sequence — two wrong claims in a row, both from trusting our own summaries — is written up in `17` §1 as the justification for the citation rules. Only 2 of 15 papers are verified at claim level; W.2c is that work. |
| 2026-08-23 (12) | W.3 and W.4 drafted, plus an unplanned W.0. **A misattribution was found and corrected:** `03` stated that the neural-augmented INDI hybrid showed "limited benefit"; the detailed summary in `paper_summaries.md` reports it achieving the **lowest tracking error** of the methods compared. The false version had already propagated into `15` (H4) and `16` (§2.7) because both were drafted from `03` rather than from the source. Corrected in all three — and the correct attribution yields a *stronger* gap: the hybrid works well where the residual is self-induced, and the multi-robot case, where it is caused by other vehicles and is a function of measurable relative state, is a different and untested problem. `17_Source_Ledger_and_Citation_Discipline.md` added in response: seven citation rules, per-paper verification status, and the known discrepancies. It records that **the repository holds no full text of any core interaction-force paper and no `.bib`**, so most citable claims are currently supported only by our own one-line matrix entries — Chapter 2 can be structured but not finalised. `18_Strategy_Descriptions.md` describes all seven strategies with *what we implement* and *the reference method* strictly separated. `19_Contribution_Statement.md` states five claims and an explicit what-is-not-claimed table, and notes that the "first systematic comparison" phrasing itself needs verifying. Writing-track next item is now W.2b, collecting the sources. No code touched. |
| 2026-08-23 (11) | **Writing track opened, running in parallel with the lab track.** The two are now explicitly separated at the top of this file: the lab track (C.0–C.4) is blocked on lab access, the writing track on nothing. W.1 drafted — `15_Problem_Statement_and_Research_Questions.md`: formal problem statement in the notation of `04`, an explicit scope table, and **four research questions with falsifiable hypotheses**, each mapped to the experiment that answers it, plus a threats-to-validity table written now so the experimental design can protect against them. Notably RQ3 (does a 2-robot residual model transfer to 3 robots — do interactions superpose?) needs only logged data, so it survives even if the 3-robot control campaign is descoped. W.2 drafted — `16_Related_Work_Structure.md`: nine sections ordered so that reading them produces the gap statement rather than asserting it, with §2.8 (how the field evaluates these methods, and why the results do not compose) as the load-bearing section. §D reorganised to show that **five of the nine chapters are not blocked by the lab**. No results are claimed in either document; hypotheses are labelled as such. No code touched. |
| 2026-08-23 (10) | C.0 entry sequence confirmed and recorded as three strictly ordered stages: (1) validate the controllers still behave, single robot, geometric then INDI; (2) retune the brushless attitude loop if needed, then **freeze the gains**; (3) two robots at large separation, confirm `a_res` non-zero, check uSD sync. Tuning before validating would mean tuning against an unknown fault; flying two robots before the freeze would let the gains move underneath the dataset. **The blanket "do not change brushless KR/KW" constraint is now scoped** rather than absolute — an attitude retune is explicitly permitted inside C.0 stage 2 and forbidden after the freeze, updated in `docs/14` and `flying_drone_stack/CLAUDE.md`. Also made explicit that single-robot flights cannot confirm the three unflown changes are *correct*, only that they are not catastrophic: `a_res ≈ 0` with no neighbour, so correctness is established at stage 3. |
| 2026-08-23 (9) | **Core Thesis Workflow recorded as the master plan, and the preparation phase declared finished.** `docs/07` now leads with the five ordered steps — C.0 Hardware Gate → C.1 Residual Data Collection → C.2 Train the Residual Model → C.3 Integrate the Strategies → C.4 Systematic Comparison — with the preparation/core boundary made explicit throughout. Section C was renumbered from four phases to five (training and integration were one step, now two); the four documents referencing the old C.2/C.3 were updated. `11_Hardware_Readiness_Checklist.md` is now titled as the operational detail of C.0 and states that it is the next actionable phase. `05_Experimental_Protocol_2Robot.md` was aligned: its prose formations mapped onto the frozen scenario library, metrics restated as the three C.4 axes (tracking error, residual rejection, robustness), and the freeze-the-gains constraint added. `13_Residual_Learning.md` opens with the foundation marked complete and a table placing it in each workflow step. **One real inconsistency corrected:** `01_Thesis_Project_Snapshot.md` still said data collection would start with *pure INDI*; C.1 uses *pure Geometric*, because INDI compensates the disturbance and would partly cancel the residual being measured. New memory `project_core_thesis_workflow.md` plus a rewritten MEMORY.md header so a future session lands on this immediately. No code changed. |
| 2026-08-23 (8) | Documentation consolidation pass. Root cause fixed first: `flying_drone_stack/.gitignore` had a blanket `*.md` rule with a hand-maintained allowlist, silently hiding **14 files** including the entire INDI oscillation investigation, the inverted-loop investigation, `GLOSSARY.md`, `paper_summaries.md`, the uSD logging README and `LOCAL_MODIFICATIONS.md` — every new document written there was invisible to git unless someone remembered to add an exception. Now ignores by name, not extension. (Separately, the **root** `.gitignore` excludes `**/CLAUDE.md` outright, so no `CLAUDE.md` in this repo is tracked at all. That rule looks deliberate and was left alone — but it means the agent instruction files live only on this machine.) `docs/14_Repository_Map.md` added (three repos, end-to-end signal flow, which-file-for-which-task, and the traps that have cost time). `docs/00_README.md` rewritten as a navigation map; root `README.md`, `flying_drone_stack/README.md`, both stack `CLAUDE.md` files, `crazyflie_examples/CLAUDE.md`, `experiments/README.md` and `sim_validation/README.md` brought current. Five course-era documents (ROADMAP, VALIDATION_PLAN, SLAM_STATUS, ADVANCED, CS2_ARCHITECTURE_PLAN) banner themselves as historical rather than being rewritten or deleted. Corrected three concrete inaccuracies: the branch tables named deleted branches, `firmware_app/CLAUDE.md` still described the controller mode as a recompiled constant rather than a runtime parameter, and `crazyflie_examples/CLAUDE.md` documented a log path that exists only on the lab machine. |
| 2026-08-23 (7) | Residual-learning step 3 of 3: end-to-end dry run in simulation. Collect under geometric → train → upload through the real `rnn.*` protocol → enable → measure, all through the code paths the lab will use. The simulator had to grow three things: peer injection (positions only, matching the real peer API), a residual log written in the same schema `merge_usd_logs.py` produces so one loader serves sim and hardware, and commanded-position logging — without which realised separation cannot be told apart from commanded motion, which made the first attempt's geometry section meaningless. `rnn.en` now feeds the prediction into the position loop (strategy 2), default off and byte-identical. Result: 90.4%/75.9% of the measured residual predicted open-loop, held z separation error 4.66 → 0.63 mm with compensation on. Simulation only — the residual is generated by a model of the same family being fitted, so this says the pipeline is ready, not that the method works. |
| 2026-08-23 (6) | Residual-learning step 2 of 3: the training pipeline (`tools/residual/`). uSD loader that owns the sign convention and replays the firmware's own input guards, PyTorch model matching the onboard layer shapes, training with a contiguous-block split and a predict-zero baseline, export folding normalisation into layer 1, weights carrying provenance, and a ROS uploader that deliberately does not enable what it uploads. Verified by 12 end-to-end checks that push a trained model through the real export and upload protocol into the *compiled* controller — agreement ~1e-6 m/s². Found two silent-data-loss bugs on the way: `decode_usd_log.py` never mapped `stateEstimate.*`, `indi.a_res_*` or `acc.*`, so `merge_usd_logs.py` would have produced no relative state and no `f_res` while reporting success; and the synthetic path bypassed the firmware's distance guards. Still trained on nothing real — no formation flight has happened. |
| 2026-08-23 (5) | Residual-learning infrastructure started (step 1 of 3). Onboard deep-sets network in `no_std` Rust — 987 weights, permutation-invariant so one weight set serves 2 or 3 robots — plus the `rnn.*` parameter upload protocol, which refuses a partial upload outright rather than running half-loaded. Peer velocity is differenced onboard from the position-only peer API, with the history kept per-vehicle inside `State` so the multi-drone simulator cannot cross-contaminate it. Prediction is logged every tick regardless of `rnn.en`, because comparing predicted against measured residual is the evaluation of every learned method. Verified by 10 numerical checks against an independent NumPy implementation; nothing consumes the prediction yet. `13_Residual_Learning.md` added. |
| 2026-08-23 (4) | Remote decisions closed. Firmware local modifications resolved — kept as local by decision, all four annotated in place, durable manifest at `host/LOCAL_MODIFICATIONS.md`; found that `usddeck.c`'s raised log-variable limit is load-bearing for the 34-variable thesis config and would silently truncate if lost. Three checklists prepared (hardware inventory, C.0 acceptance, freeze-the-gains) and the residual collection order decided: A1 → A3 → A4 under geometric. |
| 2026-08-23 (3) | External review returned. Confirmed the assessment and found no additional scenario gaps beyond the five already documented — a weak positive, not proof of completeness. Contributed the Core Task sequencing, then §C.0–C.3 (renumbered to §C.0–C.4 on 2026-08-23), with hardware validation of the two unflown flight-code fixes as an explicit gate before any data collection. |
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

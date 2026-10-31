# Auto-tuning plan, revision 3 (2026-10-28, branch `feature/auto-tuning`)

Supersedes revision 2 ([`tuning_pipeline_plan_2026-10-28.md`](tuning_pipeline_plan_2026-10-28.md), kept for the review ids E/G/R/C of [`tuning_pipeline_review_2026-10-28.md`](tuning_pipeline_review_2026-10-28.md)). Revision 3 contains **the decisions of 2026-10-28** and **what the plant work of the same day showed**. Steps A and B (lab) stay on `main` and do not block this branch.

## 0. One-paragraph summary
Every controller (S0 geometric, S1 INDI, S2 geometric + NS2, S3 INDI + NS2, S4 FBL) goes through the **same automatic tuning procedure on a host simulation of the real flight code**; the resulting gains are flown **unchanged** on hardware (no manual touch-up). Tuned object: the bottom drone only; the top drone stays on default gains. Gains are per **controller x scenario type** (A1, A4, A8), shared across dz and speed. Method: black-box (CMA-ES, with Sobol and grid baselines), one end-to-end pipeline first on FBL, then the C tree. A plant is only used for a controller if it passes a pre-registered gate against hardware; otherwise that controller is tuned with the same procedure on hardware.

## 1. Decisions (user, 2026-10-28) and where they apply
| # | Decision | Consequence in the plan |
|---|---|---|
| 1 | Tune mostly in simulation; gains reused unchanged on hardware; never manual | Plant fidelity is the critical path (§5). Gate per controller; fallback = same automatic procedure on hardware. |
| 2 | Only the bottom drone is tuned; the top drone is left alone | Top drone = S0 with default gains in every episode (closes G1). |
| 3 | A tuned S0 may absorb part of the downwash | Accepted, reported (R1); the fairness requirement is the identical procedure. |
| 4 | Black-box first, one end-to-end pipeline before other methods | §4; gradient methods are a later option, not part of the first pass. |
| 5 | One gain set per **controller and scenario type**; no finer (not per dz or speed) | S0-on-A1 has other gains than S0-on-A4 / A8; the same set is used for A1 at dz 0.3, 0.5, 0.6. Up to 5 x 3 = 15 tuning jobs. |
| 6 | Tuning settings come from the NS2 training grid: speed 0.1-0.5 m/s, dz 0.25 / 0.5 / 0.75 m, offset 0.05 / 0.1 / 0.15 m, **excluding the study cells** | §6. Same scenario types as the study, different settings (no leakage, E3). |
| 7 | INDI filter cutoffs: probably tuned | Needs a new image (compile-time 30 Hz Butterworth); after FBL, §3. |
| 8 | C tree: do both - own tuning per controller **and** "S0's gains reused for S1-S3"; discuss with the supervisor | Both variants are produced and evaluated; the reuse variant costs no tuning. FBL first. |
| 9 | Hardware budget / deadline unknown | Minimum plan assumed (§8): tuned vs default vs (FBL) hand-tuned, few flights per cell. |
| 10 | Objective weights: agreed proposal | §6. |
| 11 | Steps A and B stay on main, are completed in the lab by the user; baseline snapshot kept | `baseline_before_autotune/`; Step A/B docs in `docs/fbl/`. |

Two further rules from the earlier rounds: analysis uses complete uSD logs only; fairness rules of §2.

## 2. Fairness rules (fixed before any result counts)
Identical for all controllers: (1) one objective with one crash rule; (2) tuning cells disjoint from the study cells, separate validation cells; (3) plant, noise and seeds; plant fidelity checked per controller; (4) search procedure, settings, initial design, stopping and selection rule; (5) evaluation budget N = k·d (same k); (6) wide log bounds plus stability screen; (7) top drone fixed (decision 2); (8) takeoff and landing included in every episode; (9) pre-registration of 1-8; (10) report d, convergence curves and the spread across optimiser seeds. May differ: the parameters and their values. BO-review rules (Sec. 8.1, 8.4): shared seeds, equal effort, median + IQR over repetitions, no information leakage between tuning and test.

## 3. What is tuned (verified against `controller_omar_indi.c` 2026-10-28; baseline values in `baseline_before_autotune/README.md`)
| Controller | Parameters | d | Notes |
|---|---|---|---|
| S4 FBL (first) | `kp/kv/ki` for xy (tied x = y) and z; later the stock rate-PID gains | 6 (+3) | `kr_xy`, `kr_z`, `J` unused with `inner 1`; `cd`, `beta`, `gamma` fixed. Step B hand-sets xy (28 / 9.5 / 48) as a baseline only. |
| S0 / S2 | `Kpos_P_xy`, `Kpos_P_z`, `Kpos_D_xy`, `Kpos_D_z`, `Kpos_Iz`, `KR_xy`, `KR_z`, `Kw_xy`, `Kw_z`, `KI_xy` (+ `Kpos_I_limit`) | 10-11 | `Kpos_Ix/Iy` inert while `Kpos_Iz != 0`; P/D limits inactive (100 m); `KI` is an unclamped integral of the attitude error (R3: add a bound). |
| S1 / S3 | same, `indi 3` | 10-11 (+1-2) | INDI filter cutoff(s) optional, needs a new image; NS2 weights frozen. |
Variants for S1-S3 (decision 8): (a) own set per controller; (b) S0's position gains reused, only evaluated.

## 4. Method
Black-box optimisation on the host-compiled firmware (the same code that flies, one path for all five controllers). Optimiser: **CMA-ES** (`cma` 4.5.0 installed) with the measured budget: one episode takes 40-60 s per core, 22 cores, near-linear scaling, results deterministic for fixed seeds (§14 of revision 2). Baselines on every job: Sobol/random search with the same budget, and (S0 only, as a benchmark) GP-BO with logEI (BO review Sec. 8.2: Sobol is a mandatory baseline; only logEI beat it significantly in the review's benchmark). Selection rule for the final gains: best **robust score** (mean over the plant variants of §5), not best nominal score - the Step B experience: the nominal plant prefers ever higher gains, the robust set rejects them.
Gradient methods (DiffTune, von Prittwitz, DiffCoTune): not in the first pass; reasons in revision 2 §4 (effort of differentiable re-implementations of 4-5 controllers; the shared repo is a template, not a model of S0 - `tuning_repo_analysis_2026-10-28.md`). What we take from them: DiffTune's warning that gains tuned in simulation alone can fail on hardware (its sim-only gains crashed on two of three hardware cells) -> robust variants and the gate; von Prittwitz's Euler/step-size artefact -> our plant runs at 1 kHz with the firmware's 500 Hz controller; DiffCoTune's idea of adapting the plant to hardware -> done here by fitting the plume torque map (§5).

## 5. The plant (critical path) - what exists after 2026-10-28
Prototype in `experiments/fbl/sysid/` (`plume_torque_sim.py`, `tune_fbl_xy.py`): real firmware controller in a rigid body (host bindings), stock rate PID, motor lag 30 ms + 4 ms delay, linear motor law, EKF-like noise, **plus** two effects found from hardware logs (the quadratic motor law was **not** in this prototype, correction 2026-10-29; it is implemented in `experiments/tuning/plant_core.py`, which is the plant of record from now on):
1. **Plume torque on the bottom drone** (1.3-2.5 mN·m under the top drone, assumed 0.3 before): fitted on 240 flights, Gaussian lift loss per rotor (A 0.054 N at 0.4 m, sigma 6.1 cm, n 0.6); held-out correlation 0.96 (force), 0.83 (pitch torque), 0.63 (roll torque).
2. **Gyro vibration** (17 deg/s above 5 Hz in A1, 5-8 in A4/A8; plant had 0.5).
3. **Motor law and lag** from the logged rpm (differential gain 1.5-1.8x = quadratic thrust law, lag 30 ms): measured; in `plant_core.py` only.
Verified on hardware (FBL): the control law and gains run as designed (jerk regression R² 0.95-0.99), the z gains transfer from simulation (they were tuned there), the rate-loop response matches the plant. **Known gap:** hardware A8/A4 tracking behaves as if the feedforward worked at about 40 % (acceleration lag 290-500 ms against 0 in the plant), cause unknown; emulated by a feedforward scale 0.4, which reproduces the hardware numbers of preset 1 (A4 8.5/9.8 vs 10.2/8.2 cm, A8 5.1 vs 5.6 cm). The plant also fails to reproduce the downwash-model benefit for A2 with both vehicles moving (not needed for tuning).
**Robust variants** (every candidate is scored on all): nominal; motor lag 60 ms; gyro noise 17 deg/s; state delay 40 and 60 ms; feedforward x0.4 and x0; tilt-to-acceleration gain 0.5; feedforward x0.4 + delay 40 ms.
**To do for S0-S3 (WP-P2):** (a) replace the z downwash truth by the fitted plume model (so the truth is independent of NS2 and of the FBL ROM, E1); (b) INDI realism (rpm path, DShot latency/noise, thrust mismatch; the SIL INDI is near-ideal: S1/S3 dip 0.75 cm in simulation vs 10.6 cm on hardware); (c) per-controller **gate** (pre-registered, revision 2 §5: hardware mean within max(1.5 cm, 30 %) of the scenario metric, same sign of resolved differences, tilt within ±10 deg) using the flown default-gain data (pilot, unseen block, FBL 10-26); controllers that fail it are tuned on hardware with the same procedure.

## 6. Objective and scenarios (decision 10)
- Episode cost = RMS 3D position error of the bottom drone [m or cm], x/y/z equal weight (the downwash dip counts as much as the horizontal error), takeoff and landing included, no effort term; a **fixed large penalty** for crash/abort (tilt > 45 deg, trip, no completion); hard limits (tilt, saturation) instead of weights if gains turn violent.
- Tuning cells per scenario type, from the NS2 grid, none a study cell. Used in the FBL prototype: A1 dz 0.25 / 0.75; A4 (dz, offset, speed) 0.25/0.05/0.2, 0.5/0.1/0.3, 0.75/0.15/0.4. A8 cells to be chosen (speed 0.1-0.5, dz 0.25-0.75). Study cells (validation only): A1 dz 0.45 / 0.5, A4 dz 0.4 offsets 0.075 / 0.125 at 0.35 m/s, A8 dz 0.4 (6.25 s pass).
- One gain set per controller and scenario type is selected over its tuning cells (mean objective) and the robust variants.

## 7. Procedure and statistics
CMA-ES (population from d, fixed seed list), >= 5 optimiser seeds per job, same episode seeds (common random numbers) for all controllers, Sobol baseline with the same budget, median + IQR over optimiser seeds, convergence curves, final-gain stability across seeds. Validation: tuned vs default vs (FBL) hand-tuned on the study cells in the plant (3 noise seeds), then hardware. Friedman-type tests only for comparing optimisers, not for controller effects; hardware effect sizes with a bootstrap CI (protocol).

## 8. Hardware confirmation (minimum plan; budget open)
Per controller: default gains vs tuned gains on the study cells that matter, 3 flights per cell (the 10-29 pilot/unseen-block data become "default-gain" results). FBL: preset 1 vs hand-tuned (Step B) vs auto-tuned. Gains are set by parameters, no reflash (the INDI filter option needs one image). Analysis from uSD logs only.

## 9. Work packages
| WP | Content | Status / effort |
|---|---|---|
| P0 | Baseline snapshot of all gains/filters/hashes | done (`baseline_before_autotune/`) |
| P1 | Plant for FBL: plume torque, gyro noise, motor law, robust variants | prototype done; document and package (this branch) |
| P2 | **Pipeline package** `experiments/tuning/`: config (controller, scenario cells, parameters, bounds, variants), evaluation worker pool, CMA-ES / Sobol / grid runners, logging of every evaluation, robust scoring, report | **start now**, 2-3 days |
| P3 | End-to-end FBL: 6 parameters, budget N = k·d, 5 seeds, compare with Step B grid result (28 / 9.5 / 48) and preset 1 | 1-2 days after P2 |
| P4 | Plant for S0-S3: plume truth, INDI realism, gate per controller | **first pass done 2026-10-29** (`tuning_plant_p4_2026-10-29.md`): INDI flies in the plant, joint calibration, gate on magnitude not met (6/16), order 19/24, tilt reproduced; decision on how to use it pending |
| P5 | C-tree jobs: S0, S2, then S1, S3 (variant a and b), 3 scenario types | 4-8 days of compute and analysis |
| P6 | Hardware confirmation, protocol text | lab-dependent |
| P7 | Optional: INDI filter cutoff (new image), GP-BO benchmark, gradient pilot (vonPrittwitz-style) | after the first pass |

## 10. Risks
R1 tuned S0 absorbs downwash (accepted, report); R2 simulation-tuned INDI hurts on hardware (gate; the SIL INDI is too ideal today); R3 unclamped `KI` (add a bound as part of the parameter set); R4 host-only artefacts (stack, timing, float vs double: use the same code, check hardware once); R5 power fault on cf5 (hardware cells); R6 the A8/A4 feedforward gap is unexplained: robust variants cover it as sensitivity, a lab test (A8 with feedforward full / acceleration only / none) would settle it; R7 optimum at the bounds or on a stability edge (robust score, bounds sensitivity report).

## 11. Open points for the supervisor
Decision 8 consequences (which C-tree gains count in the study); whether a tuned S0 may absorb downwash; INDI filter cutoff in or out; per-scenario gains in the protocol text; hardware budget.

## 12. Resources used
Papers (summaries in `docs/papers/summaries/`): von Prittwitz 2024 (gradient tuning of the geometric controller, window schedule), Cheng 2024 DiffTune(+), Krishna 2025 DiffCoTune, Stenger 2026 BO tuning review (fair-comparison rules, initial designs, logEI, crash constraints, contextual BO), Hsieh 2026 flatness-based FBL (gains, C_D, the A2 protocol), Hsieh 2025. Repo: `IMRCLab/differentiable-quadcopter` (analysis in `tuning_repo_analysis_2026-10-28.md`). Own: SIL/host bindings (`docs/75`), FBL C-in-the-loop plant, `experiments/fbl/sysid/`, uSD logs, `docs/fbl/GAIN_TUNING_XY_2026-10-28.md`, `docs/fbl_xy_plant_gap_diagnostic_2026-10-28.md`.

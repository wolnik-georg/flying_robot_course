# Plan: FBL close-out and a fair, automatic gain-tuning pipeline for S0–S4 (2026-10-28, revision 2 after the independent review)

> **Superseded by [`auto_tuning_plan_rev3_2026-10-28.md`](auto_tuning_plan_rev3_2026-10-28.md) (decisions of 2026-10-28). Kept for the review ids.**

**Status: DRAFT, revision 2, nothing started.** Origin: supervisor meeting 2026-10-29 prep (user notes of 2026-10-28). Revision 1 was reviewed independently: [`tuning_pipeline_review_2026-10-28.md`](tuning_pipeline_review_2026-10-28.md) (finding ids E1–E6, G1–G8, R1–R5 and questions C1–C8 refer to it). Items marked **[proposal]** are suggestions, not decisions. Nothing here starts before the open decisions in §12 are answered with the supervisor.

## 1. Order of work
| Step | What | Who | Gate to the next step |
|---|---|---|---|
| **A0** (new, desk) | **Port-equivalence test of FBL:** replay logged 10-26 states through the host `fbl_law` and the authors' Python code; outputs must agree to float tolerance. This replaces "run the authors' gains on a thrust-upgraded drone" as the test of "the port is correct" (E2). Status and what is already covered: `docs/fbl/PORT_SPEC.md` gates G1–G4, G7, G8; **Result 2026-10-28 (§14): equivalent to float32 precision on 7 flights / 60 817 steps per preset**; one torque tolerance ratio above 1 explained in §14. | desk | Outputs agree, or the difference is understood. **Met.** |
| **A** (optional platform check) | Authors' simulation-code gains (`ctrlFbl.preset 0`) on a thrust-upgraded Crazyflie. It tests **platform transfer**, not the port: preset 0 is the authors' **simulation-code** gains (30 g simulated vehicle), not their hardware gains (unknown; no email to the authors by the decision of 10-22). The authors flew **off-board** (Vicon 120 Hz → laptop → radio commands thrust + body rates at 400 Hz, stock onboard PID); ours runs onboard at 500 Hz with the EKF, our trained 4-neuron network and the downwash gate, so "everything else as in the paper" is not achievable. A correct port can fail here (lag, estimator, thrust map) and a wrong one can pass (a hover never exercises the downwash sign or gate). Records if it is run: image hash and build flags (`CONFIG_CRAZYFLIE_THRUST_UPGRADE_KIT`, mass, thrust map); identified motor lag of that drone (step test); hover thrust; gain readback; uSD logging started **before arming** (the 10-26 flips happened before logging began); single drone first, then stacked; Nominal → ROM → Learned; video; a numeric "flies like the paper" (paper: Learned about 5 cm RMSE, about 10 cm z_max at 0.3/0.4 m). | lab (user) | Informative only; not a gate for C. |
| **B** (conditional) | Short FBL gain touch-up on the brushless drone (x/y position and/or integral gains, in the way ki_z 12 was found: simulation sweep, then flights), days not weeks. **Run B only if C is not approved**: C retunes FBL anyway. | desk + lab | FBL frozen. |
| **C** | **Common automatic tuning pipeline** for all controllers (§2–§9), run before the comparative study. | desk, then lab confirmation | Protocol v1.0 with image hash, supervisor's go, then the S0–S4 study. |

Unknowns for A [to verify]: do we own a thrust-upgraded CF2.1 and does our FBL image run on it? The firmware has a compile switch `CONFIG_CRAZYFLIE_THRUST_UPGRADE_KIT` (memory note "upgraded drone thrust model"); the FBL image so far (`study_tree_v2_fbl_stockinner_nn4_full_candidate.bin`, sha256 `e3593a6f…`) was built for the brushless drone. Preset 0 = kp_xy 56, kv_xy 38, ki_xy 48, kp_z 45, kv_z 21.6, ki_z 3 (`docs/lab_sessions/2026-10-26.md` §3; firmware comment: "the authors' values exactly as in their code (30 g simulated vehicle)").

## 2. Goal and fairness of C
**Goal:** replace subjective, scenario-specific manual tuning by one automatic procedure, applied identically to every controller, so that the comparative study compares controllers that were tuned by the same process. Closest published precedent: BO on a high-fidelity simulation "to fairly compare different controller structures" (BO review Sec. 7.5: Funk 2021, Manhães 2017, Yu 2020).

**What must be identical for all controllers [proposal]:**
1. Objective function (same terms, same weights, same normalisation) and **one crash/constraint handling rule**.
2. Scenario set used for tuning, **disjoint from the study cells** (§6), and a separate validation set.
3. Plant, noise and seeds; the plant's fidelity is checked **per controller** (§5), and the downwash "truth" in the plant is **independent of every compensator** (not the NS2 network, not the FBL ROM).
4. Search procedure **and its settings**, initial design, stopping rule, selection rule for the final gains.
5. Evaluation budget tied to the parameter count, **N = k·d** (same k for all), instead of the same N (G2).
6. Bounds rule independent of the maturity of the starting values: wide log bounds plus a stability screen, and a report of the sensitivity to the bounds (G3).
7. **Top-drone policy** fixed explicitly (G1): the top drone is always S0 in the study; does it get the tuned S0 gains or stay on the defaults? This changes the disturbance for all strategies.
8. Episodes include **takeoff and landing** (G4), not only the hold or the pass.
9. Pre-registration of 1–8 before any result is looked at (as done for NS2 retention, `docs/lab_sessions/ns2_unseen_block_plan_2026-10-22.md`).
10. Report the parameter count d and the per-controller convergence curves.

**What may differ:** the parameters themselves and their final values per controller (and per scenario, if we decide so, §12).

## 3. What gets tuned (verified against the live firmware `controller_omar_indi.c`, review E5/C6)
| Strategy | Controller | Candidate parameters | Notes |
|---|---|---|---|
| S0 geometric | Omar tree, `indi 0` | `Kpos_P`, `Kpos_D`, `Kpos_Iz`, `Kpos_I_limit`, `KR`, `Kw`, `KI_xy` (x = y tied) | **≈ 10–12 parameters** after tying: Kpos_P_xy, Kpos_P_z, Kpos_D_xy, Kpos_D_z, Kpos_Iz, Kpos_I_limit, KR_xy, KR_z, Kw_xy, Kw_z, KI_xy (KI_z / yaw optional). Exclude: `Kpos_Ix/Iy` (**inert** while `Kpos_Iz ≠ 0`: l. 442–447 forces the x/y integral term to 0), `Kpos_P_limit` / `Kpos_D_limit` (error clamps at 100 m, **inactive**), mass and J (physics). `KI` is an **unclamped integral of the attitude error eR** (l. 597, 604, reset only at controller init): decide whether it stays; a clamp would be a code change. Current study values: "Omar defaults 7/4 plus the study's Kpos_Iz 1.5" (firmware default `Kpos_I = 0`; `docs/study_parameter_decisions.md`). |
| S1 INDI | same tree, `indi 3` | same gains | The INDI filter cutoffs are a compile-time 30 Hz (4 Butterworth-2 filters at 500 Hz; the snapshot `flying_drone_stack/firmware_app/host/controller_omar_indi.c.snapshot` and the live source are identical there). Tuning them needs a new image; recommendation: leave them at 30 Hz, or one shared cutoff as a compile-time option. |
| S2 / S3 | same tree + NS2 | same gains as S0 / S1 | Network weights (ns2_v2) stay **frozen**. Open: tune S2/S3 gains separately or inherit (§12). |
| S4 FBL | `ctrlFbl`, `inner 1` | `kp/kv/ki` xy and z (6) plus the stock inner loop (`pid_rate` roll/pitch kp, ki, kd and yaw; optional, for symmetry with the tuned Omar attitude gains) | `kr_xy`, `kr_z` and `J` are **unused with `inner 1`**; `cd`, `beta`, `gamma` fixed. Preset 1 = 0.25× the authors' position loop, `ki_z 12`. |

Parameter count: about 10–12 for the Omar tree, 6–9 for FBL.

## 4. Method options
| Option | Needs | Fits all S0–S4? | Main risk |
|---|---|---|---|
| **Black-box optimisation on the host-compiled firmware** | A way to run an episode and read a cost | Yes: uses the real flight code, one code path | Evaluation cost (wall time per episode: about 40–60 s on one core, measured in §14) |
| **Gradient (DiffTune / von Prittwitz)** | Differentiable plant **and** controller | Only by re-implementing each controller (4–5 times) and the plant, with equivalence tests. "Only S0 is differentiable" is overstated: plain MLPs, Butterworth filters and the FBL algebra are differentiable almost everywhere; the real argument is **effort**. The shared repo is not a usable S0 (`tuning_repo_analysis_2026-10-28.md`). | Re-implementation errors; sim-to-real gap (DiffTune: simulation-only gains crashed the real drone on C(2) and C(3)) |
| **Co-tuning plant and controller** (DiffCoTune) | Differentiable simulator, a few real rollouts | No quadrotor example, JAX/MuJoCo port | Heavy |

"BO is inefficient above about 20 parameters" is a claim of the DiffTune and DiffCoTune papers, not of the BO review (which treats it as common practice below 10 and recommends logEI for high dimensions).

**Recommendation [proposal]:** a **black-box** optimiser on the host-compiled firmware (`docs/75_Simulation_End_to_End.md`), chosen with the measured wall time per episode (§14: about 40–60 s per two-drone episode on one core, 22 cores, near-linear parallel scaling):
- If an episode is cheap enough to run in parallel (per-episode time of tens of seconds is fine with many cores; to be confirmed after WP2a): **CMA-ES** (rank-based, so robust to crash penalties; no GP hyperparameters; parallel). Run a short benchmark against Sobol and GP-BO on S0, as BO review Sec. 8.2 asks (BO is for expensive, noisy evaluations; CMA-ES, pattern search and PSO "can be competitive" when evaluations are cheap; Sobol is the mandatory baseline).
- If an episode is slow (≫ 10 s) or for the hardware step: **GP-BO** (logEI, BO-VDP or a classifier for crashes, possibly multi-fidelity).
- Gradient methods: no (effort, see above). This is a design choice the supervisor/professor may see differently (they shared the differentiable repo).

## 5. The plant used for tuning [critical; review E1]
Existing options: (a) the SIL loop of `docs/75` (firmware via Python bindings, numpy rigid body with 44 ms motor lag, noise, network downwash plant, `above_only`); (b) the separate FBL C-in-the-loop plant (`experiments/fbl/stage3_port/c_in_the_loop_real.py`). **Neither is a validated tuning plant for S0–S4:**
- The "A8 validated within about 1 cm" result is for **controller 6** (our geometric, NS2 off / `res_sign +1`; `−1` off by 2.3 cm; `docs/72`, `docs/75` §3), **not** for the Omar tree that S0–S3 now use.
- For the tree, the SIL INDI is near-ideal: S1/S3 dip 0.75 cm against 10.6 cm on hardware (`docs/75` §3, `validation_s3_versions_2026-10-18.md` C5). S0 dips −0.4 cm at plant scale 1.0 (10-14 setup) and reaches −11.5 cm only at scale 1.3×; on hardware S0 dipped −12.7 cm (10-19, 0.5 m/s). The settings are not identical, so this is strong evidence, not a matched comparison [to verify before quoting].
- The 2026-10-11 screen (`docs/sil_study_screening.md`) used **controller 6** (`ki_z 16`); A1, A2, A5 and some A7 cells diverged with seed 0 while A1 dz 0.5 flew on hardware without divergence, with a ±35–40° oscillation (`docs/68`; there is no dz 0.25 in `tree_comparison_2026-10-19.md`). The doc blames finite-difference setpoints and start-up transients and says Omar C and our INDI "cannot run here".
- The downwash "truth" in both plants is the **NS2 network**: S2/S3's own predictor and the fitting target of FBL's ROM. Tuning there would reward compensators for predicting the simulator.
- The FBL C-in-the-loop plant hard-codes `indi = 0` and has no RPM, so it cannot run INDI. Two plants would break "same plant for all".
- Host gains of the Rust controller 6 are process-global (`docs/72`); the Omar C tree (`oot4`) keeps per-vehicle parameters, so two controller-9 drones with independent parameters are possible (§14; small cross-talk for the INDI drone, 5.8e-5 m at 8 s, cause not investigated).

**Gate (before any tuning result counts) [proposal]:** **per controller and per scenario, pre-registered**. With the current gains of each controller, simulate the flown scenarios and compare with the flown default-gain uSD data (pilot, unseen block, FBL 10-26):
- hardware mean within max(1.5 cm, 30 %) of the scenario's primary metric (A1 mean z error, A8 dip, A4 z RMS);
- the same sign for every pairwise difference that is resolved on hardware;
- tilt within ±10°.
If S1/S3 cannot pass, **do not tune them in simulation**: tune the shared position gains on S0/S2, or tune S1/S3 on hardware with few evaluations.

Before the gate (WP2a): one harness for all five controllers; INDI realism (RPM/IMU/DShot latency and noise, thrust mismatch; the "vertical model F1" of `validation_s3` A5 is a starting point); a downwash truth that is not the NS2 network (e.g. a physics model fitted to flights). WP2b: determinism, wall time, noise spread across seeds.

## 6. Objective and scenarios [proposal, needs decisions]
- Per-episode cost from the simulated trajectory, built from **the metrics we already report**: z error (A1 mean / A8 dip / A4 RMS), horizontal RMS, plus a **crash/tilt constraint** (BO review Sec. 6.1: without a tracking constraint the optimiser can pick degenerate gains) and a small effort term. Normalise each term per scenario by a controller-independent reference (the S0 default value), fixed in advance. Episodes include takeoff and landing.
- **Tuning settings must not be study cells (E3).** The study table's settings (pilot A1 dz 0.5, A8 dz 0.5 at 0.5 m/s, A4 offsets 0.10/0.15 and so on) are not used for tuning; use e.g. A1 dz 0.35 / 0.6, A8 at 0.25 / 0.45 m/s, A4 offset 0.10 at another speed. Validate in simulation on the unseen-block settings (A8 dz 0.4 at 0.35 m/s, A1 dz 0.45, A4 dz 0.4 at 0.35 m/s offsets 0.075/0.125; n = 3 for A1 and A8, n = 2 for A4 per strategy, 40 flights; flown with default gains, NS2 never trained on them, nothing tuned on them), then test in the study. They were already seen by the author and n is small, so they are weak hardware evidence.
- One gain set per controller for all scenarios, or one per scenario (contextual BO, BO review Sec. 6.6)? **Open** (§12); decide before the protocol is written.
- Downwash: the tuning episodes must contain the neighbour above.

## 7. Procedure and statistics [proposal]
- Optimiser per §4; if GP-BO: `d+4` Sobol points (`d+1` suffices for deterministic tasks; note the "deterministic noise" warning of Sec. 5.1: with fixed seeds, re-querying a point gives no new information), standardised cost, Matérn ARD, logEI. Budget N = k·d.
- **Common random numbers:** one fixed noise-seed set per evaluation; repeat each whole optimisation with **≥ 5 optimiser seeds**; report median and IQR (BO review Sec. 8.4); no averaging of costs across tasks of different scale; inspect the state trajectories of the final gains.
- **Final gains = lowest posterior mean** (GP-BO), or the pre-registered equivalent for CMA-ES (mean of the final population / re-evaluation of the best with fresh seeds).
- Friedman → Wilcoxon → Holm is the BO review's procedure for comparing **optimisers across seeds** (use it in the S0 benchmark). **Tuned vs default on hardware** uses the protocol's effect size + bootstrap CI (`comparative_study_protocol.md`), not Friedman.
- Document before any run: bounds, seeds, budget, plant version (git hash), cost definition, crash rule.

## 8. Hardware step
Per controller, a small confirmation block: default vs tuned gains on one or two study cells, n flights each, order randomised across batteries and sessions (lesson: S1 A4 packs at 3.2–3.3 V in the unseen block). The budget in flights and the deadline are **not decided** (§12). A tuned set that is worse on hardware than the default is reported, not hidden.

## 9. Effects on existing decisions (to be agreed explicitly)
- `docs/comparative_study_protocol.md` l. 28: "Frozen = gains, yaml, firmware, weights … no retuning". The same line says each strategy "was tuned on the hardware gate", but S0–S3 run on Omar's untuned defaults: the pipeline would **make that claim true**. `docs/study_parameter_decisions.md` ("Omar defaults shared by S0–S3") would be superseded by "gains = output of the pipeline, frozen after it". Protocol v1.0 is not frozen yet; it needs the supervisor's go. (Protocol §2 rows still describe the controller-6 S0 with `ki_z 16`, superseded by its header but easy to quote wrongly.)
- The **pilot, unseen-block and 10-29 meeting results** (including S3 < S1 on A1) become **default-gain results**. The NS2 verdict (`docs/topic2_ns2_quality_summary_2026-10-25.md`) was obtained with default gains; the retention check would have to be repeated or the statement limited to default gains.
- The NS2 training bank (73 flights) was flown under default S0 gains: check that the relative-state distribution of tuned S2/S3 stays inside it.
- The tuned hardware block adds to the about 19 flying hours of tier 1.
- It settles the FBL `ki_z` 8× issue (ki_z 12 vs Kpos_Iz 1.5) in a principled way, a positive point.
- If INDI filters are tuned, the image changes (new hash, reflash both drones, logging patch retained).

## 10. Work packages (effort in working days: **rough estimates, unreviewed**)
| WP | Content | Effort | Output | Gate |
|---|---|---|---|---|
| 0 | Summaries, repo analysis, this plan, independent review, revision | done | docs | review incorporated |
| 1 | Document what the shared repo does (not "match Table I": the shipped checkpoints do not reproduce it) | ≤ 1 | note | — |
| 2a | One harness for S0–S4, INDI realism, a downwash truth that is not the NS2 network | 5–10 | harness | runs all five controllers |
| 2b | Noise and seed spread of the plant (determinism and wall time are measured, §14) | 1 | measurements | decides the optimiser |
| 2c | Pre-registered per-controller plant gate (§5) | 2–3 | report + decision on the plant | passes, or the controller is tuned on hardware instead |
| 3 | S0 pilot: CMA-ES vs Sobol vs GP-BO, objective, bounds, logging | 4–6 | runnable pipeline | S0 tuned beats default on the validation settings, or we understand why not |
| 4 | Extend to S1–S4 (including FBL) with the identical configuration | 4–8 | tuned sets per controller | identical procedure, documented |
| 5 | Validation in simulation; hardware confirmation | 2–3 desk + lab (budget open) | results table | pre-registered rule |
| 6 | Protocol v1.0 update, image hash, supervisor's go | 1–2 | protocol | go |

## 11. Risks
1. Plant not good enough (E1) → tuning optimises the simulator. Mitigation: per-controller gate §5, hardware step §8.
2. Wall time per episode is about 40–60 s per two-drone episode on one core (§14); 22 cores here, so a few hours of compute per full pipeline run. The noise and seed spread of the plant is still unmeasured.
3. Over-fitting to tuning scenarios (DiffTune showed this on trajectories and on the simulator). Mitigation: disjoint tuning set, validation set, hardware.
4. Fairness is contestable if parameter sets differ in size/bounds between controllers; rules 5–6 of §2 and the reported d address it.
5. Changing frozen gains invalidates parts of earlier results (§9).
6. **R1:** a tuned S0 may absorb much of the downwash (larger Kpos_Iz/Kpos_D) and shrink the measured benefit of S1–S4. This is a scientific consequence the supervisor should agree to in advance.
7. **R2:** sim-tuned INDI gains hurt on hardware (the DiffTune C(2)/C(3) precedent), because the SIL INDI is near-ideal.
8. **R3:** the unclamped `KI` / `ki_z` can yield gains that only work in a 15–30 s episode (wind-up over a whole flight).
9. **R4:** host-only artefacts (e.g. the fake `usecTimestamp` clock changed Omar's dt, Phase 5 note); the optimum may exploit them.
10. **R5:** hardware availability: cf5 has an open power fault, possibly from the preset 0 flips; it delays A and §8.

## 12. Open decisions for the user / supervisor
1. Black-box (CMA-ES or GP-BO) as the main pipeline, no gradient? Or gradient first?
2. Tune in simulation at all, given E1? Or hardware-only / few-evaluation tuning for S1/S3?
3. One gain set per controller or per scenario (contextual)?
4. Top-drone policy: the top drone (always S0) keeps default gains or gets the tuned ones (G1)?
5. Is it acceptable if a tuned S0 absorbs part of the downwash (R1)?
6. The tuning-set choice (§6: settings disjoint from the study table).
7. Tune INDI filter cutoffs (new image) or only gains?
8. Do S2/S3 get their own gains or inherit S0/S1?
9. Hardware flight budget and the deadline.
10. Skip step B? Thrust-upgraded drone for the optional step A: available? build flag?
11. Objective weights.

## 13. Files
Summaries: [`vonprittwitz2024diffquad`](papers/summaries/vonprittwitz2024diffquad.md), [`cheng2024difftune`](papers/summaries/cheng2024difftune.md), [`krishna2025diffcotune`](papers/summaries/krishna2025diffcotune.md), [`stenger2026bo_tuning_review`](papers/summaries/stenger2026bo_tuning_review.md) (PDFs in `docs/papers/`, git-ignored). Repo analysis: [`tuning_repo_analysis_2026-10-28.md`](tuning_repo_analysis_2026-10-28.md). Review: [`tuning_pipeline_review_2026-10-28.md`](tuning_pipeline_review_2026-10-28.md). Related: `docs/75_Simulation_End_to_End.md`, `docs/sil_study_screening.md`, `docs/fbl/GAIN_TUNING_2026-10-20.md`, `docs/fbl/PORT_SPEC.md`, `docs/study_parameter_decisions.md`, `docs/comparative_study_protocol.md`, `docs/lab_sessions/2026-10-26.md`, `docs/topic2_ns2_quality_summary_2026-10-25.md`.

## 14. Desk measurements (2026-10-28; no hardware; scratch scripts listed at the end)
**A0, FBL port equivalence on logged states** (`experiments/fbl/stage3_port/c/replay_logged.py`, results `experiments/results/fbl_port_replay_2026-10-28/`). The 7 final FBL flights of 2026-10-26 (uSD, cf5, 60 817 log rows per preset) are replayed through the C law (`fbl_law.c`, float32 shared library) and the authors' code (bundle `FBLControl._position_commands` + `compute_fbl_command`, unmodified), with the authors' gains (preset 0) and ours (preset 1). Inputs from the log: position, velocity, attitude, gyro, planned x / x_dot / x_ddot, x_dddot = derivative of the planned acceleration (jerk is not logged), setpoint yaw 0, z residual from `fbl_res_az/jz`. Firmware-style call (`fbl_law_position_dt`, interval passed directly):
| Output | max abs difference preset 0 / preset 1 | worst error / tolerance of `test_fbl_law.c` (0 / 1) |
|---|---|---|
| acceleration command | 6.6e-5 / 1.7e-5 m/s² | 0.05 / 0.02 |
| jerk command | 4.2e-4 / 1.1e-4 m/s³ | 0.04 / 0.01 |
| thrust | 1.3e-7 / 1.4e-7 N | 0.02 / 0.01 |
| body-rate command | 4.0e-5 / 9.9e-6 rad/s | 0.32 / 0.09 |
| torque | 1.4e-7 / 1.4e-8 N m | **8.1 / 1.1** |
- All differences are float32-level. The torque ratio exceeds the unit test's tolerance (atol 1e-8 + 2e-5·|ref|) because that tolerance is tight for torques near zero; the absolute error 1.4e-7 N m equals about 4e-5 rad/s of rate error and traces to the body-rate command. I did not change the tolerance. Conclusion: **no logic difference between the C law and the authors' code on real flight states**; the port is equivalent to float32 precision.
- With the absolute-time call (`fbl_law_position`, float32 time stamp up to 15 s) the jerk differs by up to 1e-2 m/s³ (the unit-test tolerance, ratio 0.97) and the torque ratio reaches 107: this is the float32 time resolution (the drone does not use this call; it passes the interval, `controller_omar_indi.c:253–255`).
- Against the outputs the firmware logged on the drone (preset 1; informative, **not** an equivalence test): median difference thrust 4–11 mN (of about 0.54 N), body-rate command 0.01–0.05 rad/s, p99 0.15–0.67 rad/s. The replay lacks the drone's real estimator timing and the true jerk reference.
- Not covered by this replay (covered elsewhere): the stock inner loop (G7, bit-exact vs `controller_pid.c`), the network (G8), the ROM residual (G2), the y/x residual (zero in the force-only design).

**Wall time and determinism of the SIL (Omar C tree, S0 = `indi 0`, default gains, two drones, `omar_ns2_closed_loop_sil.py --child`):**
| Episode | Simulated time | Wall time, 1 job | 3 jobs in parallel |
|---|---|---|---|
| A8, 4 passes | 33 s | 58.8 s (≈ 1.8 s wall per simulated second) | 64–68 s each (+10 %) |
| A1, 15 s hold (scratch variant) | 22 s | 39.3 s | not run |
- 22 cores on this machine, about 50 MB per process: episodes parallelise almost linearly. Rough budget (estimate): one candidate over A1 + A8 + one A4 (about 150 s) on 22 cores; N = 20·d = 220 candidates ≈ 25 min per optimisation run, ×5 optimiser seeds ×5 controllers ≈ 10 h. This favours a parallel black-box optimiser (CMA-ES) over a sequential GP-BO, but both fit.
- **Determinism:** 4 runs (1 serial + 3 parallel) of the same A8 episode gave byte-identical result JSON (all floats). **Caveat:** this harness has **no sensor noise and no random number**, so it is trivially deterministic; the noise and seed spread of the noisy screening harness (`ns2_closed_loop_sil_sim.py`, controller 6) was not measured.
- Plant gap, one more data point (scratch variant, metric windows not matched to the hardware analysis): S0 A1 dz 0.5, mean z error over the hold **+1.1 cm in the SIL**, against about **−16 to −20 cm on hardware** (`tree_comparison_2026-10-19.md`, unseen block S0 −19.7 cm at dz 0.45). The SIL S0 compensates the downwash almost fully, as E1 says. This is evidence for E1, not a pre-registered gate.

**Two controller-9 drones with independent parameters:** yes at the parameter level. `oot4` keeps a per-vehicle `controllerOmarIndi_t` (`crazyflie_sil.py:332`), so `indi`, `Kpos_I` and the other gains are separate objects (the "process-global host gains" statement of `docs/72` applies to the Rust controller 6). Test (scratch, hover, no coupling): drone 0 (`indi 0`, Kpos_Iz 1.5) and drone 1 (`indi 3`, Kpos_Iz 0) in one process kept their own values, and drone 0 in the pair is **bit-identical** to its solo run; drone 1 (INDI, RPM path) differs from its solo run by at most 5.8e-5 m at 8 s, so there is a small cross-talk through some shared host state (cause not investigated). NS2 and the peer packets are shared globals (`g_rnn_en`, `oot_set_peer`).

**FBL `inner 1` in the SIL:** yes. With `omar_indi_control.fbl.en = 1`, `inner 1`, `nn_en 1`, `c_d 0.55` on the bottom drone of the same two-drone harness (A1 15 s hold, plant downwash = bank network) FBL ran 11 017 steps, 0 faults, 0 trips, hold mean z error −0.04 cm, z RMS 1.0 cm (scratch variant). The stock attitude / rate PIDs are file-static, so **two** FBL `inner 1` drones in one process would share them; the study has one FBL drone (the top drone is S0), so this does not matter. The separate C-in-the-loop plant (`stock_inner_sim.py`) remains the validated one for FBL gains.

Scratch scripts (not in the repo): `/tmp/claude-1000/-home-georg-Desktop-flying-robot-course/ba62da08-cf9a-4757-8613-65fbc31ca514/scratchpad/omar_a1_sil_scratch.py`, `…/omar_a1_sil_scratch2.py`, `…/two_oot4_indep.py`. Repo script: `experiments/fbl/stage3_port/c/replay_logged.py` (run `--api dt` and `--api abs_t`; system-independent, uses the pyenv python and gcc).

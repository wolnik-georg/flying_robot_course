# Handover prompt: apply the tuning-pipeline review, then start the desk work (2026-10-28)

Paste everything below the line to the agent that wrote the plan, the summaries and the repo analysis.

---

You wrote `docs/tuning_pipeline_plan_2026-10-28.md`, `docs/tuning_repo_analysis_2026-10-28.md` and the four summaries in `docs/papers/summaries/` (`vonprittwitz2024diffquad`, `cheng2024difftune`, `krishna2025diffcotune`, `stenger2026bo_tuning_review`). An independent review is done: **`docs/tuning_pipeline_review_2026-10-28.md`**. Read it in full first. Its tables (§1a–1c) give the evidence and the exact suggested wording. This prompt is the to-do list.

**Verdict of the review:** the work can start, but not as planned. The summaries need small corrections. The repo analysis needs one real correction plus missed items. The plan has three structural problems (E1–E3 below).

## Rules

- Do not commit or push unless the user asks. Never add a Co-Authored-By trailer.
- Never fly, flash or touch hardware. Analysis uses complete uSD logs only.
- Python: `~/.pyenv/versions/flying_robots/bin/python`. System `python3` is only for the firmware bindings.
- Scratch work goes in a scratch folder, not in the repo.
- Do not take the review's claims on trust. Spot-check every claim tagged V that you change, against the PDF, the code or the doc it cites. Where you disagree, write that down with your evidence instead of applying the change.
- Do not edit the review file. Do not touch `docs/meetings/2026-10-29.md`; it has uncommitted user edits.

## Task 1: fix the four summaries (review §1a)

- **von Prittwitz**
  - Patience: Alg. 1 resets `idec` only after a doubling. It doubles once the loss has failed to improve P + 1 times since the last doubling, not necessarily in a row. The text on p. 2 contradicts this; say so.
  - Eq. 3b sums over t, but the text says "MSE" and Table II says "avg."; note this.
  - Add the attitude gains from Table I. Yaw collapsed: KR_z 1e-4, Kω_z 9.7e-6. "z gains stay near 1" holds only for Kp and Kv.
  - Year: "2024 (PDF date 2024-11-01; no venue printed)".
- **DiffTune**
  - DiffTune+ (L4DC 2023) is an extension, not an earlier version.
  - The 3.5× (61.8 → 17.7 cm) is against the initial guess. Against hand-tuned it is 28.6 → 15.4 = 1.86×. Note that Table V gives 17.7 and Tables IV/VI give 15.4 for the same cell.
  - Experiment-tuned beat sim-tuned only on C(2) and C(3). On C(1), Sim₅₀ 4.3 cm beat ExpT 5.7 cm. The sim runs used α 0.01 and 50 trials.
  - Add the qualifiers to the baseline comparison:
    - baselines ran 10 times each;
    - GIBO was best on the Dubins car after 20 trials;
    - the "BO is inferior" result is SafeOpt-PSO/GIBO with fixed settings, not GP-EI.
  - Add the hardware facts:
    - loss = position + rotation error;
    - 0.63 kg Pixhawk/ArduPilot at 400 Hz;
    - Table VII initial gains: kp 10, kv 2/1/1.5, kR 0.5/0.3/0.14, kΩ 0.11/0.1/0.01.
  - Generalisation numbers: P1 on C(3) 72.8 vs hand 28.6 cm. P1 is worse than hand on all 4 lemniscate cells.
- **DiffCoTune**
  - "≤ 5 rollouts" → "L = 5 tuning iterations (rollout count inconsistent: '< 5' in the text, l = 0..L in Alg. 1)".
  - Split-Alternate gives K/2 epochs to each part; termination is at a change < 1e-3.
  - Fig. 5 shows jump lengths: target 40, DiffCoTune 34, DiffTune 27 (falls), untuned 20 cm.
  - Vanilla BO found the best final solution, after 90 trials.
  - Optional: the biped MPC result, +29.29 %.
- **Stenger (BO review)**
  - Scope of the 110 papers: hardware online tuning, up to 2025.
  - "Sweet spot < 10 parameters" is common practice, not a limit. Sec. 5.2 recommends logEI for high dimensions.
  - Acquisition: MES for noisy low-d problems, logEI for high-d low-noise. Add the Sec. 5.1 "deterministic noise" warning.
  - **Wrong example:** the degenerate-gains case is minimising *energy* without a tracking constraint (zero gains optimal), not pure tracking.
  - Benchmark: 40 evaluations, **5** Sobol starts (d + 1). Only LogEI was tested against the others, so the result is "LogEI significantly better than Sobol".
  - Add the missing sections (review G7):
    - 4.1 / 8.2: BO is for expensive, noisy evaluations; CMA-ES, pattern search and PSO are competitive when evaluations are cheap; Sobol baseline;
    - 6.3: crash constraints, BO-VDP or a classifier instead of a fixed penalty;
    - 6.6: contextual BO;
    - 6.9: TuRBO;
    - 6.10: multi-fidelity and early stopping;
    - **7.5: BO on a high-fidelity sim to compare controller structures fairly (Funk 2021, Manhães 2017, Yu 2020), the closest precedent for our plan**;
    - 8.1 / 8.4: shared seeds and designs, equal effort, median + IQR, no averaging across tasks of different scale, inspect trajectories.

## Task 2: fix the repo analysis (review §1b)

- **Gradient-clip correction (most important):** l. 185 clamps the *gradient* to ±|θ|. With SGD lr 1e-3 each step changes a gain by at most 0.1 % (about 3 % per epoch). A 5-epoch run showed kw × 0.999³¹ per epoch. With Adam the clip barely limits anything.
- **Loss weights:** the `--*-weight` arguments do not affect the gradients. Line 180 backprops the unweighted sum; the weights only enter the plateau test (l. 318).
- **`train_system_id.py`:** stale. It uses `.mass`/`.J`, but the model class has `.m`/`.I`.
- **Shipped checkpoints don't reproduce Table I:** they hold kp ≈ 1, kv ≈ 1 and kr_z = 1e-8, so the Table I recipe can't be recovered from the repo.
- **"Uninformed start diverges" is an Euler artefact:** explicit Euler on ω gives a stability bound of kw < 0.0033 (roll/pitch) / 0.0059 (yaw) at dt 0.01. The tuned Kω sits at 60–80 % of that bound, so it is shaped by the 100 Hz step, not by the vehicle.
- **Other corrections:**
  - 2457 lines, not ~2800.
  - The quaternion uses the exponential map, so it is not explicit Euler.
  - `eta = force @ B0.T` is correct; the TODO is a false alarm.
  - The code drops the −RᵀR_d ω̇_d term.
  - eR matches Lee and our firmware.
  - The patience counter resets on any improvement > 1e-4, unlike Alg. 1.
  - Windows start on the reference in position and velocity, but the attitude starts at identity.
  - The noise is zero at hover.
  - The README uses 5000 epochs.
  - The CI workflow is manual and uploads a folder that training never writes.
  - `test_lee_controller.py` has no assertions.
- **Bugs:**
  - `--visualize-trajectory False` crashes on `args.plot_errors`.
  - A bad `--gain-init` builds a `ValueError` but never raises it.
  - gradcheck runs only at hover.
- **Conclusion to add:** the repo is a starting template, not a model of our S0. It lacks the integrals, ω̇_d, 500 Hz, 44 ms lag, the thrust map, 42.7 g and the downwash.

## Task 3: fix the plan (review §1c, §2, §3)

1. **§5 (simulator), critical (E1).**
   - The "A8 validated within ~1 cm" result is for **controller 6**, not the Omar tree.
   - For the tree, the SIL INDI is near-ideal: S1/S3 dip 0.75 cm vs 10.6 cm on hardware. S0 dips −0.4 cm at plant scale 1.0 vs −12.7 cm on hardware (not a matched comparison).
   - The downwash truth in both plants is the **NS2 network**, which is S2/S3's predictor and FBL's ROM target.
   - The FBL C-in-the-loop plant cannot run INDI.
   - The SIL screen used controller 6.
   - Replace the "A1 dz 0.25/0.5 flown cleanly" example: A1 dz 0.5 flew without divergence, with the ±35–40° oscillation, and there is no dz 0.25.
2. **§1 / Step A (E2).**
   - Preset 0 is the authors' **simulation-code** gains (30 g simulated vehicle), not their hardware gains.
   - The authors flew off-board (Vicon → laptop → 400 Hz radio, stock onboard PID).
   - So Step A tests platform transfer, not the port.
   - New port test: a **desk replay** of logged 10-26 states through host `fbl_law` and the authors' Python code, compared to float tolerance. Check `docs/fbl/PORT_SPEC.md` first; the review did not open it.
   - Keep Step A as an optional platform check, with the records listed in review C1. Run B only if C is not approved.
3. **§6 (scenarios) (E3).**
   - Tune on settings **not** in the study table, e.g. A1 dz 0.35/0.6, A8 at 0.25/0.45 m/s, A4 offset 0.10 at another speed.
   - Validate in sim on the unseen-block settings, then test in the study.
4. **§3 (parameters) (E5).**
   - `Kpos_Ix/Iy` are inert while `Kpos_Iz ≠ 0`.
   - The P/D limits are inactive (100 m).
   - `KI` is an unclamped integral of eR.
   - "Omar defaults 7/4 plus the study's Kpos_Iz 1.5".
   - Omar tree with x = y tied: about 10–12 parameters, listed in review C6.
   - FBL: kr/J are unused with `inner 1`. The real inner tunables are the stock `pid_rate` gains.
   - Drop the 30 Hz filter hedge: the live source is identical.
5. **§4 (method).**
   - Attribute "BO inefficient > 20 parameters" to DiffTune/DiffCoTune only.
   - "Only S0 is differentiable" is overstated; the real argument is effort.
   - Choose the method **after measuring wall time**: CMA-ES if episodes are cheap and parallel (benchmark against Sobol and GP-BO on S0), GP-BO (logEI, BO-VDP) if they are slow or on hardware.
6. **Fairness list.** Add review C2 (a)–(j):
   - plant fidelity per controller;
   - downwash truth independent of every compensator;
   - disjoint tuning set;
   - budget N = k·d;
   - wide log bounds plus a stability screen, independent of starting maturity;
   - top-drone policy;
   - episodes include takeoff and landing;
   - one crash rule;
   - same optimiser and settings;
   - report d and the convergence curves.
7. **Plant gate (C4).**
   - Per controller and per scenario, pre-registered.
   - Hardware mean within max(1.5 cm, 30 %), same sign of resolved pairwise differences, tilt within ±10°, against the flown default-gain data.
   - Split WP2 into 2a (one harness for 5 controllers + INDI realism + a downwash truth that is not NS2), 2b (determinism, wall time) and 2c (gate).
   - If S1/S3 cannot pass, do not tune them in sim.
8. **Statistics (G6).**
   - Friedman → Wilcoxon → Holm is for comparing optimisers across seeds.
   - Tuned vs default on hardware uses the protocol's effect size + bootstrap CI.
   - Use common random numbers and ≥ 5 optimiser seeds, and report median/IQR.
9. **WP1.** ≤ 1 day; the goal is "document what the repo does", not "match Table I".
10. **Housekeeping.** Add efforts to §10 or drop "effort = rough guess".
11. **§9 consequences (G8):**
    - the pilot, unseen and 10-29 results (incl. S3 < S1 on A1) become default-gain results;
    - the tuned hardware block adds to the ~19 h of tier 1;
    - it settles the FBL `ki_z` 8× issue;
    - the protocol's claim "tuned on the hardware gate" becomes true;
    - the NS2 bank was flown under default S0, so check that tuned S2/S3 states stay inside it.
12. **Risks to add:**
    - R1: a tuned S0 absorbs downwash and shrinks the measured benefit. This needs supervisor agreement.
    - R2: sim-tuned INDI gains hurt on hardware (DiffTune C(2)/C(3) precedent).
    - R3: unclamped `KI`/`ki_z` only work in short episodes.
    - R4: host-only artefacts (fake `usecTimestamp`).
    - R5: cf5 power fault.
13. **§12 open decisions to add:**
    - top-drone policy (G1);
    - tuning in sim at all, given E1;
    - per-scenario gains (contextual);
    - acceptability of R1;
    - the tuning-set choice;
    - the hardware flight budget and deadline;
    - whether to skip Step B.

## Task 4: start the desk work (review §5, step 1). No hardware.

1. **FBL port equivalence:** read `docs/fbl/PORT_SPEC.md`, see what is already covered, then run or extend the replay of logged 10-26 states through host `fbl_law` vs the authors' code. Report the max abs difference per output.
2. **SIL measurements:** wall time per two-drone episode (A1 15 s hold, A8 4 passes) for S0 at default gains, and bit-for-bit determinism for the same seed (run 3×, compare costs). Also check:
   - whether the SIL can run two controller-9 drones with independent parameters ("host gains are process-global", `docs/72`);
   - whether FBL `inner 1` runs in it.
3. **Report, then stop:** what changed in each file (a short diff summary), every review point you rejected with your evidence, and the measurements from steps 1–2. Do not start WP2 or any tuning until the user has answered plan §12 with the supervisor.

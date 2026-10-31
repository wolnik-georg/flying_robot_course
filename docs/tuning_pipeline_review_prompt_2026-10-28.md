# Review prompt (for Opus): validate the tuning-pipeline plan, the four paper summaries and the repo analysis — find gaps and errors (2026-10-28)

## Your role
You are an independent, critical reviewer. Do not trust the documents below: **recompute and re-read the sources**. The goal is that, before any real work starts, everything is 100 % correct: that each summary says what its PDF says, that the repo analysis says what the code does, and that the plan is sound, complete and consistent with the project facts. Say plainly when something is wrong, overstated, missing or unverifiable.

## Context (short)
Master's thesis on downwash compensation with two Crazyflie 2.1 Brushless drones. Five strategies: S0 geometric, S1 INDI, S2 geometric + NS2 network, S3 INDI + NS2, S4 FBL (flatness-based feedback linearisation of the authors' paper, `docs/papers/summaries/hsieh2026flatness.md`). S0–S3 share one firmware controller tree (Omar C, `stabilizer.controller 9`). Next steps (user, 2026-10-28):
- **A**: run FBL with the authors' gains on a thrust-upgraded Crazyflie; **B**: short FBL gain touch-up on the brushless drone; **C**: before the comparative study, a **fair automatic gain/filter tuning pipeline** applied identically to every controller. The supervisor and professor supplied four papers and the repo `IMRCLab/differentiable-quadcopter` for C. NS2 (Topic 2) is closed and is not under review.

## What to review (all paths relative to the repo root `flying_robot_course`)
1. **Plan:** `docs/tuning_pipeline_plan_2026-10-28.md`.
2. **Summaries (4):** `docs/papers/summaries/vonprittwitz2024diffquad.md`, `cheng2024difftune.md`, `krishna2025diffcotune.md`, `stenger2026bo_tuning_review.md`.
3. **Sources (PDFs, git-ignored, on the user's disk):** `docs/papers/vonprittwitz2024diffquad.pdf`, `cheng2024difftune.pdf`, `krishna2025diffcotune.pdf`, `stenger2026bo_tuning_review.pdf`. Read **each paper in isolation**, one at a time; do not carry claims from one to another. Note the renaming: former names `DiffTune_Autotuning_Through_Autodifferentiation.pdf`, `2505.24068v1.pdf`, `2609.09403v1.pdf`.
4. **Repo analysis:** `docs/tuning_repo_analysis_2026-10-28.md`; the repo itself: `git clone https://github.com/IMRCLab/differentiable-quadcopter` into a scratch folder (HEAD should be `3650d09`). Do not run untrusted code outside a throwaway environment; reading is enough, running `quadrotor_pytorch.py`'s gradcheck and one short `train_lee_controller.py` run is a bonus.
5. **Project facts the plan relies on:** `docs/75_Simulation_End_to_End.md`, `docs/sil_study_screening.md`, `docs/72_Simulation_Showcase.md`, `docs/fbl/GAIN_TUNING_2026-10-20.md`, `docs/study_parameter_decisions.md`, `docs/comparative_study_protocol.md`, `docs/lab_sessions/2026-10-26.md`, `docs/tree_comparison_2026-10-19.md`, `flying_drone_stack/firmware_app/host/controller_omar_indi.c.snapshot` (and the live firmware source if you can find it), `experiments/fbl/stage3_port/c_in_the_loop_real.py`, `experiments/analysis/unseen_block_analysis.py`, results in `experiments/results/unseen_block_2026-10-23/`.

## Questions to answer, in this order
**A. Summaries (claim check, per paper).** For every number and factual statement in each summary: confirmed / corrected / not in the paper / cannot verify, with page or section. In particular:
- von Prittwitz: the trajectory-length schedule parameters (T0, P, Tmax), the clamp and 10 % clip, Table I gains, and **every cell of Table II** (the summary states that auto-tuned position error is worse than hand-tuned on all three test trajectories; check each comparison). Is "venue not printed" true? Is the 2024 year justified?
- DiffTune: Θ definition, the ranges and initial gains of the quadrotor simulation, the hardware setup (speeds, ten trials, step limit, learning rate, 7 s horizon), the 3.5× claim, the P2 early stop, the sim-versus-real crash statement (which trajectories), the comparison with AutoTune / SafeOpt-PSO / GIBO. Also read Tables III–VII, which the summary did not transcribe, and say whether anything there contradicts or qualifies the summary.
- DiffCoTune: split-alternate vs combined, L = 5, K = 100, weights, Adam, the 60 % and 1.04 m statements, the BO comparison (cart-pole LQR only?), what Fig. 5 actually shows for the biped.
- Stenger et al.: the recommendations quoted (d+4, scaling, posterior-mean selection, kernel, acquisition), the benchmark statistics (20 seeds, Friedman / Wilcoxon / Holm), the "fewer than ten parameters" sentence, the claim of 110 reviewed papers. The summary says Sec. 6.2–6.10 and Sec. 7 were read only by heading: **read those** and report anything relevant to us that is missing (safe BO, crash constraints, multi-fidelity, high-dimensional BO, time-varying/contextual BO, hardware-in-the-loop tuning, noisy evaluations, any fairness or benchmarking rules).
**B. Repo analysis.** Is every statement in `docs/tuning_repo_analysis_2026-10-28.md` correct (file list, defaults table, "no integral", "no motor dynamics", window cap 350, clip semantics, noise option, gain init, mass, initial state from the setpoint)? Look for bugs or inconsistencies in the dynamics (e.g. Euler integration of quaternion and rates, torque / mixer conventions, sign conventions of the error terms, the rate-reference computation `computeWd`, units) and say whether the model can be trusted as a differentiable S0. Does the code agree with the abstract's Table I? What else does the code show that the analysis missed?
**C. The plan.**
1. Is the A → B → C order and its gates logical? Is Step A (authors' gains on a thrust-upgraded drone) a valid test of "the port is correct", or can a correct port still fail there for hardware reasons (and a wrong port pass)? What should be recorded to make the result interpretable?
2. Fairness: is the list in §2 sufficient? What would a reviewer or examiner attack (different parameter counts and bounds, different controller maturity, unequal plant fidelity per controller, tuning scenarios = test scenarios, randomness)? What is missing?
3. Method choice (§4): are the arguments for black-box BO vs gradient correct? Is "only S0 is realistically differentiable" true given the actual S1–S4 code? Would another black-box method (CMA-ES, random search, successive halving, Bayesian optimisation with different settings) be more suitable for 15–20 parameters and a 1 ms simulation? Give a reasoned recommendation.
4. Plant (§5): is the evidence about SIL validity and the A1/A2/A5 divergences stated correctly (check `docs/sil_study_screening.md` and the flown results)? Is the proposed plant-validation gate adequate, and what would "close enough" need to look like? Is there any problem in using the host-compiled firmware as a tuning objective (determinism, speed, noise, 1 ms tick, wall time per episode)?
5. Objective, scenarios, statistics (§6–§7): sensible and unbiased? Any leakage between tuning, validation and the later comparative study? Is the proposed use of the unseen-block settings as validation OK given they were used in the NS2 check? Are the statistics (seeds, posterior mean, tests) applied correctly?
6. Parameters (§3): verify the list against the live firmware (parameter names, which are yaml-settable, which are compile-time constants such as the 30 Hz INDI filters, meaning of `KI`, limits). Anything tunable that is missing, or listed but not actually a parameter?
7. Consequences (§9): check the statements about the protocol, `study_parameter_decisions.md` and the NS2 verdict.
8. Anything wrong, missing or unrealistic in the work packages, effort, risks and open decisions.
**D. Cross-document consistency.** Does anything in the plan, summaries or repo analysis contradict the other project docs (e.g. gains, hardware facts, Strategy definitions, the FBL lab notes)?

## Rules
- **Read only.** Do not edit the plan, the summaries, the repo analysis, any other existing file, or the PDFs. Do not commit or push. Never fly, flash or touch hardware. Do not add a Co-Authored-By trailer to anything.
- Write only **one new file**: `docs/tuning_pipeline_review_2026-10-28.md`. Scratch work goes in a scratch folder, not in the repo.
- Python for analysis: `~/.pyenv/versions/flying_robots/bin/python`; system `python3` only for firmware bindings.
- Do not invent facts. If you cannot verify something, say "unverifiable" and why. Mark your confidence per finding: verified / strong evidence / hypothesis / unknown.

## Deliverable: `docs/tuning_pipeline_review_2026-10-28.md` (short, tables first)
1. Claim check table, one row per checked claim: document, claim, verdict (confirmed / corrected / not in source / unverifiable), evidence (page, section, file:line), suggested correction.
2. Findings ranked by severity: errors that must be fixed; gaps; risks; things that are fine.
3. Plan review: for each question C1–C8 a short answer and a concrete change request.
4. Missing information you would need (documents, data, decisions).
5. Your recommended revised order of work and the method you would choose, with reasons.
6. Commands you ran, so the review can be reproduced.
7. One-paragraph overall verdict: can the work start as planned, after which fixes?

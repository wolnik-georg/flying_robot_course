# Plan: auto-tuning with DiffCoTune (2026-11-11)

Decided in the meeting with the supervisor and professor on 11 Nov 2026, and with the user afterwards. This replaces the method comparison (CMA-ES, Bayesian optimisation, Sobol sampling, gradient descent) and the per-scenario picks; those results stay on disk only as the earlier record. All earlier tuning jobs were stopped on 11 Nov at 15:49 (clean break); no files were deleted.

Method source: Krishna et al. 2025, "DiffCoTune: Differentiable Co-Tuning for Cross-domain Robot Control", arXiv:2505.24068 (`docs/papers/krishna2025diffcotune.pdf`). We follow the paper as closely as possible; every deviation is listed in section 3.

## 1. Decisions

| Point | Decision |
|---|---|
| Method | DiffCoTune, Split-Alternate scheme (Alg. 3), real flights and simulator in a loop |
| Gain sets | One gain set per controller, valid for all scenarios |
| Controllers (8) | S0, S1, S2, S3; FBL base; FBL + downwash model (ROM); FBL + learning-based part (net); FBL + both. Each is tuned on its own; S2 / S3 no longer reuse the S0 / S1 gains |
| Task cost | Mean squared 3-D position tracking error (the paper's cost); reported as 3-D RMS in cm, z error reported as well |
| Not in the cost | Crash penalty, delay robustness (md40 / md60), selection-error weighting |
| Fitted simulator values β | Max motor force (thrust factor), motor lag, RPM delay, downwash scale, plume amplitude scale |
| Fixed | Mass 0.0427 kg and inertia J = 16.571710e-6 / 16.655602e-6 / 29.261652e-6 kg·m² (Omar C tree), sensor noise |
| Fitted simulator | One per controller (paper-exact): every loop starts from the same calibrated simulator and refits it on its own flights |
| One round | One real flight of each tuning scenario, then fit β, then tune θ |
| Rounds | l = 0 … 5 (6 real rounds per controller, paper L = 5) |
| Tuning scenarios | A1, A8, circle, figure 8 |
| Test scenarios (never tuned on) | A4, oval (more can be added) |
| Before the lab | Simulator-to-simulator dry run with a shifted simulator as the "real drone" |

## 2. The method as we run it

For each controller separately:

1. **Start:** θ₀ = default gains of that controller; β₀ = the frozen calibration (`simulator/frsim/data/calibration.json`).
2. **Real round l:** fly θ_l once on each tuning scenario (A1, A8, circle, figure 8), with uSD logging. Record the real states x_t^sys and the reference x_t*.
3. **Fit the simulator** (θ_l fixed): minimise
   J_sysId(β) = mean over scenarios of (1/T)·Σ‖x_t(β) − x_t^sys‖²,
   with x_t(β) the simulated closed loop of the same controller, gains and reference. Adam, up to K/2 = 50 epochs.
4. **Tune the gains** (new β fixed): minimise
   J_task(θ) = mean over scenarios of (1/T)·Σ‖x_t(θ) − x_t*‖²
   in the refitted simulator. Adam, up to K/2 = 50 epochs.
5. **Stop rule per inner loop** (paper): stop when the change is < 1e-3, or when the objective rises by more than 1e-3.
6. **Keep the best:** θ_best is replaced only if the real task cost of the new gains (measured in the next round) is not worse. After round 5, θ_best is the best of the 6 flown gain sets and becomes the controller's final gain set.

States used in J_sysId (paper: all observable states, unweighted): position, velocity, attitude and body rates, from the uSD logs only, never radio CSVs. Unweighted sum of the squared errors of each part, as in the paper.

Gains are clipped to stay positive (the paper clips its biped controller weights to keep them valid); no other bounds.

The tuning simulator runs without sensor noise (the paper's model rollouts are deterministic).

## 3. Deviations from the paper (stated in the thesis)

No official DiffCoTune code exists: the paper links only a video, and arXiv has one version (v1, 29 May 2025) with no code link. "As in the paper" therefore means Algorithms 1 and 3, the formulas and the stated settings. The closest real code is DiffTuneOpenSource (Cheng, MATLAB, gain tuning only, no simulator fit).

Unavoidable:
1. **Quadrotor**, not cart-pole, quadruped or biped; the paper has no quadrotor example.
2. **PyTorch** (our differentiable copies of S0–S3 and FBL), not JAX / MuJoCo-XLA.
3. **Our β values** (thrust factor, motor lag, RPM delay, downwash scale, plume amplitude) instead of link masses, friction, gear ratio; mass and inertia stay fixed (measured).
4. **Learning rate:** the paper does not state it; we choose one and state it.

Decided (user, 11 Nov):
5. **Several tasks:** the paper uses one task from one start state per loop; we average the cost over 4 tuning scenarios, one flight each per round, because one gain set must serve all scenarios. This is the largest deviation.
6. **Task cost on position only** (3-D position error); the paper uses the full desired state, unweighted. Position is what the study measures; a quadrotor's desired attitude is only derived.

Our reading of unclear points:
7. **Keep-best after the last round:** Alg. 1 needs the real cost of θ_{l+1}, but the last update is never flown; the paper also gives the rollout count as < 5, L = 5 (l = 0 … L) and figures with 5. We fly l = 0 … 5 and pick θ_best among the 6 flown gain sets; the last update θ₆ is not used.

## 4. Stages

### Stage 0: clean break (done 11 Nov)
- All tuning jobs stopped, daily check cancelled, files kept.
- To do: one closing line in `docs/tuning_procedure_frozen_2026-11-01.md` and `out/csim_status.txt` ("stopped 11 Nov 15:49, superseded by DiffCoTune"), and update `docs/07` next-action line.

### Stage 1: specification and inventory (desk)
- Write `docs/diffcotune_spec.md`: sections 1–3 of this plan as the frozen procedure, with the exact formulas, hyperparameters and stop rules.
- Inventory:
  - which PyTorch copies exist (`diff/s0..s3_plant_torch.py`, `fbl_plant_torch.py`, `ns2_torch.py`), which plant they use (the FBL copy names J_FBL; it must move to the Omar inertia and mass);
  - which of the 5 β values are already differentiable inputs;
  - whether the FBL firmware and the copies can switch ROM and net on / off separately (4 FBL variants);
  - what is missing for circle, figure 8 and oval (the simulator already has circle and lemniscate curves; oval is new).
- Output: a gap list. No code changes yet.

### Stage 2: simulator work (desk)
1. One plant for all 8 controllers: Omar inertia and mass.
2. β as differentiable inputs in all PyTorch copies.
3. The 4 FBL variants as switches (simulator and PyTorch copies).
4. New scenarios circle, figure 8, oval in the simulator package (`frsim`) and the PyTorch copies.
5. **Forward check:** for every controller × scenario, the PyTorch copy matches the firmware simulator (`frsim`) within a stated tolerance on fixed seeds. Without this, the tuning optimises a different system.
6. **Gradient check:** finite differences vs autograd for θ and β.

### Stage 3: dry run simulator-to-simulator (desk, no flights)
- "Real drone" = `frsim` (real firmware code) with shifted β, e.g. thrust factor × 0.7, motor lag 45 ms, RPM delay 8 ms, downwash scale × 1.3.
- Run the full loop (6 rounds) for all 8 controllers.
- Pass criteria:
  - β moves toward the shifted values;
  - the "real" task cost of θ_best is lower than for the default gains, on the tuning and on the test scenarios;
  - one round's desk time (fit + tune) is short enough for a lab session (target ≤ 15 min).
- Also run "tune only, no fit" (paper ablation) as the comparison.

### Stage 4: lab preparation (desk)
- Flight files and safety bounds for circle, figure 8, oval; A1 / A8 / A4 as before.
- FBL variant switches available on the drone (if this needs a new image: build, then flash in the lab with the usual checks).
- One command per round: ingest the uSD logs → fit β → tune θ → write the next gain file (crazyswarm2 yaml) + a round report.
- Flight card per round; uSD card file-count check before every block.
- Protocol for the loop (frozen before the first flight): rounds, scenarios, keep-best rule, what happens after a crash (the flight is repeated; it is not used for the fit).

### Stage 5: lab tuning loop
- Per controller: 6 rounds × 4 tuning scenarios = 24 flights; 8 controllers = **about 190 tuning flights** (plus repeats).
- Order: S0 first (simplest, end-to-end check on hardware), then S1, S2, S3, then the 4 FBL variants.
- After each round: run the round command, check the report, load the new gains.

### Stage 6: test flights and results
- Final gain set of each controller, plus default gains, on the test scenarios (A4, oval) and the tuning scenarios; several flights per cell.
- Results: 3-D RMS and z error per controller, tuned vs default; the fitted β per controller over the rounds; real cost per round (learning curve).
- Thesis: method section from the spec; results; the per-scenario method comparison as earlier background only.

## 5. Risks

- **Lab load:** about 190 tuning flights plus test flights; 2-drone scenarios (A1, A8, A4) need the top drone in every flight.
- **Sim-to-PyTorch gap:** the forward check (Stage 2.5) must pass, otherwise the tuning is biased.
- **Identifiability:** 5 β values from 4 flights per round; the dry run shows whether all 5 can be fitted or some must stay fixed.
- **Crashes during tuning flights:** the paper has no crash handling; the protocol repeats the flight.
- **Local solution:** the paper finds a solution near the start gains; the defaults are the start.

## 5b. Decisions after Stage 1 (user, 11 Nov evening)

1. **Thrust β = thrust-law gain** (`THRUST_LAW_GAIN_N`, the gain of the quadratic law), not the force cap; the cap `max_motor_force_N` stays fixed.
2. **β per controller = only the values that act on it:** RPM delay only for S1 / S3 (needs a smooth, fractional delay to be differentiable); downwash and plume scales are fitted from the 2-drone flights (A1, A8) of the same round; circle and figure 8 are flown solo.
3. **FBL firmware: flash `fbl_v2b`**, the image the simulator represents (199/199 equivalence); lab check (active slot, bench check, short hover) before FBL tuning.
4. **4 FBL variants kept:** a net-only model (no ROM) is trained for "FBL + learning-based part".
5. **S0 first end to end:** Stages 2 and 3 for S0 only, then the other controllers.
6. Followed from the paper (spec): each update starts from the last flown gains θ_l; the cost is rescaled so that the 1e-3 stop rule is meaningful (unit / normalisation chosen in the dry run).

## 5c. Decisions after Stage 2 (user, 11 Nov evening)

1. Gradient gate on the combined 4-scenario cost: same sign and (relative ≤ 1 % or absolute ≤ 0.1 % of the largest component); single scenarios reported only.
2. CIRCLE / FIG8: flight-file shape and lap time, smooth ~2 s ramps.
3. OVAL: long axis 1.8 m, same path speed; stays a test scenario.
4. Stop rule on the cost normalised by its value at the start of each inner loop (confirmed in the dry run).

## 5d. Decisions after the Stage 3 dry run (user, 12 Nov)

The S0 dry run showed two setup faults: the normalised stop rule (5c.4) stopped every β fit after 2–5 epochs, so β never moved; and body rates made up 94 % of J_sysId under sensor noise, which hid position and velocity.

1. **Stop rule:** no early stop. Each inner loop runs the full 50 epochs and returns the iterate with the lowest model cost. Replaces 5b.6 / 5c.4. Deviation 8 (stated): the paper stops at |ΔJ| < 1e-3 or a rise > 1e-3 and returns the last iterate; K = 100 is kept.
2. **J_sysId states:** body rates dropped; position, velocity and attitude, unweighted. A short identifiability check on noisy flights first decides between pos + vel + att and pos + vel. Deviation 9 (stated): the paper uses all observable states.
3. **Learning rate:** pilot redone after 1 and 2.
4. **Update start:** stays θ_l (paper); re-checked in the rerun.
5. **A4:** stays a test scenario. If tuned gains are still worse than the default on A4 after the fixes, that is reported as a finding; adding an A4-like lateral pass to the tuning set is the option then.
6. **Rerun** of the dry run (P1, P2, comparisons (a)–(d), same pass criteria; wall-time target about 20 min per round).

## 5e. Decisions after Stage 3c (user, 12 Nov)

Stage 3c found the cause of the drift: the β fit works (started at the truth it stays there), but the noise-free tuning model pushes the gains to where sensor noise keeps the drone on the floor; even with the true β one tuning step is no better than round 1. Decision: a short P1 comparison (rounds 0–3) of A = sensor noise in the tuning step (2 fixed seeds per round; deviation 10), B = per-round gain limit ±25 % around θ_l (deviation 11), and A + B; plus a floor rule in the crash check. Full runs and P2 only after that. Prompt: `docs/HANDOVER_DIFFCOTUNE_STAGE3D_NOISE_TRUSTREGION_2026-11-12.md`.

## 5f. Decisions after Stage 3d (user, 13 Nov)

A + B (noise in the tuning step, 2 seeds per round; ±25 % per-round gain limit) is the method version: in the P1 comparison it was the only variant whose real cost kept falling (19.8 / 10.2 / 8.5 / 6.9 cm; final 5.60 / 6.89 against default 15.29 / 9.04). The noisy gradient check (19 of 20 pass) is accepted. Next: check the torch compiler workaround and the load gate; full A + B runs (P1 with new seeds, 6 rounds, then P2); in parallel the Stage 4 desk work for S0 (real uSD ingest, round command, flight files matching frsim, lab protocol draft). Prompt: `docs/HANDOVER_DIFFCOTUNE_STAGE3E_FULLRUN_STAGE4A_2026-11-13.md`.

## 5g. Decisions after Stage 3e / 4a (user, 13 Nov)

B1 (P1, 6 rounds) and B2 (P2) pass a, b and c; final P1 6.39 / 7.76 against default 15.36 / 9.02 cm. Rounds 4–5 add little (P1) or nothing (P2, stall after round 2). Desk tools for the S0 lab exist (ingest, `round.py`, flight files, protocol draft). Next, before the lab: round-1 dry test, the gyro / attitude check on the 23 Oct flights, and the cause of the P2 stall. Open with the user: the protocol's open points and the number of lab rounds (3–4 instead of 6, for the supervisor). Prompt: `docs/HANDOVER_DIFFCOTUNE_STAGE4B_S0_CHECKS_2026-11-13.md`. The agent session was killed by systemd-oomd at 14:52:53 (two worker pools at once); resume prompt for a new agent: `docs/HANDOVER_DIFFCOTUNE_STAGE4B_RESUME_2026-11-13.md`.

## 5h. Decisions after Stage 4b (user, 13 Nov)

Stage 4b passed: the `round.py` dry test matches `loop_lib` over rounds 0 and 1 (θ₂ identical, strict on and off). Gyro mismatch = dynamics gap (A1: real and model rock at 1.27 / 1.07 Hz, uncorrelated phase), not an ingest error. P2 stall = cost surface rough on the scale of one Adam step, β fine.
1. **Lab rounds:** 4 (l = 0…3) instead of 6; rounds 4–5 gained 0.4 cm (P1) / nothing (P2). For the supervisor.
2. **Abort rule** "|roll| or |pitch| > 25° for 0.5 s" is read as 0.5 s continuous (the 23 Oct S0 A1 flights: longest run 0.22–0.25 s, bottom drone up to 40° pitch).
3. **J_sysId** keeps position + velocity + attitude on A1 too.
4. **P2 scan through θ₂:** skipped for now.
Next: the user closes the remaining protocol open points (scenario order and flights per scenario, `rotate_deg` / command lines, how the yaml is applied, gain readback, file names for `simple_flight.py`, battery rule), then a real S0 round 0 (4 flights) and `round.py` on it.

## 5i. Decisions after Stage 4c (user, 13 Nov)

Stage 4c (protocol proposals) had a safety error: the pad check took the file origin as the hover point, but a relative `startTrajectory` puts the first point there (`planner.c:335–348`); CIRCLE would reach x = −1.5 m at the cf5 pad. Decisions: gains merged into `crazyflies.yaml` per round + readback on a fresh connection; `fly_file.py` wrapper after the check is fixed; 1 flight per tuning scenario per round; order A1, A8, then CIRCLE, FIG8; circle turned 90°; OVAL dropped from the lab test set (A4 ×2 stay). Open: number of test flights, battery rule. Prompt: `docs/HANDOVER_DIFFCOTUNE_STAGE4D_FLIGHTSPACE_FIX_2026-11-13.md`.

## 5j. Decisions after Stage 4d (user, 13 Nov)

Stage 4d verified: the pad check counts from the first point; the circle is turned +90° (`rotate_deg` −90 must be given in `flights.json`); OVAL is refused. Decisions: test flights A4 cell 1 + 2, 3 each, with the default gains (round 0) and θ_best (after round 3) = 12 flights; battery rule as in `comparative_study_protocol.md` §4, repeats included; lab times measured in the first block; readback after every launch and battery swap; cf5 pad checked at the lab PC and `fly_file.py --check` rerun; FIG8 passing 0.32 m beside the parked cf_second (1 m below) accepted. The S0 desk side is ready. Before the first real round: the supervisor agrees to 4 rounds and the lab load. Round 0 = default gains, 4 tuning + 6 test flights, then `round.py` (about 20 min).

## 5k. Rounds and lab load (user, 13 Nov evening)

Lab rounds: **5** (l = 0…4, `round.py --n-rounds 5`), replacing the 4 of §5h; the paper has L = 5. Per controller: 20 tuning + 12 test flights + repeats. The lab load is accepted by the user. Still to discuss with the supervisor: the deviations 8–11 from the paper.

## 5l. Paper check before the lab (user, 13 Nov evening)

The user wants the method as close to the paper as possible. Re-reading the paper: J_sysId uses all observable states (Eq. 3), the model is noise-free, the stop rule is absolute 1e-3 with the last iterate (p. 4), the biped uses a clipping projection (p. 5), L = 5; the round-by-round evidence is sim-to-sim without sensor noise. Two deviations were never tested on their own: the paper's literal stop rule (only our normalised version failed) and the body rates in J_sysId (dropped together with the stop-rule fix). Stage 4e tests both on P1 (5 rounds, B1 seeds); a passing run undoes the deviation. Noise in the tuning step and the gain limit stay. Prompt: `docs/HANDOVER_DIFFCOTUNE_STAGE4E_PAPER_ABLATIONS_2026-11-13.md`.

## 5m. Decisions after Stage 4e (user, 13 Nov evening)

Stage 4e: the paper's literal stop rule (tuning mean 13.00 against 6.41 cm, A1 stays on the floor) and body rates in J_sysId (β worse in the fit, the loop drifts, all 4 tuning scenarios fail in round 4; real-data gyro share 95.7 %, unchanged by filtering) both fail. Deviations 8 and 9 stay, now backed by tests; the method version is frozen for S0–S3. Supervisor summary: `docs/diffcotune_supervisor_summary_2026-11-13.md` (agreement open). Next: Stage 5 = S0 lab preparation closed (preflight, ROS sim test, lab card), then S2, S1, S3 through model, checks, identifiability, 5-round dry run and lab tools; FBL later. Prompt: `docs/HANDOVER_DIFFCOTUNE_STAGE5_S0_READY_S1_S3_PREP_2026-11-13.md`.

## 5n. Decisions after Stage 5 (user, 14 Nov)

Stage 5: S0 lab preparation closed (preflight, lab card, mock test of `apply_gains.py`); S2 / S1 / S3 models pass the forward and noise gates, dry runs meet a–c (S3 only formally, flat rounds 0–3), but the A1 gradient check fails for all three (S1 / S3: reverse mode explodes to 1e20–1e24; S2: 1–14 %). Forward mode matches finite differences where reverse fails, so the reverse pass loses precision on an unstable INDI filter mode. Decisions: test a forward-mode gradient (exact autodiff, not a deviation) for S1–S3 and rerun their identifiability and dry runs; no truncation or other gradient change; S0 unchanged; RPM-delay β judged on the rerun (proposal: keep if error < 25 %); `round.py` default 5 rounds; lab order S0, S2, S1, S3; INDI thrust-arrival risk noted, no model change. Prompt: `docs/HANDOVER_DIFFCOTUNE_STAGE5B_FORWARD_MODE_GRADIENT_2026-11-14.md`.

## 5o. Decisions after the Stage 5b checks (user, 14 Nov evening)

Stage 5b: forward mode is exact for all four controllers, but uncompiled it is about 19x slower (450–590 s per gradient), so the reruns were not started. Reverse mode is wrong only on A1 without noise; with noise (the tuning step) it equals forward mode to <= 2e-11 on A1. Decisions: the tuning step keeps compiled reverse mode if that holds across gain sets; the beta fit uses forward mode on A1 only (mixed, exact); forward mode moved into the compiled model for speed (target about 30 min per round, limit 45); truncated reverse only measured as a fallback (new deviation, user decides); then the S1, S3, S2 reruns. Prompt: `docs/HANDOVER_DIFFCOTUNE_STAGE5C_FAST_EXACT_GRADIENT_2026-11-14.md`.

## 6. Next step

Stage 0 closing lines and Stage 1 (spec + inventory, desk only) as the first prompt for the main agent.

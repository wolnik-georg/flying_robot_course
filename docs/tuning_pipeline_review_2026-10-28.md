# Review: tuning-pipeline plan, four paper summaries, repo analysis (2026-10-28)

Independent review of [`tuning_pipeline_plan_2026-10-28.md`](tuning_pipeline_plan_2026-10-28.md), the four summaries in `papers/summaries/` and [`tuning_repo_analysis_2026-10-28.md`](tuning_repo_analysis_2026-10-28.md). Prompt: [`tuning_pipeline_review_prompt_2026-10-28.md`](tuning_pipeline_review_prompt_2026-10-28.md).

How it was done:
- Each PDF was read on its own, as text. Image-only tables were rendered and read: DiffTune Tables III–VII.
- The repo was cloned at `3650d09` into a scratch folder and read in full.
- `gradcheck` and a 5-epoch training run were executed in a throwaway venv. The shipped checkpoints were loaded.
- Firmware claims were checked against the live source `~/Desktop/crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c`.

Nothing else was edited.

Confidence tags: **V** = verified (read or run), **SE** = strong evidence, **H** = hypothesis, **U** = unknown.

---

## 1. Claim check

### 1a. Summaries vs PDFs

**von Prittwitz et al. (`vonprittwitz2024diffquad.md`)**

| Claim | Verdict | Evidence | Suggested correction |
|---|---|---|---|
| T0 = 3, P = 5, Tmax = 400 (4 s at 100 Hz) | confirmed (V) | p. 2 "Results" | — |
| Gains clamped ≥ 1e-8; gradients clipped to 10 % of the current parameter value, not in the pseudo-code | confirmed (V) | p. 2, Alg. 1 line 10 | — |
| "Train until the loss has not decreased for P = 5 epochs, then double T" | corrected (V) | Alg. 1 lines 11–21: `idec` is reset only after a doubling, never on an improvement | "Double once the loss has failed to improve P + 1 times since the last doubling, not necessarily in a row." The text (p. 2) says "does not decrease over a number of P epochs", so the paper contradicts itself. |
| Loss = sum of squared position, velocity, rotation and rate errors, λ = 1 | confirmed (V) | eq. 3; p. 2 | Note: eq. 3b is a sum over t, but the text says "mean squared error" and Table II says "avg. Errors". |
| Figure-8 7.28 s, 100 Hz, initial gains all 1, 500 epochs, 40 min on an i7-10510U | confirmed (V) | p. 2 | — |
| Table I: auto-tuned Kp 2.56/2.48/1.01, Kv 9.51/11.28/1.01; hand-tuned Kp 9, Kv 7 | confirmed (V) | Table I | Add the attitude gains. Yaw collapsed to almost 0: KR_z 0.0001, Kω_z 9.7e-6, against the clamp at 1e-8. "z gains stay near 1" is true only for Kp and Kv. |
| Table II, figure-8: auto-tuned better in all 4 columns | confirmed (V) | 1.023e-5 < 1.345e-5; 8.29e-6 < 2.43e-4; 5.60e-3 < 6.80e-3; 9.90e-4 < 1.74e-3 | — |
| Circle: hand better on position (2.27e-6 vs 9.66e-6) and velocity (1.470e-5 vs 1.589e-5); auto better on attitude and rate | confirmed (V) | Table II | — |
| Helix: hand better on position, attitude and rate; auto better on velocity (7.36e-5 vs 4.37e-6) | confirmed (V) | 4.886e-6 < 8.888e-6; 1.737e-3 < 1.851e-3; 1.453e-3 < 1.550e-3 | — |
| Random waypoints: hand better on position and attitude; auto better on velocity and rate | confirmed (V) | 2.206e-6 < 8.777e-6; 1.318e-3 < 1.336e-3; 6.27e-6 < 1.735e-5; 4.84e-5 < 6.33e-5 | — |
| "Position error worse than hand-tuned on all three test trajectories" | confirmed (V) | Table II, Lp column | — |
| Venue not printed; no arXiv id, no DOI | confirmed (V) | whole PDF | — |
| Year 2024 | supported (SE) | PDF CreationDate 2024-11-01; cites ASPLOS 2024 and DiffTune T-RO 2024 | Cite the year as "2024 (PDF date; no venue printed)". |

**Cheng et al., DiffTune (`cheng2024difftune.md`)**

| Claim | Verdict | Evidence | Suggested correction |
|---|---|---|---|
| Identity: T-RO vol. 40, 2024, p. 4085; DOI; received 4 Mar, accepted 8 Jul 2024; 17 pages | confirmed (V) | p. 4085 | — |
| "Earlier conference version (hyperparameter-free…)" | corrected (V) | p. 4087: ref. [32] DiffTune+ (L4DC 2023) | The paper calls it an extension, not an earlier version. |
| BO "inefficient … >20" parameters | confirmed (V) | p. 4086 | — |
| Θ = feasible set for stability, from Lyapunov analysis or engineering practice; projected gradient descent | confirmed (V) | eq. 3; p. 4089 | — |
| Sensitivity propagation; 91 ± 13 µs per step in MATLAB; L1 adaptive control against bias | confirmed (V) | Sec. IV-A/B; p. 4095 | — |
| Sim quadrotor: start kp 16, kv 5.6, kR 8.81, kΩ 2.54; ranges [15,24] [4,16] [8,12] [0.1,3]; loss = squared position error over 10 s; noise 0.1 m, 0.1 m/s, 1e-3 rad/s | confirmed (V) | Sec. V-B | — |
| Compared with AutoTune, SafeOpt-PSO and GIBO on 3 trajectories, 100 trials; DiffTune lowest RMSE with the fewest trials | confirmed for the quadrotor (V) | Fig. 7 | Add the qualifiers. The baselines ran 10 times each. On the Dubins car, GIBO reached the lowest error after 20 trials (Fig. 4, p. 4092). The claim that BO is inferior rests on SafeOpt-PSO and GIBO with fixed settings, not on standard GP-EI BO. |
| Hardware: circle at 1/2/3 m/s, tuned per speed, 10 trials, 7 s horizon, α = 0.1, each step within ±10 % | confirmed (V) | p. 4095 | Add two facts: the loss is position + rotation error, L = ‖p − p̄‖² + tr(I − R̄ᵀR)/2; the vehicle is a 0.63 kg Pixhawk/ArduPilot quadrotor at 400 Hz. |
| 3.5× RMSE reduction on the 3 m/s circle | confirmed, needs a qualifier (V) | Table V: 61.8 → 17.7 cm = 3.49× (1 m/s: 1.56×; 2 m/s: 2.57×) | The baseline is the poor initial guess. Against the hand-tuned gains it is 28.6 → 15.4 cm = 1.86× (Table IV). Table V (17.7) and Tables IV/VI (15.4) also disagree for the same P3 / C(3) cell. |
| P2 stopped after 7 trials (oscillation) | confirmed (V) | p. 4095 | — |
| Sim-only gains crashed the real drone on C(2) and C(3) | confirmed, needs detail (V) | Table VI: C(2) crashed at 50 sim trials; C(3) at 30 and 50 | "Experiment-based tuning was better" is **corrected**: it was better on C(2) and C(3). On C(1), simulation-tuned gains beat experiment-tuned ones (Sim₅₀ 4.3 cm vs ExpT 5.7 cm). The sim runs used α = 0.01 and 50 trials. |
| 10 sim trials give errors close to hand-tuned | confirmed (V) | Table VI: 7.3 / 18.2 / 32.3 vs 9.0 / 18.0 / 28.6 cm | — |
| Generalisation: circle-tuned gains do not fit other speeds; partial transfer to lemniscates | confirmed, needs numbers (V) | Table IV | Tuned gains can be much worse than hand-tuned off their own speed (P1 on C(3): 72.8 vs 28.6 cm). P1 is worse than hand-tuned on all 4 lemniscate cells, and hand-tuned is best on VF(1). This strengthens the overfitting warning. |
| Tables III–VII not transcribed | now read (V) | Tables III–VII | Nothing contradicts the summary. The two qualifiers above (the 3.5× baseline, the C(1) result) come from these tables. Hardware initial gains (Table VII) differ from the sim ones: kp 10, kv 2/1/1.5, kR 0.5/0.3/0.14, kΩ 0.11/0.1/0.01. |
| Limits: differentiability; not for contact or saturation; local minima | confirmed (V) | Sec. VII | — |

**Krishna et al., DiffCoTune (`krishna2025diffcotune.md`)**

| Claim | Verdict | Evidence | Suggested correction |
|---|---|---|---|
| arXiv 2505.24068v1, 29 May 2025, 8 pages; authors; no quadrotor | confirmed (V) | p. 1; Table I | — |
| "≤ 5 real rollouts" | corrected (V) | Contributions say "< 5"; experiments use L = 5; Alg. 1 loops l = 0..L (6 rollouts) | "L = 5 tuning iterations (the paper is not consistent about the exact rollout count)". |
| Combined vs Split-Alternate; Combined diverges under model uncertainty ("fragile") | confirmed (V) | Sec. V-B5, Fig. 7 | — |
| L = 5, K = 100, Adam, w_task = w_sysId = 1 | confirmed (V) | Sec. V | Add: Split-Alternate gives K/2 epochs to β and K/2 to θ. Termination at a change < 1e-3. |
| Quadruped: +60 % sim-to-sim; 1.04 m further sim-to-real | confirmed (V) | Sec. V-A2 | — |
| Fig. 5 biped: "exact heights not transcribed" | corrected (V) | Fig. 5 labels | These are jump **lengths**. Synthesised 40 cm. DiffCoTune 34 cm, succeeds. DiffTune 27 cm, falls on touchdown. Untuned 20 cm, fails. |
| BO comparison only on cart-pole LQR, 100 trials; gradient methods faster in the first ~10 trials | confirmed, incomplete (V) | Sec. V-B2, Fig. 6 right | Add: "vanilla BO … ultimately finding the best solution after 90 trials". BO had the best final result. |
| Biped MPC result | missing (V) | Sec. V-A3: +29.29 % | Optional. |

**Stenger et al., BO review (`stenger2026bo_tuning_review.md`)**

| Claim | Verdict | Evidence | Suggested correction |
|---|---|---|---|
| arXiv 2609.09403v1, 8 Sep 2026, 37 pages; authors and affiliations | confirmed (V) | p. 1 | — |
| 110 reviewed papers | confirmed, needs scope (V) | Sec. 7.1 | Scope: hardware online-tuning papers published up to 2025. |
| "Fewer than ~10 interpretable parameters is the sweet spot" | corrected (V) | Sec. 4.2 ("typically fewer than ten interpretable parameters"); Sec. 7.3 / Fig. 4 ("vast majority of hardware examples optimize less than 10 parameters") | This describes common practice; it is not a stated limit. Sec. 5.2 says logEI works in high dimensions, and Sec. 7.3 mentions > 100-D BO in simulation. |
| d + 4 initial points; d + 1 may suffice for deterministic tasks | confirmed (V) | Sec. 5.3 | — |
| Scale inputs to the unit cube, standardise outputs; final = lowest posterior mean | confirmed (V) | Sec. 5.3 | — |
| Matérn or SE with ARD; zero/constant mean; homoscedastic noise; acquisition "no clear winner" | confirmed, incomplete (V) | Sec. 5.1–5.2 | Add the recommendation: MES for noisy low-dimensional problems, logEI for high-dimensional low-noise ones. Add Sec. 5.1's warning on "deterministic noise": with fixed seeds, re-querying a point gives no new information. |
| "Without a constraint a **pure-tracking** objective can drive gains to degenerate values" | corrected (V) | Sec. 6.1 | The paper's example is the reverse: minimising **energy** without a tracking constraint makes zero gains optimal. |
| Benchmark: 4 gains, 20 seeds, Friedman → Wilcoxon → Holm; BO beat Sobol; acquisition functions not significantly different | mostly confirmed (V) | Sec. 9, Tables 4–5 | Add: the budget was 40 evaluations with **5** Sobol starting points (d + 1, not d + 4). Only the best method (LogEI) was tested against the others. "BO beat Sobol" holds only as "LogEI significantly better than Sobol". |
| Sec. 6.2–6.10 and 7 "only by heading" | now read (V) | — | What is relevant and missing from the summary is listed in §2, finding G7. |

### 1b. Repo analysis vs code (`differentiable-quadcopter` @ `3650d09`)

| Claim | Verdict | Evidence | Suggested correction |
|---|---|---|---|
| ~2800 lines | corrected (V) | `wc -l *.py` = 2457 | "~2500 lines of Python" |
| Explicit Euler, dt 0.01 s, 13-dim quaternion state | mostly (V) | `quadrotor_pytorch.py:127–144` | Position, velocity and rates use explicit Euler. The quaternion uses the exponential map with the world-frame rate (`qintegrate`, l. 59–61, 138–140), which is correct. |
| Mixer B0 (arm 0.046 m, t2t 0.006) | confirmed (V) | l. 91–99 (arm × 0.7071) | — |
| `eta = force @ B0.T` "might be wrong" | missed (V) | l. 116–118 | The line is **correct**: for row-vector batches `force @ B0ᵀ` equals `B0·force`. The TODO comment is a false alarm. |
| Controller: thrust, desired rotation, jerk-based rate reference, torque; no integral; jerk, not snap | confirmed (V) | `controller_pytorch.py:51–197` | Add: the code drops the −RᵀR_d ω̇_d term that the abstract's eq. 2b contains. The attitude error matches Lee and our firmware: `vee_so3` symmetrises, so `0.5·vee_so3(...)` = eR; firmware l. 544–546. |
| 12 gains as `nn.Parameter`; per-element gradient clip; clamp ≥ 1e-8; window doubling | confirmed (V) | `train_lee_controller.py:43–51, 182–190, 333–346` | Mass and inertia are also Parameters (frozen) and get clamped as well, which is harmless. |
| Patience 50, window cap 350 | confirmed, incomplete (V) | l. 242, 334–343 | The counter resets on any improvement > 1e-4, unlike Alg. 1 of the abstract. |
| Gradient clip "±1.0·\|parameter\| (i.e. ±100 %)" | **corrected (V, run)** | l. 185 clamps the **gradient** to ±\|θ\|; SGD step = lr·grad, so each step changes θ by at most lr = **0.1 %**. 5-epoch run: kw fell exactly 0.999³¹ per epoch (31 batches) | "Each step changes a gain by at most 0.1 % with the defaults (SGD, lr 1e-3), about 3 % per epoch. With Adam the clip hardly limits the step." |
| Loss weights `--*-weight` | **missed (V)** | l. 180 backpropagates the unweighted sum; the weights enter only the plateau test, l. 318 | The CLI weights have no effect on the gradients. |
| SGD lr 1e-3, `--epochs 500`, gain init ones / random / pretuned (9, 7, 0.0013, 0.0055) | confirmed (V) | l. 216–274 | The README example uses `--epochs 5000`. |
| No motor dynamics, delay or saturation | confirmed (V) | `max_u` is defined (l. 71) but used only in the numpy `controller.py:398` | — |
| Mass 0.034 kg, CF2.0 inertia; ours 0.0427 kg | confirmed (V) | l. 81, 87; `study_parameter_decisions.md` | — |
| Each window starts **on** the reference, so the tracking error starts at zero | corrected (V) | l. 79–88 | Position and velocity start on the reference, but the attitude starts at identity with zero rate. Every window restarts from the reference, so drift never builds beyond one window. |
| Noise option: `pos += v·randn·dt` only | confirmed (V) | l. 128–130 | Add: zero at hover; under `no_grad`. |
| `train_system_id.py` fits mass and inertia | corrected (V) | l. 22, 26 read `self.quad.mass` / `self.quad.J`; `QuadrotorAutograd` defines only `.m` / `.I` | It does not run with the current model class (stale). |
| CI runs only training; `test_lee_controller.py` not read | confirmed / now read (V) | `.github/workflows/benchmark.yml` | The workflow is manual (`workflow_dispatch`). It uploads `results/`, which training never writes. `test_lee_controller.py` has no assertions; it prints. |
| Gain init all 1 = the abstract's Table I start | confirmed (V) | — | See below: the shipped results do not reproduce Table I. |

Extra findings from the code (not in the analysis):
- **Table I is not reproduced by the shipped checkpoints (V).** `checkpoints/lee_controller_best_run.pt` (epoch 4999, window 192, SGD lr 1e-3) holds kp ≈ 1.00/1.00/1.00, kv ≈ 1.00/1.01/1.00, kw 0.0016/0.0014/0.0014, kr 0.0082/0.0070/**1e-8**. The `_noisy` file is similar. Table I has Kp 2.56/2.48/1.01 and Kv 9.51/11.28/1.01. The Table I recipe is not recoverable from the repo.
- **The "uninformed start diverges" problem is mostly an integration artefact (SE, analytic).** With explicit Euler on ω̇ = −(kw/I)·ω, a step is stable only if dt·kw/I < 2, i.e. kw < 0.0033 (roll/pitch) or 0.0059 (yaw) at dt = 0.01 s.
  - The start kw = 1 is ~300× above that limit.
  - The auto-tuned Kω 0.0026 / 0.0020 sits at 80 % / 60 % of it.
  - So the tuned attitude gains are shaped by the 100 Hz Euler step, not by the vehicle (our firmware runs at 500 Hz).
- **Yaw gains collapse to the clamp (V).** kr_z = 1e-8 in both checkpoints (Table I KR_z 1e-4, Kω_z 1e-5), because the figure-8 never excites yaw. Gains like this would leave yaw uncontrolled on hardware.
- **Bugs (V, run):**
  - `--visualize-trajectory False` crashes at the end (`args.plot_errors` does not exist, l. 363).
  - An unsupported `--gain-init` builds a `ValueError` without raising it (l. 276).
  - `gradcheck` passes (True), but only at hover with zero force and rate (l. 155–158).
- **Can it serve as a differentiable S0? No, not as it stands (SE).** Our S0 (Omar C, `indi 0`) has things the model lacks:
  - the z-position integral (clamped at ±2) and an attitude integral KI on eR;
  - the ω̇_d term;
  - 500 Hz;
  - 44 ms motor lag and the thrust map;
  - mass 42.7 g;
  - the downwash.

  It is a starting template, not a model of our S0.

### 1c. Plan facts vs project docs and code

| Plan claim | Verdict | Evidence | Correction |
|---|---|---|---|
| §3: INDI filter cutoffs are a compile-time 30 Hz in the snapshot (l. ~133–138); the snapshot may be older | confirmed, and the live source is the same (V) | snapshot l. 133–138; live `controller_omar_indi.c:333–338` (4 Butterworth-2 filters at 500 Hz) | Drop the hedge. |
| §3: S0 tunables "`Kpos_P/D/I` x,y,z (+ limits), KR, Kw, KI" | **corrected (V)** | live l. 66–76, 434–447, 594–605, 675–713 | See finding E5: Kpos_Ix/Iy are inert, the P/D limits are inactive, and KI is an unclamped attitude integral. |
| §3: S4 tunables "kp/kv/ki xy and z (6), `kr` rate gain" | **corrected (V)** | live l. 80–92 (`inner = 1` default), `lab_sessions/2026-10-26.md` §3 | kr_xy, kr_z and J are unused with `inner 1`. The inner loop that actually runs is the stock rate PID (`pid_rate.*`, `pid_attitude.yaw_*`). |
| §3: "Kpos_P 7, Kpos_D 4, Kpos_Iz 1.5 (Omar defaults…)" | wording (V) | firmware default `Kpos_I = 0`; 1.5 is the study value | "Omar defaults 7/4 plus the study's Kpos_Iz 1.5". |
| §1: "Authors' gains = `ctrlFbl.preset 0`" | **corrected (V)** | firmware comment l. 132–134: "the authors' values exactly as in their code (30 g simulated vehicle)"; hsieh2026flatness summary l. 230: hardware PID/Kω gains not in the paper; `fbl/GAIN_TUNING_2026-10-20.md`: "their hardware gains will tell" | Preset 0 is the authors' **simulation-code** gains. Their hardware gains are unknown, and the decision of 10-22 is not to email the authors. |
| §1: "a thrust-upgraded CF2.1 … the platform the authors built it for" | confirmed, incomplete (V) | hsieh summary l. 122–124, 147 | The authors ran FBL **off-board**: Vicon 120 Hz → laptop → radio commands (thrust + body rates) at 400 Hz, with the onboard stock PID. Ours runs onboard at 500 Hz with the EKF, our trained 4-neuron network and a downwash gate. So "everything else as in the paper" is not achievable. |
| §4: "BO is said to work up to about 20 parameters (DiffTune paper, BO review)" | corrected (V) | DiffTune p. 4086 and DiffCoTune Sec. II say BO is inefficient > 20; the BO review says nothing like this | Attribute it to DiffTune / DiffCoTune only. |
| §5: SIL "validated against hardware on A8 only (crossing dip within about 1 cm)" | **corrected, critical (V)** | `docs/72` Step 1–2, `docs/75` §3, `ns2_c_phase3_5_results_2026-10-14.md` Phase 5, `validation_s3_versions_2026-10-18.md` C5 | See finding E1. The validation was for controller 6 with NS2 off / +1; the S0–S3 tree is not reproduced. |
| §5: A1, A2, A5 and some A7 cells diverged with seed 0 | confirmed (V) | `sil_study_screening.md` | Add: the screen used **controller 6** (`ki_z 16`), not S0. The doc blames finite-difference setpoints and start-up transients, and says Omar C and our INDI "cannot run here". |
| §5: "e.g. A1 dz 0.25/0.5 flown cleanly in `tree_comparison_2026-10-19.md`" | **corrected (V)** | that doc has no dz 0.25; S0 A1 dz 0.5: z RMS 15.9–20.6 cm, roll/pitch max 40–42° | "A1 dz 0.5 flew without divergence on hardware, with the ±35–40° oscillation (`docs/68`)". |
| §6: unseen-block settings A8 dz 0.4 at 0.35 m/s, A1 dz 0.45, A4 offsets 0.075/0.125 | confirmed (V) | `results/unseen_block_2026-10-23/cells.csv` | Add: A4 is dz 0.4 at 0.35 m/s. n = 3 (A1, A8) and n = 2 (A4) per strategy, 40 flights. |
| §9: protocol "Frozen = gains, yaml, firmware, weights … no retuning" | confirmed (V) | `comparative_study_protocol.md` l. 28 | The same line claims "each strategy was tuned on the hardware gate", but S0–S3 run on Omar's untuned defaults. The plan is an opportunity to make that true. |
| §10 header "effort = rough guess" | not in table (V) | §10 table has no effort column | Add efforts or drop the phrase. |

---

## 2. Findings, ranked by severity

### Errors that must be fixed before work starts

| # | Finding | Conf. |
|---|---|---|
| **E1** | **No existing simulator is a fair tuning plant for S0–S4.** See the three points below the table. | V |
| **E2** | **Step A cannot show that "the port is correct".** See the four points below the table. | V (facts), SE (inference) |
| **E3** | **Tuning scenarios = study cells (§6: A1, A8, A4 with the pilot settings).** Gains tuned on the cells they are later tested on leak into the comparison. The BO review warns against this kind of leakage (Sec. 8.1: no leakage "from the final test instances"). | SE |
| **E4** | Repo analysis: the gradient-clip meaning is wrong (0.1 % per step, not ±100 %), and the loss-weight bug and the stale system-id script were missed. See §1b. | V |
| **E5** | **Parameter list (§3) has inert or mis-named entries.** See the four points below the table. | V |
| **E6** | Plan §5 example (A1 dz 0.25/0.5 "flown cleanly") and the BO "up to 20" attribution are wrong (§1c). | V |

**E1 — no fair tuning plant:**
- The A8 "validation within ~1 cm" is for **controller 6** (our geometric, NS2 off and `res_sign +1`; `−1` off by 2.3 cm), not for the Omar tree that S0–S3 now use.
- For the tree, the SIL gives:
  - S1 / S3 with a near-ideal INDI: dip 0.75 cm against 10.6 cm on hardware; "no dip … while the hardware shows −9 to −22 cm" (`docs/75` §3, `validation_s3` C5).
  - S0 with an A8 dip of −0.4 cm at plant scale 1.0 (10-14 setup: NS2 v1 era, our geometric top drone). The dip reaches −11.5 cm only when the plant force is scaled 1.3×. On hardware S0 dipped −12.7 cm (10-19, 0.5 m/s). The settings are not identical, so this is strong evidence, not a matched comparison.
- Two plants exist (SIL and the FBL C-in-the-loop) with different actuator models. Neither runs all five controllers. The FBL plant hard-codes `indi = 0` and has no RPM, so it cannot run INDI.
- **The downwash "truth" in both plants is the NS2 network**, which is S2/S3's own predictor and the fitting target of FBL's ROM. Tuning there would reward the compensators for predicting the simulator.
- The plan calls the plant "critical" but does not state any of this.

**E2 — Step A does not test the port:**
- Preset 0 is the authors' simulation-code gains, not their hardware gains.
- Their architecture was off-board 400 Hz with the stock PID; ours is onboard with the EKF.
- The thrust-upgrade drone needs a different image or thrust model; our image is built for brushless.
- A correct port can therefore fail (lag, estimator, thrust map), and a wrong one can pass (a hover never exercises the downwash sign or gate). Port correctness is a desk question: host `fbl_law` vs the authors' code on replayed states, already started per `docs/fbl/PORT_SPEC.md`.

**E5 — inert or mis-named parameters:**
- `Kpos_Ix/Iy` are forced to 0 whenever `Kpos_Iz ≠ 0` (l. 442–447). Their integrators still run, unclamped.
- `Kpos_P_limit` / `Kpos_D_limit` are error clamps at 100 m, so inactive.
- `KI` is an **unclamped integral of the attitude error eR** (l. 597, 604), reset only at controller init.
- FBL's kr / J are unused with `inner 1`; the real FBL inner-loop tunables are the stock `pid_rate` gains.

### Gaps

| # | Gap | Conf. |
|---|---|---|
| G1 | **Top drone:** always S0 in the study. If S0 is tuned, does the top drone get the tuned gains? This changes the disturbance for all strategies, so decide it explicitly. In the SIL the two drones may not take independent controller-9 parameters ("host gains are process-global", `docs/72`), which needs checking. | V (doc) / H (controller 9) |
| G2 | **Budget fairness:** "same N for all" gives the ~12–19-parameter Omar tree far fewer evaluations per dimension than FBL (6). Either scale N with d (e.g. N = k·d) or show convergence for each controller. | SE |
| G3 | **Bounds fairness:** "a fixed factor around current values" centres the search on starting points of different maturity: Omar's defaults from his drone, against FBL preset 1 tuned to our lag. Use wide log bounds plus a stability screen, and report sensitivity to the bounds. | SE |
| G4 | **Takeoff and landing are part of the flight:** S3 overshoots ~10–13 cm into the A1 hold; preset 0 flipped at takeoff. Tuning episodes must include them, or gains will be optimised on the hold only. | V (facts) |
| G5 | **Wall time per episode, determinism and parallelism are unmeasured.** These decide the method (C3). | U |
| G6 | **Statistics mismatch:** §7's Friedman → Wilcoxon → Holm is the review's procedure for comparing **optimisers across seeds**. For tuned vs default flights (two groups, unpaired) the protocol already prescribes effect size + bootstrap CI (protocol l. 77). | V |
| G7 | **Missing from the BO summary** (Sec. 4.1, 5.1, 6.3, 6.6, 6.9, 6.10, 7.5, 8). See the list below the table. | V |
| G8 | §9 misses three consequences: (a) the pilot / unseen / 10-29 meeting results and the S3 < S1 A1 finding become "default-gain" results; (b) the tuned-gain hardware block adds to the ~19 flying hours of tier 1; (c) it settles the FBL `ki_z` 8× fairness issue in a principled way, a positive point worth stating. | SE |

**G7 — what the BO summary is missing:**
- **Sec. 4.1 / 8.2:** BO is for **expensive, noisy** evaluations. When evaluations are cheap, CMA-ES, pattern search or PSO "can be competitive". Baselines must include **Sobol**.
- **Sec. 6.3:** crash constraints. A fixed penalty breaks GP smoothness; use BO-VDP or a classifier.
- **Sec. 6.6:** contextual BO = per-scenario gains (open decision 2).
- **Sec. 6.9:** local / trust-region BO (TuRBO) when a good start exists.
- **Sec. 6.10:** multi-fidelity (simulation + hardware) and early-stopping BO.
- **Sec. 7.5:** BO with a high-fidelity simulation "to fairly compare different controller structures" (Funk 2021, Manhães 2017, Yu 2020). This is the closest precedent for this plan.
- **Sec. 8.1 / 8.4:**
  - shared seeds and initial designs;
  - equal tuning effort for baselines;
  - median + IQR;
  - no averaging of costs across tasks of different scale;
  - always inspect the state trajectories.
- **Sec. 5.1:** deterministic noise.

### Risks

| # | Risk | Conf. |
|---|---|---|
| R1 | The tuned S0 could absorb much of the downwash (larger Kpos_Iz / Kpos_D). This shrinks the measured benefit of S1–S4, a scientific consequence the supervisor should agree to in advance. | H |
| R2 | Optimising S1 / S3 in the ideal-INDI SIL pushes gains in a direction the hardware punishes, as DiffTune's sim-only gains crashed on C(2) and C(3). | SE |
| R3 | Tuning the unclamped `KI` or `ki_z` can produce gains that only work in a 15–30 s episode (wind-up over a whole flight). | H |
| R4 | Host-only artefacts, e.g. the fake `usecTimestamp` clock changed Omar's dt (Phase 5 note). The tuned optimum may exploit such artefacts. | V (precedent) |
| R5 | A hardware failure during Step A or §8: cf5 has an open power fault, possibly from the preset 0 flips. | V |

### Things that are fine

- Fairness rules 1–6 and pre-registration are the right frame.
- NS2 weights are kept frozen.
- Final gains = lowest posterior mean.
- d + 4 Sobol, ARD Matérn and logEI follow the review correctly.
- The validation set is separate.
- A hardware confirmation where worse results are reported, not hidden.
- §9 correctly says the NS2 verdict becomes "default gains only" unless repeated.
- Most summary facts check out: all of von Prittwitz Table II, and DiffTune's setup.

---

## 3. Plan review, C1–C8

| Q | Short answer | Change request |
|---|---|---|
| **C1** Order A → B → C, Step A as a port test | The order is logical only if A really tests the port. It does not (E2). B is redundant once C is approved, because C retunes FBL anyway. | Replace A's gate with a **desk port-equivalence test**: replay logged 10-26 states through host `fbl_law` and the authors' Python code; outputs must match to float tolerance. Keep A as an optional platform-transfer check, with these records: image hash and build flags (`CONFIG_CRAZYFLIE_THRUST_UPGRADE_KIT`, mass, thrust map); identified motor lag of that drone (step test); hover thrust; gain readback; uSD logging **started before arming** (the 10-26 flips happened before logging began); single drone first, then stacked; Nominal → ROM → Learned; video; a numeric "flies like the paper" (paper: Learned ~5 cm RMSE, ~10 cm z_max at 0.3/0.4 m). Run B only if C is not approved. |
| **C2** Fairness list | Necessary, not sufficient. | Add: (a) plant fidelity checked **per controller** (E1); (b) downwash truth independent of every compensator; (c) tuning set disjoint from study cells (E3); (d) budget rule tied to d (G2); (e) bounds rule independent of the current values' maturity (G3); (f) top-drone policy (G1); (g) episodes include takeoff and landing (G4); (h) an identical crash/constraint handling rule; (i) the same optimiser **and** optimiser settings; (j) report the parameter count and the per-controller convergence curves. |
| **C3** Method | "Only S0 is realistically differentiable" is overstated. Plain MLPs (S2 network, FBL 4-neuron net), Butterworth filters and the FBL algebra are differentiable almost everywhere. The real argument is effort: 4–5 re-implementations, equivalence tests and a differentiable plant. The repo itself is not a usable S0 (§1b). Black-box on the flight code remains the right family. BO is designed for expensive evaluations (review Sec. 4.1); a host simulation may be cheap. | Decide after measuring wall time (G5). If one episode takes ≲ 10 s and runs in parallel on the cores: **CMA-ES** (rank-based, so robust to crash penalties; no GP hyperparameters; parallel). Run a short benchmark against Sobol and GP-BO on S0, as the review's Sec. 8.2 asks. If an episode is slow (≫ 10 s) or for the hardware step: GP-BO (logEI, BO-VDP for crashes), possibly multi-fidelity. Gradient methods: no. |
| **C4** Plant | The evidence is mis-stated (E1). The gate idea is right, but "same ranking of S0–S3" would already fail today: the SIL ranks S1 / S3 as having no dip. Using host firmware as the objective adds the usual unknowns. | Gate per controller and per scenario, pre-registered. Thresholds: the hardware mean within max(1.5 cm, 30 %) for the scenario's primary metric, plus the same sign of every pairwise difference that is resolved on hardware, plus tilt within ±10°. Inputs: the flown default-gain data (pilot, unseen block, FBL 10-26). Before the gate: one harness for all five controllers; INDI realism (RPM / IMU / DShot latency and noise, thrust mismatch; the "vertical model F1" of `validation_s3` A5 is a starting point); a downwash truth that is not the NS2 network (e.g. a physics model fitted to flights). Measure determinism (same seed gives the same cost bit for bit), wall time per episode, and noise spread across seeds. Usual unknowns: determinism, seed noise, host-only artefacts, 1 ms tick, wall time per episode. |
| **C5** Objective, scenarios, statistics | The objective built from reported metrics plus a tilt constraint is sensible. Normalising by the S0-default value is fine if fixed in advance. The scenarios leak (E3). The unseen-block settings can be validation: NS2 never trained on them, and nothing was tuned on them. But they were already seen by the author, n = 2–3, so they are weak as hardware evidence. | Tune on settings **not** in the study table, e.g. A1 dz 0.35 / 0.6, A8 at 0.25 / 0.45 m/s, A4 offset 0.10 at another speed. Validate in simulation on the unseen-block settings, then test in the study. Per-scenario gains = contextual BO; decide before writing the protocol. Seeds: fix a common noise-seed set per evaluation (common random numbers), and repeat whole optimisation runs with ≥ 5 optimiser seeds; report median / IQR. For tuned vs default on hardware use the protocol's effect size + bootstrap CI, not Friedman. |
| **C6** Parameters | Verified against the live source (E5). | Omar tree, after tying x = y: Kpos_P_xy, Kpos_P_z, Kpos_D_xy, Kpos_D_z, Kpos_Iz, Kpos_I_limit, KR_xy, KR_z, Kw_xy, Kw_z, KI_xy (KI_z / yaw optional) gives **≈ 10–12**. Exclude Kpos_Ix/Iy (inert), the P/D limits (inactive), mass and J (physics). Decide whether KI stays (it is unclamped; a clamp would be a code change). FBL: kp/kv/ki xy and z (6) + the stock inner loop (`pid_rate` roll / pitch kp, ki, kd; yaw) if the Omar tree's attitude gains are tuned too, for symmetry. c_d, beta and gamma are fixed. INDI filters: one shared cutoff (all 4 filters at 30 Hz) as a compile-time option, and only with a new image; recommend leaving it at 30 Hz. |
| **C7** Consequences | The §9 statements are correct (protocol l. 28; decisions 10-14 night; NS2 verdict with default gains). | Add G8, and the protocol's own claim that strategies "were tuned on the hardware gate" (now to be made true). Note that the NS2 training data (73 flights) were flown under default S0 gains; check that the relative-state distribution of the tuned S2 / S3 stays inside the bank. |
| **C8** WPs, effort, risks, decisions | WP1 (reproduce the repo) cannot hit Table I: the shipped checkpoints do not match it, and the defaults differ. WP2 is the critical path and is under-scoped. There is no effort column. | WP1 → ≤ 1 day, goal "document what the repo does", not "match Table I". WP2 splits into 2a (one harness for 5 controllers + INDI realism + independent downwash), 2b (determinism / wall time) and 2c (gate). Add efforts. Add risks R1–R4 and decisions G1–G3, the tuning-set choice, and the scientific consequence R1. |

**D. Cross-document consistency:**
- The contradictions are those in §1c:
  - the preset 0 label;
  - the SIL validation scope;
  - the A1 dz 0.25 example;
  - the S0 parameter wording;
  - the BO attribution.
- Hardware facts agree with the project docs: 42.7 g, 44 ms lag, CF2.0 inertia in the repo, 39 g IMRC logs.
- The strategy definitions agree with `tree_comparison` and the protocol header.
- The FBL gains agree with the firmware and with lab 10-26 §3.
- Protocol §2 rows still describe the controller-6 S0 (`ki_z 16`, gains 40/30/8/10). This is superseded by its header, but it is easy to quote wrongly.

---

## 4. Missing information

1. Wall time per two-drone SIL episode (A1 15 s hold, A8 4 passes), and bit-for-bit determinism per seed.
2. Whether the CS2 SIL can run two controller-9 drones with independent parameters, and whether FBL (`inner 1`) runs in it.
3. Availability of a thrust-upgraded CF2.1, its firmware config and its motor lag. Authors' hardware gains: not available by decision.
4. Supervisor / professor stance:
   - black-box vs gradient;
   - tuning in simulation at all, given E1;
   - per-scenario gains;
   - top-drone policy;
   - whether a tuned S0 that absorbs downwash is an acceptable outcome (R1).
5. Final tier-1 settings list (`comparative_study_protocol.md` §3), needed to choose disjoint tuning settings.
6. A hardware flight budget for tuning confirmation, and the thesis deadline.
7. `docs/fbl/PORT_SPEC.md` equivalence status: which inputs were tested (I did not open it).

## 5. Recommended order and method

1. **Desk, now:** FBL port-equivalence replay (replaces Step A as the port test), and measure SIL wall time and determinism (G5).
2. **Decisions with the supervisor:** items 4 and 5 above, plus whether to skip B.
3. **Plant (WP2a–c):** one harness for S0–S4, realistic INDI path, a downwash truth independent of NS2 and the ROM, then a pre-registered per-controller gate against the flown default-gain data. If S1 / S3 cannot pass, do not tune them in simulation: tune the shared position gains on S0 / S2, or tune S1 / S3 on hardware with few-evaluation BO.
4. **Pre-register** the objective, the disjoint tuning settings, the bounds rule, the budget k·d, the seeds, the crash rule and the selection rule.
5. **S0 pilot:** CMA-ES vs Sobol vs GP-BO (small benchmark), then the chosen optimiser for all controllers with identical settings.
6. **Validation in simulation** on the unseen-block settings, then the hardware confirmation of tuned vs default, randomised over batteries and sessions.
7. **Protocol v1.0** + image hash, then the study.
8. **Optional:** Step A as a platform-transfer check, once a thrust-upgraded drone and its image exist.

Method: black-box on the host-compiled flight code, which keeps one code path for every controller. Use CMA-ES if episodes are cheap and parallel, and GP-BO (logEI, crash handling via BO-VDP) if they are expensive or on hardware. The differentiable repo is background only, not a side-check, because it does not model our S0 (§1b).

## 6. Commands run (reproducible)

```bash
S=<scratch dir>
# papers
for p in vonprittwitz2024diffquad cheng2024difftune krishna2025diffcotune stenger2026bo_tuning_review; do pdftotext -layout docs/papers/$p.pdf $S/papers/$p.txt; done
pdftotext docs/papers/cheng2024difftune.pdf $S/papers/cheng_raw.txt; pdftoppm -r 110 -png -f 11 -l 11 docs/papers/cheng2024difftune.pdf $S/papers/ch   # Tables III-VI (p. 4095)
pdftoppm -r 110 -png -f 15 -l 15 docs/papers/cheng2024difftune.pdf $S/papers/ch                                                                    # Table VII (p. 4099)
pdfinfo docs/papers/vonprittwitz2024diffquad.pdf      # CreationDate 2024-11-01
# repo
git clone https://github.com/IMRCLab/differentiable-quadcopter $S/dq        # HEAD 3650d09
~/.pyenv/versions/flying_robots/bin/python -m venv --system-site-packages $S/venv && $S/venv/bin/pip install roma tensorboard tqdm
cd $S/dq && $S/venv/bin/python quadrotor_pytorch.py                          # gradcheck: True
MPLBACKEND=Agg $S/venv/bin/python train_lee_controller.py --epochs 5 --report-gains True --visualize-trajectory False   # clip behaviour; ends in AttributeError plot_errors
~/.pyenv/versions/flying_robots/bin/python -c "import torch; c=torch.load('checkpoints/lee_controller_best_run.pt',weights_only=True); print(c['epoch'],c['window_size'],{k:v.flatten().tolist() for k,v in c['model_state_dict'].items()})"
# firmware and project facts
diff flying_drone_stack/firmware_app/host/controller_omar_indi.c.snapshot ~/Desktop/crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c
grep -n "cutoff\|PARAM_ADD\|i_error_pos\|i_error_att" ~/Desktop/crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c
~/.pyenv/versions/flying_robots/bin/python -c "import pandas as pd; d=pd.read_csv('experiments/results/unseen_block_2026-10-23/per_flight.csv'); print(d.groupby(['cell','strategy']).size().unstack())"
```
Docs read: `docs/72`, `docs/75`, `sil_study_screening.md`, `ns2_c_phase3_5_results_2026-10-14.md`, `validation_s3_versions_2026-10-18.md` (grep), `fbl/GAIN_TUNING_2026-10-20.md`, `study_parameter_decisions.md`, `comparative_study_protocol.md`, `lab_sessions/2026-10-26.md`, `tree_comparison_2026-10-19.md`, `papers/summaries/hsieh2026flatness.md` (grep), `experiments/fbl/stage3_port/c_in_the_loop_real.py` (header and plant loop), `experiments/analysis/unseen_block_analysis.py` (header).

Not done:
- No SIL episode was run, so wall time is unknown.
- `docs/fbl/PORT_SPEC.md` was not opened.
- No full 500-epoch training run of the repo.

## 7. Verdict

The work can start, but not as planned.
- **Summaries:** usable after small corrections.
- **Repo analysis:** needs one correction (the clip semantics) and the missed items in §1b. The most important is that the shipped results do not reproduce Table I.
- **Plan, three structural problems:**
  - It assumes a simulator that is validated for S0–S3, when the SIL is known **not** to reproduce the Omar tree (ideal INDI, no S0 dip) and uses the NS2 network as the downwash truth (E1).
  - It tunes on the study's own cells (E3).
  - Its Step A tests platform transfer of simulation-code gains, not the port (E2).

Start with the two desk items in §5 step 1 (port replay; SIL wall time and determinism) and the supervisor decisions. Fix the plan's §1, §3, §5 and §6 as listed above. Treat WP2 (one fair plant for all five controllers, gated against the flown data) as the critical path. Only then choose the optimiser and write the pre-registration.

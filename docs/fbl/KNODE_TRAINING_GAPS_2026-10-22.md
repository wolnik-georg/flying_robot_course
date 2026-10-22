# FBL residual training — what is available, what is missing, what we decide (Task 1, step 1: all sources read, 2026-10-22)

Sources read in full for this table: the authors' code bundle (`learned_fbl_core`, incl. `LEARNED_FBL.md`, `ODEFunc`, `nominal_dynamics.py`), Chee's thesis (UPenn 2024: App. B, App. C index, Ch. 3), Hsieh et al. 2026 (arXiv 2607.12275: Sec. 4, 5, App. A), Chee et al. 2024 (arXiv 2410.09727), KNODE-MPC 2022 (arXiv 2109.04821), and the two public repositories of the KNODE author (below). No email to the authors (decision of the user).

## Result in one line
**The training method is fully specified; what is missing are our own platform constants, the C_D identification (manual in the papers) and a few hyperparameters of the small networks — all of which we can fix ourselves.** No authors' training script exists beyond the thesis appendix.

## Table (needed → where → status → our decision)
| # | Needed | Where it is | Status | Our decision |
|---|---|---|---|---|
| 1 | Model structure: ROM + net on [Δr, Δz], flow frame, scaling | bundle `ODEFunc`, `DownwashResidual`, `nominal_dynamics.py`; paper Sec. 4 | given | reuse unchanged (checkpoint conventions: x radially to neighbour, z = flow axis) |
| 2 | Loss | paper Eq. 23, thesis B.1: mean squared error of the integrated **one-step** state, weight W_x = diag(I₁₀, 0.1·I₃) (last three = body rates) | given | same |
| 3 | Integrator, optimizer, loop | thesis B.3: `torchdiffeq` RK4, Adam lr 2e-2, weight decay 2e-9, 2000 iterations, full batch, seed 123, keep best loss, init N(0, 0.1) | given (generic 17→32→13 net, not the 4/9-neuron nets) | start from these, tune for the small nets |
| 4 | Data format | thesis B.2: table of time, 13 states, 4 inputs; bundle uses 30 columns (ego 13, inputs 4, neighbour 13, neighbour exogenous) | given | build from our uSD logs |
| 5 | Data amount / recipe | 18 s (sim, 3 532 points) and 28 s (hardware, 6 720 points): static-top and stacked, 0.4 m separation, 0.4 m/s, flown under the controller without the learned residual; sampling ≥ 200 Hz | given | our 73-flight bank is far larger; keep a small recipe-like subset for a like-for-like check |
| 6 | Network size | hardware force net: 1 hidden layer, 4 neurons (thesis) / "2-layer, 4 neurons" (paper); sim: 9 neurons per branch (force, torque) | given (wording differs: layers) | start with 1 hidden layer × 4 and 9, compare |
| 7 | Hardware training input u | hardware FBL outputs thrust + body rates, only the force residual is compensated; torque input of the ODE for hardware data **not stated** | **gap** | force-only training: loss on position/velocity (+ attitude), body-rate weight 0 or measured rates fed in; to be tested |
| 8 | C_D (and S, jet constants) | ROM from Chee 2024: "adjust until the nominal-ROM controller performs reasonably" → 0.236 (manual); sim 1.3 / S 0.0697 vs controller 1.18 / 0.0997; bundle constants C_d 0.3, C_d_GT 0.55 | **gap (manual procedure)** | identify offline: fit C_D on our data (Stage 2 sim gave ≈ 0.6 for the closed loop; to be redone with the KNODE objective) |
| 9 | Vehicle constants (mass, inertia, arm, prop radius) | bundle: 30 g, Ixx 1.43e-5, Izz 2.89e-5, arm 0.046; paper hardware 36 g | **ours needed** | mass 0.0427 kg, our inertia/arm/prop radius from the CF2.1 Brushless data (already used in the port) |
| 10 | Training/runtime mismatch | training wrapper uses the neighbour's body-z, runtime uses +z (upright) | known | keep, quantify for our scenarios (tilt up to 21–35°) |
| 11 | Hardware gains, α-β-γ coefficients | not published | gap | already ours (tuned in the C-in-the-loop), authors' gains kept as preset 0 |
| 12 | Validation protocol | prediction RMSE on stacked flights at 0.2–0.5 m (sim), closed-loop RMSE/z_max over 60 runs (hardware) | given in outline | our held-out-by-setting split + closed-loop tables like S0–S3 |
| 13 | Where the state comes from | Vicon 120 Hz to the laptop; ours: EKF state, 500 Hz uSD log | different | our logs; check velocity/quaternion quality and delay |
| 14 | Thrust input for the integration | thrust command of the controller | ours: thrust command × calibrated map (command/weight 0.96–1.02) or RPM-based force | decide in step 2, test the sensitivity (residual ≈ 1.2 m/s² is small against a few % thrust error) |

## Public code found
- `github.com/TomJZ/Online-KNODE-MPC` (GPL v3): quadrotor KNODE with `train.py` — one-step lookahead 2, Adam lr 0.01 (then 0.001), L2 5e-9, input noise 1e-8, **online** variant (13 states + 4 inputs, no downwash). `github.com/TomJZ/K-NODE`: the original Chaos 2021 KNODE (generic systems).
- Use: as a **reference for training details only**; GPL v3 means no code is copied into our repository. Clone kept in the session scratch folder, not committed.

## What this means
- Step 2 (own training script) is possible now: thesis loop + bundle `ODEFunc`, checked on the bundle's sample checkpoint's simulation.
- Items 7, 8, 14 are the real design decisions; everything else is given or ours.
- Open question that decides the lab need: can the force net be trained from our existing S0/S1 flight logs (state, RPM, thrust), or does the recipe's "flown under FBL without residual" matter? Test offline in step 3.

## Update after steps 2 and 3 (2026-10-22)
- Row 7 (hardware training input): force-only training with torque 0 and body-rate weight 0 works (`KNODE_TRAINING_STEP3_2026-10-22.md`).
- Row 8 (C_D): identified on our data, **≈ 0.55** with calibrated thrust (0.65 with the raw command).
- Row 14 (thrust input): the dominant sensitivity; use the rotor-speed force or command × 0.977.
- Open question answered: **our existing S0/S1 logs are enough for the training, no FBL lab flights needed**; the nets add nothing measurable over the ROM with the fitted C_D (consistent with Stage 1).

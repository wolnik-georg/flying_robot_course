# FBL residual training — what the authors' sources give us (Task 1, 2026-10-22)

Authors' reply (Pei-An, after our request for the training script): "most of the KNODE training code is in the *Training a KNODE Model* section of the appendix of Kongyao's thesis". **Decision (user, 2026-10-22): no reply email; everything missing is worked out ourselves from the bundle, the thesis, the papers and linked code.** Status: paused behind Task 2 (`docs/ns2_quality_plan_2026-10-22.md`).

## Access
- ProQuest link = paywalled preview only. **Open full text:** `https://repository.upenn.edu/server/api/core/bitstreams/2c3c1100-4418-49a7-830e-e026f20a6d02/content` (Chee, *Learning-Based Model Predictive Control for Aerial Vehicles*, UPenn 2024). Local copy of the text used for the check: scratchpad only (not committed, copyrighted).

## In the thesis (Appendix B, pp. 128–137)
- Complete generic KNODE training script: `NominalModel` + `ODEFunc` (nominal ODE + small tanh MLP), `get_batch`, `torchdiffeq.odeint` (RK4), loss = MSE of the one-step predicted state, Adam lr 2e-2, weight decay 2e-9, 2000 iterations, full batch, seed 123, keeps the best loss.
- Data: table of time, 13 states, 4 inputs, sampling ≤ 0.005 s; no derivative measurements needed.
- Data recipe (Ch. 3): static-top and stacked flights, 0.4 m separation, 0.4 m/s, closed loop, 28 s / 6 720 points; force network 1 hidden layer, 4 neurons (hardware); C_D 0.236 set by hand ("adjust until the controller performs reasonably").

## Not given → we decide and document
- Downwash-specific model (ROM + two tiny nets, neighbour state in the input, flow frame): in the authors' bundle, `KNODE/Training/` (`ODEFunc`, `NominalModel`), read-only.
- Hyperparameters of the 4/9-neuron nets, state-weight use (2026 paper: last three weights 0.1), C_D identification for our 41 g vehicle (sim result: best ≈ 0.6, try 0.43 first), hardware gains.

## Plan (after Task 2)
1. Read all linked sources fully (bundle, thesis App. B/C + Ch. 3, Hsieh 2026, Chee 2022/2024, Jiahao 2021, any linked repo) and list each gap with where it is (not) covered.
2. Own training script = thesis loop + bundle model; validate on the bundle's simulated data (must reproduce the sample checkpoint's behaviour).
3. Data pipeline from our logs; check that our thrust map is accurate enough for a ≈ 1.2 m/s² residual; cross-check with the supervised fit.
4. Training on real data needs lab flights with FBL and no learned residual.

## Step 1 DONE 2026-10-22
All sources read; item-by-item table of what is given, missing and decided: `KNODE_TRAINING_GAPS_2026-10-22.md`. Two public repositories of the KNODE author found (`TomJZ/Online-KNODE-MPC`, `TomJZ/K-NODE`, GPL v3, reference only). Real design decisions left: the torque/thrust input for hardware training, the C_D identification, the thrust input of the integration.

## Step 2 DONE 2026-10-22
Own training script written and checked on synthetic flights with a known downwash: `KNODE_TRAINING_STEP2_2026-10-22.md` (code `experiments/fbl/stage4_knode/`). Next: step 3 — data pipeline from our logs, C_D identification, thrust-input sensitivity, force-only training.

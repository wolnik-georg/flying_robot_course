# NS2 pilot retrain on the S0 bank, 2026-10-16 (desk, after lab part 1)

Purpose: see before the next lab session whether the data collected so far trains a network that generalises, where it is weak, and what the remaining flights must add. **Pilot only: not flashed, not the study network.** Registered as `ns2_pilot_2026-10-16` (`docs/ns2_model_registry.md`, sha256 `447a408f68ea…`); v1 untouched.

## Setup
- Data: 55 flights = 26 old v1 flights relabelled with the tree definition (`a_res_z = a_imu_f − a_rpm_f`, deck RPM, top labels zeroed) + 29 usable S0 flights of lab 2026-10-16 (A8 dz 0.25/0.5/0.75 ×5 speeds, A2 and A3 dz 0.5, A1 dz 0.75). Manifest `experiments/results/ns2_pilot_2026-10-16/training_manifest_pilot.json`; two outliers (A2 18-02-54, 18-26-13) excluded.
- Same architecture, epochs and gate thresholds as v1 (leave-one-flight-out; R² ≥ 0.35 and ≥ 50 % reduction vs zero); v1 scored on the same left-out flights with the same tree labels. Output `experiments/analysis/out/ns2_pilot_2026-10-16/` (`MODEL_CARD.json`, `loo_results.json`, `eval_model/eval_report.md`). Run time 55 folds × 48 s.

## Results
- Full-bank fit (in-sample): R² 0.911, RMSE 0.225 m/s² (zero baseline 0.940, −76 %).
- **New flights (29 left-out folds): 28 pass the gate thresholds, median R² 0.80; the new network has lower RMSE than v1 on 29 of 29.** v1 on the same flights: R² between -0.36 and 0.71 (median 0.30).
  - A8 (17 flights): R² 0.68–0.89 except one (the failing fold below); A3 (5): 0.79–0.84; A2 (6): 0.52–0.73; A1 dz 0.75 (1): 0.61.
  - **Only failing fold: A8 dz 0.25, 0.1 m/s (19-08-19): R² 0.43, 36.7 % vs zero** — the flight in which the bottom drone was pushed to 4 cm above the floor (ground effect mixed into the label). Next weakest: A8 dz 0.25, 0.2 m/s (R² 0.72, 55 %; bottom drone down to 5 cm).
- Old v1 flights (26 folds): 25 pass (only the out-of-scope A6 fails); the new network is worse than v1 on 10 of them, but v1 is in-sample there (trained on them), so this is not a fair comparison (decision of 2026-10-15).

## What it tells us
- A8, A3 and A2 data are learnable: unseen flights of the same scenario family are predicted well, and the new data is far better than v1 on the tree labels.
- **Caveat on "29/29 better than v1":** v1 was trained with the old logged label definition and never saw A8 or these dz values; part of the gap is the label-definition and coverage shift, not only more data.
- **Caveat on the folds:** leave-one-flight-out leaves near-identical repeats (3 takes of A8 at 0.5 m/s, 4 A3 passes) in the training set, so the fold scores are optimistic for repeated settings. The honest test is a held-out scenario or setting (e.g. train without A3, or without dz 0.75), not yet run.
- **Weak spots to cover next:** the close-gap / floor-hugging slow flights (ground effect vs wash), A1 holds (only 1 flight, R² 0.61), and anything unlike A8/A2/A3 (lateral offsets, random motion), which is exactly what A1, R1 and A4 add. This supports the planned lab-part-2 list; nothing in the pilot argues for A5, A2/A3 repeats or A7.

## Open
- Run the held-out-setting test (train without one scenario or one dz) to measure generalisation instead of repeat-take interpolation.
- Decide how to treat the floor-hugging slow flights (keep, or add a z-above-floor feature check) before the final retrain.

## Held-out-setting test (2026-10-16, `experiments/analysis/run_heldout_setting.py`)
Whole setting removed from training, then scored (same gate thresholds). v1 was trained on the old flights, so it is only an unbiased reference on the new 2026-10-16 flights.
| Held out | trained on | new flights (2026-10-16) | old flights (v1 in-sample) |
|---|---|---|---|
| **all A3** (12 flights) | 43 | 3 of 5 pass, median R² 0.69 (0.53–0.81); LOO had 0.79–0.84; better than v1 (RMSE) on 5/5 | 4 of 7 pass, median R² 0.71 (min 0.30) |
| **all dz 0.75** (7 flights: A8 ×5, A1 ×2) | 48 | **6 of 6 pass**, median R² 0.71 (A8 0.64–0.81, A1 0.51); LOO had A8 0.68–0.83, A1 0.61; better than v1 on 6/6 | the old A1 dz 0.75 fails (R² 0.39; it was already weak in LOO, 0.58) |
- **dz generalises:** leaving dz 0.75 out costs little (A8 R² −0.0…−0.1), consistent with the flat in-gate label across dz; extrapolating from dz 0.25/0.5 to 0.75 works.
- **The A3 family generalises only partly:** the new A3 flights drop 0.1–0.3 in R² and two (0.4 m/s and 0.5 m/s) fall below the 50 % threshold; the old A3 flights (dz 0.25 / 0.4, a different bottom speed profile) fail 3 of 7. A3-type motion (a fixed source with the bottom drone shuttling under it) is not fully covered by A8/A2/A1, so A3 stays in the training set; its closest relatives, A4 offset steps and R1, add the missing variety.
- Both models are registered as experiments (`ns2_heldout_noA3_2026-10-16`, `ns2_heldout_nodz075_2026-10-16`), not for flashing.

## Held-out-setting tests on the full bank (2026-10-17, pilot-2 manifest: 26 relabelled v1 + 47 usable flights of lab 2026-10-16/17)
| Held out | trained on | result on the left-out flights |
|---|---|---|
| **all R1** (9 flights) | 64 | **9/9 pass**, median R² 0.72 (min 0.59), better than v1 on 9/9 |
| **all A9** (3 flights) | 70 | **3/3 pass**, R² 0.74–0.79 (v1 0.61–0.64) |
| **all A1** (12 flights: 6 new, 6 old) | 61 | new 5/6 pass (median R² 0.72, but 0.09 on the dz 0.75 flight of 10-17); **old 1/6 pass (R² 0.20–0.30)** |
- Random motion (R1) and the offset-step flight (A9) are predicted well by a network that has never seen them: the structured scenarios (A8, A2, A3, A1, A4) already span their state space.
- **A1 (static hold, zero speed, zero lateral offset) is unlike the moving scenarios: the others cannot stand in for it.** With every A1 flight removed from training, the holds are predicted poorly (old holds R² 0.20–0.30). This is a test of what happens WITHOUT A1 data, not of the real network: the network trained for the study contains all 12 A1 flights (6 old + 6 new, five dz values), twice the 6 of v1. With the A1 flights in training, the leave-one-flight-out folds of the first pilot pass for all 7 A1 flights (R² 0.53–0.84, median 0.64, 54–78 % better than zero; the lowest of the scenario families but all above the gate). The full pilot-2 run gives the A1 folds with the 12 flights in training.
- Models registered as experiments (`ns2_heldout_noR1|noA9|noA1_2026-10-17`). Single runs, one training seed.

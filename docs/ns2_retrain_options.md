# NS2: keep the weights or retrain? — numbers for the decision (2026-10-11)

**SUPERSEDED 2026-10-17: the tree (S0–S3) needed dedicated training data, 73 flights were collected and `ns2_v2` was released (signed off by the user, `weights/NS2_V2.md`, `ns2_pilot_retrain_2026-10-16.md`).** *Earlier: Decision 2026-10-13: keep (K); no v2.* v1 is preserved as `flying_drone_stack/tools/residual/weights/ns2_v1.npz` (sha256 `8a43b5b5…`). (Was open: protocol §11.) The weights are frozen at the shakedown. This page only collects what the data says.

## Current weights
`experiments/analysis/out/c2_e2e_2026-10-01/full_bank_c1_complete.npz` (sha256 `8a43b5b5…`), trained on 26 geometric two-drone flights (A1–A7 C.1 flights, no A8): R² = 0.944, 81.7 % reduction against predict-zero, leave-one-out OK on every new fold (`project_ns2_lab_fastpath_2026-10-01`).

## Where the bank is thin (`docs/study_readiness_matrix.md` §2)
- The network acts only while the neighbour is inside the gate (|dx| < 0.2, |dy| < 0.2): **8–9 % of an A8 flight** (A8 is not in the bank), 16–33 % of A3/A4/A5, 100 % of A1/A2/A7.
- **dz 0.75 m is hardly covered:** 0.5 % of the bank's samples lie within ±5 cm of 0.75 m (13.4 % at 0.5 m, 10.6 % at 0.25 m). A1 at dz 0.75 therefore runs the network outside the data it was trained on.

## Dry runs: do the unused flights help? (`out/ns2_retrain_dryrun_2026-10-11*`, nothing flashed)
Candidates = geometric two-drone flights with a merged log that are not in the bank (A1 ×5, A2 ×2, A3, A4); same pipeline as the 10-01 retrain (`run_c2_lab_retrain.py`, 40 epochs, leave-one-out on the new scenarios).

| | flights | combined R² | LOO gate | new-fold LOO R² |
|---|---:|---:|---|---|
| current bank | 26 | 0.944 | passed | — |
| a: bank + 9 candidates | 35 | 0.887 | **NO-GO** — fold `A4_2026-09-28_17-21-40` R² −2.0 (RMSE 3.5 m/s²), the one flight that is an outlier by itself (per-flight R² 0.56 even with the weights trained on it) | other folds 0.59–0.88 |
| b: bank + 8 candidates (without that outlier) | 34 | 0.934 | passed | A1 0.45 / 0.63 / 0.69 / 0.78, A2 0.75 / 0.78, A3 0.88 |

Reading: the flights that were left out of the bank earlier add **no measurable gain** (0.934 against 0.944) and the A1 folds generalise poorly. The existing data cannot fix dz 0.75 or A8; only new flights can.

## Options
- **K — keep the weights.** S2 flies the bank network. Known limits: dz 0.75 (extrapolation) and A8 (network active ~8 % of the flight, as on 10-08/09: dip −5.9 → −3.4 cm).
- **R1 — retrain now with the 8 extra flights.** Not supported by the numbers above (no gain).
- **R2 — retrain after new geometric data at the thin settings.** Fly S0 on A1 at dz 0.25 and 0.75 (about 3–5 flights each, uSD from both drones; these can be shakedown flights), merge, retrain (`run_c2_lab_retrain.py`, ~20 min), re-run the leave-one-out gate, flash, freeze **before the data collection**. Cost: the flights, one retrain, one flash on both drones, and the S2 shakedown flights with the old weights must be repeated once (sanity).

Commands: `python3 experiments/analysis/run_c2_lab_retrain.py --manifest <manifest.json> --out-dir <dir> --new-scenarios A1` (system python3, torch); manifests of the dry runs are in the two output folders.

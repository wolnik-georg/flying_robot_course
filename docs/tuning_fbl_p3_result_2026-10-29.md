# P3: FBL end-to-end tuning on the calibrated plant (2026-10-29)

Setup: 6 gains (xy tied), 5 tuning cells x 4 search variants, budget 120 evaluations, population 12, 3 seeds, CMA-ES vs Sobol (same budget). Plant of record = `plant_calibration_joint_best_s0.json`. Raw: `experiments/tuning/out/fbl_{cmaes,sobol}_s*.jsonl`, `fbl_rerank.json`; script `rerank_fbl.py`.

| Method | best search score per seed (cm, 3-D RMS) | median |
|---|---|---|
| CMA-ES | 1.52 / 2.27 / 2.60 | 2.27 |
| Sobol | 1.94 / 2.56 / 3.41 | 2.56 |
| preset 1 | 24.7 (crashes one tuning cell in every variant) | |
| Step B candidate A | 22.4 (same crash) | |

Re-rank (best set per run), study cells (held out), RMS cm: nominal / ffs x0.4 / state delay 40 ms
- preset 1: 6.4 / 8.5 / 6.4; candidate A: 3.0 / 3.3 / 3.2
- CMA-ES s0/s1/s2: 1.2 / 1.5 / 1.7, 1.8 / 2.4 / 2.1, 2.8 / 3.0 / 2.9
- Sobol s0/s1/s2: 1.6 / 1.9 / 1.9, 2.0 / 2.4 / 2.4, 2.5 / 3.2 / 3.0

Full 9 variants (crash = 100): preset 1 18.8, A 20.3, CMA-ES 12.4 / 6.6 / 5.2, Sobol 8.4 / 11.1 / 13.9. Tuned sets crash only at state delay 60 ms (CMA-ES s2: 1 of 5 cells; s0: 5 of 5); preset 1 and A crash one cell in all variants.

Caveats
- CMA-ES beats Sobol only modestly with 3 seeds; seed spread (1.5-2.6) is as large as the method gap.
- Gains sit at the upper bounds (kp_xy 17-77, ki_xy 54-95, kp_z 31-45): the optimum is large gains, which cost delay margin (md 60 crashes). The plant does not contain the hardware delay uncertainty beyond the variants, so these are candidates, not results.
- Nothing flown. The hardware check is the confirmation (plan option 1): fly the top candidates (CMA-ES s2 as the most delay-robust, s0 as best nominal) against preset 1 and A.

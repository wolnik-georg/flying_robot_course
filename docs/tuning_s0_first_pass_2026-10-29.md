# S0 C-tree tuning, first pass (2026-10-29)

Setup: 10 parameters (Kpos_P xy/z, Kpos_D xy/z, Kpos_Iz, KR xy/z, Komega xy/z, KI xy), wide log bounds around the baseline, one job per scenario type (A1 2 cells, A4 3, A8 2; none is a study cell), 4 search variants (nominal, gyro noise x1.7, state delay 40 ms, feedforward x0.4), CMA-ES vs Sobol, 3 seeds, budget 240. Calibrated plant of record. Code: `experiments/tuning/run_job.py` (`s0_*`), `rerank_s0.py`; raw `out/s0_*.jsonl`, `out/s0_rerank.json`.

## Search score (best 3-D RMS per seed, cm; baseline 15.4 / 7.1 / 6.0)
| | CMA-ES | median | Sobol | median |
|---|---|---|---|---|
| A1 | 0.10 / 0.21 / 0.09 | 0.10 | 1.07 / 0.12 / 1.03 | 1.03 |
| A4 | 1.11 / 1.26 / 1.50 | 1.26 | 4.59 / 4.49 / 3.05 | 4.49 |
| A8 | 1.49 / 1.93 / 1.36 | 1.49 | 2.69 / 2.56 / 2.87 | 2.69 |

## Re-rank on the 9 full variants and the study cells
- Study cells, nominal plant: tuned sets 0.1-2 cm against the baseline 15.3 / 7.1 / 4.7 (A1 / A4 / A8).
- **Every tuned set crashes at motor lag 60 ms** (all cells of A1, A4, A8); the baseline never crashes in the 9 variants. A8 CMA-ES s0/s2 also crash at state delay 60 ms. Per variant, e.g. A8 CMA-ES s2: nominal 1.4/1.0, lag 60 -> 100/100. The tuned gains sit on the motor-lag stability edge (risk R7).
- A1 0.1 cm in the plant: the plant removes the tilt oscillation (max tilt 1-2 deg) that hardware shows at +-27 deg. Treat the magnitude as a plant artefact (the plant under-predicts S0 by 1.5-2 in z, see P4).

## Decision
Not usable as is. Rerun with lag 60 ms in the search variants (`s0r_*`, CMA-ES only, 3 seeds); select by the full-variant score, not the search score.

## Robust rerun (`s0r_*`, lag 60 ms added to the search variants, CMA-ES, 3 seeds, 2026-10-30)
Ranking on the 9 full variants (crashes of 63/45/18 episodes) and study cells (nominal / ffs 0.4 / md 40 / md 60), RMS cm:
| | baseline | best robust set | full score | crashes |
|---|---|---|---|---|
| A1 | 14.9 | CMA-ES s0 | 1.28 | 0 |
| A4 | 7.2 | CMA-ES s1 (s2 equal: 2.18) | 2.15 | 0 |
| A8 | 6.3 | CMA-ES s0 | 3.51 | 0 |
- Other seeds are worse: A1 s1/s2 and A8 s1/s2 still crash in one variant (md 60, lag 60), A4 s0 scores 4.8. Choose per scenario by the full-variant score.
- The chosen sets hold over motor lag 22-60 ms (22, 30, 36, 44, 60 checked): e.g. A1 tuned 0.3-1.1 cm against baseline 10-19 cm at every lag; A8 tuned 2.0-3.3 (5.0 on one cell at lag 60) against baseline 4-10.
- Gain changes: attitude gains stay near the baseline (KR 0.0066-0.0072, Komega xy 0.0008-0.0013); the position loop changes most (Pz 4.9-27.8 against 7, Dz 4.0-10.3 against 4, Iz 3.6-4.2 against 1.5; Pxy 2.6-10.6, Dxy 2.7-5.2, KI_xy 0.016-0.12 against 0.03). Differences between scenario types are large (A8 low Pxy, A4 high Pz).
- Remaining caveats: tuned plant magnitude (plant under-predicts S0 by 1.5-2 in z), nominal lag in the plant is 22 ms (data: 30, `tuning_actuator_id_2026-10-29.md`), unclamped KI, no hardware confirmation yet.
Files: `out/s0r_*.jsonl`, `out/s0r_rerank.json`, log `out/s0r_rerank.log`.

## Lag-30 plant (`s0n_*`, 2026-10-30): the result and why it was discarded
Same job on the recalibrated plant (lag 30 ms fixed), search variants + lag 45 ms. Search scores were far worse (A1 12-17 cm, A4 5.7-9.7, A8 2.7-7.3) and the baseline itself scored 63.7 / 42.5 / 33.1 on the 9 full variants (10 / 10 / 5 crashes). Two findings:
- The baseline crashes the A1 cell dz 0.25 (tilt 101 deg, a real flip in the plant) and the first A4 cell in every variant. Hardware has no S0 flight at dz 0.25 (smallest dz 0.375); the plume amplitude (pa 4.3) is extrapolated there. Treat the dz 0.25 cells as unvalidated.
- The crash threshold of 45 deg for the tree is inside the hardware limit cycle: real S0/S2 A1 flights peak at 41-44 deg (rms 27 deg) at every dz from 0.375 to 0.75 and do not crash (50 flights checked). Threshold raised to 70 deg (`plant_core.TREE_CRASH_TILT`), rerun as `s0p_*` (same job).
- The FBL sets tuned on the lag-22 plant do not transfer: on the lag-30 plant Step B candidate A and all tuned sets crash about half of the study cells (preset 1: 8.6 cm nominal). `out/fbl_rerank_lag30.log`. The Step B hardware check should wait until FBL is retuned on this plant.

## Lag-30 plant, crash threshold 70 deg (`s0p_*`, 2026-10-30/31): result
Study cells, nominal plant, RMS cm (baseline 19.9 / 11.0 / 5.3): best sets A1 4.1 (CMA-ES s0), A4 2.9 (s0) / 3.4 (s2) / 7.4 (s1), A8 1.7 (s0). `out/s0p_rerank.json`, `out/s0p_rerank.log`.
Crashes per variant (the 9 full variants; mild = all but lag 60 and state delay 60):
- All 9 tuned sets have 0 crashes in the 7 mild variants except A4 s0 (1: kt 0.5). The baseline crashes the A1 dz 0.25 cell in 5 of 7 mild variants and A4 once.
- All tuned sets crash in 1-3 episodes at lag 60 and/or state delay 60 (baseline: A1 and A4 and A8 at lag 60, too).
- Per scenario the trade-off is between score and stress margin: A1 s0 (mean 4.4 in mild variants, crashes at lag 60 / md 60 once each) against s1 (6.1, lag 60 only); A4 s1 (7.2, lag 60 only) against s2 (4.3, 3+3 crashes at lag/md 60); A8 s0 (2.3) against s1 / s2 (3.2-3.4, lag 60 only).
- Not yet validated: the dz 0.25 cells (no hardware S0 flight), hardware itself.

## Follow-up (2026-10-31)
S2 own tuning, option 1 (no dz 0.25 cells) and the FBL retune: `tuning_s2_option1_fbl_retune_2026-10-31.md`.

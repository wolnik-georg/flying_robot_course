# S0 auto-tuning: FINAL (closed 2026-11-22, user decision)

**Final theta_best = round 4, real cost 4.20 cm** (RMS over the 5 tuning flights, real flights, keep-best).
Gains file: `S0/round_4/gains.yaml`, sha256 prefix `13ba2c82` (full: `13ba2c825b26a280d75bc523f689f3935207df9333880a8da4fb5ba16d50c0be`).

## Real cost per round (cm, RMS over the 5 tuning flights)

| Round | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| S0 | 10.53 | 7.71 | 6.12 | 5.42 | 4.20 | **failed** |

## Final gains (round 4) against the defaults (round 0)

| Gain | round 0 | round 4 |
|---|---|---|
| Kpos_Px | 7 | 8.8524 |
| Kpos_Pz | 7 | 10.921 |
| Kpos_Dx | 4 | 7.3911 |
| Kpos_Dz | 4 | 9.5001 |
| Kpos_Iz | 1.5 | 3.6621 |
| KR_xy | 0.007 | 0.01442 |
| KR_z | 0.008 | 0.0073093 |
| Kw_xy | 0.00115 | 0.0011369 |
| Kw_z | 0.002 | 0.001638 |
| KI_xy | 0.03 | 0.047967 |

## Round 5 failed
- Round 5 gains: `S0/round_5/gains.yaml` (sha prefix `5ac91e29`), KR_xy about 0.018 (round 4: about 0.0145; round 0: 0.007), KR/Kw about 15 (0.01783 / 0.00113), PREVIEW PASSED anyway (the frsim preview is noise-free and did not predict the instability).
- Flights: A1 attempt 1 (14:42) and attempt 2 (14:52) flipped on cf5 at takeoff; A8, A4, CIRCLE and FIG8 were flown afterwards and flipped too (all radio logs: z max <= 0.23 m, roll 180 deg; merge flags crashed + floor).
- Round 5 data is not used (archived raw uSD only, in `experiments/logs/usd_raw/2026-11-21/*_S0r5`); no desk step was run; the round is not eligible for keep-best.
- **Process note:** flights continued after the first flip of the round, against the stop decision (abort at once after the first A1 flip, no further flights of the round). This is logged here and in `session_log_2026-11-21.md`.

## Test flights
Pending (S0_LAB_CARD item 12: A4 cell 2 offset 0.075 held-out and A4 cell 1 offset 0.125 in-sample, 3 x round 0 gains + 3 x theta_best, role test). Skipped for now by user decision, not cancelled.

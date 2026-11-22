# S2 auto-tuning: FINAL (closed 2026-11-22, user decision)

**Final theta_best = round 4, real cost 3.20 cm** (RMS over the 5 tuning flights, real flights, keep-best).
Gains file: `S2/round_4/gains.yaml`, sha256 prefix `d3ecb62d` (full: `d3ecb62d131ae63ce2aec8a7b27e9e15a6c1c5c6883503b9476048032888905f`).

## Real cost per round (cm, RMS over the 5 tuning flights)

| Round | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| S2 | 9.92 | 6.48 | 5.22 | 4.34 | 3.20 | **failed** |

## Final gains (round 4) against the defaults (round 0)

| Gain | round 0 | round 4 |
|---|---|---|
| Kpos_Px | 7 | 11.67 |
| Kpos_Pz | 7 | 6.5323 |
| Kpos_Dx | 4 | 6.3565 |
| Kpos_Dz | 4 | 4.2518 |
| Kpos_Iz | 1.5 | 3.308 |
| KR_xy | 0.007 | 0.014784 |
| KR_z | 0.008 | 0.0055549 |
| Kw_xy | 0.00115 | 0.0012297 |
| Kw_z | 0.002 | 0.0012361 |
| KI_xy | 0.03 | 0.050826 |

## Round 5 failed
- Round 5 gains: `S2/round_5/gains.yaml` (sha prefix `788405f1`), KR_xy about 0.018 (round 4: about 0.0145; round 0: 0.007), KR/Kw about 15 (0.01848 / 0.00126), PREVIEW PASSED anyway (the frsim preview is noise-free and did not predict the instability).
- Flights: A1 attempt 1 (15:12) and attempt 2 (15:14) flipped on cf5 after liftoff (z max 0.74 / 0.67 m, roll 159 / 179 deg); A8 flipped (roll 180 deg), A4 had an empty file and a partial flight (roll 59 deg); the solo flights flew without flipping but worse than round 4: CIRCLE 8.28 cm (round 4: 4.20), FIG8 9.39 cm (round 4: 4.46).
- Round 5 data is not used (archived raw uSD only, in `experiments/logs/usd_raw/2026-11-21/*_S2r5`); no desk step was run; the round is not eligible for keep-best.
- **Process note:** flights continued after the first flip of the round, against the stop decision (abort at once after the first A1 flip, no further flights of the round). This is logged here and in `session_log_2026-11-21.md`.

## Test flights
Pending (S0_LAB_CARD item 12: A4 cell 2 offset 0.075 held-out and A4 cell 1 offset 0.125 in-sample, 3 x round 0 gains + 3 x theta_best, role test). Skipped for now by user decision, not cancelled.

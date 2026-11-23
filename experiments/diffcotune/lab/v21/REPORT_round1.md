# v2.1 (ours21, linear +-10 %) round 1 gain sets from round 0 (default gains), for the planning agent check

Cost = position + attitude (att = 2(1-<q,q*>^2) ~ angle^2/2); attitude RMS deg = sqrt(2 x att). Round 0 flights = the v1 round 0 flights (same data). Nothing is flown before your check.

## S0

Round cost (mean of 5): total 0.03913 (pos 0.01108 + att 0.02806); 3-D position RMS 10.53 cm; attitude RMS 13.57 deg; crashed 0 floor 0

| flight | pos RMS cm | pos cost | att cost | att RMS deg | total cost | max tilt deg | gate |
|---|---|---|---|---|---|---|---|
| A1 | 18.70 | 0.03496 | 0.1326 | 29.50 | 0.1675 | 39.3 | ok |
| A8 | 6.01 | 0.003612 | 0.004293 | 5.31 | 0.007905 | 21.2 | ok |
| A4 | 9.57 | 0.00916 | 0.002552 | 4.09 | 0.01171 | 18.0 | ok |
| CIRCLE | 6.08 | 0.003697 | 0.000378 | 1.58 | 0.004075 | 5.2 | ok |
| FIG8 | 6.30 | 0.00397 | 0.0004653 | 1.75 | 0.004435 | 6.9 | ok |

J_sysId per part (mean over 5 flights, unweighted), before -> after the beta fit: p 0.01301 -> 0.01064, v 0.3015 -> 0.2397, q 0.02738 -> 0.02053, w 6.971 -> 5.508; total 7.313 -> 5.779 (fit epochs 51, best epoch 50)

Fitted beta factors (x default): thrust_gain 1.000, motor_lag_ms 1.153, downwash_scale 1.523, plume_scale 0.845

| gain | round 0 | round 1 | step x | at limit |
|---|---|---|---|---|
| Kpos_Px | 7 | 6.3 | 0.9000 | yes |
| Kpos_Py | 7 | 6.3 | 0.9000 | yes |
| Kpos_Pz | 7 | 7.7 | 1.1000 | yes |
| Kpos_Dx | 4 | 3.6 | 0.9000 | yes |
| Kpos_Dy | 4 | 3.6 | 0.9000 | yes |
| Kpos_Dz | 4 | 4.4 | 1.1000 | yes |
| Kpos_Iz | 1.5 | 1.6378 | 1.0919 |  |
| KR_x | 0.007 | 0.0074677 | 1.0668 |  |
| KR_y | 0.007 | 0.0074677 | 1.0668 |  |
| KR_z | 0.008 | 0.0076794 | 0.9599 |  |
| Kw_x | 0.00115 | 0.001265 | 1.1000 | yes |
| Kw_y | 0.00115 | 0.001265 | 1.1000 | yes |
| Kw_z | 0.002 | 0.0018713 | 0.9357 |  |
| KI_x | 0.03 | 0.027 | 0.9000 | yes |
| KI_y | 0.03 | 0.027 | 0.9000 | yes |

KR_x/Kw_x 6.09 -> 5.90; KR_z/Kw_z 4.00 -> 4.10
Tuning step: J 0.03377 -> 0.02563 (best epoch 40); gain-limit hits: [35, 29, 32, 13, 13, 0, 0, 21, 0, 36]

PREVIEW:
```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.78 cm, best flown set 7.96 cm (27 s)
  A1      new: 16.1 cm tilt+floor | 15.7 cm floor   best: 20.4 cm tilt+floor | 20.5 cm tilt+floor
  A8      new: 4.9 cm | 4.7 cm                      best: 5.8 cm | 5.6 cm
  CIRCLE  new: 3.2 cm | 3.3 cm                      best: 3.5 cm | 3.5 cm
  FIG8    new: 3.1 cm | 3.2 cm                      best: 3.3 cm | 3.4 cm
  A4      new: 6.8 cm floor | 6.7 cm floor          best: 7.0 cm floor | 6.7 cm floor
  events the best flown set shows too (not counted): A1 seed 2000 tilt, A1 seed 2000 floor, A1 seed 2001 floor, A4 seed 2000 floor, A4 seed 2001 floor
```

## S1

Round cost (mean of 5): total 0.003428 (pos 0.001082 + att 0.002346); 3-D position RMS 3.29 cm; attitude RMS 3.92 deg; crashed 0 floor 0

| flight | pos RMS cm | pos cost | att cost | att RMS deg | total cost | max tilt deg | gate |
|---|---|---|---|---|---|---|---|
| A1 | 2.81 | 0.000789 | 0.004238 | 5.27 | 0.005027 | 13.0 | ok |
| A8 | 4.84 | 0.002345 | 0.004087 | 5.18 | 0.006433 | 26.9 | ok |
| A4 | 2.47 | 0.0006105 | 0.001386 | 3.02 | 0.001996 | 10.1 | ok |
| CIRCLE | 3.13 | 0.0009779 | 0.001025 | 2.59 | 0.002003 | 9.3 | ok |
| FIG8 | 2.62 | 0.0006868 | 0.0009951 | 2.56 | 0.001682 | 10.8 | ok |

J_sysId per part (mean over 5 flights, unweighted), before -> after the beta fit: p 0.01119 -> 0.01559, v 0.1185 -> 0.03288, q 0.01003 -> 0.003, w 3.661 -> 1.735; total 3.801 -> 1.787 (fit epochs 51, best epoch 50)

Fitted beta factors (x default): thrust_gain 0.751, motor_lag_ms 0.745, downwash_scale 0.772, plume_scale 0.673, rpm_delay 0.769

| gain | round 0 | round 1 | step x | at limit |
|---|---|---|---|---|
| Kpos_Px | 7 | 7.7 | 1.1000 | yes |
| Kpos_Py | 7 | 7.7 | 1.1000 | yes |
| Kpos_Pz | 7 | 6.3 | 0.9000 | yes |
| Kpos_Dx | 4 | 4.4 | 1.1000 | yes |
| Kpos_Dy | 4 | 4.4 | 1.1000 | yes |
| Kpos_Dz | 4 | 3.6 | 0.9000 | yes |
| Kpos_Iz | 1.5 | 1.65 | 1.1000 | yes |
| KR_x | 0.007 | 0.0077 | 1.1000 | yes |
| KR_y | 0.007 | 0.0077 | 1.1000 | yes |
| KR_z | 0.008 | 0.0088 | 1.1000 | yes |
| Kw_x | 0.00115 | 0.0011781 | 1.0245 |  |
| Kw_y | 0.00115 | 0.0011781 | 1.0245 |  |
| Kw_z | 0.002 | 0.0022 | 1.1000 | yes |
| KI_x | 0.03 | 0.033 | 1.1000 | yes |
| KI_y | 0.03 | 0.033 | 1.1000 | yes |

KR_x/Kw_x 6.09 -> 6.54; KR_z/Kw_z 4.00 -> 4.00
Tuning step: J 0.01151 -> 0.007903 (best epoch 42); gain-limit hits: [26, 40, 41, 40, 41, 41, 40, 0, 40, 40]

PREVIEW:
```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.50 cm, best flown set 7.96 cm (25 s)
  A1      new: 13.0 cm | 12.7 cm                    best: 14.3 cm | 14.5 cm
  A8      new: 4.3 cm | 4.3 cm                      best: 5.1 cm | 5.1 cm
  CIRCLE  new: 5.3 cm | 5.5 cm                      best: 7.1 cm | 7.2 cm
  FIG8    new: 5.0 cm | 4.9 cm                      best: 6.6 cm | 6.5 cm
  A4      new: 5.1 cm | 4.9 cm                      best: 6.7 cm | 6.4 cm
```

## S2

Round cost (mean of 5): total 0.04489 (pos 0.009849 + att 0.03504); 3-D position RMS 9.92 cm; attitude RMS 15.17 deg; crashed 0 floor 0

| flight | pos RMS cm | pos cost | att cost | att RMS deg | total cost | max tilt deg | gate |
|---|---|---|---|---|---|---|---|
| A1 | 19.31 | 0.03731 | 0.1675 | 33.17 | 0.2048 | 41.5 | ok |
| A8 | 3.97 | 0.001577 | 0.00476 | 5.59 | 0.006337 | 26.7 | ok |
| A4 | 4.26 | 0.001812 | 0.001996 | 3.62 | 0.003808 | 20.1 | ok |
| CIRCLE | 6.44 | 0.004147 | 0.0003853 | 1.59 | 0.004532 | 5.1 | ok |
| FIG8 | 6.64 | 0.004405 | 0.0005077 | 1.83 | 0.004913 | 7.0 | ok |

J_sysId per part (mean over 5 flights, unweighted), before -> after the beta fit: p 0.01131 -> 0.01094, v 0.324 -> 0.2744, q 0.03186 -> 0.02728, w 7.846 -> 6.72; total 8.213 -> 7.033 (fit epochs 51, best epoch 50)

Fitted beta factors (x default): thrust_gain 0.895, motor_lag_ms 0.978, downwash_scale 0.931, plume_scale 0.910

| gain | round 0 | round 1 | step x | at limit |
|---|---|---|---|---|
| Kpos_Px | 7 | 6.5854 | 0.9408 |  |
| Kpos_Py | 7 | 6.5854 | 0.9408 |  |
| Kpos_Pz | 7 | 7.6558 | 1.0937 |  |
| Kpos_Dx | 4 | 3.6 | 0.9000 | yes |
| Kpos_Dy | 4 | 3.6 | 0.9000 | yes |
| Kpos_Dz | 4 | 4.1766 | 1.0442 |  |
| Kpos_Iz | 1.5 | 1.65 | 1.1000 | yes |
| KR_x | 0.007 | 0.0075549 | 1.0793 |  |
| KR_y | 0.007 | 0.0075549 | 1.0793 |  |
| KR_z | 0.008 | 0.007201 | 0.9001 |  |
| Kw_x | 0.00115 | 0.001265 | 1.1000 | yes |
| Kw_y | 0.00115 | 0.001265 | 1.1000 | yes |
| Kw_z | 0.002 | 0.0020825 | 1.0413 |  |
| KI_x | 0.03 | 0.027533 | 0.9178 |  |
| KI_y | 0.03 | 0.027533 | 0.9178 |  |

KR_x/Kw_x 6.09 -> 5.97; KR_z/Kw_z 4.00 -> 3.46
Tuning step: J 0.03851 -> 0.03029 (best epoch 25); gain-limit hits: [0, 4, 39, 0, 40, 5, 20, 36, 0, 7]

PREVIEW:
```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.91 cm, best flown set 7.95 cm (18 s)
  A1      new: 16.3 cm tilt | 16.5 cm tilt+floor    best: 20.8 cm tilt+floor | 20.5 cm tilt+floor
  A8      new: 3.5 cm | 3.4 cm                      best: 3.9 cm | 3.9 cm
  CIRCLE  new: 4.2 cm | 4.2 cm                      best: 4.5 cm | 4.6 cm
  FIG8    new: 3.8 cm | 3.8 cm                      best: 4.1 cm | 4.1 cm
  A4      new: 6.7 cm | 6.7 cm                      best: 6.6 cm | 6.6 cm
  events the best flown set shows too (not counted): A1 seed 2000 tilt, A1 seed 2001 tilt, A1 seed 2001 floor
```

## S3

Round cost (mean of 5): total 0.005392 (pos 0.001211 + att 0.004181); 3-D position RMS 3.48 cm; attitude RMS 5.24 deg; crashed 0 floor 0

| flight | pos RMS cm | pos cost | att cost | att RMS deg | total cost | max tilt deg | gate |
|---|---|---|---|---|---|---|---|
| A1 | 4.21 | 0.001769 | 0.01245 | 9.04 | 0.01422 | 16.6 | ok |
| A8 | 2.59 | 0.0006721 | 0.004073 | 5.17 | 0.004745 | 25.1 | ok |
| A4 | 2.19 | 0.0004783 | 0.001843 | 3.48 | 0.002321 | 14.5 | ok |
| CIRCLE | 3.65 | 0.001331 | 0.001405 | 3.04 | 0.002736 | 11.4 | ok |
| FIG8 | 4.25 | 0.001804 | 0.001137 | 2.73 | 0.002941 | 12.5 | ok |

J_sysId per part (mean over 5 flights, unweighted), before -> after the beta fit: p 0.0134 -> 0.01213, v 0.1433 -> 0.1347, q 0.01105 -> 0.009599, w 4.499 -> 3.564; total 4.666 -> 3.72 (fit epochs 51, best epoch 45)

Fitted beta factors (x default): thrust_gain 1.101, motor_lag_ms 0.988, downwash_scale 1.114, plume_scale 0.960, rpm_delay 0.887

| gain | round 0 | round 1 | step x | at limit |
|---|---|---|---|---|
| Kpos_Px | 7 | 6.9303 | 0.9900 |  |
| Kpos_Py | 7 | 6.9303 | 0.9900 |  |
| Kpos_Pz | 7 | 6.9303 | 0.9900 |  |
| Kpos_Dx | 4 | 4.0402 | 1.0101 |  |
| Kpos_Dy | 4 | 4.0402 | 1.0101 |  |
| Kpos_Dz | 4 | 3.9602 | 0.9900 |  |
| Kpos_Iz | 1.5 | 1.4851 | 0.9900 |  |
| KR_x | 0.007 | 0.0070704 | 1.0101 |  |
| KR_y | 0.007 | 0.0070704 | 1.0101 |  |
| KR_z | 0.008 | 0.0079204 | 0.9900 |  |
| Kw_x | 0.00115 | 0.0011386 | 0.9900 |  |
| Kw_y | 0.00115 | 0.0011386 | 0.9900 |  |
| Kw_z | 0.002 | 0.0020201 | 1.0101 |  |
| KI_x | 0.03 | 0.030302 | 1.0101 |  |
| KI_y | 0.03 | 0.030302 | 1.0101 |  |

KR_x/Kw_x 6.09 -> 6.21; KR_z/Kw_z 4.00 -> 3.92
Tuning step: J 0.02244 -> 0.0207 (best epoch 1); gain-limit hits: [0, 0, 0, 0, 0, 0, 0, 6, 11, 6]

PREVIEW:
```
PREVIEW FAILED: A1 seed 2000: floor; A1 seed 2001: floor
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.78 cm, best flown set 6.93 cm (24 s)
  A1      new: 19.0 cm floor | 18.4 cm floor        best: 17.4 cm | 19.6 cm crash
  A8      new: 6.7 cm | 6.1 cm                      best: 6.8 cm | 6.3 cm
  CIRCLE  new: 2.1 cm | 2.8 cm                      best: 2.1 cm | 2.8 cm
  FIG8    new: 1.9 cm | 1.5 cm                      best: 1.9 cm | 1.5 cm
  A4      new: 4.0 cm | 5.3 cm                      best: 5.1 cm | 5.7 cm
```

## Planning agent check and user decision (23 Nov night)

- Round 0 is NOT re-flown: round 0 reuses the flown default-gain round 0 flights of all four (user decision); the next flights are the v2.1 round 1 sets.
- Checked OK: all steps in [0.90, 1.10]; rpm_delay fitted for S1/S3 only (0.769 / 0.887); J_sysId total falls for all four; KR/Kw 5.9-6.5.
- Note S1: the J_sysId p part rises 0.0112 -> 0.0156 while the total falls 3.80 -> 1.79 (the w part dominates, 3.66 -> 1.74).
- S1, S0, S2 round 1: PREVIEW PASSED, cleared to fly.
- S3 round 1: PREVIEW FAILED (A1 floor in sim, both seeds). USER DECISION: fly it anyway. Reason: the set is within +-1 % of the default gains (tune best epoch 1); those gains flew A1 on hardware with 4.2 cm and no floor; in sim the best flown set itself crashes on A1 seed 2001.
- Next session: prep and cards for round 1 of all four (GAINS_ROOT=lab/v21), S1 + S3 first, then S0 + S2; at most 2 desk steps at once; uSD logs only, cards < 85 files; lab lock before the first flight (FBL worktree untouched).

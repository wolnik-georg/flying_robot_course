# Protocol v2 (ours2, +-25 %) report, 23 Nov 2026

v2 = cost ||p - p*||^2 + tr(I - R*^T R)/2 (weight 1), J_sysId p v q w unweighted, beta 4 + rpm_delay fitted for S1 / S3, +-25 % step. Round 0 = the existing v1 round 0 flights (default gains) re-run through the v2 desk step. Total cost = position + attitude term; "keep-best" is on the total cost. Attitude RMS (deg) = sqrt(2 x attitude term). All gates ok, no flight crashed or touched the floor in any round below.

## S0

### Round 0 (cost flown with the default gains)

| flight | file | position RMS (cm) | attitude RMS (deg) | total cost | max tilt (deg) |
|---|---|---|---|---|---|
| A1 | A1_att2.csv | 18.70 | 29.5 | 1.68e-01 | 39 |
| A8 | A8.csv | 6.01 | 5.3 | 7.90e-03 | 21 |
| A4 | A4_att2.csv | 9.57 | 4.1 | 1.17e-02 | 18 |
| CIRCLE | CIRCLE.csv | 6.08 | 1.6 | 4.07e-03 | 5 |
| FIG8 | FIG8.csv | 6.30 | 1.7 | 4.44e-03 | 7 |

Round: position RMS **10.53 cm**, attitude term 2.81e-02, total cost **3.91e-02**; keep-best after this round: round 0 (3.91e-02).

Fit J_sysId 7.313 -> 5.779; beta log-factors [-0.0, 0.142, 0.421, -0.169].

Preview of the next gains: PASSED (predicted position RMS 6.95 cm new vs 7.96 cm best flown set).

### Round 1 (cost flown with the round-1 gains)

| flight | file | position RMS (cm) | attitude RMS (deg) | total cost | max tilt (deg) |
|---|---|---|---|---|---|
| A1 | A1.csv | 14.82 | 30.2 | 1.61e-01 | 37 |
| A8 | A8.csv | 5.28 | 5.1 | 6.77e-03 | 22 |
| A4 | A4_att1.csv | 7.66 | 4.3 | 8.68e-03 | 25 |
| CIRCLE | CIRCLE.csv | 7.67 | 1.8 | 6.36e-03 | 6 |
| FIG8 | FIG8.csv | 7.83 | 2.0 | 6.71e-03 | 7 |

Round: position RMS **9.23 cm**, attitude term 2.94e-02, total cost **3.79e-02**; keep-best after this round: round 1 (3.79e-02).

Fit J_sysId 5.773 -> 5.697; beta log-factors [0.01, 0.152, 0.431, -0.159].

Preview of the next gains: PASSED (predicted position RMS 7.27 cm new vs 6.99 cm best flown set).

## S1

### Round 0 (cost flown with the default gains)

| flight | file | position RMS (cm) | attitude RMS (deg) | total cost | max tilt (deg) |
|---|---|---|---|---|---|
| A1 | A1.csv | 2.81 | 5.3 | 5.03e-03 | 13 |
| A8 | A8.csv | 4.84 | 5.2 | 6.43e-03 | 27 |
| A4 | A4.csv | 2.47 | 3.0 | 2.00e-03 | 10 |
| CIRCLE | CIRCLE.csv | 3.13 | 2.6 | 2.00e-03 | 9 |
| FIG8 | FIG8.csv | 2.62 | 2.6 | 1.68e-03 | 11 |

Round: position RMS **3.29 cm**, attitude term 2.35e-03, total cost **3.43e-03**; keep-best after this round: round 0 (3.43e-03).

Fit J_sysId 3.801 -> 1.787; beta log-factors [-0.286, -0.294, -0.259, -0.396, -0.263].

Preview of the next gains: PASSED (predicted position RMS 4.92 cm new vs 7.96 cm best flown set).

### Round 1 (cost flown with the round-1 gains)

| flight | file | position RMS (cm) | attitude RMS (deg) | total cost | max tilt (deg) |
|---|---|---|---|---|---|
| A1 | A1.csv | 1.98 | 4.4 | 3.35e-03 | 12 |
| A8 | A8.csv | 4.16 | 5.1 | 5.68e-03 | 24 |
| A4 | A4.csv | 2.03 | 3.0 | 1.80e-03 | 9 |
| CIRCLE | CIRCLE.csv | 3.09 | 2.6 | 1.99e-03 | 10 |
| FIG8 | FIG8.csv | 2.21 | 2.7 | 1.57e-03 | 10 |

Round: position RMS **2.82 cm**, attitude term 2.09e-03, total cost **2.88e-03**; keep-best after this round: round 1 (2.88e-03).

Fit J_sysId 1.43 -> 1.307; beta log-factors [-0.35, -0.336, -0.24, -0.696, -0.232].

Preview of the next gains: PASSED (predicted position RMS 2.98 cm new vs 5.01 cm best flown set).

### Round 2 (flown, gains `lab/v2/S1/round_2`, KR/Kw 8.29, KR_x equal to the v1 round 2 set that oscillated)

Operator observation: large oscillation. Only the A1 radio log (20:23:08) and meta are in the repo; the uSD cards of this round were not copied, so no merged files, no per-flight numbers and no desk step. Kept as data (negative result).

## S2

### Round 0 (cost flown with the default gains)

| flight | file | position RMS (cm) | attitude RMS (deg) | total cost | max tilt (deg) |
|---|---|---|---|---|---|
| A1 | A1.csv | 19.31 | 33.2 | 2.05e-01 | 42 |
| A8 | A8.csv | 3.97 | 5.6 | 6.34e-03 | 27 |
| A4 | A4.csv | 4.26 | 3.6 | 3.81e-03 | 20 |
| CIRCLE | CIRCLE.csv | 6.44 | 1.6 | 4.53e-03 | 5 |
| FIG8 | FIG8.csv | 6.64 | 1.8 | 4.91e-03 | 7 |

Round: position RMS **9.92 cm**, attitude term 3.50e-02, total cost **4.49e-02**; keep-best after this round: round 0 (4.49e-02).

Fit J_sysId 8.213 -> 7.033; beta log-factors [-0.111, -0.022, -0.072, -0.094].

Preview of the next gains: PASSED (predicted position RMS 6.38 cm new vs 7.95 cm best flown set).

### Round 1 (flown, desk step STOPPED before the fit by decision, not run to the end)

| flight | file | position RMS (cm) | attitude RMS (deg) | total cost | max tilt (deg) |
|---|---|---|---|---|---|
| A1 | A1.csv | 13.03 | 29.8 | 1.52e-01 | 36 |
| A8 | A8.csv | 4.03 | 4.6 | 4.86e-03 | 21 |
| A4 | A4.csv | 5.09 | 3.8 | 4.81e-03 | 20 |
| CIRCLE | CIRCLE.csv | 7.72 | 1.7 | 6.40e-03 | 6 |
| FIG8 | FIG8.csv | 8.15 | 2.2 | 7.35e-03 | 8 |

Round: position RMS 8.22 cm, total cost 3.51e-02 (round 0: 9.92 cm, 4.49e-2) -> would be the new best.

## S3

### Round 0 (cost flown with the default gains)

| flight | file | position RMS (cm) | attitude RMS (deg) | total cost | max tilt (deg) |
|---|---|---|---|---|---|
| A1 | A1.csv | 4.21 | 9.0 | 1.42e-02 | 17 |
| A8 | A8.csv | 2.59 | 5.2 | 4.75e-03 | 25 |
| A4 | A4.csv | 2.19 | 3.5 | 2.32e-03 | 15 |
| CIRCLE | CIRCLE.csv | 3.65 | 3.0 | 2.74e-03 | 11 |
| FIG8 | FIG8.csv | 4.25 | 2.7 | 2.94e-03 | 13 |

Round: position RMS **3.48 cm**, attitude term 4.18e-03, total cost **5.39e-03**; keep-best after this round: round 0 (5.39e-03).

Fit J_sysId 4.666 -> 3.72; beta log-factors [0.097, -0.012, 0.108, -0.041, -0.12].

Preview of the next gains: FAILED (predicted position RMS 6.78 cm new vs 6.93 cm best flown set).

### Round 1 (cost flown with the round-1 gains)

| flight | file | position RMS (cm) | attitude RMS (deg) | total cost | max tilt (deg) |
|---|---|---|---|---|---|
| A1 | A1.csv | 4.19 | 4.4 | 4.68e-03 | 11 |
| A8 | A8.csv | 2.43 | 5.1 | 4.48e-03 | 23 |
| A4 | A4.csv | 2.71 | 3.1 | 2.24e-03 | 14 |
| CIRCLE | CIRCLE.csv | 4.02 | 4.3 | 4.48e-03 | 13 |
| FIG8 | FIG8.csv | 4.43 | 3.2 | 3.48e-03 | 13 |

Round: position RMS **3.65 cm**, attitude term 2.54e-03, total cost **3.87e-03**; keep-best after this round: round 1 (3.87e-03).

Fit J_sysId 4.125 -> 3.141; beta log-factors [0.079, -0.021, 0.09, -0.142, -0.138].

Preview of the next gains: PASSED (predicted position RMS 5.66 cm new vs 6.01 cm best flown set).

## Stopped, and why
- S1 round 2: flown with the planning agent's go (KR/Kw 8.29); oscillation observed by the operator; no desk step (not all 5 flights available).
- S2 round 1 desk step: started by its waiter, stopped by decision (end of session / protocol v2.1); only the ingest line exists in `lab/v2/S2/desk_1.log`; no round_2 gains.
- Regression waiter (ours2 desk regression): stopped by decision before it started; replaced by the checks of the v2.1 finalisation.
- S0 round 1 and S3 round 1 desk steps finished (round_2 gains exist in `lab/v2/S0/round_2`, `lab/v2/S3/round_2`, not flown).

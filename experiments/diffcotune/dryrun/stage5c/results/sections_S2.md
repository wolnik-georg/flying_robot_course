| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 15.71 | 14.37 | A1 | 26.1 |
| theta_1 | 8.92 | 5.45 | 0 | 23.5 |
| theta_2 | 7.28 | 3.10 | 0 | 23.6 |
| theta_3 | 7.00 | 2.35 | 0 | 23.7 |
| theta_4 | 6.81 | 1.95 | 0 | 0.1 |

Keep-best over the flown sets: theta_4 (6.81 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 50 | 0.18423 | 0.16767 | - |
| 0 | tune theta | 51 | 50 | 0.066804 | 0.018582 | [0, 22, 0, 0, 26, 0, 0, 0, 23, 0] |
| 1 | fit beta | 51 | 50 | 0.15873 | 0.15743 | - |
| 1 | tune theta | 51 | 49 | 0.0092268 | 0.0058374 | [0, 28, 0, 20, 24, 0, 17, 0, 0, 0] |
| 2 | fit beta | 51 | 50 | 0.13524 | 0.1351 | - |
| 2 | tune theta | 51 | 43 | 0.0063997 | 0.0060053 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 3 | fit beta | 51 | 49 | 0.13319 | 0.13309 | - |
| 3 | tune theta | 51 | 46 | 0.0054184 | 0.0047603 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume):

| Fit of round | error % |
|---|---|
| 0 | -12.9 / -37.6 / +15.0 / +34.7 |
| 1 | -0.9 / -16.8 / +30.9 / +10.0 |
| 2 | -2.8 / -4.1 / +38.9 / +8.6 |
| 3 | -1.0 / +0.8 / +33.0 / -2.7 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 29.01 / 26.98 / 5 | 8.40 / 6.35 / 0 | 6.28 / 5.94 / 0 | 5.77 / 5.31 / 0 | 6.81 / 5.80 / 0 | 8.71 / 7.37 / 0 | 4.78 / 4.59 / 0 | 12.36 | 6.77 |
| theta_best (round 4) | 12.63 / 2.89 / 0 | 3.81 / 1.79 / 0 | 2.13 / 0.38 / 0 | 2.45 / 0.24 / 0 | 6.55 / 2.70 / 0 | 7.79 / 3.79 / 0 | 1.44 / 0.34 / 0 | 5.25 | 5.26 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 8.92, limit 9.92) | **yes** (7.28, 7.00, 6.81) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **yes** (theta_best 5.25 / 5.26, theta_0 12.36 / 6.77) |

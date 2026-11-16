| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 10.32 | 7.12 | 0 | 22.7 |
| theta_1 | 6.80 | 3.47 | 0 | 20.4 |
| theta_2 | 5.05 | 1.90 | 0 | 20.4 |
| theta_3 | 3.82 | 1.38 | 0 | 20.4 |
| theta_4 | 2.93 | 1.59 | 0 | 0.1 |

Keep-best over the flown sets: theta_4 (2.93 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 50 | 0.12815 | 0.074065 | - |
| 0 | tune theta | 51 | 50 | 0.0094122 | 0.0041327 | [0, 28, 26, 21, 26, 25, 0, 0, 21, 26] |
| 1 | fit beta | 51 | 50 | 0.036118 | 0.03587 | - |
| 1 | tune theta | 51 | 40 | 0.0042068 | 0.0019366 | [0, 0, 25, 0, 0, 23, 6, 0, 0, 21] |
| 2 | fit beta | 51 | 50 | 0.010256 | 0.0099524 | - |
| 2 | tune theta | 51 | 50 | 0.0022421 | 0.00078011 | [0, 2, 26, 13, 1, 0, 0, 0, 0, 19] |
| 3 | fit beta | 51 | 45 | 0.0053363 | 0.0052976 | - |
| 3 | tune theta | 51 | 50 | 0.0011383 | 0.00024755 | [0, 20, 27, 6, 6, 0, 0, 0, 0, 24] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume / rpm delay):

| Fit of round | error % |
|---|---|
| 0 | -1.4 / -29.2 / -42.5 / +9.4 / -49.4 |
| 1 | -0.1 / -4.1 / -60.0 / +15.3 / -49.4 |
| 2 | +3.7 / +26.8 / -73.9 / +5.8 / -49.4 |
| 3 | +1.8 / +19.8 / -62.7 / +8.1 / -49.4 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 17.93 / 10.92 / 0 | 6.85 / 5.29 / 0 | 5.95 / 5.55 / 0 | 5.41 / 4.84 / 0 | 5.33 / 4.57 / 0 | 7.38 / 5.69 / 0 | 4.49 / 4.21 / 0 | 9.04 | 5.73 |
| theta_best (round 4) | 4.84 / 2.46 / 0 | 1.85 / 1.20 / 0 | 1.19 / 0.59 / 0 | 1.31 / 0.69 / 0 | 1.63 / 1.49 / 0 | 2.62 / 2.40 / 0 | 1.19 / 0.90 / 0 | 2.30 | 1.81 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 6.80, limit 7.80) | **yes** (5.05, 3.82, 2.93) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **yes** (theta_best 2.30 / 1.81, theta_0 9.04 / 5.73) |

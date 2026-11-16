| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 10.75 | 7.52 | 0 | 28.7 |
| theta_1 | 10.56 | 7.47 | 0 | 25.2 |
| theta_2 | 10.28 | 7.51 | 0 | 25.2 |
| theta_3 | 10.26 | 7.22 | 0 | 25.3 |
| theta_4 | 9.26 | 6.45 | 0 | 0.1 |

Keep-best over the flown sets: theta_4 (9.26 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 50 | 0.14825 | 0.13188 | - |
| 0 | tune theta | 51 | 13 | 0.011728 | 0.010472 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 1 | fit beta | 51 | 50 | 0.12876 | 0.1151 | - |
| 1 | tune theta | 51 | 8 | 0.027231 | 0.016267 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 2 | fit beta | 51 | 50 | 0.1142 | 0.10556 | - |
| 2 | tune theta | 51 | 8 | 0.042407 | 0.038373 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 3 | fit beta | 51 | 50 | 0.10519 | 0.10059 | - |
| 3 | tune theta | 51 | 32 | 0.040895 | 0.035964 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume / rpm delay):

| Fit of round | error % |
|---|---|
| 0 | +41.3 / -46.2 / -29.9 / +65.6 / -63.1 |
| 1 | +10.1 / -50.7 / -42.9 / +101.8 / -74.5 |
| 2 | -0.1 / -69.1 / -46.7 / +151.6 / -83.1 |
| 3 | +6.3 / -78.8 / -60.2 / +174.0 / -89.1 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 18.73 / 11.92 / 0 | 6.10 / 4.21 / 0 | 5.91 / 5.51 / 0 | 5.33 / 4.76 / 0 | 5.82 / 5.21 / 0 | 7.02 / 5.36 / 0 | 4.49 / 4.21 / 0 | 9.02 | 5.78 |
| theta_best (round 4) | 16.58 / 10.35 / 0 | 5.37 / 4.09 / 0 | 6.25 / 5.91 / 0 | 5.60 / 5.13 / 0 | 6.11 / 5.71 / 0 | 6.63 / 5.52 / 0 | 4.72 / 4.48 / 0 | 8.45 | 5.82 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 10.56, limit 11.56) | **yes** (10.28, 10.26, 9.26) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **no** (theta_best 8.45 / 5.82, theta_0 9.02 / 5.78) |

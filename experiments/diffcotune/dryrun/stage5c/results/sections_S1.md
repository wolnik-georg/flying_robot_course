| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 10.32 | 7.12 | 0 | 22.6 |
| theta_1 | 6.93 | 3.64 | 0 | 20.9 |
| theta_2 | 5.26 | 2.26 | 0 | 20.6 |
| theta_3 | 4.11 | 1.91 | 0 | 20.7 |
| theta_4 | 3.04 | 1.82 | 0 | 0.1 |

Keep-best over the flown sets: theta_4 (3.04 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 50 | 0.12815 | 0.074054 | - |
| 0 | tune theta | 51 | 50 | 0.0093921 | 0.0040964 | [0, 28, 28, 26, 26, 27, 0, 6, 21, 26] |
| 1 | fit beta | 51 | 50 | 0.040817 | 0.040512 | - |
| 1 | tune theta | 51 | 50 | 0.0039358 | 0.0020604 | [0, 0, 10, 0, 0, 2, 0, 0, 0, 0] |
| 2 | fit beta | 51 | 50 | 0.015053 | 0.014632 | - |
| 2 | tune theta | 51 | 50 | 0.0014732 | 0.00049593 | [16, 0, 25, 5, 0, 0, 0, 0, 0, 28] |
| 3 | fit beta | 51 | 50 | 0.0065075 | 0.006496 | - |
| 3 | tune theta | 51 | 49 | 0.00047738 | 0.00013282 | [20, 26, 5, 23, 27, 0, 0, 0, 9, 0] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume / rpm delay):

| Fit of round | error % |
|---|---|
| 0 | -1.3 / -29.6 / -42.6 / +9.2 / -43.8 |
| 1 | -0.1 / -1.3 / -61.2 / +10.4 / -21.6 |
| 2 | +4.7 / +63.0 / -74.8 / -12.9 / +28.9 |
| 3 | +4.5 / +59.2 / -74.9 / -15.0 / +82.1 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 17.93 / 10.92 / 0 | 6.85 / 5.29 / 0 | 5.95 / 5.55 / 0 | 5.41 / 4.84 / 0 | 5.33 / 4.57 / 0 | 7.38 / 5.69 / 0 | 4.49 / 4.21 / 0 | 9.04 | 5.73 |
| theta_best (round 4) | 5.07 / 2.82 / 0 | 1.58 / 0.99 / 0 | 1.10 / 0.60 / 0 | 1.24 / 0.71 / 0 | 1.68 / 1.57 / 0 | 2.53 / 2.32 / 0 | 1.14 / 0.91 / 0 | 2.25 | 1.79 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 6.93, limit 7.93) | **yes** (5.26, 4.11, 3.04) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **yes** (theta_best 2.25 / 1.79, theta_0 9.04 / 5.73) |

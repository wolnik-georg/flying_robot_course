| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 10.75 | 7.52 | 0 | 28.9 |
| theta_1 | 10.51 | 7.06 | 0 | 25.6 |
| theta_2 | 10.95 | 7.76 | 0 | 25.6 |
| theta_3 | 10.91 | 7.43 | 0 | 25.4 |
| theta_4 | 10.79 | 7.36 | 0 | 0.1 |

Keep-best over the flown sets: theta_1 (10.51 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 49 | 0.14825 | 0.13272 | - |
| 0 | tune theta | 51 | 0 | 0.012088 | 0.012088 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 1 | fit beta | 51 | 50 | 0.13649 | 0.12427 | - |
| 1 | tune theta | 51 | 23 | 0.034627 | 0.020063 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 2 | fit beta | 51 | 50 | 0.14359 | 0.12921 | - |
| 2 | tune theta | 51 | 17 | 0.042901 | 0.042026 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 3 | fit beta | 51 | 50 | 0.12386 | 0.11904 | - |
| 3 | tune theta | 51 | 12 | 0.038165 | 0.031777 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume / rpm delay):

| Fit of round | error % |
|---|---|
| 0 | +41.0 / -46.3 / -29.7 / +70.0 / -49.4 |
| 1 | +9.0 / -52.8 / -47.0 / +101.4 / -49.4 |
| 2 | +0.6 / -63.4 / -47.7 / +148.1 / -49.4 |
| 3 | +2.3 / -74.6 / -56.2 / +157.7 / -49.4 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 18.73 / 11.92 / 0 | 6.10 / 4.21 / 0 | 5.91 / 5.51 / 0 | 5.33 / 4.76 / 0 | 5.82 / 5.21 / 0 | 7.02 / 5.36 / 0 | 4.49 / 4.21 / 0 | 9.02 | 5.78 |
| theta_best (round 1) | 18.73 / 11.92 / 0 | 6.10 / 4.21 / 0 | 5.91 / 5.51 / 0 | 5.33 / 4.76 / 0 | 5.82 / 5.21 / 0 | 7.02 / 5.36 / 0 | 4.49 / 4.21 / 0 | 9.02 | 5.78 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 10.51, limit 11.51) | **yes** (10.95, 10.91, 10.79) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **no** (theta_best 9.02 / 5.78, theta_0 9.02 / 5.78) |

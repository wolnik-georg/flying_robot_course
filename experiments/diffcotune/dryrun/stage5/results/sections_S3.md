| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 10.75 | 7.52 | 0 | 28.4 |
| theta_1 | 10.80 | 7.56 | 0 | 25.5 |
| theta_2 | 11.04 | 8.06 | 0 | 25.7 |
| theta_3 | 11.08 | 8.31 | 0 | 25.6 |
| theta_4 | 8.90 | 5.92 | 0 | 0.1 |

Keep-best over the flown sets: theta_4 (8.90 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 0 | 0.14825 | 0.14825 | - |
| 0 | tune theta | 51 | 46 | 0.011385 | 0.010657 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 1 | fit beta | 51 | 8 | 0.14681 | 0.14238 | - |
| 1 | tune theta | 51 | 40 | 0.012501 | 0.011723 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 2 | fit beta | 51 | 35 | 0.15098 | 0.13787 | - |
| 2 | tune theta | 51 | 35 | 0.010694 | 0.009836 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 3 | fit beta | 51 | 50 | 0.12688 | 0.098052 | - |
| 3 | tune theta | 51 | 50 | 0.0099584 | 0.0067514 | [0, 21, 0, 0, 27, 0, 0, 0, 0, 0] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume / rpm delay):

| Fit of round | error % |
|---|---|
| 0 | +25.0 / -33.3 / -23.1 / +42.9 / -49.4 |
| 1 | +20.9 / -35.3 / -25.6 / +36.0 / -48.4 |
| 2 | +13.2 / -31.0 / -28.7 / +27.4 / -44.2 |
| 3 | +7.4 / -38.3 / -33.1 / +24.4 / -40.1 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 18.73 / 11.92 / 0 | 6.10 / 4.21 / 0 | 5.91 / 5.51 / 0 | 5.33 / 4.76 / 0 | 5.82 / 5.21 / 0 | 7.02 / 5.36 / 0 | 4.49 / 4.21 / 0 | 9.02 | 5.78 |
| theta_best (round 4) | 16.03 / 9.63 / 0 | 5.05 / 3.61 / 0 | 4.29 / 3.77 / 0 | 4.20 / 3.53 / 0 | 5.57 / 5.09 / 0 | 6.15 / 4.89 / 0 | 3.55 / 3.21 / 0 | 7.39 | 5.09 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 10.80, limit 11.80) | **yes** (11.04, 11.08, 8.90) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **yes** (theta_best 7.39 / 5.09, theta_0 9.02 / 5.78) |

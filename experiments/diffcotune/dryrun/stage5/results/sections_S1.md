| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 10.32 | 7.12 | 0 | 25.1 |
| theta_1 | 8.46 | 4.63 | 0 | 22.4 |
| theta_2 | 6.69 | 3.67 | 0 | 21.2 |
| theta_3 | 4.63 | 1.77 | 0 | 21.3 |
| theta_4 | 4.10 | 2.73 | 0 | 0.1 |

Keep-best over the flown sets: theta_4 (4.10 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 50 | 0.12815 | 0.10145 | - |
| 0 | tune theta | 51 | 50 | 0.0097634 | 0.007269 | [0, 0, 0, 0, 28, 0, 0, 0, 0, 0] |
| 1 | fit beta | 51 | 50 | 0.089725 | 0.082294 | - |
| 1 | tune theta | 51 | 50 | 0.0062938 | 0.0036143 | [0, 0, 8, 0, 8, 0, 0, 0, 0, 0] |
| 2 | fit beta | 51 | 2 | 0.039343 | 0.039148 | - |
| 2 | tune theta | 51 | 50 | 0.0036213 | 0.0021001 | [0, 12, 25, 21, 16, 20, 15, 0, 0, 27] |
| 3 | fit beta | 51 | 39 | 0.009879 | 0.0097582 | - |
| 3 | tune theta | 51 | 31 | 0.0019932 | 0.0010266 | [28, 29, 28, 27, 28, 27, 29, 0, 0, 28] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume / rpm delay):

| Fit of round | error % |
|---|---|
| 0 | +14.6 / -27.4 / -22.9 / +35.0 / -51.7 |
| 1 | +17.2 / -23.7 / -20.3 / +20.2 / -50.0 |
| 2 | +19.2 / -25.1 / -21.8 / +18.0 / -50.9 |
| 3 | +16.8 / -25.8 / -23.0 / +15.4 / -51.8 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 17.93 / 10.92 / 0 | 6.85 / 5.29 / 0 | 5.95 / 5.55 / 0 | 5.41 / 4.84 / 0 | 5.33 / 4.57 / 0 | 7.38 / 5.69 / 0 | 4.49 / 4.21 / 0 | 9.04 | 5.73 |
| theta_best (round 4) | 7.47 / 4.92 / 0 | 2.22 / 1.80 / 0 | 1.25 / 0.58 / 0 | 1.31 / 0.55 / 0 | 1.39 / 1.19 / 0 | 1.55 / 0.94 / 0 | 1.12 / 0.74 / 0 | 3.06 | 1.35 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 8.46, limit 9.46) | **yes** (6.69, 4.63, 4.10) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **yes** (theta_best 3.06 / 1.35, theta_0 9.04 / 5.73) |

| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 15.71 | 14.37 | A1 | 24.7 |
| theta_1 | 12.62 | 11.52 | 0 | 23.5 |
| theta_2 | 8.62 | 4.14 | 0 | 23.5 |
| theta_3 | 8.10 | 4.08 | 0 | 23.5 |
| theta_4 | 7.96 | 4.19 | 0 | 0.1 |

Keep-best over the flown sets: theta_4 (7.96 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 3 | 0.18423 | 0.18219 | - |
| 0 | tune theta | 51 | 48 | 0.010084 | 0.0073053 | [0, 0, 0, 0, 7, 0, 0, 0, 0, 0] |
| 1 | fit beta | 51 | 34 | 0.12855 | 0.12453 | - |
| 1 | tune theta | 51 | 27 | 0.042579 | 0.011502 | [0, 0, 0, 0, 1, 0, 0, 0, 9, 0] |
| 2 | fit beta | 51 | 4 | 0.16938 | 0.16925 | - |
| 2 | tune theta | 51 | 12 | 0.015048 | 0.013499 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 3 | fit beta | 51 | 6 | 0.16644 | 0.16629 | - |
| 3 | tune theta | 51 | 8 | 0.010623 | 0.010112 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume):

| Fit of round | error % |
|---|---|
| 0 | +22.6 / -31.4 / -21.6 / +46.4 |
| 1 | -10.7 / -37.0 / -22.6 / +39.2 |
| 2 | -12.9 / -39.4 / -20.4 / +43.6 |
| 3 | -9.1 / -39.9 / -15.6 / +42.3 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 29.01 / 26.98 / 5 | 8.40 / 6.35 / 0 | 6.28 / 5.94 / 0 | 5.77 / 5.31 / 0 | 6.81 / 5.80 / 0 | 8.71 / 7.37 / 0 | 4.78 / 4.59 / 0 | 12.36 | 6.77 |
| theta_best (round 4) | 14.31 / 7.09 / 0 | 5.12 / 3.13 / 0 | 3.37 / 2.50 / 0 | 3.35 / 2.22 / 0 | 5.81 / 3.31 / 0 | 6.95 / 4.31 / 0 | 2.45 / 1.95 / 0 | 6.54 | 5.07 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 12.62, limit 13.62) | **yes** (8.62, 8.10, 7.96) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **yes** (theta_best 6.54 / 5.07, theta_0 12.36 / 6.77) |

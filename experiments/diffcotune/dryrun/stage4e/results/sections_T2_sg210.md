#### Real cost per round (3-D RMS cm, mean of the 4 tuning flights; reference = B1 of Stage 3e, same flight seeds)

| Gain set | this run | B1 | failed flights this run (tilt or floor) | B1 failed | minutes (round) |
|---|---|---|---|---|---|
| theta_0 | 19.99 | 19.99 | A1 | A1 | 19.2 |
| theta_1 | 9.54 | 9.71 | 0 | 0 | 18.2 |
| theta_2 | 7.46 | 8.30 | 0 | 0 | 18.1 |
| theta_3 | 25.72 | 8.16 | A1 | 0 | 18.1 |
| theta_4 | 91.81 | 8.13 | A1, A8, CIRCLE, FIG8 | 0 | 0.1 |

Keep-best over the flown sets: theta_2 (7.46 cm); B1 over rounds 0-4: theta_4 (8.13 cm).

#### Inner loops (epochs = cost evaluations)

| Round | loop | epochs | stop reason / best epoch | J at start | J at end (returned iterate) | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | best epoch 50 of 50 (no stop) | 2.9895 | 2.9463 | - |
| 0 | tune theta | 51 | best epoch 50 of 50 (no stop) | 0.05021 | 0.014209 | [0, 20, 20, 0, 27, 0, 0, 8, 23, 17] |
| 1 | fit beta | 51 | best epoch 43 of 50 (no stop) | 1.4513 | 1.4485 | - |
| 1 | tune theta | 51 | best epoch 50 of 50 (no stop) | 0.011302 | 0.005935 | [0, 28, 0, 20, 23, 19, 4, 0, 12, 0] |
| 2 | fit beta | 51 | best epoch 50 of 50 (no stop) | 1.9211 | 1.9184 | - |
| 2 | tune theta | 51 | best epoch 50 of 50 (no stop) | 0.0055538 | 0.0017983 | [0, 0, 11, 0, 25, 25, 0, 11, 0, 0] |
| 3 | fit beta | 51 | best epoch 38 of 50 (no stop) | 7.9066 | 7.8405 | - |
| 3 | tune theta | 51 | best epoch 50 of 50 (no stop) | 0.059015 | 0.026653 | [0, 0, 0, 14, 0, 14, 26, 0, 28, 0] |

#### beta error after each fit (% against the P1 truth: thrust gain / motor lag / downwash / plume)

| Fit of round | this run | B1 |
|---|---|---|
| 0 | -5.4 / -43.5 / -1.5 / +32.4 | -1.6 / -36.0 / +6.1 / +23.3 |
| 1 | -0.9 / -43.1 / -11.4 / +30.6 | +12.0 / -21.5 / -9.3 / +39.0 |
| 2 | +12.7 / -39.3 / -4.4 / +33.9 | +5.1 / -3.0 / -12.7 / +19.5 |
| 3 | -21.2 / -56.0 / +36.0 / +38.3 | +3.7 / +5.0 / -12.8 / +9.0 |

#### Final evaluation (evaluation seeds 2000-2004; 3-D RMS cm / z RMS cm / failed flights of 5)

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 37.20 / 35.73 / 5 | 11.44 / 9.47 / 0 | 6.63 / 6.32 / 0 | 6.16 / 5.73 / 0 | 8.80 / 7.14 / 0 | 13.05 / 11.98 / 0 | 5.21 / 5.03 / 0 | 15.36 | 9.02 |
| theta_best of this run (round 2) | 13.08 / 5.95 / 0 | 4.96 / 3.61 / 0 | 2.86 / 2.09 / 0 | 2.89 / 1.83 / 0 | 8.66 / 6.74 / 0 | 9.74 / 8.62 / 0 | 2.72 / 2.39 / 0 | 5.95 | 7.04 |
| B1 theta_best over rounds 0-4 (theta_4) | 13.59 / 7.66 / 0 | 6.08 / 4.64 / 0 | 2.96 / 2.06 / 0 | 2.99 / 1.72 / 0 | 9.31 / 7.59 / 0 | 10.69 / 9.69 / 0 | 2.84 / 2.46 / 0 | 6.41 | 7.61 |

#### Pass criteria 1-4 (against B1, rounds 0-4)

| Criterion | result |
|---|---|
| 1. rounds 2-4 <= round 1 + 1 cm (round 1 = 9.54, limit 10.54) | **no** (7.46, 25.72, 91.81) |
| 2. no flown set after theta_0 fails (tilt or floor) a scenario that theta_0 or theta_1 passed | **no** (violations: theta_3 A1, theta_4 A1, theta_4 A8, theta_4 CIRCLE, theta_4 FIG8) |
| 3. theta_best beats theta_0 on the tuning and the test scenarios (5 seeds) | **yes** (theta_best 5.95 / 7.04, theta_0 15.36 / 9.02, tuning / test) |
| 4. not worse than the B1 reference by more than 1 cm on the tuning mean and the test mean | **yes** (difference -0.46 / -0.57 cm, tuning / test) |

Run passes 1-4: **no**.

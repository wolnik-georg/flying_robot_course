#### Real cost per round (3-D RMS cm, mean of the 4 tuning flights; reference = B1 of Stage 3e, same flight seeds)

| Gain set | this run | B1 | failed flights this run (tilt or floor) | B1 failed | minutes (round) |
|---|---|---|---|---|---|
| theta_0 | 19.99 | 19.99 | A1 | A1 | 3.4 |
| theta_1 | 17.45 | 9.71 | A1 | 0 | 1.0 |
| theta_2 | 17.89 | 8.30 | A1 | 0 | 0.8 |
| theta_3 | 16.09 | 8.16 | A1 | 0 | 0.8 |
| theta_4 | 16.69 | 8.13 | A1 | 0 | 0.1 |

Keep-best over the flown sets: theta_3 (16.09 cm); B1 over rounds 0-4: theta_4 (8.13 cm).

#### Inner loops (epochs = cost evaluations)

| Round | loop | epochs | stop reason / best epoch | J at start | J at end (returned iterate) | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 7 | converged |dJ| < 1e-3 | 0.1706 | 0.16291 | - |
| 0 | tune theta | 5 | converged |dJ| < 1e-3 | 0.016857 | 0.012209 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 1 | fit beta | 2 | converged |dJ| < 1e-3 | 0.15899 | 0.158 | - |
| 1 | tune theta | 3 | converged |dJ| < 1e-3 | 0.013072 | 0.011803 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 2 | fit beta | 2 | converged |dJ| < 1e-3 | 0.14493 | 0.144 | - |
| 2 | tune theta | 2 | converged |dJ| < 1e-3 | 0.013489 | 0.01261 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 3 | fit beta | 2 | converged |dJ| < 1e-3 | 0.1551 | 0.15448 | - |
| 3 | tune theta | 2 | converged |dJ| < 1e-3 | 0.012714 | 0.012682 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |

#### beta error after each fit (% against the P1 truth: thrust gain / motor lag / downwash / plume)

| Fit of round | this run | B1 |
|---|---|---|
| 0 | +17.7 / -36.8 / -18.3 / +35.2 | -1.6 / -36.0 / +6.1 / +23.3 |
| 1 | +16.5 / -36.2 / -17.5 / +36.6 | +12.0 / -21.5 / -9.3 / +39.0 |
| 2 | +15.4 / -35.5 / -16.7 / +38.0 | +5.1 / -3.0 / -12.7 / +19.5 |
| 3 | +14.2 / -34.9 / -15.8 / +39.4 | +3.7 / +5.0 / -12.8 / +9.0 |

#### Final evaluation (evaluation seeds 2000-2004; 3-D RMS cm / z RMS cm / failed flights of 5)

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 37.20 / 35.73 / 5 | 11.44 / 9.47 / 0 | 6.63 / 6.32 / 0 | 6.16 / 5.73 / 0 | 8.80 / 7.14 / 0 | 13.05 / 11.98 / 0 | 5.21 / 5.03 / 0 | 15.36 | 9.02 |
| theta_best of this run (round 3) | 30.03 / 27.85 / 5 | 9.95 / 7.95 / 0 | 6.20 / 5.86 / 0 | 5.80 / 5.33 / 0 | 8.82 / 7.03 / 0 | 11.94 / 10.19 / 0 | 4.90 / 4.71 / 0 | 13.00 | 8.56 |
| B1 theta_best over rounds 0-4 (theta_4) | 13.59 / 7.66 / 0 | 6.08 / 4.64 / 0 | 2.96 / 2.06 / 0 | 2.99 / 1.72 / 0 | 9.31 / 7.59 / 0 | 10.69 / 9.69 / 0 | 2.84 / 2.46 / 0 | 6.41 | 7.61 |

#### Pass criteria 1-4 (against B1, rounds 0-4)

| Criterion | result |
|---|---|
| 1. rounds 2-4 <= round 1 + 1 cm (round 1 = 17.45, limit 18.45) | **yes** (17.89, 16.09, 16.69) |
| 2. no flown set after theta_0 fails (tilt or floor) a scenario that theta_0 or theta_1 passed | **yes** (none) |
| 3. theta_best beats theta_0 on the tuning and the test scenarios (5 seeds) | **yes** (theta_best 13.00 / 8.56, theta_0 15.36 / 9.02, tuning / test) |
| 4. not worse than the B1 reference by more than 1 cm on the tuning mean and the test mean | **no** (difference +6.59 / +0.94 cm, tuning / test) |

Run passes 1-4: **no**.

| Gain set | real 3-D RMS (cm, mean of 5 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 18.24 | 17.25 | A1 | 9.7 |
| theta_1 | 17.24 | 16.30 | A1 | 10.2 |
| theta_2 | 22.09 | 21.67 | A1 | 3.3 |
| theta_3 | 24.33 | 23.86 | A1 | 2.4 |
| theta_4 | 24.63 | 24.18 | A1 | 3.5 |
| theta_5 | 30.81 | 30.31 | A1, A8, A4 | 0.1 |

Keep-best over the flown sets: theta_1 (17.24 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 18 | last (converged |dJ| < 1e-3) | 2.7018 | 2.5614 | - |
| 0 | tune theta | 14 | last (converged |dJ| < 1e-3) | 0.2989 | 0.21523 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 1 | fit beta | 2 | last (rose dJ > 1e-3) | 2.0677 | 2.0755 | - |
| 1 | tune theta | 41 | last (converged |dJ| < 1e-3) | 0.20618 | 0.12057 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 2 | fit beta | 8 | last (converged |dJ| < 1e-3) | 3.8236 | 3.8117 | - |
| 2 | tune theta | 7 | last (converged |dJ| < 1e-3) | 0.10796 | 0.097997 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 3 | fit beta | 6 | last (converged |dJ| < 1e-3) | 6.2016 | 6.1895 | - |
| 3 | tune theta | 4 | last (converged |dJ| < 1e-3) | 0.090447 | 0.085897 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 4 | fit beta | 9 | last (converged |dJ| < 1e-3) | 5.9694 | 5.9541 | - |
| 4 | tune theta | 5 | last (converged |dJ| < 1e-3) | 0.077681 | 0.073092 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume):

| Fit of round | error % |
|---|---|
| 0 | +38.2 / -40.3 / -11.8 / +29.0 |
| 1 | +36.9 / -39.7 / -10.9 / +27.7 |
| 2 | +28.5 / -42.6 / -4.5 / +19.8 |
| 3 | +22.7 / -44.3 / -7.2 / +14.2 |
| 4 | +15.5 / -45.3 / +0.4 / +5.6 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 (tuning, in-sample) | A4 cell 2 (held out) | OVAL (frsim only) | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 37.20 / 35.73 / 5 | 11.44 / 9.47 / 0 | 6.63 / 6.32 / 0 | 6.16 / 5.73 / 0 | 8.80 / 7.14 / 0 | 13.05 / 11.98 / 0 | 5.21 / 5.03 / 0 | 14.04 | 13.05 |
| theta_best (round 1) | 34.38 / 32.85 / 5 | 10.04 / 9.06 / 0 | 8.38 / 8.11 / 0 | 7.71 / 7.34 / 0 | 10.06 / 8.70 / 0 | 13.75 / 12.34 / 0 | 6.54 / 6.37 / 0 | 14.11 | 13.75 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 17.24, limit 18.24) | **no** (22.09, 24.33, 24.63, 30.81) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **no** (theta_5 A8, theta_5 A4) |
| c. theta_best beats theta_0 on tuning and test means | **no** (theta_best 14.11 / 13.75, theta_0 14.04 / 13.05) |

Per round: real flights of the round (3-D RMS cm per scenario), crashed (tilt abort) / floor flights, repeats, largest relative gain change to the next flown set, epochs used by the stop rule (fit / tune):

| Round | A1 | A8 | CIRCLE | FIG8 | A4 | crashed | floor | attempts (5 = first try each) | largest gain change to next set | epochs fit / tune | stop reasons |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 37.13 | 11.81 | 6.51 | 6.15 | 8.03 | 0 | 1 | 5 | 13.8 % | 18 / 14 | converged |dJ| < 1e-3 / converged |dJ| < 1e-3 |
| 1 | 34.08 | 9.98 | 8.26 | 7.78 | 9.82 | 0 | 1 | 5 | 45.7 % | 2 / 41 | rose dJ > 1e-3 / converged |dJ| < 1e-3 |
| 2 | 42.18 | 16.63 | 9.40 | 8.78 | 14.77 | 0 | 1 | 5 | 6.2 % | 8 / 7 | converged |dJ| < 1e-3 / converged |dJ| < 1e-3 |
| 3 | 45.41 | 17.20 | 9.56 | 8.84 | 20.79 | 0 | 1 | 5 | 3.0 % | 6 / 4 | converged |dJ| < 1e-3 / converged |dJ| < 1e-3 |
| 4 | 44.49 | 18.50 | 9.32 | 8.65 | 23.46 | 0 | 1 | 5 | 4.1 % | 9 / 5 | converged |dJ| < 1e-3 / converged |dJ| < 1e-3 |
| 5 | 48.76 | 31.21 | 14.54 | 10.73 | 32.67 | 0 | 3 | 5 | - | - / - | - / - |

Summary: tuning mean theta_0 14.04 -> theta_best 14.11 cm; held-out A4 cell 2 13.05 -> 13.75 cm; in-sample A4 cell 1 8.80 -> 10.06 cm; keep-best round 1; halt: None.

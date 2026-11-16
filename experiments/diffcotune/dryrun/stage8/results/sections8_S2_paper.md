| Gain set | real 3-D RMS (cm, mean of 5 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 14.38 | 13.13 | A1 | 7.2 |
| theta_1 | 14.52 | 13.26 | A1 | 4.8 |
| theta_2 | 15.03 | 13.93 | A1 | 8.6 |
| theta_3 | 17.54 | 16.64 | A1 | 2.1 |
| theta_4 | 20.54 | 19.67 | A1 | 2.2 |
| theta_5 | 19.11 | 18.37 | A1 | 0.1 |

Keep-best over the flown sets: theta_0 (14.38 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 5 | last (rose dJ > 1e-3) | 2.8776 | 2.852 | - |
| 0 | tune theta | 3 | last (rose dJ > 1e-3) | 0.49399 | 0.45518 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 1 | fit beta | 6 | last (rose dJ > 1e-3) | 2.5932 | 2.5514 | - |
| 1 | tune theta | 8 | last (rose dJ > 1e-3) | 0.34553 | 0.28345 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 2 | fit beta | 3 | last (rose dJ > 1e-3) | 2.5967 | 2.5951 | - |
| 2 | tune theta | 33 | last (converged |dJ| < 1e-3) | 0.30503 | 0.16086 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 3 | fit beta | 2 | last (rose dJ > 1e-3) | 2.7348 | 2.7809 | - |
| 3 | tune theta | 7 | last (converged |dJ| < 1e-3) | 0.16355 | 0.15549 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 4 | fit beta | 6 | last (converged |dJ| < 1e-3) | 3.5363 | 3.5227 | - |
| 4 | tune theta | 2 | last (rose dJ > 1e-3) | 0.1375 | 0.16168 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume):

| Fit of round | error % |
|---|---|
| 0 | +29.9 / -35.5 / -24.0 / +37.3 |
| 1 | +35.8 / -32.4 / -20.3 / +30.8 |
| 2 | +33.5 / -31.9 / -18.8 / +30.3 |
| 3 | +32.2 / -32.6 / -18.0 / +31.7 |
| 4 | +29.8 / -35.0 / -16.8 / +25.7 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 (tuning, in-sample) | A4 cell 2 (held out) | OVAL (frsim only) | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 29.01 / 26.98 / 5 | 8.40 / 6.35 / 0 | 6.28 / 5.94 / 0 | 5.77 / 5.31 / 0 | 6.81 / 5.80 / 0 | 8.71 / 7.37 / 0 | 4.78 / 4.59 / 0 | 11.25 | 8.71 |
| theta_best (round 0) | 29.01 / 26.98 / 5 | 8.40 / 6.35 / 0 | 6.28 / 5.94 / 0 | 5.77 / 5.31 / 0 | 6.81 / 5.80 / 0 | 8.71 / 7.37 / 0 | 4.78 / 4.59 / 0 | 11.25 | 8.71 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 14.52, limit 15.52) | **no** (15.03, 17.54, 20.54, 19.11) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **no** (theta_best 11.25 / 8.71, theta_0 11.25 / 8.71) |

Per round: real flights of the round (3-D RMS cm per scenario), crashed (tilt abort) / floor flights, repeats, largest relative gain change to the next flown set, epochs used by the stop rule (fit / tune):

| Round | A1 | A8 | CIRCLE | FIG8 | A4 | crashed | floor | attempts (5 = first try each) | largest gain change to next set | epochs fit / tune | stop reasons |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 28.87 | 9.09 | 6.15 | 5.78 | 6.85 | 0 | 1 | 5 | 2.0 % | 5 / 3 | rose dJ > 1e-3 / rose dJ > 1e-3 |
| 1 | 29.35 | 8.27 | 6.30 | 5.91 | 7.12 | 0 | 1 | 5 | 7.3 % | 6 / 8 | rose dJ > 1e-3 / rose dJ > 1e-3 |
| 2 | 29.89 | 9.48 | 7.34 | 6.67 | 6.98 | 0 | 1 | 5 | 34.0 % | 3 / 33 | rose dJ > 1e-3 / converged |dJ| < 1e-3 |
| 3 | 31.34 | 13.50 | 10.26 | 9.54 | 13.33 | 0 | 1 | 5 | 5.9 % | 2 / 7 | rose dJ > 1e-3 / converged |dJ| < 1e-3 |
| 4 | 38.26 | 15.45 | 10.70 | 9.86 | 13.97 | 0 | 1 | 5 | 1.0 % | 6 / 2 | converged |dJ| < 1e-3 / rose dJ > 1e-3 |
| 5 | 34.30 | 13.64 | 10.96 | 9.99 | 15.61 | 0 | 1 | 5 | - | - / - | - / - |

Summary: tuning mean theta_0 11.25 -> theta_best 11.25 cm; held-out A4 cell 2 8.71 -> 8.71 cm; in-sample A4 cell 1 6.81 -> 6.81 cm; keep-best round 0; halt: None.

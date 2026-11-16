## B1 pilot (P1, 3 learning rates, rounds 0 and 1 update, the real cost after round 2 = flights of theta_2)

| learning rate | real 3-D RMS cm round 0 / 1 / 2 | fit epochs (stop) round 0, 1 | tune epochs (stop) round 0, 1 |
|---|---|---|---|
| 0.01 | 19.80 / 11.93 / 30.82 | 5 (converged); 2 (converged) | 50 (max epochs); 38 (converged) |
| 0.03 | 19.80 / 19.81 / 44.25 | 4 (converged); 4 (converged) | 22 (rose); 12 (rose) |
| 0.1 | 19.80 / 8.52 / 67.21 | 3 (rose); 3 (converged) | 5 (rose); 3 (rose) |

Lowest real cost after round 2: learning rate 0.01 (30.82 cm).


## P1 (thrust gain x0.8, lag 45 ms, downwash x1.3, plume x0.7)

**1. beta** after round 5 (fit of round 5): thrust-law gain 1.0143 (true 0.80, error +26.8 %), motor lag 0.9313 (true 1.50, error -37.9 %), downwash scale 0.9635 (true 1.30, error -25.9 %), plume scale 0.9249 (true 0.70, error +32.1 %). Plot: `beta_P1.png`.

**2. real cost per round** (3-D RMS cm, 1 flight per scenario): (d) 19.80 / 11.93 / 30.82 / 27.28 / 41.95 / 72.58; (b) 19.80 / 9.40 / 13.82 / 27.99 / 26.23 / 23.01. Plot: `cost_P1.png`.

**3. final evaluation** (theta_best of each method on the real drone, 5 noise seeds per cell; mean over the non-crashed seeds of 3-D RMS / z RMS in cm, crashed seeds in brackets):

| cell | role | (a) default | (b) tune only | (c) sysId then tune | (d) DiffCoTune |
|---|---|---|---|---|---|
| A1 | tuning | 37.38 / 35.91 | 17.28 / 13.46 | 50.02 / 50.00 | 21.59 / 18.96 |
| A8 | tuning | 10.99 / 9.32 | 6.40 / 5.56 | 43.81 / 41.64 | 6.66 / 5.91 |
| CIRCLE | tuning | 6.64 / 6.32 | 2.96 / 1.59 | 87.16 / 84.43 | 3.14 / 1.88 |
| FIG8 | tuning | 6.14 / 5.73 | 2.93 / 1.17 | 73.99 / 73.15 | 2.96 / 1.17 |
| A4 dz0.4 off0.125 | test | 8.78 / 7.27 | 12.35 / 11.01 | 44.07 / 40.80 | 13.70 / 12.28 |
| A4 dz0.4 off0.075 | test | 13.15 / 11.91 | 14.38 / 13.30 | 47.10 / 42.51 | 15.49 / 14.32 |
| OVAL | test | 5.19 / 5.02 | 3.08 / 2.61 | 97.43 / 87.92 | 3.17 / 2.69 |
| **mean tuning cells** | | 15.29 | 7.39 | 63.75 | 8.59 |
| **mean test cells** | | 9.04 | 9.94 | 62.87 | 10.78 |

**4. theta**: theta_best = round 1 of the loop; factors to the defaults: Pxy 0.77, Pz 0.67, Dxy 0.86, Dz 1.21, Iz 1.39, KRxy 1.38, KRz 1.00, Kwxy 1.31, Kwz 1.00, KIxy 0.74. Plot: `theta_P1.png`.

**5. inner loops** (epochs used, stop reason; Jn = cost / cost at the start of the loop, last epoch):

| round | fit epochs | fit stop | fit Jn | tune epochs | tune stop | tune Jn | seconds fit + tune |
|---|---|---|---|---|---|---|---|
| 0 | 5 | converged | 0.993 | 50 | max epochs | 0.198 | 651 |
| 1 | 2 | converged | 1.000 | 38 | converged | 0.475 | 415 |
| 2 | 2 | rose | 1.010 | 50 | max epochs | 0.446 | 557 |
| 3 | 2 | rose | 1.008 | 50 | max epochs | 0.588 | 533 |
| 4 | 3 | converged | 0.998 | 11 | converged | 0.887 | 163 |
| 5 | 2 | rose | 1.003 | - | - | - | 20 |

**6. Adam on motor lag and plume scale**: epoch-by-epoch trace in `adam_weak_P1.png`; autograd against central finite difference (h = 1e-6) at the checked epochs:

| round | epoch | parameter | autograd | finite difference | same sign | relative difference |
|---|---|---|---|---|---|---|
| 0 | 0 | motor lag | +3.3811e-01 | +3.4021e-01 | yes | 0.6 % |
| 0 | 0 | plume scale | +6.1409e-01 | +6.2915e-01 | yes | 2.4 % |
| 2 | 0 | motor lag | +2.5087e+01 | +2.5086e+01 | yes | 0.0 % |
| 2 | 0 | plume scale | +3.1877e-01 | +3.1836e-01 | yes | 0.1 % |
| 4 | 0 | motor lag | +1.6209e+01 | +1.6208e+01 | yes | 0.0 % |
| 4 | 0 | plume scale | +1.3406e-01 | +1.2066e-01 | yes | 11.1 % |

**7. wall time per round (fit + tune, 4 scenarios in parallel processes)**: 10.9 min, 6.9 min, 9.3 min, 8.9 min, 2.7 min, 0.3 min; mean 6.5 min.


## P2 (thrust gain x0.8, lag 45 ms, downwash x1.3, plume x0.7 + state delay 10 ms)

**1. beta** after round 5 (fit of round 5): thrust-law gain 0.9686 (true 0.80, error +21.1 %), motor lag 0.9718 (true 1.50, error -35.2 %), downwash scale 0.9794 (true 1.30, error -24.7 %), plume scale 0.9341 (true 0.70, error +33.4 %). Plot: `beta_P2.png`.

**2. real cost per round** (3-D RMS cm, 1 flight per scenario): (d) 19.29 / 8.57 / 36.97 / 24.76 / 52.30 / 32.60; (b) 19.29 / 9.24 / 14.56 / 19.84 / 21.37 / 20.94. Plot: `cost_P2.png`.

**3. final evaluation** (theta_best of each method on the real drone, 5 noise seeds per cell; mean over the non-crashed seeds of 3-D RMS / z RMS in cm, crashed seeds in brackets):

| cell | role | (a) default | (b) tune only | (c) sysId then tune | (d) DiffCoTune |
|---|---|---|---|---|---|
| A1 | tuning | 35.91 / 34.10 | 15.58 / 10.49 | 47.39 / 47.23 | 17.13 / 13.09 |
| A8 | tuning | 11.28 / 9.38 | 6.42 / 5.53 | 11.24 / 10.86 | 5.95 / 5.03 |
| CIRCLE | tuning | 6.57 / 6.31 | 2.78 / 1.59 | 1.85 / 1.26 | 2.71 / 1.67 |
| FIG8 | tuning | 6.04 / 5.71 | 2.70 / 1.17 | 1.63 / 0.76 | 2.59 / 1.29 |
| A4 dz0.4 off0.125 | test | 8.90 / 7.41 | 12.19 / 10.85 | 22.44 / 21.50 | 12.22 / 10.38 |
| A4 dz0.4 off0.075 | test | 12.78 / 11.74 | 14.18 / 12.97 | 26.87 / 25.59 | 13.89 / 12.91 |
| OVAL | test | 5.18 / 5.01 | 3.06 / 2.61 | 9.03 / 8.78 | 2.96 / 2.52 |
| **mean tuning cells** | | 14.95 | 6.87 | 15.53 | 7.10 |
| **mean test cells** | | 8.95 | 9.81 | 19.45 | 9.69 |

**4. theta**: theta_best = round 1 of the loop; factors to the defaults: Pxy 0.80, Pz 0.80, Dxy 0.94, Dz 1.27, Iz 1.42, KRxy 1.37, KRz 1.01, Kwxy 1.30, Kwz 0.99, KIxy 0.75. Plot: `theta_P2.png`.

**5. inner loops** (epochs used, stop reason; Jn = cost / cost at the start of the loop, last epoch):

| round | fit epochs | fit stop | fit Jn | tune epochs | tune stop | tune Jn | seconds fit + tune |
|---|---|---|---|---|---|---|---|
| 0 | 5 | converged | 0.994 | 50 | max epochs | 0.158 | 630 |
| 1 | 2 | converged | 1.000 | 43 | rose | 0.441 | 462 |
| 2 | 2 | rose | 1.010 | 3 | rose | 0.846 | 72 |
| 3 | 2 | converged | 1.000 | 20 | rose | 0.748 | 225 |
| 4 | 2 | rose | 1.007 | 3 | rose | 0.842 | 73 |
| 5 | 2 | converged | 1.000 | - | - | - | 20 |

**6. Adam on motor lag and plume scale**: epoch-by-epoch trace in `adam_weak_P2.png`; autograd against central finite difference (h = 1e-6) at the checked epochs:

| round | epoch | parameter | autograd | finite difference | same sign | relative difference |
|---|---|---|---|---|---|---|
| 0 | 0 | motor lag | +2.6999e-01 | +2.6199e-01 | yes | 3.1 % |
| 0 | 0 | plume scale | +5.0780e-01 | +4.9025e-01 | yes | 3.6 % |
| 2 | 0 | motor lag | -3.1447e+01 | -3.1447e+01 | yes | 0.0 % |
| 2 | 0 | plume scale | +3.7695e-01 | +3.7707e-01 | yes | 0.0 % |
| 4 | 0 | motor lag | -2.6831e+01 | -2.6830e+01 | yes | 0.0 % |
| 4 | 0 | plume scale | +5.4927e-01 | +5.5329e-01 | yes | 0.7 % |

**7. wall time per round (fit + tune, 4 scenarios in parallel processes)**: 10.5 min, 7.7 min, 1.2 min, 3.7 min, 1.2 min, 0.3 min; mean 4.1 min.

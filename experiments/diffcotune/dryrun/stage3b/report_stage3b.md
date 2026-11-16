## B1 identifiability (P1, noisy flights of theta0, 2 noise seeds, 50 epochs, best iterate; error = fitted / true - 1)

| states | learning rate | thrust gain | motor lag | downwash | plume | best epoch | parts share start (p / v / q / w, % of the full 4-part cost) | end |
|---|---|---|---|---|---|---|---|---|
| pva | 0.01 | -1.7 % | -34.5 % | +5.7 % | +22.3 % | 50 | 1 / 4 / 0 / 95 | 0 / 4 / 0 / 95 |
| pva | 0.03 | +5.2 % | -26.6 % | +34.5 % | +11.5 % | 39 | 1 / 4 / 0 / 95 | 0 / 4 / 0 / 95 |
| pva | 0.1 | +6.8 % | +16.6 % | +35.1 % | -7.0 % | 50 | 1 / 4 / 0 / 95 | 0 / 5 / 0 / 95 |
| pv | 0.01 | -2.0 % | -34.0 % | +5.8 % | +22.0 % | 50 | 1 / 4 / 0 / 95 | 0 / 4 / 0 / 95 |
| pv | 0.03 | +4.7 % | -22.4 % | +37.7 % | +8.2 % | 44 | 1 / 4 / 0 / 95 | 0 / 4 / 0 / 95 |
| pv | 0.1 | +5.5 % | +10.4 % | +39.1 % | -4.1 % | 47 | 1 / 4 / 0 / 95 | 0 / 4 / 0 / 95 |

Chosen: pva (position + velocity + attitude).

## B2 pilot (P1, 3 learning rates, rounds 0 and 1 update, the real cost after round 2 = flights of theta_2)

| learning rate | real 3-D RMS cm round 0 / 1 / 2 | fit epochs (stop) round 0, 1 | tune epochs (stop) round 0, 1 |
|---|---|---|---|
| 0.01 | 19.80 / 7.98 / 16.93 | 51 (max epochs); 51 (max epochs) | 51 (max epochs); 51 (max epochs) |
| 0.03 | 19.80 / 43.81 / 56.25 | 51 (max epochs); 51 (max epochs) | 51 (max epochs); 51 (max epochs) |
| 0.1 | 19.80 / 21.45 / 23.48 | 51 (max epochs); 51 (max epochs) | 51 (max epochs); 51 (max epochs) |

Lowest real cost after round 2: learning rate 0.01 (16.93 cm).


## P1 (thrust gain x0.8, lag 45 ms, downwash x1.3, plume x0.7)

**1. beta** after round 5 (fit of round 5): thrust-law gain 0.4235 (true 0.80, error -47.1 %), motor lag 0.3162 (true 1.50, error -78.9 %), downwash scale 0.8235 (true 1.30, error -36.7 %), plume scale 0.1507 (true 0.70, error -78.5 %). Plot: `beta_P1.png`.

**2. real cost per round** (3-D RMS cm, 1 flight per scenario): (d) 19.80 / 7.98 / 16.93 / 69.77 / 95.32 / 66.73; (b) 19.80 / 9.40 / 19.74 / 15.54 / 6.81 / 18.13. Plot: `cost_P1.png`.

**3. final evaluation** (theta_best of each method on the real drone, 5 noise seeds per cell; mean over all 5 seeds (crashed flights included, their RMS up to the crash) of 3-D RMS / z RMS in cm, crashed seeds in brackets):

| cell | role | (a) default | (b) tune only | (c) sysId then tune | (d) DiffCoTune |
|---|---|---|---|---|---|
| A1 | tuning | 37.38 / 35.91 | 28.77 / 24.43 (4 crashed) | 19.18 / 9.46 | 14.39 / 8.93 |
| A8 | tuning | 10.99 / 9.32 | 3.54 / 2.95 | 1.88 / 1.20 | 5.73 / 4.70 |
| CIRCLE | tuning | 6.64 / 6.32 | 1.32 / 0.55 | 0.79 / 0.24 | 4.18 / 3.79 |
| FIG8 | tuning | 6.14 / 5.73 | 1.18 / 0.30 | 0.73 / 0.12 | 4.00 / 3.52 |
| A4 dz0.4 off0.125 | test | 8.78 / 7.27 | 21.85 / 17.96 | 11.02 / 3.46 | 7.14 / 5.50 |
| A4 dz0.4 off0.075 | test | 13.15 / 11.91 | 22.10 / 17.94 | 13.96 / 7.00 | 10.81 / 8.70 |
| OVAL | test | 5.19 / 5.02 | 1.39 / 0.77 | 0.76 / 0.32 | 3.39 / 3.17 |
| **mean tuning cells** | | 15.29 | 8.70 (4 crashed) | 5.65 | 7.08 |
| **mean test cells** | | 9.04 | 15.11 | 8.58 | 7.11 |

**4. theta**: theta_best = round 1 of the loop; factors to the defaults: Pxy 1.06, Pz 1.47, Dxy 1.19, Dz 1.42, Iz 1.55, KRxy 1.37, KRz 1.07, Kwxy 1.35, Kwz 0.92, KIxy 0.74. Plot: `theta_P1.png`.

**5. inner loops** (epochs used, stop reason; Jn = cost / cost at the start of the loop, last epoch):

| round | fit epochs | fit stop | fit Jn | tune epochs | tune stop | tune Jn | best epoch fit / tune | seconds fit + tune |
|---|---|---|---|---|---|---|---|---|
| 0 | 51 | max epochs | 0.895 | 51 | max epochs | 0.201 | 50 / 50 | 1143 |
| 1 | 51 | max epochs | 0.991 | 51 | max epochs | 0.191 | 50 / 50 | 1026 |
| 2 | 51 | max epochs | 0.912 | 51 | max epochs | 0.063 | 50 / 50 | 1088 |
| 3 | 51 | max epochs | 0.904 | 51 | max epochs | 0.992 | 50 / 50 | 1025 |
| 4 | 51 | max epochs | 0.996 | 51 | max epochs | 0.994 | 50 / 50 | 1085 |
| 5 | 51 | max epochs | 0.993 | - | - | - | 50 / - | 512 |

**6. Adam on motor lag and plume scale**: epoch-by-epoch trace in `adam_weak_P1.png`; autograd against central finite difference (h = 1e-6) at the checked epochs:

| round | epoch | parameter | autograd | finite difference | same sign | relative difference |
|---|---|---|---|---|---|---|
| 0 | 0 | motor lag | +2.0855e-02 | +2.0934e-02 | yes | 0.4 % |
| 0 | 0 | plume scale | +5.1616e-02 | +5.1088e-02 | yes | 1.0 % |
| 0 | 10 | motor lag | +1.6536e-03 | +1.6538e-03 | yes | 0.0 % |
| 0 | 10 | plume scale | +6.0867e-03 | +6.1627e-03 | yes | 1.2 % |
| 0 | 30 | motor lag | -1.5878e-03 | -1.5880e-03 | yes | 0.0 % |
| 0 | 30 | plume scale | -2.3905e-03 | -2.3852e-03 | yes | 0.2 % |
| 2 | 0 | motor lag | +8.5074e-03 | +8.5108e-03 | yes | 0.0 % |
| 2 | 0 | plume scale | -2.5118e-04 | -2.5116e-04 | yes | 0.0 % |
| 2 | 10 | motor lag | +8.3073e-04 | +8.3073e-04 | yes | 0.0 % |
| 2 | 10 | plume scale | +4.0148e-04 | +4.0148e-04 | yes | 0.0 % |
| 2 | 30 | motor lag | +1.1661e-03 | +1.1661e-03 | yes | 0.0 % |
| 2 | 30 | plume scale | +1.5067e-03 | +1.5067e-03 | yes | 0.0 % |
| 4 | 0 | motor lag | +1.5427e-03 | +1.5427e-03 | yes | 0.0 % |
| 4 | 0 | plume scale | +3.9966e-03 | +3.9966e-03 | yes | 0.0 % |
| 4 | 10 | motor lag | +1.2826e-03 | +1.2826e-03 | yes | 0.0 % |
| 4 | 10 | plume scale | +4.0725e-03 | +4.0725e-03 | yes | 0.0 % |
| 4 | 30 | motor lag | +8.2138e-04 | +8.2138e-04 | yes | 0.0 % |
| 4 | 30 | plume scale | +3.4361e-03 | +3.4361e-03 | yes | 0.0 % |

**Stage 3 against Stage 3b**

| item | Stage 3 | Stage 3b |
|---|---|---|
| beta error after round 5 (thrust / lag / downwash / plume, %) | +26.8 / -37.9 / -25.9 / +32.1 | -47.1 / -78.9 / -36.7 / -78.5 |
| (d) real 3-D RMS per round (cm) | 19.8 / 11.9 / 30.8 / 27.3 / 41.9 / 72.6 | 19.8 / 8.0 / 16.9 / 69.8 / 95.3 / 66.7 |
| final tuning cells (a) (cm) | 15.29 | 15.29 |
| final tuning cells (b) (cm) | 7.39 | 8.70 |
| final tuning cells (c) (cm) | 63.75 | 5.65 |
| final tuning cells (d) (cm) | 8.59 | 7.08 |
| final test cells (a) (cm) | 9.04 | 9.04 |
| final test cells (b) (cm) | 9.94 | 15.11 |
| final test cells (c) (cm) | 62.87 | 8.58 |
| final test cells (d) (cm) | 10.78 | 7.11 |

Real cost rises after round 1 (Stage 3b, (d)): yes (round 1 = 7.98 cm; later rounds 16.93 / 69.77 / 95.32 / 66.73).

**7. wall time per round (fit + tune, 4 scenarios in parallel processes)**: 19.1 min, 17.1 min, 18.1 min, 17.1 min, 18.1 min, 8.5 min; mean 16.3 min.


## Why (c) sysId then tune was bad in Stage 3 (63.75 cm)

Numbers from `c_diag.py` (table below). Reading:

1. The Stage 3 stop rule ended every beta fit of (c) after 2–4 epochs (epochs 4 / 3 / 2 / 2 / 2 / 2), so beta stayed at 0.96 / 0.91 / 1.03 / 0.93 of the calibration (true 0.80 / 1.50 / 1.30 / 0.70). The fit had not been done.
2. The six tune loops (174 epochs in total, no flight in between and no keep-best, unlike the loop of (b) and (d)) then drove the gains far from the defaults in this unfitted model (model task cost 0.0074 → 0.00014): Dxy x3.1, Dz x3.4, Iz x3.95, KIxy x0.25.
3. On the real drone these gains lose the altitude: z RMS 62.3 cm of the 63.75 cm 3-D RMS (the error is the height, not the lateral error). Nothing crashed by the tilt rule.
4. With the Stage 3b stop rule (6 x 51 epochs) the fit moves beta (thrust gain +5 %, lag +15 %, downwash +45 %, plume -7 %) and the same procedure gives 5.65 cm. So the Stage 3 failure was the stop rule, as decided on 12 Nov. Even so, the Stage 3b gains of (c) are extreme (Iz x11, Dz x8.8, Dxy x4.8, KIxy x0.08); they work in the 5 evaluation seeds here, but their noise sensitivity was not checked (open); Part B check 5 shows that extreme gain sets tuned in the noise-free model can fail with noise.

### (c) sysId then tune, P1: Stage 3 against Stage 3b

| item | Stage 3 | Stage 3b |
|---|---|---|
| fitted beta factors (thrust gain / lag / downwash / plume; true 0.80 / 1.50 / 1.30 / 0.70) | 0.96/0.91/1.03/0.93 | 0.84/1.73/1.88/0.65 |
| beta error vs truth (%) | +20/-39/-21/+33 | +5/+15/+45/-7 |
| epochs of the 6 beta fit loops | 4/3/2/2/2/2 | 51/51/51/51/51/51 |
| epochs of the 6 theta tune loops | 50/20/50/2/50/2 | 51/51/51/51/51/51 |
| model task cost, first -> last tune loop | 0.0074 -> 0.00014 | 0.0378 -> 0.00009 |
| final gains (factors to the default) | Pxy 1.66 Pz 1.99 Dxy 3.13 Dz 3.36 Iz 3.95 KRxy 1.92 KRz 1.02 Kwxy 1.44 Kwz 0.98 KIxy 0.25 | Pxy 0.58 Pz 5.39 Dxy 4.83 Dz 8.76 Iz 11.30 KRxy 1.00 KRz 1.10 Kwxy 1.53 Kwz 0.90 KIxy 0.08 |
| real 3-D RMS on the tuning cells (cm) | 63.75 | 5.65 |
| real z RMS on the tuning cells (cm) | 62.30 | 2.75 |
| crashed tuning flights | 0 | 0 |

## Pass criteria, P1 (Stage 3b)

| Criterion | Result | Numbers |
|---|---|---|
| beta moves toward the truth, thrust-law gain within 5 % after round 5 | **no** | thrust gain -47.1 %, lag -78.9 %, downwash -36.7 %, plume -78.5 % after round 5. After round 1 the thrust gain was -1.8 % and after round 2 +4.9 %, and downwash +5 % after round 1; from round 3 on beta walks away (log errors of round 5: -0.52 / -1.14 / 0.00 / -1.09) |
| theta_best of (d) beats (a) default on the tuning and on the test scenarios (mean of 5 seeds) | **yes** | tuning 7.08 against 15.29 cm, test 7.11 against 9.04 cm; theta_best = round 1 |
| (d) not worse than (b) and (c) | **no** | tuning cells: (c) 5.65 is better than (d) 7.08; (b) is 8.70 with its 4 crashed A1 flights counted (worse than (d)) and 3.45 over its 16 flights that did not crash (20 tuning-cell flights, 4 crashed). Test cells: (d) 7.11 is better than (b) 15.11 and (c) 8.58. Reported as it is, the method was not changed |
| no crash of theta_best of (d) | **yes** | 0 of 35 evaluation flights crashed by the tilt rule. Note: (b) theta_best (round 4) crashed 4 of 5 A1 seeds. The tilt rule does not see a drone that stays on the floor (the gain sets of rounds 3–5, see Part B check 5); theta_best of (d) is round 1 and flies |
| wall time about 20 min per round or less | **yes** | mean 16.3 min, longest round 19.1 min (round 0), round 5 8.5 min (fit only) |

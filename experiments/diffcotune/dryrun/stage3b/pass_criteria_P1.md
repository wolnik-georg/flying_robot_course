
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

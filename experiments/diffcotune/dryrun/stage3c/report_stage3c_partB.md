# Stage 3c, Part B: cause of the drift of (d) on P1 (desk checks)

Setup of every check: P1, J_sysId = position + velocity + attitude, 50 epochs, best iterate, learning rate 0.01 (Stage 3b). The six gain sets and flights of (d) are re-flown with the loop seeds (reproduced exactly, real cost equal to the loop record).

## Check 1: oracle tune (true beta, no fit), one inner loop, real drone 5 seeds, mean 3-D RMS cm (crashes by the tilt rule in brackets)

| start of the tune | model cost start -> best | best epoch | real tuning cells: start gains -> tuned | real test cells: start gains -> tuned | tuned gains (factor to default) |
|---|---|---|---|---|---|
| theta_0 | 0.03050 -> 0.00573 | 50 | 15.29 -> 7.29 | 9.04 -> 7.00 | Pxy 0.97 Pz 1.42 Dxy 1.21 Dz 1.19 Iz 1.54 KRxy 1.34 KRz 1.07 Kwxy 1.36 Kwz 0.92 KIxy 0.73 |
| theta_2 of (d) | 0.00115 -> 0.00035 | 50 | 11.48 -> 5.43 | 14.06 -> 9.40 | Pxy 1.56 Pz 1.43 Dxy 2.11 Dz 2.28 Iz 3.08 KRxy 1.41 KRz 1.11 Kwxy 1.37 Kwz 0.88 KIxy 0.30 |
| theta_3 of (d) | 0.05136 -> 0.05136 | 0 | 71.31 -> 71.31 | 67.85 -> 67.85 | Pxy 1.77 Pz 1.31 Dxy 2.54 Dz 1.80 Iz 3.02 KRxy 1.77 KRz 1.15 Kwxy 1.69 Kwz 0.85 KIxy 0.32 |

Per scenario (mean 3-D RMS cm over 5 seeds), start -> tuned:

| start | A1 | A8 | CIRCLE | FIG8 | A4 off 0.125 | A4 off 0.075 | OVAL |
|---|---|---|---|---|---|---|---|
| theta_0 | 37.4 -> 15.1 | 11.0 -> 5.8 | 6.6 -> 4.2 | 6.1 -> 4.0 | 8.8 -> 7.5 | 13.1 -> 10.1 | 5.2 -> 3.4 |
| theta_2 of (d) | 34.9 -> 15.1 | 7.4 -> 4.0 | 2.0 -> 1.4 | 1.7 -> 1.2 | 18.9 -> 11.1 | 21.2 -> 15.7 | 2.1 -> 1.4 |
| theta_3 of (d) | 50.0 -> 50.0 | 52.2 -> 52.2 | 101.0 -> 101.0 | 82.0 -> 82.0 | 50.8 -> 50.8 | 48.9 -> 48.9 | 103.9 -> 103.9 |

## Check 2: beta fit started at the true beta, flights of (d) round 3 and round 4 (those flights are the failed ones)

| flights of | best epoch | J start -> best | beta error % (thrust / lag / downwash / plume) at epoch 10 | 30 | 50 |
|---|---|---|---|---|---|
| round 3 | 4 | 0.5510 -> 0.5392 | [-5.3, 2.7, -5.2, 4.8] | [-9.7, -0.1, -5.0, 6.1] | [-10.4, -0.6, -4.9, 6.3] |
| round 4 | 42 | 1.0779 -> 1.0248 | [-1.0, 2.0, -0.9, -3.3] | [-3.2, -4.5, 4.7, -4.0] | [-5.4, -6.4, 6.5, -3.0] |

## Check 3: what drives the fit (J_sysId = p + v + attitude, at theta_l and the beta of round l)

Share of each part in the full four-part cost (mean over the 4 scenarios; w = body rates, not in the cost of Stage 3b) and the gradient of the Stage 3b cost with respect to beta, split by part (sum over the scenarios, positive = descent lowers that beta):

| round | J p | J v | J q | J w | share w % | dJ/dbeta total (thrust / lag / downwash / plume) | from p | from v | from q |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 0.0191 | 0.1417 | 0.0123 | 2.92 | 94 | [0.0765, 0.0208, -0.0164, 0.0506] | [0.0883, 0.0005, -0.0233, 0.0016] | [-0.0114, 0.0191, 0.0066, 0.046] | [-0.0005, 0.0012, 0.0003, 0.003] |
| 1 | 0.0053 | 0.1285 | 0.0096 | 3.16 | 96 | [-0.0279, -0.0006, 0.009, -0.0007] | [-0.029, -0.0001, 0.01, -0.0001] | [0.0011, -0.0006, -0.001, -0.0006] | [-0.0, -0.0, -0.0, -0.0] |
| 2 | 0.0250 | 0.1051 | 0.0086 | 7.04 | 98 | [0.0074, 0.0085, -0.0064, -0.0003] | [0.0139, -0.0005, -0.0058, -0.0002] | [-0.0078, 0.0109, -0.0006, -0.0001] | [0.0013, -0.0019, 0.0, 0.0] |
| 3 | 0.4482 | 0.0838 | 0.0027 | 22.33 | 98 | [0.005, 0.0065, -0.0009, 0.005] | [0.0254, 0.0049, -0.0063, 0.0008] | [-0.0202, 0.0015, 0.0055, 0.004] | [-0.0001, 0.0, -0.0001, 0.0003] |
| 4 | 0.7798 | 0.2467 | 0.0326 | 16.44 | 94 | [0.0245, 0.0015, -0.0021, 0.004] | [0.0925, 0.0002, -0.0228, 0.0004] | [-0.0679, 0.0014, 0.0209, 0.0036] | [-0.0002, -0.0, -0.0002, -0.0] |
| 5 | 0.3434 | 0.1412 | 0.0027 | 14.16 | 97 | [0.0111, 0.0005, 0.0027, 0.0032] | [0.0446, 0.0, -0.0077, 0.0002] | [-0.034, 0.0005, 0.0104, 0.003] | [0.0005, 0.0, -0.0, 0.0001] |

Per scenario, J parts (p / v / q) in the rounds:

| round | A1 | A8 | CIRCLE | FIG8 |
|---|---|---|---|---|
| 0 | 0.0693 / 0.5499 / 0.0484 | 0.0036 / 0.0151 / 0.0007 | 0.0019 / 0.0012 / 0.0000 | 0.0015 / 0.0006 / 0.0000 |
| 1 | 0.0207 / 0.5105 / 0.0383 | 0.0003 / 0.0028 / 0.0002 | 0.0000 / 0.0003 / 0.0000 | 0.0000 / 0.0003 / 0.0000 |
| 2 | 0.0879 / 0.3961 / 0.0333 | 0.0119 / 0.0237 / 0.0007 | 0.0000 / 0.0002 / 0.0001 | 0.0000 / 0.0004 / 0.0003 |
| 3 | 0.2194 / 0.0015 / 0.0021 | 0.2145 / 0.0483 / 0.0034 | 0.6212 / 0.1868 / 0.0035 | 0.7378 / 0.0985 / 0.0018 |
| 4 | 0.0092 / 0.0001 / 0.0012 | 0.2537 / 0.0960 / 0.0082 | 1.7756 / 0.5389 / 0.0866 | 1.0808 / 0.3520 / 0.0344 |
| 5 | 0.0034 / 0.0000 / 0.0011 | 0.1973 / 0.0557 / 0.0023 | 0.6681 / 0.1519 / 0.0029 | 0.5050 / 0.3569 / 0.0046 |

Where the state of the trace comes from: `frsim/episode.py` records `body.x, body.v, body.q, body.w`, the true state of the plant at the start of the tick, not the noisy sensor estimate (`sen.state` feeds only the controller). The model output is the true state of its own noise-free plant, so position and velocity are compared as true-state against true-state. The noise enters through the controller (noisy feedback and gyro), which makes the real true-state differ from the noise-free model.

## Check 4: per scenario real cost of (d), 3-D RMS / z RMS cm and max tilt deg of the flight (one flight per scenario and round)

| round | A1 | A8 | CIRCLE | FIG8 |
|---|---|---|---|---|
| 0 | 36.9 / 35.4, 31 deg | 11.2 / 9.6, 30 deg | 6.6 / 6.3, 5 deg | 6.2 / 5.7, 6 deg |
| 1 | 13.6 / 7.7, 29 deg | 5.9 / 4.9, 18 deg | 4.2 / 3.8, 5 deg | 4.0 / 3.5, 6 deg |
| 2 | 31.2 / 29.8, 33 deg | 12.9 / 12.6, 19 deg | 1.8 / 1.3, 6 deg | 1.5 / 0.8, 9 deg |
| 3 | 50.0 / 50.0, 9 deg | 47.2 / 46.8, 21 deg | 83.2 / 82.5, 17 deg | 88.4 / 88.1, 12 deg |
| 4 | 50.0 / 50.0, 7 deg | 51.3 / 47.5, 42 deg | 141.7 / 98.9, 49 deg | 105.5 / 92.7, 36 deg |
| 5 | 50.0 / 50.0, 6 deg | 46.4 / 45.9, 17 deg | 91.8 / 91.1, 18 deg | 68.8 / 67.7, 21 deg |

## Check 5: the model against the real drone, per scenario (3-D RMS cm, task window)

Columns: real = the flight of the loop (noise on); model, true beta = the differentiable model with the true P1 beta; model, beta0 = with the calibration; model, beta of the tune = with the beta fitted in the round that produced the gains (that is the model the tune used). `noise off` = frsim with the sensor noise off, same gains (extra flights, `clean_flights.py`).

| gains | scenario | real (noise on) | real max tilt | frsim noise off | model true beta | model beta0 | model beta of the tune |
|---|---|---|---|---|---|---|---|
| theta_0 | A1 | 36.9 | 31 | 32.1 | 32.1 | 10.7 | 10.7 |
| theta_0 | A8 | 11.2 | 30 | 9.9 | 9.9 | 8.7 | 8.7 |
| theta_0 | CIRCLE | 6.6 | 5 | 7.0 | 7.0 | 3.5 | 3.5 |
| theta_0 | FIG8 | 6.2 | 6 | 6.4 | 6.4 | 3.4 | 3.4 |
| theta_1 | A1 | 13.6 | 29 | 12.5 | 12.4 | 4.7 | 14.5 |
| theta_1 | A8 | 5.9 | 18 | 4.9 | 4.9 | 3.9 | 5.0 |
| theta_1 | CIRCLE | 4.2 | 5 | 4.4 | 4.4 | 2.2 | 4.6 |
| theta_1 | FIG8 | 4.0 | 6 | 4.2 | 4.2 | 2.2 | 4.4 |
| theta_2 | A1 | 31.2 | 33 | 3.4 | 3.4 | 1.2 | 2.7 |
| theta_2 | A8 | 12.9 | 19 | 5.3 | 5.3 | 3.4 | 3.8 |
| theta_2 | CIRCLE | 1.8 | 6 | 1.9 | 1.9 | 1.1 | 1.8 |
| theta_2 | FIG8 | 1.5 | 9 | 1.6 | 1.6 | 0.8 | 1.4 |
| theta_3 | A1 | 50.0 | 9 | 1.2 | 1.2 | 0.4 | 3.2 |
| theta_3 | A8 | 47.2 | 21 | 14.0 | 14.6 | 6.0 | 4.6 |
| theta_3 | CIRCLE | 83.2 | 17 | 2.7 | 2.7 | 5.1 | 2.0 |
| theta_3 | FIG8 | 88.4 | 12 | 42.3 | 42.8 | 5.3 | 1.7 |
| theta_4 | A1 | 50.0 | 7 | 1.6 | 1.6 | 0.7 | 40.4 |
| theta_4 | A8 | 51.3 | 42 | 6.9 | 6.3 | 7.0 | 6.4 |
| theta_4 | CIRCLE | 141.7 | 49 | 71.3 | 72.9 | 8.3 | 2.1 |
| theta_4 | FIG8 | 105.5 | 36 | 88.8 | 88.6 | 6.4 | 2.0 |
| theta_5 | A1 | 50.0 | 6 | 0.7 | 0.7 | 0.4 | 44.2 |
| theta_5 | A8 | 46.4 | 17 | 5.3 | 5.5 | 5.2 | 4.7 |
| theta_5 | CIRCLE | 91.8 | 18 | 42.2 | 41.7 | 5.3 | 1.6 |
| theta_5 | FIG8 | 68.8 | 21 | 62.8 | 63.0 | 4.3 | 1.7 |


## Reading (facts first, then the reading; no method change)

Facts:
1. Rounds 3, 4, 5 of (d) fail on all four scenarios with noise on. In A1 the drone never leaves the floor (3-D RMS 50.0 cm = z 50.0, z mean 0.00 m, max tilt 6-9 deg, so the tilt rule does not flag it); CIRCLE reaches 141.7 cm in round 4.
2. The same gain sets flown with the sensor noise off: A1 1.2 / 1.6 / 0.7 cm (theta_3 / theta_4 / theta_5), CIRCLE theta_3 2.7 cm. With noise on: 50.0 / 50.0 / 50.0 and 83.2 cm.
3. Model with the true beta against frsim with noise off (check 5): theta_3 A1 1.2 vs 1.2, A8 14.6 vs 14.0, CIRCLE 2.7 vs 2.7, FIG8 42.8 vs 42.3; theta_4 A8 6.3 vs 6.9, CIRCLE 72.9 vs 71.3, FIG8 88.6 vs 88.8. The model with the true beta reproduces the noise-free plant to a few percent.
4. The model with the beta fitted in the round that tuned the gains predicted 2.0 and 1.7 cm for CIRCLE and FIG8 of theta_3 (true-beta model: 2.7 and 42.8; real: 83.2 and 88.4) and 40.4 / 44.2 cm in A1 for theta_4 / theta_5 only.
5. Check 1: with the true beta one tune loop from theta_0 gives 7.29 cm on the tuning cells (test 7.00): the same level as the theta_best of (d) (7.08 / 7.11). From theta_2 it gives 5.43 / 9.40. From theta_3 the tune finds no step that lowers the model cost (best epoch 0); the gains stay in the failed region.
6. Check 2: started at the true beta on the failed flights of round 3 and 4, the fit stays within 10.4 % (thrust, round 3) and 6.5 % of the truth after 50 epochs; the cost changes only by 2 % and 5 %.
7. Check 3: the velocity part of J_sysId in rounds 0-2 is carried by A1 (0.55 / 0.51 / 0.40 against 0.001-0.02 for the other scenarios): the real A1 oscillates with noise (tilt 29-33 deg). It is not measurement noise: the trace holds the true plant state. In rounds 3-5 the position part grows to 0.34-0.78 (flights on the floor) and the gradients of p and v on the thrust gain have opposite signs (round 4: +0.0925 and -0.0679).

Reading:
- The fit is not the only cause. The tuning model has no sensor noise, and the gain sets from round 3 on (Dxy x2.5, Iz x3, KIxy x0.3, Pxy x1.8, KRxy x1.8, Kwxy x1.7 in round 3) are stable in the noise-free model with the true beta (A1, CIRCLE) and in noise-free frsim, but unstable with noise in the "real drone": the drone does not leave the floor in A1 and loses height in CIRCLE. Answer to the question asked: yes for A1 and CIRCLE of theta_3, and for A1 of theta_4 and theta_5; for A8 and FIG8 of theta_3 and for CIRCLE / FIG8 of theta_4 / theta_5 the noise-free model with the true beta already predicts the failure (fact 3), the fitted beta did not.
- The wrong beta hides part of the failure (fact 4) but a right beta does not by itself beat the early loop (fact 5), so the drift is a combination of a model without noise (the gains drift to values that only work noise-free) and a beta that is off.
- The beta fit does not walk away from the truth when it starts there (fact 6); the cost is nearly flat for those flights. The drift in the loop is cumulative: each round starts at the beta of the last round and moves it by up to 0.01 per epoch (0.5 in log in 50 epochs) in a flat cost with p and v gradients of opposite sign.
- Whether a noise-robust tuning cost (noise in the tuning model, an added cost term or a bound on the gains) removes the failure was not tested: open, and it would change the method, so it is a decision for the user.

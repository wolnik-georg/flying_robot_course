# Actuator identification from real flights (2026-10-29)

Script `experiments/tuning/actuator_id.py` (output `experiments/tuning/out/actuator_id.csv`). Data: bottom drone cf5, DShot `rpm_m1..4` against `motor_m1..4` (2 ms uSD log), airborne part of 187 flights from all controllers (S0, S1, S2, S3, S4 FBL, pilot, NS2 training, A1 clean). Model per motor and flight: `F_meas = g * LPF_tau((cmd/65535)^2 delayed by d)`, grid over tau and d, g by least squares.

| Group | flights x motors | tau (ms) median | delay (ms) median | R2 median |
|---|---|---|---|---|
| S0 | 40 | 30 | 6 | 0.97 |
| S2 | 56 | 30 | 6 | 0.97 |
| S4 FBL | 137 | 30 | 4 | 0.97 |
| NS2 training | 183 | 30 | 6 | 0.97 |
| pilot | 192 | 30 | 6 | 0.95 |
| S1 (INDI) | 40 | 25 | 6 | 0.76 |
| S3 (INDI) | 24 | 30 | 6 | 0.83 |

- Time constant 30 ms (flight-to-flight sd 4 ms, same for all four motors and all controllers); command-to-telemetry delay 4-6 ms.
- The plant of record has lag 22 ms and a 17.6 ms rpm-feed delay (`rd`). The lag is about 8 ms (27 %) shorter than the data say; the earlier ~44 ms was a different measurement (torque/gyro correlation, includes controller and filter delay).
- INDI flights (S1, S3) fit worse (R2 0.8) and the static gain is lower (0.14 against 0.22): the command is compensated by INDI, so the simple map does not hold there. Their tau is consistent, not exact.
- Caveat: the rpm telemetry has its own latency, so the true motor tau is at most 30 ms. 

Consequence: fix lag = 30 ms in the plant (not fitted), recalibrate the remaining parameters, and use 20 / 30 / 45 as the lag variants. Tuned S0 sets from the first pass crash at lag 30 (A1 s2, A8 s2) and at 44 (nearly all), so they would be marginal on hardware.

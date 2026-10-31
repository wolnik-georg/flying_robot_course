# FBL xy plant gap: diagnostic (2026-10-28, desk only, uSD logs of 2026-10-26, final variant)

**Question:** can the C-in-the-loop plant (`experiments/fbl/stage3_port/stock_inner_sim.py`) be used to tune the FBL xy gains? **Answer so far: no.** On hardware A4 (dz 0.4, offset 0.125, 0.35 m/s) the xy RMS is 10.2 cm (one hump: cross-track about -14 cm at 2.75 s, along-track about -20 cm at 4 s). The plant gives 0.3 cm (no disturbance) or 2.1 cm (varying body torque 300/400/60 µN·m). A8: 5.6 cm hardware vs 2.1 sim. A1: 3.0 vs 2.4 (agrees only with the torque disturbance).

| Candidate | Test | Result |
|---|---|---|
| Lateral downwash force | hardware: F = m·a - T·(body z axis), from `vx`, `est_q*`, `fbl_thrust` | lateral force <= 0.01 N everywhere (A4, A1). Not the cause. |
| Wound-up integral from takeoff | sim: lateral disturbance during takeoff, removed before the scenario | adds at most 0.7 cm xy RMS. Not the cause. |
| Measurement delay | sim: position/velocity delayed 30/60/100/150 ms (motor lag 44 and 80 ms) | 30/60/100 ms: x RMS 0.7/1.3/2.2 cm, y unchanged (0.3); 150 ms unstable (tilt > 110°). Does not give a stable 10 cm hump. |
| Wrong gains on the drone | hardware z: regress logged `fbl_acc_z` on z error, velocity error, integral | kp 11.7, kv 5.3, ki 12.1 (A1, R² 1.00); set: 11.25 / 5.4 / 12. Preset 1 is loaded, the position law runs as designed. (A4 R² 0.96-0.98, A8 not usable.) |
| Effective xy gains from lateral acceleration | same regression on xy | Not usable: the same method reads kp 0.1-2.8 on the sim traces (true 14). |
| Inner rate loop | w_des vs. filtered gyro | Inconclusive: the sim w_des is dominated by noise (std 0.29 rad/s vs 0.07 actual), hardware gyro has 0.2 rad/s of vibration noise. |

**Open:** the difference is probably in the roll/pitch path (real motors, vibration, rate PID), which z does not have. This fits why the z gains tuned in simulation transferred. Untested: a system identification of the roll/pitch response from the logged `rpmf_m*`, `gyro_*`, `fbl_w_des_*`.

Scratch scripts (not in the repo): scratchpad `lat_force.py`, `sim_ic.py` (disturbance, delay), `zfit.py`, `fit2.py`, `sim_w.py`, `hw_w.py`.

## Update (same day): the missing physics is a downwash torque

Scripts: `experiments/fbl/sysid/` (hardware: pyenv python; `plume_torque_sim.py`, `ns2_torque_at_rotors.py`: system python3).

| Step | Result |
|---|---|
| Motor command -> rpm (differential roll/pitch mode, 4 flights, corr 0.97-1.00) | lag 30 ms (44 ms in the plant), delay 4 ms (as in the plant), **gain 1.5-1.8x**: the real force is quadratic in the command, the firmware mixer assumes linear. Plant corrected (`quad`): xy gets slightly better, not worse. Not the cause. |
| Torque the motors had to produce (from rpm, mean over 0.5 s, balances the disturbance) | A4 under the top drone: pitch (y) 1300-2450 uN m, roll (x) up to 1650; it drops to about 400 when the drone has left the plume (about 3 s). A1: pitch about 1400 uN m at 4-8 s. **The plant assumes 300-400 uN m.** |
| Source | The downwash force field across the rotor span: NS2 (z force) evaluated at the four rotor positions gives the same sign and pattern, **about 0.45x the measured size** (A4 start 915 vs 2238 and 554 vs 1312; A1 -640 vs -1392). The sign of x is not reliable. |
| Plant with this torque (x2.2, `kappa`), motor law corrected, lag 30 ms | A4 xy RMS 0.3 -> **5.1 cm** (x3.0: 9.3 cm; hardware 10.2). A8 0.36 -> 0.67 (hardware 5.6). A1 0.20 -> 0.36 (hardware 3.0). |

**Status:** the main missing effect is found (a position-dependent downwash torque, absent from the plant); A4 is reproduced in size with a factor 2.2-3.0 on the NS2 gradient. A8 and A1 are not: their hardware error is mostly a constant offset (A1: x/y bias 1.9 / 2.1 cm of 3.0 cm RMS) that the plant does not produce and the integral does not remove. Not yet explained.

**Consequence for the plan:** any plant used for tuning needs the plume torque; its size has to be fitted to hardware (the rpm-based torque is logged in every flight with two drones, so a map `torque = f(relative position)` can be fitted from all S0-S3 flights). Z-only downwash truth is not enough.

## Update 2 (same day): fitted plume model and gyro vibration

Scripts: `experiments/fbl/sysid/collect_plume_samples.py`, `fit_plume_torque.py`, `plume_torque_sim.py` (output `out/plume_samples.csv`, `out/plume_fit.json`).

**1. Plume map fitted on all two-drone flights** (240 flights, 6214 quasi-steady 0.5 s windows: vertical acceleration < 0.3 m/s², angular acceleration < 6 rad/s², dz 0.15-0.8 m, horizontal distance < 0.6 m). Model: lift change per rotor `dF_k = -A (0.4/h)^n exp(-rho_k^2 / (2 sigma^2))`, rho_k = distance of rotor k to the top drone's axis, torque = sum of r_k x dF_k.

| Parameter | Value |
|---|---|
| A (lift loss per rotor at h = 0.4 m) | 0.054 N |
| sigma | 0.061 m |
| n | 0.6 (trail of the axis behind a moving top drone: 0.07 s, no effect) |

| Quantity | Correlation, all data | Correlation, fitted on half of the flights, tested on the other half |
|---|---|---|
| extra thrust needed (fz) | 0.95 (slope 1.02) | 0.96 |
| pitch torque (tau_y) | 0.83 (slope 1.10) | 0.83 |
| roll torque (tau_x) | 0.62 (slope 0.67) | 0.63 |

**2. Gyro vibration.** The hardware gyro has 17 deg/s of noise above 5 Hz in A1 (A4 about 7, A8 about 5); the plant had 0.5. The rate PID passes it on as torque noise; A1 shows a 1.2 Hz tilt oscillation of 2.3 deg (pitch std) which the plant did not have.

**3. Plant with the fitted plume torque, lag 30 ms (the quadratic motor law option was only in the earlier experiment, not in these runs), logged top-drone trajectory, measured gyro noise** (4 noise seeds, spread +-0.1 to 0.3 cm):

| Cell | Plant xy RMS | Hardware xy RMS |
|---|---|---|
| A4 off 0.125 (flight 17-25-15) | 7.2 cm | 10.2 cm |
| A4 off 0.075 (17-26-40; the plant ran off 0.125) | 7.3 cm | 8.2 cm |
| A8 | 0.7 cm | 5.6 cm |
| A1 | 1.5 cm (z RMS 15 cm: a plant artefact at this noise level, hardware 0.6) | 3.0 cm |

**Status:** A4 is reproduced within 11-30 %. A1 is half of the hardware value and has the right tilt oscillation. **A8 is not reproduced (0.7 vs 5.6 cm):** the hardware error there is a -9.5 cm excursion at the crossing and a slow +7.5 cm swing 3-4 s later, far from the top drone; the cause is open.

## Update 3 (same day): A8, cause not found; what is ruled out

Hardware A8 (17-23-37): -9.5 cm along-track at 4-5 s (r = 0.14-0.8 m from the top drone, speed 0.35 m/s) and +7.5 cm at 8-9 s (r > 1 m). The plant gives 0.7 cm. The measured lateral acceleration follows the reference acceleration with a **lag of 290-500 ms** (hardware A8 364 ms, A4 308 / 496 ms, gain about 1); the plant has 0 ms. Without any acceleration/jerk feedforward the plant gives 570 ms and 8.6 cm, so the hardware behaves like a feedforward that works at about 35-40 %.

Verified on hardware (not the cause):
| Check | Result |
|---|---|
| Control law xy | commanded jerk rebuilt from the logged `fbl_w_des` and thrust, regressed on reference jerk, position error, velocity error, acceleration error: R² 0.95-0.99, ki 10.5-13.3, kp 12.3-17, kv 8.9-10.4 (set 12 / 14 / 9.5), reference-jerk coefficient 1.1-1.3 on the moving axis. The law, gains and feedforward are as designed. |
| Rate loop | roll rate vs. commanded rate: gain 0.82, lag 102 ms (hardware A8) vs. 0.85 / 80 ms (plant). |
| Estimator | EKF position equals the mocap input (rms 0.7 mm, 4-6 ms shift); EKF velocity lags d(position)/dt by 26 ms. |
| Setpoint | updated every 6 ms, consistent (derivative of position = velocity, amplitude ratio 1.01). |

Tried in the plant, no effect on A8 (0.7 cm -> at most 1.8 cm): lateral drag 0.23 1/s (fitted from the flights), replay of the measured lateral force series, inertia x2/x4/x8 (a `quad` option passed in the same runs was not implemented in that plant version, so the quadratic law is NOT covered by this line), EKF update hold 3-6 ms with and without the estimator fix, tilt-to-acceleration gain 0.35-1.0, state delay 40/80 ms (120 ms goes unstable, hardware does not). Feedforward jerk removed: 0.7 -> 1.0 cm.

**Open:** which element weakens the feedforward on hardware by 60 %. Candidates not testable at the desk: a timing offset between the reference and the state in the real stack, or an effect of the real HLC trajectory stream.

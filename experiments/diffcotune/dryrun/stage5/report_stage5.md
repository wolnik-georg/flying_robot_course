# Stage 5 report: S0 lab preparation closed, S2 / S1 / S3 prepared like S0 (DiffCoTune Split-Alternate, 13 Nov 2026)

Desk work, nothing flown, flashed or connected. Method version frozen by the user on 13 Nov (50 epochs, best iterate, J_sysId = p + v + q, noise in the tuning step with 2 seeds, +-25 % gain limit, lr 0.01, Adam reset per loop, 5 rounds, keep-best, last fit skipped). Every pool job ran one at a time through `dryrun/stage3c/job_runner.py` under `nice -n 19` (driver `driver5.py`, log `logs/driver.log`; the runner logs load and MemAvailable at the start and one minute later in `logs/runner_*.log`). Files: `experiments/diffcotune/dryrun/stage5/`, `lab/`.

## Part A: S0, the lab preparation

### A1. Offline preflight, one command

`python3 lab/preflight.py --controller S0 [--pad X Y] [--yaml <crazyflies.yaml>]` (new, `lab/preflight.py`, `lab/flights_template.py`): output `lab/results/preflight_S0.txt` (21:00, desk PC): **PREFLIGHT PASSED**, 17 PASS and 2 NOTE lines:

- 1 imports: frsim bindings (`cffirmware` from the crazyflie-firmware build), scipy, yaml, torch; the S0 model compiles and runs the A1 reference (finite, 10 s).
- 2 flight files: `fly_file.py --check` CIRCLE margin 0.250 m (pad x window [-0.25, 0.25], y [-0.50, 2.00]) and FIG8 margin 0.540 m at the pad (0.0, 0.3); OVAL refused.
- 3 round state: `round.py --init` in a temporary directory: round_0 gains = default gains (yaml round trip 0.0), the 15 values equal `TREE_DEFAULT_GAINS`; the merge into a copy of this PC's yaml and into the yaml of the strategy's crazyswarm2 branch (read with `git show`, nothing checked out) changes only the 15 values, the switches stay.
- 4 settings: `flights.json` template (tuning order A1, A8, CIRCLE, FIG8; `rotate_deg` -90 for CIRCLE and FIG8; 6 test flights A4 cell 1 / 2 x 3); `round.py` settings (states pva, gain limit 0.25, lr 0.01, skip_last_fit, n_rounds 5). NOTE: the default of `--n-rounds` in `round.py` is still 6 (Stage 4 value); every command must pass `--n-rounds 5` (it is on the lab card).
- 5 resources: free disk 426 GB against 2.75 GB needed (largest 23 Oct merged file 28.7 MB x 32 flights x 3 repeats); load and MemAvailable against the limits (22, 3 GB).
- 6 desk-PC paths: the only desk path in a lab-run script is the default yaml of `fly_file.py` (`Path.home()/Desktop/crazyswarm2/...`); it now has a flag `--yaml` (and `--pad`); the others take paths as arguments.
Other controllers: the same command with `--controller S1|S2|S3` (model, beta count, strategy switches, branch yaml).

### A2. ROS test without hardware: **not possible on this PC**; the logic is tested against a mock

- The crazyswarm2 sim backend (`crazyflie_sim`) has no CRTP / cflib link: it writes the yaml `firmware_params` into the compiled controller through the python bindings (`crazyflie_server.py::_apply_oot_firmware_params`, only the gains of the OOT controller map, and it reports per-robot gains it cannot apply). `apply_gains.py readback|set` talk to a radio URI with cflib. cflib's UDP driver (`udp://`) would need a firmware SITL binary; none exists on this PC and building one is a firmware rebuild (not allowed).
- Done instead (`lab/test_apply_gains_mock.py`, output `lab/results/test_apply_gains_mock.txt`, MOCK TEST PASSED): a fake cflib with a parameter store that behaves as the real one for the points that matter (fresh connection sees the store, `get_value` after `set_value` on the same connection is a cache, the firmware stores float32): readback on a wrong store fails (exit 1); `set` + readback passes (float32 tolerance, exit 0); one gain 1 % off fails; the 15 parameter names; the switch check per strategy (S0 .. S3 pass with their own switches and fail with another strategy's); `merge --strategy` on the yaml of each of the four crazyswarm2 branches (main, study_S1, study_S2, study_S3) passes and changes only the 15 values, and is refused when the yaml belongs to another strategy.
- What only the lab can verify: the radio and the real cflib timing; the real firmware parameter table (names `ctrlOmarIndi.*`, `rnn.en`, `stabilizer.controller` exist on the flashed image); that CS2 re-pushes the yaml after a power cycle (not stated); the launch routine with the merged yaml.
- First-block lab steps for the two scripts: (1) CS2 stopped, power on cf5; (2) `python3 lab/apply_gains.py readback --uri <cf5 uri> --fragment lab/<C>/round_0/gains.yaml --strategy <C>` on the unmodified yaml state: expected PASS only if the yaml already carries the default gains, otherwise read the 15 values and compare by eye; (3) `apply_gains.py merge ... --strategy <C>` into the lab yaml, launch CS2, stop, `readback` again (PASS expected); (4) `set` once (CS2 stopped) with a fragment differing by a few percent, `readback` must PASS; relaunch CS2 and `readback` again to see whether the yaml overwrites the set values (this answers "does CS2 re-push the yaml"); (5) record the times of stop / launch / readback (not stated in the repository).

### A3. One-page lab card

`lab/S0_LAB_CARD.md`: before the block (uSD file count on both cards with archive at >= 85, `check_usd_deck.py`, `check_active_controller.py` after a flash, pad check, preflight, batteries, `round.py --init`), per round (merge + readback, the 4 flight commands in order, `--solo` relaunch, merge uSD files, `flights.json`, the `round.py` command with `--n-rounds 5`, outputs, test flights), rules (abort rule 0.5 s continuous, stop after 2 crashes, estimator toggle, repeat rule, battery rule, uSD only). Every line names its source (protocol draft section or comparative study protocol section).


## B. S2 (S0 + NS2), plant P1, 4 beta

### B1 Model checks (S2 differentiable model against frsim)
- Forward (noise off, 6 scenarios x 5 rows): **PASS**, 30 cells, largest relative difference 0.37 % (A1, gains2), largest position difference 0.25 mm (`results/S2_forward_check.json`).
- Noise gate (Stage 3d revised rule): **PASS**, 20 of 24 cells stable (within 0.5 %), all 24 pass (`results/S2_noise_check.json`).
- Gradient check: **FAIL for the combined J_task and J_sysId; only A1 is responsible.** Single A8, CIRCLE, FIG8: 0 failing rows. Single A1: 6 failing rows (J_task), 5 (J_sysId). Failing combined parameters, J_task: Dz (rel. 0.14), thrust gain (0.0119), downwash scale (0.0106); J_sysId: downwash scale (0.0218). Diagnostics (`logs/S2_diag_a1*.log`): no floor contact (min z 0.477 m); finite difference independent of step 1e-4…1e-7; on 6-s flights compiled AD, eager AD and FD agree; windows to 2–10 s AD = FD, 2–14 s Dz off by 27 %; S0 A1 agrees under the same test. **Cause not confirmed.** The gradient check was not repeated; the dry run below shows its effect.
- Runtime: round about 23.5 min (flying/ingest 0.2 min, fit about 11.5 min, tune about 12 min); 97 min for the 5 rounds plus evaluation.

### B2 Identifiability (P1 truth, pva, 50 epochs, best iterate)
beta errors: thrust gain +21.4 %, motor lag −30.1 %, downwash −16.8 %, plume +40.7 %; best epoch 47; J_sysId 0.1808 → 0.1786. S0 reference (Stage 3b B1): −1.7 / −34.5 / +5.7 / +22.3. **Weak: the S2 fit does not recover the thrust gain, downwash and plume.** The user decides on beta handling for S2.

### B3 Dry run (5 rounds, P1; flight seeds 100+10l+i, noise seeds 6000+2l(+1), evaluation seeds 2000–2004)
Table from `sections5.py S2` (`results/sections_S2.md`):

| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 15.71 | 14.37 | A1 | 24.7 |
| theta_1 | 12.62 | 11.52 | 0 | 23.5 |
| theta_2 | 8.62 | 4.14 | 0 | 23.5 |
| theta_3 | 8.10 | 4.08 | 0 | 23.5 |
| theta_4 | 7.96 | 4.19 | 0 | 0.1 |

Keep-best over the flown sets: theta_4 (7.96 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 3 | 0.18423 | 0.18219 | - |
| 0 | tune theta | 51 | 48 | 0.010084 | 0.0073053 | [0, 0, 0, 0, 7, 0, 0, 0, 0, 0] |
| 1 | fit beta | 51 | 34 | 0.12855 | 0.12453 | - |
| 1 | tune theta | 51 | 27 | 0.042579 | 0.011502 | [0, 0, 0, 0, 1, 0, 0, 0, 9, 0] |
| 2 | fit beta | 51 | 4 | 0.16938 | 0.16925 | - |
| 2 | tune theta | 51 | 12 | 0.015048 | 0.013499 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 3 | fit beta | 51 | 6 | 0.16644 | 0.16629 | - |
| 3 | tune theta | 51 | 8 | 0.010623 | 0.010112 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume):

| Fit of round | error % |
|---|---|
| 0 | +22.6 / -31.4 / -21.6 / +46.4 |
| 1 | -10.7 / -37.0 / -22.6 / +39.2 |
| 2 | -12.9 / -39.4 / -20.4 / +43.6 |
| 3 | -9.1 / -39.9 / -15.6 / +42.3 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 29.01 / 26.98 / 5 | 8.40 / 6.35 / 0 | 6.28 / 5.94 / 0 | 5.77 / 5.31 / 0 | 6.81 / 5.80 / 0 | 8.71 / 7.37 / 0 | 4.78 / 4.59 / 0 | 12.36 | 6.77 |
| theta_best (round 4) | 14.31 / 7.09 / 0 | 5.12 / 3.13 / 0 | 3.37 / 2.50 / 0 | 3.35 / 2.22 / 0 | 5.81 / 3.31 / 0 | 6.95 / 4.31 / 0 | 2.45 / 1.95 / 0 | 6.54 | 5.07 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 12.62, limit 13.62) | **yes** (8.62, 8.10, 7.96) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **yes** (theta_best 6.54 / 5.07, theta_0 12.36 / 6.77) |

Criteria a, b, c all met. theta_0 on A1 flags the floor (as with S0), 5 of 5 evaluation seeds. The real cost falls 15.71 → 7.96 cm, the test mean 6.77 → 5.07 cm. The beta fit stays 9–13 % off on the thrust gain after round 0 and 40 % off on lag and plume throughout, which matches B2; the tuning improved regardless.

### B3 Real-data check (10 S2 flights of 23 Oct, beta0, default gains; controller from `per_flight.csv` column `strategy`)
mean p 0.0130, v 0.4332, q 0.0416, w 11.4915 (information only), J_pva 0.4879 (S0 flights about 0.492). Per scenario means: A1 p 0.0257 / v 1.2056 / q 0.117 / w 30.613; A4 0.0114 / 0.1633 / 0.0141 / 4.4751; A8 0.0026 / 0.0207 / 0.003 / 1.7252 (`results/realdata_S2.json`).

### B4 Lab tools (S2)
`lab/dry_test5.py S2` (round-0 dry test against `loop_lib`): real cost 15.712 vs 15.710 cm, theta_1 and beta difference **0.0**, gain-limit hits identical, yaml round trip 9e-17 (`lab/results/dry_test5_S2.json`). `apply_gains.py` S2 switches (indi 0, rnn.en 1) tested against the mock including the merge on the `study_S2` yaml. Round-0 test took 25 min. The S0 regression (`S0_roundtest`) is still running.

### S0 regression (round-0 dry test with the generalised tools)
`lab/dry_test5.py S0` (`lab/results/dry_test5_S0.json`, 19 min): real cost 19.988178568413346 cm, identical to `lab/results/dry_test.json` round_0 (19.988178568413346); theta_1 difference to `dry_test.json` **0.0**, to the Stage 3e loop (`loop_P1.json`) **0.0**, beta difference 0.0. S0 behaviour is unchanged.

## C. S1 (INDI mode 3), plant P1-rd (rpm delay 8 ms, calibrated 4.0502 ms), 5 beta

### C1 Model checks (in progress; this part written 23:45)
- Forward (noise off, 6 scenarios x 5 rows): **PASS**, 30 cells, largest relative difference 0.156 % (A1 gains1), largest position difference 2.09 mm (`results/S1_forward_check.json`).
- Noise gate: **PASS**, stable cells 24 of 24 (`results/S1_noise_check.json`).
- Gradient check, identifiability, real-data check, dry run: running or pending in the driver.

### C1 (update 00:11) S1 gradient check: FAIL, and worse than S2
- Combined J_task and J_sysId: **FAIL.** Only A1 is responsible (single A8, CIRCLE, FIG8: 0 failing rows; single A1: 7 failing rows each for J_task and J_sysId). Unlike S2 (a mismatch of 1–14 %), the S1 A1 autograd values for the z-gains and for all beta are **1e20 to 1e24** (e.g. Pz autograd −1.97e24 against finite difference +1.73e-4; thrust gain +1.8e22 against −1.4e-2). The xy gains and the combined values of the other scenarios agree to 1e-7. This is a gradient explosion on A1, not a small mismatch. The cause is **not found** (the INDI loop with the saturating rpm or the Butterworth states over the A1 hover/step is the suspect; not tested).
- Consequence for the method: a combined gradient with a 1e24 component from A1 would be dominated by that scenario in the sum; the dry run (running, started 00:05) shows whether projected Adam copes. The noise gate is passed, so the driver was not stopped (the user's stop rule names the gate; this is reported as an open point).

### C2 Identifiability (P1-rd truth: thrust 0.8, lag 1.5, downwash 1.3, plume 0.7, rpm delay 8 ms = factor 1.975 of the calibrated 4.0502 ms; pva, 50 epochs, best iterate)
| beta | thrust gain | motor lag | downwash | plume | rpm delay |
|---|---|---|---|---|---|
| error % | +17.4 | −30.3 | −27.7 | +35.1 | −46.9 |
| fitted factor vs calibration | 0.939 | 1.045 | 0.940 | 0.945 | 1.048 (= 4.245 ms) |

Best epoch 50 of 50 (still falling), J_sysId 0.1367 → 0.1087; parts start p 0.01444, v 0.11464, q 0.00758, w 2.2016. **The RPM delay is not recovered:** it moves from 4.05 ms to 4.25 ms against a truth of 8 ms (−46.9 %). No beta is recovered within a useful error. (`results/identifiability_S1.json`, 834 s.) The user decides whether the fifth beta stays.

### C3 Real-data check (10 S1 flights of 23 Oct, beta0, default gains; controller from `per_flight.csv` column `strategy`)
mean p 0.0163, v 0.1826, q 0.0151, w 5.0507 (information only), J_pva 0.2140. Per scenario: A1 p 0.0283 / v 0.3697 / q 0.0294 / w 10.246; A4 0.0135 / 0.1144 / 0.0092 / 2.338; A8 0.0081 / 0.0866 / 0.0086 / 3.473 (`results/realdata_S1.json`). Whether this shows a gap that no beta covers (INDI thrust-arrival risk) is judged after the dry run, when the model's own J can be compared on the same flights.

### C1b S1 A1 gradient explosion: first diagnosis (00:27; user decision 14 Nov: blocker for the lab, S1 and S3 not lab-ready until cause found and fixed)
Diagnostics `diag_s1_a1.py` (`logs/S1_diag_a1.log`) and `diag_s1_fix.py` (`logs/S1_fix_eps*.log`), A1, theta = 0, beta = 0, J_task on the window 2 … T_end s, 4 beta + rpm delay:

| Window end | max autograd (eager / compiled) | max finite difference |
|---|---|---|
| 3 s | 1.7e-2 / 8.7e-3 | 1.2e-2 |
| 4 s | 2.4 / 2.2 | 1.1e-2 |
| 6 s | 6.3e2 (compiled) | 8.5e-3 |

- The autograd gradient grows by about two orders of magnitude per 2 s of flight, while the finite difference of the same J stays at 1e-2. **Eager and compiled autograd already differ at the 3-s window** (e.g. Pz −9.5e-4 vs −1.3e-3, FD −1.2e-3), so the backward pass is numerically ill-conditioned (large terms that cancel), not a compile artefact alone.
- **Ruled out: the square root of the motor force in `_rpm_force` (clamp min 1e-12).** Replacing the clamp by 1e-4 N (copy `s1_model_try.py`, `SQRT_EPS`) gives the identical gradient (6-s window: max 6.328e2 in both runs). `s1_model.py` itself was not changed (the S1 dry run and its round test must use the same model).
- Not yet tested: the INDI inner loop (a_rpm − a_imu with the Butterworth filters, unit loop gain with a one-step delay: the tangent of this loop may be marginal in the backward pass), the thrust/rpm saturation (`FMAX`), the ground branch of A1. Next test: tangent norm of the state per time step (forward mode) to see where it starts to grow; then a fix candidate (for example a stop-gradient on a_imu's dependence on the previous step, or a damped tangent), checked with the forward gate again.
- **Lab-ready on the desk side: S0 yes (Stage 3e/4e); S2: no, open (A1 gradient mismatch of the combined gradient, cause unknown, the dry run met criteria a–c); S1: no (this explosion); S3: no (same model).** The RPM-delay identifiability verdict (C2) is **not judged** until this is fixed; A1 dominates the summed beta gradient.

(update 00:42) The window sweep continues the growth: max autograd 5 s: 4.5e2 (eager) / 3.4e1 (compiled); 6 s: 1.6e3 / 6.3e2; 8 s: 1.1e10 / 1.8e9, against a finite difference that stays at 7e-3. The tangent grows exponentially in time (about 7 orders of magnitude between 6 s and 8 s), as expected for an unstable mode of the backward pass that the real trajectory does not excite.

### C4 S1 dry run (5 rounds, P1-rd; seeds as for S2; 91 min, 119 min estimated after round 1)
Table from `sections5.py S1` (`results/sections_S1.md`):

| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 10.32 | 7.12 | 0 | 25.1 |
| theta_1 | 8.46 | 4.63 | 0 | 22.4 |
| theta_2 | 6.69 | 3.67 | 0 | 21.2 |
| theta_3 | 4.63 | 1.77 | 0 | 21.3 |
| theta_4 | 4.10 | 2.73 | 0 | 0.1 |

Keep-best over the flown sets: theta_4 (4.10 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 50 | 0.12815 | 0.10145 | - |
| 0 | tune theta | 51 | 50 | 0.0097634 | 0.007269 | [0, 0, 0, 0, 28, 0, 0, 0, 0, 0] |
| 1 | fit beta | 51 | 50 | 0.089725 | 0.082294 | - |
| 1 | tune theta | 51 | 50 | 0.0062938 | 0.0036143 | [0, 0, 8, 0, 8, 0, 0, 0, 0, 0] |
| 2 | fit beta | 51 | 2 | 0.039343 | 0.039148 | - |
| 2 | tune theta | 51 | 50 | 0.0036213 | 0.0021001 | [0, 12, 25, 21, 16, 20, 15, 0, 0, 27] |
| 3 | fit beta | 51 | 39 | 0.009879 | 0.0097582 | - |
| 3 | tune theta | 51 | 31 | 0.0019932 | 0.0010266 | [28, 29, 28, 27, 28, 27, 29, 0, 0, 28] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume / rpm delay):

| Fit of round | error % |
|---|---|
| 0 | +14.6 / -27.4 / -22.9 / +35.0 / -51.7 |
| 1 | +17.2 / -23.7 / -20.3 / +20.2 / -50.0 |
| 2 | +19.2 / -25.1 / -21.8 / +18.0 / -50.9 |
| 3 | +16.8 / -25.8 / -23.0 / +15.4 / -51.8 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 17.93 / 10.92 / 0 | 6.85 / 5.29 / 0 | 5.95 / 5.55 / 0 | 5.41 / 4.84 / 0 | 5.33 / 4.57 / 0 | 7.38 / 5.69 / 0 | 4.49 / 4.21 / 0 | 9.04 | 5.73 |
| theta_best (round 4) | 7.47 / 4.92 / 0 | 2.22 / 1.80 / 0 | 1.25 / 0.58 / 0 | 1.31 / 0.55 / 0 | 1.39 / 1.19 / 0 | 1.55 / 0.94 / 0 | 1.12 / 0.74 / 0 | 3.06 | 1.35 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 8.46, limit 9.46) | **yes** (6.69, 4.63, 4.10) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **yes** (theta_best 3.06 / 1.35, theta_0 9.04 / 5.73) |

Criteria a, b, c met. Real cost 10.32 → 4.10 cm (θ₀ fails no scenario; S1 θ₀ on A1 does not flag the floor), evaluation tuning mean 9.04 → 3.06 cm, test mean 5.73 → 1.35 cm, 0 failed flights. From round 2 on, most tuning rounds hit the ±25 % gain limit for 20–29 of 50 epochs (round 3: 7 of 10 gains at 27–29 hits, and again Kwz-type gains 0), i.e. the result is limited by the gain bounds, not by the stop rule. **RPM-delay beta (numbers only, verdict withheld):** error −51.7, −50.0, −50.9, −51.8 % after the fits of rounds 0–3 (fitted about 3.9–4.0 ms against the plant's 8 ms); the other betas are −27…+35 % off. **These dry-run numbers were obtained with a summed gradient in which the S1 A1 gradient is ill-conditioned (C1b); they cannot be called valid until the explosion is fixed.** The S1 dry run therefore counts as "met criteria a–c, but gradient unsound".

Lab-ready on the desk side, S1: **no** (A1 gradient explosion open).

### C5 S1 lab tools and real-data judgement (02:04)
- **Round-0 dry test** (`lab/dry_test5.py S1`, `lab/results/dry_test5_S1.json`, 22 min): real cost 10.3176 vs 10.3157 cm (`loop_lib`), θ₁ difference to `loop_S1.json` **0.0**, β difference 0.0, gain-limit hits identical ([0,0,0,0,28,0,0,0,0,0]), yaml round trip 9.7e-17. `apply_gains.py` for S1 (indi 3, rnn.en 0) tested against the mock including the merge on the `study_S1` yaml; not with ROS or hardware.
- **uSD variables the ingest needs (S1):** `STATE_COLS` (x, y, z, vx, vy, vz, gyro_x/y/z, ctrltarget_x/y/z) plus the quaternion columns, as for S0. `usd_study_config.txt` lines 32–50 log, for all strategies, `rnn.pred_x/y/z`, `rnn.clamped`, `ctrltarget.x/y/z`, `motor.m1–4`, `motor.m1–4_rpm` and `rpm.m1–4` (the rpm columns and `omar_a_rpm/imu` serve as diagnostics for INDI; the ingest does not use them). Nothing extra has to be added for S1.
- **Real-data judgement (INDI thrust-arrival risk):** real J_pva on the 10 S1 flights (β₀, default gains) is 0.214 (p 0.0163, v 0.183, q 0.0151); the model's J_pva after the fit is 0.109 (first fit, identifiability) and 0.126–0.082 in the dry run's fits. The real value is about 2× the fitted simulation value, for S2 about 2.7× (0.488 vs 0.18) and S0 flights about 0.49. The sets are not comparable in scenario mix (the real set has only A1, A4, A8; the simulated one A1, A8, CIRCLE, FIG8), and S1 is not worse than S0 or S2 against its simulation. **So this check shows no S1-specific gap beyond the S0/S2 gap; it cannot show whether the 64–72 % thrust arrival of `docs/s3_indi_ns2_decision_2026-10-18.md` is covered by a β**, because no β of this model changes the INDI thrust arrival (the plant has no thrust-arrival term), and the fit's rpm-delay β did not move towards the truth. Stated as open for the user.

### C1c S1 A1 gradient explosion: cause found (02:44; following the user's hint of 14 Nov)
Scripts `jac1.py`, `eig1.py`, `iter1.py`, `tangent.py`, `revfwd.py`, `fd_h.py` (logs `logs/S1_jac1*.log`, `S1_tangent.log`, `S1_revfwd*.log`, `S1_fd_h.log`). No file of the model or of the loop was changed.

1. **One-step Jacobian, autograd against central finite differences (the hint's test).** One 2-ms block of the S1 model, 156 state numbers + 15 parameters, A1 at theta = beta = 0. At 195 recorded blocks from t = −4.5 s to +3.5 s (stride 40 over the whole flight, stride 2 over blocks 2700–3000 = t 0.9–1.5 s) the largest |AD − FD| over the whole 156 × 171 matrix is **≤ 3e-9** (largest entry 5e2). **There is no operation whose gradient disagrees with its forward map** (no detach/where/clamp/index/in-place defect; the fractional-delay indexing included).
2. **Forward mode against finite differences:** the tangent of the A1 position with respect to the thrust gain (`torch.func.jvp`, 4750 blocks) equals the finite difference to 1e-6 over the whole 5 s.
3. **Reverse mode on the same quantity drifts, then fails:** d z(t)/d thrust-gain reverse against FD: 1.0 s 0.28941 / 0.28942; 1.5 s 0.2114 / 0.2115; 2.0 s 0.2055 / 0.2087; 3.0 s 1.121 / 0.193; 4.95 s 808 / 0.153. (J_task gradient 4.5e2 at the 5-s window against FD 9.5e-3; the FD is the same for steps 1e-4 … 1e-8.)
4. **Cause: the model's own one-block state Jacobian has an unstable mode.** `eig1.py`: at every block tested (−4.3 … 3.5 s) there are 4 eigenvalues of modulus > 1, the largest **1.006 – 1.012 per 2-ms block** (about ×400 per second), singular value 7e2. The eigenvector sits in the INDI filter states (the angular-acceleration filter, `fs[36..39]`, and the z acceleration filter `fs[8..11]`) and the stored acceleration `a_prev`: the INDI loop (a_rpm − a_imu, moments from the filtered angular acceleration, one block of delay) is marginal-to-unstable as discretised. `iter1.py` confirms it in the forward map itself: iterating one block with frozen inputs, a state perturbation of 1e-10 grows to 1.6e-4 after 1200 blocks (1e-8 → 1.6e-2), factor 1.64e6, as predicted. So the exponential growth of the reverse-mode gradient is the **true linearised sensitivity of the closed-loop S1 model to the unstable mode** (rounding noise of the backward pass is amplified by it; the forward flight stays bounded because the true trajectory does not excite the mode beyond rounding and the nonlinear map limits it). It shows on A1 because A1 is the 15-s flight (about 7 orders of magnitude between 6 s and 8 s); A8, CIRCLE and FIG8 pass the check, but the same eigenvalue is present there.
5. **Is the unstable mode physical or a modelling error?** The model agrees with frsim within 0.156 % in the forward check, i.e. frsim's INDI loop has the same property for this plant (the rpm filter, the 30 Hz Butterworth and the ang-acc differentiation at 500 Hz are as in the firmware). Not tested: whether the firmware's loop has this eigenvalue on hardware (the 64–72 % thrust arrival of `docs/s3_indi_ns2_decision_2026-10-18.md` and the 0.74–0.85 motor headroom could be linked, not shown).
6. **Options for the user (none adopted):** (a) accept J_task from the other scenarios and drop A1 from the S1/S3 gradient (changes the tuning scenario set: new deviation); (b) truncate the backward pass (stop-gradient after N blocks, or a damping of the filter-state sensitivity): a new deviation from the paper, to be tested against the forward gate; (c) gradient through a shortened horizon; (d) leave as is and tune with finite differences for S1/S3 (cost 15+ more flights per step: not feasible at 50 epochs). Which option is a decision for the user.
7. S2: the same one-step Jacobian check was **not yet run** for S2 (its state tuple and block signature differ; `jac1.py` needs an adapter). Open.

**Lab-ready on the desk side: S0 yes; S1 no; S2 no (open); S3 no (the same S1 model, its gradient check also fails on A1: 7 failing rows J_task, 8 J_sysId).**

### Preflight S1, S2, S3 (03:35): `lab/preflight.py --controller S1|S2|S3` all **PASSED** (`lab/results/preflight_S1.txt`, `_S2.txt`, `_S3.txt`; same checks as S0 plus the switch merge on the `git show` yaml of `study_S1/S2/S3`).

## D. S3 (INDI 3 + NS2), plant P1-rd, 5 beta (interim, 03:40, dry run in round 3)
- Forward **PASS** (30 cells), noise gate **PASS** (24 of 24 stable), gradient check **FAIL** (A1: 7 failing rows J_task, 8 J_sysId; the S1 explosion, same model; `results/S3_gradient_check.json`).
- Identifiability (P1-rd, pva, 50 epochs): errors thrust +24.6, lag −32.8, downwash −21.7, plume +48.4, rpm delay −49.9 %; best epoch 4; J_sysId 0.1506 → 0.1445 (verdict on the rpm delay withheld, as for S1).
- Real data (10 flights, S3 per `strategy`): mean p 0.0185, v 0.2235, q 0.0153, w 5.34 (information), J_pva 0.2573.
- Dry run so far: real cost 10.75 (θ₀), 10.80 (θ₁), 11.04 (θ₂) cm: **no improvement through round 2.** In round 0 the β fit returned its start iterate (best epoch 0, Jn 1.000), so ψ stayed at the start; the tuning step reached Jn 0.936 on the model but the flown cost did not fall. Estimate 135 min after round 1 (≤ 3 h, job continues).

### D (final, 04:32) S3 dry run (5 rounds, P1-rd; 107 min; 135 min estimated after round 1) — supersedes the interim lines above
Table from `sections5.py S3` (`results/sections_S3.md`):

| Gain set | real 3-D RMS (cm, mean of 4 tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |
|---|---|---|---|---|
| theta_0 | 10.75 | 7.52 | 0 | 28.4 |
| theta_1 | 10.80 | 7.56 | 0 | 25.5 |
| theta_2 | 11.04 | 8.06 | 0 | 25.7 |
| theta_3 | 11.08 | 8.31 | 0 | 25.6 |
| theta_4 | 8.90 | 5.92 | 0 | 0.1 |

Keep-best over the flown sets: theta_4 (8.90 cm).

| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |
|---|---|---|---|---|---|---|
| 0 | fit beta | 51 | 0 | 0.14825 | 0.14825 | - |
| 0 | tune theta | 51 | 46 | 0.011385 | 0.010657 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 1 | fit beta | 51 | 8 | 0.14681 | 0.14238 | - |
| 1 | tune theta | 51 | 40 | 0.012501 | 0.011723 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 2 | fit beta | 51 | 35 | 0.15098 | 0.13787 | - |
| 2 | tune theta | 51 | 35 | 0.010694 | 0.009836 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 3 | fit beta | 51 | 50 | 0.12688 | 0.098052 | - |
| 3 | tune theta | 51 | 50 | 0.0099584 | 0.0067514 | [0, 21, 0, 0, 27, 0, 0, 0, 0, 0] |

beta error after each fit (% against the plant truth; thrust gain / motor lag / downwash / plume / rpm delay):

| Fit of round | error % |
|---|---|
| 0 | +25.0 / -33.3 / -23.1 / +42.9 / -49.4 |
| 1 | +20.9 / -35.3 / -25.6 / +36.0 / -48.4 |
| 2 | +13.2 / -31.0 / -28.7 / +27.4 / -44.2 |
| 3 | +7.4 / -38.3 / -33.1 / +24.4 / -40.1 |

Final evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 18.73 / 11.92 / 0 | 6.10 / 4.21 / 0 | 5.91 / 5.51 / 0 | 5.33 / 4.76 / 0 | 5.82 / 5.21 / 0 | 7.02 / 5.36 / 0 | 4.49 / 4.21 / 0 | 9.02 | 5.78 |
| theta_best (round 4) | 16.03 / 9.63 / 0 | 5.05 / 3.61 / 0 | 4.29 / 3.77 / 0 | 4.20 / 3.53 / 0 | 5.57 / 5.09 / 0 | 6.15 / 4.89 / 0 | 3.55 / 3.21 / 0 | 7.39 | 5.09 |

| Criterion | result |
|---|---|
| a. rounds 2-4 <= round 1 + 1 cm (round 1 = 10.80, limit 11.80) | **yes** (11.04, 11.08, 8.90) |
| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best beats theta_0 on tuning and test means | **yes** (theta_best 7.39 / 5.09, theta_0 9.02 / 5.78) |

Criteria a, b, c are met **formally**, but the run is weak and not comparable to S0/S2/S1: the real cost is flat for rounds 0–3 (10.75, 10.80, 11.04, 11.08 cm; the criterion-a limit 11.80 is simply not exceeded) and falls only in round 4 (8.90 cm) after the tuning step of round 3 (the first with gain-limit hits: gains 2 and 5 at 21 and 27 epochs). Final evaluation: tuning mean 9.02 → 7.39 cm, test mean 5.78 → 5.09 cm (S1: 9.04 → 3.06 and 5.73 → 1.35; S2: 12.36 → 6.54 and 6.77 → 5.07); A1 stays at 16.0 cm. The β fit did not improve its start iterate in round 0 (best epoch 0), which left the model at the start β for the first tuning step; later fits moved the β in the direction of the truth (thrust +25.0 → +7.4 %, plume +42.9 → +24.4 %, rpm delay −49.4 → −40.1 %) while lag and downwash drifted away (−33.3 → −38.3 %, −23.1 → −33.1 %). RPM-delay recovery: **not judged** (user, 14 Nov, until the A1 gradient is fixed). Whether the slow start is due to the A1 gradient (the tuning step is dominated by it, see C1c) or to the NS2 term is **not tested**.

Lab-ready on the desk side, S3: **no** (A1 gradient blocker, same model as S1; the dry run is weak).

### D2 S3 lab tools (04:58)
`lab/dry_test5.py S3` (`lab/results/dry_test5_S3.json`, 27 min): real cost 10.7542 vs 10.7517 cm (`loop_lib`), θ₁ difference to `loop_S3.json` **0.0**, β difference 0.0, hits identical (all 0), yaml round trip 1.1e-16. `apply_gains.py` for S3 (indi 3, rnn.en 1) tested against the mock incl. the merge on the `study_S3` yaml; not with ROS or hardware. uSD variables as for S1 plus `rnn.pred_x/y/z`, `rnn.clamped` (logged for all strategies, `usd_study_config.txt` lines 32–50). Driver finished 04:43 ("ALL DONE").

### C1d Correction of C1c item 4 and the S2 check (05:00, `jac0.py`, `jac0b.py`, same one-block test for the S0/S2 block signature)
- **S2: one-block autograd = finite differences** at 9 blocks of A1 (t −4.5 … 3.5 s): largest |AD − FD| 1.4e-9 against entries up to 60. As for S1, no operation has a wrong gradient. S2's A1 mismatch is therefore **not** a backward-graph defect either.
- **The unstable eigenvalue is not specific to S1.** The one-block state Jacobian of S0 and of S2 also has 4 eigenvalues of modulus > 1 at every block after the start: S0 1.003–1.004, S2 1.003–1.0046 (6 at t = −2.5 s), eigenvector 99 % on the body rate `w[1]`. S0's A1 gradient is nevertheless in agreement with finite differences (Stage 2 and 3: only the known A1 row mismatch), so the existence of a mode with |λ| > 1 **does not by itself explain** the explosion. What differs for S1 is the size (1.006–1.012 per block, ×400 per second, against ×10 per second for S0/S2) and the place (the INDI filter states and `a_prev`). C1c item 4 is therefore a **finding about the S1 model (unstable linearised INDI filter loop), not a proven cause of the gradient explosion**; the reason S0 survives 15 s with a mode of ×10 per second was not investigated. Status of the cause: **partly located, not proven**; the lab blocker for S1/S3 stays, S2 stays open.
- Not done: a test of whether a modified loop (for example an INDI filter-state damping) removes the explosion, because any change of the model or of the gradient (truncation, damping) is a new deviation to be decided by the user first.

## E. Docs updated (05:00)
Protocol draft (section "S1–S3": per-controller table, risks with sources, lab-time table: four controllers 497.5 min tuning loops, 566.7 min with 12 test flights each), `lab/README.md`, `experiments/diffcotune/README.md`, `docs/diffcotune_spec.md` section 18, `docs/diffcotune_inventory.md` (all marked "Stage 5").

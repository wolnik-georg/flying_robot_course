# Stage 3e / 4a report (13 Nov 2026), corrected in Stage 4b

Corrections made after the check of 13 Nov: (1) the dry test covered one round, see C2; (2) three findings added below; (3) the printed `Jn` in `loop_lib.py` is the cost at the last epoch, not at the returned iterate (see "Findings").

Nothing flown, nothing flashed, no commit. Python 3.10 for all runs.

## 1. Part A
- **A1 gate: PASS.** Compiled patched batch vs eager float64: max relative J 4.49e-15, max relative gradient component 2.18e-10 (gates 1e-9 / 1e-6). `patch_check.json`.
- **A2 load reading:** `dryrun/stage3c/job_runner.py` (kept) reads `/proc/loadavg` and logs the load 1 min after each start (`logs/chain.log`, e.g. B2_P2: 2.71). The Stage 3d chain script already read `/proc/loadavg`; the decimal-comma bug was in the MemAvailable value (fixed with `LC_ALL=C`). Workers set torch threads to 2. The second runner I wrote in `stage3e/` was deleted.

## 2. B1, P1 (6 rounds, new seeds)
### Real cost per round (P1; 3-D RMS in cm, mean of the 4 tuning flights)

| Gain set | real 3-D RMS cm | z RMS cm | failed flights (tilt or floor) | flight seeds | minutes (round) |
|---|---|---|---|---|---|
| theta_0 | 19.99 | 18.96 | 1 (A1) | [100, 101, 102, 103] | 19.4 |
| theta_1 | 9.71 | 6.46 | 0 | [110, 111, 112, 113] | 18.6 |
| theta_2 | 8.30 | 5.72 | 0 | [120, 121, 122, 123] | 18.2 |
| theta_3 | 8.16 | 5.90 | 0 | [130, 131, 132, 133] | 18.3 |
| theta_4 | 8.13 | 5.64 | 0 | [140, 141, 142, 143] | 18.2 |
| theta_5 | 7.77 | 3.93 | 0 | [150, 151, 152, 153] | 0.1 |

Keep-best (lowest real cost over the flown sets): theta_5 (7.77 cm).

Gain-limit hits (projected Adam steps of 51 evaluations per gain; order Pxy Pz Dxy Dz Iz KRxy KRz Kwxy Kwz KIxy):

- tuning after round 0: noise seeds [6000, 6001], hits [6, 0, 8, 0, 27, 0, 0, 0, 0, 18], best epoch 50
- tuning after round 1: noise seeds [6002, 6003], hits [0, 0, 0, 0, 9, 0, 0, 0, 0, 0], best epoch 47
- tuning after round 2: noise seeds [6004, 6005], hits [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], best epoch 14
- tuning after round 3: noise seeds [6006, 6007], hits [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], best epoch 50
- tuning after round 4: noise seeds [6008, 6009], hits [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], best epoch 11

### beta error after each fit (relative error of each value against the P1 truth, %; no gate)

| Round | thrust gain | motor lag | downwash | plume |
|---|---|---|---|---|
| fit of round 0 | -1.6 | -36.0 | +6.1 | +23.3 |
| fit of round 1 | +12.0 | -21.5 | -9.3 | +39.0 |
| fit of round 2 | +5.1 | -3.0 | -12.7 | +19.5 |
| fit of round 3 | +3.7 | +5.0 | -12.8 | +9.0 |
| fit of round 4 | +6.0 | +5.5 | -11.9 | +13.4 |

### Final evaluation (real drone = frsim with noise, evaluation seeds 2000-2004; 3-D RMS cm / z RMS cm / failed flights of 5; mean over seeds)

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 37.20 / 35.73 / 5 | 11.44 / 9.47 / 0 | 6.63 / 6.32 / 0 | 6.16 / 5.73 / 0 | 8.80 / 7.14 / 0 | 13.05 / 11.98 / 0 | 5.21 / 5.03 / 0 | 15.36 | 9.02 |
| theta_best (round 5) | 13.43 / 5.27 / 0 | 6.29 / 4.73 / 0 | 2.90 / 2.03 / 0 | 2.93 / 1.68 / 0 | 9.48 / 7.77 / 0 | 10.99 / 9.75 / 0 | 2.80 / 2.44 / 0 | 6.39 | 7.76 |
| theta_5 (last flown) | 13.43 / 5.27 / 0 | 6.29 / 4.73 / 0 | 2.90 / 2.03 / 0 | 2.93 / 1.68 / 0 | 9.48 / 7.77 / 0 | 10.99 / 9.75 / 0 | 2.80 / 2.44 / 0 | 6.39 | 7.76 |

### Pass criteria (P1)

| Criterion | result |
|---|---|
| a. rounds 2-5 <= round 1 + 1 cm (round 1 = 9.71, limit 10.71) | **yes** (rounds 2-5: 8.30, 8.16, 8.13, 7.77) |
| b. no flown set after theta_0 fails (tilt or floor) a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best and theta_5 beat theta_0 on the tuning and the test scenarios (mean over 5 seeds) | **yes** (theta_best 6.39 / 7.76, theta_5 6.39 / 7.76, theta_0 15.36 / 9.02, tuning / test) |
| d. beta per round | reported above, no gate |

Floor / tilt rule per flown gain set (failed = tilt or floor):

| | A1 | A8 | CIRCLE | FIG8 |
|---|---|---|---|---|
| theta_0 | FAILED | ok | ok | ok |
| theta_1 | ok | ok | ok | ok |
| theta_2 | ok | ok | ok | ok |
| theta_3 | ok | ok | ok | ok |
| theta_4 | ok | ok | ok | ok |
| theta_5 | ok | ok | ok | ok |

### Run-to-run spread, rounds 0-3 (this run against the Stage 3d A + B run; different flight and noise seeds; no gate)

| Round | real cost cm, Stage 3d | real cost cm, this run |
|---|---|---|
| theta_0 | 19.80 | 19.99 |
| theta_1 | 10.16 | 9.71 |
| theta_2 | 8.53 | 8.30 |
| theta_3 | 6.86 | 8.16 |

beta error after the fit (%, thrust gain / motor lag / downwash / plume):

| Fit of round | Stage 3d | this run |
|---|---|---|
| 0 | -1.8 / -40.5 / +5.0 / +22.2 | -1.6 / -36.0 / +6.1 / +23.3 |
| 1 | +9.1 / -26.7 / -9.3 / +35.9 | +12.0 / -21.5 / -9.3 / +39.0 |
| 2 | +11.0 / -8.7 / -10.3 / +16.9 | +5.1 / -3.0 / -12.7 / +19.5 |

theta factors against the default gains (order Pxy Pz Dxy Dz Iz KRxy KRz Kwxy Kwz KIxy):

| Gain set | Stage 3d | this run |
|---|---|---|
| theta_1 | 1.21 1.25 0.80 1.09 1.25 1.13 0.98 1.11 0.82 0.80 | 1.25 1.23 0.80 1.24 1.25 1.15 1.09 1.18 0.89 0.80 |
| theta_2 | 1.10 1.12 0.74 1.08 1.56 1.13 1.05 1.20 0.66 0.77 | 1.02 1.06 0.84 1.26 1.51 1.29 1.14 1.18 0.92 0.84 |
| theta_3 | 1.07 0.97 0.78 1.35 1.75 1.38 1.12 1.16 0.57 0.77 | 1.00 1.00 0.86 1.21 1.50 1.32 1.11 1.16 0.88 0.86 |

## 3. B2, P2 (information only)
### Real cost per round (P2; 3-D RMS in cm, mean of the 4 tuning flights)

| Gain set | real 3-D RMS cm | z RMS cm | failed flights (tilt or floor) | flight seeds | minutes (round) |
|---|---|---|---|---|---|
| theta_0 | 19.28 | 18.04 | 1 (A1) | [100, 101, 102, 103] | 19.2 |
| theta_1 | 9.77 | 6.17 | 0 | [110, 111, 112, 113] | 18.1 |
| theta_2 | 7.62 | 5.06 | 0 | [120, 121, 122, 123] | 18.1 |
| theta_3 | 7.60 | 5.04 | 0 | [130, 131, 132, 133] | 18.0 |
| theta_4 | 7.70 | 5.20 | 0 | [140, 141, 142, 143] | 18.2 |
| theta_5 | 8.21 | 6.17 | 0 | [150, 151, 152, 153] | 0.1 |

Keep-best (lowest real cost over the flown sets): theta_3 (7.60 cm).

Gain-limit hits (projected Adam steps of 51 evaluations per gain; order Pxy Pz Dxy Dz Iz KRxy KRz Kwxy Kwz KIxy):

- tuning after round 0: noise seeds [6000, 6001], hits [0, 2, 4, 0, 26, 0, 0, 0, 0, 10], best epoch 50
- tuning after round 1: noise seeds [6002, 6003], hits [6, 6, 0, 0, 6, 8, 0, 0, 0, 0], best epoch 45
- tuning after round 2: noise seeds [6004, 6005], hits [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], best epoch 0
- tuning after round 3: noise seeds [6006, 6007], hits [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], best epoch 0
- tuning after round 4: noise seeds [6008, 6009], hits [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], best epoch 5

### beta error after each fit (relative error of each value against the P1 truth, %; no gate)

| Round | thrust gain | motor lag | downwash | plume |
|---|---|---|---|---|
| fit of round 0 | +0.9 | -32.5 | +5.2 | +21.6 |
| fit of round 1 | +16.2 | -19.8 | -8.8 | +44.4 |
| fit of round 2 | +9.1 | +7.6 | -7.0 | +12.0 |
| fit of round 3 | +6.3 | +4.9 | -11.8 | +13.4 |
| fit of round 4 | +7.0 | +4.5 | -11.9 | +17.9 |

### Final evaluation (real drone = frsim with noise, evaluation seeds 2000-2004; 3-D RMS cm / z RMS cm / failed flights of 5; mean over seeds)

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| theta_0 (default) | 35.55 / 33.70 / 5 | 11.85 / 9.63 / 0 | 6.58 / 6.32 / 0 | 6.05 / 5.71 / 0 | 9.44 / 7.65 / 0 | 13.18 / 12.10 / 0 | 5.20 / 5.02 / 0 | 15.01 | 9.27 |
| theta_best (round 3) | 13.40 / 8.49 / 0 | 5.61 / 4.19 / 0 | 3.10 / 2.34 / 0 | 3.06 / 2.09 / 0 | 8.40 / 6.63 / 0 | 11.61 / 9.43 / 0 | 2.93 / 2.54 / 0 | 6.29 | 7.65 |
| theta_5 (last flown) | 14.38 / 10.72 / 0 | 5.57 / 4.20 / 0 | 3.10 / 2.34 / 0 | 3.06 / 2.08 / 0 | 8.46 / 6.76 / 0 | 12.12 / 9.94 / 0 | 2.94 / 2.55 / 0 | 6.53 | 7.84 |

### Pass criteria (P2)

| Criterion | result |
|---|---|
| a. rounds 2-5 <= round 1 + 1 cm (round 1 = 9.77, limit 10.77) | **yes** (rounds 2-5: 7.62, 7.60, 7.70, 8.21) |
| b. no flown set after theta_0 fails (tilt or floor) a scenario that theta_0 or theta_1 passed | **yes** (none) |
| c. theta_best and theta_5 beat theta_0 on the tuning and the test scenarios (mean over 5 seeds) | **yes** (theta_best 6.29 / 7.65, theta_5 6.53 / 7.84, theta_0 15.01 / 9.27, tuning / test) |
| d. beta per round | reported above, no gate |

Floor / tilt rule per flown gain set (failed = tilt or floor):

| | A1 | A8 | CIRCLE | FIG8 |
|---|---|---|---|---|
| theta_0 | FAILED | ok | ok | ok |
| theta_1 | ok | ok | ok | ok |
| theta_2 | ok | ok | ok | ok |
| theta_3 | ok | ok | ok | ok |
| theta_4 | ok | ok | ok | ok |
| theta_5 | ok | ok | ok | ok |

## 4. Part C
- **C1** `usd_ingest.py`; test on the 10 S0 flights of 23 Oct (`lab/results/ingest_oct23.json`): all pass the gate, none crashed or on the floor. Reference difference after frame and lag: A1 0, A4 1.6–1.8 mm, A8 1.4–1.6 mm. Velocity does not sit on a noise floor (J_v 0.03–0.47 against 3e-5…7e-4). Gyro part is far off (open).
- **A8 lag** (±200 ms search): 82–120 ms (A4 40–58 ms). Cause: `merge_usd_logs.py --meta` aligns t = 0 by matching the position to the commanded trajectory, which absorbs the tracking delay. Fitted per flight, reported.
- **Logged top path** is the default for A1 / A8: model 10.1 -> 20.4–20.7 cm vs real 20.5–22.1 cm.
- **C2** `lab/round.py`. Dry test (4 frsim flights as fake merged files): **one round only (round 0), not two.** In the chain the dry test stopped with `NameError: t_round` after round 0 (`logs/chain.log` 13:48:20, `dry_test exit 1`; a first attempt failed earlier on a missing `_cffirmware` module because the wrong Python was used). I fixed the script, ran round 0 again by hand (14:08), and `lab/results/dry_test.json` comes from `--reuse` on that finished round, not from a run start to end. Result of that round: real cost 19.988 vs 19.988 cm; θ₁ and β identical to `loop_lib` (max difference 0.0, also the gain-limit hits); yaml round trip 2e-16; float32 gain error 4e-8. Desk time of that round: ingest 0.5 s, fit 568 s, tune 592 s, total 19.4 min (about 10 min in the last round). The dry test needs `strict=False` because `loop_lib` fits the floor flight of θ₀ on A1; the lab default repeats it. Stage 4b repeats the dry test over rounds 0 and 1 start to end (see `dryrun/stage4b/report_stage4b.md`).
- **C3** flight files within 0.003 mm of frsim; figure 8 turned by 90°; margins and peaks in the protocol draft. A1/A8/A4 files match the frsim reference.
- **C4** `docs/diffcotune_lab_protocol_S0_DRAFT.md` with open points.
- Spec section 13 and the inventory updated. `lab/README.md` written.

## Findings that were in the tables but not in the text (added in Stage 4b)
- **A4 cell 1, P1:** θ_best is worse than the default (9.48 against 8.80 cm). The test mean improves (9.02 -> 7.76 cm) only because of A4 cell 2 (13.05 -> 10.99) and the oval (5.21 -> 2.80). In P2 A4 cell 1 improves (9.44 -> 8.40).
- **P1 against Stage 3d A + B:** final tuning / test error 6.39 / 7.76 cm here against 5.60 / 6.89 cm there (Stage 3d θ_best, round 3; different evaluation seeds and a different run). Rounds 4–5 gained 0.4 cm in P1 (8.16 -> 7.77) and nothing in P2 (round 5 is 8.21 cm, keep-best is round 3).
- **P2 stall:** after rounds 2 and 3 the best iterate of the tuning step was epoch 0, so θ₃ = θ₂ and θ₄ = θ₃ (no change of gains; the real cost 7.62, 7.60, 7.70 cm is the same gains flown again with new flight seeds). Analysis in `dryrun/stage4b/report_stage4b.md`.
- **Printed `Jn`:** `loop_lib.py` (the two `log(...)` lines after the fit and after the tune) printed the normalised cost at the last epoch, not at the returned (best) iterate. Example P2 round 2: printed 1.534 (epoch 50); the best iterate is epoch 0 (1.000). Results are not affected (they use `J_best` and `best_epoch`). From Stage 4b both values are printed.

## 5. Proposal for the next step (your decision)
1. Fly a real round 0 of S0 (4 tuning flights) and run `round.py` on it: this tests the ingest and the lag on files from a tuning round, not only the 23 Oct ones.
2. Close the open points of the protocol draft first (flights per scenario, `rotate_deg` and command lines, how the yaml is applied, file names for `simple_flight.py`).
3. Check the gyro mismatch before using the gyro part.
4. Then the other controllers (S1–S3) with their own plant models.

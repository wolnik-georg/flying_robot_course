# P4: plant for S0-S3 (and S4) with joint calibration and gate, 2026-10-29

Plan: `docs/auto_tuning_plan_rev3_2026-10-28.md` section 5 and WP P4. Code: `experiments/tuning/` (`plant_core.py`, `plant_calibration.py`, `tree_gate.py`, `tilt_check.py`, `fbl_recheck.py`). Nothing flown; hardware data = uSD logs only.

## 1. Plant
Real firmware (`controller_omar_indi.c`, host bindings) in a rigid body: stock rate PID path, motor lag, quadratic thrust law (calibrated at hover) with a thrust cap, NS2 z-force downwash (scaled), fitted plume torque on the bottom drone (`fit_plume_torque.py`, 240 flights), gyro vibration, accelerometer noise, DShot-like rpm telemetry with delay, EKF-like state noise. Strategies: S0/S1 = `indi` 0/3, S2/S3 = NS2 on, Kpos_Iz 1.5; S4 = FBL (`inner 1`, preset 1, cd 0.55, network).
Two bugs had to be fixed before INDI flew at all (S1/S3 blew up in the first runs): (1) the position INDI reads `state->acc` (world frame, gravity excluded, in g), which the plant did not set; (2) the controller must be called at 500 Hz because the host `usecTimestamp()` advances 2 ms per call.

## 2. Calibration (not tuned on the gate data)
8 plant parameters, CMA-ES (`plant_calibration.py`), objective = mean of (plant - hardware)^2 / max(1.5 cm, 30 % of hardware)^2 (capped at 25) over **28 numbers**: the pilot cells of 2026-10-20 (S0-S3 x A1, A8, A4 x 3 offsets; dz 0.5, 0.5 m/s; 20 numbers) and the S4 cells of 2026-10-26 (A1, A8, A4 x 2; z metric and horizontal RMS; 8 numbers). Joint on purpose: one plant for all five controllers (fairness rule 3). Result (objective 1.78; 1.0 = typical error equals the tolerance):
| Parameter | Value | Bounds | Comment |
|---|---|---|---|
| downwash force scale `dws` | 1.47 | 0.6-2.0 | NS2 truth scaled up |
| plume torque amplitude scale `pa` | 2.58 | 0.5-5 | over the fit made on mostly calm flights |
| plume width scale `ps` | 1.21 | 0.6-2.0 | |
| motor lag [ms] | 22.2 | 20-60 | near the lower bound |
| gyro vibration [deg/s] | 10.2 | 1-20 | |
| accelerometer noise [g] | 0.091 | 0.005-0.15 | high |
| rpm telemetry delay [ms] | 17.6 | 0-20 | near the upper bound |
| thrust available at full command `fmax` [N] | 0.291 | 0.16-0.30 | at the upper bound |

## 3. Gate on the unseen block (2026-10-23, default gains, not used for the calibration)
Metrics as in the hardware analysis (A1 mean z error 2-15 s, A4 z RMS, A8 crossing dip), 3 noise seeds:
| Cell | S0 plant / hw | S1 plant / hw | S2 plant / hw | S3 plant / hw |
|---|---|---|---|---|
| A1 dz 0.45 [cm] | -10.4 / -19.7 | -1.9 / -0.9 | -9.2 / -7.3 | -1.5 / +2.5 |
| A4 off 0.075 [cm] | 7.6 / 17.4 | 3.5 / 6.0 | 3.8 / 11.3 | 3.9 / 2.9 |
| A4 off 0.125 [cm] | 5.1 / 9.2 | 4.7 / 2.2 | 3.3 / 4.1 | 3.6 / 2.0 |
| A8 dz 0.4 dip [cm] | -11.0 / -17.0 | -9.7 / -13.2 | -1.5 / -7.9 | -5.6 / -5.7 |
- **Pre-registered magnitude criterion (within max(1.5 cm, 30 %)): 6 of 16 cells. Not met.** Before any calibration: 4 of 16.
- Pairwise order of the four strategies agrees with hardware in **19 of 24** pairs (A1 6/6, A8 5/6, A4 4/6 and 4/6); before calibration 14 of 24.
- Tilt (A1, 2-15 s, rms of roll and pitch): plant S0 26.2, S1 9.7, S2 25.8, S3 8.7 deg against hardware S0 27.0-27.6, S1 4.7-8.7, S2 27.5-28.1, S3 6.7-7.3 deg: the large oscillation of the geometric strategies under the plume is reproduced.
- S4 in the same plant: z metrics A1 -0.1 (hw 0.0), A8 -1.2 (hw -1.2), A4 2.6 / 2.1 (hw 1.5 / 1.1); horizontal RMS A4 11.2 / 10.8 (hw 8.2 / 10.2) but A1 0.3 (hw 3.4) and A8 1.3 (hw 5.6).

## 4. Reading
- The plant **ranks** the strategies and reproduces the oscillation amplitudes; it does **not** predict magnitudes: geometric strategies (S0, S2) are under-predicted by a factor 1.5-2 in z, the A8 dip of S2 by 6 cm, and the horizontal error of FBL in A1/A8 is missing (the known feedforward gap, `fbl_xy_plant_gap_diagnostic_2026-10-28.md`).
- Several parameters sit at or near their bounds (`fmax`, `rd`, `accn`) and `dws`/`pa` are large: they absorb effects the plant does not model. They are plant-fit parameters, not measurements.
- Under the rules of plan rev3 section 5 the gate is **not passed** for any controller on magnitude. What the plant can still do is relative: compare gain sets of the same controller on the same cells (rank), with hardware confirmation of the result. This is a decision for you and the supervisor (section 5).

## 5. Options
1. **Tune in the plant, confirm on hardware** (rank-based use): gains found are ranked by the robust plant score, the top few are flown and the choice is made on hardware by the same rule for every controller. The plant's job is to find candidates, not to predict numbers.
2. **More plant work first** (S0/S2 magnitude, A8 feedforward gap) before any tuning: days, uncertain.
3. **Tune S0-S3 on hardware** with the same procedure (budget-limited), simulation only for FBL.
Recommendation: option 1, with the S0-gain-reuse variant (decision 8) as the first C-tree job.

## Update 2026-10-30: recalibration with the motor lag fixed at 30 ms
Lag 30 ms from the actuator identification (`tuning_actuator_id_2026-10-29.md`), 7 other parameters re-fitted (180 evaluations, same 28 targets). Calibration objective 1.40 (lag-22 plant: 1.78). Parameters: dws 1.34, fmax 0.276, gn 12.1, accn 0.084, rd 4.1 ms (was 17.6), pa 4.26, ps 1.03. `rd` now matches the 4-6 ms command delay from the data; none sits at a bound except accn region. File `out/plant_calibration_joint_lag30_best_s0.json`, plant of record from now on (`PLANT_CAL` selects the old one).

Gate on the unseen block (default gains, 3 noise seeds), `tree_gate.py --cal ... --tag lag30`:
| | lag-22 plant | lag-30 plant |
|---|---|---|
| magnitude within max(1.5 cm, 30 %) | 6 of 16 | 9 of 16 |
| pairwise order | 19 of 24 | 23 of 24 |
S0 magnitudes (plant / hardware): A1 -13.3 / -19.7, A4 12.3 / 17.4 and 8.1 / 9.2, A8 -13.1 / -17.0 (before: under-predicted by 1.5-2). Still outside: A1 S0, S1, S3, A4 S1/S3 at off 0.125, A8 S2/S3. The pre-registered gate is still not fully met.

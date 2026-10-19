# FBL track — status, plan and decisions (started 2026-10-18)

**What this is.** Strategy 3 of the study (flatness-based feedback linearization with a learned downwash residual, Hsieh et al. 2026). We received only the inference side from the authors (`docs/73_FBL_Bundle_Analysis.md`); the training script, the hardware checkpoint and the hardware gains are missing (email sent, waiting). Decision of 2026-10-18 (user): use the waiting time for desk work that does not need the lab — build our own training pipeline from the paper and our data, so that the authors' script later replaces only the training step. **DECIDED 2026-10-19 (user): Strategy 3 (FBL) IS part of the study.** The onboard port (3–4 weeks of desk + lab work, `PORT_SPEC.md`) is therefore committed work, no longer waiting for a scope decision; the authors' training script and hardware gains, when they arrive, replace only the fitted residual and the gains.

Code: `experiments/fbl/` (see its README). The authors' bundle stays read-only at `~/Desktop/learned_fbl_core`.

## Plan
| Stage | Work | State |
|---|---|---|
| 0 | Definitions: bundle residual vs our NS2 label, constants of our vehicle, ROM against our data, coverage | **done 2026-10-18** (results below) |
| 1 | Own training script: supervised fit of the bundle's residual structure (ROM + `Linear(2,9)–Tanh–Linear(9,3)` force network, optional 4-neuron force-only variant) on the 73-flight bank, leave-one-flight-out, baselines zero / ROM only | **done 2026-10-18** (results below) |
| 2 | Closed loop in simulation: the bundle's FBL controller with our residuals against a scripted top vehicle | **done 2026-10-18** (results below) |
| 3 | Onboard port (lookup tables, firmware mode, host tests, bench, lab validation) | not started; needs your and the supervisor's go |

## Stage 0 results (`experiments/fbl/out/stage0/`, `stage0_report.md`, `stage0_overview.png`)
- **Same meaning.** The bundle's residual is the acceleration the other vehicle causes on the ego vehicle in the world frame: a physics reduced-order model (ROM, downwash jet integrated over the vehicle disc, force along +z, divided by mass) plus a learned correction from two features (radial distance r0, vertical distance z0) rotated from the local flow frame into the world frame. Our label `a_res` = filtered measured acceleration minus the acceleration implied by the motor RPM (world frame): the same physical quantity plus everything else unmodelled (baseline far from the other vehicle −0.16 m/s² over the bank, ground effect, thrust-model error). Geometry identical to the bundle's `point_line_distance` (max difference 0 for the neighbour above; the bundle assumes the neighbour is the upper vehicle and treats z0 as unsigned).
- **Our vehicle** (`common/vehicle.py`): mass 0.0427 kg, arm 0.050 m, inertia 2.395e-5/2.395e-5/3.235e-5, MOTORRPM2FORCE 3.91e-8; the bundle's simulated vehicle is a 30 g Crazyflie (0.03 kg, arm 0.046, 1.43e-5/1.43e-5/2.89e-5, k_thrust 2.3e-8). The propeller radius is hard-coded in the bundle (45 mm); ours is assumed equal and still to be verified.
- **ROM only against our label** (downwash region 0.1 < z0 < 1.3 m, r0 < 0.6 m, 88 574 samples at 50 Hz, baseline removed): Pearson r = 0.76 for every drag coefficient; RMSE 0.564 m/s² at the best C_D = 0.45 against 1.059 for zero (R² ≈ 0.72), scale fit 0.96; the paper's hardware C_D = 0.236 under-predicts our push by about 1.8×, the simulation value 0.7 over-predicts by 1.6×.
- **The authors' sample network** (trained on a 30 g simulation plant, applied unchanged): r = 0.75, RMSE 0.576, scale 0.89 — about as good as the one-parameter ROM fit, so the structure fits our data without any training.
- **Shape mismatch to learn:** our label is flat in the vertical gap (about −1.0 … −1.5 m/s² for z0 between 0.2 and 1.15 m, r0 < 0.1 m) while the ROM falls off faster with distance (C_D 0.45: −3.1 at 0.15 m to −0.8 at 1.3 m); the network correction has to flatten this (consistent with the NS2 finding that the label hardly depends on dz).
- **Coverage:** seconds in the region per z0 bin [0.1–0.3, 0.3–0.45, 0.45–0.6, 0.6–0.8, 0.8–1.0, 1.0–1.3 m]: 296, 311, 446, 499, 190, 30; per r0 bin [0–0.05, 0.05–0.1, 0.1–0.2, 0.2–0.3, 0.3–0.45, 0.45–0.6 m]: 242, 452, 428, 141, 208, 300. The paper's hardware training used one separation (0.4 m, out-of-distribution 0.3 m) and 28 s; our bank is far broader.
- **Caveats:** all labels carry the bent-bottom-drone caveat; the label is the total unmodelled acceleration (the FBL residual is only the neighbour part), so Stage 1 removes a baseline or fits one; the label's x/y components are small and noisy.

## Stage 1 results (`experiments/fbl/out/stage1/v0/`, `report.md`, `fit_overview.png`; scripts `stage1_training/s1_fit_residual.py`, `s1_report.py`)
Supervised fit (weighted MSE, x/y weight 0.3, z 1.0) on label minus far-field baseline, region 0.1 < z0 < 1.3 m, r0 < 0.6 m, neighbour above; per-flight leave-one-out over 73 flights, 400 epochs. R² per held-out flight, z component, in the NS2 gate (r0 < 0.2 m):
| variant | gate R² median (p25 / p75) | % vs zero | flights ≥ 0.35 |
|---|---|---|---|
| zero | −1.23 | 0 | 0 / 73 |
| ROM only (1 parameter, C_D) | 0.39 (0.25 / 0.55) | 50 | 45 / 73 |
| net only, 9 neurons | 0.41 | 49 | 46 / 73 |
| ROM + net, 9 neurons | 0.41 (0.27 / 0.56) | 51 | 45 / 73 |
| ROM + net, 4 neurons (the paper's hardware size) | 0.41 | 51 | 45 / 73 |
- **The network adds almost nothing over the ROM** (0.39 → 0.41). C_D is stable across folds (0.43, range 0.426–0.434); all-data fit C_D 0.46.
- **Cause: a ceiling.** Any function of (r0, z0) alone reaches pooled R² ≈ 0.56–0.58 on our data; adding own height or relative speeds does not raise it. The rest of the label is not a function of these two features.
- **Against NS2 v2 on the same 73 flights (indicative only, different R² definitions):** FBL residual 0.41, NS2 0.79. Per scenario FBL is lowest on A7 (0.19), A1/A2 (0.28–0.30), highest on A3/A9 (0.61–0.62); NS2 is ≥ 0.6 everywhere except A6 (1 flight).
- **Reading:** the paper's two-feature structure explains about half of the downwash push on our vehicle; NS2 (full relative state, 3 neighbours, ground term) explains clearly more. For Strategy 3 this means the residual part is the weaker of the two learned models; first-order effect in closed loop is for Stage 2 to show.
- Not tested: KNODE trajectory loss (no FBL flights), torque residual.

## Stage 2 results (`experiments/fbl/stage2_sim/`, outputs `out/stage2/v2_ideal`, `v2_hw`, logs `log_v2_*.txt`; run `stage2_sim/run_all.sh v2`) — CORRECTED 2026-10-19
**Correction.** The first run (`v1`) used the bundle's residual class unchanged, which divides the ROM force by ITS vehicle mass (0.03 kg) while our force side was set to 0.0427 kg: the ROM acceleration was 1.42× too large for our vehicle (found while testing the lookup table). Fixed in `residual_providers.py` (`nominal_model.mass`), whole matrix rerun as `v2`; the `v1` outputs were deleted. Stages 0 and 1 divide by our mass themselves and are unaffected.
**Setup.** The authors' `FBLControl` (position law, acceleration estimator, `compute_fbl_command`, motor allocation) runs unmodified; only the residual is replaced and the vehicle constants set to ours. Plant: own rigid body (mass 0.0427, our inertia, arm 0.05, our thrust constant, 1 kHz). Bottom vehicle flies A8 (dz 0.5, span 1 m, 2 passes, peak speed 0.1 / 0.3 / 0.5 m/s) under a scripted top vehicle; controller at 500 Hz with the bundle's gains (56/38/48 position, 238 rate), unchanged. Truth downwash: **ns2** = NS2 v2 network (data-driven, z only, ground term) or **fit** = the Stage 1 ROM + net fit. Variants: none, ROM with the paper's C_D 0.236, ROM with our C_D 0.43, ROM + fitted net, NS2 as residual. Assumed, not measured: max rotor speed 2300 rad/s (T/W ≈ 2), motor lag 20 ms, yaw-torque ratio 0.006 m. "hw" = hardware position/gyro noise from the logs + 20 ms position delay (assumption).
Worst z dip in the gate [cm], mean of the three speeds (ideal / hw):
| residual in the controller | truth ns2 | truth fit |
|---|---|---|
| none | −2.4 / −2.7 | −2.1 / −2.4 |
| ROM, paper C_D 0.236 | −1.3 / −1.5 | −1.0 / −1.2 |
| ROM, our C_D 0.43 | −0.5 / −0.7 | −0.1 / −0.4 |
| ROM + fitted net | −0.6 / −0.8 | −0.2 / −0.6 |
| NS2 as residual | −0.16 / −0.5 | −0.4 / −0.7 |
- **Ordering holds:** no residual < ROM with the paper's coefficient < ROM with our fitted coefficient (about 80 % of the dip removed; the paper's value only about half).
- **The network adds nothing, if anything slightly worse than the ROM** (−0.6 vs −0.5 cm, truth ns2); same as Stage 1.
- **Not comparable to the hardware tree numbers** (S0 −15.7, S2 −5.5 cm, `docs/s3_indi_ns2_decision_2026-10-18.md`): FBL without any residual already dips only 2–4 cm here because its gains are far stiffer than Omar's (Kp 7, Kd 4) in an idealised plant. Whether those gains hold on the real vehicle is unknown (the authors' hardware gains are missing).
- **Fragility found:** at a 100 Hz controller the bundle's rate gain (238) is marginal in discrete time and a 10 ms position delay diverges (all variants, also without residual); at 500 Hz it is stable with 20 ms delay. A port needs ≥ 500 Hz rate loop or retuned gains.
- Nothing diverged at 500 Hz. Not tested: torque residual, the real thrust deficit seen at crossings in INDI, ground contact, 3 vehicles, real estimator.

## Stage 3 preparation (2026-10-19)
Port specification `PORT_SPEC.md`; ROM lookup table (60 × 60 × 3 floats = 42 KiB) built and tested against the authors' ROM (`experiments/fbl/stage3_port/`, `out/stage3/rom_table_v1/`): acceleration error ≤ 0.005 m/s², jerk p99 0.03 m/s³. **C residual module written and host-tested (G2 passed 2026-10-19, `stage3_port/c/`, `make test`):** gate, geometry, table lookup, fade, jerk, 1–3 neighbours, peer-velocity differencing; agrees with the Python reference to 7e-7 m/s² (acceleration) and 8e-6 m/s³ (jerk) over 3077 cases; builds for the ARM target (1.5 KB). Nothing in the firmware tree; the FBL law branch (G3/G4) and the rest need the go.

## Decisions and open points
- Our supervised fit replaces the paper's KNODE trajectory loss for now (we have no FBL flights); the KNODE loss can be implemented later or the authors' script swapped in.
- Open: propeller radius of our vehicle; torque residual (we have no torque label; the hardware network in the paper is force-only); how to treat the baseline (global vs per flight).

# Handover: make our FBL controller 1:1 with the authors' flown controller (2026-10-24)

## Context
- Master's thesis: comparative study of downwash-compensation strategies on two Crazyflie 2.1 Brushless drones (bottom drone **cf5** = study drone, top drone **cf_second**). Strategy **S4** is learned flatness-based control (FBL) after Hsieh et al. 2026. We ported it into the onboard firmware controller (`controller_omar_indi.c`, FBL branch active above 0.15 m, Omar's controller below).
- 2026-10-24: the first 12 FBL hardware flights. z tracking is good; x/y, yaw and stability are poor (y error 15–80 cm rms, yaw up to 66°, 3 of 12 latch trips).
- The analysis plus an independent review established the root cause:
  - **Our control law is the authors' code, but our inner loop is not the one they flew.** In their simulation the FBL controller outputs torque through a P-only rate loop (Eq. 19). On hardware the authors sent **thrust + body rates to the stock Crazyflie rate PID** (with an integral and D, yaw held by an integrated yaw setpoint). We flew Eq. 19, P-only.
  - On cf5, any slowly varying torque need then becomes a steady rate error. The jerk path turns that into a lateral error and a wound-up xy integral, and it explains the A4 / A8 y errors almost completely from the logs.
  - Yaw has no feedback, so it drifts.
- **Goal of this task:** make our FBL controller the same controller the authors flew: their code (bundle) plus their hardware configuration (paper Sec. 5.2). Then show it in host tests and simulation, so it is ready for a bench / hover test that the user runs. Where 1:1 is impossible, make the gap as small as possible, put it behind a parameter, and document it. **No retuning beyond what 1:1 requires.**

## Read first (in this order, completely)
1. `docs/fbl/HARDWARE_REVIEW_2026-10-24.md`: findings, with the review corrections merged. §6 is the difference table, §8 the 1:1 change list, §9 the open facts.
2. `docs/fbl/HARDWARE_REVIEW_2026-10-24_independent_review.md`: claim check with evidence (file:line, numbers). §b lists the missed items, §c the **final 1:1 change list (the work spec for this task)**, §d the open facts for the authors.
3. `docs/fbl/PORT_SPEC.md` (port spec, gates G1–G3), `docs/fbl/STATUS.md`, `docs/fbl/PROGRESS_AND_RESUME.md` (resume point; §4b), `docs/73_FBL_Bundle_Analysis.md`.
4. `docs/fbl/GAIN_TUNING_2026-10-20.md`, `docs/fbl/ROBUSTNESS_2026-10-21.md` (simulation plant: 4 ms delay, 44 ms motor lag; the claim "authors' gains unstable"); `docs/fbl/KNODE_TRAINING_*_2026-10-22.md` (our C_D 0.55 fit, NN training status); `docs/fbl/LAB_PLAN.md`, `docs/fbl/FIRST_FLIGHT_CHECKLIST.md`; `docs/lab_sessions/2026-10-24.md`.
5. `docs/07_Thesis_Progress_Checklist.md`: the single source of truth for the next action. Update it in the same change as any state change.

## Sources
- **Authors' bundle (read-only, the reference for "1:1"):** `~/Desktop/learned_fbl_core`. Guide `LEARNED_FBL.md`. Controller `simulator_multi_python/Controllers/fbl_control.py` (`compute_fbl_command`, `FBLControl._position_commands`, gains l. 132–149, yaw-torque zeroing l. 228–231). Estimator `simulator_multi_python/Controllers/acceleration_estimator.py` (`AlphaBetaGammaEstimator`). Residual `KNODE/Training/downwash_residual.py`, `residual_derivatives.py`, `nominal_dynamics.py` (runtime C_D 0.3, S 0.0997, vehicle 0.03 kg / arm 0.046), `Velocity_Field/force_batch_fo.py`, `torque_batch_fo.py`. NN `KNODE/Training/nn_models_full_only_zr.py` (Linear(2,9)→Tanh→Linear(9,3); the paper's hardware used 4 neurons).
- **Paper:** `docs/papers/hsieh2026flatness.pdf`. Eqs. 15–19 (Sec. 3.3); 5.1 simulation (100 Hz, ROM C_D 0.3, truth 0.7); **5.2 hardware**: CF2.1 + thrust upgrade, 36 g, Vicon 120 Hz → laptop → Crazyradio at 400 Hz, CrazyROS (crazyflie_ros), "thrust and angular velocity commands … rely on the high-frequency PID controller onboard", only residual forces compensated, ROM C_D 0.236, 4-neuron force NN, 28 s training data (static + stacked, 0.4 m, 0.4 m/s, FBL without NN).
- **Our C port (host-tested, the source of truth for the firmware copy):** `experiments/fbl/stage3_port/c/` (`fbl_law.c/.h`, `fbl_residual.c/.h`, `test_fbl_law.c`, `test_fbl_residual.c`, `gen_law_vectors.py`, `gen_test_vectors.py`, `Makefile`; `make test`, `make test_law`). The firmware copies are byte-identical today; keep it that way.
- **Firmware (local tree, never fork / push):** `~/Desktop/crazyflie-firmware`. FBL branch `src/modules/src/controller/controller_omar_indi.c`: config init around l. 84, `fbl_apply_preset` l. 131, `controllerOmarFbl()` l. 155–228 (`fbl_law_command` l. 199, `controlModeForceTorque` l. 206), param / log groups `ctrlFbl` l. 733–768.
  - Stock reference: `controller_pid.c` (rate mode, yaw-setpoint integration, `−gyro.y`, `control->yaw = −yaw`, `controlModeLegacy`), `attitude_pid_controller.c` (`attitudeControllerInit` / `CorrectAttitudePID` / `CorrectRatePID` / `GetActuatorOutput` / `Reset*`, int16 saturation), `utils/src/pid.c` (D on the measurement, i-limit), `power_distribution_quadrotor.c` (Legacy vs ForceTorque), `drivers/src/motors.c` (`motorsCompensateBatteryVoltage`, force-linear uint16), `platform_defaults_cf21bl.h` / `platform_defaults_cf2.h` (PID gains, THRUST_MAX, ARM_LENGTH, THRUST2TORQUE), `crtp_commander_rpyt.c`, `crtp_commander_generic.c`, `crtp_commander_high_level.c`, `estimator_kalman.c` (100 Hz prediction).
- **Patches (how firmware changes are preserved):** `flying_drone_stack/firmware_patches/` (README apply order; `fbl_integration.patch`, `fbl_gain_presets.patch`, `fbl_config_survives_init.patch`) and `flying_drone_stack/firmware_app/host/LOCAL_MODIFICATIONS.md`. Every firmware change must end up as a regenerated or new patch, checked with `git apply --check` on a clean checkout.
- **Simulation:** `experiments/fbl/stage3_port/c_in_the_loop_real.py` (real firmware controller via the host bindings `~/Desktop/crazyflie-firmware/build` / `cffirmware`; needs system `python3` 3.10, `make bindings_python`), `torque_bias_sim.py` (inner-loop variants emulated in the wrapper only), `scenario_matrix.py`, `gain_tuning.py`, `robustness_plots.py`.
- **Flight data (analysis only):** `experiments/logs/bank_fbl2_2026-10-24/`, `bank_fbl3_2026-10-24/`, `bank_fbl_2026-10-24/`, `bank_unseen_*_2026-10-23/`; raw uSD `experiments/logs/usd_raw/`; decoder `flying_drone_stack/tools/decode_usd_log.py`. Use complete uSD logs only, never radio CSVs.
- **Analysis scripts:** `experiments/analysis/fbl_hw_overview.py`, `fbl_replay_check.py`, `fbl_integral_windup.py`, `fbl_a4_roll_torque.py`, `review_fbl_thrust_coupling.py`, `review_fbl_yaw_budget.py`, `review_fbl_lateral_chain.py`.
- **Yaml (cf5 strategy branches):** `~/Desktop/crazyswarm2`, branches `study_FBL`, `study_FBL_kp`, `study_FBL_kr` (`crazyflie/config/crazyflies.yaml`, `ctrlFbl:` block). Note: `study_FBL` == `study_FBL_kp` since 14:35 on 10-24.
- Python: analysis with `~/.pyenv/versions/flying_robots/bin/python`; host bindings with system `python3`.

## Facts already established (do not re-derive; verify only if you change something they depend on)
- The law matches the bundle: `make test_law` ALL PASSED. On hardware, thrust = m (a_cmd − a_res + g)·z_b holds exactly (the flown mass, 0.0427 kg, is recovered from the logs).
- Stock rate PID in physical units, with battery compensation on, uint16 force-linear:
  - CF21BL roll/pitch kp 200 = 2.47e-3 N m/(rad/s) = kr 103 1/s on J 2.395e-5.
  - CF2 + upgrade kit kp 250 = 2.56e-3 N m/(rad/s) (equal within 4 %).
  - Yaw 120 = 4.8e-4 N m/(rad/s) = kr_z 15.
  - Flown: kr 59.5 / 29.75, kr_z 35.
- Stock sign / unit conventions:
  - The rate PID takes `(gyro.x, −gyro.y, gyro.z)`, so pitch rate desired = −ω_y; units are deg/s.
  - Yaw in modeVelocity is integrated into `attitudeDesired.yaw`, then the yaw attitude PID (6 / 1 / 0.35) → yaw rate PID; there is no yaw-rate feedforward.
  - `control->yaw = −yaw`, `controlModeLegacy`.
  - Legacy thrust = per-motor uint16: T_total = 4·u/65535·THRUST_MAX (CF21BL THRUST_MAX 0.2 N).
  - rpyt inverts the yaw rate, clamps thrust at 60000 and treats < 1000 as 0.
- EKF state updates every ~5.6 ms (not 500 Hz); mocap every 9.9 ms.
- Two inertia values for cf5 exist in the tree: FBL 2.395e-5 / 3.235e-5 (`experiments/fbl/common/vehicle.py`), Omar 1.657e-5 / 2.926e-5.
- ROM: our table (arm 0.05, m 0.0427) is 15 % stronger per unit C_D than the bundle constants. At A1 dz 0.45, our C_D 0.55 → 2.86 m/s² vs the authors' 0.236 → 1.06 m/s².
- The HLC fills position / velocity / acceleration / jerk and `attitudeRate.yaw = degrees(ev.omega.z)` (body ω_z, not ψ̇_ref; ≈ 0 for our plans).

## What NOT to do
- Never fly, flash or touch hardware. The user runs bench / hover / flights.
- Never modify `~/Desktop/learned_fbl_core`. Do not edit meeting docs or lab-session docs. Do not re-run build scripts over user-proofread docs.
- Do not fork or push the firmware repo. Firmware changes live in the local tree plus patches in `flying_drone_stack/firmware_patches/`.
- The existing FBL path (Eq. 19, force-torque) must stay **bit-identical** behind a parameter (default = old behaviour until the user decides otherwise). The Omar / S0–S3 paths must be untouched (prove it: same host-test outputs before and after).
- No gain retuning, no new invented safety rules or protocol rules. Keep the existing ours-only safety (Omar below z_on, tilt latch 60°, w_max, non-finite → trip) and document it as ours-only.
- Commit only when the user asks. Never add a Co-Authored-By trailer. Never hand-count diff structure; use `git apply --check`.
- Do not round a hypothesis up to a fact. Give V / S / H / U levels as in the review docs.

## Work packages (in order; each has a gate; stop and report if a gate fails)

### WP1: Stock rate-PID inner loop (independent review §c1; the core change)
- Add `ctrlFbl.inner` (uint8): 0 = current Eq. 19 force-torque (default, bit-identical), 1 = stock rate path.
- In `controllerOmarFbl()`, when `inner == 1`, after `fbl_law_command` (use its thrust and `w_des`, ignore its torque), reproduce exactly what `controller_pid.c` does in rate mode:
  - once: `attitudeControllerInit(1.0f/ATTITUDE_RATE)` (guarded by `isInit`; call it in `controllerOmarIndiInit`).
  - at FBL switch-on (inactive → active): `attitudeControllerResetAllPID(state roll, pitch, yaw)`, and set the yaw setpoint ψ_sp to the current yaw `state->attitude.yaw` (deg).
  - each tick: `ψ_sp = capAngle(ψ_sp + degrees(w_des_z)·dt)` (honour `yawMaxDelta` like `controller_pid.c`); `attitudeControllerCorrectAttitudePID(roll, pitch, yaw, *, *, ψ_sp, &rr, &rp, &ry)`; reset the roll / pitch attitude PIDs as `controller_pid.c` does in rate mode; `attitudeControllerCorrectRatePID(gyro.x, −gyro.y, gyro.z, degrees(w_des_x), −degrees(w_des_y), ry)`; `attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw)`; `control->yaw = −control->yaw`; `control->controlMode = controlModeLegacy`; `control->thrust = clamp(T,0,max)/(4·THRUST_MAX)·65535`.
  - Verify every sign against `controller_pid.c` / `power_distribution_quadrotor.c`; do not trust this text blindly.
- Gains: the stock CF21BL defaults (`pid_rate.*`, `pid_attitude.yaw_*` params, platform defaults). Do not copy them into ctrlFbl. Log what is used.
- Logging: keep `ctrlFbl.*`. Add the rate-PID outputs / integrators and ψ_sp only if the 96-variable uSD cap allows; otherwise report which existing variables to swap, and do not change the uSD config yourself.
- **Gate 1 (host):** a new host test (bindings or C) that feeds identical setpoint / sensors / state sequences to (a) stock `controllerPid` with `mode.roll/pitch/yaw = modeVelocity`, `attitudeRate = w_des`, thrust = the same uint16, and (b) `controllerOmarFbl` with `inner = 1`. Motor outputs must be identical (bit-exact or 1 LSB). Plus: `inner = 0` is bit-identical to the current firmware on the existing FBL vectors, and the S0–S3 paths are unchanged.

### WP2: Simulation with the real stock PID (gate before any hardware)
- Extend `c_in_the_loop_real.py` (or a new `stage3_port/stock_inner_sim.py`) to run `inner = 1` through the real firmware code. Legacy motor commands must map to forces with the same THRUST_MAX / arm / THRUST2TORQUE as the firmware.
- Plant: as in GAIN_TUNING (4 ms delay, motor lag 44 ms), plus a sweep of motor lag 15 / 30 / 44 / 60 ms, because the stock loop is ~1.7× stiffer than preset 1.
- Disturbances: none; a constant torque (+80 / −300 / −30 µN m); a slowly varying torque (sign flips over the flight, amplitude as in the S0 means of the review §a #29); A1 dz 0.45, A4 0.125, A8.
- Include the takeoff handover: start on the ground under Omar, FBL switch-on at 0.15 m, so the integral starts as on hardware.
- Run gain presets 1 and 0 (bundle "Deployed") with `inner = 1`.
- **Gate 2:** with `inner = 1`, preset 1 and the identified lag: stable in all scenarios; with the torque disturbance, yaw held (< 2°) and the lateral offset removed (< 2 cm steady). Report preset 0 stability honestly. If unstable at 44 ms, report the lag margin; do not retune.

### WP3: Remaining 1:1 items in the law / estimator (independent review §c4–c6)
- **Inertia:** find which value is right for cf5 (sources in the tree / docs; `docs/09_Simulation.md` is what GAIN_TUNING cites for the identified values). Report it; do not change defaults without the user's decision. It matters only for `inner = 0`.
- **α-β-γ:** optional `ctrlFbl.abg_on_new` (default 0): update only when the EKF velocity changed, dt = time since the last change. Host test against the bundle estimator on irregular sampling. This is closer to the authors (filter at the measurement rate); the rate they actually used is U.
- **Yaw-rate input:** keep `setpoint->attitudeRate.yaw` (HLC body ω_z), but document that the bundle expects ψ̇_ref; for our plans both are ≈ 0.
- **Gain preset 0** = the bundle "Deployed" values exactly; preset 1 = ours (unchanged).
- **Gate 3:** `make test`, `make test_law` and the new tests pass; the firmware copies of `fbl_law.c` / `fbl_residual.c` are identical to `experiments/fbl/stage3_port/c/`.

### WP4: Force residual like the paper's hardware (ROM + 4-neuron force NN)
- Prepare, do not fly: data-collection card (static and stacked formations, 0.4 m separation, 0.4 m/s, ≥ 28 s total, flown with FBL `inner = 1`, ROM on, NN off; also 0.3 m for the OOD check), using the existing scenario tooling.
- Training pipeline: KNODE as the bundle, net_f Linear(2,4)→Tanh→Linear(4,3), ROM + NN, our vehicle constants. The firmware NN slot must output world-frame acceleration via `flow_frame_to_world` (local x radial toward the neighbour, z = +world z) **plus its jerk** (analytic JVP as in `residual_derivatives.py`). Host-test the C against the bundle code with the same weights.
- C_D: identified on our vehicle by the paper's method (Chee et al. 2024 sys-ID) if it is reproducible from our logs; otherwise keep 0.55 and state why. Never copy 0.236 (different constants, see the facts above).
- **Gate 4:** C NN + ROM matches the bundle `DownwashResidual` (use_nn = True) on random states within the existing tolerances, with the sample weights and with our weights.

### WP5: Ready-for-hardware package (the user flies)
- Firmware image build instructions, patch files regenerated (`git apply --check` on a clean checkout, apply order in the README), host bindings rebuilt.
- Yaml: a new crazyswarm2 branch `study_FBL_stock` with `ctrlFbl.inner: 1`, preset 1 gains, cd as decided. Do not touch the existing branches. Check the yaml write order (the init-wipe lesson: config must survive init; `flying_drone_stack/tools/fbl_prime_controller.py`).
- Bench / hover / flight card update in `docs/fbl/FIRST_FLIGHT_CHECKLIST.md` and `LAB_PLAN.md`:
  - verify the controller slot really runs (`flying_drone_stack/tools/check_active_controller.py`), and check the uSD card file count and `flying_drone_stack/tools/check_usd_deck.py` before the block;
  - start the uSD log **before takeoff** (to see the FBL switch-on and the integral);
  - first hover A1, then A8, A4 0.075, A4 0.125;
  - same analysis scripts plus the `review_fbl_*` scripts as acceptance checks: yaw held, lateral chain residual small.
- **Gate 5:** everything reproducible from the docs; nothing flashed.

## Cannot be 1:1 (document, do not try)
- The authors ran FBL on a laptop with Vicon 120 Hz and 400 Hz radio commands; we run onboard at 500 Hz with the EKF (~180 Hz effective state). Both latency chains are mocap → PC → radio, but with different sizes.
- The vehicle: CF21BL brushless, 42.7 g, larger J, vs CF2.1 + upgrade, 36 g.
- Their firmware version and Kconfig, their N→uint16 map, their exact gains and NN weights (U until the authors answer).
- Test cases: A4 jet edge and A8 crossing are outside their tested regime.
- Ours-only safety: Omar below 0.15 m, tilt latch, w_max, thrust clamp, residual gate / fade.

## Open facts for the authors (keep the list in `HARDWARE_REVIEW_2026-10-24.md` §9 current; no email is sent by us)
Interface (crazyflie_ros `cmd_vel` / rpyt RATE?), signs, ω_z sent as the yaw rate?, firmware commit and Kconfig, rate-PID params, N→uint16 map and battery compensation, controller mass / J, hardware gains (the "Deployed" values?), velocity source and FBL / α-β-γ rate, start / stop procedure and integral reset, C_D 0.236 constants, NN weights / data, RMSE 3D or z, sim truth C_D 0.7 vs 0.55.

## Deliverables
1. Code: firmware change (`inner` param, optional `abg_on_new`), C / host tests, simulation script(s). All as patches plus repo files; nothing flashed.
2. `docs/fbl/STOCK_INNER_LOOP_2026-10-XX.md`: plots first, few bullets. What changed (file:line), gate results with numbers, the sign / unit table, simulation results (preset 1 / 0, lag sweep, torque disturbance, handover), remaining gaps with V / S / H / U.
3. Updates in the same change: `docs/fbl/HARDWARE_REVIEW_2026-10-24.md` §8 (mark done / open), `docs/fbl/STATUS.md`, `docs/fbl/PROGRESS_AND_RESUME.md` (new resume point), `docs/fbl/PORT_SPEC.md` (new gate for the inner loop), `docs/07_Thesis_Progress_Checklist.md` (next-action line), `flying_drone_stack/firmware_patches/README.md`, `LOCAL_MODIFICATIONS.md`.
4. A short report to the user: what was done, each gate pass / fail with numbers, the commands to reproduce, and the decisions needed from them (inertia value; preset 1 vs 0 for the first flights; C_D; whether `inner = 1` becomes the default).

## Reporting rules
- State facts with evidence (file:line, command, number). If a test fails, show the output.
- If something in the two review docs turns out wrong, say so plainly with the evidence, and correct the doc in the same change.
- Independently validate every result before reporting it (re-run, compare against the bundle). Never report a pass you did not run.

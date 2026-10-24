# Handover: independent review of the FBL hardware analysis (2026-10-24)

## Context
- Master's thesis, comparative study of downwash-compensation strategies on two Crazyflies (bottom drone cf5, top drone cf_second). Strategy S4 = learned flatness-based control (FBL) after Hsieh et al. 2026 ("Flatness-based control with physics-informed learned downwash residuals"), ported by us into the onboard firmware controller.
- 2026-10-24: first real FBL flights on hardware (12 with complete uSD logs). z tracking is good; x/y tracking, yaw and stability are poor. We analysed this and wrote **`docs/fbl/HARDWARE_REVIEW_2026-10-24.md`**. Read it first and completely; it is the subject of this review.
- Its main claims: (1) the control law equals the authors' code, but the inner loop is not the one they flew on hardware (they sent thrust + body rates to the stock Crazyflie rate PID; we close the rate loop with P only, Eq. 19); (2) with cf5's steady body torque this causes yaw drift, a steady lateral offset and an xy-integral wind-up that couples into thrust; (3) a list of what differs from the authors' setup (§6) and what a 1:1 replication needs (§8); (4) open facts (§9).
- The user's goal: **all facts correct and complete, and an exact list of changes needed so that our FBL controller is 1:1 the controller the authors flew**, before any retuning. Nothing may be overlooked.

## Sources
- Authors' code bundle (read-only): `~/Desktop/learned_fbl_core` (guide `LEARNED_FBL.md`; `simulator_multi_python/Controllers/fbl_control.py`, `acceleration_estimator.py`; `KNODE/Training/downwash_residual.py`, `nominal_dynamics.py`, `Velocity_Field/*.py`).
- Paper: `docs/papers/hsieh2026flatness.pdf` (Sec. 3.3 Eqs. 15–19, Sec. 5.1 simulation, Sec. 5.2 hardware, App. A).
- Our bundle analysis: `docs/73_FBL_Bundle_Analysis.md`; port spec and gates: `docs/fbl/PORT_SPEC.md`; status / resume: `docs/fbl/STATUS.md`, `docs/fbl/PROGRESS_AND_RESUME.md`; gain tuning in simulation: `docs/fbl/GAIN_TUNING_2026-10-20.md`; lab session: `docs/lab_sessions/2026-10-24.md`.
- Our C port (host-tested): `experiments/fbl/stage3_port/c/` (`fbl_law.c/.h`, `fbl_residual.c/.h`, tests, `Makefile`).
- Firmware integration: `flying_drone_stack/firmware_patches/fbl_integration.patch`, `fbl_gain_presets.patch`, `fbl_config_survives_init.patch`; applied in the local firmware tree `~/Desktop/crazyflie-firmware` (`src/modules/src/controller/controller_omar_indi.c`). Stock reference in the same tree: `controller_pid.c`, `attitude_pid_controller.c`, `crtp_commander_generic.c`, `crtp_commander_rpyt.c`, `crtp_commander_high_level.c`, `platform_defaults_cf2.h`, `platform_defaults_cf21bl.h`.
- Flight data: manifests `experiments/logs/bank_fbl2_2026-10-24/`, `bank_fbl3_2026-10-24/` (FBL), `bank_fbl_2026-10-24/` (same-day S0), `bank_unseen_*_2026-10-23/` (S0–S3 reference, strategy per flight in `experiments/results/unseen_block_2026-10-23/per_flight.csv`); raw uSD logs `experiments/logs/usd_raw/`, decoder `flying_drone_stack/tools/decode_usd_log.py`.
- Analysis scripts behind the doc: `experiments/analysis/fbl_hw_overview.py`, `fbl_replay_check.py`, `fbl_integral_windup.py`, `fbl_a4_roll_torque.py`; simulation `experiments/fbl/stage3_port/torque_bias_sim.py` (uses `c_in_the_loop_real.py`). Results in `experiments/results/fbl_hw_overview_2026-10-24/`, `fbl_replay_2026-10-24/`, `experiments/fbl/out/stage3/torque_bias_sim*.{json,npz}`.
- Python: analysis with `~/.pyenv/versions/flying_robots/bin/python`; the firmware host bindings (`~/Desktop/crazyflie-firmware/build`, `cffirmware`) need system `python3` (3.10).

## What NOT to do
- Never fly, flash or touch hardware.
- Never commit or push in any repo; never add a `Co-Authored-By` trailer.
- Do not modify `~/Desktop/learned_fbl_core`, the firmware tree, the patches, the C port, or `docs/fbl/HARDWARE_REVIEW_2026-10-24.md` itself. Write your own scripts to a scratch location or `experiments/analysis/review_*.py`.
- Do not edit the meeting docs or the lab session doc.
- Do not start retuning gains; this review is about facts and 1:1 fidelity.

## What to do
1. **Every factual claim in the review doc**: verify against the code, the paper or the data, and mark it confirmed / corrected / unverifiable. In particular:
   - §6 table, row by row (bundle vs paper hardware vs our port), including the stock-firmware facts (rate PID gains, yaw-rate → yaw-setpoint integration, which interfaces accept thrust + body rates and in which units).
   - §1 gain identification and the corrected block assignment (14:44 = preset 1, 14:54 = `_kr`).
   - §5 integral wind-up: the reconstruction of a_cmd from `ctrlOmarIndi.rpyd` (check the `quat2rpy` / Euler convention, `mcolumns`, the yaw used), the dI/dt = e check, and the thrust-coupling numbers.
   - §7 cause chain and evidence levels; the torque-bias simulation (is the wrapper's emulation of a rate integral and yaw-angle hold a fair stand-in for the stock PID? signs of gyro / torque / bias; plant constants).
2. **Port fidelity to the authors' code**, independently of our earlier gate G3: re-read `fbl_law.c` and `fbl_residual.c` line by line against `fbl_control.py`, `acceleration_estimator.py`, `downwash_residual.py`; re-run `make test_law` / residual tests in `experiments/fbl/stage3_port/c/` if possible. Check frames and signs (gyro units and axes, quaternion convention, R columns, flow frame, residual sign, mass used, inertia values, `desiredYaw` / `yaw_rate`, `dt`, estimator initialisation, integral reset at the FBL switch-on, thrust floor, `w_max` clamp, thrust clamp, residual jerk).
3. **What the analysis may have missed**: anything in the code, the paper or the logs that is not in §6 / §9, e.g. the effect of the handover (FBL active from 0.15 m during takeoff while the integral starts), gyro filtering, the peer feed (timestamps, rate, velocity by differencing), C_D choice (0.55 vs the paper's 0.236 vs our fit 0.43), battery / thrust model (our hardware delivers less thrust than commanded under INDI), the A4 jet-edge regime being outside the paper's test cases, the A1 xyz oscillation, the plan-shaped y error on A4.
4. **The 1:1 change list (§8)**: is it complete and correct? Specify concretely how FBL's thrust + body rates (+ yaw) would enter the stock rate path onboard so that it matches what the authors' setup did (which mode flags, which gains — CF2 vs CF21BL defaults, thrust units, rate-PID integral limits, reset behaviour), and what cannot be made identical. List open facts that only the authors can answer (§9) and add missing ones.
5. Where a claim can be settled by a quick computation on the logs or a short simulation, do it rather than reason about it.

## Deliverable
- `docs/fbl/HARDWARE_REVIEW_2026-10-24_independent_review.md`, short and structured like the review doc (tables, few bullets, plots only if they settle a point):
  1. Claim check: one table, every claim of the review doc → confirmed / corrected / unverifiable, evidence (file:line, script, number).
  2. Missed items: what is not in the doc but should be.
  3. The 1:1 change list, final version, with for each item: what to change, where in the code, possible or not, and why.
  4. Open facts for the authors.
  5. Your overall assessment of the root-cause story in one short paragraph.

## Reporting
- Give a confidence level per claim (verified / strong evidence / hypothesis / unknown). If something is inconclusive, say so; do not round a hypothesis up to a fact.
- If you find that a conclusion in the review doc is wrong, say so plainly and show the evidence; this project has been burned before by confident wrong conclusions.
- Report what you ran (commands, scripts) so the result can be reproduced.

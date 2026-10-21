# FBL controller (Strategy 3) — progress so far and how to pick it up again (state 2026-10-20 night: DESK PART CLOSED, development PAUSED by the user 2026-10-21; only lab work left)

**One paragraph.** Strategy 3 (learned feedback linearization, Hsieh et al. 2026) is part of the study (user decision 2026-10-19). Everything that can be done without the authors' training script and without the lab is done and tested on the desk: our own residual fit, a closed-loop simulation of the authors' controller on our vehicle, a lookup-table version of the physics model, the residual and the control law in C (each tested against the authors' code), an additive integration into the firmware controller (inert unless switched on), and an ARM candidate image that builds clean. **Nothing has been flashed or flown.** Update 2026-10-20 night: gain tuning in a realistic plant is done (the authors' gains are unstable with our 44 ms motor lag and stay as preset 0; our tuned gains are preset 1, the default), handover/latch/A1/A4 checked in simulation, bench/log/yaml tools prepared. **Only lab work is left: `LAB_PLAN.md`.** Details: `GAIN_TUNING_2026-10-20.md`.

## 1. Timeline (git, all pushed)
| Date | Commit | What |
|---|---|---|
| 10-18 | `fc2952b7` | FBL track started: folder structure, read-only access to the authors' bundle, our data loader, **Stage 0** definitions check |
| 10-18 | `a6ff725c` | **Stage 1**: own supervised fit of the residual on the 73-flight bank, leave-one-flight-out |
| 10-18 | `60ca3fd6` | **Stage 2**: closed loop of the authors' controller with our residuals (A8) |
| 10-19 | `a8b342a1` | Stage 2 **rerun after a bug** (bundle divided the ROM force by its own 30 g mass → 1.42× too large), port spec, ROM lookup table |
| 10-19 | `a45b1c45` | **C residual module** + host test (G2) |
| 10-19 | `41383cfd` | decision: FBL is definitely part of the study |
| 10-19 | `cfe5127d` | **C control law** + host test against the authors' code (G3) |
| 10-19 | `c8fe5f54` | **Integration patch** `fbl_integration.patch`, G4, C-in-the-loop, ARM candidate image |

## 2. What exists, where, and how it was checked
| Component | Where | Check |
|---|---|---|
| Authors' bundle (inference side only) | `~/Desktop/learned_fbl_core` (read-only, imported, never edited; analysis `docs/73`) | hash recorded in `experiments/fbl/README.md` |
| Our vehicle constants, data loader | `experiments/fbl/common/` | Stage 0 |
| Residual fit (ROM + 2-feature net) | `experiments/fbl/stage1_training/`, outputs `out/stage1/v0/` | leave-one-flight-out over 73 flights |
| Closed-loop simulation (authors' controller, our plant) | `experiments/fbl/stage2_sim/`, outputs `out/stage2/v2_*` | ideal and hardware-like (noise, 20 ms delay), 500 Hz |
| ROM lookup table (60 × 60 × 3 floats, 42 KiB) | `stage3_port/build_rom_tables.py`, `rom_table_lib.py`, `fbl_rom_runtime.py`, output `out/stage3/rom_table_v1/` | **G1** vs the authors' ROM: acceleration ≤ 0.005 m/s², jerk p99 0.03 m/s³ |
| Residual in C | `stage3_port/c/fbl_residual.{h,c}` | **G2**: 3077 cases vs the Python reference, worst error 2–3 % of the tolerance |
| Control law in C (Eqs. 15, 10, 19, position PID, alpha-beta-gamma estimator) | `stage3_port/c/fbl_law.{h,c}` | **G3**: 3000 command cases + 6 × 400 position-law steps vs the authors' code, worst error 23–69 % of the tolerance; 20 near-singular yaw cases not compared |
| Firmware integration | `flying_drone_stack/firmware_patches/fbl_integration.patch` (applied to the live firmware tree; pre-patch host build kept as `crazyflie-firmware/build_pre_fbl_2026-10-19`) | **G4**: `ctrlFbl.en = 0` → Omar output byte-identical (4 of 4 replay cases) |
| C-in-the-loop | `stage3_port/c_in_the_loop.py`, `out/stage3/c_in_the_loop_v1.json` | **G5 first step**: the real firmware controller via the host bindings reproduces the Python Stage 2 within 0.1 cm |
| ARM candidate image | `flying_drone_stack/firmware_app/build_artifacts/study_tree_v2_fbl_candidate.bin` (sha256 `a96ad225…`, 584 104 B, flash 57 %, RAM 76 %; **local only, build_artifacts is git-ignored**, recipe in §6) | builds clean with `-Werror`; NS2 v2 flash weights + FBL; not flashed, not bench-tested |
Spec and gate list: `PORT_SPEC.md`. Detailed results of every stage: `STATUS.md`.

## 3. Results in short
- **Same quantity:** the bundle's residual is what our NS2 label measures (acceleration the other drone causes). The physics model alone explains about 72 % of the push in the downwash region (best C_D 0.43–0.45; the paper's hardware value 0.236 is for another vehicle).
- **Own fit:** leave-one-flight-out gate R² 0.41 (ROM alone 0.39); NS2 v2 reaches 0.79 (different definitions). **The network adds nothing over the physics model**; any function of (horizontal offset, vertical gap) tops out at R² ≈ 0.57 on our data.
- **Closed loop (simulation, A8, dz 0.5):** no residual −2.4 cm, paper C_D −1.3, our C_D 0.43 −0.5 cm (about 80 % of the dip removed); the C controller in the same plant: Omar geometric −22.9 / −10.8 / −7.1 cm, FBL without residual −3.7 / −1.9 / −1.6, FBL with the ROM −0.60 / −0.34 / −0.41 cm at 0.1 / 0.3 / 0.5 m/s.
- **Not comparable with the hardware tree numbers** (S0 −12.7, S3 −4.0 cm): idealised plant, and the bundle's gains (56 / 38 / 48 position, 238 rate) are far stiffer than Omar's.
- **Fragility found:** at a 100 Hz controller the bundle's rate gain is marginal and a 10 ms delay diverges; at 500 Hz it is stable with 20 ms delay (the firmware runs the controller at 500 Hz).

## 4. How the FBL branch behaves in the firmware (`ctrlFbl.*`)
- `ctrlFbl.en = 0` (default): the Omar controller as before. `en = 1`: above `z_on` 0.15 m (off below 0.10 m) the FBL law replaces the position and attitude controllers; below that, after a tilt over `tilt_max` 60° (latched until landed) or on any non-finite value, the Omar controller runs, so takeoff and landing are Omar's.
- Parameters (all `ctrlFbl.*`): `en`, `cd` (0.43), `z_on`, `z_off`, `tilt_max`, `kp_xy/kp_z` (56/45), `kv_xy/kv_z` (38/21.6), `ki_xy/ki_z` (48/3), `kr_xy/kr_z` (238/140), `J_x/J_y/J_z` (our inertia 2.395e-5/2.395e-5/3.235e-5), `beta`/`gamma` (0.4/0.1), `w_max` (20 rad/s). Logs: `active`, `tripped`, `steps`, `faults`, `res_az`, `res_jz`, `thrust`, `acc_z`, `w_des_x/y/z` (radio log; the uSD config is not changed, 96-variable cap).
- Force-only residual, z only, upper drone treated as upright; thrust along the measured body z axis; no INDI, no NS2 in this strategy.

## 4b. Update 2026-10-20 night / 10-21 (read this first when resuming)
- **Gains:** the authors' gains = preset 0 (kept), ours = preset 1 (default: kp 14/11.25, kv 9.5/5.4, ki 12/12, kr 59.5/35); the authors' gains are unstable with our 44 ms motor lag in simulation. `GAIN_TUNING_2026-10-20.md` (figures: tuning, handover, A1/A4, final round).
- **Simulation-only comparison with the tree** (one plant, S1/S3 not possible there): `SIM_COMPARISON_2026-10-20.md`. FBL ≫ S0; S2 ≈ FBL in the ideal plant (S2 flattered by its perfect model); in a stress plant FBL stays at 3.6–5.4 cm while S0/S2 collapse (mostly the z-integral gain).
- **Lab prepared, untested:** `LAB_PLAN.md`; image `study_tree_v2_fbl_candidate_presets.bin` (sha256 `956479a7…`, local only), `fbl_gain_presets.patch`, `fbl_bench_check.py`, `usd_fbl_config.txt` + `restore_study_sd_config.py --fbl`, `fbl_log_check.py`, `lab_check_strategy.py` (FBL aware), crazyswarm2 `study/from_main_FBL.patch`.
- **Open:** authors' training script/gains (third preset when they come), takeoff height swing = test artefact (explained 2026-10-21), thrust-map error is the main hardware risk, first FBL uSD log = shakedown of decode/merge. A tried-and-reverted attempt to run FBL in the two-drone harness (`omar_ns2_closed_loop_sil.py`): the residual stayed 0 there, cause not found.
- **2026-10-21 desk round (`ROBUSTNESS_2026-10-21.md`):** peer feed measured (88 Hz, ~1 mm; latency unresolved) and swept — FBL insensitive, S2 degrades with noise; thrust map measured from hover logs (bottom drone needs 1.023× weight, top 0.965×) — the 12 % shortfall assumption was too pessimistic; comparison redone in the calibrated plant (matches hardware S0/S2 on A8/A4); C_D best ≈ 0.6 (try 0.43 then 0.6), ki_z 12–16.
- **2026-10-21 later (desk):** FBL on 19 more scenarios (A8 speeds × dz, A2, A3, A5, R1) lowest everywhere, no trips; parameter sweep (60 draws): 0 failures, margin thin only when motor lag ≥ 54 ms and delay ≥ 5.5 ms coincide (`ROBUSTNESS_2026-10-21.md` §5–6).
- **2026-10-21 (desk, last round):** lower-gain fallback tested (kr ×0.5 or xy position ×0.7 removes the tilt bursts, no visible cost) and battery sag (up to 15 % fade compensated by the z integral): `ROBUSTNESS_2026-10-21.md` §7–8, fallback values in `FIRST_FLIGHT_CHECKLIST.md`.
- **2026-10-21 (desk, final):** propeller radius ±10 % ≈ C_D ±10–13 % (offline, compensable with `ctrlFbl.cd`), ground effect up to 3× nominal changes nothing at takeoff/landing/hold: `ROBUSTNESS_2026-10-21.md` §9–10. All planned desk items are done; the list of what was not simulated is in §10 (A7 descent).
- **Plant:** `experiments/fbl/stage3_port/c_in_the_loop_real.py` (+ `gain_tuning.py`, `final_tuning.py`, `scenarios_check.py`, `sim_compare.py`).

- **2026-10-21 lab prep (desk only, untested on hardware):** `FIRST_FLIGHT_CHECKLIST.md` (expected `ctrlFbl.*` traces from the calibrated plant, what to change if they differ), `LAB_PLAN.md` (updated: preflight `--study FBL`, manifest template), `lab_preflight.py` FBL-aware, crazyswarm2 study gate accepts `--strategy FBL` (`study/study_manifest_FBL_candidate.json`, copy over the live manifest after flashing; the live manifest is stale: still `study_tree_v1.bin`), mock dry run of `fbl_bench_check.py` (`test_fbl_bench_check_mock.py`), synthetic FBL uSD log through decode/merge (`make_synthetic_fbl_usd_log.py`), `pilot_cell_summary.py` has `fbl_ran`/`fbl_tripped`, step time about 10 µs estimated (`c/bench_step.c`), peer-feed faults harmless (`peer_faults.py`).
- **Final desk round 2026-10-21 (all in `ROBUSTNESS_2026-10-21.md`):** 19-scenario matrix vs S0/S2 (FBL lowest everywhere, no trips), 60-draw parameter sweep (0 failures; thin margin only when motor lag ≥ 54 ms and delay ≥ 5.5 ms coincide), tested lower-gain fallback (`kr_xy` 29.75 / `kr_z` 17.5, or `kp_xy` 9.8 / `kv_xy` 6.65 / `ki_xy` 8.4; live parameters; no visible cost), battery sag up to 15 % compensated by the z integral, propeller radius ±10 % ≈ C_D ±10–13 %, ground effect up to 3× harmless. Not simulated: A7 descent, touchdown speed, a lower default gain (no wind / xy disturbance in the plant).
- **PAUSED again 2026-10-21 by the user** (no thesis writing for FBL, only code/sim/test prep); resume with the lab steps or when the authors answer.

## 5. What is NOT done (and risks)
- **(2026-10-20: tuned in simulation, see GAIN_TUNING_2026-10-20.md; not yet the firmware defaults)** **Gains are the bundle's**, for a 30 g simulated vehicle; our vehicle (42.7 g) and the real loop (delay, noise, motor lag, thrust limit, gyro filter) are untested. **Next desk step: gain tuning in the C-in-the-loop simulation** with delay, noise, thrust limit and motor lag (`c_in_the_loop.py` has the plant; delay/noise/motor lag are not yet in it).
- **Handover at takeoff and landing** (0.15 m / 0.10 m) is new: the FBL integral and estimator start at zero, the Omar controller was flying until then; a transient is possible.
- **Never on hardware:** bench check, `ctrlFbl.en`, timing of the FBL step (expected tens of µs), behaviour with the real peer-position feed (velocity by differencing, 100 Hz).
- Limits are simple (rates 20 rad/s, thrust ≤ max, tilt 60°); near thrust ≈ 0 and near ±90° tilt the law is ill-conditioned (documented in `PORT_SPEC.md` §6b).
- C_D 0.43 is fitted on our label, which contains thrust-model error and ground effect; propeller radius of our vehicle is assumed 45 mm; no torque residual; the paper's KNODE trajectory loss is not implemented (no FBL flights yet).
- Waiting for the authors: training script, real-drone checkpoint and hardware gains (email sent; they would replace only the fitted residual and the gains).
- The uSD logs of FBL flights need the existing config (no new variables); the radio log carries `ctrlFbl.*`.

## 6. How to pick it up again
**Reproduce / rebuild**
- Host tests: `cd experiments/fbl/stage3_port/c && make test && make test_law` (expects `RESULT: ALL PASSED` twice).
- Table: `~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage3_port/build_rom_tables.py --tag <new tag>` (refuses to overwrite).
- Bit-identity gate after any change of the controller: `python3 flying_drone_stack/firmware_app/host/test_omar_ns2_off_identical.py --old-dir ~/Desktop/crazyflie-firmware/build_pre_fbl_2026-10-19 --new-dir ~/Desktop/crazyflie-firmware/build` (system python3; rebuild the host bindings first with `make bindings_python` in the firmware tree).
- C-in-the-loop: `CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build python3 experiments/fbl/stage3_port/c_in_the_loop.py --speeds 0.1 0.3 0.5 --tag <name>`.
- Re-apply the patch on another firmware tree: after the README order list, `git apply flying_drone_stack/firmware_patches/fbl_integration.patch` (last).
- ARM candidate image (the live firmware tree now contains the patch; build in a scratch copy so nothing in the repo is overwritten): copy `flying_drone_stack/firmware_app` (without `build`, `build_artifacts`) to `<scratch>/stack/firmware_app`, symlink `<scratch>/stack/tools` to `flying_drone_stack/tools`, then `cd <scratch>/stack/firmware_app && make DRONE=bl CF_CARGO_FEATURES='--features residual_nn_flash' CF_RNN_WEIGHTS_NPZ=<repo>/flying_drone_stack/tools/residual/weights/ns2_v2.npz all` → `build/cf21bl.bin`.

**Proposed order (to be confirmed by you) — steps 1 DONE 2026-10-20 night (see `GAIN_TUNING_2026-10-20.md`), the rest = `LAB_PLAN.md`**
1. Desk: gain tuning in the C-in-the-loop simulation (add measurement delay, noise, motor lag, thrust limit to the plant; retune the gains at 500 Hz; keep the bundle's structure); document the chosen gains; regenerate the candidate image.
2. Lab, bench: flash the candidate image on the bottom drone, `ns2c_bench_check.py --readback`, check `ctrlFbl.*` parameters and logs appear, `ctrlFbl.en` 0 → the drone must fly as S0.
3. Lab, first flight: yaml for the bottom drone with `ctrlFbl.en: 1` (new strategy "FBL" next to S0–S3; top drone unchanged), solo or stacked hover first, short, low, hand ready; read `ctrlFbl.active`, `steps`, `faults`, `tripped`, `thrust` from the radio log; then A1, then A8.
4. After flights: log check that FBL really ran (`ctrlFbl.steps` counts up, `active` 1), merge, compare in the same tables as S0–S3 (`docs/tree_comparison_2026-10-19.md` method), residual fit of C_D from FBL flights if useful.
5. When the authors answer: swap in their residual / gains, rerun G2/G3 vs their code.
Rules that still apply: count the `thesis` files on both cards before every block and archive at ≥ 85 (`lab_sessions/2026-10-19.md` §12); check the strategy of every block from the logs (the FBL check will be `ctrlFbl.steps > 0`, the Omar thrust model does not describe FBL).

## 7. Decisions so far
- FBL (Strategy 3) is part of the study (user, 2026-10-19).
- Own supervised fit replaces the paper's KNODE loss until the authors' script arrives.
- ROM only first (the fitted network adds nothing); force only, z only; no torque residual.
- Branch inside the Omar controller (controller 9) behind a parameter, not a new controller slot; handover below 0.15 m to Omar.
- Open for you: tolerance values of the gates G2/G3 (proposals used), the G5 band, the first-flight procedure, whether to implement the KNODE loss later.

## 8. Related documents
`PORT_SPEC.md` (architecture, gates, steps, open decisions D1–D6), `STATUS.md` (results of every stage), `docs/73_FBL_Bundle_Analysis.md` (what the authors gave us), `flying_drone_stack/firmware_patches/README.md` (patch order), `experiments/fbl/README.md`, `experiments/fbl/stage3_port/c/README.md`, `docs/meetings/2026-10-22.md` (FBL line: waiting for the authors).

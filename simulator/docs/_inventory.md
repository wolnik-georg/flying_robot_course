# Simulator package, Stage 1 inventory (read only), written 2026-11-10 (date 17:5x CET)

Scope: what the tuning simulator is today, as used for the current numbers (S0-S3 exact-inertia closing evaluations, FBL retune `fblv2o`).
Nothing was run, built, edited or fetched for this file. Line numbers refer to the files as they are on 2026-11-10 (`experiments/tuning/...` unless a path says otherwise).

Sections: 1 call chains - 2 model contents - 3 evaluation definitions - 4 external dependencies and absolute paths - 5 firmware - 6 bindings - 7 gains - 8 proposed mapping, bit-identity risks - 9 open questions.

---------------------------------------------------------------------------------------------------------------------------------------------

## 1. Call chain per strategy (numbers as currently used)

Common core for all strategies: `core.py` (Job, base_plant, episodes, score_episodes, SEARCH_VARIANTS, FULL_VARIANTS) -> `P.run(cfg)` of the plant module that is in `sys.modules['plant_core']` -> `c_in_the_loop_real.py` (`C`: firmware import, `kin`, `FSC`, `W`, `PEER_EVERY`) -> host bindings `cffirmware` (CFFIRMWARE_DIR). `core.py:12` does `import plant_core as P`, so every chain first puts the wanted plant module into `sys.modules['plant_core']` (import order matters for pickling in `multiprocessing.Pool`; `rank_final_md3o.py:13` documents the PicklingError).

Interpreter for everything: system python 3.10.12 (`/usr/bin/python3`; numpy 2.2.6, scipy 1.15.3; the bindings are a cpython-3.10 `.so`). `~/.pyenv/.../flying_robots` python is used only for stats/tables (no firmware).

| Strategy | Evaluation of record | Entry | Plant module (reference) | Job / cfg (`extra`) | CFFIRMWARE_DIR |
|---|---|---|---|---|---|
| S0 geometric | `out/closing_exactJ/eval_s2_<sc>.json` key `ref_s0` | `diff/eval_ns2_pick_o.py` (FAMILY=s0) -> `run_job.JOBS['s0q_<sc>']` | `plant_core_o.py` (standard, not fast) | `extra=dict(J='omar', tree=True, indi=0, ns2=0)` + `tree_map=_s0_map` | `~/Desktop/crazyflie-firmware/build` |
| S2 = S0 pick + NS2 | same file key `ns2_s2` | same script, job `s2q_<sc>` | `plant_core_o.py` | `ns2=1` instead of 0, same gains | same |
| S1 INDI | `out/closing_exactJ/eval_s3_<sc>.json` key `ref_s1` | same script (FAMILY=s1) -> `indi_jobs_o.JOBS['s1q_<sc>']` | `plant_core_indi_o.py`, or `plant_core_indi_fast_o.py` if `out/plant_indi_fast_accepted` exists (it does; env S1_SLOW unset) -> the FAST copy was used; `S1_PLANT=old` set by `diff/exactJ_eval_chain.sh` | `extra=dict(J='omar', tree=True, indi=3, ns2=0)` (`indi_jobs_o.plant_extra()`) | same |
| S3 = S1 pick + NS2 | same file key `ns2_s3` | job `s3q_<sc>` | same as S1 | `indi=3, ns2=1` | same |
| FBL (S4) retune | `out/fblv2o_<sc>_<method>_s<seed>.jsonl`, `out/diff_fblv2o{M,V}_<sc>_seed<k>.json`, ranks `out/rank4o*_fblv2o_<sc>.json` (still being produced) | `run_job_fbl_v2o.py` -> `fbl_jobs_v2o.JOBS['fblv2o_<sc>']`; ranking `diff/rank_final_fblv2o.py`, `diff/rank_final_md3o.py` | `plant_core_fast_o.py` (FAST; gate flag `out/plant_fast_accepted_fblv2o` exists), `plant_core_o.py` if flag absent or FBL_SLOW set | `extra=dict(J='omar')`, non-tree, `inner=1, preset=1, nn` default 1, cd 0.55 | `~/Desktop/fbl_v2_firmware/fw/build` (asserted in `fbl_jobs_v2o.py:7`) |

Plant copies and identity (design: each copy = parent + stated change; fast copies are bit-identical by test):
- `plant_core.py` (289 lines): original (rounded J_OMAR 1.657e-5 / 1.666e-5 / 2.926e-5); used for all tuning/search runs before 2026-11-09 and the old rankings `rank4*`, `rank5b`.
- `plant_core_o.py` (290): only J_OMAR exact (16.571710e-6 / 16.655602e-6 / 29.261652e-6) and `ctrlFbl.J` set to the same values (cfg `ctrlJ`, default 'omar'; `ctrlJ='fw'` keeps firmware default). Verified by diff: 3 differing places.
- `plant_core_fast.py` / `plant_core_fast_o.py` (305 / 306): bit-identical to `plant_core(_o).py` (kin memoised, one Rotation per tick, elementwise cross/solve for the diagonal J, no trace file, early stop once a scored sample has tilt >= crash threshold, so rms fields of crashed episodes are invalid but `stable` is the same). Tests: `diff/check_plant_fast.py` (old), `diff/check_plant_fast_v2.py`, `diff/check_plant_fast_v2o.py` (FBL cells x 5 variants x 4 gain sets + 6 S0 tree episodes, `==` on every field the scoring reads; writes the flag files). `plant_core_fast_o` vs `plant_core_o` is checked by `check_plant_fast_v2o.py` (flag `plant_fast_accepted_fblv2o`, 18:30 on 9 Nov, 0 mismatches).
- `plant_core_indi.py` (294): `plant_core.py` + options that are OFF by default (`accl`, `rd_indi`, `thrust_law`, `trace`). Identity test: `diff/check_indi_option_identical.py` (options off == `plant_core`, full trajectories) and `check_indi_option_identical_v2.py`. `plant_core_indi_o.py` (294) = that with exact J.
- `plant_core_indi_fast.py` / `plant_core_indi_fast_o.py` (310): fast copies of the INDI plant. Test: `diff/check_indi_fast_identical.py` (S1/S3, thrust law on/off, rd_indi, cells A1/A4/A8, writes `out/plant_indi_fast_accepted`, 6 Nov 02:21). **Gap: `plant_core_indi_fast_o.py` has no identity test of its own** (the flag was written for the rounded-J pair; the `_o` pair differs only by the J constants and one `ctrlJ` line; `indi_jobs_o.py` still reads the old flag).
- Others not used for the current numbers: `plant_core_indi_tr.py` (extra trace entries, indi_cells_task_a.py), `plant_core_indi2/3/_dl.py` (S1 fix rounds).

Does the plant module of a strategy differ between S0 and S1? Yes: S0/S2/FBL use `plant_core*`, S1/S3 use `plant_core_indi*`; with options off the INDI plant equals `plant_core` (identity test above), so one `frsim` plant with optional features can replace all of them, provided the options keep their defaults.

cfg keys actually set for each chain (everything else = default of `plant()` / `run()`):
- Built by `core.base_plant(extra, J)` (`core.py:19-23`): `plant()` defaults `J='fbl'`, `inner=1`, `preset=1`, `lag=30`, `quad=1`, `dist='none'`, `plume=(A,sigma,n)` of `out/plume_fit.json`, `gn=8.0`; then overridden by the calibration `out/plant_calibration_joint_lag30_best_s0.json` (`dws 1.34175, fmax 0.275804, lag 30.0, gn 12.0755, accn 0.0839628, rd 4.05015`) and plume scaled by `pa 4.25660`, `ps 1.02931`; then `extra` (J='omar' etc.), then the variant dict (`md`, `ffs`, `lag`, `gn`, `kt`). Then `episodes()` adds `scen, sc, gains, seed=0` (+ `tree_gains` for tree jobs) and ranking stages overwrite `seed`.
- Tree (S0-S3): `tree=True, indi=0|3, ns2=0|1, tree_gains={'Kpos_P.x': ...}`; `omar.Kpos_I.z = 1.5` is set in `run()` before `tree_gains` (so Iz tuned overrides it); `omar.fbl.en = 0`.
- FBL: `gains={'kp_xy': .., 'kv_xy', 'ki_xy', 'kp_z', 'kv_z', 'ki_z'}` set on `omar.fbl.*` after `PRESET[1]` (`kp_xy 14.0, kp_z 11.25, kv_xy 9.5, kv_z 5.4, ki_xy 12.0, ki_z 12.0, kr_xy 59.5, kr_z 35.0`); `omar.fbl.en = 1; inner = 1; nn_en = 1; c_d = 0.55; abg_on_new = 0`; `g_rnn_en = 0`.
- Defaults that apply silently: `hold=0` (state refreshed every 1 ms), `md=0`, `ffs=None`, `kt=1.0`, `dws` from calibration, `accn`, `rd`, `rpmn=0`, `plume` always on, `kappa=0`, `ad=None`, `top=None` (scripted top vehicle), `ctrlJ='omar'`.

---------------------------------------------------------------------------------------------------------------------------------------------

## 2. Model contents (values and source lines)

All in `plant_core_o.py` (same lines in `plant_core_fast_o.py` +15 offset, in `plant_core_indi_o.py` +1/+3 shifts) unless stated.

| Item | Value | Source |
|---|---|---|
| mass | M = 0.0427 kg (also `omar.mass = M`); G = 9.81 | `c_in_the_loop_real.py:26` (via `C.M, C.G`); `plant_core_o.py:24, 58` |
| inertia, plant body | J_OMAR = diag(16.571710e-6, 16.655602e-6, 29.261652e-6) when cfg J='omar'; J_FBL = diag(2.395e-5, 2.395e-5, 3.235e-5) when J='fbl' (default of `plant()`) | `plant_core_o.py:26, 55` ; all current jobs set J='omar' |
| inertia, controller | `omar.fbl.J` = same exact values (cfg `ctrlJ`, default 'omar'); no effect on thrust / rate setpoints with inner = 1 (check `out/fblv2o_ctrl_J_check.json`) | `plant_core_o.py:64` |
| arm geometry (mixer-independent) | ARM = 0.707106781 * 0.050 m for the torque lever, ARMS rotor positions (+-0.0354, +-0.0354) for the plume torque | `plant_core_o.py:25, 27` |
| THRUST_MAX, THRUST2TORQUE | THRUST_MAX = 0.2 N per motor (battery compensation assumed ideal); T2T = 0.00569278844371417 m | `plant_core_o.py:27` (comment: `platform_defaults_cf21bl.h`, bindings built with CONFIG_PLATFORM_CF21BL) |
| mixer path | `powerDistribution(ctl, u)` -> `powerDistributionCap(u, pwm)` of the firmware, `pwm / 65535 * THRUST_MAX` per motor; `wrench(F)`: thrust = sum F, torques with ARM / T2T (motor order m1..m4 sign pattern line 46) | `plant_core_o.py:40-46, 125-126` |
| command delay | 4 ms: command computed every 2 ms (500 Hz) and queued with `t + 0.004` | `:124-127` |
| motor lag | first order per motor, `a_m = dt / (lag_ms*1e-3 + dt)`, lag = 30 ms (calibration; variants lag 45 / 60) | `:73, 130` ; `core.py:19` ; calibration JSON |
| thrust law | `F_t = 0.1098 * (F_c / (0.2 * 0.618))**2` (cfg `quad=1`, default of `plant()`), then `min(F_t, fmax)` with fmax 0.2758 N | `:128-129` |
| INDI thrust law option | `thrust_law=(a,b)` -> `F_t = a + b F_c`; default OFF in all current runs (`S1_PLANT=old`) | `plant_core_indi_o.py:131` |
| integration | dt = 1e-3 s explicit Euler for v, x; w update with `solve(J, tau - w x Jw)`; q via rotvec of `R w dt` | `:73, 149-153` |
| state delay / hold | `md` (ms) = FIFO of the noisy measurements (1 ms samples), variants md 10/40/60; `hold` option (4-8 ms hold) default 0 | `:96-105` |
| sensor noise | position N(0, 0.0005 m), velocity N(0, 0.005 m/s); gyro N(0, gn) deg/s with gn from calibration 12.0755 (search variant 1.7x); accelerometer noise N(0, accn=0.08396 g) (tree only); rpm noise `rpmn` default 0 | `:97, 108, 111, 116` ; `core.py:19` |
| RPM telemetry (tree) | `rpm = sqrt(max(F,0)/3.91194027330774e-08) * 60/(2 pi)`, delayed by `rd` ms (calibration 4.05 -> `int(4.05)+1` samples FIFO) | `:115-120` |
| accelerometer (tree) | specific force in body frame `Rb^T (a_prev + g e_z)/G`; state->acc world frame minus gravity | `:110-114` |
| controller rate | tree: every 2 ms (host `usecTimestamp()` advances 2 ms per call); FBL: every 1 ms | `:121` |
| downwash truth | NS2 v2 network, z only: `a_nn = C.reference(C.W, rel, z, v, M)`, `fext = a_nn * M * dws` (dws 1.34175), updated every 2 ms, only when the top drone is above; weights `flying_drone_stack/tools/residual/weights/ns2_v2.npz` (key `weights`, 121,828 bytes, git acbffca6); python reference = `flying_drone_stack/firmware_app/host/test_residual_nn.py:121` (numpy, no firmware needed but the module imports `cffirmware` at import) | `:133-136` ; `c_in_the_loop_real.py:35` |
| plume torque | fitted Gaussian per rotor: `dF_k = -Ap (0.4 / max(dz, 0.05))**n exp(-rho^2/(2 sigma^2))`, torque `(rk_y dF, -rk_x dF, 0)`; Ap = 0.0537218 * pa(4.25660), sigma = 0.0608042 * ps(1.02931), n = 0.601427 (`out/plume_fit.json`, `plume_calibration` JSON) | `:137-142` ; `core.py:21-22` |
| top vehicle | scripted: `C.kin(sc, 1, max(ts,0), anchor)` (poly4d `_derivs` of the scenario's robot 1, anchor (0,0,0.5)); option `top` = logged trajectory (columns t,x,y,z,vx,vy,vz) built by `top_from_log` (only for the hardware validation; reads `experiments/logs/bank_fbl_b1_ours_2026-10-26/...`) | `:76-80, 179-194` |
| peer feed (FBL and NS2 in C) | `oot_set_peer_count(1)`, `oot_set_peer(0, x, y, z, i)` every `PEER_EVERY = 10` ms with the exact top position | `:63, 81` ; `c_in_the_loop_real.py:25` |
| FBL residual | firmware side: ROM + trained 4-neuron net (`fbl_nn_weights.c`: `knode_s0977_h4.pth`, sha256 630683f816a538e2, tag `..._cd0.55`), `c_d = 0.55` set from cfg `cd`, `nn_en = 1`, `dw_en = 1` (default). The fbl_v2 firmware has no `fbl_rom_table.c` (authors' ROM evaluated in `fbl_residual.c` / `fbl_law.c`; the older firmware tree still has the table file) | `plant_core_o.py:65-66`; `~/Desktop/fbl_v2_firmware/fw/src/modules/src/fbl_*.c` |
| NS2 in C (S2/S3) | `fw.cvar.g_rnn_en = ns2`, `g_rnn_div = 10`; weights compiled from `flying_drone_stack/firmware_app/ns2_weights.c` (v2 export; v1 kept as `ns2_weights_v1.c`) | `:66` ; `bindings/setup.py` oot_sources |
| crash / stable rule | tree (S0-S3): stable = completed and tilt_max < 70 deg (`TREE_CRASH_TILT`); FBL: stable = completed and not tripped and tilt_max < 45 deg and fbl_active > 99 %; score = 100 if not stable | `:49, 173-174` ; `core.py:44-46` |
| episode timing | T_TO 3.0 s smoothstep takeoff from the ground to the scenario start, T_HOLD 1.5 s, scenario time T_sc = min duration of the robots, T_LAND 3.0 s smoothstep landing, T_GND 1.0 s; scoring window `2.0 s < ts <= T_sc` (ts = time since scenario start) | `:57, 159` |
| feedforward | `ffs` scales acceleration and jerk setpoints (variants 0.4 / 0.0); `kt` scales the lateral thrust components (variant 0.5) | `:94, 149` |
| gyro / acc units | `sens.gyro` deg/s, `sens.acc` in g, attitude from `Rot.as_euler('xyz')` with the legacy pitch sign | `:106-108` |

Calibration provenance: plant of record since 2026-10-30 = `out/plant_calibration_joint_lag30_best_s0.json` (env PLANT_CAL can replace it; `core.py:15`); fit scripts `plant_calibration.py`, `plant_calibration_indi.py`, `fit_thrust_law.py`, `actuator_id.py` (docs `docs/tuning_plant_p4_2026-10-29.md`, `docs/tuning_actuator_id_2026-10-29.md`). `out/plume_fit.json` came from `fit_plume_torque.py` (233 flights, 6214 samples).

---------------------------------------------------------------------------------------------------------------------------------------------

## 3. Evaluation definitions

Tuning cells (never study cells), `run_job.py:23-29, 56-57` and `plant_core_o.py:199-205`:
- `S0_CELLS_V2` (used by s0q/s1q/s2q/s3q and fblv2o): A1 = {dz 0.375, dz 0.75} (hold 15); A4 = {dz 0.375 offset 0.05 speed 0.2; dz 0.5 offset 0.1 speed 0.3; dz 0.75 offset 0.15 speed 0.4} (axis 'y', motion 'line'); A8 = {dz 0.375, duration 8.0; dz 0.75, duration 5.0} (span 1.0, settle 2.0, passes 2).
- Study cells `STUDY_CELLS` (`plant_core_o.py:205`): A1 dz 0.45 hold 15; A4 dz 0.4 offset 0.125 / 0.075, speed 0.35 (line, axis y); A8 default scenario `A8(dz=0.4, span=1.0, duration=6.25, settle=2.0, passes=2)` (cell setting None -> `scenario('A8')`). Mapping per scenario in all rank scripts: A1 -> cell [0], A4 -> cells [1:3], A8 -> cell [3].

Variants (`core.py:25-28`, `eval_ns2_pick_o.py:19`, `rank_final.py:19`):
- SEARCH_VARIANTS (during search, objective deterministic seed 0): `{}`, `gn*1.7`, `md=40`, `ffs=0.4`; the jobs add `lag=45` (5 variants).
- FULL_VARIANTS (selection stage, 9): the 4 above + `lag=60`, `md=60`, `ffs=0.0`, `kt=0.5`, `ffs=0.4 + md=40`.
- Study variants: `nominal`, `ffs0.4`, `md10`, `md40`, `md60` (`eval_ns2_pick_o.py`; `rank_final.py` has nominal/ffs0.4/md40/md60, md10 is added by `rank_final_md*.py`).

Scores:
- Episode score = 100 if not stable else sqrt(x_rms^2 + y_rms^2 + z_rms^2) in cm (3-D RMS over the scoring window), `core.score_episodes`. (`plant_core.score()` of the older ladder code uses x,y only; not used by the current numbers.)
- Search objective = mean over (variants x tuning cells) of episode scores, noise seed 0 (`core.evaluate_batch`).
- Selection error (`sel_score`) = tuning cells x 9 FULL_VARIANTS x noise seeds `SEL_SEEDS=[0,1,2]`: mean over seeds of the mean over episodes; `sel_crashes` counts episodes with score >= 100 (`rank_final.py:run_stage`, `eval_ns2_pick_o.py:stage`).
- Test error (`study_<variant>`) = per variant, per noise seed in `STUDY_SEEDS=[0..4]` the mean over the study cells; the closing table uses the mean over the 5 seeds (100 = crash).
- Pick rule: `diff/closing_picks.py`: lowest median `sel_score` among cmaes/bo/bo2/sobol/gradM, then best seed. The picks were made on the OLD (rounded-J) ranking files; `closing_exactJ` re-scores the same gains with exact J.

Files: `core.py` (cells/variants/scores), `plant_core*.py` (cell lists, `STUDY_CELLS`), `run_job.py` (`S0_CELLS_V2`, `S0_PARAMS`, jobs), `diff/rank_final*.py` (selection+study), `diff/eval_ns2_pick_o.py` (closing evaluation), `diff/closing_table_v3.py` (table).

---------------------------------------------------------------------------------------------------------------------------------------------

## 4. External dependencies and hard-coded absolute paths

crazyswarm2 (`/home/georg/Desktop/crazyswarm2`, read only): branch `study_FBL_stock`, HEAD `aa6034c2d84257872440a596efe7b2318d664b71`; tracked files unmodified (only untracked A4 data CSVs); `formations/` last commit `e048019` (New scenario A9), contained in `origin/main` and `origin/study_FBL` (checked with `git branch -r --contains`, no fetch).
- Used: `crazyflie_examples.formations.poly4d._derivs(curve, t)` (195-line file, numpy only), `formations.scenarios.A1/A4/A8` (931-line file; imports numpy, dataclasses, `curves.py`), `curves.py` (294 lines, numpy). `formations/__init__.py` also imports `safety` (190 lines). `crazyflie_examples/__init__.py` is empty. None of the four files imports rclpy; the `MagicMock` of `rclpy`/`rosgraph_msgs` in `c_in_the_loop_real.py:11` is therefore only a precaution (to be verified when vendoring).
- Scenario objects used through: `sc.robots[i].curve.duration`, `.curve`, `.slot`; `C.kin(sc, i, t, anchor) = anchor + slot + d[0,:3], d[1,:3], d[2,:3], d[3,:3], d[0,3]`.

Other external pieces:
- `flying_drone_stack/firmware_app/host/test_residual_nn.py` (`reference`, `forward`, layer constants, `OFF_*`, gate constants): imported by `c_in_the_loop_real.py:21`; at import it also does `sys.path.insert(0, '/home/georg/Desktop/crazyflie-firmware/build')` and `import cffirmware` (works because the bindings are already imported from CFFIRMWARE_DIR and cached; for the FBL chain this relies on import order).
- Weights/data: `flying_drone_stack/tools/residual/weights/ns2_v2.npz` (python truth); `flying_drone_stack/firmware_app/ns2_weights.c` (compiled-in NS2 for S2/S3 = same v2 values; identical copy in `fbl_v2_firmware/stack/firmware_app`); `fbl_nn_weights.c` (FBL net); `experiments/tuning/out/plume_fit.json`; `out/plant_calibration_joint_lag30_best_s0.json`; `peer_intervals.npy` (only the `peer` option, not used); logged flights for `top_from_log` (validation only, not used by the current numbers).
- Host layer C files linked into the bindings: `firmware_app/host/oot_host.c` (RPM and peer injection, log sinks; 341 lines), `firmware_app/traj_iface.c` (gains/globals, `g_rnn_*`; 833 lines), `firmware_app/ns2_weights.c`, and the Rust static lib `libcf_controller_rs.a` (see section 6).

Absolute `/home/georg` paths in the files used (grep):
| File | Path | Purpose |
|---|---|---|
| `plant_core_o.py`, `plant_core_fast_o.py`, `plant_core_indi_o.py`, `plant_core_indi_fast_o.py` (4 files) | `.../experiments/fbl/stage3_port` (line 21) | import `c_in_the_loop_real` (the plant core depends on a file in `experiments/fbl/`) |
| same 4 files | `.../experiments/logs/bank_fbl_b1_ours_2026-10-26/...` (`top_from_log`) | validation only |
| `experiments/fbl/stage3_port/c_in_the_loop_real.py` | `/home/georg/Desktop/crazyflie-firmware/build` (default of CFFIRMWARE_DIR), `/home/georg/Desktop/crazyswarm2/crazyflie_examples` | firmware + scenarios |
| `flying_drone_stack/firmware_app/host/test_residual_nn.py:57` | `/home/georg/Desktop/crazyflie-firmware/build` | firmware import |
| `bindings/setup.py` (both firmware trees) | `~/Desktop/flying_robot_course/flying_drone_stack/firmware_app` (default OOT_DIR, env `OOT_FIRMWARE_APP`); fbl_v2 build used `~/Desktop/fbl_v2_firmware/stack/firmware_app` | where the Rust lib / host C files are |
| `core.py`, `run_job*.py`, `optimize*.py`, `rank_final*.py`, `indi_jobs_o.py`, `fbl_jobs_v2o.py`, `eval_ns2_pick_o.py` | none (use `Path(__file__)`, env CFFIRMWARE_DIR); `fbl_jobs_v2o.py:7` asserts the string `fbl_v2_firmware` in CFFIRMWARE_DIR | |
| chain shell scripts (`*.sh`) | many `~/Desktop/...` | not part of the simulator |

Count: 11 absolute-path occurrences in the 7 simulator files above (8 in the four plant copies, 2 in `c_in_the_loop_real.py`, 1 in `test_residual_nn.py`) plus the 2 defaults in `setup.py`; external dependencies: crazyswarm2 (4 files), `test_residual_nn.py`, ns2_v2.npz, plume/calibration JSON, 2 firmware trees, Rust lib, Python 3.10 ABI.

Three hardest to remove: (1) the firmware: the bindings are built from a locally patched tree with an out-of-tree Rust static library and C host layer (and two different variants for S0-S3 and FBL); (2) the numerics tied to that exact build (float order inside the C controllers; the two trees differ in 6 source files); (3) `c_in_the_loop_real.py` and `test_residual_nn.py` being shared, import-time-side-effect modules in `experiments/fbl` / `flying_drone_stack` (sys.path inserts, `sys.argv` trick, MagicMock), plus the crazyswarm2 scenario code (can be vendored: 4 numpy-only files, but must stay numerically identical).

---------------------------------------------------------------------------------------------------------------------------------------------

## 5. Firmware (read only)

### 5.1 `~/Desktop/crazyflie-firmware`
- Branch `stable-june20`, HEAD `ccb0606ddcfe58ca2aad240d3eed4cda3afe8ff0` ("add snap.", 2026-06-11). **Correction (Stage 2, 2026-11-10; the first version of this inventory wrongly called this commit upstream):** it is a LOCAL commit by Georg Wolnik on top of Bitcraze `origin/master` = `83aa19ed47d23cf7fd5d5fd1788f4d490dd0cd3f` ("Bidirectional DSHOT and RPM telemetry (#1556)", 2026-05-08): `git rev-list origin/master..HEAD` = 1 commit, no remote branch contains it. It changes 6 files (+14/-1): `configs/cf2_defconfig`, `src/modules/interface/pptraj.h`, `src/modules/interface/stabilizer_types.h`, `src/modules/src/crtp_commander_high_level.c`, `src/modules/src/pptraj.c`, and the submodule pointer `vendor/libdw1000` (c472ae93 -> a7761032). `pptraj.c` and `stabilizer_types.h` are part of the host bindings, the rest is flight-only. Both firmware trees carry this commit, so the simulator patches are made against `83aa19ed` and include it. Remote `origin` = bitcraze/crazyflie-firmware. No stash. All other local work (below) is uncommitted by decision (`host/LOCAL_MODIFICATIONS.md`).
- Tracked modifications: 15 files, 642 insertions, 21 deletions. Untracked: 15 entries, of which 3 are old build dirs (`build_pre_*`) and 12 source files.

| File (modified, tracked) | lines | needed by host bindings? |
|---|---|---|
| `bindings/cffirmware.i` | +233 | yes (SWIG interface: controller structs, oot_* functions, `g_*` globals) |
| `bindings/setup.py` | +78/-? | yes (sources, include dirs, `-DCONFIG_PLATFORM_CF21BL`, `-DCONFIG_MODIFIED_CF_MASS=42700`, links OOT lib) |
| `src/modules/interface/controller/controller.h` | +83 | yes (ControllerType OOT slots, headers used by bindings) |
| `src/modules/interface/controller/controller_indi.h` | +81/-? | yes (INDI structs, included by the .i file) |
| `src/modules/interface/stabilizer_types.h` | +13 | yes (`gyroNoLpf`, `attitudeAcc` fields used by Omar C) |
| `src/modules/interface/math3d.h` | +10 | yes (`vadd5`) |
| `src/platform/interface/platform_defaults_cf21bl.h` | +16 | yes (`MOTORRPM2FORCE`, `THRUST2TORQUE`; THRUST_MAX / arm come from the CF21BL platform) |
| `src/modules/src/controller/controller.c` | +21 | no (not in `fw_sources`; ARM flight build slots OOT2..OOT7) |
| `src/config/config.h` | 1 line | no (stabilizer task stack size, flight only) |
| `src/deck/drivers/src/rpm.c` (+29), `usddeck.c` (+18/-?), `src/hal/src/sensors_bmi088_bmp3xx.c` (+9) | | no (flight only: RPM deck, uSD, sensors) |
| `src/modules/src/Kbuild`, `src/modules/src/controller/Kbuild`, `src/modules/src/Kconfig` | +5, +2, +63 | no (ARM build system only) |
| untracked sources: `controller_omar_indi.c/.h`, `controller_tk_indi.c`, `ns2_residual.c/.h`, `fbl_law.c/.h`, `fbl_residual.c/.h`, `fbl_rom_table.c/.h`, `fbl_nn_weights.c` | 857 + 126 lines for Omar; fbl_rom_table.c 1803 lines | yes (all listed in `fw_sources`); `controller_tk_indi.c` is compiled but not used by S0-S4 |

Bindings-relevant changed files: 7 modified + 12 untracked sources + the OOT host layer; flight-only: 8 modified (controller.c, config.h, rpm.c, usddeck.c, sensors, 3 Kbuild/Kconfig).
Timestamp finding: `src/modules/src/controller/controller_omar_indi.c` was modified on 2026-10-28 22:30 but the loaded `build/_cffirmware.cpython-310-x86_64-linux-gnu.so` was built on 2026-10-25 17:02 (wrap.c 17:01, `controller_omar_indi.o` 17:02). Every other source file is older than the .so. The S0-S3 numbers (CFFIRMWARE_DIR=this build) therefore come from the 25 Oct build; whether the 28 Oct edit touched the tree path (indi 0 / 3, NS2) is not known. Same-day check: the diff between the 25 Oct snapshot (`host/controller_omar_indi.c.snapshot`, 18 Oct) and the source is 269 lines, so the snapshot is not a usable reference for the build. See open questions.

### 5.2 `~/Desktop/fbl_v2_firmware` (it is a git repo after all: `fw/.git`)
- Layout: `fw/` (copy of crazyflie-firmware with its own `.git`), `stack/` (copy of `flying_drone_stack/firmware_app` with its own target, `tools` symlink to the repo's tools), `snap/src` (copies of 10 FBL source files = a snapshot), `hostbuild.log` (bindings build 2026-11-04 18:27), `armbuild.log` (flight build 18:31).
- Base: `fw/.git` HEAD `ccb0606d` = the same local commit (on top of `83aa19ed`) as the crazyflie-firmware tree (no further commit). Same 15 tracked modifications (640 insertions, 21 deletions) and 10 untracked sources (no `fbl_rom_table.c/.h`).
- Differences between the two firmware trees (`diff -rq` on src/ and bindings/, build dirs excluded): `controller_omar_indi.c` (81 differing lines; removes the FBL hand-over z_on/z_off, tilt latch, rate limit, NaN trip; adds `dw_en`, `dt_real` from `usecTimestamp()`; drops the `MOTORRPM2FORCE` fallback define), `controller_omar_indi.h` (12), `fbl_law.c` (5), `fbl_law.h`, `fbl_residual.c` (164), `fbl_residual.h`, `src/modules/src/Kbuild` and `bindings/setup.py` (rom_table removed); `fbl_rom_table.c/.h` exist only in the older tree. Size of the whole FBL v2 change relative to the older tree: about 270 differing lines in 6 sources + 2 build files.
- How the bindings were built: `hostbuild.log` (python 3.10 `setup.py build_ext` in `fw/build`: links `fbl_v2_firmware/stack/firmware_app/target/x86_64-unknown-linux-gnu/release/libcf_controller_rs.a`, compiles `host/oot_host.c`, `traj_iface.c`, `ns2_weights.c`); `fw/cffirmware_setup.py` is only the stock Bitcraze install stub (version 0.1, cpython-38 name), not the build recipe; the recipe is `fw/bindings/setup.py`. `_cffirmware.cpython-310-...so` 6,020,984 bytes (2026-11-04 18:27); the older one 6,065,008 bytes.
- OOT files compared: `oot_host.c`, `traj_iface.c`, `ns2_weights.c` are byte-identical between `stack/firmware_app` and the repo's `flying_drone_stack/firmware_app`.

---------------------------------------------------------------------------------------------------------------------------------------------

## 6. Bindings

| Chain | Extension loaded | Python |
|---|---|---|
| S0-S3 (and old S0/S1/S2/FBL v1 runs) | `~/Desktop/crazyflie-firmware/build/_cffirmware.cpython-310-x86_64-linux-gnu.so` + `build/cffirmware.py` (SWIG), via `sys.path.insert(0, CFFIRMWARE_DIR)` in `c_in_the_loop_real.py:19` | 3.10.12 (system, `/usr/bin/python3`) |
| FBL v2 / v2o | `~/Desktop/fbl_v2_firmware/fw/build/_cffirmware.cpython-310-...so` | 3.10.12 |

- The extension links a Rust static library (`libcf_controller_rs.a`, 7.8 MB, built 12 Oct for x86_64 from `flying_drone_stack/firmware_app` with `RUSTFLAGS="-C panic=abort" cargo build --release --target x86_64-unknown-linux-gnu`) and the host C layer. The simulator calls only C entry points (`controllerOmarIndi*`, `oot_set_peer(_count)`, `oot_set_rpm`, `cvar.g_rnn_*`, `powerDistribution*`, `motors_*`); whether the Rust archive is needed at link time (undefined symbols from `traj_iface.c` / `oot_host.c` / weak `peer_get_all`) is an open question for Stage 2.
- Build flags: `-O3 -fno-strict-aliasing -Wno-address-of-packed-member -DUNIT_TEST_MODE -DCONFIG_PLATFORM_CF21BL -DCONFIG_MODIFIED_CF_MASS=42700` + renames `-Drpm_get_all=...`. `-O3` without `-ffast-math`; float order inside the C controllers is therefore whatever the compiler produced; reproducing results bit-for-bit needs the same compiler flags, same compiler version and the same source.

---------------------------------------------------------------------------------------------------------------------------------------------

## 7. Gains

- Defaults: S0-S3 (tree): `run_job.S0_PARAMS` (`'Pxy': (7.0,.25,4.0), 'Pz': (7.0,...), 'Dxy': (4.0,...), 'Dz': (4.0,...), 'Iz': (1.5,.1,4.0), 'KRxy': 0.007, 'KRz': 0.008, 'Kwxy': 0.00115, 'Kwz': 0.002, 'KIxy': 0.03` = default, lower factor, upper factor, log scale), mapped to C struct fields by `run_job._s0_map` (Pxy -> `Kpos_P.x/.y`, Pz -> `Kpos_P.z`, Dxy/Dz -> `Kpos_D.*`, Iz -> `Kpos_I.z`, KRxy/KRz -> `KR.*`, Kwxy/Kwz -> `Komega.*`, KIxy -> `KI.x/.y`). The baseline file is `baseline_before_autotune/`. S1/S3 use the same 10 parameters with `indi = 3`.
- FBL: `FBL_PARAMS = run_job.JOBS['fbl']().params` = `kp_xy (14.0, .25, 6.0), kv_xy (9.5, .4, 3.0), ki_xy (12.0, .25, 8.0), kp_z (11.25, .25, 4.0), kv_z (5.4, .4, 3.0), ki_z (12.0, .25, 4.0)` (preset 1 = `PRESET[1]` in the plant; the preset also holds `kr_xy 59.5, kr_z 35.0`, not tuned).
- Tuned picks S0-S3 (the 6 closing picks): `out/closing/picks_s0.json` (S0: A8 bo2 s1, A4 bo2 s3, A1 bo2 s3; S2 uses the same gains + NS2) and `out/closing/picks_s1.json` (S1: A8 cmaes s1, A4 cmaes s2, A1 sobol s4; S3 uses the same gains + NS2); each entry has `method, candidate, sel_score, method_medians, gains{10 names}, baseline_sel`. Exact-inertia re-evaluation of the same gains: `out/closing_exactJ/eval_<s2|s3>_<sc>.json` (keys `pick_source`, `ref_s0|ref_s1`, `ns2_s2|ns2_s3`, each with `baseline` and `pick`). Closing table: `out/closing/closing_table_v2.md` (old), `closing_table_v3*.md` (in production). All 18 candidates of each ranking: `out/rank5b_s0_<sc>.json` (S0), `out/rank4v6_s1_<sc>.json` (S1 6 seeds + V), `out/rank4_s1_<sc>.json`.
- Gradient picks: `out/diff_<ctrl><tag>_<sc>_seed<k>.json` (tags D, E, M, G, S, V; fields `gains`, `gains_v5`, `seed`).
- FBL picks: pending (retune `fblv2o`): will come from `out/rank4oMD_fblv2o_<sc>.json` (final) and `rank4o_3sMD_fblv2o_<sc>.json` (provisional) by the same pick rule (`diff/closing_table_v3.py` `pick()`); provisional preview in `out/closing/fbl_picks_jomar_provisional.md`; picks on the old inertia (J_FBL) are the side result `rank4MD_fblv2_<sc>.json`.

---------------------------------------------------------------------------------------------------------------------------------------------

## 8. Proposed mapping (function / constant -> `frsim` module)

| Today | `frsim` module |
|---|---|
| `J_OMAR`, `J_FBL`, `M`, `G`, `ARMS`, `THRUST_MAX`, `ARM`, `T2T`, `TREE_CRASH_TILT`, timing constants (`T_TO ...`), motor / delay constants, calibration JSON values | `config.py` (one `Params` dataclass + `defaults` loaded from `gains/` and a `calibration.yaml`, no code constants hidden in `run()`) |
| `import cffirmware`, `CFFIRMWARE_DIR`, MagicMock, sys.path | `firmware.py` (single loader: env or config path, clear error message, version/hash of the loaded `.so`) |
| `mixer_forces`, `wrench`, motor lag, command delay queue, thrust law, integrator (`run` lines 40-46, 124-153) | `vehicle.py` |
| noise, state delay `md`/`hold`, RPM telemetry, accelerometer (`run` 96-120) | `sensors.py` |
| `C.reference`, `fext`, plume torque, `kappa` (`run` 133-147) + `test_residual_nn.reference` + `ns2_v2.npz` | `downwash.py` (numpy NS2 truth + plume; weights path from config) |
| `C.kin`, `scenario()`, `C.FSC.*` (vendored curves/poly4d/scenarios), `top_from_log` | `scenarios.py` (+ vendored `formations` subset) |
| controller setup in `run` (lines 58-69: tree flags, presets, `tree_gains`, `ctrlJ`, FBL fields, NS2 flags) | `controllers.py` (S0-S4 as named controllers with their defaults) and `variants.py` (md / ffs / lag / noise / kt variants, FULL / SEARCH lists) |
| the episode loop and result dict (`run`) | `episode.py` |
| `score_episodes`, `score`, rms fields, crash rule, selection / study aggregation | `metrics.py` |
| `S0_PARAMS`, `FBL_PARAMS`, `_s0_map`, `picks_*.json`, `closing_exactJ` | `gains/defaults.yaml`, `gains/tuned/` |
| `plant()` and `core.base_plant()` merging | `config.py: build_plant_config(controller, variant)` |

Must stay bit-identical (gate): the order of every floating-point operation in the per-tick loop, the order of the `rng` draws (see below), `Rot` conversions, the sequence of firmware calls per tick (peer feed, `controllerOmarIndi`, mixer every 2nd tick, `oot_set_rpm` before the controller), and the result fields read by the scoring (`stable`, `x/y/z_rms_cm`, `tilt_max_deg`, ...).

Risks for bit-identity:
1. Float order / precision: the fast plants replace `np.cross`, `np.linalg.solve`, `Jp @ w` by elementwise formulas (bit-identical by their tests). The package must copy one of the two exactly (recommend: the fast `_o` plants as the reference, since the FBL retune and S1/S3 run on them, but the S0/S2 closing numbers came from the standard `plant_core_o`, which is bit-identical by `check_plant_fast_v2o`).
2. RNG: `np.random.default_rng(seed)` drawn in a fixed sequence per tick (state noise each time a state is taken, gyro, accelerometer, rpm); no draw may be added, removed or reordered, even for disabled features (`rpmn=0` still draws `normal(0, 0, 4)`; `hold_ms` draws `uniform` only when set; `mq` FIFO). The noise seed is `cfg['seed']`; multiprocessing does not change it (each episode builds its own generator), but `Pool.map(chunksize)` and worker count change nothing about results.
3. Globals set at import / module state: `_KIN` memo cache in the fast plants (keyed by scenario repr and time), `fw.cvar.g_rnn_div/g_rnn_en` (global C state, set per episode), `fw.oot_set_peer_count`, `fw.oot_set_rpm` (global host buffers), `controllerOmarIndiInit` plus `omar.fbl.J` assignment after init. The C controller keeps static state: episodes in one process run one after another, so an episode can see leftovers of the previous one if something is not reset (the existing numbers include whatever the pool workers had in static memory; the equivalence test must therefore compare worker-per-episode runs and also repeated runs in one process).
4. `sys.argv` trick (`plant_core_o.py:21`: `sys.argv_saved = sys.argv; sys.argv = sys.argv[:1]` because `c_in_the_loop_real` parses arguments at import in some paths); must be removed in the package without changing behaviour (check that `c_in_the_loop_real` does its `argparse` only under `__main__`).
5. MagicMock of `rclpy`/`rosgraph_msgs` (`c_in_the_loop_real.py:11`): harmless if formations do not import ROS; to be checked when vendoring.
6. Build identity: the two `.so` files carry their own float behaviour. The package builds its own `.so` from a fresh clone + patches; bit-identity to the existing `.so` requires the same source (see the timestamp finding) and compiler flags; otherwise the gate must accept differences <= 1e-9 and explain them per field. A stable/crash flip near the crash border (see the S3 A1 md40/md60 finding of 9 Nov) shows that tiny differences can flip a crashed episode, so the gate should compare per-episode (not only means) on non-border cells, and treat border cells separately.
7. Import order and pickling: `fbl_jobs_v2o` must set `sys.modules['plant_core']` before `core` is imported; the package should replace this with an explicit `Plant` object passed to the pool.
8. Per-episode side effects: `np.save` of a trace file for every episode in the standard plant (`out/traces/stock_inner_traces/...`), `P.OUT.mkdir`; the fast plants skip it.
9. Early stop in the fast plants: rms fields of crashed episodes are invalid in the fast plant; the package should either reproduce the early stop or return `stable=False` plus NaN rms; gate on `stable` only for crashed episodes (as the existing identity tests do).

---------------------------------------------------------------------------------------------------------------------------------------------

## 9. Open questions for the user

1. **Reference build:** the S0-S3 bindings (25 Oct 17:02) are older than `controller_omar_indi.c` in the same tree (28 Oct 22:30). Do we accept "same tree as now" as the reference for S0-S3 (and rebuild in Stage 2 from the fresh clone), or do you want a quick check first that S0-S3 numbers do not change with the 28 Oct source? (A check needs a build, which is not allowed in Stage 1.)
2. **Which firmware for S0-S3 in the package:** one tree for all strategies (the fbl_v2 tree has the same tree-path code plus FBL v2; the old tree has the FBL v1 with hand-over) or two builds? The gate then compares S0-S3 on the new tree against the old numbers.
3. **Rust static library:** do we keep `libcf_controller_rs.a` (needs rustup and the x86_64 target) in the package build, or may Stage 2 try to drop it (if the simulator never calls the Rust controllers)? Dropping it changes the bindings link and needs the gate.
4. **Reference plant for the gate:** the exact-inertia plants (`plant_core_o`, `plant_core_indi_o`, `plant_core_fast_o`, `plant_core_indi_fast_o`) and the existing outputs `out/closing_exactJ/*` and `out/fblv2o_*` (FBL, when finished)? Or the rounded-inertia numbers of the closing table v2? (The package must use one inertia; I assume exact.)
5. **Vendoring the scenarios:** may the four numpy-only crazyswarm2 files (curves, poly4d, scenarios, safety) be copied into `frsim/scenarios/` with the commit hash `e048019` recorded, or must the package depend on the crazyswarm2 repo?
6. **Calibration files:** the plume fit and the joint calibration live in `experiments/tuning/out/` (gitignored?) - copy their current values into `simulator/gains/` / `frsim/data/` as a frozen record?
7. **Variants of the model that are not part of the current numbers** (`hold`, `abg`, `ad`, `kappa`, `top` log replay, `dist`, `inner = 0`, presets other than 1, INDI options `accl` / `thrust_law`): keep them in `frsim` (documented but off) or drop them? Dropping simplifies the code, keeping them keeps the old tests runnable.
8. **S1 plant options:** `S1_PLANT=new` (thrust law b + new calibration) is implemented in `indi_jobs_o.py` but the numbers of record use `old`. Package only `old`?
9. **Where the equivalence test gets its reference numbers from:** store a frozen reference table (e.g. 30 episodes with `x/y/z_rms_cm`, `tilt_max`, `stable`) produced now by the existing plants, so the test needs no firmware tree from the old setup? Producing it needs simulation runs (not in Stage 1, and not while the FBL chain runs).
10. **Python/numpy versions:** the references were produced with Python 3.10.12, numpy 2.2.6, scipy 1.15.3; should the package pin them?

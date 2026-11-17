# Stage F1 report: differentiable FBL model for DiffCoTune

17 Nov 2026 (work 18:57 to 21:30 CET). Branch `feature/diffcotune-fbl` in `~/Desktop/flying_robot_course_fbl`. Facts from files and from the runs listed; "not stated" / "not yet" otherwise. Nothing flashed or flown, no drone or radio, no GPU.

## 1. Isolation check

| item | result |
|---|---|
| worktree | `~/Desktop/flying_robot_course_fbl`, created with `git worktree add ... -b feature/diffcotune-fbl db052851` (db052851 is an ancestor of the lab checkout's HEAD 981bca99) |
| branch | `feature/diffcotune-fbl` (printed by `git branch --show-current` in the worktree) |
| `simulator/` | copied with `cp -a` (848 MB) into the worktree; untracked there too; all simulator changes only in this copy |
| `reference.SIMULATOR` | `~/Desktop/flying_robot_course_fbl/simulator` |
| `frsim.__file__` | `~/Desktop/flying_robot_course_fbl/simulator/frsim/__init__.py` |
| firmware bindings | `frsim.firmware.load()` takes `config.firmware_build_dir()` = `~/Desktop/flying_robot_course_fbl/simulator/_build/crazyflie-firmware/build` (copied `.so` of 10 Nov 18:55; `FRSIM_FIRMWARE_BUILD` not set; it has the FBL fields `fbl.dw_en`, `nn_en`, `inner`, `abg_on_new`, `c_d`). **Nothing was built.** `~/Desktop/crazyflie-firmware/build` (25 Oct, a different `.so`) was not touched and is not what frsim loads (it is what `lab/preflight.py` import-checks) |
| lab checkout | `git -C ~/Desktop/flying_robot_course status --porcelain` before (18:57) and after the heavy work (20:10, and again at the end): identical output (2683 lines, `diff` empty). Only git metadata of the worktree was written there |
| lab days | `~/Desktop/LAB_SESSION_ACTIVE` did not exist and no `round.py` / `session.py` process ran at any of the checks before the heavy jobs (18:57, 19:03, 20:10, 20:54) |
| processes | only my own python jobs; no `pkill`; every job ended by itself |
| git | no commit before this report; commits only on `feature/diffcotune-fbl`, no push, no Co-Authored-By |

## 2. Inventory (full text: `INVENTORY_F1.md`)

Short version:
- frsim runs the real firmware FBL branch through the bindings (no Python copy). Variants: `fbl.dw_en` (whole residual, ROM + net) and `fbl.nn_en` (net). frsim itself sets only `nn_en 1`; base and ROM-only needed a small simulator patch (section 3). **A network without ROM has no firmware mode and no weights file**: no frsim reference exists for it.
- Mass 0.0427 kg and the Omar inertia are the frsim plant values (`config.py:24-27`), the controller inertia `fbl.J` is set to the same, `inner 1`, `c_d 0.55`, net = 4 neurons (`fbl_nn_weights.c`, same numbers as in the old torch copy).
- Old torch copies exist only in the lab checkout (`experiments/tuning/diff/fbl_plant_torch*.py`, untracked): `fbl_plant_torch.py` (first port, 60 x 60 ROM table, gate, J_FBL = 2.395e-5/2.395e-5/3.235e-5), `_v2.py` (authors-aligned, closed-form ROM, still J_FBL), `_v2o.py` (v2 with the exact Omar plant inertia). `v2o` is the closest one and was the source of the law.
- Lab side: the 6 gains map to `ctrlFbl.kp_xy, kv_xy, ki_xy, kp_z, kv_z, ki_z`; switches `ctrlFbl.dw_en`, `nn_en`, `cd`; crazyswarm2 branch `study_FBL_stock` (aa6034c); the candidate image in `docs/fbl/LAB_PLAN.md:10`.
- **Correction of one fact found during the work:** the host clock (`oot_host.c:337`, +2000 us per call) suggested a 1 kHz law with dt 2 ms; the measurement (section 4) shows the controller state advances **once per 2 ms with dt 2 ms**. The inventory was corrected.

## 3. Model: files, origin, deviations from frsim

Files (all in the worktree, committed on the branch):
- `experiments/diffcotune/fbl_model.py`: the model, interface as `s0_model.py` (`GAIN_NAMES`, `GAIN_DEFAULT`, `BETA_NAMES`, `BETA_START`, `gains_from`, `beta_from`, `rollout(ref, phi, psi, ...)` with the same return, `tilt_deg`, `TILT_CRASH_DEG` = 45, `N_BETA` 4, `N_GAINS` 6). theta = 6 log-factors relative to preset 1; beta = the same 4 values as S0 (thrust-law gain, motor lag, downwash scale, plume scale; psi = 0 is the calibration). Variant by `DIFFCOTUNE_FBL_VARIANT` = `base | rom | net | both` (default `both`, which is what frsim runs).
- `experiments/diffcotune/fbl/fsens_fbl.py`: forward-mode gradient (the FBL analogue of `fsens.py`, not touched), `gradient_check_fbl.py`, `forward_check_fbl.py`, diagnostics `diag_law.py`, `diag_crash_cell.py`, `sensitivity_failed_cells.py`, `smoke_fwd.py`.
- `experiments/diffcotune/fbl/simulator_patches/0001-fbl-variant-switch.patch`: `frsim/controllers.py` reads `FRSIM_FBL_VARIANT` (`base` = `dw_en 0`, `rom` = `dw_en 1, nn_en 0`, `both` / unset = unchanged). Default behaviour is unchanged. Backup in the worktree copy: `simulator/frsim/controllers.py.before_F1`.
- Not touched: `s0_model.py` ... `s3_model.py`, `costs.py`, `fsens.py`, `loop_lib.py`, `round.py`, `METHOD_SETTINGS`.

Copied from `lab:experiments/tuning/diff/fbl_plant_torch_v2o.py` (law, ROM with analytic partials, 4-neuron net with its jerk term, stock rate / yaw PIDs, legacy mixer, quantisation emulation, `SG_R = 0.03`). Plant, beta, reference, command delay with float jitter, downwash force and plume torque: as `s0_model._block` (copied lines, not imported, because `_block` contains the S0 controller).

Deviations from frsim's FBL:
1. **Network without ROM (`net`)** is defined as "ROM off, net on" (spec item 10 is open); frsim cannot run it, so it is checked only against itself.
2. **No sensor noise** and no state delay (as `s0_model`); the optional `noise` argument exists but was not checked.
3. **`SG_R = 0.03` stop-gradient** of the residual (carried over from v2o, forward values unchanged) changes the gradient in the 2-drone scenarios (section 5).
4. **Quantisation** of the PID outputs and motor counts is emulated in the forward pass with a straight-through gradient (`FBL_QUANT=1`, as v2o). With it the forward value is piecewise constant; a finite difference with h = 1e-8 is then not valid (a smoke run, `both`, A1: FD 0 for J_task and -0.38 for J_sysId against a forward-mode gradient of 1.9e-5 and 1.6e-4; `gradient_smoke_both_A1.log`). The gradient check was therefore run with `FBL_QUANT=0`; forward checks use `FBL_QUANT=1`.
5. The ROM / residual acts on every distance and either drone above (v2o, authors' code); in solo scenarios (CIRCLE, FIG8, OVAL) there is no peer, the residual is zero, so **all four variants are the same controller there** (identical numbers in the tables).

## 4. Forward check (`forward_check_fbl.py`, `forward_check_fbl.json`, `.log`)

Gate: that of `checks/forward_check.py` (S0, Stage 2/3; `TOL_PCT = 0.5`): relative 3-D RMS difference over the frsim scoring window (scenario time > `SCORE_AFTER_S` to the scenario end) <= 0.5 % and the same crash flag (FBL limit: tilt >= 45 deg inside the window). Noise off in both. Preset-1 gains. 14 cells per variant = 7 scenarios (tuning: A1, A8, CIRCLE, FIG8, A4 cell 1; test: A4 cell 2, OVAL) x 2 plants: `calibration` and `P1` (thrust-law gain x0.8, motor lag 45 ms, downwash x1.3, plume x0.7, as `dryrun/loop_lib.py` P1).

| variant | cells passing | worst relative 3-D RMS difference (passing cells) | worst max position / velocity / attitude difference (passing cells) | failing cells |
|---|---|---|---|---|
| base | **14 / 14 PASS** | 0.0094 % | 0.041 mm / 0.21 mm/s / 0.0047 deg | none |
| rom | 12 / 14 (FAIL) | 0.0070 % | 0.028 mm / 0.13 mm/s / 0.0035 deg | A4 calibration (0.98 %), A4 cell 2 calibration (9.7 %) |
| both | 12 / 14 (FAIL) | 0.0120 % | 0.043 mm / 0.14 mm/s / 0.0029 deg | A4 calibration (7.8 %), A4 cell 2 calibration (2.5 %) |
| net | no frsim reference | n/a | n/a | model finite in all 14 cells, max tilt up to 12.6 deg, no crash flag |

Per-cell table (variant, scenario, row, 3-D RMS frsim / model, relative %, max position mm, velocity mm/s, attitude deg, crash flags) is in `forward_check_fbl.log` and `forward_check_fbl.json`. Example rows: `both` A1 calibration 0.1812 / 0.1812 cm; FIG8 calibration 0.4896 / 0.4897 cm; A8 P1 2.4847 / 2.4848 cm.

The 4 failing cells (all at calibration, only A4, only rom and both): both frsim and the model crash there (frsim max tilt 117 deg in the `rom` A4 cell). `diag_crash_cell.py` for `rom` A4: the model and frsim agree to 0.01 mm up to scenario time 10 s; after the crash begins (tilt >= 45 deg) the paths part (max difference 261 mm; tilt 117 deg in frsim, 70.5 deg in the model, where frsim stops at floor contact and the model continues). `sensitivity_failed_cells.py` (frsim under a 1e-6 change of the thrust-law gain, the Stage 3d diagnostic): `rom` A4 0.001 %, `rom` A4 cell 2 0.002 %, `both` A4 2.56 %, `both` A4 cell 2 0.10 %. Under the 13 Nov rule for S0 (a cell only counts where frsim moves < 0.5 %, otherwise the model must show the failure) the first, second and fourth cells would not be excused, since frsim is not sensitive there; the third would be. I did not change the gate; **the result is FAIL for rom and both under the stated gate**, caused by the post-crash phase of A4 at the calibration plant. The planning agent decides whether such crash cells are gated.

Finding for the lab (not a model fact): with the calibration plant, FBL on A4 cell 1 has a 3-D RMS of 29.6 cm (base), 21.7 cm (rom), 23.0 cm (both) and crashes in rom / both; on P1 it is 17.2 / 15.4 / 15.6 cm.

## 5. Gradient check (`gradient_check_fbl.py`; `gradient_check_fbl.json/.log`, `..._sgr0.json/.log`, `gradient_reverse_info.json/.log`)

As `dryrun/stage8/gate9.py`: exact forward-mode gradient (`fsens_fbl.py`: jvp of one 2 ms block vmapped over the tangents, `make_fx` + `torch.compile`, float64) against a central finite difference along the gradient direction, h = 1e-8, gate relative <= 1e-3, at preset 1 (phi = 0) and the calibration beta (psi = 0), the 5 tuning scenarios, no noise, `FBL_QUANT=0`. J_task: gradient with respect to the 6 gains; J_sysId (p + v + q): gradient with respect to the 4 beta against a real trace = frsim FBL flight on P1 (preset-1 gains; for `net` the `both` flight).

Relative difference forward-mode vs finite difference (`*` = above 1e-3), `SG_R = 0.03` (the carried-over default):

| variant | cost | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | verdict |
|---|---|---|---|---|---|---|---|
| base | J_task | 3.4e-06 | 2.1e-07 | 1.9e-06 | 3.9e-06 | 9.8e-08 | PASS |
| base | J_sysId | 1.5e-06 | 6.1e-07 | 2.2e-06 | 2.6e-07 | 1.7e-08 | PASS |
| rom | J_task | 6.3e-01* | 1.4e-03* | 1.9e-06 | 3.9e-06 | 2.6e-01* | FAIL |
| rom | J_sysId | 8.0e-01* | 1.9e-02* | 2.2e-06 | 2.6e-07 | 9.7e-02* | FAIL |
| both | J_task | 6.0e-01* | 1.9e-02* | 1.9e-06 | 3.9e-06 | 4.2e-01* | FAIL |
| both | J_sysId | 7.7e-01* | 5.7e-04 | 2.2e-06 | 2.6e-07 | 9.6e-02* | FAIL |
| net | J_task | 4.5e-02* | 2.2e-02* | 1.9e-06 | 3.9e-06 | 7.0e-01* | FAIL |
| net | J_sysId | 9.2e-02* | 2.0e-03* | 2.2e-06 | 2.6e-07 | 1.2e+00* | FAIL |

Overall: 40 rows, **FAIL**. Cause, shown by the same check with `FBL_SG_R=0` (2-drone scenarios, variants rom / both / net, A1, A8, A4): all 18 rows PASS, relative 1.0e-07 to 1.0e-04 (worst: `rom` A4 J_sysId 1.0e-04). The stop-gradient `SG_R` (v2o: inside r0 < 0.03 m the residual is a constant for the backward pass) removes derivative terms that the finite difference sees; A1 has the top drone nearly above (r0 near 0), so it is the largest effect. Without it the gradient is exact; with it the gradient norm also differs (e.g. `rom` A1 J_task 2.6e-05 with SG_R, 7.1e-05 without). Whether to keep `SG_R` is open decision 6 of the inventory (the v2o comment gives the reason: the residual's Jacobian grows like 1/r0^2 and reaches 1e14 at r0 = 0; with SG_R = 0 no non-finite value occurred in these 18 runs at the preset / calibration point; not tested elsewhere).

Reverse mode (information): `base`, CIRCLE, compiled autograd through the rollout against the forward-mode gradient: 7.2e-15 (J_task) and 7.4e-15 (J_sysId), far below the 1e-6 limit that marks reverse mode unreliable. Not run for the 2-drone scenarios (memory); not yet.

## 6. What F2 (loop wiring, round.py FBL support, preflight, dry runs) would need

1. Decide the open items: `SG_R` (gradient exact vs safeguarded), forward-check gate for crashed cells, the `net` variant (define, train, or drop; no frsim reference), whether the lab image runs FBL at 500 Hz like frsim, and quantisation in the tuning step (forward with truncation + straight-through gradient as in v2o vs smooth).
2. `loop_lib.py`: FBL entry in `MODEL_MODULES`, 6 gain names / defaults (`controllers.FBL_GAIN_NAMES`, `FBL_DEFAULT_GAINS`), `x` of length 6 + 4, `adam_inner` index set `arange(6)`, frsim episodes with `EpisodeConfig('FBL', ...)` and the FBL gain dictionary, crash tilt 45, `N_BETA` 4. `fsens.py` has the 10-gain tree hard-wired; use `fbl/fsens_fbl.py` (or its generalisation) and a worker path `worker_rollout` for FBL.
3. Noise: not modelled yet. frsim's FBL sensors draw 9 numbers per 1 ms tick (position 3, velocity 3, gyro 3, no accelerometer / rpm: `sensors.py` header), and only the even ticks enter the controller, so an `frsim_noise` for FBL is `standard_normal((n_ticks, 9))[::2]`-like; to be implemented and forward-checked with noise (as Stage 3d).
4. The variant switch is a simulator patch (`FRSIM_FBL_VARIANT`); F2 would run in this worktree (the lab checkout's `simulator/` must stay as it is).
5. `round.py` gains-to-yaml for FBL: the 6 `ctrlFbl.*` gains (kr not written), `en 1`, `inner 1`, `cd 0.55`, variant switches `dw_en` / `nn_en`; the merge step for the `ctrlFbl` group in `lab/session.py` was not checked; preflight checks for the FBL image / slot are not yet written.
6. Round tests, dry runs, lab cards, flashing: not part of F1, not started.

## 7. Files and logs

`experiments/diffcotune/fbl_model.py`, `fbl/INVENTORY_F1.md`, `fbl/report_F1.md`, `fbl/fsens_fbl.py`, `fbl/forward_check_fbl.py` (+ `.json`, `.log`), `fbl/gradient_check_fbl.py` (+ `gradient_check_fbl.json/.log`, `gradient_check_fbl_sgr0.json/.log`, `gradient_reverse_info.json/.log`, `gradient_smoke_both_A1.json/.log`), `fbl/sensitivity_failed_cells.py` (+ `.json`), `fbl/diag_law.py`, `fbl/diag_crash_cell.py`, `fbl/smoke_fwd.py`, `fbl/simulator_patches/0001-fbl-variant-switch.patch`.

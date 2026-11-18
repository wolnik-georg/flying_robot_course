# Stage F2c (partial, stopped at the noisy forward check): gyro low-pass, noisy A1, re-checks, image facts (18 Nov 2026, desk only, nothing flown, flashed or built)

Branch `feature/diffcotune-fbl`, worktree `~/Desktop/flying_robot_course_fbl`. Lab checkout untouched (`git status --porcelain` compared at the end). No lab session at the checks; all runs were single processes on one core, `nice -n 19`, `OMP_NUM_THREADS=1`. **Stopped by the stop rule: the noisy forward check (F2c.3) is FAIL (the gate is unchanged).** F2c.3 `gateF_base` and the queue F2c.4 were **not run**.

## 1. F2c.1 Gyro low-pass (E1)
**Source facts** (`~/Desktop/fbl_v2_firmware/fw/src`, identical lines in `fbl_v2b_firmware`):
- Driver `hal/src/sensors_bmi088_bmp3xx.c`; the build config of both the host and the ARM build has `CONFIG_SENSORS_BMI088_BMP3XX=y` (`fw/build/.config:36`, `stack/firmware_app/build/.config:37`).
- Cutoff `#define GYRO_LPF_CUTOFF_FREQ 80` (`:140`); `lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ)` (`:555`): sample rate 1000 Hz (`SENSORS_READ_RATE_HZ 1000`, `:61`).
- Applied in place to `sensorData.gyro`, per axis, at every sensor read (`applyAxis3fLpf(&gyroLpf, &sensorData.gyro)`, `:339`) before the sample goes to the estimator; the pre-filter value is kept in `sensorData.gyroNoLpf` (local modification, `:333-337`).
- Filter type: `utils/src/filter.c:74-101`, `lpf2pSetCutoffFreq` / `lpf2pApply`: second order (Butterworth-type coefficients, `ohm = tan(π f_c / f_s)`, `c = 1 + 2 cos(π/4) ohm + ohm²`), direct form II, zero initial state.
- The FBL controller reads `sensors->gyro` (`controller_omar_indi.c:198, :266`) = the filtered value. The stock rate PID has no D-term filter on this platform (`ATTITUDE_RATE_LPF_ENABLE` false, `platform_defaults.h:113`; `platform_defaults_cf21bl.h` does not override it).
- **Tree path:** the tree controller also reads `sensors->gyro` (`controller_omar_indi.c:551-553`, the same filtered signal on the drone), then filters the angular acceleration with its own second-order Butterworth filters (`filter_angular_acc`, `:105`). frsim's tree path has no driver low-pass; the 12.08 deg/s was fitted with the tree (calibration, 30 Oct). That is the documented S0–S3 / FBL difference: the tree has its own filters after the gyro, the FBL stock rate PID has none.
- **Correction to F2b:** the rate-PID gains of this platform are kp 200, ki 400, kd 2.5 (`platform_defaults_cf21bl.h:93-95`), not 250 / 500 (that was `platform_defaults_cf2.h`); the model already uses 200 / 400 / 2.5. `report_F2b.md` section 1.4 is corrected (P term 2 400, not 3 000; the D term is unchanged).

**Patch** `experiments/diffcotune/fbl/simulator_patches/0002-fbl-gyro-lpf.patch` (applied in the worktree `simulator/frsim/sensors.py`; backup `sensors.py.before_F2c`; `simulator/` is untracked in the branch, as for patch 0001): class `Lpf2p` (same discrete form as `filter.c`, float64, 80 Hz at 1 kHz), applied to the whole noisy gyro at every 1 ms tick **for the non-tree (FBL) controller only**; environment `FRSIM_FBL_GYRO_LPF=0` restores the old behaviour. The noise level is unchanged (12.08 deg/s).
**Model** (`fbl_model.py`, backup `.before_F2c`; `fbl/fsens_fbl.py`, backup `.before_F2c`): the same filter in `_block`: the filter state (two delay elements per axis) is appended to the FBL state vector (`NFS_ALL = NFS + 6`); the filter runs at tick 2k (sample used by the controller) and at tick 2k+1 (state only), as frsim; `frsim_noise` now returns `(n_blocks, 18)` (the gyro noise of the odd tick is needed); environment `FBL_GYRO_LPF=0` switches it off; differentiable (plain tensor operations).

**Checks**
- (a) S0 and S2 frsim episodes with noise (A1 and A8, seed 1000, calibration plant), full state traces, new `sensors.py` against `sensors.py.before_F2c`: **max abs diff 0.0** in all four (`fbl/s0_identity_check_F2c.py`, `.log`).
- (b) Noise-free FBL forward check with the filter (`fbl/forward_check_fbl_gyrolpf.json/.log`; the filter acts on the whole gyro, so it also changes the noise-free flight and the model carries it): same result as without the filter (`forward_check_fbl_noquant.json`):

  | variant | cells passing before → with filter | worst relative 3-D RMS among passing before → now | failing cells with filter |
  |---|---|---|---|
  | base | 14/14 → 14/14 | 0.177 % → 0.177 % | none |
  | rom | 12/14 → 12/14 | 0.175 % → 0.177 % | A4 and A4 cell 2, calibration (0.92 %, 8.94 %; crash in frsim and model; the D2 cells) |
  | both | 11/14 → 12/14 | 0.175 % → 0.177 % | A4 and A4 cell 2, calibration (4.33 %, 11.22 %; crash in both; the D2 cells) |

  The earlier non-crash failure (both, A4 cell 2, P1: 0.514 %) now passes (0.089 %). Gate unchanged; overall FAIL only through the 4 D2 cells (known exception D2).

## 2. F2c.2 Noisy A1 with the filter (frsim, noise 12.08 deg/s, preset 1, seeds 1000 and 1001; `fbl/a1_noise_diag_lpf.log/.json`)
**No cell crashes (12 of 12).**

| variant | plant | tilt max (deg) | tilt RMS (deg) | 3-D position RMS (cm) |
|---|---|---|---|---|
| base | calibration | 1.7–1.9 | 0.8–0.9 | 0.42–0.49 |
| base | P1 | 0.8–1.0 | 0.3–0.4 | 0.23–0.29 |
| rom | calibration | 1.7–2.0 | 0.9 | 0.52–0.58 |
| rom | P1 | 0.8–1.0 | 0.3–0.4 | 1.15–1.35 |
| both | calibration | 1.3–2.2 | 0.4–1.3 | 0.38–0.50 |
| both | P1 | 1.6–1.8 | 0.6–0.9 | 1.02–1.20 |

Side by side with the real FBL A1 flights of 26 Oct (stock inner loop, image `e3593a6f…`, complete uSD logs, F2b.2, 6 flights; **older image**, includes the real top drone's downwash and real noise): tilt RMS 2.1–4.7°, tilt max 5.8–9.3°, 3-D RMS 1.9–3.8 cm. frsim with the filter is calmer than the real flights (tilt RMS 0.3–1.3° against 2.1–4.7°). Without the filter the same cells gave tilt RMS 16–18° or a crash (F2b). No pass threshold applied; the planning agent judges.

## 3. F2c.3 Re-checks
### Noisy forward check with the filter (`fbl/noise_forward_check_fbl_gyrolpf.json/.log`; Stage 3d gate unchanged: 5 scenarios × 3 gain rows × 2 seeds at P1, per variant)
| variant | cells passing | before (F2.3, no filter) | max relative difference among stable cells |
|---|---|---|---|
| base | 30 / 30 | 28 / 30 | 0.055 % |
| rom | 30 / 30 | 28 / 30 | 0.057 % |
| both | **26 / 30** | 27 / 30 | 0.214 % |

**Result: FAIL (the gate is unchanged).** No cell has a crash flag, in frsim or in the model (0 of 90; before: 18 cells crashed in both). The 4 failing cells are all `both`, A1, at preset 1 and gains1, both seeds:

| cell | frsim RMS | model RMS | relative difference | frsim change under a 1e-6 thrust-gain change | max position difference |
|---|---|---|---|---|---|
| A1 preset1 seed 1000 | 1.020 cm | 1.128 cm | 10.5 % | 1.7 % | 7.8 mm |
| A1 preset1 seed 1001 | 1.199 cm | 1.147 cm | 4.3 % | 4.3 % | 7.6 mm |
| A1 gains1 seed 1000 | 0.301 cm | 0.307 cm | 1.9 % | 16.8 % | 4.5 mm |
| A1 gains1 seed 1001 | 0.419 cm | 0.379 cm | 9.5 % | 16.4 % | 5.8 mm |

Reading (facts, not a decision): by the Stage 3d rule a cell where frsim moves ≥ 0.5 % under the 1e-6 perturbation counts as "chaotic", and there the model must show a failure (crash flag or RMS > 3× the preset-1 RMS of the same seed). These four cells are not failures (RMS 0.3–1.2 cm, no crash); the model and frsim agree within 0.3–1.1 cm RMS and 8 mm in position, and frsim is sensitive to a 1e-6 change, so the gate asks for a failure that does not exist. The gate was not changed.
### `gateF_base` — **not run** (stop rule).

## 4. Queue F2c.4 — **not started** (needs F2c.2 and F2c.3 to pass).

## 5. F2c.5 Image preparation (read only)
- **Images:** `fbl_v2_authors_aligned_candidate.bin` (`2fbe6034bf1f6667…`, 541 524 B, 2026-11-04 18:51:47, source `~/Desktop/fbl_v2_firmware`) and `fbl_v2b_candidate.bin` (`87bd963e8299ff6a…`, 541 700 B, 2026-11-04 20:57:46, `fbl_v2` + the timing counter `t_us_max`; host outputs byte-identical to `fbl_v2`, `EQUIVALENCE_CHECK_2026-11-02.md:324`). Both exist in `flying_drone_stack/firmware_app/build_artifacts/`; hashes from `EQUIVALENCE_CHECK:307, :325`, first 16 hex re-checked with `sha256sum`. **`fbl_v2` has no `t_us_max`** (0 occurrences in its `controller_omar_indi.c`), so the bench check (`fbl_bench_check_v2.py` reads it) needs **`fbl_v2b`**. Which of the two to flash: the user's call; the model follows the `fbl_v2` source, `fbl_v2b` differs only by the counter.
- **Which image S0–S3 fly:** `study_tree_v2.bin`, sha256 `3ae7304a31d0f01e…` (`study_tree_v2.sha256`, written 2026-10-18 18:17:26), named in `docs/tree_comparison_2026-10-19.md:13`; whether it is on the drones now: not stated.
- **Same S0–S3 tree code as that image?** Compared by source, file by file: `~/Desktop/fbl_v2_firmware/fw/src` against `~/Desktop/crazyflie-firmware/src` (the repo the S0–S3 images were built from; it has no commits in 16–26 Oct, so the exact state of the 18 Oct build is **not stated**): 9 files differ, all FBL: `controller_omar_indi.{c,h}`, `fbl_law.{c,h}`, `fbl_residual.{c,h}`, `Kbuild`, and `fbl_rom_table.{c,h}` only in the repo. In `controller_omar_indi.c` the 12 differing hunks lie at lines 53–305 (FBL configuration and `controllerOmarFbl`), 418 (the FBL dispatch) and 814–846 (FBL parameters and logs); the tree code from line 310 on is identical. The sensor driver, `filter.c`, NS2 and the other controllers are identical. **The binaries were not compared** (the FBL code changes the layout; no S0–S3 flight test of `fbl_v2`). So the S0–S3 tree **source** is identical outside the FBL blocks; the claim "the image behaves like `study_tree_v2.bin` for S0–S3" is **not proven**. Per E3, flashing before the S0–S3 sessions end is therefore not allowed.
- **NS2 v2 weights:** the `fbl_v2` ARM build used `CF_RNN_WEIGHTS_NPZ=…/flying_drone_stack/tools/residual/weights/ns2_v2.npz` (`~/Desktop/fbl_v2_firmware/armbuild.log:657`); sha256 of that file (first 16 hex) `9450131abb06a719…`, the same as the simulator's `simulator/frsim/data/ns2_v2.npz`. That `study_tree_v2.bin` was built with the same npz: the registry (`docs/ns2_model_registry.md:25`) lists `ns2_v2` as the released model for the study; the build command of `study_tree_v2.bin`: not found.
- **uSD config of the image:** `flying_drone_stack/tools/usd_fbl_v2_config.txt` (96 variables; replaces `ctrlFbl.tripped` and `faults` by `t_us_max` and `res_aaz`). All 16 channels `usd_ingest.py` reads are in it (checked with the `preflight.py` 8e logic; `usd_fbl_config.txt` too). `preflight.py` 8e still checks `usd_fbl_config.txt`, 8d the old image: not changed.
- **Flash card:** `experiments/diffcotune/lab/FBL_FLASH_CARD.md` (image table, flash, `check_active_controller.py`, bench with `fbl_bench_check_v2.py`, priming, shakedown, switch back). Open fact in the card: `fbl_prime_controller.py` reads `ctrlFbl.z_on`, removed in `fbl_v2` (`:9, :22, :25`); what it does on this image is not checked.

## 6. Open items and who decides
| # | Item | Options | Who |
|---|---|---|---|
| 1 | Noisy forward check FAIL: 4 cells `both` A1, non-crashing, gate asks the model to show a failure | (a) accept as a gate artefact (frsim sensitive to a 1e-6 change, no crash, model within 0.3–1.1 cm RMS) and continue to `gateF_base` and the queue; (b) change the gate for non-crashing sensitive cells (needs a rule: not decided here); (c) stop | planning agent |
| 2 | frsim is calmer than the real A1 flights (tilt RMS 0.3–1.3° against 2.1–4.7°) | none needed for the gate; for information | planning agent |
| 3 | `gateF_base`, queue | wait for decision 1 | planning agent |
| 4 | Which image: `fbl_v2` (model source) or `fbl_v2b` (bench read-out); S0–S3 equivalence of the binary not proven; flash only after the S0–S3 sessions | user | user |
| 5 | `fbl_prime_controller.py`, `fbl_init_survival_check.py`, `preflight.py` 8d/8e and `FBL_LAB_CARD.md` image row refer to older images | update after the image decision (not changed) | planning agent |

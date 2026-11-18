# Stage F2b: why FBL crashes on A1 with sensor noise, real FBL A1 flights, which image matches (18 Nov 2026, desk only, nothing flown, flashed or built)

Branch `feature/diffcotune-fbl`, worktree `~/Desktop/flying_robot_course_fbl`. The lab checkout `git status --porcelain` is identical to `/tmp/claude-1000/status_before_F2.txt`. Lab guard: no `LAB_SESSION_ACTIVE`, no `round.py` / `session.py` process at the checks. All runs below: one process, one core, `nice -n 19`, `OMP_NUM_THREADS=1`. The queue `dryrun/fbl/f2.sh` was **not** started. Decided nothing; section 5 lists the decisions with options.

## 0. Summary (facts)
1. **The A1 crash with noise comes from the gyro noise alone** (frsim, FBL base, preset 1, A1). Position noise and velocity noise at full level: no crash, tilt max ≤ 0.7°. Gyro noise at full level (12.08 deg/s per sample): tilt RMS 16–18° at the calibration plant (no crash), crash at plant P1 (both seeds). Gyro ×0.5: tilt max ≤ 2.6°, no crash. Cliff between ×0.5 and ×0.75 (section 1.2).
2. **Where it enters:** the stock rate PID (`inner 1`) differentiates the raw gyro with no filter in the simulator build (`ATTITUDE_RATE_LPF_ENABLE` false, `platform_defaults.h:113`; D term on the measurement, `utils/src/pid.c:94-99`). The α-β-γ estimator, the velocity used directly and the jerk term are not the cause (velocity and position noise do not crash).
3. **frsim's gyro noise is 12.08 deg/s per sample on all three axes** (`simulator/frsim/data/calibration.json`, plant of record, fitted with the S0 tree). In the real uSD logs the high-frequency gyro estimate is about 2 deg/s (x, y) and 0.5–0.65 deg/s (z), an upper bound because it includes real motion. The real drone's gyro driver has an 80 Hz low-pass (`hal/src/sensors_bmi088_bmp3xx.c:140`); frsim adds white noise and no filter. Position and velocity noise in frsim (0.5 mm, 5 mm/s) are of the same order as the real logs.
4. **Real FBL A1 flights (complete uSD logs):** with the stock inner loop (26 Oct, 6 flights) tilt RMS 2.1–4.7°, tilt max 5.8–9.3°, 3-D position RMS 1.9–3.8 cm, no trip. frsim FBL A1 with full noise: tilt RMS 16–18° (calibration plant) or crash (P1).
5. **Image:** the image named in `docs/fbl/LAB_PLAN.md:10` (`e3593a6f…`, written 25 Oct 17:10:49) does **not** match the simulator build. An image from the 4 Nov authors-aligned source exists (`fbl_v2_authors_aligned_candidate.bin`, sha256 `2fbe6034…`, 4 Nov 18:51:47, not flashed according to the docs). The FBL sources of the simulator build are byte-identical to `~/Desktop/fbl_v2_firmware` (the copy that image was built from). What is on the drones now: not stated.

## 1. F2b.1 Why FBL crashes on A1 with noise

Scripts: `fbl/a1_noise_diag.py` (modes traces | table | levels | s0; the simulator copy is not edited: noise groups are scaled through `config.STATE_NOISE_POSITION_M`, `STATE_NOISE_VELOCITY_M_S` and `plant.gyro_noise_deg_s`, so the draw order is unchanged; thrust is logged by wrapping `vehicle.wrench`), `fbl/a1_noise_plot.py`. Results: `fbl/a1_noise_diag_{traces,table,levels,s0}.json/.log`. Figures: `fbl/figs/a1_noise_crash_calibration.png`, `fbl/figs/a1_noise_crash_P1.png` (tilt, roll/pitch rate, sum of motor forces, 3-D position error; seeds 1000, 1001 and no noise).

### 1.1 Which drone, onset, histories (FBL base, A1 cell of the tuning set, preset 1, `traces`)
frsim simulates **only the bottom drone**; the top drone is a prescribed path (`simulator/frsim/episode.py:126-128`, `plan.top_vehicle`), so the drone that crashes is the bottom one.

| plant | noise seed | result | tilt first > 10° (scenario time) | crash time (tilt ≥ 45°) | tilt max | 3-D RMS x / y / z (cm) |
|---|---|---|---|---|---|---|
| calibration | 1000 | no crash | 0.54 s | none | 22.9° | 5.5 / 5.3 / 2.2 |
| calibration | 1001 | no crash | 1.24 s | none | 25.7° | 5.0 / 4.6 / 2.1 |
| calibration | none | no crash | never | none | 0.0° | 0.00 / 0.00 / 0.02 |
| P1 | 1000 | **crash** | 0.36 s | 1.80 s | 89.7° | 26.3 / 24.6 / 50.4 |
| P1 | 1001 | **crash** | 0.00 s | 1.92 s | 115.4° | 39.5 / 35.8 / 60.9 |
| P1 | none | no crash | never | none | 0.0° | 0.00 / 0.00 / 0.04 |

The summed motor force in the crash run (P1, seed 1000, 1.0–1.6 s) is 0.52–0.58 N (mean 0.55 N); hover weight is mass 0.0427 kg × 9.81 = 0.419 N; the maximum per-motor force of the plant is 0.2758 N (sum 1.10 N, `calibration.json`). The motors are not at the force limit in that window; the roll/pitch rate reaches 90 deg/s (P1, seed 1000) and 159 deg/s (calibration, seed 1000) in 1.0–1.6 s. Whether individual motors clip at zero: not looked at (only the sum was logged).

### 1.2 Noise group × level (FBL base, A1, preset 1, 2 plants × 2 seeds; `table`, `levels`). "crash" = tilt ≥ 45° in the scoring window (D7). Cells show crashes out of 4 (2 plants × 2 seeds) and tilt max range.

| noise switched on | ×1 | ×0.9 | ×0.75 | ×0.5 | ×0.25 | ×0.17 |
|---|---|---|---|---|---|---|
| position only | 0 / 4, tilt max 0.0° | | | 0 / 4, 0.0° | 0 / 4, 0.0° | |
| velocity only | 0 / 4, 0.3–0.7° | | | 0 / 4, 0.2–0.4° | 0 / 4, 0.1–0.2° | |
| gyro only | **2 / 4** (P1 crashes, calibration 22–24°) | **2 / 4** (calibration 21–22°) | **2 / 4** (calibration 18–19°) | 0 / 4, 1.8–2.6° | 0 / 4, 0.9–1.2° | 0 / 4, 0.6–0.9° |
| all three | **2 / 4** (calibration 23–26°) | | | 0 / 4, 1.8–2.4° | 0 / 4, 0.8–1.2° | |
| none | 0 / 4, 0.0° | | | | | |

×0.17 of 12.08 deg/s is 2.05 deg/s, about the real-log estimate of section 1.5. The no-noise row is the first row of `a1_noise_diag_table.log`.

### 1.3 S0 on the same noisy A1 (same seeds and plants, `s0`)
The S0 tree does not crash on these seeds, with the same gyro noise (12.08 deg/s): calibration seeds 1000 / 1001 tilt max 39.9° / 39.2°, tilt RMS 29.0° / 29.1°, 3-D RMS about 10 / 10 / 15 cm; P1 tilt max 31.9° / 29.0°, tilt RMS 25.3° / 25.6°. This is large tilt, but under the tree limit (frsim's crash limit for the tree is 70°, FBL 45°, `simulator/frsim/config.py:54`). The real S0 A1 flights (18 flights of banks `bank_pilot*`, `bank_a1clean_2026-10-19`) fall in two groups: tilt RMS 6.0–9.2° (10 flights) and 26.8–29.1° (8 flights; tilt max 41.6–43.8°). So the real S0 A1 tilt of about 28° is reproduced by frsim (29°); the cause of the two real groups was not looked at.

### 1.4 Where noise enters FBL (simulator build `simulator/_build/crazyflie-firmware/src/modules/src`, byte-identical to `~/Desktop/fbl_v2_firmware/fw`)
| path | where | what | measured effect |
|---|---|---|---|
| velocity → acceleration estimate | `fbl_law.c:20-25` | α-β-γ filter, β 0.4, γ 0.1, `dt_est` = controller interval; with `abg_on_new` 0 (`controller_omar_indi.c:91`) it updates at **every** controller call (500 Hz), on the velocity of that call (`:252`) | velocity noise alone ×1: tilt max ≤ 0.7°, no crash |
| velocity, position → jerk command | `fbl_law.c:36-37` (`pos_kp·ev`, `vel_kp·(acc_est − ref acc)`) → `w_des` (`fbl_law_command`, `:98-100`) | direct, no filter | same row |
| position → integral | `fbl_law.c:33-34` | integral of the position error | position noise alone: tilt max 0.0° |
| gyro → body-rate command | `fbl_law_command` `:116-117` uses `w` | torque is computed and **not applied** with `inner 1` (`controller_omar_indi.c:266-269`; `res_ang` 0) | none |
| **gyro → stock rate PID** | `controller_omar_indi.c:198` (`attitudeControllerCorrectRatePID(gyro…)`), PID gains kp 250, ki 500, kd 2.5 (`platform_defaults_cf2.h:96-98`), rate loop at 500 Hz (`stabilizer_types.h:383-384`) | D term on the measurement, `pid->deriv = delta / dt` (`utils/src/pid.c:94`); the D-term filter is only used if `rateFiltEnable`, which is false here (`platform_defaults.h:113`, `attitude_pid_controller.c:39`) | **gyro noise alone reproduces the crash** |

Arithmetic from those constants (mine, not measured): for white gyro noise of standard deviation σ per sample the D term has standard deviation kd·√2·σ/dt = 2.5·1.414·σ/0.002 = 1768·σ, the P term 250·σ. With σ = 12.08 deg/s that is about 21 400 (D) against 3 000 (P) in the PID output unit (int16, saturating at 32 767); with σ = 2.05 deg/s about 3 600 and 510. The step from ×0.5 to ×0.75 where the tilt goes from 2° to 10–19° or crash (section 1.2) is a threshold, not a smooth rise. Why the threshold sits there (saturation of the mixer, motor lag, delay): not analysed.

### 1.5 Is frsim's noise for FBL right? (facts)
- **Same noise for FBL and the tree:** `simulator/frsim/sensors.py:34-46` and `episode.py:160`: position N(0, 0.0005 m), velocity N(0, 0.005 m/s), gyro N(0, 12.08 deg/s), drawn at every 1 ms tick for both controllers; the tree additionally gets accelerometer noise 0.084 g (`episode.py:163-168`). FBL has no separate noise setting. `docs/_inventory.md:60` and `simulator/docs/MODEL.md:29` list the values (gyro and accelerometer "from plant calibration", position and velocity "setting of the tuning plant").
- **Origin of 12.08 deg/s:** `simulator/frsim/data/calibration.json` (`gyro_noise_deg_s`, joint calibration of 30 Oct, `experiments/tuning/out/plant_calibration_joint_lag30_best_s0.json`, S0 tree). How the value was fitted: not read here. The FBL design stages (`experiments/fbl/stage3_port/*.py`) used 0.5 deg/s (`gyro_noise_dps=0.5`).
- **Real logs** (`fbl/real_flights_f2b.py`, `fbl/real_flights_f2b.json`; complete uSD logs of the lab checkout banks, cf5, window 2.0–15 s of A1 (scenario time, `SCORE_AFTER_S` 2 s, A1 scenario length 15 s); estimate = std of the first difference / √2 at 500 Hz, includes real motion so it is an upper bound):

  | group | flights | gyro x / y / z (deg/s) | position x / y / z (mm) | velocity x / y / z (mm/s) |
  |---|---|---|---|---|
  | FBL 26 Oct (stock inner) | 6 | 1.8 / 2.1 / 0.51 | 0.21 / 0.11 / 0.10 | 1.4 / 0.56 / 0.93 |
  | FBL 24 Oct (initfix; 3 flights, the first one is plain S0) | 3 | 2.0 / 2.5 / 0.61 | 0.90 / 0.74 / 0.69 | 5.1 / 3.4 / 3.9 |
  | S0 tree A1 (pilot, a1clean) | 18 | 2.3 / 4.3 / 0.66 | 1.5 / 0.51 / 0.25 | 12 / 3.9 / 3.0 |
  | frsim (all runs) | | 12.08 / 12.08 / 12.08 | 0.5 / 0.5 / 0.5 | 5 / 5 / 5 |

  Position and velocity in frsim are of the order of the real logs. The gyro is about 5× the real estimate on x and y and about 20× on z.
- **Is the FBL sensor path in frsim the same signal as on the drone?** frsim feeds the controller the true state plus noise and a state delay (`sensors.py:34-43`, `variant.state_delay_ms`), not an EKF output; the real drone feeds the on-board EKF state (docs: `EQUIVALENCE_CHECK_2026-11-02.md` F10 "on-board EKF state … stay"). The firmware comment at `controller_omar_indi.c:241-242` says the EKF velocity changes only every ~5–6 ms on the drone; in frsim the velocity noise is a fresh white sample at each call. The gyro on the drone is low-pass filtered by the sensor driver (80 Hz, `hal/src/sensors_bmi088_bmp3xx.c:140`); frsim applies no filter. Whether the logged `gyro_*` channel is the same signal the controller reads: not checked (the log channel is named `gyro_*` in `usd_fbl_config.txt`).

## 2. F2b.2 Real FBL flights on hardware (read only, uSD logs, never radio CSVs)

Sources: `docs/lab_sessions/2026-10-24.md`, `docs/lab_sessions/2026-10-26.md`, `docs/fbl/LAB_PLAN.md`, banks `experiments/logs/bank_fbl*` (lab checkout), `experiments/logs/usd_raw/*fbl*.bin` (68 files). No FBL flight after 26 Oct 2026 is in `experiments/logs/usd_raw` or `docs/lab_sessions/` (the newest FBL files are `…2026-10-26_19-0x`).

| date, bank | image / branch | inner, cd, preset | scenarios (complete logs) | outcome (from the docs and my statistics) |
|---|---|---|---|---|
| 24 Oct 13:30–13:34, `bank_fbl_2026-10-24` | pre-init-fix image, `study_FBL` | nominally preset 1, cd 0.55 | A8, A1, A4 ×2 | **FBL was not active** (init wipe bug; "the first 4 FBL flights were plain S0", `lab_sessions/2026-10-24.md:5`); the A1 file shows tilt RMS 26.8° (S0-like) |
| 24 Oct 14:25–14:30, `bank_fbl2_2026-10-24` | `study_tree_v2_fbl_initfix_candidate.bin` (27e0e100…), `study_FBL` | `inner 0` (P-only rate loop), cd 0.55, preset 1 | A8, A4 0.075, A4 0.125, A1 (14-30-39) | A4 0.125 **tripped** (tilt 53° at 9.1 s); A4 0.075 tilt peak 38°; **A1: tilt RMS 12.0°, max 19.6°, 3-D RMS 8.1 cm**, pitch oscillation about 1 Hz (docs: std 11°) |
| 24 Oct 14:44–15:01, `bank_fbl3_2026-10-24` | same image; branches `study_FBL` (14:44), `_kr` (rate ×0.5, 14:54), `_kp` (xy ×0.7, after) | `inner 0`, cd 0.55 | A4 ×5, A8 ×2, A1 (14-57-11, `_kp` block) | preset 1 flip on A4 0.125 (tilt max 180°), A8 trip at 25 s; `_kp` block 4 of 4 clean; **A1 14-57-11: tilt RMS 12.0°, max 18.9°, 3-D RMS 12.4 cm** |
| 26 Oct 17:23–17:29, `bank_fbl_b1_ours` (= `bank_fbl_nn4`, same flights) | `…stockinner_nn4_full_candidate.bin` (e3593a6f…), `study_FBL_stock` | `inner 1`, preset 1, cd 0.55, `nn_en` 1 | A8, A4 ×2, A1 (17-28-19) | no trips (docs); **A1: tilt RMS 2.5°, max 5.8°, 3-D RMS 3.0 cm** |
| 26 Oct 17:45–17:48, `bank_fbl_b2_p0` | same image, `study_FBL_p0` | preset 0 (authors' gains) | A8, A1 | **both flipped on takeoff** (`lab_sessions/2026-10-26.md:11`); FBL active 0 % in my window, tilt values meaningless |
| 26 Oct 17:54–18:00, `bank_fbl_b3_cd236` | same image, `study_FBL_cd236` | preset 1, cd 0.236, `nn_en` 0 | A8, A4 ×2, A1 (17-59-22) | **A1: tilt RMS 3.9°, max 8.0°, 3-D RMS 2.7 cm** |
| 26 Oct 18:03–18:08, `bank_fbl_b3_cd055` | same image, `study_FBL_stock` with `nn_en` 0 | preset 1, cd 0.55 | A8, A4 ×2, A1 (18-08-05) | **A1: tilt RMS 2.1°, max 4.9°, 3-D RMS 1.9 cm** |
| 26 Oct 18:31–18:34, `bank_fbl_bV_learned` | same image | preset 1, variants of block V (`docs/lab_sessions/2026-10-26.md`, which flight is which variant: not checked) | A1 ×3 (18-31-28, 18-32-51, 18-34-11) | **A1: tilt RMS 3.0 / 4.7 / 4.5°, max 6.5 / 9.0 / 9.3°, 3-D RMS 3.7 / 3.1 / 3.8 cm** |

- **A1 flights with FBL active and a complete uSD log: 8** (2 with `inner 0`, 6 with `inner 1` / e3593a6f). Statistics: `fbl/real_flights_f2b.json`.
- **Does the real oscillation look like frsim's?** `fbl/real_vs_sim_spectrum.log`: dominant tilt frequency (window 2–11 s) 2.0 / 0.8 Hz (24 Oct, `inner 0`) and 2.1–2.4 Hz (26 Oct, `inner 1`); frsim noisy A1 (calibration, base) 0.9 and 2.3 Hz. Same order, a one-peak FFT of 9 s is weak evidence; I draw no conclusion. The amplitude does differ: real `inner 1` tilt std 1.0–1.2° in that window against frsim 1.5–4.6° (calibration plant, full noise).
- The real `inner 1` A1 flights (tilt RMS 2–5°) are between frsim at gyro ×0.5 (tilt max 1.8–2.6°) and frsim at ×0.75 (tilt max 18–19°), with the caveat that frsim's A1 hover gives 0.0° without noise and the real flights include the real top drone's downwash.

## 3. F2b.3 Which firmware the drones' FBL image was built from (read only, nothing built or flashed)

| image | file in `flying_drone_stack/firmware_app/build_artifacts/` | sha256 (first 16 hex, checked by `sha256sum`) | written | source | checked against |
|---|---|---|---|---|---|
| LAB_PLAN image | `study_tree_v2_fbl_stockinner_nn4_full_candidate.bin` | `e3593a6f39a42a61…` (= `docs/fbl/LAB_PLAN.md:10`) | 2026-10-25 17:10:49 | the flown tree of 25 Oct (hand-over `z_on/z_off`, tilt latch, rate limit, NaN trip present; `~/Desktop/fbl_v2b_firmware/snap`) | `EQUIVALENCE_CHECK_2026-11-02.md:65, 129`: a rebuild is byte-identical (`cmp`); this is the image flown on 26 Oct (`:153`) |
| `fbl_v2` (authors-aligned) | `fbl_v2_authors_aligned_candidate.bin` | `2fbe6034bf1f6667…` | 2026-11-04 18:51:47 | `~/Desktop/fbl_v2_firmware` (4 Nov: hand-over, tilt latch, rate limit, NaN trip **removed**, `EQUIVALENCE_CHECK:234`; diff `experiments/fbl/authors_align/fbl_v2_vs_flown.diff`) | `EQUIVALENCE_CHECK:307`; `HANDOVER_FBL_V2_FOLLOWUP_2026-11-04.md:10` ("the copy, the diff (577 lines) and the image sha256 `2fbe6034…` match"); not flashed |
| `fbl_v2b` | `fbl_v2b_candidate.bin` | `87bd963e8299ff6a…` | 2026-11-04 20:57:46 | `~/Desktop/fbl_v2b_firmware` = `fbl_v2` + bench timing counter (`t_us_max`, 74-line diff) | `EQUIVALENCE_CHECK:324-325`: host outputs byte-identical to `fbl_v2` in 5 outputs × 2 settings; not flashed |

- **Simulator build vs sources (my check, `diff`):** `controller_omar_indi.c`, `fbl_law.c`, `fbl_residual.c` of `simulator/_build/crazyflie-firmware/src/modules/src` against `~/Desktop/fbl_v2_firmware/fw/src/modules/src`: **0 differing lines** each. Against `fbl_v2b_firmware`: 22 / 0 / 12 differing lines (the timing counter). The rest of the host tree (other files) was not compared.
- **"199/199":** that number is `simulator/tests/test_equivalence.py` (frsim, `docs/HANDOVER_DIFFCOTUNE_STAGE2_S0_2026-11-11.md:38`), not a check of an image; no image check with that number was found.
- **What is on the drones now:** not stated by any file. The last FBL flights (26 Oct) were with `e3593a6f…`; no FBL flight and no flash of `fbl_v2` / `fbl_v2b` is recorded in `docs/lab_sessions/` or `usd_raw`. The flash of `fbl_v2` is a checklist (`docs/fbl/FIRST_FLIGHT_CHECKLIST.md:38`: bench `t_us_max` over 60 s < 1 000 µs, hover first, watch takeoff and landing).
- **Result: no image that is flown matches the simulator build yet. The image built from the matching source exists and is unflashed: `fbl_v2_authors_aligned_candidate.bin` (`2fbe6034…`, source = the simulator's FBL files). The flown image `e3593a6f…` differs from the model by the removed hand-over / tilt latch / rate limit / NaN trip (116 differing lines found in D4).** The sha256 of the binary depends on the build path embedded in assert strings (`EQUIVALENCE_CHECK:307`), so a rebuild elsewhere gives another hash.
- `lab/FBL_LAB_CARD.md` and `preflight.py` 8d still name `e3593a6f…`; not changed here.

## 4. Changes to the queue tooling (resource settings, not method)
Backups `.before_F2b`. Not started.
- `dryrun/loop_lib.py`: `Pool.__init__` reads the optional environment variable `DIFFCOTUNE_POOL` and uses `min(n, DIFFCOTUNE_POOL)` workers; unset = unchanged (S0–S3 untouched). One worker per scenario task and `chunksize=1`: each task is independent, so the number of workers does not change a result (not run to verify; the F2 regressions ran with 5 workers and were 0.0).
- `dryrun/fbl/mk_f2.py` (regenerated all `jobs_*.json`): the FBL `dryF_*` and `rtestF_*` jobs set `DIFFCOTUNE_POOL=3`, `procs` 4 (3 workers + main) and `mem_gb` 13.0. **13.0 is my estimate** from the 06:21 observation of the 5-worker pool (about 3.5–3.6 GB per worker, `ps` RSS) plus the main process; not measured with 3 workers. The `gateF_*` / `gate9F_*` jobs are unchanged (2 procs, 5 GB).
- `dryrun/fbl/f2.sh`: MemAvailable is checked every 5 minutes during a job; below 3 GB the job's process tree is stopped by exact PIDs (no `pkill`), the job's `.rc` gets "memory" and the queue exits with a log line; a job with rc ≠ 0 (a FAIL of any gate or regression, or a failed run) stops the queue. The lab guard before every job is unchanged. `bash -n` passes; the new loop was not exercised.

### dryrun/loop_lib.py (backup loop_lib.py.before_F2b)
```diff
178a179
>         n = min(n, int(os.environ['DIFFCOTUNE_POOL'])) if os.environ.get('DIFFCOTUNE_POOL') else n  # F2b: fewer workers (memory); results do not depend on n (independent tasks, chunksize 1)
```
### dryrun/fbl/mk_f2.py (backup mk_f2.py.before_F2b)
```diff
19,20c19,20
<     job('dryF_' + v, 'FBL', v, 6, 10.0, 'env DIFFCOTUNE_DRY_STRICT=1 DIFFCOTUNE_RESTART_BEST=1 %s run_8.py --tag %s_ours_strict --no-eval' % (PY, tag))
<     job('rtestF_' + v, 'FBL', v, 6, 10.0, '%s %s/lab/dry_test5_fbl.py --controller FBL --variant %s --strict-dry' % (PY, TOP, v))
---
>     job('dryF_' + v, 'FBL', v, 4, 13.0, 'env DIFFCOTUNE_POOL=3 DIFFCOTUNE_DRY_STRICT=1 DIFFCOTUNE_RESTART_BEST=1 %s run_8.py --tag %s_ours_strict --no-eval' % (PY, tag))
>     job('rtestF_' + v, 'FBL', v, 4, 13.0, 'env DIFFCOTUNE_POOL=3 %s %s/lab/dry_test5_fbl.py --controller FBL --variant %s --strict-dry' % (PY, TOP, v))
```
### dryrun/fbl/f2.sh (backup f2.sh.before_F2b)
```diff
2,3c2,4
< # Stage F2.6 queue (heavy, one job at a time through job_runner.py). Before EVERY job: the lab-day guard (F2 prompt section 2): while ~/Desktop/LAB_SESSION_ACTIVE exists or a round.py / session.py process runs,
< # wait (poll every 5 min). A regression that is not 0.0 stops the queue. The bracket pattern keeps pgrep from matching this script's own command line.
---
> # Stage F2.6 queue (heavy, one job at a time through job_runner.py), F2b version. Before EVERY job: the lab-day guard (wait while ~/Desktop/LAB_SESSION_ACTIVE exists or a round.py / session.py process runs, poll every 5 min).
> # F2b changes: (1) MemAvailable is checked every 5 min during a job; below 3 GB the job is stopped by the exact PIDs of its process tree (not pkill) and the queue stops; (2) a job with rc != 0 (a FAIL of any gate or regression, or a failed run) stops the queue.
> # The bracket pattern keeps pgrep from matching this script's own command line.
5a7,8
> memgb() { awk '/MemAvailable/{printf "%.1f", $2/1048576}' /proc/meminfo; }
> tree() { local p=$1; echo $p; for c in $(pgrep -P $p); do tree $c; done; }
9,12c12,25
<   echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; rc=$(cat logs/$j.rc 2>/dev/null); echo "$(date +%T) done $j rc $rc"
<   case $j in
<     regressF2_*) if [ "$rc" != "0" ]; then echo "$(date +%T) REGRESSION NOT 0.0 or job failed: queue stopped"; exit 1; fi ;;
<   esac
---
>   echo "$(date +%T) start $j (MemAvailable $(memgb) GB)"
>   nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1 &
>   rp=$!; low=0
>   while kill -0 $rp 2>/dev/null; do
>     for i in $(seq 30); do kill -0 $rp 2>/dev/null || break; sleep 10; done
>     kill -0 $rp 2>/dev/null || break
>     m=$(memgb)
>     if awk -v m="$m" 'BEGIN{exit !(m < 3.0)}'; then
>       pids=$(tree $rp | tac | tr '\n' ' '); echo "$(date +%T) MemAvailable $m GB < 3 GB during $j: stopping PIDs $pids; queue stopped"
>       kill $pids; echo "memory" > logs/$j.rc; exit 2
>     fi
>   done
>   wait $rp; rc=$(cat logs/$j.rc 2>/dev/null); echo "$(date +%T) done $j rc $rc"
>   if [ "$rc" != "0" ]; then echo "$(date +%T) $j rc $rc (FAIL of a gate or regression, or a failed job): queue stopped"; exit 1; fi
```

## 5. Open decisions (none taken here)
| # | Question | Options (facts for each) | Who decides |
|---|---|---|---|
| 1 | frsim's gyro noise for FBL: 12.08 deg/s per sample, no filter, about 5× (x, y) to 20× (z) the real-log estimate (section 1.5) | (a) leave as is: A1 is R3 or a meaningless gradient for FBL; (b) a documented FBL gyro noise in the worktree simulator copy, as a parameter (a separate `gyro_noise_deg_s` for FBL; the draw order stays); frsim gyro ×0.17 (2.05 deg/s) gave tilt max 0.6–0.9° and no crash, ×0.5 gave 1.8–2.6° (section 1.2), real `inner 1` A1 flights 5.8–9.3°; (c) add the sensor driver's 80 Hz low-pass to the gyro path (changes the sensor model for S0–S3 too if shared, so it would need a per-controller switch); the S0–S3 stage and its frozen calibration are not touched by (b) if the value is FBL only | planning agent / user |
| 2 | If frsim's FBL noise is kept: A1 under noise crashes in all variants and gain sets (18 of 18 cells, F2.3) | (a) A1 stays and R3 handles it (3 failed attempts → fallback flight; the model still crashes at θ_0, so the A1 gradient is meaningless); (b) smallest documented change of θ_0 under the starting-gain rule; not computed here | planning agent / user |
| 3 | Gate `gateF_base` FAIL (`J_task_noise` A1, J 0.49, gradient 4.7e10 vs finite difference 7.3e5) | consequence of decision 1: it is the crashed A1 cell; no change to the gate (as instructed). After decision 1 the gate is re-run | planning agent |
| 4 | Image: the model follows the 4 Nov source; the flown image is the 25 Oct one | (a) flash `fbl_v2_authors_aligned_candidate.bin` (2fbe6034…, or `fbl_v2b`, which adds only the timing counter) after the bench checklist `FIRST_FLIGHT_CHECKLIST.md:38`, then update `FBL_LAB_CARD.md` and `preflight.py` 8d (they name e3593a6f…); (b) fly e3593a6f… and change the model (hand-over, tilt latch, rate limit, NaN trip) to the 25 Oct source; (c) accept the 116-line gap and document it; nothing built or flashed here | user (hardware), planning agent |
| 5 | Queue: `DIFFCOTUNE_POOL=3` with `mem_gb` 13.0 is an estimate | measure the memory of one 3-worker job during the first minutes of the first job; the watchdog stops it below 3 GB | planning agent starts the queue |
| 6 | Noisy forward check F2.3 (83 of 90 cells; all 7 failures in crashed or chaotic cells) | unchanged; the 4 A1 cells with a crash in both frsim and model are the same cells as decision 1 | planning agent |

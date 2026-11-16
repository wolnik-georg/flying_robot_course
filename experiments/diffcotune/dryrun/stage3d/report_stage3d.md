# Stage 3d report (S0, P1): noise in the tuning model (A), gain limit (B), A + B

Dates: 12-13 Nov 2026 (times from `date`). Comparison runs 01:34:58 to 04:22:34 (2 h 48 min of the 5 h), then reference evaluation, gradient check at h = 1e-6 and floor re-flights, all ended 04:31:23. Nothing flown or flashed; everything is plant-only.

**Corrections of the Stage 3c report.** (1) "P1 is rerun" is not written anywhere; the P1 numbers are the Stage 3b run of 12 Nov (14:12:37 to 19:41:35), and the reference column below is read from its file, not rerun. (2) "16 flights" (not 31) is already in `report_stage3b.md` and `pass_criteria_P1.md`.

**Keep-best fix confirmed** (`results/confirm_round0.json`): round 0 = 19.80291071257835 cm (as Stage 3b), same theta_1 (max abs difference 0.0), same beta after the fit (0.0), round 1 = 7.9796 cm as before.

## 1. Noise model (read only)
frsim (`simulator/frsim/sensors.py`, `episode.py`):
- one generator per episode, `np.random.default_rng(seed)` (`sensors.py:17`); draw order per 1 ms tick fixed: position 3, velocity 3, gyro 3, accelerometer 3, rpm 4 with std 0 (`sensors.py:1-6`, called at `episode.py:140, 160, 163, 166`); 16 normals per tick, also on odd ticks (the controller runs on even ticks, `episode.py:167`).
- Gaussian, zero mean, white: position std 0.0005 m, velocity std 0.005 m/s (`config.py:49-50`), gyro `plant.gyro_noise_deg_s` (calibration 12.0755 deg/s, `calibration.json:13`) added to the gyro in deg/s (`sensors.py:46`); accelerometer 0.0840 g (`sensors.py:50`) only for the tree controllers and unused by S0 (indi 0, no NS2).
- No filter, no bias or random walk in the sensor model. The state delay (`sensors.py:37-41`) is 0 for P1. The noise is added to the measurement the controller sees; the plant state (and the quaternion given to the controller) is exact.

S0 model now (`s0_model.py`): `frsim_noise(seed, n_ticks)` reproduces that stream exactly (`Generator.normal(0, std, n) = std * standard_normal`, checked) and keeps the even ticks; `_block(..., nz)` adds position, velocity and gyro noise to the controller inputs; `rollout(..., noise=)` takes one sequence for all rows or one per row. Noise is data, not a parameter; gradients flow through the closed loop as before. Default (no noise) is unchanged.

**Case that applies: identical noise sequence** (same seed, same generator, same draw order).

## 2. Forward check with noise (`noise_forward_check.py`, `noise_gate.py`, `logs/noise_gate.log`)
Cells: gains {default, round 1, round 3 of Stage 3b loop (d)} x P1 beta x 4 tuning scenarios x seeds {1000, 1001} = 24 cells; eager float64 model against frsim with noise on.

First gate (0.5 % in every cell) failed in 3 cells; I stopped and reported. **Gate chosen by the user on 13 Nov (our choice, not from the paper):** 0.5 % only in cells where frsim itself moves < 0.5 % under a 1e-6 change of the thrust-law gain; in the other cells the model must show the failure too (floor rule failed, or 3-D RMS > 3x that cell's round-1 value).

| Gains | cells | frsim stable (< 0.5 %) | max difference in stable cells | max difference in sensitive cells |
|---|---|---|---|---|
| default | 8 | 8 | 0.033 % | - |
| round 1 | 8 | 8 | 0.304 % | - |
| round 3 | 8 | 4 | 0.065 % | 11.16 % |

Sensitive cells (frsim change under 1e-6, in %; model difference; failure shown by model): A8 s1001 (5.7; 1.56; yes), CIRCLE s1001 (8.6; 1.65; yes), FIG8 s1000 (8.9; 0.005; yes), FIG8 s1001 (11.8; 11.16; yes). Stable cells: 20 of 24, all within 0.5 %. **Forward check with noise: pass = YES under the new gate** (sensitivities per cell in `logs/noise_gate.log`). The sensitivity is the closed loop with round-3 gains being chaotic (bouncing on the floor); model float64 against firmware float32 is enough to separate the paths. That reading was not proven by a second model.

## 2b. Gradient check with noise (`noise_gradient_check.py`; combined cost over the 4 scenarios, seeds 1000/1001, all 10 gains)
Gate: same sign and relative <= 1 % or absolute <= 0.1 % of the largest component.

| Finite-difference step | default point | round-1 point | overall |
|---|---|---|---|
| h = 1e-5 (`noise_gradient_check.json`) | FAIL (Pz and Dz: autograd -0.045 and 0.003 against -0.539 and -0.491) | FAIL (Pz 1.41 %, KRxy 2.05 %, Kwxy 1.99 % relative; signs agree) | **FAIL** |
| h = 1e-6 (`noise_gradient_check_h1e-06.json`) | FAIL (only KRxy: 1.68 % relative, 0.45 % of the largest) | PASS (largest relative 1.9 % on KIxy, 13 % on Dxy, both below 0.1 % absolute; all signs agree) | **FAIL** |

**Gradient check with noise: pass = NO.** Diagnosis (`noise_gradient_diag.py`): the finite difference of the noisy cost depends on h, and the discrepancy comes from A1 (floor contact at default gains, rough cost landscape): per scenario A1 differs 1.4 / 2.1 / 2.0 % for Pz / KRxy / Kwxy, A8 and FIG8 agree to about 0 to 1 %. At h = 1e-6 nine of ten components at the default point and all ten at round 1 pass; the single failing component is KRxy at the default point. I went on with the runs as instructed; the tuning gradient with noise is therefore good to about 1-2 % in direction, not verified tighter.


## 3. Speed of one tuning epoch of variant A (4 scenarios x 2 noise seeds = 8 rollouts)

| Path | seconds per epoch | note |
|---|---|---|
| CPU pool, 4 workers, 2 rows (seeds) per worker as one compiled batch | 29.4 / 23.6 / 23.2 / 12.5 | measured epochs 0-3; steady state 12.5 s; the first epochs include compile and a leftover load from stopping my earlier job |
| GPU float64, 8 rows in one batch (compiled) | 66.1 / 37.9 | iteration 1 includes the compile; cost J identical to the CPU value (3.935414e-02) |
| CPU, 8 workers with 1 row each | not run | 8 x 1.5 GB does not fit with the other memory in use (MemAvailable 5.6 GB with one pool) |

Used for A and A + B: **the CPU pool** (about 3 x faster than the GPU). B has no noise rows and uses the same pool.
- minutes per round, variant A: 19.3 / 18.1 / 18.1 / 0.1 (tune epochs 11.5 / 10.8 / 10.8 s on average)
- minutes per round, variant B: 18.3 / 17.4 / 17.3 / 0.1 (tune epochs 10.2 / 10.0 / 10.1 s on average)
- minutes per round, variant AB: 19.1 / 18.3 / 18.1 / 0.1 (tune epochs 11.4 / 10.8 / 10.8 s on average)

## 4. Per variant: real cost per round, failed flights, gain-limit hits, beta error

Real cost = mean over the 4 tuning scenarios (one flight each) of the 3-D RMS, cm. Failed = tilt rule (after up to 2 repeats) or floor rule (z mean < 50 % of the reference z mean). In brackets the scenarios that failed.

| Gain set (round) | Stage 3b (d), reference | variant A | variant B | variant AB |
|---|---|---|---|---|
| theta_0 | 19.80 cm, failed 1 (A1) | 19.80 cm, failed 1 (A1) | 19.80 cm, failed 1 (A1) | 19.80 cm, failed 1 (A1) |
| theta_1 | 7.98 cm, failed 0 | 8.70 cm, failed 0 | 10.88 cm, failed 0 | 10.16 cm, failed 0 |
| theta_2 | 16.93 cm, failed 0 | 8.86 cm, failed 0 | 20.64 cm, failed 1 (A1) | 8.53 cm, failed 0 |
| theta_3 | 69.77 cm, failed 4 (A1, A8, CIRCLE, FIG8) | 11.38 cm, failed 0 | 97.03 cm, failed 4 (A1, A8, CIRCLE, FIG8) | 6.86 cm, failed 0 |

Gain-limit hits (number of projected Adam steps out of 50 per gain; order Pxy Pz Dxy Dz Iz KRxy KRz Kwxy Kwz KIxy):

- A, tuning after round 0: limit off, hits [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
- A, tuning after round 1: limit off, hits [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
- A, tuning after round 2: limit off, hits [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
- B, tuning after round 0: limit 0.2231 (= log 1.25), hits [0, 27, 0, 25, 28, 20, 0, 21, 0, 21]
- B, tuning after round 1: limit 0.2231 (= log 1.25), hits [0, 0, 18, 0, 27, 0, 0, 0, 0, 23]
- B, tuning after round 2: limit 0.2231 (= log 1.25), hits [8, 24, 24, 0, 26, 24, 0, 0, 0, 18]
- AB, tuning after round 0: limit 0.2231 (= log 1.25), hits [0, 20, 24, 0, 27, 0, 0, 0, 0, 23]
- AB, tuning after round 1: limit 0.2231 (= log 1.25), hits [0, 0, 0, 0, 27, 0, 0, 0, 0, 0]
- AB, tuning after round 2: limit 0.2231 (= log 1.25), hits [0, 0, 0, 8, 0, 0, 0, 0, 0, 0]

Beta error after round 3 (the fit of round 3 is skipped, so this is the beta of the round-2 fit; relative error of each value against the P1 truth, %):

| | thrust gain | motor lag | downwash | plume |
|---|---|---|---|---|
| Stage 3b (d), after round 2 fit | -19.5 | -22.1 | +30.8 | -8.4 |
| A | +8.7 | -13.0 | -15.4 | +26.4 |
| B | -20.3 | -20.7 | +31.6 | +50.1 |
| AB | +11.0 | -8.7 | -10.3 | +16.9 |

## 5. Final evaluation (5 seeds, 4 tuning + 3 test cells; mean over seeds)

3-D RMS cm / z RMS cm / failed flights of 5 (tilt or floor); per scenario:

| Gain set | A1 | A8 | CIRCLE | FIG8 | A4 cell 1 | A4 cell 2 | OVAL | tuning mean | test mean |
|---|---|---|---|---|---|---|---|---|---|
| Stage 3b: (a) default | 37.38 / 35.91 / 5 | 10.99 / 9.32 / 0 | 6.64 / 6.32 / 0 | 6.14 / 5.73 / 0 | 8.78 / 7.27 / 0 | 13.15 / 11.91 / 0 | 5.19 / 5.02 / 0 | 15.29 | 9.04 |
| Stage 3b: (d) theta_best (round 1) | 14.39 / 8.93 / 0 | 5.73 / 4.70 / 0 | 4.18 / 3.79 / 0 | 4.00 / 3.52 / 0 | 7.14 / 5.50 / 0 | 10.81 / 8.70 / 0 | 3.39 / 3.17 / 0 | 7.08 | 7.11 |
| Stage 3b: (d) theta_3 | 50.01 / 50.00 / 5 | 52.23 / 46.88 / 5 | 100.97 / 91.43 / 5 | 82.04 / 81.01 / 5 | 50.75 / 45.56 / 5 | 48.90 / 45.57 / 5 | 103.90 / 91.88 / 5 | 71.31 | 67.85 |
| A theta_best (round 1) | 14.74 / 6.35 / 0 | 7.29 / 5.25 / 0 | 4.38 / 3.92 / 0 | 4.25 / 3.59 / 0 | 7.23 / 5.31 / 0 | 8.43 / 6.49 / 0 | 3.54 / 3.31 / 0 | 7.66 | 6.40 |
| A theta_3 | 18.08 / 16.03 / 0 | 7.75 / 6.12 / 0 | 4.59 / 4.14 / 0 | 4.41 / 3.78 / 0 | 7.11 / 5.43 / 0 | 9.90 / 8.66 / 0 | 3.67 / 3.44 / 0 | 8.71 | 6.89 |
| B theta_best (round 1) | 19.91 / 16.67 / 0 | 6.59 / 5.57 / 0 | 5.36 / 5.05 / 0 | 5.03 / 4.64 / 0 | 7.16 / 5.42 / 0 | 10.37 / 8.50 / 0 | 4.27 / 4.10 / 0 | 9.22 | 7.27 |
| B theta_3 | 50.01 / 50.00 / 5 | 65.94 / 49.45 / 5 | 131.87 / 97.67 / 5 | 128.45 / 97.49 / 5 | 69.69 / 48.81 / 5 | 69.43 / 48.99 / 5 | 160.47 / 97.07 / 5 | 94.06 | 99.86 |
| AB theta_best (round 3) | 11.79 / 5.12 / 0 | 5.31 / 3.74 / 0 | 2.60 / 1.52 / 0 | 2.70 / 1.15 / 0 | 8.47 / 6.65 / 0 | 9.75 / 8.62 / 0 | 2.46 / 2.06 / 0 | 5.60 | 6.89 |
| AB theta_3 | 11.79 / 5.12 / 0 | 5.31 / 3.74 / 0 | 2.60 / 1.52 / 0 | 2.70 / 1.15 / 0 | 8.47 / 6.65 / 0 | 9.75 / 8.62 / 0 | 2.46 / 2.06 / 0 | 5.60 | 6.89 |

## 6. Pass criteria (P1, per variant)

| Criterion | variant A | variant B | variant AB | Stage 3b (d) for comparison |
|---|---|---|---|---|
| 1. real cost does not rise after round 1 (round 2, 3 <= round 1 + 1 cm) | **no** (theta_2 8.86, theta_3 11.38 vs theta_1 + 1 = 9.70) | **no** (theta_2 20.64, theta_3 97.03 vs theta_1 + 1 = 11.88) | **yes** (theta_2 8.53, theta_3 6.86 vs theta_1 + 1 = 11.16) | **no** (theta_2 16.93, theta_3 69.77 vs 8.98) |
| 2. no flown gain set after theta_0 fails a scenario that theta_0 or theta_1 passed (tilt or floor) | **yes** (none) | **no** (violations: theta_2 A1, theta_3 A1, theta_3 A8, theta_3 CIRCLE, theta_3 FIG8) | **yes** (none) | **no** (violations: theta_3 A1, theta_3 A8, theta_3 CIRCLE, theta_3 FIG8) |
| 3. theta_best and theta_3 beat (a) default on tuning and test | **yes** (theta_best tuning 7.66 test 6.40; theta_3 tuning 8.71 test 6.89; default 15.29 / 9.04) | **no** (theta_best tuning 9.22 test 7.27; theta_3 tuning 94.06 test 99.86; default 15.29 / 9.04) | **yes** (theta_best tuning 5.60 test 6.89; theta_3 tuning 5.60 test 6.89; default 15.29 / 9.04) | **no** (theta_best tuning 7.08 test 7.11; theta_3 tuning 71.31 test 67.85; default 15.29 / 9.04) |
| 4. beta after round 3 | reported in section 4, no gate | |

Floor rule per flown gain set and scenario (failed = tilt or floor; rows theta_0 ... theta_3):

| | A1 | A8 | CIRCLE | FIG8 |
|---|---|---|---|---|
| Stage 3b (d) theta_0 | FAILED | ok | ok | ok |
| Stage 3b (d) theta_1 | ok | ok | ok | ok |
| Stage 3b (d) theta_2 | ok | ok | ok | ok |
| Stage 3b (d) theta_3 | FAILED | FAILED | FAILED | FAILED |
| A theta_0 | FAILED | ok | ok | ok |
| A theta_1 | ok | ok | ok | ok |
| A theta_2 | ok | ok | ok | ok |
| A theta_3 | ok | ok | ok | ok |
| B theta_0 | FAILED | ok | ok | ok |
| B theta_1 | ok | ok | ok | ok |
| B theta_2 | FAILED | ok | ok | ok |
| B theta_3 | FAILED | FAILED | FAILED | FAILED |
| AB theta_0 | FAILED | ok | ok | ok |
| AB theta_1 | ok | ok | ok | ok |
| AB theta_2 | ok | ok | ok | ok |
| AB theta_3 | ok | ok | ok | ok |

Note on theta_best: in A and B the best flown set by real cost is theta_1 (round 1); in A + B it is theta_3, so θ_best and θ₃ are the same gain set there (identical rows).

## 7. Recommendation for the full run (for you to decide)
- **A + B is the only variant that passes criteria 1, 2 and 3** (real cost 10.16 / 8.53 / 6.86 cm after theta_0 19.80; no floor or tilt failure after theta_0; final evaluation tuning 5.60 cm and test 6.89 cm against default 15.29 / 9.04). It is also the only variant whose cost keeps falling with the rounds, and its beta error after the round-2 fit is the smallest of the three (about 9-17 %), though still not small.
- **A alone** is close (8.70 / 8.86 / 11.38 cm; criterion 1 fails by 1.7 cm against the 1 cm margin; criteria 2 and 3 pass). **B alone** does not help: it reproduces the Stage 3b blow-up in round 3 (97 cm, four flights failed); the limit is hit in 20-28 of 50 steps for several gains, so it only slows the step, it does not stop the drift of theta in a chaotic region without noise.
- Caveat: one run per variant, one set of flight seeds. A + B beating A by 1.7 cm in round 3 is within what a second run could change; I did not measure run-to-run spread. The cause of the Stage 3b blow-up in (d) and B is not isolated (not stated): noise in the tuning cost is what removed it here, which fits the reading that the noise-free model overfits the floor-touching, chaotic regime, but I did not test that directly.
- If a full run is wanted: A + B, and the budget is about 18-19 min per round on the CPU pool (about 19 min round 0, 18 min later), so 6 rounds are about 1 h 50 min. Whether 6 rounds are worth it is open: A + B was still improving at round 3 (6.86 cm), but the pass criteria were only set for 4 rounds.
- Open or not measured: the gradient check with noise failed its gate at both steps (default point, KRxy 1.68 % at h = 1e-6); the model with round-3 gains of (d) is chaotic in frsim itself, so cells like that cannot be matched; 8 workers x 1 row not run (memory); the speed numbers for GPU include compile in the first iteration.

## 8. Process notes
- First forward check (23:52, 12 Nov) briefly reached load 70 (torch threads); I stopped it by exact PIDs and set 2 torch threads in every worker. Load was above 22 for about a minute. All later starts logged load 3.3-6.8 one minute after start (`logs/chain.log`).
- `speed.py` once started its own pools in the spawned workers (no `__main__` guard, load 38); stopped by exact PIDs, guard added, failed log kept as `logs/speed_cpu_FAILED_noguard.log`.
- The chain's first load gate used a decimal comma in awk and never passed; fixed with `LC_ALL=C`, chain restarted. No result affected.
- Inductor crashes for compiled CPU batches with more than one row (`try_loop_split`); worked around by replacing that function with the identity in `s0_model._patch_inductor_loop_split` (loop structure only, J matches eager and GPU).
- Keep-best fix of `run_loop` confirmed: round 0 = 19.80 cm and theta_1 identical to Stage 3b.
- Noise seeds of variant A tuning: 5000+2l and 5001+2l (my choice; not the flight seeds 0-33 and not the evaluation seeds 1000-1004).
- Nothing outside `experiments/diffcotune/` and the two docs was changed; no commit.

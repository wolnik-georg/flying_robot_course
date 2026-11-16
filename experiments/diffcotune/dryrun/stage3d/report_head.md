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

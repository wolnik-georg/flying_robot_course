# NS2 closed-loop SIL (2026-10-07)

Two-drone closed-loop SIL: **bottom (cf5)** geometric + optional 100 Hz RNN; **top (cf_second)** geometric only. Goal: reproduce hardware A8 crossing z-dips before trusting `res_sign=-1` predictions.

**Verdict (final, 2026-10-08): VALIDATED for the A8 crossing dips (network off fitted; network on `res_sign=+1` reproduced without tuning; per-crossing pattern, roll amplitude and ordering consistent).** Plant = trained-bank vertical force **made mirror-symmetric**, plus one calibrated roll/pitch-torque scalar `c = 0.0032 m²` (fitted on the network-OFF cohort only). Network off −5.84 cm (hardware −5.9), `+1` test −9.89 ± 0.05 cm (hardware −10.9 ± 1.5, range −12.2…−9.9; |Δ| 1.0 < 1.5), `−1` prediction −2.34 ± 0.07 cm (stable, 5/5 seeds). **Not validated / not predicted:** the A1 stack with the network on (SIL unstable-looking, hardware also crashed there) and the absolute size of the `−1` benefit beyond ±~1.5 cm. Earlier plants (z-force only, unsymmetrized torque) are kept below as history.

**Root cause fix (A8):** plant + SIL start at **slot xy, z=0** (not world origin); vertical takeoff then HLC poly (no default goTo).

## Corrected diagnosis (vs earlier “z≈0 not tracking”)

The earlier failure was **lateral divergence / floor crash**, not a constant z tracking offset:

- With the harness bug (below), both drones lifted then **ran away laterally** (e.g. bottom final pos ≈ `[0.75, −3.09, 0]` on A1).
- `top_gyro ≈ 0` with `partner_ok` false often means the **top drone on the floor**, not a healthy hover.
- `max_tilt_deg` is now taken from **each drone’s own** logged quaternion (`quat_bot` / `quat_top`). Values near 0° with z on the floor indicate a **quat/tilt metric gap** (identity-like attitude in logs), not “small tilt in flight”.

## Root cause fixed in harness (2026-10-07)

| Issue | Fix |
|--------|-----|
| **Missing `controllerOutOfTreeInit()`** when `skip_rnn_upload=True` | Call **`firm.controllerOutOfTreeInit()` once** after `configure_drone` / before `takeoff` for all multi-drone OOT episodes (RNN upload still calls it only when weights are loaded). Without Init, dual-drone SIL **diverges laterally** with `motor_tau=0.044` even on A1 hover. |
| Unset `g_rnn_div` | **`configure_drone`** sets `g_rnn_div=10`, `g_rnn_en` per index (required for stable peer/RNN path). |
| Per-step gain re-apply | Default **`configure_once=True`**; per-step `configure_drone` reproduces the crash. |
| `kt` plant mismatch | Plant uses **per-motor** `g_indi_kt1…kt4`, not four copies of `kt1`. |
| Shared peer hold | Per-drone 100 Hz hold in `install_peer_patch`. |
| Top tilt in metrics | `track_t["max_tilt_deg"]` from **top** quaternion. |

**`motor_tau=0.044`** is kept everywhere (not set to 0 to pass gates).

**`kr_geo` / `kw_geo`:** not exposed on host `cffirmware.cvar`. Firmware defaults **0.01 / 0.0011** (see `traj_iface.c` / geometric attitude loop on STM32). Host SIL does not override them.

## Bisection vs `ki_z_tradeoff_sweep._sim` (levels 0–6)

Script: `experiments/analysis/ns2_closed_loop_sil_bisect.py` (8 s, A1-like hover, `backend.np`, `motor_tau=0.044`).

| Level | Change | 8 s stable? |
|-------|--------|-------------|
| 0 | Reference `_sim` (1 drone, ref cffirmware, getSetpoint) | yes |
| 1 | 2 drones | yes |
| 2 | `cmdFullState` after takeoff | yes |
| 3 | `controllerOutOfTreeInit()` + per-step gain re-apply in bisect loop | yes @ 8 s hover |
| 4 | LAB INDI extras + LAB pos 40/30 | yes |
| 5 | Peer injection | yes |
| 6 | acc + `motors_rpm_meas` feed | yes |

**Formal `first_break_level` in that table: none** (hover, 8 s).

**Harness gap not in bisect ladder:** full `run_episode` without **`controllerOutOfTreeInit()`** on the no-RNN path — that is the change that breaks stability (lateral runaway on A1/A8), not `motor_tau` itself. Same τ is stable in `ki_z_tradeoff_sweep.py` (1 drone, getSetpoint only).

## Gate results

### Step 1 — per-drone isolation — **PASS (re-run 2026-10-07 in the final flying A8 configuration, torque plant code path)**

Top-drone trace max diff 0.0 m (bottom `rnn_en` 0 vs 1, downwash off); top gyro 0.78 deg/s both. (An earlier pass with both drones crashing was invalid and is superseded.)

### Step 2 — no downwash, no network — **PASS** (slot-ground + HLC)

See `step2_baseline.json` (2026-10-07): A8/A1 lat rms ≪ 5 cm outside crossings; top tilt ~1.8° (no DW).

### Step 3 (HISTORY, superseded by the torque plant in "Final result") — NS2 Fa scalar — not passed (follow-up 4)

Extended grid + bisection (`ns2_closed_loop_sil_calibrate.py`):

| Scale | Mean crossing dip [cm] |
|-------|-------------------------|
| 3.0 | −4.10 |
| 3.75 | −5.58 |
| **3.92** | **−5.97** (3 noise seeds: −5.88…−6.06, σ≈0.07 cm) |
| 4.0 | −6.17 |
| 6.0 | −18.4 (nonlinear — too strong) |

**Dip-only scalar ≈ 3.92** hits **−5.9 ± 0.8 cm** with measurement noise (pos/gyro std from hardware merged log).

**Plausibility fail:** at scale 3.92, plant `Fa_z` ≈ **−1.04 m/s²** (A8 crossings) and **−7.6 m/s²** (A1 hold) vs hardware `rnn_pred_z` **−0.6 / −1.6** — A1 exceeds **1.5×**. Scale **1.0** matches A1 force level but dip too shallow.

**Bottom tilt:** SIL p99 **~1.1°** with NS2 Fa (hardware bottom **~23°** with DW) — scalar force does not replicate attitude coupling.

**Next plant (implemented):** `downwash_plant=bank` uses **`full_bank_c1_complete.npz`** via `ns2_closed_loop_sil_plant_bank.py` (same forward as `test_residual_nn.reference`). Run: `python3 ns2_closed_loop_sil_calibrate.py --plant bank --force`.

**Bank @ scale 1.0** (`step3_calibrate_bank.json`): dip **−4.22 ± 0.02 cm**; crossings `fa_z` **−0.50 m/s²** (close to HW −0.6); A1 hold **−2.55 m/s²** (HW −1.6, plausibility fail); bottom tilt p99 **~1.2°** vs HW ~23°.

**Step 4 not run.**

## Hardware reference (2026-10-05)

| Cohort | Mean crossing dip [cm] |
|--------|-------------------------|
| Network off | **−5.9** |
| Network on, `res_sign=+1` | **−10.9** |

## Artifacts

- Harness: `experiments/analysis/ns2_closed_loop_sil_{sim,run,metrics,bisect}.py`
- Runner: `python3 experiments/analysis/ns2_closed_loop_sil_run.py --step {1|2|3|4} [--force]`
- Out: `experiments/analysis/out/ns2_closed_loop_sil/{step2_baseline.json,step3_calibrate_extended.json,step3_calibrate_bank.json,episodes/*.json}`
- Plant bank: `ns2_closed_loop_sil_plant_bank.py`; sim cfg `downwash_plant=bank`, `plant_weights=<npz>`

## Final result (2026-10-08, closed by Claude after the Cursor rounds)

### Why a z-force-only plant fails (hardware evidence)
Hardware network-off flight `merged_A8_rnn0_2026-10-05_17-39-27` (cf5 bottom): at **every** crossing the bottom drone rolls **21–28°** within ~0.2 s (roll sign alternates +24.3, −20.6, +24.2, −28.0; gyro_x peaks 220–310 deg/s); tilt p99 away from crossings is 9.8° vs 22.9° overall; `a_res_z` peaks −3.1…−3.7 m/s² (window mean about −0.8), lateral `a_res_x/y` < 0.7. A 25° roll costs cos 25° = 0.91 of the thrust (~0.9 m/s²), which a z-force-only plant cannot produce. Plants tried: CS2 NS2 `compute_Fa` rescaled (needs scale ~4, fails plausibility), bank force at scale 1.0 (dip −4.22 cm, tilt ~1.2°, A1 stack force −2.55 vs −1.6 m/s²).

### Final plant
- Vertical force: trained bank network (`full_bank_c1_complete.npz`, same forward as onboard), scale 1.0, **averaged over the x/y mirror images** (`plant_symmetrize`): the downwash is physically mirror-symmetric, the raw network is not (it made odd crossings 2× deeper than even ones: −8.6/−3.4 at c = 0.0023). No new parameter.
- **Roll/pitch torque** from the lateral gradient of that vertical force (`compute_tau_nm`): `tau_x = c·dFz/dy`, `tau_y = −c·dFz/dx`, central difference ±2 cm, added as body torque to the CS2 `Quadrotor` (`ns2_closed_loop_sil_sim.py::Quadrotor`). **One calibrated scalar `c [m²]`.**
- Controllers: geometric (40/30/8/10, ki_z 16, mass 0.041), slot-ground start, HLC trajectory, per-drone 100 Hz peer hold with true timestamps, flashed weights (19,297), `motor_tau = 0.044`, Gaussian measurement noise on position and gyro, 4 crossings per episode.

### Calibration (network OFF only) — symmetrized plant
| c [m²] | dip off [cm] | roll peaks [deg] | note |
|---|---|---|---|
| 0 | −4.08 | ±1.5 | all crossings equal |
| 0.0025 | −4.97 | ±17…18 | |
| 0.003 | −5.51 | ±20…21 | |
| **0.0032** | **−5.84 ± 0.04 (9 valid seeds)** | **±22** | chosen (closest to −5.9 ± 0.8, hardware roll 21–28°) |
| 0.0035 | −6.42 | ±23…24 | |
| 0.004 | −7.82 | ±26 | |
| 0.0045 | −11.17 | ±30…31 | smooth, still stable |
| 0.005 | −17.45 | ±36…38 | |
| ≥ 0.006 | tumbles | | |
Per-crossing spread at c = 0.0032 is ≤ 0.2 cm (all four crossings equal); hardware spread is −5.4…−7.2 (slight odd/even difference ~1.3 cm). Tilt p99 is 13 deg vs hardware 22.9 deg (the SIL rolls are as large but shorter) — a known mismatch.

### The test (not used for the choice of c) and the prediction — c = 0.0032
| Cohort | SIL dip [cm] | Hardware dip [cm] | Result |
|---|---|---|---|
| network off | −5.84 ± 0.04 (n = 9; 1 of 10 seeds raised a host binding exception `kalmanCoreUpdateWithPose`, not a controller divergence); refreshed on the canonical rebuilt binary: −5.85 ± 0.03 (n = 5) | −5.9 (−7.2…−5.4, n = 8) | fitted |
| **network on, `res_sign=+1`** | **−9.89 ± 0.05 (n = 5)**; refreshed on the canonical rebuilt binary: −9.96 ± 0.11 (n = 4, 1 host exception) | −10.9 (−12.2…−9.9, n = 16) | **PASS** (|Δ| 1.0 < 1.5; lower edge of the hardware range) |
| network on, `res_sign=−1` | **−2.34 ± 0.07 (n = 5, stable)**; refreshed: −2.30 ± 0.08 (n = 5) | not flown | **prediction** |

Honesty note: this is the second plant iteration. The first (unsymmetrized, c = 0.0023) also passed the `+1` test (−10.02) but had a bimodal crossing pattern; the symmetrization was motivated by the network-OFF per-crossing asymmetry (and the physics), c was fitted on the network-off dip only, and the `+1` cohort was never used to choose anything — but the +1 result of the first iteration was known when the second was made, so the test is not strictly blind.

### Sensitivity to the one scalar (3 seeds each, symmetrized plant)
| c | off | +1 | −1 |
|---|---|---|---|
| 0.00256 (−20 %) | −5.02 | −9.14 | −1.59 |
| 0.0032 | −5.84 | −9.89 | −2.34 |
| 0.00384 (+20 %) | −7.27 | −12.09 | −3.79 |
The ordering −1 < off < +1 (depth) holds everywhere, every run is stable, and the dips vary smoothly (no bifurcation, unlike the unsymmetrized plant).

### What this says for the next lab session (A8, `res_sign = −1`)
SIL expectation: dip about **−2.3 cm** (−1.6…−3.8 over ±20 % of c) against −5.9 cm with the network off, i.e. **~3.5 cm shallower**; the SIL reproduces the `+1` effect (off → +1) at 4.05 cm vs 5.0 cm on hardware (~20 % smaller), so the `−1` benefit may also be somewhat larger on hardware than predicted. Hardware pass criterion ("clearly shallower than −5.9 cm by ≥ 1 cm") is predicted to be met with margin. Roll peaks stay ±23° (same as network off).

### A1 stack (network on) — NOT validated, no prediction
SIL A1 hold, bottom z error: network off +1.06 cm (tilt ≤ 2.2°); `+1`: mean +0.23, min −9.5 cm, **tilt up to 37.6°**, lateral rms 4.3 cm; `−1`: min −50 cm, tilt up to 48°. The bank force in the stack is ~1.6× the hardware network prediction (−2.55 vs −1.6 m/s²), and hardware A1 with the network on crashed on 10-05 (18:55, attributed to the pose swap). Treat A1 with the network on as unvalidated and hazardous: keep the planned order (A8 first, A1 network-off last).

### Limits
- Host timing is not STM32 timing; the plant is a model; the torque form (gradient of the vertical force) is physically motivated with one scalar, validated on the A8 dips and roll amplitude only.
- Tilt p99 13° vs 22.9° on hardware; hardware shows a slight crossing-direction asymmetry that the symmetrized plant removes.
- Simple Gaussian measurement noise; no packet loss/jitter model beyond the 100 Hz hold.
- 4 crossings per SIL episode vs 16 pooled on hardware.

### Intermediate results (history)
### Intermediate plant (unsymmetrized torque plant, 2026-10-07) — SUPERSEDED

(Plant used then:
- Vertical force: trained bank network (`full_bank_c1_complete.npz`, same forward as onboard), scale 1.0.
- **Roll/pitch torque from the lateral gradient of that vertical force** (`ns2_closed_loop_sil_plant_bank.py::compute_tau_nm`): `tau_x = c·dFz/dy`, `tau_y = −c·dFz/dx` (central difference ±2 cm), body torque added to the CS2 `Quadrotor` (`ns2_closed_loop_sil_sim.py::Quadrotor`). **One calibrated scalar: `c` [m²].** Measurement noise on (pos, gyro), 5 seeds.
- Everything else: geometric controllers (40/30/8/10, ki_z 16, mass 0.041), slot-ground start, HLC trajectory, 100 Hz peer hold per drone, flashed weights (19,297), `motor_tau = 0.044`.

### Calibration (network OFF only), then the test
| c [m²] | dip off [cm] | notes |
|---|---|---|
| 0 | −4.22 | roll peaks ~1.5° |
| 0.0020 | −5.09 | roll ±14° |
| 0.0021 | −5.33 | ±15° |
| 0.0022 | −5.63 | ±15–20° |
| **0.0023** | **−6.06 ± 0.05 (4 of 5 seeds; seed 0 diverged)** | roll −16…−30°, tilt p99 ~20–22° (hardware 21–28° / 22.9°) |
| 0.0024 | −6.98 | irregular |
| ≥ 0.0025 | −9 … −36 | tumbles / bistable |

Scalar chosen at **c = 0.0023** (closest to −5.9 with hardware-like roll). Test (not used for the choice):

| Cohort (c = 0.0023) | SIL dip [cm] | Hardware dip [cm] | Result |
|---|---|---|---|
| network off | −6.06 ± 0.05 (n = 4, 1 diverged) | −5.9 (−7.2…−5.4, n = 8) | fitted |
| **network on, `res_sign=+1`** | **−10.02 ± 0.15 (n = 5)** | **−10.9 (−12.2…−9.9, n = 16)** | **PASS (|Δ| 0.9 < 1.5)** |

### Sensitivity to the one scalar (3 seeds each)
| c | off | +1 | −1 |
|---|---|---|---|
| 0.00184 (−20 %) | −4.79 | −8.08 | −1.26 |
| 0.0023 | −6.06 | −10.02 | see below |
| 0.00276 (+20 %) | −26 ± 11 (unstable) | −13.84 | −39.7 (unstable) |

The **ordering** (−1 shallower than off, off shallower than +1) holds wherever the plant is stable. The **magnitude** match of the +1 test holds only in a narrow window around c = 0.0023 (at −20 % it would not pass).

### Prediction for the next lab session (A8, `res_sign = −1`, network on)
At c = 0.0023, 5 seeds: dips −2.1, −2.3, −5.9, −8.6, −39 cm; **3 of 5 seeds show large roll excursions (70–80°, tumble)**, first-crossing roll −36…−43° in all seeds. At c −20 %: −1.26 cm (stable). Reading: the SIL predicts that `res_sign = −1` **reduces the dip** (hardware pass criterion "clearly shallower than −5.9 cm" plausible, SIL −1…−2 cm when stable) but the plant is near a stability boundary, so **no magnitude is promised** and a tumble risk at the first crossings cannot be excluded. Treat as: direction supported, magnitude and safety not predicted; the hardware sign test decides. Keep the battery/pose rules and fly the first −1 flight with the abort ready.

### Limits
- Host timing is not STM32 timing; the plant is a model; the torque form (gradient of the vertical force) is a physically motivated guess with one scalar, validated only on the dip and the roll amplitude of the network-off cohort and on the +1 dip.
- The plant is near an instability boundary in `c` (seed-0 divergence at the chosen c, bistable roll above c ≈ 0.0025); statements about tumbles are about the model, not proven hardware behaviour.
- A1 stack: force check (bank, −2.55 vs −1.6 m/s²) is still ~1.6× high; no A1 prediction is made.
- Measurement noise is simple Gaussian on pos/gyro (values in `ns2_closed_loop_sil_sim.py`).
- 4 crossings per episode in the SIL vs 16 pooled on hardware.

Artifacts: `experiments/analysis/out/ns2_closed_loop_sil/final_results.json`, scripts `ns2_closed_loop_sil_torque_fit.py`, `ns2_closed_loop_sil_torque_seeds.py`.


### Additional checks done at closure (2026-10-07; the per-crossing finding refers to the UNSYMMETRIZED plant and led to the symmetrized final plant above)
- **Dip definition identical to hardware:** the hardware module's crossing finder (`ns2_2026_10_05_crossing_dip.find_crossing_times`) applied to a SIL episode gives −6.00 cm, identical to the SIL metric; the SIL metric applied to the hardware flights gives −6.09 (17-39-27) and −5.71 cm (17-41-09), consistent with the −5.9 cohort.
- **Step 1 isolation** re-run in the final configuration: PASS (see above).
- **Seed 0 divergence** (network off, c = 0.0023): the episode diverges between t = 8 s and 12 s, i.e. at the first crossing (t ≈ 9 s), then the host binding raises `kalmanCoreUpdateWithPose` on NaN. 1 of 5 seeds; the first-crossing roll is the sensitive point (other seeds −22…−30°).
- **Per-crossing pattern is NOT reproduced.** The mean matches, the distribution does not: SIL network off = −8.6, −3.4, −8.6, −3.4 … (odd crossings deeper, 16 crossings: min −8.8, max −3.3), hardware = −6.3, −5.4, −7.2, −5.5 / −6.1, −5.4, −5.4, −6.0 (range −7.2…−5.4). SIL network on `+1` = −13.8…−15.0 (odd) and −5.7…−5.9 (even) vs hardware −12.2…−9.9. The odd/even asymmetry exists already at c = 0 (−5.2/−3.2) and is amplified by the torque term; hardware shows only a slight asymmetry (~1.3 cm). So the agreement of the means is partly a compensation between a too-deep and a too-shallow crossing type. Cause not isolated (candidate: direction dependence of the bank network via relative velocity; torque shape).
- Consequence: use the SIL only for **direction/ordering** statements (−1 shallower than off shallower than +1), not for absolute per-crossing numbers.

## Open items (history)
1. Per-crossing asymmetry (above): needs a plant whose crossing response is direction-symmetric as on hardware; not attempted.
2. A1 stack: bank force −2.55 vs −1.6 m/s² (1.6× too high), no A1 prediction.
3. `res_sign = −1` magnitude/tumble risk is model-dependent (3/5 seeds tumble at c = 0.0023, stable at c −20 %).
4. Hardware sign test (3 × A8 `res_sign=-1`, 2 × network off, 1 × A1 network off) supersedes the SIL on all of these.


### Host-binding note (2026-10-08)
The host bindings `crazyflie-firmware/build/_cffirmware*.so` were rebuilt during the Omar z-integral work **without** `--features residual_nn`, which made every network-on SIL run fail (`RNN upload rejected`) and `test_residual_nn.py` abort. Rebuilt with the documented recipe (`DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" cargo build --release --target x86_64-unknown-linux-gnu --features residual_nn`, then `rm build/_cffirmware*.so build/cffirmware_wrap.c && make bindings_python`); `test_residual_nn.py` all checks pass, Omar Rust-vs-C 7/7. The NS2 cohorts were re-run on this binary: off −5.85 ± 0.03, `+1` −9.96 ± 0.11, `−1` −2.30 ± 0.08 (same seeds give ≤ 0.1 cm different values than the numbers above; conclusions unchanged). **Always rebuild the host bindings with `--features residual_nn`.**

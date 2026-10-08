> **FINAL STATUS (2026-10-07, supersedes the Round 4 verdict below): the replay is VALID where the comparison is valid.** The A1 "failure" was a comparison error: in the 10-02 A1 flight **96.4 % of the steady samples have a motor at the PWM ceiling (65 535)** and gyro_x std is **253 deg/s** (the full-INDI oscillation of `docs/56`), so the summed motor command is *not* the controller's `thrustSi` and cannot be used as a reference. On the **unsaturated** A1 samples (n = 540) the replay agrees with the flown command: **corr 0.928, mean 0.537 vs 0.560 N (−4 %)**. A8 (97.3 % unsaturated): corr 0.79–0.82, −10 % (within the ±~10 % uncertainty of the battery-compensated thrust reconstruction); a direct regression of the flown A8 thrust on `ep_z, ev_z, a_res_z` gives R² = 0.97. Ours-vs-Omar numbers are now available as **same-input output differences** (section "Closure addendum 2"), not as on-policy claims. Older rung-2 "FAIL" rows and the banner below are kept for history.

# INDI input replay — results (2026-10-07)

> **Ladder Round 2 (2026-10-07) — STOP at rung 1 (geometric on-policy).** Flights: 2026-10-05 geometric A8 `17-39-27` / `17-41-09`, uSD `cf5_A8_thesis00/01_*_17-44-01.bin`, yaml rev **`c13546a`** (yaml `all:` pos 64/48 is **not** what the host applies). **`globals_applied` snapshot** (from `apply_ours_globals`, meta wins on pos): `g_controller_mode=0`, `g_kp_xy=40`, `g_kp_z=30`, `g_kv_xy=8`, `g_kv_z=10`, `g_ki_z=16`, `g_ki_z_limit=1.5`, `g_indi_mass=0.041`, `g_indi_res_fc=80`, `g_indi_res_clamp=10`, `g_indi_res_sign=1`, `g_rnn_en=0`, `g_indi_fc_bw=206`, `g_indi_filt_dt_us=1000`, `g_indi_clamp_en=11`, `g_indi_rpm_source=1`, full dict in `rung0_summary.json`. Steady: `sp_z ≥ 0.9·max(sp_z)`, even 1 kHz ticks for 500 Hz `a_res`.

### Rung 0 — `a_res` (amended gate)

High-signal (logged std ≥ 0.15 m/s²): corr > 0.99 and mean error ≤ 2%. Low-signal: **mean bias** |mean(replay−log)| ≤ 0.02 m/s² and RMS ≤ 0.12 m/s² (x flagged on corr either way).

| flight | axis | gate | corr | mean bias [m/s²] | RMS [m/s²] | pass |
|--------|------|------|------|------------------|------------|------|
| 17-39-27 | x | low | 0.48 | 0.0077 | 0.100 | **Yes** (bias/RMS); corr flagged |
| 17-39-27 | y | high | 0.997 | — | 0.014 | Yes |
| 17-39-27 | z | high | **0.983** | 1.16% mean | 0.078 | **No** (corr) |
| 17-41-09 | x | low | 0.86 | 0.0019 | 0.047 | Yes |
| 17-41-09 | y | high | 0.995 | — | 0.015 | Yes |
| 17-41-09 | z | high | **0.979** | 2.22% mean | 0.087 | **No** (corr) |

**Rung 0 verdict:** y (and low-signal x bias) match; **z corr misses 0.99 by ~0.01–0.02** on both flights. Proceed to rung 1 for thrust anyway (input path usable on signal-bearing axes).

**x investigation (time-boxed, `indi_replay_x_investigation.py`):** Hand `a_meas−a_model` from logged acc/RPM/quat (lib.rs `quat_to_rot`, `m=0.041`, kt from meta) — raw corr **0.44**; adding `res_fc=80`/`res_clamp=10`, acc prefilter @ 206 Hz, or DShot slew on RPM does **not** exceed corr **0.48** (replay x corr **0.48**). **Cause not isolated** to a single documented term with numbers. Residual: mean |Δ| ≈ 0.041 m/s², RMS ≈ 0.103 m/s² → thrust impact upper bound **m·RMS ≈ 4.2 mN** (geometric mode: `a_indi=0`, so x residual does not enter thrust).

### Round 3 (2026-10-07) — flown-command mapping + amended gates

**Correct flown thrust** (`indi_replay_command.py`): logged `motor.m*` is **post** `motorsCompensateBatteryVoltage` (`CONFIG_ENABLE_THRUST_BAT_COMPENSATED=y`). Inverse per motor: `V=(m/65535)·supplyVoltage` (LPF `vbat`, **b=0.01**, from radio CSV aligned via `lag_s`), `T_i=c0+c1·V+c2·V²+c3·V³` (CF21BL constants in `platform_defaults_cf21bl.h`), **total = ΣT_i**. Legacy linear `Σ(m/65535)·0.2 N` is **`commanded_total_thrust_linear_legacy`** only.

| flight | bat-comp cmd [N] | legacy linear [N] | RPM-implied [N] | weight [N] | vbat mean [V] |
|--------|------------------|-------------------|-----------------|------------|---------------|
| 10-05 17-39-27 | **0.445** | 0.470 | 0.416 | 0.402 | 3.76 |
| 10-05 17-41-09 | **0.446** | 0.481 | 0.417 | 0.402 | 3.68 |
| 10-02 A8 19-09-54 | **0.373** | 0.372 | ~0.404 | 0.402 | 3.74 |
| 10-02 A1 19-15-47 | **0.492** | 0.479 | ~0.507 | 0.402 | 3.45 (±0.1 V → ~0.467–0.519 N) |

### Rung 1 — geometric, `ki_z=16` (dynamic gate)

Gate: **corr > 0.95**, regression **slope ∈ [0.9, 1.1]** vs bat-comp command; **mean offset reported, not gated** (observed **`i_ez` ≈ −0.046 s·m** from offset / `ki_z`, hidden state — do not initialize from data).

| flight | cmd [N] | replay [N] | offset [N] | corr | slope | pass |
|--------|---------|------------|------------|------|-------|------|
| 17-39-27 | 0.445 | 0.415 | −0.030 | 0.957 | **0.879** | **No** (slope) |
| 17-41-09 | 0.446 | 0.416 | −0.030 | 0.958 | **0.865** | **No** (slope) |

**Interpretation (Round 4):** Same **~30 mN** mean replay low thrust after mapping fix; **corr passes**. Regression **slope 0.88/0.87** narrowly misses **[0.9, 1.1]**. A missing **`i_ez`** at log entry explains an **offset** (~−30 mN / implied **`i_ez` ≈ −0.046 s·m**), **not** the slope. Slope is **sensitive to flown-command reconstruction** (supply voltage LPF **b=0.01** per `stabilizer.c`, CF21BL cubic inverse, ±0.1 V) — see Round 4 table — not asserted as hidden-integral gain error.

**Geometric `ctrl_mode=0`, `ki_z=0` in 2026-09-15…2026-10-01:** **no meta** with explicit `ki_z=0` + uSD (`geometric_ki0_candidates.json`, count **0**). Mean-thrust gate applies on **10-02 full INDI** (`ki_z=0`) only until a geometric candidate exists.

### Rung 2 — full INDI `ki_z=0` (2026-10-02) — **FAIL**

| flight | bat-comp cmd [N] | replay [N] | mean err | corr | slope | pass (≤3%, corr>0.98) |
|--------|------------------|------------|----------|------|-------|------------------------|
| A8 19-09-54 | 0.373 | 0.333 | 10.6% | 0.82 | 0.84 | **No** |
| A1 19-15-47 | 0.491 | 0.718 | 46.3% | **0.075** | 0.11 | **No** |

**Git audit (since 2026-10-02 19:00):** `54d81afc`, `75aded38`, `9098e194` — NS2/RNN peer sync / 100 Hz hold; **not** a position-law change with **`rnn.en=0`**.

**Forensics (single-pass, `forensics.json`):**

- **A8 mid-hover tick 13830:** `ctrltarget_z` = scenario **0.5 m** (Δ=0); flown **0.331 N**, replay **0.325 N**, hand **0.325 N** — no thrust clamp (`clamp_en=11`, bit2 tilt off).
- **A8 crossing tick 17178:** flown **0.552 N**, replay **0.551 N** — agreement at maneuver.
- **A1 mid-hold tick 8318:** `pos_z=0.176`, `ctrltarget_z=0.5`, **`ep_z=0.324 m`**; hand `f_d` before clamp **0.933 N**; replay **`thrustSi=0.800 N` (`g_indi_thrust_max`, clamp bit3 active)**; **flown command 0.448 N** — flown law did **not** sit on the same ceiling. `ctrltarget_z` equals scenario (no HL z offset in log).

**A1 shape:** first 3 s steady corr **0.073**; hold after 3 s corr **0.078** (both fail); replay ~**0.718 N** throughout vs cmd ~**0.49 N**.

**Rung 3 / ours-vs-Omar:** **not run** (Round 4 stop rule).

### Round 4 (2026-10-07) — FINAL bounded round — root-cause checks

**Verdict: full-INDI on-policy replay is NOT REPRODUCIBLE FROM THE LOGS** for 2026-10-02 A8/A1. Checks (a)–(d) below do **not** close the gap with numbers sufficient to rerun ours-vs-Omar. **Keep Omar C ≡ Omar Rust** as the only validated cross-implementation result. **Mark existing ours-vs-Omar / step-9 comparison tables as not valid** (do not delete rows; they used wrong gates and/or legacy thrust mapping).

#### (a) Setpoint — Mode B vs Mode D (`flying_drone_stack/firmware_app/src/lib.rs`, `traj_iface.c`)

| Path | When | Z position target `pd.z` enters `ep` → `f_d` |
|------|------|-----------------------------------------------|
| **Mode B (passthrough)** | `g_traj_mode == 0` (default at boot) | `pd` from **`setpoint->position`** — same stream as logged **`ctrltarget_*`** (HLC / `cmdFullState` poly4d). |
| **Mode D (onboard traj)** | `g_traj_mode == 1` and `TRAJ_T0 > 0` | `pd.z = g_traj_hover_z + pz` from **`eval_traj_onboard`** (+ `g_traj_origin_*`, `g_traj_dz`, `g_traj_z_mode`); **not** raw `ctrltarget_z`. |

**A1 19-15-47:** logged **`ctrltarget_z = 0.5 m`**, scenario bottom **`0.5 m`** (`anchor [0,0,0.5]` + bottom slot `[0,0,0]`, `formations.A1(dz=0.5)` — top at **1.0 m**). No HL z offset in log (`ctrltarget_minus_scenario_z = 0`).

**Counterfactual identification (NOT a fix, NOT used in tables):** which constant **`z_sp`** would make the **same hand/replay law** match flown thrust given **logged** state?

| Target | Method | `z_sp` [m] |
|--------|--------|------------|
| Flown **0.448 N** at tick **8318** | Hand `f_d` with forensic **`a_res`**, logged pos/vel/quat | **≈ 0.240** |
| Steady mean cmd **≈ 0.491 N** | PD-only grid on steady subsample (mean **`pos_z ≈ 0.233 m**) | **≈ 0.335** |
| Same steady mean | **Full OOT** replay, constant `z_sp`, flight from tick 0 (14-point sweep) | **≈ 0.34** (0.325→0.468 N, 0.346→0.506 N) |

At tick 8318, flown **0.448 N** implies **`ep_z ≈ +0.023 m`** if treated as PD-only (\(ep \approx (T/m - 9.81)/k_{p,z}\)), while logged **`sp_z − pos_z ≈ 0.324 m`** — replay then commands **0.8 N** (clamp) vs flown **0.448 N**.

**Stack trace for ~0.26 m:** **none** for A1 bottom — nominal command remains **0.5 m**. The **~0.24–0.28 m** values are **inverse targets** that would reconcile thrust **if** the controller saw them; they are **not** read from `run_formation` / meta / anchor math. **(a) does not explain rung 2** with a different logged setpoint.

#### (b) Parameters not in `INDI_G_KEYS` / `POS_G_KEYS`

**One line:** flights also set **`stabilizer.controller`**, per-robot yaml overrides, **`locSrv.extPosStdDev` / `extQuatStdDev`**, **`commander.enHighLevel`**, geometric **`kr_geo`/`kw_geo`**, default **`traj.*`** (`g_traj_mode=0`), and **`rnn.*`** ( **`rnn.en=0`** on 10-02); replay does not re-apply these — none are evidenced as the A1 46% thrust gap from yaml alone.

#### (d) Firmware build / control law

**One line:** `git log --since 2026-10-02 19:00` on `flying_drone_stack/firmware_app/{lib.rs,traj_iface.c}` → **NS2/RNN commits only** (`54d81afc`, `75aded38`, `9098e194`); **no position-law change** on the `ep`/`f_d` path with **`rnn.en=0`**. Cannot prove the exact binary flashed on 10-02 without the image hash; host replay uses current **`cffirmware`** build.

#### (c) Hidden state (INDI path)

Reset at **`controllerOutOfTreeInit()`** (log mid-hover): **`i_ez`**, **`i_ep`**, **`tau_prev`/`tau_act`**, all **Butterworth/notch** filter states (`bw_*`, `bw_res_*`, `bw_acc_*`, `bw_gyro_*`, …), **`omega_filt_prev`**, **`rpm_prev`**, peer/RNN hold — **not logged** at full state.

**A1 first 3 s after steady window** (`rung2_summary.json` **`a1_segments`**): corr **0.073**, mean cmd **0.499 N** vs replay **0.718 N** — **diverges from the start** of the steady segment (not a late hold-only effect). Hold-after-3s corr **0.078** — same failure.

#### Rung 1 — slope sensitivity to `supplyVoltage` (A8 17-39-27, bat-comp cmd vs fixed replay)

LPF: **`vbat` radio → 1 kHz**, **`b=0.01`**, **`dt=1 ms`** (`indi_replay_command.py`, matches stabilizer intent). Replay thrust unchanged; slope/corr move with reconstructed command only.

| Variant | mean cmd [N] | corr | slope |
|---------|--------------|------|-------|
| LPF **b=0.01** (default) | 0.4451 | 0.9574 | 0.8787 |
| constant **vmean** | 0.4519 | 0.9655 | 0.8629 |
| **vmean − 0.1 V** | 0.4329 | 0.9655 | **0.9083** |
| **vmean + 0.1 V** | 0.4714 | 0.9655 | 0.8202 |
| no LPF (raw interp) | 0.4450 | 0.9563 | 0.8774 |
| LPF **b=0.02** | 0.4451 | 0.9568 | 0.8781 |

Only **vmean − 0.1 V** crosses slope **≥ 0.9**; all variants stay within **reconstruction uncertainty** — **no single cause asserted**.

#### Unobservable inputs / states (why logs are insufficient)

- Initial **integrator** and **filter** memory at first replay tick inside the window.
- **`g_traj_mode` / hover_z / onboard traj** state if Mode D were active (defaults suggest Mode B; not logged every tick).
- Exact **post-clamp motor pipeline** vs **`thrustSi`** ceiling in replay (A1: replay at **0.8 N** clamp, flown **0.448 N** — flown path did not match replay clamp outcome on same logged **`ep`**).

Artifacts: `experiments/analysis/out/indi_replay_ladder/rung{0,1,2}_summary.json`, `round4_summary.json`, `round4_rung1_slope_sensitivity.json`, `replay_cache.npz`, `indi_replay_command.py`, `indi_replay_ladder.py --rung 0|1|2`.

> **STATUS: FINAL (Round 4) — full-INDI on-policy NOT REPRODUCIBLE FROM LOGS.** Geometric rung 1 remains **PRELIMINARY** (slope gate). **Omar C ≡ Omar Rust only** validated. **Ours-vs-Omar tables: NOT VALID.**

> **Review note 2 (2026-10-06):** Step-9 numbers validated with one correction. The on-policy steady window used *measured* `pos_z > 0.35 m`, which dropped most of A1 (cf5 flew at ~0.24 m against a 0.5 m setpoint, the known INDI height sag; only 1726 samples). With the window defined on the *commanded* height (`sp_z ≥ 0.9·max`, fix in `indi_replay_validate.py`): **A8** flown command 0.372 N vs replay-ours 0.333 N (−10%, corr 0.87, slope 1.43); **A1** flown 0.479 N vs replay 0.718 N (+50%, corr 0.05, slope 0.08, n=15036). The replay does **not** reproduce what ours commanded in flight, in either flight; on A1 the replay answers a 26 cm height error with a far larger thrust than the flown law did. Cause unknown (candidates: firmware build flown on 10-02 vs the current host build, hidden controller state or saturation not reproduced, per-mode gains). **Do not use the ours-vs-Omar tables; only Omar C ≡ Omar Rust (~1e-7 / 1e-9) is validated.** The thrust mapping `pwm/65535·0.2 N` per motor is the stock `power_distribution_quadrotor.c` ForceTorque inverse and ignores battery compensation or caps; treat the flown-command numbers as approximate (±~10%).

> **Review note (late 2026-10-05):** the step-8 `z_tracking_trim_m` workaround (setpoint z shifted by the mean measured tracking error) is **NOT an accepted fix**: it forces the mean PD term to zero by construction. It remains available only as `--debug-z-trim` on the replay runner (default **off**). Do not use trimmed results in this doc.

> **STATUS: see Round 4 banner above.** Step-9 table below is **legacy linear** thrust mapping and **wrong steady window on A1** — **NOT VALID** for conclusions; kept for history only.

Desk open-loop replay: same measured inputs (state, gyro, acc, deck RPM, logged `ctrltarget_*`, scenario-derived setpoint vel/acc) fed to **ours** (`controllerOutOfTree`, mode 3), **Omar C** (`controllerOmarIndi`), and **Omar Rust** (`controllerOutOfTree5`). Host: system **python3 3.10.12**, `PYTHONPATH=~/Desktop/crazyflie-firmware/build`. uSD **500 Hz** held twice → **1 kHz** ticks. Warm-up: 800 ticks after first `z > 0.35 m` in the window (discarded before CSV).

## Step 9 — on-policy check (command-based gate) — **NOT VALID**

Steady window (both flights, same rule): CSV tick ≥ `warmup_start + 800` **and** `pos_z > 0.35 m`. Superseded by ladder rung 2 (bat-comp, `sp_z`-based steady); do not cite for ours-vs-Omar.

| Flight | Flown command mean [N] | Replay-ours L0 mean [N] | Mean err | corr | slope | n | Pass (5% + corr>0.9) |
|--------|------------------------|-------------------------|----------|------|-------|---|----------------------|
| A8 | 0.372 | 0.333 | 10.3% | 0.866 | 1.43 | 26060 | **No** |
| A1 | 0.463 | 0.507 | 9.4% | 0.147 | 0.20 | 1726 | **No** |

Machine output: `experiments/analysis/out/indi_replay/on_policy_check.json`. **A1** uses the same steady rule as **A8**; `n=1726` because most A1 window samples have `pos_z ≤ 0.35 m` (max ~0.41 m, shorter high-altitude segment).

**RPM-implied force (not the gate):** A8 replay-ours ~0.333 N vs our-kt RPM-implied ~0.404 N (~17% low) remains a **plant/output** gap, not used for pass/fail after step 9.

## Step 7 — parameter sets (before / after)

| Item | Host default after bare `Init()` | Flight replay (`flight_config.json` + worker apply) |
|------|----------------------------------|-----------------------------------------------------|
| `g_indi_mass` | 0.0364 kg | **0.041** kg |
| `g_controller_mode` | 0 | **3** |
| `g_kp_xy` / `g_kp_z` | 28 / 30 | **64 / 48** |
| `g_kv_xy` / `g_kv_z` | 8 / 7 | **5 / 7** |
| `g_ki_z` | (default) | **0.0** (cf5 override 2026-10-02) |
| `g_indi_kt1..4` | ~1.48e-10 | **4.16/4.06/4.11/4.06 e-10** |

Per-run snapshots: `experiments/analysis/out/indi_replay/{A8,A1}/manifest.json` → `params_by_side_level` (dict format; refresh via `indi_replay_run.py`).

## Flights

| ID | Meta | uSD (cf5 bottom) |
|----|------|------------------|
| **A8** | `experiments/logs/A8_2026-10-02_19-09-54.meta.json` | `experiments/logs/usd_raw/cf5_thesis69_2026-10-02_19-18-54.bin` |
| **A1** | `experiments/logs/A1_2026-10-02_19-15-47.meta.json` | `experiments/logs/usd_raw/cf5_thesis72_2026-10-02_19-18-55.bin` |

## Harness sanity

- `test_omar_indi_rust_vs_c.py`: **7/7 PASS**
- Omar C vs Rust on replay inputs: ~**1e-7** thrust

## Artifacts

- Scripts: `indi_replay_{command,config,inputs,worker,run,compare,validate,plot_svg}.py`
- Outputs: `compare_summary.json`, `fig_replay_thrust_rms.svg`, `on_policy_check.json`

## INDI-variant decision

**Stopped Round 4:** full-INDI replay cannot be validated from logs (hidden state + A1 thrust/clamp mismatch). No INDI-variant decision from ours-vs-Omar. No compensation terms added.

## Closure addendum (2026-10-07, Claude) — three extra discriminating checks on the 10-02 full-INDI flights

Status after these checks: **still not reproducible, but the failure is now narrowed.** Script: `experiments/analysis/indi_replay_gain_sweep.py` (system `python3`, `PYTHONPATH=…/crazyflie-firmware/build`); constant overrides only, no data-derived trim.

1. **Not a position-gain error.** A1 19-15-47, mean thrust vs flown (bat-comp 0.491 N): `kp_z` 48 → 0.718 N (46 %, corr 0.075); 24 → 0.552 N (12.5 %, corr −0.005); 12 → 0.427 N (13 %, corr −0.034); 8 → 0.384 N (22 %, corr −0.042). A gain near 17 would match the *mean*, but **no gain gives any correlation** (|corr| < 0.08 everywhere).
2. **Not a timing/alignment error.** Replay shifted by −150…+150 ms against the flown command: best corr on A1 is 0.10 (corr@0 0.075); A8 peaks at lag 0 (0.82).
3. **The input path is fine.** With the repo's alignment convention (`t_radio = t_usd − t_usd[0] + lag_s`), replayed `a_res` matches the flown (radio-logged) `a_res` in mean and spread on both flights: A1 z mean −1.79 vs −2.05 m/s², std 0.94 vs 0.87; y mean −0.18 vs −0.16, std 0.51 vs 0.51; A8 z mean −0.02 vs −0.01, std 0.47 vs 0.44. (Sample-level correlation against the 20 Hz radio log is ~0 even on A8, so the radio↔uSD alignment is too coarse for correlation; the statistics are what this check uses.)

**Conclusion (superseded by addendum 2, which found the actual cause: motor saturation in A1).** The measured inputs and the residual estimate reproduce; the position gain and the timing do not explain the gap; yet the host law returns 0.72 N where the drone flew 0.49 N at a 0.26 m height error, and the flown thrust has a time structure the replay does not follow. What remains is a difference **inside the flown control law or its applied parameters/binary** (not observable from these logs). The only decisive step is to log the controller internals in flight (`f_d` before clamp, `thrust_si`, applied `kp_z`, mode flags) — this is the already-listed lab logging patch, **not a desk task**. Until then: no ours-vs-Omar replay comparison, no claim about the flown full-INDI law from the replay. Omar C ≡ Omar Rust (1e-7 / 1e-9) remains the only validated replay result.


## Closure addendum 2 (2026-10-07) — why A1 "failed" and what the replay does show

**A1 root cause of the mismatch: motor saturation, not a controller difference.**
| Flight | steady n | any motor ≥ 65 000 PWM | gyro_x std [deg/s] | unsaturated subset: corr / mean cmd vs replay |
|---|---|---|---|---|
| A8 19-09-54 | 26 060 | 2.7 % | 32 | corr 0.789, 0.369 vs 0.328 N (−11 %) |
| A1 19-15-47 | 15 036 | **96.4 %** | **253** | n = 540, **corr 0.928, 0.560 vs 0.537 N (−4 %)** |
Regression of the flown (bat-comp) vertical-acceleration demand on `[ep_z, ev_z, a_res_z, 1]` (50 ms smoothing): A8 R² = 0.97 with effective [kp_z 39.8, kv_z 4.7, a_res gain 0.6] (nominal 48 / 7 / 1; the 17–33 % lower values can come from noise attenuation and are not interpreted as a gain error); A1 R² = 0.03 because the clipped, oscillating motor command carries no information about the position law. Also consistent with the earlier checks: gain sweep (no gain gives correlation), lag scan (peak at 0 on A8), replayed `a_res` statistics equal the flown ones.

**Ours-vs-Omar C, same inputs, open loop (`indi_replay_harmonize.py`).** The earlier L1–L3 "harmonisation levels" were no-ops (`host_mappable: False`; all four levels gave identical numbers). Replacement: make *ours* use Omar's mass (0.0427 kg) — settable on the host.
| Flight | variant | thrust RMS vs Omar C [N] | mean(ours−Omar) [N] | corr |
|---|---|---|---|---|
| A8 | ours native | 0.080 | −0.079 | 0.968 |
| A8 | ours + Omar mass | **0.052** | −0.048 | 0.969 |
| A8 | ours + Omar mass + kt_equiv | 0.069 | −0.067 | 0.968 |
| A1 | ours native | 0.152 | +0.127 | 0.743 |
| A1 | ours + Omar mass | 0.164 | +0.149 | 0.733 |
Reading: on the calm flight (A8) the two laws agree in shape (corr 0.97) and differ by a ~0.05–0.08 N level offset, of which about one third is the mass constant; the rest is law/gain structure (e.g. Omar has no z-integral and flew +19…+22 cm high on A8 in the 10-02 comparison, ours +3 cm). On A1 both laws are driven far outside their design point (drone 26 cm below the setpoint while oscillating at saturation), so their outputs diverge (corr 0.74) and no attribution is made. `kt_equiv` is not a like-for-like constant, so that row is informational only.

**What this does and does not settle.** Settled: the replay harness reproduces the flown law on valid samples; Omar C ≡ Omar Rust; the ours-vs-Omar output difference on A8 is a level offset with a mass contribution. Not settled: the exact origin of the remaining A8 offset (needs the in-flight logging of `f_d` / applied gains, a lab item), and the INDI-variant choice, which stays based on the flight comparison (`docs/56`), not on the replay.

**How far apart are ours and Omar (same inputs, A8, native settings, `compare_summary.json`)?** Thrust: ours mean 0.333 N vs Omar 0.412 N (−19 %), RMS difference 0.080 N, correlation 0.97 (spread: ours 0.058, Omar 0.049 N). Torques (RMS ours / Omar / RMS of the difference): roll 0.00126 / 0.00122 / 0.00129 N·m (corr 0.39), pitch 0.00202 / 0.00073 / 0.00183 (corr 0.40; ours ~2.8× larger), yaw 0.00149 / 0.00136 / 0.00113 (corr 0.69). Reading: the thrust law has the same shape with a level offset (about a third from the mass constant); the attitude torques are different laws (different gains and filters), only loosely correlated. On A1 (saturated, oscillating flight) both are outside their design point: ours 0.718 N vs Omar 0.591 N, corr 0.74; torque RMS ours 0.010/0.013 vs Omar 0.0055/0.0056 N·m — not interpreted.

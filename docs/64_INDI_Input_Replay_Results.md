# INDI input replay — results (2026-10-06)

> **Review note 2 (2026-10-06):** Step-9 numbers validated with one correction. The on-policy steady window used *measured* `pos_z > 0.35 m`, which dropped most of A1 (cf5 flew at ~0.24 m against a 0.5 m setpoint, the known INDI height sag; only 1726 samples). With the window defined on the *commanded* height (`sp_z ≥ 0.9·max`, fix in `indi_replay_validate.py`): **A8** flown command 0.372 N vs replay-ours 0.333 N (−10%, corr 0.87, slope 1.43); **A1** flown 0.479 N vs replay 0.718 N (+50%, corr 0.05, slope 0.08, n=15036). The replay does **not** reproduce what ours commanded in flight, in either flight; on A1 the replay answers a 26 cm height error with a far larger thrust than the flown law did. Cause unknown (candidates: firmware build flown on 10-02 vs the current host build, hidden controller state or saturation not reproduced, per-mode gains). **Do not use the ours-vs-Omar tables; only Omar C ≡ Omar Rust (~1e-7 / 1e-9) is validated.** The thrust mapping `pwm/65535·0.2 N` per motor is the stock `power_distribution_quadrotor.c` ForceTorque inverse and ignores battery compensation or caps; treat the flown-command numbers as approximate (±~10%).

> **Review note (late 2026-10-05):** the step-8 `z_tracking_trim_m` workaround (setpoint z shifted by the mean measured tracking error) is **NOT an accepted fix**: it forces the mean PD term to zero by construction. It remains available only as `--debug-z-trim` on the replay runner (default **off**). Do not use trimmed results in this doc.

> **STATUS (step 9, 2026-10-06): PRELIMINARY — command-based on-policy gate did NOT pass.** Gate compares replay-ours `thrust_si` to **flown motor command** total thrust (`motor_m1..4` → Σ (PWM/65535)×0.2 N/motor, CF21BL `THRUST_MAX`, inverse of `power_distribution_quadrotor.c` ForceTorque path). Untrimmed replay only (`debug_z_trim=false`).

Desk open-loop replay: same measured inputs (state, gyro, acc, deck RPM, logged `ctrltarget_*`, scenario-derived setpoint vel/acc) fed to **ours** (`controllerOutOfTree`, mode 3), **Omar C** (`controllerOmarIndi`), and **Omar Rust** (`controllerOutOfTree5`). Host: system **python3 3.10.12**, `PYTHONPATH=~/Desktop/crazyflie-firmware/build`. uSD **500 Hz** held twice → **1 kHz** ticks. Warm-up: 800 ticks after first `z > 0.35 m` in the window (discarded before CSV).

## Step 9 — on-policy check (command-based gate)

Steady window (both flights, same rule): CSV tick ≥ `warmup_start + 800` **and** `pos_z > 0.35 m`.

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

No decision until command-based on-policy passes (mean within 5%, corr > 0.9). Step 9 did not pass; no further compensation terms added.

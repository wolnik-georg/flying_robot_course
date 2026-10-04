# INDI loop rates, timing comparison, and oscillation ledger

**Date:** 2026-10-04 · **Scope:** Read-only code + existing logs (no firmware edits, no flights).  
**Question:** Do INDI variants differ in call rates, decimation, filter design sample time, or setpoint/state age in ways that plausibly change delay budget at ~6.3 Hz?  
**Oscillation:** Report differences and budget only — **no root-cause claim**.

**Artifacts:** `experiments/analysis/indi_loop_rates_delay_budget.py`, `experiments/analysis/indi_loop_rates_flight_spectrum.py` → `experiments/analysis/out/indi_loop_rates/`.

---

## 1. Loop inventory

Legend: **FACT** = cited source line; **JUDGEMENT** = interpretation. Confidence: HIGH / MEDIUM / LOW.

### 1.1 Shared platform (all variants)

| Stage | Rate | Schedule | dt assumed | Notes | Evidence | Conf. |
|---|---|---|---|---|---|---|
| Stabilizer main loop | 1000 Hz | `vTaskDelayUntil(..., F2T(RATE_MAIN_LOOP))`; `sensorsWaitDataReady()` each iter. | 1 ms nominal | `controller()` called **every** tick. | `stabilizer.c:285-350`, `stabilizer_types.h:383-389` | HIGH |
| IMU read + gyro LPF | 1000 Hz | Same task as stabilizer after data-ready. | LPF designed at 1000 Hz | Gyro **80 Hz** LPF; accel **30 Hz** LPF on `sensorData`. | `sensors_bmi088_bmp3xx.c:61,140-141,555-556,339` | HIGH |
| BMI088 ODR | 1000 Hz | Driver config. | — | Gyro BW/ODR 1000 Hz mode. | `sensors_bmi088_bmp3xx.c:420-422` | HIGH |
| State estimator (EKF) | Predict **100 Hz**; state **read 1000 Hz** | Separate task; stabilizer copies latest `state_t` each ms. | Predict interval 10 ms | Gyro/accel subsampled into predict step. | `estimator_kalman.c:119-120,248-249,289-296,351-355` | HIGH |
| External pose (mocap) | Event-driven via CRTP | Queued into Kalman (`crtp_localization_service.c`); rate set by CS2 tracker, not read here. | — | Age at controller not instrumented in this pass. | `estimator_kalman.c:261`; `crtp_localization_service.c:216+` | LOW |
| HLC / Poly4D setpoint | **100 Hz** | `RATE_DO_EXECUTE(RATE_HL_COMMANDER, stabilizerStep)` else no new setpoint. | 10 ms | Mode E hover uses CS2 stream. | `stabilizer_types.h:386-387`; `crtp_commander_high_level.c:347` | HIGH |
| Motor output | **1000 Hz** | `controlMotors` every stabilizer tick when armed. | — | DShot: `motorsBurstDshot()` after mix. | `stabilizer.c:350-357`; `motors.c:386-389` | HIGH |
| uSD logging | Configurable (thesis **500 Hz**) | `RATE_DO_EXECUTE(usddeckFrequency(), stabilizerStep)`. | — | Default from deck config / README. | `stabilizer.c:367-372`; `usddeck.c:765-767`; `README_usd_thesis_logging.md` | MEDIUM |
| Optical RPM deck | Per blade pass (ISR) | `rpm.c` capture timers; logged values read synchronously in controller. | ~2-blade average in `calcRPM` | Investigation cited ~20 Hz **effective** at hover (log analysis); not re-measured here. | `rpm.c:211-218`; investigation §3 | MEDIUM |
| DShot RPM telemetry | Async / ESC-dependent | `rpm_get_all()` reads log vars (`motor.m*_rpm`); slew filter in OOT bridge. | — | Oct-02 A1 meta: `indi_rpm_source=1.0` (DShot). | `traj_iface.c:643-698`; flight meta | HIGH |

### 1.2 Stock Bitcraze INDI (`stabilizer.controller=3`)

| Stage | Rate | Schedule | dt | Filters / notes | Evidence | Conf. |
|---|---|---|---|---|---|---|
| Attitude INDI inner | **500 Hz** | `RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)` | `1/ATTITUDE_RATE`, `ATTITUDE_UPDATE_DT` | BW **70 Hz** on gyro & actuator states; α̇ from finite diff × **ATTITUDE_RATE** (500). Gyro **filtered** for derivative; rate error uses **unfiltered** gyro. | `controller_indi.c:67-98,153,177-249,265-268`; `controller_indi.h:44-45` | HIGH |
| Position INDI outer (cascade) | **500 Hz** (with attitude) | Same gate; `positionControllerINDI(...)` | Filter sample time `1/ATTITUDE_RATE` | Outer LPF cutoff **8 Hz** (`POSITION_INDI_FILT_CUTOFF`). | `controller_indi.c:177-181`; `position_controller_indi.c:59-70`; `position_controller_indi.h:35` | HIGH |
| Legacy PID position fallback | **100 Hz** | `POSITION_RATE` when outer disabled | `1/POSITION_RATE` | Not used when outer INDI active. | `controller_indi.c:170-171` | HIGH |
| Control output | **1000 Hz** | `control->roll/pitch/yaw/thrust` written **outside** 500 Hz gate | — | On skipped 500 Hz ticks, **u_in unchanged** but still published → implicit 500 Hz hold on increments. | `controller_indi.c:177-307,328-332` | HIGH |

### 1.3 Our OOT INDI (`controller=6`, `ctrl_mode=3`)

| Stage | Rate | Schedule | dt | Filters / notes | Evidence | Conf. |
|---|---|---|---|---|---|---|
| Full controller | **1000 Hz** | No `RATE_DO_EXECUTE` in `controllerOutOfTree` | Default `(tick−last_tick)×0.001`; optional `indi_gains.dt_usec` | **No attitude decimation.** | `lib.rs:2277-2353` | HIGH |
| Butterworth / notch | **1000 Hz** updates | Coefficients from `filt_dt_us` (default **2000 µs → 500 Hz design**) | `DT_NOM` from param or measured | **Firmware DEFAULT mismatch:** loop 1 kHz vs default filter design 500 Hz. **The flown yaml corrects it** (`filt_dt_us=1000`, `filt_prewarp=1`, `fc_bw=206`, stage 2b 2026-09-11, deliberate no-op; crazyswarm2 `crazyflies.yaml` L630–642). _Corrected in validation 2026-10-04: earlier text treated the mismatch as active in flight._ | `traj_iface.c:444-466,535-555`; `lib.rs:2246-2250,2286-2322` | HIGH |
| Angular acceleration | **1000 Hz** | `alpha_raw = Δω/dt` | tick-based or µs timestamp | Optional notch on α chain. | `lib.rs:2337-2353,1985+` | HIGH |
| Trajectory / coef service | **1000 Hz** | Same entry; comment notes 500 Hz param service | — | Mode D uses `tick×0.001` for traj time. | `lib.rs:2387-2460` | MEDIUM |
| Position + attitude INDI | **1000 Hz** | `controller_step` each armed tick | same dt | Force/torque `controlModeForceTorque`. | `lib.rs:2569-2590` | HIGH |

### 1.4 Omar C (`controller=9`, `controller_omar_indi.c`)

| Stage | Rate | Schedule | dt | Filters | Evidence | Conf. |
|---|---|---|---|---|---|---|
| Entire law | **500 Hz** | `if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) return;` | `dt = 1/ATTITUDE_RATE` fixed | **30 Hz** BW on acc/tau/α filters; init sample **1/500 s**. | `controller_omar_indi.c:156-162,130-136` | HIGH |
| Angular acceleration | **500 Hz** gate; **measured** Δt | Between gated calls: `(timestamp−timestamp_prev)/1e6` | Mixed: I-terms use fixed dt; α uses wall clock | Same file as NA-INDI reference pattern. | `controller_omar_indi.c:379-385` | HIGH |
| Control on odd ticks | **No update** | Early `return` → prior `control_t` held | 1 ms ZOH | Differs from stock INDI (always writes control). | `controller_omar_indi.c:156-158` | HIGH |
| RPM | Read when `indi` bitmask & deck | Sync log read | — | Oct-02: DShot via shared yaml (`indi_rpm_source=1`). | `controller_omar_indi.c:178-184`; flight meta | MEDIUM |

### 1.5 Omar Rust (`controller=10`, `omar_indi_rust.rs`)

| Stage | Rate | Schedule | dt | Filters | Evidence | Conf. |
|---|---|---|---|---|---|---|
| Entire law | **500 Hz** | `if tick % 2 != 0 { return; }` | `DT = 1/500` | Same **30 Hz** / **1/500 s** as C port. | `omar_indi_rust.rs:43-44,556-558,132-136` | HIGH |
| Odd ticks | No op | Same as Omar C early return | — | docs/41 §13 notes frozen `control` risk on Oot5; Init path fixed 2026-09-29. | `omar_indi_rust.rs:556-572`; `docs/41` | MEDIUM |
| RPM | OOT bridge | `oot_rpm_logs_available()` + `rpm_get_all()` | — | Deviation from C deck param probe (documented). | `docs/41` §12; `traj_iface.c:636-698` | HIGH |

### 1.6 NA-INDI port (`controller=7/8`, `naindi.rs`; reference `NA-INDI-firmware`)

| Stage | Rate | Schedule | dt | Filters | Evidence | Conf. |
|---|---|---|---|---|---|---|
| Controller | **500 Hz** | `if tick % 2 != 0 { return; }` | `DT_FIXED = 1/500` for P/I & filters | Acc **80 Hz**; τ roll/pitch **40 Hz**; yaw **10 Hz**; design sample **1/500**. | `naindi.rs:90-94,375,383,96-98,340-351` | HIGH |
| Angular acceleration | **500 Hz** executions | Wall-clock `dt_meas` for finite difference only | Mixed dt (preserved from reference) | Explicit “rate issue” documented in module header. | `naindi.rs:523-532`; module doc L7-34 | HIGH |
| Reference firmware | **1000 Hz** stabilizer, **500 Hz** gated controller | `controller_lee.c`: `RATE_DO_EXECUTE(ATTITUDE_RATE)` + `return` | `1/ATTITUDE_RATE` | Same cutoffs as port (acc/tau/yaw split). | `NA-INDI-firmware/.../controller_lee.c:178-184,133-143` | HIGH |
| Flown on brushless A1 2026-10-02 | **No** | — | — | Not in Oct-02 A1 radio set. | This study | HIGH |

---

## 2. Comparison table + differences + inconsistencies

### 2.1 Master table (representative INDI flight configs)

| Stage | Platform shared | Stock INDI (3) | Ours OOT (6, mode 3) | Omar C (9) | Omar Rust (10) | NA-INDI (7/8) |
|---|---|---|---|---|---|---|
| Stabilizer | 1 kHz | 1 kHz | 1 kHz | 1 kHz | 1 kHz | 1 kHz |
| Attitude INDI law | — | 500 Hz | **1000 Hz** | 500 Hz | 500 Hz | 500 Hz |
| Position outer | — | 500 Hz cascade / 8 Hz LPF | 1000 Hz, custom BW | 500 Hz, 30 Hz acc | same as C | 500 Hz, 80/40/10 Hz |
| Attitude α filter fc | — | 70 Hz @ 500 Hz design | **206 Hz, designed at 1 kHz (flown yaml)** | 30 Hz @ 500 Hz | 30 Hz @ 500 Hz | 40/10 Hz @ 500 Hz |
| α finite-diff dt | — | implicit 500 (`× ATTITUDE_RATE`) | 1 ms tick or µs | wall clock between 500 Hz steps | C match | fixed 500 + wall clock α |
| Control struct on skipped ticks | — | **Still written 1 kHz** | 1 kHz | **Frozen** (return) | **Frozen** (return) | **Frozen** (return) |
| Output type | — | legacy roll/pitch/yaw/thrust | force + torque SI | force + torque SI | force + torque SI | force + torque SI |
| HLC setpoint | 100 Hz | 100 Hz | 100 Hz | 100 Hz | 100 Hz | 100 Hz |
| EKF predict | 100 Hz | 100 Hz | 100 Hz | 100 Hz | 100 Hz | 100 Hz |
| Motor command | 1 kHz | 1 kHz | 1 kHz | 1 kHz | 1 kHz | 1 kHz |

### 2.2 Meaningful **differences** (timing / delay budget, not gains)

1. **Attitude INDI update rate:** Ours **1000 Hz** vs Omar / NA / stock inner **500 Hz** (`lib.rs:2277` vs `controller_omar_indi.c:156`, `naindi.rs:375`, `controller_indi.c:177`). **JUDGEMENT:** Adds at most ~0.5 ms equivalent phase vs 500 Hz ZOH at 6.3 Hz — small vs actuator lag. **Conf:** HIGH (code), MEDIUM (impact).
2. **Filter design sample rate (Ours, firmware DEFAULT only; the flown yaml sets 1000 µs + prewarp):** default `filt_dt_us=2000` (500 Hz) with **1 kHz** execution (`traj_iface.c:466`, `lib.rs:2247-2250`). **FACT** documented in `docs/22` §2f. **JUDGEMENT:** Shifts effective cutoff upward; at 6.3 Hz filter phase is still small vs 30 Hz Omar filters. **Conf:** HIGH (code), MEDIUM (impact at 6.3 Hz).
3. **INDI filter cutoff frequency:** Omar **30 Hz** vs flown ours **206 Hz** meta vs stock **70 Hz** vs NA **40/10 Hz**. **JUDGEMENT:** Omar/NA insert **more** LPF phase at 6.3 Hz than ours — opposite of “ours worst because slower filters.” **Conf:** HIGH (code), HIGH (qualitative direction).
4. **Control hold on odd ticks:** Omar C/Rust/NA **return** without updating `control_t`; stock INDI **always** assigns outputs (`controller_indi.c:328-332` vs `controller_omar_indi.c:156-158`). **JUDGEMENT:** 500 Hz hold on **SI wrench** vs legacy motor units — same 1 ms motor command cadence from stabilizer. **Conf:** HIGH (code), LOW (impact).
5. **Mixed dt (Omar/NA):** fixed `dt` for integrators, measured wall clock for α (`controller_omar_indi.c:379-380`, `naindi.rs:523-532`). Ours: single `dt` per tick unless `dt_usec` enabled. **Conf:** HIGH (code).
6. **Setpoint / estimator:** **No variant-specific difference** found — all use same stabilizer, EKF, HLC 100 Hz. **Conf:** HIGH.

### 2.3 **Inconsistencies** (design rate ≠ call rate, hardcoded dt)

| Issue | Where | Detail | Conf. |
|---|---|---|---|
| Filter coeffs @ 500 Hz, loop @ 1 kHz (**default only; not in the flown yaml**) | Ours `indi_gains.filt_dt_us` default 2000 | `docs/22` §2f; `traj_iface.c:444-466` | HIGH |
| Omar α uses wall-clock dt but I-terms use 1/500 | `controller_omar_indi.c:162,379` | Preserved in NA port | HIGH |
| Stock position INDI filters @ 500 Hz design, 8 Hz fc | `position_controller_indi.c:64-67` | Outer loop only | HIGH |
| Radio log ~20 Hz vs onboard 1 kHz | CS2 CSV | Cannot resolve >10 Hz without uSD | HIGH |

---

## 3. Delay budget @ 6.3 Hz

**Method:** `indi_loop_rates_delay_budget.py` — sums **non-exclusive** first-order / ZOH / scipy Butterworth phase at **6.3 Hz**. **Not** a Nyquist proof.

**Measured (shared):** motor lag **τ = 44 ms** → **~60°** at 6.3 Hz (**HIGH**, investigation §16 / bench).

| Variant | Σ phase (deg) | Σ equiv. delay (ms) | vs Omar C Δ phase |
|---|---:|---:|---:|
| Stock INDI (3) | 119.1 | 52.5 | −10.2 |
| **Ours OOT (6, mode 3)** | **113.3** | **49.9** | **−16.0** |
| Omar C (9) | 129.3 | 57.0 | 0 |
| Omar Rust (10) | 129.3 | 57.0 | 0 |
| NA-INDI (7/8) | 124.8 | 55.0 | −4.5 |

_Correction 2026-10-04 (validation): an earlier version of this table showed Omar C/Rust at 119.1° and NA-INDI at 119.8°; the script output (`delay_budget_summary.md`, `delay_budget_6p3hz.json`, re-run and reproduced) gives the values above._

**JUDGEMENT (MEDIUM):** With stated assumptions, **ours is not worse on paper** — **less** summed phase than every other variant (113° vs 119° stock, 125° NA-INDI, 129° Omar), mainly because **30 Hz** Omar filters add ~17° vs ~1° for our 206 Hz chain (designed at 1 kHz in the flown yaml). **Actuator lag dominates** every row (~60°). RPM path (50 Hz hold assumption) is **LOW** confidence — not read from ESC code this pass.

**Phase “available” vs oscillation:** Investigation harmonic-balance cited **~83°** actuator lag at limit-cycle amplitude (§15) — larger than small-signal τ=44 ms (~60°). **JUDGEMENT:** Linear budget understates nonlinear actuator phase; still **shared across variants**.

Full JSON: `experiments/analysis/out/indi_loop_rates/delay_budget_6p3hz.json`.

---

## 4. Flight-data check (A1, cf5, 2026-10-02)

**Source:** Radio CSV in `experiments/logs/` (not paired to uSD in this pass; meta includes `usd_start_s` for future pairing via `copy_usd_log.py`).

| Variant | n flights used | Files | Log rate | dt std (ms) | gyro_x peak (Hz) | Peak PSD (rel.) |
|---|---:|---|---:|---:|---:|---|
| Ours | 2 | `19-13-24`, `19-15-47` | ~20.0 Hz | 7.3–7.6 | **4.49–5.57** | **6×10⁵** |
| Omar C | 2 | `18-36-18`, `18-37-33` | ~20.0 Hz | 8.3–10.3 | 3.51–3.52 | ~1.3×10⁴ |
| Omar Rust | **1** | `18-50-20` | ~20.0 Hz | 7.7 | 3.28 | ~1.1×10⁴ |

**FACT:** All variants show **~20 Hz** mean sample rate (CS2 link), not 100 Hz — jitter σ **7–10 ms** (**HIGH**, script output).

**JUDGEMENT (MEDIUM):** Peaks **below 6.3 Hz** on 20 Hz logs may miss narrowband 6.3 Hz or reflect different axis/window; **amplitude** (PSD peak power) is **much higher on Ours** on both flights. **Same nominal timing stack** (20 Hz log) → rate/timing **hypothesis not supported** by log sample-rate differences between variants.

**Not compared here:** Stock INDI (3), NA-INDI (7/8) — not in Oct-02 A1 set. Omar Rust comparison rests on **one** clean flight.

Plot: `experiments/analysis/out/indi_loop_rates/fig_a1_cf5_gyro_x_psd.png`.

---

## 5. Oscillation ledger (what was tried)

| Idea | Where tested | Result | Documented |
|---|---|---|---|
| Lower / raise `fc_bw`, notch at ~7 Hz | Flight + 500 Hz uSD | Notch **inactive** at 7.2 Hz with 60 Hz BW; lowering fc_bw hurt tracking | investigation §10; `traj_iface.c:376-381` |
| `filt_dt_us` / 500 vs 1000 Hz filter design | Code + flight (stage 2b/2c, 2026-09-11) | Default mismatch confirmed; **corrected in the flown yaml** (no-op at fc_bw=206); lowering fc_bw to 100 made hover/circle worse | `docs/22` §2f |
| `dt_usec` / measured α dt | Code only | Param exists; **not flown** as isolation test | `traj_iface.c`; `lib.rs:2337-2343` |
| Attitude kr/kw sweeps | Flight | Limit cycle from ~500+ kr; kw pairs tried | investigation §15.8; Appendix B |
| ctrl_mode 3→2 (disable position INDI) | Flight (operator) | **No change** to shake | investigation §4 / §15 |
| Position integral KI_P | SIL + desk on C.1 logs | **Refuted** for Z bias; not oscillation fix | `docs/50`, `docs/51` |
| DShot vs optical RPM | Flight | DShot **broke** INDI early; optical OK; Oct-02 used DShot (`indi_rpm_source=1`) | investigation §15; flight meta |
| RPM guard / stale RPM hold | Flight | Enabling guard → crashes | investigation Appendix B fact 11 |
| j_scale / inertia mismatch | Param + desk | Proposed sweep; **limited flight** | `lib.rs:2184-2190` |
| Residual sign / a_res gating | Code fix | **Not** oscillation test | `firmware_app/CLAUDE.md` |
| Open-loop bench actuator ID | Bench | **τ≈44 ms**, fc≈3.6 Hz; amplitude-dependent lag | investigation §16; handoff |
| Omar vs Ours vs Omar Rust A1 | 2026-10-02 | **All show oscillation**; ours largest attitude/z error | meeting table; this doc §4 |
| Stock Bitcraze INDI on brushless A1 | — | **Never tested** in cited Oct-02 campaign | — |
| NA-INDI on hardware | SIL only | Sim fixed 2026-09-18; **never flown** brushless | `docs/22`; `flying_drone_stack/CLAUDE.md` |
| Z-only integral (`pos_ki_z`) | Oct-02 meta | Logged 0.0; investigation separate | flight meta |
| Prop interaction / downwash as sole cause | Multi-drone | Downwash **stress** scenario; oscillation also solo aggressive | investigation; user context |

---

## 6. Proposed experiments (no flights executed here)

Ordered by **value / effort**; each targets **rate/timing hypothesis**.

1. **SIL: duplicate stabilizer decimation** — Run CS2 SIL with OOT forced to `tick%2` vs full 1 kHz (host patch or param if added). **Support hypothesis:** oscillation frequency/amplitude moves with effective rate. **Refute:** no change. **Cost:** ~0.5 day desk.
2. **SIL: `filt_dt_us` {2000, 1000} + `filt_prewarp`** at fixed gains — **low value now: the flown yaml already uses 1000 + prewarp.** **Support:** phase/amplitude shifts at 6.3 Hz in SIL PSD. **Cost:** ~0.5 day.
3. **uSD @ 500 Hz paired flight** — Match radio CSV to card (`copy_usd_log.py`, `PAIRING.md` if present); compare `indi.dt_us`, `gyro`, `tau` PSD at 6.3 Hz **onboard**. **Support:** peak stable across variants but phase logs differ. **Cost:** ~1 lab session (operator).
4. **Omar C vs Rust A/B with logged `ctrlOot5.indi` / `ctrlOomar.indi`** — Confirm bitmask; odd-tick control hold test on bench (log `control` every 1 ms). **Cost:** ~2 h lab.
5. **NA-INDI first hover (C.0 gate)** — Same yaml timing as Omar; **500 Hz** law + mixed dt. **Support:** oscillation without our 1 kHz / filt mismatch. **Cost:** ~1 session.

---

## 7. Open questions

- Exact **DShot telemetry rate** and group delay on brushless ESC (not extracted from `motors.c` this pass). **LOW**
- **Mocap → EKF** latency distribution at 100 Hz predict vs 1 kHz control (needs log vars `estimator` stats). **LOW**
- Whether **6.3 Hz** visible on **500 Hz uSD** for Oct-02 A1 (radio peaks were 3.3–5.6 Hz). **MEDIUM** — requires paired uSD analysis.
- Stock **controller=3** on same brushless yaml — never flown in comparison set.

---

## 8. Rate/timing hypothesis verdict

**JUDGEMENT (MEDIUM confidence):** Variants **do differ** materially on **attitude loop rate (500 vs 1000 Hz)** and **filter/design-sample conventions**, but **shared platform** (1 kHz motors, 100 Hz EKF predict & HLC, measured **~44 ms** motor lag) dominates the **linear** delay budget. **Flight logs at ~20 Hz** show **no variant-dependent sample-rate difference**; **Ours shows stronger oscillation energy** despite **not** having the largest **small-signal** phase sum in §3. **Plain statement:** rate/timing differences alone **do not explain** why ours is worst on A1; they also **do not eliminate** timing as a **contributor** together with gains/torque model (see `docs/41`).

---

## Reporting checklist

| Item | Status |
|---|---|
| **Files created** | `docs/53_INDI_Loop_Rates_and_Oscillation_Ledger.md`; `experiments/analysis/indi_loop_rates_delay_budget.py`; `experiments/analysis/indi_loop_rates_flight_spectrum.py`; `experiments/analysis/out/indi_loop_rates/*` |
| **Files read (principal)** | `crazyflie-firmware`: `stabilizer.c`, `stabilizer_types.h`, `controller_indi.c`, `position_controller_indi.c`/`.h`, `controller_omar_indi.c`, `controller.c`, `estimator_kalman.c`, `sensors_bmi088_bmp3xx.c`, `rpm.c`, `usddeck.c`, `crtp_commander_high_level.c`, `motors.c`; `flying_drone_stack/firmware_app`: `lib.rs`, `omar_indi_rust.rs`, `naindi.rs`, `traj_iface.c`; `NA-INDI-firmware/controller_lee.c`; prior docs `41`, `22`, `50`, `51`, investigation timeline; Oct-02 A1 CSVs |
| **Existing files modified** | **None** (only new doc + new scripts under `experiments/analysis/`) |
| **Commit / push** | **None** |

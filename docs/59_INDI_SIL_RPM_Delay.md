# 59 — CS2 SIL with realistic RPM measurement path

Status 2026-10-04. **Investigation only** — no root-cause claim.

## Summary (read first)

| Question | Result | Confidence |
|---|---|---|
| **Does RPM delay/hold SIL reproduce flight limit cycle (ours 260–290 °/s, 4.7–5.7 Hz)?** | **No** — best sweep point **~36 °/s** at d=50 ms, f_rpm=20 Hz (order-of-magnitude low; f_dom not in band) | **SIMULATION (HIGH)** |
| **Does Step 1 give independent (d, f_rpm) for SIL?** | **Partially inconclusive** — τ( RPM ) vs α(gyro) lag **≈ 0–2 ms** on roll/pitch at 500 Hz logs; **does not** measure delay vs true rotor or deck native rate | **MEASUREMENT (MEDIUM)** |
| **Grid (res_sign × gains) on “validated” plant?** | **Not run as validated** — illustrative only if `--grid` with best sweep | **JUDGEMENT** |

**Baseline validity:** **NOT REPRODUCED** for flight A1 oscillation amplitude/frequency. Height sag vs `res_sign` from doc 58 remains; RPM path alone is **insufficient** in this SIL stack.

| Step 1 (ours roll, Oct-02 batch) | Median lag τ vs α | Coherence peak f |
|---|---:|---:|
| Ours (n=2) | **≈ −1 ms** (τ slightly leads α) | **≈ 5.5–5.9 Hz** |
| Omar C (n=2) | **≈ −9 ms** | **≈ 3.0–3.5 Hz** |

| Best SIL sweep (ours, flown gains) | d [ms] | f_rpm [Hz] | Gyro RMS [°/s] | f_dom [Hz] | Regime |
|---:|---:|---:|---:|---|
| Doc 58 (true RPM) | 0 | 1000 | **~1.7** | ~9.1 | stable (calm) |
| **Max gyro in sweep** | **50** | **20** | **36.1** | **11.4** | oscillatory, **not** flight band |
| Best score (1/4) | 10 | 20 | ~1.2 | 6.3 | Omar f only in band |

\*Regime uses fixed classifier in `indi_sil_rpm_delay_metrics.py` (doc 58 envelope bug removed).

Full tables: [`out/indi_sil_rpm_delay/sweep_summary.md`](../experiments/analysis/out/indi_sil_rpm_delay/sweep_summary.md), [`rpm_gyro_timing_summary.json`](../experiments/analysis/out/indi_sil_rpm_delay/rpm_gyro_timing_summary.json).

---

## 1 — RPM vs gyro timing from 500 Hz uSD (Step 1)

**Method (MEASUREMENT):** Paired Oct-02 A1 cf5 cards (`docs/54` pairing caveat — variant batch, not single-flight provenance). Steady window **6–13 s** scenario time after radio/uSD lag (`find_flight_window`). Reconstruct body torque from logged RPM (ours: per-motor `kt` + mixer; Omar: scalar `MOTORRPM2FORCE` equivalent). α from gyro via **2nd-order Butterworth on d(gyro)/dt** at **fc_bw** from meta (206 Hz), evaluated at **~500 Hz** log rate (approximate vs 1 kHz firmware `filt_dt_us`).

**RPM source (FACT):** Oct-02 ours meta `rpm_source: 1` → DShot (`A1_2026-10-02_19-13-24.meta.json`). Analysis uses `motor.m*_rpm` columns when present.

**Cross-correlation lag sign:** `lag_tau_leads_alpha_ms` > 0 ⇒ τ lags α; < 0 ⇒ τ leads α.

**What this identifies:** Time alignment **between logged RPM-based τ and logged gyro-based α** at limit-cycle band (high coherence **~0.99** roll/pitch ours @ **~5–6 Hz**).

**What it does NOT identify (FACT):** (i) delay of DShot/deck vs **true** rotor speed; (ii) deck-vs-DShot relative lag (that is a different analysis, `docs/meetings/2026-10-05.md` §3); (iii) gyro LPF/EKF group delay; (iv) 500 Hz controller output hold.

**RPM update behaviour at log rate:** On Oct-02 bins, **median hold length = 1 sample** @ 500 Hz for both `rpm.m*` and `motor.m*_rpm` ⇒ card sees **new value every 2 ms**, not ~20 Hz optical deck native rate (deck may update faster than hover telemetry suggests, or DShot fills every log tick).

**Script:** `experiments/analysis/indi_sil_rpm_delay_logs.py` → `out/indi_sil_rpm_delay/rpm_gyro_timing.json`.

---

## 2 — RPM measurement model in SIL (Step 2)

**Code path today (FACT):**

| Location | Behaviour |
|---|---|
| `crazyflie_sil.py` L603–605, L728–729 | Plant post-lag `state.rpm` → `motors_rpm_meas` → `firm.oot_set_rpm()` **every 1 kHz tick** — **no delay/hold/noise** |
| `flying_drone_stack/firmware_app/host/oot_host.c` L81–87 | `rpm_get_all()` returns `g_host_rpm[]` from `oot_set_rpm` on host |
| `flying_drone_stack/firmware_app/traj_iface.c` L643–698 | On **hardware**, DShot slew/abs caps when `g_indi_rpm_source != 0`; **not executed** on host SIL inject path |

**New model (SIMULATION):** `indi_sil_rpm_delay_model.py` — delay `d`, sample-hold at `f_rpm`, Gaussian noise σ=100 RPM, DShot slew filter copied from `traj_iface.c` constants. Injected in `indi_sil_rpm_delay_sim.py` on **bottom drone only** before `executeController()`.

**Omar C:** Same bottom injection → `oot_set_rpm` / `logGetUint("rpm","mN")` stub (`crazyflie_sil.py` ~784–789, `oot_host.c`).

---

## 3 — Baseline validation sweep (Step 3)

**Grid:** `d` ∈ {0, 10, 20, 30, 50} ms × `f_rpm` ∈ {1000, 100, 50, 20} Hz × {ours flown, Omar C}. Pass criterion (same bands as doc 56 §3): **≥3/4** of {ours gyro, ours f, Omar gyro, Omar f}.

**Result:** **No combination passes.** Strongest ours: **d=50 ms, f_rpm=20 Hz** → gyro RMS **~36 °/s** (≪ 260–290), f_dom **~11 Hz** (outside 4.7–5.7). Omar stays calm (< few °/s) across sweep.

**vs Step 1:** Measured τ–α lag **~1 ms** does **not** support using **50 ms** SIL delay as “log-identified” — any match would be **tuning**, not independent validation.

**Still missing (JUDGEMENT, MEDIUM):** Perfect gyro in SIL; no EKF/state-estimator delay; no 500 Hz torque hold; downwash/NS2 calibration; possible amplitude-dependent actuator model (`investigation_indi_oscillation_2026-07-21.md` §15 — “gyro fresh, tau_current stale”).

**Cheap tests not run here:** gyro noise/LPF on `sensors.gyro`; 2 ms INDI output hold — would be next knobs if RPM-only sweep stays sub-threshold.

**Script:** `indi_sil_rpm_delay_run.py` → `out/indi_sil_rpm_delay/results.json`, `sweep_summary.md`, `fig_sweep_gyro_heatmap.svg`.

---

## 4 — Gain grid (Step 4)

**Status:** **Illustrative subset only** (`grid_mode: illustrative_subset` in `results.json`) — **6 configs** at **d=50 ms, f_rpm=20 Hz** (max-gyro sweep corner), not full doc-58 grid. Baseline **not** validated (`flight_reproduced: false`).

---

## 5 — Factor table & lab test (Step 5)

| Factor | Flight / logs | SIL with RPM model |
|---|---|---|
| RPM vs α lag (roll) | ~0–2 ms @ 500 Hz log; high coh @ **~5.5 Hz** | Needs **d≈50 ms, f=20 Hz** for modest gyro rise — **not log-supported** |
| Limit cycle amplitude | 260–290 °/s | **Not reproduced** |
| Height sag vs res_sign | +1 sags | Still seen at d=0 (doc 58); RPM delay does not remove |

**Smallest lab test (text):**

1. **Log prep:** Apply [`docs/lab_prep_log_filter_params.patch`](../docs/lab_prep_log_filter_params.patch) — log `indi.res_sign`, `filt_dt_us`, `notch_en`, `filt_prewarp`, `rpm.m*`, `motor.m*_rpm`, `indi.tau_*`, `gyro.*`, `ctrlOmarIndi.torque*` at 500 Hz.
2. **Sortie A:** Flown baseline (doc 58 yaml) — confirm limit cycle + z sag.
3. **Sortie B:** `indi_gains.rpm_source: 0` (deck only) vs `1` (DShot) — **one change**, same hover — compare τ vs α lag from **onboard** logs (repeat Step 1 analysis).
4. **Abort:** |roll/pitch| > 25° for >0.5 s or z < 0.25 m.

---

## Files

**Created:** `docs/59_INDI_SIL_RPM_Delay.md`, `experiments/analysis/indi_sil_rpm_delay_*.py`, `out/indi_sil_rpm_delay/**`.

**Read (not modified):** `docs/56`–`58`, `docs/54`, `docs/55`, `docs/41`, `investigation_indi_oscillation_2026-07-21.md`, `crazyflie_sil.py`, `traj_iface.c`, `oot_host.c`, `indi_sil_package_*` (import NS2/plant only).

**Modified existing repo files:** none. **Committed/pushed:** none.

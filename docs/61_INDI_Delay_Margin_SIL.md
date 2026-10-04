# INDI delay margin in full SIL (investigation doc 61)

**Plant validity:** **partly validated** — limit-cycle-like gyro energy and dominant frequency sometimes match flight (≈5.3–6.7 Hz; gyro RMS up to **243 °/s** with spool asymmetry 1.30), but **two-robot NS2 SIL** shows **−50 cm mean z error** and `diverging` regime on most excited cases; **260–290 °/s @ 4.7–5.7 Hz** not simultaneously hit. **No root cause claimed.**

| Target (flight, doc 56 §3) | Ours A1 | Omar C |
|---|---:|---:|
| Gyro RMS (°/s) | 260–290 | 55–97 |
| Dominant f (Hz) | 4.7–5.7 | calm |
| Mean z err (cm) | ≈ −18 | ≈ +3 |

| SIL best partial match | gyro RMS | f (Hz) | z err (cm) | case |
|---|---:|---:|---:|---|
| 243 | 5.33 | −49 | `baseline_budget_ours_spool130` (SIMULATION) |
| 188 | 6.67 | −50 | `dead2_lpf80_only` / doc 60 echo |
| 0.7 | — | −0.4 | `dead0_lpf0` calm baseline |

---

## Step 1 — Hardware delay budget (desk)

**Method:** firmware constants + scipy 2nd-order Butterworth group delay at **6 Hz** (`indi_delay_margin_budget.py` → `out/indi_delay_margin/hardware_delay_budget.json`).

| Path | Element | Delay @ 6 Hz (ms) | Source | Conf. | Evidence |
|---|---|---:|---|---|---|
| gyro → controller | BMI088 **80 Hz** `lpf2p` | **3.25** | `sensors_bmi088_bmp3xx.c:140,555` | MEDIUM | FACT + SIMULATION GD |
| gyro → controller | Stabilizer tick (sense → control same 1 kHz) | **0** | `stabilizer.c:320–355` | HIGH | FACT |
| accel → residual | Accel **30 Hz** LPF | **5.49** | `sensors_bmi088_bmp3xx.c:141,556` | MEDIUM | FACT + SIMULATION |
| controller → thrust | DShot after `controlMotors` (same tick) | **~0.5** | `stabilizer.c:355` | LOW | JUDGEMENT |
| controller → thrust | ESC frame / commutation (not τ=44 ms) | **~2.0** | investigation §16 | MEDIUM | JUDGEMENT |
| plant | Motor τ=**44 ms** | (in plant) | bench ID / SIL | HIGH | MEASUREMENT |

**Totals (extra beyond τ=44 ms pole):**

| Quantity | Nominal (ms) | Uncertainty (ms) |
|---|---:|---|
| Command-path extra | 2.5 | 1.0–4.0 |
| Gyro LPF group delay @ 6 Hz | 3.25 | (fc fixed in FW) |
| **Dead-equiv. nominal (cmd + gyro GD)** | **5.75** | **2–8** |

**Bench τ=44 ms:** first-order spin-up/down fit (investigation §15–16) — **does not** replace ESC dead time; investigation treats **~2–4 ms** additional dead as separate from the pole. **Not double-counted** in SIL (`motor_tau=0.044` only).

**Gyro noise in SIL:** σ=**1.0 °/s** per axis (SIMULATION knob) — order-of-magnitude above BMI088 density **0.014 °/s/√Hz** × √80 ≈ **0.13 °/s** (datasheet-level FACT); kept conservative for grid, not tuned to flight.

---

## Step 2 — Delay margin map (full SIL)

**Simulator knobs** (`indi_delay_margin_sim.py`, new only):

| Knob | Implementation |
|---|---|
| Fractional **cmd dead time** | **Linear interpolation** on 1 kHz `(t, rpm_cmd)` history (`CmdDelay.METHOD = linear_interp_on_rpm_history`) |
| Gyro / accel LPF | 2nd-order Butterworth @ **80 / 30 Hz**, 1 kHz |
| Gyro noise | AWGN σ (deg/s) on filtered gyro |
| Sensor delay | Integer buffer on true ω, acc before LPF |
| Spool asymmetry | 1.25–1.30× slower spool-down on bottom plant only (investigation §16) |

**Ours flown gains:** kr/kw **2400/170**, res_sign **+1**, KP **64/48/5/7**, fc_bw **206**, filt_dt_us **1000**, prewarp **1**; Omar C **`oot4`**; NS2 downwash; τ=**44 ms**.

### Dead time sweep (kr 2400), with / without gyro LPF

| dead (ms) | LPF 80 Hz | Ours gyro RMS | f (Hz) | regime | Omar gyro |
|---:|---|---:|---:|---|---:|
| 0 | off | 0.7 | 2.0 | stable | 0.0 |
| 0 | **on** | **161** | **8.0** | diverging | 0.0 |
| 1.5 | off | 124 | 8.7 | diverging | 0.0 |
| 2 | off | 140 | 8.0 | diverging | 0.0 |
| 2 | on | 188 | 6.67 | diverging | 0.0 |

**Confidence:** SIMULATION, HIGH for trends; MEDIUM for absolute °/s (z diverges).

### Smallest cmd dead (ms) for limit-cycle-like behaviour **with gyro LPF 80 Hz**

| kr / kw | Margin (ms) | Notes |
|---|---:|---|
| 2400 / 170 | **0** | LPF alone → large oscillation (161 °/s @ 8 Hz) |
| 1200 / 120 | 0.5 | |
| 987 / 109 | 1.0 | |
| 600 / 90 | 1.0 | |
| 483 / 76 | **0** | LPF alone unstable |

**Cmd-only margin (LPF off, kr 2400):** first excited case **1.5 ms** (124 °/s); **2 ms** → 140 °/s @ **8 Hz** (doc 60: 233 °/s @ 7.4 Hz with integer 2-tick dead — comparable amplitude, **frequency high**).

**Omar C (embedded gains):** bottom calm **0–4 ms** dead ± LPF in this SIL (SIMULATION).

**Compare to hardware budget:** Nominal extra delay **5.75 ms** is **above** cmd-only margin **~1.5 ms** (LPF off). With **gyro LPF on**, margin is **0 ms** — the desk gyro path alone moves the SIL past the kr 2400 threshold **before** adding cmd dead. **Plain statement:** the hardware delay estimate is **not smaller** than the cmd-only margin; whether that **explains flight** remains **inconclusive** (frequency mismatch, z divergence, partner drone artefact).

---

## Step 3 — Baseline validation (budget latency, not tuned)

**Latency bundle:** cmd_dead **2.5 ms**, gyro LPF **80 Hz**, accel **30 Hz**, sensor delay **0**, gyro noise **1 °/s** (from Step 1 nominal).

| Case | Gyro RMS | f (Hz) | z err (cm) | vs flight |
|---|---:|---:|---:|---|
| `baseline_budget_ours` | 110 | 6.67 | −50 | f ok; amp low; z wrong |
| `baseline_budget_ours_spool130` | **243** | **5.33** | −49 | **amp + f closest**; z wrong |
| `baseline_dead2_lpf80_only` | 188 | 6.67 | −50 | partial |
| `baseline_budget_omar` | 1.1 | 6.0 | −0.03 | calm (bottom) |

**Frequency gap:** cmd-only dead often **7–8 Hz**; adding **80 Hz gyro LPF** or **spool-down 1.30×** pulls toward **5.3–6.7 Hz** but **does not** complete the story (height, full amplitude, single-robot calm hover).

**Label:** plant **partly validated** for attitude spectrum; **not validated** for hover height / two-robot stack.

---

## Step 4 — Grid (illustrative; budget plant)

All cases use **Step 3 budget latency** unless noted. Regime classifier: calm if gyro **< 5 °/s** (`indi_delay_margin_metrics.py`).

| Lever | Example | Gyro RMS | f | z err (cm) | Comment |
|---|---|---:|---:|---:|---|
| res_sign +1 | kr 2400 flown | 110 | 6.67 | −50 | SIMULATION |
| res_sign −1 | kr 2400 flown | 140 | 4.67 | −50 | f → band; z still bad |
| kr 987 + matched KP | card 7b | 76–110 | 4–6.7 | ≈ −50 | lower kr not calm under full budget |
| kr 483 + geom KP | card 7d | 76–147 | 4–6 | ≈ −37..−50 | |
| Gyro LPF **250 Hz** (speculative FW) | lever | 186 | 4.67 | −49 | **SIM only** — less phase lag |

**2026-09-11 card attempts 2–4:** hover **PASS** in lab at lower kr / matched gains; same combos here stay **excited** under **full delay budget** — **not reproduced** (SIMULATION vs MEASUREMENT).

---

## Step 5 — Bench + lab proposals

### Bench: measure cmd → thrust and gyro → controller latency

1. **Fixture:** one motor / arm or full quad tethered; **500 Hz** uSD log: `rpm` (DShot/deck), gyro (raw + filtered if logged), `motor` power command.
2. **Excitation:** **50–200 ms** step on motor command (or chirp **2–20 Hz** on one motor, low amplitude).
3. **Extract:** cross-correlate **command step** → **RPM** (τ + dead from fit); cross-correlate **RPM** → **gyro α** (doc 59 method); separate **gyro LPF group delay** at **5–8 Hz** from step on body rate (or compare raw vs filtered if both logged).
4. **Success criteria:** dead-time 95% CI width **< 1 ms**; repeat 5×.

### Smallest lab flight (separate sign / delay levers)

1. **Solo hover** (not A1 stack) after solo baseline PASS.
2. **Order:** (a) confirm `res_sign`, `filt_dt_us`, `notch_en`, `filt_prewarp`, RPM source logged (`docs/lab_prep_log_filter_params.patch`); (b) optional **kr 987 + matched KP** (known PASS hover); (c) **A1** only if (b) clean.
3. **Abort:** gyro RMS **> 150 °/s** sustained 2 s, |z−z_sp| **> 15 cm**, tilt **> 25°**.
4. **Do not** change gyro LPF cutoff in FW until bench delay confirmed (250 Hz lever is sim-only here).

---

## Artifacts

| Output | Path |
|---|---|
| Budget JSON | `experiments/analysis/out/indi_delay_margin/hardware_delay_budget.json` |
| Full results | `results.json`, `results.csv`, `summary.json` |
| Figures | `fig_dead_kr_margin.svg`, `fig_baseline_gyro.svg` |
| Scripts | `indi_delay_margin_{budget,metrics,sim,run,plot_svg}.py` |

**Files read (not modified):** docs 54–60, 56 §8, 41, `FLIGHT_CARD_VALIDATION.md`, `investigation_indi_oscillation_2026-07-21.md` §15–16, firmware `stabilizer.c`, `sensors_bmi088_bmp3xx.c`, prior `indi_harness_sil_gap_*`, `indi_sil_package_*`.

**Files created:** this doc + scripts/outputs above.

**Confirm:** no existing repo file edited for this task; **no commit/push**; **no flight**.

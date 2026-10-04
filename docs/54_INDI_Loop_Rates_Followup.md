# INDI loop rates — follow-up (paired uSD, harness, logging patch)

**Date:** 2026-10-04 · Builds on [`docs/53_INDI_Loop_Rates_and_Oscillation_Ledger.md`](53_INDI_Loop_Rates_and_Oscillation_Ledger.md).  
**Scope:** Read-only analysis + host harness + **unapplied** CS2 patch. No flights, no firmware edits.

---

## Validation notes (2026-10-04, independent re-check — read these first)

- **Flown filter settings are known:** `crazyswarm2/crazyflie/config/crazyflies.yaml` (stage 2b, 2026-09-11) sets `filt_dt_us=1000`, `filt_prewarp=1`, `fc_bw=206` (a deliberate no-op). So the 500 Hz filter-design mismatch is only the firmware *default*, not the flown state. The `filt_dt_us` sweep in B2 is therefore of low value, and docs/53's "mismatch" statements were corrected.
- **Pairing (A1) is only reliable per variant, not per flight.** There are **6** A1 card segments (`thesis60, 61, 64, 65, 71, 72`) and **6** radio A1 flights with a card log (`18-36-18, 18-37-33, 18-48-33, 18-50-20, 19-13-24, 19-15-47`). The table below assigns only 5 and skips `thesis65` and radio `18-48-33`; if the segments follow chronology, Omar Rust `18-50-20` is `thesis65`, not `thesis64`. A signal-correlation check (roll/pitch/z at 20 Hz) was too weak (≤ 0.5) to settle it. What is solid: the copy-time batches (18:40 → Omar C, 18:53 → Omar Rust, 19:18 → Ours) fix the **variant**. Omar Rust therefore has **two** usable segments (64, 65), not one.
- **Spectra re-computed independently** (gyro, 6–13 s, Welch 2048/1024, fs ≈ 505 Hz): peaks Omar C 3.5–3.7 Hz, Omar Rust 3.2–3.9 Hz, Ours 4.7–5.7 Hz; gyro RMS Omar 55–97 vs Ours 260–290; band 5.5–7.5 Hz share Omar 1–12 %, Ours 14–36 %. Same picture as below; no sharp 6.3 Hz line in any variant.
- **B harness: `decimate_hold` result is an artifact.** In `indi_loop_rate_harness.rs` the angular acceleration is `(ω_filt − ω_filt_prev)/dt` with `dt = 1 ms`, but on the decimated run it is updated every 2 ms, so α is estimated **2× too large**, and the filters are stepped at half rate with 1 ms coefficients. "σ → 0 / stabilizes" comes from this inconsistency, not from a real 500 Hz output hold. **Do not use it.** A correct test needs `dt = 2 ms` in the derivative and filters on decimated ticks.
- **B1 "validation" is weak:** the test only asserts σ > 0.5 rad/s and a frequency between 3 and 10 Hz. The harness gives 7.9 Hz; the card logs show 4.7–5.7 Hz for Ours and the investigation 6.3 Hz. Treat the harness as a toy.
- **Patch C verified:** `git apply --check` passes in `~/Desktop/crazyswarm2`; not applied.

---

## A1 — Pairing evidence (Oct-02 A1, cf5)

### A1. Method

| Step | Evidence type | Confidence |
|---|---|---|
| Trajectory fit | `find_flight_window.py` 3D MSE vs `A1_*.meta.json`, role `bottom` | HIGH for “contains A1” |
| Unique flight ID | Same geometry → **many bins tie** (~5 cm RMS) | HIGH that correlation alone fails |
| Session order | `thesisNN` vs wall-clock CSV order within copy batches (18:40 / 18:53 / 19:18) | MEDIUM |
| `usd_start_s` vs `lag` | Radio meta vs best lag — all pairs ~**1.0 s** offset, not discriminating | LOW as pairing key |

**FACT:** Repeated A1 runs are indistinguishable by trajectory correlation alone (`find_flight_window.py` NOTE, lines 209–224).

### A1. Assigned pairs (used for spectra below)

| Variant | n | Radio CSV | uSD bin | Trajectory RMS | Span | Status |
|---|---|---|---|---:|---:|---|
| Omar C | 1 | `A1_cf5_2026-10-02_18-36-18.csv` | `cf5_thesis60_2026-10-02_18-40-29.bin` | 6.3 cm | 16.1 s | ASSIGNED |
| Omar C | 1 | `A1_cf5_2026-10-02_18-37-33.csv` | `cf5_thesis61_2026-10-02_18-40-29.bin` | 5.0 cm | 16.1 s | ASSIGNED |
| Omar Rust | 1 | `A1_cf5_2026-10-02_18-50-20.csv` | `cf5_thesis64_2026-10-02_18-53-09.bin` | 10.0 cm | 16.1 s | ASSIGNED |
| Ours | 1 | `A1_cf5_2026-10-02_19-13-24.csv` | `cf5_thesis71_2026-10-02_19-18-55.bin` | 23.4 cm | 16.1 s | MARGINAL (high RMS) |
| Ours | 1 | `A1_cf5_2026-10-02_19-15-47.csv` | `cf5_thesis72_2026-10-02_19-18-55.bin` | 28.5 cm | 16.1 s | MARGINAL (high RMS) |

Machine-readable: `experiments/analysis/out/indi_loop_rates/usd_pairing_a1_cf5_2026-10-02.json`, `PAIRING.md`.

**JUDGEMENT (MEDIUM):** Ordering is plausible for Omar block (60→61→64) but **not proven** per flight. Ours pairs (71/72) have **poor trajectory RMS** (23–28 cm) — treat Ours uSD spectra as **supporting**, not tight validation.

**FACT:** All decoded bins log **`ctrlOmarIndi.*`** channels; **`indi.tau_*` is absent** from uSD config (decode warnings). Torque for analysis uses `ctrlOmarIndi.torquex` alias — valid for Omar C/Rust, **not** for `controller=6` OOT.

---

## A2 — 500 Hz uSD results (steady hover)

**Window (explicit):** scenario time **6.0–13.0 s** after lag alignment; excludes takeoff/landing.  
**Welch:** `nperseg=2048`, `noverlap=1024`, signals de-meaned; **fs ≈ 505 Hz** from median Δt.

### Timing regularity

| Variant | dt mean (ms) | dt std (ms) | p99 (ms) | dropouts >3 ms |
|---|---:|---:|---:|---:|
| Omar C (60) | 1.98 | 0.11 | 2.19 | 0 |
| Omar C (61) | 1.99 | 0.18 | 2.10 | 2 |
| Omar Rust (64) | 1.98 | 0.05 | 2.08 | 0 |
| Ours (71) | 1.98 | 0.04 | 2.05 | 0 |
| Ours (72) | 1.98 | 0.15 | 2.32 | 0 |

**FACT (HIGH):** Card logs are **~500 Hz** with **<0.2 ms** typical jitter — unlike **~20 Hz** radio CSVs in docs/53.

### Gyro_x spectrum (2–20 Hz)

| Variant | Peak (Hz) | Peak PSD | Energy fraction 5.5–7.5 Hz | Limit-cycle envelope |
|---|---:|---:|---:|---|
| Omar C | 3.72 / 3.44 | ~2e3 | **3.5–6.3%** | stationary |
| Omar Rust | 4.00 | ~1.2e3 | **3.4%** | growing (ratio 1.37) |
| Ours | **4.57 / 5.72** | **~6.5e4** | **24–26%** | stationary |

**JUDGEMENT (MEDIUM):** At 500 Hz, **Ours carries much more energy in the 5.5–7.5 Hz band** and peaks **closer to 6.3 Hz** than Omar (~3.4–4.0 Hz peaks). This **does not** show a sharp isolated line exactly at 6.3 Hz for every variant; peaks are **broadband-ish** in 3.5–5.7 Hz.

**FACT (HIGH):** docs/53’s **20 Hz Nyquist limit** prevented resolving this band; **500 Hz data can**.

**INCONCLUSIVE / LOW:** `tau_x` PSD for Ours — **`tau_diff_frac_near_zero = 1.0`** (constant `ctrlOmarIndi.torquex`); **do not use** for OOT INDI torque diagnosis on these files.

**INCONCLUSIVE (LOW):** 500 Hz **hold visibility** on Omar — `tau_x` updates every sample (~0.6% flat diffs on Omar 60); **cannot confirm** 500 Hz control hold from logged torque alone.

**Figures:** `fig_usd_gyro_x_psd_paired.png`, `fig_usd_gyro_x_envelope.png`.  
**JSON:** `usd_spectrum_paired.json`.

---

## B — Host harness (actuator lag + filter-rate sweep)

**File:** `flying_drone_stack/tests/indi_loop_rate_harness.rs` (extends `test_indi_actuator_lag.rs` pattern; **1-axis**, not full `lib.rs`).

**Plant (CALCULATION):** τ_act = **44 ms**, +2×1 ms dead-time, RPM-base INDI, **kr=2400**, **kw=170**, **fc_bw=206**, loop **1 kHz**.

### B1 Validation (flown yaml-style: `filt_dt_us=1000`, `prewarp=1`, `notch_en=0`)

| Metric | Result | Confidence |
|---|---|---|
| ω σ (tail 4 s) | **4.27 rad/s** | CALCULATION |
| Zero-crossing freq | **7.88 Hz** | CALCULATION |
| vs investigation ~6.3 Hz | Same order, **~1 Hz high** | JUDGEMENT MEDIUM |

**FACT:** Test **`harness_validates_limit_cycle_at_flown_gains` passes** — harness is **fit for illustrative sweeps**, not a byte-identical clone of full OOT INDI.

### B2 Sweep (`INDI_HARNESS_SWEEP=1`, 16 cells)

Output: `experiments/analysis/out/indi_loop_rates/harness_sweep.json`, plot `fig_harness_sweep_freq.png`.

| Factor | Effect on limit cycle (harness) | Confidence |
|---|---|---|
| **`decimate_hold=true` (500 Hz hold)** | σ → **0** (frozen law) — **stabilizes in this toy model** | HIGH (calc) |
| **`notch_en=1` (6.9/3 Hz)** | **Increases** σ (~12–22 vs ~3–4 rad/s) | HIGH (calc) |
| **`filt_dt_us` 2000 vs 1000** | Minor freq shift (~8.5–9.1 vs ~7.9–8.5 Hz) | MEDIUM |
| **`prewarp`** | Small σ/freq change when paired with same `filt_dt_us` | LOW–MEDIUM |

**JUDGEMENT:** In this harness, **notch ON does not remove** the limit cycle; **500 Hz output hold artificially stabilizes** because the law stops integrating. **Does not prove** real firmware hold behaves the same under full state.

**Does NOT show:** Full position loop, mocap/EKF, or DShot RPM path.

---

## C — Logging patch (not applied)

**Target:** `crazyswarm2/crazyflie_examples/crazyflie_examples/flight.py` — `_load_firmware_controller_config` + `_save_log` meta lines.

**Adds to radio CSV meta:** `indi_filt_dt_us`, `indi_filt_prewarp`, `indi_dt_usec`, `indi_rpm_source` (plus existing `fc_bw`, `notch_*`, `notch_en` via diag).

**Patch:** [`docs/lab_prep_log_filter_params.patch`](lab_prep_log_filter_params.patch)  
**Verify:** `cd ~/Desktop/crazyswarm2 && git apply --check docs/lab_prep_log_filter_params.patch` (from repo copy path) — **passes** when patch path is `crazyflie_examples/crazyflie_examples/flight.py`.

**NOT applied** to the live tree (per instructions).

---

## What this changes in docs/53’s verdict

| docs/53 claim | After follow-up | Confidence |
|---|---|---|
| 20 Hz radio cannot see 6.3 Hz | **Confirmed** — inconclusive for spectral peak | HIGH |
| Rate/timing alone explains “ours worst” | **Still no** — shared 500 Hz logging; Ours **more** 5.5–7.5 Hz energy | MEDIUM |
| Filter design mismatch (500 vs 1 kHz) | Harness: **small** freq shift vs **notch/decimation** in toy model | MEDIUM |
| Omar 500 Hz hold | **Not confirmed** on uSD torque logs | LOW |

**Open questions**

1. uSD config without `indi.*` for OOT flights — need config snapshot per session.  
2. Tighter pairing (e.g. `usd.runTag` vs meta) for Ours 71/72.  
3. Full-host SIL linking compiled `controllerOutOfTree` (future).

---

## Files created

- `docs/54_INDI_Loop_Rates_Followup.md` (this file)  
- `docs/lab_prep_log_filter_params.patch`  
- `experiments/analysis/indi_loop_rates_usd_pairing.py`  
- `experiments/analysis/indi_loop_rates_usd_spectrum.py`  
- `experiments/analysis/indi_loop_rates_harness_plot.py`  
- `flying_drone_stack/tests/indi_loop_rate_harness.rs`  
- `experiments/analysis/out/indi_loop_rates/*` (pairing JSON, spectra JSON, figures, harness sweep)

## Files read (principal)

Oct-02 A1 CSV/meta; `cf5_thesis60/61/64/71/72` bins; `decode_usd_log.py`, `find_flight_window.py`; `test_indi_actuator_lag.rs`; investigation §16; `crazyswarm2/.../flight.py`; docs/53.

## Modified / committed

**No existing project files modified.** **No git commit or push.**

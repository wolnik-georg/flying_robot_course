# 27 — Analysis, Metrics and Visualisation: audit and plan

Audit of what the analysis toolchain already produces against what a defensible thesis
comparison (C.4) actually needs, with the gaps ranked. Written 2026-09-18, before C.1, so the
gaps can be closed while the lab work proceeds rather than discovered at writing time.

---

## What already exists — a better foundation than expected

**`experiments/analysis/metrics.py`** — per-vehicle rows:

- `pos_rmse_m` and per-axis `pos_rmse_x/y/z`, `pos_peak_m`, `pos_peak_z`
- `a_res_rms`, `a_res_z_mean`, `a_res_z_rms` — the measured interaction force
- **`a_hat_res_rms`, `a_hat_vs_a_res_rmse`** — predicted vs measured residual. This is *the*
  evaluation metric for any learned residual model, and it already exists.
- `e_R_rmse`, `e_R_peak` — attitude error
- `nan_fraction`, `rate_hz_est`, `n_samples`, `n_raw` — data-quality guards
- `formation_row`: `dz_mean_m`, `dz_err_mean_m`, `dz_err_rms_m`, with an explicitly documented
  sign convention and interpolation onto the slower vehicle's grid

**`plot_flight.py`** — XY path vs commanded, per-axis vs t, tracking error, vertical separation,
and a predicted-vs-measured residual panel.

**Supporting**: multi-format loaders (`ros` / `merged` / `usd` / sim), `test_metrics.py` with
fixtures, `compare_downwash.py` (2026-09-18), `analyze_usd_hover_shake.py`, `find_flight_window.py`.

**Assessment: the per-run measurement layer is in good shape.** The gaps are almost entirely in
(a) turning many runs into a defensible claim and (b) metrics a controls examiner will expect
that nobody has needed yet.

---

## Gaps, ranked

### 1. No statistical aggregation across repeats — **highest priority**

Everything currently reports **single-run** numbers. `summarise_matrix.py` prints one row per
case; there is no mean ± std, no confidence interval, no paired test.

C.4's claims ("INDI reduces tracking error 2.9×") are **not defensible from one run each**.
Tonight's geometric-vs-INDI result is explicitly labelled *indicative, not statistical* for
exactly this reason.

**Needed:** an aggregation layer that groups by (scenario, controller), reports **mean ± std and
n**, and runs a **paired test across matched scenarios** (Wilcoxon signed-rank is the safe
choice — small n, no normality assumption). Report effect size, not just a p-value.

### 2. No control-effort metric — **high priority, and examiners will ask**

Nothing measures what the controller *spends*. A controller that tracks better while commanding
far more actuator authority is not straightforwardly better, and a comparison that omits this is
incomplete by the standards of the controls literature.

**Needed:** thrust RMS, torque RMS (`tau_x/y/z` already logged), **motor saturation fraction**
(time at PWM ceiling), and an energy proxy (∫ thrust dt or ∫ Σ PWM² dt). All derivable from data
already collected — this is an analysis gap, not a logging gap.

### 3. No frequency-domain attitude metric — **high priority, quantifies a finding you already have**

The project has a documented 6–8 Hz attitude limit cycle, and tonight's comparison found
**gyro std essentially unchanged** between geometric (58.95) and INDI (58.53) while attitude
*excursion* fell 2.6×. That is a real and interesting result — INDI suppresses the low-frequency
disturbance response but not the high-frequency limit cycle — and right now it can only be
described, not quantified.

**Needed:** attitude/gyro PSD, **dominant frequency and its power**, and **band-limited power in
5–9 Hz** as a single comparable scalar per run. `analyze_usd_hover_shake.py` does related work
but is not integrated into the comparison pipeline.

### 4. No phase segmentation

Metrics currently span the whole logged window, mixing ramp, cruise, crossing and landing. The
physically meaningful number for downwash is the **closest-approach window**. Tonight that
window (`t = 11–25 s`) was chosen by hand.

**Needed:** phase segmentation as a first-class concept — `ramp` / `scenario` / `closest_approach`
/ `land` — derived from the scenario definition and the meta sidecar, with every metric
reportable per phase.

### 5. No minimum-separation / safety metric

For close-proximity formation flight, **minimum achieved separation** is a headline safety number
and a natural axis for "how close can each controller safely fly?" — which is arguably the
thesis's practical payoff. `formation_row` gives mean and RMS `dz` error but not the minimum.

**Needed:** `min_separation_m`, `min_dz_m`, time-below-threshold, and realized-vs-commanded
closest approach.

### 6. No disturbance normalisation

Two runs are not comparable if one experienced stronger downwash — and tonight's pair differed
measurably (`a_res_z` peak −2.81 vs −4.14 m/s²). Reporting tracking error *alone* across runs
with different realized disturbance is not a like-for-like comparison.

**Needed:** always report realized disturbance magnitude alongside tracking error, and consider a
normalised figure (error per unit residual) as a secondary metric. Flag comparisons where
realized disturbance differs by more than ~20 %.

### 7. No publication figures for the comparison itself

`compare_downwash.py` prints numbers; there is no figure. The thesis needs, at minimum:

- grouped bar chart, controllers × metrics, **with error bars** (depends on gap 1)
- time-series overlay of the same scenario flown by different controllers
- `a_res` predicted vs measured scatter with R², plus an error histogram
- **residual-model error vs `dz`** — does the model degrade at close range, i.e. exactly where it
  matters most? (for C.2)

### 8. Residual-model evaluation is thin

Only `a_hat_vs_a_res_rmse` and one plot panel. For C.2 that is not enough to claim the model works.

**Needed:** R², error distribution, error vs `dz` and vs relative velocity, **train/val/test split
by flight** (not by shuffled sample — adjacent samples in one flight are not independent, and
shuffling leaks), and a **baseline comparison against predicting the mean** (`train.py` already
prints a predict-zero baseline, which is the right instinct — extend it).

---

## Plan, in dependency order

| Phase | Work | Blocks |
|---|---|---|
| **P1** | ✅ **DONE 2026-09-18.** Control effort, attitude spectrum and min-separation added to `metrics.py`, wired into `vehicle_metrics`/`formation_row`, loaders extended (`gyro`/`tau` for ros, `+motor` for uSD) | — |
| **P2** | Phase segmentation, driven by the scenario definition + meta sidecar | P1 metrics become per-phase |
| **P3** | Aggregation layer: mean ± std, n, Wilcoxon paired test, effect size | needs repeats from the lab, but can be written and tested on synthetic/existing data now |
| **P4** | Figure generation for C.4 | P3 |
| **P5** | Residual-model evaluation suite (R², error vs dz, flight-wise split) | C.1 data |

**P1 and P2 are pure desk work on data that already exists and should be done first.** P3 can be
written and unit-tested before the repeats exist. Only P5 genuinely waits on C.1.

---

## Logging coverage — checked 2026-09-18, and it is complete

`flying_drone_stack/tools/usd_thesis_config.txt` logs **38 variables at 500 Hz**, and every gap
above is an *analysis* gap, not a logging gap. Confirmed present:

- `motor.m1`–`m4` — **PWM ratios**, exactly what `motorsGetRatio()` returns. Control-effort
  metrics are supported, **and so is `controller=9` retraining** (docs/26), which needs this
  rather than `motor.m*_rpm`.
- `indi.tau_x/y/z` — torque, for control effort.
- `gyro.x/y/z` and `acc.x/y/z` at 500 Hz — enough bandwidth for the 5–9 Hz PSD work in gap 3
  (Nyquist 250 Hz, far above the band of interest).
- `indi.a_res_*` — the measured interaction force, the thesis's core signal.
- `rnn.pred_x/y/z` + `rnn.clamped` — the residual model's own prediction, logged even when
  `rnn.en=0`, which is what makes predicted-vs-measured evaluation possible on flights where the
  prediction is *not* used.
- `indi.e_r_*` + `e_r_norm` — attitude error.
- `ctrltarget.x/y/z` — commanded position, so uSD tracking error needs **no trajectory
  reconstruction** at all (unlike the radio CSVs, where `compare_downwash.py` must rebuild it).

**Nothing needs adding before C.1.** Every metric proposed above is computable from data the
current config already captures.


---

## P1 implementation notes (2026-09-18) — two findings from first real use

`control_effort()`, `attitude_spectrum()` and `separation_metrics()` are implemented in
`metrics.py` and folded into `vehicle_metrics()` / `formation_row()`. `VehicleLog` gained
`tau`, `motor` and `gyro`; the uSD loader populates all three, and the **ros loader now
populates `gyro` and `tau`, which `ROS_HEADER` always listed but nothing ever read**. Verified
on synthetic data (a 7 Hz line injected into the gyro is recovered at exactly 7.00 Hz) and then
on the real 2026-09-18 A8 pair — which immediately surfaced two things worth knowing:

**1. Frequency-domain analysis needs the uSD stream, not the radio CSVs.** The radio logs run
at **20 Hz** (dropped from 100 for 2-drone bandwidth headroom), so Nyquist is 10 Hz — barely
above this band's 9 Hz top, with everything above 10 Hz folding straight back into it. The first
run produced a confident-looking "dominant 7.17 Hz, 82 % band power" for INDI that is **not
trustworthy**. `attitude_spectrum` now requires genuine oversampling (`fs ≥ 4 × band_hi`) rather
than a bare Nyquist pass, and reports `spectrum_ok` so a caller cannot silently consume a bad
number. **The 500 Hz uSD stream clears this comfortably — use it for any frequency-domain claim.**

**2. `tau_*` is not a controller-agnostic effort metric.** `indi.tau_*` is the INDI path's own
commanded torque and reads **exactly zero under geometric** (`ctrl_mode=0`), so comparing
`tau_rms` across those two controllers compares a real number against a structural zero. The
controller-agnostic effort signal is the **per-rotor PWM ratio** (`motor.m1-4`), which every
controller drives — and which is **uSD-only**, absent from the radio CSV schema. Use `motor_*`
for cross-controller effort claims; treat `tau_*` as an INDI-internal diagnostic.

**Consequence for C.4: the comparison must be built on uSD logs, not radio CSVs.** Both of the
metrics added here need signals or rates the radio stream does not carry. That is a protocol
decision worth making explicitly now rather than discovering during writing.

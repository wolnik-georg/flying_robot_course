# 27 — Analysis, Metrics and Visualisation: audit and plan

Audit of what the analysis toolchain already produces against what a defensible thesis
comparison (C.4) actually needs, with the gaps ranked. Written 2026-09-18, before C.1, so the
gaps can be closed while the lab work proceeds rather than discovered at writing time.

**Desk quick start (2026-09-21):** [`31_Desk_Parallel_Track.md`](31_Desk_Parallel_Track.md) ·
run `python3 experiments/analysis/run_c4_desk_prep.py` on archived phase metrics.

**Controller column policy (2026-09-21):** phase CSVs carry **`study_controller`** (what was
flown on cf5) and **`controller`** (metrics label). Stock-Lee-on-both baselines must stay
`stock_lee` — never `geometric`. C.4 summarise/plots use the compare subset only
(`run_c4_desk_prep.py` → `phase_metrics_c4_compare.csv`). Details: [`35`](35_Desk_Parallel_Audit_Closeout.md).

---

**Operational workflow (match → package → suite):** `docs/28_USD_Radio_Matching_and_Session_Analysis.md`.

## Policy: uSD logs are the analysis source of record

**Decided 2026-09-18.** Every metric, plot, comparison row and thesis figure is built from the
**uSD stream**. The radio CSVs are for **live monitoring and quick troubleshooting only** and
must not be the basis of a reported result.

This is not a preference, it is forced by what each stream can carry:

| | uSD (`usd_thesis_config.txt`) | radio CSV |
|---|---|---|
| Rate | **500 Hz** | **20 Hz** for 2-drone runs (dropped from 100 for bandwidth headroom) |
| Frequency-domain work (5–9 Hz band) | ✅ Nyquist 250 Hz, comfortable | ❌ Nyquist 10 Hz — the band's own top edge; content above 10 Hz aliases into it |
| `motor.m1-4` PWM ratios | ✅ present | ❌ absent from the schema |
| Cross-controller control effort | ✅ via `motor_*` | ❌ only `tau_*`, which is **structurally zero** under geometric |
| Commanded position | ✅ `ctrltarget.*` logged directly | ❌ must be reconstructed from the scenario |
| Dropouts | none — written locally | packets dropped silently under load |

Practical consequences:

- **uSD logging must be on for every run that will be reported**, comparison and collection
  alike. `check_usd_deck.py` before flying is already the standing rule.
- `compare_downwash.py` currently reads radio CSVs and reconstructs the commanded trajectory.
  That was right for the 2026-09-18 runs (no uSD extraction yet) but it should move to uSD
  input, where `ctrltarget.*` removes the reconstruction step entirely — strictly more
  trustworthy, since it is the setpoint the firmware itself acted on.
- A radio-only run is still useful for "did it fly clean?", and useless for "how clean, exactly?"

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
| **P2** | ✅ **DONE 2026-09-18.** `approach_times()`, `phase_windows()`, `slice_log()`, `vehicle_metrics_by_phase()` — every metric now reportable per phase | — |
| **P3** | ✅ **DONE 2026-09-18.** `aggregate.py` — mean±std/sem, paired Wilcoxon, bootstrap CI on the ratio, rank-biserial effect size, plus small-n and pseudo-replication guards | needs repeat FLIGHTS to produce a claim |
| **P4** | ✅ **DONE 2026-09-18.** `plot_comparison.py` — grouped bars with SEM error bars, per-phase breakdown, n annotated, honesty rules enforced in code | — |
| **P5** | ✅ **Desk suite 2026-09-21.** `flying_drone_stack/tools/residual/eval_model.py` — R², mean/zero baselines, contiguous val blocks, per-flight metrics, binned error vs $d_y$/ $d_z$; optional plots | meaningful numbers need **C.1 volume**, dry-run OK on rehearsal weights |

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


---

## P2 implementation notes (2026-09-18) — one bug caught, one finding exposed

`phase_windows()` segments a log into `ramp` / `scenario` / `approach<k>` / `land`, derived from
the scenario definition and the meta sidecar rather than chosen by hand. `slice_log()` cuts every
array field together so metrics cannot silently mix windows, and `vehicle_metrics_by_phase()`
returns one tagged row per phase.

**Design decision worth keeping:** the approach window is computed from the **commanded**
trajectory, never from measured positions. A window defined by what each vehicle actually did
would differ per controller — each would then be scored over a window its own performance chose,
which is precisely the silent bias that makes a comparison indefensible. The command is identical
across controllers by construction, so the window is too.

**Bug caught during testing.** The first version took a single `argmin` of commanded separation
and reported one crossing. A8 with `passes=2` crosses **twice**, at the same 0.25 m minimum, so
`argmin` chose between them on floating-point noise — returning t=11.02 s where
`run_formation.py` itself reports the first crossing at **t=5.0 s**. Half the interaction data
would have been dropped from every A8 row, invisibly. `approach_times()` now returns every
crossing (local minima within 15 % of the global), numbered in time order, and its first value
matches `run_formation.py`'s exactly.

**Finding this exposed, on the 2026-09-18 A8 pair** (bottom drone, position RMSE):

| Phase | geometric | full INDI | ratio |
|---|---|---|---|
| whole `scenario` | 103.5 mm | 35.6 mm | **2.9×** |
| `approach1` | 119.7 mm | 54.5 mm | 2.2× |
| `approach2` | 100.9 mm | 52.3 mm | 1.9× |

**INDI's advantage is *smaller* during the actual crossings (1.9–2.2×) than over the scenario as
a whole (2.9×)**, and both controllers are worse there in absolute terms. So INDI does relatively
better in the easy parts and relatively worse exactly when the disturbance is strongest — the
opposite of the naive expectation, and invisible in a whole-window number.

That matters for the thesis argument: **the headroom available to the learning-based strategies is
concentrated in the crossing**, where INDI is weakest relative to its own average. It also gives a
sharper target than "beat INDI" — beat it *at closest approach*.

Pass-to-pass consistency is reassuring (INDI 54.5 vs 52.3 mm; geometric 119.7 vs 100.9 mm), which
is itself a useful data-quality check that only per-crossing reporting can provide.


---

## P3 implementation notes (2026-09-18) — and a methodological trap it caught

`aggregate.py` provides `summarise()` (mean ± std ± sem, median, IQR, **n**) and
`paired_compare()` (ratio with a bootstrap CI, matched-pairs rank-biserial effect size, and a
Wilcoxon signed-rank p-value **when the design earns one**).

**Choices, and why:**

- **Wilcoxon, not a t-test.** n is a handful, the metrics are RMSEs (bounded below,
  right-skewed), and normality is neither plausible nor checkable at this n. Wilcoxon assumes
  only symmetry of the paired differences.
- **Paired, not independent.** Runs are matched by (scenario, phase) — the same commanded
  trajectory flown by each controller. Pairing removes scenario-to-scenario variance, which is
  large here and would otherwise swamp the controller effect.
- **Bootstrap CI on the ratio, not Gaussian propagation.** The quantity of interest is a *ratio*
  of RMSEs; its distribution is asymmetric and propagation understates the upper tail. A
  percentile bootstrap over pairs assumes nothing.
- **Effect size always, p-value conditionally.** Below n=6 the p-value is suppressed with an
  explicit note, because at that n a Wilcoxon cannot reach conventional significance *regardless
  of effect size* — printing one invites a reader to conclude "no effect" from "not enough runs".
- A CI straddling 1.0 triggers an explicit warning that no difference has been established.

Verified on three synthetic designs: small-n with a real effect (reports ratio + CI, withholds
p), adequate-n with a real effect (p = 0.0078, CI excludes 1.0), and adequate-n with **no** effect
(ratio 0.993, CI [0.95, 1.04], warning fires, p = 0.64). It declines to overclaim in exactly the
cases where it should.

### ⚠️ The trap: phases are not repeats

Running it on the real 2026-09-18 pair produced `n_pairs=5` — because the five *phases* of one
flight were being treated as five independent samples. The arithmetic is happy to produce a
confident CI and, past n=6, a p-value from that, but it would be **measuring within-flight
variation and reporting it as between-flight evidence**. This is pseudo-replication, and it is
the single easiest way to publish an indefensible number from a correct-looking pipeline.

`paired_compare()` now checks the `run` column and, if either controller has only one distinct
flight, flags it loudly and suppresses the p-value regardless of n.

**Direct consequence for the flight plan: repeats must be repeated FLIGHTS.** Three A8 runs per
controller is a design that can support a claim; one A8 run per controller analysed five ways is
not, however many phases it is sliced into. That should be budgeted into the C.4 comparison
sessions explicitly — roughly 3–5 flights per controller per scenario, not one.


---

## P4 implementation notes (2026-09-18) — honesty rules in code, and a third trap

`plot_comparison.py` produces the thesis figures: grouped bars per metric, and a per-phase
breakdown. It reads the **same per-run rows** `aggregate.py` does, so a figure and its table can
never disagree, and it prints the matching summary table alongside every figure.

**Honesty rules enforced in code, not left to discipline:**

- **Error bars are SEM across repeat FLIGHTS**, never across phases of one flight. With n=1 there
  is no measured spread, so the bar is drawn **hollow and hatched** with `n=1*` on its face — a
  single run can never be mistaken for a measured mean.
- **n is printed on every bar.** A reader never has to look elsewhere to see how much data it rests on.
- **Axes always start at zero.** Truncated bar axes exaggerate differences and are the commonest
  way an honest number becomes a misleading picture.
- **Nothing is normalised away** — absolute units on the axis; ratios belong in the caption.
- Okabe-Ito colour-blind-safe palette, with each controller keeping its colour across every
  figure so the mapping is learned once.

### Third trap, caught the moment the first figure was drawn

The initial by-phase figure showed `ramp` at 0.72 m and `land` at 0.79 m, dwarfing the 0.04–0.10 m
in-scenario bars and rendering the plot useless. Those numbers are **meaningless**: outside the
scenario window the vehicle follows the *takeoff / goTo / land* profile, not the scenario curve,
so a caller who reconstructs `pos_des` from the scenario can only clamp it at the endpoints — the
"error" is the distance from a setpoint that was never commanded.

`vehicle_metrics_by_phase()` now emits `cmd_logged` and `pos_err_valid` per row, and annotates the
void rows; `plot_comparison.py` drops them with a printed count rather than silently. The
underlying effort, spectrum and attitude metrics in those rows remain valid — only position error
is void, so the row is flagged rather than discarded.

**This is a third independent argument for the uSD-as-source-of-record policy**: uSD logs carry
`ctrltarget.*`, the setpoint the firmware actually acted on, so position error is real in *every*
phase and no reconstruction — and no exclusion — is needed at all.

With the guard in place the figure reads cleanly and shows the P2 finding at a glance: INDI's
advantage is visibly narrower at `approach1`/`approach2` than over the whole `scenario`.

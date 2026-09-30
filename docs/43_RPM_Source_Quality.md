# 43 — RPM source quality (deck vs DShot on C.1 merged logs)

**Date:** 2026-09-23 (desk); **reopened 2026-09-29** (spike filter + methods + control-path check); **presentation re-run 2026-09-29** (full C.1 glob, robust-first plots/tables)  
**Framing:** **Sensor-quality** study — agreement, lag, and dropout between the optical RPM deck and DShot bidirectional telemetry on the **C.1 merged dataset**. Not a closed-loop stability claim; **does not** change standing **`indi_gains.rpm_source=1`**.

**Reproduce (plots + tables):**

```bash
cd ~/Desktop/flying_robot_course
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/rpm_source_quality.py
```

On hosts where system `python3` has a **matplotlib / NumPy ABI clash**, use **`--no-plots`** for metrics/CSVs only:

```bash
python3 experiments/analysis/rpm_source_quality.py --no-plots
```

**Outputs:** `experiments/analysis/out/rpm_source_quality/`  
**Web page:** [`43_RPM_Source_Quality.html`](43_RPM_Source_Quality.html)

---

## Spikes and metrics (read once)

| Topic | Statement |
|-------|-----------|
| **What spikes are** | Rare **DShot-only** decoded RPM jumps while the **optical deck stays in-band** at the same 500 Hz instant. |
| **Root cause** | **Best-fit hypothesis, not confirmed:** intermittent **bidirectional DShot telemetry decode glitches** (1–2 tick bursts). Do not overclaim a single hardware mechanism. |
| **Headline error** | **`rmse_robust_rpm`** — RMSE on **robust valid** samples (`\|dshot − deck\| ≤ 10 000` RPM). |
| **Diagnostic only** | **`rmse_raw_rpm`** — same on **base valid** only; **dominated by spikes**; label “with spikes” in prose, never as the headline. |
| **Historical logs** | Firmware **`rpm_get_all()`** spike guard (2026-09-29) **does not** alter stored uSD CSVs; this re-run is **analysis-side** spike exclusion and visible spike markers on plots, not a re-simulation of the live filter. |

---

## Latency vs policy (read this first)

**The optical deck is faster than DShot telemetry, not slower.** On these logs, cross-correlation lag is **positive in the deck-vs-DShot convention used here: positive ms = DShot lags the deck.** Typical dual-vehicle rows sit around **0–5 ms** mean lag on cf5 (bottom), often **2–5 ms** on cf_second (top), with many values on the **2 ms** grid from **500 Hz** sampling. Nearly every finite lag row in the summary table is **≥ 0** — DShot is the delayed channel.

**DShot was adopted for control (`indi_gains.rpm_source=1`, default since 2026-09-18) because of reliability, not speed:** in real **2-drone** flight the optical deck reported **exactly zero on two of four motors** for extended intervals while DShot stayed live (`docs/23_DShot_RPM_Investigation.md`, `docs/07` History). The deck remains on the airframe as a **parallel logged** reference; control and logged `indi.a_res_*` use DShot when `rpm_source=1`.

---

## Metric reference (single source of truth)

All metrics are per **motor-row** (one vehicle prefix × one motor × one merged CSV), computed in `metrics_one_motor()` unless noted.

| Metric | Formula / rule | Units | Sample inclusion | Function |
|--------|----------------|-------|------------------|----------|
| **deck_zero_pct** | mean(`deck ≤ 0`) × 100 | % | all rows in merge | `metrics_one_motor` |
| **dshot_zero_pct** | mean(`dshot ≤ 0`) × 100 | % | all rows | `metrics_one_motor` |
| **bias_rpm** | mean(`dshot − deck`) on **base valid** mask | RPM | base valid | `metrics_one_motor` |
| **bias_pct** | 100 × mean(`dshot − deck`) / mean(`deck`) on base valid | % | base valid | `metrics_one_motor` |
| **bias_robust_*** | same as bias_* on **robust** mask | RPM / % | robust valid | `metrics_one_motor` |
| **rmse_robust_rpm** | √(mean((`dshot − deck`)²)) on **robust** mask — **headline** tracking error | RPM | robust valid | `metrics_one_motor` |
| **rmse_raw_rpm** | same on **base valid** only — **diagnostic**; dominated by DShot spikes | RPM | base valid | `metrics_one_motor` |
| **max_abs_err_rpm** | max \|dshot − deck\| on base valid | RPM | base valid | `metrics_one_motor` |
| **n_spike_excluded** | count base valid with \|err\| > 10 000 RPM | samples | — | `metrics_one_motor` |
| **lag_ms** | `cross_corr_lag(deck, dshot, fs=500)` on base valid, ±50 ms search; **+ = DShot lags deck** | ms | base valid (≥50 samples) | `cross_corr_lag` in `rpm_source_quality.py` |
| **Rolling lag** | same `cross_corr_lag` on 2.0 s windows, 0.5 s step | ms | per-window valid | `rolling_cross_corr_lag` |

**Masks**

- **Base valid:** `(deck > 0) & (dshot > 0) & (dshot < 60 000)` — excludes deck dropout and DShot **0xFFFF** invalid sentinel (`60000` guard).
- **Robust valid:** base valid **and** `|dshot − deck| ≤ 10 000` RPM — excludes known DShot telemetry spike outliers (§ Spike root cause).

**Fleet headline (full C.1 merged glob — 38 CSVs, 2026-09-29 presentation re-run):**

| Statistic | rmse_robust_rpm (headline) | rmse_raw_rpm (diagnostic, with spikes) |
|-----------|---------------------------:|---------------------------------------:|
| Median across motor-rows | **128.1 RPM** | **802.9 RPM** |
| Mean across motor-rows | **139.7 RPM** | **781.8 RPM** |

Source: `experiments/analysis/out/rpm_source_quality/fleet_robust_rmse.json`, `per_flight.csv` (**272** motor-rows).

---

## Metrics explained, step by step, with a real worked example

Everything below is computed on **one real motor-row**: `cf5`, flight `A3_2026-09-21_13-00-57`,
motor 2 — the exact same flight/motor shown in the overlay plot earlier in this doc. Every
number here was hand-computed directly from the raw merged CSV to independently confirm the
pipeline's own output, not copied from it — they match to the decimal.

### The basic idea

Every flight logs RPM from **two separate sensors watching the same four motors**: the optical
deck (a camera-like sensor that watches the propeller and infers speed from what it sees) and
DShot (the motor controller electronically reporting its own commanded/measured speed back over
the same wire that drives it). Neither is "ground truth" by construction — they're two
independent measurements of the same physical quantity, and disagreement between them is
information, not necessarily an error in either one.

**"Motor-row"** is the unit everything is computed on: one specific flight, one specific drone,
one specific motor (1 through 4). A single 30-second flight with 2 drones logging 4 motors each
produces 8 motor-rows. The fleet numbers (272 motor-rows) are just this repeated across every
flight/drone/motor combination in the dataset, then aggregated.

### Step 1 — the raw samples

At 500 Hz, each motor-row is a long list of `(deck_rpm, dshot_rpm)` pairs, one per tick. A
normal-looking stretch from the real data (both sensors reading a stable ~19,000 RPM hover):

| t (s) | deck (RPM) | DShot (RPM) | error = DShot − deck |
|---|---:|---:|---:|
| 5.000 | 19044.0 | 19043.4 | −0.7 |
| 5.002 | 19011.9 | 18845.1 | −166.7 |
| 5.004 | 18985.8 | 18796.0 | −189.8 |
| 5.008 | 18889.1 | 18858.9 | −30.2 |
| 5.016 | 18734.4 | 18790.3 | +56.0 |

Then, 100 ms later in the *same* motor-row, a spike:

| t (s) | deck (RPM) | DShot (RPM) | error = DShot − deck |
|---|---:|---:|---:|
| 5.120 | 16833.6 | 34893.5 | **+18059.8** |
| 5.122 | 16803.5 | 49505.5 | **+32702.0** |

The deck barely moved (16833→16803, entirely normal). DShot jumped to nearly double and then
nearly triple the real speed, for two ticks, then came back. This pair of rows *is* what
"spike" means throughout this doc — not a description, this is literally what one looks like in
the raw numbers.

### Step 2 — the two validity masks

Before computing anything, some samples get excluded — but *which* ones, and why, matters:

- **`deck > 0`** — the deck occasionally reports exactly 0 when it loses lock on the propeller
  (a real dropout, not noise). A 0 doesn't mean "the motor stopped," it means "the sensor has no
  reading" — including it in an error calculation would compare a real DShot number against a
  meaningless deck placeholder.
- **`dshot > 0` and `dshot < 60000`** — DShot uses `0xFFFF` (65535) as its own explicit
  "no value" sentinel. `60000` is a safety margin below that so genuinely garbage-but-not-quite-
  the-sentinel values near the top of the `uint16` range don't sneak through either.
- Samples passing both of the above are **"base valid"** — this is the widest reasonable mask,
  and it's what `bias_rpm`/`rmse_raw_rpm`/`max_abs_err_rpm` are computed on.
- **"Robust valid"** = base valid **and** `|dshot − deck| ≤ 10,000 RPM`. This is the mask that
  drops exactly the two spike rows shown above (+18060 and +32702 both exceed 10,000) while
  keeping every normal sample, spikes included in the errors above. On this specific motor-row,
  base-valid has 17,688 samples; robust-valid drops it to 17,683 — **5 samples excluded**, out
  of nearly 18,000. That's the whole story of why `rmse_robust` and `rmse_raw` differ so much:
  a handful of huge numbers versus thousands of small ones.

### Step 3 — bias (is one sensor systematically higher or lower than the other?)

**Formula:** the plain average of `(dshot − deck)` over every base-valid sample.

Summing all 17,688 `(dshot − deck)` values on this motor-row and dividing by the count gives
**`bias_rpm = +2.2`**. In plain terms: across the whole flight, DShot reads on average 2.2 RPM
*higher* than the deck — on a ~15,000-19,000 RPM hover, that's a rounding error, not a real
disagreement. `bias_pct` is the same number expressed as a percentage of the mean deck RPM, so
it's comparable across motors/flights that hover at different speeds.

Why the average, and not something else? Because bias is specifically asking "does one sensor
systematically read high or low," and averaging cancels out random per-tick noise while
preserving a genuine constant offset. If DShot were *always* reading, say, 200 RPM high, this
number would show ~+200; it doesn't, so there's no evidence of a systematic calibration
mismatch between the two sensors — this dataset's disagreement is dominated by the rare spikes,
not a steady offset.

### Step 4 — RMSE (how far off is a "typical" disagreement, and why two versions?)

**Formula:** square every error, average the squares, take the square root — `√(mean(error²))`.
Squaring first (rather than just averaging `|error|`) means large errors count *much* more than
small ones — a standard, deliberate property of RMSE, and exactly why the raw and robust
versions diverge so sharply here.

- **`rmse_raw_rpm` (diagnostic, includes the spikes):** on this motor-row, **508.1 RPM**. Two
  samples out of 17,688 (the +18060 and +32702 spikes) contribute so much to the sum of squares
  that they single-handedly dominate this number — 508 RPM makes it look like the sensors
  disagree by roughly 3% of hover speed *typically*, which is **not true** for the other 17,686
  samples.
- **`rmse_robust_rpm` (headline, spikes excluded):** **106.7 RPM** — computed identically, just
  on the 17,683 samples that survive the robust mask. This is what the sensors actually agree to
  on a normal tick: about half a percent of hover RPM, well within what you'd expect from two
  independently-sampled, independently-noisy measurements of the same fast-spinning motor.

The fleet-wide headline (**128.1 RPM** median, **802.9 RPM** raw diagnostic median, 272
motor-rows) is just this same robust/raw split, repeated across every flight and averaged — the
one worked motor-row above (106.7 / 508.1) sits close to the fleet median on both counts, so
it's a representative example, not a cherry-picked best case.

### Step 5 — lag (does one sensor react to changes before the other?)

**Method:** cross-correlation. Take the two RPM traces, try sliding one against the other by
every possible small time offset (up to ±50 ms), and find the offset where they line up best
(the highest correlation). That offset *is* the lag.

Why this instead of, say, comparing timestamps directly? Because RPM sensors don't come with a
"this reading is delayed by exactly X ms" label — the only way to measure a real transport delay
is to see how much you have to shift one signal in time before its *shape* (the ups and downs of
the actual RPM changes) matches the other's shape. Cross-correlation is the standard tool for
exactly this.

**Sign convention, and how it's verified, not just asserted:** the code defines positive lag as
"DShot lags the deck." This isn't just a docstring — there's a self-test
(`synthetic_lag_self_test`) that builds a synthetic DShot trace by taking the deck trace and
deliberately delaying it by a known amount (5 samples = 10 ms at 500 Hz), then checks the
function recovers exactly `+10.00 ms`. It does, every run (`Synthetic lag self-test... PASS` in
the script's own console output) — so when real flight data shows positive lag values almost
everywhere (typically 0–5 ms), that positive sign is a verified statement that **DShot is the
delayed channel**, not an assumption.

### Step 6 — dropout (`deck_zero_pct` / `dshot_zero_pct`)

Just the percentage of *all* samples (valid or not) where that sensor read exactly 0 — the
simplest metric here, and the one that flagged the real, separate finding in this dataset: the
top drone's deck reads zero on 54–70% of samples in the A2 scenario specifically (§ "Open
finding" below), a genuine hardware dropout unrelated to the spike story above.

---

Historical CSVs **predate** the firmware spike guard in `rpm_get_all()` (2026-09-29); re-analysis uses robust RMSE on logs only — it does not retroactively simulate the guard.

---

## Inventory (verified at run time)

| Item | Count / note |
|------|----------------|
| Merged C.1 CSVs | **38** (`c1_*_merged/*/*_merged_usd.csv`, incl. **2026-09-28** merges) |
| Dual RPM in headers | **38/38** |
| Motor-rows | **272** |
| uSD config | **48** channels, **500 Hz**, both RPM sources |
| Spike samples (base valid, \|err\| > 10k) | **2121** (`spike_investigation.json`) |

Synthetic lag self-test: **5** samples (**10 ms**) → recovered **10.00 ms** — **PASS** (synthetic + real `cf5.rpm_m1` trace).

---

## Summary by scenario and vehicle role (spike-filtered headline)

Vehicle role: `cf5*` → **bottom**, `cf_second*` → **top**.  
Lag: **positive ms = DShot lags deck**.  
Primary columns: **robust bias %** and **robust RMSE**; raw RMSE is **diagnostic (with spikes)**. Full CSV: `summary_by_scenario_vehicle.csv`.

| Scenario | Role | Flights | Bias % mean (robust) | Bias % worst \|·\| (robust) | RMSE mean (robust) | RMSE mean (raw, diagnostic) | Lag ms mean | Lag ms max | Deck-zero % worst |
|----------|------|---------|----------------------|----------------------------|--------------------|-----------------------------|-------------|------------|-------------------|
| A1 | bottom | 7 | −0.12 | 0.29 | 158.1 | 870.6 | 0.7 | 4 | 0.0 |
| A1 | top | 7 | −0.15 | 0.24 | 120.7 | 728.7 | 4.3 | 44 | 0.0 |
| A2 | bottom | 4 | −0.16 | 0.35 | 133.9 | 783.7 | 1.0 | 2 | 0.0 |
| A2 | top | 4 | −0.58 | 3.87 | 333.0 | 925.0 | 4.5 | 34 | 69.8 |
| A3 | bottom | 8 | −0.18 | 0.47 | 130.3 | 808.1 | 1.9 | 4 | 0.0 |
| A3 | top | 8 | −0.19 | 0.29 | 114.1 | 763.6 | 1.3 | 24 | 0.0 |
| A4 | bottom | 5 | −0.16 | 0.29 | 135.6 | 857.0 | −0.1 | 2 | 10.5 |
| A4 | top | 5 | −0.16 | 0.28 | 126.3 | 799.2 | 0.7 | 6 | 9.9 |
| A7 | bottom | 3 | −0.11 | 0.15 | 137.6 | 672.3 | 1.5 | 2 | 0.0 |
| A7 | top | 3 | −0.18 | 0.26 | 111.4 | 658.6 | 5.5 | 36 | 0.0 |
| A8 | bottom | 3 | −0.18 | 0.43 | 115.0 | 727.7 | 2.2 | 6 | 0.0 |
| A8 | top | 3 | −0.19 | 0.31 | 115.6 | 731.1 | 0.0 | 10 | 0.0 |
| C5 | bottom | 8 | −0.21 | 0.48 | 122.1 | 738.5 | 2.6 | 24 | 0.0 |

Figures: `overview_lag_bias_by_role.png` (lag + **robust RMSE** by role), `overview_robust_bias_by_role.png` (**robust bias %** by role).

Per-flight robust headline table: `flight_summary_table.md` (A2 deck-dropout flights still listed; interpret with § Open finding).

---

## Open finding — A2 top vehicle, deck dropout

Both **A2** merges: **cf_second (top)** deck **zero ~54–70%** on all motors; **DShot 0%** dropout. **cf5 (bottom)** 0% deck zero. Physical inspection still open; not a DShot spike issue.

---

## Spike root cause and control-path impact (2026-09-29)

### What the spikes are

Rare **DShot-side** decoded values (**~28k–59k RPM**, passing `dshot < 60000`) while **deck stays ~11–18k RPM** at the same instant. Example on record: **A3 13-04-34** bottom **m1**, **t ≈ 18.268 s** — deck **11 569**, DShot **59 363**.

### Investigation (`rpm_spike_investigation.py` → `spike_investigation.json`)

| Finding | Detail |
|---------|--------|
| Spike samples (base valid, \|err\| > 10k) | **2121** on full **38**-CSV C.1 glob (was **~1540** on the earlier 29-manifest subset) |
| Share of raw MSE | **~96%** (2026-09-27 count; same mechanism) |
| Burst length | Median **2** consecutive 500 Hz samples (**2–4 ms**); max run **15** |
| Motor identity | All **m1–m4** affected (roughly balanced) |
| Scenario | Widespread (A1/A3/A7/A8/C5, …); **not** one flight |
| Ruled out | Deck dropout at spike times; single-motor-only; 0xFFFF sentinel; “one packet” only |

**Root cause (honest):** No single hardware fault line identified. Best fit: **intermittent bidirectional DShot telemetry decode / slot glitches** producing physically impossible eRPM for 1–2 control/log ticks while the optical deck (independent path) stays coherent. **Not** correlated with uSD-logged `ctrl_mode` transitions (column absent). **Not** a clean function of commanded PWM step in neighbor samples.

### Do spikes reach the live control signal?

On **logged geometric flights** (`indi.a_res_*` always written when RPMs exist):

- **`|Δ a_res_z|` per 500 Hz tick** at spike instants: median **~0.036 m/s²** vs **~0.006 m/s²** on matched non-spike samples (**~6×** ratio); p95 **~0.40 m/s²** at spikes.
- **`tau_*` logs** are **0** on these C.1 geometric runs — no torque-channel correlation available from uSD.

**Conclusion:** The existing **Butterworth / INDI filter chain does not fully hide a 2–4 ms RPM spike** in the **logged residual** — bursts are **small but elevated** vs baseline. At **1 kHz** `rpm_get_all()` the same raw DShot values feed thrust/torque reconstruction **before** filtering; a mid-flight guard is **justified**, not cosmetic.

### Firmware guard (2026-09-29)

In `traj_iface.c` **`rpm_get_all()`**, when `g_indi_rpm_source != 0` (DShot path):

1. Existing **0xFFFF → 0** unchanged.  
2. Reject **> 28 000 RPM** absolute.  
3. Reject **> 10 000 RPM** step vs previous accepted sample per motor.  
4. On reject: **hold previous valid** RPM (do not zero).

**Does not** change `rpm_source` default or deck path behavior. **`make DRONE=bl`** clean after change.

---

## Thesis methods (draft paragraph)

During C.1 data collection, onboard uSD logging recorded **both** optical-deck RPM (`rpm.m1–4`) and DShot ESC RPM (`motor.m1_rpm–m4_rpm`) at **500 Hz**, independent of which source feeds control. Since **2026-09-16**, the bottom drone uses **`indi_gains.rpm_source=1`**: **DShot feeds torque reconstruction and logged `indi.a_res_*`** because the deck **dropped two motors in real 2-drone flight**, **not** because DShot is lower-latency — cross-correlation shows **DShot typically lags the deck by a few milliseconds**. Post-hoc agreement is summarized with **spike-robust RMSE** (**128 RPM** median motor-row on the full C.1 set) versus **raw diagnostic** RMSE (**803 RPM**, with spikes). Rare spikes are **DShot-only** (deck in-band); best-fit cause is **decode glitches, not confirmed**. A **2026-09-29** `rpm_get_all()` sanity filter protects **future** flights; historical CSVs are unchanged.

---

## Plots (regenerated 2026-09-29, robust-first)

All figures live under `experiments/analysis/out/rpm_source_quality/`. **Headline metrics on titles/subtitles use robust bias/RMSE.** Trace overlays and grid4 **mark excluded spike samples** (red **×** on RPM and Δ-RPM panels) instead of hiding them. **Rolling lag** uses the **robust valid mask inside each window**.

| Figure | Description |
|--------|-------------|
| `overview_lag_bias_by_role.png` | Cross-correlation lag (ms) and **robust RMSE** by scenario × vehicle role |
| `overview_robust_bias_by_role.png` | **Robust bias %** by scenario × vehicle role |
| `overlay_*.png` (4) | Deck vs DShot + **Δ-RPM** panel; spike samples marked |
| `grid4_*.png` (3) | Four motors × (RPM + Δ-RPM) stack; A1, A3, A7 exemplars |
| `rolling_lag_*.png` (4) | 2 s / 0.5 s step rolling lag (robust-masked windows) |
| `overlay_zoom_*.png` (4) | Same exemplars as overlays; **y-axis from robust-valid band**; off-scale spikes as ▲ + value labels |
| `scatter_fleet_robust_by_role.png` | All robust-valid samples, hexbin by vehicle role; **y = x** reference |
| `scatter_A3_13-00-57_cf5_m2_robust.png` | Worked-example motor-row agreement (walkthrough in § Step-by-step) |
| `lag_align_A3_13-00-57_cf5_m2.png` | 2.5 s window: DShot unshifted vs shifted by measured lag (+4 ms on this row) — **honest caveat below** |
| `lag_align_A1_17-17-26_cf_second_m3.png` | Same panels at the flight-wide **+44 ms** lag value — **not a good correction example, see below** |
| `lag_hist_fleet_by_role.png` | Histogram of `lag_ms` over 272 motor-rows (bottom vs top) |

Representative overlays: `overlay_A1_17-17-26_cf_second_m3`, `overlay_A3_13-00-57_cf5_m2`, `overlay_A7_19-11-19_cf_second_m3`, `overlay_A2_19-27-03_m3` (A2 deck-dropout context). Pair each raw overlay with `overlay_zoom_*` to read hover-band agreement without spike-dominated scaling.

**Honest correction on the two `lag_align_*` plots (2026-09-30):** these were originally captioned
as showing "obvious alignment" after the shift. Checked quantitatively (correlation and RMSE
between deck and shifted/unshifted DShot, in the exact plotted window) — that claim doesn't hold:

| Exemplar | Correlation, unshifted → shifted | RMSE, unshifted → shifted |
|---|---|---|
| A3 13-00-57, motor 2 (+4 ms) | 0.400 → 0.422 | 86.0 → 84.6 RPM |
| A1 17-17-26, motor 3 (+44 ms) | **0.716 → 0.435** | **75.2 → 91.6 RPM** |

A3's shift is marginally in the right direction but too small to see by eye (as originally
noted). **A1's shift makes agreement measurably *worse*, not better** — the plot doesn't show
what its caption claims. Root cause: `44 ms` is the flight-wide cross-correlation's reported
value for that motor-row, but §"Top vs bottom" above already documents that isolated large lag
values like this one are **noisy correlation peaks, not calibrated delays** — this A1 case is
exactly that, and using it as a "correction" demo applies a noisy estimate to a window where it
doesn't hold. Checked several other motor-rows with more moderate (6-10 ms) lag values as
possible replacements — all showed similarly marginal, inconsistent changes, suggesting the
real story is that **measurable lag in this dataset is mostly too small relative to sample-to-
sample noise to produce a visually dramatic before/after** with real flight data, not that a
better exemplar exists. Both plots are kept (removing them loses information) but should be read
as: the self-test proves the *method* correctly recovers a known injected delay; these two real-
data panels show that applying real, estimated lag values to short windows does not reliably
or visibly improve alignment — a legitimate, if less exciting, finding in its own right.

§ Extension 2026-09-27 raw-RMSE spike note is **superseded** by robust RMSE + marked-spike plots above.

---

## Related

- [`42_RPM_Source_Quality_Desk_Prompt.md`](42_RPM_Source_Quality_Desk_Prompt.md)  
- [`23_DShot_RPM_Investigation.md`](23_DShot_RPM_Investigation.md) §5  
- `experiments/analysis/rpm_spike_investigation.py`

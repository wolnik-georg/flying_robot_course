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

Representative overlays: `overlay_A1_17-17-26_cf_second_m3`, `overlay_A3_13-00-57_cf5_m2`, `overlay_A7_19-11-19_cf_second_m3`, `overlay_A2_19-27-03_m3` (A2 deck-dropout context).

§ Extension 2026-09-27 raw-RMSE spike note is **superseded** by robust RMSE + marked-spike plots above.

---

## Related

- [`42_RPM_Source_Quality_Desk_Prompt.md`](42_RPM_Source_Quality_Desk_Prompt.md)  
- [`23_DShot_RPM_Investigation.md`](23_DShot_RPM_Investigation.md) §5  
- `experiments/analysis/rpm_spike_investigation.py`

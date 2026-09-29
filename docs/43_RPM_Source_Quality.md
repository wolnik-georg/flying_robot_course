# 43 — RPM source quality (deck vs DShot on C.1 merged logs)

**Date:** 2026-09-23 (desk); **reopened 2026-09-29** (spike filter + methods + control-path check)  
**Framing:** **Sensor-quality** study — agreement, lag, and dropout between the optical RPM deck and DShot bidirectional telemetry on the **C.1 merged dataset**. Not a closed-loop stability claim; **does not** change standing **`indi_gains.rpm_source=1`**.

**Reproduce:** `python3 experiments/analysis/rpm_source_quality.py` (add `--no-plots` if matplotlib/numpy clash)  
**Outputs:** `experiments/analysis/out/rpm_source_quality/`  
**Web page:** [`43_RPM_Source_Quality.html`](43_RPM_Source_Quality.html)

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

**Fleet headline (29 manifest merges, 2026-09-29 re-run):**

| Statistic | rmse_robust_rpm (headline) | rmse_raw_rpm (diagnostic) |
|-----------|---------------------------:|--------------------------:|
| Median across motor-rows | **~126 RPM** | **~771 RPM** |
| Mean across motor-rows | **~143 RPM** | **~766 RPM** |

Source: `experiments/analysis/out/rpm_source_quality/fleet_robust_rmse.json`, `per_flight.csv`.

Historical CSVs **predate** the firmware spike guard in `rpm_get_all()` (2026-09-29); re-analysis uses robust RMSE on logs only — it does not retroactively simulate the guard.

---

## Inventory (verified at run time)

| Item | Count / note |
|------|----------------|
| Merged C.1 CSVs | **29** (manifest-filtered) |
| Dual RPM in headers | **29/29** |
| Motor-rows | **200** (29 flights × vehicles × 4 motors, minus solo C5 top) |
| uSD config | **48** channels, **500 Hz**, both RPM sources |

Synthetic lag self-test: **5** samples (**10 ms**) → recovered **10.00 ms** — **PASS** (synthetic + real `cf5.rpm_m1` trace).

---

## Summary by scenario and vehicle role

Vehicle role: `cf5*` → **bottom**, `cf_second*` → **top**.  
Lag: **positive ms = DShot lags deck**.  
Bias / lag table unchanged in structure — see `summary_by_scenario_vehicle.csv` (bias still sub-percent except A2 top deck dropout).

Figure: `overview_lag_bias_by_role.png` (optional `--no-plots` skip).

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
| Spike samples (base valid, \|err\| > 10k) | **~1 540** on original 29-flight set; **~2 121** if extra merges present in tree |
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

During C.1 data collection, onboard uSD logging recorded **both** optical-deck RPM (`rpm.m1–4`) and DShot ESC RPM (`motor.m1_rpm–m4_rpm`) at **500 Hz**, independent of which source feeds control. Since **2026-09-16**, the bottom drone uses **`indi_gains.rpm_source=1`**: **DShot feeds torque reconstruction and logged `indi.a_res_*`** because the deck **dropped two motors in real 2-drone flight**, **not** because DShot is lower-latency — cross-correlation shows **DShot typically lags the deck by a few milliseconds**. Post-hoc agreement is summarized with **spike-robust RMSE** (~**126 RPM** median motor-row) versus raw RMSE (~**771 RPM**), reflecting rare DShot telemetry glitches. A **2026-09-29** `rpm_get_all()` sanity filter rejects implausible DShot spikes on the live path; historical logs predate that filter.

---

## Extensions (plots and tables)

Earlier desk extensions (2026-09-26 overlays, rolling lag, grid4, flight summary table) remain in `experiments/analysis/out/rpm_source_quality/`. Read **lag** panels with the **positive = DShot lags deck** convention and A2 dropout caveat.

§ Extension 2026-09-27 (4) raw RMSE spike note is **superseded** by robust RMSE + firmware guard above.

---

## Related

- [`42_RPM_Source_Quality_Desk_Prompt.md`](42_RPM_Source_Quality_Desk_Prompt.md)  
- [`23_DShot_RPM_Investigation.md`](23_DShot_RPM_Investigation.md) §5  
- `experiments/analysis/rpm_spike_investigation.py`

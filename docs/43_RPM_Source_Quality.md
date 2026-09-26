# 43 — RPM source quality (deck vs DShot on C.1 merged logs)

**Date:** 2026-09-23 (desk)  
**Framing:** This is a **sensor-quality** study only — agreement, lag, and dropout between the optical RPM deck and DShot bidirectional telemetry on the **actual C.1 dataset**. It does **not** reopen July’s DShot/INDI root-cause narrative, does **not** claim closed-loop stability, and does **not** propose changing the standing **`indi_gains.rpm_source=1`** policy.

**Reproduce:** `experiments/analysis/out/rpm_source_quality/README.md`  
**Tool:** `experiments/analysis/rpm_source_quality.py` (imports validated `cross_corr_lag` from `flying_drone_stack/tools/investigate_dshot_rpm.py`; does not modify that script).  
**Web page:** [`43_RPM_Source_Quality.html`](43_RPM_Source_Quality.html) — same content, browsable format.

---

## Inventory (verified at run time)

| Item | Count / note |
|------|----------------|
| Merged C.1 CSVs | **29** (`8` × `c1_2026-09-21_merged/`, `21` × `c1_2026-09-23_merged/`) |
| Manifests | `manifest_2026-09-21_c1.json` (8 paths, no `merge_status` field), `manifest_2026-09-23_c1.json` (**25** entries: **21** `merged`, **2** `no_usd`, **2** `merge_failed`) |
| Dual RPM in headers | **29/29** (line 1 `# meta:`; column names on line 2) |
| Single-vehicle merges | **8** (all **C5**, cf5-only — excluded from cross-vehicle comparison) |
| Raw `.bin` under `usd_raw/` | **91** (not used as analysis input; filenames can invert card vs role — see `*_PAIRING.md`) |
| uSD config | `flying_drone_stack/tools/usd_thesis_config.txt` — **48** channels, **500 Hz**, both RPM sources |

Analysis uses the **full merged time series** per motor with validity mask `(deck>0) & (dshot>0) & (dshot<60000)` (guards DShot `0xFFFF` invalid sentinel), same as `investigate_dshot_rpm.py`.

**Synthetic lag self-test:** injected **5** samples (**10.0 ms**) → recovered **10.00 ms** — **PASS**.

---

## Summary by scenario and vehicle role

Vehicle role from merge column prefix: `cf5*` → **bottom**, `cf_second*` → **top**.  
Lag: **positive ms = DShot lags deck**. Values often quantize to **2 ms** (500 Hz sample period).  
**Worst** columns are max over motor-rows in that bucket (not a single global mean).

| Scenario | Role | n flights | n motor-rows | bias % mean | bias % worst \|·\| | lag ms mean | lag ms max | deck zero % worst | DShot zero % worst |
|----------|------|-----------|--------------|-------------|-------------------|-------------|------------|-------------------|---------------------|
| A1 | bottom | 6 | 24 | +0.019 | 0.28 | 0.67 | 4 | 0.0 | 0.0 |
| A1 | top | 6 | 24 | −0.017 | 0.30 | 4.67 | **44** | 0.0 | 0.0 |
| A2 | bottom | 2 | 8 | −0.063 | 0.22 | 1.0 | 2 | 0.0 | 0.0 |
| A2 | top | 2 | 8 | −0.899 | **3.72** | 0.0 | 0 | **69.8** | 0.0 |
| A3 | bottom | 7 | 28 | −0.065 | 0.34 | 1.79 | 4 | 0.0 | 0.0 |
| A3 | top | 7 | 28 | −0.073 | 0.22 | 2.71 | 24 | 0.0 | 0.0 |
| A7 | bottom | 3 | 12 | −0.003 | 0.16 | 1.5 | 2 | 0.0 | 0.0 |
| A7 | top | 3 | 12 | −0.070 | 0.21 | 5.5 | 36 | 0.0 | 0.0 |
| A8 | bottom | 3 | 12 | −0.053 | 0.34 | 2.17 | 6 | 0.0 | 0.0 |
| A8 | top | 3 | 12 | −0.078 | 0.27 | 0.0 | 10 | 0.0 | 0.0 |
| C5 | bottom | 8 | 32 | −0.091 | 0.42 | 2.62 | 24 | 0.0 | 0.0 |

Source CSV: `experiments/analysis/out/rpm_source_quality/summary_by_scenario_vehicle.csv`  
Per-flight detail: `per_flight.csv` (**200** motor-rows = 29 flights × dual vehicles where present × 4 motors, minus single-vehicle C5 top).

Figure: `experiments/analysis/out/rpm_source_quality/overview_lag_bias_by_role.png`.

---

## Open finding — A2 top vehicle, deck dropout (all four motors)

Both **A2** merges (`19-21-05`, `19-27-03`; legacy circle profile, top align RMS flagged in manifest) show **cf5 (bottom): 0% deck zero** on all motors. **cf_second (top): deck reads zero ~54–70% of samples on every motor** in both flights; **DShot zero % = 0%** on the same rows.

| Flight | Role | Motor | deck zero % | DShot zero % | bias % (valid samples only) |
|--------|------|-------|-------------|--------------|----------------------------|
| 19-21-05 | top | m1–m4 | 63.1, 67.9, 68.1, 67.6 | 0.0 each | −1.20, −0.26, −0.61, −0.21 |
| 19-27-03 | top | m1–m4 | 69.6, 68.9, **53.8**, 69.8 | 0.0 each | −0.19, −0.38, **−3.72**, −0.62 |

**Scope:** Not m1-specific — **airframe-wide on the top drone during A2 only** among this dataset. No root-cause claim here (markers, geometry, deck driver, etc.). Lag estimates on top/A2 are **not meaningful** when most deck samples are zero (reported as 0 ms where cross-correlation has insufficient valid pairs).

---

## Top vs bottom — lag and noise

On **dual-vehicle** scenarios, **top (`cf_second`)** shows **larger and more variable** cross-correlation lags than **bottom (`cf5`)** when deck data are present (e.g. A1 top lag mean **4.7 ms** vs bottom **0.7 ms**; A7 top max **36 ms** vs bottom **2 ms**). Many values sit on **±2 ms** grid from 500 Hz sampling; occasional larger peaks (44 ms on one A1 top motor-row) should be read as **noisy extrema**, not calibrated actuator delay. **Bias magnitude** stays **sub-percent** except where deck dropout dominates (A2 top).

---

## Thesis methods (draft paragraph)

During C.1 data collection, onboard uSD logging recorded **both** optical-deck RPM (`rpm.m1–4`) and DShot bidirectional ESC RPM (`motor.m1_rpm–m4_rpm`) at **500 Hz** on every flight (`usd_thesis_config.txt`), independent of the control path. Since **2026-09-16**, the bottom geometric controller drone uses **`indi_gains.rpm_source=1`**, so **DShot telemetry feeds INDI torque reconstruction and the logged residual `indi.a_res_*`**; the deck remained on the airframe as a **parallel logged channel** for post-hoc agreement checks. This document quantifies deck–DShot bias, lag, and dropout on **29** merged two-drone (or single-drone C5) sessions; it does not assert that either source would behave identically inside a closed-loop INDI experiment.

---

## Extension — 2026-09-26: raw overlays + rolling lag

**Reproduce:** re-run `experiments/analysis/rpm_source_quality.py` (same entry point as above; writes additive PNGs only).

### Metric definitions (plots)

- **Deck RPM (blue):** optical-deck-derived rotor speed (`<role>.rpm_m1`–`m4`), logged at **500 Hz** on the merged uSD CSV.
- **DShot RPM (orange):** ESC bidirectional-telemetry rotor speed (`<role>.motor_m1_rpm`–`m4_rpm`), **same drone, same `t` column**. The overlay is both raw traces on one clock — gaps, dropouts, and tracking differences are visible without a summary statistic.
- **Deck ≤ 0 shading / markers:** where the deck reads zero, the plot shades that time range (and marks samples at **y = 0**) so dropout shape (sustained vs intermittent) is visible.

- **Rolling lag:** the existing **`lag_ms`** is **one** cross-correlation over the **entire** flight (valid samples only, ±50 ms search, same as `cross_corr_lag()`). **Rolling lag** repeats that same function on **2.0 s** windows stepped every **0.5 s** (`rolling_cross_corr_lag()` in `rpm_source_quality.py`); the x-axis is **window center time**. A flat rolling curve means timing is stable; a drifting x-axis trend would mean relative delay changes during the flight.

### New figures

| File | What it shows |
|------|----------------|
| [`overlay_A2_19-27-03_m3.png`](../experiments/analysis/out/rpm_source_quality/overlay_A2_19-27-03_m3.png) | **A2 top, m3** — intermittent deck zeros (~54% flight) while DShot stays non-zero; dropout pattern visible. |
| [`overlay_A3_13-00-57_cf5_m2.png`](../experiments/analysis/out/rpm_source_quality/overlay_A3_13-00-57_cf5_m2.png) | **Clean baseline** — A3 bottom **m2**, ~0% deck zero, deck/DShot track together. |
| [`overlay_A1_17-17-26_cf_second_m3.png`](../experiments/analysis/out/rpm_source_quality/overlay_A1_17-17-26_cf_second_m3.png) | A1 top **m3** (flight-wide **lag_ms = 44** in table). |
| [`overlay_A7_19-11-19_cf_second_m3.png`](../experiments/analysis/out/rpm_source_quality/overlay_A7_19-11-19_cf_second_m3.png) | A7 top **m3** (flight-wide **lag_ms = 36**). |
| `rolling_lag_*` PNGs (same stems) | Rolling lag vs time for the same four cases; dashed line = flight-wide **`lag_ms`**. |

### Rolling lag vs single-number `lag_ms`

On **A3 13-00-57 cf5 m2** (clean), rolling lag stays near **0–4 ms** (median **~2 ms**), consistent with the flight-wide **4 ms** — **no meaningful drift**.

On **A1 top m3** and **A7 top m3**, rolling lag spends most windows near **±2 ms** (500 Hz quantization) with **occasional ±24–50 ms spikes**; the flight-wide **44 ms / 36 ms** values are **not** a sustained offset visible across the whole timeline — they are **extrema from one global correlation**, not a phase where DShot systematically lags by tens of ms. **Do not reinterpret those peaks as actuator delay without further checks.**

On **A2 top m3**, many windows lack enough valid deck samples; finite rolling values jump between correlation-window limits — **lag plots are not interpretable** where deck dropout dominates (same caveat as § open finding).

---

## Extension — 2026-09-26 (2): 4-motor grids + flight summary table

**Reproduce:** `rpm_source_quality.py` also writes `grid4_*.png` and `flight_summary_table.md`.

### 4-motor overlay grids (top-3 |lag|, A2 excluded)

Full-flight **2×2** deck vs DShot on the **top** vehicle (`cf_second` / suffixed prefix), subplot titles show that motor’s **`bias_pct`** and **`lag_ms`** from `per_flight.csv`.

| Plot | Flight (max |lag| motor) |
|------|-------------------------|
| [`grid4_A1_17-17-26.png`](../experiments/analysis/out/rpm_source_quality/grid4_A1_17-17-26.png) | A1 top **m3**, **44 ms** |
| [`grid4_A7_19-11-19.png`](../experiments/analysis/out/rpm_source_quality/grid4_A7_19-11-19.png) | A7 top **m3**, **36 ms** |
| [`grid4_A3_13-04-34.png`](../experiments/analysis/out/rpm_source_quality/grid4_A3_13-04-34.png) | A3 top **m4**, **24 ms** |

Figure legend (once per plot): **deck RPM** = optical deck; **DShot RPM** = ESC telemetry; same **500 Hz** clock as § above.

### Successful flights — deck vs DShot (A2 excluded)

One row per merged flight; metrics are **means/maxes over all motor rows** in that CSV (4 on solo C5, 8 on dual-vehicle merges). Source: `per_flight.csv` only (no recomputation).

**Headline (27 flights, A2 omitted):** median **|bias| ≈ 0.115%**, median **lag ≈ 2.0 ms** — sub-percent bias and ~one sample period lag for typical sessions; larger **max |lag|** peaks (24–44 ms) appear on individual motor-rows, not as fleet-wide medians.

| Scenario | Flight | Mean |bias| % | Mean lag (ms) | Max |lag| (ms) | Deck dropout % | DShot dropout % |
|----------|--------|---------------|---------------|----------------|-----------------|-----------------|
| A1 | 12-51-16 | 0.064 | 2.2 | 6.0 | 0.0 | 0.0 |
| A1 | 13-25-10 | 0.123 | -1.8 | 10.0 | 0.0 | 0.0 |
| A1 | 17-17-26 | 0.094 | 8.5 | 44.0 | 0.0 | 0.0 |
| A1 | 17-18-48 | 0.095 | 3.5 | 24.0 | 0.0 | 0.0 |
| A1 | 17-36-06 | 0.083 | 2.8 | 8.0 | 0.0 | 0.0 |
| A1 | 17-37-26 | 0.078 | 0.8 | 4.0 | 0.0 | 0.0 |
| A3 | 13-00-57 | 0.131 | 2.0 | 4.0 | 0.0 | 0.0 |
| A3 | 13-02-56 | 0.115 | 2.0 | 4.0 | 0.0 | 0.0 |
| A3 | 13-04-34 | 0.085 | 4.0 | 24.0 | 0.0 | 0.0 |
| A3 | 17-45-03 | 0.081 | 1.2 | 6.0 | 0.0 | 0.0 |
| A3 | 17-46-43 | 0.078 | 4.2 | 16.0 | 0.0 | 0.0 |
| A3 | 17-54-32 | 0.108 | 1.8 | 4.0 | 0.0 | 0.0 |
| A3 | 17-57-32 | 0.090 | 0.5 | 4.0 | 0.0 | 0.0 |
| A7 | 19-11-19 | 0.099 | 7.0 | 36.0 | 0.0 | 0.0 |
| A7 | 19-12-38 | 0.073 | 0.5 | 4.0 | 0.0 | 0.0 |
| A7 | 19-13-55 | 0.094 | 3.0 | 16.0 | 0.0 | 0.0 |
| A8 | 12-11-05 | 0.133 | 1.8 | 10.0 | 0.0 | 0.0 |
| A8 | 12-11-46 | 0.165 | 1.0 | 2.0 | 0.0 | 0.0 |
| A8 | 13-14-36 | 0.128 | 0.5 | 8.0 | 0.0 | 0.0 |
| C5 | 18-02-55 | 0.164 | 1.5 | 6.0 | 0.0 | 0.0 |
| C5 | 18-03-53 | 0.172 | 4.0 | 8.0 | 0.0 | 0.0 |
| C5 | 18-06-27 | 0.229 | 3.5 | 10.0 | 0.0 | 0.0 |
| C5 | 18-08-14 | 0.196 | 0.0 | 8.0 | 0.0 | 0.0 |
| C5 | 18-16-49 | 0.149 | 9.5 | 24.0 | 0.0 | 0.0 |
| C5 | 18-17-45 | 0.210 | 0.0 | 4.0 | 0.0 | 0.0 |
| C5 | 18-18-41 | 0.194 | 0.5 | 4.0 | 0.0 | 0.0 |
| C5 | 18-21-19 | 0.144 | 2.0 | 4.0 | 0.0 | 0.0 |
| **All flights (mean)** | — | **0.125** | **2.5** | **11.2** | — | — |
| **All flights (median)** | — | **0.115** | **2.0** | **8.0** | — | — |

---

## Related

- Planning prompt: [`42_RPM_Source_Quality_Desk_Prompt.md`](42_RPM_Source_Quality_Desk_Prompt.md)
- Historical DShot tooling + one-flight spot check: [`23_DShot_RPM_Investigation.md`](23_DShot_RPM_Investigation.md) §5 (unchanged scope)

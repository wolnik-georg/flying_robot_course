# 43 — RPM source quality (deck vs DShot on C.1 merged logs)

**Date:** 2026-09-23 (desk)  
**Framing:** This is a **sensor-quality** study only — agreement, lag, and dropout between the optical RPM deck and DShot bidirectional telemetry on the **actual C.1 dataset**. It does **not** reopen July’s DShot/INDI root-cause narrative, does **not** claim closed-loop stability, and does **not** propose changing the standing **`indi_gains.rpm_source=1`** policy.

**Reproduce:** `experiments/analysis/out/rpm_source_quality/README.md`  
**Tool:** `experiments/analysis/rpm_source_quality.py` (imports validated `cross_corr_lag` from `flying_drone_stack/tools/investigate_dshot_rpm.py`; does not modify that script).

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

## Related

- Planning prompt: [`42_RPM_Source_Quality_Desk_Prompt.md`](42_RPM_Source_Quality_Desk_Prompt.md)
- Historical DShot tooling + one-flight spot check: [`23_DShot_RPM_Investigation.md`](23_DShot_RPM_Investigation.md) §5 (unchanged scope)

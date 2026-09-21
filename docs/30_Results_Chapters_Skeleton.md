# 30 — Results chapters skeleton (Ch. 6–7)

**Purpose.** Outline for the experimental results once **C.4** data exists. Fill numbers from
[`24`](24_Downwash_Compensation_Comparison.md) and [`experiments/analysis/out/`](experiments/analysis/out/).
Thesis `.tex` lives in the local tree (`docs/thesis/`); this file is the repo-visible checklist.

---

## Chapter 6 — Two-robot results

### 6.1 Experimental campaign overview

- Formation pair: cf5 + cf_second; brushless; 4-point mocap.
- Scenarios flown for comparison (IDs from [`10`](10_Formation_Library.md) / C.1 matrix in [`25`](25_C1_Data_Collection_Plan.md)).
- Controllers compared: S0–S4 minimum set ([`01`](01_Thesis_Project_Snapshot.md)).
- Repeats: *n* per (scenario, controller); uSD source of record.

### 6.2 Tracking and attitude (reactive baseline)

- **Table:** pos RMSE (x, y, z, ‖e‖), peak error — **mean ± std**, *n*.
- **Figure:** grouped bars, controllers × metrics, SEM error bars (`plot_comparison.py`).
- **Figure:** time-series overlay — same scenario, different controllers (pick one representative scenario, e.g. A8 or A4).
- Subsection: geometric vs our INDI vs stock / reference INDI where flown.

### 6.3 Control effort and safety

- Thrust/torque proxies, motor saturation fraction (`effort_proxy`, `motor_*` from phase metrics).
- Minimum separation / closest approach (`min_sep_m`, `min_dz_m`).
- Short discussion: better tracking at higher effort is not automatically “better”.

### 6.4 Predictive strategy (Neural-Swarm2)

- Training data: C.1 scenarios, geometric collection.
- **Figure:** predicted vs measured `a_res` (scatter + R²); error vs relative geometry.
- In-flight enable: Strategy 2 vs S0 on matched scenarios.

### 6.5 Hybrid (NA-INDI), if in scope

- Configuration (c=8, weights source: retrain vs reference).
- Same metric table as 6.2–6.3 for fair comparison.

### 6.6 Summary table — “when which strategy”

- Rows: strategies; columns: scenario classes (vertical stack, lateral crossing, near-ground, …).
- Qualitative + quantitative: best tracking, effort, data/compute cost.

---

## Chapter 7 — Scaling / three robots (if time)

- Placeholder: B1/B2/B3 or subset; same metric pipeline.
- n≥4 extension sketch (not primary C.4): [`36`](36_Four_Five_Robot_Formation_Sketch.md).
- If not flown: one paragraph limitation + future work.

---

## Figures checklist (generate before writing prose)

| Figure | Tool / source |
|---|---|
| Controller × metric bars | `plot_comparison.py` on phase CSV |
| Per-phase breakdown | `plot_comparison.py --by-phase` |
| Single-flight dashboard | `plot_flight.py` |
| Downwash interaction | `plot_interaction.py` |
| Cross-flight comparison PNG | `experiments/analysis/out/a8_2026-09-19/figures/comparison.png` |

---

## Research questions mapping

| RQ | Section |
|---|---|
| RQ1 Reactive vs predictive vs hybrid | 6.2–6.5 + 6.6 |
| RQ2 Regime dependence | Scenario breakdown in 6.6; train/test by scenario class |
| RQ3 … | Per [`15`](15_Problem_Statement_and_Research_Questions.md) |
| RQ4 Cost of each strategy | 6.3 + data/compute notes in 6.4–6.5 |

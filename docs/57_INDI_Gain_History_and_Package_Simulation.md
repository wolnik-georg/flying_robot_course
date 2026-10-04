# INDI gain history & “three-change package” simulation

**Date:** 2026-10-04 · Investigation only. Builds on [`docs/56`](56_INDI_Oscillation_Overview_Tables.md), [`docs/54`](54_INDI_Loop_Rates_Followup.md) (validation notes), [`docs/55`](55_INDI_Motor_Model_and_Structure_Comparison.md).

---

## One-screen summary

| Item | Result | Confidence |
|---|---|---|
| **Package Python sim reproduces Oct-02 flight baselines?** | **NO** — ours/Omar both **stable** in toy model; **all grid results ILLUSTRATIVE ONLY** | HIGH (sim output) |
| **Attitude-only 1-D limit cycle at flown kr/kw?** | **YES** in existing **`indi_loop_rate_harness.rs`** (σ≈4.3 rad/s, **~7.9 Hz**); **not** a full-stack or A1 predictor | MEDIUM (SIMULATION, separate harness) |
| **Lowest kr in radio meta (c=6, mode≥1)** | **2400 only** (945 flights) | HIGH (log scan) |
| **Lowest kr documented flown** | **483** (2026-09-11 card, lab notes) — **not** in CSV meta | MEDIUM (lab doc) |
| **`res_sign=-1` ever flown?** | **NO** runtime; 2026-09-09 **hard-coded flip** diverged hover | HIGH (`lib.rs:1886`, lab_sessions) |
| **Softer KP &lt; 30 with INDI in meta** | **None** — meta always **pos_kp_xy=64** when logged | HIGH (log scan) |
| **Typical flown combo (when meta complete)** | **kr/kw 2400/170**, **pos 64/48/5/7**, **pos_ki_z=0** on Oct-02 A1 INDI | HIGH (meta) |
| **500 Hz decimation (corrected dt=2 ms)** | New **`indi_package_sim_harness.rs`**: limit cycle **σ&gt;0.1**; old harness decimation **artifact** if dt=1 ms on 2 ms updates | MEDIUM (SIMULATION) |

---

## A — Gain / parameter history (desk)

**Method:** Scan **1269** top-level `experiments/logs/*.csv` metas; metrics in steady **6–13 s** when data exist.  
**Outputs:** `experiments/analysis/out/indi_package/gain_history_flights.csv` (963 INDI-related rows), `gain_history_summary.json`.

### A1 Compact table (meta-logged INDI, controller=6, ctrl_mode≥1)

| Field | Value in radio meta | Notes |
|---|---|---|
| `indi_kr` / `indi_kw` | **2400 / 170** only | Reflects yaml `all:` block at log time, **not** per-flight retunes |
| `pos_kp_xy`, `pos_kp_z`, `pos_kv_*` | **64, 48, 5, 7** | Same |
| `pos_ki_z` | **0.0** on Oct-02 full INDI A1 (e.g. `A1_cf5_2026-10-02_19-15-47.csv:44`) | **FACT:** contradicts doc-56 row assuming 16 for those flights |
| `indi_fc_bw` | **206** (when present) | Matches yaml stage-2b |
| `res_sign`, `filt_dt_us`, `notch_en`, `filt_prewarp` | **NOT in meta** | Defaults: `+1`, `1000/1`, `0` per yaml / `traj_iface.c:441–477` |

### A2 Explicit answers

| # | Question | Answer | Evidence |
|---|---|---|---|
| 1 | Lowest **kr/kw with INDI** flown? | Meta: **2400/170 only**. Lab card: **483/76**, **987/109** on **2026-09-11** (`docs/FLIGHT_CARD_VALIDATION.md`, `docs/lab_sessions/2026-09-11.md`). Ladder mentions **600** (`crazyflies.yaml` comments). | HIGH meta; MEDIUM lab |
| 2 | **`res_sign=-1` flown?** | **No** as param. Code: **NEVER FLOWN** (`lib.rs:1886`). Sept-9 **committed flip** diverged (`docs/lab_sessions/2026-09-09.md`). H0 modes crashed **without** changing res_sign (`lib.rs:1910–1918`, `2026-09-12.md`). | HIGH |
| 3 | **KP &lt; 30 with INDI** in meta? | **No rows** | HIGH scan |
| 4 | **Combinations flown together?** | Locked block: **2400/170 + 64/48/5/7 + fc_bw 206**; retune sessions temporarily paired **lower kr** with **matched or geometric pos gains** (lab docs only) | MEDIUM |

### A3 Oct-02 A1 reference (controller variants)

| Variant | n (CSV) | gyro RMS °/s (20 Hz, 6–13 s) | Notes |
|---|---:|---:|---|
| Ours c=6 mode 3 | 2 | **~311** | `log_stats` in prior work / recomputed in package folder |
| Omar C c=9 | 2 | **~130** | τ log **zero** (telemetry gap) |
| Omar Rust c=10 | 2 | **~129** | τ non-zero |

---

## B — Small-signal analysis

**Script:** `experiments/analysis/indi_package_small_signal.py` → `small_signal_margins.json`.  
**Label:** CALCULATION, linear SISO, **not** a flight predictor.

### B1 Residual → tilt coupling (ASSUMPTION: `k_tilt`, τ_res=55 ms)

| pos (kp_z, kv_z) | res_sign | f_c [Hz] | PM [deg] (unwrap artifact possible) |
|---|---|---:|---:|
| 64, 5 | +1 | 0.14 | ~89 |
| 64, 5 | −1 | 0.14 | ~269 |
| 28, 5 | +1 | 0.06 | ~92 |
| 7, 4 | +1 | 0.02 | ~93 |

**JUDGEMENT (LOW):** Sign flip moves phase on this **toy** loop; does **not** prove hardware effect.

### B2 Attitude loop — unit conversion

**FACT:** Omar **KR [N·m/rad]** ↔ our **kr [1/s²]** via **kr_equiv = KR / J**.

| Variant | Gain | J·kr or KR [N·m/rad] | ω_n [Hz] | ζ |
|---|---|---:|---:|---:|
| Ours flown | kr=2400, kw=170 | **0.0575** | **7.80** | 1.74 |
| Ours mid | 1200 / 120 | 0.0287 | 5.51 | 1.73 |
| Omar-equiv | kr=**292** (0.007/J_ours) | 0.007 | 2.72 | — |
| Omar-equiv | kr=**420** (0.007/J_omar mapped) | — | 3.26 | 1.68 |
| Omar geo | KR=0.007, KW=0.00115 | 0.007 | **3.27** | 0.082* |

\*Underdamped in **linear geo-only** model; real Omar adds damping paths + 500 Hz hold.

**Investigation cross-check:** sim ceiling **kr≈800** (`investigation_indi_oscillation_2026-07-21.md` §16) vs flown **2400** (~3×).

---

## C — Closed-loop package simulation

### C0 Path chosen

| Path | Status |
|---|---|
| CS2 SIL + compiled `controllerOutOfTree` + 2-drone downwash | **Not executed** in this desk pass (no new SIL wrapper without editing existing scripts) |
| **New** `experiments/analysis/indi_package_sim.py` | 1-D roll + lagged **a_res** + optional pos coupling + **ours vs omar** law switch |
| **New** `flying_drone_stack/tests/indi_package_sim_harness.rs` | **500 Hz decimation with dt=2 ms** on derivatives (contrast `indi_loop_rate_harness.rs` **1 ms bug**) |

### C1 Baseline validation (required)

| Case | Target (flight/docs/56) | Python package sim | Pass? |
|---|---|---:|---|
| Ours flown | gyro **260–290** °/s, **4.7–5.7 Hz** | **~0.07–0.32** °/s, **&lt;1 Hz**, stable | **NO** |
| Omar C | **55–97** °/s, **~3.3 Hz** | ~0 °/s, stable | **NO** |

**FACT:** `harness_validated_for_baseline_separation: false` in `package_sim_results.json`.

**Separate check (attitude-only, existing harness):** `indi_loop_rate_harness.rs` **`harness_validates_limit_cycle_at_flown_gains`** — σ≈**4.27 rad/s**, **~7.88 Hz** (SIMULATION, 1-axis, no position/residual/downwash).

### C2 Grid (30 cells, **ILLUSTRATIVE ONLY**)

Run: `INDI_PACKAGE_SWEEP=1 python indi_package_sim.py`. Factors: **kr** {2400,1200,420,290}, **res_sign** {±1}, **pos** {64/5, 28/5, 7/4}, **decimate_500** (kr=2400 only).

**SIMULATION (LOW):** In this toy model **all cells stable**; **no meaningful ranking** vs flight. **Decimate_500** drives σ→0 (same **class** of artifact as old harness, but derivative uses **2 ms** in new Rust test).

**Plots:** `fig_baseline_bars.png`, `fig_grid_gyro_rms.png` (bars reflect failed baseline scale — interpret accordingly).

### C3 What is missing for validation

- Full **`lib.rs`** on host/SIL, mocap/EKF, **Rd** from `f_d`, multi-axis coupling, measured **a_res** delay, partner downwash model, torque clamps/saturation.

---

## D — Proposed lab test (text only; **do not run**)

**Goal:** Separate **(1) kr level**, **(2) res_sign**, **(3) pos stiffness** on **A1**, yaml-only, no reflash.

**Pre:** Apply [`docs/lab_prep_log_filter_params.patch`](lab_prep_log_filter_params.patch) on CS2 so meta logs **`res_sign`, `filt_dt_us`, `notch_en`, `filt_prewarp`**.

| Step | `indi_gains` / `pos_gains` | Abort if |
|---|---|---|
| 0 | Baseline **2400/170**, pos **64/48/5/7**, **res_sign=+1**, `ctrl_mode=3` | Re-establish reference shake (gyro RMS ≫ Omar run same day) |
| 1 | **kr=420, kw=69** (Omar-equiv), else same | tilt **&gt;25°** sustained or crash |
| 2 | **res_sign=-1**, restore kr=2400 | Same abort |
| 3 | **pos 28/30/5/7**, res_sign=+1, kr=2400 | Same abort |
| 4 | **Combined:** kr=420, kw=69, res_sign=−1, pos 28/30/5/7 | Same abort |

**Supports / refutes (JUDGEMENT):** Large drop in gyro RMS at step 1 **supports** gain-level hypothesis; step 2 **supports/refutes** sign; step 4 clean hover **supports** package story — **none** proves root cause alone.

**Log:** uSD + radio; verify meta records new fields post-patch.

---

## Files created

- `docs/57_INDI_Gain_History_and_Package_Simulation.md`
- `experiments/analysis/indi_gain_history_scan.py`
- `experiments/analysis/indi_package_small_signal.py`
- `experiments/analysis/indi_package_sim.py`
- `experiments/analysis/indi_package_sim_plot.py`
- `flying_drone_stack/tests/indi_package_sim_harness.rs`
- `experiments/analysis/out/indi_package/*`

## Files read (principal)

`docs/56`, `54`, `55`, `41`, `FLIGHT_CARD_VALIDATION.md`, `lab_sessions/2026-09-09.md`, `2026-09-11.md`, `2026-10-02.md`,  
`lib.rs` (res_sign block), `traj_iface.c`, `crazyflies.yaml`, `experiments/logs/*.csv` metas,  
`indi_loop_rate_harness.rs` (reference only, not modified).

## Modified / committed

**No existing file modified.** **No commit or push.**

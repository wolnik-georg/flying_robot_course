# 35 — Desk parallel track audit closeout

**Date:** 21 September 2026  
**Verdict:** **COMPLETE** (independent re-audit after fixes)

---

## Scope

Twelve indexed items in [`31_Desk_Parallel_Track.md`](31_Desk_Parallel_Track.md) (§1–12), plus hub
docs and analysis tooling for C.4 / C.1 alignment.

---

## Critical fix (C.4 grouping)

**Problem:** `run_a8_2026_09_19_suite.py` mapped `stock_lee` → `controller=geometric`, so
`run_c4_desk_prep.py` headline tables showed geometric **n=5** (3 stock-Lee + 2 geometric).

**Fix:**

- `CTRL_LABEL["stock_lee"]` → `"stock_lee"` (distinct from geometric).
- `run_c4_desk_prep.py` writes `phase_metrics_c4_compare.csv` (geo + full INDI only) for
  summarise/plots; archive rows stay in `phase_metrics_filtered.csv`.
- `aggregate.py` / `plot_comparison.py`: `--group-by` supported.
- Patched `a8_2026-09-19_phase_metrics_all.csv` so `controller` matches `study_controller`.

**Verified (19 Sep A8, scenario phase, cf5):** geometric mean pos RMSE **~42.9 mm** (n=2) vs
indi **~36.9 mm** (n=2) — indicative, not statistical.

---

## Other audit follow-ups (done)

| Item | Where |
|---|---|
| Canonical docs/24 row pipeline | [`24`](24_Downwash_Compensation_Comparison.md) — `compare_downwash --merged-usd` scenario window |
| Post-session cf5 yaml reset | [`28`](28_USD_Radio_Matching_and_Session_Analysis.md) checklist |
| docs/07 ↔ docs/34 NA-INDI pointer | History (46) in [`07`](07_Thesis_Progress_Checklist.md) |
| compare_downwash docstring (cf5) | `flying_drone_stack/tools/compare_downwash.py` |
| dataset stats clarity | `flying_drone_stack/tools/residual/dataset.py` |
| Paired compare stdout | `run_c4_desk_prep.py` calls `aggregate` in-process (no subprocess swallow) |

---

## Does not block lab

- **C.1 (updated 2026-09-23):** ~24 training merges banked; **A4 ×4** only required next. Before flying: geometric on **cf5**, yaml `controller: 6`, `ctrl_mode: 0`.
- **Desk track:** no open items; re-run suite after new flight days only.

---

## Re-audit prompt

Copy-paste template for a future pass: see chat export / regenerate from §1–12 checklist in
`docs/31` (items, paths, verification commands).

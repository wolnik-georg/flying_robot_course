# 31 — Desk parallel track (no lab)

**When to use this.** Lab priority: **C.1** (NS2 data, geometric on cf5) and **INDI baseline**
flights. Between sessions, use this index so desk work stays aligned with the **comparative
study (C.4)** and does not drift into stale docs.

**Last updated:** 21 September 2026  
**Status:** **COMPLETE** — independent audit passed after C.4 grouping fix; see
[`35_Desk_Parallel_Audit_Closeout.md`](35_Desk_Parallel_Audit_Closeout.md).

---

## Grouping rule (do not regress)

- **`study_controller`** on each flight: `stock_lee` | `geometric` | `full_indi` | …
- **`controller`** in phase CSV must **not** alias stock Lee to geometric (`run_a8_*_suite.py`
  `CTRL_LABEL`).
- **Headline C.4 desk output** uses `phase_metrics_c4_compare.csv` (geo + INDI only), not the
  full filtered archive that includes baselines.

---

## 1. C.4 analysis prep (P3–P4)

| What | Where |
|---|---|
| Gap analysis & policy | [`27_Analysis_and_Metrics_Plan.md`](27_Analysis_and_Metrics_Plan.md) |
| Aggregation + paired tests | `experiments/analysis/aggregate.py` |
| Thesis figures | `experiments/analysis/plot_comparison.py` |
| **One-command desk run** | `python3 experiments/analysis/run_c4_desk_prep.py` (figures use pyenv `flying_robots` if present) |
| Default input | `experiments/analysis/out/a8_2026-09-19/a8_2026-09-19_phase_metrics_all.csv` |
| Default output | `experiments/analysis/out/c4_desk_prep/` (`phase_metrics_c4_compare.csv` → figures) |
| Audit closeout | [`35_Desk_Parallel_Audit_Closeout.md`](35_Desk_Parallel_Audit_Closeout.md) |

After new flight days: re-run `run_a8_*_suite.py` or extend suite → refresh phase CSV →
`run_c4_desk_prep.py`.

---

## 2. Comparison protocol (docs + thesis Ch. 5)

| What | Where |
|---|---|
| Living results table | [`24_Downwash_Compensation_Comparison.md`](24_Downwash_Compensation_Comparison.md) — **cf5**, uSD |
| C.1 collection protocol | [`25_C1_Data_Collection_Plan.md`](25_C1_Data_Collection_Plan.md) |
| Full C.4 protocol prose | [`05_Experimental_Protocol_2Robot.md`](05_Experimental_Protocol_2Robot.md) |
| uSD match → merge workflow | [`28_USD_Radio_Matching_and_Session_Analysis.md`](28_USD_Radio_Matching_and_Session_Analysis.md) |

**Desk task:** After each lab day, add rows to **docs/24**; update Ch. 5 experimental section
(local thesis) to past tense for scenarios actually flown.

---

## 3. C.2 dry-run (train pipeline)

| What | Where |
|---|---|
| Tooling | `flying_drone_stack/tools/residual/train.py`, `dataset.py` |
| Docs | [`13_Residual_Learning.md`](13_Residual_Learning.md), `tools/residual/README.md` |
| Log of dry-runs | [`experiments/analysis/out/c2_dryrun/manifest.json`](experiments/analysis/out/c2_dryrun/manifest.json) |

```bash
# Rehearsal on archived merge (not a deployable model — A8 lacks lateral C.1 coverage)
python3 flying_drone_stack/tools/residual/train.py \
  experiments/logs/a8_2026-09-19_successful/2026-09-19_15-04-41/A8_2026-09-19_15-04-41_merged_usd.csv \
  -o flying_drone_stack/tools/residual/weights/c2_dryrun_2026-09-19_geo.npz
```

Re-run after C.1 blocks land; keep weights under `weights/` with date in filename.

---

## 4. INDI baseline desk pack

[`29_INDI_Baseline_Desk_Plan.md`](29_INDI_Baseline_Desk_Plan.md) — S0/S1/S1b/S1c/S4 rows, protocol,
lab order.

---

## 5. Results skeleton (Ch. 6–7)

[`30_Results_Chapters_Skeleton.md`](30_Results_Chapters_Skeleton.md) — section outline + figure
checklist tied to analysis scripts.

---

## 6. P5 — residual evaluation

| What | Where |
|---|---|
| Script | `flying_drone_stack/tools/residual/eval_model.py` |
| Policy | [`27`](27_Analysis_and_Metrics_Plan.md) gap 7–8 |

```bash
python3 flying_drone_stack/tools/residual/eval_model.py \
  flying_drone_stack/tools/residual/weights/c2_dryrun_2026-09-19_geo.npz \
  experiments/logs/a8_2026-09-19_successful/2026-09-19_15-04-41/A8_2026-09-19_15-04-41_merged_usd.csv \
  -o experiments/analysis/out/residual_eval/c2_dryrun_geo
```

---

## 7. C.4 flight budget

[`32_C4_Flight_Budget.md`](32_C4_Flight_Budget.md) — C.1 block counts + C.4 scenario × controller × repeats.

---

## 8. Ch. 5 / validity (thesis)

Local draft: `docs/thesis/ch5_experimental_setup.tex` — campaign status paragraph and uSD/mocap
threat rows (21 Sep 2026).

---

## 9. NA-INDI supervisor memo

[`34_NA_INDI_Decision_Memo.md`](34_NA_INDI_Decision_Memo.md)

---

## 10. docs/24 from uSD merges

```bash
python3 flying_drone_stack/tools/compare_downwash.py \
  experiments/logs/a8_2026-09-19_successful/2026-09-19_15-04-41/A8_2026-09-19_15-04-41.meta.json \
  --merged-usd experiments/logs/a8_2026-09-19_successful/2026-09-19_15-04-41/A8_2026-09-19_15-04-41_merged_usd.csv \
  --markdown --drone cf5
```

Legacy radio CSV path unchanged (no `--merged-usd`).

---

## 11. Post-session checklist

[`28_USD_Radio_Matching_and_Session_Analysis.md`](28_USD_Radio_Matching_and_Session_Analysis.md) — **Post-session desk checklist** (copy-paste bash block: suite → C.4 prep → docs/24 → lab log).

---

## 12. Three-drone sim smoke

| What | Where |
|---|---|
| Interpretation + hardware blockers | [`33_Three_Drone_Sim_Smoke.md`](33_Three_Drone_Sim_Smoke.md) |
| Re-run script | `experiments/analysis/run_3drone_smoke.sh` |
| Latest attempt log | `experiments/sim_validation/three_drone_smoke_2026-09-21.md` |
| Authoritative sim matrix | [`12_Sim_Formation_Validation_Report.md`](12_Sim_Formation_Validation_Report.md) |

---

## 13. Optional — n≥4 formation sketch (desk)

| What | Where |
|---|---|
| D/E/F scenario tables + sim matrix tiers | [`36_Four_Five_Robot_Formation_Sketch.md`](36_Four_Five_Robot_Formation_Sketch.md) |
| Literature complement (Grok merge) | [`36_Four_Five_Robot_Complement.md`](36_Four_Five_Robot_Complement.md) |

Not on the C.1 critical path; useful for Ch. 7 and supervisor discussion.

---

## Also see

- Audit closeout (COMPLETE): [`35_Desk_Parallel_Audit_Closeout.md`](35_Desk_Parallel_Audit_Closeout.md)
- Visual two-track overview: [`Thesis_Progress_Overview.html`](Thesis_Progress_Overview.html)
- Status SSOT: [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md)
- Next lab: [`next_flight_card.html`](next_flight_card.html)

# 34 — NA-INDI decision memo (~1 page for supervisor)

> **2026-09-22 — Supervisor decision (Sep 2026 meeting): NA-INDI / LINDI / Cobo hybrid is NOT in the
> compared set.** Thesis compared strategies are 0–3 only (see `docs/07` banner). **This memo is
> kept as history** of the pre-decision options; do not treat Option A/B/C below as open campaign work.

**Date:** 21 September 2026  
**Question for meeting:** How much NA-INDI work belongs in the thesis **before** C.1 volume is banked?

---

## What we are deciding

NA-INDI appears in the thesis as **Strategy 4 / hybrid** and as **Cobo-Briesewitz reference
(`controller=8`)**. Three paths are on the table:

| Option | What it is | Lab cost | What it proves |
|---|---|---|---|
| **A — Reference only** | Fly **`c=8`** with **their** weights unchanged | Low (once sim/hover ladder clean) | Faithful reproduction; **not** trained on downwash — weak alone as “NA-INDI failed” |
| **B — Retrain own-state net** | Same 19→24→… architecture, **our** C.1 labels, new min-max stats | Medium (desk train + upload slot policy) | Fair test of **own-state-only** compensation on **our** disturbance |
| **C — Both A and B** | 8 vs retrained vs Strategy 2/4 | Highest | Clean attribution: weights vs input space vs neighbour-aware NS2 |

**Project doc stance ([`25`](25_C1_Data_Collection_Plan.md)):** Strategy **4 hybrid** (`c=6`,
`ctrl_mode=3`, `rnn.en=1`) reuses **the same NS2 weights as Strategy 2** — no separate
`controller=9` slot required for neighbour-aware hybrid. That is **not** the same experiment as
retraining **their** MLP on own state ([`26`](26_Controller9_NA_INDI_Retrained.md) — scoped as
`controller=9` in older notes; [`25`](25_C1_Data_Collection_Plan.md) supersedes for NS2 hybrid).

---

## Recommendation (desk view)

1. **Do not block C.1** on NA-INDI. Collection stays **geometric on cf5**.
2. **Prioritise C.4 S0 vs S1** repeats on A8 / A4 / A7 ([`32`](32_C4_Flight_Budget.md)).
3. **NA-INDI for thesis minimum (Methods 1–4):** plan **Option A** (reference `c=8`) **plus**
   **Strategy 4 hybrid** after C.2 weights exist — answers “additive NN on full INDI” without a
   second training pipeline.
4. **Option B (retrain their architecture on C.1)** is the right **optional / advanced** chapter
   if supervisor wants the **own-state vs neighbour-state** headline isolated; it is **desk +
   one upload path**, not a second collection campaign.

---

## What to agree in the meeting

- [ ] Is **reference `c=8` only** enough for “Hybrid/NA-INDI” in the minimum thesis, with Strategy 4
      as the learned hybrid once C.2 lands?
- [ ] If **retrain own-state MLP (Option B)** is required, confirm slot policy (`c=9` vs separate
      weight file on `c=8`) and whether it **blocks** or **parallelises** with C.1.
- [ ] Confirm **S1c (`c=7`)** priority vs **`c=8`** — both were sim-clean; hardware ladder is
      similar.

---

## References

- [`26_Controller9_NA_INDI_Retrained.md`](26_Controller9_NA_INDI_Retrained.md) — architecture and 8 vs 9 reasoning  
- [`29_INDI_Baseline_Desk_Plan.md`](29_INDI_Baseline_Desk_Plan.md) — S4 row  
- [`meetings/2026-09-21.md`](meetings/2026-09-21.md) — discussion prompt  

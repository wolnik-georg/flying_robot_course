# 32 — Flight budget: C.1 collection vs C.4 comparison

**Purpose.** Turn [`25`](25_C1_Data_Collection_Plan.md) and [`05`](05_Experimental_Protocol_2Robot.md)
into a **calendar-style count** so lab time and battery planning stay honest. Numbers are **scenario
time only**; add ~25 s ramp/land/arm per flight and ~6–8 flights per battery pair ([`25`](25_C1_Data_Collection_Plan.md)).

**Last updated:** 21 September 2026

---

## C.1 — NS2 training data (geometric on cf5)

| Block | Flights | Notes |
|---|---:|---|
| A — dz coverage | **10** | A7×3 + A1 holds |
| B — relative velocity | **7** | A3 / A2 |
| C — lateral inside gate | **4** | A4 lemniscate |
| D — ground (`phi_G`) | **5** | C5 single-drone |
| **Full plan total** | **26** | |
| **Minimum viable** | **18** | A + B-1/B-2 + D-1/D-2 only |

**Wall-clock (order of magnitude):** assume ~40–90 s logged window per flight plus ground ops
→ **~4–6 min per flight** all-in → **~2–2.5 h** for 18 flights, **~3 h** for 26 (plus swaps).

C.1 does **not** include controller A/B repeats; bottom stays geometric throughout.

---

## C.4 — Systematic comparison (repeats)

Protocol: **≥ 5 repeats** per controller × scenario ([`05`](05_Experimental_Protocol_2Robot.md));
analysis layer expects **3–5** for mean±std ([`27`](27_Analysis_and_Metrics_Plan.md) P3).

### Headline scenario set (desk default)

| Scenario | Role |
|---|---|
| **A8** | Vertical exchange — already indicative rows (19 Sep) |
| **A4** | Lateral offset — mandatory for C.1 *and* C.4 |
| **A7** | Dynamic merge — high gate coverage |

Optional expansion: **A3** (velocity sweeps), **C1–C3** nulls (already in library).

### Controller rows (phased)

| Phase | Strategies | Count |
|---|---|---:|
| **Now** | S0 geometric, S1 our INDI | 2 |
| **After C.2** | + S2 geometric+NN | 3 |
| **After NA-INDI decision** | + S4 (`c=8`) or hybrid `ctrl_mode=3` + `rnn.en` | 4 |
| **Stretch** | S1b stock INDI, S1c Briesewitz | +2 each if flown |

### Flight counts (2-drone, uSD both)

| Plan | Scenarios | Controllers | Reps | **Flights** |
|---|---|---:|---:|---:|
| **Minimum claim** | 3 | 2 | 3 | **18** |
| **Protocol target** | 3 | 2 | 5 | **30** |
| **With S2** | 3 | 3 | 5 | **45** |
| **Full Methods 1–4** | 3 | 4 | 5 | **60** |
| **+ A3 fourth scenario** | 4 | 4 | 5 | **80** |

**Wall-clock:** same ~4–6 min/flight → **30 flights ≈ 2.5–3 h** of lab time (minimum statistical
set with 5 reps), **60 flights ≈ 5–6 h** spread over several sessions.

---

## Combined campaign (sequencing)

Recommended order (matches [`07`](07_Thesis_Progress_Checklist.md) / flight card):

1. **C.1 blocks** (18–26 flights) — do not wait for INDI repeats.
2. **C.2 train** on merges — desk + one upload rehearsal.
3. **C.4 S0 vs S1** on A8, A4, A7 with 3–5 reps each.
4. **C.3 / S2** once weights and RAM path are gated.
5. **S4 / NA-INDI** per [`34`](34_NA_INDI_Decision_Memo.md).

**Double-counting:** C.1 geometric flights on A8 **do not** substitute for C.4 INDI repeats on A8 —
different `ctrl_mode`, different claim.

---

## Desk hooks

| Output | Tool / doc |
|---|---|
| Per-run rows | `compare_downwash.py --merged-usd` or analysis suite → [`24`](24_Downwash_Compensation_Comparison.md) |
| Aggregated C.4 | `run_c4_desk_prep.py` after phase CSV exists |
| Residual quality | `eval_model.py` after C.2 weights exist |

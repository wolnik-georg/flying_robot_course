# 36b — Four- and five-robot sketch: literature complement

**Status:** Literature pass complete (Grok + operator merge). **Not a campaign gate.**  
**Parent sketch:** [`36_Four_Five_Robot_Formation_Sketch.md`](36_Four_Five_Robot_Formation_Sketch.md)  
**Frozen library:** A1–C5 in [`10`](10_Formation_Library.md) — do not change.

**Last updated:** 21 September 2026

> **Critical path unchanged.** This sketch must **not** delay C.1 / lab work. Tier 1 n≥4 sim is optional desk time only, after n=2/3 hardware story is stable.

---

## Headline

Among papers in our comparison set, **only two free-flight works actually fly ≥4 vehicles:**

| Paper | n (free flight) | Geometry to copy |
|-------|-----------------|-------------------|
| **Neural-Swarm** (2020) | **5** (swap manoeuvre); **4** in training config | Vertical swap; Δz **0.20–0.25 m** (0.25 m cited for 3-vehicle case) |
| **Neural-Swarm2** (2022) | **16** | **3-ring** task; **min vertical 24 cm** |

**Gielis et al. (2023)** uses **4 vehicles total** but the **sufferer is load-stand locked** — side-by-side, leader–follower, stack, triangle hybrid; three force regimes. **Not** free-flight RMSE benchmarks; **do not** treat their Δz = 0.24 m as our citation for stack spacing without verifying in PDF.

---

## 1. Literature that constrains D/E/F

| Paper | n | What to copy | Do not copy |
|-------|---|--------------|-------------|
| **Neural-Swarm** | 5 (swap); 4 (train) | Swap geometry; Δz 0.20–0.25 m | **2.4 cm** height error as “2-trained” headline — that number is the **4-trained** network result |
| **Neural-Swarm2** | 16 | Min vertical **24 cm**; **3-ring** task layout | Treating “16 @ 24 cm” **and** “3× improvement” as one single experiment (body text ~**1.5×** on the 16-robot flight) |
| **Gielis** | 4 total, rig | Side-by-side, LF, stack, triangle hybrid; 3 aggregation regimes | Free-flight tracking RMSE; **Δz = 0.24 m** as their published stack spacing without verification |
| **Hsieh 2025** (L1 KNODE-DW / flatness line) | **3** | V-stack / I-stack **0.2 m** | Any **n = 4** hardware claim |
| **Smith, ProxFly, Shankar, Gu, Abro, Chee MPC, …** | 1–2 | Spacing / docking **ideas** | A published **4-robot free-flight** geometry |

---

## 2. Existing sketch — literature anchors

| Strong map | Weak / thesis-original |
|------------|-------------------------|
| **D4** 2×2 vertical swap → NS **n = 4** train / swap family | **D3** tetra, **D5** star, **D6** reverse-circle platoon, **D7** offset train |
| **E5** five-way swap → NS **n = 5** swap | **E2** 3+2 hybrid, **E3** double-V, **E4** pentagon stack |
| **D1 / E1** column + **E6** @ **0.24 m** → NS2 **spacing discipline**, **not** a published I-stack of 4 or 5 in their paper | |
| **F4a / F5a** | In-plane **controls** (low interaction) |
| **F4b / F5b** | Gielis **LF line shape** only (rig ≠ our free flight) |

---

## 3. Six additional scenarios (Tier 2+ only — after Tier 1 smoke)

| ID | Short name | Rationale |
|----|------------|-----------|
| **D8** | Two-ring four | Scaled-down **NS2 three-ring** idea at n = 4 |
| **D9** | Gielis-quad | Their **3-flyer triangle + one free bottom** vehicle (aggregate / sufferer not on stand) |
| **D10** | Ground + stack | NS2 **ground-effect residual** thread; gated behind C5-style near-ground policy |
| **E7** | Two-ring five | **D8** + centre column (n = 5) |
| **E8** | Swap-column five | **E5** crossings held on a **circle** so `a_res` sees sustained excitation, not a single spike |
| **F4c** | Rotated square | **Packing / mocap** check only — not a science scenario |

Add to [`36`](36_Four_Five_Robot_Formation_Sketch.md) scenario tables when implementing code; not before Tier 1.

---

## 4. Room and dz budget

- **Volume:** 2 m (x) × 4 m (y) × 1.70 m (z), tape-measured — same as doc 10. Prefer long axis **y**; use `--auto-center --rotate 90`.
- **E1 at Δz = 0.50 m does not fit:** four gaps × 0.50 m exceeds z budget with safe bottom clearance. **Five-stack default: dz ∈ {0.24, 0.30} m** only unless re-measured.
- **D1:** tallest four-stack that still fits with bottom ≥ **0.15 m** is roughly **dz = 0.50 m** per gap (verify with `--check`).

---

## 5. Superposition narrative (D1 / E1)

Gielis: pairwise downwash is **not** a single linear superposition (chaotic / linear / nonlinear regimes). Neural-Swarm: same warning for **n > 2** in free flight.

Our **B1** column (3-robot I-stack, sim) already shows the split: **geometric** mean |Δz| ≈ **58.7 mm** (EXPECTED above 50 mm tolerance); **full INDI** ≈ **0.1 mm** ([`12`](12_Sim_Formation_Validation_Report.md)).

**D1 / E1** ask how **uncompensated geometric sag** grows when a **fourth and fifth** washer joins the column — that is **RQ3 past three robots**, not a claim that Gielis’s newton-scale rig results will reproduce on the Crazyflie.

---

## 6. Recommended Tier 1 execution order

Sequential smoke (sim or lab), **both controllers** where noted:

1. **F4a** — geometric → full INDI (**n = 4 plumbing**)
2. **D1** @ **dz = 0.30 m** — geometric → full INDI (use dz = 0.50 m only if z-budget confirmed with `--check`)
3. **F5a** — geometric → full INDI
4. **E1** @ **dz = 0.24 m** — geometric → full INDI
5. Then literature-aligned transients: **D4**, **E5** (not before step 1–4 clean)

**Not** part of C.1. **Not** before the n = 2/3 comparison matrix is frozen for the thesis core.

---

## 7. Full literature ↔ scenario table (completed)

| Scenario ID | Primary anchor | Secondary | Notes |
|-------------|----------------|-----------|-------|
| F4a, F5a | — (control) | Smith ~0.5 m scale (2-robot) | No n≥4 anchor |
| F4b, F5b | Gielis LF | C2 | Shape only |
| D1, E1, E6 | NS2 min Δz **24 cm** | NS 25 cm (verify vs 24) | Column hover ≠ NS2 “3-ring” task |
| D4 | NS n=4 train / swap | A8 | |
| E5 | NS n=5 swap | B3, A8 | Extreme |
| D2 | Gielis multi-body (qualitative) | B1+B2 | Rig vs flight |
| D8 | NS2 3-ring (scaled) | — | Thesis extension |
| D9 | Gielis triangle + free 4th | — | Hybrid of rig geometry |
| D10 | NS2 ground effect | C5 | Gated |
| D3, D5, D6, D7, E2, E3, E4 | — | Internal library analogues | **Thesis-original** unless sim motivates |
| E7, E8 | NS2 ring + swap ideas | D8, E5 | Tier 2+ |
| F4c | — | — | Logistics only |

---

## 8. Related

| Doc | Role |
|-----|------|
| [`36_Four_Five_Robot_Formation_Sketch.md`](36_Four_Five_Robot_Formation_Sketch.md) | Scenario IDs, sim matrix tiers |
| [`12_Sim_Formation_Validation_Report.md`](12_Sim_Formation_Validation_Report.md) | B1 EXPECTED geometric |
| [`06_References_Overview.md`](06_References_Overview.md) | Paper list |

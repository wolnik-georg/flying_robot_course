# 36 — Four- and five-robot formation sketch (n≥4 extension)

**Status:** SKETCH — **not a campaign gate.** A1–C5 stay **frozen** ([`10`](10_Formation_Library.md)). No code, no sim matrix run yet.  
**Literature complement (complete):** [`36_Four_Five_Robot_Complement.md`](36_Four_Five_Robot_Complement.md) — only **Neural-Swarm** (5-swap) and **Neural-Swarm2** (16, 3-ring, 24 cm min Δz) free-fly ≥4; Gielis = 4 vehicles with sufferer on a **stand**.  
**Purpose:** Tight overview + validation-style matrix for Ch. 7 / optional sim — **must not delay C.1.**  
**Last updated:** 21 September 2026

**Do not** treat rows below as committed experiments until each passes `run_formation --check` and a supervisor sign-off on room / fleet size.

---

## 0. Scope and naming

| Prefix | Robots | Role |
|--------|--------|------|
| A / B / C | 2 / 3 / mixed | **Frozen** core library — sim-validated ([`12`](12_Sim_Formation_Validation_Report.md)) |
| **D** | **4** | Proposed **Priority D** — interaction + superposition at n=4 |
| **E** | **5** | Proposed **Priority E** — tight scaling smoke + literature-aligned spacing |
| **F** | control | Coplanar / near-zero-interaction controls at n=4,5 (extends C1/C3) |

Default grids (same philosophy as doc 10): **dz** ∈ {0.24, 0.30, 0.40, 0.50} m where stack-like; **speed** ∈ {0.20, 0.30} m/s for moving paths only; below **0.20 m** requires `--allow-extreme`.  
**Flight volume** (tape-measured): x ∈ [−1, 1], y ∈ [−2, 2], z ∈ [0, 1.70] — every geometry must pass `--check` with `--auto-center --rotate 90` before lab.

**Pass criteria (when sim/lab runs exist):** same as doc 12 — mean |Δz| < 50 mm, mean horizontal < 80 mm, no divergence; report coverage fraction.

---

## 1. Four-robot scenario list (compact)

| ID | Short name | Geometry (commanded) | Motion | Default params | What it stresses | Maps from (library) | ⚠ |
|----|------------|----------------------|--------|----------------|------------------|---------------------|---|
| **F4a** | Coplanar square | 2×2 square, same z | hover | side **s** = 0.40 m | **Control** — low wash; plumbing check | C1 | |
| **F4b** | Coplanar line | 4 in a row, same z | hover | gap **g** = 0.40 m | Wake chain (weak wash) | C2 | |
| **D1** | Four-stack | Single vertical column | hover → optional **line** / **circle** (top-led) | dz = 0.24–0.30 m | Bottom in **three** upstream fields; superposition limit | B1 | tight dz |
| **D2** | Twin I-stack | Two parallel 2-stacks; bottom pair co-planar | hover | dz = 0.30 m, lateral **w** = 0.30 m | Two independent wash columns + coupling | B1+B2 | room |
| **D3** | Skew tetra | 4 distinct (x,y,z) — tetrahedron | hover | edge ≈ 0.35 m | Non-planar 4-body; asymmetric overlap | — | `--check` |
| **D4** | 2×2 vertical swap | Two pairs (low/high); cross swap | transient (A8-style) | dz_pair = 0.25 m | Multi-transient wash entry | A8, B3 | extreme |
| **D5** | Square + hover leader | 3 coplanar + 1 elevated center | hover | dz = 0.30 m | “Star” — one dominant source, 3 receivers | A3-like | |
| **D6** | Reverse-circle platoon | 4-high stack, counter-rotate path | circle | dz = 0.30 m, v = 0.30 m/s | Moving superposition (hardest geo) | A5, B1 | |
| **D7** | Offset stack train | 4 with staggered (x, dz) | line path | offset 0.15 m, dz 0.30 m | Partial overlap while translating | A4 | |

---

## 2. Five-robot scenario list (compact)

| ID | Short name | Geometry | Motion | Default params | What it stresses | Maps from | ⚠ |
|----|------------|----------|--------|----------------|------------------|-----------|---|
| **F5a** | Coplanar pentagon | Regular pentagon, same z | hover | circumradius **R** ≈ 0.35 m | **Control** + symmetric spacing | C3 | |
| **F5b** | Coplanar line-5 | 5 colinear, same z | hover | g = 0.35 m | Extended wake / chain | C2 | |
| **E1** | Five-stack | Single column | hover → optional circle (bottom-led) | dz = **0.24–0.30 m** (not 0.50) | Max **n−1** superposition on bottom | B1, D1 | z-budget; see complement §4 |
| **E2** | 3+2 hybrid | Line of 3 coplanar + stack of 2 above center | hover | dz = 0.30 m, line span ≤ 1.0 m | Mixed planar + vertical (Gielis-style aggregate) | B2 | |
| **E3** | Double-V | Two V-stacks (4 corners) + 1 center high | hover | dz = 0.30 m, r = 0.25 m | Multiple asymmetric sources | B2 | room |
| **E4** | Pentagon stack | F5a footprint; one vehicle +0.30 m z | hover | R = 0.30 m, dz = 0.30 m | One elevated over coplanar ring | — | |
| **E5** | Five-way swap (lite) | 5 altitudes; single crossing wave | transient | dz = 0.22 m | Scaling B3/A8 — high risk | B3 | extreme |
| **E6** | Tight NS2 spacing grid | 5 at literature min separation | hover only | dz = **0.24 m** (NS2 min) | Spacing discipline (not NS2 3-ring task) | — | dz |

### 2.1 After Tier 1 only ([`complement §3`](36_Four_Five_Robot_Complement.md))

| ID | Short name | Notes |
|----|------------|-------|
| D8 | Two-ring four | Scaled NS2 3-ring |
| D9 | Gielis-quad | Triangle + free 4th bottom |
| D10 | Ground + stack | NS2 ground-effect; gate like C5 |
| E7 | Two-ring five | D8 + centre column |
| E8 | Swap-column five | E5 on a circle for sustained `a_res` |
| F4c | Rotated square | Mocap/packing only |

---

## 3. Matrix overview — proposed validation campaign (n=4, n=5)

**Legend:** Sim/HW = ⬜ not run | 🔲 planned Tier-1 smoke | ✅ pass | ⚠ expected geo miss (like B1) | — skip by default  
**Controllers (when run):** our **geometric** + our **full INDI** only (same as doc 12); not stock INDI / c=7/8 unless explicitly added.

### 3.1 Tier 1 — smoke (2 scenarios × 2 controllers × 2 group sizes = 8 runs)

| Tier | n | Scenario | ctrl geo | ctrl indi | Params (default) | Sim | HW | Notes |
|------|---|----------|----------|-----------|------------------|-----|-----|-------|
| T1 | 4 | **F4a** square hover | ⬜ | ⬜ | s=0.40 | ⬜ | ⬜ | Must pass before any stack |
| T1 | 4 | **D1** four-stack hover | ⬜ | ⬜ | dz=0.30 | ⬜ | ⬜ | Expect geo **⚠** at dz=0.24 (like B1) |
| T1 | 5 | **F5a** pentagon hover | ⬜ | ⬜ | R=0.35 | ⬜ | ⬜ | Control + roster check |
| T1 | 5 | **E1** five-stack hover | ⬜ | ⬜ | dz=0.24 | ⬜ | ⬜ | Literature-aligned spacing |

**Execution order (Tier 1):** F4a geo→INDI → D1 @ 0.30 geo→INDI → F5a geo→INDI → E1 @ 0.24 geo→INDI → then **D4 / E5**. Full rationale: [`complement §6`](36_Four_Five_Robot_Complement.md).

### 3.2 Tier 2 — interaction core (add after Tier 1 clean)

| Tier | n | Scenario | ctrl geo | ctrl indi | Motion | Sim | HW |
|------|---|----------|----------|-----------|--------|-----|-----|
| T2 | 4 | D2 twin I-stack | ⬜ | ⬜ | hover | ⬜ | ⬜ |
| T2 | 4 | D3 skew tetra | ⬜ | ⬜ | hover | ⬜ | ⬜ |
| T2 | 4 | D6 reverse-circle platoon | ⬜ | ⬜ | circle v=0.3 | ⬜ | ⬜ |
| T2 | 5 | E2 3+2 hybrid | ⬜ | ⬜ | hover | ⬜ | ⬜ |
| T2 | 5 | E6 NS2 spacing | ⬜ | ⬜ | hover dz=0.24 | ⬜ | ⬜ |

### 3.3 Tier 3 — transients / extreme (optional; thesis “future work” if not flown)

| Tier | n | Scenario | ctrl | Sim | HW | Gate |
|------|---|----------|------|-----|-----|------|
| T3 | 4 | D4 2×2 swap | geo, indi | ⬜ | ⬜ | T2 + `--allow-extreme` |
| T3 | 5 | E5 five-way swap | geo, indi | ⬜ | ⬜ | T2 + lab time |

**Campaign size (full matrix, both controllers):**  
Tier 1: **8** | Tier 1+2: **+10 → 18** | All tiers: **+4 → 22** sim cells (×2 controllers already counted in columns).

---

## 4. Literature ↔ scenario mapping

**Completed** — see [`36_Four_Five_Robot_Complement.md`](36_Four_Five_Robot_Complement.md) §1, §2, §7. Verify numbers in `20_Verified_Claims.md` / PDF before thesis citations.

---

## 5. Engineering prerequisites (checklist)

| Item | Status | Notes |
|------|--------|-------|
| `crazyflies_sim4.yaml` / `sim5.yaml` rosters | ⬜ | Mirror `crazyflies_sim3.yaml` pattern |
| Scenario entries in `formations/scenarios.py` | ⬜ | D1–D7, E1–E6, F4a/b, F5a/b |
| `scenarios --self-test` + `--check` geofence | ⬜ | Must include new IDs |
| `run_sim_matrix.sh` extension or `run_4drone_smoke.sh` | ⬜ | Tier 1 only first ([`33`](33_Three_Drone_Sim_Smoke.md) as template) |
| Hardware: 4–5 mocap bodies + radios + batteries | ⬜ | Inventory in [`11`](11_Hardware_Readiness_Checklist.md) |
| NN residual trained on n≥4 data | ⬜ | NS2 port expects **pairwise** geometry in training — **revisit [`13`](13_Residual_Learning.md)** before claiming Strategy 2 at n=5 |

---

## 6. Thesis placement

| Chapter | Use this doc |
|---------|----------------|
| **Ch. 5** | One paragraph: “extension scenarios D/E sketched, not part of C.4 primary matrix” |
| **Ch. 7** | Tables §1–§3 if any Tier 1–2 cells run; else “future work” with this sketch as appendix |
| **Future work** | Yawed formations, sustained close proximity, n>5 |

Update [`30`](30_Results_Chapters_Skeleton.md) §7 when first n≥4 sim smoke exists.

---

## 7. Related docs

| Doc | Link |
|-----|------|
| Literature complement | [`36_Four_Five_Robot_Complement.md`](36_Four_Five_Robot_Complement.md) |
| Frozen library | [`10`](10_Formation_Library.md) |
| Sim validation (n≤3) | [`12`](12_Sim_Formation_Validation_Report.md) |
| 3-drone smoke template | [`33`](33_Three_Drone_Sim_Smoke.md) |
| Results Ch. 7 placeholder | [`30`](30_Results_Chapters_Skeleton.md) |
| References list | [`06`](06_References_Overview.md) |

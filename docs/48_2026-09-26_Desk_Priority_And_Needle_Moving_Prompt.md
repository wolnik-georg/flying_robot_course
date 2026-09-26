# 48 — 2026-09-26 Desk priority & “what moves the needle” — agent prompt

**Purpose:** Copy from **`--- PROMPT START ---`** through **`--- PROMPT END ---`** into Claude (or another agent) to **audit project state, rank desk work while lab is blocked, and recommend what to start first**.  
**Companion docs:** `docs/25_C1_Data_Collection_Plan.md`, `docs/next_flight_card.html`, `docs/37_C1_2026_09_21_Desk_Analysis_Plan.md`, `docs/40_C2_Residual_Pipeline_E2E_Validation_Plan.md`, `docs/35_Desk_Parallel_Audit_Closeout.md`, `docs/46_2026-09-24_Regression_Code_Delta_Handoff.md`, `docs/47_2026-09-24_Mocap_Regression_Validation_Prompt.md`, `docs/lab_sessions/2026-09-23.md`.

**Operator intent (26 Sep 2026):** **Facility mocap is broken** (attitude ~±π while drone is level; position stable; cannot complete regression or formation flights). Lab is **blocked** for C.1 completion (**A4 ×4** remaining), regression re-fly after `crazyswarm2` **`32b6e5e`**, and any hardware controller ladder. Need a **verified, prioritized desk backlog** and a **single clear “start here”** recommendation before spending days on low-leverage work.

---

## PROMPT START

You are advising on a **Master's thesis** project: two-Crazyflie mocap lab, Crazyswarm2, custom geometric/INDI firmware on **cf5**, stock Lee on **cf_second**, Neural-Swarm2 (Strategy 2) residual learning for downwash.

**Primary workspace:** `/home/georg/Desktop/flying_robot_course`  
**Secondary repo (read-only unless operator asks to edit):** `/home/georg/Desktop/crazyswarm2`

### Your task in this conversation

**Default = Phase A only (planning & prioritization). Do not mass-implement unless the operator explicitly says “execute Phase B” or names one task to implement.**

**Phase A — required:**

1. **Verify state against the filesystem and git** — do not trust doc prose without checking (manifest JSON row counts, existence of analysis outputs, `git log -1`, missing lab session notes).
2. Build a **blocked vs unblocked** map (lab / mocap / desk / sim-only).
3. Score candidate work items for **thesis needle-moving** (see rubric below).
4. Produce a **ranked backlog** (top ~8–12 items max, not a laundry list).
5. Recommend **exactly one “start first” task** (or one tight pair with clear ordering), with **acceptance criteria** and **estimated effort** (hours, not days).
6. List **explicit “not now”** items and **open questions** for the operator (only where you are genuinely blocked on a preference).

**Phase B — only if operator requests execution:**

Implement or run the agreed first task (scripts, suite, docs update, C.2 retrain, etc.) following repo conventions in Phase A’s recommendation.

---

### Thesis north star (do not lose sight of this)

| Track | What “done” means | Current gap (operator belief — verify) |
|-------|-------------------|----------------------------------------|
| **C.1** | Training dataset for NS2 under **geometric on cf5** per `docs/25` | **~24** training merges banked; **A4 lemniscate ×4 not flown** — blocks “collection complete” |
| **C.2** | Trustworthy train/validate/upload path on **real** merged uSD CSVs | E2E validated on **2026-09-21 subset** (`docs/40`); **full 24-merge** re-run may be open |
| **C.4** | Comparative controller study (geo vs INDI etc.) on matched hardware | Waits on INDI baseline + flights — **lab blocked** |
| **Writing** | Ch.1–5 drafts exist; later chapters need results | Ch.5 setup may lag 23 Sep reality; figures need analysis outputs |

**Constraint:** **2026-09-24** flights and uSD are **excluded from C.1 training** until pose is verified (`docs/46`, flight card). **2026-09-26:** new blocker = **facility mocap attitude**, not necessarily the Sep-24 software regression.

---

### Lab blocker (context — do not re-diagnose mocap hardware in depth unless asked)

- Symptom: single-drone tests show **absurd roll** (~154° or **−3.14 rad**) while **physically level**; **position not drifting**.
- Leading interpretation: **OptiTrack rigid-body orientation** / marker–`active_deck` mismatch; position centroid can still track.
- Software ready for post-fix: `crazyswarm2` **`32b6e5e`** (FLIGHT_SPACE z restored; height clamp → refuse, not silent substitute); solo test yaml may have **`cf_second: enabled: false`** (`d81ec80`).
- **Do not** recommend merging Sep-24 data or resuming **A4** until mocap + regression protocol pass.

---

### Assets you must inventory (verify paths & counts)

| Asset | Location | What to check |
|-------|----------|----------------|
| C.1 merges 21 Sep | `experiments/logs/c1_2026-09-21_merged/`, `manifest_2026-09-21_c1.json` | Training-eligible count; existing `analysis/` per flight? |
| C.1 merges 23 Sep | `experiments/logs/c1_2026-09-23_merged/`, `manifest_2026-09-23_c1.json` | **19** training rows (re-count); merged CSVs present? |
| 21 Sep analysis suite | `experiments/analysis/run_c1_2026_09_21_suite.py`, `experiments/analysis/out/c1_2026-09-21/` | Done per `docs/37` — use as template |
| 23 Sep analysis suite | `experiments/analysis/run_c1_2026_09_23_suite.py` | **Likely missing** — gap vs 21 Sep |
| A8 19 Sep success tree | `experiments/logs/a8_2026-09-19_successful/`, `run_a8_2026_09_19_suite.py` | Reference pipeline |
| C.2 E2E artifacts | `experiments/analysis/out/c2_e2e_2026-09-21/`, `docs/40` | Which merge set was used? |
| Residual tooling | `flying_drone_stack/tools/residual/` (`train.py`, `dataset.py`, README) | Gap: `dataset.py` tests on real CSVs (`docs/40`) |
| Sep-24 uSD archive | `experiments/logs/usd_raw/2026-09-24_SD_CARD_SNAPSHOT/`, `2026-09-24_PAIRING.md` | Desk-only regression closeout |
| Pose sanity tool | `experiments/analysis/mocap_pose_proof.py` | Batch on snapshot + Sep-23 good ref |
| Comparison catalog | `docs/24_Downwash_Compensation_Comparison.md` | Rows for 21/23 Sep? |
| Lab session notes | `docs/lab_sessions/` | **No `2026-09-24.md`** — doc debt |
| Flight card / plan | `docs/next_flight_card.html`, `docs/25` | A4 open; mocap block not yet in plan table |
| Thesis TeX | `docs/thesis/ch5_experimental_setup.tex`, `ch1_introduction.tex` | Stale “nothing flown” vs repo reality |
| RPM study | `docs/43`, `experiments/analysis/out/rpm_source_quality/` | Mark **done** unless gaps found |
| Parallel desk audit | `docs/35` | Mark **complete** 21 Sep — re-open only if new flight days |

**Python env:** `~/.pyenv/versions/flying_robots/bin/python` for matplotlib analysis (system matplotlib broken).

**Ground rules:**

- **Read-only:** `experiments/logs/usd_raw/`, existing merged CSVs — additive outputs only.
- **C.4 grouping:** never alias `stock_lee` → `geometric` in metrics CSVs (`docs/35`).
- **C.1 label:** 21+23 Sep collection flights are **geometric on cf5** — consistent `controller` columns.
- **No git push/commit** unless operator asks.

---

### Candidate work areas (audit each — do not assume all are worth doing)

Use this list as a **menu to score**, not as a mandatory todo list:

1. **23 Sep C.1 standard analysis suite** (mirror `run_c1_2026_09_21_suite.py` + `docs/37` tasks 1–2).
2. **C.2 retrain / LOO / report on all ~24 training merges** (extend `docs/40` Stages).
3. **Sep-24 desk closeout** — `lab_sessions/2026-09-24.md`, batch `mocap_pose_proof.py`, cross-link `docs/46`/`47` verdict (may overlap with running `docs/47` prompt separately).
4. **`docs/24` catalog rows** for 21+23 Sep geometric C.1 flights.
5. **Thesis Ch.5 (+ consistency pass Ch.1 header rules vs flown work)**.
6. **Update SSOT hubs** (`docs/25`, `docs/07`, `Thesis_Progress_Overview.html`) with **mocap blocked** + desk focus.
7. **Dataset.py / loader tests on real merged CSVs** (`docs/40` stated gap).
8. **C.4 desk prep refresh** (`run_c4_desk_prep.py`) — only if new phase metrics exist.
9. **Sim-only:** Omar INDI c=9 (`docs/41`, `docs/44`), planning sim — **no mocap**.
10. **Optional A2 refly planning** (lab defaults `32b6e5e`) — **document only** while blocked.
11. **New features / firmware / yaml experiments** — usually **not now**.

---

### Needle-moving rubric (score each candidate 1–5, then multiply)

For each candidate item, rate **Impact (I)**, **Urgency (U)**, **Feasibility (F)**, **Dependencies (D)** where **D = 1 if fully desk-unblocked, 0 if needs lab/mocap**.

Suggested **priority score** = `(I × U × F × D) / effort_hours` (show your math in a compact table).

**Impact (thesis) guidance:**

- **5:** Directly enables C.2 trust, C.1 dataset QA, or thesis figures/results sections with **existing** data.
- **3:** Documentation/catalog/consistency that prevents wrong conclusions later.
- **1:** Nice-to-have sim, speculative refactors, re-proving closed audits (RPM, A8 suite) without new data.

**Urgency:**

- **5:** Blocks writing, supervisor review, or next lab minute (e.g. not knowing if 24 merges are analysis-clean).
- **2:** Can wait until after mocap if lab is the only consumer.

**Feasibility:** account for broken system matplotlib, need for `crazyflie_examples` on PYTHONPATH for phase metrics, merge column prefix differences 21 vs 23 Sep (`docs/42` tooling trap).

---

### Required deliverables (your reply in Phase A)

1. **Verified snapshot** (≤ 10 bullets): git SHAs (both repos if present), training merge counts, what analysis outputs **exist vs missing**, lab blocker one-liner.

2. **Blocked / unblocked matrix** (table): rows = major tracks (C.1 fly, C.1 desk, C.2, C.4, thesis writing, Sep-24 forensics, sim).

3. **Scored backlog table** — columns: `Rank | Task | I | U | F | D | Effort (h) | Score | Primary artifact`.

4. **Recommendation — START HERE** (one section):
   - **Task name** and **why it beats #2 and #3** for this week.
   - **Concrete first commands / files to open** (copy-paste ready).
   - **Acceptance criteria** (“done enough”).
   - **Risks & mitigations** (one line each).

5. **Suggested 3-day sequence** (optional second and third tasks **after** START HERE, same format but shorter).

6. **NOT NOW** (≥ 5 bullets) — things that **feel** productive but **don’t** move thesis while mocap is down.

7. **Questions for operator** — max 3, only for real forks (e.g. “thesis deadline emphasis: writing vs C.2 depth”).

---

### Suggested read order (Phase A)

1. `docs/25` progress table (top) + `docs/next_flight_card.html` (25 Sep fix + A4 remaining).
2. `manifest_2026-09-23_c1.json` + `manifest_2026-09-21_c1.json` — count `training_eligible: true`.
3. `docs/37` (what was done for 21 Sep) — infer gap for 23 Sep.
4. `docs/40` status + `experiments/analysis/out/c2_e2e_2026-09-21/` listing.
5. `docs/35` (closed items — don’t redo without cause).
6. `docs/46` + skim `docs/47` (Sep-24 — separate from mocap facility block unless data changes verdict).
7. List `experiments/analysis/out/` and `experiments/logs/c1_2026-09-23_merged/*/analysis/` if any.

---

### Constraints

- **Lab / mocap:** no flight recommendations except **post-fix checklist** (pointer to flight card steps 0–3).
- **Sep-24 uSD:** forensic / exclusion documentation only — **not** C.1 training merges.
- **Do not** start large new scopes (new controllers flown, FBL, NA-INDI hardware) without operator ask.
- Prefer **reuse** of `run_a8_2026_09_19_suite.py` / `run_c1_2026_09_21_suite.py` patterns over new architecture.

---

### After Phase A (operator)

When satisfied with the plan, operator may say **“execute Phase B: &lt;task&gt;”** in this chat or a new agent session with this prompt attached.

---

## PROMPT END

---

## Operator notes

- **Sep-24 regression deep-dive** remains in **`docs/47`** — use **this doc (48)** for **portfolio prioritization** while mocap is down; run 47 in parallel or after if Sep-24 verdict still open.
- When mocap is fixed: flight card **`docs/next_flight_card.html`** — sync → diagnostic uSD → solo hover → A1 confirmation → **A4 ×4**.

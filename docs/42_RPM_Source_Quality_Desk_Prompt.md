# 42 — RPM source quality desk study — agent prompt (plan refinement)

**Purpose:** Copy everything below the `--- PROMPT START ---` line into Claude (or another agent) to **refine the analysis plan, metrics, and execution strategy** before running batch analysis.  
**Not in scope for this study:** July 2026 INDI loop-stability narratives, `docs/23` §4 flight gates, or “prove DShot is safe for INDI.” Treat those as unrelated history.

**Operator intent (2026-09-23):** Isolated investigation of **optical RPM deck** vs **DShot bidirectional ESC telemetry** as **measurements** — agreement, lag, dropouts, per-motor behavior, robustness across scenarios and vehicles — using data already logged on C.1 flights.

---

## PROMPT START (copy from here)

You are helping with a **Master's thesis** repo: Crazyflie 2.1 brushless, Crazyswarm2, custom Rust/C controllers. **Workspace:** `/home/georg/Desktop/flying_robot_course`.

### Your task in this conversation

**Phase A only (unless the operator explicitly says “execute”):**

1. Read the cited files and **verify inventory against the filesystem** (manifests, merge CSV headers, raw `.bin` counts). Do not trust doc narratives without checking.
2. **Improve** the analysis plan: metrics, segmentation (hover vs maneuver), aggregation, edge cases, tooling gaps, deliverable structure, and risks.
3. Propose a **minimal execution order** (what to run first, sanity checks, acceptance criteria for “done enough for thesis methods”).
4. Flag anything **unverifiable** rather than inventing numbers.

**Do not** in Phase A: mass-edit firmware, retrain models, or push git. Analysis script changes are OK to *propose*; implement only if operator approves.

---

### Scope boundary (critical)

| In scope | Out of scope |
|----------|----------------|
| Compare **logged** `rpm.m1–4` (optical deck) vs `motor.m1_rpm–m4_rpm` (DShot) on the **same uSD timeline** at **500 Hz** | Re-opening July “DShot crashed INDI” root-cause or Story A/B |
| Sensor **quality**: bias, lag, correlation, dropout/invalid samples, per-motor spread, scenario dependence | Mandatory **`rpm_source` A/B flight tests** or `ctrl_mode` H0 partitions |
| Top vs bottom vehicle (**cf5** vs **cf_second**) where both exist in merge or raw archives | Proving closed-loop INDI stability from logs alone |
| Optional: how differing RPM sources would change **offline** τ/a_res reconstruction (sensitivity), clearly labeled hypothetical | Changing standing **`indi_gains.rpm_source=1`** policy without operator sign-off |

**Operational context (not the study question):** Since **2026-09-16**, **`cf231_active`** uses **`rpm_source=1` (DShot)** for control because the optical deck reported **zeros on 2/4 motors** in real 2-drone flight (historical hardware issue). C.1 still logs **both** sources on the uSD card for comparison. C.1 flights are **geometric on cf5** (`stabilizer.controller=6`), not full INDI — that is **fine** for sensor comparison.

---

### Ground truth data (verify and refresh counts)

| Asset | Location | Notes |
|-------|----------|--------|
| C.1 merged logs | `experiments/logs/c1_2026-09-21_merged/`, `experiments/logs/c1_2026-09-23_merged/` | **~29** `*_merged_usd.csv` (re-count). Manifests: `manifest_2026-09-21_c1.json`, `manifest_2026-09-23_c1.json`; 21 Sep training split also in `c1_2026-09-21_merged/training_eligible_crosswalk.json`. |
| Raw uSD archives | `experiments/logs/usd_raw/` (**~87+** `.bin` files; re-count) | Pairing notes e.g. `experiments/logs/usd_raw/2026-09-23_PAIRING.md`. THESIS1/THESIS2 card folders by date. |
| uSD config in use for C.1 | `flying_drone_stack/tools/usd_thesis_config.txt` | Logs **both** deck and DShot RPM **and** PWM, gyro, `indi.a_res_*`, state, etc. at **500 Hz**, 48 channels. |
| Alternate uSD config (narrower, diagnostic) | `flying_drone_stack/tools/usd_dshot_investigation_config.txt` | Also dual RPM; not required if thesis config already dual-logs. |
| Existing analysis script | `flying_drone_stack/tools/investigate_dshot_rpm.py` | Reads **single-vehicle** decoded columns (`rpm_m1`, `motor_m1_rpm`, `z`, `gyro_*`, `tau_*`). Accepts **`.bin`** (decodes via `decode_usd_log.py`) or decoded CSV. **Does not** understand merged 2-drone prefixes (`cf5.*`, `cf5_A3_13-00-57.*`). |
| Decode / rename | `flying_drone_stack/tools/decode_usd_log.py` | `RENAME` maps `rpm.m*` and `motor.m*_rpm`. |
| Python env | `~/.pyenv/versions/flying_robots/bin/python` | Use for numpy/matplotlib scripts (system matplotlib broken on this machine). |
| Prior spot check (one flight) | `docs/23_DShot_RPM_Investigation.md` §5 | 2026-09-21 A3 cf5: bias sub-%, lag 0–4 ms. **Do not treat as comprehensive.** |

**Merged CSV column naming (tooling trap):**

- **2026-09-23 merges:** prefixes `cf5.` and `cf_second.` (e.g. `cf5.rpm_m1`, `cf5.motor_m1_rpm`).
- **2026-09-21 merges:** long run-specific prefixes (e.g. `cf5_A3_13-00-57.rpm_m1`) — dual RPM **is present** but naive `--prefix cf5` fails.

Some **C5** merges may be **cf5-only** (no top vehicle columns) — still valid for bottom-drone RPM study; document exclusions.

**Do not modify:** `experiments/logs/c1_2026-09-21_merged/` except existing crosswalk; **`flying_drone_stack/tools/residual/`** unless operator expands scope.

---

### Preliminary desk observations (hypothesis-generating — re-verify)

A prior quick pass (not thesis-grade) suggested:

- **cf5 (bottom):** cross-correlation lag often **~0.5–3 ms**; |bias| often **&lt;0.35%** on recent A1/A3/A7/C5 merges.
- **cf_second (top):** sometimes **larger** apparent lags on A1 — investigate (deck update rate, marker geometry, analysis window, merge alignment); do not pool with cf5 without labeling.
- Recent sample: **~0%** deck/DShot zeros on m1 in early flight segments — contrasts with **Sep 16** deck failure mode (historical).

Treat these as **sanity targets**, not conclusions.

---

### Suggested metric families (refine and prioritize)

1. **Agreement:** per motor bias (RPM and %), RMSE, max absolute error; scatter deck vs DShot.
2. **Timing:** cross-correlation lag (ms) with documented sign convention — script negates `np.correlate` so **positive = DShot lags deck**; validate on synthetic delay before batch.
3. **Validity:** fraction invalid (`rpm==0`, DShot zero, DShot sentinel/high); per-motor heatmaps; time segments with dropouts.
4. **Frequency content:** coherence or cross-spectrum deck vs DShot; compare to **500 Hz** logging vs **~20 Hz radio** (radio CSVs are **not** sufficient for this study — use uSD/merged at 500 Hz).
5. **Scenario stratification:** A1 hover/downwash spacing, A3 formation, A7, C5 dynamic, A2 circle (note legacy profile flags in manifest where present).
6. **Vehicle stratification:** cf5 vs cf_second; THESIS1 vs THESIS2 card if identifiable from paths/pairing notes.
7. **Optional sensitivity:** offline τ from k·RPM² with deck vs DShot — does implied residual differ meaningfully vs logged `indi.a_res_*` (DShot-fed on vehicle)? Label as **counterfactual**, not flown behavior.

---

### Deliverables (after execution — define structure in Phase A)

Propose where results should live. Options:

- New **`docs/…_RPM_Source_Quality.md`** (recommended — keeps `docs/23` historical July material separate), **or** a clearly labeled new § in `docs/23` that states “sensor quality only.”
- **`experiments/analysis/out/rpm_source_quality/`** — `manifest.csv`, per-flight summary, aggregate plots, README with reproduction commands.
- Short bullet for **`docs/07`** / flight card **only if** operator policy changes (e.g. “keep dual logging indefinitely”).

Project doc convention: when updating stale claims, add **dated Update / SUPERSEDED callouts**; preserve history.

---

### Project rules

- **No code files** (`.rs`, `.c`, `.h`, flight `.py`) unless operator approves execution and scope includes tooling.
- **No invented counts** — derive from manifests and directory listings.
- **No git push**; commits only if operator asks; **no `Co-Authored-By` / Claude session trailers**.
- Thesis PDF (`docs/thesis/thesis_draft.pdf`) usually stays out of commits.

---

### Related docs (read selectively)

| Doc | Why |
|-----|-----|
| `docs/23_DShot_RPM_Investigation.md` | Tooling + one spot check; **ignore §4 flight procedure for this study** unless operator widens scope |
| `docs/25_C1_Data_Collection_Plan.md` | Scenario definitions (A1, A3, A4, …) |
| `docs/28_USD_Radio_Matching_and_Session_Analysis.md` | Merge workflow |
| `docs/lab_sessions/2026-09-23.md`, `2026-09-21.md`, `2026-09-16.md` | What was flown; Sep 16 deck-zero → DShot default |
| `docs/next_flight_card.html` | Mentions dual RPM logging on uSD |
| `docs/37_C1_2026_09_21_Desk_Analysis_Plan.md` Task 6 | Prior idea to run `investigate_dshot_rpm.py` on one bin — supersede with batch plan |

---

### What to output in Phase A (your reply)

1. **Verified inventory table** (N merges with dual RPM per vehicle, N raw bins, gaps).
2. **Refined analysis plan** (metrics, segmentation, exclusions, statistical aggregation).
3. **Tooling plan** (extend `investigate_dshot_rpm.py` vs new `experiments/analysis/…` script; merged CSV prefix handling).
4. **Sanity checks & failure modes** (sign flip, 2 ms quantization, single-drone merges, bad pairing).
5. **Execution checklist** ordered for a half-day desk session + optional second pass.
6. **Thesis-facing paragraph outline** (methods: which RPM source feeds control vs what was logged for validation).
7. **Open questions** for the operator (only where filesystem cannot decide).

Do **not** run the full batch until the operator approves the refined plan (unless they say “execute now”).

--- PROMPT END ---

## Local note

After Claude refines the plan, operator runs execution in a follow-up session; link results from `experiments/analysis/out/rpm_source_quality/` when it exists.

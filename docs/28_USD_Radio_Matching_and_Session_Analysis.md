# 28 — uSD ↔ radio matching and session analysis (workflow + catalogs)

**Purpose.** After a lab day with uSD + radio logging, match files once, package successful
flights without touching the raw card archive, and run the P1–P4 analysis stack. This doc is the
**single entry point** for the next session; per-day flight tables live here as sections.

**Related:** `docs/27_Analysis_and_Metrics_Plan.md` (metrics policy), `docs/24` (controller
comparison table), `flying_drone_stack/tools/README_usd_thesis_logging.md` (recording + copy).

---

## Post-session desk checklist (copy-paste)

Run this **once per lab day**, in order. Do not skip steps 6–8 if the day feeds C.4 or the
thesis table.

```bash
# --- variables: edit for the session ---
DATE=2026-09-19          # lab day
STAMP=15-04-41           # one successful run (repeat block for each stamp)
SCENARIO=A8
STUDY=geometric          # geometric | indi | stock_lee — label for analysis outputs
RAW=experiments/logs/usd_raw/${DATE}_THESIS{1,2}
PKG=experiments/logs/a8_${DATE}_successful/${STAMP}
REPO=/home/georg/Desktop/flying_robot_course
cd "$REPO"

# 1) Raw archive already on disk? If not, from card copy (byte-verified):
#    python3 flying_drone_stack/tools/copy_usd_log.py ...

# 2) Radio + meta present?
test -f experiments/logs/${SCENARIO}_${DATE}_${STAMP}.meta.json
test -f experiments/logs/${SCENARIO}_cf5_${DATE}_${STAMP}.csv

# 3) Package folder + symlinks (suite does this; manual merge example):
mkdir -p "$PKG"
# merge_usd_logs.py --meta --roles bottom top -o "$PKG/${SCENARIO}_${DATE}_${STAMP}_merged_usd.csv"

# 4) Full per-flight pipeline (preferred: edit FLIGHTS in suite, then one command):
python3 experiments/analysis/run_a8_${DATE//-/_}_suite.py
# If no suite yet: copy run_a8_2026_09_19_suite.py → run_a8_<date>_suite.py, update FLIGHTS.

# 5) C.4 desk figures (after phase CSV exists):
python3 experiments/analysis/run_c4_desk_prep.py

# 6) docs/24 row (uSD, cf5 study vehicle):
python3 flying_drone_stack/tools/compare_downwash.py \
  "$PKG/${SCENARIO}_${DATE}_${STAMP}.meta.json" \
  --merged-usd "$PKG/${SCENARIO}_${DATE}_${STAMP}_merged_usd.csv" \
  --markdown --drone cf5

# 7) Optional: residual eval when training weights exist:
# python3 flying_drone_stack/tools/residual/eval_model.py weights/....npz .../merged_usd.csv \
#   -o experiments/analysis/out/residual_eval/<tag>

# 8) Log the day:
#    - Add catalog table below (section "Catalog — <date>")
#    - Link in docs/lab_sessions/<date>.md § analysis
#    - Append indicative notes only to docs/24 (label n= and uSD vs radio)

# --- C.1 study drone (crazyswarm2 crazyflies.yaml, in git since 2026-09-21) ---
# cf5: controller: 6, indi_gains.ctrl_mode: 0, rpm_source: 1, rnn.en: 0, per-robot rpm log topic
# cf_second: controller: 5 (unchanged). Pull/sync crazyswarm2 on the lab PC before flying.
```

**Gate before trusting a package:** merge role check **RMS &lt; 15 cm**; thesis counter order
matches lab notes; **not** “same git pull” pairing ([`match_usd_to_radio.py`](experiments/analysis/match_usd_to_radio.py) is exploratory only when all runs share identical A8 params).

**Outputs you should have per successful stamp:** `package_manifest.json`, `*_merged_usd.csv`,
`analysis/*_dashboard.png`, `*_phase_metrics.csv`, session aggregate under
`experiments/analysis/out/a8_<date>/`.

---

## Fast path (next lab day)

### During flying (saves hours later)

In the lab session markdown, **one line per 2-drone run**, before cards are pulled:

```text
HH:MM  A8  cf5=THESIS1  cf_second=THESIS2  thesis07+thesis08  stock Lee / geometric / …
```

If you log `thesisNN` at start/stop from the terminal, matching becomes trivial (counter order
on each card, same scenario params).

### After flying

1. **Archive cards (read-only forever)**  
   `copy_usd_log.py` → `experiments/logs/usd_raw/<date>_THESIS{1,2}/` (byte-verified). Do not
   rename or edit files in place.

2. **Radio + meta** (already in repo)  
   `experiments/logs/A8_<stamp>.meta.json`, `A8_cf5_<stamp>.csv`, `A8_cf_second_<stamp>.csv`.

3. **Match + package + analyze**  
   - Edit the `FLIGHTS` table in `experiments/analysis/run_a8_2026_09_19_suite.py` (clone for a
     new date as `run_a8_<date>_suite.py`), **or** merge manually:
   ```bash
   python3 flying_drone_stack/tools/merge_usd_logs.py \
     cf5_A8_thesisNN_....usd cf_second_A8_thesisMM_....usd \
     --meta experiments/logs/A8_<stamp>.meta.json --roles bottom top \
     -o merged.csv
   ```
   Symlink raw `thesisNN` with `copy_usd_log`-style names so columns are `cf5.*` / `cf_second.*`.

4. **One command (2026-09-19 session)**  
   ```bash
   python3 experiments/analysis/run_a8_2026_09_19_suite.py
   ```

5. **Optional matcher (exploratory only)**  
   `experiments/analysis/match_usd_to_radio.py` — scores pairs by radio correlation; **do not**
   trust it alone when all runs share the same A8 params (see 2026-09-15). Use **thesis counter
   order + merge RMS &lt; 15 cm** as the gate.

### THESIS1 / THESIS2 card labels ≠ bottom / top drone

The SD volume names (**THESIS1**, **THESIS2**) are **card IDs**, not roles. Either card can sit
in **cf5** (bottom) or **cf_second** (top) on any given day. **`copy_usd_log.py`’s `drone`
argument is only for the archive filename** — it does not prove which vehicle wore the card.

**Always pair using merge alignment:**

1. For each candidate uSD file, score **bottom** vs **top** role against the radio
   `A8_<stamp>.meta.json` (same logic as `merge_usd_logs.py --meta --roles`).
2. **Bottom** = role whose trajectory fits with lower RMS at the bottom slot; assign that file
   to **`cf5`** in symlinks regardless of whether it came off THESIS1 or THESIS2.
3. **Top** → symlink as **`cf_second`**.
4. Pass logs to merge as **`cf5_…` first, `cf_second_…` second**, with `--roles bottom top`.

When several flights share **identical A8 params**, trajectory RMS alone cannot distinguish
which `thesisNN` belongs to which radio stamp — use **flight order on the card**, **lab notes**,
and **radio row count / `t_start_sim`**, then confirm merge RMS **&lt; 15 cm** per package.

### Matching rules (non-negotiable)

**Primary (firmware with `usd.runTag`, from 2026-09-22 onward after flash + bench verify):**

1. Host writes `usd_run_tag` (unix seconds) into the flight `.meta.json` and broadcasts
   `usd.runTag` before `usd.logging=1` (see `docs/39_USD_Radio_Logging_Robustness_Plan.md`).
2. After copy, `index_usd_archive.py` on each card directory → lookup by tag.
3. `merge_usd_logs.py --run-tag <tag> --archive-t1 … --archive-t2 …` (+ `--meta` for QA).
   **Tag identifies the pair; RMS is a quality gate only** in this mode (bad RMS with a
   matching tag = real tracking/estimation issue, not a pairing mistake).

**Fallback (archives ≤ 2026-09-21, no `run_tag` column):**

| Do | Don't |
|---|---|
| `merge_usd_logs.py --meta --roles` (each drone vs **its own** commanded trajectory) | Cross-drone z-correlation for A8 alignment |
| Pair by **role fit + scenario + time + lab notes + thesisNN order** | Assume THESIS1 = bottom or THESIS2 = top; match uSD to radio from `git pull` order alone |
| Keep raw uSD under `usd_raw/`; symlink into success packages | Overwrite or delete raw card files |
| Treat merge **RMS &gt; 15 cm** as wrong file/role | Merge anyway |
| Use **uSD merged CSV** for thesis metrics (500 Hz, `ctrltarget`, `motor`, `a_res`) | Report frequency or motor metrics from 20 Hz radio CSV |

---

## Artifact layout (every successful flight)

```text
experiments/logs/a8_<date>_successful/<stamp>/
  package_manifest.json          # status, thesisNN, paths to raw archive
  A8_<stamp>.meta.json           # copy
  A8_cf5_<stamp>.csv             # copy (monitoring)
  A8_cf_second_<stamp>.csv       # copy
  cf5_A8_thesisNN_<stamp>.usd    # symlink → usd_raw/.../thesisNN
  cf_second_A8_thesisMM_....usd  # symlink
  A8_<stamp>_merged_usd.csv      # 500 Hz, source of record
  analysis/
    A8_{geometric|indi}_*_metrics.csv
    A8_*_report.md
    A8_*_dashboard.png           # plot_flight.py
    A8_<stamp>_interaction.png   # plot_interaction.py
    A8_<stamp>_phase_metrics.csv # P2 phases
```

Session index: `.../session_manifest.json`. Cross-flight aggregates:
`experiments/analysis/out/a8_<date>/`.

---

## Catalog — 2026-09-19 A8 (cf5 + cf_second)

**Drones:** cf5 = bottom / study vehicle (THESIS1 card). cf_second = top / stock Lee (THESIS2).  
**Raw archive:** `experiments/logs/usd_raw/2026-09-19_THESIS{1,2}/` (unchanged).  
**Success packages:** `experiments/logs/a8_2026-09-19_successful/`.  
**Suite script:** `experiments/analysis/run_a8_2026_09_19_suite.py`.

### Successful uSD flights (7)

| Stamp | Study config (cf5) | uSD raw (bottom + top) | Radio + meta | Merged uSD (500 Hz) | Pipeline | cf5 pos RMSE (whole log, QA only — **docs/24 uses compare_downwash scenario window**) | Key outputs |
|---|---|---|---|---|---|---|---|
| 14-58-02 | Stock Lee (`c=5`) | `THESIS1/thesis00` + `THESIS2/thesis07` | `A8_*_14-58-02.{meta.json,cf5,cf_second}.csv` | `.../14-58-02/A8_2026-09-19_14-58-02_merged_usd.csv` | merge → `run_analysis` → `plot_flight` → `plot_interaction` → phase metrics | **36.9 mm** | `analysis/A8_*_dashboard.png`, `A8_2026-09-19_14-58-02_interaction.png`, `*_metrics.csv`, `*_report.md` |
| 15-00-55 | Stock Lee | `thesis29` + `thesis18` | `A8_*_15-00-55.*` | `.../15-00-55/A8_2026-09-19_15-00-55_merged_usd.csv` | same | **104.2 mm** | same pattern under `.../15-00-55/analysis/` |
| 15-01-33 | Stock Lee | `thesis31` + `thesis32` | `A8_*_15-01-33.*` | `.../15-01-33/A8_2026-09-19_15-01-33_merged_usd.csv` | same | **31.6 mm** | `.../15-01-33/analysis/` |
| 15-04-01 | Geometric (`c=6 m=0`) | `thesis53` + `thesis33` | `A8_*_15-04-01.*` | `.../15-04-01/A8_2026-09-19_15-04-01_merged_usd.csv` | same | **40.1 mm** | `.../15-04-01/analysis/` |
| 15-04-41 | Geometric | `thesis54` + `thesis34` | `A8_*_15-04-41.*` | `.../15-04-41/A8_2026-09-19_15-04-41_merged_usd.csv` | same | **42.4 mm** | `.../15-04-41/analysis/` |
| 15-06-38 | Full INDI (`c=6 m=3`) | `thesis55` + `thesis35` | `A8_*_15-06-38.*` | `.../15-06-38/A8_2026-09-19_15-06-38_merged_usd.csv` | same | **35.7 mm** | `.../15-06-38/analysis/` (label `indi`) |
| 15-07-11 | Full INDI | `thesis56` + `thesis36` | `A8_*_15-07-11.*` | `.../15-07-11/A8_2026-09-19_15-07-11_merged_usd.csv` | same | **36.0 mm** | `.../15-07-11/analysis/` |

Paths abbreviated: full prefix is `experiments/logs/a8_2026-09-19_successful/<stamp>/`.  
**Note:** Stock-Lee-on-both meta rows are a **baseline**; geometric vs full INDI comparison uses
only the last two geometric + two INDI rows (see below).

### Excluded (not in success metrics)

| Stamp | Reason | Package folder |
|---|---|---|
| 14-59-47 | False start — radio &lt; 400 rows | yes (merge exists; `status: failed_short_radio`) |
| 15-12-08 | Stock Bitcraze INDI unstable; no distinct THESIS1 bottom file for 9th run | meta + radio only |

### Cross-flight analytics (P3–P4)

| Output | Path |
|---|---|
| All phase rows | `experiments/analysis/out/a8_2026-09-19/a8_2026-09-19_phase_metrics_all.csv` |
| Geometric vs INDI compare input (cf5, scenario phase, **n=2**) | `.../a8_2026-09-19_compare_input.csv` |
| Bar chart | `.../figures/comparison.png` |
| Aggregate CLI | `aggregate.py ... --baseline geometric --treatment indi` |

**Indicative (scenario window, uSD):** geometric 41–44 mm; full INDI ~37 mm — **not** a
statistical claim (see `docs/27`).

---

## Cloning for the next session

1. Copy `run_a8_2026_09_19_suite.py` → `run_a8_<YYYY-MM-DD>_suite.py`.
2. Update `FLIGHTS`, `USD_BOTTOM` / `USD_TOP`, `OUT_ROOT`, `AN_OUT`.
3. Add a **Catalog — &lt;date&gt;** section to this file (table like above).
4. Link from `docs/lab_sessions/<date>.md` § analysis.

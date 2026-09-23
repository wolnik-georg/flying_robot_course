# 37 — 2026-09-21 C.1 flight-analysis desk pack (dashboards, docs/24, C.4 prep, root cause, coverage, RPM, Ch.5)

**Status:** **executed 2026-09-21 (desk).** C.1 coverage grid in this doc is a **21 Sep snapshot** —
current plan status · [`25`](25_C1_Data_Collection_Plan.md) top table + [`lab_sessions/2026-09-23.md`](lab_sessions/2026-09-23.md).
Task 4 corrected same day: unified
`commanded_trajectory()` + `find_offset()` for healthy and archive paths; afternoon bins
thesis16–18 per pairing (not reused midday files). Tasks 1–8 complete — suite
`run_c1_2026_09_21_suite.py`, `experiments/analysis/out/c1_2026-09-21/`, docs/24 catalog,
`c4_desk_prep_2026-09-21/`, z diagnostic, docs/25 coverage, docs/23 DShot spot check, Ch. 5
tense, consistency pass. Agent prompt below kept for replay/audit.

## Why this exists

`docs/40`'s residual-model E2E validation closed one specific track (is the ML pipeline
trustworthy). It never ran the project's *general* flight-analysis pipeline — the same one
already used for the 2026-09-19 A8 session (`run_a8_2026_09_19_suite.py` →
`run_analysis.py`/`plot_flight.py`/`plot_interaction.py`/phase metrics) — on any of the eight
2026-09-21 merges (`experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json`).
Verified directly: `docs/24` has zero rows dated 2026-09-21, and no dashboard/phase-metric
files exist anywhere under `experiments/analysis/out/` for that date outside the residual-ML
artifacts. This plan closes that gap using data already on disk — no lab time needed.

## Ground rules for every task below

- **Read before writing.** `experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json`
  is the source of truth for which 8 flights are verified and mergeable, and what their
  `bottom`/`top` RMS and thesis-pair provenance are. Do not re-derive pairing — it's settled.
- **Read-only inputs.** Do not modify `experiments/logs/usd_raw/`, the manifest, or any
  existing `*_merged_usd.csv` under `c1_2026-09-21_merged/`. All new outputs are additive.
- **Respect the C.4 grouping rule** from `docs/35_Desk_Parallel_Audit_Closeout.md`: a flight's
  `study_controller` (`geometric`, `stock_lee`, `full_indi`, …) must never be aliased to another
  controller in any metrics CSV `controller` column — this was a real bug fixed once already
  (`CTRL_LABEL` in the 2026-09-19 suite script). All 8 of the 2026-09-21 flights are
  **geometric on cf5** (per `docs/lab_sessions/2026-09-21.md` §1) — label them that consistently,
  and do not silently pool them with the 2026-09-19 INDI/stock-Lee rows as if directly
  comparable without saying so.
- **No stale docs when done.** The final task in this plan is a documentation consistency pass
  — every doc this work touches or references must end in a state that is correct and
  cross-linked, not just individually updated in isolation.

## Task 1 — Run the standard analysis pipeline on all 8 merges

Build `experiments/analysis/run_c1_2026_09_21_suite.py`, modeled directly on
`run_a8_2026_09_19_suite.py` (same repo, read it first) but generalized:

- Iterate the 8 entries in `manifest_2026-09-21_c1.json` instead of a hardcoded flight list.
- For each entry, the merged CSV and `.meta.json` already exist under
  `experiments/logs/c1_2026-09-21_merged/<SCENARIO>_2026-09-21_<stamp>/` — do not re-merge or
  re-symlink, just consume what's there.
- Unlike the 2026-09-19 script (A8-only, hardcoded `--dz-cmd 0.25`), **read `--scenario` and
  the commanded parameter from each flight's own `meta.json`** (`scenario`, `params.dz` where
  present) — A1 and A3 flights have different `dz`/`speed` values from each other and from A8.
- Run, per flight: `run_analysis.py --scenario <SC> --ctrl geometric --logs <merged.csv>
  --sidecar <meta.json> --source hardware --out <flight_dir>/analysis` (same flags
  `run_a8_2026_09_19_suite.py` already uses — read that file for the exact argument list), then
  `plot_flight.py` and `plot_interaction.py` with the pyenv `flying_robots` interpreter (system
  matplotlib is broken on this machine — see that same script for why).
- Compute phase metrics the same way (`phase_metrics()` helper in the 2026-09-19 script, which
  imports `crazyflie_examples.formations.scenarios` — needs that repo on `PYTHONPATH`) and
  write a combined `experiments/analysis/out/c1_2026-09-21/c1_2026-09-21_phase_metrics_all.csv`
  with `study_controller=geometric` and `controller=geometric` on every row (no aliasing needed
  since there's only one controller in this dataset — state that plainly in the script's own
  docstring so a future reader doesn't wonder why `CTRL_LABEL` is trivial here).
- Output layout: dashboards/interaction plots/phase CSVs go into each flight's own
  `experiments/logs/c1_2026-09-21_merged/<SCENARIO>_2026-09-21_<stamp>/analysis/` folder
  (matching the existing `a8_..._successful/<stamp>/analysis/` convention), plus the combined
  CSV and a session manifest under `experiments/analysis/out/c1_2026-09-21/`.
- Run it, confirm dashboards/plots are produced for all 8 flights without error, and report any
  flight where `run_analysis.py`/`plot_flight.py` fails or produces an empty/degenerate plot —
  don't paper over a failure by skipping that flight silently.

## Task 2 — Add `docs/24` rows for 2026-09-21

For each of the 8 flights, run (pattern already in `docs/24`'s "Cloning for the next session"
and `README_usd_thesis_logging.md`):

```bash
python3 flying_drone_stack/tools/compare_downwash.py \
  experiments/logs/c1_2026-09-21_merged/<SCENARIO>_2026-09-21_<stamp>/<SCENARIO>_2026-09-21_<stamp>.meta.json \
  --merged-usd experiments/logs/c1_2026-09-21_merged/<SCENARIO>_2026-09-21_<stamp>/<SCENARIO>_2026-09-21_<stamp>_merged_usd.csv \
  --markdown --drone cf5
```

Add a new `## Catalog — 2026-09-21 (A1/A3/A8, geometric)` section to `docs/24`, following the
exact table format the existing `## Catalog — 2026-09-19 A8` section uses (same columns: stamp,
study config, uSD raw, radio+meta, merged uSD, pipeline, cf5 pos RMSE, key outputs). Note in
the section header that every row here is **geometric only** — there is no INDI counterpart
flown on 2026-09-21, so this catalog section is a coverage record, not a same-day geo-vs-INDI
comparison (that comparison still lives in the 2026-09-19 section). Cross-link this new section
from `docs/25`'s progress table if that table currently points only at the manifest.

## Task 3 — Extend C.4 desk prep with the new phase metrics

`run_c4_desk_prep.py` currently defaults to `experiments/analysis/out/a8_2026-09-19/
a8_2026-09-19_phase_metrics_all.csv` (2026-09-19 A8 only). Using the `--rows` flag it already
supports:

1. Run it once against the new `c1_2026-09-21_phase_metrics_all.csv` from Task 1 alone
   (`--rows <path> --out experiments/analysis/out/c4_desk_prep_2026-09-21`) and inspect the
   output — this session has no INDI flights, so `phase_metrics_c4_compare.csv` (which
   `run_c4_desk_prep.py` restricts to geometric + full INDI) will be geometric-only; confirm
   `aggregate.py`/`plot_comparison.py` handle an INDI-empty group gracefully (no crash, no
   fabricated INDI row) rather than assuming both groups exist.
2. **Do not silently concatenate** the 2026-09-19 and 2026-09-21 phase CSVs into one pooled
   comparison without checking `docs/35`'s grouping caveat first — 2026-09-19's A8 flights and
   2026-09-21's A1/A3/A8 flights are different scenarios; a pooled geometric-vs-INDI comparison
   across both days needs the grouping to be by `(scenario, study_controller)`, not just
   `study_controller`, or it will silently average across incompatible scenario geometries. If
   `aggregate.py`/`plot_comparison.py` don't already support grouping by scenario, say so
   explicitly rather than force a comparison the tooling doesn't actually support yet — a
   correctly-scoped "not done, needs a grouping-key change" is a better outcome here than a
   plausible-looking but wrong pooled table.
3. Document whichever of (1)/(2) was actually done in `docs/31_Desk_Parallel_Track.md` §1 (the
   "After new flight days" note there) and in `docs/07`'s C.4 status line, so the next reader
   doesn't have to reverse-engineer what got extended and what didn't.

## Task 4 — cf5 z-error root cause writeup (A1 @ 0.30/0.50)

Real, checkable analysis, not speculation:

1. From Task 1's phase metrics / dashboards for the successful A1 flights (dz 0.75 and dz
   0.20), extract cf5's z-tracking error time series and its RMS as a baseline for "what
   healthy cf5 z-tracking looks like on this rig."
2. For the **unmergeable** A1 @ 0.30/0.50 stamps (`12-40-41`, `12-43-34`, `12-49-36`,
   `13-27-27`, `13-28-49`, `13-30-30` — see `docs/lab_sessions/2026-09-21.md` §4's "CONFIRMED
   unmergeable" table), the archived `.bin` files still exist and can still be decoded and
   plotted even though `merge_usd_logs.py --meta --roles` correctly refuses to merge them (RMS
   ≥ 15 cm is a merge-quality gate, not a "the data doesn't exist" gate). Use
   `decode_usd_log.py` directly on the best-fit candidate bottom file the 2026-09-21 grid
   search already identified for each stamp (recorded in `experiments/logs/usd_raw/
   2026-09-21_PAIRING.md`'s grid-search table) and plot cf5's z-position and z-tracking error
   against the commanded trajectory (`find_flight_window.py`'s `commanded_trajectory()` helper
   can build the reference curve from the flight's own `.meta.json` without needing a merge).
3. Compare the shape of the error across the healthy (dz 0.75/0.20) and unhealthy (dz
   0.30/0.50) flights: is it a constant bias, a growing divergence, oscillation, or a step at a
   specific point in the flight? That shape is the actual diagnostic content — "RMS 18–21 cm"
   alone (already known) doesn't say whether this is a gain problem, an EKF problem, or
   something scenario-specific to those two dz values.
4. Write this up as a short new section in `docs/next_flight_card.html`'s existing "Z tracking
   (from plots review)" bullet (it already flags this as an open item — extend it with the
   actual evidence instead of leaving it as a bare TODO) and cross-reference from
   `docs/lab_sessions/2026-09-21.md` §4's refly table so the refly list carries a concrete
   hypothesis to test, not just "refly and hope."

## Task 5 — dz/speed coverage table/figure for `docs/25`

Using the 8 manifest merges' `meta.json` `params` (`dz` for A1, `dz`/`speed` for A3, `dz`/`span`
for A8) plus the full target grid already described in `docs/25` (blocks A–D), build a table (and
a simple 2D scatter/heatmap figure if `matplotlib` via pyenv is available) showing:

- Which `(scenario, dz)` or `(dz, speed)` cells have a **training-eligible** merge (in the
  manifest, excluding the 3 A8 QA-only flights per `docs/28`'s training-data rule).
- Which cells were attempted but are **unmergeable** (from `docs/lab_sessions/2026-09-21.md`
  §4) — shown distinctly from cells never attempted at all.
- Which cells the full C.1 plan (`docs/25`) still requires and have **no attempt yet** (A2, A4,
  C5, most of the A1 dz grid).

This is the concrete "what can the residual model have possibly learned" visual `docs/40`'s
Stage D discussion referred to qualitatively (structured error near A3's common dz, bad
extrapolation to A1's dz 0.75) — put the actual coverage picture next to that discussion so a
reader doesn't have to reconstruct it from two different documents. Add the table/figure to
`docs/25` directly (it already has a per-block checklist; this is the same information, spatial
instead of tabular) and cross-link from `docs/13`'s Stage D real-data note.

## Task 6 — DShot vs. deck RPM investigation on a good A3 merge

`investigate_dshot_rpm.py`'s own docstring says it takes a decoded CSV **or** a raw uSD log
file directly (it decodes internally in that case) — it is written for a single drone's log,
not a 2-drone merged CSV. Run it on **cf5's raw archive file** backing one of the accepted A3
merges (e.g. `experiments/logs/usd_raw/2026-09-21_THESIS1/cf5_pm_thesis09_thesis09_...bin`, the
bottom file behind `A3_2026-09-21_13-00-57` per the manifest) — not the merged CSV. Report the
four numbers the script is designed to produce (gyro sigma, tau_x/y FFT peak, deck-vs-DShot
agreement, deck-vs-DShot lag) and record the result in `docs/23_DShot_RPM_Investigation.md`
(referenced by the script's own header) as a new dated entry, not a replacement of its existing
history. State plainly whether this changes the standing `rpm_source=1` (DShot) choice or not —
the script deliberately doesn't make that call itself, per its own docstring; you make it from
the numbers, in the doc.

## Task 7 — Ch.5 experimental section, past tense

Check `docs/thesis/ch5_experimental_setup.tex` (referenced in `docs/31_Desk_Parallel_Track.md`
§8) for a campaign-status paragraph. If it does not yet reflect the 2026-09-21 session (A1/A3/A8
geometric flights actually flown, the 8 verified merges, the confirmed refly list), update it to
past tense for what was actually flown and keep future-tense only for what's still pending.
Cross-check against `docs/07`'s C.1 status line so the thesis prose and the checklist agree —
if either was more current than the other before this task, reconcile toward whichever is
actually correct rather than mechanically copying one over the other.

## Task 8 — Documentation consistency pass (final task, do not skip)

After Tasks 1–7, read back through every doc this work touched or references —
`docs/24`, `docs/25`, `docs/23`, `docs/07`, `docs/31`, `docs/next_flight_card.html`,
`docs/lab_sessions/2026-09-21.md`, `docs/thesis/ch5_experimental_setup.tex` — and check for:

- **Contradictions**: does one doc say a flight is "done" while another still lists it as a gap?
- **Dangling references**: does anything point at a file this work was supposed to create but
  didn't (or created under a different name/path)?
- **Redundant or superseded content**: if a doc had a placeholder ("no analysis yet for
  2026-09-21") that this work has now filled in, remove the placeholder rather than leaving both
  the placeholder and the real content side by side.
- **`docs/31_Desk_Parallel_Track.md`'s own status line** — it currently says "**COMPLETE**"
  (2026-09-21, after the 2026-09-19-scoped audit in `docs/35`). This new work extends several
  of its indexed items (§1 C.4 prep, §2 comparison protocol, §6 residual eval is out of scope
  here) to 2026-09-21 data — add a short dated note there, don't silently invalidate the
  "COMPLETE" verdict for the audit that was actually about the 2026-09-19 C.4 grouping fix.

Report, in plain language, what you found and fixed in this pass — a documentation
consistency pass that finds nothing wrong is worth reporting as "checked, nothing stale found,"
not skipped silently.

---

## Prompt for Cursor agent

Copy everything below into a new Cursor agent session.

---

**Context.** `flying_robot_course` is a Crazyflie multi-drone thesis project. Eight verified,
merged uSD flights from 2026-09-21 exist at `experiments/logs/c1_2026-09-21_merged/`
(`manifest_2026-09-21_c1.json` lists them — 5 A1/A3 flights are C.1 training-eligible, 3 A8
flights are QA-only). None of them have ever been run through this project's general
flight-analysis pipeline (`run_analysis.py`, `plot_flight.py`, `plot_interaction.py`, phase
metrics, `docs/24`'s comparison table) — verified directly: `docs/24` has zero 2026-09-21 rows,
and no analysis output exists for that date outside a separate, already-closed residual-ML
validation (`docs/40`). This is a desk-only task using data already on disk; no lab time or new
flights needed. Full rationale and exact per-task detail:
`docs/37_C1_2026_09_21_Desk_Analysis_Plan.md` in this repo — **read it in full before starting**,
it has exact script/flag references already verified against the current source. Execute Tasks
1 through 8 in order; **Task 8 (documentation consistency pass) is not optional** — every doc
this work touches must end in a correct, non-contradictory, non-stale state, not just
individually edited in isolation.

**Task 1.** Build `experiments/analysis/run_c1_2026_09_21_suite.py`, modeled on the existing
`run_a8_2026_09_19_suite.py` (read it first for the exact `run_analysis.py`/`plot_flight.py`/
`plot_interaction.py` argument patterns and the `phase_metrics()` helper), generalized to
iterate the 8 entries in `manifest_2026-09-21_c1.json` instead of a hardcoded flight list, and
to read `--scenario` and the commanded `dz`/`speed` from each flight's own `.meta.json` instead
of a hardcoded `--dz-cmd 0.25`. Consume the merged CSVs and meta.json files that already exist
under each `c1_2026-09-21_merged/<SCENARIO>_2026-09-21_<stamp>/` folder — do not re-merge. Write
dashboards/interaction plots/phase CSVs into each flight's own `.../analysis/` subfolder
(matching the 2026-09-19 convention) plus a combined
`experiments/analysis/out/c1_2026-09-21/c1_2026-09-21_phase_metrics_all.csv` with
`study_controller=geometric`/`controller=geometric` on every row (all 8 flights are geometric on
cf5 — no aliasing needed, say so in the script's docstring). Use the pyenv `flying_robots`
interpreter for the plotting scripts (system matplotlib is broken on this machine — see the
2026-09-19 script for why). Run it and confirm all 8 flights produce dashboards without error;
report (don't silently skip) any flight where analysis fails or produces a degenerate plot.

**Task 2.** For each of the 8 flights, run `compare_downwash.py <meta.json> --merged-usd
<merged.csv> --markdown --drone cf5` and add a new `## Catalog — 2026-09-21 (A1/A3/A8,
geometric)` section to `docs/24_Downwash_Compensation_Comparison.md`, matching the existing
`## Catalog — 2026-09-19 A8` section's table format exactly. State in the section header that
every row is geometric-only (no same-day INDI counterpart), so this section is a coverage
record, not a geo-vs-INDI comparison.

**Task 3.** Run `run_c4_desk_prep.py --rows <Task 1's combined CSV> --out
experiments/analysis/out/c4_desk_prep_2026-09-21` and confirm `aggregate.py`/`plot_comparison.py`
handle an INDI-empty dataset gracefully (no crash, no fabricated INDI row). Do **not** pool the
2026-09-19 and 2026-09-21 phase CSVs into one comparison unless you've confirmed the grouping
is keyed by `(scenario, study_controller)`, not just `study_controller` — different scenarios
across the two days aren't directly comparable, and `docs/35`'s C.4 grouping audit exists
specifically because of a prior aliasing bug in this exact area. If the tooling doesn't support
scenario-keyed grouping yet, say so explicitly rather than force a pooled comparison it can't
correctly produce. Document what was actually done in `docs/31_Desk_Parallel_Track.md` §1 and
`docs/07`'s C.4 status line.

**Task 4.** Using Task 1's phase metrics for the two successful A1 flights (dz 0.75, dz 0.20) as
a "healthy cf5 z-tracking" baseline, decode (via `decode_usd_log.py`, no merge needed) the best-
fit candidate cf5 `.bin` files the 2026-09-21 grid search already identified for the unmergeable
A1 @ 0.30/0.50 stamps (`12-40-41`, `12-43-34`, `12-49-36`, `13-27-27`, `13-28-49`, `13-30-30` —
candidates are in `experiments/logs/usd_raw/2026-09-21_PAIRING.md`'s grid-search table), and
plot cf5's z-position/z-error against the commanded trajectory (`find_flight_window.py`'s
`commanded_trajectory()` builds the reference from `.meta.json` alone). Characterize the error
shape (constant bias vs. growing divergence vs. oscillation vs. a step) — this is the actual
diagnostic content, not the already-known RMS numbers. Write the finding into
`docs/next_flight_card.html`'s existing "Z tracking" bullet (extend it, don't duplicate it) and
cross-reference from `docs/lab_sessions/2026-09-21.md` §4's refly table.

**Task 5.** Build a coverage table (and a simple figure if pyenv matplotlib is available) from
the 8 manifest merges' `meta.json` params, showing which `(scenario, dz)`/`(dz, speed)` cells are
training-eligible, which were attempted but unmergeable, and which the full `docs/25` plan
still requires with no attempt yet. Add it to `docs/25_C1_Data_Collection_Plan.md` and
cross-link from `docs/13_Residual_Learning.md`'s Stage D real-data note (the qualitative
"structured near A3's dz, bad extrapolation to A1 0.75" discussion there should point at this
concrete picture).

**Task 6.** Run `investigate_dshot_rpm.py` on **cf5's raw archive `.bin`** behind one accepted
A3 merge (e.g. the bottom file behind `A3_2026-09-21_13-00-57` — check the manifest for the
exact thesis index/path) — not the merged CSV; the script is written for a single decoded drone
log and decodes raw `.bin` files itself. Report the four numbers it produces (gyro sigma,
tau_x/y FFT peak, deck-vs-DShot agreement, deck-vs-DShot lag) as a new dated entry in
`docs/23_DShot_RPM_Investigation.md`, and state explicitly whether this changes the standing
`rpm_source=1` choice — the script deliberately doesn't decide that itself.

**Task 7.** Check `docs/thesis/ch5_experimental_setup.tex` for a campaign-status paragraph. If
it predates 2026-09-21, update it to past tense for the flights actually flown (8 verified
merges) and keep future tense only for what's genuinely still pending, reconciled against
`docs/07`'s C.1 status line.

**Task 8 (mandatory, not optional).** Re-read every doc this work touched or references
(`docs/24`, `docs/25`, `docs/23`, `docs/07`, `docs/31`, `docs/next_flight_card.html`,
`docs/lab_sessions/2026-09-21.md`, `docs/thesis/ch5_experimental_setup.tex`) and fix any
contradiction (one doc says done, another says gap), dangling reference (points at something
this work should have created but didn't, or created elsewhere), or leftover placeholder text
that the new content has now superseded. Add a short dated note to
`docs/31_Desk_Parallel_Track.md` that this work extended several of its indexed items to
2026-09-21 data — do not touch or invalidate its existing "COMPLETE" verdict, which was about a
different, already-closed audit (the 2026-09-19 C.4 grouping fix in `docs/35`). Report what was
found and fixed in this pass explicitly — if nothing was stale, say that plainly rather than
skipping the report.

Do not modify `experiments/logs/usd_raw/`, `manifest_2026-09-21_c1.json`, or any existing
`*_merged_usd.csv`. All outputs are additive. Do not fly anything — every task here is desk-only.

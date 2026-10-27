# Handover: simplify and finish the meeting doc 2026-10-29 (Topic 1 FBL, Topic 2 NS2)

State 2026-10-26 (evening). Written for a fresh agent that has not seen this session. Read this file completely first, then the draft `docs/meetings/2026-10-29.md`.

## 1. Your task
Rework `docs/meetings/2026-10-29.md` so that it is **as simple, short and well structured as the last meeting doc `docs/meetings/2026-10-22.md`**, which the supervisor liked. The draft is already close (structure, plots), but still too long and too wordy.

- **Size target:** 10-22 has 107 lines, 5 numbered sections, 4 plots + 2 videos. The draft has 158 lines (Topic 1: 58, Topic 2: 78, plus overview and questions) and 6 plots. Aim for roughly the same size **per topic** as the whole 10-22 doc's sections: **about 3 sections and 2–3 plots per topic**, total about 110–130 lines.
- **Keep:** the structure of 10-22: overview of short bold bullets with key numbers, one notation line, numbered sections (table first, one italic metric line, the plot with one "how the plot is made" bullet, 1–3 plain bullets), short Questions and Next Steps.
- **Cut or merge, in this order of preference:** repeated numbers (the overview already carries them), second tables that repeat a plot, long Limits lists (one short bullet), caveat sentences, section 8 "logging" (two lines are enough, or one overview bullet), the NN-vs-INDI table if the plot carries it.
- Language: plain, short, concrete numbers; no hedging chains. Wording like the 10-22 doc ("S0 is the worst in all three scenarios", "NS2 cuts it by a third to two thirds").

## 2. Hard rules (from the user, do not break)
- **Do not commit or push.** The user reviews first. Do not add a Co-Authored-By trailer to any commit ever.
- **Never fly, flash or touch hardware.** Nothing here needs it.
- **uSD logs only** for every result in the doc. Never radio CSVs.
- **FBL: show only the final variant** (our gains = preset 1 with the position loop 0.25× the authors', stock rate-PID inner loop, C_D 0.55, network on). The internal comparisons (authors' gains preset 0 flipped on takeoff, C_D 0.236 vs 0.55, network on/off, the 2026-10-24 flights "before") are **not** part of the doc. They stay in the lab doc and results folder.
- Do not touch the 10-22 meeting doc. Do not re-run build scripts over the user's proofreading (edit the md directly, small steps).
- Do not invent protocol rules or new claims. Report what the data show; do not claim FBL beats INDI + NS2 in general (it does not horizontally; n = 1 per cell).
- Keep the **Questions** list as it is (NS2 questions of the supervisor). Do not add FBL author questions.
- Python: `~/.pyenv/versions/flying_robots/bin/python` (system `python3` only for the firmware bindings).
- Keep text and numbers consistent: every number in the doc must be traceable to a file listed below. Check them before you finish (section 6).

## 3. Style reference: how 10-22 is built
`docs/meetings/2026-10-22.md` (read it): Overview (7 bullets, bold label + key numbers) → notation line → sections 1–5, each = table (strategies as rows, `value (n=)` cells) → one italic metric line → plot(s) → 1–3 bullets → Questions → Next Steps. The main plot is `assets/2026-10-19/overview.png` (pastel bars, black whisker = std between flights, coloured dots = flights). New strategies go in the **same table as an extra bold row** (S4 FBL).

Related user preferences: `~/.claude/projects/-home-georg-Desktop-flying-robot-course/memory/feedback_meeting_doc_style_like_2026-10-22.md`, `feedback_meeting_doc_minimal.md`, `feedback_md_docs_plots_not_text.md` (plots first, little text, plots stay, one Limits bullet).

## 4. What the draft contains and where every number comes from

### Topic 1: FBL controller (S4), final variant only
| Content in the draft | Source |
|---|---|
| S4 vs S0–S3: z metrics (A1 mean z error, A8 crossing dip, A4 z RMS) | `experiments/results/fbl_rework_2026-10-26/summary_z.txt`, `fbl_flights.csv` (variant `final`), `ref_flights.csv` (S0–S3, unseen block 2026-10-23); metric code `experiments/analysis/unseen_block_analysis.py::flight_row` |
| Horizontal RMS table | same csvs, column `xy_rms` (sample std, ddof = 1) |
| Plots | `fbl2_overview.png`, `fbl2_timeseries.png` (in `docs/meetings/assets/2026-10-29/`, produced by `experiments/analysis/fbl_rework_meeting.py`) |
| Per-flight health table (7 flights) | `health_final.csv` + `fbl_flights.csv` |
| Structure of the controller, differences to the authors | `docs/fbl/STOCK_INNER_LOOP_2026-10-24.md`, paper `docs/papers/hsieh2026flatness.pdf` |
| A1 at dz 0.5 (3 flights, inside the trained range 0.25–0.75 m) | `fbl_flights.csv` cell `A1 dz0.5`; pilot S0–S3 at dz 0.5 from `experiments/results/unified_overview_2026-10-20/summary.json` |

**Key results (n = 1 per cell, A1 dz 0.5: 3):** z better than all four strategies in all cells (A1 hold −0.0 cm, A8 dip −1.2 cm, A4 z RMS 1.5 / 1.1 cm). Horizontal RMS: A1 3.0 cm (like S1 / S3), A8 5.6, A4 8.2 / 10.2 cm against 2.6–3.3 cm for S1 / S3 (2–4× worse). 7 final flights: 0 faults, yaw ≤ 9°, tilt ≤ 9.3°. Horizontal error in A4 is mostly one hump at the first move (14–20 cm at 3–4 s); the likely cause is a lag behind the plan because our position gains are 0.25× the authors' (**not tested**, say so in one clause).

**Correct facts, easy to get wrong:**
- FBL steps run at about **500 Hz** (not 1 kHz).
- The new image has **no tilt latch** (`tilt_max 180`, no rate limit, as in the authors' controller), so "no trips" means nothing for the final flights. Use tilt peaks and faults instead.
- The flight metadata of 2026-10-26 has **NaN** controller-parameter values (server readback failed); the settings are identified from the git branch (`study_FBL_stock`, commit `aa6034c`) and the lab confirmation. This belongs in one Limits clause.
- A8 crossing dip for FBL: computed like S0–S3 (cell `A8 dz0.4 dur6.25`).

### Topic 2: NS2 (supervisor feedback of 2026-10-22), closed, independent review done
The supervisor's concern: (a) the network is trained on the same cases it is tested on, (b) boring near-zero rows dominate, (c) compare the NN output with the INDI output (log both on every strategy). Everything is in `docs/topic2_ns2_quality_summary_2026-10-25.md` (read it completely, it lists every claim) and was independently checked: `docs/topic2_ns2_quality_independent_review.md`.

| Content in the draft | Source |
|---|---|
| Four strategies at unseen settings (table, bars) | `experiments/results/unseen_block_2026-10-23/` (`per_flight.csv`, `cells.csv`, `unseen_block.png`) |
| NS2 gain seen vs unseen, retention R 1.03 (CI 0.93–1.17; leave one flight out 0.99–1.06; A4 0.075 trained-only 0.68, pooled 0.98) | `retention.csv`, `verdict.json`, plot `ns2_gain_seen_vs_unseen.png` |
| NN vs INDI (corr, lag, slope; examples) | `experiments/results/nn_vs_indi_2026-10-22/`, `unseen_nn_vs_indi_2026-10-23/` |
| Held-out settings and boring rows (offline) | `experiments/results/ns2_heldout_2026-10-22/` (`results.json`; simple plot `heldout_rmse_simple.png`) |
| Logging change | `flying_drone_stack/firmware_patches/indi_estimate_always_logged.patch`, `docs/lab_sessions/2026-10-23.md` |

**Conclusions (do not change them):** (a) retention 1.03 ≥ 0.7 → no material overfit, ns2_v2 stays, no v3; (b) no uniform gain from down-weighting boring rows → no retrain (A4 about 10 % better, A1 worse at unseen settings); (c) the NN follows the INDI signal in phase (corr 0.9–0.96 with geometric control; weaker under INDI in A1), magnitude the same once the label bias of the new bottom drone (−0.3 to −0.45 m/s² far from the neighbour) is removed. **Nuances to keep (one clause each):** unseen = between the trained settings (interpolation), 2–3 flights per cell, A4 offset 0.05 of the pilot was not in training.

## 5. Plots
Current plots in `docs/meetings/assets/2026-10-29/`: FBL: `fbl2_overview.png`, `fbl2_timeseries.png`. NS2: `unseen_block.png`, `ns2_gain_seen_vs_unseen.png`, `nn_vs_indi_examples.png`, `heldout_rmse_simple.png`. Other files in that folder (`fbl_*.png` without the `2`, `nn_vs_indi_lag_slope.png`, `unseen_nn_vs_indi_*.png`, `heldout_rmse.png`) are old and unused.
- You may drop one plot per topic if a table says the same; do not add new plots unless they replace two.
- A plot must be self-explanatory: title states the takeaway, legend and axes explain the encoding; one "how the plot is made" bullet under it.
- To change a plot: edit `experiments/analysis/fbl_rework_meeting.py` (FBL) or the scripts named above and rerun; keep the 10-22 pastel style (S0 grey, S1 blue, S2 green, S3 red, S4 purple).

## 6. Check before you finish
- Every number in the doc against the csv/json above (grep it). Std in tables = sample std (ddof = 1) as in the existing 10-29 tables.
- All image links exist (`grep -o 'assets/2026-10-29/[a-z0-9_]*\.png'` and test each).
- No mention of the internal FBL comparisons (preset 0, C_D 0.236, network off, "before", 2026-10-24).
- Section numbers and cross references consistent. Line count in the target range.
- Questions untouched; Next Steps updated only for FBL: (repair or replace the bottom drone; A1 at dz 0.5 / 0.2 and the paper's A2 line for Nominal / ROM / Learned with 3–5 flights each; then freeze gains and network setting; FBL cells n like S0–S3).
- `git status` shows only your doc and asset changes; **do not commit**.

## 7. Status around the doc (for context only)
- Lab 2026-10-26: FBL flown on this machine; session ended with a power fault on the bottom drone (cf5); a new bottom drone is probable. Lab doc `docs/lab_sessions/2026-10-26.md`, state `docs/07_Thesis_Progress_Checklist.md` (next-action line) and `docs/next_steps_checklist.md` are updated locally, **not pushed**.
- Final FBL parameters: preset 1, inner loop 1, C_D 0.55, branch `study_FBL_stock`; network on/off decided after the A1 (dz 0.5 / 0.2) and A2-line tests (Nominal / ROM / Learned).
- The paper's validation (A2 straight line, dz 0.4 in distribution / 0.3 out of distribution for the authors' network trained at 0.4 only; ROM vs Nominal −24 % RMSE, Learned vs ROM −29 %) is **not flown yet** and must not appear as a result.
- Comparative study after that: protocol v1.0 with the image hash, supervisor's go, S0–S4.

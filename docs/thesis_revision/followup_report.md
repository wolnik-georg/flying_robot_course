# Closing pass — final report

Scope: Tasks 1–5 of the closing pass, working only in `docs/thesis_revision/`. `docs/thesis/*.tex`,
`docs/references.bib` and all code are untouched (code was read only). No DECISION row was decided. Nothing is
committed.

## Build

`revised/thesis_draft.tex` was built as delivered (a copy of `revised/` placed so that `\bibliography{../../references}`
resolves to `docs/references.bib`; pdflatex → bibtex → pdflatex ×2):
- **60 pages**; **0 errors**; **0 LaTeX warnings** (no undefined references or citations, no multiply defined labels);
  **0 BibTeX warnings**.
- 1 package warning: `todonotes`, marginparwidth < 2 cm. This comes from the draft preamble and is harmless.
- 11 overfull hboxes, most of a few points (the original draft: 8 overfull, 51 pages). Three exceed 10 pt:
  - Ch4 L453 (29 pt, "permutation-invariant" at a line end) and Ch2 L416 (16 pt, "Crazyflie~2.1" at a line end), both
    in original wording;
  - the Ch4 section title "Strategy 3: feedback linearisation with a flatness-preserving residual" (25 pt), which is
    the original title.

  These are left for the final document class.
- 26 underfull boxes (original: 16), mostly in the narrow table columns.

## What is closed

| Item | Result | File |
|---|---|---|
| [VERIFY] V3, V4, V6, V8 | settled from code and tool source; text fixed (`% CHANGED (V-x)`) | `followup_verify_closure.md` |
| V5 | moot (the numbers were removed under H1) | same |
| V7 (architecture half) | simulator network = flight network architecture, confirmed | same |
| Bibliography, 11 suspected errors | checked against the held PDFs and against the reference lists of held papers; corrected entries ready to paste | `followup_bib_corrections.md` |
| Re-confirmation of cited numbers, quotations and attributions in Ch1–Ch5 | all confirmed except the two items without a PDF (V1, V2) | same, last section |
| Diff audit (259 hunks) | every hunk justified; 6 statements stronger than their source fixed; 1 notation clash and 2 acronym issues fixed; 2 pieces of Ch1 prose that a Phase 4 marker had commented out restored; sentence-survival check over all 748 original sentences | `followup_diff_audit.md`, `followup_diff_audit_hunks.md` |
| DECISION rows as one-click choices | paste-ready text for every option at every place, with current line numbers and "also changes"; key snippets compile-tested | `author_decisions.md` |
| Examiner pass | 1 sign error fixed (Ch5 "closing the gap"); 5 weakest remaining passages and 6 student questions, each with a ready fix | `followup_examiner.md` |

## What is still open, and who must act

| Item | Who | What |
|---|---|---|
| **D-11 — which INDI controller Strategy 1 is** (new) | **author + supervisor** | The frozen study block "S1" selects controller 9 (Lee's controller with the measured force and moment residuals subtracted, 500 Hz, deck rotor speeds, its own vertical integral). Ch4 describes the controller_step INDI (ctrl_mode 3). With controller 9, D-01 is moot for Strategy 1, but `indi.a_res` is not computed on Strategy 1 flights, which weakens "one residual definition logged under every controller". Must be decided before D-01, D-02 and D-06. |
| D-01 residual sign | author (lab, if option c) | depends on D-11 |
| D-02 integral terms | author | depends on D-11 (controller 9 has `Kpos_Iz` and an attitude integral) |
| D-03 gap vs Hsieh et al. 2025 | **author + supervisor** | recommended (a) + (c) |
| D-04 collection rationale | author | recommended (a) |
| D-05 lab-status passages, Strategy 3 status | author (supervisor for Strategy 3) | per-place recommendations in the table |
| D-06 realised filter cut-off | author (lab, if option c) | fill ⟨f_c⟩, ⟨f_real⟩ from the filter design, not from a measurement |
| D-07, D-08, D-09, D-10 | author | D-10: paste the entries of `followup_bib_corrections.md` |
| V1 Lee CDC 2010 stability regions and numbering | author | obtain the CDC paper |
| V2 Abro et al. paragraph | author | obtain the full text, or take D-08 option 2 |
| V7 (weights half) | **supervisor** | confirm that `crazyflie_sim/backend/data/neuralswarm2/*.pth` (added by W. Hönig, commit `51a12dd`) are the Neural-Swarm2 paper's networks |
| V9 Busetto et al. | author | only if inertia values are to be stated |
| Examiner W1–W5, student questions | author | ready-to-paste fixes in `followup_examiner.md` (W1 also needs the motor-distance definition, `[VERIFY]`) |
| `\todo`s added in Ch5 (n, windows, Δ, effort measures, decision rule against the floor, transfer-penalty pairing) | author | fix before the campaign |
| Lab | **lab** | if D-01 (c) or D-06 (c) is chosen: change and read back the frozen parameter block before the first comparison flight |

## Merge order and the check after each merge

Before merging: decide D-11 first, because it changes Ch4 §4.3 most. The other decisions can be applied after merging,
since the text at every DECISION place is unchanged. In `docs/thesis/`, copy one `revised/chN_*.tex` over the original
at a time and run the check after each copy (`docs/thesis/chN_standalone.tex` builds one chapter; `thesis_draft.tex`
builds all):

| Step | Merge | Why this position | Check after merging |
|---|---|---|---|
| 1 | `ch3_system_modelling.tex` | defines the residual, sign and guards that Ch4/Ch5 refer to; adds no label that others need first | `pdflatex ch3_standalone` ×2 + `bibtex`: 0 errors, 0 undefined; full `thesis_draft` build: 0 undefined; `grep -c '% DECISION' ch3*.tex` = 7 |
| 2 | `ch4_control_architectures.tex` | after D-11 is decided; refers to Ch3 labels only | standalone + full build; check that eq. labels `ctrl-ades`, `ctrl-tau-geo`, `ctrl-indi-tau` resolve; `grep -n 'D-11' ch4*.tex` (the comments are gone once D-11 is applied) |
| 3 | `ch5_experimental_setup.tex` | defines the new `sec:setup-stats` that Ch6/Ch7 need; refers to Ch3/Ch4 labels | full build: 0 undefined; check Table 5.1 fits the page (a 5-column `p{}` table) |
| 4 | `ch2_related_work.tex` | independent of 1–3 | standalone + full build; check that the cite count is unchanged (20 keys) and the bibliography has no "?" |
| 5 | `ch1_introduction.tex` | after Ch2 (it refers to `sec:rw-existing-comparisons`) | full build; read §1.1 aloud (two restored sentences) |
| 6 | `ch6_results_2robot.tex`, `ch7_results_multirobot.tex` | need `sec:setup-stats` from step 3 | full build: 0 undefined; `grep -n 'ref{ch:results' ch6* ch7*` returns comments only |
| 7 | `ch8_discussion.tex`, `ch9_conclusion.tex` | refer to labels of Ch5–Ch7 | full build: 0 undefined; page count about 60 |
| 8 | `references.bib` (D-10) | last, after every chapter is in | `bibtex`: 0 warnings; check the printed years of NS2, Cobo-Briesewitz, Tal & Karaman and Bauersfeld |
| 9 | Clean-up | — | after all decisions: `grep -rn '% DECISION\|% VERIFY\|CLOSING PASS' docs/thesis/ch*.tex` returns nothing that is still open |

## Files written or changed in this pass

New: `followup_verify_closure.md`, `followup_bib_corrections.md`, `followup_diff_audit.md`,
`followup_diff_audit_hunks.md`, `followup_examiner.md`, `followup_report.md`.
Updated: `author_decisions.md` (paste-ready texts, D-11, VERIFY status), `revised/ch1`–`ch9` (V3/V4/V6/V7/V8 changes,
audit fixes, D-11 comments and notes, examiner sign fix, restored Ch1 prose).

# Thesis draft revision — STATUS (read this first)

**CLOSED 2026-10-15: the revised draft (rounds 1–3 + closing edits) is MERGED into `docs/thesis/ch1..ch9` and `docs/references.bib` (64 pages, 0 errors, 0 undefined). Revert with `git revert` of the merge commit if needed.** What is still open is listed in the last section ("Open after the merge"); everything above is history.

State as of 2026-10-14: **revision work finished, nothing merged, all decisions open.** The originals in `docs/thesis/`,
`docs/references.bib` and the code are untouched. The revised draft is in `revised/` (60 pages, 0 undefined, builds clean).
Chapters 6–9 stay skeletons until the study data exists.

## What to do, in this order
1. **D-11: decided 2026-10-14 by the author — Strategy 1 = Omar C + Iz 1.5 (controller 9), option (b); supervisor not yet consulted.** Next: apply (b) — rewrite Ch4 §4.3 "Implemented" and dependent sentences, describe `indi.a_res` logging with the controller-9 caveat; D-01 is moot for Strategy 1; re-read D-02 and D-06 for controller 9. Original question: Which INDI is Strategy 1: Omar C + Iz (frozen block `S1` = controller 9, as in
   `crazyswarm2/crazyflie/config/study_strategies.json`) or ours (`ours` block = controller 6, ctrl_mode 3, which Ch4 describes)?
   This is the project's open "Pure INDI" decision. It settles D-01, D-02 and D-06.
2. Go through `author_decisions.md` D-01 … D-10; every option has paste-ready text with line numbers in `revised/`.
   | ID | Topic | Who | Depends on |
   |---|---|---|---|
   | D-01 | residual sign wording (Ch3, Ch4, Ch5, Ch6) | author (lab if option c) | D-11 |
   | D-02 | integral terms in the equations vs code | author | D-11 |
   | D-03 | research-gap wording vs Hsieh et al. 2025 | author + supervisor | — |
   | D-04 | reason for collecting training data under the baseline only | author | — |
   | D-05 | lab-status passages, Strategy 3 status | author (+ supervisor for S3) | — |
   | D-06 | realised filter cut-off | author (lab if option c) | D-11 |
   | D-07 | H1 removals (applied by rule, just confirm) | author | — |
   | D-08 | Abro et al. paragraph (no PDF) | author | PDF |
   | D-09 | hypotheses, falsifiers, statistics (`\todo`s in Ch5) | author | — |
   | D-10 | 11 bib corrections, paste from `followup_bib_corrections.md` | author | — |
3. Items needing something you do not hold: V1 Lee CDC 2010 PDF, V2 Abro PDF, V9 Busetto (only if inertia values are stated),
   V7 supervisor confirms the NS2 weight files (`crazyflie_sim/backend/data/neuralswarm2/*.pth`).
4. Merge chapter by chapter in the order of `followup_report.md` (Ch3, Ch4, Ch5, Ch2, Ch1, Ch6–7, bib last), rebuild after each.
5. Examiner weak points W1–W5 and student questions: ready fixes in `followup_examiner.md`.

## Update 2026-10-14 night (after the revision)
The study moved to ONE Omar C controller tree (S0 geometric, S1 INDI, S2 geometric + NS2, S3 INDI + NS2, FBL = S4; `docs/plan_topics_2026-10-14.md`). Consequences for the draft, to apply only after the tree is validated on the drones: Ch4 describes Omar's controller (one law with an `indi` switch, shared position gains, z integral) for all strategies and the NS2 term entering as `a_d − a_nn`; the Ch4 table needs the four strategies (hybrid S3 now in scope, Ch1 scope/gap sentences change; FBL becomes S4); D-01 (residual sign) is moot because Omar's controller subtracts the residual; D-02 integrals = Omar's `Kpos_Iz`; D-06 filters = Omar's 30 Hz; the logging statement ("one residual definition under every controller") needs the caveat that `indi.a_res` is not computed on the tree and the residual is `a_imu_f − a_rpm_f` (reconstructed offline for indi 0). No flown results enter the draft.

## Round 2 (prepared 2026-10-15)
Targeted update of the revised draft for the final study controllers (one controller with three switches: S0 geometric, S1 INDI, S2 geometric + NS2, S3 INDI + NS2, S4 FBL; no implementation names, no verification text, no results; edit now, three places `% CONFIRM-AFTER-LAB`). Prompt: `docs/thesis_revision_prompt_round2.md` (paste between its markers). Output goes to `docs/thesis_revision/round2/`; D-03 and D-05 stay open.

## Round 2 result (2026-10-15)
Done and validated (62 pages, 0 undefined, no forbidden names in visible text, 20 bib keys all present, originals untouched). Outputs `round2/`. New decisions: **D-12** (how the measured and predicted terms combine in S3: they add; the prediction becomes an offset of the INDI acceleration reference; options keep / partition / drop) and **D-13** (protocol flies two vehicles; null scenarios C1–C3, RQ3, contribution 2 and Ch7 assume three). D-03, D-05, [VERIFY] V-items (inertia from a Crazyflie 2.0 identification, mass 42.7 g vs 41 g) and three `% CONFIRM-AFTER-LAB` places remain.

## Decisions 2026-10-15 (after round 2)
- **D-13 = option (b):** the study is two phases: phase 1 two vehicles (A scenarios, 48 cells), phase 2 three vehicles (B/C scenarios incl. the C1–C3 floor, RQ3 transfer). The draft keeps the three-vehicle content; only status wording changes (six `% DECISION D-13` places). The two-vehicle results are first read against a provisional floor from the baseline repeats.
- **D-12 default = (a): no code change;** the text already states the plain-sum behaviour. Revisit after S0–S2 data (only S3 would need re-flying).

## Round 3 result (2026-10-15)
Done and validated: 63 pages, 0 errors, 0 undefined, naming sweep 0, bib keys present, originals untouched, every changed sentence is an intended edit (34, all phase/floor/hybrid wording). One minor fix by us: Table 5.1 caption says A6, C4 and C5 are defined but not part of the comparison. Open: D-14 (provisional floor: repeat-to-repeat variation within a cell, alternative = variation of the S0 anchor flights), D-03, D-05, [VERIFY] items, three `% CONFIRM-AFTER-LAB` places. Files: `round3/`. **Ready to merge into `docs/thesis/` chapter by chapter (one deliberate step, reversible with git); bib corrections (`followup_bib_corrections.md`) are separate.**

## Files
`author_decisions.md` (decisions + V items), `followup_report.md` (open items, merge order), `followup_verify_closure.md`,
`followup_bib_corrections.md`, `followup_diff_audit*.md`, `followup_examiner.md`, `revised/` (chapters + `thesis_draft.tex`),
`stage*.md` (phase outputs, `stage6_final_report.md`), prompts: `docs/thesis_revision_prompt_full.md`, `docs/thesis_revision_followup_prompt.md`.

## Rules that still hold
No flown / lab / simulation results in Ch1–5. Never edit `docs/thesis/*.tex` or `references.bib` until you merge deliberately.
Proposed references in `proposed_references.md` are UNVERIFIED — read the source before adding.

## Open after the merge (2026-10-15)
- **Decided and applied in the closing edits:** D-14 (provisional floor = repeat-to-repeat variation within a cell), D-03 (narrowed gap with the hybrid as added terms, Hsieh et al. 2025 named as the closest study), D-05 (conservative status wording; the campaign-status paragraph of the retired controllers deleted), D-08 (Abro paragraph dropped), D-10 (11 bib corrections in `docs/references.bib`), D-12 = (a), D-13 = (b). Script and log: `docs/thesis_revision/final/apply_closing_edits.py`, `closing_edits_log.txt`.
- **Stays open on purpose:** `% VERIFY` V1 (Lee CDC 2010 numbering, PDF not held), V7 (simulator weights, supervisor), V10 (inertia from a Crazyflie 2.0 identification), V11 (RPM source of the offline labels, default deck), V12 (controller mass 42.7 g vs about 41 g); three `% CONFIRM-AFTER-LAB` places (INDI takeoff handling, training-flight counts, NS2 execution time); D-09 (hypotheses/statistics `\todo`s, fixed after phase 1); the phase-2 protocol; round-2 examiner items E3-E5 (gain provenance, inertia and mass, onboard vs offline residual agreement).
- **Supervisor to confirm:** Pure INDI = one tree (D-11), D-12 hybrid as built, D-13 two phases, D-14 provisional floor.

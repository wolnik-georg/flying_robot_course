# Thesis draft revision — STATUS (read this first)

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

## Files
`author_decisions.md` (decisions + V items), `followup_report.md` (open items, merge order), `followup_verify_closure.md`,
`followup_bib_corrections.md`, `followup_diff_audit*.md`, `followup_examiner.md`, `revised/` (chapters + `thesis_draft.tex`),
`stage*.md` (phase outputs, `stage6_final_report.md`), prompts: `docs/thesis_revision_prompt_full.md`, `docs/thesis_revision_followup_prompt.md`.

## Rules that still hold
No flown / lab / simulation results in Ch1–5. Never edit `docs/thesis/*.tex` or `references.bib` until you merge deliberately.
Proposed references in `proposed_references.md` are UNVERIFIED — read the source before adding.

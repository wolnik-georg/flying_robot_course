# Thesis revision, round 3 — apply the decisions D-12 and D-13 (short, targeted)

Created 2026-10-15. Paste everything between the markers into the model, in the repository, as ONE message. Round 2 output stays as it is; this pass reads `docs/thesis_revision/round2/revised/` and writes a new copy.

===== BEGIN ROUND-3 PROMPT =====

# ROLE
Same combined professor / supervisor / student-reader as in rounds 1 and 2. Read the ROLES, RULES H1–H9 and CITATION DISCIPLINE of `docs/thesis_revision_prompt_full.md`, and the "WHAT CHANGED" and "HARD RULES" of `docs/thesis_revision_prompt_round2.md`; they all still apply (no results of any kind; no implementation, person, language, file or slot names in visible text; controllers cited only through their papers; no verification wording; never edit `docs/thesis/` or `docs/references.bib`; no commits; no invented numbers). Work in one continuous run. APPLY = settled by the decisions below, the code or a held PDF; anything else is a DECISION (leave the text, `% DECISION` comment, list in the open-items file).

# BASE AND OUTPUT
Base: `docs/thesis_revision/round2/revised/` (all chapters, `thesis_draft.tex`, `preamble_draft.tex`). Output: `docs/thesis_revision/round3/revised/` (full copy with the edits), plus `round3_change_log.md` (every hunk: file, line, old → new, which decision), `round3_open_items.md`, `round3_report.md` (summary, build result, what to read first).

# DECISIONS TO APPLY (author, 2026-10-15)
**D-13 = option (b): the study has two phases.** Phase 1: two vehicles — the two-vehicle scenarios (library "A" scenarios), 12 settings × 4 strategies = 48 cells, 5 clean flights per cell = 240 flights (numbers as in round 2). Phase 2: three vehicles — the three-vehicle scenarios (library "B" and "C" scenarios, including the null scenarios C1–C3 that set the measurement floor) and the transfer of the learned model from two to three vehicles (RQ3), flown with the same controller after phase 1. Therefore:
1. Keep all three-vehicle content (null scenarios, RQ3, contribution 2, Chapter 7, the scope table, the 16-scenario library table). Do NOT remove or weaken any research question.
2. Replace the six `% DECISION D-13` places (round-2 line numbers: Chapter 1 L205 and L300; Chapter 5 L181, L412, L601; Chapter 7 L30; find them with grep) and every other sentence that implies the three-vehicle scenarios are flown in the same campaign or already flown, by a conservative phase wording: they are flown in a second phase, after the two-vehicle campaign, with the same controllers; the protocol of phase 2 (cells, counts) is fixed when phase 1 is complete — state NO phase-2 counts.
3. Where the text says the two-vehicle results are interpreted against the measurement floor of the null scenarios (Chapter 3 assumption table, Chapter 5 §null scenarios, §statistical treatment, §threats to validity, Chapter 6 "negative answer" paragraphs): state the order — the floor is known only after phase 2; the two-vehicle results are provisionally read against the repeat-to-repeat variation within a cell (the same strategy, scenario and setting), and finally against the null-scenario floor. Mark this paragraph `% DECISION D-14 (new): provisional floor from repeat-to-repeat variation; author to confirm`.
4. Chapters 6–9 skeleton status texts and the summary/outline paragraphs of Chapters 1 and 5: state the two phases consistently (Chapter 6 = phase 1 results, Chapter 7 = phase 2 results).

**D-12 = option (a): the hybrid is flown as built — the measured term and the predicted term are added.**
1. In Chapter 4 §Strategy 3, remove the `% DECISION D-12` comment and keep the derivation (the measured residual is formed from measured rotor speeds and is unchanged when the prediction raises the thrust; the terms add; the prediction acts as an offset of the INDI acceleration reference). State it as the definition of Strategy 3 in this thesis, not as a defect, and keep the deviation sentence relative to the published hybrid (which lets the incremental term handle only the remaining mismatch).
2. Add one sentence of scope in the limitations of the fairness section ("What this does not guarantee"): the variant in which the measured term handles only the remaining mismatch is not part of this comparison. No result or expectation about which variant is better.
3. Check that every other mention of the hybrid (Chapter 1 list and contributions, Chapter 2 hybrid section, Chapter 5 hypotheses/metrics, Chapter 6 skeleton) is consistent with "added, not partitioned".

# ALSO CHECK (read-only unless clearly stale)
- The counts 48 / 240 and the strategy numbering S0–S4 everywhere (`grep`), including captions and tables.
- That no visible text names implementations, people, languages or files (grep visible text with comments stripped; `our` as the ordinary pronoun is fine).
- That no sentence was lost behind a mid-line `%` marker (compare the visible sentences of round 2 and round 3: every round-2 sentence must either print or be listed in the change log as replaced).
- Build `docs/thesis_revision/round3/revised/thesis_draft.tex` with latexmk (bib `docs/references.bib`): 0 errors, 0 undefined references/citations; report the page count.

# OUTPUT CHECKLIST
`round3_open_items.md` must list: D-14 (provisional floor), D-03 (gap wording, still open — do not decide), D-05 (campaign-status passages, still open), the `[VERIFY]` and `% CONFIRM-AFTER-LAB` items carried over from round 2 (unchanged), and anything you found stale that you did not change.

===== END ROUND-3 PROMPT =====

# Follow-up prompt for the thesis revision (closing pass)

Paste everything between the markers into the same model, in the same repository. Run it as one continuous pass.

===== BEGIN FOLLOW-UP PROMPT =====

You finished the seven-phase revision. This is a closing pass. Goal: the author should only have to make decisions and merge, nothing else.
Same rules as before (H1–H9, no flown/lab/simulation results, authority order code > held PDFs > draft, 100 % citation correctness,
never edit `docs/thesis/*.tex` or `docs/references.bib`). Work only in `docs/thesis_revision/`. Do not decide any DECISION row.

## Task 1 — Close the [VERIFY] items
For each `% VERIFY` marker in `revised/ch*.tex` and each V1–V9 in `author_decisions.md`:
1. Try to settle it from the code (`lib.rs`, `residual_nn.rs`, `traj_iface.c`, the yaml/patches) or a held PDF. Quote file:line or PDF page.
2. If settled: fix the text in `revised/` (mark `% CHANGED (V-x)`), and mark the item CLOSED with evidence.
3. If not settleable from local material: leave the text, state exactly what the author must check and where (one sentence).
Output: `followup_verify_closure.md` (table: item, line, status CLOSED/OPEN, evidence, action).

## Task 2 — Check the suspected bib errors against the held PDFs
For the 11 rows in `proposed_references.md` §B: open the held PDF where one exists and report the actual venue, year, volume, pages, and
whether the cited content (equations, numbers, statements the thesis uses) is in that version. For each give the exact corrected
`.bib` entry text as a proposal (do not edit the bib). Output: `followup_bib_corrections.md`.
Also: for every statement in `revised/ch1–ch5` that carries a number, equation reference or attribution to a paper, re-open the source
and confirm it once more; list any you cannot confirm.

## Task 3 — Independent audit of your own APPLY changes
Diff each `revised/ch*.tex` against `docs/thesis/ch*.tex`. For every changed hunk list: ID, what changed, which rule/source justifies it.
Flag (a) any hunk without a justification, (b) any statement that is now stronger than its source supports, (c) any place where a
removed project finding is still implied, (d) any new unexplained symbol, acronym or forward reference.
Output: `followup_diff_audit.md`, with a verdict per chapter (ready to merge / needs fix) and fix the flagged items you can.

## Task 4 — Make every DECISION a one-click choice
For each D-01…D-10: write the exact replacement text for every option at every listed place (short, ready to paste, with line
numbers in `revised/`), and state in one sentence what else changes if that option is picked (other chapters, tables, equations).
Do not apply any option. Output: update `author_decisions.md` in place (add an "Ready-to-paste text" subsection per decision).

## Task 5 — Last examiner pass
Re-read Chapters 1–5 as an examiner who has not seen the project. List the five weakest passages that remain (location, why, concrete
fix). Then the three questions a student would still ask after Chapter 3 and Chapter 4. Output: `followup_examiner.md`.

## Final report
Write `followup_report.md`: what is closed, what is still open and who must act (author / supervisor / lab), and a merge order
(chapter by chapter) with the check to run after each merge. Rebuild `revised/thesis_draft.tex` and report page count and warnings.
Do not commit.

===== END FOLLOW-UP PROMPT =====

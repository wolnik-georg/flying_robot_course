# Round 3 — report

Base: `round2/revised/`. Output: `round3/revised/` (all chapters, `thesis_draft.tex`, preamble), plus
`round3_change_log.md` (54 hunks with the decision for each, and the list of round-2 sentences that were replaced) and
`round3_open_items.md`. `docs/thesis/`, `docs/references.bib` and the code are unchanged; nothing is committed.

## What to read first

1. **D-14 (new): the provisional floor.** The null scenarios are flown in phase 2, so every two-vehicle result is first
   read against the repeat-to-repeat variation within a cell and later against the null-scenario floor. That is my
   wording of your instruction, marked at six places (plus the two threats-table rows); please confirm or replace it (`round3_open_items.md` §A, which
   also offers the session anchor flights as an alternative provisional floor).
2. **Stale but not changed:** A6, C4 and C5 are still listed in the library table, although the protocol does not fly
   them. A6 is an "A" scenario that phase 1 excludes (`round3_open_items.md` §D).

## What changed

**D-13 = (b), two phases.**
- **Chapter 5** introduces the two phases:
  - Phase 1 is the two-vehicle campaign, with tier 1 = 12 settings × 4 strategies = 48 cells, 5 clean flights per cell
    = 240.
  - Phase 2 flies the three-vehicle scenarios, including C1–C3, and RQ3, with the same controllers, after phase 1. Its
    cells and counts are fixed when phase 1 is complete; no phase-2 counts are given.
  - The changes are in the introduction, the platform paragraph, §null scenarios, §comparison campaign, the Table 5.3
    caption, §statistical treatment, the threats table and the summary.
- **Chapter 1:** RQ3 is answered in phase 2; the "each question is answerable" paragraph; contributions 1 and 2; the
  scope-table comment; and the outline (two-robot results chapter = phase 1, multi-robot results chapter = phase 2).
- **Chapter 3:** the assumptions table.
- **Chapters 6–9:**
  - Ch6 = phase 1, with the provisional floor in its statistic slot and its four negative answers.
  - Ch7 = phase 2, with a new section "Null scenarios and the measurement floor", and the fallback now also keeps the
    provisional floor.
  - Ch8: the discussion of the floor.
  - Ch9: the summary and the RQ1 slot; the future-work bullet "Three-robot scaling" became "Larger teams".
- No research question was removed or weakened, and all three-vehicle content is kept.

**D-12 = (a), the hybrid as built.**
- **Chapter 4 §Strategy 3:** the `% DECISION D-12` comment is removed. The law now says the two terms are "added, not
  partitioned" and that "this is the definition of Strategy 3 in this thesis". The derivation and the deviation from
  the published hybrid are kept.
- **Fairness limitations:** one scope sentence says the variant in which the measured term handles only the remaining
  mismatch, as in Cobo-Briesewitz et al., is not part of this comparison. No expectation about which variant is better.
- **Consistency:** Ch1's strategy list says "added together", and Ch2's two "this thesis flies/evaluates such a hybrid"
  sentences add "with the measured and the predicted term added". Ch6 §Hybrid already matched.

**Checks.**
- **Counts and numbering:** the only counts in the text are 48 and 240 (Ch5 campaign and summary). Strategy numbering
  S0–S4 is consistent in every chapter, table and caption.
- **Naming sweep:** 0 hits in the visible text for implementation, person, language, file or slot names, code
  identifiers, internal labels or verification wording (comments stripped).
- **Lost sentences:** every round-2 sentence prints in round 3 or is listed in the change log with its replacement.
  The check first found **4 sentences swallowed by mid-line `%` markers** in my own edits (Ch1 outline, Ch4 fairness,
  Ch5 introduction, Ch9 summary). All four are fixed, and the re-run shows 0 sentences surviving only inside comments.

## Build

`latexmk -pdf` on `round3/revised/thesis_draft.tex` (bib `docs/references.bib`):
- **exit 0, 63 pages**, 0 errors, 0 LaTeX warnings (0 undefined references or citations, 0 multiply defined labels),
  0 BibTeX warnings.
- 1 package warning (`todonotes` marginparwidth, from the preamble).
- 11 overfull hboxes. The two above 10 pt are original wording, as in round 2: the Strategy 4 section title (25 pt) and
  Ch2 (16 pt).
- 28 underfull hboxes.

## Still open
D-14 (new, confirm), D-03 and D-05 (not decided), D-07–D-10, the `[VERIFY]` items V1, V2, V7, V10–V12, and the three
`% CONFIRM-AFTER-LAB` places, all unchanged from round 2; details and line numbers in `round3_open_items.md`.

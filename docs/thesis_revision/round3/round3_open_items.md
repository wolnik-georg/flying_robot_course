# Round 3 — open items

Line numbers refer to `round3/revised/`. Applied in this round: **D-13 = (b)** (two phases) and **D-12 = (a)** (the
hybrid is flown as built, with the measured and the predicted term added). Their `% DECISION` comments are removed.

## A. Decisions

### D-14 (new) — Provisional measurement floor in phase 1  *(author to confirm)*

**What the text now says.** The null scenarios C1–C3 are three-vehicle scenarios flown in phase 2, so the measurement
floor is known only after the two-vehicle campaign. The two-vehicle results are read in two steps:
1. provisionally, against the repeat-to-repeat variation within a cell (same strategy, scenario and setting);
2. finally, against the null-scenario floor.

In phase 1, the hardware-gate flights at large separation give the first check for model error.

**Places** (each carries `% DECISION D-14 (new): provisional floor from repeat-to-repeat variation; author to confirm`):
- Ch3 L554 (assumptions table, two rows);
- Ch5 L206 (§null scenarios) and L528 (§statistical treatment); the threats table rows "Measurement floor" and "Model
  error read as interaction";
- Ch6 L35 (chapter-level paragraph; the four "negative answer" paragraphs say "provisional floor");
- Ch7 L54 (new section "Null scenarios and the measurement floor", where the two-vehicle results are re-read);
- Ch8 L124 (discussion: whether the provisional and the final reading agree).

**Questions for the author.**
- Is the repeat-to-repeat variation within a cell the right provisional floor? An alternative is the variation of the
  baseline anchor flights at the start and end of each session, which the protocol already flies.
- Is the hardware-gate check at large separation enough to catch model error in phase 1? The sentence in Ch5 §null
  scenarios says it is the *first* check.
- The decision rule against either floor is still a `\todo` in Ch5 §statistical treatment (with D-09).

### D-03 — Gap wording (still open; not decided)
Text unchanged at Ch1 L103, L118, L235, L280 and Ch2 L634, L686, L717, L755. The proposal in
`../round2/round2_open_items.md` (D-03) still applies. With the hybrid as built (D-12 = (a)), "their combination" in
the proposed sentence should read "their combination, with the measured and the predicted term added".

### D-05 — Campaign-status passages (still open)
- Ch4 L47 (Table 4.1 "Implemented" column) and L530 (Strategy 4 status).
- Ch5 L42 (campaign-status paragraph of the retired controllers; nothing has been flown with the final controllers)
  and L316 ("to be confirmed … on the first two-vehicle flight").
- Ch6 L48 (§6.1, now with "C1–C3 … flown in the second phase") and L239 (Strategy 4 row).
- Ch7 L37 ("third brushless vehicle in the roster").
- Ch8 L173.
- Ch9 L77 and L81 (future-work list). Its "Three-robot scaling" bullet was changed to "Larger teams" under D-13, since
  the three-vehicle scenarios are now phase 2 of this thesis.

### D-07 – D-10 (carried over, unchanged)
- D-07: H1 removals, confirm.
- D-08: Abro paragraph.
- D-09: hypotheses and statistics, Ch5 L518.
- D-10: bibliography corrections (`../followup_bib_corrections.md`).

## B. `[VERIFY]` items (carried over from round 2, unchanged)

| # | Place | What |
|---|---|---|
| V1 | Ch2 L107; Ch4 L313 | Lee et al. CDC 2010 stability regions and numbering |
| V2 | Ch2 L673 | Abro et al. paragraph |
| V7 | Ch5 L549 | simulator interaction-model weights (supervisor) |
| V10 | Ch4 L301 | controller inertia identified for the Crazyflie 2.0 |
| V11 | Ch3 L179; Ch4 L448 | rotor-speed source of the offline labels |
| V12 | Ch4 Table 4.2 vs Ch5 §5.1 | controller mass parameter 42.7 g vs vehicle mass about 41 g |
| (radio) | Ch5 L266 | vendor radio specification, only if the removed numbers are restored |

## C. `% CONFIRM-AFTER-LAB` places (carried over, unchanged)

| Tag | Places |
|---|---|
| (a) take-off and landing of the INDI strategies | Ch4 L249; Ch5 L341 |
| (b) training-flight counts, retrained model | Ch4 L442; Ch5 L373 |
| (c) execution time of the learned model | Ch4 L423; Ch5 L350 |

## D. Found stale, not changed

| Place | Why stale | Why not changed |
|---|---|---|
| Ch5 Table 5.1 and §5.2 (flight volume): A6 and C4 listed as gated scenarios, C5 with the 0.10 m floor | The protocol excludes A6 and C4 ("dropped after repeated crashes") and the single-vehicle C5. Under D-13 = (b), C5 and C4 belong to the "C" set of phase 2, but A6 is an "A" scenario that phase 1 does not fly | D-13 keeps the 16-scenario library table. Whether A6/C4/C5 are listed as "defined, not flown" is an author decision (suggest a table footnote) |
| Ch5 §5.7 last paragraph and Ch3 assumption table: "battery discharge changes the thrust constant … bounded by the null scenarios" | Mechanism doubtful with thrust from measured rotor speeds (round-1 examiner W4); "bounded by the null scenarios" now also waits for phase 2 | not part of D-13/D-14 wording; round-1 fix W4 is still available in `../followup_examiner.md` |
| Ch7 §7.4 heading "If the third vehicle arrives too late" | It is now the contingency for phase 2 and reads correctly, but it is project-status wording | D-05 family |
| Protocol §3 body (outside the thesis) | Still says 12 × 3 = 36 cells; the banner and the thesis say 48 / 240 | outside the thesis; update when the protocol is re-frozen |
| Round-2 examiner items E3–E5 (`../round2/round2_report.md`) | gain provenance; inertia and mass; onboard vs offline residual agreement | still open; need author input |
| Ch5 §5.5.4 transfer penalty `\todo` (pairing of two- and three-vehicle scenarios) | belongs to the phase-2 protocol, which is fixed after phase 1 | consistent with "no phase-2 counts"; left as a `\todo` |

# Round 2 — report

Targeted update of the round-1 draft (`../revised/`) for the final study controllers. Output: `round2/revised/` (all
nine chapters, `thesis_draft.tex`, preamble). `docs/thesis/`, `docs/references.bib` and all code are unchanged. Nothing
is committed.

## What a human must read first

1. **D-12 — the hybrid (Strategy 3) double-counts the disturbance as implemented** (`round2_open_items.md` §A). The
   prompt asked Ch4 to say that the measured residual "falls as the prediction takes effect". In this controller it does
   not. The measured residual is formed from measured rotor speeds, so it is independent of what the controller
   commands, and the measured and the predicted term **add**. In increment form the prediction becomes an offset of the
   INDI loop's acceleration reference, which the shared vertical integral removes at steady state. Ch4 §Strategy 3
   states the code's behaviour with its derivation and a `% DECISION D-12` comment. Decide with the supervisor before
   the validation flights: fly as is, or change the controller so the measured term handles only the remaining
   mismatch, as NA-INDI does. A one-line formula is in the open-items file.
2. **D-13 — the protocol flies two vehicles; the thesis still depends on three.** The null scenarios C1–C3 (the
   measurement floor), RQ3, contribution 2, Ch7 and the scope table all need three-vehicle flights, which the
   two-vehicle protocol excludes. Flagged at six places.
3. **The inertia in the controller is that of a Crazyflie 2.0 identification** (code comment), not of the brushless
   vehicle; the controller mass is 42.7 g against a stated vehicle mass of about 41 g. Ch4 states the inertia as a
   deviation; V10 and V12.
4. **The three `% CONFIRM-AFTER-LAB` places** (take-off/landing of S1/S3, training-flight counts, execution time of the
   learned model): Ch4 L248, L422, L441; Ch5 L327, L336, L359.

## Summary of the changes

- **Chapter 4, rewritten.**
  - One common controller: the position loop with a vertical-only integral (eq. ctrl-ad); switch terms in eq.
    (ctrl-ades); the desired attitude from the direction of $a_{cmd}$ and the heading; Lee et al.'s full torque law with
    an attitude integral (eq. ctrl-tau-geo).
  - Table 4.2 with every shared parameter, from the code. Rates: 500 Hz controller in a 1 kHz loop, 100 Hz
    motion-capture pose and learned model.
  - Three run-time switches; the upper vehicle always flies S0.
  - S1 as the additive measured force and moment terms. These correspond to Cobo-Briesewitz et al. eqs. (8a), (8b),
    (10a), (10b), and the increment form (Tal & Karaman) is derived from them, keeping the old equation labels.
  - S2 with dedicated training flights. The new S3, the hybrid, with its derivation. S4 (FBL) renumbered, reference
    design only.
  - Fairness rewritten for one controller, with four honest limitations. The retired implementation is gone: no
    velocity-error conditioning, no incremental law with INDI gains, no DShot or plausibility filter, no configurable
    sign.
- **Chapter 3.**
  - One thrust constant, rotor speeds in rad/s from the optical sensor, and identical 30 Hz filters.
  - The residual computed onboard with switch A and offline otherwise, with one definition. The measured moment term is
    in scope.
  - The neighbour-velocity rules; evaluation every tenth main-loop tick. Sign, integral and filter decisions resolved.
  - No file names in footnotes.
- **Chapter 5.**
  - Optical rotor-speed sensor; rates; the recorded list for the final controller ($e_R$ reconstructed from the actual
    and desired attitude).
  - Hardware gate with shared gains and the take-off and execution-time CONFIRM places.
  - Dedicated training flights; the campaign design from the protocol: tier 1 = 12 settings (new Table 5.3) × 4
    strategies = **48 cells, 5 clean flights per cell = 240**, at most 8 attempts, blocks rotated between sessions,
    baseline flights at the start and end.
  - The simulation section without verification text; threats and summary updated.
- **Chapters 1 and 2.**
  - Five strategies by function; the hybrid in scope everywhere it was said to be out of scope; RQ3's applicability.
  - Contribution 1 and the fairness wording for a shared gain set; no verification text in the apparatus paragraph.
  - Ch2: three scope sentences and the Gu-fairness paragraph.
  - The D-03 gap statements themselves are unchanged; the proposal is rewritten in the open items.
- **Chapters 6–9.**
  - Renumbering.
  - A new §Hybrid strategy in Ch6 (the label `sec:res2-hybrid` is reused), and §6.5 reduced to the direction of the
    effect plus attribution by switch.
  - Shared gains in Ch8; S4 references.
  - Names removed from the D-05 passages, whose content is unchanged.
- **Naming sweep.** Zero occurrences of implementation names, people, languages, file names, code identifiers,
  vehicle nicknames, internal labels or verification wording in the visible text: a grep over all chapters, footnotes,
  captions, table cells and `\todo`s returns 0 hits. Code paths remain in comments only.
- **Round-1 defects found and fixed.** In Ch1 the words "It is described in" had been swallowed by a marker comment.
  The same pattern occurred three times in my own round-2 edits, was caught by a sentence-survival check against the
  round-1 text, and is fixed.

Details: `round2_audit.md` (what was stale and why), `round2_change_log.md` (166 hunks with justification, plus a
section-by-section table for Ch4), `round2_open_items.md`, `round2_proposed_references.md` (no new bib keys; three
UNVERIFIED candidates).

## Build

`latexmk -pdf` on `round2/revised/thesis_draft.tex`, with `\bibliography{../../../references}` = `docs/references.bib`:
- **exit 0, 62 pages, 0 errors, 0 LaTeX warnings (0 undefined references or citations, 0 multiply defined labels),
  0 BibTeX warnings.**
- 1 package warning: `todonotes` marginparwidth, from the draft preamble.
- 11 overfull hboxes. Two exceed 10 pt, both in original wording: the Strategy 4 section title (25 pt) and Ch2 L417
  (16 pt). Table 4.1 was narrowed in this round.
- 26 underfull hboxes, mostly in narrow table columns.

One build issue was fixed: `\cubed` is not provided by the preamble's siunitx fallback, so the integral gain's unit is
written as $\mathrm{s}^{-3}$.

## Consistency sweep (Phase-5 style)

- **Strategy names and numbers** are consistent in every chapter, table and caption: S0 baseline, S1 INDI, S2 geometric
  + NN, S3 INDI + NN, S4 FBL + NN (checked by grep, every occurrence read).
- **Rates** are the same wherever stated: 1 kHz main loop, 500 Hz controller, 100 Hz motion capture, 100 Hz learned
  model, 500 Hz log. The filters are 30 Hz second-order Butterworth everywhere.
- **Gains, mass, inertia and thrust constant** appear only in Table 4.2, from the code.
- **Counts** of 48 cells and 240 clean flights match the protocol banner and the shakedown-card footer. The protocol §3
  body still says 36, pre-banner (F8).
- **Equation labels:** all `\ref`/`\eqref` resolve. Old labels are kept with true content. New labels: `eq:ctrl-ad`,
  `eq:ctrl-s1-torque`, `eq:ctrl-s3-increment`, `tab:ctrl-params`, `tab:setup-tier1`, `sec:ctrl-s4`.
- **Notation:**
  - $e_p = p_{des} - p$ is used throughout.
  - $\hat\cdot$ is the hat map on angular velocities and denotes a prediction on accelerations, as Ch3 explains.
  - $l$ is the arm length in Table 4.2; in Ch2 $l$ is Bauersfeld's motor distance, local to that paragraph.
  - $\kappa$ (drag-to-thrust) is new and defined in the table.
- **Acronyms:** FBL and NN are defined in Ch1 and in the Table 4.1 caption.

## Examiner / student read (Chapters 1, 3, 4, 5, after the change)

### Five weakest passages

| # | Location | Why weak | Fix |
|---|---|---|---|
| E1 | Ch4 §Strategy 3 | The hybrid as flown adds two estimates of one disturbance. An examiner will call it "INDI with a reference offset", not a hybrid in the NA-INDI sense. | D-12: decide (a) or (b) before flying. With (b), replace the second paragraph by the derivation for $a_{cmd} = a_d - a_{res,f} - (\hat a - \hat a_f)$. |
| E2 | Ch5 §5.3.1, Table 5.2, §5.5.5 | The measurement floor rests on three-vehicle null scenarios that the two-vehicle protocol does not fly. | D-13 (a): define a two-vehicle null condition (e.g. the baseline anchor flights, or a lateral offset beyond the network gate) and rewrite §5.3.1. |
| E3 | Ch4 Table 4.2, §fairness | The shared gains carry no provenance. The reader learns they are shared, not where they come from or for which switch combination they were chosen; Ch4 "what this does not guarantee" admits the effect but cannot size it. | Add one sentence on how the gain set was obtained and that it was not re-tuned per switch combination (the author must supply the provenance; `[VERIFY]`). |
| E4 | Ch4 §S0 deviations, Table 4.2 | A Crazyflie 2.0 inertia and a 42.7 g mass parameter on a ≈41 g brushless vehicle. Both are model errors that enter the measured residual (Ch3 "It absorbs model error"). | Either identify the brushless vehicle (V10, V12) or state in Ch5 that the null condition absorbs this, and why that is enough. |
| E5 | Ch3 §3.3 / Ch5 §5.4.2 "one definition, computed onboard or offline" | The onboard residual uses the estimator's rotated accelerometer reading at the controller's step; the offline one is rebuilt from logged channels. They are the same definition, but nothing in the design shows they agree numerically. | Add a protocol step to Ch5: on flights with switch A, compare the onboard residual with the offline reconstruction of the same flight and report the difference. This is a check of the measurement chain, not of a method (V11 for the rotor-speed source). |

### Questions a student would still ask

**About the unified controller**
1. "If Strategies 0–3 share every gain, were those gains chosen for the geometric baseline or for INDI, and does that
   favour one of them?" Partly answered in Ch4 §fairness (limitation 1); provenance missing (E3).
2. "What happens at the moment switches A and B are turned on in flight?" The residual filters run only while the
   switches are set, so they start from their last state. The text says only that take-off and landing are fixed at
   the hardware gate (CONFIRM-AFTER-LAB (a)). Fix: one sentence on the filter warm-up once (a) is settled.
3. "Is the additive moment correction really INDI, or a disturbance observer?" Answered in Ch4 §S1: eq.
   (ctrl-indi-tau) shows it is the angular-acceleration increment with $J\alpha_{ref} = \tau_{geo}$.
4. "Why is the upper vehicle always the baseline? It feels the lower vehicle and the ground too." Ch4 states the fact,
   not the reason. Fix: one sentence. The reason is to keep the disturbance source identical across strategies; that is
   a design rationale and needs the author's confirmation.

**About the hybrid**
5. "If the prediction is perfect, why does Strategy 3 not simply cancel the disturbance twice and push the vehicle
   up?" It does, until the vertical integral removes the offset; Ch4 §S3 derives this. Students will need the
   one-line steady-state argument spelled out with numbers, as Ch3 does for the sign. Fix: a two-line illustrative
   example in Ch4 §S3 (no results; invented values, labelled as such).
6. "Where, then, can the hybrid be better than INDI alone?" Ch4 §S3: where the prediction leads the filtered
   measurement, in transients. Ch6 §6.6 has the slot that tests this (A8 vs A1).

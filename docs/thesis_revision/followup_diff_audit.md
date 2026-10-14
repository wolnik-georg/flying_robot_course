# Follow-up 3 — Independent audit of the APPLY changes

Method: every `revised/chN_*.tex` was diffed against `docs/thesis/chN_*.tex` (259 hunks). Each hunk was matched to the
nearest marker comment (`% NEW`, `% CHANGED`, `% DECISION`, `% REMOVED`, `% VERIFY`); the per-hunk inventory (ID, line
range, change, justification) is in **`followup_diff_audit_hunks.md`**. Hunks with no marker nearby were read by hand
(§2). Every new or changed prior-work sentence was then re-read against its source for (b) overstatement; the chapters
were scanned for (c) implied project findings and (d) new symbols, acronyms and forward references. Items fixed in this
pass are marked **fixed**.

## 1. Summary

| Chapter | Hunks | Without marker | (a) unjustified | (b) stronger than source | (c) implied finding | (d) symbol/acronym/forward ref | Verdict |
|---|---|---|---|---|---|---|---|
| Ch1 | 20 | 0 | 0 | 0 | 0 (1 checked, §4); 2 lines of prose lost to markers → **fixed** (§6b) | 0 | **ready to merge** (D-03/D-05 comments stay until decided) |
| Ch2 | 50 | 1 | 0 | 2 → **fixed** | 0 | 2 → **fixed** | **ready to merge** |
| Ch3 | 47 | 0 (4 → markers added) | 0 | 0 | 0 | 0 | **ready to merge** (D-01/D-02/D-04/D-06 open) |
| Ch4 | 40 | 1 | 0 | 0 | 0 | 0 | **ready to merge** after D-11 is decided (§5) |
| Ch5 | 46 | 5 | 0 | 2 → **fixed** | 0 | 1 → **fixed**; sign error "closing the gap" → **fixed** (`followup_examiner.md` W1) | **ready to merge** |
| Ch6 | 23 | 9 | 0 | 1 → **fixed** | 0 | 0 | **ready to merge** |
| Ch7 | 13 | 2 | 0 | 1 → **fixed** | 0 | 0 | **ready to merge** |
| Ch8 | 11 | 5 | 0 | 0 | 0 | 0 | **ready to merge** |
| Ch9 | 9 | 5 | 0 | 1 → **fixed** | 0 | 0 | **ready to merge** |

No hunk lacks a justification. The 25 marker-less hunks left (inventory column "see audit §2") are all of the kinds
listed in §2.

## 2. Hunks without a marker, and their justification

| Hunks | Change | Justification |
|---|---|---|
| ch2-H3 | "companion arXiv paper, of which version 4 is held" | R15 §2 (v4 held); P2 Lee note (Stage 4 Ch2 pass A) |
| ch4-H2 | header comment: obsolete seven-strategy numbering replaced | P4-22 (comment only; the new text states "CHANGED P4-22" itself) |
| ch5-H15–H17 | Table 5.1: two columns added (team size, default parameters) | P5-13, code `scenarios.py:134–470`; the marker sits above the table, more than 12 lines before these rows |
| ch5-H33 | position-error formulas | P5-11 (marker at the start of the metrics block); notation fixed in this pass (§3, item 6) |
| ch5-H40 | Table 5.2: internal names (`ctrltarget.*`, "EKF-vs-mocap gate") replaced by generic wording | P5-17 (marker below the table) |
| ch6-H1, H2; ch7-H1, H2; ch8-H1, H2; ch9-H1 | file headers: revision note, "C.4" removed from the honesty note, `docs/*.md` source lines removed | H3/H9 (comments only) |
| ch6-H9, H10, H15, H16, H19 | "S0" → "Strategy 0"; "phase metrics" → "per-flight metrics"; figure-slot wording; heading "(Neural-Swarm2)" → "(Strategy 2)"; "No invented rankings until C.4 data exist" → "No ranking … without the statistical support" | P6-02 (internal labels and file names); the heading change avoids implying that Strategy 2 *is* Neural-Swarm2 (Ch3 §3.4.3 lists the differences) |
| ch8-H8–H10; ch9-H8, H10 | `Chapter~\ref{ch:results…}` → names in prose; "C.4" → "the comparison" | H6, P8-02 |
| ch9-H7 | RQ4 answer slot restated | P9-02 |

## 3. (b) Statements stronger than their source — found and fixed

1. **Ch2 §2.2, Kiran et al.** "at the closest vertical separation tested, 13 cm". The paper says "the closest vertical
   separation (Δz/l = 4)"; 13 cm is derived (l = 32.5 mm), and "tested" overstated the test matrix. **Fixed:** "at the
   closest vertical separation they report, four centre-to-motor distances (about 13 cm)".
2. **Ch2 §2.2, Gielis et al.** "negligible influence on neighbours outside it" left out the paper's own caveat (small,
   chaotic lateral forces at larger distances, R03 §5). **Fixed:** caveat added.
3. **Ch2 §2.3, INDI mechanism.** The sentence attributed to Tal & Karaman and Smeur an explanation ("a spurious term …
   an artefact of the delay") that they do not give; they state the requirement (identical filters, equal phase lag).
   **Fixed:** the requirement is cited and the explanation stands as the thesis's own reasoning.
4. **Ch5 §5.5.5, statistical treatment.** "A difference … smaller than the spread … on C1–C3 is reported as
   indistinguishable" stated a decision rule that neither the draft nor any source fixes (cf. the author's standing
   rule against invented protocol rules). Pairing within sessions was also stated as a rule. **Fixed:** the floor is
   named, the rule moved into the `\todo` the author must fill, and pairing is "can be made".
5. **Ch6 §6.2/§6.4/§6.6, Ch7 §7.2, Ch9 §9.2 negative-answer paragraphs.** These used the same invented rule
   ("null-scenario spread"); Ch7 also set a usability threshold ("comparable to the measured residual … not usable").
   **Fixed:** each paragraph now refers to "the rule fixed in Section setup-stats", and Ch7 reports a detected penalty
   relative to the measured residual without stating a threshold.
6. **Ch5 §5.5.4, effort measures.** The definitions are proposals, but no `\todo` covered them. **Fixed:** `\todo{Confirm
   the effort measures before the campaign.}`

Re-read and kept (each supported as worded): Ch1 "≈9 g … within thrust capability" (R01 §I); Ch2 Bauersfeld "about three
to five" (Table III: 69.1→24.8 mm and 134.2→25.4 mm, factors 2.8 and 5.3); Ch2 "repetitions between three and twenty"
(Li 3, Shi 6, Hsieh 5, Shankar 10, Cobo-Briesewitz 20); Ch2 Chee achieved separation (0.12 m vs 0.08 m); Ch2 NS2
crash remark (quoted, caveated); Ch2 Shankar observer remark (quoted); Ch2 Cobo-Briesewitz PWM result (Table 1); Ch2 SN
sentence (Shi 2020: "spectrally normalized g is γ-Lipschitz continuous", SN "to the weight matrices in each layer");
Ch2 Hsieh 2026 SE(3) −43.7 % (§5.2); Ch5 Gielis null-scenario basis (R03 §5, with the caveat).

## 4. (c) Removed project findings still implied — none found

Scanned for "in practice", "turned out", "was found", "has shown", "crash", "tumble", "verified", "checked". Hits:
- Ch3 L498 (Gu et al. crash): literature, explicitly "in simulation" ✓.
- Ch2 L248 (NS2 crash remark): literature ✓.
- Ch4 L240 "verified against the source": a citation-discipline statement about the companion paper, not a flight
  result ✓.
- Ch1 L244 "checked numerically against the compiled controller": an apparatus property (a test of the pipeline), not a
  result about any method. It is kept, and the author may delete it if it reads as a result.
- Ch5 V6 rewrite: the merge tool's internal note on why cross-drone correlation failed in development is **not** used ✓.
- The `% REMOVED (H1)` passages (Ch5 ×3, Ch7 ×1) have no surviving paraphrase in the printed text ✓.

## 5. (d) New symbols, acronyms, forward references

| Place | Issue | Status |
|---|---|---|
| Ch5 §5.5.4 | `e_i` (position error of vehicle i) collided with the world unit vectors `e_ℓ`/`e_3`, and its sign was opposite to Ch4's `e_p = p_des − p` | **fixed:** renamed `e_{p,i} = p_cmd,i − p_i`; `e_1, e_2, e_3` defined as world unit vectors |
| Ch2 §2.3 footnote | "MPC" used before it is spelled out in Ch2 | **fixed:** "model predictive control (MPC)" |
| Ch2 §2.3 footnote | "$\mathcal L_1$-adaptive" not explained at first use | **fixed:** one-clause gloss (estimates the disturbance from the mismatch between a predicted and a measured state, R10 §2.2) |
| Ch5 | `W`, `W_c`, `Δ`, `t_c`, `RMS_W`, `e_sep`, `ε_pred` | all defined where first used ✓ |
| Ch2 §2.1 | `SO(3)`, `SE(3)`, hat/vee, Ψ | defined ✓ |
| Ch6 §6.5 | `\ref{sec:ctrl-s1}`, `sec:model-sign` | backward references ✓ |
| All | forward references to Ch6–9 | by name in prose (H6) ✓ |

## 6. New issue found during the audit (not an APPLY defect)

**D-11 — which INDI controller Strategy 1 is.** Ch4 §4.1.3 and §4.3 describe Strategy 1 as the INDI inside
`lib.rs:controller_step` (ctrl_mode 3, `res_sign` and `ki_z` parameters). The frozen study block
`crazyswarm2/crazyflie/config/study_strategies.json` "S1" selects `stabilizer.controller: 9`
(`crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c`). That controller subtracts the residual
(`a_indi = a_rpm_f − a_imu_f`, L260; thrust L268), filters both terms with identical 30 Hz Butterworths (L131–136), and
has its own vertical integral (`Kpos_Iz`, study value 1.5). By H5 the thesis must not state which implementation is
used, so nothing was changed in the printed text. D-11 is added to `author_decisions.md`, and every D-01/D-02 comment that
states Strategy 1's configured values now carries a note saying that those values belong to the ctrl_mode 3 block.

## 6b. Lost prose found by a sentence-survival check (fixed)

The examiner re-read (Task 5) found that two `% CHANGED` markers inserted in Phase 4 had commented out the prose that
followed them on the same line in Ch1:
- Ch1 §1.1: "It is also hard to derive from" (the sentence was cut in half in the PDF).
- Ch1 §1.1: "The margin has been substantially reduced."

Both are restored. To rule out further cases, every printed sentence of each original chapter was searched for
verbatim in the revised printed text (comments stripped). Sentences that were not found were matched by similarity,
and checked against the `% REMOVED` comments. Of 748 original sentences, 153 are not verbatim. All of them are either
edits with a marker or plan row, or removals kept as comments. The 47 with low similarity (< 0.6) were each traced to a
plan row (Ch1 P1-04/P1-08; Ch2 P2-02, P2-03, P2-08, P2-10, P2-12, P2-13, P2-15, P2-16, P2-23, P2-26; Ch3 P3 network and
guard rows; Ch4 P4-07/08/10/20, V3; Ch5 P5-01/05/07/11/16/20, V6; Ch6–Ch9 P6–P9). No other prose was lost.

## 7. Build after the fixes
Clean: 60 pages, no errors, no undefined references or citations, no BibTeX warnings.

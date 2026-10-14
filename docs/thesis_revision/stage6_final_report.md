# Stage 6 — Final report (examiner, student reader, supervisor)

Thesis: *Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams* (G. Wolnik; supervisor
Prof. W. Hönig). Revised draft: `revised/thesis_draft.tex` (Ch1–Ch9, attached preamble) — **builds cleanly, 60 pages**,
no undefined references or citations. No results were added (H1); code was only read.

## 1. Overall verdict

The draft's design chapters (Ch3–Ch5) are now an accurate description of the flight code and a reproducible protocol,
and Ch1–Ch2 state prior work with the right context for every number. The revision fixed **stale design facts** (network
size and guards, loop rate, logged variables, $e_R$ logging, rotor-speed source), **reference-context errors** (NS2,
Smith, Smeur, Gielis, Shankar, Cobo-Briesewitz, Hsieh 2026 in Ch4), and **two factual errors about Hsieh et al. 2025**.
What it could not fix by rule is concentrated in five author decisions, of which two — **the residual sign of
Strategy 1 (D-01)** and **the gap statement (D-03)** — would each draw the first question at a defence.

## 2. Rubric per chapter (0–5), before → after

| Criterion | Ch1 | Ch2 | Ch3 | Ch4 | Ch5 | Ch6–9 |
|---|---|---|---|---|---|---|
| 1 Correctness | 3→4 | 3→4 | see pass file | 2→3 | 2→4 | 3→4 |
| 2 Completeness | 4→4 | 3→4 | ″ | 3→4 | 2→4 | 3→4 |
| 3 Depth | 3→3 | 4→4 | ″ | 3→4 | 3→4 | — |
| 4 Academic standard | 4→4 | 4→4 | ″ | 4→4 | 3→4 | 2→3 |
| 5 Explainability | 4→4 | 3→4 | ″ | 3→4 | 4→4 | — |
| 6 Related work / referencing | 3→4 | 3→4 | ″ | 3→4 | 2→3 | — |
| 7 Cross-chapter consistency | 3→4 | 3→4 | ″ | 2→4 | 2→4 | 2→4 |
| 8 Reference correctness | 3→5 | 3→5 | ″ | 3→5 | 3→4 | — |

(Ch3 figures are in `stage4_ch3_passes.md`.) The scores after revision are capped by the open DECISION rows:
correctness of Ch3/Ch4 reaches 4–5 only once D-01 and D-02 are decided.

## 3. Examiner's report

**Strengths.** A clearly argued comparison design: frozen scenario library with null scenarios; one residual definition
logged under every controller; runtime selection allowing interleaving; per-strategy tuning then freezing, with the
fairness rule stated honestly; explicit separation of implemented strategies from reference methods; a citation
discipline that ties every number to what it is a number of. Simulation is correctly excluded from the evidence chain.

**Main concerns (in order).**
1. **Sign of Strategy 1 (D-01).** The thesis derives that the residual must be subtracted; Strategy 1 is configured to
   add it. By the thesis's own Ch3 argument such a controller doubles the disturbance. Either the configuration or the
   text must change before any Strategy 1 result is interpretable.
2. **Gap (D-03).** Hsieh et al. (2025), a paper the thesis cites, compares measured, predicted and combined compensation
   on three vehicles. The remaining novelty (onboard INDI vs learned feedforward, multi-regime library, transfer penalty)
   is real but must be stated narrowly.
3. **Equations vs code (D-02).** The control-law equations show integral terms the code does not run, and omit the
   vertical integral it does run, with different gains per strategy — a fairness question as well as a correctness one.
4. **Collection rationale (D-04).** The stated reason for collecting under the baseline is contradicted by the thesis's
   own residual definition; better reasons exist.
5. **Statistics (D-09).** $n$, test and effect size are open; the metric definitions are now in place (Ch5 §5.5.4) but
   the inference procedure is a skeleton.
6. **Lab status in the text (D-05).** Several passages report campaign progress with nicknames, dates and file names.

**Questions to expect** (answered where noted): sign of Strategy 1 (D-01); how the gap differs from Hsieh 2025 (D-03);
whether the controller has integral action (D-02); whether INDI's filters are matched as Tal & Karaman require (Ch4
§4.3 deviations, D-06); how windows are defined so they cannot favour a controller (Ch5 §5.5.4, commanded geometry);
how the transfer penalty is computed (Ch5 §5.5.4, Ch7 §7.2); what C1 tests when the network gate excludes 0.30 m
neighbours (Ch5 §5.3.2); whether Strategy 2 is Neural-Swarm2 (Ch3 §3.4.3, Ch4 §4.4: same architecture, no spectral
normalisation, vertical scalar, code-defined gate); whether anything rests on simulation (Ch5 §5.6: no); what an
indistinguishable result would mean (Ch1 §1.6, Ch6 negative-answer paragraphs).

## 4. Student-reader report

A Master's student with control basics can now follow the thesis without outside reading on: rotation matrices, hat map
and the attitude error $\Psi$ (Ch2 §2.1, Ch4 §4.2); what differential flatness fixes (Ch2 §2.1); how INDI forms the
residual and why the filters must match, with a one-axis example (Ch2 §2.3, Ch4 §4.3); the residual and its sign
(Ch3); deep sets and the summation assumption (Ch3); spectral normalisation in one sentence (Ch2 §2.4). Remaining
stumbling points: feedback linearisation is used before it is defined (a textbook reference is proposed, A26); the
two "Hsieh et al." papers are easy to confuse in prose (add years, Stage 5 §2).

## 5. Supervisor items

1. **D-03** (gap) and **D-05** for Strategy 3 (whether "four strategies are compared" can stand) need the supervisor.
2. **D-01** and **D-06** are lab actions before the campaign (reconfigure the sign; make nominal and realised filter
   cut-offs equal) — cheapest fixed in configuration, not in text.
3. **D-09**: agree $n$ and the analysis before the first comparison flight.
4. The two provenance `\todo`s should be revised once D-03 is decided.

## 6. What remains for the author

- Decide D-01–D-10 (`author_decisions.md`), then apply the recommended wording at the listed lines.
- Resolve V1–V9 (`author_decisions.md`, [VERIFY] list).
- Check and, if wanted, add references from `proposed_references.md` §A; apply §B bibliography corrections.
- Fill Ch6–Ch9 from the campaign using the slots provided; each slot names its metric, statistic and negative answer.
- Copy the revised chapters into `docs/thesis/` only after review (originals were not overwritten).

## 7. Deliverables

`stage0_draft_audit.md`, `stage1_sources.md`, `stage1b_reconciliation.md`, `stage2_coverage_gaps.md`,
`stage3_revision_plan.md`, `stage4_ch{1,2,3,4,5,6-9}_passes.md`, `revised/ch1_introduction.tex` …
`revised/ch9_conclusion.tex` (+ `revised/thesis_draft.tex`, `revised/preamble_draft.tex`), `proposed_references.md`,
`stage5_consistency.md`, `stage6_final_report.md`, `author_decisions.md`.

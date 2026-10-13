# Thesis revision prompt — Chapters 1–5 (and the Chapter 6–9 skeletons)

**Prepared 2026-10-13.** One prompt for a more capable model. Everything between `===== BEGIN PROMPT =====` and `===== END PROMPT =====` is the prompt; the rest is for the author (how to run it, what to attach, how to check the output).

## For the author: how to run it
1. **One chapter per run** (Ch. 1, then 2, 3, 4, 5; the skeletons 6–9 together in a last run). Long outputs get cut and sloppy otherwise.
2. **Attach, for every run** (paste the text; file paths are in this repo):
   - the prompt below, unchanged, with the `TARGET CHAPTER` line filled in;
   - the target chapter `docs/thesis/chN_*.tex`, and **all other chapters read-only** (for consistency): `ch1_introduction.tex`, `ch2_related_work.tex`, `ch3_system_modelling.tex`, `ch4_control_architectures.tex`, `ch5_experimental_setup.tex`, plus `ch6…ch9` skeletons;
   - `docs/thesis/thesis_draft.tex` and `docs/thesis/preamble_draft.tex` (macros, build);
   - `docs/references.bib` (the only bibliography);
   - `docs/_archive/citation_notes/20_Verified_Claims.md` (every citable claim in the sources' own words), `docs/_archive/citation_notes/17_Source_Ledger_and_Citation_Discipline.md` (rules and the three past misattributions), `docs/_archive/citation_notes/16_Related_Work_Structure.md`;
   - `docs/15_Problem_Statement_and_Research_Questions.md`, `docs/19_Contribution_Statement.md`, `docs/18_Strategy_Descriptions.md`, `docs/04_Unified_Residual_Wrench_Model.md`, `docs/10_Formation_Library.md`, `docs/05_Experimental_Protocol_2Robot.md`;
   - optionally the PDFs of the sources in `docs/papers/` (20 PDFs; gitignored, `docs/papers/fetch_papers.sh` rebuilds them) — if the model can read PDFs, attach them for the chapters that cite them, because the verification step below is much stronger with the paper in hand.
3. **Never merge the output blindly.** Merge chapter by chapter, `cd docs/thesis && latexmk -pdf thesis_draft.tex` must stay clean (no undefined citation or reference), check every item of the model's `[VERIFY]` list against the paper before accepting it, then commit.
4. **Do not add flown, lab or simulation results while merging.** That is a separate step after the study.

===== BEGIN PROMPT =====

# ROLE

You play three roles at the same time, and you must keep them apart in your output.

1. **Senior professor and thesis supervisor** in robotics and control, specialised in aerial robotics and multi-robot systems. You have supervised many Master's theses and sat on many examination boards. You are demanding, fair and precise. You know what a Master's thesis must contain and what an examiner will attack. You do not flatter.
2. **The author's writing assistant.** The author is a Master's student (name: Georg Wolnik; supervisor: Prof. Wolfgang Hönig; thesis: *Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams*, Crazyflie 2.1 Brushless quadrotors, deadline 26 February 2027). The text must stay **the author's text**: you improve it, you do not replace it with a different voice or a different argument. Impersonal academic prose, the existing first-person-plural usage only where it is already there.
3. **The reader.** A fellow Master's student in robotics or control engineering: knows linear algebra, basic control, rigid-body dynamics, has heard of PID and Kalman filtering, but **has not worked on multirotor interaction forces, INDI, geometric control on SE(3), or learned residual models**. After reading a chapter this student must be able to follow every step, know why it is there, and reproduce the key derivations.

# TARGET CHAPTER

`TARGET CHAPTER: <fill in: e.g. "Chapter 3, ch3_system_modelling.tex">`

All other chapters are attached **read-only** for consistency checking. Change only the target chapter.

# WHAT THE THESIS IS (so you can judge what belongs in a chapter)

- **Question:** which family of interaction-force compensation (measure the residual online, predict it from the neighbours' relative state, or combine both) is preferable for tight multirotor formation flight, under which conditions and at what cost?
- **What the thesis claims:** a *controlled comparison* on one airframe, one frozen scenario library and one measurement pipeline. It proposes **no new control law**. Overclaiming is the single most likely thing an examiner punishes.
- **Compared set (decided 2026-09-22, this is the scope of Chapters 1–5):**
  - Strategy 0: geometric tracking control without compensation (baseline, outside the "compensating" set);
  - Strategy 1: pure INDI (incremental nonlinear dynamic inversion; reactive: measures the residual from rotor speeds and acceleration);
  - Strategy 2: geometric control with a learned residual (Neural-Swarm family: a network predicts the residual from neighbours' relative position and velocity);
  - Strategy 3: feedback linearisation with a flatness-preserving residual (implementation **not yet available**: the authors' code has not been received; it must be described as planned or as a reference method, never as implemented).
  - **Not compared** (supervisor decision): NA-INDI / LINDI hybrid, residual RL, learning-based MPC. They appear **only** as related work in Chapter 2. Their reserve text is `docs/thesis/reserve/advanced_strategies_reserve.tex` and is not part of the build. If any numbered strategy above 3 survives in the main chapters, that is a defect.
- **Fairness rule (current):** every strategy is its own controller; each is tuned for itself on the hardware gate; its gains are then frozen before the comparison flights. "Identical conditions" means identical trajectories, formations, hardware, separations, estimator and residual definition, **not** identical control structure or one gain set. Any text that still speaks of "one shared control-law substrate" or "one shared gain set" is stale and must be fixed.
- **Research questions RQ1–RQ4 and contributions C1–C5** are defined in Chapter 1 and `docs/15`, `docs/19`. RQ3 is the *size of the transfer penalty* of a summed per-neighbour model going from two to three vehicles; it is **not** a test of whether superposition holds (already published as not exact).

# HARD RULES (violating one of these is a failure of the whole run)

**H1. No results.** The draft must contain **no** number, statement or table that comes from a flight, a lab session, a simulation run or a replay of this project. No "we observed", "we found", "the controller achieved". Chapter 5 stays in the **future tense** (nothing has been run yet in the thesis' own timeline); Chapters 6–9 stay skeletons with placeholders. You may improve the *structure*, the *definitions of metrics*, the *expected form* of tables and figures, and the *hypotheses* (stated as falsifiable), never their values.

**H2. Nothing invented.** Never invent a reference, author, year, venue, DOI, page number, quotation, dataset, number or experimental detail. Every statement about the literature must be supported by (a) `20_Verified_Claims.md` or (b) the source PDF if attached. If you believe something is true but cannot trace it, write it as an **open item** in the `[VERIFY]` list and leave the sentence in its current (weaker or unchanged) form with a `% VERIFY:` comment in the LaTeX. Never *strengthen* a claim.

**H3. Citation discipline.**
- Cite the **primary** source. Never cite the author's own notes (`02_Literature_Matrix`, `paper_summaries`, any `docs/*.md`) as a source.
- Keep these pairs apart: *Pei-An Hsieh et al. 2026 (flatness-preserving residual, two vehicles, off-board controller)* vs *Hsieh et al. 2025 (L1 KNODE-DW MPC, three vehicles)*; *Neural-Swarm (2020)* vs *Neural-Swarm2 (2022)*; Lee, Leok, McClamroch CDC 2010 is the primary geometric-control citation (arXiv 1003.2005 only for equation-level checks); Tal & Karaman 2018 is the primary INDI citation (Smeur 2017 for the cascaded variant).
- The recurring fault in this project was **attaching a correct number to the wrong thing** (wrong axis, wrong training set, wrong platform, wrong experiment, an abstract-only figure presented as a body result). Treat every number with its **context** (what, on which platform, trained how, simulation or hardware, abstract or body).
- Three misattributions and about twenty accuracy corrections have already been caught. Assume more exist. Hunt for them.
- Do not write "nobody compares controllers for close-proximity flight": it is false (Abro et al. 2025, Gu et al. 2025 exist). The surviving, narrower claim is in Chapter 1/2 and is hedged with "to the best of our knowledge".

**H4. The equations are the code.** Chapters 3 and 4 state that their equations are the equations implemented in the flight code. Do not change an equation, a sign, a frame or a symbol silently. If you believe one is wrong, report it in the review (with the derivation of why) and leave the text unchanged unless the fix is purely typographical. Frame convention: **z-up (ENU), gravity along $-e_3$**. The residual is **subtracted** in the control law (Chapter 3 §3.3.3 derives why and states the falsifiable consequence of the opposite sign).

**H5. Implementation honesty.** Chapter 4 separates, for every strategy, *what we implement* (verifiable against our code) from *the reference method* (cited, deviations stated). Never merge them, never describe an unimplemented strategy as if it exists, never state which of several of our own INDI implementations will be used in the study (that decision is open).

**H6. Keep the build and the author's machinery.** Keep labels, `\ref`/`\cite` keys, macros, `\todo` markers (`\todo{}` that are open items stay; "Provenance" notes stay), section order unless you justify a move in the review. Do not touch `references.bib` except through the "proposed references" output below. LaTeX must compile with the attached preamble. Units with `\SI{}`. Chapters 6–9 are not referred to by `\ref` (they are referred to by name in prose) until they exist.

**H7. Scope of change.** Improve **what exists**. You may add explanation, definitions, worked examples, figures described in text or TikZ, tables, signposting and caveats. You may restructure within the chapter. You may **not** add new experiments, new strategies, new claims about the project's own results, or new research questions.

# WHAT "BETTER" MEANS — THE CRITERIA

Score the chapter 0–5 on every criterion before and after your revision (a rubric table in the output). 5 means "an examiner finds nothing to criticise on this point".

1. **Correctness.** Every technical statement, equation, definition and number is right and consistent with the sources and with the other chapters; units and frames consistent; no sign errors; no stale scope (strategies, fairness rule, platform).
2. **Completeness.** Everything a reader needs to understand and judge the chapter is there: definitions before use, assumptions stated, limits stated, each claim supported, each term explained. Nothing the thesis later relies on is missing.
3. **Depth and detail.** Derivations shown where the reader would otherwise have to trust the author; the "why" of each design choice; alternatives mentioned and rejected with a reason; where Chapter 2 or 3 would be too thin for a Master's thesis, say exactly what is missing.
4. **Academic standard.** Clear argument structure (claim → evidence → limitation); precise terminology used consistently; hedged where the evidence is limited; no marketing language ("novel", "state-of-the-art", "significantly" without a statistic); signposting (what a section does and why here); figure and table captions that stand alone; consistent notation; consistent citation form; limitations stated honestly; no overclaiming; a Master's-level but not padded register.
5. **Explainability for students.** Intuition before formalism; one worked or toy example per non-obvious idea; each symbol introduced once, in words, at first use; jargon defined (INDI, flatness, deep sets, residual wrench, downwash, hardware gate, …); "what this means in practice" sentences after dense paragraphs; a short "reader's map" at the start of a long section where useful; no step left to the reader that a student of the target profile would not make.
6. **Related work: coverage and referencing.** Every family relevant to the thesis' question is covered, each represented fairly (what it does, on what platform, with what evidence, and its limits), positioned against the thesis' question; the gap is argued from the literature, not asserted; references complete and correct (authors, year, venue, what exactly is claimed). Identify **missing** relevant work and propose it (see output §5) — but never add it as a verified citation yourself.
7. **Consistency across chapters.** Same strategy numbering, same names, same symbols, same numbers; Chapter 1's promises (RQs, contributions, outline) are exactly what Chapters 2–5 deliver and Chapters 6–9 will report.
8. **Reference correctness (zero tolerance).** Each in-text claim → the right paper → the right thing within the paper. This is audited separately (Pass D).

# PROCESS — do the passes in this order and show their output

**Pass A — claim extraction.** Read the target chapter sentence by sentence. Produce a table: *claim → type (literature fact / own design statement / derived result / definition / opinion) → source key or "own" → status (verified in `20_Verified_Claims` / verified in PDF / unverified)*. Unverified literature claims go to `[VERIFY]`.

**Pass B — defect log.** List every defect with: location (section, line or paragraph start), category (correctness / completeness / depth / standard / explainability / related work / consistency / reference), severity (blocker / major / minor), what is wrong, the proposed fix. Be concrete; "could be clearer" is not a defect, "the symbol $\omega_d$ is used in Eq. (3.12) before it is defined at Eq. (3.15)" is.

**Pass C — rewrite.** Produce the full revised chapter as compilable LaTeX. Operations you may apply, in order of preference: fix errors; add missing definitions and assumptions; add motivation ("why this step"); add a worked or toy example; add a short summary box or "key idea" sentence per section; tighten and de-duplicate; reorder within the chapter for flow. Keep the author's sentences wherever they are already good (minimal edit principle) so that the author can recognise and defend the text. Mark every substantive new paragraph with `% NEW:` and every changed claim with `% CHANGED:` plus a one-line reason.

**Pass D — citation audit.** For every `\cite` in the revised chapter: key exists in `references.bib`; the cited work really makes the stated claim (give the supporting quote or the section/page if you have the PDF; otherwise mark `UNVERIFIED`); the number is attached to the right context. Output a table. A failed row is a blocker.

**Pass E — examiner simulation.** Write the 12 hardest questions an external examiner would ask about this chapter, and for each state where the revised text answers it (or that it does not and what is needed). Include at least two questions attacking the gap claim, two attacking fairness or scope, one on each equation block, and one on the limitations.

**Pass F — student-reader test.** Simulate a student of the reader profile reading the revised chapter once. List every place where they would stumble (undefined term, skipped step, unexplained symbol, unmotivated choice) and show how the revised text resolves it. If any remain, fix them and say so.

# CHAPTER BRIEFS (what to check in each, beyond the criteria above)

**Chapter 1 — Introduction.** Motivation grounded in the physical problem (downwash, close-proximity flight) and in real applications, without hype; the problem stated precisely; the gap argued narrowly and with the audited wording; strategies introduced at the right level (one paragraph each, no detail that belongs in Ch. 4); RQs falsifiable and answerable by the Ch. 5 design; contributions mapped 1:1 to C1–C5 and not overclaiming (comparison, not a method); scope and limitations; outline matching the real chapters. A reader who stops after Chapter 1 should know exactly what is asked, why, how, and what is *not* claimed.

**Chapter 2 — Background and related work.** Verify the structure by *how the interaction force is obtained* (measured / predicted / learned directly / hybrid) and that the families are complete and fairly described. Check every number (training set, platform, vehicles, separation, simulation vs hardware, onboard vs off-board, abstract vs body). Section 2.8 ("how these methods are evaluated") is load-bearing: it must stay methodological, not become criticism of individual authors. Check that background material (rigid-body dynamics, geometric control, INDI, deep sets, flatness) is sufficient for the student reader without becoming a textbook. Identify missing literature (aerodynamics of rotor interaction, close-proximity formation control, INDI/NDI foundations, learned dynamics residuals, uncertainty and safety, benchmarking practice in aerial robotics).

**Chapter 3 — System modelling and residual-force formulation.** The unified wrench model, notation (§3.1 is authoritative), how the residual is measured and why that measurement fails in specific ways (rotor-speed source, filters, delay, thrust-model mismatch), the sign convention, the deep-sets prediction model, the superposition assumption, input conditioning and normalisation, assumptions and where they fail. Check each derivation step; add the missing steps; add small numeric or toy examples that show orders of magnitude **without using project results** (use textbook or clearly labelled illustrative values). Check consistency with the code claims (H4).

**Chapter 4 — Control architectures.** For each strategy: the reference method, what we implement, the deviations, the failure modes expected from the structure, the tuning surface. Check the position/attitude loops against Lee et al. and Tal & Karaman. Check the fairness section against the current rule (above) and that it states what the design does **not** guarantee. No strategy above 3 in the main text. No statement about which of several INDI implementations is used.

**Chapter 5 — Experimental setup and protocol.** Platform, volume, formation library (regimes, null scenarios), measurement and logging (why onboard, what is recorded, time alignment), the hardware gate, data collection, comparison campaign, metrics (each defined mathematically, with the window and detector it uses), role of simulation (used to disprove implementations, not to validate methods), threats to validity. Future tense. Every open `\todo` stays and is listed. Check that every metric is computable from what is logged and that every threat has a mitigation or an honest "not mitigated".

**Chapters 6–9 (skeletons, last run only).** Improve the structure and the placeholders only: what table, what figure, what statistic each section will contain, how each RQ will be answered and what a negative answer looks like, the structure of the discussion (including a section that reports where a frozen gain appears to have disadvantaged a strategy), the limits section, the conclusion's mapping to RQ1–RQ4. Placeholders stay placeholders; no numbers, no "we found".

# STYLE GUIDE

- British English. Impersonal, precise, past/present tense as already used; future tense in Chapter 5.
- Definitions in the sentence of first use; abbreviations spelled out once; consistent symbols (notation section of Chapter 3).
- Sentences that carry one idea; paragraphs that carry one point; every section opens with what it does and why it is here and closes, if long, with what the reader should now know.
- No filler, no rhetorical questions, no exclamation, no "obviously/clearly/trivially" for steps the student would not find obvious.
- Hedging proportional to evidence: "is reported to", "to the best of our knowledge", "in the cited experiments". Numbers always with their context.
- Figures and tables: caption that stands alone; every figure and table referenced and explained in text. Describe missing figures as `\todo`/TikZ proposals; do not invent data to fill them.

# OUTPUT FORMAT (exactly this order, one chapter per answer)

1. **Verdict** (5 lines): the chapter's state, the three biggest problems, the three biggest improvements.
2. **Rubric table**: the 8 criteria, score before / after, one-line justification.
3. **Defect log** (Pass B), sorted by severity.
4. **Revised chapter** — complete LaTeX in one code block (do not abbreviate, do not write "unchanged"; if you cannot output it all, stop at a section boundary and say exactly where, so the next answer continues there).
5. **Proposed additional references** — only works you are confident exist; for each: full bibliographic data, why it belongs (which claim or section), confidence, and the status `UNVERIFIED — author must check`. A separate list of **suspected errors in existing `references.bib` entries**.
6. **Citation audit table** (Pass D).
7. **`[VERIFY]` list for the author**: every item the author must check against a paper, a log or the code, with the exact place to check.
8. **Examiner questions** (Pass E) and **student-reader test** (Pass F).
9. **Change log**: what changed and why, short, so the author can defend every edit.
10. **Questions to the author**: only those that block a correct rewrite.

# WHAT YOU MUST NOT DO

Do not add results or numbers from the project. Do not invent or "improve" references. Do not strengthen claims. Do not change equations silently. Do not expand the scope (new strategies, new experiments). Do not rewrite the author's argument into a different one. Do not pad. Do not output a summary instead of the chapter. Do not describe unimplemented strategies as implemented. Do not state study decisions that are still open (which INDI implementation, whether Strategy 3 is implemented).

# FACT SHEET (project facts through 2026-10-13; use for consistency, never as results)

- **Platform:** Crazyflie 2.1 Brushless, ≈ 41 g; rotor speed from DShot telemetry or an optical RPM deck (needed to reconstruct applied thrust and hence the residual; without it the residual reads zero); micro-SD deck for onboard logging; on-board extended Kalman filter fed by 100 Hz motion capture; at least two vehicles (three for the three-robot scenarios).
- **Flight volume:** ≈ 2 m × 4 m × 1.7 m tracked; operational floor 0.30 m (0.10 m for one near-ground scenario). The figures are not yet tape-measured (open `\todo`).
- **Formation library:** 16 frozen scenarios (A1–A8 two-robot, B1–B3 three-robot, C1–C5 other), including null scenarios that establish the measurement floor.
- **Residual definition:** $a_{\mathrm{res}} = a_{\mathrm{measured}} - a_{\mathrm{model}}$ with the model acceleration reconstructed from rotor speeds; logged under every controller whether or not the controller uses it.
- **Strategy 2 network:** deep-sets residual model, per-neighbour embedding summed, evaluated onboard at about 100 Hz; weights flash-resident; trained on two-vehicle flights. Do not state its accuracy, its training set size or any flight number in the draft.
- **Controllers:** Strategies 0–2 are implemented in the project's flight code; Strategy 3 is not (author code pending). Which INDI implementation the study uses is undecided.
- **Timeline:** deadline 26 Feb 2027; the study has not been run in the thesis' timeline; the thesis draft must therefore read as written before the results.
- **Known open items in the text:** tape-measure the volume (Ch. 5); revise Ch. 5 to past tense after the campaign; two "provenance" notes in Ch. 1 and Ch. 2 document audits and are not pending work.
- **Citation hygiene incidents** (why H2/H3 exist): a note said a hybrid showed "limited benefit" (wrong), its correction said "lowest tracking error" (also wrong: small, condition-dependent margin, headline is sensor elimination); one paper's "simulation and hardware experiments" were a simulator with a Jetson in the loop, not flight; one lateral figure was attached to 3D.

===== END PROMPT =====

## After the run (for the author)
- Merge one chapter, build, read the PDF (defects have been caught by reading the rendered PDF, not the source).
- Walk the `[VERIFY]` list; accept proposed references only after checking them against the paper.
- Keep Ch. 5 in the future tense and Ch. 6–9 as skeletons until the study has run; results enter in a separate pass.
- Record what changed in `docs/07` (History) and in the memory note `project_thesis_revision_prompt_plan_2026-10-13.md`.

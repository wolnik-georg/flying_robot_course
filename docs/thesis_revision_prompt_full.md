# Thesis revision — ONE complete end-to-end prompt

**Prepared 2026-10-14.** One prompt, one run: the model reads the whole draft, reads every locally held paper, checks the code, plans, rewrites Chapters 1–5 (and the skeletons 6–9), audits itself and reports. Everything between `===== BEGIN MASTER PROMPT =====` and `===== END MASTER PROMPT =====` is pasted as one message. For a model without file access, paste the thesis chapters, `references.bib`, the four code files and the PDFs as attachments first. The staged version (`docs/thesis_revision_prompt.md`) remains for runs where the context window is too small.

**Honest limits of one run:** a single session cannot hold 20k words of thesis, 18 papers and 2,600 lines of code at full attention; the prompt therefore makes the model re-open sources before each step and write outputs to files. Read the `author_decisions.md` it produces before merging anything, and read the rendered PDF after each merged chapter.

===== BEGIN MASTER PROMPT =====

# ROLE

You play three roles and keep them apart in your output.

1. **Senior professor and thesis supervisor** in robotics and control, specialised in aerial robotics and multi-robot systems. You have supervised many Master's theses and sat on many examination boards. Demanding, fair, precise; you do not flatter.
2. **The author's writing assistant.** The author is a Master's student (Georg Wolnik; supervisor Prof. Wolfgang Hönig; thesis *Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams*; Crazyflie 2.1 Brushless quadrotors; deadline 26 February 2027). The text stays **the author's text**: you improve it, you do not replace the voice or the argument. Impersonal academic prose; first-person plural only where it is already used.
3. **The reader.** A fellow Master's student in robotics or control engineering: knows linear algebra, basic control, rigid-body dynamics, PID and Kalman filtering, but **has not worked on multirotor interaction forces, INDI, geometric control on SE(3), or learned residual models**. After a chapter this student must follow every step, know why it is there and reproduce the key derivations.

# WHAT THE THESIS IS

- **Question:** which family of interaction-force compensation (measure the residual online, predict it from neighbours' relative state, or combine both) is preferable for tight multirotor formation flight, under which conditions and at what cost?
- **Claim:** a *controlled comparison* on one airframe, one frozen scenario library and one measurement pipeline. It proposes **no new control law**. Overclaiming is what an examiner punishes first.
- **Compared set (decided 2026-09-22; the scope of Chapters 1–5):** Strategy 0 geometric tracking without compensation (baseline); Strategy 1 pure INDI (reactive: measures the residual from rotor speeds and acceleration); Strategy 2 geometric control with a learned residual (Neural-Swarm family); Strategy 3 feedback linearisation with a flatness-preserving residual (**not implemented yet**: the authors' code has not been received; describe as planned or as a reference method only). **Not compared** (supervisor decision): NA-INDI / LINDI hybrid, residual RL, learning-based MPC; they appear only as related work (Chapter 2). Their reserve text `docs/thesis/reserve/advanced_strategies_reserve.tex` is not part of the build. A numbered strategy above 3 in the main chapters is a defect.
- **Fairness rule (current):** every strategy is its own controller, tuned for itself on the hardware gate, gains then frozen before the comparison flights. "Identical conditions" = identical trajectories, formations, hardware, separations, estimator and residual definition, **not** identical control structure or one gain set. Text speaking of "one shared control-law substrate" or "one shared gain set" is stale.
- **RQ1–RQ4** are defined in Chapter 1 and `docs/15`. **Chapter 1 lists three contributions** (controlled comparison; quantification of the summed-model transfer penalty; engineering account of cost) plus the apparatus; `docs/19` has a longer list of five claims. Chapter 1 is authoritative; report any drift between the two. RQ3 is the *size of the transfer penalty* of a summed per-neighbour model from two to three vehicles, **not** a test of whether superposition holds (already published as not exact).

# CODE ACCESS — what the code settles and how to use it

You can read the repository. **Order of authority:** (1) for *what we implement* (equations, constants, gains structure, network, rates, logged variables): the source code; (2) for *what a paper says*: the paper PDF; (3) the thesis draft (to be checked against 1 and 2); (4) every `docs/*.md` note is **not** a source.

| File | Settles |
|---|---|
| `flying_drone_stack/firmware_app/src/lib.rs` | the control law: position loop (`controller_step`), geometric attitude law (`geometric_step_ref`), INDI increment, residual terms and the residual sign (`g_indi_res_sign`, near the assembly of `f_d`), integral terms and their enable flags (`ENABLE_POSITION_INTEGRAL`, `ENABLE_Z_INTEGRAL`, `ENABLE_ATTITUDE_INTEGRAL`), filters, clamps, loop rates, the call of the network (`rnn_predict`) |
| `flying_drone_stack/firmware_app/src/residual_nn.rs` | the onboard network: architecture, input gate, neighbour limit, output clamp, units (the header comment is a reliable description) |
| `flying_drone_stack/firmware_app/traj_iface.c` | parameter groups and their default values, logging groups |
| `flying_drone_stack/tools/usd_study_config.txt` | the recorded variables and rate |

Rules for using the code:
- **Read-only.** Never run, modify or "fix" the code. Cite it as file, function and line. Do not copy code comments about flights, crashes or lab sessions into the thesis.
- **The repository also contains lab, simulation and replay result documents** (everything under `docs/lab_sessions/`, `docs/meetings/`, and `docs/71`, `72`, `74`, `76`, `77`, `docs/sil_study_screening.md`, `experiments/`, any "results", "replay", "SIL" file). They are **off-limits as sources and as content** for the draft in this round (H1). You may use the code, the PDFs, the thesis chapters and the reference documents named in the attachment list only.
- **When the code and the draft disagree**, do not silently change either. Put it in the defect log with the code location and state precisely what the code does. Design facts the code settles unambiguously (network architecture and size, input gate, output, evaluation rate, recorded variables, loop rate) are corrected to the code (APPLY, `% CHANGED: code, file:function`). Anything that touches the residual sign, which integral terms the thesis describes, a claim about scope or an argument is a DECISION row: text unchanged, `% DECISION` comment (H4).
- **Verification checklist for PHASE 0, repeated in PHASE 4 for the target chapter** — compare every item with the code and report the result: (a) network architecture, parameter count, input gate, neighbour limit, output clamp, output dimension, evaluation rate, units; (b) the residual sign and which configuration uses which; (c) which integral terms exist (position, vertical-only, attitude) and which are enabled — Chapter 4 writes a full position integral and an attitude integral; (d) the INDI increment and the filters it uses; (e) control-loop rates (the draft says 500 Hz); (f) the number and rate of recorded variables (the draft says 35); (g) the scenario count and parameters of the formation library.

# HARD RULES (violating one fails the whole run)

**H1. No results.** No number, statement or table from a flight, a lab session, a simulation run or a replay of this project. No "we observed / we found / the controller achieved". Chapter 5 describes the design and protocol as a standing design or as planned (present or future tense) and must not claim in the past tense what was done or found; Chapters 6–9 stay skeletons with placeholders. You may improve structure, metric definitions, the expected form of tables and figures, and hypotheses (falsifiable), never their values.

**H2. Nothing invented.** Never invent a reference, author, year, venue, DOI, page, quotation, dataset, number or experimental detail. A literature statement must be supported by the source PDF (PHASE 1 onward) — your own reading, with section and quotation; a statement about *our implementation* must be supported by the code (file, function, line). If you believe something is true but cannot trace it, put it on the `[VERIFY]` list and leave the sentence unchanged or weaker with a `% VERIFY:` comment. Never *strengthen* a claim. Never claim to have read what you did not: state exactly which files you read in full, partially or not at all.

**H3. Citation discipline.**
- Cite the **primary** source. Never cite the author's own notes (`docs/*.md`, `papers/summaries/`) as a source; they are working notes and have been wrong three times.
- Keep apart: *Hsieh et al. 2026* (flatness-preserving residual, two vehicles, off-board controller) vs *Hsieh et al. 2025* (online adaptation / L1 KNODE-DW MPC, three vehicles); *Neural-Swarm (2020)* vs *Neural-Swarm2 (2022)*; Lee, Leok, McClamroch CDC 2010 is the primary geometric-control citation (no PDF held; the arXiv companion for equation-level checks only); Tal and Karaman 2018 primary for INDI (Smeur 2017 for the cascaded variant).
- The recurring fault was **a correct number attached to the wrong thing** (wrong axis, training set, platform, experiment, an abstract-only figure presented as a body result). Treat every number with its **context**: what, which platform, trained how, simulation or hardware, abstract or body.
- Do not write "nobody compares controllers for close-proximity flight": false (Abro et al. 2025 and Gu et al. 2025 exist). The surviving narrower claim is hedged with "to the best of our knowledge".

**H4. The equations are the code.** Chapters 3 and 4 state their equations are those implemented in the flight code, so **verify each equation, constant and design number against the code** (see CODE ACCESS). Do not change an equation, sign, frame or symbol silently. Report each mismatch with the code location; correct code-settled design facts (APPLY); for anything about the residual sign, the integral terms, or a derivation you merely suspect to be wrong, classify a DECISION row with the derivation and leave the text unchanged with a `% DECISION` comment, unless the fix is typographical. Frame convention **z-up (ENU), gravity along $-e_3$**; the residual is **subtracted** in the control law (Chapter 3, subsection "Sign convention in the control law", states why and the falsifiable consequence of the opposite sign; the code has a configurable sign, so the wording is an author decision).

**H5. Implementation honesty.** Chapter 4 separates, per strategy, *what we implement* from *the reference method*; never merge them, never describe an unimplemented strategy as existing, never state which of several of our own INDI implementations the study will use (undecided).

**H6. Keep the machinery.** Keep labels, `\ref` / `\cite` keys, macros and open `\todo` markers ("Provenance" notes stay). Do not touch `references.bib` except through the "proposed references" output. LaTeX must compile with the attached preamble. Units with `\SI{}`. Chapters 6–9 are referred to by name in prose, not by `\ref`, until they exist.

**H7. Scope of change.** Improve **what exists**: add explanation, definitions, worked examples, tables, TikZ or described figures, signposting, caveats; restructure within a chapter. You may **not** add experiments, strategies, claims about the project's own results, or research questions.

**H8. Systematic means exhaustive.** When a phase says "every", it means every: every citation, number, symbol, equation, section, figure, table, `\todo`. If the context window cannot hold the task, say so and process in numbered batches, listing what is done and what is left. Never sample and call it complete.

**H9. Lab status is not part of this round.** The draft already contains passages that report the *progress of the project's own campaign* (for example Chapter 5, paragraph "Campaign status (September 2026)": flights executed, vehicle names, merges, manifest file names, "indicative comparisons exist"; and the "Status" column of the strategy summary table in Chapter 4: "flying", "implemented, unflown"). These are not results but they are lab-derived and they are stale. Flag every such passage in PHASE 0 (list with location). In PHASE 3 classify each such passage as a DECISION row, with these options: delete it; replace it by a design-level statement in the future tense ("the campaign will ..."); or keep a neutral implementation-status wording that makes **no flight claim** ("implemented in the flight code" / "not implemented"). Never replace it with new status or numbers of your own. Internal file names, manifests, session dates and vehicle nicknames do not belong in the thesis text.

# CRITERIA — score each chapter 0–5 before and after (5 = an examiner finds nothing to criticise)

1. **Correctness:** every statement, equation, definition and number right and consistent with sources and other chapters; units and frames consistent; no stale scope.
2. **Completeness:** definitions before use, assumptions and limits stated, each claim supported, nothing the thesis later relies on is missing.
3. **Depth and detail:** derivations shown where the reader would otherwise have to trust the author; the "why" of each design choice; alternatives mentioned and rejected with reasons.
4. **Academic standard:** claim → evidence → limitation; precise, consistent terminology; hedging proportional to evidence; no marketing words; signposting; stand-alone captions; consistent notation and citation form; honest limitations; Master's level, not padded.
5. **Explainability for students:** intuition before formalism; a worked or toy example per non-obvious idea; every symbol introduced once in words at first use; jargon defined; "what this means in practice" after dense paragraphs; no step left that the target student would not make.
6. **Related work, coverage and referencing:** every family relevant to the question covered and fairly represented (what, platform, evidence, limits), positioned against the question; the gap argued from the literature; references complete and correct; missing work identified (proposed, never added as verified).
7. **Consistency across chapters:** strategy numbering, names, symbols, numbers; Chapter 1's promises (RQs, contributions, outline) equal what Chapters 2–5 deliver and 6–9 will report.
8. **Reference correctness (zero tolerance):** each in-text claim → the right paper → the right thing within the paper.

# STYLE

British English. Impersonal, precise. Definitions in the sentence of first use; abbreviations spelled out once; symbols as in the notation section of Chapter 3. One idea per sentence, one point per paragraph; each section says what it does and why it is here. No filler, rhetorical questions, "obviously / clearly / trivially". Numbers always with context. Figures and tables: caption that stands alone, referenced and explained in the text; never invent data to fill one.

# FACT SHEET (project facts through 2026-10-13; for consistency, never results)

**If the fact sheet and the code disagree, the code wins**; report the disagreement (it means the fact sheet is stale).

- Platform: Crazyflie 2.1 Brushless ≈ 41 g; rotor speed from DShot telemetry or an optical RPM deck (needed to reconstruct applied thrust; without it the residual reads zero); micro-SD deck for onboard logging (500 Hz; the current configuration records 96 variables, an earlier draft passage says 35); onboard extended Kalman filter fed by 100 Hz motion capture; at least two vehicles (three for the three-robot scenarios).
- Volume ≈ 2 m × 4 m × 1.7 m tracked (not yet tape-measured: open `\todo`); operational floor 0.30 m (0.10 m for one near-ground scenario).
- Formation library: 16 frozen scenarios (A1–A8 two-robot, B1–B3 three-robot, C1–C5 other), with null scenarios for the measurement floor.
- Residual: $a_{\mathrm{res}} = a_{\mathrm{measured}} - a_{\mathrm{model}}$, model from rotor speeds; logged under every controller.
- Strategies 0–2 are implemented in the project's flight code, Strategy 3 is not. The study has not been run in the thesis' timeline: the draft reads as written before the results.
- **Strategy 2 network (current implementation; Chapters 3 and 4 may describe an older one):** a port of the Neural-Swarm2 reference architecture since 2026-09-14: $\phi$-network 6 → 25 → 40 → 40 → 20 on the neighbour's relative position and velocity, a separate ground-effect branch (4 inputs), $\rho$-network 20 → 40 → 40 → 40 → 1 with a **scalar vertical force** output, 19,297 weights in total (separate small / large vehicle sets, only "small" is used), a neighbour is summed in only if $|\Delta x| < 0.2$ m, $|\Delta y| < 0.2$ m and $|\Delta v_x| < 1.5$ m/s, at most three neighbours, output bound 8 m/s², evaluated at about 100 Hz (every tenth control step), weights flash-resident. Trained on two-vehicle flights; state no accuracy, no training-set size, no flight number. An earlier 987-parameter network (6 → 16 → 16 → 8 / 8 → 16 → 16 → 3) no longer exists. Statements in the draft about the architecture, parameter count, input guards, output dimension and evaluation rate must be compared with this and reported, not silently kept.
- **Residual sign:** in `lib.rs` the residual enters through `g_indi_res_sign`; the firmware default is `+1` (the behaviour flown since the July campaign), `-1` is called the "derivation-correct sign" in the code comments. The study configurations set `-1` for the geometric controller with the network and `+1` for our INDI. Chapter 3 states that the residual is subtracted. How to reconcile the text with this is an **author decision**: report it prominently with the code location, do not resolve it.
- **Loop rates and logging:** the thesis says "control loop at 500 Hz" and "35 variables recorded"; the code comments state that the loop runs at 1000 Hz with some services at 500 Hz, and the current logging configuration records 96 variables. Check both against the code and report.
- **Integrals (code state):** the vertical-only position integral is enabled (`ENABLE_Z_INTEGRAL`), the vector position integral and the attitude integral are disabled constants. Chapter 4's equations carry both a full position integral and an attitude integral: report the mismatch with the code location and let the author decide the wording.
- Open items in the text: tape-measure the volume (Chapter 5); revise Chapter 5 to the past tense after the campaign; two "provenance" notes in Chapters 1 and 2 document audits and are not pending work.
- Past incidents (why H2/H3 exist): a note said a hybrid showed "limited benefit" (wrong); its correction said "lowest tracking error" (also wrong: small, condition-dependent margin; the paper's headline is sensor elimination); a paper's "simulation and hardware experiments" were a simulator with a Jetson in the loop, not flight; one lateral figure was attached to 3D.

# INPUTS (paths relative to the repository root `flying_robot_course/`; read them yourself)

**The thesis (target):** `docs/thesis/ch1_introduction.tex`, `ch2_related_work.tex`, `ch3_system_modelling.tex`, `ch4_control_architectures.tex`, `ch5_experimental_setup.tex`, `ch6_results_2robot.tex`, `ch7_results_multirobot.tex`, `ch8_discussion.tex`, `ch9_conclusion.tex`, `thesis_draft.tex`, `preamble_draft.tex` (all in `docs/thesis/`), and `docs/references.bib`.
**Reference documents (context only, not sources of truth):** `docs/15_Problem_Statement_and_Research_Questions.md`, `docs/19_Contribution_Statement.md`, `docs/18_Strategy_Descriptions.md`, `docs/04_Unified_Residual_Wrench_Model.md`, `docs/10_Formation_Library.md`, `docs/05_Experimental_Protocol_2Robot.md`.
**Existing citation notes (use only in PHASE 1B; never in PHASE 1):** `docs/_archive/citation_notes/20_Verified_Claims.md`, `17_Source_Ledger_and_Citation_Discipline.md`, `16_Related_Work_Structure.md`, `THESIS_REVIEW_BRIEF.md` (stale on scope).
**Code (read-only):** `flying_drone_stack/firmware_app/src/lib.rs`, `flying_drone_stack/firmware_app/src/residual_nn.rs`, `flying_drone_stack/firmware_app/traj_iface.c`, `flying_drone_stack/tools/usd_study_config.txt`; optionally `crazyswarm2/crazyflie_examples/crazyflie_examples/formations/scenarios.py` if that repository is available.
**Source PDFs (18 held; read each in full in PHASE 1):** `docs/papers/*.pdf` — keys: shi2020neuralswarm, shi2022neuralswarm2, gielis2023aggregate, smith2023so2equivariant, hsieh2026flatness, yang2025flatnessresiduals, cobobriesewitz2026lindi, zhang2024proxfly, chee2024knodedwmpc, hsieh2025onlineadaptation, kiran2025downwash, bauersfeld2024airflow, tal2018indi, smeur2017cascadedindi, lee2010geometricse3 (arXiv companion), li2023ndpnmpc, shankar2023docking, gu2025comparative. **No PDF held** for `lee2010geometrictracking` (Lee, Leok, McClamroch, CDC 2010, the primary geometric-control citation) and `abro2025fractional`: write nothing about their content beyond what the thesis draft already says, and quote nothing. Do **not** use `docs/papers/summaries/` (our own notes; they caused misattributions).

# EXECUTION — ONE CONTINUOUS RUN, SEVEN PHASES, NO WAITING

You execute **all** phases below in one continuous session, in order, without stopping to ask for approval. Work through them systematically; each phase's output is the input of the next.

**Where outputs go.**
- If you can write files: write every phase output to `docs/thesis_revision/` (names given per phase) and the revised chapters to `docs/thesis_revision/revised/chN_*.tex`. **Never overwrite `docs/thesis/*.tex`, `docs/references.bib` or any code.** Print a short status line in the conversation after each phase (phase, files written, number of defects, number of open decisions).
- If you cannot write files: emit each phase output in the conversation, in order, in full, and continue until the final deliverable. If you hit an output limit, stop at a section boundary, say exactly where, and continue when asked ("continue"); never abbreviate, never summarise instead of delivering.

**The approval gate becomes a decision filter.** There is no human between phases. Therefore, in PHASE 3, classify every planned change as **APPLY** (safe: errors that the code or a paper unambiguously settles, missing definitions, derivations, examples, structure, signposting, consistency, reference-context fixes) or **DECISION** (needs the author: the residual-sign wording, which integral terms the thesis describes, what to do with lab-status passages, whether to add or drop a related-work item, anything that changes the thesis' scope or an argument). Facts that the code settles unambiguously — the network's architecture, parameter count, input gate, output dimension and rate, the recorded-variable count, the loop rate — are **APPLY**: correct the text to the code and mark `% CHANGED: code, <file:function>`. In PHASE 4 apply **only** APPLY rows. For DECISION rows leave the text unchanged, insert a `% DECISION <id>:` comment at the location with the options, and list it in the final report. This is the single most important safety rule of this run.

**Persistence of context.** Maintain, and re-read before every chapter rewrite, three working artefacts: the claim inventory (PHASE 0), the per-paper source records (PHASE 1) and the plan (PHASE 3). Do not rely on memory of the draft; re-open the chapter text before rewriting it.

**Order of the rewrite.** Chapters 3, 4, 5 first (they hold the code-dependent facts), then 2, then 1 (the introduction promises what the body delivers), then the Chapter 6–9 skeletons last. After each chapter, re-check its citations (PHASE 4 pass D) before moving on.

**Final deliverables** (name them in the last message): (1) `stage0_draft_audit.md`; (2) `stage1_sources.md` (one record per paper); (3) `stage1b_reconciliation.md`; (4) `stage2_coverage_gaps.md`; (5) `stage3_revision_plan.md` with APPLY / DECISION classification; (6) the revised chapters with `% NEW:`, `% CHANGED:`, `% DECISION:`, `% VERIFY:` markers; (7) a `proposed_references.md` (UNVERIFIED) and a list of suspected errors in `references.bib`; (8) `stage5_consistency.md`; (9) `stage6_final_report.md`; (10) `author_decisions.md`: every DECISION row with options and your recommendation, and every `[VERIFY]` item with the exact place to check.

# THE PHASES

## PHASE 0 — Whole-draft comprehension and audit (no edits)

Output: `stage0_draft_audit.md`

Read **the entire thesis draft** (Chapters 1–9, preamble, bibliography) from first to last page before writing anything. Then produce `stage0_draft_audit.md` with these parts, each complete (H8):

1. **Reading record:** every file, read fully / partially / not at all.
2. **Thesis map:** per chapter, its job in one sentence, its section tree, word count, tense, state (draft / skeleton).
3. **Argument chain:** the thesis' argument as numbered steps (problem → gap → question → method → expected evidence), each with the section where it is made. Mark any step that is asserted but not supported.
4. **Promise–delivery matrix:** every RQ, contribution and outline statement of Chapter 1 against where Chapters 2–5 deliver it and where Chapters 6–9 will report it. Gaps and orphans.
5. **Claim inventory:** every factual claim about the literature (≈ one row per sentence): chapter and location, claim, cited key(s), context given (platform / vehicles / separation / sim vs hardware / training), hedge level. This table is the work list for PHASE 1.
6. **Number inventory:** every number with unit and its context, and where it is repeated (are repetitions identical?).
7. **Notation table:** every symbol: meaning, first use, definition location, conflicts (same symbol two meanings, two symbols one meaning).
8. **Terminology list:** every technical term, where defined, where first used before definition.
9. **Cross-reference check:** forward and backward references, promises of the type "as shown in Chapter N", citations used but unresolved, bibliography entries unused.
10. **Stale-scope scan:** strategies above 3, "shared substrate", "shared gains", tense mismatches, statements that contradict the current fairness rule or compared set; **every lab-status passage (H9)** and **every statement derived from the project's own simulations or lab sessions (H1)**; **design facts that disagree with the fact sheet** (network architecture, parameter counts, guards, rates, variable counts); **internal vocabulary** that does not belong in a thesis (file names, vehicle nicknames, S0–S3, C.1 / C.4, "our notes"). Each with its location.
11. **Equation inventory:** each equation block, its claim, whether it is derived or quoted, the cited source, frame and sign convention consistency (z-up, residual subtracted).
12. **Open `\todo` and `% VERIFY` markers.**
13. **First impressions:** the ten weakest places of the whole draft, in order, one line each (no fixes yet).
14. **Code-vs-draft check:** every item of the verification checklist (CODE ACCESS (a)–(g)) with the code location, what the code does, what the draft says, and the verdict.
Do not propose changes in this stage. Do not read the source papers in this stage.

## PHASE 1 — Source reading, paper by paper, from the PDFs (no edits; do NOT use `20_Verified_Claims.md` or any `docs/*.md` note here)

Output: `stage1_sources.md`

Use the claim inventory from PHASE 0 and all 18 PDFs. **Do not use `20_Verified_Claims.md` or any `docs/*.md` note in this stage**; the point is an independent reading. For each paper, read the **whole PDF** (all pages, appendix included) and write one record:

1. **Reading record:** pages read; anything unreadable (figures, tables) named.
2. **Bibliographic check:** authors, title, year, venue against `references.bib` (note arXiv vs published year).
3. **What the paper does:** problem, method, in five sentences a student could follow.
4. **Setup facts, each with a location (section / page / table):** platform and mass, number of vehicles, separations tested, trajectories, controller type and where it runs (on-board / off-board), training data (amount, vehicles, distribution), simulation vs hardware (be strict: what exactly flew), baselines, metrics, headline results **with their exact context** and whether each is in the abstract only or in the body.
5. **Authors' stated limitations** and assumptions (verbatim where short).
6. **Quotable claims:** up to 12 sentences in the paper's own words with page or section, the ones the thesis is most likely to rely on.
7. **What this paper does NOT show** (frequently misattributed things).
8. **The thesis' use of it:** for every row of the PHASE 0 claim inventory that cites this paper: verdict `supported / supported with changed context / overstated / wrong / cannot tell`, with the evidence and the corrected formulation.
9. **Confidence and gaps:** what you could not verify and why.
If a paper is not attached, say "no PDF held" and write nothing about its content (`lee2010geometrictracking`, `abro2025fractional`).

## PHASE 1B — Reconcile with the existing ledger

Output: `stage1b_reconciliation.md`

Use your PHASE 1 records and `docs/_archive/citation_notes/20_Verified_Claims.md`. Compare your independent readings with the ledger paper by paper. Output `stage1b_reconciliation.md`: (a) agreements; (b) every disagreement with both versions, the evidence for each, and your recommendation; (c) claims in the ledger that you could not find in the paper; (d) claims you found that the ledger lacks and the thesis should perhaps use. Do not average: where they differ, say which is right and why, or mark it for the author.

## PHASE 2 — Literature coverage and gap analysis (no edits)

Output: `stage2_coverage_gaps.md`

Using the 18 PDFs' own reference lists and related-work sections:
1. **Family map:** the families of work relevant to the thesis' question (aerodynamics of rotor interaction, close-proximity formation control, INDI / NDI foundations, geometric control, learned residual and dynamics models, flatness, MPC-based approaches, benchmarking and evaluation practice in aerial robotics, safety and uncertainty), and for each family which held papers represent it.
2. **Coverage matrix:** families × (covered in Chapter 2 / thin / absent), with the missing foundational or comparison works you are **confident exist**. For each: full bibliographic data, why it belongs, where in the thesis, confidence, status `UNVERIFIED — author must check`. Never present these as verified citations.
3. **Gap claim stress test:** attack the thesis' gap statement with the held corpus and your knowledge; list works that would weaken it and the narrowest defensible wording.
4. **Background sufficiency:** for each background topic the student reader needs (SE(3) geometry, rotation matrices vs quaternions, INDI derivation, deep sets, differential flatness, residual force estimation, filtering and delay) judge whether Chapters 2–4 give enough; list what is missing, at what depth.
5. **Evaluation practice:** how the held papers evaluate (metrics, baselines, separation, simulation vs hardware); what the thesis' Chapter 2 §2.8 should say that it does not.
6. **Ranked list** of the 15 most valuable additions or corrections to the related-work and background material.

## PHASE 3 — Revision plan (no edits; classify every row APPLY or DECISION)

Output: `stage3_revision_plan.md`

Output `stage3_revision_plan.md`, **no edits to the thesis**, every change row classified `APPLY` or `DECISION` (see EXECUTION):
1. **Target outline:** the section tree you propose for Chapters 1–5 (and the skeletons of 6–9), changes marked.
2. **Change list per chapter:** every intended change as a row: id, location, category (error / missing definition / missing derivation / missing example / structure / related work / consistency / reference), reason with the PHASE 0/1/2 evidence, severity, risk (could it change a claim or an equation?), dependency on other rows.
3. **Additions list:** new material to be written (background sections, worked examples, figures, tables), with a length estimate and the sources it will rest on.
4. **Reference actions:** entries to correct, to add (UNVERIFIED), to remove, claims to re-attach to the right context.
5. **Decisions needed from the author:** only those that block a correct rewrite, each with options and your recommendation.
6. **Rejected ideas:** what you considered and will not do, and why (scope, H1, H7).
7. **Order of work** and expected size of each chapter after revision.
Do not stop: continue to PHASE 4 with the APPLY rows only.

## PHASE 4 — Chapter rewrite (order: 3, 4, 5, 2, 1, then 6–9; APPLY rows only)

Output: `docs/thesis_revision/revised/chN_*.tex`

For each target chapter in the order above (the other chapters read-only; the PDFs cited by that chapter re-opened), execute **only** the APPLY change rows for that chapter, plus defects you newly find (flag them as such). Work in passes and show each:
- **A. Claim re-check:** every claim of the chapter against the PHASE 1 record and the PDF, and every statement about our implementation against the code; table.
- **B. Defect log** (new findings only, with severity).
- **C. Rewrite:** the complete revised chapter as compilable LaTeX. Minimal-edit principle: keep the author's good sentences. Operations in order of preference: fix errors; add definitions and assumptions; add the "why"; add a worked or toy example (textbook or clearly illustrative values, never project data); add one "key idea" sentence per section; tighten; reorder within the chapter. Mark substantive new paragraphs with `% NEW:` and changed claims with `% CHANGED:` + reason + plan row id.
- **D. Citation audit:** for every `\cite`: key exists; the paper makes the stated claim (quote and location); number attached to the right context. A failed row is a blocker.
- **E. Examiner simulation:** the 12 hardest questions an external examiner would ask about this chapter and where the revised text answers each (at least two on the gap claim, two on fairness or scope, one per equation block, one on limitations).
- **F. Student-reader test:** every place a student of the reader profile would stumble and how the revised text resolves it.
**Chapter briefs.** *Ch. 1:* motivation from the physical problem without hype; problem precise; gap narrow and audited; strategies one paragraph each; RQs falsifiable and answerable by Chapter 5; contributions consistent with `docs/19`, comparison not method; scope; outline equals reality. *Ch. 2:* families by how the interaction force is obtained; every number with context; §2.8 methodological, not a criticism of authors; background sufficient for the student without becoming a textbook. *Ch. 3:* wrench model, §3.1 notation authoritative; the network description (architecture, parameter count, guards, output, rate) is corrected to the code (APPLY); the residual-sign wording is a DECISION row, why the residual measurement fails (rotor-speed source, filters, delay, thrust-model mismatch), sign convention, deep-sets model, superposition, conditioning, normalisation, assumptions; every derivation step shown; magnitudes with illustrative, clearly labelled values. *Ch. 4:* per strategy reference method / what we implement (checked against the code, including which integral terms exist and which are enabled) / deviations (the Strategy 2 deviations paragraph describes an older network) / expected failure modes / tuning surface; position and attitude loops against Lee and Tal and Karaman; fairness section against the current rule and what it does not guarantee; no strategy above 3; no INDI implementation chosen. *Ch. 5:* platform, volume, library, measurement and logging, hardware gate, data collection, comparison campaign, metrics (each defined mathematically with window and detector), role of simulation, threats to validity with mitigation or "not mitigated"; design and protocol wording (present or future tense, no past-tense claims of what was done or found); the two statements derived from the project's own simulations (the "dry run" paragraph and the "role of simulation" findings) are flagged per H1; the "Campaign status" paragraph and any status column are handled per H9; design numbers such as the count of recorded variables are corrected to `flying_drone_stack/tools/usd_study_config.txt` (the draft says 35 variables, the current logging configuration records 96) as a design statement (APPLY, `% CHANGED: code`). *Ch. 6–9 (last run):* structure and placeholders only: which table, figure and statistic each section will hold, how each RQ is answered and what a negative answer looks like, a discussion section on where a frozen gain appears to have disadvantaged a strategy, limitations, conclusion mapped to RQ1–RQ4; no numbers.
**Output order:** verdict (5 lines); rubric table before/after; the passes above; **revised chapter in one code block** (never abbreviate, never "unchanged"; if you must stop, stop at a section boundary and say where); proposed additional references (UNVERIFIED) and suspected errors in `references.bib`; `[VERIFY]` list for the author (exact place to check); change log keyed to plan rows; DECISION rows left open for the author.

## PHASE 5 — Cross-chapter consistency and citation re-audit (on the revised chapters)

Output: `stage5_consistency.md`

Use the revised chapters and the PHASE 0 audit. Repeat the PHASE 0 inventories on the **revised** draft and diff against PHASE 0: promise–delivery matrix, notation table, number inventory (identical repeats?), terminology (defined before use?), cross-references, equation inventory, tense map, stale-scope scan, unused or unresolved citations. Re-run the citation audit for **every** `\cite` of the whole thesis. Output `stage5_consistency.md`: a defect list by severity with exact locations and fixes, plus the build check you can make from the text (undefined labels, duplicate labels, missing bibliography keys).

## PHASE 6 — Examiner and student simulation on the whole revised thesis (no edits)

Output: `stage6_final_report.md`

(1) As **external examiner**: read it once as a hostile but fair reader; write the report an examiner would write (summary, strengths, weaknesses by severity, the 20 questions for the viva, a recommendation of what must change before submission). (2) As **student reader**: list every place where you, with the stated profile, needed outside knowledge, and whether the thesis supplies it. (3) As **supervisor**: the ten edits that would raise the thesis most, in order. Do not suggest results, new experiments or new strategies.

# START

Begin with PHASE 0 now. State at the start which inputs you can access (thesis files, code, PDFs) and which you cannot; if anything required is missing, say exactly what, work with what you have, and mark every consequence.

===== END MASTER PROMPT =====

# Review summary — Professor + Student passes — 2026-10-04

**Status:** read-only review. Two roles were run separately and in order (Professor first, then Student). No `.tex`, `.bib`, `WRITING_RULES.md` or other existing file was modified. Nothing was committed or pushed. No hardware was touched. Builds were made in a temp directory outside `docs/thesis/`.

Companion files: `REVIEW_professor_2026-10-04.md` (43 findings P-001…P-043) and `REVIEW_student_2026-10-04.md` (40 findings S-001…S-040).

## Step 3 — Reconciliation

### 3a. Where the two roles' needs conflict, and a middle option

| # | Professor wants | Student wants | Middle option (keeps every qualifier and locked number) |
|---|---|---|---|
| 1 | Every number with its source and qualifier (WRITING_RULES §3); the Hsieh figures stay as they are. | Fewer numbers per paragraph; the Hsieh paragraph is hard to hold (S-009). | Keep the content, change the form: a small table of the Hsieh figures (claim, source part, value, qualifier) plus a two-sentence reading. Nothing is deleted. |
| 2 | Exact notation, a rename of the Ch.4 `a_des`, subscripts for the two gain sets (P-011, P-018). | Fewer symbols and a plain-words sentence next to each equation (S-019, S-015). | Keep the symbols, add a symbol table (symbol, meaning, unit, first defined in) and one plain-words sentence after the three key equations. |
| 3 | More specification in Ch.5: statistical plan, scenario parameters, hypotheses, alignment method (P-023, P-030, P-022). | Ch.5 is already hard to use; the status paragraph is unreadable (S-027). | Put numbers into tables and an appendix; keep the Ch.5 prose short. Replace the status paragraph by a dated table of what has been flown. |
| 4 | Remove repository-internal pointers and the hybrid result slot (P-040, P-035). | Wants to be able to find the object that a sentence refers to (S-037). | Move artefact names to one appendix 'Reproducibility and artefacts' and refer to it by section; describe deferred ideas in words, not project numbers. |
| 5 | WRITING_RULES §1 'do not write as if C.0/C.1 is done' is superseded by the repository state (P-042). | A reader needs to know what has been done and what is planned; tense changes between Ch.1–5 and Ch.6–9 are noticeable (S-035). | Update the rule text (not done here: report only) to 'report design and protocol; one dated campaign-status table in Ch.5; no results before Ch.6'. |
| 6 | Keep the word 'uncompensated' for Strategy 0 because it carries the doctrine (it records but does not use the residual). | Three names for one controller (S-026). | Pick one name in Ch.4 (for example 'geometric baseline'), define 'uncompensated' once as 'does not use the residual', and use the same name in Ch.1–Ch.6. |
| 7 | Rigour about status (Strategy 2 'implemented, unflown' vs the hardware attempt, P-019). | Column meanings in Table 4.1 unclear (S-018). | Define the status vocabulary in the caption (planned / implemented / flown / validated) and date the table. |

The only direct conflict with `WRITING_RULES.md` is P-042 (the rule is stale). No suggestion in either report removes a qualifier, simplifies a locked number, or swaps a technical term for a vaguer one.

### 3b. Findings raised independently by both roles (highest confidence)

| Topic | Professor | Student | Note |
|---|---|---|---|
| Ch.1 promises hypotheses that do not exist | P-001 | S-001 | Both MUST-FIX. Highest confidence. |
| Ch.9 says Strategies 0–3 'were implemented' | P-038 | S-034 | Both MUST-FIX. One-line fix. |
| `a_des` has two meanings (Ch.3 vs Ch.4) | P-011 | S-013 | Both SHOULD-FIX. |
| `a_meas` accelerometer vs 'differentiated' | P-012 | S-014 | Both SHOULD-FIX. |
| Table 2.1 overflows and is clipped | P-008 | S-008 | Both SHOULD-FIX. |
| `omega_des` undefined; `K_R`/`K_omega` change meaning; Lee symbol map missing | P-018 | S-021 | Both SHOULD-FIX. |
| 'Half' vs 'two' of the six inputs | P-032 | S-031 | Both SHOULD-FIX; Professor adds the 'four of six' count. |
| No numerical scenario parameters in Ch.5 | P-030 | S-030 | Both SHOULD-FIX. |
| `Chapter~\ref{sec:...}` prints 'Chapter 7.2' | P-037 | S-035 | Both SHOULD-FIX. |
| 'Strategy 4 / 5/7' numbering that the thesis never defines | P-035 | S-036 | Both SHOULD-FIX; Professor's concern is the doctrine, Student's is comprehension. |
| Repository-internal references (`docs/25`, 'Priority D/E') in thesis text | P-040 | S-037 | Both SHOULD-FIX. |
| The draft has no figure at all | P-043 | S-039 | Both SHOULD-FIX. |
| Campaign status paragraph in Ch.5 (stale for the Professor; unreadable jargon for the Student) | P-025 | S-027 | Different angle, same paragraph. |
| What exactly is the training target / which flights are differenced | P-016 | S-012 | Related, not identical: Professor asks for the label definition, Student for the solo reference flights. |
| Hsieh paragraph in Ch.2 | P-006 | S-009 | Related: Professor found a factual mismatch (sevenfold), Student found the paragraph too dense; both point to splitting or tabulating it. |
| Status cell and column meaning in Table 4.1 | P-019 | S-018 | Related: Professor wants the status updated, Student wants the columns explained. |
| Vehicle identity / 'role' vocabulary | P-029 | S-033 | Related: Professor sees a confound, Student does not know what 'role' means. |

The first 12 rows are the same issue seen from both roles; the last five are related but not identical. All 17 pairs were identified after both files were written. Caveat: both roles were run by the same agent, one after the other, so 'independent' means 'from a separate reading and a different brief', not two different people.

### 3c. Findings seen by only one role

- **Professor only (need the repository or the papers; a reader of the PDF cannot see them):** P-021 (z-gain history vs 'frozen before the dataset'), P-022 (time-alignment method), P-023 (no statistical plan), P-036 (RQ3 fallback needs the third vehicle), P-016 (training label), P-017 (Strategy 1 variant), P-006 (sevenfold), P-007 (Li et al. onboard), P-013 (moment residual not logged), P-028 (RPM source), P-029 (vehicle identity), P-041 (stale README), P-042 (rule §1).
- **Student only (need a reader outside the field):** S-038 (no abbreviation/symbol list), S-028 (campaign phases C.0–C.4 undefined and colliding with C1–C5), S-002 (INDI/FBL/NN unexpanded), S-012 (no 'solo' reference flights described for 'differencing'), S-019 (INDI equation without intuition), S-015 (deep sets without picture), S-003 (Strategy 2/3 before introduced), S-032 ('residual rejection' metric is a separation error).

### 3d. The ten changes with the best value per effort

Effort: S = a few words/lines, M = a paragraph or a table, L = new section or a decision outside the text. Ordered by value divided by effort, not by severity.

| # | Change | Findings | Effort | Why it ranks here |
|---|---|---|---|---|
| 1 | Ch.9: replace 'Strategies 0–3 were implemented' by the true status (0–2 implemented, 3 specified, pending author code). | P-038, S-034 | S | Both roles MUST-FIX; a false sentence in the closing summary; one line. |
| 2 | Ch.1: either state the hypotheses (docs/15 H1–H3 exist; re-word H1 to Strategies 0–3) or correct the pointer 'stated in Chapter 2 and Chapter 5'. | P-001, S-001 | S–M | Both roles MUST-FIX; the minimal fix is one sentence, the proper fix is a short subsection that also feeds the statistical plan. |
| 3 | Ch.8–9: change `Chapter~\ref{sec:...}` to `Section~\ref{...}` (six places; point the RQ2 reference at `sec:setup-scenarios`). | P-037, S-035 | S | Mechanical; removes 'Chapter 7.2' from the PDF. |
| 4 | Ch.3: 'a_meas is differentiated' → 'filtered from the accelerometer signal' (force) and keep 'differentiated' for angular acceleration. | P-012, S-014 | S | One phrase; removes a conceptual error that an expert would stop at. |
| 5 | Ch.2: fix Table 2.1 (186 pt overfull) with `tabularx`/narrower columns. | P-008, S-008 | S | Layout only; the key evidence table of the chapter becomes readable. |
| 6 | Ch.2: correct two factual statements about papers: Hsieh 'sevenfold' (PDF says order of magnitude) and Li et al. 'off-board' (ran onboard); pin the arXiv version in the ledger. | P-006, P-007 | S | Two sentences; removes the only two MISMATCH items in the citation check. |
| 7 | Add a list of abbreviations and a table of symbols after the contents (seed from `flying_drone_stack/GLOSSARY.md`); expand INDI/FBL/NN/ROM at first use in Ch.1–2. | S-038, S-002, S-010, S-029 | M | Removes most of the Student's term problems at once. |
| 8 | Ch.3/4 notation hygiene: rename the Ch.4 `a_des`, define `omega_des`, give the two `K_R` sets different subscripts, add a symbol map to Lee's law. | P-011, P-018, S-013, S-021 | M | Both roles; a few lines of text and no result changes. |
| 9 | Ch.5: replace the campaign-status paragraph by a dated table of what has been flown; define phases C.0–C.4 once and rename either phases or scenarios C1–C5. | P-025, P-026, S-027, S-028 | M | Fixes staleness (Professor) and jargon (Student) together. |
| 10 | Ch.1/Ch.4: add a four-strategy paragraph (or a three-row table) before the RQs, and add the control-loop block diagram to Ch.4. | S-003, P-043, S-039, S-019 | M | Largest comprehension gain per page; the block diagram is also what an examiner expects. |

**Not in the ten, but must be decided before the C.4 campaign** (high value, high effort or outside the text): P-021 (which z-gain defines Strategy 0 and how the C.1 bank relates to it), P-023 (statistical plan, n per cell, decision rule), P-036 (RQ3 fallback without a third vehicle), P-022 (alignment text vs the merge tool), P-016 (training target), P-017 (which INDI variant is Strategy 1).

## Counts

| Role | MUST-FIX | SHOULD-FIX | NICE | Total |
|---|---|---|---|---|
| Professor | 6 | 31 | 6 | 43 |
| Student | 2 | 26 | 12 | 40 |
| **Both** | 8 | 57 | 18 | 83 |

| Role | Ch1 | Ch2 | Ch3 | Ch4 | Ch5 | Ch6 | Ch7 | Ch8 | Ch9 | Cross |
|---|---|---|---|---|---|---|---|---|---|---|
| Professor | 5 | 5 | 5 | 5 | 14 | 1 | 1 | 1 | 2 | 4 |
| Student | 7 | 4 | 6 | 9 | 7 | 0 | 0 | 0 | 0 | 7 |

- Citations checked against the source PDFs (Professor): **42** claims (VERIFIED 39, MISMATCH 2, UNVERIFIABLE 1); cite keys resolved: 20 of 20.
- Equations / derivations re-derived by hand (Professor): **10**.
- Terms audited (Student): **35** (defined at or before first use 4, only later 3, partly 1, assumed known 1, never 26).
- Defence questions (Professor): 15 (YES 3 / PARTLY 7 / NO 5). Reader questions (Student): 50.

## All findings

Status: **Open** = reported, not fixed (this review is read-only). 'Also raised by' links the same or a related issue in the other role's file (see 3b).

| ID | Role | Chapter | Severity | Status | Also raised by | Title | Location |
|---|---|---|---|---|---|---|---|
| P-001 | Professor | Ch1 | MUST-FIX | Open | S-001 | Chapter 1 promises falsifiable hypotheses that no chapter states | `docs/thesis/ch1_introduction.tex L145` |
| P-002 | Professor | Ch1 | SHOULD-FIX | Open | — | The gap sentence is wider than the gap the thesis closes (hybrid) | `docs/thesis/ch1_introduction.tex L94` |
| P-003 | Professor | Ch1 | SHOULD-FIX | Open | — | RQ4 does not list tuning effort, but Ch.4 and Ch.5 put tuning effort inside RQ4 | `docs/thesis/ch1_introduction.tex L136` |
| P-004 | Professor | Ch1 | SHOULD-FIX | Open | — | Ch.1 uses the 31 % figure without source or 'abstract' qualifier; the 9 cm size has no source | `docs/thesis/ch1_introduction.tex L98` |
| P-005 | Professor | Ch1 | NICE | Open | — | Stale header comments in Ch.1 ('Nothing has flown') | `docs/thesis/ch1_introduction.tex L10` |
| P-006 | Professor | Ch2 | SHOULD-FIX | Open | S-009 | Ch.2 attributes a 'sevenfold compute reduction' to Hsieh et al.; the PDF says 'an order of magnitude' | `docs/thesis/ch2_related_work.tex L296` |
| P-007 | Professor | Ch2 | SHOULD-FIX | Open | — | Li et al. are grouped with off-board controllers, but their NMPC ran on an onboard computer | `docs/thesis/ch2_related_work.tex L317` |
| P-008 | Professor | Ch2 | SHOULD-FIX | Open | S-008 | Table 2.1 (evaluation conditions) overflows the text width; the last column is clipped on page 16 | `docs/thesis/ch2_related_work.tex L420` |
| P-009 | Professor | Ch2 | NICE | Open | — | Smith et al. are internally inconsistent about the 36 %; the writing rule states one reading as fact | `docs/thesis/ch2_related_work.tex L217` |
| P-010 | Professor | Ch2 | NICE | Open | — | Abro et al. cannot be verified from the repository | `docs/thesis/ch2_related_work.tex L506` |
| P-011 | Professor | Ch3 | SHOULD-FIX | Open | S-013 | `a_des` has two different meanings in Ch.3 and Ch.4 | `docs/thesis/ch3_system_modelling.tex L161` |
| P-012 | Professor | Ch3 | SHOULD-FIX | Open | S-014 | Ch.3 says `a_meas` comes from the accelerometer and, a page later, that it is differentiated | `docs/thesis/ch3_system_modelling.tex L146` |
| P-013 | Professor | Ch3 | SHOULD-FIX | Open | — | The moment residual is said to be 'measured and logged', but no such channel exists | `docs/thesis/ch3_system_modelling.tex L89` |
| P-014 | Professor | Ch3 | NICE | Open | — | 'Approximately twice' for an inverted sign holds only in a linear, unsaturated steady state | `docs/thesis/ch3_system_modelling.tex L178` |
| P-015 | Professor | Ch3 | NICE | Open | — | The 2 m neighbour cut-off is justified by an unsupported statement about the noise floor | `docs/thesis/ch3_system_modelling.tex L265` |
| P-016 | Professor | Ch4 | SHOULD-FIX | Open | S-012 | The training target of the predictive strategy is not defined (raw `a_res` or interaction-only) | `docs/thesis/ch4_control_architectures.tex L244` |
| P-017 | Professor | Ch4 | SHOULD-FIX | Open | — | Strategy 1 is described as one controller; the repo has three INDI variants and no choice yet | `docs/thesis/ch4_control_architectures.tex L188` |
| P-018 | Professor | Ch4 | SHOULD-FIX | Open | S-021 | Symbols in the attitude loop: `omega_des` is undefined and `K_R`, `K_omega` change units between two equations | `docs/thesis/ch4_control_architectures.tex L113` |
| P-019 | Professor | Ch4 | SHOULD-FIX | Open | S-018 | Strategy 2 status and 'evaluated at the control rate' contradict the latest hardware attempt | `docs/thesis/ch4_control_architectures.tex L242` |
| P-020 | Professor | Ch4 | SHOULD-FIX | Open | — | Each family is represented by one home-made instantiation; the generalisation sentence is missing | `docs/thesis/ch4_control_architectures.tex L265` |
| P-021 | Professor | Ch5 | MUST-FIX | Open | — | The 'gains frozen before the dataset' doctrine is not what the repository shows for the geometric z-gain | `docs/thesis/ch5_experimental_setup.tex L242` |
| P-022 | Professor | Ch5 | MUST-FIX | Open | — | The time-alignment method described in Ch.5 is not the method the merge tool now uses | `docs/thesis/ch5_experimental_setup.tex L217` |
| P-023 | Professor | Ch5 | MUST-FIX | Open | — | No statistical plan: repeats, test, interval or decision rule are unspecified | `docs/thesis/ch5_experimental_setup.tex L272` |
| P-024 | Professor | Ch5 | SHOULD-FIX | Open | — | 'A1: nothing moves, so any relative displacement is interaction' is contradicted by the z-bias finding | `docs/thesis/ch5_experimental_setup.tex L116` |
| P-025 | Professor | Ch5 | SHOULD-FIX | Open | S-027 | Campaign status in Ch.5 is stale relative to the checklist (C.1 'has begun' / 'partly filled') | `docs/thesis/ch5_experimental_setup.tex L37` |
| P-026 | Professor | Ch5 | SHOULD-FIX | Open | — | Logged-channel count and the missing `e_R` note are out of date; the logging configuration changed after the first bank | `docs/thesis/ch5_experimental_setup.tex L189` |
| P-027 | Professor | Ch5 | SHOULD-FIX | Open | — | Train/test separation is stated, but 'held-out scenarios' are not defined and segments of one flight still share conditions | `docs/thesis/ch5_experimental_setup.tex L337` |
| P-028 | Professor | Ch5 | SHOULD-FIX | Open | — | The rotor-speed source is a first-order threat to `a_res` and INDI, but not in the threat table | `docs/thesis/ch5_experimental_setup.tex L56` |
| P-029 | Professor | Ch5 | SHOULD-FIX | Open | S-033 | Vehicle identity is confounded with role (cf5 always bottom, cf_second always top) and is not in the threats | `docs/thesis/ch6_results_2robot.tex L31` |
| P-030 | Professor | Ch5 | SHOULD-FIX | Open | S-030 | The scenario library has no numerical parameters in the thesis (separations, speeds, durations, n) | `docs/thesis/ch5_experimental_setup.tex L94` |
| P-031 | Professor | Ch5 | SHOULD-FIX | Open | — | C2 is marked a null condition (no interaction expected) but its purpose is the wake | `docs/thesis/ch5_experimental_setup.tex L132` |
| P-032 | Professor | Ch5 | SHOULD-FIX | Open | S-031 | 'Half of its six inputs' and 'two of the six inputs' disagree; a vertical stack leaves four of six nearly constant | `docs/thesis/ch5_experimental_setup.tex L167` |
| P-033 | Professor | Ch5 | SHOULD-FIX | Open | — | Aligning each drone to its commanded trajectory couples the time base to the quantity being evaluated | `docs/thesis/ch5_experimental_setup.tex L340` |
| P-034 | Professor | Ch5 | NICE | Open | — | Tracked-volume `\todo` still says 'before the first session' | `docs/thesis/ch5_experimental_setup.tex L86` |
| P-035 | Professor | Ch6 | SHOULD-FIX | Open | S-036 | Ch.6 keeps a conditional 'Hybrid (NA-INDI)' results slot that contradicts the locked doctrine | `docs/thesis/ch6_results_2robot.tex L142` |
| P-036 | Professor | Ch7 | MUST-FIX | Open | — | The RQ3 fallback needs the same third vehicle that the main plan needs | `docs/thesis/ch7_results_multirobot.tex L71` |
| P-037 | Professor | Ch8 | SHOULD-FIX | Open | S-035 | Six `Chapter~\ref{sec:...}` references render as 'Chapter 7.2', 'Chapter 6.3', 'Chapter 5.7' | `docs/thesis/ch8_discussion.tex L61` |
| P-038 | Professor | Ch9 | MUST-FIX | Open | S-034 | The conclusion says Strategies 0–3 'were implemented'; Strategy 3 is not | `docs/thesis/ch9_conclusion.tex L24` |
| P-039 | Professor | Ch9 | SHOULD-FIX | Open | — | The future-work list mixes core campaign items with extensions and is internally inconsistent | `docs/thesis/ch9_conclusion.tex L59` |
| P-040 | Professor | Cross | SHOULD-FIX | Open | S-037 | Repository-internal references appear in thesis text (docs numbers, script names, 'Priority D/E') | `docs/thesis/ch5_experimental_setup.tex L39` |
| P-041 | Professor | Cross | SHOULD-FIX | Open | — | `docs/thesis/README.md` and `docs/00_README.md` are stale and point to a ledger path that does not exist | `docs/thesis/README.md L16` |
| P-042 | Professor | Cross | SHOULD-FIX | Open | — | WRITING_RULES §1 'do not write as if C.0 or C.1 is done' is superseded by the repository state | `docs/thesis/WRITING_RULES.md L33` |
| P-043 | Professor | Cross | SHOULD-FIX | Open | S-039 | The draft contains no figure at all (no setup photo, block diagram, scenario geometry or wake sketch) | `docs/thesis/ch4_control_architectures.tex L74` |
| S-001 | Student | Ch1 | MUST-FIX | Open | P-001 | Chapter 1 sends me to Ch.2 and Ch.5 for 'falsifiable hypotheses'; I could not find any there | `docs/thesis/ch1_introduction.tex L145` |
| S-002 | Student | Ch1 | SHOULD-FIX | Open | — | INDI is used in Ch.1 without a word of explanation; FBL and NN never get an expansion at all | `docs/thesis/ch1_introduction.tex L164` |
| S-003 | Student | Ch1 | SHOULD-FIX | Open | — | 'Strategy 2' and 'Strategy 3' are used in RQ3 before any strategy has been introduced | `docs/thesis/ch1_introduction.tex L133` |
| S-004 | Student | Ch1 | SHOULD-FIX | Open | — | The italic gap sentence is one 60-word sentence with three claims; I read it three times | `docs/thesis/ch1_introduction.tex L96` |
| S-005 | Student | Ch1 | NICE | Open | — | RQ3 packs the question, a disclaimer and a scope note into one item | `docs/thesis/ch1_introduction.tex L130` |
| S-006 | Student | Ch1 | NICE | Open | — | Terms that a BSc reader has to guess in Ch.1: downwash, blade flapping, slung payload, rotor-speed sensing, wrench-free 'residual' | `docs/thesis/ch1_introduction.tex L107` |
| S-007 | Student | Ch1 | NICE | Open | — | The outline names Ch.6–9 only as 'the two- and multi-robot results chapters' and ends with 'the concluding chapter concludes' | `docs/thesis/ch1_introduction.tex L248` |
| S-008 | Student | Ch2 | SHOULD-FIX | Open | P-008 | Table 2.1 runs off the page; I cannot read the last column | `docs/thesis/ch2_related_work.tex L420` |
| S-009 | Student | Ch2 | SHOULD-FIX | Open | P-006 | The Hsieh et al. paragraph carries about a dozen numbers and four caveats in a single block | `docs/thesis/ch2_related_work.tex L291` |
| S-010 | Student | Ch2 | SHOULD-FIX | Open | — | ROM-FBL, FBL, MPC/NMPC are used with no expansion; 'reduced-order model' never appears | `docs/thesis/ch2_related_work.tex L291` |
| S-011 | Student | Ch2 | NICE | Open | — | The rigid-body equations are printed in Ch.2 and again in Ch.3 under different labels | `docs/thesis/ch2_related_work.tex L67` |
| S-012 | Student | Ch3 | SHOULD-FIX | Open | P-016 | 'Differencing' is the key step for isolating the interaction, but no chapter says which flights are differenced | `docs/thesis/ch3_system_modelling.tex L82` |
| S-013 | Student | Ch3 | SHOULD-FIX | Open | P-011 | `a_des` means one thing in Ch.3 and another in Ch.4 | `docs/thesis/ch3_system_modelling.tex L161` |
| S-014 | Student | Ch3 | SHOULD-FIX | Open | P-012 | Ch.3 says a_meas comes from the accelerometer, then that it is 'differentiated' | `docs/thesis/ch3_system_modelling.tex L146` |
| S-015 | Student | Ch3 | SHOULD-FIX | Open | — | The deep-sets idea is given as an equation without a plain-words picture; 'latent space' and 'embedding' are not explained | `docs/thesis/ch3_system_modelling.tex L242` |
| S-016 | Student | Ch3 | NICE | Open | — | Rotor speed is written with varpi, which looks like omega in print; omega is also body rate | `docs/thesis/ch3_system_modelling.tex L57` |
| S-017 | Student | Ch3 | NICE | Open | — | STM32, 'specific force', 'allocator', 'crossover' are used without a gloss | `docs/thesis/ch3_system_modelling.tex L217` |
| S-018 | Student | Ch4 | SHOULD-FIX | Open | P-019 | Table 4.1 is the map of the thesis but uses four undefined column ideas ('Knows', 'Enters at', 'Rotor speed', 'unflown') | `docs/thesis/ch4_control_architectures.tex L56` |
| S-019 | Student | Ch4 | SHOULD-FIX | Open | — | The INDI torque equation has no plain-words companion; I could not see why it cancels a disturbance | `docs/thesis/ch4_control_architectures.tex L209` |
| S-020 | Student | Ch4 | SHOULD-FIX | Open | — | 'Through the usual flatness construction' hides the step from thrust vector to attitude, and flatness is used before it is explained here | `docs/thesis/ch4_control_architectures.tex L99` |
| S-021 | Student | Ch4 | SHOULD-FIX | Open | P-018 | Attitude-loop symbols: omega_des is never defined, Omega/omega and R_d/R_des alternate, and K_R changes meaning between equations | `docs/thesis/ch4_control_architectures.tex L113` |
| S-022 | Student | Ch4 | SHOULD-FIX | Open | — | No gain values, units or on/off state anywhere in the thesis for K_p, K_v, K_i, k_i,att | `docs/thesis/ch4_control_architectures.tex L89` |
| S-023 | Student | Ch4 | NICE | Open | — | Strategy 3 opens with 'Implemented. Not implemented.' | `docs/thesis/ch4_control_architectures.tex L274` |
| S-024 | Student | Ch4 | NICE | Open | — | 'Lower-triangular structure', 'flat output' and 'flatness-preserving' are used without a picture of what they mean | `docs/thesis/ch4_control_architectures.tex L284` |
| S-025 | Student | Ch4 | NICE | Open | — | Runtime selection is described with 'one bit ... another ... a third parameter'; I cannot map them to the four configurations | `docs/thesis/ch4_control_architectures.tex L130` |
| S-026 | Student | Ch4 | NICE | Open | — | The same controller is called 'geometric baseline', 'uncompensated baseline' and 'uncompensated geometric baseline' | `docs/thesis/ch4_control_architectures.tex L142` |
| S-027 | Student | Ch5 | SHOULD-FIX | Open | P-025 | The Campaign status paragraph uses repository jargon I cannot parse (merges, role alignment, manifest, QA shakedowns, C.1) | `docs/thesis/ch5_experimental_setup.tex L30` |
| S-028 | Student | Ch5 | SHOULD-FIX | Open | — | 'C.1', 'C.4' (campaign phases) are never defined in the thesis, and they collide with the scenarios C1–C5 | `docs/thesis/ch5_experimental_setup.tex L37` |
| S-029 | Student | Ch5 | SHOULD-FIX | Open | — | Shop-floor terms: 'trajectory ladder', 'dz/speed grid', 'mocap', 'uSD', 'DShot', 'wake rather than wash' | `docs/thesis/ch5_experimental_setup.tex L233` |
| S-030 | Student | Ch5 | SHOULD-FIX | Open | P-030 | The sixteen scenarios are described by purpose but have no separations, speeds, durations or sketches | `docs/thesis/ch5_experimental_setup.tex L94` |
| S-031 | Student | Ch5 | SHOULD-FIX | Open | P-032 | 'Half of its six inputs' and 'two of the six inputs' disagree; I did not know the six inputs until I went back to Ch.3 | `docs/thesis/ch5_experimental_setup.tex L167` |
| S-032 | Student | Ch5 | SHOULD-FIX | Open | — | The metric called 'Residual rejection' is the separation error between vehicles, not a residual | `docs/thesis/ch5_experimental_setup.tex L290` |
| S-033 | Student | Ch5 | NICE | Open | P-029 | The threat table says the merge tool 'aborts above 15 cm RMS role check'; I do not know what a role check is | `docs/thesis/ch5_experimental_setup.tex L340` |
| S-034 | Student | Cross | MUST-FIX | Open | P-038 | The conclusion says Strategies 0–3 'were implemented'; Chapter 4's table says Strategy 3 is not | `docs/thesis/ch9_conclusion.tex L24` |
| S-035 | Student | Cross | SHOULD-FIX | Open | P-037 | Chapters 6–9 read as a plan with red notes; the references inside them print as 'Chapter 7.2' and 'Chapter 5.7' | `docs/thesis/ch8_discussion.tex L61` |
| S-036 | Student | Cross | SHOULD-FIX | Open | P-035 | 'Strategy 4', 'Strategy 5/7' appear in Ch.6 and Ch.9, but only Strategies 0–3 exist in Ch.1–5 | `docs/thesis/ch9_conclusion.tex L71` |
| S-037 | Student | Cross | SHOULD-FIX | Open | P-040 | The thesis points me to files and labels I cannot see: 'docs/25', 'Priority D/E', 'docs/36' | `docs/thesis/ch7_results_multirobot.tex L58` |
| S-038 | Student | Cross | SHOULD-FIX | Open | — | No list of abbreviations, list of symbols or glossary, although the repository has a glossary | `docs/thesis/thesis_draft.tex L5` |
| S-039 | Student | Cross | SHOULD-FIX | Open | P-043 | There is not a single figure; the stack geometry, the control loop and the network are all described only in words | `docs/thesis/ch1_introduction.tex L28` |
| S-040 | Student | Cross | NICE | Open | — | No title page, abstract, or declaration in the draft build | `docs/thesis/thesis_draft.tex L3` |

## What could not be done

- `docs/20_Verified_Claims.md` (the ledger named in the brief) does not exist at that path; the archived copy `docs/_archive/citation_notes/20_Verified_Claims.md` is stale in places, so the source PDFs in `docs/papers/` were the primary evidence for every citation check. The dead path is reported as P-041.
- No PDF for Abro et al. in the repository: that claim is UNVERIFIABLE (P-010). Hsieh et al. v1 was not available (P-006).
- Firmware and analysis scripts were read only where a finding depends on them (merge tool, logging configuration); the real C.2 split, the training label and the RPM source in use were not inspected.
- No numerical experiment was run; derivations were done by hand. Only the page with Table 2.1 was inspected as an image.
- The Student is a simulated reader: assumed meanings reflect a typical BSc background.
- Ch.6–9 were judged on structure only (no results exist).

## Files created and confirmation

Created (all new, in the new folder `docs/thesis/reviews/`):

- `docs/thesis/reviews/REVIEW_professor_2026-10-04.md`
- `docs/thesis/reviews/REVIEW_student_2026-10-04.md`
- `docs/thesis/reviews/REVIEW_summary_2026-10-04.md`

Confirmation, from `git` at generation time:

- `git diff --stat -- docs/thesis`: empty (no tracked file in `docs/thesis/` changed)
- `git status --short` excluding the pre-existing untracked `experiments/logs/usd_raw/` data: nothing listed
- `.gitignore` rule that hides the new files from `git status`: `.gitignore:27:docs/thesis/*	docs/thesis/reviews/REVIEW_summary_2026-10-04.md`
- No `.tex`, `.bib`, `WRITING_RULES.md` or other existing file was edited; no `git add`, `commit` or `push` was run; no `Co-Authored-By` trailer was written. The temp builds and extracted texts are in `/tmp` and are not part of the repository.


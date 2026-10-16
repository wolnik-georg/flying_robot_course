# Student review of the thesis draft — 2026-10-04

**Role:** a capable engineering student (BSc, knows basic linear algebra, rigid-body dynamics, PID and state space) who is **not** a specialist in quadrotor control, INDI, geometric control, neural residual models or multi-robot aerodynamics. Linear reading, Chapters 1–5 in detail, 6–9 for orientation only. **Status:** read-only review; no `.tex`, `.bib` or existing file was modified, nothing was committed or pushed, no hardware was touched. This file was written after the Professor file was finished and checked. I built its findings from my own reading of the chapters and did not copy from the Professor file, but one agent ran both roles, so the independence is limited; where both reports mention the same issue this is listed in the summary file.

Build used: the temp-dir PDF from the Professor pass (50 pages, 0 undefined references) and the `.tex` sources. All line numbers below were found by exact string search at generation time. Where I write 'I assumed', this is what a student without outside help would guess; it is not a statement about what the term means.

## Executive summary (top 10 findings)

1. **S-034** (Cross, MUST-FIX) — The conclusion says Strategies 0–3 'were implemented'; Chapter 4's table says Strategy 3 is not
2. **S-001** (Ch1, MUST-FIX) — Chapter 1 sends me to Ch.2 and Ch.5 for 'falsifiable hypotheses'; I could not find any there
3. **S-012** (Ch3, SHOULD-FIX) — 'Differencing' is the key step for isolating the interaction, but no chapter says which flights are differenced
4. **S-028** (Ch5, SHOULD-FIX) — 'C.1', 'C.4' (campaign phases) are never defined in the thesis, and they collide with the scenarios C1–C5
5. **S-038** (Cross, SHOULD-FIX) — No list of abbreviations, list of symbols or glossary, although the repository has a glossary
6. **S-039** (Cross, SHOULD-FIX) — There is not a single figure; the stack geometry, the control loop and the network are all described only in words
7. **S-027** (Ch5, SHOULD-FIX) — The Campaign status paragraph uses repository jargon I cannot parse (merges, role alignment, manifest, QA shakedowns, C.1)
8. **S-019** (Ch4, SHOULD-FIX) — The INDI torque equation has no plain-words companion; I could not see why it cancels a disturbance
9. **S-015** (Ch3, SHOULD-FIX) — The deep-sets idea is given as an equation without a plain-words picture; 'latent space' and 'embedding' are not explained
10. **S-013** (Ch3, SHOULD-FIX) — `a_des` means one thing in Ch.3 and another in Ch.4

Overall: the argument of the thesis is clear from Chapter 1 and Chapter 2 — I can say in one sentence what is compared and why — and the prose is careful. What costs me effort is not the logic but the **vocabulary and the notation**: abbreviations (INDI, FBL, NN, ROM-FBL, EKF, uSD, DShot, RPM) used before or without an expansion, phase labels (C.0–C.4) that are never defined and collide with scenario names (C1–C5), repository jargon in the status paragraph of Ch.5, no list of symbols, and no figure at all. Chapter 4 is the hardest to read because the INDI and attitude equations come without an intuitive sentence, and Chapter 5 is the hardest to use because the scenarios have no numbers.

### Readability rating (1 = very hard, 5 = easy for the reader described above)

| Chapter | Terminology | Flow | Intuition | Orientation | Three sections that cost me the most effort |
|---|---|---|---|---|---|
| Ch1 Introduction | 3 | 4 | 4 | 4 | 1. `ch1_introduction.tex` L92 (§Gap: one dense italic sentence)<br>2. `ch1_introduction.tex` L117 (§Research questions: RQ3 and the pointer to hypotheses)<br>3. `ch1_introduction.tex` L203 (§Scope table: terms without gloss) |
| Ch2 Related work | 2 | 3 | 3 | 4 | 1. `ch2_related_work.tex` L261 (§Injecting the prediction: the Hsieh paragraph)<br>2. `ch2_related_work.tex` L403 (§How these methods are evaluated: Table 2.1 clipped)<br>3. `ch2_related_work.tex` L170 (§Predictive compensation: deep sets, spectral normalisation, latent space) |
| Ch3 Modelling | 3 | 4 | 3 | 4 | 1. `ch3_system_modelling.tex` L183 (§Predicting the residual: deep sets)<br>2. `ch3_system_modelling.tex` L158 (§Sign convention: a_des and the doubling argument)<br>3. `ch3_system_modelling.tex` L240 (§Superposition assumption: latent-space wording) |
| Ch4 Control architectures | 2 | 3 | 2 | 3 | 1. `ch4_control_architectures.tex` L186 (§Strategy 1 pure INDI: the incremental torque law)<br>2. `ch4_control_architectures.tex` L75 (§Position and attitude loop: flatness, symbols)<br>3. `ch4_control_architectures.tex` L272 (§Strategy 3: flatness-preserving residual) |
| Ch5 Experimental setup | 2 | 3 | 3 | 3 | 1. `ch5_experimental_setup.tex` L92 (§Formation library: no numbers; null scenarios; six inputs)<br>2. `ch5_experimental_setup.tex` L323 (§Threats to validity: table jargon)<br>3. campaign-status paragraph at the top of the chapter (`ch5_experimental_setup.tex` L28) |

Ch.6–9 are not rated: they are structure and `\todo` notes only and were read for orientation (section F).

## Preparation

- **Reading order:** `thesis_draft.tex` order (Ch.1 → Ch.9). The finding list was written from a fresh reading of the chapters, not from the Professor file (which had been completed and checked before).
- **Tables and figures per chapter** (counted with `\begin{table}` / `\begin{figure}` in the sources): Ch1: 1 table(s), 0 figure(s), Ch2: 1 table(s), 0 figure(s), Ch3: 1 table(s), 0 figure(s), Ch4: 1 table(s), 0 figure(s), Ch5: 2 table(s), 0 figure(s), Ch6: 3 table(s), 0 figure(s), Ch7: 0 table(s), 0 figure(s), Ch8: 0 table(s), 0 figure(s), Ch9: 0 table(s), 0 figure(s).
- **Terms audited:** 35 terms/abbreviations/symbol groups in section A (counts and first-use lines by regex on the non-comment lines of ch1–ch9).

## A. Term and symbol audit

Counts are occurrences in non-comment lines of ch1–ch9. 'First use' is the first non-comment line. 'Definition' is where I found a definition by reading; — means I found none. Of 35 audited terms: **4** are defined at or before first use; **3** only after first use; **1** partly (idea given, term not glossed); **1** I treat as standard BSc vocabulary; **26** are never defined in the thesis text.

| Term | Uses | First use | Definition | Status | What I assumed |
|---|---|---|---|---|---|
| INDI | 43 | Ch1 L164 | Ch2 L134 | **defined after first use** | inverse-dynamics controller of some kind |
| FBL | 14 | Ch1 L240 | — | **not defined** | feedback linearisation (guess from the section title) |
| ROM-FBL | 4 | Ch2 L289 | — | **not defined** | reduced-order-model FBL (guess) |
| NN | 12 | Ch1 L240 | — | **not defined** | neural network |
| MPC / NMPC | 4 | Ch2 L314 | — | **not defined** | model predictive control (spelled once, abbreviation not linked) |
| EKF | 1 | Ch5 L341 | — | **not defined** | extended Kalman filter (spelled out in Ch.5 L64, abbreviation not linked) |
| uSD | 8 | Ch5 L61 | — | **not defined** | micro SD card on the vehicle |
| DShot | 1 | Ch5 L56 | — | **not defined** | a digital motor protocol with telemetry |
| RPM | 4 | Ch3 L344 | — | **not defined** | rotor speed (the text otherwise says 'rotor speed' or varpi) |
| SIL | 2 | Ch8 L120 | — | **not defined** | software-in-the-loop (spelled once in Ch.5) |
| mocap / OptiTrack | 10 | Ch2 L223 | — | **not defined** | motion capture |
| STM32 | 2 | Ch2 L181 | — | **not defined** | the vehicle's microcontroller |
| downwash | 18 | Ch1 L28 | Ch1 L28 | defined at or before first use | air pushed down by the upper rotors (explained by its effect in Ch.1; no formal definition) |
| wash / wake | 2 | Ch2 L105 | — | **not defined** | wake = trailing flow of the leader; wash = downwash |
| wrench | 9 | Ch2 L123 | Ch3 L74 | **defined after first use** | unmodelled force and moment |
| differential flatness | 22 | Ch2 L85 | Ch2 L85 | defined at or before first use | state and inputs computable from output derivatives |
| SE(3) / SO(3) | 4 | Ch2 L75 | — | **not defined** | rigid-body motion group / rotation group |
| specific force | 1 | Ch3 L120 | — | **not defined** | accelerometer reading |
| allocator | 2 | Ch3 L101 | — | **not defined** | maps thrust and torque to four motors |
| deep sets | 5 | Ch2 L194 | Ch3 L193–200 (properties only) | **partly** (idea explained, term not glossed) | a network that sums per-neighbour features (Ch.3 L193–200 gives the two required properties, the term itself is not glossed) |
| latent space / embedding | 4 | Ch2 L228 | — | **not defined** | hidden feature vector |
| spectral normalisation | 3 | Ch2 L197 | — | **not defined** | a weight-norm constraint that bounds the network gain |
| differencing | 1 | Ch3 L82 | — | **not defined** | subtracting a solo-flight residual (see finding) |
| crossover | 4 | Ch3 L342 | — | **not defined** | the point where one strategy beats the other |
| feedforward | 12 | Ch2 L87 | — | assumed known (BSc control vocabulary) | a term computed from the plan, not from feedback (standard control vocabulary) |
| snap | 4 | Ch2 L90 | — | **not defined** | 4th derivative of position |
| C.0–C.4 (phases) | 33 | Ch5 L32 | — | **not defined** | C.1 = data collection, C.4 = comparison; C.0, C.2, C.3 unknown |
| trajectory ladder | 1 | Ch5 L233 | — | **not defined** | sequence of flights of rising difficulty |
| dz/speed grid | 2 | Ch5 L39 | — | **not defined** | vertical separation times speed |
| manifest | 2 | Ch5 L33 | — | **not defined** | file listing the merged flights |
| role alignment / role check | 3 | Ch5 L31 | — | **not defined** | deciding which vehicle is top/bottom |
| null scenario | 10 | Ch1 L195 | Ch5 L143 | **defined after first use** | control flights without interaction |
| Ψ (attitude error function) | 3 | Ch4 L171 | — | **not defined** | an error measure on rotations |
| hat map / vee | 3 | Ch3 L55 | Ch3 L55 | defined at or before first use | hat map is defined; vee is not |
| east-north-up (ENU) | 1 | Ch3 L47 | Ch3 L47 | defined at or before first use | frame convention |

**Notation overload and unit problems found** (each has a finding): `a_des` (Ch.3 vs Ch.4: S-013); `varpi`, `omega`, `Omega` (S-016); `K_R`, `K_omega` in the torque law and the angular-acceleration law (S-021); `C.1` phase vs `C1` scenario (S-028); `a_meas` accelerometer vs differentiated (S-014). Symbols without units that I could not resolve: `K_p`, `K_v`, `K_i`, `k_{i,att}` (S-022). Equations where I could not tell input from output without effort: `eq:ctrl-indi-tau` (S-019) and `eq:model-deepsets` (S-015).

**Good practice I noticed:** Ch.3 Section 3.1 states the frame (east-north-up) and the gravity sign explicitly; the residual is defined in words before the equation; the sign convention gets its own subsection; Table 3.1 lists every assumption with its failure mode.

## B. Reading-flow test

Sentences or paragraphs where I had to read twice, or where a bridge or target is missing. Each quote is verbatim (≤ 25 words) and its line number was found by search. 'Missing' is a one-line description, not a rewrite.

| # | Location | Quote | Why I stumbled | What is missing |
|---|---|---|---|---|
| 1 | `docs/thesis/ch1_introduction.tex` L77 | “Combining them is the obvious next construction. It” | 'It' could be 'combining' or 'the obvious next construction'; the next sentence says it 'has been built, on a single vehicle'. | a clear subject: 'The hybrid has been built...' |
| 2 | `docs/thesis/ch1_introduction.tex` L83 | “literature. Not because the individual results are weak, and not because the field avoids” | Two negative clauses before the real reason ('It is that the results cannot be composed'), which comes two sentences later. | state the reason first, then the two things it is not |
| 3 | `docs/thesis/ch1_introduction.tex` L109 | “is measurable. That is where the predictive component has the most to contribute. Multi-robot” | 'That' points to a whole situation (other vehicles produce the residual, relative state is measurable); the next sentence is a scope exclusion. | a bridge between 'where the predictive part would help' and 'but we do not evaluate hybrids' |
| 4 | `docs/thesis/ch1_introduction.tex` L96 | “not been evaluated against one another under identical conditions, and the field's reporting conventions make such a comparison” | Three claims in one sentence (see S-004). | sentence break |
| 5 | `docs/thesis/ch3_system_modelling.tex` L93 | “The decision has company. Neural-Swarm compensates the vertical interaction” | 'Has company' is an idiom; it took a second read to see that it means 'other works make the same choice'. | plain statement: 'Others make the same choice.' |
| 6 | `docs/thesis/ch3_system_modelling.tex` L242 | “Equation~\eqref{eq:model-deepsets} assumes the aggregate interaction is a sum of per-neighbour” | 'In the latent space of phi' is not explained; I could not tell what is being summed (see S-015). | plain-words companion sentence |
| 7 | `docs/thesis/ch3_system_modelling.tex` L168 | “so the desired-acceleration vector passed to the thrust-and-attitude allocation carries” | 'Carries -f_res/m' is a statement about a vector made of three parts; with a_des in Ch.4 meaning the whole thing it is confusing (see S-013). | name of the passed vector |
| 8 | `docs/thesis/ch4_control_architectures.tex` L40 | “Four controllers: an uncompensated baseline (Strategy~0), flown in every scenario as the” | The sentence has no verb; I had to read the next line to see it is a list. | 'There are four controllers: ...' |
| 9 | `docs/thesis/ch4_control_architectures.tex` L78 | “position loop and an attitude law for the commanded thrust direction. Strategy~1 replaces the” | 'Replaces the attitude law's inner term with an incremental one' — I did not know which term is 'inner'. | name the term (the torque computed from the attitude errors) |
| 10 | `docs/thesis/ch4_control_architectures.tex` L209 | “Because \eqref{eq:ctrl-indi-tau} corrects relative to what is actually being achieved, it rejects” | The key idea of INDI in one clause, after an equation that has no explanation (see S-019). | one intuitive sentence before the claim |
| 11 | `docs/thesis/ch4_control_architectures.tex` L99 | “with the desired yaw, gives the desired attitude $R_{\mathrm{des}}$ through the usual flatness” | 'Usual flatness construction' is a pointer to knowledge I do not have (see S-020). | a three-line recipe or a reference |
| 12 | `docs/thesis/ch5_experimental_setup.tex` L214 | “Residual error comes from propagation, task wake phase and crystal drift between” | 'Task wake phase' and 'crystal drift' are firmware / hardware terms; the sentence also mixes causes (offset) with a rate error. | one sentence of explanation for each cause |
| 13 | `docs/thesis/ch5_experimental_setup.tex` L340 | “Radio vs onboard clock & uSD logs at \SI{500}{\hertz}; analysis uses merged uSD and firmware \texttt{ctrltarget.*} where available” | 'ctrltarget.*' is a log-channel name pattern; the row mixes the threat, the mitigation and a file-name detail. | plain words for what 'ctrltarget' is |
| 14 | `docs/thesis/ch5_experimental_setup.tex` L30 | “the brushless pair \textbf{cf5} (study vehicle) and \textbf{cf\_second} (top). Micro-SD merges” | Vehicle names (cf5, cf_second) and roles (study vehicle, top) are introduced in passing; 'study vehicle' is not explained (see S-027). | a sentence on naming and roles |
| 15 | `docs/thesis/ch5_experimental_setup.tex` L167 | “trained only on such data has seen no variation in half of its six inputs and is extrapolating the” | I had to go back to Ch.3 to know the six inputs; then 'half' did not match the 'two of the six' below (see S-031). | list the six inputs here |
| 16 | `docs/thesis/ch5_experimental_setup.tex` L244 | “Stage~1 cannot confirm that the compensation terms are \emph{correct}, only that they are not” | 'Dormant' compensation terms are explained logically, but I had to read the paragraph twice to see why a clean single-vehicle flight 'demonstrates nothing'. | one concrete example (a_res is about zero alone, so a wrong sign has no effect) |

**Dense sentences with two or more subordinate clauses (counted by eye in the passages above):** the Ch.1 gap sentence, the Ch.2 Hsieh paragraph, the Ch.4 Strategy 3 reference paragraph. **'As shown above/below' without a clear target:** I looked for these by search (`above`, `below`, `as shown`, `see below`) in Ch.1–5 and found no unclear target; the few uses (for example the layer widths 'above' in Ch.3, 'the works above' in Ch.2) point to the right place. **Paragraphs without a bridge:** the Ch.1 'Problem' to 'Gap' transition is clean; Ch.2 sections open with a one-line link to the previous one; Ch.5 jumps from the status paragraph into 'Platform' without a link.

## C. Prerequisites and intuition

**Ch1.** (i) What it assumes that is not given: what downwash is physically; what a residual force does to a controller; what INDI, FBL and 'geometric tracking' are; what a neural-network feedforward is. (ii) Where a figure would have saved me: a sketch of two stacked quadrotors with the downwash cone and the 60 cm / 9 cm numbers (Section 1.1); a four-box diagram of the compared strategies (Section 1.4). A worked example for the 36 % vs 31 % non-comparability would help. (iii) Equations: none. The 'measure vs predict' pair in Section 1.2 deserves a one-line diagram (measure = after the fact, predict = before).

**Ch2.** (i) Assumes: SE(3) geometric control, differential flatness, INDI, MPC, neural network basics (spectral normalisation, deep sets), reinforcement learning, Gaussian processes. These are explained in a sentence each at most. (ii) A timeline or taxonomy figure (reactive / predictive / hybrid / direct learning) with the injection point of each; Table 2.1 must be readable (S-008). (iii) The jet-flow sentence (2.5 motor-to-motor distances) could have a picture; the flatness paragraph deserves a plain-words companion.

**Ch3.** (i) Assumes: rigid-body dynamics with SO(3), the hat map (defined), accelerometer specific force, low-pass filtering, ReLU networks, first-layer weight folding. (ii) A block diagram of 'measure' vs 'predict' ending at the common point in the control law (Section 3.3 / 3.4); a numerical example of the sign argument (doubling). (iii) `eq:model-ares` (what is subtracted from what), `eq:model-sign` (why minus), `eq:model-deepsets` (what is summed), `eq:model-fold` (why the folding is exact).

**Ch4.** (i) Assumes: SE(3) attitude error and the vee operator, differential flatness, angular-acceleration estimation by differentiating the gyro, INDI, feedback linearisation. (ii) The control-loop block diagram is the single most missing figure: position loop, attitude loop, INDI increment, and the three injection points. A worked number for INDI (e.g. requested 10 rad/s^2, measured 8, increment J*2). (iii) `eq:ctrl-ades` (what each term contributes), `eq:ctrl-tau-geo`, `eq:ctrl-indi-tau`.

**Ch5.** (i) Assumes: motion-capture set-ups, extended Kalman filtering, SD logging, the Crazyflie ecosystem (cf5, DShot, uSD), polynomial trajectories, train/test splitting for time series. (ii) A photo or plan of the tracked volume (2 m x 4 m x 1.7 m) with marked wall distances; one sketch each for scenario groups A, B, C; a timeline of campaign phases C.0–C.4. (iii) None needed; the metrics paragraph (position RMSE, separation error) would benefit from one worked example.

## D. Explain-it-back test

In my own words, after each chapter. 'Unsure' lists what I could not confirm; I have not guessed to sound confident.

**Ch1 — what I understood.** Quadrotors flying close together disturb each other through downwash, so people keep them far apart. If a controller could compensate the disturbance, they could fly closer. There are two ways to learn about the disturbance: measure it afterwards from how the vehicle accelerated, or predict it from where the neighbours are. Each has been shown to work separately, but nobody has compared them on the same vehicle, trajectories and conditions. The thesis does exactly this comparison on two Crazyflies, adds a question about moving from two to three vehicles, and counts a 'no difference' outcome as a result. It claims no new controller.

*Unsure: what exactly 'hybrid' adds and why it is out of scope; what 'transfer penalty' is in numbers; what the hypotheses are; whether INDI and 'geometric' are two controllers or two modes of one.*

**Ch2 — what I understood.** The chapter walks from the basic model, via the aerodynamics of the interaction, to methods that measure the residual (INDI), predict it (Neural-Swarm and others), put the prediction into different places of a controller, learn actions directly, or combine both (a hybrid). Then it argues that reported numbers cannot be compared, because every paper uses a different airframe, separation, metric and baseline, and ends with a list of what a fair experiment would need. Results are quoted with care about which part of the paper they come from.

*Unsure: the differences between the three NMPC-based works; what 'flatness-preserving' means in practice; which of the many numbers I should remember; whether the hybrid result is on a single vehicle only (I think yes).*

**Ch3 — what I understood.** The residual is everything the nominal model gets wrong: interaction, but also model error and ground effect. It can be measured by subtracting the modelled acceleration from the measured one, which needs rotor speeds. It can be predicted by a small network that adds up per-neighbour contributions. The residual is subtracted in the control law, and a wrong sign roughly doubles the displacement. A table lists the assumptions and when each one breaks.

*Unsure: how the 'interaction alone' is separated from the other parts of the residual in practice (differencing, S-012); what the network's output is trained against; why summing in latent space is better than summing forces.*

**Ch4 — what I understood.** Four controllers are compared: a geometric controller as the reference (Strategy 0), the same controller with the measured residual and incremental torque (1, INDI), the geometric controller with the network's prediction (2), and a feedback-linearising one with a structure-preserving prediction (3, not implemented). The chapter says what is implemented and what is only a published reference, and states that each controller is tuned for itself and then frozen, so fairness means equal conditions, not equal gains.

*Unsure: how the incremental torque law cancels a disturbance (S-019); what the reduced torque law changes for stability; how the 'bits' map to the four INDI configurations; what the differences between Ch.3's and Ch.4's a_des are.*

**Ch5 — what I understood.** Two brushless Crazyflies are flown in a motion-capture volume. Sixteen frozen scenarios cover vertical stacks, offsets, dynamic merges, three-vehicle shapes and coplanar null controls. Each vehicle logs 500 Hz to an SD card because radio saturates. Before any data count, a three-step gate validates, tunes, freezes. The comparison interleaves controllers in a session. Threats are tabulated.

*Unsure: what the numbers of each scenario are (S-030); how many repeats; which solo flights exist (S-012); how the time alignment is really done; what C.0–C.4 stand for.*

## E. Questions to the author (10 per chapter, most blocking first)

**Ch1.**

1. What are the hypotheses that Section 1.4 says are stated in Chapters 2 and 5, and where are they?
2. What do INDI, FBL and NN stand for, and why are they in the contribution list before they are explained?
3. How many strategies are compared, and which are the 'reactive' and the 'predictive' ones? (RQ3 mentions Strategy 2 before I know what it is.)
4. Why is the hybrid in the gap statement if its evaluation is outside the thesis?
5. What does 'transfer penalty' mean in numbers (in which unit, against which baseline)?
6. Which quantity would show that the strategies 'do not separate'?
7. Why is only the force residual in scope, and what does 'measured but not separately compensated' mean for the moment residual?
8. What is 'self-induced' residual, and why does it matter that the hybrid was only shown there?
9. Why do two vehicles suffice for RQ1–RQ2 but a third is needed for RQ3?
10. Which of the numbers in Section 1.1 (36 %, 56 %, 24 cm, 2–4x) can I compare, and which not?

**Ch2.**

1. What do ROM, FBL, NMPC and EKF stand for?
2. Can the table of evaluation conditions be shown in full (it is cut off)?
3. Which of the many reported numbers is the one to remember per paper?
4. What is the difference between reactive INDI and the online adaptation of Hsieh et al. (2025)?
5. What does 'flatness-preserving' mean in terms of what a network may depend on?
6. Why is the jet-flow region (2.5 motor distances) relevant to the 20–25 cm separations of the cited flight experiments?
7. Why are the 'Comparisons that do exist' not enough?
8. What would a minimal fair comparison require, in the author's words, in three bullet points?
9. Where does the hybrid (NA-INDI) fail, and is that failure relevant to this thesis?
10. Which papers did the author check against the PDF, and which only against the abstract?

**Ch3.**

1. How is the interaction separated from the other parts of the residual, concretely: which flights are subtracted from which?
2. What exactly is the training target of the network: a_res as logged, or the difference?
3. Is a_meas from the accelerometer or from differentiation? Which filter, which cut-off?
4. What does the network see as input when a neighbour is more than 2 m away, and how is the sum then defined?
5. Why a sum in latent space rather than a sum of pairwise forces?
6. Why 987 parameters, and what limits the size on the vehicle?
7. What is the unit and the frame of every symbol in `eq:model-ares`?
8. Why do the airframe constants stay fixed over a battery discharge?
9. How does 'a_res reads identically zero' show up in practice, and what is the check?
10. What is the 'crossover' in RQ2 in measurable terms?

**Ch4.**

1. How does the incremental torque law cancel a disturbance? Can you give a number?
2. What are omega_des and Omega_d, and how do they differ?
3. Are K_R and K_omega the same in the torque law and in the angular-acceleration law?
4. Which integral terms are active in the flown configuration, and with what gains?
5. What does 'through the usual flatness construction' compute?
6. What does the reduced torque law lose with respect to Lee et al.'s?
7. Where exactly is the network evaluated: every control step or at a lower rate?
8. What does 'status' in Table 4.1 mean, and when was it last updated?
9. What is Strategy 3 expected to add that Strategy 2 cannot?
10. How are the 'bits' mapped to the four INDI configurations, and which of the four are flown?

**Ch5.**

1. What do C.0–C.4 stand for, and how do they differ from scenarios C1–C5?
2. What are the separations, speeds and durations of each scenario?
3. How many repeats per cell, and how is it decided that two strategies 'separate'?
4. Which single-vehicle flights exist for the 'differencing' reference?
5. How is time alignment between two vehicles actually done today?
6. What is a 'role check', and why does it abort at 15 cm?
7. Which are the six inputs, and how many of them are constant in a vertical stack?
8. How is the 'separation error' computed, and what sign does it have?
9. What is the 'trajectory ladder'?
10. How does the thesis prevent training data and test data from sharing flights? (the table says 'held-out scenarios'; which?)

(Chapters 6–9: no question lists, because they are skeletons without results; orientation notes for them are in section F.)

## F. Orientation

- **From the Introduction alone, what the thesis does, why, and what I learn:** yes for the first two (a controlled comparison of measuring vs predicting the downwash residual, because published numbers do not compose); partly for the third — Section 1.4 promises hypotheses I cannot find (S-001) and Section 1.5 lists contributions as intended, not achieved, which is honest.
- **Roadmap:** exists (Section 1.7). It numbers Chapters 2–5 but not 6–9 and ends with a tautology (S-007).
- **Why am I reading this chapter?** Ch.2 and Ch.3 open with a paragraph that answers it well; Ch.4 opens with one sentence of link and a list; Ch.5 opens with a paragraph, then goes straight into campaign status (S-027), which interrupts the 'why'.
- **Cross-references:** `Section~\ref` and `Chapter~\ref` to chapters work; section labels referred to with the word 'Chapter' in Ch.8–9 print as 'Chapter 7.2' (S-035). Pointers to repository files (`docs/25`) are not usable (S-037).
- **Glossary or notation table:** none in the thesis (S-038); `flying_drone_stack/GLOSSARY.md` exists and covers position derivatives, snap, min-snap and more, but is not included or referred to.
- **Ch.6–9 for orientation:** (Ch.6) what is reported for two robots and how; (Ch.7) the transfer penalty and the three-vehicle limits; (Ch.8) how each RQ is interpreted and what would count as a negative result; (Ch.9) the answers and future work. All four are plans with red `\todo` notes; Ch.9 starts with a sentence that contradicts Ch.4 (S-034), and 'Strategy 4', 'Strategy 5/7' appear without a definition (S-036).

## G. Figures, tables, captions

**Figures: none in the whole draft** (S-039). Missing, in order of how much they would have helped me: control-loop block diagram (Ch.4); stack geometry with downwash cone (Ch.1/Ch.2); the deep-sets network (Ch.3); the lab layout and scenario sketches (Ch.5).

| Table | Where | Understandable from the caption? | What I could not read |
|---|---|---|---|
| Table 1.1 Scope | `docs/thesis/ch1_introduction.tex` L210 | Yes | 'Homogeneous vehicles', 'Onboard inference, offline training' need Ch.5/Ch.4 to be understood; fine as a summary. |
| Table 2.1 Evaluation conditions | `docs/thesis/ch2_related_work.tex` L420 | Mostly | The last column is cut off (S-008); abbreviations in the cells are not expanded. |
| Table 3.1 Assumptions | `docs/thesis/ch3_system_modelling.tex` L318 | Yes | 'Crossover' of RQ2; otherwise clear. |
| Table 4.1 Strategies compared | `docs/thesis/ch4_control_architectures.tex` L46 | Partly | 'Enters at', 'Rotor speed', INDI/NN/FBL abbreviations (S-018). |
| Table 5.1 Formation library | `docs/thesis/ch5_experimental_setup.tex` L106 | Partly | No numbers; 'wake', 'wash', 'docking approach' undefined (S-030). |
| Table 5.2 Threats to validity | `docs/thesis/ch5_experimental_setup.tex` L327 | Partly | 'role check', 'ctrltarget.*', 'EKF-vs-mocap gate' (S-033). |

The Ch.6 tables are empty placeholders and were not rated.

## Findings (ordered by chapter, then severity)

Counts by chapter and severity:

| Chapter | MUST-FIX | SHOULD-FIX | NICE | Total |
|---|---|---|---|---|
| Ch1 | 1 | 3 | 3 | 7 |
| Ch2 | 0 | 3 | 1 | 4 |
| Ch3 | 0 | 4 | 2 | 6 |
| Ch4 | 0 | 5 | 4 | 9 |
| Ch5 | 0 | 6 | 1 | 7 |
| Cross | 1 | 5 | 1 | 7 |
| **Total** | 2 | 26 | 12 | 40 |

### S-001 — Chapter 1 sends me to Ch.2 and Ch.5 for 'falsifiable hypotheses'; I could not find any there

- **Severity:** MUST-FIX
- **Location:** `docs/thesis/ch1_introduction.tex` L145 (§Research questions `sec:intro-rq`)
- **Quote:** “The questions are stated with falsifiable hypotheses in Chapter~\ref{ch:related} and”
- **Problem:** I followed the pointer to both chapters and looked for a hypothesis (a prediction with something that would count against it). Chapter 2 is a literature survey and Chapter 5 describes the set-up; neither states one. As a reader I then do not know what outcome the thesis would count as a negative answer, which the same paragraph says it will report as a result. The pointer is a statement about the thesis that I cannot confirm, which is worse than an omission.
- **Evidence:** Search of the nine chapters for `hypothes` finds the sentence above and unrelated uses; e.g. `docs/thesis/ch3_system_modelling.tex` L350: “Two entries deserve emphasis because they are not merely caveats but hypotheses the experiments” (a comment on two table rows, not the RQs).
- **Suggested direction:** Either add a short 'Hypotheses and decision rules' subsection (one prediction per RQ, the metric, the rule for 'no separation') and keep the pointer, or change the pointer to what exists.
- **Confidence:** HIGH — I looked in the two chapters named

### S-002 — INDI is used in Ch.1 without a word of explanation; FBL and NN never get an expansion at all

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch1_introduction.tex` L164 (§Contributions `sec:intro-contrib`)
- **Quote:** “Geometric tracking, INDI and feedback linearisation are different controllers; each is tuned for”
- **Also at:** `docs/thesis/ch1_introduction.tex` L240 (§Outline `sec:intro-outline`): “geometric+NN and FBL+NN), and the point at which each”
- **Problem:** The first appearance of INDI is in the contribution list; I did not know what it stood for or what it did. I assumed 'some inverse-dynamics controller'. The expansion comes only in Chapter 2 (Section 2.3), a whole chapter later. 'FBL' and 'NN' are never expanded: 'feedback linearisation' is spelled out in words but nothing says FBL is its abbreviation, and 'NN' was my guess for 'neural network'. A reader who starts at Chapter 1, as examiners do, meets three abbreviations in the outline before any definition.
- **Evidence:** Expansion of INDI: `docs/thesis/ch2_related_work.tex` L134: “Incremental nonlinear dynamic inversion (INDI) compares measured acceleration with the”. Search for the other expansions (`incremental nonlinear`, `feedback linearis`, `neural network`, `reduced-order`) over ch1–ch9: INDI is expanded once (Ch.2); 'FBL', 'ROM-FBL' and 'NN' are never introduced as abbreviations; `(FBL)` and `(NN)` do not occur, and 'neural network' appears spelled out only in Ch.2 (L165, L178, L373).
- **Suggested direction:** Expand each abbreviation at first use in Ch.1 with a half-sentence gloss, and add the list of abbreviations (see S-finding on the missing glossary).
- **Confidence:** HIGH — text search over all nine chapters

### S-003 — 'Strategy 2' and 'Strategy 3' are used in RQ3 before any strategy has been introduced

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch1_introduction.tex` L133 (§Research questions `sec:intro-rq`)
- **Quote:** “applies to the predictive strategies (Strategy~2 and, if implemented, Strategy~3). It does not”
- **Also at:** `docs/thesis/ch1_introduction.tex` L239 (§Outline `sec:intro-outline`): “\textbf{Chapter~\ref{ch:control}} describes Strategies~0--3 (the uncompensated baseline, pure INDI,”
- **Problem:** RQ3 refers to Strategy 2 and Strategy 3 as if they were known. In Chapter 1 the strategies are listed only in the outline, a few pages later, and numbered 0–3 there for the first time. When I read RQ3 I did not know that 'Strategy 2' is a particular controller, nor that there are four of them, nor what 'if implemented' refers to.
- **Evidence:** RQ3 line and outline line quoted above; the only definition of the numbering in Ch.1 is the outline sentence.
- **Suggested direction:** Introduce the four strategies in one short paragraph (or a three-row table) in Section 1.2 or at the start of Section 1.4, before the RQs.
- **Confidence:** HIGH — reading order checked line by line

### S-004 — The italic gap sentence is one 60-word sentence with three claims; I read it three times

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch1_introduction.tex` L96 (§Gap `sec:intro-gap`)
- **Quote:** “not been evaluated against one another under identical conditions, and the field's reporting conventions make such a comparison”
- **Problem:** The sentence says (a) methods exist and are validated, (b) they have not been compared under identical conditions, (c) the way results are reported makes a post-hoc comparison impossible. These are three claims joined by commas. It is the sentence the whole thesis rests on, so it is the one a reader most needs to take in at once.
- **Evidence:** Judgement from reading; the sentence starts at `docs/thesis/ch1_introduction.tex` L94: “\emph{Representative methods exist for reactive compensation, for predictive compensation, and for”.
- **Suggested direction:** Split into three short sentences, same content and same qualifiers ('to the best of our knowledge' stays).
- **Confidence:** MEDIUM — readability judgement; the content is fine

### S-005 — RQ3 packs the question, a disclaimer and a scope note into one item

- **Severity:** NICE
- **Location:** `docs/thesis/ch1_introduction.tex` L130 (§Research questions `sec:intro-rq`)
- **Quote:** “small enough that the summed architecture remains the right engineering choice? This measures”
- **Problem:** RQ3 is a question, then 'This measures the size of a known approximation. It is not a test of whether superposition holds...' then a scope sentence. I could not tell which part is the question to be answered by the experiments. The terms 'summed per-neighbour residual model' and 'transfer penalty' are explained only later (Ch.3, Section 3.4).
- **Evidence:** Judgement from reading.
- **Suggested direction:** Keep the question in the list item and move the disclaimer to the paragraph below, which already repeats it.
- **Confidence:** MEDIUM — readability judgement

### S-006 — Terms that a BSc reader has to guess in Ch.1: downwash, blade flapping, slung payload, rotor-speed sensing, wrench-free 'residual'

- **Severity:** NICE
- **Location:** `docs/thesis/ch1_introduction.tex` L107 (§Gap `sec:intro-gap`)
- **Quote:** “\emph{self-induced}: a single vehicle, with blade flapping, drag, ground forces, or an optional”
- **Problem:** I know what downwash roughly is, but not 'blade flapping', 'slung payload' or why 'rotor-speed sensing' is a prerequisite for measuring a residual. I assumed: flapping = blades bending; slung payload = hanging load; rotor-speed sensing = a tachometer. The residual itself is defined well in words in Section 1.2.
- **Evidence:** Chapter 3 later explains the residual (see S-finding on the definition order); none of these terms is glossed in Ch.1.
- **Suggested direction:** A one-line gloss at first use, or a forward pointer to the notation section of Ch.3.
- **Confidence:** MEDIUM — my own background; another reader may know these

### S-007 — The outline names Ch.6–9 only as 'the two- and multi-robot results chapters' and ends with 'the concluding chapter concludes'

- **Severity:** NICE
- **Location:** `docs/thesis/ch1_introduction.tex` L248 (§Outline `sec:intro-outline`)
- **Quote:** “\textbf{The discussion chapter} interprets them against the research questions and the stated”
- **Problem:** The outline gives chapter numbers for Chapters 2–5 but not for 6–9, and the last sentence says the concluding chapter concludes, which adds nothing. A reader who wants to jump to the results cannot do it from the outline.
- **Evidence:** Outline section of Ch.1 (quote above is the first of the three closing sentences).
- **Suggested direction:** Use `Chapter~\ref{...}` for all nine chapters and say in half a sentence what the last chapter delivers (answers to RQ1–4, limits, next steps).
- **Confidence:** HIGH — text read

### S-008 — Table 2.1 runs off the page; I cannot read the last column

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch2_related_work.tex` L420 (§How these methods are evaluated `sec:rw-eval`)
- **Quote:** “\caption{Evaluation conditions across the interaction-force literature. Each result is reported”
- **Problem:** The table that is supposed to prove the key claim of the chapter (results cannot be composed) is cut off on the right, so the 'reported quantity' column is not readable in the PDF. I also could not tell from the caption what the abbreviations in the cells mean.
- **Evidence:** Rendered page 16 of the temp build (viewed as an image) shows the last column cut at the page edge; the LaTeX log reports an overfull box of 186 pt for this table.
- **Suggested direction:** Narrower columns or `tabularx`; expand abbreviations in the caption.
- **Confidence:** HIGH — page image and log both checked

### S-009 — The Hsieh et al. paragraph carries about a dozen numbers and four caveats in a single block

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch2_related_work.tex` L291 (§Injecting the prediction: the controller matters `sec:rw-inject`)
- **Quote:** “ROM-FBL. On hardware they match the approximately \SI{5}{\centi\metre} RMSE and \SI{10}{\centi\metre}”
- **Problem:** I could not hold the paragraph together: percentages for three comparisons, a 20-fold solve-time ratio, a sevenfold and an order-of-magnitude statement, two training durations, a platform description and a loop rate. I lost track of which number belongs to which claim. The care with qualifiers is clear, but the reader pays for it.
- **Evidence:** Paragraph runs from `docs/thesis/ch2_related_work.tex` L287: “Their abstract reports a \SI{31}{\percent} reduction in average tracking error against nominal” to the 'onboard' sentence a page later.
- **Suggested direction:** Split into three paragraphs (what they claim; what the hardware shows; what it means for us) or move the numbers into a small table. No qualifier or number removed.
- **Confidence:** HIGH — readability judgement; content not questioned here

### S-010 — ROM-FBL, FBL, MPC/NMPC are used with no expansion; 'reduced-order model' never appears

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch2_related_work.tex` L291 (§Injecting the prediction: the controller matters `sec:rw-inject`)
- **Quote:** “ROM-FBL. On hardware they match the approximately \SI{5}{\centi\metre} RMSE and \SI{10}{\centi\metre}”
- **Also at:** `docs/thesis/ch2_related_work.tex` L314 (§Injecting the prediction: the controller matters `sec:rw-inject`): “constraint of a nonlinear MPC; Li et al.~\cite{li2023ndpnmpc} inject a predicted downwash-force”
- **Problem:** 'ROM-FBL' and 'learned FBL' are the two Hsieh variants that the paragraph compares; I did not know what ROM stands for and had to guess 'reduced-order model'. 'nonlinear model predictive control' is spelled out once and then 'MPC' and 'NMPC' appear without a link.
- **Evidence:** Search for `reduced-order` / `reduced order` in ch1–ch9: no match. First expansion of nonlinear MPC: `docs/thesis/ch2_related_work.tex` L294: “maximum-\(z\) error can be worse than that of nonlinear model predictive control. The \num{20}-”.
- **Suggested direction:** Add the expansions in the text where the terms first occur and in an abbreviation list.
- **Confidence:** HIGH — text search

### S-011 — The rigid-body equations are printed in Ch.2 and again in Ch.3 under different labels

- **Severity:** NICE
- **Location:** `docs/thesis/ch2_related_work.tex` L67 (§Multirotor dynamics and baseline control `sec:rw-baseline`)
- **Quote:** “in a world frame with $e_3$ pointing up, where $f_i$ and $\tau_i$ are the collective thrust and”
- **Problem:** Section 2.1 prints the translational and rotational equations with their own labels, and Chapter 3 prints the same equations again (with `model-` labels) and calls them 'the model'. I wondered whether the small differences are intentional. They are not, as far as I can tell, but I had to compare the two by hand to be sure.
- **Evidence:** Ch.3 version: `docs/thesis/ch3_system_modelling.tex` L68: “m\,\dot{v}_i &= -m g e_3 + f_i R_i e_3 + f_{\mathrm{res},i}, \label{eq:model-trans}\\”.
- **Suggested direction:** Refer back to one printed version (probably Ch.3) and keep only a sentence in Ch.2.
- **Confidence:** MEDIUM — text comparison

### S-012 — 'Differencing' is the key step for isolating the interaction, but no chapter says which flights are differenced

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch3_system_modelling.tex` L82 (§Rigid-body model with a residual wrench `sec:model-dynamics`)
- **Quote:** “by \emph{differencing}, comparing the residual measured in formation against the residual”
- **Problem:** Chapter 3 says the interaction can only be isolated by comparing the residual in formation with the residual measured alone under the same trajectory. I looked in Chapter 5 for the 'alone' flights: the scenario table has one single-vehicle scenario (C5, near-ground) and the null scenarios C1–C3, which are formation flights at equal height. The word 'solo' or 'reference flight' does not occur in Chapters 1–9. So I do not know what the second term of the difference is, or how the learned model's training label is built.
- **Evidence:** Search for `solo|reference flight|flown alone|single-vehicle reference` over ch1–ch9: no match. Table row: `docs/thesis/ch5_experimental_setup.tex` L135: “C5 & Near-ground pass, one vehicle & Ground effect alone, separable from downwash \\”.
- **Suggested direction:** Add two sentences to Ch.5 (Data collection): which single-vehicle flights are made, on which trajectories, and how the difference is formed.
- **Confidence:** MEDIUM — absence by search; the protocol may exist in the lab documents (not read for this pass)

### S-013 — `a_des` means one thing in Ch.3 and another in Ch.4

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch3_system_modelling.tex` L161 (§Measuring the residual `sec:model-measure` › §Sign convention in the control law)
- **Quote:** “flight. From \eqref{eq:model-trans}, holding a desired acceleration $a_{\mathrm{des}}$ requires”
- **Also at:** `docs/thesis/ch4_control_architectures.tex` L97 (§The position and attitude loop `sec:ctrl-common` › §Position loop): “in the horizontal and vertical axes. The commanded thrust vector is $m\,a_{\mathrm{des}}$; its”
- **Problem:** In Chapter 3, to hold a desired acceleration the thrust vector is m a_des + m g e3 − f_res. In Chapter 4, 'the commanded thrust vector is m a_des', and a_des already contains g e3 and minus the residuals. I could not make the two agree and had to work out that Ch.4's quantity is the whole bracket of Ch.3's equation. The symbol overload is exactly where I most need to be sure what goes into what.
- **Evidence:** Equation labels `eq:model-sign` (Ch.3) and `eq:ctrl-ades` (Ch.4), compared by substituting one into the other.
- **Suggested direction:** Give the Ch.4 quantity its own name and say once how it relates to Ch.3's a_des.
- **Confidence:** HIGH — algebra checked

### S-014 — Ch.3 says a_meas comes from the accelerometer, then that it is 'differentiated'

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch3_system_modelling.tex` L146 (§Measuring the residual `sec:model-measure` › §What this measurement requires, and how it fails)
- **Quote:** “\paragraph{It is delayed.} $a_{\mathrm{meas}}$ is differentiated from noisy inertial measurement”
- **Problem:** A page earlier the text says the accelerometer gives a_meas directly (rotate, subtract gravity). Now it is 'differentiated from noisy inertial measurement'. As a student I read this as: velocity is differentiated, or the gyro is differentiated, and could not tell which quantity is meant.
- **Evidence:** Earlier statement: `docs/thesis/ch3_system_modelling.tex` L119: “Both terms are available onboard. The measured acceleration comes from the accelerometer, which”.
- **Suggested direction:** State which signal is filtered (the accelerometer, for force) and which is differentiated (the gyro, for angular acceleration).
- **Confidence:** HIGH — both passages read

### S-015 — The deep-sets idea is given as an equation without a plain-words picture; 'latent space' and 'embedding' are not explained

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch3_system_modelling.tex` L242 (§Predicting the residual from relative state `sec:model-deepsets` › §The superposition assumption)
- **Quote:** “Equation~\eqref{eq:model-deepsets} assumes the aggregate interaction is a sum of per-neighbour”
- **Problem:** Equation (model-deepsets) is the centre of the predictive strategies. I understood only after reading it three times that phi turns each neighbour's relative position and velocity into 8 numbers, the 8-vectors are added up over neighbours, and rho turns the sum into a 3-vector acceleration. 'Latent space', 'embedding' and 'permutation-invariant' are used without explanation. There is no figure of the network and no example with numbers (for example one neighbour, 0.2 m below).
- **Evidence:** Layer sizes are given in `docs/thesis/ch3_system_modelling.tex` L222: “The concrete network is small: $\phi$ maps $6 \rightarrow 16 \rightarrow 16 \rightarrow 8$ and”; the word 'embedding' occurs once in the whole draft.
- **Suggested direction:** One plain-words sentence after the equation and a small block diagram (input, phi per neighbour, sum, rho, output).
- **Confidence:** HIGH — reading experience; background of a BSc student

### S-016 — Rotor speed is written with varpi, which looks like omega in print; omega is also body rate

- **Severity:** NICE
- **Location:** `docs/thesis/ch3_system_modelling.tex` L57 (§Notation and conventions `sec:model-notation`)
- **Quote:** “Rotor $k$ spins at $\varpi_k$ and produces thrust $k_{t,k}\varpi_k^2$ along the body $z$-axis.”
- **Problem:** In the PDF the two letters (varpi for rotor speed, omega for angular velocity) are hard to tell apart. Chapter 4 adds capital Omega for Lee's body rate. Three similar symbols for three different things, and the chapter never says 'do not confuse'.
- **Evidence:** Chapter 4: `docs/thesis/ch4_control_architectures.tex` L163: “with $e_\Omega = \Omega - R^\top R_d \Omega_d$.”.
- **Suggested direction:** Pick a clearly different symbol for rotor speed (for example n_k) or add one sentence of warning.
- **Confidence:** MEDIUM — typographic judgement

### S-017 — STM32, 'specific force', 'allocator', 'crossover' are used without a gloss

- **Severity:** NICE
- **Location:** `docs/thesis/ch3_system_modelling.tex` L217 (§Predicting the residual from relative state `sec:model-deepsets` › §Deep-sets parameterisation)
- **Quote:** “widths and the resulting \num{987} parameters are ours, sized for the STM32 control period. This is”
- **Also at:** `docs/thesis/ch3_system_modelling.tex` L342 (§Assumptions and where they fail `sec:model-assumptions`): “Prediction degrades while measurement does not; this is the crossover of \textbf{RQ2} \\”
- **Problem:** I guessed: STM32 = the vehicle's microcontroller; specific force = acceleration measured by an accelerometer including gravity effects; allocator = the part that maps thrust and torque onto four motors; crossover = the separation or speed at which one strategy becomes better than the other. 'Crossover' is used again in Ch.6 and Ch.8 as if defined.
- **Evidence:** First occurrences found by search: `crossover` at Ch.3 (table), then Ch.6 and Ch.8; never defined.
- **Suggested direction:** A short gloss at first use; define 'crossover' once in Section 1.4 (next to RQ2).
- **Confidence:** MEDIUM — text search plus my own assumptions

### S-018 — Table 4.1 is the map of the thesis but uses four undefined column ideas ('Knows', 'Enters at', 'Rotor speed', 'unflown')

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L56 (front matter / chapter opener)
- **Quote:** “1 & Pure INDI & measured & $a_{\mathrm{des}}$, torque & yes & flying \\”
- **Problem:** The table is the best one-page summary of the strategies. To read it I needed INDI, NN, FBL, the symbol a_des and 'torque' as an injection point, and I did not yet know what 'flying' versus 'implemented, unflown' means. 'Knows' is explained in the caption, 'Enters at' is not.
- **Evidence:** Table 4.1 as printed; its caption defines only 'Knows'.
- **Suggested direction:** Add one sentence per column in the caption, expand the abbreviations in the cells, and give 'Status' a date.
- **Confidence:** MEDIUM — reading experience

### S-019 — The INDI torque equation has no plain-words companion; I could not see why it cancels a disturbance

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L209 (§Strategy 1: pure INDI `sec:ctrl-s1`)
- **Quote:** “Because \eqref{eq:ctrl-indi-tau} corrects relative to what is actually being achieved, it rejects”
- **Problem:** The equation tau = tau_cur + J (alpha_ref − alpha_meas) is given, and then the claim that it rejects any disturbance appearing in alpha_meas. I could not reconstruct why from the text: that the increment adds exactly the torque that would produce the missing angular acceleration if J is right is never said. INDI is the thesis's 'reactive' strategy, so this is the one equation I would like explained in words and with one number.
- **Evidence:** Equation `eq:ctrl-indi-tau` and the sentence above; the position-level INDI is not given as an equation at all, only as the minus-a_res term of `eq:ctrl-ades`.
- **Suggested direction:** Two sentences of intuition (a numerical example if possible) and a pointer to the position-level counterpart.
- **Confidence:** HIGH — reading experience; background of a BSc student

### S-020 — 'Through the usual flatness construction' hides the step from thrust vector to attitude, and flatness is used before it is explained here

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L99 (§The position and attitude loop `sec:ctrl-common` › §Position loop)
- **Quote:** “with the desired yaw, gives the desired attitude $R_{\mathrm{des}}$ through the usual flatness”
- **Problem:** I could not follow how a commanded thrust vector and a yaw give a desired rotation matrix. Differential flatness is explained in one paragraph of Chapter 2 (Section 2.1), and 'snap' is used there without definition. In Chapter 4 it is referred to as 'usual'.
- **Evidence:** Chapter 2 paragraph: `docs/thesis/ch2_related_work.tex` L85: “A second property of the quadrotor is used throughout: differential flatness. Position and yaw,”. The repository GLOSSARY has a definition of snap (`flying_drone_stack/GLOSSARY.md` L23: “Rate of change of jerk. Fourth derivative of position.”), but it is not part of the thesis.
- **Suggested direction:** A three-line recipe (z-axis of the body = thrust direction; yaw fixes the other two axes) or a pointer to the Lee paper's construction.
- **Confidence:** HIGH — reading experience

### S-021 — Attitude-loop symbols: omega_des is never defined, Omega/omega and R_d/R_des alternate, and K_R changes meaning between equations

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L113 (§The position and attitude loop `sec:ctrl-common` › §Attitude loop)
- **Quote:** “$e_\omega = \omega - \omega_{\mathrm{des}}$. In the geometric configuration the torque is”
- **Also at:** `docs/thesis/ch4_control_architectures.tex` L194 (§Strategy 1: pure INDI `sec:ctrl-s1`): “\alpha_{\mathrm{ref}} = \alpha_{\mathrm{des}} - K_R e_R - K_\omega e_\omega,”
- **Problem:** I could not tell what omega_des is, how it is obtained, or whether K_R in the torque law and K_R in the angular-acceleration law are the same numbers (their units differ). Two different symbol sets are used for the same law (the implemented one and the source's, with capital Omega and R_d); no table maps them. Psi (the attitude error function) and the 'vee' operator are used without definition.
- **Evidence:** Source-law version in the same chapter: `docs/thesis/ch4_control_architectures.tex` L163: “with $e_\Omega = \Omega - R^\top R_d \Omega_d$.”.
- **Suggested direction:** Define omega_des, add subscripts or units for the two gain sets, and add one line mapping implemented to source symbols.
- **Confidence:** HIGH — equations compared directly

### S-022 — No gain values, units or on/off state anywhere in the thesis for K_p, K_v, K_i, k_i,att

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L89 (§The position and attitude loop `sec:ctrl-common` › §Position loop)
- **Quote:** “+ K_p e_p + K_v e_v + K_i \!\int\! e_p \,\mathrm{d}t”
- **Problem:** The position law has three gain matrices and the torque law has an integral gain. The chapter says 'diagonal but distinct in the horizontal and vertical axes' and nothing else. I cannot tell whether the integral terms are active, what the gains are, or what 'tuned' means numerically. For a thesis whose fairness argument is per-strategy tuning, the tuned values are what the reader would want to see.
- **Evidence:** Search for `K_p|K_i|k_{i` over ch5 and ch6: no gain table or value. Sentence in Ch.4: `docs/thesis/ch4_control_architectures.tex` L96: “with $e_p = p_{\mathrm{des” - p$ and $e_v = v_{\mathrm{des}} - v$, and gains diagonal but distinct}}.
- **Suggested direction:** A gain table per strategy (values, units, source of tuning) in an appendix, referenced from Section 4.6.
- **Confidence:** MEDIUM — may be intended for the results chapters

### S-023 — Strategy 3 opens with 'Implemented. Not implemented.'

- **Severity:** NICE
- **Location:** `docs/thesis/ch4_control_architectures.tex` L274 (§Strategy 3: feedback linearisation with a flatness-preserving residual `sec:ctrl-s3`)
- **Quote:** “\paragraph{Implemented.} \textbf{Not implemented.} The controller code was requested from the”
- **Problem:** Two run-in headings in a row. It reads like a leftover of the template, and a reader skimming the paragraph headings sees 'Implemented'. The next sentences are clear, but the first impression is the wrong way round.
- **Evidence:** Line quoted.
- **Suggested direction:** Drop the first run-in heading for this strategy.
- **Confidence:** HIGH — text read

### S-024 — 'Lower-triangular structure', 'flat output' and 'flatness-preserving' are used without a picture of what they mean

- **Severity:** NICE
- **Location:** `docs/thesis/ch4_control_architectures.tex` L284 (§Strategy 3: feedback linearisation with a flatness-preserving residual `sec:ctrl-s3`)
- **Quote:** “correction inside the state derivative, not a free wrench, preserve it, and with it the original”
- **Problem:** I understood that a wrongly shaped learned correction can break a good structural property and a correctly shaped one cannot, but not what 'lower-triangular' means for a correction or what the flat output is. The strategy is not implemented, so this costs only understanding, not the comparison.
- **Evidence:** Sentence begins `docs/thesis/ch4_control_architectures.tex` L282: “whose validity depends on the augmented system remaining differentially flat. A generic learned”.
- **Suggested direction:** One sentence with the quadrotor example (which states the correction may depend on) or a pointer to Yang et al.
- **Confidence:** MEDIUM — reading experience

### S-025 — Runtime selection is described with 'one bit ... another ... a third parameter'; I cannot map them to the four configurations

- **Severity:** NICE
- **Location:** `docs/thesis/ch4_control_architectures.tex` L130 (§The position and attitude loop `sec:ctrl-common` › §Runtime selection)
- **Quote:** “The control law is selected by a parameter at runtime rather than by recompiling: one bit enables”
- **Problem:** The next page lists geometric, position-INDI only, attitude-INDI only, full INDI. I would like a small truth table of the switches and the configurations they give, and which of them the campaign flies.
- **Evidence:** Configurations listed in `docs/thesis/ch4_control_architectures.tex` L215: “The position and attitude increments are independently selectable, giving four configurations:”.
- **Suggested direction:** A four-row table (switch settings → configuration → used in).
- **Confidence:** MEDIUM — reading experience

### S-026 — The same controller is called 'geometric baseline', 'uncompensated baseline' and 'uncompensated geometric baseline'

- **Severity:** NICE
- **Location:** `docs/thesis/ch4_control_architectures.tex` L142 (§Strategy 0: uncompensated geometric baseline `sec:ctrl-s0`)
- **Quote:** “\section{Strategy 0: uncompensated geometric baseline}”
- **Also at:** `docs/thesis/ch4_control_architectures.tex` L55 (front matter / chapter opener): “0 & Geometric baseline & nothing & --- & no & flying \\”
- **Problem:** Three names for Strategy 0 across Ch.1, Ch.4 and Ch.6. 'Uncompensated' also sounds as if the controller has no integral action, which is not what is meant (it records but does not use the residual).
- **Evidence:** Ch.1 RQ1 says 'the uncompensated baseline'; Ch.4 table says 'Geometric baseline'.
- **Suggested direction:** Choose one name and define it once.
- **Confidence:** MEDIUM — text read

### S-027 — The Campaign status paragraph uses repository jargon I cannot parse (merges, role alignment, manifest, QA shakedowns, C.1)

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L30 (front matter / chapter opener)
- **Quote:** “the brushless pair \textbf{cf5} (study vehicle) and \textbf{cf\_second} (top). Micro-SD merges”
- **Problem:** 'Micro-SD merges with per-vehicle role alignment replaced radio-only windows', 'eight verified geometric merges', 'five C.1 training-eligible A1/A3 cells plus three A8 QA shakedowns', a JSON manifest filename and 'the standard analysis suite' all depend on knowledge that is not in the thesis. I guessed: merge = combine two vehicles' SD logs; role alignment = deciding which log is top and bottom; QA shakedown = a test flight. 'Study vehicle' vs 'top' is not explained either.
- **Evidence:** Other pointers in the same paragraph: `docs/thesis/ch5_experimental_setup.tex` L34: “\texttt{c1\_2026\_09\_21\_merged/manifest\_2026\_09\_21\_c1.json}), with dashboards and phase”.
- **Suggested direction:** Rewrite the status in terms of what was flown (scenarios, number of flights, which vehicle where) and move file names to an appendix or footnote.
- **Confidence:** HIGH — reading experience

### S-028 — 'C.1', 'C.4' (campaign phases) are never defined in the thesis, and they collide with the scenarios C1–C5

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L37 (front matter / chapter opener)
- **Quote:** “training data collection (C.1) \emph{has begun} under”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L131 (§The formation library `sec:setup-scenarios`): “C1 & Side-by-side coplanar $\ast$ & A residual here, and not in A1, is not downwash \\”
- **Problem:** The phase labels C.0–C.4 occur 33 times in the draft (first in Ch.5). Only C.1 (training collection) and C.4 (comparison) can be guessed from the text; C.0, C.2 and C.3 appear only in comments or `\todo` notes. C1 (side-by-side coplanar) is a scenario, and 'C.1' is a phase: I read 'the C.1/C.4 matrix' in Chapter 6 as a mix-up the first time.
- **Evidence:** In Ch.6: `docs/thesis/ch6_results_2robot.tex` L38: “the C.1/C.4 matrix prioritises vertical separation and downwash-dominated geometries (e.g.\ A1,”. No table of phases exists in the thesis text.
- **Suggested direction:** Add a small table of the five phases (name, purpose, controller used, output) at the start of Chapter 5, and rename either the phases or the scenarios.
- **Confidence:** HIGH — text search and reading experience

### S-029 — Shop-floor terms: 'trajectory ladder', 'dz/speed grid', 'mocap', 'uSD', 'DShot', 'wake rather than wash'

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L233 (§Protocol `sec:setup-protocol` › §Hardware gate)
- **Quote:** “geometric first and then INDI, on the trajectory ladder.”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L39 (front matter / chapter opener): “but the planned dz/speed grid is only partly filled (see \texttt{docs/25} coverage table).”
- **Problem:** I assumed: trajectory ladder = a sequence of flights of increasing difficulty; dz/speed grid = a table of vertical separations times speeds; mocap = motion capture; uSD = micro SD card; DShot = a digital motor protocol that can return rotor speed. None is explained. 'Wake' versus 'wash' is the difference between C2 and A-scenarios in the table, but neither word is defined.
- **Evidence:** Table cell: `docs/thesis/ch5_experimental_setup.tex` L132: “C2 & Leader--follower line $\ast$ & Wake rather than wash \\”. `DShot` occurs once in the draft; `uSD` is used in Ch.5 L340 after 'micro SD deck' earlier.
- **Suggested direction:** Gloss at first use or add to the abbreviation list.
- **Confidence:** HIGH — text search and my own assumptions

### S-030 — The sixteen scenarios are described by purpose but have no separations, speeds, durations or sketches

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L94 (§The formation library `sec:setup-scenarios`)
- **Quote:** “Sixteen scenarios, fixed before any data collection and not modified afterwards. Freezing them is”
- **Problem:** I can read the table but cannot picture A4 'offset stack' or A7 'dynamic merge': offset by how much, over how long, at what speed? Chapter 1 argues that the interaction is strong only inside about 2.5 rotor distances, but I cannot check which scenarios are inside that range.
- **Evidence:** Search of the scenario section for `\SI`: only the floor and volume values.
- **Suggested direction:** A parameter table and one sketch per group (A, B, C).
- **Confidence:** HIGH — section read in full

### S-031 — 'Half of its six inputs' and 'two of the six inputs' disagree; I did not know the six inputs until I went back to Ch.3

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L167 (§The formation library `sec:setup-scenarios` › §Regimes, and why the library spans them)
- **Quote:** “trained only on such data has seen no variation in half of its six inputs and is extrapolating the”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L174 (§The formation library `sec:setup-scenarios` › §Regimes, and why the library spans them): “produced training data in which two of the six network inputs never varied, which the training”
- **Problem:** Ch.5 says 'half of its six inputs' (three) in one paragraph and 'two of the six' in the next. The six inputs are relative position and velocity (Ch.3), which Ch.5 does not repeat. I tried to count for a purely vertical stack and got four constant ones (dx, dy, dvx, dvy).
- **Evidence:** Inputs defined in `docs/thesis/ch3_system_modelling.tex` L188: “f_{\mathrm{res},i} \;\approx\; \mathcal{F}\!\left( \{\, p_j - p_i,\; v_j - v_i \,\}_{j \neq i} \right),”.
- **Suggested direction:** State the six inputs where the argument is made and fix the count.
- **Confidence:** MEDIUM — my own counting; the pipeline report was not inspected

### S-032 — The metric called 'Residual rejection' is the separation error between vehicles, not a residual

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L290 (§Protocol `sec:setup-protocol` › §Metrics)
- **Quote:** “\paragraph{Residual rejection.} The error in the \emph{separation between vehicles}, measured”
- **Problem:** Chapter 3 defines the residual as an acceleration. Here 'residual rejection' is a position-type error between two vehicles, and a second sentence then says 'alongside it, the magnitude of the measured residual that remains'. I could not tell which of the two is the headline for RQ1's 'residual-force rejection'.
- **Evidence:** RQ1 wording: `docs/thesis/ch1_introduction.tex` L122: “in trajectory-tracking accuracy and residual-force rejection?”.
- **Suggested direction:** Rename the first metric (for example 'separation error') and name the residual metric separately.
- **Confidence:** HIGH — text read

### S-033 — The threat table says the merge tool 'aborts above 15 cm RMS role check'; I do not know what a role check is

- **Severity:** NICE
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L340 (§Threats to validity `sec:setup-validity`)
- **Quote:** “Residual-time alignment across vehicles depends on merge quality; \texttt{merge\_usd\_logs.py} aborts above”
- **Problem:** 'Role' (top or bottom vehicle) is never introduced; 'RMS role check' sounds like a test of who is who, but the 15 cm suggests a position error. The cell also names a script that the thesis does not describe.
- **Evidence:** Cell as printed, ending "... RMS role check \\" on the same line.
- **Suggested direction:** Say in words what is checked and why it can fail.
- **Confidence:** MEDIUM — reading experience; the merge tool was not part of this pass

### S-034 — The conclusion says Strategies 0–3 'were implemented'; Chapter 4's table says Strategy 3 is not

- **Severity:** MUST-FIX
- **Location:** `docs/thesis/ch9_conclusion.tex` L24 (§Summary `sec:conclusion-summary`)
- **Quote:** “were implemented and specified in Chapters~\ref{ch:model}--\ref{ch:control}; the experimental”
- **Problem:** I read Chapter 4 (Table 4.1: FBL + NN 'not implemented') and then the first line of the conclusion. They cannot both be right, and a reader who only reads Chapters 1 and 9 gets the wrong picture.
- **Evidence:** Table 4.1 row: `docs/thesis/ch4_control_architectures.tex` L58: “3 & FBL + NN & predicted & linearising law & no & not implemented \\”.
- **Suggested direction:** State which of the four were implemented.
- **Confidence:** HIGH — two quoted lines contradict each other

### S-035 — Chapters 6–9 read as a plan with red notes; the references inside them print as 'Chapter 7.2' and 'Chapter 5.7'

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch8_discussion.tex` L61 (§RQ3 --- Two-robot training on three-robot formations `sec:disc-rq3`)
- **Quote:** “Chapter~\ref{sec:res3-transfer} supplies the numbers; here they are interpreted against deployment”
- **Problem:** As orientation for the later chapters this is fine: I can see what each will contain. But `Chapter~\ref` to a section label prints 'Chapter 7.2', and I looked for a chapter with that number. The tone also switches from past or present (Ch.1–5) to future conditional ('will state', 'is intended to be repeated').
- **Evidence:** Rendered text of the temp build showed 'Chapter 7.2 supplies the numbers' and similar.
- **Suggested direction:** Use `Section~\ref` for section labels.
- **Confidence:** HIGH — rendered PDF text checked

### S-036 — 'Strategy 4', 'Strategy 5/7' appear in Ch.6 and Ch.9, but only Strategies 0–3 exist in Ch.1–5

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch9_conclusion.tex` L71 (§Future work `sec:conclusion-future`)
- **Quote:** “\item \textbf{Residual RL strategies (Strategy~5/7).} Explicitly deferred in the project convention:”
- **Problem:** After Chapter 4 told me the compared set is 0–3, I met 'Strategy 4 in the wider project numbering' and 'Strategy 5/7'. I do not know what 4–7 are, why the numbering skips from 3 to 5/7, or what 'the project convention' is.
- **Evidence:** Chapter 6: `docs/thesis/ch6_results_2robot.tex` L142: “(Strategy~4 in the wider project numbering) is \emph{not} in that set unless explicitly restored to”.
- **Suggested direction:** Describe the deferred ideas in words, without project numbers.
- **Confidence:** HIGH — text read

### S-037 — The thesis points me to files and labels I cannot see: 'docs/25', 'Priority D/E', 'docs/36'

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch7_results_multirobot.tex` L58 (§Extension to four and five vehicles `sec:res3-n4sketch`)
- **Quote:** “A Priority~D/E scenario sketch (proposed IDs F4a, D1, E1, etc.) describes how n$\geq$4 formations”
- **Problem:** As a reader of the PDF, 'see docs/25 coverage table' and 'Priority D/E sketches (docs/36)' lead nowhere. I assumed these are internal project notes, which means the sentence has no content for me.
- **Evidence:** Another instance: `docs/thesis/ch5_experimental_setup.tex` L39: “but the planned dz/speed grid is only partly filled (see \texttt{docs/25} coverage table).”.
- **Suggested direction:** Describe the object in the text, or cite an appendix.
- **Confidence:** HIGH — text read

### S-038 — No list of abbreviations, list of symbols or glossary, although the repository has a glossary

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/thesis_draft.tex` L5 (front matter / chapter opener)
- **Quote:** “\tableofcontents”
- **Problem:** The draft front matter has only a table of contents. Between them the chapters introduce about 40 abbreviations and a few dozen symbols, many defined once in one chapter and used in another. To follow Chapter 4 I had to go back to Chapters 2 and 3 repeatedly. The repository already has a glossary of technical terms, but the thesis does not include it or any derived list.
- **Evidence:** Draft front matter: lines 1–5 of `thesis_draft.tex` (no `\listoffigures`, `\printglossary`, `\nomenclature`, `\maketitle` or abstract found by search). Repository glossary: `flying_drone_stack/GLOSSARY.md` L3: “Short precise definitions of technical terms used throughout this project.”.
- **Suggested direction:** Add a list of abbreviations and a table of symbols (symbol, meaning, unit, first defined in) after the contents.
- **Confidence:** HIGH — search of the draft source

### S-039 — There is not a single figure; the stack geometry, the control loop and the network are all described only in words

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch1_introduction.tex` L28 (§Motivation `sec:intro-motivation`)
- **Quote:** “When several quadrotors fly close together, the downwash of the upper vehicle can destabilise the”
- **Problem:** I would have drawn: two quadrotors with the downwash cone (Ch.1), the cascade of position loop, attitude loop, INDI increment and injection points (Ch.4), the deep-sets network (Ch.3), and the lab layout with the sixteen scenarios (Ch.5). Without any of them the thesis is 50 pages of text and tables for someone like me.
- **Evidence:** Search for `\begin{figure}|\includegraphics|tikzpicture` over ch1–ch9: no match. Tables: Ch.1 ×1, Ch.2 ×1, Ch.3 ×1, Ch.4 ×1, Ch.5 ×2, Ch.6 ×3.
- **Suggested direction:** Four figures (see 'Prerequisites and intuition' for where).
- **Confidence:** HIGH — search count

### S-040 — No title page, abstract, or declaration in the draft build

- **Severity:** NICE
- **Location:** `docs/thesis/thesis_draft.tex` L3 (front matter / chapter opener)
- **Quote:** “\input{preamble_draft}”
- **Problem:** The draft starts with the contents. A reader cannot tell the title, author, date or version, nor read a one-page summary before Chapter 1. This is expected for a work-in-progress build but it is the first thing an outside reader misses.
- **Evidence:** `thesis_draft.tex` lines 1–5; search for `\maketitle` and `abstract` in the draft sources: no match.
- **Suggested direction:** Add a title page and a 200-word abstract (final text can wait until results exist).
- **Confidence:** HIGH — search count

## Report counts

- Findings: **40** (MUST-FIX 2, SHOULD-FIX 26, NICE 12).
- Terms / symbol groups audited: **35** (defined before or at first use: 4; defined only later: 3; partly: 1; assumed known: 1; never defined: 26).
- Reading-flow items: **16** listed in section B.
- Questions to the author: **50** (10 each for Ch.1–5).

## What could not be done

- I am a simulated student: the 'assumed meaning' column reflects a typical BSc background, not a measured one. A real colleague outside the field would find other places.
- I did not read the later chapters' `\todo` text exhaustively; Ch.6–9 were read for orientation only, as requested.
- I did not open the repository documents behind the status paragraph (merge manifests, coverage tables) for this pass, so I could not check whether the 'solo' reference flights exist in practice (S-012); I only report that the thesis text does not say.
- Page images: only the page with Table 2.1 was inspected visually; other layout problems may exist.

## Verified facts vs judgement

Verified facts: every quote, every line number, every count of terms and tables, and every 'search found nothing' statement. Judgement: what I assumed a term meant, the ratings in the readability table, the 'what is missing' column and the severities.


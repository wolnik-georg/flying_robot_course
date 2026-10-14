# Stage 4 — Chapters 6–9 (skeletons): passes A–F

Revised files: `revised/ch6_results_2robot.tex`, `revised/ch7_results_multirobot.tex`, `revised/ch8_discussion.tex`,
`revised/ch9_conclusion.tex`. Full build (all nine chapters revised) compiles with no errors, no undefined references or
citations (60 pages). The skeletons contain no result (H1): every slot is a `\todo` naming the table/figure, the
statistic and the negative answer.

## Verdicts

**Ch6.** (1) Structure follows Ch5's metrics and statistics exactly, so the results can be filled without redesign.
(2) New section "Sign and loop attribution" holds the checks Ch3/Ch4 promise; it cannot be written before D-01.
(3) Prediction-off/on layers made explicit. (4) The "Strategy 4" hybrid section is removed (kept as a comment; its label
retained as an anchor). (5) §6.1 status paragraphs and the Strategy 3 row are D-05.

**Ch7.** (1) Transfer penalty defined by reference to Ch5. (2) Simulation removed from the evidence chain (H1).
(3) Table columns limited to implemented strategies (H5). (4) Fallback narrative kept. (5) Roster sentence D-05.

**Ch8.** (1) RQ sections kept. (2) Three sections added that earlier chapters promise (assumptions table, moment residual,
frozen gains/Strategy 3 absence). (3) Internal vocabulary removed. (4) No interpretation written. (5) One D-05 comment.

**Ch9.** (1) Answer slots now state the form of each answer including the negative one. (2) "C.4" removed from prose.
(3) Future-work list (internal vocabulary and campaign tasks) is D-05, unchanged. (4) Ch6–9 named in prose (H6).

## Rubric (0–5), before → after (skeletons judged as skeletons)

| Criterion | Ch6 | Ch7 | Ch8 | Ch9 |
|---|---|---|---|---|
| 1 Correctness | 3→4 | 3→4 | 3→4 | 3→4 |
| 2 Completeness | 3→4 | 3→4 | 2→4 | 3→4 |
| 4 Academic standard | 2→3 (D-05 open) | 3→4 | 3→4 | 2→3 (D-05 open) |
| 7 Cross-chapter consistency | 2→4 | 3→4 | 3→4 | 3→4 |

## A. Claim re-check
No literature claims added or changed. Implementation statements: Ch6 "No other INDI controller is reported" (H5,
consistent with Ch4 §4.3: diagnostic configurations are not strategies); Ch8 "force only, moment left to the attitude
loop" (Ch3 §3.2, code: no separate τ_res term in the `lib.rs` torque branches; the INDI increment absorbs moment disturbances as part of the attitude loop).

## B. Defect log

| # | Defect | Sev | Handling |
|---|---|---|---|
| B6-1 | file and variable names in todos (`effort_proxy`, `min_sep_m`, `firmware_forward`, `full_bank_*.npz`) | S3 | APPLY |
| B6-2 | "any reference INDI flights included for context" (H5: which INDI implementation) | S2 | APPLY |
| B6-3 | Strategy 4 section in a main chapter | S2 | APPLY (commented) |
| B7-1 | Ch7 table "Strategy 0–3" includes an unimplemented strategy | S2 | APPLY (0–2) |
| B7-2 | simulation as evidence for the summed-architecture decision | S1 | APPLY |
| B8-1 | promised sections missing (assumptions, moment residual) | S2 | APPLY |
| B9-1 | numeric refs to Ch6–9 in prose (H6) | S3 | APPLY |

## C. Rewrite
All four files complete. New labels: `sec:res2-attribution`, `sec:disc-assumptions`, `sec:disc-moment`,
`sec:disc-frozen`. `sec:res2-hybrid` kept as an anchor (resolves to the enclosing section; no reference to it exists in
the thesis).

## D. Citation audit
No `\cite` in Ch6–9. Nothing to audit.

## E. Examiner simulation (on the skeletons)

| # | Question | Where answered |
|---|---|---|
| 1 | What is n per cell? | Ch5 §5.5.3 `\todo`; Ch6 §6.1 table slot |
| 2 | What counts as "no difference"? | Ch6 negative-answer paragraphs (null spread) |
| 3 | How do you know a Strategy 1 effect is the position loop? | Ch6 §6.5 |
| 4 | Is the sign right? | Ch6 §6.5 + D-01 |
| 5 | How is prediction quality separated from closed-loop effect? | Ch6 §6.4 |
| 6 | Transfer penalty defined how? | Ch7 §7.2 |
| 7 | What if no third vehicle? | Ch7 §7.4 |
| 8 | Does any claim rest on simulation? | Ch7 §7.4 (removed), Ch8 §8.6 |
| 9 | Cost of not compensating torque? | Ch8 new section |
| 10 | Are the assumptions confirmed? | Ch8 new section |
| 11 | What does a frozen-gain ranking generalise to? | Ch8 new section |
| 12 | Hybrid not compared — why mention it? | Ch6 removed section comment; Ch2 |

## F. Student-reader test
Each section opens with the question it answers and the metric it uses; "prediction-off" is defined where used.

## Change log
P6-01 → D-05 comment; P6-02 ✓; P6-03 ✓; P6-04 ✓ (+D-01); P6-05 ✓; P6-06 ✓; P7-01 → D-05 comment; P7-02 ✓; P7-03 ✓;
P8-01 ✓ (+D-05); P8-02 ✓; P9-01 → D-05 comment; P9-02 ✓.

## DECISION rows left open
D-01 (Ch6 §6.5); D-05 (Ch6 §6.1, Tab 6.3 Strategy 3 row; Ch7 §7.1; Ch8 frozen-gains section; Ch9 §9.4).

## [VERIFY]
None.

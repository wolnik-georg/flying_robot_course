# Round 2 — audit of the round-1 text (read-only pass)

Base: `docs/thesis_revision/revised/ch*.tex` (round-1 output). Line numbers below are those of that base. Method: a grep
over all chapters for strategy numbers and names, the hybrid's scope, FBL numbering, rates, the rotor-speed source and
filters, cut-offs, gains and the "tuned for itself" framing, integrals, residual-logging claims, mass, inertia and
thrust constants, take-off and landing, campaign counts and rotation, tiers, null scenarios, threats, the Ch6–9
placeholders, summaries and outlines, tables, cross-references and labels, implementation and file names, and
verification text (271 hits). Each hit was read in context; the stale ones are listed here. Treatment: **APPLY**
(settled by the code, a held PDF or an author decision of round 2) or **DECISION** (left, with a `% DECISION` comment).

Code facts used (comments only in the text): `crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c`
(+ .h), `src/modules/src/ns2_residual.c` (+ .h), `src/platform/interface/platform_defaults_cf21bl.h`,
`kalman_core.c:790–797`; `flying_drone_stack/firmware_app/app-config-bl:48–49`; `crazyswarm2/crazyflie/config/
study_strategies.json`, `study/tree_S0..S3.patch`; `flying_drone_stack/tools/usd_study_config.txt`; protocol draft v0.2 +
banner, `docs/study_shakedown_card.md` footer.

## Chapter 4 (rewritten as a whole)

| Line | Statement | Why stale | Treatment |
|---|---|---|---|
| 23–26 (header), 36–38 | four controllers; S3 = FBL | five strategies; S3 = hybrid, S4 = FBL | APPLY (decision 1) |
| 45–71 | Table 4.1 with "Status: flying / implemented, unflown", S3 = FBL | numbering; status refers to retired controllers | APPLY numbering and switch column; status → neutral "Implemented" column with DECISION D-05 |
| 73–83 | "fair … not a claimed shared implementation"; "each family represented by one implementation of our own" | one controller for S0–S3 | APPLY (decision 6) |
| 89–92 | S1 "replaces the attitude law's inner term with an incremental one" | the switch-B term is additive | APPLY (code L365–410) |
| 98–106 | eq. (ctrl-ades) with $K_i\int e_p$ | the vertical-only integral is active, $K_{i,z}=1.5$ | APPLY (D-02 resolved) |
| 107–129 | D-02, D-01, closing-pass comments | resolved | APPLY: removed |
| 131 | "gains diagonal but distinct in horizontal and vertical" | $K_p = 7$, $K_v = 4$ on all axes | APPLY |
| 140–156 | $R_{des}$ from $z_{des}\times x_c$; velocity-error clamp 0.6 m/s, 10 Hz LP; footnote with file names | retired implementation; file names | APPLY (code L276–282; no conditioning) |
| 158–166 | "both are subtracted … nothing else changes"; D-01/D-02 comment | now true by construction | APPLY |
| 170–192 | torque law without feedforward bracket, $e_\omega = \omega - \omega_{des}$, attitude integral "disabled"; INDI "replaces" the law | flown law has the bracket, the transported $\omega_r$ and an ACTIVE integral $K_I=0.03$ | APPLY (code L318–364) |
| 199–216 | runtime selection: "INDI bits", "each strategy's own gains, the sign, ki_z"; D-11 comment | three switches, shared gains, no sign parameter | APPLY |
| 228–235 | S0 "Implemented": $e_\omega$ not transported | wrong for the flown law | APPLY |
| 249–264 | "reduced form … omits the bracket" | the flown law has the bracket | APPLY: deviations rewritten (integrals, $R_d$, inertia) |
| 281–288 | S0 Role: "residual that survived compensation" | D-04 resolved | APPLY (dedicated training flights) |
| 291–420 | S1: $\alpha_{ref}$ with INDI gains, increment eq. as the flown law, effectiveness factor, torque limit, DShot, plausibility filter, unfiltered thrust, "Tal & Karaman primary" | retired implementation | APPLY: switch-A/B law, eq. (ctrl-s1-torque); increment form derived from it keeping labels; reference Cobo-Briesewitz eqs. (8)–(10) |
| 383–387 | four configurations, diagnostic bits | still true (indi bit 0 / bit 1) | APPLY (reworded as switches A/B) |
| 426–451 | S2: "every tenth control step", "compiled", "data collected under S0"; D-01/D-04 comments | rates (100 Hz main-loop ticks), dedicated training flights | APPLY |
| 457–460 | Hsieh SE(3) "main controller is … Strategy 3" | FBL is S4 | APPLY |
| 489–534 | §Strategy 3 = FBL; status "requested from the authors" | renumbered | APPLY (S4; D-05 neutral) |
| — (missing) | no hybrid section | hybrid in scope | APPLY: new §Strategy 3 with derivation; DECISION D-12 (double counting) |
| 537–576 | fairness: "each strategy tuned for itself", "differs by design: attitude law, gains, filters, sign, integral" | one controller | APPLY (decision 6) |
| 582–594 | summary; "at the time of writing two strategies fly" | numbering; status | APPLY numbering; status sentence removed with the neutral summary (D-05 recorded) |

## Chapter 3

| Line | Statement | Why stale | Treatment |
|---|---|---|---|
| 12–15 | header: code basis lib.rs / residual_nn.rs / traj_iface.c | final controller elsewhere | APPLY (comment) |
| 35–38 | "Strategy 0 records it … Strategy 3 if implemented" | one definition, computed onboard (switch A) or offline; S3 = hybrid | APPLY |
| 64–67 | $\varpi$ in rpm, one $k_{t,k}$ per rotor | one constant $k_t$; rad/s | APPLY (code L188–191, platform header :65) |
| 96–111 | "no strategy compensates the moment residual" | switch B corrects a measured moment residual | APPLY |
| 136–145 | thrust from $k_{t,k}$; accelerometer pre-filter; plausibility filter; DShot or deck; file footnote | optical deck, one $k_t$, both terms 30 Hz Butterworth; no names | APPLY |
| 163–170 | "evaluates at every control step, 1000 Hz, records in every controller mode" | 500 Hz, only with switch A; offline otherwise | APPLY |
| 187–190 | absence of rotor-speed source → zero | still true; reworded for switches | APPLY |
| 202–209 | "filters the accelerometer but not the thrust"; D-06 | both 30 Hz, design rate = run rate | APPLY (D-06 resolved) |
| 231–234 | D-04 comment | resolved | APPLY: removed |
| 261–271 | D-01 comment; sign configurable | subtracts by construction | APPLY |
| 280 | "The two-robot results chapter can check that prediction" | no inverted-sign flight exists | APPLY: removed |
| 296–304 | D-02 comment | integral active in all four | APPLY: sentence added |
| 327–333 | positions "from the motion-capture system"; footnote with file names | over the radio with time stamps; finite-difference rules | APPLY |
| 352 | "gram-equivalent unit of the reference implementation" | implementation wording | APPLY (NS2 PDF reports grams) |
| 379, 388–391 | "port of the authors' public reference code"; "compiled … upload path"; "trained on this project's flights collected under S0" | implementation wording; dedicated training flights | APPLY |
| 406–411 | "every tenth step, about 100 Hz at the 1000 Hz control rate" | every tenth main-loop tick | APPLY |
| 415–426 | "values in the residual-network source file" + footnote | file names | APPLY |
| 472–474 | "ported literally" | implementation wording | APPLY |
| 589–590 | "No rotor-speed source fitted or telemetry absent" | optical sensor; a missing rotor reading | APPLY |
| 607–618 | summary: "Strategies 1 and 2 consume it, Strategy 3 if implemented"; D-01 | numbering; sign resolved | APPLY |
| 565, 569 | "bounded by the null scenarios" | null scenarios are three-vehicle (outside protocol) | DECISION D-13 |

## Chapter 5

| Line | Statement | Why stale | Treatment |
|---|---|---|---|
| 26 | "Strategies 0–3, each as its own controller" | one controller | APPLY |
| 33–53 | campaign-status paragraph with nicknames, manifest, C.1/C.4, docs/25 | names forbidden in visible text; status of retired controllers | APPLY names removed; content DECISION D-05 |
| 57–64 | mass 41 g | controller mass parameter 42.7 g | kept (vehicle mass); difference listed as an open item |
| 70–79 | rotor-speed source "normally DShot … parameter selects" | optical deck; DShot recorded only | APPLY |
| 83–91 | "control loop runs at 1000 Hz" | controller at 500 Hz in a 1 kHz loop | APPLY |
| 126–128 | "same motion in simulation and on hardware" | verification text | APPLY |
| 174–193 | null scenarios C1–C3 | three-vehicle; outside the two-vehicle protocol | DECISION D-13 |
| 259–270 | recorded list incl. $a_{res}$, $e_R$, INDI torque, "recorded in every controller mode" | those channels are not computed by the final controller | APPLY |
| 304–316 | gate: "geometric first and then INDI"; "Tune, then freeze"; "correctly signed" | take-off handling; shared gains; no sign | APPLY + CONFIRM-AFTER-LAB (a), (c) |
| 319–332 | D-01 comment | resolved | APPLY |
| 337–344 | collection rationale "survived compensation"; D-04 | resolved | APPLY + CONFIRM-AFTER-LAB (b) |
| 354–366 | "\todo{Fix the number of repetitions}"; "interleaved within a session"; "own gains frozen" | protocol: 5 clean flights per cell, 48 cells, blocks and rotation, anchors; shared gains | APPLY (protocol v0.2 + banner; shakedown card footer) |
| 368–372 | prediction-off "flown under Strategy 0 with weights loaded" | Strategies 0 and 1 record the prediction | APPLY |
| 395–404 | "logging configuration records $e_R$" | not computed; reconstructed from attitude and desired attitude | APPLY |
| 469–490 | simulation "runs the same controller source"; "used to check that every scenario executes … find defects" | verification text | APPLY (decision 4) |
| 510–515 | threats: tuning confound, session effects, leakage, silent loss | shared gains; blocks; dedicated training flights; rotor reading | APPLY |
| 546–553 | summary: 1000 Hz, "each strategy tuned", three null scenarios | rates; shared gains; null D-13 | APPLY + DECISION D-13 |

## Chapter 1

| Line | Statement | Why stale | Treatment |
|---|---|---|---|
| 118–124 | "This thesis compares the first two; the hybrid is outside its scope" | hybrid in scope | APPLY (gap statement itself: DECISION D-03) |
| 145 | "Multi-robot evaluation of such hybrids is outside this thesis" | hybrid in scope | APPLY |
| 154–177 | "Four control strategies are compared"; S1 "pure INDI"; S3 = FBL; "A hybrid … not compared" | five strategies | APPLY |
| 196 | RQ3 applies to "Strategy 2 and, if implemented, Strategy 3" | numbering | APPLY; three-vehicle need: DECISION D-13 |
| 225–230 | contribution 1: "residual logged under every controller"; "each is tuned for itself" | one definition, reconstructed; shared gains | APPLY |
| 242–249 | apparatus: weights compiled or uploaded; "checked numerically against the compiled controller"; "simulation environment running the same controller source"; a round-1 defect swallowed "It is described in" into a comment | verification text; defect | APPLY |
| 256 | "simulation … used to disprove implementations" | verification text | APPLY |
| 260 | "gains frozen after each strategy is tuned" | shared gains | APPLY |
| 287–288 | scope table: two and three vehicles; "moment residual measured but not separately compensated" | protocol two vehicles; switch B | DECISION D-13; APPLY (moment row) |
| 306–311 | outline: S0–S3 incl. FBL | numbering | APPLY |

## Chapter 2

| Line | Statement | Why stale | Treatment |
|---|---|---|---|
| 533–534 | "This thesis does not fly such hybrids" | hybrid in scope | APPLY |
| 660–664 | "This thesis tunes each compared strategy" | shared gains | APPLY |
| 688–690 | "per-strategy gains frozen after tuning" | shared gains | APPLY |
| 724–727 | "Multi-robot evaluation of a reactive–predictive hybrid is outside the scope" | hybrid in scope | APPLY (first gap: DECISION D-03) |
| 735–737 | second gap: "evaluating it is outside the scope of this thesis" | hybrid in scope | APPLY |

## Chapters 6–9 (skeletons)

| File:line | Statement | Why stale | Treatment |
|---|---|---|---|
| ch6:25, 55–58 | S0–S3 with S3 = FBL; "each flown with gains frozen" | numbering; shared gains | APPLY (D-05 rest unchanged) |
| ch6:43–60 | nicknames, C.1/C.4 | names forbidden | APPLY names; DECISION D-05 |
| ch6:104, 108–110 | "Strategies 0–2"; "INDI implementation that constitutes Strategy 1" | numbering; implementation wording | APPLY |
| ch6:151–170 | prediction-off "flown under Strategy 0 with weights loaded" | S0 and S1 flights | APPLY |
| ch6:176–186 | sign check; D-01 | sign fixed | APPLY (direction of the effect; attribution via switches) |
| ch6:193–201 | removed hybrid section, label anchor | hybrid in scope | APPLY: new §Hybrid strategy (label reused) |
| ch6:225–229 | table rows S0–S3 (S3 = FBL) | numbering | APPLY (+ S4 row, DECISION D-05) |
| ch7:30–40, 60 | "third brushless vehicle"; "Strategies 0–2"; "if Strategy 2 flown" | numbering; protocol two vehicles | APPLY numbering; DECISION D-13, D-05 |
| ch8:77, 111, 159–162 | "tuning time per strategy"; "per-strategy tuning"; "absence of Strategy 3" | shared gains; numbering | APPLY |
| ch9:25–26 | "Strategies 0–2 implemented, Strategy 3 specified" | numbering | APPLY |
| ch9:72–98 | future work: C.1/C.4, cf5, `rnn.en=1`, "SIL and desk parity", NA-INDI "out of compared set" | names forbidden; verification text | APPLY names; content DECISION D-05 |

## Labels and cross-references

- `sec:ctrl-s3` now labels the hybrid; the FBL section has the new label `sec:ctrl-s4`. The only external reference to the
  FBL section (ch8) now points to `sec:ctrl-s4`.
- Kept with new content: `eq:ctrl-alpharef`, `eq:ctrl-indi-tau` (now derived from the additive law), `sec:res2-hybrid` (a
  real section again).
- New: `eq:ctrl-ad`, `eq:ctrl-s1-torque`, `eq:ctrl-s3-increment`, `tab:ctrl-params`, `tab:setup-tier1`.
- No label was removed.

# Author decisions and [VERIFY] list

Every `% DECISION` comment in `revised/ch*.tex` belongs to one of the rows below (D-01–D-11); the text at those places was left
**unchanged**. Line numbers in the original sections refer to the first delivery; the **Ready-to-paste text** subsections (added in
the closing pass) give current line numbers in `revised/` and take precedence. Search for `DECISION D-0x` to
find each place. Recommendations are mine; the decision is the author's (with the supervisor where marked).

---

## D-01 — Residual sign (the ctrl_mode 3 block of Strategy 1 is configured to *add* the residual; see D-11)

**Facts.** The derivation (Ch3 §3.3.2, eq. model-sign) and every held source that compensates a residual subtract it
(Shi 2020 eq. 9; NS2 eq. 18; Tal & Karaman eq. 20; Cobo-Briesewitz eq. 8a; Hsieh 2026 eq. 14a; Gu eq. 15). The flight
code adds `res_sign * a_res + res_sign * a_nn` (`lib.rs:1857–1861, 1930–1931`); the default is +1 (`traj_iface.c:442`);
the study configuration sets −1 for geometric+NN and +1 for our INDI (`crazyflies.yaml:223–239`, `study/ours.patch`).
Ch4 §4.3 now shows that position INDI *is* the subtracted-residual law — so with +1 Strategy 1 is not that law.

**Options.** (a) keep "subtracted" as the derived law and state that Strategy 1 is configured with the opposite sign,
with the consequence Ch3 predicts; (b) write the law with an explicit sign parameter $s\in\{-1,+1\}$, derive that $s=-1$
cancels the residual, state each strategy's configured value; (c) reconfigure Strategy 1 to −1 before the campaign and
keep the text.

**Recommendation: (c) if feasible, otherwise (b).** The thesis must not describe a sign the study does not fly.

**Places:** Ch3 L261, L415, L603 · Ch4 L115, L153, L287, L352, L405, L521 · Ch5 L322 · Ch6 L183.

### Ready-to-paste text (D-01)

Places (lines in `revised/` as of the closing pass): Ch3 L259–260, L275, L421, L616 · Ch4 eq. (ctrl-ades) L96–105,
L158–163, L296–297, L372–374, L545–553 · Ch5 L315–316 · Ch6 L179–182.

**Option (a) — keep the derived law, state the configured sign.**
- Ch3 after L260 insert: `In the study configuration the measured residual of Strategy~1 enters with the opposite sign, $+a_{\mathrm{res}}$; the consequence derived below applies to it.`
- Ch3 L421: replace `The same applies to \eqref{eq:model-ares} and the sign convention of Section~\ref{sec:model-sign}.` with `The same applies to \eqref{eq:model-ares}; the sign of Section~\ref{sec:model-sign} is the derived one, and each strategy's configured sign is stated in Section~\ref{sec:ctrl-fairness}.`
- Ch3 L616: replace `at the same point with the same sign, \eqref{eq:model-sign}.` with `at the same point, with the sign derived in \eqref{eq:model-sign} or, for Strategy~1 as configured, the opposite one.`
- Ch4 L158–159: replace `and both are subtracted, for the reason derived in Section~\ref{sec:model-sign}.` with `and Section~\ref{sec:model-sign} derives that both must be subtracted; as configured, Strategy~2 subtracts its prediction and Strategy~1 adds its measurement.`
- Ch4 L296–297: replace `enters the position loop as $-a_{\mathrm{res}}$ in \eqref{eq:ctrl-ades}.` with `enters the position loop of \eqref{eq:ctrl-ades} with the configured sign $+a_{\mathrm{res}}$, the opposite of the sign derived in Section~\ref{sec:model-sign}.`
- Ch4 after L374 insert: `With the measured residual added, as Strategy~1 is configured, the position term is not this increment.`
- Ch4 after L553 insert: `As configured, Strategy~2 subtracts its prediction and Strategy~1 adds its measured residual.`
- Ch5 L315–316: replace `Confirm $a_{\mathrm{res}}$ is non-zero and correctly signed;` with `Confirm $a_{\mathrm{res}}$ is non-zero and has the sign expected from Section~\ref{sec:model-sign};`
- Ch6 L180–182: replace `predicts that with the sign correct … approximately twice as large.` with `predicts that, with the measured residual added as Strategy~1 is configured, the formation-induced displacement in a quasi-static stack is larger than under Strategy~0.`
- **Also changes:** by Ch3's own argument Strategy 1 then doubles the disturbance, so Ch1 §1.4 ("corrects for it") and the interpretation of RQ1 must say it is not a representative reactive controller.

**Option (b) — explicit sign parameter $s$.**
- Ch4 eq. (ctrl-ades): replace `\;\underbrace{-\; a_{\mathrm{res}}}_{\text{measured}} \;\underbrace{-\; \hat{a}_{\mathrm{res}}}_{\text{predicted}},` with `\;\underbrace{+\; s\,a_{\mathrm{res}}}_{\text{measured}} \;\underbrace{+\; s\,\hat{a}_{\mathrm{res}}}_{\text{predicted}},`
- Ch4 after L131 (definitions) insert: `where $s \in \{-1, +1\}$ is a configured sign. Section~\ref{sec:model-sign} shows that $s = -1$ cancels the residual; Strategy~2 is configured with $s = -1$ and Strategy~1 with $s = +1$.`
- Ch4 L158–159: replace `and both are subtracted, for the reason derived in Section~\ref{sec:model-sign}.` with `and both enter with the configured sign $s$; Section~\ref{sec:model-sign} derives $s = -1$.`
- Ch4 L296–297: replace `as $-a_{\mathrm{res}}$` with `as $s\,a_{\mathrm{res}}$`.
- Ch3 L259–260: keep; add `The flight code multiplies both terms by a configured sign $s$; the value derived here is $s = -1$.`
- Ch3 L421, L616, Ch4 L374, L553, Ch5, Ch6: as in option (a), with "$s = +1$" in place of "added".
- **Also changes:** every later use of eq. (ctrl-ades) reads with $s$, and the same consequence for Strategy 1 as in (a).

**Option (c) — reconfigure Strategy 1 to −1 before the campaign (lab action).**
- Text unchanged except Ch4 after L553: `Both residual terms enter with the derived sign in every strategy that uses them.`
- Delete all `% DECISION D-01` comments.
- **Also changes:** nothing in the text; the frozen parameter block of the controller_step INDI must be changed and read back.

**Option (d) — via D-11 = (b): Strategy 1 is the controller that subtracts the residual (controller 9).**
- D-01 becomes moot for Strategy 1 (controller 9 subtracts, `controller_omar_indi.c` L260, L268); Strategy 2 is
  configured −1. Apply the D-11 (b) text instead; delete the D-01 comments.

## D-02 — Integral terms in the equations vs the code

**Facts.** Ch4 eq. (ctrl-ades) shows $K_i\int e_p$; eq. (ctrl-tau-geo) shows $k_{i,\mathrm{att}}\int e_R$. Code:
`ENABLE_POSITION_INTEGRAL = false` (`lib.rs:525`), `ENABLE_Z_INTEGRAL = true` (L535, L1548–1569, L1880),
`ENABLE_ATTITUDE_INTEGRAL = false` (L540). Study configuration: `ki_z = 16` for Strategies 0/2, `0` for Strategy 1.

**Options.** (a) rewrite the equations to the code (vertical integral $k_{i,z}\int e_z\,e_3$ with per-strategy gain);
(b) keep the general form and state which terms are zero; (c) leave as is.

**Recommendation: (a)**, and name the per-strategy integral in Ch4 §4.6 as part of "each tuned for itself"; it also
qualifies Ch3's "approximately twice" (an integrator removes a constant offset whatever the sign).

**Places:** Ch3 L291 · Ch4 L106, L153, L169, L248, L381, L521.

### Ready-to-paste text (D-02)

Places: Ch4 eq. (ctrl-ades) L100, L133, eq. (ctrl-tau-geo) L175–178, L186–189, L258–260, L409, L545–553 · Ch3 L294–296.

**Option (a) — rewrite to the code.**
- Ch4 eq. (ctrl-ades) L100: replace `+ K_p e_p + K_v e_v + K_i \!\int\! e_p \,\mathrm{d}t` with `+ K_p e_p + K_v e_v + k_{i,z}\, e_3 \!\int\! e_3^\top e_p \,\mathrm{d}t`
- Ch4 L133: replace `$a_{\mathrm{ff}} + K_p e_p + K_v e_v + K_i \!\int\! e_p \,\mathrm{d}t$` with `$a_{\mathrm{ff}} + K_p e_p + K_v e_v + k_{i,z} e_3 \!\int\! e_3^\top e_p \,\mathrm{d}t$`, and after that sentence insert: `The integral acts on the vertical axis only, with anti-windup and a bound on its contribution; its gain $k_{i,z}$ is part of each strategy's gain set.`
- Ch4 eq. (ctrl-tau-geo): delete the line `- k_{i,\mathrm{att}} \textstyle\int e_R \, \mathrm{d}t,` (keep the comma after `\omega \times J\omega`).
- Ch4 L186–189: delete `; $k_{i,\mathrm{att}}$ is the gain of an integral of the attitude error` and `and adds an integral term the source does not have`.
- Ch4 L258–260: delete `, and adds an integral term on attitude error that the source does not contain`.
- Ch4 after L553 insert: `The vertical integral is used by Strategies~0 and~2 and not by Strategy~1.` *(If D-11 = (b): `… and, with its own gain, by Strategy~1.`)*
- Ch3 after L296 insert: `In the study configuration the geometric strategies run such an integrator on the vertical axis, so for them the two signs differ in the transient only.`
- **Also changes:** Ch4 L409 (Tal & Karaman "no integral action") then supports Strategy 1's configuration directly; the "approximately twice" prediction of Ch3 and the Ch6 §6.5 check apply only to a strategy without the integral.

**Option (b) — keep the general form, state which terms are zero.**
- Ch4 after L131 insert: `In the flight code the horizontal components of $K_i$ and the attitude integral gain $k_{i,\mathrm{att}}$ are zero; only the vertical position integral is used, with a gain set per strategy.`
- Ch4 after L553 insert the sentence of option (a).
- **Also changes:** none in other chapters; the equations still show terms that are never active.

**Option (c) — leave.** No text; delete the D-02 comments. **Also changes:** the examiner question "does your
controller have integral action?" stays unanswered.

## D-03 — Gap statement vs Hsieh et al. (2025)  *(with the supervisor)*

**Facts.** Hsieh, Chee & Hsieh (2025) fly three Crazyflies in V- and I-stacks with no compensation, an online
$\mathcal L_1$ residual estimate, a learned downwash prediction, and both, inside one off-board MPC; the combination is
best and the estimate alone beats the prediction alone for the middle vehicle (R10). The thesis's broad gap ("families …
not evaluated against one another under identical conditions") does not survive this paper. The factual sentences about
the hybrid ("only on a single vehicle") were corrected minimally (APPLY); the gap itself was not changed.

**Options.** (a) narrow the gap to the wording below and list Hsieh et al. (2025) among the existing comparisons;
(b) keep the broad gap and describe Hsieh et al. (2025) as adjacent (MPC-internal); (c) recast contribution 1 around
onboard controllers, the frozen multi-regime library and RQ3.

**Recommendation: (a) with elements of (c).** (b) leaves a claim an examiner can refute with a paper the thesis cites.
Recommended wording:

> To the best of our knowledge, no study has compared rotor-speed-based reactive compensation (INDI) with learned
> per-neighbour feedforward prediction in onboard controllers on one airframe, across a fixed library of interaction
> regimes — static and transient, vertical and lateral, two and three vehicles — with the residual logged under every
> controller and the transfer of the summed model from two to three vehicles quantified. The closest study, Hsieh et al.
> (2025), compares an online $\mathcal L_1$ estimate, a learned downwash model and their combination inside one
> off-board model predictive controller on three vehicles in two stacked formations; it finds the combination best and
> the online estimate alone ahead of the learned model alone for the middle vehicle.

Also revise both provenance `\todo`s (the audits did not classify this paper as spanning the families).

**Places:** Ch1 L99 (comparisons ¶), L114 (italic gap), L219 (contribution 1), L262 (provenance todo) · Ch2 L618
(comparisons ¶ opening), L668 (gap ¶), L699 (first gap), L735 (provenance todo).

### Ready-to-paste text (D-03)

Places: Ch1 L102–107, L118–121, L221–224, L264–266 · Ch2 L634, L675–677, L682–685, L717–721, L748–751.

**Option (a) — narrow the gap and name Hsieh et al. (2025) (recommended).**
- Ch1 L104–106: replace `Recent work benchmarks online-learning estimators on a single vehicle~\cite{gu2025comparative} and fractional-order and classical regulators for close-proximity swarms~\cite{abro2025fractional}.` with `Recent work benchmarks online-learning estimators on a single vehicle~\cite{gu2025comparative}, fractional-order and classical regulators for close-proximity swarms~\cite{abro2025fractional}, and, inside one off-board model predictive controller on three vehicles, an online residual estimate against a learned downwash model and their combination~\cite{hsieh2025onlineadaptation}.` and replace `It is that the results for \emph{these} families cannot be composed:` with `It is that the results for these families, as they are used in onboard controllers, cannot be composed:`
- Ch1 L118–121 (italic gap): replace the `\emph{…}` block with `\emph{To the best of our knowledge, no study has compared rotor-speed-based reactive compensation with learned per-neighbour predictive compensation in onboard controllers on one airframe, across a fixed library of interaction regimes, with the residual logged under every controller and the transfer of the summed model from two to three vehicles quantified. The closest study compares an online estimate, a learned downwash model and their combination inside one off-board model predictive controller on three vehicles in two stacked formations~\cite{hsieh2025onlineadaptation}.}`
- Ch1 L221–224: replace `the first evaluation of reactive and predictive compensation on the same airframe, the same trajectories and the same formations,` with `the first evaluation of rotor-speed-based reactive compensation against learned per-neighbour predictive compensation in onboard controllers, on the same airframe, the same trajectories and the same formations, across a fixed library of interaction regimes,`
- Ch1 L264–266 (todo): replace `Both found adjacent comparisons, cited in §\ref{sec:rw-existing-comparisons}, but none spanning these families.` with `Both found adjacent comparisons, cited in §\ref{sec:rw-existing-comparisons}; one of them compares the measured, predicted and combined information sources inside an off-board predictive controller~\cite{hsieh2025onlineadaptation}, and none compares onboard INDI with learned feedforward across a scenario library.`
- Ch2 L634: replace `and two recent ones bound what remains open.` with `and three recent ones bound what remains open.` and insert directly after it this paragraph: `Hsieh et al.~\cite{hsieh2025onlineadaptation} come closest. On three Crazyflies in two stacked formations, and inside one off-board model predictive controller, they compare no compensation, an online estimate of the residual, a learned downwash prediction and both (Section~\ref{sec:rw-inject}). That is a comparison of the information sources this thesis distinguishes. It differs from the present one in where the controller runs, off-board and inside an optimiser; in the reactive method, an adaptive estimate from a velocity predictor rather than rotor-speed INDI; in the range of conditions, two formation types; and in that a summed model trained on two vehicles is used on three without the penalty being isolated.`
- Ch2 L677: replace `so the reactive, predictive and hybrid families remain uncompared against one another.` with `so they do not compare these families either.`
- Ch2 L682–685: replace the paragraph with `Together these delimit the gap precisely. It is not that close-proximity control is uncompared, nor that the information sources have never been placed side by side: inside an off-board predictive controller they have. It is that rotor-speed-based reactive compensation and learned per-neighbour feedforward, as used in onboard controllers, have not been compared across a range of interaction regimes on one airframe.`
- Ch2 L717–721 (first gap): replace from `First, and principally,` to `obtain the interaction force.` with `First, and principally, to the best of our knowledge rotor-speed-based reactive compensation and learned per-neighbour predictive compensation have not been compared in onboard controllers on one airframe across a fixed library of interaction regimes. The closest study compares an online estimate, a learned downwash model and their combination inside one off-board model predictive controller on three vehicles in two formations~\cite{hsieh2025onlineadaptation}; other systematic comparisons concern online-learning estimators on a single vehicle and classical regulators (Section~\ref{sec:rw-existing-comparisons}).`
- Ch2 L748–751 (todo): replace `Both found adjacent comparisons but none spanning these families.` with `Both found adjacent comparisons; the closest, which spans the information sources inside an off-board MPC, is discussed in Section~\ref{sec:rw-existing-comparisons}.`
- **Also changes:** Ch8 §8.5 "Contributions revisited" must test the narrowed contribution 1; the abstract (not yet written) must use the narrowed wording.

**Option (b) — keep the broad gap, call Hsieh et al. (2025) adjacent.**
- Ch2 L719–721: replace `--- of online-learning estimators on a single vehicle, and of classical regulators for close-proximity swarms (Section~\ref{sec:rw-existing-comparisons}) --- but none spans the families distinguished by how they obtain the interaction force.` with `--- of online-learning estimators on a single vehicle, of classical regulators for close-proximity swarms, and of an online estimate against a learned model inside one off-board predictive controller~\cite{hsieh2025onlineadaptation} (Section~\ref{sec:rw-existing-comparisons}) --- but none spans these families in onboard controllers.`
- Ch2 L634: insert the Hsieh paragraph of option (a), ending with `We treat it as adjacent because the comparison is internal to one model predictive controller.`
- Ch1 L104–106: add the Hsieh clause of option (a).
- **Also changes:** the italic gap, contribution 1 and both todos stay as they are, and an examiner can cite Hsieh et al. (2025) against them.

**Option (c) — recast contribution 1 (combine with (a)).**
- Ch1 L221–224: replace item 1 with `\item \textbf{A controlled comparison of onboard controllers across interaction regimes.} To the best of our knowledge, the first evaluation of rotor-speed-based reactive and learned per-neighbour predictive compensation running onboard, on one airframe, across a frozen library of static and transient, vertical and lateral scenarios, with one definition of the residual logged under every controller and with null flights that establish the measurement floor.` (keep the remaining sentences of the item).
- **Also changes:** as (a).

## D-04 — Why training data are collected only under the baseline

**Facts.** Ch4 §4.2 (Role), §4.4 and Ch5 §5.5.2 say a compensating controller records "the residual that survived
compensation". With the residual defined from *measured* rotor speeds, compensation changes the thrust but not the
observed residual (Ch3 §3.3.1; Shi 2020 §III-C; NS2 §IV-D).

**Options.** (a) replace with reasons that hold — prediction-off flights compare prediction and measurement without
the model moving the vehicle; the visited state distribution does not depend on the model being trained; one collection
controller serves every model; and note that compensated flights *could* add data; (b) keep (not recommended);
(c) adopt a curriculum as Shi et al. and Smith et al. do.

**Recommendation: (a).**

**Places:** Ch3 L231 (the property, correct) · Ch4 L269, L414 · Ch5 L332 (and the sentence "so measured $a_{\mathrm{res}}$
is not cancelled by full INDI" inside the D-05 status paragraph, Ch5 L47).

### Ready-to-paste text (D-04)

Places: Ch4 L281–284, L444–446 · Ch5 L339–341 (and L47 inside the D-05 status paragraph).

**Option (a) — reasons that hold (recommended).**
- Ch4 L281–284: replace `It is also the controller under which all training data is collected, precisely because it does not compensate: a residual observed under a compensating controller is the residual that survived compensation, which is not what a predictive model should be fitted to.` with `It is also the controller under which all training data is collected. Compensation does not change the measured residual at a given state (Section~\ref{sec:model-measure}); what it changes is the set of states the vehicle visits. Collecting under one uncompensated controller keeps that set independent of any model being trained, and the same for every model.`
- Ch4 L444–446: replace `Once the prediction is driving the controller, the measured residual is the one that survived compensation and the comparison no longer isolates model quality.` with `Once the prediction is driving the controller, it has itself moved the vehicle, and the comparison no longer isolates model quality.`
- Ch5 L339–341: replace `This is not a convenience: a compensating controller partly cancels the disturbance, so the residual it records is the one that survived compensation, which is not the quantity a predictive model should be fitted to.` with `This is not a convenience: the states a compensating controller visits depend on the model it compensates with, so data collected under it would depend on the model being trained. Under one uncompensated controller the training distribution is the same for every model, and prediction and measurement can be compared on flights in which the prediction has not moved the vehicle.`
- Ch5 L47 (status paragraph): deleted with D-05 (i); otherwise replace `so measured $a_{\mathrm{res}}$ is not cancelled by full INDI` with `so that the training distribution does not depend on a compensating model`.
- **Also changes:** none; this makes Ch4/Ch5 consistent with Ch3 §3.3.1.

**Option (b) — keep.** No text. **Also changes:** Ch3 §3.3.1 and Shi et al. (2020) §III-C contradict Ch4/Ch5.

**Option (c) — curriculum.**
- Ch5 L339–341: replace as in (a), then add `Once a first model exists, further data may be collected with it in the loop, as Shi et al.\ do~\cite{shi2020neuralswarm}, because the measured residual does not depend on the compensation.`
- **Also changes:** Ch5 §5.5.2 protocol (data collected under Strategy 2 must be labelled) and the prediction-off/on definitions in §5.5.3.

## D-05 — Lab-status passages (H9) and Strategy 3's status

**Options for each place.** (i) delete; (ii) future-tense design statement; (iii) neutral implementation status
("implemented in the flight code" / "not implemented").

| Place | Text (short) | Recommendation |
|---|---|---|
| Ch1 L152 | "Four control strategies are compared" | (iii): "Four strategies are defined; three are implemented and compared, the fourth is described as a reference design." |
| Ch1 L169 | S3 "stays in the comparison table only until the authors' code is available" | (iii): "It is described as a reference design and is not implemented in this thesis." |
| Ch4 L40 | Table 4.1 "Status" column, "at the time of writing" | (iii): column "Implemented: yes/yes/yes/no" |
| Ch4 L459 | §4.5 "requested from the authors and had not been received" | (iii): "Not implemented in the flight code. The reference method is described below." |
| Ch4 L551 | §4.7 "two strategies fly, one is …" | (iii) |
| Ch5 L33 | campaign-status paragraph (nicknames, dates, manifest, `docs/25`) | (i) delete |
| Ch5 L286 | "to be confirmed against real logs on the first two-vehicle flight" | (ii): "The measured offset and drift are reported for every two-vehicle session." |
| Ch6 L38 | §6.1 nicknames, camera count, C.1/C.4, "indicative flights exist" | (ii) neutral description to complete after the campaign |
| Ch6 L212 | Table 6.3 Strategy 3 row | keep only if Strategy 3 is implemented; else "not implemented" |
| Ch7 L30 | "third brushless vehicle in the roster" | (ii) |
| Ch8 L162 | "what the absence of Strategy 3 implies" | keep if Strategy 3 stays unimplemented |
| Ch9 L68 | future-work bullets with internal vocabulary | (ii) without internal names; move campaign tasks to Ch5 or delete |

### Ready-to-paste text (D-05)

| Place | (i) delete | (ii) future-tense design | (iii) neutral implementation status |
|---|---|---|---|
| Ch1 L154 "Four control strategies are compared." | — | `Four control strategies are defined and will be compared where implemented.` | `Four control strategies are defined; Strategies~0--2 are implemented and compared, and Strategy~3 is described as a reference design.` **(recommended)** |
| Ch1 L167–168 "It stays in the comparison table only until the authors' code is available." | delete the sentence | `It will be added to the comparison if it is implemented.` | `It is described as a reference design and is not implemented in this thesis.` **(recommended)** |
| Ch4 Table 4.1 caption L47 "and the implementation status of each at the time of writing" + column "Status" | delete the column and the phrase | — | caption `The strategies compared, and whether each is implemented in the flight code.`; column header `Implemented`; cells `yes / yes / yes / no` **(recommended)** |
| Ch4 L495–496 "The controller code was requested from the authors and had not been received at the time of writing." | delete | — | `Not implemented in the flight code. The reference method is described below.` **(recommended)** |
| Ch4 L592–594 "At the time of writing, two strategies fly …" | delete the paragraph | `Chapter~\ref{ch:setup} describes the experimental design; the two- and multi-robot results chapters report what was flown.` | `Strategies~0--2 are implemented in the flight code and Strategy~3 is not. Chapter~\ref{ch:setup} describes the experimental design.` **(recommended)** |
| Ch5 L37–49 campaign-status paragraph | **delete the paragraph (recommended)** | `\paragraph{Campaign.} The campaign is flown on two, and for the three-robot scenarios three, vehicles of the platform of Section~\ref{sec:setup-platform}, with onboard logging at \SI{500}{\hertz}; training data are collected under Strategy~0 before the comparison flights.` | — |
| Ch5 L296–297 "that figure is to be confirmed against real logs on the first two-vehicle flight." | delete the clause | `The measured offset and drift are reported for every two-vehicle session.` **(recommended)** | — |
| Ch6 L43–59 §6.1 (three paragraphs) | — | **(recommended)** `The comparison flights use the platform of Section~\ref{sec:setup-platform}, with external pose from motion capture and onboard logging at \SI{500}{\hertz} as the source of record. Metrics are computed from the merged onboard logs (Section~\ref{sec:setup-logging}). Scenarios are drawn from the frozen library of Section~\ref{sec:setup-scenarios}; each (scenario, strategy) cell is repeated $n$ times under interleaved runtime selection. The compared set is that of Chapter~\ref{ch:control}, each strategy flown with gains frozen after the hardware gate.` | — |
| Ch6 Table 6.3 "Strategy~3 (pred.\ + FBL)" row | delete the row if Strategy 3 stays unimplemented | — | keep the row with cells `not implemented` |
| Ch7 L33 "require the third brushless vehicle in the roster" | — | `Three-robot flights use three vehicles of the platform of Section~\ref{sec:setup-platform} and scenarios B1, B2 and/or B3 …` **(recommended)** | — |
| Ch8 L162 "and what the absence of Strategy~3 implies" | delete the clause if Strategy 3 is implemented | — | keep if Strategy 3 stays unimplemented |
| Ch9 L73–90 future-work list | — | **(recommended)** replace the list with `\begin{itemize} \item \textbf{Three-robot scaling and larger teams.} Formations of four or more vehicles, reusing the logging, merge and analysis tools (Section~\ref{sec:res3-n4sketch}). \item \textbf{A hybrid of measurement and prediction.} An INDI-based hybrid such as that of Cobo-Briesewitz et al.~\cite{cobobriesewitz2026lindi}, evaluated on teams. \item \textbf{Residual reinforcement-learning strategies.} Learning the corrective action directly (Section~\ref{sec:rw-rl}), with its own safety and data budget. \end{itemize}` and change L73 to `Three extensions follow from the scope of this thesis:` (the C.1/C.4 and `rnn.en=1` bullets are campaign tasks and belong in the protocol, not in future work) | — |

**Also changes:** with Ch1 (iii) and Ch4 (iii), Ch6 Table 6.3 and Ch8 L162 follow the same choice; Ch7 table columns
(Strategies 0–2) already match.

## D-06 — Filter cut-off actually realised

**Facts.** The Butterworth coefficients are computed with a design sample time (`filt_dt_us = 2000`) and optional
prewarping (`filt_prewarp = 0`) while the filters run at the 1 kHz loop (`traj_iface.c:444–478`); the realised −3 dB
frequency therefore need not equal the configured nominal cut-off.

**Options.** (a) state nominal and realised cut-off; (b) state only that a second-order Butterworth is used with a
configured cut-off; (c) reconfigure so nominal = realised before the campaign.

**Recommendation: (c), then (a).** **Places:** Ch3 L207 · Ch4 L321.

### Ready-to-paste text (D-06)

Places: Ch3 L201–204 · Ch4 L341–347 (and L398–403). Numbers in `⟨…⟩` must be filled by the author **from the filter
design** (analytically, from the coefficients and the 1 kHz sample rate), not from a flight or bench measurement (H1).

**Option (a) — state nominal and realised cut-off.**
- Ch4 after L340 insert: `The filters are second-order Butterworth filters with a configured cut-off of \SI{⟨f_c⟩}{\hertz}. Their coefficients are computed for a sample time of \SI{2}{\milli\second} without pre-warping, while the loop runs at \SI{1}{\milli\second}, so the realised \SI{-3}{dB} frequency is \SI{⟨f_real⟩}{\hertz}.`
- Ch3 L203–204: replace `Which setting the comparison campaign uses is stated in Chapter~\ref{ch:control}.` with `The realised cut-off and the setting used in the comparison are stated in Chapter~\ref{ch:control}.`
- **Also changes:** the Ch3 delay example (7.5 ms at 30 Hz) stays illustrative; quote the delay at ⟨f_real⟩ if wanted.

**Option (b) — state only that the cut-off is configured.**
- Ch4 after L340 insert: `The filters are second-order Butterworth filters whose cut-off is a configuration parameter of each strategy's gain set.`
- **Also changes:** none; an examiner may ask for the value.

**Option (c) — reconfigure (lab action) so that nominal = realised, then (a).**
- Lab: in the frozen parameter block of the controller_step INDI set the design sample time to the loop rate and
  enable pre-warping (`filt_dt_us = 1000`, `filt_prewarp = 1`; `traj_iface.c` L444–478 explain both), re-tune if the
  effective filtering changes, freeze.
- Ch4 after L340 insert: `The filters are second-order Butterworth filters designed for the \SI{1}{\kilo\hertz} loop with pre-warping, so that the realised cut-off equals the configured \SI{⟨f_c⟩}{\hertz}.`
- **Also changes:** if D-11 = (b), D-06 does not apply to Strategy 1: controller 9 designs its 30 Hz filters for
  its own 500 Hz rate (`controller_omar_indi.c` L131–136, L156).

## D-07 — H1 removals (applied by rule)

Ch5: simulation dry-run paragraph, simulation-defects sentence, radio packet numbers; Ch7: "simulation sanity checks".
The original text is kept as `% REMOVED` comments. **Recommendation:** keep removed from the body; if wanted, report the
software-verification findings later in an appendix framed as verification, not as results.

### Ready-to-paste text (D-07)

Places: Ch5 `% REMOVED (P5-05)` (lateral excitation), `% REMOVED (P5-06)` (§5.6), `% REMOVED (P5-07)` (radio) ·
Ch7 `% REMOVED (P7-03)`.

**Option 1 — keep removed (recommended).** No text; the comments may be deleted at merge.
**Option 2 — appendix after the campaign.** Add to the back matter:
`\chapter{Software verification}\label{app:verification} This appendix reports checks of the implementation, not results about any method: \todo{Scenario execution in the software-in-the-loop simulator; defects found before flight; degenerate input statistics in a collection set limited to vertical stacks; radio packet budget.}` and in Ch5 §5.6 add `Checks of the implementation made in simulation are listed in Appendix~\ref{app:verification}.`
**Also changes:** a new appendix label; H1 still applies to everything outside that appendix.
**Option 3 — restore with "development observation" framing.** Not recommended (H1); the original text is in the comments.

## D-08 — Abro et al. paragraph (no PDF held)

**Options.** Verify every clause against the full text; or reduce to "Abro et al. compare fractional-order and integral
state-feedback regulators for close-proximity flight". **Recommendation:** verify; otherwise reduce.
**Places:** Ch2 L655–656 (and Ch1 §1.2 ¶6, which cites the same paper).

### Ready-to-paste text (D-08)

Places: Ch2 L667–677 · Ch1 L105–106.

**Option 1 — verify** every clause against the full text; then delete the `% UNVERIFIED`/`% VERIFY` comments. No text change.
**Option 2 — reduce (if not verified).** Ch2 L667–677: replace the paragraph with `Abro et al.~\cite{abro2025fractional} compare fractional-order and integral state-feedback regulators for close-proximity flight, motivated by downwash and by the safety distances that neglecting it imposes. All are feedback regulators without a predictor of the interaction force, so they do not compare the families distinguished here.` (keep the footnote). Ch1 L105–106 is consistent and stays.
**Also changes:** with D-03 (a) the last clause is already replaced; use the D-03 (a) clause.

## D-09 — Hypotheses, falsifiers and statistics

**Facts.** RQs exist but no hypotheses with falsifiers; $n$, the test and the effect-size measure are open; RQ4's
"training data required" implies a data-size measurement that the protocol does not specify.

**Options.** (a) add falsifiable hypotheses (one per RQ1–RQ3) to Ch5 §5.5 as expected outcomes with falsifiers, fix
$n$ and the paired analysis, and decide whether RQ4's data question is a data-size sweep or a reported quantity;
(b) leave RQs only.

**Recommendation: (a).** **Places:** Ch5 L430 and the `\todo`s in §5.5.3, §5.5.4, §5.5.5.

### Ready-to-paste text (D-09)

Places: Ch5 L446 (D-09 comment), §5.5.3 `\todo` (n), §5.5.4 `\todo`s, §5.5.5 `\todo`.

**Option (a) — add expected outcomes with falsifiers, and fix the statistics.** Insert before `\subsection{Statistical treatment}`:
```latex
\paragraph{Expected outcomes.} Each research question is paired with an expected outcome and the observation that would
refute it. These are stated before the data and are not results.
\begin{description}
  \item[RQ1] Compensating strategies reduce the mean separation error relative to Strategy~0 in the stacked scenarios.
    Refuted if no strategy differs from Strategy~0 beyond the measurement floor (Section~\ref{sec:setup-stats}).
  \item[RQ2] The advantage of predictive over reactive compensation is larger in the transient scenarios (A8, B3) than
    in the quasi-static ones (A1, A2). Refuted if their ordering does not depend on the scenario class.
  \item[RQ3] The prediction error on three-vehicle scenarios exceeds that on the paired two-vehicle scenarios.
    Refuted if no transfer penalty is detected.
\end{description}
\todo{Confirm or replace each expected outcome before the campaign.}
```
For RQ4's data question choose one and paste: (a1) `The training-data requirement is measured by retraining the network on nested subsets of the collection set and reporting $\epsilon_{\mathrm{pred}}$ on the held-out scenarios for each subset size.` (offline analysis of data that are already collected; no extra flight), or (a2) `The training-data requirement is reported as the amount of data collected and used, without a data-size sweep.`
**Option (b) — research questions only.** No text; the `\todo`s in §5.5.3–§5.5.5 remain.
**Also changes:** with (a), the Ch6 and Ch7 "Negative answer" paragraphs become the falsifiers' report slots (no change needed); Ch9 §9.2 answer slots stay.

## D-10 — Bibliography corrections

Apply the list in `proposed_references.md` §B after checking each primary record (NS2 year/type; Cobo-Briesewitz year;
Tal & Karaman version; Lee companion note; Gu stale note; Bauersfeld year).

### Ready-to-paste text (D-10)

The corrected `.bib` entries, ready to paste over the current ones, are in **`followup_bib_corrections.md`** (§1–§11),
each checked against the held PDF. `\cite` keys are unchanged, so no chapter changes.
**Also changes:** the printed years of NS2 (2020 → 2022), Cobo-Briesewitz (2025 → 2026), Tal & Karaman (2018 → 2021)
and Bauersfeld (2024 → 2025) change in the bibliography; prose that says "Hsieh et al.\ (2025)" etc. is unaffected.

## D-11 — Which INDI controller Strategy 1 is (new, closing pass)

**Facts.** Ch4 §4.1.3 and §4.3 describe Strategy 1 as the INDI inside `firmware_app/src/lib.rs:controller_step`
(ctrl_mode 3: position INDI by the measured residual, incremental torque law eqs. ctrl-alpharef/ctrl-indi-tau). The
frozen study parameter blocks (`crazyswarm2/crazyflie/config/study_strategies.json`) define **S1** as
`stabilizer.controller: 9`, `ctrlOmarIndi.indi: 3`, `ctrlOmarIndi.Kpos_Iz: 1.5`, `deck.bcRpm: 1`, and keep the
controller_step INDI as a separate block ("ours": controller 6, ctrl_mode 3, `res_sign` +1, `ki_z` 0). Controller 9
(`crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c`, MIT licence, © 2024 K. Wahba; based on Lee et
al., CDC 2010) is a different law:
- thrust $f = m\,(a_d + a_{\mathrm{rpm},f} - a_{\mathrm{imu},f})^\top R e_3$ (L250–268), i.e. the measured residual is **subtracted**;
- both accelerations pass identical second-order Butterworth filters at 30 Hz, designed for and run at 500 Hz (L131–136, L156);
- torque = Lee's full law with the angular feedforward bracket and $e_\Omega = \Omega - R^\top R_d\Omega_d$, plus an
  attitude integral, plus the measured moment residual $\tau_{\mathrm{rpm},f} - J\dot\Omega_f$ (L368–420), i.e. residual-moment
  feedforward rather than the incremental law of eq. (ctrl-indi-tau);
- rotor speeds from the deck log variables `rpm.m1–m4` (L123), with one thrust constant;
- optional vertical integral `Kpos_Iz` (L201–234).

This structure is the "INDI" configuration of Cobo-Briesewitz et al. (their eqs. 8a–b, 10a; R07), which the author
should confirm. When controller 9 is selected, the out-of-tree controller is not called (`controller.c:105` updates
only the selected controller), so the logged `indi.a_res_*` of Ch3/Ch5 is **not computed** during Strategy 1 flights;
the residual is then available only as the filtered `ctrlOmarIndi.a_imu_f* − a_rpm_f*` (logged, `usd_study_config.txt`
L80–85).

**Options.** (a) Strategy 1 = controller_step INDI (ctrl_mode 3): text unchanged; D-01, D-02, D-06 apply as written.
(b) Strategy 1 = controller 9: rewrite Ch4 §4.3 "Implemented" and the dependent sentences; D-01 is moot for Strategy 1,
and D-02/D-06 change. (c) Decide after the remaining lab comparison; keep the text and the comments.

**Recommendation:** decide with the supervisor before the campaign (memory of the project marks the Pure-INDI choice
as open). Whichever is chosen, the thesis must describe *that* controller only (H5) and must not mention the other.

### Ready-to-paste text (D-11)

**Option (a).** No text change. Delete the `% DECISION D-11` comments and the `CLOSING PASS (D-11)` notes.

**Option (b) — Strategy 1 is controller 9.**
- Ch4 L296–311 ("Implemented." up to eq. ctrl-alpharef) and L313–347: replace with
```latex
\paragraph{Implemented.} The geometric controller of Lee et al.~\cite{lee2010geometrictracking} with the measured
residual force and moment subtracted. The measured residual of \eqref{eq:model-ares}, formed from accelerations that
both pass the same second-order Butterworth filter, enters the commanded thrust,
\begin{equation}
  f = m\left(a_{\mathrm{des}} - a_{\mathrm{res},f}\right)^{\!\top} R e_3 ,
  \label{eq:ctrl-s1-thrust}
\end{equation}
and the residual moment, formed from the moment reconstructed from rotor speeds and the filtered angular
acceleration, $\tau_{\mathrm{res},f} = J\dot\omega_f - \tau_{\mathrm{cur},f}$, is subtracted from the full torque law
of~\cite{lee2010geometrictracking} including its feedforward bracket,
\begin{equation}
  \tau = \tau_{\mathrm{Lee}} - \tau_{\mathrm{res},f} .
  \label{eq:ctrl-s1-torque}
\end{equation}
The controller runs at \SI{500}{\hertz}; rotor speeds come from the optical rotor-speed deck.
```
  The paragraph "Why the increment rejects disturbances" (L349–381) references `eq:ctrl-indi-tau` (L356) and derives
  the incremental law; under (b) keep it only as background on INDI (retitled "The incremental view") or delete it
  together with `eq:ctrl-alpharef`/`eq:ctrl-indi-tau`; no other chapter references these labels.
- Ch4 L202–205: replace `Strategy~1 has both INDI bits on.` with `Strategy~1 selects a separate controller, described in Section~\ref{sec:ctrl-s1}.`
- Ch5 L83–85: replace `The control loop runs onboard at \SI{1000}{\hertz};` with `The control loop runs onboard at \SI{1000}{\hertz} for Strategies~0 and~2 and at \SI{500}{\hertz} for Strategy~1;`
- Ch3 §3.3 and Ch5 §5.4.2: add `Under Strategy~1 the residual is recorded as the difference of the two filtered accelerations of that controller.`
- **Also changes:** Ch1 contribution 1 and Ch3/Ch5 "one definition of the residual logged under every controller" no
  longer hold literally (different filter, rotor-speed source and thrust constant under Strategy 1); either state the
  difference or log `indi.a_res` in parallel (a firmware change, outside this revision). D-01 is moot for
  Strategy 1; D-02 must name Strategy 1's own vertical integral (`Kpos_Iz`) and attitude integral; D-06 does not apply
  to Strategy 1; Ch4 §4.3 "Deviations" (Tal & Karaman) must be rewritten against Lee et al. and Cobo-Briesewitz et al.

**Option (c).** No text change now; the D-11 comments stay.

---

## [VERIFY] list (exact places)

**Closing-pass status** (evidence in `followup_verify_closure.md`): V1 OPEN · V2 OPEN · V3 CLOSED (text changed; raised
D-11) · V4 CLOSED (text changed: 10 cm diagonal) · V5 CLOSED (moot) · V6 CLOSED (text changed) · V7 PARTLY CLOSED
(architecture confirmed; weights → supervisor) · V8 CLOSED (text changed) · V9 OPEN (only needed if inertia values are
stated). Line numbers below are those of the first delivery.

| # | Place | What to verify | How |
|---|---|---|---|
| V1 | Ch2 L105; Ch4 L259 | Lee et al. CDC 2010: stability regions (90°/180°) and proposition numbering | obtain the CDC paper; companion v4 numbers them Props. 3–4 |
| V2 | Ch2 L655–656 | every clause of the Abro et al. paragraph | full text (D-08) |
| V3 | Ch4 L198 | "a single configuration file" — the study keeps per-strategy variants | decide wording against the actual study configuration |
| V4 | Ch5 L54 | "9 cm rotor-to-rotor" for the Crazyflie 2.1 Brushless; code arm 0.050 m ⇒ ≈0.10 m between opposite motors (`lib.rs:261`) | vendor data sheet (A31) |
| V5 | Ch5 L243 | radio packet numbers, if restored | vendor radio specification (A31) |
| V6 | Ch5 L283 | alignment method: text says cross-correlation against a radio trace; the merge tool may align by commanded trajectory | check the merge tool used for the campaign |
| V7 | Ch5 L453 | the simulator's interaction-model weights are the Neural-Swarm2 authors' published ones | `crazyflie_sim/backend/neuralswarm.py` model folder provenance |
| V8 | Ch5 L497 | 15 cm RMS role-check threshold of the merge procedure | merge tool |
| V9 | `proposed_references.md` A28 | Busetto et al., arXiv:2512.14450 (brushless inertia) | the paper |

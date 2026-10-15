# Round 2 — change log

Every hunk of `round2/revised/chN_*.tex` against the round-1 output `revised/chN_*.tex` (`git diff --no-index -U0`).
Columns: hunk, line range in `round2/revised/`, added/removed lines, old text (first changed line), new text (first
changed printed line), justification = the nearest round-2 marker comment in the hunk or within 15 lines above it
(it names the code line, PDF page or author decision). Chapter 4 was rewritten as a whole: its single large hunk is
followed by a section-by-section justification table.

Total hunks: 166.


## ch1_introduction (18 hunks)

| Hunk | Lines | +/− | Old (first changed line) | New (first changed printed line) | Justification |
|---|---|---|---|---|---|
| ch1-R1 | 6–6 | +1/−0 | comment only | — | see section table / comment-only |
| ch1-R2 | 124–127 | +4/−2 | `such a comparison impossible to assemble after the fact.} Th` | `such a comparison impossible to assemble after the fact.} Th` | CHANGED (round 2, author decision 2): the hybrid is in scope. The italic statement itself is D-03. |
| ch1-R3 | 147–150 | +4/−2 | `is measurable. That is where the predictive component has th` | `is measurable. That is where the predictive component has th` | CHANGED (round 2, author decision 2): "Multi-robot evaluation of such hybrids is outside this thesis" removed. |
| ch1-R4 | 159–163 | +5/−3 | `Four control strategies are compared. Chapter~\ref{ch:contro` | `Five control strategies are defined. Strategies~0--3 are one` | CHANGED (round 2, author decisions 1 and 3): five strategies, described by function. |
| ch1-R5 | 166–171 | +6/−5 | `\item[Strategy 0, geometric baseline.] A geometric tracking ` | `\item[Strategy 0, geometric baseline.] The geometric trackin` | CHANGED (round 2, author decisions 1 and 3): five strategies, described by function. |
| ch1-R6 | 175–182 | +8/−6 | `\item[Strategy 3, feedback linearisation (FBL) + NN.] A feed` | `\item[Strategy 3, INDI + NN.] The measured residual of Strat` | CHANGED (round 2, author decision 1; D-05 for this sentence resolved by that decision). |
| ch1-R7 | 184–184 | +0/−3 | `A hybrid of Strategies 1 and 2 exists in the literature. It ` | — | CHANGED (round 2, author decision 1; D-05 for this sentence resolved by that decision). |
| ch1-R8 | 203–206 | +4/−2 | `applies to the predictive strategies (Strategy~2 and, if imp` | `applies to the strategies with a learned prediction (Strateg` | DECISION D-13 (round 2): RQ3 needs three-vehicle flights, which the two-vehicle protocol of the |
| ch1-R9 | 234–239 | +6/−5 | `formations, with one definition of the residual logged under` | `formations, with one definition of the residual for every fl` | DECISION D-03: "the first evaluation of reactive and predictive compensation on the same |
| ch1-R10 | 241–241 | +1/−0 | comment only | — | CHANGED (round 2, author decision 6): the fairness argument for one controller. |
| ch1-R11 | 255–256 | +2/−6 | `them feasible: the frozen scenario library, the residual mea` | `them feasible: the frozen scenario library, the residual mea` | CHANGED (round 2, author decision 6): the fairness argument for one controller. |
| ch1-R12 | 258–260 | +3/−0 | comment only | — | CHANGED (round 2, author decision 4): no verification text (numerical check, simulation environment |
| ch1-R13 | 266–266 | +1/−1 | `on simulation, which is used to disprove implementations and` | `on simulation. And no` | CHANGED (round 2, author decision 4): no verification text (numerical check, simulation environment |
| ch1-R14 | 272–272 | +1/−1 | `The null conditions and gains frozen after each strategy is ` | `The null conditions and the shared, frozen gains serve that ` | CHANGED (round 2, author decision 4): no verification text (numerical check, simulation environment |
| ch1-R15 | 300–300 | +1/−0 | comment only | — | DECISION D-13 (round 2): the two-vehicle protocol of the final study; row left unchanged. |
| ch1-R16 | 304–305 | +2/−1 | `The force residual & The moment residual, which is measured ` | `The force residual, measured and predicted; the moment resid` | CHANGED (round 2): switch B corrects a measured moment residual (Ch4 Sec. ctrl-s1). |
| ch1-R17 | 318–319 | +2/−2 | `\textbf{Chapter~\ref{ch:model}} defines the residual this st` | `\textbf{Chapter~\ref{ch:model}} defines the residual this st` | CHANGED (round 2): switch B corrects a measured moment residual (Ch4 Sec. ctrl-s1). |
| ch1-R18 | 322–324 | +3/−4 | `\textbf{Chapter~\ref{ch:control}} describes Strategies~0--3 ` | `\textbf{Chapter~\ref{ch:control}} describes Strategies~0--4 ` | see section table / comment-only |

## ch2_related_work (7 hunks)

| Hunk | Lines | +/− | Old (first changed line) | New (first changed printed line) | Justification |
|---|---|---|---|---|---|
| ch2-R1 | 6–6 | +1/−0 | comment only | — | see section table / comment-only |
| ch2-R2 | 534–536 | +3/−2 | `estimate~\cite{hsieh2025onlineadaptation}, not for an INDI-b` | `estimate~\cite{hsieh2025onlineadaptation}, not for an INDI-b` | CHANGED (round 2, author decision 2). |
| ch2-R3 | 662–667 | +6/−5 | `near its best achievable performance. This thesis tunes each` | `near its best achievable performance. This thesis answers th` | CHANGED (round 2, author decision 6): per-strategy tuning replaced by one shared gain set. |
| ch2-R4 | 682–682 | +1/−0 | comment only | — | CHANGED (round 2): "per-strategy gains frozen after tuning" -> "gains frozen" in the requirements below. |
| ch2-R5 | 692–694 | +3/−3 | `airframe; identical trajectories and formations across all m` | `airframe; identical trajectories and formations across all m` | CHANGED (round 2): "per-strategy gains frozen after tuning" -> "gains frozen" in the requirements below. |
| ch2-R6 | 729–731 | +3/−3 | `(INDI) and predictive compensation (geometric+NN, and FBL+NN` | `(INDI), predictive compensation (geometric+NN) and their hyb` | CHANGED (round 2, author decision 2): the hybrid is in scope; the gap statement itself is D-03. |
| ch2-R7 | 740–742 | +3/−2 | `larger and is a function of a measurable relative state, rem` | `larger and is a function of a measurable relative state, rem` | CHANGED (round 2, author decision 2). |

## ch3_system_modelling (33 hunks)

| Hunk | Lines | +/− | Old (first changed line) | New (first changed printed line) | Justification |
|---|---|---|---|---|---|
| ch3-R1 | 8–10 | +3/−0 | comment only | — | VERIFY, % REMOVED. Plan rows P3-xx refer to docs/thesis_revision/stage3_revision_plan.md. |
| ch3-R2 | 15–15 | +1/−2 | comment only | — | VERIFY, % REMOVED. Plan rows P3-xx refer to docs/thesis_revision/stage3_revision_plan.md. |
| ch3-R3 | 36–42 | +7/−5 | `This chapter defines the residual acceleration $a_{\mathrm{r` | `This chapter defines the residual acceleration $a_{\mathrm{r` | CHANGED (round 2): one definition for all strategies; computed onboard or offline. |
| ch3-R4 | 68–72 | +5/−4 | `vehicles are homogeneous, of mass $m$ and diagonal inertia $` | `vehicles are homogeneous, of mass $m$ and diagonal inertia $` | CHANGED (round 2): one thrust constant (MOTORRPM2FORCE, platform_defaults_cf21bl.h:65); the sensor |
| ch3-R5 | 101–101 | +1/−1 | `\paragraph{Scope: force, not moment}` | `\paragraph{Scope: the force is predicted, the moment only me` | see section table / comment-only |
| ch3-R6 | 103–109 | +7/−4 | `This thesis compensates the \emph{force} residual $f_{\mathr` | `The learned model of this thesis predicts the \emph{force} r` | CHANGED (round 2): switch B compensates a measured moment residual (Ch4 Sec. ctrl-s1). |
| ch3-R7 | 117–119 | +3/−4 | `We therefore log $\tau_{\mathrm{res}}$ and do not cancel it ` | `We therefore do not learn $\tau_{\mathrm{res}}$. The cost is` | CHANGED (round 2): switch B compensates a measured moment residual (Ch4 Sec. ctrl-s1). |
| ch3-R8 | 143–150 | +8/−10 | `thrust, which is reconstructed from measured rotor speeds as` | `thrust, which is reconstructed from measured rotor speeds as` | CHANGED (round 2; D-06 resolved): code, controller_omar_indi.c L179-196 (rotor speeds from the |
| ch3-R9 | 156–156 | +1/−1 | `thrust coefficients overstate the real thrust by \SI{5}{\per` | `thrust constant overstates the real thrust by \SI{5}{\percen` | CHANGED (round 2; D-06 resolved): code, controller_omar_indi.c L179-196 (rotor speeds from the |
| ch3-R10 | 167–180 | +14/−10 | `The implementation evaluates \eqref{eq:model-ares} at every ` | `When the measured residual is used (switch~A, Chapter~\ref{c` | CHANGED (round 2): computed onboard with switch A, otherwise after the flight; one definition. |
| ch3-R11 | 196–201 | +6/−4 | `In the implementation, the absence of a rotor-speed source c` | `Without the rotor-speed sensor the controller does not form ` | CHANGED: code, controller_omar_indi.c (rpm_deck_available = deck.bcRpm == 1 gates every INDI term). |
| ch3-R12 | 212–216 | +5/−9 | `the difference is not corrupted by a phase mismatch~\cite{ta` | `the difference is not corrupted by a phase mismatch~\cite{ta` | CHANGED (round 2; D-06 resolved): code, controller_omar_indi.c L131-136 (cutoff 30 Hz, sample time |
| ch3-R13 | 238–239 | +2/−4 | comment only | — | CHANGED (round 2; D-04 resolved): Ch4/Ch5 now give the reason for dedicated training flights |
| ch3-R14 | 265–268 | +4/−13 | `(Section~\ref{sec:model-measure}) or predicted (Section~\ref` | `(Section~\ref{sec:model-measure}) or predicted (Section~\ref` | CHANGED (round 2; D-01 resolved): code, controller_omar_indi.c L248 (a_d - hold) and L260, L268 |
| ch3-R15 | 278–278 | +1/−1 | `the sign inverted, the displacement must be larger, approxim` | `the sign inverted, the displacement would be larger, approxi` | CHANGED (round 2; D-01 resolved): code, controller_omar_indi.c L248 (a_d - hold) and L260, L268 |
| ch3-R16 | 280–282 | +3/−1 | `need not be exactly two. The two-robot results chapter can c` | `need not be exactly two.` | CHANGED (round 2): "The two-robot results chapter can check that prediction" removed: the flown |
| ch3-R17 | 293–296 | +4/−10 | `sign, and the difference between the signs then appears only` | `sign, and the difference between the signs then appears only` | CHANGED (round 2; D-02 resolved): code, vertical integral Kpos_Iz 1.5 in S0-S3. |
| ch3-R18 | 319–326 | +8/−7 | `The neighbours' positions reach each vehicle from the motion` | `The neighbours' positions reach each vehicle over the radio,` | CHANGED (round 2): code, ns2_residual.c:ns2_eval (peer_get_all; pt == 0 -> zero; ts_new == pt -> |
| ch3-R19 | 345–345 | +1/−1 | `the gram-equivalent unit of the reference implementation; th` | `grams-force, the unit in which Neural-Swarm2 reports the lea` | CHANGED (P3-02): the equation now states what the code computes. |
| ch3-R20 | 353–354 | +2/−1 | comment only | — | CHANGED: code, ns2_residual.c:ns2_eval_rel (ground term, gate, GRAMS_TO_NEWTONS, OUT_CLAMP 8). |
| ch3-R21 | 369–369 | +1/−1 | `\paragraph{What is borrowed and what is ours.} The \emph{fam` | `\paragraph{What is borrowed and what is specific to this the` | CHANGED (P3-01): what is borrowed and what is ours, corrected to the code. |
| ch3-R22 | 373–375 | +3/−3 | `effect~\cite{shi2022neuralswarm2}. The implementation is a p` | `effect~\cite{shi2022neuralswarm2}. The weight layout keeps N` | CHANGED (P3-01): what is borrowed and what is ours, corrected to the code. |
| ch3-R23 | 382–384 | +3/−3 | `lie on the evaluated path; the weights are compiled into the` | `lie on the evaluated path; the weights are fixed in the flig` | CHANGED (P3-01): what is borrowed and what is ours, corrected to the code. |
| ch3-R24 | 399–403 | +5/−5 | `The network is not evaluated at every control step. It runs ` | `The network is not evaluated at every control step. It runs ` | CHANGED (round 2): evaluation rate of the final controller. |
| ch3-R25 | 406–414 | +9/−15 | — | — | CHANGED (round 2): file names removed from the visible text; code basis in the header comment. |
| ch3-R26 | 462–462 | +1/−1 | `those of the authors' public code, ported literally, includi` | `those of the authors' public code, including the asymmetry t` | CHANGED (P3-04): the gate of the code replaces the distance cut-off and the near-field clamp. |
| ch3-R27 | 473–473 | +1/−1 | comment only | — | CHANGED: code, ns2_residual.c GATE_DXY/GATE_DVX and ns2_eval_rel; earlier: residual_nn.rs GATE_DXY/GATE_DVX (L |
| ch3-R28 | 494–494 | +1/−1 | comment only | — | CHANGED: code, ns2_residual.c:ns2_eval_rel (isfinite, OUT_CLAMP). |
| ch3-R29 | 562–562 | +1/−1 | comment only | — | CHANGED: code, ns2_residual.c:ns2_eval (finite-difference peer velocity). |
| ch3-R30 | 565–565 | +1/−1 | comment only | — | NEW: code, ns2_residual.c GATE_DXY, GATE_DVX (input gate). |
| ch3-R31 | 576–578 | +3/−3 | `& No rotor-speed source fitted or telemetry absent` | `& Rotor-speed sensor absent, or a rotor reading missing` | CHANGED (round 2): code, controller_omar_indi.c (optical deck, rpm_deck_available). |
| ch3-R32 | 596–599 | +4/−3 | `The residual of \eqref{eq:model-trans}--\eqref{eq:model-rot}` | `The residual of \eqref{eq:model-trans}--\eqref{eq:model-rot}` | CHANGED (round 2). |
| ch3-R33 | 606–610 | +5/−11 | — | — | CHANGED (round 2; D-01 resolved): both are subtracted by construction. |

## ch4_control_architectures (51 hunks)

| Hunk | Lines | +/− | Old (first changed line) | New (first changed printed line) | Justification |
|---|---|---|---|---|---|
| ch4-R1 | 5–10 | +6/−3 | comment only | — | NEW, % CHANGED, % DECISION, % VERIFY, % CONFIRM-AFTER-LAB. Change log: |
| ch4-R2 | 12–17 | +6/−7 | comment only | — | NEW, % CHANGED, % DECISION, % VERIFY, % CONFIRM-AFTER-LAB. Change log: |
| ch4-R3 | 19–22 | +4/−2 | comment only | — | NEW, % CHANGED, % DECISION, % VERIFY, % CONFIRM-AFTER-LAB. Change log: |
| ch4-R4 | 24–25 | +2/−7 | comment only | — | NEW, % CHANGED, % DECISION, % VERIFY, % CONFIRM-AFTER-LAB. Change log: |
| ch4-R5 | 31–47 | +17/−13 | `Chapter~\ref{ch:model} defined the residual wrench and the t` | `Chapter~\ref{ch:model} defined the residual wrench and the t` | CHANGED (round 2): the strategy list of the final study. |
| ch4-R6 | 50–54 | +5/−4 | `\caption{The strategies compared, and the implementation sta` | `\caption{The strategies. Switches A and B add the measured r` | CHANGED (round 2): the strategy list of the final study. |
| ch4-R7 | 57–57 | +1/−1 | `\begin{tabular}{@{}clllcc@{}}` | `\begin{tabular}{@{}cllccc@{}}` | DECISION D-05 (H9): the last column states implementation in the flight code, not flight status; |
| ch4-R8 | 59–59 | +1/−1 | `\# & Strategy & Knows & Enters at & Rotor speed & Status \\` | `\# & Strategy & Knows & Switches & Rotor speed & Implemented` | DECISION D-05 (H9): the last column states implementation in the flight code, not flight status; |
| ch4-R9 | 61–65 | +5/−5 | `0 & Geometric baseline           & nothing    & ---         ` | `0 & Geometric baseline           & nothing             & ---` | DECISION D-05 (H9): the last column states implementation in the flight code, not flight status; |
| ch4-R10 | 74–74 | +1/−0 | — | `The evaluation of Chapter~\ref{ch:setup} records rotor speed` | see section table / comment-only |
| ch4-R11 | 78–83 | +6/−4 | `The chapter treats each strategy as its own controller. What` | `Because Strategies~0--3 are one controller, they share the p` | CHANGED (round 2): the fairness argument reverses (author decision 6). |
| ch4-R12 | 85–86 | +2/−6 | `Each strategy is described in two parts, kept deliberately s` | `Each strategy is described in two parts, kept separate: the ` | CHANGED (round 2): the fairness argument reverses (author decision 6). |
| ch4-R13 | 89–89 | +1/−1 | `\section{The position and attitude loop}` | `\section{The common controller}` | CHANGED (round 2): the fairness argument reverses (author decision 6). |
| ch4-R14 | 92–94 | +3/−4 | `Strategies~0, 1 and~2 (the geometric baseline, pure INDI and` | `Strategies~0--3 share one position loop and one attitude loo` | CHANGED (round 2): one controller for Strategies 0-3. |
| ch4-R15 | 98–108 | +11/−1 | `The commanded acceleration is` | `The desired acceleration is` | CHANGED (round 2): one controller for Strategies 0-3. |
| ch4-R16 | 112–114 | +3/−5 | `\;=\; \underbrace{a_{\mathrm{ff}}}_{\text{trajectory}}` | `\;=\; a_d` | see section table / comment-only |
| ch4-R17 | 116–116 | +0/−23 | comment only | — | see section table / comment-only |
| ch4-R18 | 118–135 | +18/−12 | `with $e_p = p_{\mathrm{des}} - p$ and $e_v = v_{\mathrm{des}` | `where $a_{\mathrm{res},f}$ is the measured residual \eqref{e` | CHANGED (round 2; D-01 and D-02 resolved): code, controller_omar_indi.c L214-249 (acc_d with |
| ch4-R19 | 137–139 | +3/−3 | `z_{\mathrm{des}} = \frac{a_{\mathrm{cmd}}}{\lVert a_{\mathrm` | `x_{\mathrm{des}} = \frac{y_c \times a_{\mathrm{cmd}}}{\lVert` | CHANGED (round 2; D-01 and D-02 resolved): code, controller_omar_indi.c L214-249 (acc_d with |
| ch4-R20 | 141–145 | +5/−19 | `the construction of Lee et al.\ for the position-and-heading` | `so that the body $z$-axis points along $a_{\mathrm{cmd}}$ an` | CHANGED: code, controller_omar_indi.c L276-282 (xb = normalize(yc x F_d), yb = normalize(F_d x xb), |
| ch4-R21 | 149–156 | +8/−4 | `The attitude error is the standard geometric one,` | `The attitude error and the rate error are those of Lee et al` | CHANGED (round 2): the full geometric law of Lee et al. with an attitude integral; the switch-B |
| ch4-R22 | 159–160 | +2/−2 | `\tau = -K_R^{\mathrm{geo}} e_R - K_\omega^{\mathrm{geo}} e_\` | `\tau_{\mathrm{geo}} = -K_R e_R - K_\omega e_\omega - K_I \!\` | CHANGED (round 2): the full geometric law of Lee et al. with an attitude integral; the switch-B |
| ch4-R23 | 162–162 | +0/−2 | comment only | — | CHANGED (round 2): the full geometric law of Lee et al. with an attitude integral; the switch-B |
| ch4-R24 | 164–222 | +59/−11 | `the third term compensating the gyroscopic coupling. Here $K` | `and switch~B adds to it the measured-moment correction of Se` | CHANGED (round 2): rates of the final controller. |
| ch4-R25 | 227–260 | +34/−25 | `The control law is selected by a parameter at runtime rather` | `The switches are parameters set at run time, not compile-tim` | CONFIRM-AFTER-LAB (a): take-off and landing of the INDI strategies. A take-off with switches A and B |
| ch4-R26 | 266–267 | +2/−8 | `\paragraph{Implemented.} Equations~\eqref{eq:ctrl-ades} and ` | `\paragraph{The law.} Equations~\eqref{eq:ctrl-ad}--\eqref{eq` | see section table / comment-only |
| ch4-R27 | 272–272 | +1/−3 | `$e_R = \tfrac{1}{2}(R_d^\top R - R^\top R_d)^\vee$, verified` | `$e_R = \tfrac{1}{2}(R_d^\top R - R^\top R_d)^\vee$.` | see section table / comment-only |
| ch4-R28 | 278–278 | +1/−3 | — | `The torque law of the source is` | see section table / comment-only |
| ch4-R29 | 286–302 | +17/−7 | `with $e_\Omega = \Omega - R^\top R_d \Omega_d$. Equation~\eq` | `with $e_\Omega = \Omega - R^\top R_d \Omega_d$; the symbols ` | CHANGED (round 2): deviations of the flown law from the source. |
| ch4-R30 | 307–308 | +2/−3 | `below \SI{180}{\degree}. The distinction matters here only i` | `below \SI{180}{\degree}. This thesis claims neither for the ` | VERIFY: the inertia source (Foerster, BA thesis, ETH Zurich) is not held; see |
| ch4-R31 | 312–312 | +1/−1 | comment only | — | VERIFY: the regions are attributed to the CDC paper (no PDF held); they are checked against the |
| ch4-R32 | 315–315 | +1/−0 | comment only | — | CHANGED (round 2; D-04 resolved): the role, and why training data come from dedicated flights. |
| ch4-R33 | 319–321 | +3/−8 | `Section~\ref{sec:model-measure-fail}. It is also the control` | `Section~\ref{sec:model-measure-fail}. It is also the control` | CHANGED (round 2; D-04 resolved): the role, and why training data come from dedicated flights. |
| ch4-R34 | 324–324 | +1/−1 | `\section{Strategy 1: pure INDI}` | `\section{Strategy 1: INDI}` | CHANGED (round 2; D-04 resolved): the role, and why training data come from dedicated flights. |
| ch4-R35 | 325–325 | +0/−2 | comment only | — | CHANGED (round 2; D-04 resolved): the role, and why training data come from dedicated flights. |
| ch4-R36 | 327–338 | +12/−4 | `\paragraph{Implemented.} The measured residual of \eqref{eq:` | `\paragraph{The law.} Switches A and B. Switch~A subtracts th` | CHANGED (round 2; D-11 and D-01 resolved): the measured residual force and moment are added to the |
| ch4-R37 | 341–342 | +2/−2 | `\alpha_{\mathrm{ref}} = \alpha_{\mathrm{des}} - K_R^{\mathrm` | `\tau = \tau_{\mathrm{geo}} + \big(\tau_{\mathrm{cur},f} - J ` | CHANGED (round 2; D-11 and D-01 resolved): the measured residual force and moment are added to the |
| ch4-R38 | 343–343 | +0/−15 | `where $\alpha_{\mathrm{des}}$ is the feedforward angular acc` | — | see section table / comment-only |
| ch4-R39 | 345–365 | +21/−0 | — | `where $\tau_{\mathrm{cur}}$ is the torque the rotors are pro` | CHANGED: code, controller_omar_indi.c L179-196 (t_k = MOTORRPM2FORCE * (rpm_k * 2 pi / 60)^2 from the |
| ch4-R40 | 367–372 | +6/−1 | `\tau = \tau_{\mathrm{cur}} + J\left(\alpha_{\mathrm{ref}} - ` | `J\alpha_{\mathrm{ref}} = \tau_{\mathrm{geo}},` | see section table / comment-only |
| ch4-R41 | 375–405 | +31/−96 | `In words: the controller compares the angular acceleration i` | `the angular-acceleration increment of Tal and Karaman~\cite{` | CHANGED: code, controller_omar_indi.c L408 (indi_moments = tau_rpm_f - tau_gyro_f; no omega x J omega |
| ch4-R42 | 411–450 | +40/−28 | `\paragraph{Implemented.} The geometric controller of Section` | `\paragraph{The law.} Switch~C. The deep-sets prediction $\ha` | CONFIRM-AFTER-LAB (b): the number of training flights and every figure about the retrained model |
| ch4-R43 | 452–465 | +14/−18 | `line~\cite{shi2020neuralswarm,shi2022neuralswarm2}.` | `line~\cite{shi2020neuralswarm,shi2022neuralswarm2}. The clos` | CONFIRM-AFTER-LAB (b): the number of training flights and every figure about the retrained model |
| ch4-R44 | 467–472 | +6/−13 | `component, its output is bounded at \SI{8}{\metre\per\second` | `component, its output is bounded at \SI{8}{\metre\per\second` | see section table / comment-only |
| ch4-R45 | 475–475 | +1/−1 | `\section{Strategy 3: feedback linearisation with a flatness-` | `\section{Strategy 3: INDI with a learned residual}` | see section table / comment-only |
| ch4-R46 | 478–537 | +60/−7 | `\paragraph{Implementation status.} \textbf{Not implemented.}` | `\paragraph{The law.} Switches A, B and C together: the measu` | CHANGED (round 2): renumbered from Strategy 3 to Strategy 4. |
| ch4-R47 | 546–546 | +0/−1 | comment only | — | CHANGED (round 2): renumbered from Strategy 3 to Strategy 4. |
| ch4-R48 | 551–551 | +0/−1 | comment only | — | see section table / comment-only |
| ch4-R49 | 564–564 | +0/−4 | comment only | — | see section table / comment-only |
| ch4-R50 | 573–595 | +23/−37 | `\paragraph{What is held constant.} The airframe, the traject` | `\paragraph{What is shared.} The airframe, the trajectories, ` | CHANGED (round 2): the fairness argument rewritten for one controller (author decision 6). |
| ch4-R51 | 601–608 | +8/−13 | `Three strategies (1--3: pure INDI, geometric+NN and FBL+NN),` | `Strategies~0--3 are one geometric controller with three swit` | CHANGED (round 2). |

## ch5_experimental_setup (27 hunks)

| Hunk | Lines | +/− | Old (first changed line) | New (first changed printed line) | Justification |
|---|---|---|---|---|---|
| ch5-R1 | 6–7 | +2/−0 | comment only | — | see section table / comment-only |
| ch5-R2 | 28–30 | +3/−1 | `Chapter~\ref{ch:control} described Strategies~0--3, each as ` | `Chapter~\ref{ch:control} described the strategies: one contr` | CHANGED (round 2). |
| ch5-R3 | 37–38 | +2/−1 | comment only | — | DECISION D-05 (H9, lab status; still open in round 2): the following paragraph reports campaign |
| ch5-R4 | 42–44 | +3/−0 | comment only | — | CHANGED (round 2, author decision 3): vehicle nicknames, file names and internal labels removed from |
| ch5-R5 | 47–53 | +7/−10 | `the brushless pair \textbf{cf5} (study vehicle) and \textbf{` | `a pair of brushless vehicles. Micro-SD merges with per-vehic` | CHANGED (round 2, author decision 3): vehicle nicknames, file names and internal labels removed from |
| ch5-R6 | 75–83 | +9/−9 | `\paragraph{A rotor-speed source}, normally the rotor speed t` | `\paragraph{An optical rotor-speed sensor} (a rotor-speed dec` | CHANGED (round 2): the optical rotor-speed sensor is the source the controller uses. |
| ch5-R7 | 88–88 | +1/−1 | comment only | — | CHANGED (round 2): rates of the final controller. |
| ch5-R8 | 90–93 | +4/−3 | `motion-capture (mocap) system at \SI{100}{\hertz}. The contr` | `motion-capture (mocap) system at \SI{100}{\hertz}. The contr` | CHANGED (round 2): rates of the final controller. |
| ch5-R9 | 95–96 | +2/−2 | comment only | — | CHANGED (round 2): rates of the final controller. |
| ch5-R10 | 133–133 | +1/−2 | `the standard trajectory interface, so that the same command ` | `the standard trajectory interface before take-off. The comma` | CHANGED (P5-13): the table is referenced, and parameters are given. |
| ch5-R11 | 181–182 | +2/−0 | comment only | — | DECISION D-13 (round 2): C1-C3 are three-vehicle scenarios, outside the two-vehicle protocol of the |
| ch5-R12 | 264–264 | +1/−1 | comment only | — | CHANGED (round 2): the list for the final controller. |
| ch5-R13 | 266–275 | +10/−6 | `accelerometer; the measured residual $a_{\mathrm{res}}$; the` | `accelerometer; the controller's two filtered accelerations, ` | CHANGED (round 2): the list for the final controller. |
| ch5-R14 | 280–286 | +7/−4 | `Two of these deserve comment. The measured residual is recor` | `Two of these deserve comment. The controller computes the me` | CHANGED (round 2; logging statement made true for the final controller). |
| ch5-R15 | 324–336 | +13/−7 | `\item \textbf{Validate.} Single vehicle, geometric first and` | `\item \textbf{Validate.} Single vehicle, the baseline first ` | CONFIRM-AFTER-LAB (c): execution time of the learned model; no figure before the measurement. |
| ch5-R16 | 345–347 | +3/−9 | `at which the sign of the compensation is observable at all, ` | `at which the interaction residual is observable at all, whic` | CHANGED (round 2; D-01 resolved): no sign to confirm, the controller subtracts by construction. |
| ch5-R17 | 352–360 | +9/−6 | `Collection flights are flown under the \emph{uncompensated} ` | `The learned model is trained on \emph{dedicated} training fl` | CONFIRM-AFTER-LAB (b): the number of training flights and their sizes are stated after the training |
| ch5-R18 | 373–386 | +14/−6 | `Each strategy flies the frozen library. Because the control ` | `Each strategy flies the same scenario settings. A \emph{cell` | CHANGED (round 2): the campaign design of the protocol (draft v0.2 with the 2026-10-14 banner; |
| ch5-R19 | 388–414 | +27/−3 | `Each strategy is compared under identical trajectories, form` | `\begin{table}[t]` | DECISION D-13 (round 2, new): the protocol flies two vehicles only and excludes the three-vehicle |
| ch5-R20 | 416–420 | +5/−6 | `For Strategy~2 two kinds of flight are distinguished. On \em` | `For the learned model two kinds of flight are distinguished.` | CHANGED (round 2): every flight with switch C off records the prediction. |
| ch5-R21 | 453–458 | +6/−6 | `The logging configuration of Section~\ref{sec:setup-recorded` | `The controller records its actual and its desired attitude (` | CHANGED (round 2): e_R is computed from the recorded actual and desired attitude. |
| ch5-R22 | 521–524 | +4/−3 | `A software-in-the-loop simulator runs the same controller so` | `A software-in-the-loop simulator with the published Neural-S` | CHANGED (round 2, author decision 4): no verification text. |
| ch5-R23 | 531–532 | +2/−5 | `\textbf{Simulation is used to disprove implementations, neve` | `\textbf{Simulation is never used to validate a method.} It c` | CHANGED (round 2, author decision 4): no verification text. |
| ch5-R24 | 556–556 | +1/−1 | `Tuning confound & Each strategy tuned on the hardware gate; ` | `Tuning confound & One controller with one shared gain set fo` | CHANGED (P5-17): internal vocabulary in the "radio vs onboard clock" row. |
| ch5-R25 | 559–561 | +3/−3 | `Training/test leakage & Held-out scenarios; split by contigu` | `Training/test leakage & Dedicated training flights, never pa` | CHANGED (P5-17): internal vocabulary in the "radio vs onboard clock" row. |
| ch5-R26 | 592–601 | +10/−8 | `Two or three identical brushless quadrotors, each carrying a` | `Identical brushless quadrotors, each carrying an optical rot` | DECISION D-13: "null scenarios" (C1-C3) are three-vehicle scenarios outside the two-vehicle protocol. |
| ch5-R27 | 605–605 | +1/−1 | `campaign's repetitions are completed. Record any deviation f` | `campaign's repetitions are completed. Record any deviation f` | DECISION D-13: "null scenarios" (C1-C3) are three-vehicle scenarios outside the two-vehicle protocol. |

## ch6_results_2robot (15 hunks)

| Hunk | Lines | +/− | Old (first changed line) | New (first changed printed line) | Justification |
|---|---|---|---|---|---|
| ch6-R1 | 6–6 | +1/−0 | comment only | — | see section table / comment-only |
| ch6-R2 | 26–26 | +1/−1 | `defined Strategies~0--3 compared against the baseline. The s` | `defined Strategies~0--4, of which Strategies~0--3 are compar` | CHANGED (P6-02): internal campaign label removed. |
| ch6-R3 | 44–46 | +3/−2 | `The comparison flights use the brushless pair \textbf{cf5} (` | `The comparison flights use two brushless vehicles, the study` | CHANGED (round 2, author decision 3): nicknames and internal labels removed; D-05 content unchanged. |
| ch6-R4 | 52–52 | +1/−1 | `the C.1/C.4 matrix prioritises vertical separation and downw` | `the training and comparison matrix prioritises vertical sepa` | CHANGED (round 2, author decision 3): nicknames and internal labels removed; D-05 content unchanged. |
| ch6-R5 | 58–62 | +5/−4 | `Strategy~1 (pure INDI), Strategy~2 (geometric position loop ` | `Strategy~1 (INDI), Strategy~2 (geometric + learned residual)` | CHANGED (round 2, author decision 1): strategy list; the rest of this D-05 paragraph is unchanged. |
| ch6-R6 | 107–107 | +1/−1 | `representative transient scenario (A8 or A4), Strategies~0--` | `representative transient scenario (A8 or A4), Strategies~0--` | see section table / comment-only |
| ch6-R7 | 111–113 | +3/−3 | `A short subsection will contrast geometric tracking (Strateg` | `A short subsection will contrast Strategy~0 with Strategy~1 ` | CHANGED (round 2, author decision 3): no implementation wording. |
| ch6-R8 | 157–162 | +6/−6 | `Strategy~2 is evaluated in two layers. On \emph{prediction-o` | `The learned model is evaluated in two layers. On \emph{predi` | CHANGED (round 2): prediction-off flights are those of Strategies 0 and 1 (Ch5 Sec. setup-campaign). |
| ch6-R9 | 179–179 | +1/−1 | `\section{Sign and loop attribution}` | `\section{Loop attribution}` | NEW (P6-04): the checks promised in Chapter 3 (Sec. model-sign) and Chapter 4 (Sec. ctrl-s1). |
| ch6-R10 | 182–183 | +2/−0 | comment only | — | CHANGED (round 2; D-01 resolved): the controller subtracts the residual by construction, so the |
| ch6-R11 | 185–190 | +6/−8 | `and~\ref{ch:control}. First, the sign of the compensation: C` | `and~\ref{ch:control}. First, the direction of the effect: Ch` | CHANGED (round 2; D-01 resolved): the controller subtracts the residual by construction, so the |
| ch6-R12 | 196–198 | +3/−8 | `% REMOVED (P6-03): the former section "Hybrid (NA-INDI), if ` | `\section{Hybrid strategy (Strategy~3)}` | NEW (round 2, author decision 2): the hybrid is in the comparison; the label of the former slot is |
| ch6-R13 | 201–215 | +15/−0 | — | `Strategy~3 is compared with both of its parents on matched s` | NEW (round 2, author decision 2): the hybrid is in the comparison; the label of the former slot is |
| ch6-R14 | 228–229 | +2/−2 | comment only | — | DECISION D-05 (H5): the Strategy 4 row (renumbered from 3 in round 2) describes an unimplemented |
| ch6-R15 | 242–243 | +2/−1 | `Strategy~3 (pred.\ + FBL) & --- & --- & --- & --- \\` | `Strategy~3 (INDI + pred.) & --- & --- & --- & --- \\` | DECISION D-05 (H5): the Strategy 4 row (renumbered from 3 in round 2) describes an unimplemented |

## ch7_results_multirobot (4 hunks)

| Hunk | Lines | +/− | Old (first changed line) | New (first changed printed line) | Justification |
|---|---|---|---|---|---|
| ch7-R1 | 6–6 | +1/−0 | comment only | — | see section table / comment-only |
| ch7-R2 | 30–31 | +2/−0 | comment only | — | DECISION D-13 (round 2): the protocol of the final study flies two vehicles only; whether this |
| ch7-R3 | 43–43 | +1/−1 | `\todo[color=red!25]{PLACEHOLDER: Table --- B1/B2/B3 (subset ` | `\todo[color=red!25]{PLACEHOLDER: Table --- B1/B2/B3 (subset ` | DECISION D-13 (round 2): the protocol of the final study flies two vehicles only; whether this |
| ch7-R4 | 63–63 | +1/−1 | `\todo[color=red!25]{PLACEHOLDER: Closed-loop block --- if St` | `\todo[color=red!25]{PLACEHOLDER: Closed-loop block --- if St` | CHANGED (P7-02): the penalty as defined in Chapter 5; negative answer; internal labels removed. |

## ch8_discussion (4 hunks)

| Hunk | Lines | +/− | Old (first changed line) | New (first changed printed line) | Justification |
|---|---|---|---|---|---|
| ch8-R1 | 6–6 | +1/−0 | comment only | — | see section table / comment-only |
| ch8-R2 | 78–78 | +1/−1 | `onboard logging, how the network weights are deployed, tunin` | `onboard logging, how the network weights are deployed, the t` | see section table / comment-only |
| ch8-R3 | 112–116 | +5/−3 | `\paragraph{Tuning and fairness.} Per-strategy tuning followe` | `\paragraph{Tuning and fairness.} One shared gain set, frozen` | CHANGED (round 2, author decision 6). |
| ch8-R4 | 162–169 | +8/−5 | `Each strategy was tuned for itself and then frozen (Section~` | `Strategies~0--3 share one controller and one frozen gain set` | CHANGED (round 2, author decision 6; Strategy 4 renumbered). |

## ch9_conclusion (5 hunks)

| Hunk | Lines | +/− | Old (first changed line) | New (first changed printed line) | Justification |
|---|---|---|---|---|---|
| ch9-R1 | 6–6 | +1/−0 | comment only | — | see section table / comment-only |
| ch9-R2 | 26–30 | +5/−2 | `predictive families under the same residual definition and n` | `predictive families under the same residual definition and n` | CHANGED (round 2, author decisions 1 and 2). |
| ch9-R3 | 77–82 | +6/−1 | `Several extensions are already scoped in project documentati` | `Several extensions are already scoped in project documentati` | CHANGED (round 2, author decision 3): internal labels, nicknames and identifiers removed from this list; |
| ch9-R4 | 85–86 | +2/−2 | `\item \textbf{C.1 coverage and C.4 repeats.} Complete the pl` | `\item \textbf{Training coverage and comparison repeats.} Com` | CHANGED (round 2, author decision 3): internal labels, nicknames and identifiers removed from this list; |
| ch9-R5 | 97–98 | +2/−2 | `\item \textbf{Hardware validation of onboard prediction.} SI` | `\item \textbf{Hardware validation of onboard prediction.} Si` | see section table / comment-only |

## thesis_draft (2 hunks)

| Hunk | Lines | +/− | Old (first changed line) | New (first changed printed line) | Justification |
|---|---|---|---|---|---|
| the-R1 | 2–2 | +1/−1 | comment only | — | see section table / comment-only |
| the-R2 | 16–16 | +1/−1 | `\bibliography{../../references}` | `\bibliography{../../../references}` | see section table / comment-only |

## Justification of hunks without a nearby round-2 marker

| Hunks | Justification |
|---|---|
| ch1-R1, ch2-R1, ch5-R1, ch6-R1, ch7-R1, ch8-R1, ch9-R1, the-R1 | header comments (revision note) |
| ch1-R18 | outline: Strategies 0–4 named by function (author decisions 1 and 3) |
| ch3-R5 | heading: switch B corrects a measured moment residual (code `controller_omar_indi.c` L365–410); marker below the heading |
| ch6-R6 | Strategies 0–3 in the figure slot (author decision 2) |
| ch8-R2 | shared gain set (author decision 6) |
| ch9-R5 | "SIL and desk parity" → "Simulation" (author decision 4: no verification text; D-05 content otherwise unchanged) |
| the-R2 | bibliography path one level deeper for `round2/revised/` |
| ch4-R10 … ch4-R49 | Chapter 4 rewrite; see the table below |

## Chapter 4 — section-by-section justification

| Section (round 2) | Change | Justification |
|---|---|---|
| Intro, Table 4.1 | five strategies, switches A/B/C, "Implemented" column, footnote on evaluation rotor speeds | author decisions 1–3; code `study_strategies.json` S0–S3 (indi 0/3 × rnn.en 0/1); D-05 column kept neutral (open) |
| Intro ¶ fairness | "not from a claimed shared implementation" → one controller, shared parameters | author decision 6 |
| §common: eq. (ctrl-ad) new, (ctrl-ades) rewritten | $a_d$ with vertical-only integral and gravity; $a_{cmd} = a_d - a_{res,f} - \hat a_{res}$ | code `controller_omar_indi.c` L214–268; D-01, D-02 resolved |
| §common: velocity-error conditioning removed | 0.6 m/s limit and 10 Hz filter belonged to the retired implementation | fact sheet; code (no such conditioning in this controller) |
| §common: $R_{des}$ construction | $x = y_c \times a_{cmd}$ normalised (heading via $y_c$) | code L276–282 |
| §common: eq. (ctrl-tau-geo) | full Lee law incl. feedforward bracket and transported rate error, plus attitude integral $K_I$ | code L318–364; D-02 resolved |
| §common: Table 4.2 (new) | shared parameters | code L56–73, `app-config-bl:48–49` (mass), `platform_defaults_cf21bl.h:45, 65–67`, study yaml (Kpos_Iz 1.5) |
| §common: rates | 500 Hz controller in 1 kHz loop, 100 Hz mocap and NN | code L156, `stabilizer_types.h:383`, `ns2_residual.c` |
| §runtime | three switches, frozen parameter sets, upper vehicle always S0, diagnostic A/B alone, take-off/landing CONFIRM-AFTER-LAB (a) | code (indi bits, rnn.en), `tree_S*.patch`; author decision 5 |
| §S0 | "reduced law" text removed (the flown law has the bracket); deviations: integrals, $R_d$ construction, inertia of the Crazyflie 2.0 | code L59–62 (inertia comment), L318–364 |
| §S0 Role | D-04 resolved: dedicated training flights | author decision 7 |
| §S1 | measured force and moment terms; eq. (ctrl-s1-torque) new; increment form derived, keeping labels (ctrl-alpharef), (ctrl-indi-tau); reference Cobo-Briesewitz eqs. (8a), (8b), (10a), (10b); deviations (no gyroscopic term, integrals, optical sensor, one $k_t$) | code L179–196, L250–262, L365–410; Cobo-Briesewitz PDF §4.1–4.2 (eqs. 8–10); Tal & Karaman PDF §III-B/D |
| §S1 | incremental law with INDI gains, effectiveness factor, torque limit, DShot and plausibility filter removed | retired implementation (fact sheet) |
| §S2 | rates, inputs, training on dedicated flights, CONFIRM-AFTER-LAB (b), (c); deviations kept | code `ns2_residual.c`; author decisions 5, 7 |
| §S3 (new) | the hybrid, derivation eq. (ctrl-s3-increment), DECISION D-12 | author decision 2; derivation from eqs. (model-ares), (ctrl-ades); Cobo-Briesewitz PDF §4.4 |
| §S4 | renumbered from S3; status neutral; new label `sec:ctrl-s4` | author decision 1 |
| §fairness | rewritten for one controller; four limitations | author decision 6 |
| §summary | rewritten | author decisions 1, 2 |

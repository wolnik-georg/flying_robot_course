# Round 3 — change log

Every hunk of `round3/revised/` against the round-2 output `round2/revised/` (`git diff --no-index -U0`). Columns: hunk,
line range in `round3/revised/`, added/removed lines, old (first changed printed line), new (first changed printed
line), decision (the nearest round-3 marker in the hunk or within 15 lines above it).

Total hunks: 54.


## ch1_introduction (9 hunks)

| Hunk | Lines | +/− | Old | New | Decision |
|---|---|---|---|---|---|
| ch1-T1 | 6–6 | +1/−0 | — | — | header comment |
| ch1-T2 | 178–179 | +2/−1 | `Strategy~2 together. This is the hybrid strategy.` | `Strategy~2, added together. This is the hybrid strategy.` | CHANGED (round 3, D-12 = (a)): "added". |
| ch1-T3 | 207–208 | +2/−2 | — | `RQ3 is answered in the second, three-vehicle phase of the st` | CHANGED (round 3, D-13 = (b)): RQ3 kept; flown in phase 2. |
| ch1-T4 | 220–224 | +5/−2 | `Each question is answerable by the experiments described in ` | `Each question is answerable by the experiments described in ` | CHANGED (round 3, D-13 = (b)). |
| ch1-T5 | 240–241 | +2/−1 | `strategies that do not use it, and with null flights that es` | `strategies that do not use it, and with null flights, in the` | round-3 phase wording (see marker above) |
| ch1-T6 | 250–250 | +1/−1 | `model trained on two vehicles meets three. This measures a p` | `model trained on two vehicles meets three, in the second pha` | round-3 phase wording (see marker above) |
| ch1-T7 | 306–306 | +1/−1 | — | — | CHANGED (round 3, D-13 = (b)): row kept; three vehicles in phase 2. |
| ch1-T8 | 333–333 | +1/−1 | `measurement chain and the protocol, and states the threats t` | `measurement chain and the two-phase protocol, and states the` | round-3 phase wording (see marker above) |
| ch1-T9 | 336–339 | +4/−2 | `The \textbf{two-robot results chapter} reports the two-vehic` | `The \textbf{two-robot results chapter} reports the results o` | CHANGED (round 3, D-13 = (b)). |

## ch2_related_work (3 hunks)

| Hunk | Lines | +/− | Old | New | Decision |
|---|---|---|---|---|---|
| ch2-T1 | 6–6 | +1/−0 | — | — | header comment |
| ch2-T2 | 536–538 | +3/−1 | `hybrid on a team (Chapter~\ref{ch:control}).` | `hybrid on a team, in the variant in which the measured and t` | CHANGED (round 3, D-12 = (a)). |
| ch2-T3 | 744–745 | +2/−1 | `one on a team.` | `one on a team, with the measured and the predicted term adde` | CHANGED (round 3, D-12 = (a)). |

## ch3_system_modelling (3 hunks)

| Hunk | Lines | +/− | Old | New | Decision |
|---|---|---|---|---|---|
| ch3-T1 | 553–554 | +2/−1 | `& Model error appears as a constant residual; bounded by the` | `& Model error appears as a constant residual; bounded by the` | DECISION D-14 (new): provisional floor from repeat-to-repeat variation; author to confirm. |
| ch3-T2 | 558–558 | +1/−1 | `& Appears as a constant residual; bounded by the null scenar` | `& Appears as a constant residual; bounded by the null scenar` | DECISION D-14 (new): provisional floor from repeat-to-repeat variation; author to confirm. |
| ch3-T3 | 572–572 | +1/−1 | `& \textbf{Known to be imperfect}~\cite{shi2020neuralswarm,gi` | `& \textbf{Known to be imperfect}~\cite{shi2020neuralswarm,gi` | round-3 phase wording (see marker above) |

## ch4_control_architectures (5 hunks)

| Hunk | Lines | +/− | Old | New | Decision |
|---|---|---|---|---|---|
| ch4-T1 | 7–7 | +1/−0 | — | — | header comment |
| ch4-T2 | 480–480 | +1/−0 | — | — | CHANGED (round 3, D-12 = (a)): stated as the definition of Strategy 3 in this thesis. |
| ch4-T3 | 484–485 | +2/−1 | `$\hat a_{\mathrm{res}}$; in the attitude loop, \eqref{eq:ctr` | `$\hat a_{\mathrm{res}}$, and the two are \emph{added}, not p` | CHANGED (round 3, D-12 = (a)): stated as the definition of Strategy 3 in this thesis. |
| ch4-T4 | 508–509 | +2/−9 | — | — | CHANGED (round 3, D-12 = (a)): DECISION comment removed; the derivation is the definition. Code |
| ch4-T5 | 588–593 | +6/−2 | `the predicted term in Strategy~3 is the one derived in Secti` | `the predicted term in Strategy~3 is the one derived in Secti` | CHANGED (round 3, D-12 = (a)): scope sentence, no expectation about which variant is better. |

## ch5_experimental_setup (13 hunks)

| Hunk | Lines | +/− | Old | New | Decision |
|---|---|---|---|---|---|
| ch5-T1 | 6–6 | +1/−0 | — | — | header comment |
| ch5-T2 | 32–36 | +5/−1 | `describes the platform, scenarios, measurement chain and pro` | `describes the platform, scenarios, measurement chain and pro` | CHANGED (round 3, D-13 = (b)): the two phases. |
| ch5-T3 | 75–77 | +3/−2 | `Two vehicles are the minimum for the protocol. A third is re` | `Two vehicles are needed for the first phase of the study. A ` | CHANGED (round 3, D-13 = (b)). |
| ch5-T4 | 187–187 | +1/−2 | — | — | CHANGED (round 3, D-13 = (b)): the null scenarios are flown in phase 2; D-14 for the order. |
| ch5-T5 | 204–213 | +10/−1 | `before the campaign proceeds rather than explained afterward` | `rather than explained afterwards.` | DECISION D-14 (new): provisional floor from repeat-to-repeat variation; author to confirm. |
| ch5-T6 | 389–398 | +10/−3 | `Each strategy flies the same scenario settings. A \emph{cell` | `The study has two phases. The \emph{first phase} is the two-` | CHANGED (round 3, D-13 = (b)): two phases. |
| ch5-T7 | 401–402 | +2/−1 | `clean flights being reported as not reached. Further tiers a` | `clean flights being reported as not reached. Further two-veh` | CHANGED (round 3, D-13 = (b)): two phases. |
| ch5-T8 | 412–413 | +2/−1 | `\caption{Settings of the first tier of the comparison. Speed` | `\caption{Settings of the first tier of the first, two-vehicl` | round-3 phase wording (see marker above) |
| ch5-T9 | 435–435 | +1/−3 | — | — | CHANGED (round 3, D-13 = (b)): three-vehicle scenarios in phase 2 (paragraph above). |
| ch5-T10 | 527–532 | +6/−3 | `Strategy~0 can be made on pairs of flights from the same ses` | `Strategy~0 can be made on pairs of flights from the same ses` | DECISION D-14 (new): provisional floor from repeat-to-repeat variation; author to confirm. |
| ch5-T11 | 534–535 | +2/−2 | `the effect-size measure, and the rule by which a difference ` | `the effect-size measure, and the rule by which a difference ` | DECISION D-14 (new): provisional floor from repeat-to-repeat variation; author to confirm. |
| ch5-T12 | 581–583 | +3/−2 | `Measurement floor & Null scenarios C1--C3 bound it under the` | `Measurement floor & Null scenarios C1--C3 bound it under the` | CHANGED (round 3, D-13 = (b), D-14): rows for the two phases. |
| ch5-T13 | 622–628 | +7/−5 | `set, frozen at the hardware gate and selected at run time in` | `set, frozen at the hardware gate and selected at run time in` | CHANGED (round 3, D-13 = (b), D-14). |

## ch6_results_2robot (9 hunks)

| Hunk | Lines | +/− | Old | New | Decision |
|---|---|---|---|---|---|
| ch6-T1 | 6–6 | +1/−0 | — | — | header comment |
| ch6-T2 | 25–27 | +3/−1 | `This chapter reports the systematic comparison on the two-ve` | `This chapter reports the first phase of the study, the syste` | CHANGED (round 3, D-13 = (b)): phase 1. |
| ch6-T3 | 35–40 | +6/−0 | — | `The null scenarios that set the measurement floor are flown ` | DECISION D-14 (new): provisional floor from repeat-to-repeat variation; author to confirm. |
| ch6-T4 | 62–64 | +3/−1 | `A3, A4, A7) together with null coplanar controls (C1--C3) th` | `A3, A4, A7); the null coplanar controls (C1--C3) that bound ` | CHANGED (round 3, D-13 = (b)): C1-C3 in phase 2; the rest of this D-05 paragraph is unchanged. |
| ch6-T5 | 95–96 | +2/−2 | `Section~\ref{sec:setup-stats}, judged against the measuremen` | `Section~\ref{sec:setup-stats}, judged against the provisiona` | round-3 phase wording (see marker above) |
| ch6-T6 | 130–130 | +1/−1 | `measurement floor on the separation error (by the rule of Se` | `provisional floor on the separation error (by the rule of Se` | round-3 phase wording (see marker above) |
| ch6-T7 | 187–187 | +1/−1 | `within the measurement floor (Section~\ref{sec:setup-stats})` | `within the provisional floor (Section~\ref{sec:setup-stats})` | round-3 phase wording (see marker above) |
| ch6-T8 | 223–223 | +1/−1 | `\paragraph{Negative answer.} If Strategy~3 does not differ f` | `\paragraph{Negative answer.} If Strategy~3 does not differ f` | round-3 phase wording (see marker above) |
| ch6-T9 | 263–263 | +1/−1 | `measurement floor (Section~\ref{sec:setup-stats}), RQ2 is an` | `provisional floor (Section~\ref{sec:setup-stats}), RQ2 is an` | round-3 phase wording (see marker above) |

## ch7_results_multirobot (5 hunks)

| Hunk | Lines | +/− | Old | New | Decision |
|---|---|---|---|---|---|
| ch7-T1 | 6–6 | +1/−0 | — | — | header comment |
| ch7-T2 | 22–30 | +9/−5 | `RQ3 (Section~\ref{sec:intro-rq}) asks for the transfer penal` | `This chapter reports the second phase of the study: the thre` | CHANGED (round 3, D-13 = (b)): phase 2. |
| ch7-T3 | 35–35 | +1/−2 | — | — | CHANGED (round 3, D-13 = (b)): the chapter reports phase 2. |
| ch7-T4 | 50–63 | +14/−0 | — | `\section{Null scenarios and the measurement floor}` | DECISION D-14 (new): provisional floor from repeat-to-repeat variation; author to confirm. |
| ch7-T5 | 111–114 | +4/−1 | `on one neighbour at a time. They say nothing about the summe` | `on one neighbour at a time. They say nothing about the summe` | CHANGED (round 3, D-14). |

## ch8_discussion (2 hunks)

| Hunk | Lines | +/− | Old | New | Decision |
|---|---|---|---|---|---|
| ch8-T1 | 6–6 | +1/−0 | — | — | header comment |
| ch8-T2 | 121–124 | +4/−1 | `RMSE gaps.` | `RMSE gaps. Because they are flown in the second phase, the d` | DECISION D-14 (new): provisional floor from repeat-to-repeat variation; author to confirm. |

## ch9_conclusion (4 hunks)

| Hunk | Lines | +/− | Old | New | Decision |
|---|---|---|---|---|---|
| ch9-T1 | 6–6 | +1/−0 | — | — | header comment |
| ch9-T2 | 32–35 | +4/−2 | `campaign is described in Chapter~\ref{ch:setup} and reported` | `campaign is described in Chapter~\ref{ch:setup} and reported` | CHANGED (round 3, D-13 = (b)). |
| ch9-T3 | 45–46 | +2/−1 | `whether it differs from Strategy~0 beyond the measurement fl` | `whether it differs from Strategy~0 beyond the measurement fl` | CHANGED (round 3, D-13 = (b)). |
| ch9-T4 | 92–95 | +4/−2 | `\item \textbf{Three-robot scaling.} Fly or analyse B1/B2/B3 ` | `\item \textbf{Larger teams.} The three-vehicle scenarios are` | CHANGED (round 3, D-13 = (b)): the three-vehicle scenarios are phase 2 of this thesis, not future |

## thesis_draft (1 hunks)

| Hunk | Lines | +/− | Old | New | Decision |
|---|---|---|---|---|---|
| the-T1 | 2–2 | +1/−1 | — | — | header comment |

## Round-2 sentences that no longer print verbatim (all replaced deliberately)

Every round-2 sentence either prints in round 3 or is listed here with its replacement (sentence-survival check, comments stripped; 0 sentences survive only inside comments).

| Chapter | Similarity | Round-2 sentence (start) | Round-3 replacement (start) |
|---|---|---|---|
| ch1_introduction | 0.97 | `\item[Strategy 3, INDI + NN.] The measured residual of Strategy~1 and ` | `\item[Strategy 3, INDI + NN.] The measured residual of Strategy~1 and ` |
| ch1_introduction | 0.56 | `Each question is answerable by the experiments described in Chapter~\r` | `It is described in Chapter~\ref{ch:setup} and offered for reuse.` |
| ch1_introduction | 0.97 | `\begin{enumerate} \item \textbf{A controlled comparison.} To the best ` | `\begin{enumerate} \item \textbf{A controlled comparison.} To the best ` |
| ch1_introduction | 0.93 | `\item \textbf{A quantification of what the summed per-neighbour approx` | `\item \textbf{A quantification of what the summed per-neighbour approx` |
| ch1_introduction | 0.98 | `\textbf{Chapter~\ref{ch:setup}} describes the platform, the frozen sce` | `\textbf{Chapter~\ref{ch:setup}} describes the platform, the frozen sce` |
| ch1_introduction | 0.82 | `The \textbf{two-robot results chapter} reports the two-vehicle results` | `The \textbf{two-robot results chapter} reports the results of the firs` |
| ch2_related_work | 0.59 | `This thesis flies such a hybrid on a team (Chapter~\ref{ch:control}).` | `This thesis flies such a hybrid on a team, in the variant in which the` |
| ch2_related_work | 0.88 | `An INDI-based hybrid on a team, in which the residual is larger and is` | `An INDI-based hybrid on a team, in which the residual is larger and is` |
| ch3_system_modelling | 0.95 | `\begin{table}[t] \centering \caption{Assumptions of the residual formu` | `\begin{table}[t] \centering \caption{Assumptions of the residual formu` |
| ch4_control_architectures | 0.9 | `In the position loop, \eqref{eq:ctrl-ades} subtracts both the filtered` | `In the position loop, \eqref{eq:ctrl-ades} subtracts both the filtered` |
| ch4_control_architectures | 0.81 | `Third, the interaction of the measured and the predicted term in Strat` | `Third, the interaction of the measured and the predicted term in Strat` |
| ch5_experimental_setup | 0.6 | `Two vehicles are the minimum for the protocol.` | `Two vehicles are needed for the first phase of the study.` |
| ch5_experimental_setup | 0.89 | `A third is required for the three-robot scenarios, which cannot be run` | `A third is required for the three-robot scenarios of the second phase,` |
| ch5_experimental_setup | 0.93 | `And they detect model error: a residual that appears when no neighbour` | `And they detect model error: a residual that appears when no neighbour` |
| ch5_experimental_setup | 0.75 | `\subsection{Comparison campaign} \label{sec:setup-campaign} Each strat` | `\subsection{Comparison campaign} \label{sec:setup-campaign} The study ` |
| ch5_experimental_setup | 0.92 | `The first tier of the campaign comprises twelve settings at a lower-ve` | `The first tier of the first phase comprises twelve two-vehicle setting` |
| ch5_experimental_setup | 0.71 | `Further tiers are flown only if time remains.` | `Further two-vehicle tiers are flown only if time remains before the se` |
| ch5_experimental_setup | 0.85 | `\begin{table}[t] \centering \caption{Settings of the first tier of the` | `\begin{table}[t] \centering \caption{Settings of the first tier of the` |
| ch5_experimental_setup | 0.46 | `The null scenarios bound the measurement floor (Section~\ref{sec:setup` | `They bound the measurement floor, so that a reported improvement can b` |
| ch5_experimental_setup | 0.96 | `\todo{Fix before the campaign: the summary statistic per cell, the pai` | `\todo{Fix before the campaign: the summary statistic per cell, the pai` |
| ch5_experimental_setup | 0.94 | `\section{Threats to validity} \label{sec:setup-validity} \begin{table}` | `\section{Threats to validity} \label{sec:setup-validity} \begin{table}` |
| ch5_experimental_setup | 0.54 | `Strategies~0--3 are one controller with one shared gain set, frozen at` | `Strategies~0--3 are one controller with one shared gain set, frozen at` |
| ch6_results_2robot | 0.9 | `\chapter{Two-Robot Experimental Results} \label{ch:results2} This chap` | `\chapter{Two-Robot Experimental Results} \label{ch:results2} This chap` |
| ch6_results_2robot | 0.93 | `Scenarios are drawn from the frozen formation library (Chapter~\ref{ch` | `Scenarios are drawn from the frozen formation library (Chapter~\ref{ch` |
| ch6_results_2robot | 0.95 | `Statistic: the per-cell summary and the paired within-session differen` | `Statistic: the per-cell summary and the paired within-session differen` |
| ch6_results_2robot | 0.97 | `\paragraph{Negative answer.} If no compensating strategy differs from ` | `\paragraph{Negative answer.} If no compensating strategy differs from ` |
| ch6_results_2robot | 0.98 | `Training scenarios held out from each panel stated in the caption.} \t` | `Training scenarios held out from each panel stated in the caption.} \t` |
| ch6_results_2robot | 0.98 | `\todo[color=red!25]{PLACEHOLDER: Table --- Strategy~3 against Strategy` | `\todo[color=red!25]{PLACEHOLDER: Table --- Strategy~3 against Strategy` |
| ch6_results_2robot | 0.96 | `\paragraph{Negative answer.} If no strategy is preferable in one class` | `\paragraph{Negative answer.} If no strategy is preferable in one class` |
| ch7_results_multirobot | 0.62 | `\chapter{Multi-Robot and Scaling Results} \label{ch:results3} RQ3 (Sec` | `It flies the null scenarios C1--C3, which set the measurement floor ag` |
| ch7_results_multirobot | 0.72 | `This chapter reports that penalty on the B-series scenarios if three-r` | `The chapter also states how an extension to four or five vehicles woul` |
| ch7_results_multirobot | 0.6 | `\todo[color=red!25]{PLACEHOLDER: Table --- B1/B2/B3 (subset as flown) ` | `\todo[color=red!25]{PLACEHOLDER: Table --- C1--C3 $\times$ strategies ` |
| ch9_conclusion | 0.8 | `Strategies~0--3, the baseline, the reactive, the predictive and the hy` | `Strategies~0--3, the baseline, the reactive, the predictive and the hy` |
| ch9_conclusion | 0.98 | `\section{Answers to the research questions} \label{sec:conclusion-rq} ` | `\section{Answers to the research questions} \label{sec:conclusion-rq} ` |
| ch9_conclusion | 0.63 | `\item \textbf{Three-robot scaling.} Fly or analyse B1/B2/B3 under the ` | `\item \textbf{Larger teams.} The three-vehicle scenarios are the secon` |

## Hunks whose marker is further away

| Hunks | Decision |
|---|---|
| ch1-T5, ch1-T6, ch1-T8, ch3-T3, ch5-T8 | D-13 = (b): phase wording ("in the second phase", "two-phase protocol", "first, two-vehicle phase") |
| ch6-T5 … ch6-T9 | D-14: "measurement floor" → "provisional floor" in the Ch6 statistic slot and the four negative-answer paragraphs; one chapter-level `% DECISION D-14` paragraph near the top of Ch6 defines the term |
| header-comment hunks | revision notes only |

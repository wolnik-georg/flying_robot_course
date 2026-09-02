# Citation edit log

Rows are appended for every edited sentence that contains a `\cite` **or a number about prior work**,
after the `citation-audit-2026-08-27` baseline.

This log records **what changed and why**, not what the papers claim. Claims live in the `.tex`,
which is the source of truth. Do not copy claims here, and do not re-verify unchanged locked papers.

---

## Files whose SHA changed vs `CITATION_BASELINE.md`

| File | Status |
|---|---|
| `ch1_introduction.tex` | **changed** |
| `ch2_related_work.tex` | **changed** |
| `ch3_system_modelling.tex` | **changed** |
| `ch4_control_architectures.tex` | **changed** |
| `ch5_experimental_setup.tex` | **changed** |
| `docs/references.bib` | unchanged |
| `ch1`–`ch5_standalone.tex` | unchanged (wrappers) |

Regenerate with `sha256sum ch*.tex ../references.bib` once this pass is settled.

---

## Edits

| Date | File | Cite key | Old fragment | New fragment | Why |
|---|---|---|---|---|---|
| 2026-09-01 | ch1 | none | "For Strategies 0, 1, 2 and 4 it is" | "For Strategies 1, 2, 3 and 5 it is" | numbering |
| 2026-09-01 | ch1 | none | "does not extend to Strategies 3 and 6" | "does not extend to Strategies 4 and 6" | numbering |
| 2026-09-01 | ch2 | `zhang2024proxfly` | separation cell as one long line | same text wrapped with `\makecell` | clarity (overfull table) |
| 2026-09-01 | ch2 | `hsieh2026flatness` | separation cell as one long line | same text wrapped with `\makecell` | clarity (overfull table) |
| 2026-09-01 | ch2 | none | `\subsection{The superposition assumption}` | `\paragraph{...}`, label kept | structure (lone child) |
| 2026-09-01 | ch2 | none | `\subsection{Comparisons that do exist}` | `\paragraph{...}`, label kept | structure (lone child) |
| 2026-09-01 | ch2 | various | 15 clause-glue em-dashes | commas / sentence breaks | style (student comment) |
| 2026-09-01 | ch3 | none | "two controllers that compensate differently-defined residuals are not comparable" | families "are comparable as *controllers*… not required to compute the same internal quantity" | student comment: the old claim was wrong |
| 2026-09-01 | ch3 | `shi2020neuralswarm`, `hsieh2026flatness`, `kiran2025downwash`, `gielis2023aggregate` | force-scope sentence citing Kiran only | adds that Neural-Swarm compensates the vertical force, Hsieh's hardware FBL commands thrust and rates, and Kiran/Gielis measure force and moment **on a rig** | clarity: name the company the scoping decision keeps |
| 2026-09-01 | ch3 | `shi2020neuralswarm`, `shi2022neuralswarm2`, `cobobriesewitz2026lindi` | (new paragraph) | states family = deep sets from Neural-Swarm/NS2; widths and 987 params are **ours**; not Cobo's network; no spectral normalisation; ablation out of scope | student comment: architecture provenance was ambiguous |
| 2026-09-01 | ch3 | none | firmware path inline (overfull) | paths moved to a footnote | style (overfull line) |
| 2026-09-01 | ch4 | none | Strategy sections 0–7 | 1, 2, 3, 4, 5, 6, 7a, 7b + labels `sec:ctrl-s1..s7b` | numbering |
| 2026-09-01 | ch4 | none | Table 4.1 inside §4.9 | moved to chapter start, rows renumbered, 7a/7b split | structure: reader sees the set before the equations |
| 2026-09-01 | ch4 | none | "Every strategy is built on a geometric tracking controller on SE(3)" | "On *this* stack, Strategies 1, 2, 3 and 5 share a geometric substrate…" | student comment: was a claim about the literature |
| 2026-09-01 | ch4 | none | "representatives of their families, not reproductions" | core 1/2/3/5 follow families on one substrate, deviations listed, not bit-identical; optional 4/6/7 await author code | clarity |
| 2026-09-01 | ch4 | `zhang2024proxfly` | one section "Strategies 5 and 7" | split into 7a (cascaded, ProxFly) and 7b (geometric, ours), shared stance vs different inner loop | structure |
| 2026-09-01 | ch4 | `lee2010geometrictracking` | reduced-law paragraph | adds that restoring the feedforward bracket is a firmware change, outside this draft | clarity |
| 2026-09-01 | ch4 | none | eq. (4.6) began with `\dots` | full `a_des` line printed, last two terms labelled | student comment |
| 2026-09-01 | ch4 | none | "the intermediate ones are diagnostic" | adds that they are **not** additional strategies and will not be reported as Strategy 2 | clarity |
| 2026-09-01 | ch4 | `chee2024knodedwmpc`, `hsieh2025onlineadaptation`, `li2023ndpnmpc` | "computational cost may exclude it" | adds that nothing settles it: Chee and Hsieh run off-board, Li runs on a Jetson, not an STM32 | student comment: onboard feasibility stays open |
| 2026-09-01 | ch4 | none | "No training data." | "The method needs no training data; the residual is formed online from inertial and rotor-speed measurements." | style (stub) |
| 2026-09-01 | ch5 | none | Strategy numbers 0/1/2/3/4/6 | 1/2/3/4/5/6 | numbering |
| 2026-09-01 | ch5 | none | metrics: position RMSE only | adds attitude error $e_R$ (RMSE and peak), **and states the signal is not yet logged** and will be added before the campaign | student comment |
| 2026-09-01 | ch5 | none | "A1 and C1--C3 are static" | names them at first mention | clarity |
| 2026-09-01 | ch1–ch5 | none | 77 clause-glue em-dashes | commas / sentence breaks | style |

---

## Not changed in this pass

Locked citation numbers and experiment-identity facts are untouched: Neural-Swarm 2.4/9.4 and the
training-set split; NS2 16 @ 24 cm vs body ~1.5× vs abstract 3×; Smith 36 % = 3D; Chee off-board;
Hsieh 2025 hardware-only and off-board; Hsieh 2026 31 % abstract-only; Cobo 40 % no-payload;
ProxFly cascaded and off-board; Lee's full $M$ vs our reduced $\tau$; Kiran = rig; Bauersfeld
scale = $l$. The bibliography is unchanged. Tal's optical encoders and Smeur's motor-speed
measurement remain separate.

---

## Consistency pass, 2026-09-01 (second)

| Date | File | Cite key | Old fragment | New fragment | Why |
|---|---|---|---|---|---|
| 2026-09-01 | ch1 | `bauersfeld2024airflow` | "part of the problem , the mean flow" | "part of the problem: the mean flow" | punctuation scar left by em-dash removal; no claim changed |
| 2026-09-01 | ch4 | none | "strategies 1, 2 and 4 share the entire control loop" | "Strategies 2, 3 and 5 share the entire control loop" | **renumber bug** — the sentence is about INDI / geo+NN / hybrid |
| 2026-09-01 | ch1 | none | "defines the residual wrench that every strategy estimates" | "defines the residual this stack measures and logs… Strategy 7 does not form it" | leftover of the removed same-quantity claim |
| 2026-09-01 | ch3 | none | "is the single quantity every strategy in this thesis estimates" | "is what this stack measures and logs. Strategies 2, 3 and 5 consume it. Strategy 7 never forms it…" | same leftover |
| 2026-09-01 | ch4 | none | "Role. Not one of the seven, but flown as a condition…" | "Strategy 1 is the uncompensated baseline of Table 4.1. It is flown in every scenario so that…" | Strategy 1 is in the table, not outside the set |
| 2026-09-01 | ch1–ch5 | none | "the seven strategies" (5 places) | "Strategies 1–6 plus 7a and 7b (eight controllers; seven families, with residual RL split by inner loop)" | seven-vs-eight consistency |
| 2026-09-01 | ch4 | none | "Finally, each implementation is a representative of its family and not a reproduction…" | deleted | duplicated the new families-on-one-substrate paragraph |
| 2026-09-01 | ch1 | none | "the two- and multi-robot results chapters report… the discussion chapter…" | sentence-initial capitals | typography |
| 2026-09-01 | ch4 | none | "All of them track… through the same attitude allocation." | "The fair set, Strategies 1, 2, 3 and 5, tracks… Strategies 4, 6 and 7 do not share that allocator." | the claim was too broad after the renumber |

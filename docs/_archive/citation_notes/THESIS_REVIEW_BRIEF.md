# Thesis Writing Track — External Review Brief

**Thesis:** *Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams*
Georg Wolnik, supervised by Prof. Wolfgang Hönig.
**Prepared:** 23 August 2026. **Revised:** 24 August 2026 after a revision round, an accuracy audit and a full-text reading of every held source. **For:** independent review of the theoretical/writing track.

---

## 1. What you are being asked to check

Five thesis chapters were drafted from scratch, together with the planning documents behind them.
**Nothing has been flown.** No result is claimed anywhere. What is being reviewed is whether the
*argument* holds and whether the *sourcing* is sound.

The five questions that matter most:

1. **Is the gap claim defensible?** It has already been narrowed twice under evidence (§5).
2. **Is the fairness argument sound** — that strategies 0/1/2/4 are terms in one control law, and
   that this does not extend to strategies 3 and 6?
3. **Are the research questions answerable** by the experiment described, and is each hypothesis
   falsifiable? Note that **RQ3 was narrowed** once reading showed the literature already answers
   its original form.
4. **Is anything overclaimed**, especially in the contribution statement?
5. **Is any citation wrong?** Three misattributions and eighteen further corrections have already
   been caught here (§6) — assume there are more.

---

## 2. What was produced

### Thesis chapters — `docs/thesis/`

| Ch. | Title | Words | State |
|---|---|---|---|
| 1 | Introduction | 2,020 | Draft |
| 2 | Background and Related Work | 4,141 | Draft |
| 3 | System Modelling and Residual-Force Formulation | 2,433 | Draft |
| 4 | Control Architectures | 3,098 | Draft |
| 5 | Experimental Setup and Protocol | 2,664 | Draft |
| | **Total** | **14,356** | **38 pages, builds clean** |

Chapters 6–9 (results, discussion, conclusion) require flight data and are not started.

Build: `cd docs/thesis && latexmk -pdf thesis_draft.tex`. Zero undefined citations, zero dangling
cross-references, all 20 sources cited.

### Supporting documents — `docs/`

| Doc | Purpose |
|---|---|
| `15_Problem_Statement_and_Research_Questions.md` | Formal problem statement, 4 RQs with falsifiable hypotheses, threats to validity |
| `16_Related_Work_Structure.md` | The skeleton and argument of Chapter 2 |
| `17_Source_Ledger_and_Citation_Discipline.md` | 7 citation rules, per-source verification status, **and the three misattribution incidents** |
| `18_Strategy_Descriptions.md` | The 7 strategies, with *what we implement* separated from *the reference method* |
| `19_Contribution_Statement.md` | 5 claims + an explicit "what this thesis does **not** claim" table |
| `20_Verified_Claims.md` | **Every citable claim in each paper's own words**, with reading depth per source |
| `references.bib` + `papers/` | 20 sources. **All read in full**, Abro et al. included (obtained 27 Aug) |

---

## 3. The core argument, in brief

**Problem.** A controller can obtain information about the inter-vehicle interaction force in
exactly two ways: **measure** it online (no prior model needed, but delayed and requiring
rotor-speed sensing) or **predict** it from neighbours' relative states using an offline-learned
model (acts ahead of the disturbance, but only valid inside its training distribution). A hybrid is
the obvious third option.

**Gap.** Representative methods exist for each and are individually validated, but — to the best of
our knowledge — they have not been evaluated against one another under identical conditions. The
field's reporting conventions make the comparison impossible to assemble post hoc: each result is
against its own baseline, on its own airframe, at its own separations.

**Approach.** Frozen 16-scenario formation library; gains frozen before data collection and shared;
one residual definition logged under every controller including those not using it; null scenarios
to establish the measurement floor; runtime strategy selection so conditions interleave within a
session.

**Fairness argument.** Strategies 0, 1, 2 and 4 are not four controllers but four settings of one
controller, differing only in which residual term is active in the desired-acceleration vector.
This is verifiable against source code, not asserted. **It does not extend to strategies 3 and 6**,
which are structurally different controllers — stated explicitly in Ch. 4 §4.9.

---

## 4. Honest status — please probe these

| | |
|---|---|
| **Nothing has flown** | Every chapter is written in that knowledge. Ch. 5 is deliberately in future tense |
| **3 of 7 strategies are not implemented** | 2 fly, 1 implemented-unflown, 1 exists as separable components, 3 not implemented. Ch. 4 states this per strategy and does not describe unimplemented ones as existing |
| **3 flight-code changes have never flown** | Residual sign fix, `a_res` gating fix, `rnn.en` compensation. None is detectable in single-drone flight |
| **Simulation is used only to disprove** | Never to validate a method — the simulated interaction comes from a learned model of the same family being fitted |
| **One source has no PDF in `papers/`** | Abro et al. (IEEE JESTIE 2025). Its full text was consulted on 27 Aug, so the earlier abstract-only status is obsolete — but with no PDF held, **nothing is quoted from it and none of its numbers are imported**. It is cited for the methods compared and the downwash motivation. **It remains the closest published work to our setting** |

---

## 5. The gap claim has been narrowed twice — please attack it again

This is the claim the contribution rests on, so it has had the most scrutiny.

**Audit 1 (arXiv):** keyword sweep. Found no comparison or survey study. Claim changed from *"the
first systematic evaluation"* to *"to the best of our knowledge, the first…"*.

**Audit 2 (cross-database + citation graph):** OpenAlex, Crossref, DBLP, Semantic Scholar — which
index IEEE Xplore, ACM DL, Springer, Elsevier. Plus the stronger structural test: **all 157 works
citing Neural-Swarm, Neural-Swarm2 or Gielis et al.**, filtered for comparison/survey-like titles.

Audit 2 found **two genuine comparison studies**, both now cited in a new §2.8.1:

| Study | Compares | Why it isn't ours |
|---|---|---|
| Gu, Zhao & Rizzo, JIRS 2025 | 4 *online-learning* estimators (ELM, RBFNN, ESN, GP) | Single vehicle, generic uncertainty, online learning, **simulation throughout** |
| Abro, Alsayegh & Mustafa, IEEE JESTIE 2025 | **FOPID, FOLQR and integral state feedback** on close-proximity, downwash-motivated flight | Multi-vehicle and downwash-aware, but all three are **feedback regulators** — only the last is classical — and none presents a **learned or first-principles predictor** of the interaction force |

**Consequence:** *"Nobody compares controllers for close-proximity flight"* would be **false** —
Abro et al. do. What survives is narrower: the families defined by **how they obtain information
about the interaction force** have not been placed under common conditions.

> **Reviewer question:** is that survivable formulation still too strong? And can you find a study
> either audit missed — particularly one citing none of the three seed papers, or using different
> terminology?

---

## 6. Three misattributions were caught here — assume more remain

Recorded in full in `17_Source_Ledger_and_Citation_Discipline.md` §1. Listed because they show the
failure mode and because the same discipline should be applied to this review.

| # | What happened |
|---|---|
| 1 | `03` claimed the neural-augmented INDI hybrid showed *"limited benefit"*. Propagated into `15` and `16` because both were drafted from our own gap statement rather than the source |
| 2 | The **correction** was also wrong. Made from our own detailed summary (`paper_summaries.md`), which said the hybrid *"achieved the lowest tracking error"*. The paper shows the margin is **small and condition-dependent** (4.14 vs 4.28 cm no-payload; *"remains similar"* with payload), and its headline is **sensor elimination**, not the hybrid. That summary now carries a DO-NOT-CITE banner |
| 3 | Gu et al.'s abstract says *"simulation and hardware experiments"*; our note implied real flight. **No flight took place** — the hardware component is a Jetson in the loop with a simulator, and the conclusion lists real-quadrotor validation as future work. The abstract was not wrong; we read into it more than it said |

**A further 18 corrections followed on 24 August**, from an accuracy audit and then a full-text
reading of every held paper. The four that changed meaning rather than wording:

| Finding | Effect |
|---|---|
| **Smith et al. describe their 36 % three different ways** — abstract says "3D trajectory tracking", contributions list says "lateral", results measure a lateral transect | **Resolved the other way on 26 Aug.** The primary assignment is now **36 % 3D / 56 % vertical, "on average"**, following the abstract and the translation-result body; the contributions list's "lateral" is the dissenting reading, footnoted as the paper's own inconsistency. Transect figures are reported separately (0.154 → 0.098 m; vertical 55 % / 49 %) |
| **Hsieh et al.'s controller runs off-board** (i5 laptop, 400 Hz radio) | The 5 ms loop we cited as evidence of deployability is a workstation figure. Now stated as such |
| **Gielis et al. report three interaction regimes**, one of which is *well estimated by linear summation* | We had said they refute superposition. They refine it — which sharpens RQ3 |
| **Neural-Swarm trained separate networks** per team size, not one on combined data | A weaker generalisation claim than we implied |

Two attributes in our own literature matrix were refuted outright by the full text: Gielis et al.
"usable onboard" (the paper never discusses onboard deployment), and the framing of Neural-Swarm's
baseline as "a geometric controller" (it is a nonlinear tracking controller whose attitude part
cites Lee et al.).

### A further pass over eight papers, 26–27 August

An independent close reading re-checked **Neural-Swarm, Neural-Swarm2, Hsieh 2026, Cobo-Briesewitz,
Smith, Chee 2024, Gielis and Hsieh 2025**, and returned corrections for every one. All are applied
to the chapters and recorded in `20_Verified_Claims.md`.

**The recurring fault was not misreading a number but attaching it to the wrong thing** — the wrong
axis, the wrong training set, the wrong platform, the wrong experiment, or an abstract-level summary
presented as a body result. Representative corrections:

| Finding | Effect |
|---|---|
| Neural-Swarm's **2.4 cm is the four-vehicle-trained** network | The two-vehicle-trained one reaches 2.7 cm. The abstract's phrasing does not separate task from training set |
| Neural-Swarm2's **16 robots @ 24 cm and its 3×** are separate reports | On that flight the body gives ≈1.5× (15 → 10 cm). They must never share a sentence |
| Hsieh 2026's **31 % is abstract-only**; its **20× is simulation** | The body gives 24 %/30 % and a further 29 %/43 %; the intro says 7× |
| Cobo's **40 % is no-payload only** | With a payload the reductions are 19–24 %, and the hybrid margin is "similar", not "small" |
| Gielis's sufferer is **held on a load stand**, not in free flight | And "graph" is abstract language for a Deep-Sets implementation; K ≤ 3, not four neighbours |
| **Chee 2024 = 2 robots · Hsieh 2025 = 3 robots · Hsieh 2026 = 2 robots, FBL** | Shared authors and platform; they were being conflated. The 20× belongs to none but Hsieh 2026, in simulation |

> **Reviewer note:** `02_Literature_Matrix.md` is a working note, not a source. Where it and
> `20_Verified_Claims.md` disagree, the ledger is authoritative. Three of the misattributions above
> arose from treating our own summary files as primary.

**The pattern in all three: trusting a secondary source as primary.** `20_Verified_Claims.md` now
records every citable claim in the source's own words, with reading depth stated per paper.

---

## 7. Specific things worth challenging

1. **RQ3 was reframed** after reading revealed the literature already answers the original version
   (Shi et al. 2020 state the effect *"is not a simple superposition of each pair"*). It now asks
   *how much* accuracy a 2-robot-trained model loses on 3 robots. **Is the narrowed version still
   a contribution, or is it too small to carry a research question?**

2. **Frozen shared gains vs per-method tuning.** Gu et al. chose the opposite (Bayesian
   optimisation per method). Ch. 4 §4.9 argues our choice is right *because strategies 1/2/4 share
   the whole loop*. **Is that reasoning sound, or does it privilege the strategies that share a
   substrate over those that do not?**

3. **Strategy 4's form is now fixed** to the reference arrangement: the network predicts the bulk
   and the incremental term corrects only the remainder. Independent subtraction was rejected
   because it double-counts, so a *better* prediction would over-compensate. **Is that the right
   call, given that it partly masks a poor prediction?** The thesis answers by assessing model
   quality open-loop instead, but the reviewer should test whether that is sufficient.

4. **The apparatus has been demoted** out of the numbered contributions entirely. **Is that the
   right treatment, or should it not appear at all?**

5. **A null result is designed to be reportable.** If the strategies do not separate, the claimed
   conclusion is that compensation family matters less than compensation presence. **Is the design
   genuinely able to distinguish that from a failed experiment?**

---

## 8. What is still to do

| Item | Blocked by |
|---|---|
| **Obtain Abro et al. full text** | Institutional access — the last verification gap |
| Chapters 6–7 (results) | Flight data (C.4) |
| Chapters 8–9 (discussion, conclusion) | Flight data |
| Tape-measure the flight volume | Lab access |
| Revise Ch. 5 to past tense | The campaign itself |

| ~~Full re-audit of `02_Literature_Matrix.md`~~ | ✅ Corrected 2026-08-24 from the full-text readings |

---

## 9. Files to read, in order

1. **`docs/thesis/thesis_draft.pdf`** — the 38-page compiled draft. Start here.
2. **`docs/20_Verified_Claims.md`** — what we may cite and why. The most important supporting file.
3. **`docs/17_Source_Ledger_and_Citation_Discipline.md`** — the rules and the three incidents.
4. **`docs/19_Contribution_Statement.md`** — claims and the explicit non-claims.
5. **`docs/15_Problem_Statement_and_Research_Questions.md`** — RQs, hypotheses, threats to validity.

**These files are deliberately not in git.** The whole writing track — `docs/thesis/`,
`docs/papers/`, `references.bib`, `docs/15`–`20` and this brief — is gitignored and was purged from
the repository history on 24 August 2026, so it exists only on the author's machine. The public
repository at `github.com/wolnik-georg/flying_robot_course` holds the engineering track
(`docs/01`–`14`, flight code, experiments) and none of the material described above. Ask the author
directly for a copy; do not expect to find it by cloning.

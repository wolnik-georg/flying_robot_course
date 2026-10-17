# Source Ledger and Citation Discipline

**Track:** Writing. **Created:** 23 August 2026.

This document exists because a misattribution has already happened in this project, and it was
caught by accident rather than by process.

---

## 1. Two incidents, on the same paper

### Incident 1 — a claim copied from our own gap statement

[`03_Gap_and_Contribution_Statement.md`](03_Gap_and_Contribution_Statement.md) stated that the
neural-augmented INDI hybrid "has been proposed for single vehicles, but **its benefits were
limited**". That propagated into the first drafts of [`15`](15_Problem_Statement_and_Research_Questions.md)
and [`16`](16_Related_Work_Structure.md), because those were written from `03` and
[`02`](02_Literature_Matrix.md) rather than from the source.

### Incident 2 — the *correction* was also wrong

The fix was made from `flying_drone_stack/paper_summaries.md`, which states:

> *"NA-INDI lowest error, IL-NDI slightly better than classical INDI, both clearly beat baseline
> geometric controller."*

So `03`, `15`, `16` were corrected to say the hybrid **achieved the lowest tracking error of the
methods compared**. When the actual paper was obtained (W.2b) and read, that was wrong too:

| Figure-8, mean tracking error | Lee | INDI | LINDI | NA-INDI |
|---|---|---|---|---|
| No payload | 7.07 | 4.28 | 4.41 | **4.14** |
| With payload | 7.06 | **5.38** | 5.46 | 5.39 |

The paper's own words: without payload NA-INDI "leads to the best performance, **although with a
small margin**"; with payload it "shows the potential to outperform INDI, but **still remains
similar in performance**". And its headline contribution is neither — it is **LINDI**, a learned
residual that matches INDI *without requiring rotor-speed sensors*.

Also wrong in our summary: the method is called **LINDI**, not "IL-NDI"; and LINDI is *similar to*
INDI, not better (4.41 vs 4.28 on figure-8, slightly worse).

### Incident 3 — an abstract that overstates, and we repeated it

Gu et al. (JIRS 2025) state in their abstract: *"Through extensive simulation and hardware
experiments"*. Our first note on the paper, written from that abstract, said the same. Reading the
paper shows **no real flight took place** — the "hardware experiments" are the algorithms running
on a Jetson in the loop with a simulator, and the conclusion lists real-flight validation as future
work.

The lesson is narrower than the first two and more uncomfortable: the abstract was not *wrong*
(embedded hardware was used), but it was **read into more than it said**. Rule 2 already covers
this — reading the abstract supports "this paper proposes X", not a claim about what was flown.

### What this sequence demonstrates

The first error came from trusting our own gap statement. The second came from trusting our own
detailed summary — a document that *looks* authoritative, running to fifty lines with method
names, hyperparameters and results. **Both failures were reading a secondary source and treating
it as primary.** The rule below is not bureaucracy; it is the only thing that would have caught
either.

It also shows the practical cost of getting this wrong in both directions. "Benefits were limited"
understated the result; "achieved the lowest tracking error" overstated it. The accurate version —
*a real but small and condition-dependent advantage, on a single vehicle, where the paper's actual
headline is sensor elimination* — is the one that makes our RQ4 honest: we are asking whether a
small margin becomes a meaningful one when the residual comes from neighbours instead of from the
vehicle itself. That is a real open question. The overstated version would have made our
hypothesis look like a foregone conclusion, and the understated version would have made it look
like reviving a failed idea.

## 2. The rules

1. **One claim, one source.** Every factual statement about prior work cites exactly one paper.
   If two papers are needed, it is two statements.
2. **Never cite a paper you have not read at the level of the claim.** Reading the abstract
   supports "this paper proposes X"; it does not support "this paper reports a 36 % improvement".
3. **No compound attributions.** "Shi et al. and Gielis et al. show that…" hides which showed what.
   Write two sentences.
4. **Do not cite a summary as if it were the source.** `02_Literature_Matrix.md` and
   `paper_summaries.md` are *our compressed notes*. They are a navigation aid and a drafting
   input — never a citation. Every claim that reaches the thesis is verified against the actual
   paper.
5. **Numbers require the full text.** Percentages, separations, team sizes and error figures are
   copied from the paper with the section noted, or not stated at all.
6. **Separate "what the paper did" from "what we implement."** Our implementations follow published
   methods but are not identical to them. Every strategy description states the deviations
   explicitly — see [`18_Strategy_Descriptions.md`](18_Strategy_Descriptions.md).
7. **When in doubt, mark it and move on.** A `⚠️ UNVERIFIED` marker in a draft costs nothing. An
   unmarked wrong claim in a submitted thesis is an examinable error.

---

## 3. Verification status of every cited source

**Status legend**

| | Meaning |
|---|---|
| 🟢 **FULL TEXT** | Read in full against our claims; quotations and section context in [`20_Verified_Claims.md`](20_Verified_Claims.md) |
| 🟡 **ABSTRACT ONLY** | Full text unavailable. Supports *what the paper proposes*, **nothing about its experiments or results** |
| 🔴 **UNVERIFIED** | Not read against our claims. **No specific claim may be made.** |

> ✅ **Resolved, and extended 2026-08-24.** **Every held paper has now been read in full**; the only
> source then limited to its abstract was Abro et al. **Its full text was consulted on 27 Aug**,
> so every source has now been read in full — but **the Abro PDF is not in `papers/`**, so
> nothing may be *quoted* from it under Rule 5. All other papers are in
> [`papers/`](papers/) with [`references.bib`](references.bib) — **20 sources, 18 PDFs held** — and
> every quotable claim, in the paper's own words, is in
> [`20_Verified_Claims.md`](20_Verified_Claims.md), with the depth of reading stated per paper.
>
> ⚠️ **Re-verified 26–27 August 2026.** An independent close reading of **eight** papers —
> Neural-Swarm, Neural-Swarm2, Hsieh 2026, Cobo-Briesewitz, Smith, Chee 2024, Gielis, Hsieh 2025 —
> returned corrections for **every one**, recorded as ⚠️ **Corrections** blocks in `20`. Reading a
> paper in full is evidently not sufficient on its own: the recurring fault was **attaching a
> correct number to the wrong thing** — the wrong axis, training set, platform, experiment, or an
> abstract-level summary presented as a body result.
>
> **One earlier decision was reversed.** Smith's 36 % had been assigned to *lateral* tracking on the
> strength of the contributions list; it is now **3D**, following the abstract and the
> translation-result body, with the contributions list footnoted as the paper's own inconsistency.
> A reading that was defensible when made can still be wrong.

| Paper | arXiv | In-repo PDF | Status |
|---|---|---|---|
| Cobo-Briesewitz, Wahba, Hönig — *Learned INDI for Quadrotors with and without Slung Payloads*, **L4DC 2026** | 2503.09441 | ✅ | 🟢 full text |
| Smith, Shankar, Gielis, Blumenkamp, Prorok — *SO(2)-Equivariant Downwash Models for Close Proximity Flight*, 2023 | 2305.18983 | ✅ | 🟢 full text |
| Shi, Hönig, Yue, Chung — *Neural-Swarm*, 2020 | 2003.02992 | ✅ | 🟢 full text |
| Shi, Hönig, X. Shi, Yue, Chung — *Neural-Swarm2*, **T-RO 2022** (preprint 2020) | 2012.05457 | ✅ | 🟢 full text |
| Gielis, Shankar, Kortvelesy, Prorok — *Modeling Aggregate Downwash Forces for Dense Multirotor Flight*, 2023 | 2312.03488 | ✅ | 🟢 full text |
| P.-A. Hsieh, Yang, Matni, M. A. Hsieh — *Flatness-Preserving Residual Learning for Real-Time Tight Quadrotor Formation Flight*, 2026 | 2607.12275 | ✅ | 🟢 full text |
| Yang, Welde, Matni — *Learning Flatness-Preserving Residuals for Pure-Feedback Systems*, 2025 | 2504.04324 | ✅ | 🟢 full text |
| Zhang, Zhang, Mueller — *ProxFly*, 2024 | 2409.13193 | ✅ | 🟢 full text |
| Chee, P.-A. Hsieh, Pappas, M. A. Hsieh — *Flying Quadrotors in Tight Formations using Learning-based MPC*, 2024 | 2410.09727 | ✅ | 🟢 full text |
| P.-A. Hsieh, Chee, M. A. Hsieh — *Online Adaptation for Flying Quadrotors in Tight Formations*, 2025 | 2506.17488 | ✅ | 🟢 full text |
| Kiran, Ayanian, Breuer — *Influence of Static and Dynamic Downwash Interactions on Multi-Quadrotor Systems*, 2025 | 2507.09463 | ✅ | 🟢 full text |
| Bauersfeld, Muller, Ziegler, Coletti, Scaramuzza — *Robotics meets Fluid Dynamics*, **RA-L 2024** | 2403.13321 | ✅ | 🟢 full text |
| Tal, Karaman — *Accurate Tracking of Aggressive Quadrotor Trajectories using INDI and Differential Flatness*, 2018 | 1809.04048 | ✅ | 🟢 full text |
| Smeur, de Croon, Chu — *Cascaded INDI for MAV Disturbance Rejection*, 2017 (journal 2018) | 1701.07254 | ✅ | 🟢 full text |
| **Lee, Leok, McClamroch — *Geometric tracking control of a quadrotor UAV on SE(3)*, CDC 2010** | — (DOI) | ❌ | 🟢 **PRIMARY citation** — metadata via Crossref; equations verified in the arXiv companion |
| Lee, Leok, McClamroch — *Control of Complex Maneuvers … on SE(3)*, 2010 | 1003.2005 | ✅ | 🟢 full text |
| Li, Han, Yu, Lin, Li, Ren — *NDP-NMPC*, 2023 | 2304.07794 | ✅ | 🟢 full text |
| Shankar, Woo, Prorok — *Docking Multirotors using Learnt Downwash Models*, 2023 | 2311.13988 | ✅ | 🟢 full text |
| Gu, Zhao, Rizzo — *Learning uncertainties online: a comparative study*, **JIRS 2025** | — (DOI) | ✅ OA | 🟢 full text |
| Abro, Alsayegh, Mustafa — *Evaluating Fractional-Order Methods for Close-Proximity Operations*, **IEEE JESTIE 2025** | — (DOI) | ❌ not in `papers/` | 🟢 full text **consulted 27 Aug**, PDF not held — cite for methods compared and downwash motivation only; **do not quote, and do not import its numbers** |

**All 15 PDFs are in [`papers/`](papers/) and each was checked to match its filename by reading its
first page.** Bibliographic metadata came from the arXiv API, not from memory or from our notes.
**Every quotable claim is recorded with the paper's own words in
[`20_Verified_Claims.md`](20_Verified_Claims.md)** — cite from there, or from the PDF, never from
`02` or `paper_summaries.md`.

### Known discrepancies to resolve

| Item | Discrepancy | Action |
|---|---|---|
| ~~Cobo-Briesewitz et al. year~~ | ~~`02`/`06` say 2026; `paper_summaries.md` says ~2025~~ | ✅ **Resolved:** arXiv preprint 2025-03, published L4DC **2026**. Both correct; cite the L4DC version |
| Hsieh et al. | Two distinct entries (Flatness-Preserving Residual 2026; L1 KNODE-DW MPC 2025) share a surname and must not be conflated | Always cite with year **and** short title |
| "Neural-Swarm" vs "Neural-Swarm2" | Distinct papers, different architectures, different team sizes | Never write "Neural-Swarm" meaning either |

---

## 4. What to do next (writing track)

- [ ] **Collect the PDFs** into `docs/papers/` (gitignored if size or licensing requires) and
      create `docs/references.bib`. Nothing in Chapter 2 is final until this exists
- [x] Promote each entry to 🟢 — done in W.2c, recorded in [`20`](20_Verified_Claims.md)
- [x] Resolve the discrepancies above — done
- [x] **Lee et al. citation resolved (W.10):** arXiv:1003.2005 contains our exact attitude error
      (eq. 8) and torque law (eq. 11). The citation was correct; the doubt was unfounded
- [x] **"First systematic comparison" audited (W.10)** — arXiv only; claim narrowed to "to the best
      of our knowledge". See [`20`](20_Verified_Claims.md)
- [x] **Non-arXiv sweep done** — OpenAlex, Crossref, DBLP, Semantic Scholar, plus a citation-graph
      scan of 157 works citing the Neural-Swarm line. Two adjacent comparisons found and cited;
      the claim was sharpened rather than withdrawn. See [`20`](20_Verified_Claims.md)
- [ ] Re-audit `02_Literature_Matrix.md` fully against the sources

---

## 5. Standing check before any Related Work text is considered final

For each paragraph:

1. Which single paper does this claim come from?
2. Have I read that paper *at the level of this claim*, or am I repeating our own summary?
3. If a number appears — do I have the section it came from?
4. Am I describing what the paper did, or what we implement? (These are separate sentences.)
5. Could a reader who knows this literature find anything here surprising, in the sense of "that
   is not what that paper says"?

Question 5 is the one worth asking last, and honestly.

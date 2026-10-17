# Thesis text

Actual thesis prose, as opposed to the planning documents in [`../`](../).

| File | Chapter | State |
|---|---|---|
| `ch1_introduction.tex` | 1 — Introduction | draft (W.9), ~1810 words |
| `ch2_related_work.tex` | 2 — Background and Related Work | draft (W.5), ~3250 words |
| `ch3_system_modelling.tex` | 3 — System Modelling and Residual-Force Formulation | draft (W.6), ~2250 words |
| `ch4_control_architectures.tex` | 4 — Control Architectures | draft (W.8), ~2850 words |
| `ch5_experimental_setup.tex` | 5 — Experimental Setup and Protocol | draft (W.7), ~2460 words |

## Building

```bash
latexmk -pdf thesis_draft.tex        # everything drafted so far, 35 pages -- USE THIS
latexmk -pdf ch2_standalone.tex      # one chapter; cross-chapter refs will be undefined
latexmk -C                           # clean
```

Build the combined document when you care about cross-references: Chapter 3 refers back into
Chapter 2, so the standalone builds legitimately report those as undefined.

## Frame convention

**z-up (ENU); gravity acts along $-e_3$.** This matches `firmware_app/src/lib.rs` and Crazyswarm2.
`../04_Unified_Residual_Wrench_Model.md` was written with the opposite sign and is superseded on
that point by Chapter 3 §3.1–3.2. Getting this wrong flips the sign of every compensation term.

`preamble_draft.tex` exists **only** for drafting. It degrades gracefully where packages are
missing (`siunitx` is stubbed if absent) and renders forward references to unwritten chapters as
`[?]` via `\chapref`. The real thesis preamble should require `siunitx` properly and define
`\newcommand{\chapref}[1]{\ref{#1}}`.

Bibliography: [`../references.bib`](../references.bib). PDFs: [`../papers/`](../papers/).

## Before editing — citation discipline

**Every factual claim about prior work must be traceable to
[`../20_Verified_Claims.md`](../20_Verified_Claims.md)**, which records each claim in the source's
own words and states how deeply that source was read.

Do **not** cite [`../02_Literature_Matrix.md`](../02_Literature_Matrix.md) or
`flying_drone_stack/paper_summaries.md`. They are our own notes, and two misattributions in this
project came from treating them as sources — see
[`../17_Source_Ledger_and_Citation_Discipline.md`](../17_Source_Ledger_and_Citation_Discipline.md) §1.

`\todo{}` markers in the text are real open items, not placeholders for prose. Each names what has
to be checked and where.

## The separation Chapter 4 must not lose

Every strategy is presented as **what we implement** (verifiable against
`firmware_app/src/lib.rs` and `residual_nn.rs`) and **the reference method** (cited, with
deviations stated). These are never merged into one sentence: our implementations are
representatives of their families, not reproductions of specific papers, and RQ1's validity rests
on that distinction being visible to the reader.

Implementation status is stated honestly per strategy — two fly, one is implemented and unflown,
one exists as separable components, three are not implemented. **Do not describe an unimplemented
strategy as though it exists.**

## Chapter 5 is written in the future tense on purpose

**Nothing has flown.** Chapter 5 describes a design, not a completed campaign. As results arrive,
revise that chapter to past tense and record protocol deviations *there*, not only in the results
chapters — a setup chapter that no longer matches what was done is worse than none.

## Chapter 1 promises what Chapters 2–5 deliver

Written last of the unblocked chapters, on purpose. Its five contributions correspond one-to-one to
C1–C5 in [`../19_Contribution_Statement.md`](../19_Contribution_Statement.md), and its four research
questions to [`../15_Problem_Statement_and_Research_Questions.md`](../15_Problem_Statement_and_Research_Questions.md).
**If any of those three documents changes, change the other two.** An introduction that promises
something the body does not deliver is the failure mode reviewers find first.

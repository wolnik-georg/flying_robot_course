# Thesis writing rules

**Thesis:** Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams  
**Last updated:** 2026-09-27

This file is the style and doctrine checklist for future prose edits
(manual or agent). It does not replace the `.tex`. Where this file and
a chapter disagree on a *fact*, fix the file or the chapter in a
dedicated pass — do not silently “smooth” the chapter to match a stale
rule.

## 1. Campaign doctrine (do not drift)

Compared strategies:

| # | Name | Role |
|---|---|---|
| 0 | Geometric baseline (Lee family, reduced torque law) | Reference; C.1 collection |
| 1 | Pure INDI | Reactive |
| 2 | Geometric + learned residual (Neural-Swarm line) | Predictive feedforward |
| 3 | FBL + flatness-preserving residual | Predictive; table only until author code |

- Hybrid / NA-INDI / LINDI (Cobo): **literature only** (`ch2` §2.7). Not a
  Strategy section, not in Table 4.1, not in RQ1.
- Chee NMPC, ProxFly, Li NDP-NMPC: related work on injection / residual-RL.
  Not compared strategies unless a later explicit decision adds them.
- Fairness: each law is tuned on the hardware gate; **those** gains are
  frozen before comparison and training-collection flights. Not one
  shared gain set for every family.
- Residual \(a_{\mathrm{res}}\) is **subtracted**. Sign error doubles the
  disturbance and is invisible on a single drone.
- C.1 collection: geometric only (`ctrl_mode=0`, `rnn.en=0`).
- Nothing in Ch.1–5 is a measured flight result. Do not write as if C.0
  or C.1 is done.

## 2. Voice

- One idea per sentence when a sentence contains two “which”, two
  “rather than”, or a comma splice of the form “narrower than it sounds,
  mass, inertia…”.
- Default rhythm: subject–verb–object. Put the caveat in the next
  sentence.
- Keep hedges that carry truth: “in our reading”, abstract vs body,
  “the paper does not claim”, “not a Crazyflie”.
- Cut stall phrases: “this is recorded here because”, “it may appear”,
  “purely to avoid repeating”, repeating “argument rather than a
  catalogue” in every chapter opener (once in Ch.2 is enough).
- Extra pages are allowed. Deleting a qualifier to sound punchy is not.
- Do not replace technical terms with vague synonyms (wrench, residual,
  feedforward, incremental, flatness).

## 3. Citations and numbers

- The `.tex` is the source of truth for claims. Verify against that
  paper’s PDF, never against a summary, ledger, or this file.
- A number stays glued to: which axis, which training set, which
  platform, abstract vs body, sim vs hardware, onboard vs off-board.
- Locked examples (do not “simplify” into each other):
  - Neural-Swarm: 9.4 cm baseline vs 2.4 cm (4-vehicle-trained) vs
    2.7 cm (2-vehicle-trained); swaps 0.20–0.25 m.
  - Neural-Swarm2: 16 robots / 24 cm is not the same experiment as the
    abstract “threefold”; body 15 cm → 10 cm on the 16-robot flight.
  - Smith: 36 % is 3D tracking; 56 % is vertical. Not “36 % lateral”.
  - Cobo: ~40 % no-payload vs geometric; 19–24 % with payload. Not 40 %
    on the payload case.
  - Hsieh 2026 FBL: 31 % abstract-only vs body hardware 24 % / 29 % splits.
  - Chee: off-board NMPC on Crazyflie 2.1.
  - Gielis: load-stand sufferer, not free flight.
  - Lee: CDC 2010 vs arXiv v4 companion are different documents; firmware
    torque law is reduced + integral, not the full CDC law.
  - Smeur: indoor 10 m/s jet and outdoor GNSS are two experiments.
- Log any wording that touches a prior-work claim in
  `docs/thesis/CITATION_EDIT_LOG.md`.
- Next literature audit = `git diff citation-audit-2026-08-27` plus any
  new cite key or new number on an old key. Do not re-audit unchanged
  sentences.

## 4. Structure

- A section may have subsections only if there are at least two.
- Name a strategy in words at first use in a chapter; do not lead with
  A1/B2 before Ch.5 has defined the library.
- Chapter opener: one short paragraph on what the chapter does, then
  content. No second methodological sermon.
- Keep Implemented / Reference split in Ch.4.

## 5. When adding or rewriting a section

1. Does the compared set change? If yes, update together: Ch.1 RQs and
   contributions, Table 4.1, Ch.4 summary, Ch.5 protocol.
2. New number → same sentence carries the qualifier (axis / platform /
   abstract-vs-body).
3. Rebuild standalone + full PDF. Exit 0, 0 undefined citations.
4. One log line in `CITATION_EDIT_LOG.md`.
5. Do not mark flights as completed from simulation.

## 6. Agent constraints

- Do not edit `references.bib` on a style pass.
- Do not delete §2.7 to “simplify scope”.
- Do not add Strategy 4 / hybrid as a campaign row.
- Do not push the thesis or the citation-audit tag unless asked.

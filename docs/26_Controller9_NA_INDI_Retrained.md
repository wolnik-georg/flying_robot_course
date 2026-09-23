# 26 — NA-INDI Retrained for Downwash (superseded, never built)

> **⚠️ Number collision, resolved 2026-09-23.** This document's own title used to be
> "`controller=9`: NA-INDI Retrained for Downwash" — but per `docs/34_NA_INDI_Decision_Memo.md`'s
> 2026-09-22 banner, the whole NA-INDI/hybrid track this document describes is **out of the
> compared set** (thesis strategies are 0–3 only) and was **never implemented** — no enum value,
> no slot patch, nothing built, exactly as this file always said. Because it was purely a
> reservation on paper and never coded, `stabilizer.controller=9` was reused on 2026-09-23 for
> something unrelated: a literal C port of the supervisor's own INDI implementation
> (`controller_omar_indi.c`, see `docs/41_Pure_INDI_Implementation_Comparison.md` §8 and
> `firmware_app/host/LOCAL_MODIFICATIONS.md`). **If this document is ever revived, it needs a
> different enum value** (`controller=10` or later) — `=9` is taken. Kept below for its
> architecture reasoning (8 vs 9 own-state-vs-neighbour-state question), not as a live plan.

**Status: superseded (`docs/34`, 2026-09-22) and never built. Kept for its reasoning only, not
as an active plan or a claim on the `controller=9` slot.**

---

## The question this exists to answer

> **Does the learned component of a neural-augmented INDI controller actually need neighbour
> state, or is own-state information enough to compensate inter-vehicle downwash?**

Nobody knows the answer in advance, and it sits at the centre of this thesis. Strategy 2
(Neural-Swarm2) assumes neighbour state is necessary and builds a permutation-invariant network
around relative position and velocity. NA-INDI as published assumes nothing about neighbours at
all — its network sees only the vehicle's own state. Both compensate *a* residual; they differ
in what information they are given to do it with.

Answering this cleanly requires giving NA-INDI's architecture its **best shot at our task**.

---

## Why this is needed, and why `controller=8` cannot answer it

`controller=8` is a faithful port of Cobo-Briesewitz's NA-INDI, carrying **their own real trained
weights**, extracted from their compiled `nn.c`. Those weights were trained on their
**single-vehicle payload-swing** experiment — a different disturbance, a different task.

Evaluating `controller=8` on downwash and reporting that it underperforms would be a **weak
result bordering on a straw man**: the method is being judged on a problem it was never trained
for. The honest comparison retrains the same architecture on *our* data and then asks how it does.

`controller=8` must therefore stay **byte-untouched** as the faithful reproduction and
citation-check. The retrained variant needs its own slot. That is `controller=9`.

---

## What `controller=9` is

**Identical to `controller=8` in every respect except the weights**, which are retrained on this
project's C.1 downwash dataset:

| | `controller=8` (reference) | `controller=9` (retrained) |
|---|---|---|
| Control law | Cobo-Briesewitz INDI, `use_nn` enabled | **identical** |
| Network architecture | 19→24→24→24→6, LeakyReLU (0.01) | **identical** |
| Input vector | 19-dim own-state: R columns (6), EKF accel (3), velocity (3), gyro (3), motor PWM ratios (4) | **identical** |
| Output | 6: force residual (3) + torque residual (3) | **identical** |
| Normalisation | min-max to [−1,1], `X_MINS`/`X_MAXS` | identical form, **re-derived from our data** |
| **Weights** | **theirs** (payload-swing) | **ours** (C.1 downwash) |
| Purpose | faithful reproduction, citation check | answers the own-state-vs-neighbour-state question |

Holding *everything* else constant is the entire point: any performance difference between 8 and
9 is attributable to training data alone, and any difference between 9 and Strategy 2 is
attributable to **input space** — own-state versus neighbour-aware — rather than to architecture,
base controller or tuning.

---

## What it can and cannot show

**Can:**
- Give NA-INDI's architecture a fair evaluation on the task this thesis actually studies.
- Isolate *training data* from *architecture* as explanatory variables (8 vs 9).
- Isolate *input space* from everything else (9 vs Strategy 2 / Strategy 4a).
- Potentially act as a **fast residual estimator**: the network's output is instantaneous, while
  INDI's own measurement is delayed by its filters (`fc_bw`, `res_fc`). A well-trained own-state
  network could plausibly reduce that lag even without geometric information.

**Cannot:**
- **Anticipate from geometry.** It sees a disturbance only once that disturbance has already
  perturbed its own state. Neural-Swarm2 knows "a neighbour is 30 cm above, expect downwash" and
  can act before the vehicle is displaced.
- **Disambiguate causes.** Downwash from a neighbour and any other disturbance with a similar
  own-state signature are indistinguishable to it.

⚠️ **The second list is a set of results to report, not reasons to skip the experiment.**
Quantifying *how much* is lost by omitting neighbour state is precisely the contribution.

---

## Cost

| Item | Cost | Notes |
|---|---|---|
| **Flight data** | **free** | The C.1 flights serve both. Inputs (`motor.m1-4` PWM ratios, `gyro.*`, accel, velocity, R) and the `a_res` label are already logged. ⚠️ **Verify `motor.m1-4` PWM ratios are in the current uSD config** — they are distinct from `motor.m1_rpm` (DShot RPM), and only the former matches `motorsGetRatio()`. |
| **Training pipeline** | ~1 day | A plain MLP with min-max normalisation — substantially simpler than the Deep Sets net. Separate from `tools/residual/` (different architecture, I/O and normalisation); do **not** try to generalise one trainer to serve both. |
| **Weight deployment** | the real friction | Weights are **compile-time Rust constants** (`pub const W0: [f32; 19*24]` in `naindi_hybrid_weights.rs`), not runtime-uploadable like the Neural-Swarm2 net. Each retrain needs codegen → rebuild → reflash rather than a radio upload. |
| **Slot patch** | ~half a day | `ControllerTypeOot4`/`CONFIG_CONTROLLER_OOT4`, following exactly the pattern used for `OOT2`/`OOT3` (`controller.h` enum + declarations, `controller.c` dispatch row, `Kconfig`, `app-config`, `bindings/cffirmware.i` — both SWIG blocks, which are *not* byte-identical). Own Rust module, own state, no sharing with 7 or 8. |
| **Scope risk** | real | A fourth strategy implementation on a thesis that already has several in flight. |

**Deployment note:** if iteration proves too slow, a runtime weight-upload path for
`controller=9` could be added (mirroring `upload_residual_weights.py`). Do **not** retrofit that
onto `controller=8` — it would compromise its value as the untouched reference.

---

## Sequencing

**After C.1 and C.2.** Deliberately not before:

1. By then the dataset exists, so this costs **no extra flights**.
2. Strategy 2's result will be known. If the neighbour-aware net gives a large win over plain
   INDI, "is neighbour state necessary?" becomes a much sharper and more interesting question —
   and if it gives only a small win, that changes how much effort this deserves.
3. It keeps the critical path (C.1 → C.2 → C.3 → C.4) clear.

**Prerequisite:** `controller=8` must have flown first. A retrained variant of a controller that
has never flown on hardware would confound two unknowns at once.

---

## How it enters the comparison

Once built, `controller=9` becomes a row in
[`docs/24_Downwash_Compensation_Comparison.md`](24_Downwash_Compensation_Comparison.md) under
the same protocol as every other controller. The three-way reading that matters:

| Comparison | Isolates |
|---|---|
| `c=8` vs `c=9` | **training data** (architecture held constant) |
| `c=9` vs Strategy 4a (`c=6, m=3, rnn.en=1`) | **input space** — own-state vs neighbour-aware |
| `c=9` vs `c=6, m=3` (plain INDI) | whether the learned component helps **at all** without neighbour state |

That third row is the one that most directly answers the headline question, and it is worth
stating in the thesis as an explicit hypothesis test rather than as an incidental observation.

---

## Naming and invariants

- `controller=7` (their plain INDI) and `controller=8` (their NA-INDI) are **frozen reproductions**.
  Nothing in this work may modify either. Their value is precisely that they are untouched.
- `controller=9` is **not** "our improved NA-INDI". It is *their* method, retrained for *our*
  task — and that framing should survive into the thesis text, where it is a statement about
  fairness of comparison rather than a claim of contribution to their method.

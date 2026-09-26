# 40 — C.2 Neural-Swarm2 residual pipeline: end-to-end validation plan (with real 2026-09-21 data)

**Status:** **executed 2026-09-21** (Stages A–E desk/SIL on **5-file / 4-usable** subset). **Extended
2026-09-26 (desk):** full **18-file** training bank train + **17-fold LOO**, Stage C **pass** after
`residual_nn` host rebuild, **first SIL inference with full-bank weights** — see § **Full bank —
2026-09-26** (historical) and § **Extension — LOO, Stage C, SIL — 2026-09-26** below. Artifacts:
`experiments/analysis/out/c2_e2e_2026-09-21/` (`c2_validation_report.json`,
`C2_VALIDATION_REPORT.md`, `loo_weights/`). **2026-09-26:** `experiments/analysis/out/c2_e2e_2026-09-26/`.
Docs updated: `13`, `07` C.2, `CLAUDE.md`,
`tools/residual/README.md`. **Stage E re-run 2026-09-21 (evening): pass** after SIL plumbing fix
(`docs/38`, `crazyswarm2` `crazyflie_sim`) — see
`experiments/sim_validation/c2_e2e_stage_e.json` (`meaningful_flight: true`, 5k+ rows each arm).
**Caveat:** in sim, `{name}/state` and `{name}/pose` both publish **ground-truth physics** (Kalman
externalize is computed but unused for ROS); the shared 150 mm gate is therefore **vacuously true**
in sim but the **closed-loop residual dry run is real** (hover, logs, no divergence). Original plan
+ agent prompt below kept for audit trail.

## Correction to an earlier assumption (important context)

Prior guidance in this project (including my own earlier feedback) said `train.py` was
broken ("still imports removed `DeepSets`"). **That is stale.** I read `train.py` directly and
ran it myself:

```
python3 flying_drone_stack/tools/residual/train.py --synthetic --epochs 20 -o /tmp/synth_test.npz
# ... fold check: max |trained - exported| = 1.25e-07 m/s^2 -- PASSES

python3 flying_drone_stack/tools/residual/train.py \
  experiments/logs/c1_2026-09-21_merged/A1_2026-09-21_12-51-16/*_merged_usd.csv \
  experiments/logs/c1_2026-09-21_merged/A3_2026-09-21_13-00-57/*_merged_usd.csv \
  experiments/logs/c1_2026-09-21_merged/A3_2026-09-21_13-02-56/*_merged_usd.csv \
  experiments/logs/c1_2026-09-21_merged/A3_2026-09-21_13-04-34/*_merged_usd.csv \
  experiments/logs/c1_2026-09-21_merged/A1_2026-09-21_13-25-10/*_merged_usd.csv \
  --epochs 40 -o /tmp/real_test.npz
# 53644 samples, val RMSE 0.3652 m/s^2 vs baseline 1.2384 m/s^2 -> 70.5% reduction
# fold check: max |trained - exported| = 1.12e-06 m/s^2 -- PASSES
```

`train.py` was rewritten 2026-09-17 for the Neural-Swarm2 architecture and **already runs
correctly end to end on real gathered data**, including its own internal correctness gate (the
fold check). **Do not re-litigate "does it run" — it does.** The actual work is validating
*whether the result is trustworthy*, which nothing has checked yet. That is the real gap, and
it is a genuine one — stated by the codebase itself, not inferred:

> `test_pipeline.py`'s own docstring: *"this file no longer exercises `dataset.py`... The loader
> tests that used to live here (peer-minus-own convention, both drones as ego, dropping
> `a_res == 0` samples) are NOT covered anywhere right now. That is a real gap, not an omission
> by choice."*

So: `model.py` (the network) is verified against the compiled Rust firmware, on **synthetic**
tensors (`test_pipeline.py`, `test_residual_nn.py`). `dataset.py` (the loader) is verified by
code review and comments, not by a test, and never against real merged CSVs feeding the
firmware-verified path. **Closing exactly that gap, with the real data now in hand, is the
highest-value thing to do before trusting any trained weights.**

## What "end to end, A to Z" actually means here — the full chain

```
merged uSD CSV (real flight)
   │  dataset.build()                      <- loader: UNTESTED against firmware, by the codebase's
   ▼                                            own admission
(rel, mask, ground, y) tensors
   │  train.py (NeuralSwarm2, torch)        <- verified: runs, fold-check passes
   ▼
trained weights (in-memory)
   │  fold_normalisation + flatten()        <- verified numerically (fold check, every run)
   ▼
19297-float exported vector (.npz)
   │  model.firmware_forward() (NumPy)      <- verified against compiled Rust on SYNTHETIC data
   │  ≡ residual_nn.rs (compiled firmware)     (test_pipeline.py, test_residual_nn.py) — NOT yet
   ▼                                            on real data
predicted a_res_z
   │  upload_residual_weights.py + rnn.en=1 <- exists, never exercised with real-trained weights
   ▼
onboard inference (SIL or hardware)
```

Five stages below (A–E) walk this chain with the real 2026-09-21 data, in order, each with a
concrete pass/fail bar. Do not skip a stage because an earlier one looked good — a plausible
RMSE at stage B does not imply the loader is wired correctly (stage C), and stage C passing does
not imply the result means anything physically (stage D).

## Stage A — Data audit (few minutes, prerequisite for everything else)

The manifest has 8 flights; only 5 are C.1 training-eligible (A8×3 excluded — neighbour-gated
out). Of those 5, running `dataset.build` on them today showed:

| Flight | Rows kept | Note |
|---|---:|---|
| A1 `12-51-16` (dz 0.75) | 7554 | OK |
| A3 `13-00-57` | 17691 | OK |
| A3 `13-02-56` | 17897 | OK |
| A3 `13-04-34` | 10502 | OK |
| A1 `13-25-10` (dz 0.20) | **0** | `a_res` reads exactly zero — no RPM source that flight |

**Action:** confirm this is still true, and be explicit in every downstream report that the
real usable dataset is **4 flights, not 5** (2× A1-shape at only one dz value, 3× A3). This
matters enormously for Stage B's honest generalization check below — don't let "53644 samples"
imply more independent information than 4 flights actually contain.

## Stage B — Train with an honest generalization check, not just a random split

`train.py`'s current validation split (`dataset.split`) is a **contiguous block split within
the same pooled flights** — it tells you whether the model memorized part of the same
trajectories it trained on, not whether it generalizes to a flight it never saw. With only 4
usable flights and heavy 500 Hz autocorrelation within each, that is a materially weaker claim
than "70.5% reduction" sounds like. Do a proper **leave-one-flight-out cross-validation**:

1. For each of the 4 usable flights, train on the other 3, evaluate (via `eval_model.py`, which
   already supports per-file evaluation with no retraining) on the held-out one.
2. Report per-fold RMSE vs. the predict-zero baseline **for that held-out flight specifically**
   — not the pooled number. A fold where the held-out flight's baseline RMSE is already tiny
   (little real interaction happened) will show a misleadingly small "reduction %"; report raw
   RMSE alongside the percentage so this isn't hidden.
3. If any fold's held-out RMSE is *worse* than that flight's own predict-zero baseline, that is
   the honest headline finding — report it plainly, do not average it away against the folds
   that look good.
4. Also fold in **A3-vs-A1 generalization specifically**: train on all 3 A3 flights only,
   evaluate on the single usable A1 flight (`12-51-16`), and vice versa. A3 and A1 are
   different motion patterns (A3: one drone translating past a hovering one; A1: pure static
   stack) — if the model trained on one generalizes poorly to the other, that's a scoping fact
   for the thesis (what the model has actually learned to interpolate vs. extrapolate), not a
   bug to fix.

**Bar:** every fold beats its own held-out baseline by a real margin, and the A3→A1 /
A1→A3 cross-scenario numbers are reported honestly even if they're weak. A model that only
"works" on a random split of the same 4 flights it trained on is not evidence it works on a 5th
flight, and should not be reported as if it were.

## Stage C — Close the stated loader↔firmware gap, with real data (the actual new work)

This is genuinely unbuilt, per `test_pipeline.py`'s own docstring. Build a script (new file,
e.g. `flying_drone_stack/tools/residual/test_real_data_pipeline.py`) that:

1. Loads real samples via `dataset.build()` on one or two of the real merged CSVs (a few
   hundred rows is enough — this is a correctness check, not a training run).
2. Exports a small set of real trained weights (from Stage B, any one fold is fine).
3. Evaluates those same real `(rel, mask, ground)` rows through **two independent paths**:
   - `model.firmware_forward()` (NumPy) — the same function `train.py` already trusts.
   - The **actual compiled Rust `residual_nn.rs`** via the `cffirmware` SIL bindings and the
     real CRTP-style weight-upload protocol, exactly as `test_residual_nn.py` already does for
     synthetic weights (needs the SIL build with `--features residual_nn` — see that file's
     header for the exact build command).
4. Assert the two predictions agree to the same tolerance `test_pipeline.py` already uses for
   its synthetic checks. This is the check that specifically catches "the loader computes
   `rel` with the wrong sign / wrong drone as ego / doesn't drop zero-`a_res` rows correctly" —
   none of which would show up as a training crash, only as a silently wrong dataset.
5. While doing this, explicitly verify the **peer-minus-own sign convention** on a real sample
   you can reason about by hand: e.g. take one row where drone A (bottom) is directly below
   drone B (top) mid-flight, print `rel` for ego=A, and confirm `rel[...,2]` (dz, peer − own,
   world frame) is **positive** (peer is above → higher z). Getting this backwards is exactly
   the kind of bug `model.py`'s own docstring warns trains "a mirrored model that would push
   the drone *into* the disturbance" — worth one explicit manual sanity check, not just trusting
   the assertion above to catch it.

**Bar:** numeric agreement within the existing test suite's tolerance, on real data, plus the
manual sign sanity check passing by inspection. If this fails, **stop here** — nothing past
this point (Stage D, E, or any thesis claim about the trained model) is trustworthy until it's
fixed, because it means the network the firmware would run is not the network you evaluated in
Python.

## Stage D — Physical plausibility, not just RMSE

A small RMSE with only 4 flights of mostly-correlated data can come from fitting incidental
structure rather than the actual downwash physics. Use `eval_model.py`'s existing
`binned_error` machinery (already bins by horizontal offset and dz) plus a direct read of the
learned function's shape:

1. Using the real trained weights from Stage B, sweep `model.firmware_forward()` over a
   synthetic grid of `(dz, own_z)` with a neighbour held directly overhead (`dx=dy=dvx=0`,
   varying `dz` from 0 to 1.2 m) and confirm the predicted `a_res_z` is **larger in magnitude
   close underneath a neighbour and decays as `dz` grows** — the qualitative signature every
   downwash reference (including this project's own crude synthetic test in `test_pipeline.py`)
   expects. If the learned curve is flat, noisy, or non-monotonic, say so plainly — with 4
   flights that is a real possibility, not something to paper over.
2. Cross-check that shape against the **real** binned error report from `eval_model.py` on the
   held-out folds from Stage B — does the model's error *pattern* (not just aggregate RMSE)
   look structured by dz/offset, or does it look like noise? A model that's actually learning
   downwash should show smaller residual error near the dz values it saw most often (A3's dz
   0.25) and larger error extrapolating to A1's dz 0.75, which never appeared in A3 training —
   report this as a concrete, checkable claim, not a vague caveat.

**Bar:** the sign and rough shape of the learned function match known downwash physics
(stronger closer, weaker farther, roughly monotonic in dz for a fixed lateral offset). This is
a sanity check, not a publication-grade validation — say exactly that in the writeup.

## Stage E — SIL closed-loop dry run (still desk-only, no hardware)

The project has flown a residual-model **dry run protocol before** (phase 1/3/4,
`experiments/analysis/analyse_residual_dryrun.py`), but that was on hardware with different
weights. The desk-only equivalent, doable now:

1. Build the CS2 SIL bindings with `--features residual_nn` (same build as Stage C).
2. Run one already-validated CS2 sim scenario (e.g. A3, since that's what most of the real
   training data came from) with the Stage B weights uploaded and `rnn.en=1`, geometric
   controller, and watch for: output clamp saturating on a large fraction of ticks (a fault
   signature per `train.py`'s own clamp warning), any closed-loop divergence, or an obviously
   degenerate control response.
3. Compare against the same scenario with `rnn.en=0` (no compensation) as the control — the
   point is not "does it look better" (this project's own dry-run doc already warns that a
   small residual with the network active is also consistent with "stable but wrong"), it's
   **"is it safe to fly without immediately diverging."**

**Bar:** no divergence, no majority-clamp-saturation, over a full sim run of the scenario. This
is a go/no-fly gate for a *future* real dry run, not a claim the model is good.

**2026-09-21 execution note:** Stage E **passed** after `docs/38` (`c2_e2e_stage_e.json`). The
sim fix republishes **ground truth** on both `{name}/state` and `{name}/pose`, so the EKF-vs-mocap
check does **not** validate SIL estimator behavior — only that formation scripts can run a full
predict/compensate loop without aborting at spawn. Hardware still needs the real gate.

## What to write up regardless of outcome

Whatever Stages A–E find — including "the model doesn't generalize past A3" or "the loader had
a sign bug we just fixed" — **that is the real result** and belongs in
`docs/13_Residual_Learning.md` and the C.2 status in `docs/07_Thesis_Progress_Checklist.md`,
with the honest caveat that this is validation on 4 flights, heavily skewed toward one scenario
(A3) and one dz value for A1. A well-documented negative or partial result closes C.2 to the
point where C.3/C.4 can proceed with clear eyes about what the residual model actually knows;
a glossed-over positive result does not.

---

## Prompt for Cursor agent

Copy everything below into a new Cursor agent session.

---

**Context.** `flying_robot_course` is a Crazyflie multi-drone thesis project. **Historical note
(2026-09-21 prompt below):** at first E2E, `manifest_2026-09-21_c1.json` had 8 merges (5 C.1
training + 3 A8 QA — see `training_eligible_crosswalk.json`). **As of 2026-09-23:** **24**
training merges (`manifest_2026-09-23_c1.json` + 21 Sep crosswalk). Extend validation to the full
bank after **A4** lands. Original 21 Sep wording: The residual-force
model (`flying_drone_stack/tools/residual/{dataset,model,train,eval_model}.py`, firmware
counterpart `firmware_app/src/residual_nn.rs`) is meant to learn `a_res_z` (measured downwash
disturbance) from relative multi-drone state. **Important, verified fact, do not re-derive:**
`train.py` already runs correctly end to end on this real data (I ran it myself — 53644
samples, 70.5% RMSE reduction vs. predict-zero baseline, internal fold-check passes). The task
is NOT "make it run" — it already does. The task is **rigorous end-to-end validation of
whether the result is trustworthy**, because a real, self-acknowledged gap exists: per
`test_pipeline.py`'s own docstring, the model (`model.py`) is numerically verified against the
compiled Rust firmware only on **synthetic** tensors — the real data loader (`dataset.py`) has
never been exercised through that same firmware-verified path. Full plan, rationale, and exact
pass/fail bars for each stage: `docs/40_C2_Residual_Pipeline_E2E_Validation_Plan.md` in this
repo — **read it in full before doing anything.** Execute Stages A through E in order; do not
skip a stage because an earlier one looked good, and do not average away a bad fold/result to
make the summary look better — a documented weakness is a valid, useful outcome here.

**Stage A — data audit.** Confirm which of the 5 C.1-eligible merges actually contribute
usable rows via `dataset.build()`. Verified today: `A1_2026-09-21_13-25-10` (dz 0.20) merges
cleanly but contributes **0 rows** (all-zero `a_res`, no RPM source that flight). Confirm this
is still the case and state plainly, in every later report, that the real usable dataset is
**4 flights** (A1 dz 0.75, A3×3), not 5.

**Stage B — leave-one-flight-out cross-validation.** Using `train.py` (already working, no
changes needed to it for this) and `eval_model.py` (already supports per-file evaluation with
no retraining), for each of the 4 usable flights: train on the other 3, evaluate on the
held-out one via `eval_model.py`, and report that flight's own RMSE and its own predict-zero
baseline (not the pooled/random-split numbers `train.py` prints by default — those only show
memorization within the same 4 flights, not generalization). Additionally run the specific
A3→A1 and A1→A3 cross-scenario folds (train on all A3, test on the A1 flight, and the reverse)
and report those numbers even if they're weak. If any fold's held-out RMSE is worse than that
flight's own baseline, report it as the headline finding for that fold, not as an outlier to
discard.

**Stage C — close the loader↔firmware gap with real data (the core new work).** Write a new
script, `flying_drone_stack/tools/residual/test_real_data_pipeline.py`, that: loads a few
hundred real rows via `dataset.build()` from one or two of the real merged CSVs; takes one set
of real trained weights from Stage B; evaluates those same real `(rel, mask, ground)` rows
through both `model.firmware_forward()` (NumPy) and the actual compiled Rust `residual_nn.rs`
via the `cffirmware` SIL bindings and the real weight-upload protocol (build with
`DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" cargo build --release --target
x86_64-unknown-linux-gnu --features residual_nn` per `test_pipeline.py`'s header, then `cd
~/Desktop/crazyflie-firmware && make bindings_python`); assert numeric agreement within the
same tolerance `test_pipeline.py` already uses for its synthetic checks. Also do one manual
sanity check by hand: pick a real row where one drone is directly below the other, print
`rel` for the bottom drone as ego, and confirm the z-component of `rel` (peer − own, world
frame) is positive (peer above → higher z) — this is the sign convention the whole model
depends on getting right, and `model.py`'s own docstring warns a sign flip trains a model that
pushes the drone *into* the disturbance rather than away from it. **If this stage fails, stop
and report it — do not proceed to Stage D/E or write up any result from the trained weights
until this is fixed**, since a mismatch here means the firmware would run a different function
than the one evaluated in Python.

**Stage D — physical plausibility.** Using the real trained weights, sweep
`model.firmware_forward()` over a synthetic grid with a neighbour held directly overhead
(varying `dz` 0 → 1.2 m, `dx=dy=dvx=0`) and confirm the predicted `a_res_z` magnitude is larger
close underneath a neighbour and decays as `dz` grows (the qualitative downwash signature).
Cross-check against `eval_model.py`'s existing `binned_error` output on the Stage B held-out
folds — does prediction error look structured by dz/offset (smaller near A3's common dz ~0.25,
larger extrapolating to A1's dz 0.75) or unstructured/noisy? Report the actual shape found,
including "it's flat/noisy" if that's what the 4-flight dataset actually supports — don't
overstate a physically-plausible-looking curve as proof if the underlying data is this thin.

**Stage E — SIL closed-loop dry run, no hardware needed.** Build the CS2 SIL bindings with
`--features residual_nn` (same build as Stage C). Run one already-validated CS2 sim scenario
(A3, since that's most of the real training data) with the Stage B weights uploaded, `rnn.en=1`,
geometric controller, and check for: the output clamp (8 m/s²) saturating on a large fraction of
ticks, any closed-loop divergence, or an obviously degenerate control response, compared against
the same scenario with `rnn.en=0` as a control run. This is a go/no-fly gate for a future
hardware dry run, not a quality claim — say so explicitly in the report.

**Final step — write up honestly, whatever was found.** Update
`docs/13_Residual_Learning.md` and the C.2 entry in `docs/07_Thesis_Progress_Checklist.md`
with the actual Stage A–E results, explicitly caveated as validation on **4 flights**, skewed
toward A3 and a single A1 dz value. If Stage C found and fixed a real bug, document what it was
and why it wouldn't have shown up as a crash. If any fold in Stage B or D looked weak, report
that plainly — a clear-eyed partial result is more useful to the thesis than an inflated one,
and is exactly what unblocks C.3/C.4 with honest expectations about what this model knows.

**Also fix stale docs while you're in there.** `flying_drone_stack/CLAUDE.md`'s
"Residual-learning quick reference" section (~line 411) currently says *"`train.py` does NOT
run... Blocks C.2, not C.1"* — that is disproven by this plan's own Stage A/B runs (`train.py`
already works end to end; the real gap was the untested loader↔firmware path, not a broken
script). Update that section to describe the actual current status once Stages A–E are done,
and check `flying_drone_stack/tools/residual/README.md` (referenced from that same section) for
the same stale claim — fix it there too if present. Leaving this uncorrected means the next
person reads a confidently wrong blocker and re-derives this same investigation from scratch.

Do not modify `experiments/logs/c1_2026-09-21_merged/` or its manifest — this validation reads
that data, it does not change it. Do not fly anything — every stage here is desk/SIL-only.

---

## Full bank — 2026-09-26 (desk, lab blocked on mocap)

**Inputs:** all **24** C.1 training merge paths (19× `manifest_2026-09-23_c1.json` +
5× `training_eligible_crosswalk.json` on 21 Sep). **Read-only** on merged CSVs.

### Stage A — row audit

Artifact: `experiments/analysis/out/c2_e2e_2026-09-26/stage_a_full_bank.json`

| Result | Detail |
|--------|--------|
| **18** files contribute rows | **181 489** training samples |
| **6× C5** solo | skipped by `dataset.build` (no neighbour — expected) |
| **A1 `13-25-10` (21 Sep)** | **0** kept rows (`a_res` zero — same as 2026-09-21 Stage A) |

### Stage B — single full-bank train (40 epochs)

```bash
python3 flying_drone_stack/tools/residual/train.py <all 24 paths> --epochs 40 \
  -o experiments/analysis/out/c2_e2e_2026-09-26/full_bank_40.npz
```

Log: `full_bank_train.log`. **Results (measured):**

| Metric | Value |
|--------|------:|
| Val RMSE | **0.2806 m/s²** |
| Predict-zero baseline | 1.2900 m/s² |
| Reduction vs zero | **78.2%** |
| Fold check max &#124;trained − exported&#124; | **1.17×10⁻⁶ m/s²** |

**Not re-run:** full **leave-one-flight-out** on 18 files (would be 18× train jobs) — prior 21 Sep
LOO remains the generalization reference until operator requests a long desk batch.

### Stage C — loader ↔ firmware on **23 Sep** data

```bash
python3 flying_drone_stack/tools/residual/test_real_data_pipeline.py \
  --weights experiments/analysis/out/c2_e2e_2026-09-26/full_bank_40.npz \
  --csv experiments/logs/c1_2026-09-23_merged/A3_2026-09-23_17-45-03/..._merged_usd.csv
```

| Check | Result |
|-------|--------|
| Sign sanity (peer above → `rel_z` > 0) | **PASS** |
| NumPy vs firmware on real rows | **BLOCKED** — `FwHarness.upload()` returned **0** on this desk
  (**2026-09-26**), including with the **2026-09-21 LOO weights** that passed Stage C when first run.
  Likely **host `cffirmware` / `residual_nn` build drift**, not the new weights (fold check passed).
  Rebuild per `test_pipeline.py` header before trusting a re-run. Log:
  `stage_c_23sep_A3.log`. |

Summary JSON: `experiments/analysis/out/c2_e2e_2026-09-26/c2_full_bank_report.json`.

**Stages D–E:** not re-executed on the full bank in this pass (21 Sep SIL Stage E remains the
closed-loop reference). After mocap returns, priority is lab regression — not SIL re-run.

---

## Extension — LOO, Stage C, SIL — 2026-09-26 (desk)

**Does not replace** § **Full bank — 2026-09-26** above (that section stays the record of the first
full-bank train and the initial Stage C failure).

### A.1 — 17-fold leave-one-flight-out (18-file bank)

Script: `experiments/analysis/run_c2_loo_18fold_2026_09_26.py`  
Artifacts: `experiments/analysis/out/c2_e2e_2026-09-26/loo_weights_18fold/`, `loo_18fold_results.json`

| Scope | Detail |
|--------|--------|
| Training-eligible paths | **24** manifest entries → **18** files with `dataset.build` rows (6× C5 solo excluded; **A1 `13-25-10`** → **0** rows) |
| LOO folds executed | **17** — no fold for `A1_2026-09-21_13-25-10` (nothing to hold out) |
| A2 @ dz 0.30 | **Kept** despite `top_align_rms_high` (~28 cm on cf_second): scenario is circle r=0.75, h=1.0 — expected Lee-on-circle tracking under downwash, not a merge defect |

**Per-flight held-out RMSE (m/s²):**

| Flight | RMSE | vs predict-zero |
|--------|-----:|----------------:|
| A1_12-51-16 | 0.797 | 55% reduction |
| A1_17-17-26 | 0.441 | 75% |
| A1_17-18-48 | 0.341 | 81% |
| A1_17-36-06 | 0.381 | 82% |
| A1_17-37-26 | 0.382 | 81% |
| A2_19-21-05 | 0.499 | 56% |
| A2_19-27-03 | 0.432 | 60% |
| A3_13-00-57 | 0.184 | 76% |
| A3_13-02-56 | 0.187 | 75% |
| A3_13-04-34 | 0.251 | 66% |
| A3_17-45-03 | 0.174 | 74% |
| A3_17-46-43 | 0.164 | 75% |
| A3_17-54-32 | 0.161 | 80% |
| A3_17-57-32 | 0.151 | 80% |
| A7_19-11-19 | 0.453 | 71% |
| A7_19-12-38 | 0.413 | 74% |
| A7_19-13-55 | 0.431 | 75% |

Weakest hold-outs: **A1_12-51-16** (single 21 Sep A1 dz 0.75 flight); **A2** pair and **A7** triplet
generalise moderately vs A3-heavy training mass.

### A.2 — Stage C (23 Sep A3, full-bank weights) — **PASS**

Root cause of the § **Full bank** failure: host `cffirmware` built **without** `--features residual_nn`
so `rnn_service()` was compiled out and `g_rnn_ready` never set.

Rebuild (same as `test_pipeline.py` header):

```bash
cd flying_drone_stack/firmware_app
DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" cargo build --release \
  --target x86_64-unknown-linux-gnu --features residual_nn
cd ~/Desktop/crazyflie-firmware && make bindings_python
```

Re-run log: `experiments/analysis/out/c2_e2e_2026-09-26/stage_c_23sep_A3_rerun.log`

| Path | Max &#124;NumPy − compiled&#124; |
|------|--------------------------------:|
| Positions, peer dv forced 0 in NumPy | **5.80×10⁻⁷ m/s²** |
| Differenced peer velocity (100 ms) | **2.06×10⁻⁶ m/s²** |
| Overall | **PASS** (tol 2×10⁻⁴) |

### A.3 — Live CS2 SIL inference (full-bank weights, predict-only)

First run with **`full_bank_40.npz`** (not 21-Sep LOO subset):  
`experiments/analysis/c2_fullbank_sim_predict.sh` → `server_c2_fullbank_predict.yaml`  
(`rnn.en=0`, `backend=neuralswarm`, A3 dz 0.30).

Summary: `experiments/sim_validation/c2_fullbank_sim_inference.json` (**5206** log rows).

| Check | Result |
|--------|--------|
| Weights upload + `rnn.ready` | Log: 19297 weights, val RMSE 0.2806, `rnn.en=0` |
| `rnn.pred_*` finite, non-NaN | ✅ |
| `rnn.pred_z` non-zero | RMS **~7.1 m/s²** (clamp **±8** active ~20% of samples — safety limiter, not missing network) |
| Corr(`a_res_z`, `rnn_pred_z`) on cf231_active | **0.16** (open-loop predict — not claiming closed-loop quality) |

**21 Sep Stage E** remains the closed-loop stability reference; this extension adds **trained-weight**
SIL predict logging on the **full bank** model.

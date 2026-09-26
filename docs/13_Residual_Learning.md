# Residual Learning — Onboard Inference and Offline Training

> ## ✅ Pipeline complete; **2026-09-26: full 18-file bank LOO + Stage C + SIL predict** — deploy trust still A4-limited
>
> **Revised 2026-09-21** after [`40_C2_Residual_Pipeline_E2E_Validation_Plan.md`](40_C2_Residual_Pipeline_E2E_Validation_Plan.md).
> Report: `experiments/analysis/out/c2_e2e_2026-09-21/` (`c2_validation_report.json`,
> `C2_VALIDATION_REPORT.md`).
>
> | Piece | State |
> |---|---|
> | Onboard network + weight upload | ✅ 19 checks passing |
> | `model.py` / `test_pipeline.py` | ✅ vs compiled firmware ~1e-6 m/s² (synthetic tensors) |
> | `dataset.py` + `train.py` on real merges | ✅ 53644 samples pooled; **4 usable flights** (`A1_13-25-10` → 0 rows) |
> | **`test_real_data_pipeline.py` (Stage C)** | ✅ Path A: positions, NumPy peer dv=0 → ~**6.6×10⁻⁷** m/s². Path B: measured `rel` velocity via two `oot_set_peer` samples (100 ms) → ~**6×10⁻⁶** m/s². See script docstring — Path A does not exercise \|dvx\| gate alone |
> | **LOO generalisation** | ✅ **17-fold** on 18-file bank (`docs/40` § Extension 2026-09-26); weakest: A1_12-51-16 hold-out; A3 hold-outs ~0.15–0.25 m/s² |
> | **Stage C on 23 Sep + full-bank weights** | ✅ **Pass** after host rebuild with `--features residual_nn` (~2×10⁻⁶ m/s²) |
> | **SIL predict, full-bank weights** | ✅ **First run** 5206 rows (`c2_fullbank_sim_inference.json`); 21 Sep Stage E still closed-loop reference |
> | **SIL closed-loop gate (Stage E, 21 Sep LOO weights)** | ✅ **Pass** — `c2_e2e_stage_e.json`; go/no-fly for **closed-loop stability**, not model quality |
> | Hardware open-loop `rnn.en=0` | ⬜ Not flown yet |
>
> **Honest takeaway:** Software chain is trustworthy enough to integrate cautiously; the **learned
> model is not trustworthy off the A3 training manifold** until **A4 ×4** is in the bank (see
> `next_flight_card.html`; **24 training merges** after 23 Sep). Collect under **geometric** (§6 banner rationale unchanged).
>
> **Where this sits in the Core Thesis Workflow** ([`07`](07_Thesis_Progress_Checklist.md)):
>
> | Step | This document's role |
> |---|---|
> | **C.0 Hardware Gate** | Closed enough for C.1; hardware open-loop `rnn.en=0` still unflown |
> | **C.1 Residual Data Collection** | **~24 merges banked** — **A4 (C-1/C-2) ×4** remains required (`docs/25`) |
> | **C.2 Train the Residual Model** | Full **18-file** bank trained + LOO 2026-09-26; **A4 ×4** still required before claiming deployable coverage |
> | **C.3 Integrate the Strategies** | Compared set 0–3; Strategies 2–3 consume this prediction; only Strategy 2 is wired |
> | **C.4 Systematic Comparison** | §4's logged prediction vs measurement is the evaluation |

**Last updated:** 26 September 2026 (full-bank C.2 extension; flash-resident weight variant — §3.1)

The **predictive strategies in the compared set (Strategy 2; Strategy 3 when FBL arrives)** need a
model that predicts the interaction force one vehicle is about to feel from the others. This
document is the engineering record of that shared foundation — what runs on the drone, how weights
get there, and how the prediction is evaluated. **NA-INDI / campaign hybrid is not in the compared
set** (Sep 2026 supervisor decision).

It is deliberately one layer down from
[`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md), which holds status. The
physics is in [`04_Unified_Residual_Wrench_Model.md`](04_Unified_Residual_Wrench_Model.md).

---

## 1. Why this is shared infrastructure, not one method's implementation

| Strategy (thesis 0–3) | Uses the residual network? |
|---|---|
| 0. Geometric baseline | No |
| 1. Pure INDI | No — reacts to the measured residual instead |
| 2. Geometric + NN | **Yes**, as a feedforward term on the geometric loop |
| 3. FBL + NN | **Yes** *(FBL code still with the authors — not implemented)* |
| *(stretch 5–7)* | RL / MPC — deferred, not core compared set |

Building the network once, with one weight format and one upload path, means those methods differ
in *how they use the prediction* rather than in how they obtain it. That is what makes the
comparison a comparison. Building it per-method would introduce differences that are implementation
artefacts and would be indistinguishable from real differences between the control laws.

---

## 2. What runs onboard

`flying_drone_stack/firmware_app/src/residual_nn.rs` — `no_std` Rust, cross-compiled to the same
Cortex-M4F binary that flies, and linked unchanged into the host simulator.

### Architecture: Neural-Swarm2, ported exactly

> **Rewritten 2026-09-14.** This section previously described a smaller custom deep-sets network
> (φ 6→16→16→8, ρ 8→16→16→3, 987 weights, 3-vector output, a distance cutoff with near-field
> rescaling). **That network no longer exists anywhere in the codebase.** `residual_nn.rs` is now
> a byte-faithful port of Neural-Swarm2's own `phi_Net`/`rho_Net`, taken from the vendored
> reference implementation rather than re-derived from the paper. Standing instruction: the
> network is never to be modified — not to fit RAM, not to simplify, not to tidy.

```
rho_input = phi_G(ground_term)
for each neighbour j:
    if |dx| < 0.2 and |dy| < 0.2 and |dvx| < 1.5:     # NOT a distance cutoff
        rho_input += phi_S(x_12)
Fa = (0, 0, rho_S(rho_input))                          # scalar — Z ONLY
```

`phi` runs on each neighbour separately and the results are summed, so the model is
permutation-invariant by construction and one weight set serves two drones or three — which
matters directly here, since the campaign flies both, and B1/B2 exist precisely to test whether
interactions superpose.

| | Dimensions |
|---|---|
| `phi_Net(d)` | d → 25 → 40 → 40 → **H=20**, ReLU ×3 then linear |
| `rho_Net` | 20 → 40 → 40 → 40 → **1**, ReLU ×3 then linear |
| Weight sets | **five**: `phi_S`, `phi_L`, `phi_G`, `rho_S`, `rho_L` |
| Weights + biases | **19297** floats (~77 KB) |
| Neighbours | up to 3 (a firmware buffer limit, not the reference's) |

Three differences from the old network carry real consequences, not just different numbers:

- **The output is a scalar, Z only.** `compute_Fa` returns `(0, 0, faz)`. **This architecture can
  only ever predict the vertical residual** — the x/y components of the measured `a_res` have no
  predictor in Strategy 2. That is a property of the method being compared, and it must be stated
  in the results rather than discovered there.
- **Ground effect is part of the model.** `phi_G` takes `[0 − own_z, −own_vx, −own_vy, −own_vz]`
  and is evaluated **unconditionally, every tick**, with no neighbour to gate it on. A lone drone
  with weights loaded therefore does **not** predict zero.
- **Neighbour inclusion is a three-term gate**, not a distance cutoff: `|dx| < 0.2`, `|dy| < 0.2`,
  `|dvx| < 1.5`. Asymmetric — it ignores `dz`, `dvy`, `dvz` — and replicated literally from the
  reference rather than regularised.

`phi_L`/`rho_L` are the reference's "large vehicle" path. This fleet is Crazyflies only, so they
are never evaluated; they still occupy their slots in the flat layout and are **exported as
zeros**, which states plainly that nothing was trained there.

**Unit conversion.** The network's output is a force in the reference's equivalent-grams unit.
This project's residual convention is an acceleration, so the firmware converts
grams → N → `/ mass` once, at the very end. The network is untouched by this; it is a boundary
conversion, the same class of necessary deviation as using this project's own mass in the NA-INDI
port.

**Normalisation is deliberately absent from the firmware.** Input scaling and mean-subtraction are
folded into the first layer's weights and biases at export time. That is exactly equivalent, and it
keeps the firmware from carrying a second set of constants that could drift out of step with the
trained model. There are now **two** first layers to fold into — `phi_S`'s (6 inputs) and
`phi_G`'s (4 inputs) — and they need separate statistics, since a neighbour's relative state and a
vehicle's own height are not the same distribution.

**RAM.** The **RAM-upload** weight buffer (~77 KB `.bss`) overflows real firmware RAM unless
weights live in flash. Two Cargo features, **both off by default**:

| Feature | Weights | Hardware fit |
|---------|---------|--------------|
| `residual_nn` | CRTP/ROS upload into `[f32; N_WEIGHTS]` | **Link fails** (+40732 B RAM on CF21BL) — fine for **host/SIL** |
| `residual_nn_flash` | `CF_RNN_WEIGHTS_NPZ` embedded at build → flash `.rodata` | **Links** on CF21BL (see `docs/45` § Implementation 2026-09-26) |

Host/SIL tests use `--features residual_nn` unless exercising flash parity
(`tools/residual/test_flash_vs_upload.py`).

### Where the neighbour states come from

Crazyswarm2 already broadcasts every vehicle's pose, and the firmware already stores the ones that
are not its own. `peerLocalizationGetPositionByIdx()` reads them, so the network's main input costs
**no new communication**.

The peer API carries **position only**. Relative velocity is differenced onboard against the
previous sample using the timestamp the same struct carries. A peer seen for the first time, or one
whose timestamp has not advanced, is given zero relative velocity rather than a divide-by-nothing
spike. Peer history is per-vehicle state and lives inside the controller `State` struct, not in a
global — the host simulator swaps that block per drone, and a global would let one vehicle's peer
history corrupt another's velocity estimate. (This is the same class of bug that made INDI fail to
leave the ground in simulation; see [`09_Simulation.md`](09_Simulation.md).)

### Guards

| Guard | Value | Why |
|---|---|---|
| Output clamp | 8 m/s² | Downwash between Crazyflies is 0.1–3 m/s². Past this it is a fault, not a prediction. Clamping is logged (`rnn.clamped`), because a silently clamped network looks exactly like a well-behaved one. **Ours, not the reference's** |
| Proximity gate | `\|dx\|<0.2`, `\|dy\|<0.2`, `\|dvx\|<1.5` | The reference's own cutoff, replicated literally. Asymmetric — it ignores `dz`, `dvy`, `dvz` — and deliberately not regularised |
| Not-loaded | returns 0 | A drone that has never been given weights behaves exactly as it did before this existed |

> **Removed 2026-09-14:** the minimum-distance clamp (0.04 m, radial rescaling) and the
> maximum-distance skip (2.0 m). Neither exists in Neural-Swarm2 — the three-term gate above **is**
> its cutoff — so both went with the architecture port rather than being kept alongside it.

---

## 3. Getting weights onto the drone

### 3.1 Flash-resident build (`residual_nn_flash`)

Trained `.npz` → `build.rs` / `export_weights_rs.py` → `EMBEDDED_RNN_WEIGHTS` in flash. No upload;
`rnn.ready` is true from boot. **Retrain ⇒ rebuild ⇒ reflash.** Details: `docs/45`.

### 3.2 CRTP upload build (`residual_nn`)

Parameter group `rnn`, serviced once per 500 Hz control tick — **before the arming check**, so the
network can be loaded while the drone is on the ground.

| Parameter | Meaning |
|---|---|
| `rnn.n` | number of weights the host intends to send |
| `rnn.begin` | 1 starts a fresh upload (clears the buffer, drops `ready`) |
| `rnn.wi` / `rnn.wv` / `rnn.wc` | index, value, commit — one weight per packet |
| `rnn.end` | 1 validates and commits |
| `rnn.ready` | read-only: 1 once a complete, finite weight set is loaded |
| `rnn.en` | 0 = predict but do not use; 1 = feed into the controller |

**A partial upload is refused outright.** `finish_upload` accepts only if the declared count matches
the architecture, every index was written, and every value is finite. This matters more than it
looks: a half-arrived network produces plausible numbers rather than an obvious failure, and would
be indistinguishable from a badly trained model. Refusing turns a dropped parameter packet into
"compensation off", which is diagnosable from the logs.

Layout is layer by layer in `phi` then `rho` order; within a layer, the weight block row-major
(`[n_out][n_in]`) followed by the bias block. The training pipeline must flatten to exactly this.
Nothing in either language checks that the two agree — which is why the layout is tested
numerically rather than trusted (§5).

---

## 4. Logging: the prediction is recorded even when it is not used

`rnn.pred_x/y/z` and `rnn.clamped` are computed and logged **every tick, regardless of `rnn.en`**.

This is not incidental. Comparing predicted against measured residual *is* the evaluation of every
learned method in this thesis, and that comparison is only possible if the prediction is recorded
on flights where it is not being used — including flights under the plain geometric controller,
which is where the training data comes from in the first place.

| Path | Status |
|---|---|
| uSD, 500 Hz | **Active** — added to `tools/usd_thesis_config.txt`, now 35 variables (limit 40) |
| Radio, 100 Hz | **Present but commented** in `crazyflies.yaml` as topic `rnn_pred` — radio saturates around 2 drones already. Enable only for single-drone tuning |

Adding these to the uSD config **now**, before any data collection, is deliberate: the collection
flights and the evaluation flights then use an identical logging config, and a config change
between the two is exactly the kind of thing that quietly invalidates a campaign.

---

## 5. Verification

`flying_drone_stack/firmware_app/host/test_residual_nn.py` links the same code that flies, drives
the real upload protocol one parameter at a time, and compares against an independent NumPy
implementation written from the architecture description rather than from the Rust source.

```bash
cd ~/Desktop/crazyflie-firmware && make bindings_python   # if not already built
cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app
python3 host/test_residual_nn.py
```

**Both suites rewritten 2026-09-14** for the ported architecture — the previous ten checks
validated the old 987-weight network and several of their assertions are now actively wrong (most
plainly "zero with no neighbours", which `phi_G` makes false by construction).

**Note the build requirement:** the `residual_nn` feature is off by default, and without it
`rnn_predict` returns zero and `rnn_service` is a no-op. Both suites detect that and fail loudly
rather than reporting a green run on a network that was never there.

```bash
cd firmware_app && DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" \
    cargo build --release --target x86_64-unknown-linux-gnu --features residual_nn
cd ~/Desktop/crazyflie-firmware && make bindings_python
```

**`host/test_residual_nn.py` — 19 checks, all passing.** Firmware-facing.

| Check | What it would catch |
|---|---|
| `residual_nn` feature present | A green run on a network that does not exist |
| Inert before upload (poisoned sentinel), incomplete upload refused, still inert | The dangerous half-loaded case |
| `rnn.pred_*` stale while not ready | Documents a real behaviour, see below |
| Ground effect only, and non-zero | `phi_G` running unconditionally — the single-drone case is **not** zero |
| Prediction is z-only | A phantom x/y prediction the architecture cannot produce |
| Numerical agreement, zero and differenced peer velocity | **Weight-layout drift** between PyTorch and the firmware's hand-indexing |
| Permutation invariance | The property deep sets exist for |
| Gate excludes `|dx|`, `|dy|`, `|dvx|` — one check each | Any "tidying" of the reference's asymmetric gate |
| Neighbour inside the gate changes the prediction | The gate tests passing because the neighbour path is dead |
| `dz` is **not** gated | The gate being made symmetric |
| grams → N → /mass applied once | A dropped or doubled unit conversion |
| Output clamped; respects `MAX_NEIGHBOURS` | The safety ceiling and the peer-buffer limit |

**`tools/residual/test_pipeline.py` — 13 checks, all passing.** Train → normalise → fold →
flatten → upload → compare against the compiled controller. Agreement is **~1e-6 m/s²** across
gated-in, gated-out, zero-velocity and differenced-velocity cases.

Random weights are used precisely so that any layout disagreement shows up immediately; trained
weights are smooth enough that an index error can look like a slightly worse model.

### Two firmware behaviours the rewrite pinned down

- **`RNN` is a process-global static that `controllerOutOfTreeInit()` does not reset.** Weights
  uploaded by one test stay loaded for the rest of the process. Any test asserting inertness must
  explicitly unload first, or it is asserting nothing.
- **`rnn.pred_*` is written only from inside `rnn_predict`, which runs only while
  `rnn.ready != 0`.** When the network is not ready the *control path* uses zero, but the *logged*
  value keeps whatever it last held. From a cold boot that is 0.0, so it never shows up in a normal
  flight — but it does if `ready` ever drops back to 0 mid-session, which starting a fresh weight
  upload does. Worth knowing before reading `rnn.pred_*` from a flight where weights were
  re-uploaded.

**Build cost:** the weight array is ~77 kB and overflows real firmware RAM, hence the
`residual_nn` feature gate (default off). With it off, all three platforms link with ~37–38 kB
free.

---

## 6. Training pipeline

`flying_drone_stack/tools/residual/` — see its
[`README.md`](../flying_drone_stack/tools/residual/README.md) for the commands. Run on **system
`python3`**, which has both torch and the SIL bindings; the pyenv `flying_robots` environment has
neither.

```
merge_usd_logs.py  ->  dataset.py  ->  train.py  ->  .npz  ->  upload_residual_weights  ->  drone
```

| File | Role | State (2026-09-17) |
|---|---|---|
| `model.py` | The PyTorch model, the flat weight layout, `fold_normalisation`, `build_mask`. **The contract with `residual_nn.rs` lives here** | ✅ rewritten for the port, verified against the compiled firmware |
| `dataset.py` | Merged CSV → tensors. Owns the sign convention and the validity filters | ✅ rewritten, verified against real flight data — see below |
| `train.py` | Training, validation, export with provenance | ✅ **rewritten 2026-09-17** — matches `dataset.py`'s 5-tuple/two-normalisation contract, exercised end-to-end via `--synthetic` |
| `test_pipeline.py` | End-to-end verification against the compiled controller | ✅ **13/13 checks pass against the real compiled firmware** (2026-09-17 re-run; does not exercise `dataset.py`'s real-CSV path) |
| `crazyflie_examples/upload_residual_weights.py` | `ros2 run crazyflie_examples upload_residual_weights` | 🔴→✅ **bug fixed 2026-09-17**: hardcoded `N_WEIGHTS=987` (stale, pre-port) would have rejected every real export. Now 19297 |

> **`dataset.py` rewritten 2026-09-16, resolving the four items below.** Three were genuinely
> just code (now done); one was never actually a decision to defer:
>
> 1. **The target is scalar** — not a decision, a fixed property of the already-verified
>    `model.NeuralSwarm2` (Z-only by construction). `y` is `a_res_z` alone, still in acceleration
>    units — the grams conversion (`model.accel_to_grams`) is left to the training step, where a
>    per-flight mass is actually known. `a_res_x`/`a_res_y` remain fully logged for other analysis;
>    nothing about data collection is narrowed by this.
> 2. `build()` now emits the **ground-effect input** `[0 − own_z, −own_vx, −own_vy, −own_vz]` —
>    `ground`, a new required return value, every row, unconditionally.
> 3. **`--z-floor` defaults to 0.0 (off)**, not removed — Neural-Swarm2 models ground effect
>    explicitly via `phi_G`, so the old default of dropping those samples would starve it. Still a
>    parameter for any other analysis that wants them excluded.
> 4. `model.build_mask` (presence AND the reference's gate) is now the only filter, applied exactly
>    as the firmware applies it. Rows with no gated neighbour are **kept**, not dropped — `phi_G`
>    still applies, and "no interaction" is a real, needed training example.
>
> Verified end-to-end against the first real A8 uSD flight (2026-09-15/16, `A8_2026-09-15_19-56-36_merged.csv`):
> `cf_second` (a_res always exactly 0, no RPM source) correctly excluded as ego/target while still
> contributing as the neighbour behind `cf231_active`'s 7518 training rows; ~7.7% of samples have
> an active neighbour gate, consistent with A8's crossings being brief; the accel↔grams round trip
> is exact to float precision. Also fixed in passing: `load_merged()` didn't know about the
> `# meta:t_zero=...` comment line `merge_usd_logs.py --meta` writes, so it read that as the header
> and fed the real header row to `np.loadtxt` as data.
>
> The loader checks that used to live in `test_pipeline.py` (peer-minus-own convention, both
> drones as ego, dropping `a_res == 0`) are re-verified above against real data rather than restored
> as unit tests — a real gap still, if a regression-proof suite is wanted later.
>
> **`train.py` rewritten 2026-09-17** to match this contract exactly: `NeuralSwarm2` (not the
> removed `DeepSets`), the two separate `normalisation_rel`/`normalisation_ground` calls
> `model.fold_normalisation` needs, the 5-tuple `build()` return, and the grams↔acceleration
> conversion applied at the loss boundary (a fixed linear scale, `model.GRAMS_TO_NEWTONS/mass`,
> not learned). `--synthetic` run: fold check `8.45e-08 m/s²`, 19297 weights exported, accepted
> by the real compiled firmware (`test_pipeline.py`, `rnn.ready=1`). `dataset.build()` against a
> *real* merged CSV through this path is still untested — only reachable once a real geometric
> C.1 flight exists.

### Decisions worth knowing

**Both drones are ego vehicles.** A 2-drone flight yields two trajectories of training data, not
one. The interaction is not symmetric — the lower vehicle is in the wash, the upper one barely
notices — so both roles carry information.

**The train/validation split is by contiguous block, not per sample.** At 500 Hz, consecutive
samples are nearly identical; a random per-sample split leaks almost every validation point into
training and reports a validation error that means nothing. Blocks of ~1 s keep the sets genuinely
separate while still covering the whole flight.

**Normalisation statistics come from present neighbours only.** Padding rows are exact zeros;
including them would pull the mean toward zero by an amount that depends on how many drones
happened to be flying, so a 2-drone and a 3-drone model would normalise differently for no
physical reason.

**Huber loss, not squared error.** Log spikes should not steer the fit.

**The reported number is RMSE against predicting zero.** A model that does not beat that baseline
has learned nothing, however small its loss looks — `train.py` says so explicitly and warns if
sampled predictions are hitting the output clamp.

**Weights carry provenance.** Each `.npz` records the source logs, sample count, validation and
baseline RMSE, seed and git revision. Weights nobody can trace back to flights are weights nobody
can defend in a thesis.

**Upload does not enable.** `upload_residual_weights` leaves `rnn.en` at 0 unless `--enable` is
passed. The first flights with a new model exist to compare predicted against measured residual,
not to hand the model authority over the vehicle.

### Verification — `test_pipeline.py`

> **Superseded 2026-09-14.** The 12 checks recorded below were run against the OLD 987-weight
> network. The file has since been rewritten for the Neural-Swarm2 port (13 checks, all passing;
> see §5). The reasoning below is kept because it is still why the test exists; the specific
> checks and numbers are not current.

Step 1's test proved the firmware evaluates *a given* weight vector correctly. It said nothing
about whether the pipeline produces *that* vector. This closes that gap by running a trained model
through the real export and the real upload protocol into the compiled controller:

| Check | What it would catch |
|---|---|
| `flatten` → `load_flat` lossless | A layout that is not its own inverse |
| Normalisation fold is exact | A mis-derived fold — the algebra in §2 being wrong |
| torch == `firmware_forward` on the exported vector | PyTorch's storage order disagreeing with the firmware's hand-indexing |
| **Compiled firmware == reference**, zero peer velocity | Everything above, plus the upload protocol, against the real binary |
| **Compiled firmware == reference**, differenced peer velocity | The onboard timestamp differencing |
| Neighbour above pushes down; below does not | **The sign convention**, checked against the firmware rather than asserted |
| Loader uses peer-minus-own | The `rel.*` sign trap in `merge_usd_logs.py` |
| Both drones kept as ego | Half the dataset being silently discarded |
| `a_res == 0` dropped | A dataset of zeros training a network that predicts nothing and looks converged |
| Output clamp still engages | The safety ceiling surviving a pipeline upload |

Agreement between torch and the compiled firmware is ~1e-6 m/s², i.e. float32 rounding.

### Two bugs this step found

**`decode_usd_log.py` never mapped the thesis channels.** `stateEstimate.x`, `indi.a_res_*`,
`acc.*`, `ctrltarget.*` and the z-axis of `tau`/`alp` all kept their firmware names, dots and all,
while `merge_usd_logs.py` looked up `x`, `vx`, `a_res_x` — and guarded those lookups with
`if k in out`. The merge would have **succeeded while producing no relative state and no `f_res`
at all**, which is precisely the input this pipeline consumes. `RENAME` is now complete for every
channel in `usd_thesis_config.txt`, `load()` warns about anything unmapped, and `merge()` fails
loudly instead of skipping.

**Synthetic data bypassed the firmware's input guards.** Caught by `test_pipeline.py` as a 1.2e-2
disagreement between torch and the firmware while the compiled firmware itself matched the
reference exactly. The guards are now one function, `dataset.apply_input_guards`, used by both the
real and the synthetic path.

### Synthetic mode

`train.py --synthetic` fits a made-up downwash-shaped function so the whole path can be exercised
before a drone has flown. **No result may be quoted from it.** Files written this way carry a
`synthetic: true` flag that the uploader prints in capitals.

### Real-data E2E validation (2026-09-21)

Executed per [`40`](40_C2_Residual_Pipeline_E2E_Validation_Plan.md). Scripts:
`experiments/analysis/c2_e2e_validation.py`, `tools/residual/test_real_data_pipeline.py`,
`experiments/analysis/c2_stage_e_dryrun.sh`.

| Stage | Result |
|---|---|
| **A** | 5 C.1-eligible merges (21 Sep) → **4** usable (`A1_13-25-10`: all-zero `a_res`). **Bank now 24** training merges after 23 Sep — re-run Stage A on full manifest set when extending C.2. |
| **B LOO** | A3 folds: RMSE ↓ vs predict-zero (~33–71%). **A1 hold-out: RMSE 5.66 vs baseline 1.79 (−216%)** |
| **B cross** | Train A3 → test A1: same failure; train A1 → test A3: RMSE ~9–10 vs baseline ~0.74 |
| **C** | Path A (positions, NumPy peer dv=0): max \|Δ\| ≈ 6.6×10⁻⁷ m/s². Path B (differenced `oot_set_peer`, measured `rel` vel): ≈ 6×10⁻⁶ m/s². Path A alone does not prove the \|dvx\| gate; Path B does. Sign sanity OK |
| **D** | Synthetic overhead sweep on LOO weights is **not** a clean decay signature (thin data / extrapolation). Held-out **binned error vs dz** is structured on A3 (errors smaller near dz ~0.2–0.3 m) and flat-bad on A1 dz ~0.75 m — see the **2026-09-21 coverage grid** in [`25_C1_Data_Collection_Plan.md`](25_C1_Data_Collection_Plan.md) (which dz/speed cells are merged vs refly-only) |
| **E** | **First attempt:** inconclusive (EKF `[0,0,0]` vs real `/pose` — root cause in `docs/38`). **After sim plumbing fix + rerun:** A3 dz 0.30, `rnn.en=0/1`, **5200+ rows**, hover to ~1.0 m / ~1.3 m, `meaningful_flight: true`, no divergence (`c2_e2e_stage_e.json`). **Not** hardware-like EKF: ROS “EKF” = ground-truth passthrough (Kalman externalize unused for publish). Log path: `FLYING_ROBOT_COURSE_ROOT` (2026-09-21) |

---

## 7. End-to-end dry run in simulation

> **2026-08-23 run:** OLD 987-weight network — numbers below are historical. **2026-09-21:** Stage E
> dry run with real LOO weights: `c2_stage_e_dryrun.sh` → `c2_e2e_stage_e.json` (**pass**, real
> closed loop in sim; see `docs/38` for ground-truth `/state`/`/pose` caveat). Older
> `run_residual_dryrun.sh` path is hardware-oriented rehearsal.

`experiments/analysis/run_residual_dryrun.sh` rehearses the whole of thesis C.1 → C.2 → C.3 without a
drone. It exists because every step of that sequence can fail, and finding out in the lab costs
flight time that cannot be recovered.

| Phase | Config | What runs |
|---|---|---|
| 1 COLLECT | `server_sim_res_collect.yaml` | Plain geometric, no network. The downwash is *observed*, not partly cancelled — the same reason C.1 collects under geometric on hardware |
| 2 TRAIN | — | `tools/residual/train.py` on the phase-1 CSV |
| 3 PREDICT | `server_sim_res_predict.yaml` | Weights loaded, **`rnn.en=0`** — the network predicts and is logged but touches nothing |
| 4 COMPENSATE | `server_sim_res_compensate.yaml` | Same weights, **`rnn.en=1`** — the prediction feeds the position loop (strategy 2) |
| 5 COMPARE | — | `experiments/analysis/analyse_residual_dryrun.py` |

**Phase 3 is the one that carries the result.** With the network in the loop the residual it sees
is the residual that *survives its own compensation*, so a small `a_res` in phase 4 is equally
consistent with a good model and with a stable-but-wrong feedback loop. Only phase 3, where
prediction and measurement are independent, distinguishes them. Phase 4 answers a different and
narrower question: does turning it on help, and does it stay stable.

### What the simulator had to grow

**Peer injection.** The firmware reads neighbours from `peer_localization`, which the simulator
does not have. `crazyflie_server.py` now hands every vehicle the others' **positions** each tick,
and `oot_host.c` presents them through the same `peer_get_all` the drone calls. Positions only,
matching the real API exactly — feeding a velocity the firmware cannot have would make simulation
easier than reality in precisely the place the residual model depends on.

**A residual log.** `sim.residual_log` writes a CSV whose columns are exactly what
`merge_usd_logs.py` produces from real uSD logs, so `dataset.py` reads simulation and flight data
through one path with no special cases — which means the dry run genuinely tests the loader that
hardware data will use.

**`oot_rnn_service()`.** On the drone, `rnn_service` runs from the control loop every tick, before
the arming check, so weights load on the ground. The simulator's SIL layer returns early while a
vehicle is idle and never calls the controller at all, so piggybacking there would have stalled the
upload until takeoff and then landed weights *mid-flight*. The export lets the simulator drive the
same routine at the same point in the run. It is not a second code path — it calls `rnn_service`
and nothing else.

### Result — 2026-08-23, A3 at Δz 0.30, two robots, geometric

Full report: [`experiments/sim_validation/RESIDUAL_DRYRUN.md`](../experiments/sim_validation/RESIDUAL_DRYRUN.md).

Training set: 4397 log rows → **7731 samples** (both drones as ego). 1157 samples dropped for
`a_res` identically zero — the pre-arming rows, correctly rejected. 26 neighbour observations hit
the `MIN_DIST` clamp.

| | |
|---|---|
| Validation RMSE | **0.021 m/s²** against a predict-zero baseline of **0.186** — 88.7% reduction |
| Normalisation fold check | 7e-7 m/s², i.e. float32 rounding |
| Upload | 987 weights through the real `rnn.*` protocol, `rnn.ready = 1` |

**Phase 3 — does it predict? (`rnn.en = 0`, prediction and measurement independent)**

| Drone | RMSE(`a_res`) | RMSE(pred − `a_res`) | reduction | corr | clamped |
|---|---|---|---|---|---|
| `cf231_active` | 0.341 | 0.033 | **+90.4%** | 0.994 | 0% |
| `cf_second` | 0.125 | 0.030 | **+75.9%** | 0.967 | 0% |

**Phase 4 — does compensation help? (`rnn.en = 1`, error against the *commanded* separation)**

| Axis | quantity | collect | compensate | change |
|---|---|---|---|---|
| z | mean error | 4.66 mm | **0.63 mm** | −86.6% |
| z | spread (std) | 11.74 mm | **3.22 mm** | −72.6% |
| x | spread (std) | 1.65 mm | 1.14 mm | −31.1% |

The z bias is the one that matters: downwash pushes the lower vehicle down, so it appears as the
held gap closing, and the feedforward removed most of it. Nothing clamped, nothing destabilised.

**What this does and does not show.** It shows the plumbing works: collect, train, fold, upload,
enable, measure — through the code paths the lab will use, with the same loader that will read
hardware logs. It does **not** show the method works, because the residual here is generated by
the simulator's own Neural-Swarm2 model and then fitted by a model of the same family. Learning a
network's output with another network is a far easier problem than learning real air. The honest
claim is "the pipeline is ready", not "the method achieves 90%".

Two axes were never excited: A3 is a vertical stack, so relative `y` and `v_y` are constant and
`train.py` reported `sigma = 1.0` for both. The warning worked; the lesson for C.1 is that the
collection set must include laterally offset scenarios (A4, A7) or the model is extrapolating
across half its input space on the first flight that moves sideways.

### The control-law change

`rnn.en` now does something. In the position loop:

```rust
let a_nn = if g_rnn_en != 0 && g_rnn_ready != 0 { rnn_pred } else { Vec3::zero() };
let f_d = ad + kp*ep + kv*ev + ki + g_comp - a_indi - a_nn;
```

Same sign as the INDI residual term, because it is an estimate of the **same quantity** — the
difference is only that INDI measures the disturbance after the fact while the network predicts it
before it arrives. Default `rnn.en = 0` makes `a_nn` exactly zero, so the control law is
byte-identical to what flew before.

For **Strategy 2**, only the network term is used in the campaign (`ctrl_mode=0`, `rnn.en=1`).
The INDI term (`a_indi`) is zero on that mode. **`ctrl_mode=3` + `rnn.en=1` is not a compared
mode** (former hybrid slot removed Sep 2026).

---

## 8. What is not done

| | |
|---|---|
| **Flying any of it** | The control-law change (`rnn.en`) has never been on a drone, and it changes a control term. It comes *after* C.0 validates the two already-unflown fixes, not before |
| **Training on a lateral scenario** | A3 alone leaves relative `y` unexcited. C.1 must collect A4/A7 as well |
| **Strategy 3 (FBL)** | Not wired — author code pending. Only Strategy 2's geometric feedforward exists |
| **Training on real data** | The pipeline has only been run on synthetic data and a hand-built CSV fixture — no formation flight has happened yet, so nothing has been trained on a measurement |
| **Strategies 5 and 7 (RL policies)** | Deliberately not started |
| **Training from flight logs at all** | Pipeline ✅ complete and verified against the compiled firmware (§6, `test_pipeline.py` 13/13). Blocked only on C.1 producing a real log — `dataset.build()` against real data is untested |
| **Re-running the simulation dry run** | §7's rehearsal predates the port; would need its own re-run against the new architecture, not blocked by anything above anymore |

---

## 9. Files

| File | Role |
|---|---|
| `firmware_app/src/residual_nn.rs` | The network: architecture, upload state machine, inference |
| `firmware_app/src/lib.rs` | `rnn_service()` (upload, with the other upload handlers), `rnn_predict()` (per-tick evaluation), `State.peer_prev` |
| `firmware_app/traj_iface.c` | `rnn` param and log groups, `peer_get_all()` |
| `firmware_app/host/oot_host.c` | Peer injection for the simulator, which has no `peer_localization` module |
| `firmware_app/host/test_residual_nn.py` | The numerical check for the onboard evaluation — 19 checks |
| `firmware_app/Cargo.toml` | The `residual_nn` feature gate (default **off**; the weight array overflows real RAM) |
| `tools/residual/` | The training pipeline, all ✅: `model.py`, `test_pipeline.py`, `dataset.py` (2026-09-16), `train.py` (2026-09-17) |
| `tools/merge_usd_logs.py`, `tools/decode_usd_log.py` | Per-drone uSD logs → one time-aligned CSV |
| `crazyflie_examples/upload_residual_weights.py` | Weight upload over ROS/CRTP — ✅ `N_WEIGHTS` bug fixed 2026-09-17 |
| `tools/usd_thesis_config.txt` | 500 Hz onboard logging, now including `rnn.pred_*` |
| `crazyswarm2/crazyflie/config/crazyflies.yaml` | Commented `rnn_pred` radio topic |

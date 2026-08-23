# Residual Learning — Onboard Inference and Offline Training

> ## ✅ The residual-learning **software foundation is COMPLETE**
>
> All three build steps are done and verified: the onboard network + weight upload, the training
> pipeline, and an end-to-end dry run in simulation. **Nothing further is needed here before the
> lab.**
>
> **Nothing has been trained on a real flight** — no formation flight has happened, so every
> number in this document is a simulation number.
>
> **Where this sits in the Core Thesis Workflow** ([`07`](07_Thesis_Progress_Checklist.md)):
>
> | Step | This document's role |
> |---|---|
> | **C.0 Hardware Gate** ⬅️ next | Must clear `rnn.en` — §7 "The control-law change". **Unflown** |
> | **C.1 Residual Data Collection** | Produces the training set. §7 says why A4/A7 are mandatory |
> | **C.2 Train the Residual Model** | Run §6's pipeline on real data. **Ready** |
> | **C.3 Integrate the Strategies** | 5 of 7 consume this prediction; only Strategy 2 is wired |
> | **C.4 Systematic Comparison** | §4's logged prediction vs measurement is the evaluation |

**Last updated:** 23 August 2026

Five of the seven control strategies being compared need the same thing underneath them: a model
that predicts the interaction force one vehicle is about to feel from the others. This document
is the engineering record of that shared foundation — what runs on the drone, how weights get
there, and how the prediction is evaluated.

It is deliberately one layer down from
[`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md), which holds status. The
physics is in [`04_Unified_Residual_Wrench_Model.md`](04_Unified_Residual_Wrench_Model.md).

---

## 1. Why this is shared infrastructure, not one method's implementation

| Strategy | Uses the residual network? |
|---|---|
| 1. Pure INDI | No — reacts to the measured residual instead |
| 2. Geometric + NN | **Yes**, as a feedforward term |
| 3. FBL + NN | **Yes** *(FBL code still with the authors)* |
| 4. Hybrid neural-augmented INDI | **Yes**, alongside the INDI measurement |
| 5. Residual RL | Later — separate policy network, deliberately not started |
| 6. Learning-based MPC | **Yes**, as the prediction model inside the horizon |
| 7. Geometric + residual RL | Later |

Building the network once, with one weight format and one upload path, means those methods differ
in *how they use the prediction* rather than in how they obtain it. That is what makes the
comparison a comparison. Building it per-method would introduce differences that are implementation
artefacts and would be indistinguishable from real differences between the control laws.

---

## 2. What runs onboard

`flying_drone_stack/firmware_app/src/residual_nn.rs` — `no_std` Rust, cross-compiled to the same
Cortex-M4F binary that flies, and linked unchanged into the host simulator.

### Architecture: deep sets

```
a_res  =  rho( sum_j phi(relative_state_j) )
```

`phi` runs on each neighbour separately; the results are summed; `rho` maps the sum to a
3-vector of acceleration in the world frame.

A plain fixed-input MLP would have to be retrained for every neighbour count, and its answer would
depend on the order the neighbours happened to be listed in. Neither is true of the physics. The
deep-sets form (Neural-Swarm2) is permutation-invariant by construction and one weight set serves
two drones or three — which matters directly here, since the campaign flies both, and B1/B2 exist
precisely to test whether interactions superpose.

| | Dimensions |
|---|---|
| `phi` | 6 → 16 → 16 → 8, ReLU |
| `rho` | 8 → 16 → 16 → 3, ReLU except the linear output |
| Weights + biases | **987** floats |
| Cost | ≈ 1400 MACs for two neighbours |
| Neighbours | up to 3 |

Input per neighbour is the relative position and relative velocity in the world frame, 6 numbers.
Output is `f_res / m` in m/s², the same quantity `indi.a_res_*` measures — so prediction and
measurement are directly comparable without a conversion step that could be got wrong.

**Normalisation is deliberately absent from the firmware.** Input scaling and mean-subtraction are
folded into the first layer's weights and biases at export time. That is exactly equivalent, and it
keeps the firmware from carrying a second set of constants that could drift out of step with the
trained model.

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
| Output clamp | 8 m/s² | Downwash between Crazyflies is 0.1–3 m/s². Past this it is a fault, not a prediction. Clamping is logged (`rnn.clamped`), because a silently clamped network looks exactly like a well-behaved one |
| Minimum distance | 0.04 m | Below this the model extrapolates past anything it can have been trained on, and a learned function is least trustworthy exactly where the physics is strongest. The radial distance is clamped without changing direction |
| Maximum distance | 2.0 m | Beyond this a neighbour contributes nothing measurable; skipping keeps the cost proportional to the neighbours that matter |
| Not-loaded | returns 0 | A drone that has never been given weights behaves exactly as it did before this existed |

---

## 3. Weight upload

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

Ten checks, all passing as of 2026-08-23:

| Check | What it would catch |
|---|---|
| Inert before upload | A network that acts on zero weights |
| Incomplete upload refused, and still inert | The dangerous half-loaded case |
| Complete upload accepted | The protocol working at all |
| Numerical agreement, first sample (zero peer velocity) | **Weight-layout drift** between PyTorch and the firmware's hand-indexing — an off-by-one in a bias block |
| Numerical agreement, differenced peer velocity | The timestamp differencing |
| Permutation invariance | The property deep sets exist for |
| Distant neighbour ignored | The `MAX_DIST` gate |
| Output clamped | The safety ceiling engaging |
| Zero with no neighbours | The single-drone case staying untouched |

Random weights are used precisely so that any layout disagreement shows up immediately; trained
weights are smooth enough that an index error can look like a slightly worse model.

**Build cost:** 80 bytes of flash and ~3.9 kB of RAM (the weight buffer). Flash 35 %, RAM 73 %.

---

## 6. Training pipeline

`flying_drone_stack/tools/residual/` — see its
[`README.md`](../flying_drone_stack/tools/residual/README.md) for the commands. Run on **system
`python3`**, which has both torch and the SIL bindings; the pyenv `flying_robots` environment has
neither.

```
merge_usd_logs.py  ->  dataset.py  ->  train.py  ->  .npz  ->  upload_residual_weights  ->  drone
```

| File | Role |
|---|---|
| `model.py` | The PyTorch model, the flat weight layout, and `fold_normalisation`. **The contract with `residual_nn.rs` lives here** |
| `dataset.py` | Merged CSV → tensors. Owns the sign convention, the firmware's input guards, and the validity filters |
| `train.py` | Training, validation, export with provenance |
| `test_pipeline.py` | End-to-end verification against the compiled controller |
| `crazyflie_examples/upload_residual_weights.py` | `ros2 run crazyflie_examples upload_residual_weights` |

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

### Verification — `test_pipeline.py`, 12 checks, all passing 2026-08-23

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

---

## 7. End-to-end dry run in simulation

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

The INDI term and the network term are deliberately allowed to be on together. Double-counting is
a real risk, and it is exactly what strategy 4 (hybrid neural-augmented INDI) exists to measure;
preventing it here would remove the comparison.

---

## 8. What is not done

| | |
|---|---|
| **Flying any of it** | The control-law change (`rnn.en`) has never been on a drone, and it changes a control term. It comes *after* C.0 validates the two already-unflown fixes, not before |
| **Training on a lateral scenario** | A3 alone leaves relative `y` unexcited. C.1 must collect A4/A7 as well |
| **Strategies 3, 4, 6** | The prediction is available to them but none is wired up. Only strategy 2's feedforward exists |
| **Training on real data** | The pipeline has only been run on synthetic data and a hand-built CSV fixture — no formation flight has happened yet, so nothing has been trained on a measurement |
| **Strategies 5 and 7 (RL policies)** | Deliberately not started |

---

## 9. Files

| File | Role |
|---|---|
| `firmware_app/src/residual_nn.rs` | The network: architecture, upload state machine, inference |
| `firmware_app/src/lib.rs` | `rnn_service()` (upload, with the other upload handlers), `rnn_predict()` (per-tick evaluation), `State.peer_prev` |
| `firmware_app/traj_iface.c` | `rnn` param and log groups, `peer_get_all()` |
| `firmware_app/host/oot_host.c` | Peer injection for the simulator, which has no `peer_localization` module |
| `firmware_app/host/test_residual_nn.py` | The numerical check for the onboard evaluation |
| `tools/residual/` | The training pipeline: `model.py`, `dataset.py`, `train.py`, `test_pipeline.py` |
| `tools/merge_usd_logs.py`, `tools/decode_usd_log.py` | Per-drone uSD logs → one time-aligned CSV |
| `crazyflie_examples/upload_residual_weights.py` | Weight upload over ROS/CRTP |
| `tools/usd_thesis_config.txt` | 500 Hz onboard logging, now including `rnn.pred_*` |
| `crazyswarm2/crazyflie/config/crazyflies.yaml` | Commented `rnn_pred` radio topic |

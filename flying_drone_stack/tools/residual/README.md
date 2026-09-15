# Residual model — training pipeline

Trains the Neural-Swarm2 residual model and exports weights the drone will accept. The onboard
half lives in `firmware_app/src/residual_nn.rs`; the full design is in
[`docs/13_Residual_Learning.md`](../../../docs/13_Residual_Learning.md).

**Use system `python3`** — it has torch and the SIL bindings. The pyenv `flying_robots`
environment has neither.

## ⚠️ State, 2026-09-14 — half of this pipeline runs, half does not

`residual_nn.rs` was replaced with a faithful port of Neural-Swarm2's own architecture
(`phi_Net`/`rho_Net`, 19297 weights, scalar Z-only output, an always-on ground-effect term, and
the reference's three-term proximity gate). The Python side was rewritten to match — partly.

| File | State |
|---|---|
| `model.py` | **Current.** Rewritten for the port; verified against the compiled firmware to ~1e-6 m/s². |
| `test_pipeline.py` | **Current.** 13 checks, all passing, including upload into the real controller. |
| `../../firmware_app/host/test_residual_nn.py` | **Current.** 19 checks, all passing. |
| `dataset.py` | **Stale — raises on import.** Cannot produce the scalar target or the ground-effect input. |
| `train.py` | **Blocked** by `dataset.py`. |

So the **model ↔ firmware contract is verified end to end**, and **training from flight logs is
not possible yet**. Nothing here is on the critical path until C.1 produces logs, but it must be
finished before C.2.

Four decisions are needed before `dataset.py` can be rewritten; all four are stated in full in
that file's docstring:

1. The target is now **scalar** — the architecture predicts vertical residual only, so the x/y
   components of the measured `a_res` have no predictor. This is a scoping consequence for the
   thesis comparison, not only for the code.
2. `build()` must also emit the **ground-effect input** `[0 − own_z, −own_vx, −own_vy, −own_vz]`.
3. **`--z-floor` now contradicts the architecture** — it drops low-altitude samples because
   "ground effect is a different force", but Neural-Swarm2 models ground effect explicitly and
   needs exactly those samples.
4. The distance cutoff and near-field rescaling are gone; `model.build_mask` replaces them.

The loader tests that used to live in `test_pipeline.py` (peer-minus-own convention, both drones
as ego, dropping `a_res == 0` samples) went with `dataset.py` and are **not covered anywhere
right now**. That is a real gap.

`phi_L`/`rho_L` — the reference's "large vehicle" path — export as **zeros**: this fleet is
Crazyflies only, there is no data to train them on, and zeros state that plainly rather than
shipping an untrained copy of the small path that would look trained.

## The loop

```bash
# 1. merge the per-drone uSD logs from one flight into a common clock
python3 ../merge_usd_logs.py cf231.usd cf232.usd -o merged_a1.csv

# 2. train, and export weights folded and flattened for the firmware
python3 train.py merged_a1.csv -o weights/a1_geometric.npz

# 3. upload (from the crazyswarm2 workspace; does NOT switch the model on)
ros2 run crazyflie_examples upload_residual_weights -- --weights weights/a1_geometric.npz
```

Before any of that works on a new machine, and after any change to the architecture:

```bash
python3 test_pipeline.py          # torch -> export -> upload -> compiled firmware, 12 checks
```

## Files

| File | Role |
|---|---|
| `model.py` | The PyTorch model, the flat weight layout, and the normalisation fold. **The contract with the firmware lives here** |
| `dataset.py` | Merged CSV → training tensors. Owns the sign convention and the firmware's input guards |
| `train.py` | Training, validation, export with provenance |
| `test_pipeline.py` | End-to-end verification against the compiled controller |

## Four things that will bite

**Sign convention.** The firmware feeds `peer − own`. `merge_usd_logs.py` writes `rel.i_j` as
`i − j`, which is the opposite when `i` is the ego drone. `dataset.py` therefore computes relative
states from the absolute columns and never reads the `rel.*` columns. Getting this backwards
trains a model that pushes the drone *into* the disturbance, and it will look perfectly
well-converged while doing it.

**Weight layout.** Layer by layer (φ then ρ); within a layer, the weight block row-major
`[n_out][n_in]`, then the bias block. Nothing cross-checks PyTorch's storage against the
firmware's hand-indexing — `test_pipeline.py` is the only thing standing between an off-by-one
and a plausible wrong answer. Change a layer size and you must change it in `model.py` *and*
`residual_nn.rs`, then re-run that test.

**Normalisation is not in the firmware.** It is folded into layer 1 at export
(`fold_normalisation`). Do not add a second normalisation anywhere; do not call the fold twice.
`train.py` verifies the folded model reproduces the trained one and refuses to write the file if
it does not.

**`a_res` identically zero.** That is what a missing RPM source looks like — an absent
measurement, not an observed absence of interaction. `dataset.py` drops those samples and says how
many. If most of a flight is dropped, the flight needs redoing, not the dataset trimming.

## Reading the output

`train.py` prints validation RMSE next to the RMSE of *predicting zero*. The second number is the
one that matters: a model that does not beat it has learned nothing, however small its loss looks.
It also samples the exported weights through the firmware's own evaluation and warns if
predictions are hitting the 8 m/s² output clamp — that is a fault signature, not a strong model.

## Synthetic mode

`train.py --synthetic` fits a made-up downwash-shaped function. It exists so the whole path can be
exercised before a drone has flown, because every step of it can fail and finding out in the lab
costs flight time. **No result may be quoted from it**, and `.npz` files written this way carry a
`synthetic: true` flag that the uploader prints in capitals.

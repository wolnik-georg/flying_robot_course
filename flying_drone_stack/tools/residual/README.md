# Residual model — training pipeline

Trains the Neural-Swarm2 residual model and exports weights the drone will accept. The onboard
half lives in `firmware_app/src/residual_nn.rs`; the full design is in
[`docs/13_Residual_Learning.md`](../../../docs/13_Residual_Learning.md).

**Use system `python3`** — it has torch and the SIL bindings. The pyenv `flying_robots`
environment has neither.

## State (2026-09-21) — pipeline verified on real C.1 data; model quality data-limited

Neural-Swarm2 port (`phi_Net`/`rho_Net`, 19297 weights, scalar Z). **Authoritative detail:**
[`docs/13_Residual_Learning.md`](../../../docs/13_Residual_Learning.md). **E2E validation:**
[`docs/40_C2_Residual_Pipeline_E2E_Validation_Plan.md`](../../../docs/40_C2_Residual_Pipeline_E2E_Validation_Plan.md)
→ `experiments/analysis/out/c2_e2e_2026-09-21/`.

| File | State |
|---|---|
| `model.py` | ✅ Verified vs compiled firmware (~1e-6 m/s²) |
| `dataset.py` | ✅ Rewritten 2026-09-16; tested on real merged CSV |
| `train.py` | ✅ Runs on real merges; LOO on 4 flights (2026-09-21) |
| `test_pipeline.py` | ✅ 13/13 vs compiled firmware |
| `test_real_data_pipeline.py` | ✅ Real rows vs compiled (~1e-6 m/s²) |
| `../../firmware_app/host/test_residual_nn.py` | ✅ 19 checks |

**Still missing for a defensible deployed model:** more **C.1 merged logs** (scenario/dz coverage,
see `docs/25`) — especially A1 reflys and A3 B-2/B-3. Software is not the blocker.

**Desk dry-run (2026-09-21):** trained on one A8 geometric merge — rehearsal only.
Manifest: `experiments/analysis/out/c2_dryrun/manifest.json` · weights:
`weights/c2_dryrun_2026-09-19_geo.npz`. Index: [`docs/31_Desk_Parallel_Track.md`](../../../docs/31_Desk_Parallel_Track.md).

`phi_L`/`rho_L` — the reference's "large vehicle" path — export as **zeros**: this fleet is
Crazyflies only, there is no data to train them on, and zeros state that plainly rather than
shipping an untrained copy of the small path that would look trained.

## The loop

```bash
# 1. merge the per-drone uSD logs from one flight into a common clock
python3 ../merge_usd_logs.py --meta --roles ... -o merged_a1.csv

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
| `eval_model.py` | P5 evaluation on merged CSVs (R², flight-wise, error vs geometry) |
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

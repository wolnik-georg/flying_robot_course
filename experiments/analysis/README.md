# Flight-metrics analysis

`run_analysis.py` turns a set of logs plus a scenario id into the numbers Chapter 5
already promised (RMSE, sag, residual, e_R), so the first real flight campaign does not
wait on ad-hoc pandas. Added 2026-09-08, desk work while lab-blocked (see
`docs/07_Thesis_Progress_Checklist.md`).

## Run it

```bash
python experiments/analysis/run_analysis.py \
  --scenario A1 --ctrl geometric \
  --logs path/cf1.csv path/cf2.csv \
  --dz-cmd 0.75 \
  --out experiments/analysis/out/
```

`--logs` accepts either **one merged CSV** (all vehicles inside) or **N per-vehicle
files** (see "Log formats" below — mixing the two is refused).

## Plot it

`plot_flight.py` (added 2026-09-09) is the plotting companion — same loaders, same
commanded-trajectory reconstruction as `run_analysis.py` (it imports `metrics.py`
directly), so a plot and the numbers in `*_metrics.csv` never disagree. One PNG per
call, single- or multi-drone:

```bash
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/plot_flight.py \
  --scenario A1 --ctrl geometric \
  --logs path/cf1.csv path/cf2.csv \
  --dz-cmd 0.75 [--sidecar path/A1_<stamp>.meta.json] \
  --source sim --out experiments/analysis/out/
```

**Use the pyenv env, not system `python3`** — this machine's system `matplotlib` is
broken (compiled against a numpy 1.x ABI, current numpy is 2.x); `run_analysis.py`
runs fine under system `python3` since it only needs numpy, but plotting needs a
working matplotlib, and the pyenv `flying_robots` env has one.

Seven panels, single drone or many, arranged so the ones a comparative study actually
needs are together: XY path vs commanded, altitude vs commanded, position-error norm,
pairwise vertical separation vs the commanded value (multi-drone only), `|a_res|` per
vehicle, `|e_R|` per vehicle, and predicted-vs-measured residual magnitude
(`rnn.pred_*` vs `a_res`, when present) — plus a text panel with the same RMSE/sag/rms
numbers `run_analysis.py` writes to `*_metrics.csv`, so the plot is never the only
place a number lives. A panel with no data for that log (e.g. `e_R` on anything logged
before 2026-09-08, or the predicted-residual panel when `rnn.en` was never touched)
says so explicitly instead of being blank by omission — same NaN-≠-0 discipline as
the metrics script.

Superset of what `archive/Controls/analyze_flight.py` did for the old single-drone
Mode-D/E trajectories (path vs planned, per-axis vs planned, tracking error), rebuilt
for the current formation-scenario commanded-trajectory representation
(`scenarios.py`'s `curve(t)`, not the old poly4d/onboard8 formats) and extended with
the residual/attitude-error signals that script predates.

`--sidecar path/A1_<stamp>.meta.json` is needed only when the log itself carries no
setpoint (see "Commanded trajectory" below).

`--source {sim,hardware}` labels the report. Defaults to `sim` — it never silently
claims hardware. Passing `--source hardware` against a `sim`-format log (see below) is
refused as a contradiction.

## Log formats — three, and they are NOT the same schema

| Format | What it is | Has a_res / e_R / setpoint? |
|---|---|---|
| `ros` | Per-vehicle CSV from `DroneLogger` (`formation_flight.py` / `run_formation.py`), header `time_s,pos_x,...,a_res_z` | `a_res` yes, `e_R` **no** (never wired into this path), setpoint **no** |
| `merged` | One CSV, one shared `t`, then `{vehicle}.{field}` columns — what `tools/merge_usd_logs.py` produces from real uSD logs, and what `analyse_residual_dryrun.py`'s sim dry run also writes directly | whatever fields the source had, per-vehicle |
| `usd` | A single vehicle's decoded uSD binary log via `flying_drone_stack/tools/decode_usd_log.py`'s `load()` | `a_res` yes, `e_R` yes (added 2026-09-08), `ctrltarget` (setpoint) yes — the richest format, hardware only |

There is a fourth thing that looks like a format but isn't one here: **`record_states`**,
the CS2 simulator's own ground-truth pose CSV (`timestamp,x,y,z,qw,qx,qy,qz`, one file
per vehicle, written by `verify_formation_sim.py`'s pipeline). It carries position only
— no controller-internal signal exists in it at all. `run_analysis.py` calls this the
`sim` format; every `a_res`/`e_R`/`a_hat` column comes out **NaN**, not 0, because the
signal genuinely was never recorded, not because it happened to be zero.

**A real finding from building this script (2026-09-08):** every `ros`-format CSV this
repo has produced *in simulation* so far has **zero data rows** — checked across
A1/A2/B2 sim runs on disk. The sim server never publishes the custom radio log topics
`DroneLogger` subscribes to (`{name}/state`, `{name}/indi_a_res`, ...); only real
hardware does. `run_analysis.py` handles this without crashing (reports `n_samples=0`,
`nan_fraction=1.0`, a `NO DATA` note) rather than pretending it has a result. The one
genuinely populated sim artifact on disk is `experiments/sim_validation/residual_collect.csv`
(`merged` format), used as this script's real-data test fixture.

**No converter exists between `ros` and the other two.** `ros` is missing fields that
were simply never wired into that logging path. Do not pretend it can be upgraded to
`usd`'s schema — it can't, without changing what `DroneLogger` subscribes to.

## Commanded trajectory

RMSE is computed against:
1. A setpoint already in the log (`cmd_*` in `merged`, `ctrltarget_*` in `usd`) — used
   directly, no reconstruction.
2. Otherwise, the scenario polynomial reconstructed from `--sidecar` + the scenario
   library (`anchor + robot.slot + robot.curve(t)`), the exact same rule
   `verify_formation_sim.py` already uses — not a second, invented way to do this.
3. If neither exists: **the script refuses to run**, loudly, rather than RMSE a
   vehicle's position against nothing.

## Column dictionary (`*_metrics.csv`)

| Column | Meaning |
|---|---|
| `n_samples` / `n_raw` / `nan_fraction` | Rows actually used / rows in the source file / fraction of used rows with a NaN commanded position. **No sample is ever silently dropped** — this triple is how you'd notice if one were. |
| `pos_rmse_m`, `pos_rmse_x/y/z` | RMSE of measured vs. commanded position, whole-vector and per-axis. |
| `pos_peak_m`, `pos_peak_z` | Largest tracking error seen, not the mean — the number a safety read cares about. |
| `dz_cmd_m`, `dz_mean_m` | Commanded vs. measured mean vertical separation, on the formation row only. |
| `dz_err_mean_m`, `dz_err_rms_m` | **Sag** = `dz_cmd - dz_measured`. **Positive sag means the gap closed** (lower vehicle too high, or equivalently too low relative to the upper one) — this sign convention is fixed here, not left for the reader to infer from a plot. |
| `a_res_rms`, `a_res_z_mean`, `a_res_z_rms` | The residual signal itself — `f_res/m`, the thesis's core measurement. |
| `a_hat_res_rms`, `a_hat_vs_a_res_rmse` | The learned model's own predicted magnitude, and how well it matches the measured residual. **This is NaN whenever `rnn_pred_*` is not in the log — never silently 0**, which would read as "the model predicts nothing" instead of "the model was not asked." |
| `e_R_rmse`, `e_R_peak` | RMS and peak of `‖e_R‖`, the geometric attitude error. There is no "commanded e_R" — it is already an error signal — so this is a plain RMS/peak over the flight window, not an error-against-a-setpoint. **NaN whenever the log predates the 2026-09-08 e_R logging addition**, e.g. every fixture and every flight from before that date. |

**NaN ≠ 0, everywhere in this script.** A NaN residual/prediction/attitude-error column
means the signal was not recorded in that log — not that it measured zero. Only real
zero-valued samples produce a real 0.

**Sim files are not results.** Every `sim`/`merged`-format row is exactly what
`--source` says it is: a simulation. Simulation numbers validate the pipeline (this is
what `test_metrics.py`'s end-to-end test does); they are not a substitute for the C.0
hardware-validation flights, and the `report.md` header says so on every run.

## Tests

```bash
python3 experiments/analysis/test_metrics.py
```

Six checks: zero RMSE when position exactly matches the commanded trajectory, a known
3 cm z-offset reproduced exactly as `pos_rmse_z`, a known 3 cm sag reproduced exactly as
`dz_err_mean_m`, missing `a_hat`/`e_R`/`a_res` coming out as NaN rather than 0, a
zero-row `ros`-format CSV handled cleanly instead of crashing, and the real sim fixture
(`fixtures/merged_sim_snippet.csv`, a trimmed 200-row slice of
`experiments/sim_validation/residual_collect.csv`) running through the full CLI to
confirm it exits 0 and writes all three output files.

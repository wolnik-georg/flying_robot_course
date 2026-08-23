# Thesis Experiments

Working area for the thesis's own experimental work, separate from the archived course-phase
material in `archive/old_logs/` and `archive/Controls/`.

| Folder | Contents |
|---|---|
| `logs/` | Raw flight logs. `run_formation` writes a per-robot CSV **plus a `*.meta.json` sidecar** holding the effective scenario parameters |
| `analysis/` | Analysis and harness scripts — see below |
| `sim_validation/` | Simulator results: the 34-case formation matrix and the residual dry run |
| `protocols/` | Per-session run checklists |

---

## Analysis and harness scripts

### Formation validation in simulation

| Script | What it does |
|---|---|
| `run_sim_matrix.sh` | The 34-case matrix: every scenario × both controllers. `all`, or `one geo 2 --scenario A3 --dz 0.30` |
| `verify_formation_sim.py` | Scores one run: realised geometry vs what the scenario commanded. **Use this, not the runner's own report** — the runner's report and `/pose` do not work in simulation |
| `rerun_failures.sh` | Re-runs only the cases that failed, from `matrix_results.md` |
| `summarise_matrix.py` | Completeness audit over the results table; knows the earned `EXPECTED` verdict |
| `sim_matrix_cases.txt` | The case list the matrix iterates |

### Residual learning

| Script | What it does |
|---|---|
| `run_residual_dryrun.sh` | The full 4-phase dry run: collect → train → predict → compensate. ~13 min |
| `analyse_residual_dryrun.py` | Compares the phases and writes `sim_validation/RESIDUAL_DRYRUN.md` |
| `probe_residual_sign.py` | Applies a known disturbance and checks `a_res` has the right sign. This is what caught position INDI reinforcing disturbances at exactly 2.00× |
| `analyze_formation.py` | Post-flight formation analysis, including `a_res` |

The training pipeline itself lives in `flying_drone_stack/tools/residual/`.

---

## Two things that will waste your time if you don't know them

**Pair a run with its own sidecar.** Taking the newest `*.meta.json` unconditionally is wrong: if
a run dies before writing one, the *previous* run's file is still newest, and the flight gets
silently scored against a different scenario's commanded geometry. That produced a
plausible-looking but meaningless number once. `run_sim_matrix.sh` uses a pre-run marker and
`find -newer`.

**`a_res` reading exactly 0.0 is not a measurement.** It means no RPM source reached the
controller. A dataset of zeros trains a network that predicts nothing and looks like it converged
beautifully. The loaders drop and count those samples; check the count.

---

## Which Python

| Use | Interpreter |
|---|---|
| Anything with torch or the simulator bindings — the residual pipeline, SIL tests | **system `python3`** (3.10) |
| Older cflib flight/analysis scripts | `~/.pyenv/versions/flying_robots/bin/python` |

The pyenv environment has neither torch nor `cffirmware`; the system one has both.

---

See [`../docs/`](../docs/) for the protocols and planning documents this folder supports —
start at [`../docs/07_Thesis_Progress_Checklist.md`](../docs/07_Thesis_Progress_Checklist.md).

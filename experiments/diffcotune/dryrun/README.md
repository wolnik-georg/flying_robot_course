# Stage 3: S0 dry run (simulator to simulator)

`loop_lib.py` the loop (flights in frsim, fit, tune, Adam, stop rule, evaluation), `run_dryrun.py` (modes `pilot`, `full`, `eval`), `report.py` (tables + plots, run with
`~/.pyenv/versions/flying_robots/bin/python`), `diag_fit.py` / `diag_fit3.py` (diagnostics, not the method), `smoke.py` (one-epoch smoke test).
Results: `results/*.json`, logs `logs/`, tables `report_stage3.md`, plots `beta_P*.png`, `cost_P*.png`, `theta_P*.png`, `adam_weak_P*.png`.
Interpretation and the spec changes: `docs/diffcotune_spec.md` section 9. Note: the `full` runs of 11 Nov crashed at their final evaluation step (arguments swapped,
fixed afterwards); the method results were saved first and the evaluation was run with `run_dryrun.py eval`.

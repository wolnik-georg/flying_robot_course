# Sim formation validation

Output of `experiments/analysis/run_sim_matrix.sh`.

- `matrix_results.md` — one row per run: scenario, controller, robot count, verdict,
  mean |e_z|, RMSE, max tilt, divergence flag, how much of the trajectory the recording
  covered, parameters, and the `record_states` directory the numbers came from.
- `run_*.log` / `client_*.log` — server and client output from the most recent run of each
  controller. Overwritten each run; copy anything worth keeping.

Reproduce a single case:

```bash
experiments/analysis/run_sim_matrix.sh one geo 2 --scenario A3 --dz 0.30
```

Verify an existing run by hand:

```bash
python experiments/analysis/verify_formation_sim.py \
    experiments/logs/A3_<stamp>.meta.json  ~/Desktop/crazyswarm2/state_geo/<ts>/csv \
    --controller geo
```

Pass criteria are stated in `docs/12_Sim_Formation_Validation_Report.md`.

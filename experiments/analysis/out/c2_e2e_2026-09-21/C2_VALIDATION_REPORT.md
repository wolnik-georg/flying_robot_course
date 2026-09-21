# C.2 E2E validation (2026-09-21 C.1 data)

Four usable flights (A1 dz 0.75 + A3×3). A1_13-25-10 contributes **0 rows**.

## Stage B — LOO (headline)

- **A1_12-51-16**: RMSE 5.658 vs predict-zero 1.789 (-216.3%)
- **A3_13-00-57**: RMSE 0.224 vs predict-zero 0.770 (70.9%)
- **A3_13-02-56**: RMSE 0.293 vs predict-zero 0.757 (61.3%)
- **A3_13-04-34**: RMSE 0.498 vs predict-zero 0.740 (32.7%)

## Stage B — cross-scenario (headline)

- **Train A1 only → test A3_13-00-57**: RMSE 8.92 vs predict-zero 0.770 (-1058% — ~**1058% worse than predicting zero**)
- **Train A1 only → test A3_13-02-56**: RMSE 8.69 vs predict-zero 0.757 (-1048% — ~**1048% worse than predicting zero**)
- **Train A1 only → test A3_13-04-34**: RMSE 10.44 vs predict-zero 0.740 (-1310% — ~**1310% worse than predicting zero**)
- **Train A3 only → test A1_12-51-16**: RMSE 5.66 vs predict-zero 1.789 (-216%)

## Stage C — loader↔firmware: PASS

- max |NumPy−compiled| = 6.55e-07 m/s² (position / zero peer dv path); max |NumPy−compiled| = 5.95e-06 m/s² (differenced peer dv)
- *Scope:* Path A: loader positions, NumPy peer dv forced to 0 (original check). Path B: measured rel velocity via differenced oot_set_peer (100 ms). Ego velocity in Path A is 0 in the harness — see test_real_data_pipeline.py.

## Stage D — physical plausibility

- Synthetic overhead sweep (LOO A3 weights): |a| larger at small dz = True; corr(|a|, dz) = -0.960
- Held-out binned error: see JSON `stage_d.held_out_binned_error`.

## Stage E — SIL gate: PASS (go/no-fly only)

- After crazyswarm2 SIL fix (`docs/38`), full dry-run ~8.4 min, exit 0.
- Real flight in sim (z ~1.0 m / ~1.3 m); predict/compensate ~5200 rows each, no divergence.
- Sim `{name}/state` and `{name}/pose` are both **ground truth** — EKF gate is vacuous in sim; hardware gate unchanged.
- **Not** evidence the residual model is good — only closed-loop stability with weights loaded.
- Artifacts: `experiments/sim_validation/c2_e2e_stage_e.json`, `c2_e2e_*.csv`.

Full JSON: `c2_validation_report.json`. Plan: `docs/40_C2_Residual_Pipeline_E2E_Validation_Plan.md`.

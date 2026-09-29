# RPM source quality — C.1 merged uSD logs

Compares optical-deck RPM (`rpm.m*`) vs DShot telemetry (`motor.m*_rpm`) at **500 Hz**. Control uses DShot (`indi_gains.rpm_source=1`) for **reliability** (deck motor dropouts), not because DShot is lower-latency — **positive lag ms = DShot lags the deck**.

## Reproduce

```bash
cd ~/Desktop/flying_robot_course
python3 experiments/analysis/rpm_source_quality.py
# if matplotlib/numpy clash:
python3 experiments/analysis/rpm_source_quality.py --no-plots
```

Outputs (overwritten each run):

| File | Description |
|------|-------------|
| `per_flight.csv` | Per motor-row metrics incl. **rmse_robust_rpm** (headline) and **rmse_raw_rpm** (diagnostic) |
| `fleet_robust_rmse.json` | Fleet median/mean robust vs raw RMSE |
| `spike_investigation.json` | Spike root-cause + control-path correlation summary |
| `summary_by_scenario_vehicle.csv` | By scenario × vehicle role |
| `overview_lag_bias_by_role.png` | Optional (skip with `--no-plots`) |

Metric definitions: `docs/43_RPM_Source_Quality.md` § Metric reference.

Write-up: [`docs/43_RPM_Source_Quality.md`](../../../docs/43_RPM_Source_Quality.md).

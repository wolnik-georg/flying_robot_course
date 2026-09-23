# RPM source quality — C.1 merged uSD logs

Compares optical-deck RPM (`rpm.m*`) vs DShot telemetry (`motor.m*_rpm`) logged concurrently at **500 Hz** on C.1 flights. Control uses DShot (`indi_gains.rpm_source=1`); this output is **sensor agreement only**.

## Reproduce

```bash
cd ~/Desktop/flying_robot_course
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/rpm_source_quality.py
```

Outputs (overwritten each run):

| File | Description |
|------|-------------|
| `per_flight.csv` | One row per scenario / stamp / vehicle prefix / motor |
| `summary_by_scenario_vehicle.csv` | Aggregated by scenario and vehicle role (`bottom` / `top`) |
| `overview_lag_bias_by_role.png` | Boxplots of lag and \|bias\| by role |

**Input:** only `experiments/logs/c1_*_merged/*/*_merged_usd.csv` (not raw `.bin` — see `experiments/logs/usd_raw/*_PAIRING.md`).

**Lag convention:** positive ms = DShot lags deck (`cross_corr_lag` from `flying_drone_stack/tools/investigate_dshot_rpm.py`).

Write-up: [`docs/43_RPM_Source_Quality.md`](../../../docs/43_RPM_Source_Quality.md).

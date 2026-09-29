# RPM source quality — C.1 merged uSD logs

Compares optical-deck RPM (`rpm.m*`) vs DShot telemetry (`motor.m*_rpm`) at **500 Hz**. Control uses DShot (`indi_gains.rpm_source=1`) for **reliability** (deck motor dropouts), not because DShot is lower-latency — **positive lag ms = DShot lags the deck**.

**Headline metric:** `rmse_robust_rpm` (spike-filtered). **`rmse_raw_rpm`** is diagnostic only (with spikes). Spikes are **DShot-only**; root cause is a **best-fit decode-glitch hypothesis, not confirmed**. See `docs/43_RPM_Source_Quality.md`.

## Reproduce

**Full run (plots + CSVs)** — use pyenv env if system `python3` breaks matplotlib:

```bash
cd ~/Desktop/flying_robot_course
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/rpm_source_quality.py
```

**Metrics / tables only** (no matplotlib):

```bash
python3 experiments/analysis/rpm_source_quality.py --no-plots
```

Dataset: all `experiments/logs/c1_*_merged/*/*_merged_usd.csv` (**38** merges incl. 2026-09-28).

Outputs (overwritten each run):

| File | Description |
|------|-------------|
| `per_flight.csv` | Per motor-row metrics — **rmse_robust_rpm** (headline), **rmse_raw_rpm** (diagnostic) |
| `fleet_robust_rmse.json` | Fleet median/mean robust vs raw RMSE |
| `spike_investigation.json` | Spike counts + control-path correlation summary |
| `summary_by_scenario_vehicle.csv` | By scenario × vehicle role (robust-first columns) |
| `flight_summary_table.md` | Per-flight robust headline table |
| `overview_*.png`, `overlay_*.png`, `grid4_*.png`, `rolling_lag_*.png` | Regenerated figures (spike markers on traces; rolling lag uses robust mask) |

Metric definitions: `docs/43_RPM_Source_Quality.md` § Metric reference.

Write-up: [`docs/43_RPM_Source_Quality.md`](../../../docs/43_RPM_Source_Quality.md).

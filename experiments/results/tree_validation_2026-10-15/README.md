# Tree validation 2026-10-15 — plot-ready extract
One CSV per flight (`tree_<S0..S3>_<A8|A1>_<hhmm>.csv`), 100 Hz, time `t` = seconds on the commanded-trajectory alignment, `bottom_*` = cf5 (carries the strategy), `top_*` = cf_second (always S0). Channels: position/velocity, commanded target, roll/pitch (deg), NS2 prediction z (m/s^2), Omar a_rpm_f / a_imu_f z, measured residual a_res_z, battery.
`metrics.json`: per-flight numbers (z error mean/std, xy rms, attitude max, separation error, A8 crossing dip / upward peak / offset). Definitions: `experiments/analysis/tree_validation_analysis.py`.
Flights used: outliers (battery / logging cut-offs) already excluded, see docs/lab_sessions/2026-10-15_tree_validation_coverage.md. Raw uSD logs: experiments/logs/usd_raw/*tree_*.

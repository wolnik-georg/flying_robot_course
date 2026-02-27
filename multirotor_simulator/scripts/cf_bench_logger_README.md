Crazyflie bench logger (minimal)

Goal
- Produce a timestamped CSV suitable for `scripts/bench_motor_id.py`:
  columns: time_s, cmd, measured_thrust_N (optional), imu_z (optional), height (optional)

Modes
- dry-run: generates synthetic first-order response CSV for testing the analysis pipeline.
- live (Crazyflie): uses `cflib` to connect and send setpoints; logs IMU/range if available.
- load-cell: if you have a load cell connected to the PC, provide `--loadcell-port /dev/ttyUSB0` and the script will attempt to read ASCII lines with either `thrust` or `timestamp,thrust` format.

Quick start (dry run):

```bash
python3 scripts/cf_bench_logger.py --dry-run --out results/bench_step_dry.csv --cmd0 0.2 --cmd1 0.6 --hold 1.0 --step 3.0 --dt 0.005
```

Quick start (real Crazyflie):

```bash
# install cflib: pip install cflib
python3 scripts/cf_bench_logger.py --uri radio://0/80/2M --out results/bench_step_cf.csv --cmd0 0.2 --cmd1 0.6 --hold 1.0 --step 3.0 --dt 0.005
```

Notes
- Props OFF for bench testing.
- If using load cell, ensure its output is ASCII numeric values per line or `t,value` lines.
- If you want, I can adapt this to publish motor PWMs or ESC telemetry instead of the high-level thrust command depending on your flight stack.

Bench motor identification (minimal)

Purpose
- Record a simple step response of a single motor (props OFF) and fit a first-order time constant tau.
- Recommend a conservative thrust-rate-limit R (N/s) for controller per-step updates.

Record procedure (quick):
1. Bench setup: motor mounted, props off. Thrust measured with a scale or load cell aligned vertically.
2. Start high-rate logging on the flight stack or a separate data logger. Aim for >=200 Hz if possible.
3. Command sequence (examples):
   - Hold a baseline command (cmd0) for 1 s (e.g., hover throttle or 0.2 PWM), then step to cmd1 for 2-5 s, then hold baseline again.
   - Save CSV with columns: time_s, cmd, measured_thrust_N
   - Ensure times are in seconds and monotonically increasing.

Run the analysis script:

```bash
python3 scripts/bench_motor_id.py path/to/step_response.csv --dt 0.005 --plot --out results/bench_motor_id_summary.json
```

Outputs
- Printed: tau (s), steady-state gain K (N/command), empirical max slope (N/s), recommended R (N/s), recommended delta per controller step.
- Optional JSON summary if `--out` is provided.

Notes and interpretation
- Use the recommended R as a starting point for `thrust_rate_limit` in `assignment5` and for safe motor commands on hardware.
- If measured thrust is noisy, increase logging duration and repeat the test; use a larger step to improve SNR.
- If no measured thrust is available, run an open-loop PWM step and record motor RPM (if possible). The script expects measured thrust for accurate R in N/s.

Safety
- Props OFF while bench testing. If you must test with props installed, use a tether and follow a strict kill-switch procedure.

If you want, I can add an automated logger script for the Crazyflie that records timestamp/cmd/thrust to CSV (requires specifying how you plan to measure thrust).
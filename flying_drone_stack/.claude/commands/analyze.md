Run a complete post-flight analysis on a CSV file.

If `$ARGUMENTS` is given, use that CSV path. Otherwise find the latest CSV:
```bash
ls -t runs/*.csv | head -1
```

Run all three analysis steps in sequence:

**1. MEKF evaluation:**
```bash
cargo run --release --bin mekf_eval -- <CSV>
```

**2. SLAM evaluation:**
```bash
cargo run --release --bin slam_eval -- <CSV>
```

**3. Diagnostic plot:**
```bash
~/.pyenv/versions/flying_robots/bin/python scripts/plot_flight_diagnostic.py <CSV>
```

Summarise:
- MEKF RMSE vs firmware EKF (roll/pitch/yaw/x/y/z)
- Number of keyframes, loop closures, max pose graph correction
- Whether `vo_x/vo_y` were non-zero (camera was contributing)
- Whether `lc_count` incremented (loop closure fired)
- Any anomalies (large attitude jumps, EKF resets, battery dips)

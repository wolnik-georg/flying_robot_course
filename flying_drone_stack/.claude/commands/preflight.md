Show the pre-flight checklist and verify the development environment is ready.

Print this checklist and check each item where possible:

## Hardware checklist
- [ ] Battery ≥ 3.7 V (check with multimeter or cfclient)
- [ ] Textured paper/cardboard under drone (PMW3901 needs texture)
- [ ] Crazyradio PA plugged in (`lsusb | grep -i bitcraze` should show device)
- [ ] Multi-ranger Deck attached (required — safety repulsion uses it)
- [ ] Clear 1.5 × 1.5 m flight area, ceiling > 0.6 m
- [ ] If using `--ai-deck`: laptop WiFi → "WiFi streaming example" AP, drone powered ≥ 15 s

## Software checklist
Run these checks:
```bash
# 1. Build is clean
cargo build --release 2>&1 | tail -3

# 2. Tests pass
cargo test --quiet 2>&1 | tail -3

# 3. Crazyradio visible
lsusb | grep -i bitcraze || echo "WARNING: Crazyradio not found"

# 4. Latest CSV (for reference)
ls -t runs/*.csv 2>/dev/null | head -3 || echo "No previous flights"
```

## Flight mode reminder
| Mode | Command | Needs OOT firmware? |
|------|---------|-------------------|
| A — hover/circle/figure8/explore | `cargo run --release --bin main -- --maneuver circle` | No (uses firmware PID) |
| B — spline full-state | `cargo run --release --bin spline_circle_test` | **Yes** |
| C — HLC Python | `~/.pyenv/versions/flying_robots/bin/python Controls/run_spline_circle.py` | **Yes** |

Flash firmware if needed: `/flash`

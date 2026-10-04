# Delay budget snapshot @ 6.3 Hz

Motor lag: **τ=44 ms** (bench, investigation §16).

| Variant | Σ phase (deg) | Σ equiv. delay (ms) | Confidence |
|---|---:|---:|---|
| Stock Bitcraze INDI (ctrl=3) | 119.1 | 52.51 | MEDIUM (overlapping terms) |
| Ours OOT (ctrl=6, ctrl_mode=3) | 113.3 | 49.94 | MEDIUM (overlapping terms) |
| Omar C (ctrl=9) | 129.3 | 57.01 | MEDIUM (overlapping terms) |
| Omar Rust (ctrl=10) | 129.3 | 57.01 | MEDIUM (overlapping terms) |
| NA-INDI port (ctrl=7/8, ref ctrl=7/8) | 124.8 | 55.04 | MEDIUM (overlapping terms) |

_Full term breakdown: `delay_budget_6p3hz.json`_

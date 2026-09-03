# simple_flight.py validation matrix — final, 2026-09-03

8/8 PASS, staged-takeoff fix confirmed (docs/07 2026-09-03 (7)-(9)). Every 2-drone
separation is a real, positive number -- no near-zero climb-together window anywhere.

| Trajectory | N | Verdict | Exit | Notes |
|---|---|---|---|---|
| figure8 | 1 | PASS | 0 | clean, no verification sidecar (script has none by design) |
| figure8 | 2 | PASS | 0 | min in-flight inter-drone separation: 0.493 m |
| circle | 1 | PASS | 0 | clean, no verification sidecar (script has none by design) |
| circle | 2 | PASS | 0 | min in-flight inter-drone separation: 0.197 m |
| oval | 1 | PASS | 0 | clean, no verification sidecar (script has none by design) |
| oval | 2 | PASS | 0 | min in-flight inter-drone separation: 0.481 m |
| hover | 1 | PASS | 0 | clean, no verification sidecar (script has none by design) |
| hover | 2 | PASS | 0 | min in-flight inter-drone separation: 0.494 m |

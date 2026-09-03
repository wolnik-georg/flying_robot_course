| Trajectory | N | Verdict | Exit | Notes |
|---|---|---|---|---|
| figure8 | 1 | PASS | 0 | clean, no verification sidecar (script has none by design) |
| figure8 | 2 | PASS | 0 | clean at 220s client timeout (150s timed out mid-cleanup, flight itself completed either way) |
| circle | 1 | PASS | 0 | clean, no verification sidecar (script has none by design) |
| circle | 2 | PASS | 0 | clean at 220s client timeout (150s timed out mid-cleanup, flight itself completed either way) |
| oval | 1 | PASS | 0 | clean, no verification sidecar (script has none by design) |
| oval | 2 | PASS | 0 | clean at 220s client timeout (150s timed out mid-cleanup, flight itself completed either way) |
| hover | 1 | PASS | 0 | clean, no verification sidecar (script has none by design) |
| hover | 2 | PASS | 0 | clean, no verification sidecar (script has none by design) |

## Multi-drone safety check (added after the matrix, see docs/07 2026-09-03 (7))

The matrix's own "min inter-drone separation" check never fired due to the same
`find -newer $MARKER` flakiness documented in `run_formation_flight_matrix.sh` -- verified
manually instead, against the 3 re-run 2-drone `state_geo/` directories directly.

**Result: min separation 0.000-0.001 m during takeoff, all 3 cases checked** (figure8,
circle, oval). Root cause: `allcfs.takeoff(targetHeight=args.height)` is one broadcast to
the SAME height for every drone; only the later per-drone `goTo` sends each drone to its
real distinct target height. Two drones sharing similar (x,y) in the roster climb on top
of each other before ever separating. See the WARNING in `simple_flight.py`'s own
docstring (added 2026-09-03) and the full trace in the conversation/History log.

**Verdict: single-drone use is validated and safe (8/8 clean above). Multi-drone use is
NOT safe as written -- confirmed by data.**

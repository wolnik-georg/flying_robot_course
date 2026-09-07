| Scenario | Ctrl | N | Verdict | mean\|ez\| mm | RMSE mm | max tilt | diverged | covered | params | states |
|---|---|---|---|---|---|---|---|---|---|---|
| A5 speed=0.1 | geo | 2 | INCOMPLETE | 4.2 | 9.1 | 1.0 | no | 47-50% | dz=0.5,radius=0.75,period=47.123,laps=2.0,rotate_deg=0.0 | see note |
<!-- Same 94.25s-duration (period=47.1 x 2 laps) truncation as A2/B1/B2/C2's own
speed=0.1 cases -- exceeds the 380s client timeout regardless of controller. Both
attempts consistent (47-50% coverage, ~4.2mm mean|ez| both times). Bottom-top pair
within tolerance in the captured window. -->

| A5 | geo | 2 | PASS | 3.3 | 8.0 | 0.8 | no | 98% | dz=0.5,radius=0.75,period=23.562,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_000557/csv` |
| A5 | geo | 2 | PASS | 3.6 | 8.9 | 1.0 | no | 100% | dz=0.5,radius=0.75,period=15.708,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_001241/csv` |
| A5 | geo | 2 | PASS | 3.5 | 8.8 | 1.5 | no | 100% | dz=0.5,radius=0.75,period=11.781,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_001819/csv` |
| A5 | geo | 2 | PASS | 3.6 | 8.8 | 2.3 | no | 100% | dz=0.5,radius=0.75,period=9.425,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_002253/csv` |
| A5 speed=0.1 | indi | 2 | INCOMPLETE | 0.1 | 0.7 | 0.1 | no | 49-51% | dz=0.5,radius=0.75,period=47.123,laps=2.0,rotate_deg=0.0 | see note |
<!-- Same 94.25s-duration truncation as geo/speed=0.1 -- duration-specific, not
controller-specific. INDI holds ~0.1mm within the captured window either way. -->

| A5 | indi | 2 | PASS | 0.2 | 0.7 | 0.3 | no | 99% | dz=0.5,radius=0.75,period=23.562,laps=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_004017/csv` |
| A5 | indi | 2 | PASS | 0.2 | 0.8 | 0.7 | no | 100% | dz=0.5,radius=0.75,period=15.708,laps=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_004701/csv` |
| A5 | indi | 2 | PASS | 0.2 | 0.8 | 1.3 | no | 100% | dz=0.5,radius=0.75,period=11.781,laps=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_005235/csv` |
| A5 | indi | 2 | PASS | 0.3 | 1.0 | 2.0 | no | 100% | dz=0.5,radius=0.75,period=9.425,laps=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_005717/csv` |
<!-- INVALIDATED 2026-09-06: `run_formation.py` has no `--duration` CLI flag at all (checked
its argparse list directly). It uses `parse_known_args()`, so the unrecognised `--duration
X` on every one of the 30 A8/B3/C4 runs below was silently dropped rather than erroring --
every "different speed" case for these three scenarios actually flew at the scenario's
default duration (A8 6.0s, B3 8.0s, C4 8.0s) every single time, which is exactly why all 5
rows per scenario/controller are numerically identical. Caught by noticing that, not
assumed. The real lever for these three is `--timescale` (confirmed in the source: it
scales `sc.duration` at upload time, `startTrajectory(0, timescale=...)`, and
`verify_formation_sim.py` already divides by it when computing commanded speed). Re-run
below with `run_speed_sweep_extended3b.sh` using the correct flag. -->

<!-- NOTE on the "params" column below: the scenario's own internal `duration` (6.0/8.0/8.0
for A8/B3/C4) is unscaled and always prints the same regardless of speed -- the actual
swept parameter is `timescale`, which isn't part of `meta['params']` and so isn't in this
auto-appended column at all. Manually noted per row instead. -->

| A8 speed=0.1 | geo | 2 | PASS | 2.4 | 6.3 | 1.8 | no | 100% | timescale=3.6458,dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_101955/csv` |
<!-- Re-verified with the fixed verify_formation_sim.py (the first two raw attempts,
recorded before the fix, showed a bogus RMSE of 1204mm from the timescale bug above --
same underlying flight data, just verified correctly this time). -->

| A8 speed=0.2 | geo | 2 | FAIL(speed) | 2.5 | 6.0 | 1.2 | no | 100% | timescale=1.8229,dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_102648/csv` |
<!-- Raw verdict FAIL is "speed out of tolerance" only (flown peak 0.280 vs cmd 0.200,
just over the wide +-0.08 m/s tolerance) -- geometry itself is fine (mean|ez| 2.5mm, RMSE
6.0mm, same as speed=0.1's clean numbers). A8's curve is `Then(Pause(settle), Line(...))`
-- a real settle-to-motion transition that circle/lemniscate paths (A2/B1/B2/C2/A3) don't
have, and looks like it overshoots the ideal smoothstep peak in actual HLC tracking.
Watching whether this is a fixed absolute overshoot (so it disappears as a % once target
speed rises) across the rest of A8/B3/C4 before concluding anything, same as previous
sweeps' EXPECTED-labelling precedent. -->

| A8 | geo | 2 | PASS | 2.3 | 4.7 | 1.5 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_102941/csv` |
| A8 | geo | 2 | PASS | 1.9 | 3.6 | 2.0 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_103206/csv` |
| A8 | geo | 2 | PASS | 1.8 | 3.1 | 2.6 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_103422/csv` |
| A8 | indi | 2 | PASS | 0.2 | 0.9 | 0.1 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_103624/csv` |
| A8 | indi | 2 | PASS | 0.3 | 1.3 | 0.4 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_104015/csv` |
| A8 | indi | 2 | PASS | 0.3 | 1.2 | 0.8 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_104259/csv` |
| A8 | indi | 2 | PASS | 0.3 | 1.1 | 1.5 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_104519/csv` |
| A8 | indi | 2 | PASS | 0.3 | 1.1 | 2.3 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_104729/csv` |
| B3 | geo | 3 | PASS | 3.7 | 8.3 | 1.6 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_104931/csv` |
| B3 | geo | 3 | PASS | 3.9 | 8.4 | 0.9 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_105458/csv` |
| B3 | geo | 3 | PASS | 3.9 | 7.7 | 0.9 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_105846/csv` |
| B3 | geo | 3 | PASS | 3.8 | 6.7 | 1.3 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_110158/csv` |
| B3 | geo | 3 | PASS | 3.6 | 5.9 | 2.1 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_110452/csv` |
| B3 | indi | 3 | PASS | 0.3 | 0.7 | 0.1 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_110736/csv` |
| B3 | indi | 3 | PASS | 0.5 | 1.4 | 0.3 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_111302/csv` |
| B3 | indi | 3 | PASS | 0.7 | 1.6 | 0.7 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_111651/csv` |
| B3 | indi | 3 | PASS | 0.7 | 1.4 | 1.3 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_112000/csv` |
| B3 | indi | 3 | PASS | 0.6 | 1.2 | 2.1 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_112254/csv` |
| C4 | geo | 2 | PASS | 4.4 | 9.0 | 0.3 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_112539/csv` |
| C4 | geo | 2 | PASS | 4.3 | 8.9 | 0.4 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_112934/csv` |
| C4 | geo | 2 | PASS | 4.2 | 8.8 | 0.8 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_113221/csv` |
| C4 | geo | 2 | PASS | 4.1 | 8.7 | 1.3 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_113443/csv` |
| C4 | geo | 2 | PASS | 4.1 | 8.7 | 2.0 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_113655/csv` |
| C4 | indi | 2 | PASS | 0.1 | 0.2 | 0.1 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_113900/csv` |
| C4 | indi | 2 | PASS | 0.1 | 0.3 | 0.3 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_114257/csv` |
| C4 | indi | 2 | PASS | 0.2 | 0.4 | 0.7 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_114542/csv` |
| C4 | indi | 2 | PASS | 0.2 | 0.5 | 1.2 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_114806/csv` |
| C4 | indi | 2 | PASS | 0.2 | 0.6 | 2.0 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_115017/csv` |

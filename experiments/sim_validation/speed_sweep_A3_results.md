| Scenario | Ctrl | N | Verdict | mean\|ez\| mm | RMSE mm | max tilt | diverged | covered | params | states |
|---|---|---|---|---|---|---|---|---|---|---|
| A3 speed=0.1 | geo | 2 | PASS | 3.4 | 8.6 | 4.4 | no | 100% | dz=0.3,speed=0.1,length=1.2,passes=2 | `state_geo/2026-09-03_224855/csv` |
<!-- run_with_retry logged this as NO-DATA (client timed out at the outer 380s budget during
final landing cleanup -- same "flight completed, outer timeout cut off the tail end" pattern
seen elsewhere today). speed=0.1 gives the LONGEST duration in this sweep (52.5s vs much
shorter for faster speeds), pushing close to the budget. Verified manually against the real
meta.json/record_states from the first attempt: genuinely PASS, not a failure. Corrected here
by hand 2026-09-03. -->
| A3 speed=0.2 | geo | 2 | PASS | 3.3 | 8.3 | 1.5 | no | 100% | dz=0.3,speed=0.2,length=1.2,passes=2 | `state_geo/2026-09-03_230702/csv` |
<!-- Same find -newer $MARKER pairing flakiness as speed=0.1, but this time the flight
completed with NO traceback at all (full "landing"+report printed) -- confirms this is a
one_run() pairing bug in run_sim_matrix.sh itself, not a flight-duration timeout issue.
Same bug class already found and fixed in the two companion matrix scripts today
(run_formation_flight_matrix.sh, run_simple_flight_matrix.sh). Verified manually,
genuinely PASS. Worth fixing in run_sim_matrix.sh itself after this sweep finishes. -->

| A3 speed=0.3 | geo | 2 | PASS | 3.5 | 8.4 | 1.4 | no | 100% | dz=0.3,speed=0.3,length=1.2,passes=2 | `state_geo/2026-09-03_231514/csv` |
<!-- Same pairing bug as 0.1/0.2, verified manually, genuinely PASS. -->

| A3 speed=0.4 | geo | 2 | PASS | 3.5 | 8.0 | 2.0 | no | 100% | dz=0.3,speed=0.4,length=1.2,passes=2 | `state_geo/2026-09-03_232205/csv` |
<!-- Same pairing bug as 0.1/0.2/0.3, verified manually, genuinely PASS. -->

| A3 speed=0.5 | geo | 2 | PASS | 3.6 | 7.7 | 2.8 | no | 100% | dz=0.3,speed=0.5,length=1.2,passes=2 | `state_geo/2026-09-03_232810/csv` |
<!-- Same pairing bug as 0.1-0.4, verified manually, genuinely PASS. geo half complete: 5/5. -->

| A3 speed=0.1 | indi | 2 | PASS | 0.1 | 0.5 | 0.1 | no | 100% | dz=0.3,speed=0.1,length=1.2,passes=2 | `state_indi/2026-09-03_233748/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. -->

| A3 speed=0.2 | indi | 2 | PASS | 0.3 | 1.1 | 0.3 | no | 100% | dz=0.3,speed=0.2,length=1.2,passes=2 | `state_indi/2026-09-03_234858/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. -->

| A3 speed=0.3 | indi | 2 | PASS | 0.5 | 1.4 | 0.7 | no | 100% | dz=0.3,speed=0.3,length=1.2,passes=2 | `state_indi/2026-09-03_235655/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. -->

| A3 speed=0.4 | indi | 2 | PASS | 0.6 | 1.6 | 1.2 | no | 100% | dz=0.3,speed=0.4,length=1.2,passes=2 | `state_indi/2026-09-04_000340/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. -->

| A3 speed=0.5 | indi | 2 | PASS | 0.7 | 1.7 | 1.9 | no | 100% | dz=0.3,speed=0.5,length=1.2,passes=2 | `state_indi/2026-09-05_095908/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. Sweep complete: 10/10 PASS. -->


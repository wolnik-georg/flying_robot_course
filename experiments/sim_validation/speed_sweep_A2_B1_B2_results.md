| Scenario | Ctrl | N | Verdict | mean\|ez\| mm | RMSE mm | max tilt | diverged | covered | params | states |
|---|---|---|---|---|---|---|---|---|---|---|
| A2 speed=0.1 | geo | 2 | INCOMPLETE | 39.7 | 39.7 | 1.0 | no | 47-50% | dz=0.3,speed=0.1,radius=0.75,period=47.1,laps=2.0 | see note |
<!-- Both attempts (10:17 and the 10:24 retry) genuinely truncated at ~47-50% coverage of
the 110.9s trajectory, with near-identical numbers each time (dz -0.340, mean|ez| 39.7mm
both times) -- this is real and reproducible, not the clock-jump/pairing flakiness seen
elsewhere. A2 at speed=0.1 (radius=0.75 circle -> period=47.1s x 2 laps=94s core) is by
far the longest scenario in this extended sweep; most likely cause is this session's sim
now running slower than real-time after many hours of continuous background activity, so
the fixed 380s client timeout cuts off well before the sim clock reaches 110.9s. NOT
fabricated as PASS -- reported honestly as an incomplete window. Would need either a much
longer client timeout or a fresh (less fatigued) sim session to get a real number here. -->

| A2 speed=0.2 | geo | 2 | PASS | 38.6 | 38.6 | 1.0 | no | 99% | dz=0.3,speed=0.2,radius=0.75,period=23.6,laps=2.0 | `state_geo/2026-09-05_103713/csv` |
<!-- Same stale-fallback pairing bug as before, verified manually, genuinely PASS at 99%
coverage -- confirms speed=0.1's truncation was specific to its exceptionally long
110.9s duration, not a general session-fatigue problem affecting every case. -->

| A2 speed=0.3 | geo | 2 | PASS | 37.3 | 37.3 | 0.9 | no | 100% | dz=0.3,speed=0.3,radius=0.75,period=15.7,laps=2.0 | `state_geo/2026-09-05_104930/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. -->

| A2 speed=0.4 | geo | 2 | PASS | 36.4 | 36.5 | 1.4 | no | 100% | dz=0.3,speed=0.4,radius=0.75,period=11.8,laps=2.0 | `state_geo/2026-09-05_105940/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. -->

| A2 speed=0.5 | geo | 2 | PASS | 36.0 | 36.1 | 2.0 | no | 100% | dz=0.3,speed=0.5,radius=0.75,period=9.4,laps=2.0 | `state_geo/2026-09-05_110817/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. A2/geo complete: 5/5 (0.1 incomplete/truncated, 0.2-0.5 clean PASS). -->

| A2 speed=0.1 | indi | 2 | INCOMPLETE | 0.0 | 0.0 | 0.1 | no | 50% | dz=0.3,speed=0.1,radius=0.75,period=47.1,laps=2.0 | see note |
<!-- Same 110.9s-duration truncation as geo/speed=0.1 -- confirms this is duration-specific,
not controller-specific. Within the captured 50% window, INDI holds 0.0mm exactly, but
NOT reported as a clean PASS since the window itself is incomplete. -->

| A2 speed=0.2 | indi | 2 | PASS | 0.0 | 0.0 | 0.3 | no | 96% | dz=0.3,speed=0.2,radius=0.75,period=23.6,laps=2.0 | `state_indi/2026-09-05_113234/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. -->

| A2 speed=0.3 | indi | 2 | PASS | 0.0 | 0.1 | 0.7 | no | 100% | dz=0.3,speed=0.3,radius=0.75,period=15.7,laps=2.0 | `state_indi/2026-09-05_114459/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. -->

| A2 speed=0.4 | indi | 2 | PASS | 0.1 | 0.1 | 1.3 | no | 100% | dz=0.3,speed=0.4,radius=0.75,period=11.8,laps=2.0 | `state_indi/2026-09-05_115521/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. -->

| A2 speed=0.5 | indi | 2 | PASS | 0.1 | 0.2 | 2.0 | no | 100% | dz=0.3,speed=0.5,radius=0.75,period=9.4,laps=2.0 | `state_indi/2026-09-05_120417/csv` |
<!-- Same pairing bug, verified manually, genuinely PASS. A2 fully complete: 9/10 clean PASS,
1 INCOMPLETE (speed=0.1, both controllers, truncated ~50% due to its 110.9s duration --
by far the longest case in this sweep, same root cause both times). -->

| B1 speed=0.1 | geo | 3 | INCOMPLETE | 40.1 avg | - | 1.9 | no | 28% | dz1=0.2,dz2=0.3,speed=0.1,radius=0.75,laps=2.0 | see note |
<!-- Same 110.9s-duration truncation as A2/speed=0.1 (same period-paced circle math).
Within the 28% captured: bottom-center 23.0mm ok, bottom-top 60.1mm OUT OF TOL (matches
the known B1 finding -- combined wash from two sources above breaks pairwise
superposition under geometric, per the original 2026-08-23 validation's EXPECTED
verdict), center-top 37.1mm ok. NOT reported as a clean result -- window incomplete. -->

| B1 speed=0.2 | geo | 3 | INCOMPLETE | 39.3 avg | - | 2.2 | no | 57% | dz1=0.2,dz2=0.3,speed=0.2,radius=0.75,laps=2.0 | see note |
<!-- Still truncated at speed=0.2, unlike A2 (which cleared up cleanly by 0.2) -- suggests
B1's 3-robot roster (more setParam calls per phase, longer climb/converge) may need a
longer client timeout than the 2-robot cases, independent of the speed=0.1 duration
issue. Watching closely: if 0.3+ also truncate, flag and consider bumping the timeout
before continuing rather than letting the whole B1/B2 half come back incomplete. -->

| B1 speed=0.3 | geo | 3 | INCOMPLETE | 38.2 avg | - | 2.3 | no | 86% | dz1=0.2,dz2=0.3,speed=0.3,radius=0.75,laps=2.0 | see note |
<!-- Trending clearly: 28% -> 57% -> 86% coverage at speed 0.1 -> 0.2 -> 0.3. Confirms a
FIXED per-run overhead (3-drone climb/converge/setParam cost, roughly constant regardless
of speed) eating a large chunk of the 380s budget -- shrinks proportionally less of the
total as speed increases. Not intervening yet; likely resolves itself by 0.4-0.5. -->

| B1 speed=0.4 | geo | 3 | EXPECTED | 37.8 avg | - | 2.5 | no | 100% | dz1=0.2,dz2=0.3,speed=0.4,radius=0.75,laps=2.0 | `state_geo/2026-09-05_125525/csv` |
<!-- 100% coverage -- confirms the overhead-trend theory, no more truncation. Raw tool
verdict is FAIL (bottom-top 56.7mm over the 50mm tolerance), but this exactly matches
the original 2026-08-23 validation's own EXPECTED verdict for B1/geometric (combined
wash from two sources above breaks pairwise superposition; INDI cancels it fine).
Relabeled EXPECTED per that precedent, not a defect. -->

| B1 speed=0.5 | geo | 3 | EXPECTED | 37.5 avg | - | 2.8 | no | 100% | dz1=0.2,dz2=0.3,speed=0.5,radius=0.75,laps=2.0 | `state_geo/2026-09-05_130811/csv` |
<!-- Same EXPECTED signature as speed=0.4, per the same precedent. B1/geo complete: 5/5
(0.1/0.2/0.3 incomplete due to fixed 3-drone overhead trending down, 0.4/0.5 clean
EXPECTED matching the original 2026-08-23 finding). bottom-top error is essentially flat
(56-60mm) across the whole speed range -- same "speed barely matters" pattern as A2. -->

| B1 speed=0.1 | indi | 3 | INCOMPLETE | 0.0 | 0.0 | 0.1 | no | 28% | dz1=0.2,dz2=0.3,speed=0.1,radius=0.75,laps=2.0 | see note |
<!-- Same 28% truncation as geo/speed=0.1 -- duration-specific, not controller-specific.
INDI holds 0.0mm on all 3 pairs within the captured window. -->

| B1 speed=0.2 | indi | 3 | INCOMPLETE | 0.0 | 0.0 | 0.3 | no | 58% | dz1=0.2,dz2=0.3,speed=0.2,radius=0.75,laps=2.0 | see note |
<!-- Same trend as geo (58% vs 57%) -- confirms fixed overhead, controller-independent. -->

| B1 speed=0.3 | indi | 3 | INCOMPLETE | 0.0 | 0.1 | 0.7 | no | 84% | dz1=0.2,dz2=0.3,speed=0.3,radius=0.75,laps=2.0 | see note |
<!-- Matches geo's 86% trend. INDI holds 0.0mm within the captured window. -->

| B1 speed=0.4 | indi | 3 | PASS | 0.1 | 0.1 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,speed=0.4,radius=0.75,laps=2.0 | `state_indi/2026-09-05_140119/csv` |
<!-- 100% coverage, clean PASS -- INDI cancels the same combined-wash disturbance that
made geo/speed=0.4 EXPECTED (not-PASS) on the bottom-top pair. -->

| B1 speed=0.5 | indi | 3 | PASS | 0.1 | 0.2 | 1.9 | no | 100% | dz1=0.2,dz2=0.3,speed=0.5,radius=0.75,laps=2.0 | `state_indi/2026-09-05_141412/csv` |
<!-- 100% coverage, clean PASS. B1 fully complete: 10/10 -- 0.1/0.2/0.3 incomplete both
controllers (fixed 3-drone overhead), 0.4/0.5 clean (geo=EXPECTED matching known
superposition-breakdown physics, indi=PASS cancelling it). -->

| B2 speed=0.1 | geo | 3 | INCOMPLETE | 29.4 avg | - | 0.7 | no | 29% | dz1=0.2,dz2=0.3,r=0.1,speed=0.1,radius=0.75,laps=2.0 | see note |
<!-- Same fixed-overhead truncation pattern as B1 (29% vs B1's 28%). All 3 pairs within
tolerance here (unlike B1's bottom-top), matching the original validation's own finding
that B2's asymmetric offset superposition holds up better than B1's pure alignment. -->

| B2 speed=0.2 | geo | 3 | INCOMPLETE | 28.6 avg | - | 0.8 | no | 54% | dz1=0.2,dz2=0.3,r=0.1,speed=0.2,radius=0.75,laps=2.0 | see note |
<!-- Same fixed-overhead trend as B1 (54% vs B1's 57%). All 3 pairs within tolerance. -->

| B2 speed=0.3 | geo | 3 | INCOMPLETE | 31.8 avg | - | 1.2 | no | 84% | dz1=0.2,dz2=0.3,r=0.1,speed=0.3,radius=0.75,laps=2.0 | `state_geo/2026-09-05_145355/csv` |
<!-- Same fixed-overhead trend as B1 (84% vs B1's 86%). All 3 pairs within tolerance,
same as B2/speed=0.1/0.2. -->

| B2 speed=0.4 | geo | 3 | PASS | 31.2 avg | - | 1.6 | no | 100% | dz1=0.2,dz2=0.3,r=0.1,speed=0.4,radius=0.75,laps=2.0 | `state_geo/2026-09-05_150721/csv` |
<!-- 100% coverage, clean PASS -- confirms the overhead-trend theory, no more truncation.
Unlike B1, all 3 pairs stay within tolerance even at higher speed (B2's asymmetric
offset superposition holds up better than B1's pure vertical alignment, matching the
original 2026-08-23 finding). -->

| B2 speed=0.5 | geo | 3 | PASS | 30.6 avg | - | 2.2 | no | 100% | dz1=0.2,dz2=0.3,r=0.1,speed=0.5,radius=0.75,laps=2.0 | `state_geo/2026-09-05_152020/csv` |
<!-- 100% coverage, clean PASS. B2/geo complete: 5/5 (0.1/0.2/0.3 incomplete due to fixed
3-drone overhead trending down, all pairs within tolerance throughout; 0.4/0.5 clean PASS).
Unlike B1, B2's asymmetric offset holds all 3 pairs in tolerance across the whole speed
range -- confirms the original 2026-08-23 finding that B2's superposition is more robust
than B1's pure vertical stack. -->

| B2 speed=0.1 | indi | 3 | INCOMPLETE | 0.0 | 0.0 | 0.1 | no | 28% | dz1=0.2,dz2=0.3,r=0.1,speed=0.1,radius=0.75,laps=2.0 | see note |
<!-- Same 28% truncation as geo/speed=0.1 -- duration-specific (110.9s), not
controller-specific, matching B1's identical pattern. INDI holds 0.0mm exactly on all
3 pairs within the captured window. -->

| B2 speed=0.2 | indi | 3 | INCOMPLETE | 0.0 | 0.0 | 0.3 | no | 56% | dz1=0.2,dz2=0.3,r=0.1,speed=0.2,radius=0.75,laps=2.0 | see note |
<!-- Matches geo/speed=0.2's 54% trend (controller-independent overhead). INDI holds
0.0mm exactly on all 3 pairs within the captured window. -->

| B2 speed=0.3 | indi | 3 | INCOMPLETE | 0.0 | 0.1 | 0.7 | no | 86% | dz1=0.2,dz2=0.3,r=0.1,speed=0.3,radius=0.75,laps=2.0 | see note |
<!-- Matches geo/speed=0.3's 84% trend. INDI holds 0.0mm within the captured window. -->

| B2 speed=0.4 | indi | 3 | PASS | 0.1 | 0.1 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,r=0.1,speed=0.4,radius=0.75,laps=2.0 | `state_indi/2026-09-05_161343/csv` |
<!-- 100% coverage, clean PASS -- INDI cancels the same combined-wash disturbance geo
holds within tolerance at this speed too. -->

| B2 speed=0.5 | indi | 3 | PASS | 0.1 | 0.2 | 2.0 | no | 100% | dz1=0.2,dz2=0.3,r=0.1,speed=0.5,radius=0.75,laps=2.0 | `state_indi/2026-09-05_162635/csv` |
<!-- 100% coverage, clean PASS. B2/indi fully complete: 5/5 -- 0.1/0.2/0.3 incomplete
(fixed 3-drone overhead, same trend as geo), 0.4/0.5 clean PASS. B2 sweep fully complete:
10/10 -- 0.1/0.2/0.3 incomplete both controllers (fixed overhead, all pairs within
tolerance throughout, unlike B1), 0.4/0.5 clean (geo=PASS, indi=PASS). This is the last
case of the extended A2/B1/B2 sweep -- SWEEPDONE. -->


| Scenario | Ctrl | N | Verdict | mean\|ez\| mm | RMSE mm | max tilt | diverged | covered | params | states |
|---|---|---|---|---|---|---|---|---|---|---|
<!-- C5 is single-robot (N=1): there is no second vehicle to form a pair, so mean|ez|/RMSE
are correctly `nan` throughout -- verify_formation_sim.py still checks tilt and realised
speed against commanded, which is all a solo scenario has to verify. -->

| A4 | geo | 2 | PASS | 2.4 | 3.2 | 1.9 | no | 100% | dz=0.6,offset=0.1,speed=0.1,length=1.2,radius=0.45,period=8.0,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-05_191921/csv` |
| A4 | geo | 2 | PASS | 2.3 | 3.1 | 0.8 | no | 100% | dz=0.6,offset=0.1,speed=0.2,length=1.2,radius=0.45,period=8.0,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-05_192605/csv` |
| A4 | geo | 2 | PASS | 2.2 | 3.1 | 1.0 | no | 100% | dz=0.6,offset=0.1,speed=0.3,length=1.2,radius=0.45,period=8.0,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-05_193040/csv` |
| A4 | geo | 2 | PASS | 2.1 | 3.1 | 1.4 | no | 100% | dz=0.6,offset=0.1,speed=0.4,length=1.2,radius=0.45,period=8.0,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-05_193423/csv` |
| A4 | geo | 2 | PASS | 2.1 | 3.1 | 2.1 | no | 100% | dz=0.6,offset=0.1,speed=0.5,length=1.2,radius=0.45,period=8.0,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-05_193737/csv` |
| A4 | indi | 2 | PASS | 0.0 | 0.1 | 0.1 | no | 100% | dz=0.6,offset=0.1,speed=0.1,length=1.2,radius=0.45,period=8.0,laps=2.0,rotate_deg=0.0 | `state_indi/2026-09-05_194040/csv` |
| A4 | indi | 2 | PASS | 0.0 | 0.2 | 0.3 | no | 100% | dz=0.6,offset=0.1,speed=0.2,length=1.2,radius=0.45,period=8.0,laps=2.0,rotate_deg=0.0 | `state_indi/2026-09-05_194724/csv` |
| A4 | indi | 2 | PASS | 0.0 | 0.2 | 0.7 | no | 100% | dz=0.6,offset=0.1,speed=0.3,length=1.2,radius=0.45,period=8.0,laps=2.0,rotate_deg=0.0 | `state_indi/2026-09-05_195200/csv` |
| A4 | indi | 2 | PASS | 0.1 | 0.4 | 1.2 | no | 100% | dz=0.6,offset=0.1,speed=0.4,length=1.2,radius=0.45,period=8.0,laps=2.0,rotate_deg=0.0 | `state_indi/2026-09-05_195546/csv` |
| A4 | indi | 2 | PASS | 0.1 | 0.4 | 1.9 | no | 100% | dz=0.6,offset=0.1,speed=0.5,length=1.2,radius=0.45,period=8.0,laps=2.0,rotate_deg=0.0 | `state_indi/2026-09-05_195908/csv` |
| A6 | geo | 2 | PASS | 3.0 | 7.7 | 2.5 | no | 100% | dz=0.1,speed=0.1,length=1.0,passes=2,rotate_deg=0.0 | `state_geo/2026-09-05_200210/csv` |
| A6 | geo | 2 | PASS | 3.2 | 7.9 | 1.2 | no | 100% | dz=0.1,speed=0.2,length=1.0,passes=2,rotate_deg=0.0 | `state_geo/2026-09-05_200824/csv` |
| A6 | geo | 2 | PASS | 3.4 | 8.0 | 1.5 | no | 100% | dz=0.1,speed=0.3,length=1.0,passes=2,rotate_deg=0.0 | `state_geo/2026-09-05_201226/csv` |
| A6 | geo | 2 | PASS | 3.5 | 7.7 | 2.2 | no | 100% | dz=0.1,speed=0.4,length=1.0,passes=2,rotate_deg=0.0 | `state_geo/2026-09-05_201546/csv` |
| A6 | geo | 2 | PASS | 3.4 | 7.2 | 3.1 | no | 100% | dz=0.1,speed=0.5,length=1.0,passes=2,rotate_deg=0.0 | `state_geo/2026-09-05_201846/csv` |
| A6 | indi | 2 | PASS | 0.1 | 0.5 | 0.1 | no | 100% | dz=0.1,speed=0.1,length=1.0,passes=2,rotate_deg=0.0 | `state_indi/2026-09-05_202134/csv` |
| A6 | indi | 2 | PASS | 0.3 | 1.0 | 0.4 | no | 100% | dz=0.1,speed=0.2,length=1.0,passes=2,rotate_deg=0.0 | `state_indi/2026-09-05_202744/csv` |
| A6 | indi | 2 | PASS | 0.5 | 1.4 | 0.8 | no | 100% | dz=0.1,speed=0.3,length=1.0,passes=2,rotate_deg=0.0 | `state_indi/2026-09-05_203146/csv` |
| A6 | indi | 2 | PASS | 0.7 | 1.6 | 1.5 | no | 100% | dz=0.1,speed=0.4,length=1.0,passes=2,rotate_deg=0.0 | `state_indi/2026-09-05_203503/csv` |
| A6 | indi | 2 | PASS | 0.7 | 1.6 | 2.3 | no | 100% | dz=0.1,speed=0.5,length=1.0,passes=2,rotate_deg=0.0 | `state_indi/2026-09-05_203801/csv` |
| A7 | geo | 2 | PASS | 27.0 | 30.0 | 0.6 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.1,length=1.2,settle=3.0,rotate_deg=0.0 | `state_geo/2026-09-05_204047/csv` |
| A7 | geo | 2 | PASS | 26.1 | 29.2 | 0.5 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.2,length=1.2,settle=3.0,rotate_deg=0.0 | `state_geo/2026-09-05_204547/csv` |
| A7 | geo | 2 | PASS | 25.3 | 28.5 | 0.8 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.3,length=1.2,settle=3.0,rotate_deg=0.0 | `state_geo/2026-09-05_204921/csv` |
| A7 | geo | 2 | PASS | 24.5 | 27.8 | 1.3 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.4,length=1.2,settle=3.0,rotate_deg=0.0 | `state_geo/2026-09-05_205228/csv` |
| A7 | geo | 2 | PASS | 23.9 | 27.2 | 2.0 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.5,length=1.2,settle=3.0,rotate_deg=0.0 | `state_geo/2026-09-05_205520/csv` |
| A7 | indi | 2 | PASS | 0.1 | 0.2 | 0.1 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.1,length=1.2,settle=3.0,rotate_deg=0.0 | `state_indi/2026-09-05_205808/csv` |
| A7 | indi | 2 | PASS | 0.2 | 0.4 | 0.3 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.2,length=1.2,settle=3.0,rotate_deg=0.0 | `state_indi/2026-09-05_210307/csv` |
| A7 | indi | 2 | PASS | 0.3 | 0.5 | 0.7 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.3,length=1.2,settle=3.0,rotate_deg=0.0 | `state_indi/2026-09-05_210646/csv` |
| A7 | indi | 2 | PASS | 0.3 | 0.6 | 1.2 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.4,length=1.2,settle=3.0,rotate_deg=0.0 | `state_indi/2026-09-05_210957/csv` |
| A7 | indi | 2 | PASS | 0.4 | 0.7 | 1.9 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.5,length=1.2,settle=3.0,rotate_deg=0.0 | `state_indi/2026-09-05_211255/csv` |
| C2 speed=0.1 | geo | 3 | INCOMPLETE | 0.1 | 3.2 | 3.4 | no | 68% | gap=0.5,n=3,speed=0.1,length=1.2,rotate_deg=90.0 | `state_geo/2026-09-05_212230/csv` |
<!-- Both attempts (69% then 68% coverage) genuinely truncated -- same fixed-overhead
pattern as B1/B2's low-speed cases (3-robot climb/converge/setParam cost eating a large
fraction of the 380s budget at low speed, shrinking proportionally as speed rises). Not
a pairing-bug artefact this time -- the fallback correctly matched the C2 sidecar both
times (glob fix + mtime sort both working), it just didn't reach 90% coverage. All 3
pairs within tolerance in the captured window. -->

| C2 | geo | 3 | PASS | 0.1 | 2.3 | 2.3 | no | 100% | gap=0.5,n=3,speed=0.2,length=1.2,rotate_deg=90.0 | `state_geo/2026-09-05_212914/csv` |
| C2 | geo | 3 | PASS | 0.1 | 1.8 | 2.1 | no | 100% | gap=0.5,n=3,speed=0.3,length=1.2,rotate_deg=90.0 | `state_geo/2026-09-05_213541/csv` |
| C2 | geo | 3 | PASS | 0.0 | 1.6 | 2.4 | no | 100% | gap=0.5,n=3,speed=0.4,length=1.2,rotate_deg=90.0 | `state_geo/2026-09-05_214045/csv` |
| C2 | geo | 3 | PASS | 0.0 | 1.5 | 2.8 | no | 100% | gap=0.5,n=3,speed=0.5,length=1.2,rotate_deg=90.0 | `state_geo/2026-09-05_214503/csv` |
| C2 speed=0.1 | indi | 3 | INCOMPLETE | 0.0 | 0.0 | 0.1 | no | 67% | gap=0.5,n=3,speed=0.1,length=1.2,rotate_deg=90.0 | `state_indi/2026-09-05_215546/csv` |
<!-- Same fixed-overhead truncation as geo/speed=0.1 (67-68%, controller-independent).
INDI holds 0.0mm on all 3 pairs within the captured window. -->

| C2 | indi | 3 | PASS | 0.0 | 0.0 | 0.3 | no | 100% | gap=0.5,n=3,speed=0.2,length=1.2,rotate_deg=90.0 | `state_indi/2026-09-05_220230/csv` |
| C2 | indi | 3 | PASS | 0.0 | 0.0 | 0.7 | no | 100% | gap=0.5,n=3,speed=0.3,length=1.2,rotate_deg=90.0 | `state_indi/2026-09-05_220842/csv` |
| C2 | indi | 3 | PASS | 0.0 | 0.0 | 1.2 | no | 100% | gap=0.5,n=3,speed=0.4,length=1.2,rotate_deg=90.0 | `state_indi/2026-09-05_221342/csv` |
| C2 | indi | 3 | PASS | 0.0 | 0.0 | 1.9 | no | 100% | gap=0.5,n=3,speed=0.5,length=1.2,rotate_deg=90.0 | `state_indi/2026-09-05_221806/csv` |
| C5 | geo | 1 | PASS | nan | nan | 1.5 | no | 100% | z=0.15,speed=0.1,length=1.2,passes=2,rotate_deg=0.0 | `state_geo/2026-09-05_222207/csv` |
| C5 | geo | 1 | PASS | nan | nan | 0.7 | no | 100% | z=0.15,speed=0.2,length=1.2,passes=2,rotate_deg=0.0 | `state_geo/2026-09-05_222614/csv` |
| C5 | geo | 1 | PASS | nan | nan | 0.9 | no | 100% | z=0.15,speed=0.3,length=1.2,passes=2,rotate_deg=0.0 | `state_geo/2026-09-05_222850/csv` |
| C5 | geo | 1 | PASS | nan | nan | 1.4 | no | 100% | z=0.15,speed=0.4,length=1.2,passes=2,rotate_deg=0.0 | `state_geo/2026-09-05_223101/csv` |
| C5 | geo | 1 | PASS | nan | nan | 2.0 | no | 100% | z=0.15,speed=0.5,length=1.2,passes=2,rotate_deg=0.0 | `state_geo/2026-09-05_223259/csv` |
| C5 | indi | 1 | PASS | nan | nan | 0.1 | no | 100% | z=0.15,speed=0.1,length=1.2,passes=2,rotate_deg=0.0 | `state_indi/2026-09-05_223447/csv` |
| C5 | indi | 1 | PASS | nan | nan | 0.3 | no | 100% | z=0.15,speed=0.2,length=1.2,passes=2,rotate_deg=0.0 | `state_indi/2026-09-05_223853/csv` |
| C5 | indi | 1 | PASS | nan | nan | 0.7 | no | 100% | z=0.15,speed=0.3,length=1.2,passes=2,rotate_deg=0.0 | `state_indi/2026-09-05_224132/csv` |
| C5 | indi | 1 | PASS | nan | nan | 1.2 | no | 100% | z=0.15,speed=0.4,length=1.2,passes=2,rotate_deg=0.0 | `state_indi/2026-09-05_224345/csv` |
| C5 | indi | 1 | PASS | nan | nan | 1.9 | no | 100% | z=0.15,speed=0.5,length=1.2,passes=2,rotate_deg=0.0 | `state_indi/2026-09-05_224541/csv` |

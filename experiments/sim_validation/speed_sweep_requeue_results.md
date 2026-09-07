| Scenario | Ctrl | N | Verdict | mean\|ez\| mm | RMSE mm | max tilt | diverged | covered | params | states |
|---|---|---|---|---|---|---|---|---|---|---|
| A2 | geo | 2 | FAIL | 39.5 | 39.7 | 11.0 | no | 100% | dz=0.3,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_120534/csv` |
| A2 | geo | 2 | FAIL | 39.5 | 39.7 | 11.0 | no | 100% | dz=0.3,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_121629/csv` |
| --scenario A2 --dz 0.30 --speed 0.1 | geo | 2 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| A2 | indi | 2 | PASS | 0.0 | 0.0 | 0.1 | no | 100% | dz=0.3,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_indi/2026-09-06_122721/csv` |
| A5 | geo | 2 | FAIL | 4.2 | 9.5 | 9.8 | no | 100% | dz=0.5,radius=0.75,period=47.123,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_123827/csv` |
| A5 | geo | 2 | FAIL | 4.2 | 9.5 | 9.8 | no | 100% | dz=0.5,radius=0.75,period=47.123,laps=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_124853/csv` |
| --scenario A5 --dz 0.50 --radius 0.75 --period 47.123 --laps 2.0 | geo | 2 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| A5 | indi | 2 | PASS | 0.1 | 0.6 | 0.1 | no | 100% | dz=0.5,radius=0.75,period=47.123,laps=2.0,rotate_deg=0.0 | `state_indi/2026-09-06_125925/csv` |
| B1 | geo | 3 | FAIL | 58.8 | 59.1 | 14.3 | no | 73% | dz1=0.2,dz2=0.3,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_130955/csv` |
| B1 | geo | 3 | FAIL | 58.6 | 59.0 | 14.7 | no | 74% | dz1=0.2,dz2=0.3,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_132200/csv` |
| --scenario B1 --dz2 0.30 --speed 0.1 | geo | 3 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| B1 | geo | 3 | FAIL | 58.8 | 58.8 | 5.3 | no | 100% | dz1=0.2,dz2=0.3,speed=0.2,length=1.2,radius=0.75,period=23.561730326205623,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_133405/csv` |
| B1 | geo | 3 | FAIL | 58.8 | 58.8 | 5.3 | no | 100% | dz1=0.2,dz2=0.3,speed=0.2,length=1.2,radius=0.75,period=23.561730326205623,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_134332/csv` |
| --scenario B1 --dz2 0.30 --speed 0.2 | geo | 3 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| B1 | geo | 3 | FAIL | 57.5 | 57.6 | 2.7 | no | 100% | dz1=0.2,dz2=0.3,speed=0.3,length=1.2,radius=0.75,period=15.707820217470417,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_135252/csv` |
| B1 | geo | 3 | FAIL | 57.5 | 57.6 | 2.7 | no | 100% | dz1=0.2,dz2=0.3,speed=0.3,length=1.2,radius=0.75,period=15.707820217470417,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_135949/csv` |
| --scenario B1 --dz2 0.30 --speed 0.3 | geo | 3 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| B1 | indi | 3 | FAIL | 0.0 | 0.0 | 0.1 | no | 74% | dz1=0.2,dz2=0.3,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_indi/2026-09-06_140648/csv` |
| B1 | indi | 3 | FAIL | 0.0 | 0.0 | 0.1 | no | 74% | dz1=0.2,dz2=0.3,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_indi/2026-09-06_141853/csv` |
| --scenario B1 --dz2 0.30 --speed 0.1 | indi | 3 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| B1 | indi | 3 | PASS | 0.0 | 0.0 | 0.3 | no | 100% | dz1=0.2,dz2=0.3,speed=0.2,length=1.2,radius=0.75,period=23.561730326205623,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_indi/2026-09-06_143058/csv` |
| B1 | indi | 3 | PASS | 0.0 | 0.1 | 0.7 | no | 100% | dz1=0.2,dz2=0.3,speed=0.3,length=1.2,radius=0.75,period=15.707820217470417,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_indi/2026-09-06_213723/csv` |
| B2 | geo | 3 | FAIL | 43.9 | 44.1 | 6.5 | no | 75% | dz1=0.2,dz2=0.3,r=0.1,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_214418/csv` |
| B2 | geo | 3 | FAIL | 43.9 | 44.1 | 6.3 | no | 74% | dz1=0.2,dz2=0.3,r=0.1,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_215623/csv` |
| --scenario B2 --dz2 0.30 --r 0.10 --speed 0.1 | geo | 3 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| B2 | geo | 3 | PASS | 42.8 | 42.9 | 1.9 | no | 100% | dz1=0.2,dz2=0.3,r=0.1,speed=0.2,length=1.2,radius=0.75,period=23.561730326205623,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_220828/csv` |
| B2 | geo | 3 | PASS | 41.5 | 41.5 | 1.3 | no | 100% | dz1=0.2,dz2=0.3,r=0.1,speed=0.3,length=1.2,radius=0.75,period=15.707820217470417,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_geo/2026-09-06_221745/csv` |
| B2 | indi | 3 | FAIL | 0.0 | 0.0 | 0.1 | no | 72% | dz1=0.2,dz2=0.3,r=0.1,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_indi/2026-09-06_222455/csv` |
| B2 | indi | 3 | FAIL | 0.0 | 0.0 | 0.1 | no | 73% | dz1=0.2,dz2=0.3,r=0.1,speed=0.1,length=1.2,radius=0.75,period=47.12346065241125,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_indi/2026-09-06_223700/csv` |
| --scenario B2 --dz2 0.30 --r 0.10 --speed 0.1 | indi | 3 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| B2 | indi | 3 | PASS | 0.0 | 0.0 | 0.3 | no | 100% | dz1=0.2,dz2=0.3,r=0.1,speed=0.2,length=1.2,radius=0.75,period=23.561730326205623,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_indi/2026-09-06_224905/csv` |
| B2 | indi | 3 | PASS | 0.0 | 0.1 | 0.7 | no | 100% | dz1=0.2,dz2=0.3,r=0.1,speed=0.3,length=1.2,radius=0.75,period=15.707820217470417,laps=2.0,hold=12.0,rotate_deg=0.0 | `state_indi/2026-09-06_225838/csv` |
| C2 | geo | 3 | FAIL | 0.1 | 5.4 | 7.1 | no | 100% | gap=0.5,n=3,speed=0.1,length=1.2,rotate_deg=90.0 | `state_geo/2026-09-06_230549/csv` |
| C2 | geo | 3 | FAIL | 0.1 | 5.4 | 7.1 | no | 100% | gap=0.5,n=3,speed=0.1,length=1.2,rotate_deg=90.0 | `state_geo/2026-09-06_231400/csv` |
| --scenario C2 --speed 0.1 --rotate 90 | geo | 3 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| C2 | indi | 3 | PASS | 0.0 | 0.0 | 0.1 | no | 100% | gap=0.5,n=3,speed=0.1,length=1.2,rotate_deg=90.0 | `state_indi/2026-09-06_232218/csv` |
| A8 | geo | 2 | FAIL | 2.5 | 6.0 | 1.2 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_233024/csv` |
| A8 | geo | 2 | FAIL | 2.5 | 6.0 | 1.2 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-09-06_233316/csv` |
| --scenario A8 --dz 0.25 --span 1.0 --timescale 1.8229 | geo | 2 | NO-DATA | - | - | - | - | 0% | - | failed twice |

| Scenario | Ctrl | N | Verdict | mean\|ez\| mm | RMSE mm | max tilt | diverged | covered | params | states |
|---|---|---|---|---|---|---|---|---|---|---|
| A1 | geo | 2 | PASS | 27.6 | 27.6 | 0.0 | no | 100% | dz=0.5,hold=10.0 | `state_geo/2026-08-23_121437/csv` |
| A1 | geo | 2 | PASS | 41.5 | 41.5 | 0.0 | no | 100% | dz=0.25,hold=10.0 | `state_geo/2026-08-23_121707/csv` |
| A2 | geo | 2 | PASS | 38.4 | 38.4 | 1.2 | no | 100% | dz=0.3,speed=0.4,radius=0.75,period=7.5,laps=2.0 | `state_geo/2026-08-23_121938/csv` |
| A3 | geo | 2 | PASS | 3.5 | 8.0 | 1.8 | no | 100% | dz=0.3,speed=0.4,length=1.2,passes=2 | `state_geo/2026-08-23_122153/csv` |
| A4 | geo | 2 | FAIL | 2.0 | 208.4 | 4.0 | no | 100% | dz=0.6,offset=0.1,radius=0.45 | `state_geo/2026-08-23_122435/csv` |
| A4 | geo | 2 | FAIL | 2.0 | 208.4 | 4.0 | no | 100% | dz=0.6,offset=0.1,radius=0.45 | `state_geo/2026-08-23_122700/csv` |
| --scenario A4 --dz 0.60 --offset 0.10 --laps 1 | geo | 2 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| A5 | geo | 2 | PASS | 4.2 | 9.5 | 6.4 | no | 100% | dz=0.5,radius=0.75,period=7.5,laps=1.0 | `state_geo/2026-08-23_122928/csv` |
| A8 | geo | 2 | PASS | 2.1 | 4.0 | 1.7 | no | 100% | dz=0.25,span=1.0,duration=6.0 | `state_geo/2026-08-23_123152/csv` |
| B1 | geo | 3 | FAIL | 58.7 | 58.7 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,radius=0.75,period=7.5 | `state_geo/2026-08-23_123422/csv` |
| B1 | geo | 3 | FAIL | 58.7 | 58.7 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,radius=0.75,period=7.5 | `state_geo/2026-08-23_123733/csv` |
| --scenario B1 --dz2 0.30 --path line | geo | 3 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| B2 | geo | 3 | PASS | 42.6 | 42.7 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,r=0.1 | `state_geo/2026-08-23_124039/csv` |
| B3 | geo | 3 | PASS | 3.9 | 7.7 | 0.8 | no | 100% | dz=0.22,span=0.55,duration=8.0 | `state_geo/2026-08-23_124356/csv` |
| A1 | indi | 2 | FAIL | 52.3 | 52.3 | 0.0 | no | 100% | dz=0.5,hold=10.0 | `state_indi/2026-08-23_124722/csv` |
| A1 | indi | 2 | FAIL | 52.3 | 52.3 | 0.0 | no | 100% | dz=0.5,hold=10.0 | `state_indi/2026-08-23_124958/csv` |
| --scenario A1 --dz 0.50 --hold 10 | indi | 2 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| A1 | indi | 2 | FAIL | 82.4 | 82.4 | 0.0 | no | 100% | dz=0.25,hold=10.0 | `state_indi/2026-08-23_125233/csv` |
| A1 | indi | 2 | FAIL | 82.4 | 82.4 | 0.0 | no | 100% | dz=0.25,hold=10.0 | `state_indi/2026-08-23_125506/csv` |
| --scenario A1 --dz 0.25 --hold 10 | indi | 2 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| A2 | indi | 2 | FAIL | 75.4 | 75.4 | 1.2 | no | 100% | dz=0.3,speed=0.4,radius=0.75,period=7.5,laps=2.0 | `state_indi/2026-08-23_125742/csv` |
| A2 | indi | 2 | FAIL | 75.4 | 75.4 | 1.2 | no | 100% | dz=0.3,speed=0.4,radius=0.75,period=7.5,laps=2.0 | `state_indi/2026-08-23_125958/csv` |
| --scenario A2 --dz 0.30 --path line | indi | 2 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| A3 | indi | 2 | PASS | 7.3 | 16.5 | 1.3 | no | 100% | dz=0.3,speed=0.4,length=1.2,passes=2 | `state_indi/2026-08-23_130213/csv` |
| A4 | indi | 2 | FAIL | 4.1 | 208.5 | 4.0 | no | 100% | dz=0.6,offset=0.1,radius=0.45 | `state_indi/2026-08-23_130455/csv` |
| A4 | indi | 2 | FAIL | 4.1 | 208.5 | 4.0 | no | 100% | dz=0.6,offset=0.1,radius=0.45 | `state_indi/2026-08-23_130723/csv` |
| --scenario A4 --dz 0.60 --offset 0.10 --laps 1 | indi | 2 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| A5 | indi | 2 | PASS | 8.0 | 17.9 | 6.1 | no | 100% | dz=0.5,radius=0.75,period=7.5,laps=1.0 | `state_indi/2026-08-23_130953/csv` |
| A8 | indi | 2 | PASS | 4.4 | 8.2 | 1.3 | no | 100% | dz=0.25,span=1.0,duration=6.0 | `state_indi/2026-08-23_131218/csv` |
| B1 | indi | 3 | FAIL | 111.0 | 111.0 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,radius=0.75,period=7.5 | `state_indi/2026-08-23_131438/csv` |
| B1 | indi | 3 | FAIL | 111.0 | 111.0 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,radius=0.75,period=7.5 | `state_indi/2026-08-23_131753/csv` |
| --scenario B1 --dz2 0.30 --path line | indi | 3 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| B2 | indi | 3 | FAIL | 81.9 | 81.9 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,r=0.1 | `state_indi/2026-08-23_132112/csv` |
| B2 | indi | 3 | FAIL | 81.9 | 81.9 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,r=0.1 | `state_indi/2026-08-23_132424/csv` |
| --scenario B2 --dz2 0.30 --r 0.10 --path line | indi | 3 | NO-DATA | - | - | - | - | 0% | - | failed twice |
| B3 | indi | 3 | PASS | 7.9 | 15.4 | 0.7 | no | 100% | dz=0.22,span=0.55,duration=8.0 | `state_indi/2026-08-23_132739/csv` |
| A1 | indi | 2 | PASS | 0.0 | 0.0 | 0.0 | no | 100% | dz=0.5,hold=10.0 | `state_indi/2026-08-23_134830/csv` |
| A1 | indi | 2 | PASS | 0.0 | 0.0 | 0.0 | no | 100% | dz=0.25,hold=10.0 | `state_indi/2026-08-23_135109/csv` |
| A2 | indi | 2 | PASS | 0.1 | 0.1 | 1.2 | no | 100% | dz=0.3,speed=0.4,radius=0.75,period=7.5,laps=2.0 | `state_indi/2026-08-23_135342/csv` |
| A3 | indi | 2 | PASS | 0.6 | 1.6 | 1.2 | no | 100% | dz=0.3,speed=0.4,length=1.2,passes=2 | `state_indi/2026-08-23_135558/csv` |
| A4 | indi | 2 | FAIL | 0.2 | 208.4 | 4.1 | no | 100% | dz=0.6,offset=0.1,radius=0.45 | `state_indi/2026-08-23_135839/csv` |
| A5 | indi | 2 | PASS | 0.5 | 1.4 | 6.0 | no | 100% | dz=0.5,radius=0.75,period=7.5,laps=1.0 | `state_indi/2026-08-23_140104/csv` |
| A8 | indi | 2 | PASS | 0.3 | 1.1 | 1.2 | no | 100% | dz=0.25,span=1.0,duration=6.0 | `state_indi/2026-08-23_140331/csv` |
| B1 | indi | 3 | PASS | 0.1 | 0.1 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,radius=0.75,period=7.5 | `state_indi/2026-08-23_140549/csv` |
| B2 | indi | 3 | PASS | 0.1 | 0.1 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,r=0.1 | `state_indi/2026-08-23_140854/csv` |
| B3 | indi | 3 | PASS | 0.7 | 1.6 | 0.7 | no | 100% | dz=0.22,span=0.55,duration=8.0 | `state_indi/2026-08-23_141200/csv` |
| A4 | geo | 2 | FAIL | 2.0 | 208.4 | 4.0 | no | 100% | dz=0.6,offset=0.1,radius=0.45 | `state_geo/2026-08-23_141518/csv` |
| B1 | geo | 3 | FAIL | 58.7 | 58.7 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,radius=0.75,period=7.5 | `state_geo/2026-08-23_141743/csv` |
| A4 | geo | 2 | PASS | 2.0 | 3.3 | 4.0 | no | 100% | dz=0.6,offset=0.1,speed=0.4,length=1.2,radius=0.45,period=8.0,laps=1.0 | `state_geo/2026-08-23_152231/csv` |
| A4 | indi | 2 | PASS | 0.2 | 0.7 | 4.1 | no | 100% | dz=0.6,offset=0.1,speed=0.4,length=1.2,radius=0.45,period=8.0,laps=1.0 | `state_indi/2026-08-23_152459/csv` |
| A1 | geo | 2 | PASS | 27.6 | 27.6 | 0.0 | no | 100% | dz=0.5,hold=10.0 | `state_geo/2026-08-23_153510/csv` |
| A1 | geo | 2 | PASS | 41.5 | 41.5 | 0.0 | no | 100% | dz=0.25,hold=10.0 | `state_geo/2026-08-23_153751/csv` |
| A2 | geo | 2 | PASS | 38.4 | 38.4 | 1.2 | no | 100% | dz=0.3,speed=0.4,length=1.2,radius=0.75,period=7.5,laps=2.0,hold=12.0 | `state_geo/2026-08-23_154023/csv` |
| A3 | geo | 2 | PASS | 3.5 | 8.0 | 1.8 | no | 100% | dz=0.3,speed=0.4,length=1.2,passes=2 | `state_geo/2026-08-23_154238/csv` |
| A5 | geo | 2 | PASS | 4.2 | 9.5 | 6.4 | no | 100% | dz=0.5,radius=0.75,period=7.5,laps=1.0 | `state_geo/2026-08-23_154520/csv` |
| A8 | geo | 2 | PASS | 2.1 | 4.0 | 1.7 | no | 100% | dz=0.25,span=1.0,duration=6.0,settle=2.0 | `state_geo/2026-08-23_154746/csv` |
| B2 | geo | 3 | PASS | 42.6 | 42.7 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,r=0.1,speed=0.4,length=1.2,radius=0.75,period=7.5,laps=2.0,hold=12.0 | `state_geo/2026-08-23_155005/csv` |
| B3 | geo | 3 | PASS | 3.9 | 7.7 | 0.8 | no | 100% | dz=0.22,span=0.55,duration=8.0,settle=2.0 | `state_geo/2026-08-23_155314/csv` |
| B1 | geo | 3 | FAIL | 58.7 | 58.7 | 1.2 | no | 100% | dz1=0.2,dz2=0.3,speed=0.4,length=1.2,radius=0.75,period=7.5,laps=2.0,hold=12.0 | `state_geo/2026-08-23_155638/csv` |
| A6 | geo | 2 | PASS | 3.2 | 8.2 | 1.2 | no | 100% | dz=0.1,speed=0.2,length=1.0,passes=2 | `state_geo/2026-08-23_161841/csv` |
| A7 | geo | 2 | PASS | 25.5 | 28.6 | 0.9 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.3,length=1.2,settle=3.0,rotate_deg=0.0 | `state_geo/2026-08-23_162223/csv` |
| C4 | geo | 2 | PASS | 7.2 | 12.2 | 0.8 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_geo/2026-08-23_162522/csv` |
| C1 | geo | 3 | PASS | 0.0 | 1.4 | 0.9 | no | 100% | sep=0.3,n=3,hold=12.0,rotate_deg=0.0 | `state_geo/2026-08-23_162749/csv` |
| C2 | geo | 3 | PASS | 0.0 | 0.3 | 1.7 | no | 100% | gap=0.4,n=3,speed=0.4,length=0.9,rotate_deg=0.0 | `state_geo/2026-08-23_163123/csv` |
| C3 | geo | 3 | PASS | 0.0 | 1.6 | 1.0 | no | 100% | side=0.5,hold=12.0,rotate_deg=0.0 | `state_geo/2026-08-23_163438/csv` |
| C5 | geo | 1 | PASS | nan | nan | 0.8 | no | 100% | z=0.15,speed=0.25,length=1.2,passes=2,rotate_deg=0.0 | `state_geo/2026-08-23_163808/csv` |
| A6 | indi | 2 | PASS | 0.4 | 1.1 | 0.4 | no | 100% | dz=0.1,speed=0.2,length=1.0,passes=2,rotate_deg=0.0 | `state_indi/2026-08-23_164011/csv` |
| A7 | indi | 2 | PASS | 0.3 | 0.5 | 0.7 | no | 100% | dz_start=1.1,dz_end=0.1,speed=0.3,length=1.2,settle=3.0,rotate_deg=0.0 | `state_indi/2026-08-23_164339/csv` |
| C4 | indi | 2 | PASS | 0.2 | 0.5 | 0.7 | no | 100% | lateral=1.0,dz_start=0.5,dz_end=0.1,duration=8.0,settle=2.0,rotate_deg=0.0 | `state_indi/2026-08-23_164631/csv` |
| C1 | indi | 3 | PASS | 0.0 | 0.0 | 0.0 | no | 100% | sep=0.3,n=3,hold=12.0,rotate_deg=0.0 | `state_indi/2026-08-23_164855/csv` |
| C2 | indi | 3 | PASS | 0.0 | 0.0 | 1.6 | no | 100% | gap=0.4,n=3,speed=0.4,length=0.9,rotate_deg=0.0 | `state_indi/2026-08-23_165216/csv` |
| C3 | indi | 3 | PASS | 0.0 | 0.0 | 0.0 | no | 100% | side=0.5,hold=12.0,rotate_deg=0.0 | `state_indi/2026-08-23_165524/csv` |
| C5 | indi | 1 | PASS | nan | nan | 0.5 | no | 100% | z=0.15,speed=0.25,length=1.2,passes=2,rotate_deg=0.0 | `state_indi/2026-08-23_165846/csv` |

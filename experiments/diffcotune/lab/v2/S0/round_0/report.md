# Round 0 report (S0, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **10.53 cm**; keep-best: round 0 (10.53 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_att1.csv | 1 | ok | True | False | 30.92 | no |
| A1 | A1_att2.csv | 2 | ok | False | False | 18.70 | yes |
| A8 | A8.csv | 1 | ok | False | False | 6.01 | yes |
| A4 | A4.csv | 1 | top drone missing | False | False | 7.79 | no |
| A4 | A4_att2.csv | 2 | ok | False | False | 9.57 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 6.08 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 6.30 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.0003, 0.1423, 0.4207, -0.1687]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.8451, 'Pz': 1.2031, 'Dxy': 0.8335, 'Dz': 1.1792, 'Iz': 1.1664, 'KRxy': 0.9692, 'KRz': 0.8972, 'Kwxy': 1.0444, 'Kwz': 0.8881, 'KIxy': 0.8623}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 9, 'Pz': 20, 'Dxy': 21, 'Dz': 0, 'Iz': 5, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 15}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.95 cm, best flown set 7.96 cm (27 s)
  A1      new: 16.4 cm tilt | 16.4 cm tilt          best: 20.4 cm tilt+floor | 20.5 cm tilt+floor
  A8      new: 4.9 cm | 4.7 cm                      best: 5.8 cm | 5.6 cm
  CIRCLE  new: 3.1 cm | 3.2 cm                      best: 3.5 cm | 3.5 cm
  FIG8    new: 3.1 cm | 3.1 cm                      best: 3.3 cm | 3.4 cm
  A4      new: 7.3 cm floor | 7.2 cm floor          best: 7.0 cm floor | 6.7 cm floor
  events the best flown set shows too (not counted): A1 seed 2000 tilt, A1 seed 2001 tilt, A4 seed 2000 floor, A4 seed 2001 floor
```

Desk time of this command: 1960 s (ingest 5 s, fit 920 s, tune 1003 s).

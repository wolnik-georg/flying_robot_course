# Round 1 report (S0, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **9.23 cm**; keep-best: round 1 (9.23 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 14.82 | yes |
| A8 | A8.csv | 1 | ok | False | False | 5.28 | yes |
| A4 | A4_att1.csv | 1 | ok | False | False | 7.66 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 7.67 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 7.83 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0097, 0.1523, 0.4307, -0.1587]
theta flown (factor to the default gains): {'Pxy': 0.8451, 'Pz': 1.2031, 'Dxy': 0.8335, 'Dz': 1.1792, 'Iz': 1.1664, 'KRxy': 0.9692, 'KRz': 0.8972, 'Kwxy': 1.0444, 'Kwz': 0.8881, 'KIxy': 0.8623}
theta of the next round (factor to this round): {'Pxy': 0.9091, 'Pz': 1.0273, 'Dxy': 0.8752, 'Dz': 1.085, 'Iz': 1.0358, 'KRxy': 1.0698, 'KRz': 1.0754, 'Kwxy': 1.0875, 'Kwz': 0.9847, 'KIxy': 0.8449}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 7.27 cm, best flown set 6.99 cm (18 s)
  A1      new: 16.1 cm | 15.9 cm                    best: 16.4 cm tilt | 17.1 cm tilt
  A8      new: 4.6 cm | 4.3 cm                      best: 4.9 cm | 4.7 cm
  CIRCLE  new: 3.0 cm | 3.1 cm                      best: 3.0 cm | 3.1 cm
  FIG8    new: 2.9 cm | 3.0 cm                      best: 2.9 cm | 3.0 cm
  A4      new: 8.2 cm | 11.7 cm floor               best: 7.5 cm floor | 7.3 cm floor
  events the best flown set shows too (not counted): A4 seed 2001 floor
```

Desk time of this command: 4166 s (ingest 3 s, fit 926 s, tune 3215 s).

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

beta (log-factors) after the fit: [-0.0229, 0.1913, 0.4061, -0.1775]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.9867, 'Pz': 1.0582, 'Dxy': 0.991, 'Dz': 1.2493, 'Iz': 1.25, 'KRxy': 1.2011, 'KRz': 0.8976, 'Kwxy': 1.0518, 'Kwz': 0.8271, 'KIxy': 0.9105}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 13, 'Iz': 16, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.00 cm, best flown set 8.43 cm (18 s)
  A1      new: 13.3 cm tilt+floor | 13.4 cm tilt+floor best: 22.0 cm tilt+floor | 22.0 cm tilt+floor
  A8      new: 4.4 cm | 4.4 cm                      best: 6.1 cm | 5.9 cm
  CIRCLE  new: 2.8 cm | 2.7 cm                      best: 3.8 cm | 3.8 cm
  FIG8    new: 2.6 cm | 2.6 cm                      best: 3.5 cm | 3.6 cm
  A4      new: 7.2 cm | 6.7 cm                      best: 7.0 cm floor | 6.8 cm floor
  events the best flown set shows too (not counted): A1 seed 2000 tilt, A1 seed 2000 floor, A1 seed 2001 tilt, A1 seed 2001 floor
```

Desk time of this command: 1380 s (ingest 3 s, fit 650 s, tune 706 s).

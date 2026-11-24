# Round 1 report (S0, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **8.36 cm**; keep-best: round 1 (8.36 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 13.16 | yes |
| A8 | A8.csv | 1 | ok | False | False | 5.02 | yes |
| A4 | A4.csv | 1 | ok | False | False | 7.49 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 6.87 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 6.90 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.0424, 0.1098, 0.5029, -0.2084]
theta flown (factor to the default gains): {'Pxy': 0.9, 'Pz': 1.1, 'Dxy': 0.9, 'Dz': 1.1, 'Iz': 1.0919, 'KRxy': 1.0668, 'KRz': 0.9599, 'Kwxy': 1.1, 'Kwz': 0.9357, 'KIxy': 0.9}
theta of the next round (factor to this round): {'Pxy': 0.9, 'Pz': 1.1, 'Dxy': 0.9897, 'Dz': 1.0964, 'Iz': 1.1, 'KRxy': 1.1, 'KRz': 1.0211, 'Kwxy': 0.9759, 'Kwz': 1.0025, 'KIxy': 0.9598}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 8, 'Pz': 4, 'Dxy': 2, 'Dz': 2, 'Iz': 26, 'KRxy': 6, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.54 cm, best flown set 7.20 cm (24 s)
  A1      new: 14.0 cm floor | 13.9 cm floor        best: 16.2 cm tilt+floor | 16.0 cm tilt+floor
  A8      new: 4.8 cm | 4.6 cm                      best: 5.3 cm | 5.1 cm
  CIRCLE  new: 3.6 cm | 3.6 cm                      best: 3.9 cm | 3.9 cm
  FIG8    new: 3.4 cm | 3.5 cm                      best: 3.6 cm | 3.7 cm
  A4      new: 7.3 cm | 6.8 cm                      best: 7.3 cm floor | 7.1 cm floor
  events the best flown set shows too (not counted): A1 seed 2000 floor, A1 seed 2001 floor
```

Desk time of this command: 1959 s (ingest 3 s, fit 889 s, tune 1039 s).

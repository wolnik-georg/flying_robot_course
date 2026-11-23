# Round 0 report (S2, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **9.92 cm**; keep-best: round 0 (9.92 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 19.31 | yes |
| A8 | A8.csv | 1 | ok | False | False | 3.97 | yes |
| A4 | A4.csv | 1 | ok | False | False | 4.26 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 6.44 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 6.64 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.1105, -0.0223, -0.0719, -0.0945]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.9408, 'Pz': 1.0937, 'Dxy': 0.9, 'Dz': 1.0442, 'Iz': 1.1, 'KRxy': 1.0793, 'KRz': 0.9001, 'Kwxy': 1.1, 'Kwz': 1.0413, 'KIxy': 0.9178}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 4, 'Dxy': 39, 'Dz': 0, 'Iz': 40, 'KRxy': 5, 'KRz': 20, 'Kwxy': 36, 'Kwz': 0, 'KIxy': 7}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.91 cm, best flown set 7.95 cm (18 s)
  A1      new: 16.3 cm tilt | 16.5 cm tilt+floor    best: 20.8 cm tilt+floor | 20.5 cm tilt+floor
  A8      new: 3.5 cm | 3.4 cm                      best: 3.9 cm | 3.9 cm
  CIRCLE  new: 4.2 cm | 4.2 cm                      best: 4.5 cm | 4.6 cm
  FIG8    new: 3.8 cm | 3.8 cm                      best: 4.1 cm | 4.1 cm
  A4      new: 6.7 cm | 6.7 cm                      best: 6.6 cm | 6.6 cm
  events the best flown set shows too (not counted): A1 seed 2000 tilt, A1 seed 2001 tilt, A1 seed 2001 floor
```

Desk time of this command: 2011 s (ingest 3 s, fit 1025 s, tune 958 s).

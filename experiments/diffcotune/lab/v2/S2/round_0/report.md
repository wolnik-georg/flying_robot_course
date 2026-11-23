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
theta of the next round (factor to this round): {'Pxy': 0.8437, 'Pz': 1.0126, 'Dxy': 0.8, 'Dz': 1.1123, 'Iz': 1.25, 'KRxy': 1.0916, 'KRz': 0.9143, 'Kwxy': 1.157, 'Kwz': 0.9945, 'KIxy': 0.9579}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 19, 'Dz': 0, 'Iz': 20, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.38 cm, best flown set 7.95 cm (25 s)
  A1      new: 16.0 cm | 15.7 cm floor              best: 20.8 cm tilt+floor | 20.5 cm tilt+floor
  A8      new: 3.0 cm | 3.0 cm                      best: 3.9 cm | 3.9 cm
  CIRCLE  new: 3.2 cm | 3.2 cm                      best: 4.5 cm | 4.6 cm
  FIG8    new: 3.0 cm | 3.1 cm                      best: 4.1 cm | 4.1 cm
  A4      new: 6.9 cm | 6.6 cm                      best: 6.6 cm | 6.6 cm
  events the best flown set shows too (not counted): A1 seed 2001 floor
```

Desk time of this command: 2280 s (ingest 3 s, fit 1115 s, tune 1128 s).

# Round 3 report (S3, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **3.77 cm**; keep-best: round 2 (3.24 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 4.37 | yes |
| A8 | A8.csv | 1 | ok | False | False | 3.06 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.67 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 3.82 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 4.57 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.0266, -0.0037, 0.2034, -0.1151, -0.033]
theta flown (factor to the default gains): {'Pxy': 0.9045, 'Pz': 1.0785, 'Dxy': 1.1122, 'Dz': 1.0783, 'Iz': 0.9007, 'KRxy': 1.0753, 'KRz': 1.0701, 'Kwxy': 0.9145, 'Kwz': 0.9394, 'KIxy': 1.1002}
theta of the next round (factor to this round): {'Pxy': 0.9994, 'Pz': 1.051, 'Dxy': 0.9998, 'Dz': 0.992, 'Iz': 1.0423, 'KRxy': 0.9718, 'KRz': 0.9724, 'Kwxy': 1.0026, 'Kwz': 0.9637, 'KIxy': 1.0221}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.30 cm, best flown set 6.72 cm (26 s)
  A1      new: 15.6 cm | 17.4 cm                    best: 16.7 cm | 16.7 cm
  A8      new: 5.3 cm | 4.9 cm                      best: 6.1 cm | 5.7 cm
  CIRCLE  new: 3.5 cm | 3.7 cm                      best: 3.4 cm | 3.7 cm
  FIG8    new: 3.0 cm | 2.7 cm                      best: 3.3 cm | 2.6 cm
  A4      new: 3.7 cm | 3.3 cm                      best: 4.4 cm | 4.7 cm
```

Desk time of this command: 2386 s (ingest 4 s, fit 1063 s, tune 1285 s).

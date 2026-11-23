# Round 0 report (S3, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **3.48 cm**; keep-best: round 0 (3.48 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 4.21 | yes |
| A8 | A8.csv | 1 | ok | False | False | 2.59 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.19 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 3.65 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 4.25 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0967, -0.0116, 0.1082, -0.0406, -0.1197]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.99, 'Pz': 0.99, 'Dxy': 1.0101, 'Dz': 0.99, 'Iz': 0.99, 'KRxy': 1.0101, 'KRz': 0.99, 'Kwxy': 0.99, 'Kwz': 1.0101, 'KIxy': 1.0101}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 6, 'Kwz': 11, 'KIxy': 6}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW FAILED: A1 seed 2000: floor; A1 seed 2001: floor
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.78 cm, best flown set 6.93 cm (24 s)
  A1      new: 19.0 cm floor | 18.4 cm floor        best: 17.4 cm | 19.6 cm crash
  A8      new: 6.7 cm | 6.1 cm                      best: 6.8 cm | 6.3 cm
  CIRCLE  new: 2.1 cm | 2.8 cm                      best: 2.1 cm | 2.8 cm
  FIG8    new: 1.9 cm | 1.5 cm                      best: 1.9 cm | 1.5 cm
  A4      new: 4.0 cm | 5.3 cm                      best: 5.1 cm | 5.7 cm
```

Desk time of this command: 2509 s (ingest 2 s, fit 1207 s, tune 1266 s).

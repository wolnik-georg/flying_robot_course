# Round 1 report (S0, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **7.71 cm**; keep-best: round 1 (7.71 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 10.84 | yes |
| A8 | A8.csv | 1 | ok | False | False | 5.11 | yes |
| A4 | A4.csv | 1 | ok | False | False | 7.99 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 6.75 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 6.66 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.0229, 0.191, 0.3953, -0.2383]
theta flown (factor to the default gains): {'Pxy': 0.9867, 'Pz': 1.0582, 'Dxy': 0.991, 'Dz': 1.2493, 'Iz': 1.25, 'KRxy': 1.2011, 'KRz': 0.8976, 'Kwxy': 1.0518, 'Kwz': 0.8271, 'KIxy': 0.9105}
theta of the next round (factor to this round): {'Pxy': 0.9934, 'Pz': 1.2157, 'Dxy': 1.1933, 'Dz': 1.2277, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 0.8145, 'Kwxy': 0.8425, 'Kwz': 0.816, 'KIxy': 1.1239}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 26, 'KRxy': 9, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 3.99 cm, best flown set 5.86 cm (19 s)
  A1      new: 6.4 cm | 6.4 cm                      best: 12.8 cm tilt+floor | 12.9 cm tilt+floor
  A8      new: 3.6 cm | 3.6 cm                      best: 4.2 cm | 4.2 cm
  CIRCLE  new: 2.3 cm | 2.2 cm                      best: 2.8 cm | 2.7 cm
  FIG8    new: 2.2 cm | 2.2 cm                      best: 2.6 cm | 2.6 cm
  A4      new: 5.6 cm | 5.6 cm                      best: 7.3 cm | 6.6 cm
```

Desk time of this command: 1372 s (ingest 2 s, fit 643 s, tune 704 s).

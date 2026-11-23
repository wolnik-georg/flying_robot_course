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

beta (log-factors) after the fit: [-0.1251, 0.0912, -0.088, -0.2234]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.1716, 'Pz': 0.8694, 'Dxy': 0.8213, 'Dz': 1.0121, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 1.023, 'Kwxy': 1.0962, 'Kwz': 0.9498, 'KIxy': 0.8894}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 22, 'KRxy': 3, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 5.36 cm, best flown set 7.95 cm (18 s)
  A1      new: 13.9 cm tilt | 13.9 cm tilt+floor    best: 20.5 cm tilt+floor | 20.7 cm tilt+floor
  A8      new: 2.7 cm | 2.6 cm                      best: 3.8 cm | 3.8 cm
  CIRCLE  new: 2.5 cm | 2.6 cm                      best: 4.7 cm | 4.8 cm
  FIG8    new: 2.4 cm | 2.5 cm                      best: 4.3 cm | 4.3 cm
  A4      new: 5.4 cm | 5.3 cm                      best: 6.2 cm | 6.3 cm
  events the best flown set shows too (not counted): A1 seed 2000 tilt, A1 seed 2001 tilt, A1 seed 2001 floor
```

Desk time of this command: 1644 s (ingest 2 s, fit 762 s, tune 855 s).

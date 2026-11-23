# Round 3 report (S2, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **4.34 cm**; keep-best: round 3 (4.34 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 2.39 | yes |
| A8 | A8.csv | 1 | ok | False | False | 3.78 | yes |
| A4 | A4.csv | 1 | ok | False | False | 3.45 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 5.53 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 5.64 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.1361, -0.3252, 0.3933, -0.855]
theta flown (factor to the default gains): {'Pxy': 1.3337, 'Pz': 0.7465, 'Dxy': 1.2713, 'Dz': 0.8504, 'Iz': 1.8911, 'KRxy': 1.7614, 'KRz': 0.868, 'Kwxy': 0.895, 'Kwz': 0.7726, 'KIxy': 1.3554}
theta of the next round (factor to this round): {'Pxy': 1.25, 'Pz': 1.25, 'Dxy': 1.25, 'Dz': 1.25, 'Iz': 1.1662, 'KRxy': 1.1991, 'KRz': 0.8, 'Kwxy': 1.1947, 'Kwz': 0.8, 'KIxy': 1.25}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 27, 'Pz': 25, 'Dxy': 27, 'Dz': 23, 'Iz': 0, 'KRxy': 0, 'KRz': 26, 'Kwxy': 0, 'Kwz': 26, 'KIxy': 27}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 0.81 cm, best flown set 1.15 cm (18 s)
  A1      new: 0.1 cm | 0.1 cm                      best: 0.2 cm | 0.2 cm
  A8      new: 0.9 cm | 0.9 cm                      best: 1.3 cm | 1.3 cm
  CIRCLE  new: 1.0 cm | 1.0 cm                      best: 1.3 cm | 1.3 cm
  FIG8    new: 1.0 cm | 1.0 cm                      best: 1.3 cm | 1.3 cm
  A4      new: 1.0 cm | 1.0 cm                      best: 1.8 cm | 1.7 cm
```

Desk time of this command: 1642 s (ingest 2 s, fit 767 s, tune 848 s).

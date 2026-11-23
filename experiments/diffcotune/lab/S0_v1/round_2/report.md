# Round 2 report (S0, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **6.12 cm**; keep-best: round 2 (6.12 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 5.20 | yes |
| A8 | A8.csv | 1 | ok | False | False | 5.37 | yes |
| A4 | A4.csv | 1 | ok | False | False | 7.33 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 6.20 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 6.24 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0242, 0.1064, 0.4144, -0.2389]
theta flown (factor to the default gains): {'Pxy': 0.9803, 'Pz': 1.2864, 'Dxy': 1.1826, 'Dz': 1.5338, 'Iz': 1.5625, 'KRxy': 1.5013, 'KRz': 0.7311, 'Kwxy': 0.8861, 'Kwz': 0.6749, 'KIxy': 1.0233}
theta of the next round (factor to this round): {'Pxy': 1.0321, 'Pz': 0.9702, 'Dxy': 1.25, 'Dz': 1.25, 'Iz': 1.25, 'KRxy': 1.0977, 'KRz': 0.9997, 'Kwxy': 0.9106, 'Kwz': 0.9829, 'KIxy': 1.25}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 26, 'Dz': 14, 'Iz': 28, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 25}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 2.81 cm, best flown set 3.72 cm (18 s)
  A1      new: 3.5 cm | 3.5 cm                      best: 6.0 cm | 6.1 cm
  A8      new: 3.1 cm | 3.1 cm                      best: 3.3 cm | 3.4 cm
  CIRCLE  new: 1.4 cm | 1.4 cm                      best: 2.0 cm | 1.9 cm
  FIG8    new: 1.1 cm | 1.1 cm                      best: 1.8 cm | 1.8 cm
  A4      new: 4.9 cm | 4.9 cm                      best: 5.5 cm | 5.5 cm
```

Desk time of this command: 1375 s (ingest 2 s, fit 649 s, tune 702 s).

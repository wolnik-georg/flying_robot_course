# Round 4 report (S2, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **3.20 cm**; keep-best: round 4 (3.20 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 1.40 | yes |
| A8 | A8.csv | 1 | ok | False | False | 2.64 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.18 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 4.20 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 4.46 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.1129, -0.7742, 0.3481, -0.8791]
theta flown (factor to the default gains): {'Pxy': 1.6671, 'Pz': 0.9332, 'Dxy': 1.5891, 'Dz': 1.063, 'Iz': 2.2053, 'KRxy': 2.112, 'KRz': 0.6944, 'Kwxy': 1.0693, 'Kwz': 0.6181, 'KIxy': 1.6942}
theta of the next round (factor to this round): {'Pxy': 1.25, 'Pz': 1.25, 'Dxy': 1.25, 'Dz': 1.2435, 'Iz': 1.2238, 'KRxy': 1.25, 'KRz': 1.1075, 'Kwxy': 1.0259, 'Kwz': 1.25, 'KIxy': 1.25}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 27, 'Pz': 13, 'Dxy': 27, 'Dz': 6, 'Iz': 0, 'KRxy': 26, 'KRz': 0, 'Kwxy': 0, 'Kwz': 9, 'KIxy': 25}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 0.52 cm, best flown set 0.72 cm (18 s)
  A1      new: 0.1 cm | 0.1 cm                      best: 0.1 cm | 0.1 cm
  A8      new: 0.5 cm | 0.5 cm                      best: 0.7 cm | 0.7 cm
  CIRCLE  new: 0.7 cm | 0.7 cm                      best: 0.9 cm | 0.9 cm
  FIG8    new: 0.7 cm | 0.7 cm                      best: 1.0 cm | 1.0 cm
  A4      new: 0.5 cm | 0.5 cm                      best: 0.9 cm | 0.9 cm
```

Desk time of this command: 1634 s (ingest 2 s, fit 776 s, tune 831 s).

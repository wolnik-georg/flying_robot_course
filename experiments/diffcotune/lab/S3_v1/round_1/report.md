# Round 1 report (S3, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **4.76 cm**; keep-best: round 0 (3.48 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 8.03 | yes |
| A8 | A8.csv | 1 | ok | False | False | 2.92 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.48 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 4.03 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 4.23 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0083, -0.0432, -0.0238, 0.0044, 0.0]
theta flown (factor to the default gains): {'Pxy': 0.9317, 'Pz': 0.9473, 'Dxy': 1.0579, 'Dz': 0.9597, 'Iz': 0.9695, 'KRxy': 1.0618, 'KRz': 1.0309, 'Kwxy': 0.9323, 'Kwz': 0.9601, 'KIxy': 1.0525}
theta of the next round (factor to this round): {'Pxy': 1.0168, 'Pz': 1.0163, 'Dxy': 0.9835, 'Dz': 1.0167, 'Iz': 1.0158, 'KRxy': 0.9835, 'KRz': 1.0167, 'Kwxy': 1.0169, 'Kwz': 1.0167, 'KIxy': 0.9835}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.57 cm, best flown set 6.86 cm (18 s)
  A1      new: 17.4 cm | 18.1 cm                    best: 18.4 cm | 18.1 cm
  A8      new: 5.7 cm | 5.3 cm                      best: 6.4 cm | 6.0 cm
  CIRCLE  new: 2.7 cm | 3.1 cm                      best: 2.7 cm | 3.1 cm
  FIG8    new: 2.4 cm | 1.9 cm                      best: 2.4 cm | 1.9 cm
  A4      new: 4.8 cm | 4.2 cm                      best: 4.3 cm | 5.4 cm
```

Desk time of this command: 1818 s (ingest 2 s, fit 873 s, tune 917 s).

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

beta (log-factors) after the fit: [0.0167, -0.0167, 0.0167, 0.0167, 0.0]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.9317, 'Pz': 0.9473, 'Dxy': 1.0579, 'Dz': 0.9597, 'Iz': 0.9695, 'KRxy': 1.0618, 'KRz': 1.0309, 'Kwxy': 0.9323, 'Kwz': 0.9601, 'KIxy': 1.0525}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.29 cm, best flown set 6.90 cm (18 s)
  A1      new: 17.0 cm | 17.1 cm                    best: 18.7 cm floor | 17.9 cm
  A8      new: 5.7 cm | 5.2 cm                      best: 6.5 cm | 6.1 cm
  CIRCLE  new: 2.6 cm | 3.1 cm                      best: 2.6 cm | 3.0 cm
  FIG8    new: 2.4 cm | 1.9 cm                      best: 2.3 cm | 1.8 cm
  A4      new: 4.4 cm | 3.5 cm                      best: 4.3 cm | 5.8 cm
```

Desk time of this command: 1909 s (ingest 2 s, fit 896 s, tune 981 s).

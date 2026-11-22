# Round 1 report (S1, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **4.07 cm**; keep-best: round 0 (3.29 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 5.95 | yes |
| A8 | A8.csv | 1 | ok | False | False | 5.37 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.55 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 2.83 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 2.07 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.0749, 0.1041, -0.6395, -0.82, 0.0]
theta flown (factor to the default gains): {'Pxy': 1.25, 'Pz': 0.8, 'Dxy': 1.25, 'Dz': 1.25, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 0.8967, 'Kwxy': 1.147, 'Kwz': 1.0416, 'KIxy': 1.25}
theta of the next round (factor to this round): {'Pxy': 1.25, 'Pz': 1.0245, 'Dxy': 1.25, 'Dz': 1.25, 'Iz': 1.2221, 'KRxy': 1.25, 'KRz': 1.25, 'Kwxy': 1.049, 'Kwz': 1.0549, 'KIxy': 1.25}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 20, 'Pz': 0, 'Dxy': 28, 'Dz': 9, 'Iz': 0, 'KRxy': 24, 'KRz': 6, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 28}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 2.07 cm, best flown set 4.83 cm (19 s)
  A1      new: 5.2 cm | 5.0 cm                      best: 10.4 cm | 10.4 cm
  A8      new: 1.2 cm | 1.2 cm                      best: 2.7 cm | 2.7 cm
  CIRCLE  new: 1.3 cm | 1.3 cm                      best: 3.7 cm | 3.9 cm
  FIG8    new: 1.5 cm | 1.1 cm                      best: 3.5 cm | 3.4 cm
  A4      new: 1.5 cm | 1.5 cm                      best: 3.7 cm | 3.7 cm
```

Desk time of this command: 3642 s (ingest 2 s, fit 725 s, tune 2892 s).

# Round 2 report (S0, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **7.61 cm**; keep-best: round 2 (7.61 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 8.67 | yes |
| A8 | A8.csv | 1 | ok | False | False | 5.06 | yes |
| A4 | A4.csv | 1 | ok | False | False | 8.28 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 7.64 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 7.89 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.0278, 0.14, 0.7078, -0.3367]
theta flown (factor to the default gains): {'Pxy': 0.81, 'Pz': 1.21, 'Dxy': 0.8908, 'Dz': 1.206, 'Iz': 1.2011, 'KRxy': 1.1735, 'KRz': 0.9802, 'Kwxy': 1.0735, 'Kwz': 0.938, 'KIxy': 0.8638}
theta of the next round (factor to this round): {'Pxy': 0.9235, 'Pz': 1.1, 'Dxy': 1.0559, 'Dz': 1.1, 'Iz': 1.0375, 'KRxy': 1.0664, 'KRz': 0.9409, 'Kwxy': 0.9876, 'Kwz': 0.9271, 'KIxy': 1.0298}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 17, 'Dxy': 0, 'Dz': 39, 'Iz': 0, 'KRxy': 0, 'KRz': 14, 'Kwxy': 0, 'Kwz': 20, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.37 cm, best flown set 6.67 cm (25 s)
  A1      new: 12.9 cm floor | 13.0 cm floor        best: 13.2 cm floor | 13.6 cm floor
  A8      new: 4.6 cm | 4.6 cm                      best: 5.2 cm | 4.9 cm
  CIRCLE  new: 3.6 cm | 3.6 cm                      best: 3.5 cm | 3.5 cm
  FIG8    new: 3.4 cm | 3.4 cm                      best: 3.3 cm | 3.4 cm
  A4      new: 7.4 cm | 7.3 cm                      best: 8.5 cm floor | 7.5 cm floor
  events the best flown set shows too (not counted): A1 seed 2000 floor, A1 seed 2001 floor
```

Desk time of this command: 1905 s (ingest 3 s, fit 895 s, tune 977 s).

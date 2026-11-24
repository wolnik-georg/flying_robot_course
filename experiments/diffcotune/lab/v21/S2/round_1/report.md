# Round 1 report (S2, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **7.67 cm**; keep-best: round 1 (7.67 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 12.81 | yes |
| A8 | A8.csv | 1 | ok | False | False | 3.63 | yes |
| A4 | A4.csv | 1 | ok | False | False | 4.52 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 7.11 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 6.78 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.1747, -0.2145, 0.065, -0.436]
theta flown (factor to the default gains): {'Pxy': 0.9408, 'Pz': 1.0937, 'Dxy': 0.9, 'Dz': 1.0442, 'Iz': 1.1, 'KRxy': 1.0793, 'KRz': 0.9001, 'Kwxy': 1.1, 'Kwz': 1.0413, 'KIxy': 0.9178}
theta of the next round (factor to this round): {'Pxy': 1.0165, 'Pz': 0.9, 'Dxy': 0.9, 'Dz': 0.9, 'Iz': 1.1, 'KRxy': 1.0879, 'KRz': 0.9, 'Kwxy': 1.1, 'Kwz': 1.1, 'KIxy': 0.9}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 33, 'Dxy': 39, 'Dz': 17, 'Iz': 41, 'KRxy': 11, 'KRz': 20, 'Kwxy': 40, 'Kwz': 17, 'KIxy': 40}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.19 cm, best flown set 7.18 cm (24 s)
  A1      new: 14.4 cm | 14.3 cm                    best: 15.7 cm floor | 15.6 cm floor
  A8      new: 2.9 cm | 2.8 cm                      best: 3.4 cm | 3.4 cm
  CIRCLE  new: 3.9 cm | 3.8 cm                      best: 5.1 cm | 5.1 cm
  FIG8    new: 3.6 cm | 3.6 cm                      best: 4.7 cm | 4.7 cm
  A4      new: 6.1 cm | 6.5 cm                      best: 6.9 cm | 7.3 cm
```

Desk time of this command: 1979 s (ingest 3 s, fit 944 s, tune 1001 s).

# Round 0 report (S1, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **3.29 cm**; keep-best: round 0 (3.29 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 2.81 | yes |
| A8 | A8.csv | 1 | ok | False | False | 4.84 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.47 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 3.13 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 2.62 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.2864, -0.2937, -0.2592, -0.3959, -0.263]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.2282, 'Pz': 0.8, 'Dxy': 1.25, 'Dz': 0.8001, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 1.25, 'Kwxy': 1.1963, 'Kwz': 1.197, 'KIxy': 1.25}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 27, 'Dxy': 20, 'Dz': 4, 'Iz': 27, 'KRxy': 26, 'KRz': 26, 'Kwxy': 0, 'Kwz': 3, 'KIxy': 26}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 4.92 cm, best flown set 7.96 cm (25 s)
  A1      new: 10.8 cm | 11.0 cm                    best: 14.3 cm | 14.5 cm
  A8      new: 3.5 cm | 3.5 cm                      best: 5.1 cm | 5.1 cm
  CIRCLE  new: 3.5 cm | 3.5 cm                      best: 7.1 cm | 7.2 cm
  FIG8    new: 3.4 cm | 3.3 cm                      best: 6.6 cm | 6.5 cm
  A4      new: 3.5 cm | 3.2 cm                      best: 6.7 cm | 6.4 cm
```

Desk time of this command: 2267 s (ingest 3 s, fit 1114 s, tune 1119 s).

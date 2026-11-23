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
theta of the next round (factor to this round): {'Pxy': 1.1, 'Pz': 0.9, 'Dxy': 1.1, 'Dz': 0.9, 'Iz': 1.1, 'KRxy': 1.1, 'KRz': 1.1, 'Kwxy': 1.0245, 'Kwz': 1.1, 'KIxy': 1.1}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 26, 'Pz': 40, 'Dxy': 41, 'Dz': 40, 'Iz': 41, 'KRxy': 41, 'KRz': 40, 'Kwxy': 0, 'Kwz': 40, 'KIxy': 40}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.50 cm, best flown set 7.96 cm (25 s)
  A1      new: 13.0 cm | 12.7 cm                    best: 14.3 cm | 14.5 cm
  A8      new: 4.3 cm | 4.3 cm                      best: 5.1 cm | 5.1 cm
  CIRCLE  new: 5.3 cm | 5.5 cm                      best: 7.1 cm | 7.2 cm
  FIG8    new: 5.0 cm | 4.9 cm                      best: 6.6 cm | 6.5 cm
  A4      new: 5.1 cm | 4.9 cm                      best: 6.7 cm | 6.4 cm
```

Desk time of this command: 2176 s (ingest 2 s, fit 1027 s, tune 1118 s).

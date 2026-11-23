# Round 2 report (S1, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **7.27 cm**; keep-best: round 0 (3.29 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 2.77 | yes |
| A8 | A8.csv | 1 | ok | False | False | 4.93 | yes |
| A4 | A4.csv | 1 | ok | False | False | 7.68 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 6.34 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 11.52 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0144, 0.0533, -1.0097, -1.2305, 0.0]
theta flown (factor to the default gains): {'Pxy': 1.5625, 'Pz': 0.8196, 'Dxy': 1.5625, 'Dz': 1.5625, 'Iz': 1.5276, 'KRxy': 1.5625, 'KRz': 1.1208, 'Kwxy': 1.2032, 'Kwz': 1.0989, 'KIxy': 1.5625}
theta of the next round (factor to this round): {'Pxy': 1.25, 'Pz': 1.25, 'Dxy': 1.25, 'Dz': 1.25, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 0.8965, 'Kwxy': 0.8, 'Kwz': 0.8, 'KIxy': 1.25}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 24, 'Pz': 27, 'Dxy': 22, 'Dz': 26, 'Iz': 27, 'KRxy': 21, 'KRz': 0, 'Kwxy': 12, 'Kwz': 25, 'KIxy': 16}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 0.85 cm, best flown set 3.59 cm (19 s)
  A1      new: 0.9 cm | 0.7 cm                      best: 8.3 cm | 8.3 cm
  A8      new: 0.8 cm | 0.7 cm                      best: 2.0 cm | 1.9 cm
  CIRCLE  new: 0.9 cm | 0.9 cm                      best: 2.6 cm | 2.7 cm
  FIG8    new: 1.0 cm | 0.8 cm                      best: 2.5 cm | 2.4 cm
  A4      new: 1.0 cm | 0.9 cm                      best: 2.6 cm | 2.6 cm
```

Desk time of this command: 1605 s (ingest 2 s, fit 752 s, tune 828 s).

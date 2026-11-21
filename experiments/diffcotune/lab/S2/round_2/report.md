# Round 2 report (S2, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **5.22 cm**; keep-best: round 2 (5.22 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 5.21 | yes |
| A8 | A8.csv | 1 | ok | False | False | 3.79 | yes |
| A4 | A4.csv | 1 | ok | False | False | 4.86 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 5.85 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 6.09 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.1624, 0.1099, 0.2432, -0.5058]
theta flown (factor to the default gains): {'Pxy': 1.067, 'Pz': 0.7064, 'Dxy': 1.017, 'Dz': 1.0479, 'Iz': 1.5625, 'KRxy': 1.5625, 'KRz': 1.0849, 'Kwxy': 0.8966, 'Kwz': 0.8535, 'KIxy': 1.0843}
theta of the next round (factor to this round): {'Pxy': 1.25, 'Pz': 1.0568, 'Dxy': 1.25, 'Dz': 0.8115, 'Iz': 1.2103, 'KRxy': 1.1273, 'KRz': 0.8, 'Kwxy': 0.9982, 'Kwz': 0.9052, 'KIxy': 1.25}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 25, 'Pz': 0, 'Dxy': 25, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 15, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 14}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 1.41 cm, best flown set 1.87 cm (18 s)
  A1      new: 0.2 cm | 0.2 cm                      best: 1.0 cm | 0.7 cm
  A8      new: 1.7 cm | 1.8 cm                      best: 1.9 cm | 2.0 cm
  CIRCLE  new: 1.3 cm | 1.3 cm                      best: 1.7 cm | 1.7 cm
  FIG8    new: 1.5 cm | 1.5 cm                      best: 1.9 cm | 1.9 cm
  A4      new: 2.3 cm | 2.4 cm                      best: 2.9 cm | 2.9 cm
```

Desk time of this command: 3252 s (ingest 2 s, fit 771 s, tune 2455 s).

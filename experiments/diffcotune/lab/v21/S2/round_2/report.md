# Round 2 report (S2, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **7.06 cm**; keep-best: round 2 (7.06 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 10.06 | yes |
| A8 | A8.csv | 1 | ok | False | False | 3.52 | yes |
| A4 | A4.csv | 1 | ok | False | False | 4.85 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 7.15 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 7.79 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.2885, -0.3252, 0.1746, -0.5183]
theta flown (factor to the default gains): {'Pxy': 0.9563, 'Pz': 0.9843, 'Dxy': 0.81, 'Dz': 0.9397, 'Iz': 1.21, 'KRxy': 1.1741, 'KRz': 0.8101, 'Kwxy': 1.21, 'Kwz': 1.1454, 'KIxy': 0.826}
theta of the next round (factor to this round): {'Pxy': 0.944, 'Pz': 0.9, 'Dxy': 0.9, 'Dz': 1.1, 'Iz': 1.1, 'KRxy': 1.1, 'KRz': 0.9, 'Kwxy': 1.0953, 'Kwz': 0.9, 'KIxy': 0.9}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 40, 'Dxy': 39, 'Dz': 41, 'Iz': 40, 'KRxy': 41, 'KRz': 39, 'Kwxy': 16, 'Kwz': 40, 'KIxy': 39}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.69 cm, best flown set 7.60 cm (23 s)
  A1      new: 14.2 cm floor | 14.1 cm              best: 15.8 cm floor | 16.0 cm floor
  A8      new: 3.0 cm | 3.0 cm                      best: 3.9 cm | 3.9 cm
  CIRCLE  new: 4.0 cm | 4.0 cm                      best: 5.2 cm | 5.2 cm
  FIG8    new: 4.2 cm | 4.1 cm                      best: 5.0 cm | 5.0 cm
  A4      new: 7.9 cm | 8.3 cm                      best: 7.6 cm | 8.4 cm
  events the best flown set shows too (not counted): A1 seed 2000 floor
```

Desk time of this command: 2125 s (ingest 2 s, fit 1030 s, tune 1062 s).
